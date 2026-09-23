// The global graph as a roadmap: folding accepted exploration paths and
// clustered frontier paths into it, and searching it for the next frontier
// once the local graph has run dry.
//
// Ported from the three pieces of rrg.cpp that the port had dropped, under
// their upstream names:
//
//   * Rrg::addRefPathToGraph (rrg.cpp:4808, pose overload at 4963) with
//     Rrg::connectStateToGraph (rrg.cpp:5468),
//   * Rrg::addFrontiers (rrg.cpp:2397) with
//     Rrg::performShortestPathsClustering (rrg.cpp:5369),
//   * the frontier ranking of Rrg::runGlobalPlanner (rrg.cpp:5610 to 5834),
//     here searchGlobalFrontier. Routing to the chosen frontier stays with
//     TopologicalGoalPlanner, which every objective already uses,
//   * Rrg::expandGlobalGraphTimerCallback (rrg.cpp:2535), here
//     expandGlobalGraph, with Rrg::sampleVertex (rrg.cpp:455) and the
//     RobotStateHistory it consults (rrg.h:58, rrg.cpp:6210). This is what
//     grows the roadmap into observed space between plans, so it spans
//     explored and frontier space rather than tracing the robot's track.
//
// What upstream reached through Rrg members arrives here as arguments: the
// ExpandContext for map and footprint, and a gain callback standing in for
// computeVolumetricGainRayModelNoBound, so all of them run in unit tests on
// a synthetic map.

#ifndef MGG_CORE_GLOBAL_GRAPH_H_
#define MGG_CORE_GLOBAL_GRAPH_H_

#include <deque>
#include <functional>
#include <vector>

#include <Eigen/Dense>

#include "mgg_kdtree/kdtree.h"

#include "mgg_core/graph_base.h"
#include "mgg_core/graph_expansion.h"
#include "mgg_core/graph_manager.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/random_sampler.h"
#include "mgg_core/types.h"

namespace mgg {

/// Recomputes a vertex's volumetric gain in place, as
/// Rrg::computeVolumetricGainRayModelNoBound (rrg.cpp:3767) did for global
/// frontiers. Supplied by the caller, which owns the map, sensors and bounds.
using RecomputeGainFn = std::function<void(Vertex&)>;

/// Where the robot has been, one state per kMinLength of travel (rrg.h:58
/// RobotStateHistory, rrg.cpp:6210 to 6275). Keeps a kd-tree for the range
/// query the global graph expansion makes for every sample.
class RobotStateHistory {
 public:
  RobotStateHistory();
  ~RobotStateHistory();
  RobotStateHistory(const RobotStateHistory&) = delete;
  RobotStateHistory& operator=(const RobotStateHistory&) = delete;

  void addState(const StateVec& state);
  /// Every recorded state within `range` of `state` (rrg.cpp:6240
  /// getNearestStates). Returns false when there is none.
  bool getNearestStates(const StateVec& state, double range,
                        std::vector<const StateVec*>* s_res) const;
  void reset();
  std::size_t size() const { return state_hist_.size(); }
  const std::deque<StateVec>& states() const { return state_hist_; }

 private:
  /// A deque so the pointers the kd-tree holds stay valid as it grows.
  std::deque<StateVec> state_hist_;
  kdtree* kd_tree_ = nullptr;
};

/// rrg.h:364 and 366: how often the global graph expansion runs, and how
/// long each run may spend sampling.
inline constexpr double kGlobalGraphUpdateTimerPeriod = 0.5;
inline constexpr double kGlobalGraphUpdateTimeBudget = 0.1;
/// rrg.cpp:2574: unvisited vertices this close to a randomly chosen one form
/// one cluster, sampled around its centroid.
inline constexpr double kLocalBoxRadius = 10.0;
/// rrg.cpp:2634 and 2635: a sample is only expanded in sparse areas, i.e.
/// with no recorded robot state and no global vertex within kSparseRadius,
/// and no frontier within kOverlappedFrontierRadius.
inline constexpr double kSparseRadius = 5.0;
inline constexpr double kOverlappedFrontierRadius = 5.0;

/// Draws a collision-free state around `root_state` (rrg.cpp:455
/// Rrg::sampleVertex): up to 1000 draws from `sampler`, ground robots dropped
/// onto the terrain first, kept when the robot's box around it is free.
/// `vertex.is_hanging` is set when the draw found no ground beneath it
/// (rrg.cpp:477). Returns false when no draw was free.
bool sampleVertex(RandomSampler& sampler, const StateVec& root_state,
                  const ExpandContext& ctx, Vertex& vertex);

struct GlobalGraphExpansionReport {
  /// kUnvisited vertices the run started from and the clusters they formed.
  int unvisited_vertices = 0;
  int clusters = 0;
  /// Passes over the clusters within the budget (rrg.cpp:2610 loop_count),
  /// and samples that passed the sparsity checks and went to expandGraph
  /// (loop_count_success).
  int passes = 0;
  int samples = 0;
  int vertices_added = 0;
  int edges_added = 0;
  /// Added vertices whose gain made them frontiers.
  int frontiers_added = 0;
  double elapsed_s = 0.0;
};

/// Grows the global graph into the space around its unvisited vertices
/// (rrg.cpp:2535 Rrg::expandGlobalGraphTimerCallback), in the steps its
/// comment lists: collect the kUnvisited vertices; group them greedily, a
/// random seed vertex and everything within kLocalBoxRadius of it forming
/// one cluster with a centroid, until none is left; then, for as long as
/// `time_budget_s` allows, sample one vertex around each centroid, skip it
/// when it hangs, when a recorded robot state or any global vertex lies
/// within kSparseRadius, or a frontier within kOverlappedFrontierRadius,
/// otherwise expandGraph it and, on success, score the added vertex with
/// `compute_gain` and type it kFrontier if it is one.
///
/// A budget of zero makes no pass at all, as upstream's loop condition did;
/// otherwise every pass started is completed, so the run overshoots the
/// budget by at most one pass.
GlobalGraphExpansionReport expandGlobalGraph(
    GraphManager& global_graph, const ExpandContext& ctx,
    RandomSampler& sampler, const RobotStateHistory& robot_state_hist,
    const RecomputeGainFn& compute_gain, double time_budget_s);

/// Links `state` into `graph` (rrg.cpp:5468 Rrg::connectStateToGraph).
/// Approximate attachment may reuse a vertex within 0.1 m; exact attachment
/// preserves the requested position and collision-checks even a short link.
/// Nearby links reject known obstacles; farther links use expandGraph.
/// Returns the attached vertex, or null.
Vertex* connectStateToGraph(GraphManager& graph, const StateVec& state,
                            const ExpandContext& ctx,
                            double dist_ignore_collision_check,
                            bool exact_state);

/// Folds a verified path into `graph` (rrg.cpp:4808 Rrg::addRefPathToGraph):
/// links the first pose to the nearest graph vertex, adds the remaining poses
/// as a chain with edges along it, wires every chain vertex to its reachable
/// neighbours (expandGraphEdges) and densifies long segments.
///
/// Poses closer than `vertex_spacing` to the last kept pose are dropped, so
/// the roadmap holds about one vertex per spacing as upstream's lattice path
/// did at grid resolution; the final pose is always kept. Returns false when
/// the path is empty or its first pose cannot be linked. `path_vertices`,
/// when given, receives the chain vertices in path order.
bool addRefPathToGraph(GraphManager& graph, const std::vector<StateVec>& path,
                       const ExpandContext& ctx, double vertex_spacing,
                       std::vector<Vertex*>* path_vertices = nullptr);

/// The vertex overload (rrg.cpp:4808): carries each vertex's type and gain
/// across, and stops at the first hanging vertex (rrg.cpp:4869).
bool addRefPathToGraph(GraphManager& graph,
                       const std::vector<Vertex*>& path,
                       const ExpandContext& ctx, double vertex_spacing,
                       std::vector<Vertex*>* path_vertices = nullptr);

/// Roadmap vertices a goal lattice may bridge to, e.g. those the robot can
/// reach. Null admits every vertex.
using UsableVertexFn = std::function<bool(const Vertex&)>;

/// Bounds the lattice-to-roadmap edge checks of one connectGoalThroughLattice.
inline constexpr int kMaxGoalBridgeChecks = 4096;
/// Bounds its lattice sweeps: each one reaches round one more turn.
inline constexpr int kMaxGoalLatticePasses = 8;

struct GoalLatticeReport {
  /// Lattice sweeps made; vertices of the lattice laid out around the goal,
  /// the goal included, and how many of them the goal reaches through it.
  int passes = 0;
  int lattice_vertices = 0;
  int reachable_vertices = 0;
  /// Lattice-to-roadmap edges checked, and whether the cap stopped the search.
  int bridge_checks = 0;
  bool hit_check_limit = false;
  /// Lattice vertices folded into the roadmap, bridge and goal included.
  int chain_vertices = 0;
};

/// Attaches `goal`, exactly where it is, to `graph` through a lattice laid
/// out around it: the paper's local planner lattice, built with `grid` as
/// around the robot, but rooted at the goal, checked with `ctx` (a roadmap
/// context, so unobserved space blocks) and swept again while a sweep still
/// adds vertices (at most kMaxGoalLatticePasses, and num_vertices_max in
/// total), so it follows known space round turns. This is for a goal no
/// single roadmap edge reaches, where known space turns or narrows between
/// the roadmap and the goal.
///
/// Lattice vertices are visited by their lattice distance from the goal; the
/// first one within edge_length_max of a usable, supported roadmap vertex
/// with a traversable edge (roadmapEdgeTraversable) is the bridge. The
/// lattice path from the bridge to the goal then joins the roadmap as a
/// verified path (addRefPathToGraph, every lattice vertex kept), so later
/// goals nearby attach directly. Returns the goal's roadmap vertex, or null
/// when no lattice vertex bridges, leaving the roadmap untouched.
Vertex* connectGoalThroughLattice(GraphManager& graph, const StateVec& goal,
                                  const GridGraphParams& grid,
                                  const ExpandContext& ctx, double heading,
                                  const UsableVertexFn& usable,
                                  GoalLatticeReport* report = nullptr);

/// Groups root-to-leaf paths that run the same way (rrg.cpp:5369
/// Rrg::performShortestPathsClustering) and returns the leaf id of each
/// cluster's principal path, longest paths first. Sets Vertex::cluster_id on
/// the input vertices. With `refinement_enable`, principal paths shorter than
/// `principle_path_min_length` are dropped and every vertex is reassigned to
/// the closest remaining principal path.
std::vector<int> performShortestPathsClustering(
    GraphManager& graph, const ShortestPathsReport& rep,
    std::vector<Vertex*>& vertices, double dist_threshold = 1.0,
    double principle_path_min_length = 1.0, bool refinement_enable = true);

struct FrontierAdditionReport {
  /// Global frontiers whose gain was recomputed, and how many stopped being
  /// frontiers.
  int global_frontiers_rechecked = 0;
  int global_frontiers_demoted = 0;
  /// Frontier leaves of the local graph and the clusters they formed.
  int local_frontiers = 0;
  int clusters = 0;
  /// Principal paths folded into the global graph.
  int paths_added = 0;
};

/// Adds the local graph's frontier paths to the global graph (rrg.cpp:2397
/// Rrg::addFrontiers), in the four steps its comment lists:
///   1) leaf and frontier marks on the local graph (Dijkstra from its root;
///      gain evaluation has already typed the frontiers),
///   2) every global frontier is re-scored with `recompute_gain` and demoted
///      to kUnvisited when it no longer borders unknown space,
///   3) the frontier leaves' paths are clustered, longest first,
///   4) each cluster's principal path joins the global graph unless the
///      frontier is surrounded: a global vertex within `range_check` of it
///      (rrg.cpp:2453), or a visited global vertex within `update_radius`
///      (rrg.cpp:2457, where upstream asked the robot state history; the
///      trajectory backbone's vertices are that history here). Only the leaf
///      stays typed kFrontier (rrg.cpp:2465).
FrontierAdditionReport addFrontiers(GraphManager& global_graph,
                                    GraphManager& local_graph,
                                    const ExpandContext& ctx,
                                    const RecomputeGainFn& recompute_gain,
                                    double vertex_spacing,
                                    double range_check = 1.0,
                                    double update_radius = 3.0);

/// Distance discount of a global frontier's gain (rrg.cpp:5798
/// kGDistancePenalty). Gentler than the local path_length_penalty on purpose:
/// the global planner runs when nothing nearby is worth it, so it should not
/// throw away a large frontier for being far.
inline constexpr double kGlobalDistancePenalty = 0.05;
/// Factor applied to another robot's frontier (rrg.cpp:5799
/// kGOtherRobotPenalty), so each robot prefers the frontiers it found.
inline constexpr double kGlobalOtherRobotPenalty = 0.001;

struct GlobalFrontierReport {
  /// The best feasible frontier, or null when none exists.
  Vertex* best_frontier = nullptr;
  /// Its discounted gain (rrg.cpp:5800) and graph distance from the source.
  /// A frontier only counts with positive discounted gain; upstream started
  /// at -1 and could pick a zero-gain frontier (rrg.cpp:5675).
  double best_gain = 0.0;
  double best_distance = 0.0;
  /// Frontiers still standing after the re-check, how many the re-check
  /// demoted, and how many are reachable from the source and not excluded.
  int frontiers = 0;
  int demoted = 0;
  int feasible = 0;
};

/// Picks the global frontier to reposition to (rrg.cpp:5610 to 5834, the auto
/// branch of Rrg::runGlobalPlanner): re-scores every kFrontier vertex with
/// `recompute_gain` and demotes those that stopped being frontiers
/// (rrg.cpp:5616), runs Dijkstra from `source_id` (rrg.cpp:5670) and ranks the
/// reachable frontiers by gain * exp(-kGlobalDistancePenalty * distance),
/// times kGlobalOtherRobotPenalty for another robot's frontier
/// (rrg.cpp:5800 to 5808). Frontiers within `exclusion_radius` of an
/// `excluded` point (a peer's reservation or a refused target) are skipped.
GlobalFrontierReport searchGlobalFrontier(
    GraphManager& graph, int source_id, int robot_id,
    const RecomputeGainFn& recompute_gain,
    const std::vector<Eigen::Vector3d>& excluded = {},
    double exclusion_radius = 0.0);

}  // namespace mgg

#endif  // MGG_CORE_GLOBAL_GRAPH_H_
