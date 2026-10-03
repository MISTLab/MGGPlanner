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

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <optional>
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

/// rrg.cpp:4831: maximum pose-to-roadmap snap distance. Route admission
/// must check the lead-in hop over this same distance.
inline constexpr double kDeltaLimit = 0.1;

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

/// Where a route out of the robot's current pose leaves the graph.
struct DepartureLink {
  /// The graph vertex the route starts from; null when nothing links.
  Vertex* vertex = nullptr;
  /// The pose is joined to `vertex` for this route only: the segment between
  /// them is clear along its centre line but not for the robot's box, so it
  /// is not stored, and the caller prepends the pose to the route itself.
  bool query_local = false;
};

/// Links the pose the robot stands on for departing from it: as
/// connectStateToGraph with approximate attachment, and when that finds no
/// link, the nearest in-service vertex within `link_radius` whose segment to
/// the pose is clear of known obstacles along its centre line. A robot whose
/// box touches a wall can then leave it, but not through it, and the graph
/// never holds an edge the box was not checked on.
DepartureLink linkDeparture(GraphManager& graph, const StateVec& state,
                            const ExpandContext& ctx, double link_radius);

/// Folds a verified path into `graph` (rrg.cpp:4808 Rrg::addRefPathToGraph):
/// links the first pose it can to the graph, adds the poses after it as a
/// chain with edges along it, wires every chain vertex to its reachable
/// neighbours (expandGraphEdges) and densifies long segments.
///
/// A pose links when the graph reaches it: a vertex within 0.1 m, a link
/// within 0.5 m not known to cross an obstacle, or a checked expandGraph
/// edge no longer than edge_length_max. Aerial links (including nearby-state
/// snaps) always require an observed-free swept body. Upstream linked only the first pose
/// and dropped the path when that failed; here the poses before the first
/// linked one stay out of the graph.
///
/// Poses closer than `vertex_spacing` to the last kept pose are dropped, so
/// the roadmap holds about one vertex per spacing as upstream's lattice path
/// did at grid resolution; the final pose is always kept. Returns false when
/// the path is empty or none of its poses can be linked. `path_vertices`,
/// when given, receives the chain vertices in path order, the linked vertex
/// first.
bool addRefPathToGraph(GraphManager& graph, const std::vector<StateVec>& path,
                       const ExpandContext& ctx, double vertex_spacing,
                       std::vector<Vertex*>* path_vertices = nullptr);

/// Roadmap vertices a goal lattice may bridge to, e.g. those the robot can
/// reach; path vertices whose gain may be carried across. Null admits every
/// vertex.
using UsableVertexFn = std::function<bool(const Vertex&)>;

/// The vertex overload (rrg.cpp:4808): carries each vertex's type and gain
/// across, and stops at the first hanging vertex (rrg.cpp:4869). A vertex
/// `carries_gain` refuses enters as a pose: its gain was computed under a
/// restriction (an exploration region) the roadmap must not keep, so it
/// neither marks a new vertex nor overwrites the one it lands on.
bool addRefPathToGraph(GraphManager& graph,
                       const std::vector<Vertex*>& path,
                       const ExpandContext& ctx, double vertex_spacing,
                       std::vector<Vertex*>* path_vertices = nullptr,
                       const UsableVertexFn& carries_gain = {});

/// One keyframe of the robot's trajectory.
struct TrajectoryKeyframe {
  /// The robot's base pose: x, y, z and yaw.
  StateVec pose = StateVec::Zero();
  /// The base's roll and pitch, radians: how far the robot was tipped.
  double roll = 0.0;
  double pitch = 0.0;
};

/// How rebuildRoadmapFromTrajectory lays a trajectory out.
struct RoadmapRebuildParams {
  /// About one vertex per this much travel (the roadmap's spacing).
  double vertex_spacing = 1.0;
  /// A vertex may sit up to this far beside its keyframe, across the
  /// direction of travel, where the robot's box has room.
  double max_offset = 0.8;
  /// Vertices within this distance (and edge_length_max) are joined when
  /// their edge passes: where the robot came back the same way, and round a
  /// vertex whose chain edge was refused.
  double link_radius = 2.0;
};

/// What rebuildRoadmapFromTrajectory made of a keyframe trajectory.
struct RoadmapRebuildReport {
  /// Keyframes offered, and those with no mapped ground under them.
  int keyframes = 0;
  int unsupported_keyframes = 0;
  /// Keyframes where the robot was tipped past max_inclination, left out
  /// (the latest aside).
  int tilted_keyframes = 0;
  /// Vertices that found no spot with room for the robot's box at or
  /// beside their keyframe (a robot against a wall or wedged) and were
  /// left out.
  int boxed_vertices = 0;
  /// The home keyframe has mapped ground under it. Nothing is built without.
  bool home_supported = false;
  int vertices = 0;
  /// Vertices placed beside their keyframe rather than on it.
  int offset_vertices = 0;
  /// Edges between consecutive vertices: added, refused by the edge check,
  /// and not tried because the two lie farther apart than edge_length_max.
  int chain_edges = 0;
  int chain_edges_refused = 0;
  int chain_gaps = 0;
  /// The refusals: by ProjectedEdgeStatus (ground robots; index 0
  /// unused), by the geofence, and across a stretch where the robot was
  /// tipped.
  int chain_refusals_by_status[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  int chain_refusals_geofence = 0;
  int chain_refusals_tilted = 0;
  /// Edges added between other vertices within link_radius.
  int link_edges = 0;
  /// Connected parts of the rebuilt graph, and the vertices in home's.
  int components = 0;
  int home_component_vertices = 0;
  double elapsed_s = 0.0;
};

/// Rebuilds a roadmap from where the robot has been: its keyframe poses,
/// home first and then in time order, as base poses in the graph's frame.
/// `graph` must be empty. Poses are dropped from the base onto mapped
/// ground (a ground robot's driving height); one with no ground under it,
/// or only ground more than a step above its base, is left out, and
/// without ground under home nothing is built.
///
/// Vertex 0 is home. After it, a vertex about every vertex_spacing of
/// travel (and on the latest keyframe, where the robot is now), each
/// kVisited: on its keyframe, or up to max_offset beside it across the
/// direction of travel, the first spot where the robot's box has room and
/// the edge from the previous vertex passes. Keyframes more than a spacing
/// apart get vertices in between. Consecutive vertices are joined only
/// where the edge check passes (drivenEdgeTraversable), so a stretch where
/// the robot tipped, climbed a rock or went over a ledge is dropped and
/// splits the graph there. Then vertices within link_radius are joined by
/// the same check.
///
/// A keyframe whose roll or pitch exceeds max_inclination is where the
/// robot tipped, whatever the map shows under it (an obstacle it tipped on
/// may lie in space the map never observed). It gets no vertex, the chain
/// does not continue across it, and no chain or link edge passes within
/// the robot's half diagonal of it. The latest keyframe keeps its vertex
/// even when tipped, with no chain edge into it but its links, so the
/// robot's pose can link and leave from there.
RoadmapRebuildReport rebuildRoadmapFromTrajectory(
    GraphManager& graph, const std::vector<TrajectoryKeyframe>& keyframes,
    const ExpandContext& ctx, const RoadmapRebuildParams& params);

/// The edge check of a roadmap rebuilt from the robot's trajectory, the
/// roadmap edge check (roadmapEdgeTraversable) with two differences, both
/// because the ground robot drove this trajectory: unobserved space does not
/// block, and the body swept is the robot's planning box turned along the
/// edge (orientedBoxPathStatus), not the map-aligned box grown to hold it.
/// Aerial edges instead require a strict observed-free axis-aligned body sweep.
/// A geofence refusal sets rep.status to kErrorGeofenceViolated.
bool drivenEdgeTraversable(const ExpandContext& ctx, const Vertex& from,
                           const Vertex& to, ExpandGraphReport& rep);

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
  /// Merged peers' frontiers not re-checked: farther than update_radius
  /// from every vertex of the local graph.
  int global_frontiers_left_to_owners = 0;
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
///   2) the global frontiers are re-scored with `recompute_gain` and demoted
///      to kUnvisited when they no longer border unknown space: this
///      robot's own (ctx.robot_id), and a merged peer's only within
///      `update_radius` of a vertex of `local_graph`, where this robot's
///      map may have changed; the rest are left to their owners, whose
///      broadcasts demote them (mergeNeighbourGraph),
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
/// Toward an exploration target, a frontier is further discounted by its
/// straight-line distance to the target, at the same rate as the distance
/// the robot must travel to reach it: a soft preference, never a refusal.
inline constexpr double kGlobalTargetPenalty = 0.05;

/// Fleet coverage (demoteFleetCoveredFrontiers): how far above or below
/// this robot's frontier a peer's visited vertex may be and still cover
/// it, metres. An aerial peer flying over a floor is not on it.
inline constexpr double kFleetCoverageMaxDzM = 1.0;
/// Fleet coverage: how long the roadmap walk from this robot's frontier to
/// the covering peer vertex may be, times the coverage radius.
inline constexpr double kFleetCoveragePathFactor = 1.5;
/// Fleet coverage: the longest query-only link from this robot's frontier
/// to a peer vertex the walk may start with, metres.
inline constexpr double kFleetCoverageLinkM = 1.5;
/// Fleet coverage: the swept link checks one coverage pass may make, by
/// default (fleet_coverage_max_link_checks).
inline constexpr int kFleetCoverageMaxLinkChecks = 200;
/// Fleet coverage: the peer vertices, nearest first, one frontier's links
/// may run to, by default (fleet_coverage_max_links_per_frontier). A
/// frontier's links are all checked in one pass or none are (review r4).
inline constexpr int kFleetCoverageMaxLinksPerFrontier = 8;

/// Where the next coverage pass starts among this robot's frontiers (by
/// vertex id, wrapping round): at the first one the last pass's budget
/// left unjudged, so every frontier is reached in turn whatever changes
/// between passes (review r3, P2).
struct FleetCoverageCursor {
  int next_frontier_id = 0;
};

/// One coverage pass's query-only links: the map and the box they are
/// swept with, a budget of swept checks, and where the pass starts.
struct FleetCoverageLinks {
  /// The map, and the robot whose center_offset places the box; required.
  const ExpandContext* ctx = nullptr;
  /// Optional; the pass starts where it points and moves it on.
  FleetCoverageCursor* cursor = nullptr;
  /// The box swept along a link: the robot's nominal planning box, not a
  /// request's bound mode (review r3, P1).
  Eigen::Vector3d box = Eigen::Vector3d::Zero();
  /// Swept checks this pass may make.
  int max_checks = kFleetCoverageMaxLinkChecks;
  /// The most links one frontier may be checked for: its nearest in-service
  /// peer vertices within kFleetCoverageLinkM. At most max_checks.
  int max_links_per_frontier = kFleetCoverageMaxLinksPerFrontier;
  /// Out: the swept checks made, and the frontiers the budget left
  /// unchanged, to be judged next pass.
  int checks = 0;
  int deferred = 0;
};

/// Whether `box`, swept in a straight line from `from` to `to`, meets
/// nothing `map` knows occupied in its static occupancy
/// (MapInterface::getStaticBoxStatus): no-go discs are not walls and
/// neither block nor clear it (review r3, P1); unknown
/// space passes. The sweep is split into steps of at most a map cell, each
/// box grown by its step.
bool fleetCoverageLinkClear(const MapInterface& map,
                            const Eigen::Vector3d& from,
                            const Eigen::Vector3d& to,
                            const Eigen::Vector3d& box);

/// MGG shared its global graph so the fleet explored as one (event E1,
/// rrg.cpp:5270, marks the roadmap round where a robot drove visited). Each
/// robot's map holds only its own observations, so a frontier of this
/// robot's (`robot_id`) where peers have driven never demotes on it. Here
/// every in-service kFrontier vertex of this robot within `radius_m`
/// horizontally and `max_dz_m` vertically of an in-service peer vertex its
/// owner marked visited (Vertex::owner_visited), and joined to it by a walk
/// within kFleetCoveragePathFactor * `radius_m`, is covered by the fleet:
/// fleet_covered, kUnvisited, not a frontier, no gain. The walk follows
/// in-service roadmap edges. Failing that, with `links`, it may start with
/// one link, never added to the graph, to an in-service peer vertex of any
/// type within kFleetCoverageLinkM that is fleetCoverageLinkClear with
/// links->box, centred at the robot's center_offset from the vertices; its
/// length counts. Only the links->max_links_per_frontier nearest are
/// candidates; they are tried nearest first, up to the first that joins.
/// Frontiers are taken in vertex id order from links->cursor on. One is
/// started only while the budget left (links->max_checks less the checks
/// made) holds all its candidates, and a started one is finished; at the
/// first that does not fit, link checks stop for the pass, the frontiers
/// still needing them are left as they are (links->deferred), and the
/// cursor points at that first one (review r4). Nothing is kept from one
/// pass to the next but the cursor. Without such a walk
/// (a wall between them, review r0 I-1) the frontier stays for the greedy
/// search and completion to see.
/// One-way: a covered vertex re-typed a frontier since (a lattice path
/// passing it) is demoted again. A radius of zero or less covers nothing.
/// Returns how many frontiers it demoted.
int demoteFleetCoveredFrontiers(GraphManager& graph, int robot_id,
                                double radius_m,
                                FleetCoverageLinks* links = nullptr,
                                double max_dz_m = kFleetCoverageMaxDzM);

struct GlobalFrontierReport {
  /// The best feasible frontier, or null when none exists.
  Vertex* best_frontier = nullptr;
  /// Its discounted gain (rrg.cpp:5800) and graph distance from the source.
  /// A frontier only counts with positive discounted gain; upstream started
  /// at -1 and could pick a zero-gain frontier (rrg.cpp:5675).
  double best_gain = 0.0;
  double best_distance = 0.0;
  /// Frontiers still standing after the re-check, how many the re-check
  /// demoted, and how many are reachable, unexcluded and outside reach.
  int frontiers = 0;
  int demoted = 0;
  /// This robot's gain re-checks (the work charged to the time budget).
  int rechecked = 0;
  int feasible = 0;
  /// Reachable, unexcluded frontiers still holding positive gain but too
  /// close to the robot for a repositioning. They are not explored.
  int within_reach = 0;
  /// Frontiers the time budget left unchecked: counted in `frontiers`, as
  /// they stand, but not ranked. With any, the search was cut short, and a
  /// frontier it did not rank may be the best: no frontier found is then
  /// no answer, not "none left".
  int unchecked = 0;
  bool cut_short() const { return unchecked > 0; }
};

/// Picks the global frontier to reposition to (rrg.cpp:5610 to 5834, the auto
/// branch of Rrg::runGlobalPlanner): re-scores every kFrontier vertex with
/// `recompute_gain` and demotes those that stopped being frontiers
/// (rrg.cpp:5616), runs Dijkstra from `source_id` (rrg.cpp:5670) and ranks the
/// reachable frontiers by gain * exp(-kGlobalDistancePenalty * distance),
/// times kGlobalOtherRobotPenalty for another robot's frontier
/// (rrg.cpp:5800 to 5808). Frontiers within `exclusion_radius` of an
/// `excluded` point (a peer's reservation or a refused target) are skipped.
/// With `robot_position`, frontiers within planar `reach_distance` of it
/// are also skipped and counted separately; source_id may link elsewhere.
/// With a `target` (exploring toward a goal with no known route), each
/// frontier is also discounted by exp(-kGlobalTargetPenalty * its
/// straight-line distance to the target).
///
/// The re-check is what costs: in run 8 robot_1's search over 496 frontiers
/// held its node for 6 to 16 s, past its neighbour transforms' TTL, and
/// every peer roadmap was quarantined. Reachable frontiers are re-checked
/// first, the most promising by their last gain first (a frontier never
/// scored counts as the best), then the rest; the re-check stops once
/// `time_budget_s` is spent (after at least one frontier), and the report
/// counts what it left unchecked (GlobalFrontierReport::unchecked).
/// `eligible`, when set, excludes frontiers before re-checking or ranking;
/// an operator's temporary region must not demote persistent frontiers.
/// A frontier it excludes is not counted within reach either: the search
/// could not take it anyway.
GlobalFrontierReport searchGlobalFrontier(
    GraphManager& graph, int source_id, int robot_id,
    const RecomputeGainFn& recompute_gain,
    const std::vector<Eigen::Vector3d>& excluded = {},
    double exclusion_radius = 0.0, const Eigen::Vector3d* target = nullptr,
    double time_budget_s = std::numeric_limits<double>::infinity(),
    const Eigen::Vector3d* robot_position = nullptr, double reach_distance = 0.0,
    const UsableVertexFn& eligible = {},
    std::optional<std::chrono::steady_clock::time_point> deadline = std::nullopt);

}  // namespace mgg

#endif  // MGG_CORE_GLOBAL_GRAPH_H_
