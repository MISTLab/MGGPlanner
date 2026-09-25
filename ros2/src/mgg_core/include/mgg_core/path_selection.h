// Choosing where to go: scoring the paths through the local graph and picking
// the best one.
//
// Ported from Rrg::evaluateGraph. Given a graph whose vertices already carry
// volumetric gain, this runs Dijkstra from the root, walks each root-to-leaf
// path, and scores it by accumulated gain discounted for length and for
// deviation from the direction the robot is already exploring in.

#ifndef MGG_CORE_PATH_SELECTION_H_
#define MGG_CORE_PATH_SELECTION_H_

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/gain.h"
#include "mgg_core/graph_manager.h"
#include "mgg_core/map_interface.h"
#include "mgg_core/params.h"
#include "mgg_core/path_turns.h"
#include "mgg_core/types.h"

namespace mgg {

/// Inclination of each graph edge, keyed by the vertex pair.
///
/// Sparse. The ROS 1 code used a dense num_vertices_max squared matrix of
/// doubles, rebuilt every planning cycle, which at the shipped
/// num_vertices_max of 8000 meant 512 MB of allocation churn per iteration to
/// hold an entry per real edge.
///
/// Entries are stored both ways round. The original wrote only
/// [new][nearest] while the path walk reads [further][closer]; those coincide
/// for the primary link but not for RRG neighbour edges, which then read back
/// as flat and escaped the negative-slope check entirely.
class EdgeInclinations {
 public:
  void set(int a, int b, double value) {
    values_[key(a, b)] = value;
    values_[key(b, a)] = value;
  }
  /// Unknown edges read as flat, matching the zero-filled matrix.
  double get(int a, int b) const {
    auto it = values_.find(key(a, b));
    return it == values_.end() ? 0.0 : it->second;
  }
  void clear() { values_.clear(); }
  size_t size() const { return values_.size(); }

 private:
  static uint64_t key(int a, int b) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
           static_cast<uint32_t>(b);
  }
  std::unordered_map<uint64_t, double> values_;
};

/// How much farther from an obstacle than its turning radius a ground
/// robot's path may end, metres: where a controller stops the robot within
/// its goal tolerance, it still has room to turn.
inline constexpr double kViewpointArrivalSlack = 0.05;

/// Whether the robot, stopped at `viewpoint`, keeps its footprint clear of
/// known obstacles: no occupied voxel within a radius, in the xy plane, over
/// the height of its collision box. For a ground robot the radius is its
/// turning radius (RobotParams::turningRadius) plus kViewpointArrivalSlack
/// plus PlanningParams::viewpoint_clearance_margin, and never less than the
/// turning radius whatever the margin, so a robot that reaches
/// a clear path end can turn there (roomToTurn; with
/// PlanningParams::min_observed_ground_fraction set, a ground robot's end
/// also needs turnSpaceObserved): in run 5 (2026-09-25) path
/// ends allowed 0.49 m from a wall left Bunkers, whose corners reach
/// 0.64 m, boxed in. For an aerial robot it is its inscribed radius (half
/// the smaller of RobotParams::size x and y) plus the margin. The lattice's
/// body check is a box aligned with the map and only size_extension larger
/// than the robot, so a vertex it admits can still end a path with the
/// robot's flank or corner against a wall: routes on the SubT finals ended
/// 0.08 and 0.11 m from a lethal cell of the controller's costmap
/// (2026-09-23). Unknown space passes, as it does for the lattice's body
/// check, and so does a query the map cannot answer.
bool viewpointClear(const MapInterface& map, const RobotParams& robot,
                    const PlanningParams& planning, const StateVec& viewpoint);

/// Whether an exploration path may end at a vertex, e.g. viewpointClear.
using ViewpointClearFn = std::function<bool(const Vertex&)>;

/// How far from `state` the nearest known occupied voxel lies in the xy
/// plane, over the height of the robot's collision box, up to `limit`
/// metres: the largest radius of the occupied-only cylinder check
/// (MapInterface::getOccupiedOnlyCylinderPathStatus, as viewpointClear asks
/// it) that is clear, found to within limit / 16. `limit` when nothing is
/// that close, 0 when the check fails at any radius. Unknown space and a
/// query the map cannot answer count as clear.
double obstacleClearance(const MapInterface& map, const RobotParams& robot,
                         const StateVec& state, double limit);

/// The clearance of a vertex, e.g. obstacleClearance.
using VertexClearanceFn = std::function<double(const Vertex&)>;

/// The factor selectBestPath multiplies a ground robot's path score by for
/// the least clearance along it, metres: 1 at path_clearance_distance or
/// more, falling linearly to path_clearance_min_factor at 0; 1 when
/// path_clearance_distance is 0.
double pathClearanceFactor(double clearance, const PlanningParams& planning);

/// Ends `route` at its last pose that passes `clear`, dropping the poses
/// after it; the first pose, where the robot stands, is never asked. Returns
/// false, leaving `route` untouched, when no pose after the first passes.
bool pullBackToClearViewpoint(
    std::vector<StateVec>& route,
    const std::function<bool(const StateVec&)>& clear);

struct PathSelectionResult {
  /// Vertex ending the best path, or -1 if none scored above zero: a leaf, or
  /// the vertex its path was pulled back to.
  int best_path_id = -1;
  double best_gain = 0.0;
  /// The gain of the whole path the best path was chosen for: best_gain,
  /// or when the best path was pulled back to a clear vertex, the gain of
  /// the path it was pulled back from, whose prefix may carry none of it
  /// (leafs_only_for_volumetric_gain).
  double best_full_gain = 0.0;
  /// The best path, root first.
  std::vector<Vertex*> best_path;
  /// Any vertex on any evaluated path was a frontier.
  bool frontier_exists = false;
  int leaves_evaluated = 0;
  /// Paths discarded for descending more steeply than max_negative_inclination.
  int paths_rejected_steep = 0;
  /// Paths whose leaf failed `viewpoint_clear`: those pulled back along the
  /// path to the last vertex that passes, and those with none past the root.
  int paths_pulled_back = 0;
  int paths_without_clear_viewpoint = 0;
  /// No admissible path ended clear, so the best path was chosen without
  /// the clearance check.
  bool unclear_viewpoint = false;
  /// Candidate paths, as they are or pulled back, that failed
  /// `turns_admissible`.
  int paths_with_sharp_turns = 0;
  /// No path that passes `turns_admissible` could be chosen, so the best
  /// path was chosen without the check.
  bool sharp_turn_fallback = false;
  /// No leaf's shortest path passed `turns_admissible`, and the path chosen
  /// is a longer route to gain that does (findTurnCompliantRoutes).
  bool sharp_turn_detour = false;
  /// Whether that route was looked for (the refusals allowed it and a
  /// `sharp_turn_allowed` was given), what the search cost, whether it
  /// stopped at kMaxDetourSearchStates rather than running out of routes,
  /// and how many routes to gain it found. A fallback after a capped search
  /// may have missed a compliant route; after a completed one, the search
  /// found none that was chosen.
  bool detour_searched = false;
  int detour_states_expanded = 0;
  bool detour_search_capped = false;
  int detour_routes_found = 0;
};

/// Whether the best path of `selection` takes the robot nowhere, and so is
/// no path: it ends within `reach` of `robot` in the xy plane, a
/// controller's goal tolerance, reached before it starts; or the path it
/// was chosen for leads to no gain at all (best_full_gain). In run 5 a Spot
/// on a slope was sent a 2-pose path with no gain 28 times, which its
/// controller refused each time, and neither its boxed-in departure nor
/// global repositioning ran. False when there is no best path.
bool pathGoesNowhere(const PathSelectionResult& selection,
                     const Eigen::Vector3d& robot, double reach);

/// Bound on findTurnCompliantRoutes in selectBestPath, in states: one per
/// directed edge at most; a Bunker lattice of 979 vertices on a ramp had
/// 18973 edges, 37946 directed.
constexpr int kMaxDetourSearchStates = 100000;

/// Scores every root-to-leaf path and returns the best.
///
/// `exploring_direction` is the heading the robot has been travelling, used to
/// penalise paths that double back. Gain must already have been computed, for
/// instance by computeExplorationGain. Paths ending within
/// `exclusion_radius` of an `excluded_endpoints` point are skipped; a path
/// pulled back is checked where it now ends.
///
/// With `viewpoint_clear`, a path whose leaf fails it ends at the last vertex
/// along it that passes, and is scored up to there. The best path ending
/// clear wins; only when there is none is the best path chosen without the
/// check, flagged unclear_viewpoint, so that clearance never stops
/// exploration where it would have gone on. A clear end within
/// `goal_reach` of the root, or on a path leading to no gain, goes nowhere
/// (pathGoesNowhere) and does not count as clear: at the mouth of a
/// passage too narrow to end a path in, the path into it is chosen
/// unclear, not the robot's own place.
///
/// With `turns_admissible` (PathTurnCheck), a candidate that fails it is not
/// admissible, and outranks clearance: a path that turns only where it may
/// is chosen, clear or not, over one that ends clear. When no such path is
/// chosen at all, and `sharp_turn_allowed` is given, the selection is made
/// again over the cheapest routes to every vertex with gain that turn
/// sharply only where it allows (findTurnCompliantRoutes), still subject to
/// the check, flagged sharp_turn_detour; a longer route round to the same
/// frontier then wins over the short one that turns where it may not.
/// Only when that too chooses nothing is the selection made without the
/// check, flagged sharp_turn_fallback.
///
/// With `clearance`, a ground robot's path score is multiplied by
/// pathClearanceFactor of the least clearance of its end and of its
/// vertices farther than PlanningParams::path_clearance_distance from the
/// root: nearer the robot, every path passes the obstacles it stands by.
PathSelectionResult selectBestPath(GraphManager& graph,
                                   const PlanningParams& planning,
                                   const RobotParams& robot,
                                   const EdgeInclinations& inclinations,
                                   double map_resolution,
                                   double exploring_direction,
                                   const std::vector<Eigen::Vector3d>&
                                       excluded_endpoints = {},
                                   double exclusion_radius = 0.0,
                                   const ViewpointClearFn& viewpoint_clear =
                                       nullptr,
                                   const PathTurnsFn& turns_admissible =
                                       nullptr,
                                   const SharpTurnAllowedFn&
                                       sharp_turn_allowed = nullptr,
                                   const VertexClearanceFn& clearance =
                                       nullptr,
                                   double goal_reach = 0.0);

}  // namespace mgg

#endif  // MGG_CORE_PATH_SELECTION_H_
