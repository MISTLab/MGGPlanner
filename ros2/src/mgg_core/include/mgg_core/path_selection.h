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
/// 0.64 m, boxed in. For an aerial robot it is its circumscribed radius
/// (half the diagonal of RobotParams::size x and y) plus
/// PlanningParams::aerial_viewpoint_clearance_margin, and never less than
/// that radius: with the inscribed radius and 0.1 m, the drone smoke test's
/// first goal ended 0.7 m from a wall the drone struck on the way (drone
/// scout Task 15). The lattice's body check is a box aligned with the map
/// and only size_extension larger than the robot, so a vertex it admits
/// can still end a path with the robot's flank or corner against a wall:
/// routes on the SubT finals ended
/// 0.08 and 0.11 m from a lethal cell of the controller's costmap
/// (2026-09-23). Unknown space passes, as it does for the lattice's body
/// check, and so does a query the map cannot answer.
///
/// `slope` is the ground's measured slope at the viewpoint, radians
/// (terrainSlope, PathTurnCheck::slopeAt). Where it is steeper than
/// kLevelGroundSlopeRad the robot may not turn (PathTurnCheck), so the end
/// does not need turnSpaceObserved (slopeExemptsTurnSpace): on a ramp down
/// past a crest the air over the ground is carved in patches, and every end
/// on it had a wholly unknown column in its turning circle (diag-ramp, run
/// 7). A slope that was not measured (kUnknownSlopeRad, the default) is
/// not exempt. Such an end needs a way back, which only the path to it can
/// tell: selectBestPath's SlopeEndRetreat.
bool viewpointClear(const MapInterface& map, const RobotParams& robot,
                    const PlanningParams& planning, const StateVec& viewpoint,
                    double slope = kUnknownSlopeRad);

/// Whether a ground robot's `slope` at a path end, measured (terrainSlope),
/// exempts the end from turnSpaceObserved in viewpointClear: steeper than
/// kLevelGroundSlopeRad, where the robot may not turn, and not
/// kUnknownSlopeRad.
inline bool slopeExemptsTurnSpace(double slope) {
  return slope > kLevelGroundSlopeRad && slope < kUnknownSlopeRad;
}

/// Whether an exploration path may end at a vertex, e.g. viewpointClear.
using ViewpointClearFn = std::function<bool(const Vertex&)>;
/// Whether no path may end at a vertex at all, as none may in a
/// reservation: e.g. one inside a no-go zone.
using EndExcludedFn = std::function<bool(const Vertex&)>;

/// A narrow or slope-exempt endpoint needs a refuge back along its own
/// candidate path: observed room on level ground where it can turn. Reverse
/// capability is mandatory. With reverse_edge_admissible every edge, including
/// the forward-only root edge, is explicitly checked in reverse, bounded by
/// PlanningParams::reverse_exit_max_length. Without that validator the legacy
/// kDepartureMaxM limit remains. Neither case relaxes the path-turn veto.
struct SlopeEndRetreat {
  /// Whether viewpoint_clear admits the vertex only because it stands on a
  /// slope (slopeExemptsTurnSpace, and not turnSpaceObserved).
  std::function<bool(const Vertex&)> admitted_on_slope;
  /// Whether the robot has room to turn in place at a vertex, e.g.
  /// roomToTurn.
  std::function<bool(const Vertex&)> room_to_turn;
  /// Explicit reverse-direction checks, including the forward-only root
  /// edge. When present the search uses reverse_exit_max_length; without
  /// it, the legacy two-metre bound remains (never an unchecked extension).
  std::function<bool(const Vertex&, const Vertex&)> reverse_edge_admissible;
};

/// Ends `route` (the robot's pose first) at its last pose with a way back,
/// dropping the poses after it: a pose that is not `on_slope` has one, and
/// one that is has one when a pose within `max_reverse_length` of route length
/// back from it (the first included) has `room_to_turn`, with every reverse
/// edge passing `reverse_edge_admissible` when supplied (SlopeEndRetreat's
/// rule, for a route of poses: review r1, R1-3). Both are asked by the
/// pose's index in `route`. Returns false, leaving `route` untouched, when
/// no pose after the first has one. Without explicit reverse validation the
/// bound cannot exceed kDepartureMaxM. Zero uses that legacy bound.
bool cutBackToWayBack(std::vector<StateVec>& route,
                      const std::function<bool(std::size_t)>& on_slope,
                      const std::function<bool(std::size_t)>& room_to_turn,
                      bool reverse_allowed = true,
                      const std::function<bool(std::size_t, std::size_t)>&
                          reverse_edge_admissible = {},
                      double max_reverse_length = 2.0);

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
  /// Narrow or slope-exempt path ends refused for having no room to turn
  /// within the bounded, validated reverse route
  /// (SlopeEndRetreat), counted once per candidate end.
  int slope_ends_without_way_back = 0;
  /// No admissible path ended clear, so the best fallback was chosen.
  /// It still obeys slope_end_retreat when that check is provided.
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

/// The factor path_direction_penalty puts on a path's score for leaving
/// `exploring_direction`: exp(-path_direction_penalty * deviation), and
/// with `bounded` never below path_direction_min_factor, as selectBestPath
/// applies it.
double pathDirectionFactor(const std::vector<Eigen::Vector3d>& path,
                           double exploring_direction,
                           const PlanningParams& planning, bool bounded);

/// Whether a chosen path turns back: it ends more than 90 degrees from
/// `exploring_direction`, as seen from its start, with the penalty bounded
/// (path_direction_min_factor above 0).
bool pathTurnsBack(const std::vector<Eigen::Vector3d>& path,
                   double exploring_direction, const PlanningParams& planning);

/// Light hysteresis on turning back. With path_direction_min_factor a path
/// back needs only a few times the gain of one ahead, and on the run-8
/// replays a robot on an unchanging map went back and forth between two
/// viewpoints 2.8 m apart, each turning back for the other. The selection
/// right after a path that turned back uses the unbounded penalty, so the
/// robot does not turn straight back again for a similar gain.
class TurnBackHysteresis {
 public:
  /// `planning` as the next selection should use it.
  PlanningParams selectionParams(const PlanningParams& planning) const;
  /// Records the lattice path sent (empty for none) and the direction it
  /// was scored against.
  void record(const std::vector<Eigen::Vector3d>& path,
              double exploring_direction, const PlanningParams& planning);
  bool lastTurnedBack() const { return last_turned_back_; }
  /// Forgets the last path: another mode superseded it (a global route, an
  /// objective, a new target), or none was sent.
  void reset() { last_turned_back_ = false; }

 private:
  bool last_turned_back_ = false;
};

/// Scores every root-to-leaf path and returns the best. A path may not end
/// where `end_excluded` says, treated as ending in a reservation, and no
/// path, however cut back, is chosen that `path_admissible` refuses (one
/// entering a no-go zone): another is, rather than none (review r1, R1-1).
///
/// `exploring_direction` is the direction paths are penalised for leaving,
/// by path_direction_penalty: the planner node passes the robot's heading,
/// or the bearing to an exploration target. Gain must already have been
/// computed, for instance by computeExplorationGain. Paths ending within
/// `exclusion_radius` of an `excluded_endpoints` point are skipped; a path
/// pulled back is checked where it now ends.
///
/// With `viewpoint_clear`, prefer an end that passes, cutting back when
/// needed. With `slope_end_retreat`, a narrow or slope-exempt end may also
/// compete if it has a bounded reverse way back (SlopeEndRetreat). This
/// applies to every candidate end, including cutbacks and fallback: no room
/// anywhere means no path, not an unchecked narrow-end fallback.
/// Wherever a path ends, it must lie outside reservations and its path must
/// be admissible. A clear end within `goal_reach` of the root or leading to
/// no gain goes nowhere and does not count as clear. A gainless clear prefix
/// may win if it leads to gain at an admissible leaf. Without a whole
/// admissible path to gain or a clear end with gain, no path is chosen.
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
                                   double goal_reach = 0.0,
                                   const SlopeEndRetreat& slope_end_retreat =
                                       {},
                                   const EndExcludedFn& end_excluded =
                                       nullptr,
                                   const PathTurnsFn& path_admissible =
                                       nullptr);

}  // namespace mgg

#endif  // MGG_CORE_PATH_SELECTION_H_
