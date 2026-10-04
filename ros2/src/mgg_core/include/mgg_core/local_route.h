// A NAVIGATE route over MGG's local graph. Ground robots try an exact
// lattice-policy direct shortcut, then lazily expand an eight-neighbour
// ground-relative lattice with clearance-weighted Dijkstra. Aerial robots
// retain the full 3D lattice (buildGridGraph). The planner node adds the
// turn rule, no-go zones, shortcutting and the peer checks; the navigation
// benchmark (mgg_map_octomap/test/nav_bench.cpp) runs the same code.

#ifndef MGG_CORE_LOCAL_ROUTE_H_
#define MGG_CORE_LOCAL_ROUTE_H_

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/graph_expansion.h"
#include "mgg_core/graph_manager.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/types.h"

namespace mgg {

/// How close, metres, a linked goal may be to a lattice vertex without a
/// checked edge (connectStateToGraph's dist_ignore_collision_check).
inline constexpr double kLocalGoalLinkRadius = 0.1;

struct LocalRouteResult {
  bool routed = false;
  /// Why there is no route; empty when routed.
  std::string reason;
  /// Root to goal, vertices of `graph`.
  std::vector<Vertex*> route;
  GridGraphResult lattice;
};

/// `graph` is reset and expanded round `robot_pose` (the robot's odometry
/// pose; projected to driving height here, or anchored at the physical
/// driving height when no ground is mapped under it). `ctx.ground` must be
/// the plan's GroundProjection. `goal` is projected with projectGoal.
LocalRouteResult routeOverLocalLattice(GraphManager& graph,
                                       const StateVec& robot_pose,
                                       const StateVec& goal,
                                       const GridGraphParams& grid,
                                       const ExpandContext& ctx,
                                       std::optional<double> lattice_heading = std::nullopt);

/// The ground robot's driving-height root for `robot_pose`: dropped onto
/// the ground below it, or, with no ground mapped there, the base's
/// physical driving height (`hanging` set).
StateVec localRouteRoot(const ExpandContext& ctx, const StateVec& robot_pose,
                        bool& hanging);

/// Whether a ground robot may drive the straight shortcut segment from
/// `from` to `to`, both vertex states at driving height: the check the
/// planner node's shortcut pass applies before its no-go and turn checks.
/// It is the check a lattice edge driven that way passes (expandGraph):
/// the planning body turned to the segment and swept along it
/// (orientedBoxPathStatus), every ground test of getProjectedEdgeStatus,
/// and unknown space as `stop_at_unknown` says, the policy of the graph the
/// route came from (the local lattice admits unknown body volume, the
/// roadmap does not). The axis-aligned box that once checked it refused a
/// shortcut whenever its corners met unknown air the lattice had allowed,
/// so lattice weaves were sent as they came (botman, 2026-10-01).
/// `is_hanging` and `physical_root` are getProjectedEdgeStatus's: a segment
/// of a departure from a hanging root, whose ground may be unobserved, and
/// that root's own ground evidence on its first segment.
bool groundShortcutSegmentAdmissible(const ExpandContext& ctx,
                                     const Eigen::Vector3d& from,
                                     const Eigen::Vector3d& to,
                                     bool stop_at_unknown,
                                     bool is_hanging = false,
                                     const Eigen::Vector3d* physical_root = nullptr);

/// How many vertices within edge_length_max of a goal, nearest first, the
/// goal may be linked from when its nearest vertex cannot reach it.
inline constexpr int kGoalLinkCandidates = 16;

/// Links `goal_state` (at driving height) into `graph` as an exact
/// endpoint through a checked edge (expandGraphFrom) from vertices within
/// edge_length_max that `reached` says the robot reaches, nearest first, at
/// most kGoalLinkCandidates. Coincident reachable vertices are reused;
/// aerial robots retain connectStateToGraph. Null when none links.
Vertex* linkGoalToLattice(GraphManager& graph, const StateVec& goal_state,
                          const ExpandContext& ctx,
                          const std::function<bool(const Vertex&)>& reached);

}  // namespace mgg

#endif  // MGG_CORE_LOCAL_ROUTE_H_
