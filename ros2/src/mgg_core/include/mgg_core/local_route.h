// A ground or aerial NAVIGATE route over the local lattice: the lattice
// swept round the robot (buildGridGraph), the goal linked into it, and
// Dijkstra over its clearance-weighted edges. The planner node adds the
// turn rule, no-go zones, shortcutting and the peer checks; the navigation
// benchmark (mgg_map_octomap/test/nav_bench.cpp) runs the same code.

#ifndef MGG_CORE_LOCAL_ROUTE_H_
#define MGG_CORE_LOCAL_ROUTE_H_

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

/// `graph` is reset and rebuilt round `robot_pose` (the robot's odometry
/// pose; projected to driving height here, or anchored at the physical
/// driving height when no ground is mapped under it). `ctx.ground` must be
/// the plan's GroundProjection. `goal` is projected with projectGoal.
LocalRouteResult routeOverLocalLattice(GraphManager& graph,
                                       const StateVec& robot_pose,
                                       const StateVec& goal,
                                       const GridGraphParams& grid,
                                       const ExpandContext& ctx);

/// The ground robot's driving-height root for `robot_pose`: dropped onto
/// the ground below it, or, with no ground mapped there, the base's
/// physical driving height (`hanging` set).
StateVec localRouteRoot(const ExpandContext& ctx, const StateVec& robot_pose,
                        bool& hanging);

/// Whether a ground robot may drive the straight shortcut segment from
/// `from` to `to`, both at driving height: the check the planner node's
/// shortcut pass applies before its no-go and turn checks.
bool groundShortcutSegmentAdmissible(const ExpandContext& ctx,
                                     const Eigen::Vector3d& from,
                                     const Eigen::Vector3d& to);

}  // namespace mgg

#endif  // MGG_CORE_LOCAL_ROUTE_H_
