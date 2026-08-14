// Connecting a new sample into an exploration graph.
//
// Ported from Rrg::expandGraph (the Vertex& overload, which is the one the
// grid builder uses and the richer of the two: it propagates is_hanging into
// the ground projection).
//
// The ROS 1 function reached into eight Rrg members. Those become an explicit
// ExpandContext, which is what lets the grid builder and the merge be tested
// without a planner object, a map server or a robot.

#ifndef MGG_CORE_GRAPH_EXPANSION_H_
#define MGG_CORE_GRAPH_EXPANSION_H_

#include <Eigen/Dense>

#include "mgg_core/geofence_manager.h"
#include "mgg_core/graph_base.h"
#include "mgg_core/graph_manager.h"
#include "mgg_core/ground_projection.h"
#include "mgg_core/map_interface.h"
#include "mgg_core/path_selection.h"
#include "mgg_core/params.h"
#include "mgg_core/types.h"

namespace mgg {

/// Everything expandGraph needs from its surroundings.
struct ExpandContext {
  const MapInterface* map = nullptr;
  const PlanningParams* planning = nullptr;
  const RobotParams* robot = nullptr;
  /// Required when robot->type is kGroundRobot.
  const GroundProjection* ground = nullptr;
  /// May be null when planning->geofence_checking_enable is false.
  const GeofenceManager* geofence = nullptr;
  /// Optional. Receives the ground-following polyline of each accepted edge,
  /// which the ROS 1 code always built for RViz. Null skips that work
  /// entirely, which is the common case off-robot.
  GraphManager* projected_graph = nullptr;

  /// Optional. Receives the average inclination of each accepted edge, which
  /// selectBestPath needs for its negative-slope check. Without it that check
  /// falls back to straight-line geometry and misses terrain undulating
  /// between the endpoints.
  EdgeInclinations* inclinations = nullptr;

  int robot_id = 0;
  /// Planning footprint, i.e. robot->getPlanningSize().
  Eigen::Vector3d robot_box_size = Eigen::Vector3d::Zero();
};

/// Attaches `new_vertex` to `graph`: finds the nearest existing vertex, checks
/// that the robot could actually travel between them, and if so inserts the
/// vertex plus an edge. In kGraph mode it then adds edges to every other
/// vertex within nearest_range that is likewise reachable.
///
/// `new_vertex.state` may be adjusted: the edge is clipped to edge_length_max,
/// and ground robots have the result dropped onto the terrain.
void expandGraph(GraphManager& graph, Vertex& new_vertex,
                 ExpandGraphReport& rep, const ExpandContext& ctx,
                 bool allow_short_edge = false);

}  // namespace mgg

#endif  // MGG_CORE_GRAPH_EXPANSION_H_
