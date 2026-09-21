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

#include <functional>
#include <vector>

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
  /// A hanging root may make one longer candidate connection across a
  /// sensor's near-field ground blind spot. The edge is only topology here;
  /// explicit objectives revalidate it with strict body/unknown checks.
  double hanging_root_edge_length_max = 0.0;
  /// Qualified simulation exploration may offer a lattice candidate whose
  /// body volume is partly unobserved. Known occupied volume still rejects
  /// the candidate, and ground projection plus the edge policy remain
  /// mandatory before it can enter the graph. False keeps the hardware and
  /// legacy strict-volume prefilter.
  bool allow_unknown_lattice_body = false;
  /// Unobserved space blocks the candidate's edge and its neighbour edges,
  /// as upstream's expandGraph always had it (stop_at_unknown_voxel true at
  /// rrg.cpp:725, 813 and 820). The local lattice leaves this false and
  /// keeps its own unknown policy; the global roadmap sets it, so a roadmap
  /// edge is one the map has seen traversable, not one it has not seen.
  bool stop_at_unknown = false;
  /// Qualified simulation bootstrap keeps the physical root at its odometry
  /// height while the first edge is projected. This applies only when vertex
  /// zero is the edge start; all later samples and endpoints remain projected.
  bool preserve_hanging_root_start_height = false;
  /// Explicit-objective graph builds require a ground-projected candidate's
  /// final body box to be observed free. Explore leaves this false to retain
  /// its legacy frontier policy; explicit refinement still validates the full
  /// swept route independently.
  bool strict_projected_endpoint = false;
  /// Optional final policy check for a ground-projected edge. This is applied
  /// before either its candidate vertex or a graph-mode neighbour edge is
  /// admitted. The points use the same coordinates passed to GroundProjection.
  std::function<bool(const std::vector<Eigen::Vector3d>&)>
      projected_edge_admissible;
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

/// Adds edges from `new_vertex`, already in `graph`, to every vertex within
/// nearest_range whose straight connection is between edge_length_min and
/// edge_length_max and runs through space the map knows to be free
/// (rrg.cpp:867 Rrg::expandGraphEdges). Used to wire a verified path into the
/// global roadmap; unknown space blocks, unlike the lattice's own edges.
void expandGraphEdges(GraphManager& graph, Vertex* new_vertex,
                      ExpandGraphReport& rep, const ExpandContext& ctx);

}  // namespace mgg

#endif  // MGG_CORE_GRAPH_EXPANSION_H_
