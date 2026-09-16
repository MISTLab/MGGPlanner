#include "mgg_core/graph_expansion.h"

#include <cmath>
#include <vector>

namespace mgg {
namespace {

/// Records the ground-following polyline of an accepted edge into the optional
/// debug graph. The ROS 1 code did this inline, twice, for RViz.
void recordProjectedEdge(GraphManager* projected_graph,
                         const std::vector<Eigen::Vector3d>& projected_edge,
                         int robot_id) {
  if (projected_graph == nullptr || projected_edge.empty()) return;
  Vertex* prev = new Vertex(projected_graph->generateVertexID(),
                            StateVec(projected_edge[0](0), projected_edge[0](1),
                                     projected_edge[0](2), 0.0));
  prev->robot_id = robot_id;
  projected_graph->addVertex(prev);
  for (size_t i = 1; i < projected_edge.size(); ++i) {
    Vertex* v = new Vertex(projected_graph->generateVertexID(),
                           StateVec(projected_edge[i](0), projected_edge[i](1),
                                    projected_edge[i](2), 0.0));
    v->robot_id = robot_id;
    projected_graph->addVertex(v);
    projected_graph->addEdge(v, prev, (v->state - prev->state).norm());
    prev = v;
  }
}

bool geofenceBlocks(const ExpandContext& ctx, const Eigen::Vector3d& start,
                    const Eigen::Vector3d& end) {
  if (!ctx.planning->geofence_checking_enable || ctx.geofence == nullptr) {
    return false;
  }
  return ctx.geofence->getPathStatus(
             Eigen::Vector2d(start[0], start[1]),
             Eigen::Vector2d(end[0], end[1]),
             Eigen::Vector2d(ctx.robot_box_size[0], ctx.robot_box_size[1])) ==
         GeofenceManager::CoordinateStatus::kViolated;
}

/// Mean inclination of a ground-projected polyline, as the ROS 1 code
/// computed it before storing.
double averageInclination(const std::vector<Eigen::Vector3d>& edge) {
  if (edge.size() < 2) return 0.0;
  double total = 0.0;
  for (size_t i = 1; i < edge.size(); ++i) {
    const Eigen::Vector3d seg = edge[i] - edge[i - 1];
    total += std::atan2(std::abs(seg(2)), seg.head(2).norm());
  }
  return total / static_cast<double>(edge.size() - 1);
}

/// Can the robot travel the segment? Fills `projected_edge` for ground robots.
bool edgeTraversable(const ExpandContext& ctx, const Eigen::Vector3d& start,
                     const Eigen::Vector3d& end, bool is_hanging,
                     std::vector<Eigen::Vector3d>& projected_edge,
                     ExpandGraphReport& rep) {
  if (ctx.robot->type == RobotType::kAerialRobot) {
    return ctx.map->getPathStatus(start, end, ctx.robot_box_size, false) ==
           VoxelStatus::kFree;
  }
  // Ground robot: the edge has to follow the terrain.
  const ProjectedEdgeStatus es = ctx.ground->getProjectedEdgeStatus(
      start, end, ctx.robot_box_size, false, projected_edge, is_hanging);
  ++rep.edge_status[static_cast<int>(es)];
  if (es == ProjectedEdgeStatus::kAdmissible) return true;
  if (es == ProjectedEdgeStatus::kSteep) ++rep.steep_edges;
  return false;
}


}  // namespace

void expandGraph(GraphManager& graph, Vertex& new_vertex,
                 ExpandGraphReport& rep, const ExpandContext& ctx,
                 bool allow_short_edge) {
  StateVec new_state = new_vertex.state;

  Vertex* nearest_vertex = nullptr;
  if (!graph.getNearestVertex(&new_state, &nearest_vertex) ||
      nearest_vertex == nullptr) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }

  const Eigen::Vector3d origin(nearest_vertex->state[0],
                               nearest_vertex->state[1],
                               nearest_vertex->state[2]);
  Eigen::Vector3d direction(new_state[0] - origin[0], new_state[1] - origin[1],
                            new_state[2] - origin[2]);
  double direction_norm = direction.norm();

  const bool bounded_hanging_root =
      nearest_vertex->id == 0 && nearest_vertex->is_hanging &&
      std::isfinite(ctx.hanging_root_edge_length_max) &&
      ctx.hanging_root_edge_length_max > ctx.planning->edge_length_max &&
      direction_norm <= ctx.hanging_root_edge_length_max;
  if (direction_norm > ctx.planning->edge_length_max &&
      !bounded_hanging_root) {
    direction = ctx.planning->edge_length_max * direction.normalized();
  } else if (!allow_short_edge &&
             direction_norm <= ctx.planning->edge_length_min) {
    rep.status = ExpandGraphStatus::kErrorShortEdge;
    return;
  }
  direction_norm = direction.norm();
  new_state[0] = origin[0] + direction[0];
  new_state[1] = origin[1] + direction[1];
  new_state[2] = origin[2] + direction[2];

  // Ground robots snap onto the terrain before anything else looks at the
  // state; getProjectedEdgeStatus requires endpoints already at driving
  // height.
  if (ctx.robot->type == RobotType::kGroundRobot) {
    Eigen::Vector3d new_pos(new_state[0], new_state[1], new_state[2]);
    VoxelStatus vs;
    const double ground_height = ctx.ground->projectSample(new_pos, vs);
    if (vs != VoxelStatus::kOccupied) {
      rep.no_ground = true;
      rep.status = ExpandGraphStatus::kErrorCollisionEdge;
      return;
    }
    new_pos[2] -= (ground_height - ctx.planning->max_ground_height);
    new_state[0] = new_pos[0];
    new_state[1] = new_pos[1];
    new_state[2] = new_pos[2];
    direction = new_state.head<3>() - origin;
    direction_norm = direction.norm();
    // The lattice precheck happens before ground projection, so its Z may not
    // describe the body box ultimately stored in the graph. Explicit
    // objectives cannot admit a waypoint that their refiner must immediately
    // reject at the projected driving height.
    if (ctx.strict_projected_endpoint &&
        ctx.map->getStrictBoxStatus(new_state.head<3>() +
                                        ctx.robot->center_offset,
                                    ctx.robot_box_size) != VoxelStatus::kFree) {
      rep.status = ExpandGraphStatus::kErrorCollisionEdge;
      return;
    }
  }

  // Overshoot both ends, except at the root, so an edge that just grazes an
  // obstacle is rejected.
  const Eigen::Vector3d overshoot =
      (direction_norm > 1e-12)
          ? Eigen::Vector3d(ctx.planning->edge_overshoot * direction.normalized())
          : Eigen::Vector3d::Zero();

  Eigen::Vector3d start_pos = origin + ctx.robot->center_offset;
  if (nearest_vertex->id != 0) start_pos -= overshoot;
  const Eigen::Vector3d end_pos =
      origin + ctx.robot->center_offset + direction + overshoot;


  if (geofenceBlocks(ctx, start_pos, end_pos)) {
    rep.status = ExpandGraphStatus::kErrorGeofenceViolated;
    return;
  }

  std::vector<Eigen::Vector3d> projected_edge;
  const bool is_hanging = nearest_vertex->is_hanging || new_vertex.is_hanging;
  bool admissible_edge = edgeTraversable(ctx, start_pos, end_pos, is_hanging,
                                         projected_edge, rep);
  if (admissible_edge && ctx.robot->type == RobotType::kGroundRobot) {
    recordProjectedEdge(ctx.projected_graph, projected_edge, ctx.robot_id);
  }

  // Reject long edges that also change height sharply.
  if (nearest_vertex->distance > ctx.planning->nearest_range_max &&
      std::fabs(nearest_vertex->state[2] - new_state[2]) >
          ctx.planning->nearest_range_z) {
    admissible_edge = false;
  }
  // Re-check segment inclination. getProjectedEdgeStatus already rejects
  // steep edges, so this is redundant for ground robots and a no-op for
  // aerial ones (projected_edge stays empty). Kept because removing it would
  // be a behaviour change on a path this port cannot yet exercise end to end.
  for (size_t i = 1; i < projected_edge.size(); ++i) {
    const Eigen::Vector3d segment = projected_edge[i] - projected_edge[i - 1];
    if (std::abs(segment(2)) > ctx.planning->max_step_height + 1e-6 &&
        std::atan2(std::abs(segment(2)), segment.head(2).norm()) >
        ctx.planning->max_inclination) {
      admissible_edge = false;
    }
  }

  if (!admissible_edge) {
    rep.status = ExpandGraphStatus::kErrorCollisionEdge;
    return;
  }



  Vertex* added = new Vertex(graph.generateVertexID(), new_state);
  added->robot_id = ctx.robot_id;
  added->parent = nearest_vertex;
  added->distance = nearest_vertex->distance + direction_norm;
  added->is_hanging = new_vertex.is_hanging;
  nearest_vertex->children.push_back(added);
  graph.addVertex(added);
  ++rep.num_vertices_added;
  rep.vertex_added = added;
  graph.addEdge(added, nearest_vertex, direction_norm);
  ++rep.num_edges_added;
  if (ctx.inclinations != nullptr && !projected_edge.empty()) {
    ctx.inclinations->set(added->id, nearest_vertex->id,
                          averageInclination(projected_edge));
  }

  if (ctx.planning->rr_mode != RRModeType::kGraph) {
    rep.status = ExpandGraphStatus::kSuccess;
    return;
  }

  // Graph mode: wire the new vertex to every reachable neighbour in range.
  std::vector<Vertex*> nearest_vertices;
  if (!graph.getNearestVertices(&new_state, ctx.planning->nearest_range,
                                &nearest_vertices)) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  const Eigen::Vector3d new_origin(added->state[0], added->state[1],
                                   added->state[2]);
  for (Vertex* neighbour : nearest_vertices) {
    const Eigen::Vector3d to_neighbour(neighbour->state[0] - new_origin[0],
                                       neighbour->state[1] - new_origin[1],
                                       neighbour->state[2] - new_origin[2]);
    const double d_norm = to_neighbour.norm();
    if (d_norm <= ctx.planning->nearest_range_min ||
        d_norm >= ctx.planning->nearest_range_max) {
      continue;
    }
    const Eigen::Vector3d p_overshoot =
        to_neighbour / d_norm * ctx.planning->edge_overshoot;
    const Eigen::Vector3d p_start =
        new_origin + ctx.robot->center_offset - p_overshoot;
    Eigen::Vector3d p_end = new_origin + ctx.robot->center_offset + to_neighbour;
    if (neighbour->id != 0) p_end += p_overshoot;

    if (geofenceBlocks(ctx, p_start, p_end)) continue;

    std::vector<Eigen::Vector3d> neighbour_edge;
    if (!edgeTraversable(ctx, p_start, p_end, false, neighbour_edge, rep)) {
      continue;
    }
    if (ctx.robot->type == RobotType::kGroundRobot) {
      recordProjectedEdge(ctx.projected_graph, neighbour_edge, ctx.robot_id);
    }
    graph.addEdge(added, neighbour, d_norm);
    ++rep.num_edges_added;
    if (ctx.inclinations != nullptr && !neighbour_edge.empty()) {
      ctx.inclinations->set(added->id, neighbour->id,
                            averageInclination(neighbour_edge));
    }
  }

  rep.status = ExpandGraphStatus::kSuccess;
}

}  // namespace mgg
