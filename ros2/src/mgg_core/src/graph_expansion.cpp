#include "mgg_core/graph_expansion.h"
#include "mgg_core/departure.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>
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
/// Only ground lattice edges may use the relaxed unknown policy.
bool edgeTraversable(const ExpandContext& ctx, const Eigen::Vector3d& start,
                     const Eigen::Vector3d& end, bool is_hanging,
                     bool preserve_start_height,
                     std::vector<Eigen::Vector3d>& projected_edge,
                     ExpandGraphReport& rep, bool stop_at_unknown = false,
                     EdgeTravel travel = EdgeTravel::kBothWays,
                     const OrientedBox* standing = nullptr,
                     bool root_edge = false,
                     const Eigen::Vector3d* physical_root = nullptr) {
  if (ctx.robot->type == RobotType::kAerialRobot) {
    return ctx.map->getStrictPathStatus(start, end, ctx.robot_box_size) ==
           VoxelStatus::kFree;
  }
  EdgeVerdictCache::Key key{};
  // Root evidence is outside the sweep's cache key. Never reuse a root
  // verdict (including a supported root's) across different physical poses.
  if (ctx.edge_verdicts != nullptr && !root_edge) {
    const auto exactBits = [](double value) {
      std::int64_t bits;
      if (value == 0.0) value = 0.0;  // canonicalize signed zero
      std::memcpy(&bits, &value, sizeof(bits));
      return bits;
    };
    key = {exactBits(start.x()), exactBits(start.y()), exactBits(start.z()),
           exactBits(end.x()),   exactBits(end.y()),   exactBits(end.z()),
           (is_hanging ? 1 : 0) | (preserve_start_height ? 2 : 0) |
               (stop_at_unknown ? 4 : 0) |
               (travel == EdgeTravel::kForward ? 8 : 0) | (standing ? 16 : 0),
           exactBits(ctx.robot_box_size.x()), exactBits(ctx.robot_box_size.y()),
           exactBits(ctx.robot_box_size.z()), 0, 0};
    if (const EdgeVerdictCache::Verdict* known = ctx.edge_verdicts->find(key)) {
      if (PlanProfile* profile = ctx.ground->profile()) {
        ++profile->edge_cache_hits;
      }
      ++rep.edge_status[static_cast<int>(known->status)];
      if (known->status == ProjectedEdgeStatus::kSteep) ++rep.steep_edges;
      if (known->status != ProjectedEdgeStatus::kAdmissible) return false;
      projected_edge = known->projected_edge;
      return true;
    }
  }
  // Keep the extended body aligned with travel, as for driven roadmap
  // edges; the terrain and observed-ground checks remain unchanged.
  OrientedBox body;
  body.heading = std::atan2(end.y() - start.y(), end.x() - start.x());
  body.size = ctx.robot_box_size;
  EdgeBodyCheck check;
  check.sweep = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return orientedBoxPathStatus(*ctx.map, a, b, body, stop_at_unknown, standing, true,
                                  ctx.unknown_body_above_center,
                                  ctx.own_body_known_free.get(), true);
  };
  // Ground robot: the edge has to follow the terrain.
  ProjectedEdgeStatus es = ctx.ground->getProjectedEdgeStatus(
      start, end, ctx.robot_box_size, stop_at_unknown, projected_edge,
      is_hanging, preserve_start_height, &check, travel, physical_root);
  // The graph is undirected. Reversing the chassis heading moves a
  // noncentral footprint; validate that support and swept volume too.
  if (es == ProjectedEdgeStatus::kAdmissible && travel == EdgeTravel::kBothWays &&
      ctx.robot->center_offset.head<2>().squaredNorm() > 0.0) {
    const Eigen::Vector3d reverse_shift =
        ctx.robot->offsetForHeading(body.heading + M_PI) -
        ctx.robot->offsetForHeading(body.heading);
    body.heading += M_PI;
    std::vector<Eigen::Vector3d> reverse;
    es = ctx.ground->getProjectedEdgeStatus(
        end + reverse_shift, start + reverse_shift, ctx.robot_box_size,
        stop_at_unknown, reverse, is_hanging, false, &check, EdgeTravel::kBothWays);
  }
  ++rep.edge_status[static_cast<int>(es)];
  if (ctx.edge_verdicts != nullptr && !root_edge) {
    ctx.edge_verdicts->add(
        key, {es, es == ProjectedEdgeStatus::kAdmissible
                      ? projected_edge
                      : std::vector<Eigen::Vector3d>{}});
  }
  if (es == ProjectedEdgeStatus::kAdmissible) return true;
  if (es == ProjectedEdgeStatus::kSteep) ++rep.steep_edges;
  return false;
}


}  // namespace

void expandGraph(GraphManager& graph, Vertex& new_vertex,
                 ExpandGraphReport& rep, const ExpandContext& ctx,
                 bool allow_short_edge) {
  Vertex* nearest_vertex = nullptr;
  if (!graph.getNearestVertex(&new_vertex.state, &nearest_vertex) ||
      nearest_vertex == nullptr) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  expandGraphFrom(graph, new_vertex, nearest_vertex, rep, ctx,
                  allow_short_edge);
}

void expandGraphFrom(GraphManager& graph, Vertex& new_vertex,
                     Vertex* nearest_vertex, ExpandGraphReport& rep,
                     const ExpandContext& ctx, bool allow_short_edge) {
  StateVec new_state = new_vertex.state;
  if (nearest_vertex == nullptr) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }

  const Eigen::Vector3d origin(nearest_vertex->state[0],
                               nearest_vertex->state[1],
                               nearest_vertex->state[2]);
  Eigen::Vector3d direction(new_state[0] - origin[0], new_state[1] - origin[1],
                            new_state[2] - origin[2]);
  double direction_norm = direction.norm();

  const bool hanging_root =
      nearest_vertex->id == 0 && nearest_vertex->is_hanging &&
      std::isfinite(ctx.hanging_root_edge_length_max) &&
      ctx.hanging_root_edge_length_max > 0.0;
  // The physical root's blind-start allowance is an independent safety
  // bound. It may extend a short ordinary edge limit to reach first support,
  // or reduce a longer ordinary limit so configuration ordering can never
  // turn the hanging exception into an unbounded edge.
  const double effective_edge_length_max =
      hanging_root ? ctx.hanging_root_edge_length_max
                   : ctx.planning->edge_length_max;
  if (direction_norm > effective_edge_length_max) {
    direction = effective_edge_length_max * direction.normalized();
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
    // Projection can collapse a raw Z-offset sample into the root column.
    // Keep nondegenerate in-reach connectors: the controller's endpoint
    // tolerance may exceed the ordinary edge length, but not the whole route.
    if (nearest_vertex->id == 0 && !allow_short_edge &&
        direction.head<2>().norm() <= ctx.planning->edge_length_min) {
      rep.status = ExpandGraphStatus::kErrorShortEdge;
      return;
    }
    if (hanging_root &&
        direction_norm > ctx.hanging_root_edge_length_max + 1e-9) {
      rep.status = ExpandGraphStatus::kErrorCollisionEdge;
      return;
    }
    // The lattice precheck happens before ground projection, so its Z may not
    // describe the body box ultimately stored in the graph. Explicit
    // objectives cannot admit a waypoint that their refiner must immediately
    // reject at the projected driving height.
    if (ctx.strict_projected_endpoint) {
      rep.projected_endpoint_status = ctx.map->getStrictBoxStatus(
          new_state.head<3>() + ctx.robot->offsetForHeading(
              std::atan2(direction.y(), direction.x())), ctx.robot_box_size);
      if (rep.projected_endpoint_status != VoxelStatus::kFree) {
        rep.status = ExpandGraphStatus::kErrorCollisionEdge;
        return;
      }
    }
  }

  // Check the final aerial endpoint, including after edge-length clipping.
  // The unknown-body allowance belongs to ground exploration, never a hover
  // viewpoint (even when the endpoint still overlaps the physical root).
  if (ctx.robot->type == RobotType::kAerialRobot &&
      ctx.map->getStrictBoxStatus(new_state.head<3>() + ctx.robot->center_offset,
                                  ctx.robot_box_size) != VoxelStatus::kFree) {
    rep.status = ExpandGraphStatus::kErrorCollisionEdge;
    return;
  }

  // Overshoot both ends, except at the root, so an edge that just grazes an
  // obstacle is rejected.
  const Eigen::Vector3d overshoot =
      (direction_norm > 1e-12)
          ? Eigen::Vector3d(ctx.planning->edge_overshoot * direction.normalized())
          : Eigen::Vector3d::Zero();

  const Eigen::Vector3d offset = ctx.robot->offsetForHeading(
      std::atan2(direction.y(), direction.x()));
  Eigen::Vector3d start_pos = origin + offset;
  if (nearest_vertex->id != 0) start_pos -= overshoot;
  const Eigen::Vector3d end_pos =
      origin + offset + direction + overshoot;
  // The root is where the robot stands. The map cannot say the robot cannot
  // be there: a floor mapped a step higher around a robot resting in a dip,
  // or a wall's returns smeared into the footprint of a robot parked against
  // it, put occupied voxels inside its own body box and every edge out of
  // the lattice was refused (SubT hangar, 2026-09-21). The sweep out of the
  // root therefore starts where the robot's own footprint ends; the rest of
  // the edge, and every other edge, is checked in full.
  if (ctx.robot->type == RobotType::kGroundRobot &&
      nearest_vertex->id == 0 && ctx.root_footprint_exempt &&
      direction_norm > 1e-9) {
    const Eigen::Vector3d unit = direction / direction_norm;
    const double footprint =
        0.5 * (std::abs(unit.x()) * ctx.robot_box_size.x() +
               std::abs(unit.y()) * ctx.robot_box_size.y());
    if (footprint < direction_norm) start_pos += footprint * unit;
  }


  if (geofenceBlocks(ctx, start_pos, end_pos)) {
    rep.status = ExpandGraphStatus::kErrorGeofenceViolated;
    return;
  }

  std::vector<Eigen::Vector3d> projected_edge;
  const bool is_hanging = nearest_vertex->is_hanging || new_vertex.is_hanging;
  // An edge out of the root, when the root is where the robot stands, is
  // driven outwards only; any other lattice or roadmap edge either way.
  // Vertex zero is home in the roadmap and the goal in a goal lattice,
  // both of which the robot drives into.
  const bool aerial_root = ctx.robot->type == RobotType::kAerialRobot &&
                           nearest_vertex->id == 0 && ctx.root_is_robot &&
                           ctx.root_footprint_exempt;
  bool admissible_edge =
      aerial_root
          ? aerialRootDepartureTraversable(*ctx.map, start_pos, end_pos,
                                           ctx.robot_box_size)
          : edgeTraversable(
                ctx, start_pos, end_pos, is_hanging,
                ctx.preserve_hanging_root_start_height && nearest_vertex->id == 0,
                projected_edge, rep,
                ctx.stop_at_unknown &&
                    !(hanging_root && ctx.hanging_root_unknown_body),
                nearest_vertex->id == 0 && ctx.root_is_robot
                    ? EdgeTravel::kForward : EdgeTravel::kBothWays,
                nearest_vertex->id == 0 && ctx.root_is_robot && ctx.standing_body
                    ? &*ctx.standing_body : nullptr,
                nearest_vertex->id == 0,
                ctx.preserve_hanging_root_start_height &&
                        nearest_vertex->id == 0 && ctx.root_is_robot &&
                        nearest_vertex->is_hanging
                    ? &origin : nullptr);
  if (admissible_edge && ctx.projected_edge_admissible &&
      !ctx.projected_edge_admissible(projected_edge)) {
    admissible_edge = false;
  }
  if (admissible_edge && ctx.robot->type == RobotType::kGroundRobot) {
    recordProjectedEdge(ctx.projected_graph, projected_edge, ctx.robot_id);
  }

  // Reject long edges that also change height sharply.
  if (nearest_vertex->distance > ctx.planning->nearest_range_max &&
      std::fabs(nearest_vertex->state[2] - new_state[2]) >
          ctx.planning->nearest_range_z) {
    admissible_edge = false;
  }
  // Ground terrain was certified by getProjectedEdgeStatus's native window.
  // Rechecking its projected vertices as raw pairs would reject admissible
  // quantised ramps solely because a short edge straddles one riser.

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
  graph.addEdge(added, nearest_vertex,
                ctx.edge_cost ? ctx.edge_cost(origin, new_state.head<3>())
                              : direction_norm);
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
    const Eigen::Vector3d offset = ctx.robot->offsetForHeading(
        std::atan2(to_neighbour.y(), to_neighbour.x()));
    const Eigen::Vector3d p_start = new_origin + offset - p_overshoot;
    Eigen::Vector3d p_end = new_origin + offset + to_neighbour;
    if (neighbour->id != 0) p_end += p_overshoot;

    if (geofenceBlocks(ctx, p_start, p_end)) continue;

    std::vector<Eigen::Vector3d> neighbour_edge;
    if (!edgeTraversable(ctx, p_start, p_end, false, false, neighbour_edge,
                         rep, ctx.stop_at_unknown, EdgeTravel::kBothWays,
                         nullptr, neighbour->id == 0)) {
      continue;
    }
    if (ctx.projected_edge_admissible &&
        !ctx.projected_edge_admissible(neighbour_edge)) {
      continue;
    }
    if (ctx.robot->type == RobotType::kGroundRobot) {
      recordProjectedEdge(ctx.projected_graph, neighbour_edge, ctx.robot_id);
    }
    const double cost = ctx.edge_cost
                            ? ctx.edge_cost(new_origin, neighbour->state.head<3>())
                            : d_norm;
    graph.addEdge(added, neighbour, cost);
    ++rep.num_edges_added;
    if (ctx.inclinations != nullptr && !neighbour_edge.empty()) {
      ctx.inclinations->set(added->id, neighbour->id,
                            averageInclination(neighbour_edge));
    }
  }

  rep.status = ExpandGraphStatus::kSuccess;
}

static bool checkedExistingEdge(const ExpandContext& ctx, const Vertex& from,
                                const Vertex& to, ExpandGraphReport& rep,
                                std::vector<Eigen::Vector3d>* projected_edge,
                                bool stop_at_unknown) {
  const Eigen::Vector3d origin = from.state.head<3>();
  const Eigen::Vector3d direction = to.state.head<3>() - origin;
  const double d_norm = direction.norm();
  const Eigen::Vector3d p_overshoot =
      d_norm > 1e-12
          ? Eigen::Vector3d(direction / d_norm * ctx.planning->edge_overshoot)
          : Eigen::Vector3d::Zero();
  const Eigen::Vector3d offset = ctx.robot->offsetForHeading(
      std::atan2(direction.y(), direction.x()));
  const Eigen::Vector3d p_start = origin + offset - p_overshoot;
  Eigen::Vector3d p_end = origin + offset + direction;
  if (to.id != 0) p_end += p_overshoot;
  if (geofenceBlocks(ctx, p_start, p_end)) return false;
  std::vector<Eigen::Vector3d> edge;
  if (!edgeTraversable(ctx, p_start, p_end, false, false, edge, rep,
                       stop_at_unknown, EdgeTravel::kBothWays, nullptr,
                       from.id == 0 || to.id == 0)) {
    return false;
  }
  if (projected_edge != nullptr) *projected_edge = std::move(edge);
  return true;
}

bool roadmapEdgeTraversable(const ExpandContext& ctx, const Vertex& from,
                            const Vertex& to, ExpandGraphReport& rep,
                            std::vector<Eigen::Vector3d>* projected_edge) {
  return checkedExistingEdge(ctx, from, to, rep, projected_edge, true);
}

bool latticeEdgeTraversable(const ExpandContext& ctx, const Vertex& from,
                             const Vertex& to, ExpandGraphReport& rep) {
  std::vector<Eigen::Vector3d> edge;
  return checkedExistingEdge(ctx, from, to, rep, &edge, ctx.stop_at_unknown) &&
         (!ctx.projected_edge_admissible || ctx.projected_edge_admissible(edge));
}

void expandGraphEdges(GraphManager& graph, Vertex* new_vertex,
                      ExpandGraphReport& rep, const ExpandContext& ctx) {
  // rrg.cpp:867 Rrg::expandGraphEdges. The vertex is already in the graph;
  // only edges are added, and only where the map already knows the way is
  // clear (stop_at_unknown_voxel true at rrg.cpp:897 and 903), because the
  // roadmap these edges join is routed over without a second look.
  std::vector<Vertex*> nearest_vertices;
  if (!graph.getNearestVertices(&new_vertex->state,
                                ctx.planning->nearest_range,
                                &nearest_vertices)) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  const Eigen::Vector3d origin = new_vertex->state.head<3>();
  for (Vertex* neighbour : nearest_vertices) {
    if (neighbour == nullptr || neighbour == new_vertex) continue;
    const Eigen::Vector3d direction = neighbour->state.head<3>() - origin;
    const double d_norm = direction.norm();
    if (d_norm <= ctx.planning->edge_length_min ||
        d_norm >= ctx.planning->edge_length_max) {
      continue;
    }
    // Boost's setS edge list already refuses a duplicate; skipping it here
    // also keeps the adjacency map free of repeats.
    if (graph.graph_->edgeExists(new_vertex->id, neighbour->id)) continue;
    std::vector<Eigen::Vector3d> projected_edge;
    if (!roadmapEdgeTraversable(ctx, *new_vertex, *neighbour, rep,
                                &projected_edge)) {
      continue;
    }
    // rrg.cpp:907: a long way round the tree and a height change is a
    // different floor, not a shortcut.
    if (neighbour->distance > ctx.planning->nearest_range_max &&
        std::fabs(neighbour->state[2] - new_vertex->state[2]) >
            ctx.planning->nearest_range_z) {
      continue;
    }
    graph.addEdge(new_vertex, neighbour, d_norm);
    ++rep.num_edges_added;
    if (ctx.inclinations != nullptr && !projected_edge.empty()) {
      ctx.inclinations->set(new_vertex->id, neighbour->id,
                            averageInclination(projected_edge));
    }
  }
  rep.status = ExpandGraphStatus::kSuccess;
}

}  // namespace mgg
