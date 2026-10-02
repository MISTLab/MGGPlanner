#include "mgg_core/local_route.h"

#include <algorithm>
#include <cmath>

#include "mgg_core/departure.h"
#include "mgg_core/global_graph.h"
#include "mgg_core/ground_projection.h"

namespace mgg {

StateVec localRouteRoot(const ExpandContext& ctx, const StateVec& robot_pose,
                        bool& hanging) {
  StateVec root = robot_pose;
  hanging = false;
  if (ctx.robot->type != RobotType::kGroundRobot) return root;
  Eigen::Vector3d pos = robot_pose.head<3>();
  VoxelStatus status = VoxelStatus::kUnknown;
  const double ground_height = ctx.ground->projectSample(pos, status);
  if (status == VoxelStatus::kOccupied) {
    root[0] = pos.x();
    root[1] = pos.y();
    root[2] = pos.z() - (ground_height - ctx.planning->max_ground_height);
    return root;
  }
  // The base sits half the body height above the floor it stands on; the
  // driving height is max_ground_height above that floor.
  hanging = true;
  root = robot_pose;
  root[2] += ctx.planning->max_ground_height - ctx.robot->size[2] / 2.0;
  return root;
}

LocalRouteResult routeOverLocalLattice(GraphManager& graph,
                                       const StateVec& robot_pose,
                                       const StateVec& goal,
                                       const GridGraphParams& grid,
                                       const ExpandContext& ctx) {
  LocalRouteResult result;
  graph.reset();
  if (ctx.inclinations != nullptr) ctx.inclinations->clear();
  bool root_hanging = false;
  const StateVec root_state = localRouteRoot(ctx, robot_pose, root_hanging);
  auto* root = new Vertex(0, root_state);
  root->robot_id = ctx.robot_id;
  root->is_hanging = root_hanging;
  graph.addVertex(root);
  result.lattice = buildGridGraph(graph, root_state, grid, ctx, robot_pose[3]);
  if (result.lattice.status == GridGraphStatus::kInvalidBounds ||
      graph.getNumVertices() <= 1) {
    result.reason = "the local lattice holds no admissible cell";
    return result;
  }
  PlanProfile* const profile = ctx.ground ? ctx.ground->profile() : nullptr;
  StateVec goal_state = goal;
  if (ctx.robot->type == RobotType::kGroundRobot) {
    Eigen::Vector3d pos = goal.head<3>();
    VoxelStatus status = VoxelStatus::kUnknown;
    const double ground_height = ctx.ground->projectGoal(pos, status);
    if (status != VoxelStatus::kOccupied) {
      result.reason = "no mapped ground under the goal";
      return result;
    }
    goal_state[0] = pos.x();
    goal_state[1] = pos.y();
    goal_state[2] = pos.z() - (ground_height - ctx.planning->max_ground_height);
  }
  Vertex* goal_vertex = nullptr;
  {
    ProfileScope timed(profile ? &profile->goal_link : nullptr);
    // Only vertices the robot reaches can carry the route.
    ShortestPathsReport from_root;
    const bool searched = graph.findShortestPaths(0, from_root);
    goal_vertex = linkGoalToLattice(
        graph, goal_state, ctx, [&](const Vertex& v) {
          if (!searched || !from_root.status) return true;
          if (v.id == 0) return true;
          const auto it = from_root.parent_id_map.find(v.id);
          return it != from_root.parent_id_map.end() && it->second != v.id;
        });
  }
  if (goal_vertex == nullptr) {
    result.reason = "goal cannot be linked to the local lattice";
    return result;
  }
  ShortestPathsReport rep;
  bool searched = false;
  {
    ProfileScope timed(profile ? &profile->search : nullptr);
    searched = graph.findShortestPaths(0, rep);
  }
  if (!searched || !rep.status ||
      (goal_vertex->id != 0 &&
       rep.parent_id_map.find(goal_vertex->id) == rep.parent_id_map.end())) {
    result.reason = "no route through the local lattice reaches the goal";
    return result;
  }
  graph.getShortestPath(goal_vertex->id, rep, true, result.route);
  if (result.route.size() < 2) {
    result.route.clear();
    result.reason = "already at the goal";
    return result;
  }
  result.routed = true;
  return result;
}

bool groundShortcutSegmentAdmissible(const ExpandContext& ctx,
                                     const Eigen::Vector3d& from,
                                     const Eigen::Vector3d& to,
                                     bool stop_at_unknown) {
  OrientedBox body;
  body.heading = std::atan2(to.y() - from.y(), to.x() - from.x());
  body.size = ctx.robot_box_size;
  EdgeBodyCheck check;
  check.sweep = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return orientedBoxPathStatus(*ctx.map, a, b, body, stop_at_unknown,
                                 nullptr);
  };
  std::vector<Eigen::Vector3d> projected;
  // Driven from `from` to `to`: the shortcut is the path itself.
  return ctx.ground->getProjectedEdgeStatus(
             from + ctx.robot->center_offset, to + ctx.robot->center_offset,
             ctx.robot_box_size, stop_at_unknown, projected, false, false,
             &check, EdgeTravel::kForward) == ProjectedEdgeStatus::kAdmissible;
}

Vertex* linkGoalToLattice(GraphManager& graph, const StateVec& goal_state,
                          const ExpandContext& ctx,
                          const std::function<bool(const Vertex&)>& reached) {
  Vertex* linked = connectStateToGraph(graph, goal_state, ctx,
                                       kLocalGoalLinkRadius,
                                       /*exact_state=*/true);
  if (linked != nullptr) return linked;
  std::vector<Vertex*> around;
  if (!graph.getNearestVertices(&goal_state, ctx.planning->edge_length_max,
                                &around)) {
    return nullptr;
  }
  std::sort(around.begin(), around.end(),
            [&goal_state](const Vertex* a, const Vertex* b) {
              return (a->state.head<3>() - goal_state.head<3>()).squaredNorm() <
                     (b->state.head<3>() - goal_state.head<3>()).squaredNorm();
            });
  int tried = 0;
  for (Vertex* parent : around) {
    if (parent == nullptr || (reached && !reached(*parent))) continue;
    if (tried++ >= kGoalLinkCandidates) break;
    ExpandGraphReport rep;
    Vertex candidate(-1, goal_state);
    expandGraphFrom(graph, candidate, parent, rep, ctx,
                    /*allow_short_edge=*/true);
    if (rep.status != ExpandGraphStatus::kSuccess || rep.vertex_added == nullptr) {
      continue;
    }
    // Within edge_length_max nothing is clipped; the projected endpoint is
    // the goal's own driving height.
    if ((rep.vertex_added->state.head<3>() - goal_state.head<3>())
            .squaredNorm() <= 1e-12) {
      return rep.vertex_added;
    }
  }
  return nullptr;
}

}  // namespace mgg
