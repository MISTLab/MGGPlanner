#include "mgg_core/planning_stages.h"

#include <algorithm>
#include <utility>

namespace mgg {

TopologicalGoalPlanner::TopologicalGoalPlanner(
    std::string component_id, std::uint64_t graph_revision,
    std::uint64_t map_revision, double goal_vertex_tolerance)
    : component_id_(std::move(component_id)),
      graph_revision_(graph_revision),
      map_revision_(map_revision),
      goal_vertex_tolerance_(std::max(0.0, goal_vertex_tolerance)) {}

RouteCorridor TopologicalGoalPlanner::plan(
    GraphManager& graph, const StateVec& current,
    const PlanningRequest& request) const {
  RouteCorridor result;
  result.request = request;
  if (request.objective != ObjectiveKind::kNavigate &&
      request.objective != ObjectiveKind::kReturnHome) {
    result.status = PlanningStatus::kUnsupportedObjective;
    result.reason = "objective is not an explicit topological goal";
    return result;
  }
  if (!current.allFinite() || !request.goal.pose.allFinite()) {
    result.status = PlanningStatus::kUnsupportedObjective;
    result.reason = "current pose and goal must contain only finite values";
    return result;
  }
  if (request.component_id != component_id_ ||
      request.graph_revision != graph_revision_ ||
      request.map_revision != map_revision_) {
    result.status = PlanningStatus::kStaleRevision;
    result.reason = "request revisions do not match the active planning snapshot";
    return result;
  }

  Vertex* source = nullptr;
  Vertex* target = nullptr;
  StateVec goal = request.goal.pose;
  if (!graph.getNearestVertex(&current, &source) ||
      !graph.getNearestVertexInRange(&goal, goal_vertex_tolerance_, &target)) {
    result.reason = "current pose or goal is outside the graph";
    return result;
  }

  ShortestPathsReport shortest;
  if (!graph.findShortestPaths(source->id, shortest)) {
    result.reason = "could not solve the topological graph";
    return result;
  }
  graph.getShortestPath(target->id, shortest, true, result.poses);
  if (result.poses.empty() || !result.poses.front().isApprox(source->state)) {
    result.poses.clear();
    result.reason = "goal is in a disconnected graph component";
    return result;
  }
  result.status = PlanningStatus::kSucceeded;
  return result;
}

}  // namespace mgg
