#include "mgg_core/planning_stages.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <tuple>
#include <utility>

namespace mgg {

TopologicalGoalPlanner::TopologicalGoalPlanner(
    std::string component_id, std::uint64_t graph_revision,
    std::uint64_t map_revision, double goal_vertex_tolerance,
    double minimum_partial_progress)
    : component_id_(std::move(component_id)),
      graph_revision_(graph_revision),
      map_revision_(map_revision),
      goal_vertex_tolerance_(std::max(0.0, goal_vertex_tolerance)),
      minimum_partial_progress_(std::max(0.0, minimum_partial_progress)) {}

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
  // Both ends must bind to the active graph.  An unbounded source lookup can
  // turn a lone home landmark into an apparently valid one-pose route after
  // the robot has travelled far beyond the last admitted breadcrumb.
  if (!graph.getNearestVertexInRange(&current, goal_vertex_tolerance_,
                                     &source)) {
    if (request.objective == ObjectiveKind::kNavigate) {
      result.status = PlanningStatus::kSucceeded;
      result.reason.clear();
      // Bootstrap an unvalidated global corridor from the live pose. The
      // rolling grid stage exposes and checks only its bounded first window.
      result.poses = {current, goal};
      return result;
    }
    char reason[192];
    std::snprintf(reason, sizeof(reason),
                  "current pose is outside the graph tolerance %.2f m at "
                  "(%.2f, %.2f, %.2f)",
                  goal_vertex_tolerance_, current.x(), current.y(),
                  current.z());
    result.reason = reason;
    return result;
  }
  ShortestPathsReport shortest;
  if (!graph.findShortestPaths(source->id, shortest)) {
    if (request.objective == ObjectiveKind::kNavigate) {
      result.status = PlanningStatus::kSucceeded;
      result.reason.clear();
      result.poses = {current, goal};
      return result;
    }
    result.reason = "could not solve the topological graph";
    return result;
  }

  if (!graph.getNearestVertexInRange(&goal, goal_vertex_tolerance_, &target)) {
    target = nullptr;
    if (request.objective != ObjectiveKind::kNavigate) {
      char reason[192];
      std::snprintf(reason, sizeof(reason),
                    "goal is outside the graph tolerance %.2f m at "
                    "(%.2f, %.2f, %.2f)",
                    goal_vertex_tolerance_, goal.x(), goal.y(), goal.z());
      result.reason = reason;
      return result;
    }
    // The prefix is only a seed for exact bounded grid completion. It is never
    // returned as a successful proxy. An empty prefix asks the grid stage to
    // search directly from current to the exact request-owned goal.
    const double initial_remaining =
        (goal.head<2>() - current.head<2>()).norm();
    struct RankedSeed {
      Vertex* vertex;
      double remaining;
      double graph_cost;
    };
    std::vector<RankedSeed> eligible;
    eligible.reserve(graph.vertices_map_.size());
    for (const auto& entry : graph.vertices_map_) {
      Vertex* candidate = entry.second;
      if (candidate == nullptr || candidate == source) continue;
      const auto distance = shortest.distance_map.find(candidate->id);
      if (distance == shortest.distance_map.end() ||
          !std::isfinite(distance->second) ||
          distance->second == std::numeric_limits<double>::max()) continue;
      const double remaining =
          (goal.head<2>() - candidate->state.head<2>()).norm();
      if (!std::isfinite(remaining) ||
          initial_remaining - remaining + 1e-9 < minimum_partial_progress_)
        continue;
      eligible.push_back({candidate, remaining, distance->second});
    }
    std::sort(eligible.begin(), eligible.end(),
              [](const RankedSeed& a, const RankedSeed& b) {
                const double a_total = a.graph_cost + a.remaining;
                const double b_total = b.graph_cost + b.remaining;
                return std::tie(a_total, a.remaining, a.vertex->id) <
                       std::tie(b_total, b.remaining, b.vertex->id);
              });
    for (const RankedSeed& candidate : eligible) {
      std::vector<StateVec> candidate_path;
      graph.getShortestPath(candidate.vertex->id, shortest, true,
                            candidate_path);
      if (!candidate_path.empty() &&
          candidate_path.front().isApprox(source->state)) {
        result.poses = std::move(candidate_path);
        break;
      }
    }
    if (result.poses.empty()) {
      // Even when no existing vertex advances toward the goal, anchor the
      // tentative connector at the current graph vertex.  This keeps an
      // arbitrary distant Navigate in the persistent-route hierarchy instead
      // of handing the complete distance to the bounded local grid.
      result.poses.push_back(source->state);
    }
    // The persistent graph is the global route authority.  Its best reachable
    // vertex is followed by an optimistic connector to the request-owned goal;
    // rolling local refinement will validate this connector as the robot
    // approaches it.  Keeping the exact goal here prevents a nearby graph
    // vertex from being mistaken for successful Navigate completion.
    if (!result.poses.empty() &&
        !result.poses.back().head<3>().isApprox(goal.head<3>())) {
      result.poses.push_back(goal);
    }
    result.status = PlanningStatus::kSucceeded;
    result.partial = false;
    result.reason.clear();
    return result;
  } else {
    graph.getShortestPath(target->id, shortest, true, result.poses);
  }
  if (result.poses.empty() || !result.poses.front().isApprox(source->state)) {
    result.poses.clear();
    if (request.objective == ObjectiveKind::kNavigate) {
      result.status = PlanningStatus::kSucceeded;
      result.reason.clear();
      result.poses = {current, goal};
      return result;
    }
    result.reason = "goal is in a disconnected graph component";
    return result;
  }
  result.status = PlanningStatus::kSucceeded;
  return result;
}

}  // namespace mgg
