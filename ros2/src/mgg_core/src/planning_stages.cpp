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
    result.reason = "could not solve the topological graph";
    return result;
  }

  if (!graph.getNearestVertexInRange(&goal, goal_vertex_tolerance_, &target)) {
    // getNearestVertexInRange reports an out-of-range nearest vertex through
    // its output pointer even when it returns false.  Do not let that vertex
    // masquerade as a selected proxy when no candidate makes progress.
    target = nullptr;
    if (request.objective != ObjectiveKind::kNavigate ||
        minimum_partial_progress_ <= 0.0) {
      char reason[192];
      std::snprintf(reason, sizeof(reason),
                    "goal is outside the graph tolerance %.2f m at "
                    "(%.2f, %.2f, %.2f)",
                    goal_vertex_tolerance_, goal.x(), goal.y(), goal.z());
      result.reason = reason;
      return result;
    }

    const double initial_remaining =
        (goal.head<2>() - current.head<2>()).norm();
    struct RankedProxy {
      Vertex* vertex;
      double remaining;
      double cost;
    };
    std::vector<RankedProxy> eligible;
    eligible.reserve(graph.vertices_map_.size());
    for (const auto& entry : graph.vertices_map_) {
      Vertex* candidate = entry.second;
      if (candidate == nullptr || candidate == source) continue;
      const auto distance = shortest.distance_map.find(candidate->id);
      if (distance == shortest.distance_map.end() ||
          !std::isfinite(distance->second) ||
          distance->second == std::numeric_limits<double>::max()) {
        continue;
      }
      const double remaining =
          (goal.head<2>() - candidate->state.head<2>()).norm();
      const double progress = initial_remaining - remaining;
      if (!std::isfinite(remaining) ||
          progress + 1e-9 < minimum_partial_progress_) {
        continue;
      }
      eligible.push_back({candidate, remaining, distance->second});
    }
    std::sort(eligible.begin(), eligible.end(),
              [](const RankedProxy& a, const RankedProxy& b) {
                return std::tie(a.remaining, a.cost, a.vertex->id) <
                       std::tie(b.remaining, b.cost, b.vertex->id);
              });
    for (const RankedProxy& candidate : eligible) {
      std::vector<StateVec> candidate_path;
      graph.getShortestPath(candidate.vertex->id, shortest, true,
                            candidate_path);
      if (candidate_path.size() >= 2 &&
          candidate_path.front().isApprox(source->state)) {
        target = candidate.vertex;
        result.poses = std::move(candidate_path);
        break;
      }
    }
    if (target == nullptr) {
      char reason[224];
      std::snprintf(reason, sizeof(reason),
                    "no reachable local proxy makes the required %.2f m "
                    "progress toward the exact goal",
                    minimum_partial_progress_);
      result.reason = reason;
      return result;
    }
    result.partial = true;
    // Face the remaining objective at the segment boundary. The exact target
    // pose and its requested yaw remain untouched in result.request.goal.
    result.poses.back()[3] =
        std::atan2(goal.y() - target->state.y(),
                   goal.x() - target->state.x());
  } else {
    graph.getShortestPath(target->id, shortest, true, result.poses);
  }
  if (result.poses.empty() || !result.poses.front().isApprox(source->state)) {
    result.poses.clear();
    result.reason = "goal is in a disconnected graph component";
    return result;
  }
  result.status = PlanningStatus::kSucceeded;
  return result;
}

}  // namespace mgg
