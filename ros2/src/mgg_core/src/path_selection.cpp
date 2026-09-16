#include "mgg_core/path_selection.h"

#include <algorithm>
#include <cmath>

#include "mgg_core/trajectory.h"

namespace mgg {

PathSelectionResult selectBestPath(GraphManager& graph,
                                   const PlanningParams& planning,
                                   const RobotParams& robot,
                                   const EdgeInclinations& inclinations,
                                   double map_resolution,
                                   double exploring_direction,
                                   const std::vector<Eigen::Vector3d>&
                                       excluded_endpoints,
                                   double exclusion_radius) {
  PathSelectionResult result;

  ShortestPathsReport rep;
  if (!graph.findShortestPaths(rep)) return result;
  graph.findLeafVertices(rep);
  std::vector<Vertex*> leaves;
  graph.getLeafVertices(leaves);

  for (Vertex* leaf : leaves) {
    if (leaf == nullptr) continue;
    std::vector<Vertex*> path;
    graph.getShortestPath(leaf->id, rep, true, path);
    if (path.size() <= 1) continue;  // needs at least root and leaf
    const bool excluded = std::any_of(
        excluded_endpoints.begin(), excluded_endpoints.end(),
        [leaf, exclusion_radius](const Eigen::Vector3d& center) {
          return (leaf->state.head(3) - center).norm() <= exclusion_radius;
        });
    if (excluded) continue;
    ++result.leaves_evaluated;

    double path_gain = 0.0;
    bool inadmissible = false;

    for (size_t ind = 0; ind < path.size(); ++ind) {
      Vertex* v = path[ind];
      const double path_length = graph.getShortestDistance(v->id, rep);
      // Hanging vertices are worth less: there is no ground under them.
      const double vol_gain =
          v->vol_gain.gain *
          std::exp(-static_cast<double>(v->is_hanging) *
                   planning.hanging_vertex_penalty);

      if (ind > 0 && robot.type == RobotType::kGroundRobot) {
        const Eigen::Vector3d segment =
            path[ind]->state.head(3) - path[ind - 1]->state.head(3);
        // Only descents are checked: driving down a steep slope is what the
        // robot cannot recover from.
        if ((path[ind]->state(2) - path[ind - 1]->state(2)) < -std::max(map_resolution, planning.max_step_height)) {
          const double stored =
              inclinations.get(path[ind]->id, path[ind - 1]->id);
          const double geometric =
              std::atan2(std::abs(segment(2)), segment.head(2).norm());
          if (stored > planning.max_negative_inclination ||
              geometric > planning.max_negative_inclination) {
            inadmissible = true;
            break;
          }
        }
      }

      // Gain discounted by how far the robot must travel to collect it.
      path_gain += vol_gain * std::exp(-planning.path_length_penalty * path_length);
      v->vol_gain.accumulative_gain = path_gain;
      if (v->vol_gain.is_frontier && !v->is_hanging) {
        result.frontier_exists = true;
      }
    }

    if (inadmissible) {
      ++result.paths_rejected_steep;
      continue;
    }

    // Penalise paths that head away from the direction of travel.
    std::vector<Eigen::Vector3d> path_points;
    graph.getShortestPath(leaf->id, rep, true, path_points);
    const double deviation = computeDistanceBetweenTrajectoryAndDirection(
        path_points, exploring_direction, 0.2, true);
    path_gain *= std::exp(-planning.path_direction_penalty * deviation);

    if (path_gain > result.best_gain) {
      result.best_gain = path_gain;
      result.best_path_id = leaf->id;
      result.best_path = path;
    }
  }

  // Re-parent along the winning path so the branch can be walked from the root.
  if (result.best_path_id >= 0) {
    std::vector<int> ids;
    graph.getShortestPath(result.best_path_id, rep, false, ids);
    for (size_t i = 0; i + 1 < ids.size(); ++i) {
      Vertex* child = graph.getVertex(ids[i]);
      Vertex* parent = graph.getVertex(ids[i + 1]);
      if (child != nullptr && parent != nullptr) child->parent = parent;
    }
  }
  return result;
}

}  // namespace mgg
