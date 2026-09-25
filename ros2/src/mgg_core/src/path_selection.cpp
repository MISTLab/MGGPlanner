#include "mgg_core/path_selection.h"

#include <algorithm>
#include <cmath>

#include "mgg_core/trajectory.h"

namespace mgg {

bool viewpointClear(const MapInterface& map, const RobotParams& robot,
                    const PlanningParams& planning,
                    const StateVec& viewpoint) {
  const double radius = 0.5 * std::min(robot.size.x(), robot.size.y()) +
                        planning.viewpoint_clearance_margin;
  const Eigen::Vector3d center = viewpoint.head<3>() + robot.center_offset;
  return map.getOccupiedOnlyCylinderPathStatus(
             center, center, radius, robot.getPlanningSize().z()) !=
         VoxelStatus::kOccupied;
}

bool pullBackToClearViewpoint(
    std::vector<StateVec>& route,
    const std::function<bool(const StateVec&)>& clear) {
  std::size_t end = route.empty() ? 0 : route.size() - 1;
  while (end > 0 && !clear(route[end])) --end;
  if (end == 0) return false;
  route.resize(end + 1);
  return true;
}

PathSelectionResult selectBestPath(GraphManager& graph,
                                   const PlanningParams& planning,
                                   const RobotParams& robot,
                                   const EdgeInclinations& inclinations,
                                   double map_resolution,
                                   double exploring_direction,
                                   const std::vector<Eigen::Vector3d>&
                                       excluded_endpoints,
                                   double exclusion_radius,
                                   const ViewpointClearFn& viewpoint_clear,
                                   const PathTurnsFn& turns_admissible,
                                   const SharpTurnAllowedFn&
                                       sharp_turn_allowed) {
  // Leaves share their paths' inner vertices; check each vertex once.
  std::unordered_map<int, bool> clear_by_id;
  const auto clear = [&](const Vertex* v) {
    const auto found = clear_by_id.find(v->id);
    if (found != clear_by_id.end()) return found->second;
    return clear_by_id[v->id] = viewpoint_clear(*v);
  };

  ShortestPathsReport rep;
  if (!graph.findShortestPaths(rep)) return PathSelectionResult();
  graph.findLeafVertices(rep);
  std::vector<Vertex*> leaves;
  graph.getLeafVertices(leaves);

  // A candidate is a route from the root with the edge cost to each of its
  // vertices: every leaf's shortest path and, before a fallback, the routes
  // findTurnCompliantRoutes finds.
  using Candidate = TurnCompliantRoutes::Route;
  std::vector<Candidate> shortest;
  for (Vertex* leaf : leaves) {
    if (leaf == nullptr) continue;
    Candidate candidate;
    graph.getShortestPath(leaf->id, rep, true, candidate.path);
    for (const Vertex* v : candidate.path) {
      candidate.along.push_back(graph.getShortestDistance(v->id, rep));
    }
    shortest.push_back(std::move(candidate));
  }

  // One selection over `candidates`; with `check_turns`, a candidate that
  // fails turns_admissible is not admissible.
  const auto choose = [&](bool check_turns,
                          const std::vector<Candidate>& candidates) {
    PathSelectionResult result;
    const auto turns_ok = [&](const std::vector<Vertex*>& path) {
      if (!check_turns || turns_admissible(path)) return true;
      ++result.paths_with_sharp_turns;
      return false;
    };
    // Scores one root-to-viewpoint path: accumulated gain discounted for
    // length and for heading away from the direction of travel. Returns false
    // for a path that descends too steeply.
    const auto score = [&](const std::vector<Vertex*>& path,
                           const std::vector<double>& along, double& gain) {
      double path_gain = 0.0;
      for (size_t ind = 0; ind < path.size(); ++ind) {
        Vertex* v = path[ind];
        const double path_length = along[ind];
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
              return false;
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

      // Penalise paths that head away from the direction of travel.
      std::vector<Eigen::Vector3d> path_points;
      path_points.reserve(path.size());
      for (const Vertex* v : path) path_points.push_back(v->state.head(3));
      const double deviation = computeDistanceBetweenTrajectoryAndDirection(
          path_points, exploring_direction, 0.2, true);
      gain = path_gain * std::exp(-planning.path_direction_penalty * deviation);
      return true;
    };
    const auto excluded = [&](const Vertex* viewpoint) {
      return std::any_of(
          excluded_endpoints.begin(), excluded_endpoints.end(),
          [viewpoint, exclusion_radius](const Eigen::Vector3d& center) {
            return (viewpoint->state.head(3) - center).norm() <=
                   exclusion_radius;
          });
    };

    // Clearance is a preference that always wins while it can be had: the best
    // path ending with room for the robot, pulled back to its last clear vertex
    // where needed, is chosen whenever there is one, even when the prefix it
    // was pulled back to carries no gain of its own (with leaf-only gain it
    // seldom does); between equal clear candidates, the one whose full path is
    // worth more wins. Only when no path ends clear, e.g. in a passage narrower
    // than the robot plus twice the margin, is the best path chosen as without
    // the check, so exploration goes on wherever it went on before. With no
    // gain anywhere there is still no path.
    bool have_clear = false;
    double best_clear_gain = 0.0;
    double best_clear_full_gain = 0.0;
    std::vector<Vertex*> best_clear_path;
    for (const Candidate& candidate : candidates) {
      std::vector<Vertex*> path = candidate.path;
      if (path.size() <= 1) continue;  // needs at least root and leaf
      const Vertex* leaf = path.back();
      // Reservations are checked where each candidate ends: the leaf for the
      // path as it is, the vertex it is pulled back to for the clear one.
      const bool leaf_excluded = excluded(leaf);
      double path_gain = 0.0;
      bool admissible = false;
      if (!leaf_excluded) {
        ++result.leaves_evaluated;
        admissible = score(path, candidate.along, path_gain);
        if (!admissible) {
          ++result.paths_rejected_steep;
        } else if (!turns_ok(path)) {
          admissible = false;
        } else if (path_gain > result.best_gain) {
          result.best_gain = path_gain;
          result.best_path = path;
        }
      }
      const double full_gain = admissible ? path_gain : 0.0;
      if (!viewpoint_clear) continue;

      // The robot stops where the path ends; the root is where it stands.
      std::size_t end = path.size() - 1;
      while (end > 0 && !clear(path[end])) --end;
      if (end == 0) {
        ++result.paths_without_clear_viewpoint;
        continue;
      }
      if (end + 1 < path.size()) {
        path.resize(end + 1);
        ++result.paths_pulled_back;
        admissible = !excluded(path.back()) && score(path, candidate.along, path_gain) &&
                     turns_ok(path);
        // Toward a reserved leaf, only gain outside the reservation counts.
        if (leaf_excluded && !(path_gain > 0.0)) continue;
      } else if (leaf_excluded) {
        continue;
      }
      if (admissible &&
          (!have_clear || path_gain > best_clear_gain ||
           (path_gain == best_clear_gain && full_gain > best_clear_full_gain))) {
        have_clear = true;
        best_clear_gain = path_gain;
        best_clear_full_gain = full_gain;
        best_clear_path = path;
      }
    }
    // A branch whose only gain lies in a peer's reservation is not pursued,
    // not even through a clear prefix: that is what reservations prevent.
    if (viewpoint_clear &&
        (!result.best_path.empty() || best_clear_gain > 0.0)) {
      if (!have_clear) {
        result.unclear_viewpoint = true;
      } else {
        result.best_gain = best_clear_gain;
        result.best_path = best_clear_path;
      }
    }
    return result;
  };

  // A path that turns sharply where it may not is taken only when no other
  // path would be chosen at all, and then as the selection without the
  // check makes it, so that the rule never stops exploration where it
  // would have gone on.
  PathSelectionResult result =
      choose(static_cast<bool>(turns_admissible), shortest);
  if (result.best_path.empty() && result.paths_with_sharp_turns > 0) {
    const int refused = result.paths_with_sharp_turns;
    // Before giving up on the rule: the gain the refused paths lead to may
    // be reached another way, turning where it may.
    PathSelectionResult detour;
    int states = 0;
    bool capped = false;
    if (sharp_turn_allowed) {
      std::vector<int> destinations;
      for (const auto& [id, v] : graph.vertices_map_) {
        if (id != 0 && v != nullptr && v->vol_gain.gain > 0.0) {
          destinations.push_back(id);
        }
      }
      const Vertex* root = graph.getVertex(0);
      const TurnCompliantRoutes routes = findTurnCompliantRoutes(
          graph, root != nullptr ? root->state[3] : 0.0,
          std::max(robot.size.x(), robot.size.y()), destinations,
          sharp_turn_allowed, kMaxDetourSearchStates);
      states = routes.states_expanded;
      capped = routes.capped;
      std::vector<Candidate> candidates;
      candidates.reserve(routes.to.size());
      for (const int id : destinations) {
        const auto found = routes.to.find(id);
        if (found != routes.to.end()) candidates.push_back(found->second);
      }
      detour = choose(true, candidates);
    }
    if (!detour.best_path.empty()) {
      result = detour;
      result.sharp_turn_detour = true;
    } else {
      result = choose(false, shortest);
      result.sharp_turn_fallback = !result.best_path.empty();
    }
    result.paths_with_sharp_turns = refused;
    result.detour_states_expanded = states;
    result.detour_search_capped = capped;
  }
  if (!result.best_path.empty()) {
    result.best_path_id = result.best_path.back()->id;
  }

  // Re-parent along the winning path so the branch can be walked from the
  // root. The path is the leaf's shortest path, or a detour.
  for (size_t i = 1; i < result.best_path.size(); ++i) {
    result.best_path[i]->parent = result.best_path[i - 1];
  }
  return result;
}

}  // namespace mgg
