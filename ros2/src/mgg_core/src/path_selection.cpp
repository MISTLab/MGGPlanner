#include "mgg_core/planning_cancellation.h"
#include "mgg_core/path_selection.h"

#include <algorithm>
#include <cmath>

#include "mgg_core/departure.h"
#include "mgg_core/trajectory.h"

namespace mgg {

bool viewpointClear(const MapInterface& map, const RobotParams& robot,
                    const PlanningParams& planning, const StateVec& viewpoint,
                    double slope) {
  // Lattice vertices carry a build-time yaw, not an arrival yaw. Ground
  // viewpoints use the reference-centred envelope covering every spin centre.
  // This is larger than turnClear's heading-aware chassis-centred circle.
  double radius = 0.0;
  if (robot.type == RobotType::kGroundRobot) {
    const double envelope = robot.turningRadius() +
        robot.physicalOffsetForHeading(0).head<2>().norm();
    radius = std::max(envelope + kViewpointArrivalSlack +
                          planning.viewpoint_clearance_margin, envelope);
  } else {
    const double drone_radius = 0.5 * robot.size.head<2>().norm();
    radius = drone_radius +
             std::max(0.0, planning.aerial_viewpoint_clearance_margin);
  }
  // Where it can turn, as roomToTurn has it: with the space it would turn
  // in observed (item 7). Not on a measured slope, where it may not turn.
  StateVec unknown_arrival = viewpoint;
  unknown_arrival[3] = kUnknownTurnHeading;
  if (robot.type == RobotType::kGroundRobot && !slopeExemptsTurnSpace(slope) &&
      !turnSpaceObserved(map, robot, planning, unknown_arrival)) {
    return false;
  }
  Eigen::Vector3d center = viewpoint.head<3>() + robot.center_offset;
  if (robot.type == RobotType::kGroundRobot) center.head<2>() = viewpoint.head<2>();
  return map.getOccupiedOnlyCylinderPathStatus(
             center, center, radius, robot.getPlanningSize().z()) !=
         VoxelStatus::kOccupied;
}

bool pathGoesNowhere(const PathSelectionResult& selection,
                     const Eigen::Vector3d& robot, double reach) {
  if (selection.best_path.empty() || selection.best_path.back() == nullptr) {
    return false;
  }
  const Eigen::Vector2d end = selection.best_path.back()->state.head<2>();
  return (end - robot.head<2>()).norm() <= reach ||
         !(selection.best_full_gain > 0.0);
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

double pathDirectionFactor(const std::vector<Eigen::Vector3d>& path,
                           double exploring_direction,
                           const PlanningParams& planning, bool bounded) {
  const double deviation = computeDistanceBetweenTrajectoryAndDirection(
      path, exploring_direction, 0.2, true);
  const double factor = std::exp(-planning.path_direction_penalty * deviation);
  return bounded ? std::max(planning.path_direction_min_factor, factor)
                 : factor;
}

bool pathTurnsBack(const std::vector<Eigen::Vector3d>& path,
                   double exploring_direction,
                   const PlanningParams& planning) {
  if (path.size() < 2 || !(planning.path_direction_min_factor > 0.0)) {
    return false;
  }
  const Eigen::Vector2d heading(std::cos(exploring_direction),
                                std::sin(exploring_direction));
  return (path.back() - path.front()).head<2>().dot(heading) < 0.0;
}

PlanningParams TurnBackHysteresis::selectionParams(
    const PlanningParams& planning) const {
  PlanningParams params = planning;
  if (last_turned_back_) params.path_direction_min_factor = 0.0;
  return params;
}

void TurnBackHysteresis::record(const std::vector<Eigen::Vector3d>& path,
                                double exploring_direction,
                                const PlanningParams& planning) {
  last_turned_back_ = pathTurnsBack(path, exploring_direction, planning);
}

bool cutBackToWayBack(std::vector<StateVec>& route,
                      const std::function<bool(std::size_t)>& on_slope,
                      const std::function<bool(std::size_t)>& refuge_admissible,
                      bool reverse_allowed,
                      const std::function<bool(std::size_t, std::size_t)>& reverse_edge_admissible,
                      double max_reverse_length) {
  if (route.size() < 2) return false;
  std::vector<double> along(route.size(), 0.0);
  for (std::size_t i = 1; i < route.size(); ++i) {
    planningCheckpoint();
    along[i] = along[i - 1] +
               (route[i].head<3>() - route[i - 1].head<3>()).norm();
  }
  std::vector<signed char> room(route.size(), -1);
  const auto room_at = [&](std::size_t i) {
    if (room[i] < 0) room[i] = refuge_admissible(i) ? 1 : 0;
    return room[i] == 1;
  };
  for (std::size_t end = route.size() - 1; end > 0; --end) {
    planningCheckpoint();
    bool way_back = !on_slope(end);
    for (std::size_t i = end; reverse_allowed && !way_back && i-- > 0;) {
      planningCheckpoint();
      const double limit = reverse_edge_admissible
          ? (max_reverse_length > 0.0 ? max_reverse_length : kDepartureMaxM) : kDepartureMaxM;
      if (along[end] - along[i] > limit + 1e-9) break;
      if (reverse_edge_admissible && !reverse_edge_admissible(i + 1, i)) break;
      way_back = room_at(i);
    }
    if (way_back) {
      route.resize(end + 1);
      return true;
    }
  }
  return false;
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
                                       sharp_turn_allowed,
                                   double goal_reach,
                                   const SlopeEndRetreat& slope_end_retreat,
                                   const EndExcludedFn& end_excluded,
                                   const PathTurnsFn& path_admissible) {
  // Leaves share their paths' inner vertices; check each vertex once.
  std::unordered_map<int, bool> clear_by_id;
  const auto clear = [&](const Vertex* v) {
    const auto found = clear_by_id.find(v->id);
    if (found != clear_by_id.end()) return found->second;
    return clear_by_id[v->id] = viewpoint_clear(*v);
  };
  const bool retreat_checked =
      static_cast<bool>(slope_end_retreat.admitted_on_slope) &&
      static_cast<bool>(slope_end_retreat.refuge_admissible);
  std::unordered_map<int, bool> on_slope_by_id;
  const auto cached = [](std::unordered_map<int, bool>& by_id,
                         const Vertex* v, const auto& ask) {
    const auto found = by_id.find(v->id);
    if (found != by_id.end()) return found->second;
    return by_id[v->id] = ask(*v);
  };
  // Whether path[end] may end the path: a narrow or slope-exempt end
  // needs a reverse way back to turn room. Extending beyond the old two
  // metres requires explicit reverse-edge validation. A refusal counts in
  // `refused`, when given.
  const auto way_back = [&](const std::vector<Vertex*>& path,
                            std::size_t end, int* refused) {
    if (!retreat_checked) return true;
    const bool needs_retreat =
        (viewpoint_clear && !clear(path[end])) ||
        cached(on_slope_by_id, path[end], slope_end_retreat.admitted_on_slope);
    if (!needs_retreat) return true;
    if (!planning.departure_reverse_allowed) return false;
    double retreat_distance = 0.0;
    for (std::size_t i = end; i-- > 0;) {
      planningCheckpoint();
      // Routing weights include soft clearance; retreat reach is metres.
      retreat_distance += (path[i + 1]->state.head<3>() -
                           path[i]->state.head<3>()).norm();
      const double limit = slope_end_retreat.reverse_edge_admissible
          ? (planning.reverse_exit_max_length > 0.0 ? planning.reverse_exit_max_length : kDepartureMaxM) : kDepartureMaxM;
      if (retreat_distance > limit + 1e-9) break;
      if (slope_end_retreat.reverse_edge_admissible &&
          !slope_end_retreat.reverse_edge_admissible(*path[i + 1], *path[i])) break;
      // A refuge's band depends on this path, not just the vertex id.
      if (slope_end_retreat.refuge_admissible(path, end, i)) {
        return true;
      }
    }
    if (refused != nullptr) ++*refused;
    return false;
  };

  ShortestPathsReport rep;
  if (!graph.findShortestPaths(rep)) return PathSelectionResult();
  graph.findLeafVertices(rep);
  std::vector<Vertex*> leaves;
  graph.getLeafVertices(leaves);

  // Weighted searches choose routes; gain (and its absolute low-gain
  // threshold) is discounted in physical metres, including detour routes.
  using Candidate = TurnCompliantRoutes::Route;
  const auto metric_along = [](Candidate& candidate) {
    candidate.along.assign(candidate.path.size(), 0.0);
    for (std::size_t i = 1; i < candidate.path.size(); ++i) {
      planningCheckpoint();
      candidate.along[i] = candidate.along[i - 1] +
          (candidate.path[i]->state.head<3>() -
           candidate.path[i - 1]->state.head<3>()).norm();
    }
  };
  std::vector<Candidate> shortest;
  for (Vertex* leaf : leaves) {
    planningCheckpoint();
    if (leaf == nullptr) continue;
    Candidate candidate;
    graph.getShortestPath(leaf->id, rep, true, candidate.path);
    metric_along(candidate);
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
        planningCheckpoint();
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
      gain = path_gain * pathDirectionFactor(path_points, exploring_direction,
                                             planning, /*bounded=*/true);
      return true;
    };
    const auto excluded = [&](const Vertex* viewpoint) {
      if (end_excluded && end_excluded(*viewpoint)) return true;
      return std::any_of(
          excluded_endpoints.begin(), excluded_endpoints.end(),
          [viewpoint, exclusion_radius](const Eigen::Vector3d& center) {
            return (viewpoint->state.head(3) - center).norm() <=
                   exclusion_radius;
          });
    };

    // What a candidate's whole path, to its leaf, scored.
    struct Leaf {
      /// The leaf lies in a reservation.
      bool excluded = false;
      /// The whole path is not too steep and turns only where it may.
      bool admissible = false;
      /// The whole path's gain, once scored.
      double gain = 0.0;
      /// The gain the whole path leads to, whether or not it may be driven
      /// as it is: its gain when it is not too steep, else 0.
      double leads_to = 0.0;
    };
    enum class Ending { kAdmissible, kInadmissible, kGoesNowhere };
    // The one test of where a path may end, its leaf or a vertex it is cut
    // back to, in the fallback and the clear selection alike: everything the
    // leaf had to satisfy, checked where the path now ends (review r2).
    //   * The end lies in no reservation; cut back from a reserved leaf, the
    //     path must carry gain of its own, outside the reservation.
    //   * With slope_end_retreat, a narrow or slope-exempt end has a way
    //     back (way_back).
    //   * The path up to there is not too steep and turns only where it
    //     may.
    //   * With `must_go_somewhere` (a clear end), the end lies beyond
    //     goal_reach of the root and the path leads to gain; else it goes
    //     nowhere (review r0, I-3).
    // Fills `path`, the candidate up to `end`, and its `gain`.
    const auto ending = [&](const Candidate& candidate, const Leaf& leaf,
                            std::size_t end, bool must_go_somewhere,
                            std::vector<Vertex*>& path, double& gain) {
      path.assign(candidate.path.begin(), candidate.path.begin() + end + 1);
      gain = 0.0;
      if (path_admissible && end > 0 && !path_admissible(path)) {
        return Ending::kInadmissible;
      }
      if (end == 0 || excluded(path.back()) ||
          !way_back(candidate.path, end, nullptr)) {
        return Ending::kInadmissible;
      }
      if (end + 1 == candidate.path.size()) {
        if (!leaf.admissible) return Ending::kInadmissible;
        gain = leaf.gain;
      } else if (!score(path, candidate.along, gain) || !turns_ok(path) ||
                 (leaf.excluded && !(gain > 0.0))) {
        return Ending::kInadmissible;
      }
      if (must_go_somewhere &&
          ((path.back()->state.head<2>() - path.front()->state.head<2>())
                   .norm() <= goal_reach ||
           !(std::max(leaf.leads_to, gain) > 0.0))) {
        return Ending::kGoesNowhere;
      }
      return Ending::kAdmissible;
    };

    // Prefer a clear end or an admitted narrow end with a bounded reverse
    // way back. Cut back to the last admissible end when needed, including
    // a gainless prefix that leads to the leaf's gain. Without the retreat
    // check, clearance remains a preference and an unclear fallback may win.
    // With it, no room anywhere means no path: the caller may try a boxed-in
    // departure, but selection never waives the way-back safety requirement.
    //
    // Some whole path, to a leaf outside every reservation, may be driven
    // and has gain: without one, no path is chosen, clear or not.
    bool gain_reachable = false;
    // The fallback obeys the same end predicate, including narrow/slope
    // way-back checks. Rank by its gain, then the whole path's gain.
    std::vector<Vertex*> fallback_path;
    double fallback_gain = 0.0;
    double fallback_leads_to = 0.0;
    bool have_clear = false;
    double best_clear_gain = 0.0;
    double best_clear_full_gain = 0.0;
    double best_clear_leads_to = 0.0;
    std::vector<Vertex*> best_clear_path;
    for (const Candidate& candidate : candidates) {
      planningCheckpoint();
      if (candidate.path.size() <= 1) continue;  // needs root and leaf
      // Reservations are checked where each candidate ends: the leaf for the
      // path as it is, the vertex it is cut back to otherwise.
      Leaf leaf;
      leaf.excluded = excluded(candidate.path.back());
      if (!leaf.excluded) {
        ++result.leaves_evaluated;
        leaf.admissible = score(candidate.path, candidate.along, leaf.gain);
        if (leaf.admissible) leaf.leads_to = leaf.gain;
        if (!leaf.admissible) {
          ++result.paths_rejected_steep;
        } else if (!turns_ok(candidate.path)) {
          leaf.admissible = false;
        } else if (leaf.gain > 0.0) {
          gain_reachable = true;
        }
      }
      if (leaf.admissible) {
        std::size_t end = candidate.path.size() - 1;
        while (end > 0 &&
               !way_back(candidate.path, end, nullptr)) {
          --end;
        }
        std::vector<Vertex*> path;
        double gain = 0.0;
        if (ending(candidate, leaf, end, false, path, gain) ==
                Ending::kAdmissible &&
            std::max(gain, leaf.leads_to) > 0.0 &&
            (fallback_path.empty() || gain > fallback_gain ||
             (gain == fallback_gain && leaf.leads_to > fallback_leads_to))) {
          fallback_path = std::move(path);
          fallback_gain = gain;
          fallback_leads_to = leaf.leads_to;
        }
      }
      if (!viewpoint_clear) continue;

      // The robot stops where the path ends; the root is where it stands.
      std::size_t end = candidate.path.size() - 1;
      while (end > 0 &&
             !((clear(candidate.path[end]) || retreat_checked) &&
               way_back(candidate.path, end,
                        &result.slope_ends_without_way_back))) {
        --end;
      }
      if (end == 0) {
        ++result.paths_without_clear_viewpoint;
        continue;
      }
      if (end + 1 < candidate.path.size()) ++result.paths_pulled_back;
      std::vector<Vertex*> path;
      double gain = 0.0;
      const Ending status = ending(candidate, leaf, end, true, path, gain);
      if (status == Ending::kGoesNowhere) {
        ++result.paths_without_clear_viewpoint;
        continue;
      }
      if (status != Ending::kAdmissible) continue;
      const double full_gain = leaf.admissible ? leaf.gain : 0.0;
      if (!have_clear || gain > best_clear_gain ||
          (gain == best_clear_gain && full_gain > best_clear_full_gain)) {
        have_clear = true;
        best_clear_gain = gain;
        best_clear_full_gain = full_gain;
        best_clear_leads_to = std::max(leaf.leads_to, gain);
        best_clear_path = std::move(path);
      }
    }
    // A branch whose only gain lies in a peer's reservation is not pursued,
    // not even through a clear prefix: that is what reservations prevent.
    if (viewpoint_clear && have_clear &&
        (gain_reachable || best_clear_gain > 0.0)) {
      result.best_gain = best_clear_gain;
      result.best_full_gain = best_clear_leads_to;
      result.best_path = best_clear_path;
    } else if (!fallback_path.empty()) {
      result.best_gain = fallback_gain;
      result.best_full_gain = fallback_leads_to;
      result.best_path = fallback_path;
      result.unclear_viewpoint = static_cast<bool>(viewpoint_clear);
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
    int found_routes = 0;
    if (sharp_turn_allowed) {
      std::vector<int> destinations;
      for (const auto& [id, v] : graph.vertices_map_) {
        planningCheckpoint();
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
      found_routes = static_cast<int>(routes.to.size());
      std::vector<Candidate> candidates;
      candidates.reserve(routes.to.size());
      for (const int id : destinations) {
        planningCheckpoint();
        const auto found = routes.to.find(id);
        if (found != routes.to.end()) {
          candidates.push_back(found->second);
          metric_along(candidates.back());
        }
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
    result.detour_searched = static_cast<bool>(sharp_turn_allowed);
    result.detour_routes_found = found_routes;
  }
  if (!result.best_path.empty()) {
    result.best_path_id = result.best_path.back()->id;
  }

  // Re-parent along the winning path so the branch can be walked from the
  // root. The path is the leaf's shortest path, or a detour.
  for (size_t i = 1; i < result.best_path.size(); ++i) {
    planningCheckpoint();
    result.best_path[i]->parent = result.best_path[i - 1];
  }
  return result;
}

}  // namespace mgg
