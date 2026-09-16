#include "mgg_core/grid_refinement.h"

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <cstdio>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace mgg {
namespace {

constexpr std::size_t kNoParent = std::numeric_limits<std::size_t>::max();

struct Cell {
  StateVec state = StateVec::Zero();
  double cost = std::numeric_limits<double>::infinity();
  std::size_t parent = kNoParent;
  bool projection_tried = false;
  bool valid = false;
  bool closed = false;
};

struct QueueEntry {
  double estimate = 0.0;
  double cost = 0.0;
  std::size_t index = 0;
  std::size_t sequence = 0;
};

struct LaterEntry {
  bool operator()(const QueueEntry& a, const QueueEntry& b) const {
    if (a.estimate != b.estimate) return a.estimate > b.estimate;
    if (a.cost != b.cost) return a.cost > b.cost;
    if (a.index != b.index) return a.index > b.index;
    return a.sequence > b.sequence;
  }
};

struct DirectedEdgeKey {
  std::array<double, 8> values{};

  bool operator==(const DirectedEdgeKey& other) const {
    return values == other.values;
  }
};

struct DirectedEdgeHash {
  std::size_t operator()(const DirectedEdgeKey& key) const {
    std::size_t seed = 0;
    for (const double value : key.values) {
      const std::size_t item = std::hash<double>{}(value);
      seed ^= item + 0x9e3779b9u + (seed << 6u) + (seed >> 2u);
    }
    return seed;
  }
};

struct TraversalResult {
  bool accepted = false;
  std::vector<StateVec> checked;
  double length = 0.0;
};

DirectedEdgeKey directedEdgeKey(const StateVec& from, const StateVec& to) {
  DirectedEdgeKey key;
  for (Eigen::Index i = 0; i < 4; ++i) {
    key.values[static_cast<std::size_t>(i)] = from[i];
    key.values[static_cast<std::size_t>(i + 4)] = to[i];
  }
  return key;
}

void copyRequest(const RouteCorridor& corridor, FeasiblePath& path) {
  path.status = corridor.status;
  path.mission_id = corridor.request.mission_id;
  path.component_id = corridor.request.component_id;
  path.graph_revision = corridor.request.graph_revision;
  path.map_revision = corridor.request.map_revision;
  path.map_epoch = corridor.request.map_epoch;
  path.mapping_graph_revision = corridor.request.mapping_graph_revision;
  path.geometry_revision = corridor.request.geometry_revision;
  path.map_source_stamp_sec = corridor.request.map_source_stamp_sec;
  path.map_source_stamp_nanosec = corridor.request.map_source_stamp_nanosec;
  path.partial = corridor.partial;
  path.reason = corridor.reason;
}

bool samePosition(const StateVec& a, const StateVec& b) {
  return (a.head<3>() - b.head<3>()).cwiseAbs().maxCoeff() <= 1e-6;
}

const char* projectionFailureClass(GridProjectionStatus status) {
  switch (status) {
    case GridProjectionStatus::kNoGround:
      return "no mapped ground support";
    case GridProjectionStatus::kBodyOccupied:
      return "body intersects occupied space";
    case GridProjectionStatus::kBodyUnknown:
      return "body includes unknown space";
    case GridProjectionStatus::kGeofenceViolation:
      return "geofence violation";
    case GridProjectionStatus::kSupported:
      return "projector returned an invalid supported state";
  }
  return "unknown projection failure";
}

std::string projectionFailure(const std::string& endpoint,
                              GridProjectionStatus status,
                              const StateVec& state) {
  char reason[256];
  std::snprintf(reason, sizeof(reason),
                "%s rejected: %s at (%.2f, %.2f, %.2f)", endpoint.c_str(),
                projectionFailureClass(status), state.x(), state.y(),
                state.z());
  return reason;
}

double pathLength(const std::vector<StateVec>& path,
                  const StateVec& from, const StateVec& to) {
  if (path.size() < 2) return (to.head<3>() - from.head<3>()).norm();
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += (path[i].head<3>() - path[i - 1].head<3>()).norm();
  }
  return length;
}

void appendCheckedPath(const std::vector<StateVec>& checked,
                       std::vector<StateVec>& output) {
  for (std::size_t i = 1; i < checked.size(); ++i) {
    if (output.empty() || !samePosition(output.back(), checked[i])) {
      output.push_back(checked[i]);
    }
  }
}

}  // namespace

BoundedGridPlanner::BoundedGridPlanner(
    StateVec current, GridRefinementLimits limits, GridProjectState project,
    GridTraverseSegment traversable, GridCancelled cancelled)
    : current_(std::move(current)),
      limits_(limits),
      project_(std::move(project)),
      traversable_(std::move(traversable)),
      cancelled_(std::move(cancelled)) {}

FeasiblePath BoundedGridPlanner::refine(const RouteCorridor& corridor) {
  FeasiblePath result;
  copyRequest(corridor, result);
  if (corridor.status != PlanningStatus::kSucceeded) return result;
  const auto diagnostics_started = std::chrono::steady_clock::now();
  std::size_t projection_counts[5] = {0, 0, 0, 0, 0};
  std::size_t traversal_attempts = 0;
  std::size_t traversal_rejections = 0;
  std::size_t expansions = 0;
  std::unordered_map<DirectedEdgeKey, TraversalResult, DirectedEdgeHash>
      traversal_cache;
  const auto fail = [&](const std::string& reason) {
    result.status = PlanningStatus::kBlocked;
    result.poses.clear();
    result.partial = false;
    result.reason = reason;
    const std::size_t projections =
        projection_counts[0] + projection_counts[1] + projection_counts[2] +
        projection_counts[3] + projection_counts[4];
    if (projections != 0 || traversal_attempts != 0 || expansions != 0) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - diagnostics_started);
      char summary[320];
      std::snprintf(
          summary, sizeof(summary),
          " [grid evidence: projections=%zu supported=%zu no_ground=%zu "
          "occupied=%zu unknown=%zu geofence=%zu traversals=%zu rejected=%zu "
          "expansions=%zu elapsed_ms=%lld]",
          projections, projection_counts[0], projection_counts[1],
          projection_counts[2], projection_counts[3], projection_counts[4],
          traversal_attempts, traversal_rejections, expansions,
          static_cast<long long>(elapsed.count()));
      result.reason += summary;
    }
    return result;
  };
  const auto project_state = [&](StateVec& state) {
    const GridProjectionStatus status = project_(state);
    const std::size_t index = static_cast<std::size_t>(status);
    if (index < 5) ++projection_counts[index];
    return status;
  };
  if (corridor.request.objective != ObjectiveKind::kNavigate &&
      corridor.request.objective != ObjectiveKind::kReturnHome) {
    return fail("grid refinement supports Navigate and ReturnHome only");
  }
  if (!current_.allFinite() || !std::isfinite(limits_.resolution_m) ||
      !std::isfinite(limits_.detour_margin_m) ||
      !std::isfinite(limits_.start_connector_max_distance_m) ||
      limits_.resolution_m <= 0.0 || limits_.detour_margin_m < 0.0 ||
      limits_.start_connector_max_distance_m < 0.0 ||
      limits_.max_cells == 0 || limits_.max_expansions == 0 || !project_ ||
      !traversable_ || limits_.timeout.count() <= 0) {
    return fail("grid refinement configuration or current pose is invalid");
  }

  const auto started = std::chrono::steady_clock::now();
  const auto timedOut = [this, started]() {
    return std::chrono::steady_clock::now() - started >= limits_.timeout;
  };
  const auto interrupted = [this, &timedOut](std::string& reason) {
    if (cancelled_ && cancelled_()) {
      reason = "grid refinement was cancelled";
      return true;
    }
    if (timedOut()) {
      reason = "grid refinement exceeded its cooperative deadline";
      return true;
    }
    return false;
  };

  std::string interruption_reason;
  if (interrupted(interruption_reason)) return fail(interruption_reason);
  StateVec start = current_;
  const GridProjectionStatus start_status = project_state(start);
  if (interrupted(interruption_reason)) return fail(interruption_reason);
  if (start_status != GridProjectionStatus::kSupported ||
      !start.allFinite()) {
    return fail(projectionFailure("current pose", start_status, start));
  }

  std::vector<StateVec> waypoints;
  waypoints.reserve(corridor.poses.size() + 2);
  waypoints.push_back(start);
  for (std::size_t waypoint_index = 0;
       waypoint_index < corridor.poses.size(); ++waypoint_index) {
    StateVec waypoint = corridor.poses[waypoint_index];
    if (interrupted(interruption_reason)) return fail(interruption_reason);
    if (!waypoint.allFinite()) {
      return fail("route corridor waypoint[" +
                  std::to_string(waypoint_index) +
                  "] rejected: state is non-finite");
    }
    const GridProjectionStatus waypoint_status = project_state(waypoint);
    if (interrupted(interruption_reason)) return fail(interruption_reason);
    if (waypoint_status != GridProjectionStatus::kSupported ||
        !waypoint.allFinite()) {
      return fail(projectionFailure(
          "route corridor waypoint[" + std::to_string(waypoint_index) + "]",
          waypoint_status, waypoint));
    }
    if (!samePosition(waypoints.back(), waypoint)) {
      waypoints.push_back(waypoint);
    }
  }
  if (corridor.partial && waypoints.size() < 2) {
    return fail("partial route has no progress proxy");
  }
  StateVec goal = corridor.partial ? waypoints.back()
                                   : corridor.request.goal.pose;
  const double requested_goal_yaw = goal[3];
  if (interrupted(interruption_reason)) return fail(interruption_reason);
  if (!goal.allFinite()) {
    return fail("the exact goal has no supported collision-free grid state");
  }
  const GridProjectionStatus goal_status = project_state(goal);
  if (interrupted(interruption_reason)) return fail(interruption_reason);
  goal[3] = requested_goal_yaw;
  if (goal_status != GridProjectionStatus::kSupported || !goal.allFinite()) {
    return fail(projectionFailure("exact goal", goal_status, goal));
  }
  if (corridor.partial) {
    // The final corridor pose is the checked local proxy. The exact operator
    // goal remains only in corridor.request.goal for the next continuation.
    waypoints.back() = goal;
  } else if (!samePosition(waypoints.back(), goal)) {
    waypoints.push_back(goal);
  } else {
    // The graph vertex supplies XYZ, but the request owns exact final yaw.
    waypoints.back() = goal;
  }
  if (waypoints.size() == 1) {
    if (interrupted(interruption_reason)) return fail(interruption_reason);
    result.status = PlanningStatus::kSucceeded;
    result.poses.push_back(goal);
    result.reason.clear();
    return result;
  }

  std::size_t sequence = 0;
  // refine() runs against one immutable planner/map snapshot. Memoize exact
  // directed states so reverse traversals and yaw-qualified endpoints retain
  // their own terrain and swept-body evidence.
  const std::size_t cache_basis =
      std::min(limits_.max_cells, limits_.max_expansions);
  const std::size_t traversal_cache_limit =
      cache_basis > std::numeric_limits<std::size_t>::max() / 16u
          ? std::numeric_limits<std::size_t>::max()
          : cache_basis * 16u;
  const auto traverse = [this, &interrupted, &traversal_attempts,
                         &traversal_rejections, &traversal_cache,
                         traversal_cache_limit](const StateVec& a,
                               const StateVec& b,
                               std::vector<StateVec>* checked,
                               double* length,
                               std::string& interruption_reason) {
    if (interrupted(interruption_reason)) return false;
    const DirectedEdgeKey key = directedEdgeKey(a, b);
    const auto cached = traversal_cache.find(key);
    if (cached != traversal_cache.end()) {
      if (length != nullptr) *length = cached->second.length;
      if (checked != nullptr) *checked = cached->second.checked;
      return cached->second.accepted;
    }
    std::vector<StateVec> local;
    ++traversal_attempts;
    const bool accepted = traversable_(a, b, local);
    if (interrupted(interruption_reason)) return false;
    if (!accepted) {
      ++traversal_rejections;
      if (traversal_cache.size() < traversal_cache_limit) {
        traversal_cache.emplace(key, TraversalResult{});
      }
      return false;
    }
    if (local.size() < 2 || !samePosition(local.front(), a) ||
        !samePosition(local.back(), b)) {
      if (traversal_cache.size() < traversal_cache_limit) {
        traversal_cache.emplace(key, TraversalResult{});
      }
      return false;
    }
    for (const StateVec& pose : local) {
      if (!pose.allFinite()) return false;
    }
    local.front()[3] = a[3];
    local.back()[3] = b[3];
    const double checked_length = pathLength(local, a, b);
    if (!std::isfinite(checked_length) || checked_length <= 0.0) {
      if (traversal_cache.size() < traversal_cache_limit) {
        traversal_cache.emplace(key, TraversalResult{});
      }
      return false;
    }
    if (length != nullptr) *length = checked_length;
    if (checked != nullptr) *checked = local;
    if (traversal_cache.size() < traversal_cache_limit) {
      traversal_cache.emplace(
          key, TraversalResult{true, std::move(local), checked_length});
    }
    return true;
  };
  for (std::size_t segment = 1; segment < waypoints.size(); ++segment) {
    const StateVec& from = waypoints[segment - 1];
    const StateVec& to = waypoints[segment];
    if (interrupted(interruption_reason)) return fail(interruption_reason);
    std::vector<StateVec> direct_path;
    if (traverse(from, to, &direct_path, nullptr, interruption_reason)) {
      if (result.poses.empty()) result.poses.push_back(from);
      appendCheckedPath(direct_path, result.poses);
      continue;
    }
    if (!interruption_reason.empty()) return fail(interruption_reason);

    const double resolution = limits_.resolution_m;
    const long double cells_bound = static_cast<long double>(
        std::min<std::size_t>(limits_.max_cells,
                              static_cast<std::size_t>(
                                  std::numeric_limits<long>::max())));
    const long double width_bound =
        std::ceil((std::abs(static_cast<long double>(to.x()) - from.x()) +
                   2.0L * limits_.detour_margin_m) /
                  limits_.resolution_m) +
        3.0L;
    const long double height_bound =
        std::ceil((std::abs(static_cast<long double>(to.y()) - from.y()) +
                   2.0L * limits_.detour_margin_m) /
                  limits_.resolution_m) +
        3.0L;
    if (!std::isfinite(width_bound) || !std::isfinite(height_bound) ||
        width_bound > cells_bound || height_bound > cells_bound) {
      return fail("grid refinement bounds are invalid or exceed the cell limit");
    }
    const double min_x = std::min(from.x(), to.x()) - limits_.detour_margin_m;
    const double max_x = std::max(from.x(), to.x()) + limits_.detour_margin_m;
    const double min_y = std::min(from.y(), to.y()) - limits_.detour_margin_m;
    const double max_y = std::max(from.y(), to.y()) + limits_.detour_margin_m;
    const double left = std::ceil((from.x() - min_x) / resolution);
    const double down = std::ceil((from.y() - min_y) / resolution);
    const double origin_x = from.x() - left * resolution;
    const double origin_y = from.y() - down * resolution;
    const double nx_d = std::ceil((max_x - origin_x) / resolution) + 1.0;
    const double ny_d = std::ceil((max_y - origin_y) / resolution) + 1.0;
    if (!std::isfinite(origin_x) || !std::isfinite(origin_y) ||
        !std::isfinite(nx_d) || !std::isfinite(ny_d) || nx_d < 1.0 ||
        ny_d < 1.0 || nx_d > static_cast<double>(limits_.max_cells) ||
        ny_d > static_cast<double>(limits_.max_cells) ||
        left < 0.0 || down < 0.0 || left >= nx_d || down >= ny_d) {
      return fail("grid refinement bounds are invalid or exceed the cell limit");
    }
    const std::size_t nx = static_cast<std::size_t>(nx_d);
    const std::size_t ny = static_cast<std::size_t>(ny_d);
    if (nx > limits_.max_cells / ny) {
      return fail("grid refinement exceeds the cell limit");
    }
    const std::size_t cell_count = nx * ny;
    if (cell_count > limits_.max_cells) {
      return fail("grid refinement exceeds the cell limit");
    }
    const std::size_t start_x = static_cast<std::size_t>(left);
    const std::size_t start_y = static_cast<std::size_t>(down);
    if (start_x >= nx || start_y >= ny) {
      return fail("grid refinement could not represent the segment start");
    }

    std::vector<Cell> cells(cell_count);
    const auto index = [nx](std::size_t x, std::size_t y) {
      return y * nx + x;
    };
    const auto xy = [nx](std::size_t cell) {
      return std::pair<std::size_t, std::size_t>(cell % nx, cell / nx);
    };
    const auto ensureCell = [&](std::size_t cell, double parent_z,
                                std::string& reason) {
      Cell& value = cells[cell];
      if (value.projection_tried) return value.valid;
      value.projection_tried = true;
      const auto [x, y] = xy(cell);
      const double lattice_x = origin_x + static_cast<double>(x) * resolution;
      const double lattice_y = origin_y + static_cast<double>(y) * resolution;
      value.state = StateVec(lattice_x, lattice_y, parent_z, 0.0);
      if (interrupted(reason)) return false;
      const GridProjectionStatus projection_status = project_state(value.state);
      if (interrupted(reason)) return false;
      if (projection_status != GridProjectionStatus::kSupported ||
          !value.state.allFinite()) {
        return false;
      }
      // Keep the discrete XY identity separate from a small support-probe
      // correction, but reject a projection that jumps into another cell.
      if (std::abs(value.state.x() - lattice_x) > 0.5 * resolution + 1e-9 ||
          std::abs(value.state.y() - lattice_y) > 0.5 * resolution + 1e-9) {
        return false;
      }
      value.valid = true;
      return true;
    };

    const std::size_t source = index(start_x, start_y);
    cells[source].projection_tried = true;
    cells[source].valid = true;
    cells[source].state = from;
    cells[source].cost = 0.0;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, LaterEntry> open;
    // Every expanded edge invokes terrain and swept-body map queries. Bias the
    // queue toward the goal to trade shortest-path optimality for substantially
    // less map work on wide, mostly traversable objective grids.
    constexpr double kHeuristicWeight = 1.5;
    open.push({kHeuristicWeight * (to.head<2>() - from.head<2>()).norm(), 0.0, source,
               sequence++});

    constexpr int kDx[8] = {1, 0, -1, 0, 1, -1, -1, 1};
    constexpr int kDy[8] = {0, 1, 0, -1, 1, 1, -1, -1};
    std::size_t reached = kNoParent;
    while (!open.empty()) {
      if (interrupted(interruption_reason)) return fail(interruption_reason);
      const QueueEntry entry = open.top();
      open.pop();
      Cell& active = cells[entry.index];
      if (active.closed || entry.cost > active.cost + 1e-12) continue;
      if (expansions >= limits_.max_expansions) {
        return fail("grid refinement exceeded the expansion limit");
      }
      ++expansions;
      active.closed = true;

      if ((to.head<2>() - active.state.head<2>()).norm() <=
              std::sqrt(2.0) * resolution + 1e-9 &&
          traverse(active.state, to, nullptr, nullptr, interruption_reason)) {
        reached = entry.index;
        break;
      }
      if (!interruption_reason.empty()) return fail(interruption_reason);

      const auto [active_x, active_y] = xy(entry.index);
      bool admitted_neighbor = false;
      for (int direction = 0; direction < 8; ++direction) {
        if (interrupted(interruption_reason)) return fail(interruption_reason);
        const long next_x = static_cast<long>(active_x) + kDx[direction];
        const long next_y = static_cast<long>(active_y) + kDy[direction];
        if (next_x < 0 || next_y < 0 ||
            next_x >= static_cast<long>(nx) ||
            next_y >= static_cast<long>(ny)) {
          continue;
        }
        const std::size_t next =
            index(static_cast<std::size_t>(next_x),
                  static_cast<std::size_t>(next_y));
        if (!ensureCell(next, active.state.z(), interruption_reason) ||
            cells[next].closed) {
          if (!interruption_reason.empty()) return fail(interruption_reason);
          continue;
        }

        if (kDx[direction] != 0 && kDy[direction] != 0) {
          const std::size_t side_x =
              index(static_cast<std::size_t>(next_x), active_y);
          const std::size_t side_y =
              index(active_x, static_cast<std::size_t>(next_y));
          if (!ensureCell(side_x, active.state.z(), interruption_reason) ||
              !ensureCell(side_y, active.state.z(), interruption_reason) ||
              !traverse(active.state, cells[side_x].state, nullptr, nullptr,
                        interruption_reason) ||
              !traverse(active.state, cells[side_y].state, nullptr, nullptr,
                        interruption_reason)) {
            if (!interruption_reason.empty()) return fail(interruption_reason);
            continue;
          }
        }
        double edge = 0.0;
        if (!traverse(active.state, cells[next].state, nullptr, &edge,
                      interruption_reason)) {
          if (!interruption_reason.empty()) return fail(interruption_reason);
          continue;
        }
        const double candidate = active.cost + edge;
        if (candidate + 1e-12 >= cells[next].cost) continue;
        cells[next].cost = candidate;
        cells[next].parent = entry.index;
        const double heuristic =
            (to.head<2>() - cells[next].state.head<2>()).norm();
        open.push({candidate + kHeuristicWeight * heuristic, candidate, next,
                   sequence++});
        admitted_neighbor = true;
      }

      // A physical start may sit in a camera/lidar ground blind spot. If none
      // of its adjacent cells can be admitted, look for the nearest supported
      // lattice cell within the explicitly bounded connector distance. The
      // caller's traversal predicate still checks the complete swept body,
      // terrain step and geofence; this search grants no unknown-space waiver.
      if (entry.index == source && segment == 1 && !admitted_neighbor &&
          limits_.start_connector_max_distance_m > resolution) {
        struct Candidate {
          double distance;
          std::size_t cell;
        };
        std::vector<Candidate> candidates;
        const double maximum = limits_.start_connector_max_distance_m;
        for (std::size_t cell = 0; cell < cell_count; ++cell) {
          if ((cell & 255u) == 0u && interrupted(interruption_reason))
            return fail(interruption_reason);
          if (cell == source) continue;
          const auto [candidate_x, candidate_y] = xy(cell);
          const double dx =
              (static_cast<double>(candidate_x) - start_x) * resolution;
          const double dy =
              (static_cast<double>(candidate_y) - start_y) * resolution;
          const double distance = std::hypot(dx, dy);
          if (distance > resolution + 1e-9 && distance <= maximum + 1e-9)
            candidates.push_back({distance, cell});
        }
        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate& a, const Candidate& b) {
                    return a.distance < b.distance ||
                           (a.distance == b.distance && a.cell < b.cell);
                  });
        if (interrupted(interruption_reason)) return fail(interruption_reason);
        for (const Candidate& candidate : candidates) {
          if (!ensureCell(candidate.cell, active.state.z(), interruption_reason)) {
            if (!interruption_reason.empty()) return fail(interruption_reason);
            continue;
          }
          double edge = 0.0;
          if (!traverse(active.state, cells[candidate.cell].state, nullptr, &edge,
                        interruption_reason)) {
            if (!interruption_reason.empty()) return fail(interruption_reason);
            continue;
          }
          cells[candidate.cell].cost = edge;
          cells[candidate.cell].parent = source;
          const double heuristic =
              (to.head<2>() - cells[candidate.cell].state.head<2>()).norm();
          open.push({edge + kHeuristicWeight * heuristic, edge,
                     candidate.cell, sequence++});
        }
      }
    }
    if (reached == kNoParent) {
      return fail("no observed traversable grid detour for route segment " +
                  std::to_string(segment - 1) + "->" +
                  std::to_string(segment));
    }

    std::vector<StateVec> reversed;
    for (std::size_t cell = reached; cell != source;) {
      if (cell == kNoParent || cells[cell].parent == kNoParent) {
        return fail("grid refinement produced an invalid parent chain");
      }
      reversed.push_back(cells[cell].state);
      cell = cells[cell].parent;
    }
    std::reverse(reversed.begin(), reversed.end());
    StateVec previous = from;
    for (const StateVec& pose : reversed) {
      std::vector<StateVec> checked;
      if (!traverse(previous, pose, &checked, nullptr,
                    interruption_reason)) {
        if (!interruption_reason.empty()) return fail(interruption_reason);
        return fail("grid detour failed final segment validation");
      }
      if (result.poses.empty()) result.poses.push_back(previous);
      appendCheckedPath(checked, result.poses);
      previous = pose;
    }
    std::vector<StateVec> checked_goal;
    if (!traverse(previous, to, &checked_goal, nullptr,
                  interruption_reason)) {
      if (!interruption_reason.empty()) return fail(interruption_reason);
      return fail("grid detour failed final goal-connector validation");
    }
    if (result.poses.empty()) result.poses.push_back(previous);
    appendCheckedPath(checked_goal, result.poses);
  }

  for (std::size_t i = 1; i < result.poses.size(); ++i) {
    if (interrupted(interruption_reason)) return fail(interruption_reason);
    result.poses[i - 1][3] = std::atan2(
        result.poses[i].y() - result.poses[i - 1].y(),
        result.poses[i].x() - result.poses[i - 1].x());
  }
  if (interrupted(interruption_reason)) return fail(interruption_reason);
  result.poses.back()[3] = goal[3];
  result.status = PlanningStatus::kSucceeded;
  result.reason.clear();
  return result;
}

}  // namespace mgg
