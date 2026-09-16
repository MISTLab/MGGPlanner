#include "mgg_core/grid_refinement.h"

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mgg {
namespace {

constexpr std::size_t kNoParent = std::numeric_limits<std::size_t>::max();

struct Cell {
  StateVec state = StateVec::Zero();
  std::size_t grid_index = 0;
  double cost = std::numeric_limits<double>::infinity();
  std::size_t parent = kNoParent;
  bool projection_tried = false;
  bool valid = false;
  bool closed = false;
};

struct CellKey {
  std::size_t grid_index = 0;
  std::uint64_t parent_height_bits = 0;

  bool operator==(const CellKey& other) const {
    return grid_index == other.grid_index &&
           parent_height_bits == other.parent_height_bits;
  }
};

struct CellKeyHash {
  std::size_t operator()(const CellKey& key) const {
    std::size_t seed = std::hash<std::size_t>{}(key.grid_index);
    const std::size_t height =
        std::hash<std::uint64_t>{}(key.parent_height_bits);
    return seed ^ (height + 0x9e3779b9u + (seed << 6u) + (seed >> 2u));
  }
};

std::uint64_t heightBits(double height) {
  if (height == 0.0) height = 0.0;  // Canonicalize negative zero.
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(height));
  std::memcpy(&bits, &height, sizeof(bits));
  return bits;
}

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
    case GridProjectionStatus::kProvisionalUnknown:
      return "projector returned an invalid provisional state";
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
  // Indices into corridor.poses for the segment currently being validated.
  // kNoCorridorIndex is the implicit start (the live pose) and
  // corridor.poses.size() the exact request-owned goal.
  std::size_t active_from_index = kNoCorridorIndex;
  std::size_t active_to_index = kNoCorridorIndex;
  bool active_segment_known = false;
  const auto fail = [&](const std::string& reason) {
    result.status = PlanningStatus::kBlocked;
    result.poses.clear();
    result.partial = false;
    result.reason = reason;
    result.blocked_segment_identified = active_segment_known;
    result.blocked_from_index = active_from_index;
    result.blocked_to_index = active_to_index;
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
    const std::size_t index =
        status == GridProjectionStatus::kProvisionalUnknown
            ? 0
            : static_cast<std::size_t>(status);
    if (index < 5) ++projection_counts[index];
    return status;
  };
  const auto projectionAccepted = [](GridProjectionStatus status) {
    return status == GridProjectionStatus::kSupported ||
           status == GridProjectionStatus::kProvisionalUnknown;
  };
  if (corridor.request.objective != ObjectiveKind::kNavigate &&
      corridor.request.objective != ObjectiveKind::kReturnHome &&
      corridor.request.objective != ObjectiveKind::kExplore) {
    return fail(
        "grid refinement supports Explore, Navigate and ReturnHome only");
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
  // Corridor-pose index behind each waypoint, so a rejected waypoint or an
  // unrefinable segment can name the corridor it belongs to.
  std::vector<std::size_t> waypoint_source;
  waypoints.reserve(corridor.poses.size() + 2);
  waypoint_source.reserve(corridor.poses.size() + 2);
  waypoints.push_back(start);
  waypoint_source.push_back(kNoCorridorIndex);
  for (std::size_t waypoint_index = 0;
       waypoint_index < corridor.poses.size(); ++waypoint_index) {
    StateVec waypoint = corridor.poses[waypoint_index];
    active_from_index = waypoint_source.back();
    active_to_index = waypoint_index;
    active_segment_known = true;
    if (interrupted(interruption_reason)) return fail(interruption_reason);
    if (!waypoint.allFinite()) {
      return fail("route corridor waypoint[" +
                  std::to_string(waypoint_index) +
                  "] rejected: state is non-finite");
    }
    const GridProjectionStatus waypoint_status = project_state(waypoint);
    if (interrupted(interruption_reason)) return fail(interruption_reason);
    const bool provisional_navigate_waypoint =
        corridor.request.objective == ObjectiveKind::kNavigate &&
        waypoint_status == GridProjectionStatus::kProvisionalUnknown;
    if ((waypoint_status != GridProjectionStatus::kSupported &&
         !provisional_navigate_waypoint) ||
        !waypoint.allFinite()) {
      return fail(projectionFailure(
          "route corridor waypoint[" + std::to_string(waypoint_index) + "]",
          waypoint_status, waypoint));
    }
    if (!samePosition(waypoints.back(), waypoint)) {
      waypoints.push_back(waypoint);
      waypoint_source.push_back(waypoint_index);
    } else {
      waypoint_source.back() = waypoint_index;
    }
  }
  active_segment_known = false;
  active_from_index = kNoCorridorIndex;
  active_to_index = kNoCorridorIndex;
  if (corridor.partial && waypoints.size() < 2) {
    return fail("partial route has no progress proxy");
  }
  StateVec goal = corridor.partial ? waypoints.back()
                                   : corridor.request.goal.pose;
  if (corridor.partial) {
    // The proxy is the last corridor pose, so name the corridor segment that
    // ends there rather than the pose on its own.
    active_from_index = waypoint_source.size() >= 2u
                            ? waypoint_source[waypoint_source.size() - 2u]
                            : kNoCorridorIndex;
    active_to_index = waypoint_source.back();
  } else {
    active_from_index = waypoint_source.back();
    active_to_index = corridor.poses.size();
  }
  active_segment_known = true;
  const double requested_goal_yaw = goal[3];
  if (interrupted(interruption_reason)) return fail(interruption_reason);
  if (!goal.allFinite()) {
    return fail("the exact goal has no supported collision-free grid state");
  }
  GridProjectionStatus goal_status = project_state(goal);
  if (interrupted(interruption_reason)) return fail(interruption_reason);
  goal[3] = requested_goal_yaw;
  const bool provisional_navigate_goal =
      !corridor.partial &&
      corridor.request.objective == ObjectiveKind::kNavigate &&
      goal_status == GridProjectionStatus::kProvisionalUnknown;
  if ((goal_status != GridProjectionStatus::kSupported &&
       !provisional_navigate_goal) ||
      !goal.allFinite()) {
    return fail(projectionFailure("exact goal", goal_status, goal));
  }
  if (provisional_navigate_goal) {
    // A ground goal from the UI carries navigation-base Z, while grid states
    // use driving height. Bind an unsupported endpoint to the nearest checked
    // corridor state before the direct attempt. A later A* connector may
    // advance this same provisional plane from its adjacent checked parent.
    goal.z() = waypoints.back().z();
    goal_status = project_state(goal);
    if (interrupted(interruption_reason)) return fail(interruption_reason);
    goal[3] = requested_goal_yaw;
    if (goal_status != GridProjectionStatus::kProvisionalUnknown ||
        !goal.allFinite()) {
      return fail("exact goal height changed while binding provisional terrain");
    }
  }
  if (corridor.partial) {
    // The final corridor pose is the checked local proxy. The exact operator
    // goal remains only in corridor.request.goal for the next continuation.
    waypoints.back() = goal;
  } else if (!samePosition(waypoints.back(), goal)) {
    waypoints.push_back(goal);
    waypoint_source.push_back(corridor.poses.size());
  } else {
    // The graph vertex supplies XYZ, but the request owns exact final yaw.
    // Its corridor index is kept: when the goal coincides with the last
    // corridor pose, that pose is the more useful thing to name as blocked.
    waypoints.back() = goal;
  }
  active_segment_known = false;
  active_from_index = kNoCorridorIndex;
  active_to_index = kNoCorridorIndex;
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
    active_from_index = waypoint_source[segment - 1];
    active_to_index = waypoint_source[segment];
    active_segment_known = true;
    StateVec connected_to = to;
    if (interrupted(interruption_reason)) return fail(interruption_reason);
    std::vector<StateVec> direct_path;
    if (traverse(from, to, &direct_path, nullptr, interruption_reason)) {
      if (result.poses.empty()) result.poses.push_back(from);
      appendCheckedPath(direct_path, result.poses);
      continue;
    }
    if (!interruption_reason.empty()) return fail(interruption_reason);

    const double resolution = limits_.resolution_m;
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
        width_bound >= static_cast<long double>(std::numeric_limits<long>::max()) ||
        height_bound >= static_cast<long double>(std::numeric_limits<long>::max())) {
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
        ny_d < 1.0 ||
        static_cast<long double>(nx_d) >=
            static_cast<long double>(std::numeric_limits<long>::max()) ||
        static_cast<long double>(ny_d) >=
            static_cast<long double>(std::numeric_limits<long>::max()) ||
        left < 0.0 || down < 0.0 || left >= nx_d || down >= ny_d) {
      return fail("grid refinement bounds are invalid or exceed the cell limit");
    }
    const std::size_t nx = static_cast<std::size_t>(nx_d);
    const std::size_t ny = static_cast<std::size_t>(ny_d);
    if (nx > std::numeric_limits<std::size_t>::max() / ny) {
      return fail("grid refinement bounds are invalid or exceed index range");
    }
    const std::size_t start_x = static_cast<std::size_t>(left);
    const std::size_t start_y = static_cast<std::size_t>(down);
    if (start_x >= nx || start_y >= ny) {
      return fail("grid refinement could not represent the segment start");
    }

    std::vector<Cell> cells;
    cells.reserve(std::min(
        {limits_.max_cells, limits_.max_expansions, std::size_t{4096}}));
    std::unordered_map<CellKey, std::size_t, CellKeyHash> cell_lookup;
    cell_lookup.reserve(cells.capacity());
    std::unordered_map<CellKey, std::size_t, CellKeyHash>
        known_incoming_lookup;
    known_incoming_lookup.reserve(cells.capacity());
    const auto index = [nx](std::size_t x, std::size_t y) {
      return y * nx + x;
    };
    const auto xy = [nx](std::size_t cell) {
      return std::pair<std::size_t, std::size_t>(cell % nx, cell / nx);
    };
    const auto ensureCell = [&](std::size_t cell, double parent_z,
                                std::string& reason) {
      if (!std::isfinite(parent_z)) return kNoParent;
      const CellKey key{cell, heightBits(parent_z)};
      const auto found = cell_lookup.find(key);
      if (found != cell_lookup.end()) {
        return cells[found->second].valid ? found->second : kNoParent;
      }
      const auto known_incoming = known_incoming_lookup.find(key);
      if (known_incoming != known_incoming_lookup.end()) {
        return cells[known_incoming->second].valid ? known_incoming->second
                                                   : kNoParent;
      }
      if (cells.size() >= limits_.max_cells) {
        reason = "grid refinement exceeded the cell limit";
        return kNoParent;
      }
      const std::size_t node = cells.size();
      cells.emplace_back();
      cell_lookup.emplace(key, node);
      Cell& value = cells.back();
      value.grid_index = cell;
      value.projection_tried = true;
      const auto [x, y] = xy(cell);
      const double lattice_x = origin_x + static_cast<double>(x) * resolution;
      const double lattice_y = origin_y + static_cast<double>(y) * resolution;
      value.state = StateVec(lattice_x, lattice_y, parent_z, 0.0);
      if (interrupted(reason)) return kNoParent;
      const GridProjectionStatus projection_status = project_state(value.state);
      if (interrupted(reason)) return kNoParent;
      if (!projectionAccepted(projection_status) || !value.state.allFinite()) {
        return kNoParent;
      }
      // Keep the discrete XY identity separate from a small support-probe
      // correction, but reject a projection that jumps into another cell.
      if (std::abs(value.state.x() - lattice_x) > 0.5 * resolution + 1e-9 ||
          std::abs(value.state.y() - lattice_y) > 0.5 * resolution + 1e-9) {
        return kNoParent;
      }
      if (projection_status == GridProjectionStatus::kSupported) {
        const CellKey canonical{cell, heightBits(value.state.z())};
        if (!(canonical == key)) {
          const auto canonical_found = cell_lookup.find(canonical);
          if (canonical_found != cell_lookup.end()) {
            const std::size_t canonical_node = canonical_found->second;
            // Do not retain an alias for every incoming parent height. Each
            // live node owns at most its input key and one known-height key,
            // so lookup memory remains bounded by twice max_cells.
            cell_lookup.erase(key);
            cells.pop_back();
            // The immutable planner snapshot makes this exact incoming-height
            // alias stable for the rest of refine(). Keep it outside the node
            // lookup so node-owned keys retain their existing 2*max_cells
            // bound; once the separate max_cells memo is full, safely fall
            // back to the projection path above.
            if (known_incoming_lookup.size() < limits_.max_cells) {
              known_incoming_lookup.emplace(key, canonical_node);
            }
            return cells[canonical_node].valid ? canonical_node : kNoParent;
          }
          cell_lookup.emplace(canonical, node);
        }
      }
      value.valid = true;
      return node;
    };

    const std::size_t source = index(start_x, start_y);
    const std::size_t source_node = cells.size();
    cells.emplace_back();
    cells.back().grid_index = source;
    cell_lookup.emplace(CellKey{source, heightBits(from.z())}, source_node);
    Cell& source_cell = cells.back();
    source_cell.projection_tried = true;
    source_cell.valid = true;
    source_cell.state = from;
    source_cell.cost = 0.0;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, LaterEntry> open;
    // Every expanded edge invokes terrain and swept-body map queries. Bias the
    // queue toward the goal to trade shortest-path optimality for substantially
    // less map work on wide, mostly traversable objective grids.
    constexpr double kHeuristicWeight = 1.5;
    open.push({kHeuristicWeight * (to.head<2>() - from.head<2>()).norm(), 0.0, source_node,
               sequence++});

    constexpr int kDx[8] = {1, 0, -1, 0, 1, -1, -1, 1};
    constexpr int kDy[8] = {0, 1, 0, -1, 1, 1, -1, -1};
    std::size_t reached = kNoParent;
    while (!open.empty()) {
      if (interrupted(interruption_reason)) return fail(interruption_reason);
      const QueueEntry entry = open.top();
      open.pop();
      Cell& active = cells.at(entry.index);
      if (active.closed || entry.cost > active.cost + 1e-12) continue;
      if (expansions >= limits_.max_expansions) {
        return fail("grid refinement exceeded the expansion limit");
      }
      ++expansions;
      active.closed = true;
      const StateVec active_state = active.state;
      const double active_cost = active.cost;
      const std::size_t active_grid_index = active.grid_index;

      if ((to.head<2>() - active_state.head<2>()).norm() <=
          std::sqrt(2.0) * resolution + 1e-9) {
        if (traverse(active_state, to, nullptr, nullptr,
                     interruption_reason)) {
          connected_to = to;
          reached = entry.index;
          break;
        }
        if (!interruption_reason.empty()) return fail(interruption_reason);

        // A provisional target initially carries the request/start height,
        // while the search may have reached it through a gradual mapped
        // climb. Reproject a failed connector from this checked parent's
        // driving plane. Known terrain still selects its measured surface;
        // unknown terrain can only inherit the adjacent connected height.
        StateVec inherited_to = to;
        inherited_to.z() = active_state.z();
        const Eigen::Vector2d requested_xy = inherited_to.head<2>();
        const double requested_yaw = inherited_to[3];
        const GridProjectionStatus inherited_status =
            project_state(inherited_to);
        if (interrupted(interruption_reason)) return fail(interruption_reason);
        const bool exact_endpoint_preserved =
            (inherited_to.head<2>() - requested_xy).norm() <= 1e-9;
        inherited_to[3] = requested_yaw;
        if (!corridor.partial && segment + 1 == waypoints.size() &&
            corridor.request.objective == ObjectiveKind::kNavigate &&
            goal_status == GridProjectionStatus::kProvisionalUnknown &&
            inherited_status == GridProjectionStatus::kProvisionalUnknown &&
            exact_endpoint_preserved && inherited_to.allFinite() &&
            traverse(active_state, inherited_to, nullptr, nullptr,
                     interruption_reason)) {
          connected_to = inherited_to;
          reached = entry.index;
          break;
        }
      }
      if (!interruption_reason.empty()) return fail(interruption_reason);

      const auto [active_x, active_y] = xy(active_grid_index);
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
        const std::size_t next_grid =
            index(static_cast<std::size_t>(next_x),
                  static_cast<std::size_t>(next_y));
        const std::size_t next =
            ensureCell(next_grid, active_state.z(), interruption_reason);
        if (next == kNoParent || cells.at(next).closed) {
          if (!interruption_reason.empty()) return fail(interruption_reason);
          continue;
        }

        if (kDx[direction] != 0 && kDy[direction] != 0) {
          const std::size_t side_x_grid =
              index(static_cast<std::size_t>(next_x), active_y);
          const std::size_t side_y_grid =
              index(active_x, static_cast<std::size_t>(next_y));
          const std::size_t side_x =
              ensureCell(side_x_grid, active_state.z(), interruption_reason);
          const std::size_t side_y =
              ensureCell(side_y_grid, active_state.z(), interruption_reason);
          if (side_x == kNoParent || side_y == kNoParent ||
              !traverse(active_state, cells.at(side_x).state, nullptr, nullptr,
                        interruption_reason) ||
              !traverse(active_state, cells.at(side_y).state, nullptr, nullptr,
                        interruption_reason)) {
            if (!interruption_reason.empty()) return fail(interruption_reason);
            continue;
          }
        }
        double edge = 0.0;
        if (!traverse(active_state, cells.at(next).state, nullptr, &edge,
                      interruption_reason)) {
          if (!interruption_reason.empty()) return fail(interruption_reason);
          continue;
        }
        const double candidate = active_cost + edge;
        if (candidate + 1e-12 >= cells.at(next).cost) continue;
        cells.at(next).cost = candidate;
        cells.at(next).parent = entry.index;
        const double heuristic =
            (to.head<2>() - cells.at(next).state.head<2>()).norm();
        open.push({candidate + kHeuristicWeight * heuristic, candidate, next,
                   sequence++});
        admitted_neighbor = true;
      }

      // A physical start may sit in a camera/lidar ground blind spot. If none
      // of its adjacent cells can be admitted, look for the nearest supported
      // lattice cell within the explicitly bounded connector distance. The
      // caller's traversal predicate still checks the complete swept body,
      // terrain step and geofence; this search grants no unknown-space waiver.
      if (entry.index == source_node && segment == 1 && !admitted_neighbor &&
          limits_.start_connector_max_distance_m > resolution) {
        struct Candidate {
          double distance;
          std::size_t cell;
        };
        const double maximum = limits_.start_connector_max_distance_m;
        const long double radius_bound =
            std::ceil(static_cast<long double>(maximum) /
                      static_cast<long double>(resolution));
        if (!std::isfinite(radius_bound) ||
            radius_bound >=
                static_cast<long double>(std::numeric_limits<long>::max())) {
          return fail("grid start connector bounds are invalid");
        }
        const long grid_radius = std::max(
            {static_cast<long>(start_x),
             static_cast<long>(nx - 1u - start_x),
             static_cast<long>(start_y),
             static_cast<long>(ny - 1u - start_y)});
        const long radius =
            std::min(static_cast<long>(radius_bound), grid_radius);
        const std::size_t remaining_cells = limits_.max_cells - cells.size();
        const std::size_t candidate_limit =
            std::min(remaining_cells, limits_.max_expansions);
        if (candidate_limit == 0u) {
          return fail("grid refinement exceeded the cell limit");
        }
        const auto closer = [](const Candidate& a, const Candidate& b) {
          return a.distance < b.distance ||
                 (a.distance == b.distance && a.cell < b.cell);
        };
        // Keep only the nearest candidates that can fit in this request's
        // existing cell/expansion budgets. The scan may inspect the clipped
        // grid, but every iteration observes the cooperative deadline and its
        // retained storage does not grow with the configured radius.
        std::priority_queue<Candidate, std::vector<Candidate>,
                            decltype(closer)>
            nearest(closer);
        for (long offset_y = -radius; offset_y <= radius; ++offset_y) {
          for (long offset_x = -radius; offset_x <= radius; ++offset_x) {
            if (interrupted(interruption_reason)) return fail(interruption_reason);
            if (offset_x < -static_cast<long>(start_x) ||
                offset_x > static_cast<long>(nx - 1u - start_x) ||
                offset_y < -static_cast<long>(start_y) ||
                offset_y > static_cast<long>(ny - 1u - start_y)) {
              continue;
            }
            const long double dx =
                static_cast<long double>(offset_x) * resolution;
            const long double dy =
                static_cast<long double>(offset_y) * resolution;
            const long double distance = std::hypot(dx, dy);
            if (!std::isfinite(distance) ||
                distance <= static_cast<long double>(resolution) + 1e-9L ||
                distance > static_cast<long double>(maximum) + 1e-9L) {
              continue;
            }
            const std::size_t candidate_x = static_cast<std::size_t>(
                static_cast<long>(start_x) + offset_x);
            const std::size_t candidate_y = static_cast<std::size_t>(
                static_cast<long>(start_y) + offset_y);
            const Candidate candidate{
                static_cast<double>(distance), index(candidate_x, candidate_y)};
            if (nearest.size() < candidate_limit) {
              nearest.push(candidate);
            } else if (closer(candidate, nearest.top())) {
              nearest.pop();
              nearest.push(candidate);
            }
          }
        }
        std::vector<Candidate> candidates;
        candidates.reserve(nearest.size());
        while (!nearest.empty()) {
          candidates.push_back(nearest.top());
          nearest.pop();
        }
        std::sort(candidates.begin(), candidates.end(),
                  closer);
        if (interrupted(interruption_reason)) return fail(interruption_reason);
        for (const Candidate& candidate : candidates) {
          const std::size_t candidate_node = ensureCell(
              candidate.cell, active_state.z(), interruption_reason);
          if (candidate_node == kNoParent) {
            if (!interruption_reason.empty()) return fail(interruption_reason);
            continue;
          }
          double edge = 0.0;
          if (!traverse(active_state, cells.at(candidate_node).state, nullptr, &edge,
                        interruption_reason)) {
            if (!interruption_reason.empty()) return fail(interruption_reason);
            continue;
          }
          cells.at(candidate_node).cost = edge;
          cells.at(candidate_node).parent = source_node;
          const double heuristic =
              (to.head<2>() - cells.at(candidate_node).state.head<2>()).norm();
          open.push({edge + kHeuristicWeight * heuristic, edge,
                     candidate_node, sequence++});
        }
      }
    }
    if (reached == kNoParent) {
      return fail("no observed traversable grid detour within the bounded "
                  "search window for route segment " +
                  std::to_string(segment - 1) + "->" +
                  std::to_string(segment));
    }

    std::vector<StateVec> reversed;
    for (std::size_t cell = reached; cell != source_node;) {
      if (cell == kNoParent || cells.at(cell).parent == kNoParent) {
        return fail("grid refinement produced an invalid parent chain");
      }
      reversed.push_back(cells.at(cell).state);
      cell = cells.at(cell).parent;
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
    if (!traverse(previous, connected_to, &checked_goal, nullptr,
                  interruption_reason)) {
      if (!interruption_reason.empty()) return fail(interruption_reason);
      return fail("grid detour failed final goal-connector validation");
    }
    if (result.poses.empty()) result.poses.push_back(previous);
    appendCheckedPath(checked_goal, result.poses);
    waypoints[segment] = connected_to;
  }

  for (std::size_t i = 1; i < result.poses.size(); ++i) {
    if (interrupted(interruption_reason)) return fail(interruption_reason);
    result.poses[i - 1][3] = std::atan2(
        result.poses[i].y() - result.poses[i - 1].y(),
        result.poses[i].x() - result.poses[i - 1].x());
  }
  active_segment_known = false;
  if (interrupted(interruption_reason)) return fail(interruption_reason);
  result.poses.back()[3] = goal[3];
  result.status = PlanningStatus::kSucceeded;
  result.reason.clear();
  result.blocked_segment_identified = false;
  return result;
}

}  // namespace mgg
