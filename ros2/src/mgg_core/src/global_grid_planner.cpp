#include "mgg_core/global_grid_planner.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <queue>
#include <utility>

namespace mgg {
namespace {

constexpr std::size_t kNoParent = std::numeric_limits<std::size_t>::max();
/// Bound on the cells a raster may hold before the per-cell search arrays
/// (cost, parent, flags) are refused: 16 million cells is a 2 km square at
/// 0.5 m, far beyond any single-component map this planner serves.
constexpr std::size_t kMaxRasterCells = std::size_t{16} * 1024u * 1024u;
constexpr int kDx[8] = {1, 0, -1, 0, 1, -1, -1, 1};
constexpr int kDy[8] = {0, 1, 0, -1, 1, 1, -1, -1};

struct QueueEntry {
  double estimate = 0.0;
  double cost = 0.0;
  std::size_t index = 0;
  std::size_t sequence = 0;
};

struct LaterEntry {
  bool operator()(const QueueEntry& a, const QueueEntry& b) const {
    if (a.estimate != b.estimate) return a.estimate > b.estimate;
    // Among equal estimates prefer the deeper node: it reaches the goal
    // sooner across open ground. Then the lower index, then insertion order,
    // so the search is deterministic for one raster.
    if (a.cost != b.cost) return a.cost < b.cost;
    if (a.index != b.index) return a.index > b.index;
    return a.sequence > b.sequence;
  }
};

double median(std::vector<double>& values) {
  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();
  return n % 2u == 1u ? values[n / 2u]
                      : 0.5 * (values[n / 2u - 1u] + values[n / 2u]);
}

}  // namespace

const char* toString(GlobalGridPlanStatus status) {
  switch (status) {
    case GlobalGridPlanStatus::kSucceeded:
      return "succeeded";
    case GlobalGridPlanStatus::kNoRaster:
      return "no raster";
    case GlobalGridPlanStatus::kInvalidConfiguration:
      return "invalid configuration";
    case GlobalGridPlanStatus::kStartOutside:
      return "start outside the raster";
    case GlobalGridPlanStatus::kStartUnknown:
      return "start unknown";
    case GlobalGridPlanStatus::kStartObstacle:
      return "start obstacle";
    case GlobalGridPlanStatus::kGoalOutside:
      return "goal outside the raster";
    case GlobalGridPlanStatus::kGoalUnknown:
      return "goal unknown";
    case GlobalGridPlanStatus::kGoalObstacle:
      return "goal obstacle";
    case GlobalGridPlanStatus::kGoalInDisc:
      return "goal inside a transient disc";
    case GlobalGridPlanStatus::kBudgetExceeded:
      return "expansion budget exceeded";
    case GlobalGridPlanStatus::kDeadlineExceeded:
      return "deadline exceeded";
    case GlobalGridPlanStatus::kUnreachable:
      return "unreachable";
  }
  return "unknown";
}

GlobalGridPlanner::GlobalGridPlanner(
    std::shared_ptr<const TraversabilityRaster> raster,
    GlobalGridPlannerLimits limits)
    : raster_(std::move(raster)), limits_(limits) {}

GlobalGridPlan GlobalGridPlanner::plan(const Eigen::Vector2d& start_xy,
                                       const Eigen::Vector2d& goal_xy,
                                       const std::vector<TransientDisc>& discs,
                                       const BlockedEdge& blocked) const {
  const auto started = std::chrono::steady_clock::now();
  GlobalGridPlan result;
  const auto finish = [&](GlobalGridPlanStatus status,
                          const std::string& reason) {
    result.status = status;
    result.reason = reason;
    result.elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - started);
    if (status != GlobalGridPlanStatus::kSucceeded) {
      result.poses.clear();
      result.length_m = 0.0;
    }
    return result;
  };

  if (raster_ == nullptr || raster_->empty() || !raster_->consistent()) {
    return finish(GlobalGridPlanStatus::kNoRaster,
                  "no traversability raster is available");
  }
  const TraversabilityRaster& raster = *raster_;
  if (raster.size() > kMaxRasterCells) {
    return finish(GlobalGridPlanStatus::kNoRaster,
                  "traversability raster exceeds the cell bound");
  }
  if (!std::isfinite(limits_.max_step_height) || limits_.max_step_height < 0.0 ||
      !std::isfinite(limits_.max_drop_height) || limits_.max_drop_height < 0.0 ||
      !std::isfinite(limits_.step_tolerance) || limits_.step_tolerance < 0.0 ||
      !std::isfinite(limits_.driving_offset) ||
      !std::isfinite(limits_.body_radius) || limits_.body_radius < 0.0 ||
      !std::isfinite(limits_.blocked_penalty) || limits_.blocked_penalty < 0.0 ||
      limits_.max_expansions == 0 || limits_.timeout.count() <= 0) {
    return finish(GlobalGridPlanStatus::kInvalidConfiguration,
                  "global grid planner limits are invalid");
  }
  std::size_t start_x = 0;
  std::size_t start_y = 0;
  std::size_t goal_x = 0;
  std::size_t goal_y = 0;
  if (!raster.cellOf(start_xy, start_x, start_y)) {
    char reason[160];
    std::snprintf(reason, sizeof(reason),
                  "start (%.2f, %.2f) lies outside the traversability raster",
                  start_xy.x(), start_xy.y());
    return finish(GlobalGridPlanStatus::kStartOutside, reason);
  }
  if (!raster.cellOf(goal_xy, goal_x, goal_y)) {
    char reason[160];
    std::snprintf(reason, sizeof(reason),
                  "goal (%.2f, %.2f) lies outside the traversability raster",
                  goal_xy.x(), goal_xy.y());
    return finish(GlobalGridPlanStatus::kGoalOutside, reason);
  }
  const std::size_t start = raster.index(start_x, start_y);
  const std::size_t goal = raster.index(goal_x, goal_y);
  const double cell = raster.cell_size;
  const double rise_limit = limits_.max_step_height + limits_.step_tolerance;
  const double drop_limit = limits_.max_drop_height + limits_.step_tolerance;

  // Endpoint ground. The robot's own footprint and a goal on a departed
  // neighbour's masked footprint are unobserved, so an unknown endpoint cell
  // may adopt the median ground of the known free cells whose square lies
  // within the body radius (at least the immediate ring) of the endpoint,
  // provided those cells agree within one step of each other. An obstacle
  // endpoint is refused outright.
  const auto resolveEndpoint = [&](std::size_t cx, std::size_t cy,
                                   const Eigen::Vector2d& xy, bool is_start,
                                   double& ground, GlobalGridPlanStatus& status,
                                   std::string& reason) {
    const char* label = is_start ? "start" : "goal";
    const std::size_t index = raster.index(cx, cy);
    const RasterCellState state = raster.state[index];
    if (state == RasterCellState::kObstacle) {
      char text[160];
      std::snprintf(text, sizeof(text),
                    "%s (%.2f, %.2f) lies on an obstacle cell", label, xy.x(),
                    xy.y());
      status = is_start ? GlobalGridPlanStatus::kStartObstacle
                        : GlobalGridPlanStatus::kGoalObstacle;
      reason = text;
      return false;
    }
    if (state == RasterCellState::kFree) {
      ground = raster.ground_z[index];
      if (std::isfinite(ground)) return true;
    }
    const double radius = std::max(limits_.body_radius, cell);
    const long reach = static_cast<long>(std::ceil(radius / cell)) + 1;
    std::vector<double> heights;
    for (long dy = -reach; dy <= reach; ++dy) {
      for (long dx = -reach; dx <= reach; ++dx) {
        if (dx == 0 && dy == 0) continue;
        const long nx = static_cast<long>(cx) + dx;
        const long ny = static_cast<long>(cy) + dy;
        if (nx < 0 || ny < 0 || nx >= static_cast<long>(raster.width) ||
            ny >= static_cast<long>(raster.height)) {
          continue;
        }
        const std::size_t neighbour =
            raster.index(static_cast<std::size_t>(nx),
                         static_cast<std::size_t>(ny));
        if (raster.state[neighbour] != RasterCellState::kFree) continue;
        const double height = raster.ground_z[neighbour];
        if (!std::isfinite(height)) continue;
        // Distance from the endpoint to the nearest point of the cell square.
        const Eigen::Vector2d centre = raster.centre(
            static_cast<std::size_t>(nx), static_cast<std::size_t>(ny));
        const double gap_x = std::max(0.0, std::abs(xy.x() - centre.x()) - 0.5 * cell);
        const double gap_y = std::max(0.0, std::abs(xy.y() - centre.y()) - 0.5 * cell);
        if (std::hypot(gap_x, gap_y) > radius + 1e-9) continue;
        heights.push_back(height);
      }
    }
    status = is_start ? GlobalGridPlanStatus::kStartUnknown
                      : GlobalGridPlanStatus::kGoalUnknown;
    if (heights.empty()) {
      char text[200];
      std::snprintf(text, sizeof(text),
                    "%s (%.2f, %.2f) is unobserved and no known free cell lies "
                    "within %.2f m",
                    label, xy.x(), xy.y(), radius);
      reason = text;
      return false;
    }
    const auto extremes = std::minmax_element(heights.begin(), heights.end());
    if (*extremes.second - *extremes.first > rise_limit) {
      char text[200];
      std::snprintf(text, sizeof(text),
                    "%s (%.2f, %.2f) is unobserved and its known neighbours "
                    "disagree on the ground (%.2f m spread)",
                    label, xy.x(), xy.y(), *extremes.second - *extremes.first);
      reason = text;
      return false;
    }
    ground = median(heights);
    return true;
  };

  double start_ground = 0.0;
  double goal_ground = 0.0;
  GlobalGridPlanStatus endpoint_status = GlobalGridPlanStatus::kSucceeded;
  std::string endpoint_reason;
  if (!resolveEndpoint(start_x, start_y, start_xy, true, start_ground,
                       endpoint_status, endpoint_reason)) {
    return finish(endpoint_status, endpoint_reason);
  }
  if (!resolveEndpoint(goal_x, goal_y, goal_xy, false, goal_ground,
                       endpoint_status, endpoint_reason)) {
    return finish(endpoint_status, endpoint_reason);
  }

  // Transient discs, rasterised once: a cell is refused when its centre lies
  // within a disc. The start cell is exempt (the robot stands where it
  // stands and may drive away from a parked neighbour); a goal inside a live
  // disc is refused, because the neighbour has not left.
  std::vector<std::uint8_t> in_disc;
  for (const TransientDisc& disc : discs) {
    if (!disc.centre.allFinite() || !std::isfinite(disc.radius) ||
        disc.radius <= 0.0) {
      continue;
    }
    if (in_disc.empty()) in_disc.assign(raster.size(), 0);
    const double min_x = disc.centre.x() - disc.radius;
    const double max_x = disc.centre.x() + disc.radius;
    const double min_y = disc.centre.y() - disc.radius;
    const double max_y = disc.centre.y() + disc.radius;
    const double lo_x = std::floor((min_x - raster.origin.x()) / cell);
    const double hi_x = std::floor((max_x - raster.origin.x()) / cell);
    const double lo_y = std::floor((min_y - raster.origin.y()) / cell);
    const double hi_y = std::floor((max_y - raster.origin.y()) / cell);
    const std::size_t first_x = static_cast<std::size_t>(std::max(0.0, lo_x));
    const std::size_t first_y = static_cast<std::size_t>(std::max(0.0, lo_y));
    if (hi_x < 0.0 || hi_y < 0.0 || lo_x >= static_cast<double>(raster.width) ||
        lo_y >= static_cast<double>(raster.height)) {
      continue;
    }
    const std::size_t last_x = static_cast<std::size_t>(
        std::min(hi_x, static_cast<double>(raster.width) - 1.0));
    const std::size_t last_y = static_cast<std::size_t>(
        std::min(hi_y, static_cast<double>(raster.height) - 1.0));
    const double radius_squared = disc.radius * disc.radius;
    for (std::size_t y = first_y; y <= last_y; ++y) {
      for (std::size_t x = first_x; x <= last_x; ++x) {
        if ((raster.centre(x, y) - disc.centre).squaredNorm() < radius_squared) {
          in_disc[raster.index(x, y)] = 1;
        }
      }
    }
  }
  const auto discBlocked = [&](std::size_t index) {
    return !in_disc.empty() && in_disc[index] != 0 && index != start;
  };
  if (goal != start && discBlocked(goal)) {
    char reason[160];
    std::snprintf(reason, sizeof(reason),
                  "goal (%.2f, %.2f) lies inside a transient disc", goal_xy.x(),
                  goal_xy.y());
    return finish(GlobalGridPlanStatus::kGoalInDisc, reason);
  }

  const auto groundOf = [&](std::size_t index) {
    if (index == start) return start_ground;
    if (index == goal) return goal_ground;
    return raster.ground_z[index];
  };
  // A cell the search may stand on: known free, or one of the two endpoints,
  // whose ground was resolved above.
  const auto admissible = [&](std::size_t index) {
    if (index == start || index == goal) return !discBlocked(index);
    return raster.state[index] == RasterCellState::kFree && !discBlocked(index) &&
           std::isfinite(raster.ground_z[index]);
  };
  const auto poseOf = [&](std::size_t index) {
    const std::size_t x = index % raster.width;
    const std::size_t y = index / raster.width;
    const Eigen::Vector2d centre = raster.centre(x, y);
    return StateVec(centre.x(), centre.y(),
                    groundOf(index) + limits_.driving_offset, 0.0);
  };
  const auto emit = [&](const std::vector<std::size_t>& chain) {
    result.poses.clear();
    result.poses.reserve(chain.size());
    for (const std::size_t index : chain) result.poses.push_back(poseOf(index));
    result.length_m = 0.0;
    for (std::size_t i = 1; i < result.poses.size(); ++i) {
      const Eigen::Vector2d delta =
          result.poses[i].head<2>() - result.poses[i - 1].head<2>();
      result.length_m += delta.norm();
      result.poses[i - 1][3] = std::atan2(delta.y(), delta.x());
    }
    if (result.poses.size() >= 2u) {
      result.poses.back()[3] = result.poses[result.poses.size() - 2u][3];
    }
    return finish(GlobalGridPlanStatus::kSucceeded, std::string());
  };

  if (start == goal) return emit({start});

  std::vector<double> cost(raster.size(),
                           std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parent(raster.size(), kNoParent);
  std::vector<std::uint8_t> closed(raster.size(), 0);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, LaterEntry> open;
  const Eigen::Vector2d goal_centre = raster.centre(goal_x, goal_y);
  const auto heuristic = [&](std::size_t index) {
    return (raster.centre(index % raster.width, index / raster.width) -
            goal_centre)
        .norm();
  };
  std::size_t sequence = 0;
  cost[start] = 0.0;
  open.push({heuristic(start), 0.0, start, sequence++});
  const double straight = cell;
  const double diagonal = cell * std::sqrt(2.0);
  const auto deadline = started + limits_.timeout;

  while (!open.empty()) {
    const QueueEntry entry = open.top();
    open.pop();
    if (closed[entry.index] != 0 || entry.cost > cost[entry.index] + 1e-12) {
      continue;
    }
    if (entry.index == goal) {
      std::vector<std::size_t> chain;
      for (std::size_t index = goal; index != kNoParent; index = parent[index]) {
        chain.push_back(index);
        if (chain.size() > raster.size()) {
          return finish(GlobalGridPlanStatus::kUnreachable,
                        "global grid search produced an invalid parent chain");
        }
      }
      std::reverse(chain.begin(), chain.end());
      return emit(chain);
    }
    if (result.expansions >= limits_.max_expansions) {
      char reason[160];
      std::snprintf(reason, sizeof(reason),
                    "global grid search exceeded its expansion budget (%zu)",
                    limits_.max_expansions);
      return finish(GlobalGridPlanStatus::kBudgetExceeded, reason);
    }
    if ((result.expansions & 63u) == 0u &&
        std::chrono::steady_clock::now() >= deadline) {
      char reason[160];
      std::snprintf(reason, sizeof(reason),
                    "global grid search exceeded its deadline (%lld ms) after "
                    "%zu expansions",
                    static_cast<long long>(limits_.timeout.count()),
                    result.expansions);
      return finish(GlobalGridPlanStatus::kDeadlineExceeded, reason);
    }
    ++result.expansions;
    closed[entry.index] = 1;
    const std::size_t active_x = entry.index % raster.width;
    const std::size_t active_y = entry.index / raster.width;
    const double active_ground = groundOf(entry.index);
    StateVec active_pose;
    bool active_pose_ready = false;
    for (int direction = 0; direction < 8; ++direction) {
      const long nx = static_cast<long>(active_x) + kDx[direction];
      const long ny = static_cast<long>(active_y) + kDy[direction];
      if (nx < 0 || ny < 0 || nx >= static_cast<long>(raster.width) ||
          ny >= static_cast<long>(raster.height)) {
        continue;
      }
      const std::size_t next = raster.index(static_cast<std::size_t>(nx),
                                            static_cast<std::size_t>(ny));
      if (closed[next] != 0 || !admissible(next)) continue;
      const double next_ground = groundOf(next);
      const double delta = next_ground - active_ground;
      if (delta > rise_limit || -delta > drop_limit) continue;
      const bool is_diagonal = kDx[direction] != 0 && kDy[direction] != 0;
      if (is_diagonal) {
        const std::size_t side_x = raster.index(
            static_cast<std::size_t>(nx), active_y);
        const std::size_t side_y = raster.index(
            active_x, static_cast<std::size_t>(ny));
        if (!admissible(side_x) || !admissible(side_y)) continue;
      }
      double edge = is_diagonal ? diagonal : straight;
      if (blocked) {
        if (!active_pose_ready) {
          active_pose = poseOf(entry.index);
          active_pose_ready = true;
        }
        if (blocked(active_pose, poseOf(next))) edge += limits_.blocked_penalty;
      }
      const double candidate = entry.cost + edge;
      if (candidate + 1e-12 >= cost[next]) continue;
      cost[next] = candidate;
      parent[next] = entry.index;
      open.push({candidate + heuristic(next), candidate, next, sequence++});
    }
  }
  char reason[200];
  std::snprintf(reason, sizeof(reason),
                "no traversable route from (%.2f, %.2f) to (%.2f, %.2f) on the "
                "traversability raster after %zu expansions",
                start_xy.x(), start_xy.y(), goal_xy.x(), goal_xy.y(),
                result.expansions);
  return finish(GlobalGridPlanStatus::kUnreachable, reason);
}

}  // namespace mgg
