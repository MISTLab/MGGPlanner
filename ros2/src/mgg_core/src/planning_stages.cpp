#include "mgg_core/planning_stages.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mgg {
namespace {

// Walks the parent chain of a report this file produced. GraphManager's own
// helper is kept for reports produced by its Boost search, so that search
// keeps its existing behaviour exactly.
void extractPath(GraphManager& graph, int target_id,
                 const ShortestPathsReport& report,
                 std::vector<StateVec>& path) {
  path.clear();
  if (!report.status) return;
  std::vector<int> ids;
  int cursor = target_id;
  // The chain is at most one vertex per graph vertex; the visited set makes a
  // corrupted parent map terminate instead of looping.
  std::unordered_set<int> visited;
  while (true) {
    if (!visited.insert(cursor).second) {
      path.clear();
      return;
    }
    ids.push_back(cursor);
    if (cursor == report.source_id) break;
    const auto parent = report.parent_id_map.find(cursor);
    if (parent == report.parent_id_map.end() || parent->second == cursor) {
      path.clear();
      return;
    }
    cursor = parent->second;
  }
  std::reverse(ids.begin(), ids.end());
  path.reserve(ids.size());
  for (const int id : ids) {
    const auto vertex = graph.vertices_map_.find(id);
    if (vertex == graph.vertices_map_.end() || vertex->second == nullptr) {
      path.clear();
      return;
    }
    path.push_back(vertex->second->state);
  }
}

// Deterministic Dijkstra over GraphManager's adjacency map that refuses to
// traverse any segment the caller has marked blocked. Only used while at
// least one mark is live, so the unmarked case keeps the existing Boost
// search and its exact results.
bool findShortestPathsAvoiding(GraphManager& graph, int source_id,
                               const BlockedCorridorView& blocked,
                               ShortestPathsReport& report) {
  report.reset();
  report.parent_id_map.clear();
  report.distance_map.clear();
  const auto source = graph.vertices_map_.find(source_id);
  if (source == graph.vertices_map_.end() || source->second == nullptr) {
    return false;
  }
  report.source_id = source_id;
  report.status = true;
  report.parent_id_map[source_id] = source_id;
  report.distance_map[source_id] = 0.0;
  struct Entry {
    double cost;
    int id;
  };
  const auto later = [](const Entry& a, const Entry& b) {
    if (a.cost != b.cost) return a.cost > b.cost;
    return a.id > b.id;
  };
  std::priority_queue<Entry, std::vector<Entry>, decltype(later)> open(later);
  open.push({0.0, source_id});
  std::unordered_set<int> closed;
  while (!open.empty()) {
    const Entry entry = open.top();
    open.pop();
    if (!closed.insert(entry.id).second) continue;
    const auto neighbours = graph.edge_map_.find(entry.id);
    if (neighbours == graph.edge_map_.end()) continue;
    const auto from = graph.vertices_map_.find(entry.id);
    if (from == graph.vertices_map_.end() || from->second == nullptr) continue;
    for (const auto& edge : neighbours->second) {
      const int next_id = edge.first;
      if (closed.count(next_id) != 0) continue;
      const double weight = edge.second;
      if (!std::isfinite(weight) || weight < 0.0) continue;
      const auto to = graph.vertices_map_.find(next_id);
      if (to == graph.vertices_map_.end() || to->second == nullptr) continue;
      if (blocked.blocked(from->second->state, to->second->state)) continue;
      const double candidate = entry.cost + weight;
      const auto known = report.distance_map.find(next_id);
      if (known != report.distance_map.end() &&
          known->second <= candidate + 1e-12) {
        continue;
      }
      report.distance_map[next_id] = candidate;
      report.parent_id_map[next_id] = entry.id;
      open.push({candidate, next_id});
    }
  }
  return true;
}

}  // namespace

BlockedCorridorRegistry::BlockedCorridorRegistry(
    const BlockedCorridorLimits& limits) {
  setLimits(limits);
}

void BlockedCorridorRegistry::setLimits(const BlockedCorridorLimits& limits) {
  limits_ = limits;
  if (!std::isfinite(limits_.cell_size_m) || limits_.cell_size_m <= 0.0) {
    limits_.cell_size_m = 0.5;
  }
  if (!std::isfinite(limits_.ttl_s) || limits_.ttl_s < 0.0) {
    limits_.ttl_s = 0.0;
  }
  while (entries_.size() > limits_.max_entries) {
    auto oldest = entries_.begin();
    for (auto it = entries_.begin(); it != entries_.end(); ++it) {
      if (it->second.sequence < oldest->second.sequence) oldest = it;
    }
    entries_.erase(oldest);
  }
}

bool BlockedCorridorRegistry::makeKey(const StateVec& from, const StateVec& to,
                                      Key& key) const {
  if (!from.allFinite() || !to.allFinite()) return false;
  if ((from.head<3>() - to.head<3>()).cwiseAbs().maxCoeff() <= 1e-9) {
    return false;
  }
  const double cell = limits_.cell_size_m;
  std::int64_t a[3];
  std::int64_t b[3];
  for (int i = 0; i < 3; ++i) {
    const double from_cell = std::floor(from[i] / cell);
    const double to_cell = std::floor(to[i] / cell);
    // A pose far outside any plausible world frame cannot be keyed safely.
    constexpr double kCellBound = 1e15;
    if (std::abs(from_cell) > kCellBound || std::abs(to_cell) > kCellBound) {
      return false;
    }
    a[i] = static_cast<std::int64_t>(from_cell);
    b[i] = static_cast<std::int64_t>(to_cell);
  }
  // Undirected: order the two endpoint cells so both traversal directions
  // produce one mark.
  const bool swap = std::tie(a[0], a[1], a[2]) > std::tie(b[0], b[1], b[2]);
  const std::int64_t* first = swap ? b : a;
  const std::int64_t* second = swap ? a : b;
  for (int i = 0; i < 3; ++i) {
    key.cells[i] = first[i];
    key.cells[i + 3] = second[i];
  }
  // A segment shorter than one cell would key to itself and block the pose
  // rather than a corridor.
  return !(key.cells[0] == key.cells[3] && key.cells[1] == key.cells[4] &&
           key.cells[2] == key.cells[5]);
}

bool BlockedCorridorRegistry::active(const Entry& entry,
                                     std::uint64_t map_revision,
                                     double now_s) const {
  if (std::isfinite(now_s) && now_s > entry.deadline_s) return false;
  if (map_revision >= entry.map_revision &&
      map_revision - entry.map_revision >= limits_.revision_window) {
    return false;
  }
  // A revision that moved backwards means the snapshot was replaced; the mark
  // no longer describes the map it was taken against.
  if (map_revision < entry.map_revision) return false;
  return true;
}

void BlockedCorridorRegistry::block(const StateVec& from, const StateVec& to,
                                    std::uint64_t map_revision, double now_s) {
  if (limits_.max_entries == 0 || !std::isfinite(now_s)) return;
  Key key;
  if (!makeKey(from, to, key)) return;
  expire(map_revision, now_s);
  Entry entry;
  entry.map_revision = map_revision;
  entry.deadline_s = now_s + limits_.ttl_s;
  entry.sequence = ++sequence_;
  entries_[key] = entry;
  while (entries_.size() > limits_.max_entries) {
    auto oldest = entries_.begin();
    for (auto it = entries_.begin(); it != entries_.end(); ++it) {
      if (it->second.sequence < oldest->second.sequence) oldest = it;
    }
    entries_.erase(oldest);
  }
}

bool BlockedCorridorRegistry::isBlocked(const StateVec& from,
                                        const StateVec& to,
                                        std::uint64_t map_revision,
                                        double now_s) const {
  Key key;
  if (!makeKey(from, to, key)) return false;
  const auto found = entries_.find(key);
  return found != entries_.end() &&
         active(found->second, map_revision, now_s);
}

std::size_t BlockedCorridorRegistry::activeCount(std::uint64_t map_revision,
                                                 double now_s) const {
  std::size_t count = 0;
  for (const auto& entry : entries_) {
    if (active(entry.second, map_revision, now_s)) ++count;
  }
  return count;
}

void BlockedCorridorRegistry::expire(std::uint64_t map_revision,
                                     double now_s) {
  for (auto it = entries_.begin(); it != entries_.end();) {
    it = active(it->second, map_revision, now_s) ? std::next(it)
                                                 : entries_.erase(it);
  }
}

void BlockedCorridorRegistry::clear() { entries_.clear(); }

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
    const PlanningRequest& request,
    const BlockedCorridorView& blocked) const {
  RouteCorridor result;
  result.request = request;
  if (request.objective != ObjectiveKind::kNavigate &&
      request.objective != ObjectiveKind::kReturnHome &&
      request.objective != ObjectiveKind::kExplore) {
    result.status = PlanningStatus::kUnsupportedObjective;
    result.reason = "objective has no topological goal stage";
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
  // Routing around marks is the only reason to leave the existing search:
  // with nothing marked, the corridor is bit-for-bit the one MGG produced
  // before blocked-corridor feedback existed.
  const bool avoid_blocked = blocked.active();
  ShortestPathsReport shortest;
  const auto shortestPathTo = [&graph, &shortest, avoid_blocked](
                                  int target_id, std::vector<StateVec>& path) {
    if (avoid_blocked) {
      extractPath(graph, target_id, shortest, path);
    } else {
      graph.getShortestPath(target_id, shortest, true, path);
    }
  };
  if (!(avoid_blocked
            ? findShortestPathsAvoiding(graph, source->id, blocked, shortest)
            : graph.findShortestPaths(source->id, shortest))) {
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
      shortestPathTo(candidate.vertex->id, candidate_path);
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
    shortestPathTo(target->id, result.poses);
  }
  if (result.poses.empty() || !result.poses.front().isApprox(source->state)) {
    result.poses.clear();
    if (request.objective == ObjectiveKind::kNavigate) {
      result.status = PlanningStatus::kSucceeded;
      result.reason.clear();
      result.poses = {current, goal};
      return result;
    }
    result.reason = avoid_blocked
                        ? "goal has no unblocked corridor in the active graph"
                        : "goal is in a disconnected graph component";
    return result;
  }
  result.status = PlanningStatus::kSucceeded;
  return result;
}

}  // namespace mgg
