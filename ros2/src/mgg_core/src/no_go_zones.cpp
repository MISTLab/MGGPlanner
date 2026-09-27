#include "mgg_core/no_go_zones.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <queue>
#include <utility>

namespace mgg {
namespace {

constexpr double kEps = 1e-9;

/// Where along a -> b (0 to 1) the segment comes nearest `c`.
double nearestFraction(const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                       const Eigen::Vector2d& c) {
  const Eigen::Vector2d d = b - a;
  const double length_squared = d.squaredNorm();
  if (length_squared < 1e-18) return 0.0;
  return std::clamp((c - a).dot(d) / length_squared, 0.0, 1.0);
}

double nearestDistance(const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                       const Eigen::Vector2d& c) {
  return (c - (a + nearestFraction(a, b, c) * (b - a))).norm();
}

}  // namespace

void NoGoZones::set(std::vector<Eigen::Vector2d> centres, double reach) {
  centres_.clear();
  if (!(std::isfinite(reach) && reach > 0.0)) return;
  for (const Eigen::Vector2d& c : centres) {
    if (c.allFinite()) centres_.push_back(c);
  }
  reach_ = reach;
}

bool NoGoZones::inside(const Eigen::Vector3d& p) const {
  return std::any_of(centres_.begin(), centres_.end(),
                     [this, &p](const Eigen::Vector2d& c) {
                       return (p.head<2>() - c).norm() < reach_;
                     });
}

std::uint64_t NoGoZones::departing(const Eigen::Vector3d& p) const {
  std::uint64_t mask = 0;
  for (std::size_t z = 0; z < centres_.size() && z < 64; ++z) {
    if ((p.head<2>() - centres_[z]).norm() < reach_) mask |= 1ull << z;
  }
  return mask;
}

bool NoGoZones::step(const Eigen::Vector3d& a3, const Eigen::Vector3d& b3,
                     std::uint64_t& departing) const {
  const Eigen::Vector2d a = a3.head<2>();
  const Eigen::Vector2d b = b3.head<2>();
  std::uint64_t next = departing;
  for (std::size_t z = 0; z < centres_.size(); ++z) {
    const Eigen::Vector2d& c = centres_[z];
    const bool leaving = z < 64 && (departing & (1ull << z)) != 0;
    if (leaving) {
      // Monotonic outward: the segment is nearest the centre at its start.
      if (nearestFraction(a, b, c) > kEps &&
          nearestDistance(a, b, c) < (a - c).norm() - kEps) {
        return false;
      }
      if ((b - c).norm() >= reach_) next &= ~(1ull << z);
      continue;
    }
    if (nearestDistance(a, b, c) < reach_) return false;
  }
  departing = next;
  return true;
}

bool NoGoZones::pathAdmissible(const std::vector<Eigen::Vector3d>& path) const {
  if (centres_.empty() || path.empty()) return true;
  std::uint64_t leaving = departing(path.front());
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (!step(path[i - 1], path[i], leaving)) return false;
  }
  return !inside(path.back());
}

bool NoGoZones::blocksEdge(const Eigen::Vector3d& a3, const Eigen::Vector3d& b3,
                           const Eigen::Vector3d& robot) const {
  const Eigen::Vector2d a = a3.head<2>();
  const Eigen::Vector2d b = b3.head<2>();
  for (const Eigen::Vector2d& c : centres_) {
    const double t = nearestFraction(a, b, c);
    const double nearest = (c - (a + t * (b - a))).norm();
    if (nearest >= reach_) continue;
    const double robot_distance = (robot.head<2>() - c).norm();
    const bool at_an_end = t <= kEps || t >= 1.0 - kEps;
    if (robot_distance < reach_ && at_an_end &&
        nearest >= robot_distance - 1e-6) {
      continue;  // only ever driven outward, on the robot's way out
    }
    return true;
  }
  return false;
}

}  // namespace mgg

namespace mgg {

std::vector<Vertex*> zoneRespectingRoute(GraphManager& graph, int source_id,
                                         int target_id,
                                         const Eigen::Vector3d& start,
                                         const NoGoZones& zones) {
  const auto vertex = [&graph](int id) -> Vertex* {
    const auto it = graph.vertices_map_.find(id);
    return it == graph.vertices_map_.end() ? nullptr : it->second;
  };
  Vertex* source = vertex(source_id);
  Vertex* target = vertex(target_id);
  if (source == nullptr || target == nullptr || zones.inside(target->state.head<3>())) {
    return {};
  }
  std::uint64_t first = zones.departing(start);
  if ((source->state.head<3>() - start).norm() > 1e-9 &&
      !zones.step(start, source->state.head<3>(), first)) {
    return {};
  }
  struct State {
    int at;
    std::uint64_t leaving;
    double cost;
    int parent;  // index into states, -1 at the source
  };
  std::vector<State> states{{source_id, first, 0.0, -1}};
  std::map<std::pair<int, std::uint64_t>, std::size_t> index{
      {{source_id, first}, 0}};
  std::vector<bool> settled{false};
  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  open.push({0.0, 0});
  while (!open.empty()) {
    const auto [cost, si] = open.top();
    open.pop();
    if (settled[si] || cost > states[si].cost) continue;
    settled[si] = true;
    const State s = states[si];
    if (s.at == target_id) {
      std::vector<Vertex*> route;
      for (int k = static_cast<int>(si); k >= 0; k = states[k].parent) {
        route.push_back(vertex(states[k].at));
      }
      std::reverse(route.begin(), route.end());
      return route;
    }
    const Vertex* u = vertex(s.at);
    const auto edges = graph.edge_map_.find(s.at);
    if (u == nullptr || edges == graph.edge_map_.end()) continue;
    for (const auto& [w, weight] : edges->second) {
      Vertex* v = vertex(w);
      if (v == nullptr || !graph.inService(*v) ||
          !graph.graph_->edgeExists(s.at, w)) {
        continue;
      }
      std::uint64_t leaving = s.leaving;
      if (!zones.step(u->state.head<3>(), v->state.head<3>(), leaving)) {
        continue;
      }
      const double next_cost = cost + weight;
      const auto [it, added] =
          index.emplace(std::make_pair(w, leaving), states.size());
      if (added) {
        states.push_back({w, leaving, next_cost, static_cast<int>(si)});
        settled.push_back(false);
      } else if (next_cost < states[it->second].cost) {
        states[it->second].cost = next_cost;
        states[it->second].parent = static_cast<int>(si);
      } else {
        continue;
      }
      open.push({next_cost, it->second});
    }
  }
  return {};
}

}  // namespace mgg
