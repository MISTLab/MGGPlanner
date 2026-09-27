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

NoGoZones::Departing NoGoZones::departing(const Eigen::Vector3d& p) const {
  Departing zones;
  for (std::size_t z = 0; z < centres_.size(); ++z) {
    if ((p.head<2>() - centres_[z]).norm() < reach_) {
      zones.push_back(static_cast<int>(z));
    }
  }
  return zones;
}

bool NoGoZones::step(const Eigen::Vector3d& a3, const Eigen::Vector3d& b3,
                     Departing& departing) const {
  const Eigen::Vector2d a = a3.head<2>();
  const Eigen::Vector2d b = b3.head<2>();
  // The zones being departed, checked by their own rule, and those still
  // departed at b.
  Departing next;
  next.reserve(departing.size());
  for (const int z : departing) {
    const Eigen::Vector2d& c = centres_[static_cast<std::size_t>(z)];
    // Monotonic outward: the segment is nearest the centre at its start.
    if (nearestFraction(a, b, c) > kEps &&
        nearestDistance(a, b, c) < (a - c).norm() - kEps) {
      return false;
    }
    if ((b - c).norm() < reach_) next.push_back(z);
  }
  // Every other zone is blocked.
  auto leaving = departing.begin();
  for (std::size_t z = 0; z < centres_.size(); ++z) {
    while (leaving != departing.end() && *leaving < static_cast<int>(z)) {
      ++leaving;
    }
    if (leaving != departing.end() && *leaving == static_cast<int>(z)) continue;
    if (nearestDistance(a, b, centres_[z]) < reach_) return false;
  }
  departing.swap(next);
  return true;
}

bool NoGoZones::pathAdmissible(const std::vector<Eigen::Vector3d>& path) const {
  if (centres_.empty() || path.empty()) return true;
  Departing leaving = departing(path.front());
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
  NoGoZones::Departing first = zones.departing(start);
  if ((source->state.head<3>() - start).norm() > 1e-9 &&
      !zones.step(start, source->state.head<3>(), first)) {
    return {};
  }
  struct State {
    int at;
    NoGoZones::Departing leaving;
    double cost;
    int parent;  // index into states, -1 at the source
  };
  std::vector<State> states{{source_id, first, 0.0, -1}};
  std::map<std::pair<int, NoGoZones::Departing>, std::size_t> index{
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
    const State s = states[si];  // a copy: states grows below
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
      NoGoZones::Departing leaving = s.leaving;
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
