#include "mgg_core/path_turns.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <queue>
#include <utility>

namespace mgg {
namespace {

constexpr double kMinMove = 1e-9;

double headingBetween(const Eigen::Vector3d& from, const Eigen::Vector3d& to) {
  return std::atan2(to.y() - from.y(), to.x() - from.x());
}

/// Absolute difference of two headings, in [0, pi].
double headingChange(double a, double b) {
  const double d = std::fmod(std::abs(a - b), 2.0 * M_PI);
  return d > M_PI ? 2.0 * M_PI - d : d;
}

}  // namespace

std::vector<double> pathTurns(const std::vector<Eigen::Vector3d>& points,
                              double start_heading, double window) {
  const std::size_t n = points.size();
  std::vector<double> turns(n, 0.0);
  if (n < 2) return turns;
  // Planar distance along the path to each point.
  std::vector<double> along(n, 0.0);
  for (std::size_t i = 1; i < n; ++i) {
    along[i] = along[i - 1] + (points[i] - points[i - 1]).head<2>().norm();
  }
  const auto far_enough = [&](std::size_t a, std::size_t b) {
    const double d = along[b] - along[a];
    return d >= window && d > kMinMove;
  };
  for (std::size_t i = 0; i + 1 < n; ++i) {
    std::size_t ahead = i + 1;
    while (ahead + 1 < n && !far_enough(i, ahead)) ++ahead;
    if (along[ahead] - along[i] <= kMinMove) continue;  // no way out: no turn
    double in = start_heading;
    if (i > 0) {
      std::size_t back = i - 1;
      while (back > 0 && !far_enough(back, i)) --back;
      if (along[i] - along[back] > kMinMove) {
        in = headingBetween(points[back], points[i]);
      }
    }
    turns[i] = headingChange(in, headingBetween(points[i], points[ahead]));
  }
  return turns;
}

double terrainSlope(GraphManager& graph, const Vertex& vertex, double radius) {
  std::vector<Vertex*> nearby;
  StateVec state = vertex.state;
  if (!graph.getNearestVertices(&state, radius, &nearby)) {
    return kUnknownSlopeRad;
  }
  std::vector<Eigen::Vector3d> ground;
  for (const Vertex* v : nearby) {
    if (v != nullptr && !v->is_hanging) {
      ground.push_back(v->state.head<3>() - vertex.state.head<3>());
    }
  }
  if (ground.size() < 3) return kUnknownSlopeRad;
  // z = a x + b y + c, about the vertex.
  Eigen::MatrixXd a(ground.size(), 3);
  Eigen::VectorXd z(ground.size());
  for (std::size_t i = 0; i < ground.size(); ++i) {
    a.row(i) << ground[i].x(), ground[i].y(), 1.0;
    z(i) = ground[i].z();
  }
  const auto qr = a.colPivHouseholderQr();
  if (qr.rank() < 3) return kUnknownSlopeRad;
  const Eigen::Vector3d plane = qr.solve(z);
  return std::atan(std::hypot(plane.x(), plane.y()));
}

TurnCompliantRoutes findTurnCompliantRoutes(
    GraphManager& graph, double start_heading, double window,
    const std::vector<int>& destinations,
    const SharpTurnAllowedFn& sharp_turn_allowed, int max_states) {
  TurnCompliantRoutes out;
  const auto vertex = [&graph](int id) -> Vertex* {
    const auto it = graph.vertices_map_.find(id);
    return it == graph.vertices_map_.end() ? nullptr : it->second;
  };
  if (vertex(0) == nullptr || destinations.empty()) return out;

  struct State {
    int at;
    int from;  // -1 at vertex 0
    double cost;
    int parent;  // index into states, -1 at vertex 0
  };
  std::vector<State> states = {{0, -1, 0.0, -1}};
  std::vector<bool> settled = {false};
  const auto key = [](int at, int from) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(at)) << 32) |
           static_cast<std::uint32_t>(from);
  };
  std::unordered_map<std::uint64_t, int> index = {{key(0, -1), 0}};
  // Destination -> the cheapest state that arrives there with every turn
  // allowed.
  std::unordered_map<int, int> arrival;
  std::unordered_map<int, bool> allowed;
  const auto allowed_at = [&](const Vertex* at) {
    auto found = allowed.find(at->id);
    if (found == allowed.end()) {
      found = allowed.emplace(at->id, sharp_turn_allowed(*at)).first;
    }
    return found->second;
  };
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  open.push({0.0, 0});

  std::vector<const Vertex*> route;
  std::vector<double> back;
  std::size_t destinations_left = destinations.size();
  std::unordered_map<int, bool> wanted;
  for (int id : destinations) wanted.emplace(id, true);

  while (!open.empty() && destinations_left > 0) {
    const auto [cost, si] = open.top();
    open.pop();
    if (settled[si] || cost > states[si].cost) continue;
    if (out.states_expanded >= max_states) {
      out.capped = true;
      break;
    }
    settled[si] = true;
    ++out.states_expanded;
    const State s = states[si];
    const Vertex* u = vertex(s.at);
    if (u == nullptr) continue;
    // The route back from `u`, with the planar distance back to each of
    // its vertices.
    route.clear();
    back.clear();
    for (int p = si; p >= 0; p = states[p].parent) {
      const Vertex* v = vertex(states[p].at);
      if (v == nullptr) break;
      back.push_back(route.empty() ? 0.0
                                   : back.back() + (route.back()->state -
                                                    v->state)
                                                       .head<2>()
                                                       .norm());
      route.push_back(v);
    }
    const auto far = [window](double d) {
      return d >= window && d > kMinMove;
    };
    // The heading into route[k] as pathTurns measures it: from the first
    // vertex at least `window` back, or vertex 0 when the route is shorter,
    // and at vertex 0 the robot's heading.
    const auto heading_into = [&](std::size_t k) {
      for (std::size_t j = k + 1; j < route.size(); ++j) {
        if (far(back[j] - back[k]) ||
            (j + 1 == route.size() && back[j] - back[k] > kMinMove)) {
          return headingBetween(route[j]->state.head<3>(),
                                route[k]->state.head<3>());
        }
      }
      return start_heading;
    };

    // Arriving: the turns at the vertices less than `window` back, whose
    // heading out is towards the route's end, are settled only now. An
    // arrival that turns sharply where it may not is not a route there;
    // the search goes on for another.
    if (wanted.count(s.at) > 0 && arrival.count(s.at) == 0) {
      bool refused = false;
      for (std::size_t k = 1; k < route.size() && !far(back[k]) && !refused;
           ++k) {
        if (back[k] <= kMinMove) continue;
        const double out =
            headingBetween(route[k]->state.head<3>(), u->state.head<3>());
        refused = headingChange(heading_into(k), out) > kSharpTurnRad + 1e-9 &&
                  !allowed_at(route[k]);
      }
      if (refused) {
        ++out.arrivals_refused;
      } else {
        arrival.emplace(s.at, si);
        --destinations_left;
      }
    }

    const auto edges = graph.edge_map_.find(s.at);
    if (edges == graph.edge_map_.end()) continue;
    for (const auto& [w, weight] : edges->second) {
      if (w == s.from) continue;
      const Vertex* next = vertex(w);
      if (next == nullptr) continue;
      // A route passes each vertex once: it is sent as a path, and walked
      // back through parents.
      if (std::find(route.begin(), route.end(), next) != route.end()) {
        continue;
      }
      // Every vertex of the route whose turn `next` now settles, being the
      // first vertex at least `window` ahead of it, turns sharply only where
      // it may.
      const double step = (next->state - u->state).head<2>().norm();
      bool refused = false;
      for (std::size_t k = 0; k < route.size() && !far(back[k]) && !refused;
           ++k) {
        if (!far(back[k] + step)) continue;
        const Vertex* at = route[k];
        const double out =
            headingBetween(at->state.head<3>(), next->state.head<3>());
        if (headingChange(heading_into(k), out) <= kSharpTurnRad + 1e-9) {
          continue;
        }
        refused = !allowed_at(at);
      }
      if (refused) continue;
      const double next_cost = cost + weight;
      const auto [it, added] = index.emplace(key(w, s.at), states.size());
      if (added) {
        states.push_back({w, s.at, next_cost, si});
        settled.push_back(false);
      } else if (next_cost < states[it->second].cost &&
                 !settled[it->second]) {
        states[it->second].cost = next_cost;
        states[it->second].parent = si;
      } else {
        continue;
      }
      open.push({next_cost, it->second});
    }
  }

  for (int id : destinations) {
    const auto found = arrival.find(id);
    if (found == arrival.end()) continue;
    TurnCompliantRoutes::Route route;
    for (int si = found->second; si >= 0; si = states[si].parent) {
      route.path.push_back(vertex(states[si].at));
      route.along.push_back(states[si].cost);
    }
    std::reverse(route.path.begin(), route.path.end());
    std::reverse(route.along.begin(), route.along.end());
    out.to.emplace(id, std::move(route));
  }
  return out;
}

bool turnClear(const MapInterface& map, const RobotParams& robot,
               const StateVec& state) {
  const double radius = 0.5 * robot.size.head<2>().norm();
  const Eigen::Vector3d center = state.head<3>() + robot.center_offset;
  return map.getOccupiedOnlyCylinderPathStatus(
             center, center, radius, robot.getPlanningSize().z()) !=
         VoxelStatus::kOccupied;
}

PathTurnCheck::PathTurnCheck(GraphManager& graph, const RobotParams& robot,
                             TurnRoomFn room_to_turn)
    : graph_(graph),
      window_(std::max(robot.size.x(), robot.size.y())),
      room_to_turn_(std::move(room_to_turn)) {}

namespace {

/// Millimetres: a path's points are graph vertices or lie on its edges, so
/// equal positions are equal to far better than that.
std::array<long long, 3> positionKey(const Eigen::Vector3d& p) {
  return {std::llround(p.x() * 1000.0), std::llround(p.y() * 1000.0),
          std::llround(p.z() * 1000.0)};
}

}  // namespace

double PathTurnCheck::slopeAt(const Eigen::Vector3d& position) {
  const PositionKey key = positionKey(position);
  const auto found = slope_at_.find(key);
  if (found != slope_at_.end()) return found->second;
  const Vertex probe(-1, StateVec(position.x(), position.y(), position.z(), 0));
  return slope_at_[key] = terrainSlope(graph_, probe, window_);
}

bool PathTurnCheck::roomAt(const Eigen::Vector3d& position) {
  if (!room_to_turn_) return true;
  const PositionKey key = positionKey(position);
  const auto found = room_at_.find(key);
  if (found != room_at_.end()) return found->second;
  return room_at_[key] = room_to_turn_(
             StateVec(position.x(), position.y(), position.z(), 0.0));
}

PathTurnCheck::Refusal PathTurnCheck::firstRefusal(
    const std::vector<Eigen::Vector3d>& points, double start_heading) {
  const std::vector<double> turns = pathTurns(points, start_heading, window_);
  for (std::size_t i = 0; i < turns.size(); ++i) {
    if (turns[i] <= kSharpTurnRad + 1e-9) continue;
    if (slopeAt(points[i]) > kLevelGroundSlopeRad) return Refusal::kSlope;
    if (!roomAt(points[i])) return Refusal::kRoom;
  }
  return Refusal::kNone;
}

bool PathTurnCheck::sharpTurnAllowedAt(const Eigen::Vector3d& position) {
  return slopeAt(position) <= kLevelGroundSlopeRad && roomAt(position);
}

bool PathTurnCheck::admissible(const std::vector<Eigen::Vector3d>& points,
                               double start_heading) {
  return firstRefusal(points, start_heading) == Refusal::kNone;
}

bool PathTurnCheck::operator()(const std::vector<Vertex*>& path) {
  if (path.size() < 2) return true;
  std::vector<Eigen::Vector3d> points;
  points.reserve(path.size());
  for (const Vertex* v : path) points.push_back(v->state.head<3>());
  switch (firstRefusal(points, path.front()->state[3])) {
    case Refusal::kSlope:
      ++refused_on_slope;
      return false;
    case Refusal::kRoom:
      ++refused_without_room;
      return false;
    case Refusal::kNone:
      break;
  }
  return true;
}

}  // namespace mgg
