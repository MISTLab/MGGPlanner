#include "mgg_core/planning_cancellation.h"
#include "mgg_core/path_turns.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <climits>
#include <cstdint>
#include <functional>
#include <queue>
#include <utility>

namespace mgg {
namespace {

constexpr double kMinMove = 1e-9;

/// How far down turnSpaceObserved looks for the ground under a standing
/// start: GroundProjection's max_projection_length.
constexpr double kStandingGroundDepth = 5.0;

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
    planningCheckpoint();
    along[i] = along[i - 1] + (points[i] - points[i - 1]).head<2>().norm();
  }
  const auto far_enough = [&](std::size_t a, std::size_t b) {
    const double d = along[b] - along[a];
    return d >= window && d > kMinMove;
  };
  for (std::size_t i = 0; i + 1 < n; ++i) {
    planningCheckpoint();
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

namespace {

/// Slope of the least-squares plane through `points`, radians;
/// kUnknownSlopeRad when fewer than three of them span a plane.
double planeSlope(const std::vector<Eigen::Vector3d>& points) {
  if (points.size() < 3) return kUnknownSlopeRad;
  // z = a x + b y + c.
  Eigen::MatrixXd a(points.size(), 3);
  Eigen::VectorXd z(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    planningCheckpoint();
    a.row(i) << points[i].x(), points[i].y(), 1.0;
    z(i) = points[i].z();
  }
  const auto qr = a.colPivHouseholderQr();
  if (qr.rank() < 3) return kUnknownSlopeRad;
  const Eigen::Vector3d plane = qr.solve(z);
  return std::atan(std::hypot(plane.x(), plane.y()));
}

}  // namespace

double terrainSlope(GraphManager& graph, const Vertex& vertex, double radius) {
  std::vector<Vertex*> nearby;
  StateVec state = vertex.state;
  if (!graph.getNearestVertices(&state, radius, &nearby)) {
    return kUnknownSlopeRad;
  }
  std::vector<Eigen::Vector3d> ground;
  for (const Vertex* v : nearby) {
    planningCheckpoint();
    if (v != nullptr && !v->is_hanging) {
      ground.push_back(v->state.head<3>() - vertex.state.head<3>());
    }
  }
  // About the vertex.
  return planeSlope(ground);
}

double groundSlope(const GroundProjection& ground,
                   const Eigen::Vector3d& position, double radius,
                   GraphManager* lattice) {
  std::vector<Eigen::Vector3d> points;
  Eigen::Vector3d center;
  if (ground.groundBelow(position, center)) {
    points.push_back(center - position);
    // The inner ring retains a non-collinear fit on a real ramp when the
    // outer samples differ by more than a step. Wall/kerb tops are not floor.
    for (double scale : {1.0, 0.5}) {
      planningCheckpoint();
      for (int k = 0; k < 8; ++k) {
        planningCheckpoint();
        const double angle = k * M_PI / 4.0;
        Eigen::Vector3d probe = position;
        probe.x() += scale * radius * std::cos(angle);
        probe.y() += scale * radius * std::sin(angle);
        Eigen::Vector3d found;
        if (ground.groundBelow(probe, found) &&
            (ground.maxStepHeight() <= 0.0 ||
             std::abs(found.z() - center.z()) <= ground.maxStepHeight())) {
          points.push_back(found - position);
        }
      }
    }
  }
  double slope = planeSlope(points);
  if (lattice != nullptr) {
    StateVec probe(position.x(), position.y(), position.z(), 0);
    std::vector<Vertex*> nearby;
    std::vector<Eigen::Vector3d> samples;
    lattice->getNearestVertices(&probe, radius, &nearby);
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto* v : nearby) {
      planningCheckpoint();
      if (v == nullptr || v->is_hanging) continue;
      samples.push_back(v->state.head<3>() - position);
      mean += samples.back().head<2>();
    }
    // A sparse or thin strip can alias a real ramp as level. Only a
    // neighbourhood spanning the fit radius in BOTH horizontal principal
    // axes, with at least six measurements, may reduce the map slope.
    if (samples.size() >= 6) {
      mean /= samples.size();
      Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
      for (const auto& sample : samples) {
        planningCheckpoint();
        const Eigen::Vector2d p = sample.head<2>() - mean;
        covariance += p * p.transpose();
      }
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> axes(covariance);
      if (axes.info() == Eigen::Success) {
        Eigen::Vector2d low = Eigen::Vector2d::Constant(INFINITY);
        Eigen::Vector2d high = Eigen::Vector2d::Constant(-INFINITY);
        for (const auto& sample : samples) {
          planningCheckpoint();
          const Eigen::Vector2d p = axes.eigenvectors().transpose() *
                                    (sample.head<2>() - mean);
          low = low.cwiseMin(p);
          high = high.cwiseMax(p);
        }
        if ((high - low).minCoeff() + 1e-9 >= radius) {
          slope = std::min(slope, planeSlope(samples));
        }
      }
    }
  }
  return slope;
}

TurnCompliantRoutes findTurnCompliantRoutes(
    GraphManager& graph, double start_heading, double window,
    const std::vector<int>& destinations,
    const SharpTurnAllowedFn& sharp_turn_allowed, int max_states,
    int start_id) {
  const GraphManager::EdgeValidationScope validation(graph);
  TurnCompliantRoutes out;
  const auto vertex = [&graph](int id) -> Vertex* {
    const auto it = graph.vertices_map_.find(id);
    return it == graph.vertices_map_.end() ? nullptr : it->second;
  };
  if (vertex(start_id) == nullptr || destinations.empty()) return out;

  struct State {
    int at;
    int from;  // -1 at the start
    double cost;
    int parent;  // index into states, -1 at the start
  };
  std::vector<State> states = {{start_id, -1, 0.0, -1}};
  std::vector<bool> settled = {false};
  const auto key = [](int at, int from) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(at)) << 32) |
           static_cast<std::uint32_t>(from);
  };
  std::unordered_map<std::uint64_t, int> index = {{key(start_id, -1), 0}};
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
    planningCheckpoint();
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
      planningCheckpoint();
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
    // vertex at least `window` back, or the start when the route is shorter,
    // and at the start the robot's heading.
    const auto heading_into = [&](std::size_t k) {
      for (std::size_t j = k + 1; j < route.size(); ++j) {
        planningCheckpoint();
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
      planningCheckpoint();
      if (w == s.from) continue;
      const Vertex* next = vertex(w);
      if (next == nullptr) continue;
      // An edge the graph closes to its searches (a no-go zone) is closed
      // to this one too (review r0, I-2).
      if (graph.edgeBlocked(*u, *next)) continue;
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
    planningCheckpoint();
    const auto found = arrival.find(id);
    if (found == arrival.end()) continue;
    TurnCompliantRoutes::Route route;
    for (int si = found->second; si >= 0; si = states[si].parent) {
      planningCheckpoint();
      route.path.push_back(vertex(states[si].at));
      route.along.push_back(states[si].cost);
    }
    std::reverse(route.path.begin(), route.path.end());
    std::reverse(route.along.begin(), route.along.end());
    out.to.emplace(id, std::move(route));
  }
  return out;
}

namespace {
Eigen::Vector3d turnCenter(const RobotParams& robot, const StateVec& state) {
  Eigen::Vector3d center = state.head<3>();
  if (std::isfinite(state[3]))
    center.head<2>() += robot.physicalOffsetForHeading(state[3]).head<2>();
  // The physical Z band is evidence only; collision checks retain the full
  // planning band, independently of the chassis' physical vertical centre.
  center.z() += robot.center_offset.z();
  return center;
}
double turnRadius(const RobotParams& robot, const StateVec& state) {
  return robot.turningRadius() + (std::isfinite(state[3]) ? 0.0 :
      robot.physicalOffsetForHeading(0).head<2>().norm());
}
}  // namespace

bool turnTransitionClear(const MapInterface& map, const RobotParams& robot,
                         const StateVec& incoming, const StateVec& outgoing) {
  return map.getOccupiedOnlyCylinderPathStatus(turnCenter(robot,incoming),
      turnCenter(robot,outgoing), robot.turningRadius(),
      robot.getPlanningSize().z()) != VoxelStatus::kOccupied;
}

bool turnClear(const MapInterface& map, const RobotParams& robot,
               const StateVec& state) {
  const double radius = turnRadius(robot, state);
  const Eigen::Vector3d center = turnCenter(robot, state);
  return map.getOccupiedOnlyCylinderPathStatus(
             center, center, radius, robot.getPlanningSize().z()) !=
         VoxelStatus::kOccupied;
}

bool turnSpaceObserved(const MapInterface& map, const RobotParams& robot,
                       const PlanningParams& planning, const StateVec& state,
                       const StandingStart* standing,
                       StandingTurnBody standing_body) {
  const double min_ground = planning.min_observed_ground_fraction;
  if (!(min_ground > 0.0)) return true;
  const Eigen::Vector3d center = turnCenter(robot, state);
  std::vector<XYCellCenter> cells;
  constexpr std::size_t kMaxTurnCells = 1024;
  if (!center.allFinite() ||
      !map.getCircleIntersectingXYCellCenters(
          center.head<2>(), turnRadius(robot, state), kMaxTurnCells, cells)) {
    return false;
  }
  const double resolution = map.getResolution();
  const double height = robot.getPlanningSize().z();
  const double lowest = center.z() - 2.0 * planning.max_ground_height;
  int observed_ground = 0;
  for (const XYCellCenter& cell : cells) {
    planningCheckpoint();
    // The ground the robot stands on at its start, which its lidar has not
    // seen: observed unless it was seen to fall away.
    const bool standing_on =
        standing != nullptr && standing->covers(cell.center);
    bool observed =
        standing_on &&
        (standing_body == StandingTurnBody::kCellCentreInDisk ||
         standing->coversCell(cell.center, resolution));
    for (double z = center.z() - 0.5 * height;
         z <= center.z() + 0.5 * height + 1e-9 && !observed;
         z += resolution) {
      observed = map.getVoxelStatus(Eigen::Vector3d(
                     cell.center.x(), cell.center.y(), z)) !=
                 VoxelStatus::kUnknown;
    }
    if (!observed) return false;
    // Under the standing start, as far down as ground projection looks
    // (GroundProjection::max_projection_length), so that ground seen far
    // below is a drop, not ground never seen.
    const double depth =
        standing_on ? kStandingGroundDepth : 2.0 * planning.max_ground_height;
    const Eigen::Vector3d from(cell.center.x(), cell.center.y(), center.z());
    Eigen::Vector3d ground;
    if (map.getGroundRayStatus(from, from - Eigen::Vector3d(0.0, 0.0, depth),
                               false, ground) == VoxelStatus::kOccupied) {
      if (ground.z() >= lowest) ++observed_ground;
    } else if (standing_on) {
      ++observed_ground;
    }
  }
  return observed_ground >= min_ground * static_cast<double>(cells.size()) - 1e-9;
}

bool roomToTurn(const MapInterface& map, const RobotParams& robot,
                const PlanningParams& planning, const StateVec& state,
                const StandingStart* standing,
                StandingTurnBody standing_body) {
  return turnClear(map, robot, state) &&
         turnSpaceObserved(map, robot, planning, state, standing,
                           standing_body);
}

bool observedArrivalDisk(const MapInterface& map, const RobotParams& robot,
                          const PlanningParams& planning, const StateVec& goal,
                          double arrival_tolerance) {
  if (!roomToTurn(map, robot, planning, goal, nullptr)) return false;
  const Eigen::Vector3d center = turnCenter(robot, goal);
  const double radius = turnRadius(robot, goal) + std::max(0.0, arrival_tolerance);
  std::vector<XYCellCenter> cells;
  if (!map.getCircleIntersectingXYCellCenters(center.head<2>(), radius, 4096, cells) ||
      cells.empty()) return false;
  if (map.getOccupiedOnlyCylinderPathStatus(center, center, radius,
      robot.getPlanningSize().z()) == VoxelStatus::kOccupied) return false;
  for (const auto& cell : cells) {
    planningCheckpoint();
    const Eigen::Vector3d from(cell.center.x(), cell.center.y(), center.z());
    Eigen::Vector3d ground;
    if (map.getGroundRayStatus(from,
            from - Eigen::Vector3d(0, 0, 2.0 * planning.max_ground_height),
            false, ground) != VoxelStatus::kOccupied) return false;
  }
  return true;
}

PathTurnCheck::PathTurnCheck(GraphManager& graph, const RobotParams& robot,
                             TurnRoomFn room_to_turn, SlopeFn slope, bool reverse,
                             TurnTransitionFn transition)
    : graph_(graph),
      window_(std::max(robot.size.x(), robot.size.y())),
      room_to_turn_(std::move(room_to_turn)),
      reverse_(reverse), transition_(std::move(transition)),
      slope_(std::move(slope)) {}

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
  double slope = 0.0;
  if (slope_) {
    slope = slope_(position);
  } else {
    const Vertex probe(-1,
                       StateVec(position.x(), position.y(), position.z(), 0));
    slope = terrainSlope(graph_, probe, window_);
  }
  if ((slope_ || slope >= kUnknownSlopeRad) &&
      robot_tilt_ && robot_tilt_->first == key &&
      robot_tilt_->second < 4.0 * M_PI / 180.0) {
    slope = robot_tilt_->second;
  } else if (slope >= kUnknownSlopeRad && unmeasured_) {
    slope = unmeasured_(position);
  }
  return slope_at_[key] = slope;
}

void PathTurnCheck::setRobotTilt(const Eigen::Vector3d& position,
                                 double tilt) {
  robot_tilt_.emplace(positionKey(position), std::abs(tilt));
  slope_at_.erase(robot_tilt_->first);
}

bool PathTurnCheck::roomAt(const Eigen::Vector3d& position, double heading) {
  if (!room_to_turn_) return true;
  const auto key = std::make_pair(positionKey(position),
      std::isfinite(heading) ? std::llround(std::remainder(heading, 2*M_PI)*1e9) : LLONG_MAX);
  const auto found = room_at_.find(key);
  if (found != room_at_.end()) return found->second;
  return room_at_[key] = room_to_turn_(
             StateVec(position.x(), position.y(), position.z(), heading));
}

PathTurnCheck::Refusal PathTurnCheck::firstRefusal(
    const std::vector<Eigen::Vector3d>& points, double start_heading, bool record) {
  const std::vector<double> turns = pathTurns(points, start_heading, window_);
  for (std::size_t i = 0; i < turns.size(); ++i) {
    planningCheckpoint();
    if (turns[i] <= kSharpTurnRad + 1e-9) continue;
    const double slope = slopeAt(points[i]);
    const bool on_slope = slope > kLevelGroundSlopeRad;
    // At rest the chassis centre is fixed. Interior corners can translate:
    // certify their incoming/outgoing circles and the cylinder between them.
    std::size_t back = i, ahead = i;
    double distance = 0;
    while (back > 0 && (back == i || distance < window_)) {
      distance += (points[back] - points[back-1]).head<2>().norm();
      --back;
    }
    distance = 0;
    while (ahead+1 < points.size() && (ahead == i || distance < window_)) {
      distance += (points[ahead+1] - points[ahead]).head<2>().norm();
      ++ahead;
    }
    const double incoming = (back == i ? start_heading : headingBetween(points[back], points[i])) + (reverse_ ? M_PI : 0);
    const double outgoing = (ahead == i ? incoming : headingBetween(points[i], points[ahead]) + (reverse_ ? M_PI : 0));
    const auto pose = [&](double yaw) { return StateVec(points[i].x(),points[i].y(),points[i].z(),yaw); };
    if (on_slope || !roomAt(points[i], incoming) ||
        (i > 0 && (!roomAt(points[i], outgoing) ||
          (transition_ && !transition_(pose(incoming),pose(outgoing)))))) {
      if (record && !first_refused_corner) {
        first_refused_corner = RefusedCorner{points[i], turns[i], slope, on_slope};
      }
      return on_slope ? Refusal::kSlope : Refusal::kRoom;
    }
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
  switch (firstRefusal(points, path.front()->state[3] + (reverse_ ? M_PI : 0), true)) {
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

RouteTurnChoice chooseTurnCompliantRoute(
    GraphManager& graph, PathTurnCheck& check,
    const std::vector<Eigen::Vector3d>& lead_in, double start_heading,
    std::vector<Vertex*>& route, int max_states) {
  RouteTurnChoice out;
  if (route.empty()) return out;
  const auto points_of = [&lead_in](const std::vector<Vertex*>& path) {
    std::vector<Eigen::Vector3d> points = lead_in;
    for (const Vertex* v : path) points.push_back(v->state.head<3>());
    return points;
  };
  if (check.admissible(points_of(route), start_heading)) return out;
  out.searched = true;
  // Onto the first vertex from the lead-in, as pathTurns measures it: from
  // the path's start.
  double heading = start_heading;
  const Eigen::Vector3d first = route.front()->state.head<3>();
  if (!lead_in.empty() &&
      (first - lead_in.front()).head<2>().norm() > kMinMove) {
    heading = headingBetween(lead_in.front(), first);
  }
  const int goal = route.back()->id;
  const TurnCompliantRoutes found = findTurnCompliantRoutes(
      graph, heading, check.window(), {goal},
      [&check](const Vertex& v) {
        return check.sharpTurnAllowedAt(v.state.head<3>());
      },
      max_states, route.front()->id);
  out.states_expanded = found.states_expanded;
  out.capped = found.capped;
  const auto detour = found.to.find(goal);
  if (detour != found.to.end() &&
      check.admissible(points_of(detour->second.path), start_heading)) {
    route = detour->second.path;
    out.detour = true;
  } else {
    out.fallback = true;
  }
  return out;
}

}  // namespace mgg
