#include "mgg_core/path_turns.h"

#include <algorithm>
#include <cmath>
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
  if (!graph.getNearestVertices(&state, radius, &nearby)) return 0.0;
  std::vector<Eigen::Vector3d> ground;
  for (const Vertex* v : nearby) {
    if (v != nullptr && !v->is_hanging) {
      ground.push_back(v->state.head<3>() - vertex.state.head<3>());
    }
  }
  if (ground.size() < 3) return 0.0;
  // z = a x + b y + c, about the vertex.
  Eigen::MatrixXd a(ground.size(), 3);
  Eigen::VectorXd z(ground.size());
  for (std::size_t i = 0; i < ground.size(); ++i) {
    a.row(i) << ground[i].x(), ground[i].y(), 1.0;
    z(i) = ground[i].z();
  }
  const auto qr = a.colPivHouseholderQr();
  if (qr.rank() < 3) return 0.0;
  const Eigen::Vector3d plane = qr.solve(z);
  return std::atan(std::hypot(plane.x(), plane.y()));
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
