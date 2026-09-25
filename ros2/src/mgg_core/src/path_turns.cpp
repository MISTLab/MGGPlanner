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

double PathTurnCheck::slopeAt(const Vertex& vertex) {
  const auto found = slope_by_id_.find(vertex.id);
  if (found != slope_by_id_.end()) return found->second;
  return slope_by_id_[vertex.id] = terrainSlope(graph_, vertex, window_);
}

bool PathTurnCheck::roomAt(const Vertex& vertex) {
  if (!room_to_turn_) return true;
  const auto found = room_by_id_.find(vertex.id);
  if (found != room_by_id_.end()) return found->second;
  return room_by_id_[vertex.id] = room_to_turn_(vertex);
}

bool PathTurnCheck::operator()(const std::vector<Vertex*>& path) {
  if (path.size() < 2) return true;
  std::vector<Eigen::Vector3d> points;
  points.reserve(path.size());
  for (const Vertex* v : path) points.push_back(v->state.head<3>());
  const std::vector<double> turns =
      pathTurns(points, path.front()->state[3], window_);
  for (std::size_t i = 0; i < turns.size(); ++i) {
    if (turns[i] <= kSharpTurnRad + 1e-9) continue;
    if (slopeAt(*path[i]) > kLevelGroundSlopeRad) {
      ++refused_on_slope;
      return false;
    }
    if (!roomAt(*path[i])) {
      ++refused_without_room;
      return false;
    }
  }
  return true;
}

}  // namespace mgg
