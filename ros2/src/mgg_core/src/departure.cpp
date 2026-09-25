#include "mgg_core/departure.h"

#include <algorithm>
#include <cmath>

#include "mgg_core/path_turns.h"

namespace mgg {

namespace {

/// Most map cells one step of the sweep considers: a Bunker's box and a
/// step span about 60 of 0.2 m.
constexpr std::size_t kMaxSweepCells = 4096;

/// `state` moved down onto the ground below it at driving height, as the
/// planner projects its root; false when no ground is mapped there.
bool toDrivingHeight(const GroundProjection& ground,
                     const PlanningParams& planning, StateVec& state) {
  Eigen::Vector3d position = state.head<3>();
  VoxelStatus status = VoxelStatus::kUnknown;
  const double below = ground.projectSample(position, status);
  if (status != VoxelStatus::kOccupied) return false;
  state[2] = position.z() - (below - planning.max_ground_height);
  return true;
}

}  // namespace

bool cellMeetsBox(const Eigen::Vector2d& cell_center, double resolution,
                  const OrientedBox& box, double touch) {
  const Eigen::Vector2d along(std::cos(box.heading), std::sin(box.heading));
  const Eigen::Vector2d across(-along.y(), along.x());
  const double half_length = 0.5 * box.size.x();
  const double half_width = 0.5 * box.size.y();
  const double half_cell = 0.5 * resolution;
  const Eigen::Vector2d offset = cell_center - box.center.head<2>();
  // Separating axes: the box's two and the cell's two.
  const double reach_along =
      half_length + half_cell * (std::abs(along.x()) + std::abs(along.y()));
  const double reach_across =
      half_width + half_cell * (std::abs(across.x()) + std::abs(across.y()));
  const double reach_x = half_cell + half_length * std::abs(along.x()) +
                         half_width * std::abs(across.x());
  const double reach_y = half_cell + half_length * std::abs(along.y()) +
                         half_width * std::abs(across.y());
  return reach_along - std::abs(offset.dot(along)) > touch &&
         reach_across - std::abs(offset.dot(across)) > touch &&
         reach_x - std::abs(offset.x()) > touch &&
         reach_y - std::abs(offset.y()) > touch;
}

VoxelStatus orientedBoxPathStatus(const MapInterface& map,
                                  const Eigen::Vector3d& start,
                                  const Eigen::Vector3d& end,
                                  const OrientedBox& box,
                                  bool stop_at_unknown_voxel,
                                  const OrientedBox* standing) {
  const double resolution = map.getResolution();
  if (!start.allFinite() || !end.allFinite() || !box.size.allFinite() ||
      (box.size.array() < 0.0).any() || !std::isfinite(box.heading) ||
      !std::isfinite(resolution) || resolution <= 0.0) {
    return VoxelStatus::kUnknown;
  }
  const double length = (end - start).norm();
  const double steps_d = std::max(1.0, std::ceil(length / resolution));
  if (!std::isfinite(steps_d) || steps_d > double(kMaxSweepCells)) {
    return VoxelStatus::kUnknown;
  }
  const int steps = static_cast<int>(steps_d);
  const Eigen::Vector3d step = (end - start) / steps_d;
  const Eigen::Vector2d along(std::cos(box.heading), std::sin(box.heading));
  const Eigen::Vector2d across(-along.y(), along.x());
  OrientedBox swept = box;
  swept.size += Eigen::Vector3d(std::abs(step.head<2>().dot(along)),
                                std::abs(step.head<2>().dot(across)),
                                std::abs(step.z()));
  const double radius = 0.5 * swept.size.head<2>().norm();
  bool unknown = false;
  std::vector<XYCellCenter> cells;
  for (int i = 0; i < steps; ++i) {
    swept.center = start + (i + 0.5) * step;
    if (!map.getCircleIntersectingXYCellCenters(
            swept.center.head<2>(), radius, kMaxSweepCells, cells)) {
      return VoxelStatus::kUnknown;
    }
    for (const XYCellCenter& cell : cells) {
      if (!cellMeetsBox(cell.center, resolution, swept, -1e-9)) continue;
      if (standing != nullptr &&
          cellMeetsBox(cell.center, resolution, *standing, 1e-6)) {
        continue;
      }
      const VoxelStatus status = map.getBoxStatus(
          Eigen::Vector3d(cell.center.x(), cell.center.y(), swept.center.z()),
          Eigen::Vector3d(0.0, 0.0, swept.size.z()), stop_at_unknown_voxel);
      if (status == VoxelStatus::kOccupied) return status;
      if (status == VoxelStatus::kUnknown) unknown = true;
    }
  }
  return unknown && stop_at_unknown_voxel ? VoxelStatus::kUnknown
                                          : VoxelStatus::kFree;
}

bool findDeparture(const MapInterface& map, const GroundProjection& ground,
                   const RobotParams& robot, const PlanningParams& planning,
                   const StateVec& start, Departure& departure) {
  departure = Departure();
  const bool ground_robot = robot.type == RobotType::kGroundRobot;
  const double spacing = planning.path_interpolation_distance;
  const double step =
      spacing > 0.0 ? std::min(spacing, kDepartureMinM) : map.getResolution();
  const int steps = static_cast<int>(std::ceil(kDepartureMaxM / step - 1e-9));
  OrientedBox standing;
  standing.center = start.head<3>();
  standing.heading = start[3];
  standing.size = robot.getPlanningSize();

  // Straight out at `heading`, ahead then back; fills departure.path.
  const auto straight = [&](double heading) {
    OrientedBox body = standing;
    body.heading = heading;
    EdgeBodyCheck check;
    check.sweep = [&map, &body, &standing](const Eigen::Vector3d& from,
                                           const Eigen::Vector3d& to) {
      return orientedBoxPathStatus(map, from, to, body, true, &standing);
    };
    const auto step_free = [&](const StateVec& from, const StateVec& to,
                               bool from_start) {
      if (!ground_robot) {
        return check.sweep(from.head<3>(), to.head<3>()) ==
               VoxelStatus::kFree;
      }
      check.standing_at_start = from_start;
      std::vector<Eigen::Vector3d> projected;
      return ground.getProjectedEdgeStatus(from.head<3>(), to.head<3>(),
                                           body.size, true, projected, false,
                                           false, &check) ==
             ProjectedEdgeStatus::kAdmissible;
    };
    for (const bool backwards : {false, true}) {
      if (backwards && !planning.departure_reverse_allowed) break;
      const double direction = backwards ? heading + M_PI : heading;
      const Eigen::Vector2d unit(std::cos(direction), std::sin(direction));
      StateVec here = start;
      here[3] = heading;
      departure.path.assign(1, here);
      for (int i = 1; i <= steps; ++i) {
        const double out = std::min(i * step, kDepartureMaxM);
        const Eigen::Vector2d xy = start.head<2>() + out * unit;
        StateVec to(xy.x(), xy.y(), departure.path.back().z(), heading);
        if (ground_robot && !toDrivingHeight(ground, planning, to)) break;
        // On the line, at the height of the ground found beside it if that
        // is where projectSample found it.
        to[0] = xy.x();
        to[1] = xy.y();
        to[3] = heading;
        if (!step_free(departure.path.back(), to, i == 1)) break;
        departure.path.push_back(to);
        if (out >= kDepartureMinM - 1e-9 && turnClear(map, robot, to)) {
          departure.reverse = backwards;
          return true;
        }
      }
    }
    departure.path.clear();
    return false;
  };

  if (straight(start[3])) return true;
  // Turned in place first, nearest first; a way blocked at some turn stays
  // blocked beyond it.
  const int turns =
      static_cast<int>(std::round(kDepartureMaxTurnRad / kDepartureTurnStepRad));
  bool blocked[2] = {false, false};
  for (int k = 1; k <= turns; ++k) {
    for (int side = 0; side < 2; ++side) {
      if (blocked[side]) continue;
      const double turn = (side == 0 ? 1.0 : -1.0) * k * kDepartureTurnStepRad;
      OrientedBox turned = standing;
      turned.heading = start[3] + turn;
      if (orientedBoxPathStatus(map, start.head<3>(), start.head<3>(), turned,
                                true, &standing) != VoxelStatus::kFree) {
        blocked[side] = true;
        continue;
      }
      if (straight(turned.heading)) {
        departure.turn = turn;
        return true;
      }
    }
  }
  return false;
}

}  // namespace mgg
