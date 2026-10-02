#include "mgg_core/planning_cancellation.h"
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

bool pointInBox(const Eigen::Vector2d& point, const OrientedBox& box) {
  const Eigen::Vector2d along(std::cos(box.heading), std::sin(box.heading));
  const Eigen::Vector2d across(-along.y(), along.x());
  const Eigen::Vector2d offset = point - box.center.head<2>();
  return std::abs(offset.dot(along)) <= 0.5 * box.size.x() + 1e-9 &&
         std::abs(offset.dot(across)) <= 0.5 * box.size.y() + 1e-9;
}

namespace {

/// Whether the body `swept` reaches into a map cell's square anywhere the
/// robot does not already stand (`standing`). A cell whose centre lies
/// inside the standing body is the robot's own place and never is. For a
/// cell that only reaches into it, the square is sampled every
/// resolution / 8, edges included, for a point inside `swept` and outside
/// `standing`: a post just past the robot's front is met driving ahead but
/// not backing away, and a wall column beside it is not met driving along
/// it (review r0, I-1). A cell clear of the standing body always is.
bool entersBeyondStanding(const Eigen::Vector2d& cell_center,
                          double resolution, const OrientedBox& swept,
                          const OrientedBox& standing) {
  if (pointInBox(cell_center, standing)) return false;
  if (!cellMeetsBox(cell_center, resolution, standing, 1e-6)) return true;
  constexpr int kSamples = 8;
  const double half = 0.5 * resolution;
  for (int i = 0; i <= kSamples; ++i) {
    planningCheckpoint();
    for (int j = 0; j <= kSamples; ++j) {
      planningCheckpoint();
      const Eigen::Vector2d point =
          cell_center + Eigen::Vector2d(-half + resolution * i / kSamples,
                                        -half + resolution * j / kSamples);
      if (pointInBox(point, swept) && !pointInBox(point, standing)) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

VoxelStatus orientedBoxPathStatus(const MapInterface& map,
                                  const Eigen::Vector3d& start,
                                  const Eigen::Vector3d& end,
                                  const OrientedBox& box,
                                  bool stop_at_unknown_voxel,
                                  const OrientedBox* standing,
                                  bool clearance_prefilter) {
  const double resolution = map.getResolution();
  if (!start.allFinite() || !end.allFinite() || !box.size.allFinite() ||
      (box.size.array() < 0.0).any() || !std::isfinite(box.heading) ||
      !std::isfinite(resolution) || resolution <= 0.0) {
    return VoxelStatus::kUnknown;
  }
  if (map.dynamicSweepBlocked(start, end,
                              0.5 * std::max(box.size.x(), box.size.y()))) {
    return VoxelStatus::kOccupied;
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
  if (clearance_prefilter && standing == nullptr) {
    // Contains every conservative step box below (not an inscribed disc).
    // Padding also contains boundary-touching native XY cells.
    const Eigen::Vector3d span = (end - start).cwiseAbs();
    Eigen::Vector3d bound(
        std::abs(along.x()) * swept.size.x() + std::abs(across.x()) * swept.size.y(),
        std::abs(along.y()) * swept.size.x() + std::abs(across.y()) * swept.size.y(),
        swept.size.z());
    bound += span + Eigen::Vector3d(2 * resolution, 2 * resolution, 0);
    if (map.getStaticBoxStatus((start + end) / 2, bound, stop_at_unknown_voxel) ==
        VoxelStatus::kFree) return VoxelStatus::kFree;
  }
  const double radius = 0.5 * swept.size.head<2>().norm();
  bool unknown = false;
  std::vector<XYCellCenter> cells;
  for (int i = 0; i < steps; ++i) {
    planningCheckpoint();
    swept.center = start + (i + 0.5) * step;
    if (!map.getCircleIntersectingXYCellCenters(
            swept.center.head<2>(), radius, kMaxSweepCells, cells)) {
      return VoxelStatus::kUnknown;
    }
    for (const XYCellCenter& cell : cells) {
      planningCheckpoint();
      if (!cellMeetsBox(cell.center, resolution, swept, -1e-9)) continue;
      if (standing != nullptr &&
          !entersBeyondStanding(cell.center, resolution, swept, *standing)) {
        continue;
      }
      const VoxelStatus status = map.getStaticBoxStatus(
          Eigen::Vector3d(cell.center.x(), cell.center.y(), swept.center.z()),
          Eigen::Vector3d(0.0, 0.0, swept.size.z()), stop_at_unknown_voxel);
      if (status == VoxelStatus::kOccupied) return status;
      if (status == VoxelStatus::kUnknown) unknown = true;
    }
  }
  return unknown ? VoxelStatus::kUnknown : VoxelStatus::kFree;
}

// Only the physical aerial root may contain unknown air. Check occupied
// volume over the whole departure, then check the swept volume outside the
// root AABB strictly. Splitting each swept interval into slabs preserves the
// unknown check beside the root, not just after the body has left it.
bool aerialRootDepartureTraversable(const MapInterface& map,
                                     const Eigen::Vector3d& start,
                                     const Eigen::Vector3d& end,
                                     const Eigen::Vector3d& size) {
  const auto occupied = map.getOccupiedOnlyPathStatus(start, end, size);
  if (occupied == VoxelStatus::kOccupied)
    return map.aerialRootRecoveryTraversable(start, end, size);
  if (occupied != VoxelStatus::kFree) return false;
  const double resolution = map.getResolution();
  const double steps_d = std::ceil((end - start).norm() / resolution);
  constexpr int kMaxRootSweepIntervals = 4096;
  if (!std::isfinite(steps_d) || resolution <= 0.0 ||
      steps_d > kMaxRootSweepIntervals)
    return false;
  const int steps = std::max(1, static_cast<int>(steps_d));
  const Eigen::Vector3d step = (end - start) / steps;
  const Eigen::Vector3d root_lo = start - size / 2;
  const Eigen::Vector3d root_hi = start + size / 2;
  for (int i = 0; i < steps; ++i) {
    planningCheckpoint();
    const Eigen::Vector3d center = start + (i + 0.5) * step;
    const Eigen::Vector3d half = (size + step.cwiseAbs()) / 2;
    const Eigen::Vector3d lo = center - half, hi = center + half;
    for (int axis = 0; axis < 3; ++axis) {
      planningCheckpoint();
      for (bool upper : {false, true}) {
        planningCheckpoint();
        Eigen::Vector3d slab_lo = lo, slab_hi = hi;
        if (upper) slab_lo[axis] = std::max(lo[axis], root_hi[axis]);
        else slab_hi[axis] = std::min(hi[axis], root_lo[axis]);
        if (slab_hi[axis] <= slab_lo[axis]) continue;
        if (map.getStaticStrictBoxStatus((slab_lo + slab_hi) / 2,
                                        slab_hi - slab_lo) !=
            VoxelStatus::kFree) return false;
      }
    }
  }
  return true;
}

bool reverseExitEdgeAdmissible(
    const MapInterface& map, const GroundProjection& ground,
    const RobotParams& robot, const PlanningParams& planning,
    const StateVec& from, const StateVec& to,
    const std::function<bool(const Eigen::Vector3d&, const Eigen::Vector3d&)>&
        segment_admissible) {
  if (!planning.departure_reverse_allowed || ground.standingStart() ||
      robot.type != RobotType::kGroundRobot) return false;
  const Eigen::Vector3d delta = to.head<3>() - from.head<3>();
  if (delta.z() < -std::max(map.getResolution(), planning.max_step_height) &&
      std::atan2(-delta.z(), delta.head<2>().norm()) > planning.max_negative_inclination) {
    return false;
  }
  OrientedBox body;
  // Signed motion is backwards: chassis heading stays that of entry.
  body.heading = std::atan2(from.y() - to.y(), from.x() - to.x());
  body.size = robot.getPlanningSize();
  EdgeBodyCheck check;
  check.sweep = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return orientedBoxPathStatus(map, a, b, body, false, nullptr);
  };
  std::vector<Eigen::Vector3d> projected;
  if (ground.getProjectedEdgeStatus(
          from.head<3>() + robot.offsetForHeading(body.heading),
          to.head<3>() + robot.offsetForHeading(body.heading),
          body.size, false, projected, false, false, &check,
          EdgeTravel::kForward) != ProjectedEdgeStatus::kAdmissible) return false;
  for (std::size_t i = 1; i < projected.size(); ++i) {
    planningCheckpoint();
    const Eigen::Vector3d delta = projected[i] - projected[i - 1];
    if (delta.z() < -planning.max_step_height - 1e-6 &&
        std::atan2(-delta.z(), delta.head<2>().norm()) >
            planning.max_negative_inclination) return false;
    if (segment_admissible && !segment_admissible(projected[i - 1], projected[i])) {
      return false;
    }
  }
  return true;
}

bool findDeparture(const MapInterface& map, const GroundProjection& ground,
                   const RobotParams& robot, const PlanningParams& planning,
                   const StateVec& start, Departure& departure,
                   const std::function<bool(const std::vector<StateVec>&)>&
                       end_admissible,
                   double max_distance, bool straight_only) {
  departure = Departure();
  const bool ground_robot = robot.type == RobotType::kGroundRobot;
  const double spacing = planning.path_interpolation_distance;
  const double step =
      spacing > 0.0 ? std::min(spacing, kDepartureMinM) : map.getResolution();
  if (!std::isfinite(max_distance) || max_distance < kDepartureMinM ||
      max_distance > 3.0 || !std::isfinite(step) || step <= 0) return false;
  const int steps = static_cast<int>(std::ceil(max_distance / step - 1e-9));
  OrientedBox standing;
  standing.center = start.head<3>() + robot.offsetForHeading(start[3]);
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
        // Always keep the original root: successive interpolation steps
        // must not move the unknown-volume exemption into unobserved air.
        return aerialRootDepartureTraversable(
            map, start.head<3>() + robot.center_offset,
            to.head<3>() + robot.center_offset, robot.getPlanningSize());
      }
      check.standing_at_start = from_start;
      std::vector<Eigen::Vector3d> projected;
      return ground.getProjectedEdgeStatus(
          from.head<3>() + robot.offsetForHeading(heading),
          to.head<3>() + robot.offsetForHeading(heading),
                                           body.size, true, projected, false,
                                           false, &check,
                                           EdgeTravel::kForward) ==
             ProjectedEdgeStatus::kAdmissible;
    };
    for (const bool backwards : {false, true}) {
      planningCheckpoint();
      if (backwards && !planning.departure_reverse_allowed) break;
      const double direction = backwards ? heading + M_PI : heading;
      const Eigen::Vector2d unit(std::cos(direction), std::sin(direction));
      StateVec here = start;
      here[3] = heading;
      departure.path.assign(1, here);
      for (int i = 1; i <= steps; ++i) {
        planningCheckpoint();
        const double out = std::min(i * step, max_distance);
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
        if (out >= kDepartureMinM - 1e-9 &&
            (ground_robot ||
             map.getStrictBoxStatus(to.head<3>() + robot.center_offset,
                                     robot.getPlanningSize()) == VoxelStatus::kFree) &&
            (end_admissible ? end_admissible(departure.path)
                            : roomToTurn(map, robot, planning, to,
                                          ground.standingStart()))) {
          departure.reverse = backwards;
          return true;
        }
      }
    }
    departure.path.clear();
    return false;
  };

  if (straight(start[3])) return true;
  if (straight_only) return false;
  // Turned in place first, nearest first; a way blocked at some turn stays
  // blocked beyond it. Each 5-degree pose is checked, not the arc between
  // them (review r0, M-1).
  const int turns =
      static_cast<int>(std::round(kDepartureMaxTurnRad / kDepartureTurnStepRad));
  bool blocked[2] = {false, false};
  for (int k = 1; k <= turns; ++k) {
    planningCheckpoint();
    for (int side = 0; side < 2; ++side) {
      planningCheckpoint();
      if (blocked[side]) continue;
      const double turn = (side == 0 ? 1.0 : -1.0) * k * kDepartureTurnStepRad;
      OrientedBox turned = standing;
      turned.heading = start[3] + turn;
      if (orientedBoxPathStatus(map,
          start.head<3>() + robot.offsetForHeading(turned.heading),
          start.head<3>() + robot.offsetForHeading(turned.heading), turned,
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
