// The contract between the MGG planner and whatever builds its map.
//
// This is the ROS 2 successor to planner_common/map_manager.h. It is what
// makes the mapping backend swappable: the planner asks only for ternary
// occupancy, never for a signed distance, so voxblox is not required (see
// section 4 of ROS2_PORT_PLAN.md).
//
// Differences from the ROS 1 MapManager, all deliberate:
//
//  * No ROS. The ROS 1 base class stored two ros::NodeHandle members that no
//    method used.
//  * Public inheritance is now the documented expectation. The ROS 1
//    MapManagerVoxblox inherited *privately* (the access specifier was simply
//    omitted from a `class`), so the abstraction was decorative and nothing
//    could hold a base pointer to it.
//  * The five-argument getRayStatus has lost its `tsdf_dist` out-parameter.
//    After the dead distance-field paths were removed it had exactly one
//    caller, which wrote it and never read it. `end_voxel` is read and stays.
//  * Point-cloud outputs are std::vector<Eigen::Vector3d> rather than PCL
//    clouds, keeping PCL out of the core. Conversion happens at the ROS
//    boundary.
//  * The gain tally is a named GainCounts instead of std::tuple<int,int,int>.

#ifndef MGG_CORE_MAP_INTERFACE_H_
#define MGG_CORE_MAP_INTERFACE_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/types.h"

namespace mgg {

/// Largest tilt, in radians, between a map authority transform's Z axis and
/// the navigation Z axis that XY footprint queries accept. A single robot's
/// peer SLAM correction carries a few milliradians of pitch and roll; an
/// inter-robot merge is a 6 DoF registration between base frames and leaves
/// up to 1.4 degrees (benchbot 2026-09-20, four robots merged: robot_0's
/// frame tilted 1.174 degrees, 0.0205 rad, and at the earlier 0.02 rad bound
/// every one of its plans was refused as "footprint cell grid unavailable").
/// At 0.05 rad the circle and cylinder contracts still hold to well under a
/// centimetre across a footprint (cos(0.05) differs from one by 0.13
/// percent, 1.2 mm across a metre) and the ground of a distant cell is off
/// by the tilt times the distance, a slope of 5 percent that adjacent-cell
/// step checks see as 1 cm per 0.2 m cell. Larger tilts still refuse,
/// because a tilted footprint is no longer a circle in the map's XY plane.
inline constexpr double kMaxAuthorityTiltRad = 0.05;

/// Angle, in radians, between the rotated Z axis and Z.
inline double authorityTiltRad(const Eigen::Matrix3d& rotation) {
  const Eigen::Vector3d up = rotation * Eigen::Vector3d::UnitZ();
  const double norm = up.norm();
  if (!std::isfinite(norm) || norm <= 0.0) return std::numeric_limits<double>::infinity();
  return std::acos(std::clamp(up.z() / norm, -1.0, 1.0));
}

/// True when XY footprint queries may treat the transform as level.
inline bool authorityTiltAcceptable(const Eigen::Matrix3d& rotation) {
  const double tilt = authorityTiltRad(rotation);
  return std::isfinite(tilt) && tilt <= kMaxAuthorityTiltRad;
}

/// Heights, in the caller's frame, at which an occupied voxel is evidence of
/// a wall for getVisibleScanStatus. The band starts above the floor, so that
/// floor returns are never taken for a wall. A voxel belongs to the band
/// when its centre does.
struct WallBand {
  double min_z = 0.0;
  double max_z = 0.0;
};

/// Most voxels, stacked in a column between two wall returns, that
/// getVisibleScanStatus takes for a gap in the wall. A lidar that carves
/// free space with one ray per 5 degree bin leaves 0.4 m between returns
/// at 4.6 m; a window or a doorway is taller.
inline constexpr int kMaxWallGapVoxels = 2;

struct XYCellCenter {
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  std::int64_t grid_x = 0;
  std::int64_t grid_y = 0;
};

class MapInterface {
 public:
  virtual ~MapInterface() = default;

  /// Edge length of one voxel, in metres.
  virtual double getResolution() const = 0;

  /// Centre of the uniform, axis-aligned XY cell containing `position`.
  /// Backends without this grid contract leave the query unsupported.
  virtual bool getAxisAlignedXYCellCenter(
      const Eigen::Vector2d& position, Eigen::Vector2d& center) const {
    (void)position;
    (void)center;
    return false;
  }

  /// Centres of every XY map cell whose closed square intersects a circle.
  /// The result is expressed in the caller's frame and is bounded by
  /// `maximum_cells`. Backends whose voxel grid is rotated relative to that
  /// frame override this method and transform the complete cell set.
  virtual bool getCircleIntersectingXYCellCenters(
      const Eigen::Vector2d& circle_center, double radius,
      std::size_t maximum_cells,
      std::vector<XYCellCenter>& centers) const {
    centers.clear();
    const double resolution = getResolution();
    if (!circle_center.allFinite() || !std::isfinite(radius) || radius < 0.0 ||
        !std::isfinite(resolution) || resolution <= 0.0 ||
        maximum_cells == 0) {
      return false;
    }
    Eigen::Vector2d containing_center;
    if (!getAxisAlignedXYCellCenter(
            circle_center - Eigen::Vector2d::Constant(radius),
            containing_center) ||
        !containing_center.allFinite()) {
      return false;
    }
    const double half_cell = 0.5 * resolution;
    const double intervals_d = std::ceil(2.0 * radius / resolution) + 4.0;
    constexpr std::size_t kMaxVisitedCells = 4096;
    constexpr double kMaxIntervals = 63.0;
    if (!std::isfinite(intervals_d) || intervals_d < 1.0 ||
        intervals_d > kMaxIntervals ||
        intervals_d > static_cast<double>(maximum_cells)) {
      return false;
    }
    const std::size_t intervals = static_cast<std::size_t>(intervals_d);
    const std::size_t side = intervals + 1;
    // Bound visited cells as well as output cells. This method runs inside a
    // planner callback, so a huge but finite radius must fail before looping.
    const std::size_t visited_limit =
        std::min(maximum_cells, kMaxVisitedCells);
    if (side > visited_limit / side) return false;
    const Eigen::Vector2d first_center =
        containing_center - Eigen::Vector2d::Constant(resolution);
    if (!first_center.allFinite() ||
        ((circle_center - Eigen::Vector2d::Constant(radius) -
          containing_center)
                 .cwiseAbs()
                 .array() >
             half_cell + 1e-6)
            .any()) {
      return false;
    }
    for (std::size_t ix = 0; ix <= intervals; ++ix) {
      for (std::size_t iy = 0; iy <= intervals; ++iy) {
        const Eigen::Vector2d cell_center =
            first_center + resolution * Eigen::Vector2d(ix, iy);
        const Eigen::Vector2d outside =
            ((circle_center - cell_center).cwiseAbs() -
             Eigen::Vector2d::Constant(half_cell))
                .cwiseMax(0.0);
        if (outside.squaredNorm() > radius * radius + 1e-12) continue;
        if (centers.size() >= maximum_cells) {
          centers.clear();
          return false;
        }
        centers.push_back(
            {cell_center, static_cast<std::int64_t>(ix),
             static_cast<std::int64_t>(iy)});
      }
    }
    return !centers.empty();
  }

  /// Whether the map has received enough data to be queried meaningfully.
  virtual bool getStatus() const = 0;

  // --------------------------------------------------------------- queries

  virtual VoxelStatus getVoxelStatus(const Eigen::Vector3d& position) const = 0;

  /// Occupancy along the segment from `view_point` to `voxel_to_test`.
  /// When `stop_at_unknown_voxel` is set, the first unknown voxel ends the
  /// walk and returns kUnknown; otherwise unknown voxels are traversed and
  /// only occupancy terminates it.
  virtual VoxelStatus getRayStatus(const Eigen::Vector3d& view_point,
                                   const Eigen::Vector3d& voxel_to_test,
                                   bool stop_at_unknown_voxel) const = 0;

  /// As above, additionally reporting the voxel centre where the walk
  /// stopped. Used to measure how far the ground lies below a sample.
  virtual VoxelStatus getRayStatus(const Eigen::Vector3d& view_point,
                                   const Eigen::Vector3d& voxel_to_test,
                                   bool stop_at_unknown_voxel,
                                   Eigen::Vector3d& end_voxel) const = 0;

  /// Ground queries may use a backend's measured surface height while
  /// retaining the ordinary ray's occupancy and stopping semantics.
  virtual VoxelStatus getGroundRayStatus(
      const Eigen::Vector3d& view_point,
      const Eigen::Vector3d& voxel_to_test, bool stop_at_unknown_voxel,
      Eigen::Vector3d& end_voxel) const {
    return getRayStatus(view_point, voxel_to_test, stop_at_unknown_voxel,
                        end_voxel);
  }

  /// Occupancy of an axis-aligned box. kOccupied if any voxel is occupied.
  virtual VoxelStatus getBoxStatus(const Eigen::Vector3d& center,
                                   const Eigen::Vector3d& size,
                                   bool stop_at_unknown_voxel) const = 0;

  /// Occupancy of the volume swept by `box_size` moved from `start` to `end`.
  /// This is the collision check for a graph edge, and the planner's hottest
  /// map query.
  virtual VoxelStatus getPathStatus(const Eigen::Vector3d& start,
                                    const Eigen::Vector3d& end,
                                    const Eigen::Vector3d& box_size,
                                    bool stop_at_unknown_voxel) const = 0;

  virtual VoxelStatus getStrictBoxStatus(const Eigen::Vector3d& center,
                                         const Eigen::Vector3d& size) const {
    return getBoxStatus(center, size, true);
  }
  virtual VoxelStatus getStrictPathStatus(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end,
      const Eigen::Vector3d& box_size) const {
    return getPathStatus(start, end, box_size, true);
  }
  /// Conservatively reject occupied swept volume while allowing valid map
  /// queries to contain unknown air. Each sample is the AABB swept over one
  /// <=voxel-resolution interval, so obstacles at sample boundaries cannot
  /// fall through gaps. kUnknown remains an error signal from invalid or
  /// over-budget map queries rather than being silently accepted.
  virtual VoxelStatus getOccupiedOnlyPathStatus(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end,
      const Eigen::Vector3d& box_size) const {
    if (!start.allFinite() || !end.allFinite() || !box_size.allFinite() ||
        (box_size.array() < 0).any())
      return VoxelStatus::kUnknown;
    const double resolution = getResolution();
    const double length = (end - start).norm();
    if (!std::isfinite(resolution) || resolution <= 0.0 ||
        !std::isfinite(length))
      return VoxelStatus::kUnknown;
    if (length < 1e-9) return getBoxStatus(start, box_size, false);
    constexpr std::uint64_t kMaxSweepCells = 1u << 22;
    const double steps_d = std::max(1.0, std::ceil(length / resolution));
    if (!std::isfinite(steps_d) || steps_d > double(kMaxSweepCells))
      return VoxelStatus::kUnknown;
    const auto steps = static_cast<std::uint64_t>(steps_d);
    const Eigen::Vector3d step = (end - start) / double(steps);
    const Eigen::Vector3d swept_size = box_size + step.cwiseAbs();
    std::uint64_t box_cells = 1;
    for (int axis = 0; axis < 3; ++axis) {
      const double count = std::ceil(swept_size[axis] / resolution) + 2.0;
      if (!std::isfinite(count) ||
          count > double(kMaxSweepCells / box_cells))
        return VoxelStatus::kUnknown;
      box_cells *= static_cast<std::uint64_t>(count);
    }
    if (steps > kMaxSweepCells / box_cells) return VoxelStatus::kUnknown;
    for (std::uint64_t i = 0; i < steps; ++i) {
      const double fraction = (double(i) + 0.5) / double(steps);
      const VoxelStatus status = getBoxStatus(
          start + fraction * (end - start), swept_size, false);
      if (status != VoxelStatus::kFree) return status;
    }
    return VoxelStatus::kFree;
  }

  /// Occupancy of a vertical circular body swept between two centres.
  /// Unknown air is admissible, while invalid/unbounded queries and any known
  /// occupied voxel reject. Backends may override this to enumerate their
  /// native grid without a map call per XY cell.
  virtual VoxelStatus getOccupiedOnlyCylinderPathStatus(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end, double radius,
      double height) const {
    if (!start.allFinite() || !end.allFinite() || !std::isfinite(radius) ||
        radius < 0.0 || !std::isfinite(height) || height < 0.0) {
      return VoxelStatus::kUnknown;
    }
    const double resolution = getResolution();
    const double length = (end - start).norm();
    if (!std::isfinite(resolution) || resolution <= 0.0 ||
        !std::isfinite(length)) {
      return VoxelStatus::kUnknown;
    }
    constexpr std::uint64_t kMaxWork = 1u << 22;
    constexpr std::size_t kMaxCircleCells = 4096;
    const double steps_d = std::max(1.0, std::ceil(length / resolution));
    if (!std::isfinite(steps_d) || steps_d > double(kMaxWork)) {
      return VoxelStatus::kUnknown;
    }
    const auto steps = static_cast<std::uint64_t>(steps_d);
    const Eigen::Vector3d step = (end - start) / double(steps);
    const double sample_radius = radius + 0.5 * step.head<2>().norm();
    const double sample_height = height + std::abs(step.z());
    const double z_cells_d = std::ceil(sample_height / resolution) + 2.0;
    if (!std::isfinite(sample_radius) || !std::isfinite(sample_height) ||
        !std::isfinite(z_cells_d) || z_cells_d < 1.0 ||
        z_cells_d > double(kMaxWork)) {
      return VoxelStatus::kUnknown;
    }
    const auto z_cells = static_cast<std::uint64_t>(z_cells_d);
    std::uint64_t work = 0;
    std::vector<XYCellCenter> cells;
    for (std::uint64_t i = 0; i < steps; ++i) {
      const Eigen::Vector3d center =
          start + (double(i) + 0.5) * step;
      if (work > kMaxWork - kMaxCircleCells) {
        return VoxelStatus::kUnknown;
      }
      work += kMaxCircleCells;
      if (!getCircleIntersectingXYCellCenters(
              center.head<2>(), sample_radius, kMaxCircleCells, cells)) {
        return VoxelStatus::kUnknown;
      }
      if (cells.size() > (kMaxWork - work) / z_cells) {
        return VoxelStatus::kUnknown;
      }
      work += static_cast<std::uint64_t>(cells.size()) * z_cells;
      for (const XYCellCenter& cell : cells) {
        const VoxelStatus status = getBoxStatus(
            Eigen::Vector3d(cell.center.x(), cell.center.y(), center.z()),
            Eigen::Vector3d(0.0, 0.0, sample_height), false);
        if (status != VoxelStatus::kFree) return status;
      }
    }
    return VoxelStatus::kFree;
  }
  // ------------------------------------------------------- volumetric gain

  /// Casts a ray to every endpoint and tallies what each one passes through.
  /// `voxel_log` receives every visited voxel with its status, for
  /// visualisation. This drives frontier selection, so it is the query whose
  /// behaviour matters most when swapping backends.
  virtual void getScanStatus(
      const Eigen::Vector3d& pos,
      const std::vector<Eigen::Vector3d>& multiray_endpoints,
      GainCounts& gain, std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>&
                            voxel_log,
      const SensorModel& sensor) = 0;

  /// Same tally, counting each voxel once however many rays cross it; a ray
  /// still walks on through voxels an earlier ray counted. This is what
  /// volumetric gain uses.
  virtual void getScanStatusIterative(
      const Eigen::Vector3d& pos,
      const std::vector<Eigen::Vector3d>& multiray_endpoints,
      GainCounts& gain, std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>&
                            voxel_log,
      const SensorModel& sensor) = 0;

  /// getScanStatusIterative for rays that see only what the sensor could.
  /// A lidar leaves unknown gaps in a wall between the voxels its returns
  /// landed in, and a ray through such a gap counts the space behind the
  /// wall. So an unknown voxel with an occupied voxel of `wall` below it and
  /// another above it in its XY column, with at most kMaxWallGapVoxels
  /// voxels between the two, ends the ray as an occupied voxel does, and is
  /// neither counted nor logged. Nothing is inferred above a column's highest
  /// return or below its lowest: the open space over a window sill, a
  /// railing or a rising ramp, and a doorway, still let rays through.
  /// Backends without this override scan as getScanStatusIterative does.
  virtual void getVisibleScanStatus(
      const Eigen::Vector3d& pos,
      const std::vector<Eigen::Vector3d>& multiray_endpoints,
      const WallBand& wall, GainCounts& gain,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
      const SensorModel& sensor) {
    (void)wall;
    getScanStatusIterative(pos, multiray_endpoints, gain, voxel_log, sensor);
  }

  // ----------------------------------------------------------- mutation

  /// Force a box to free. Used to clear the robot's own footprint at start,
  /// which would otherwise be unknown and block every outgoing edge.
  virtual bool augmentFreeBox(const Eigen::Vector3d& position,
                              const Eigen::Vector3d& box_size) = 0;

  /// Force the initial sensor frustum to free.
  virtual void augmentFreeFrustum() = 0;

  virtual void resetMap() = 0;

  // ----------------------------------------------------------- extraction

  virtual void extractLocalMap(const Eigen::Vector3d& center,
                               const Eigen::Vector3d& bounding_box_size,
                               std::vector<Eigen::Vector3d>& occupied_voxels,
                               std::vector<Eigen::Vector3d>& free_voxels) = 0;

  virtual void extractLocalMapAlongAxis(
      const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
      const Eigen::Vector3d& bounding_box_size,
      std::vector<Eigen::Vector3d>& occupied_voxels,
      std::vector<Eigen::Vector3d>& free_voxels) = 0;

  /// Occupied voxel centres within `range` of `center`. Used by the adaptive
  /// bounding box to fit the sampling volume to nearby geometry.
  virtual void getLocalPointcloud(const Eigen::Vector3d& center, double range,
                                  double yaw,
                                  std::vector<Eigen::Vector3d>& points,
                                  bool include_unknown_voxels = false) = 0;

  /// Free voxel centres along the given rays from `state`.
  virtual void getFreeSpacePointCloud(
      const std::vector<Eigen::Vector3d>& multiray_endpoints,
      const StateVec& state, std::vector<Eigen::Vector3d>& points) = 0;

  // ------------------------------------------------------------- tuning

  virtual void setRaycastingParams(bool nonuniform_ray_cast,
                                   double ray_cast_step_size_multiplier) = 0;

  virtual void setRobotRadius(double robot_radius) = 0;
};

}  // namespace mgg

#endif  // MGG_CORE_MAP_INTERFACE_H_
