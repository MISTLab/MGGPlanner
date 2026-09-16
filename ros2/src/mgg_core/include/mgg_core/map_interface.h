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
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/types.h"

namespace mgg {

class MapInterface {
 public:
  virtual ~MapInterface() = default;

  /// Edge length of one voxel, in metres.
  virtual double getResolution() const = 0;

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

  /// Same tally, but skipping rays whose voxels a neighbouring ray has
  /// already covered. Cheaper and slightly less exact.
  virtual void getScanStatusIterative(
      const Eigen::Vector3d& pos,
      const std::vector<Eigen::Vector3d>& multiray_endpoints,
      GainCounts& gain, std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>&
                            voxel_log,
      const SensorModel& sensor) = 0;

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
