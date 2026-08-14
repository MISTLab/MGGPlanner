// Core value types for the MGG planner.
//
// Deliberately free of ROS: this header pulls in Eigen and nothing else, so
// the planning core can be unit tested without a ROS installation, a
// simulator or a robot. ROS messages are converted at the mgg_ros boundary.

#ifndef MGG_CORE_TYPES_H_
#define MGG_CORE_TYPES_H_

#include <cstdint>

#include <Eigen/Dense>

namespace mgg {

/// Robot state: x, y, z, yaw. Matches the ROS 1 StateVec so the port is a
/// rename rather than a reinterpretation.
using StateVec = Eigen::Vector4d;

/// Occupancy of a point in the map. This is the *entire* semantic contract
/// the planner needs from the mapping backend: it never reads a distance
/// field. See section 4 of ROS2_PORT_PLAN.md.
enum class VoxelStatus : std::uint8_t {
  kUnknown = 0,
  kOccupied,
  kFree,
};

/// Result of a volumetric-gain scan: how many voxels of each kind the sensor
/// would see from a pose. The exploration planner maximises `unknown`.
///
/// Replaces the bare std::tuple<int,int,int> the ROS 1 code passed around,
/// where the field order was only discoverable by reading std::get<N> call
/// sites.
struct GainCounts {
  int unknown = 0;
  int occupied = 0;
  int free = 0;
};

/// What the map layer needs to know about a sensor in order to compute
/// volumetric gain.
///
/// Only these three fields are used: the ROS 1 map implementation touched
/// exactly sensor_params.width, .height and .resolution and nothing else.
/// Range and field of view do not appear because the caller supplies the ray
/// endpoints already resolved into world coordinates.
struct SensorModel {
  /// Number of rays horizontally.
  int width = 0;
  /// Number of rays vertically.
  int height = 0;
  /// Angular step between adjacent rays, radians, [horizontal, vertical].
  /// Used to skip raycasts that would land in an already-visited voxel.
  Eigen::Vector2d resolution = Eigen::Vector2d::Zero();
};

}  // namespace mgg

#endif  // MGG_CORE_TYPES_H_
