// Sensor geometry: field of view, the ray table used for volumetric gain, and
// the frontier test.
//
// Ported from planner_common's SensorParamsBase with one structural change.
// In the ROS 1 version the body-to-sensor rotation and the frustum ray table
// were computed *inside loadParams*, so they existed only as a side effect of
// reading ROS parameters. Constructing the struct any other way left
// rot_B2S as uninitialised Eigen memory, and the first caller to do that
// (the phase 0 baseline harness) got garbage ray endpoints and an
// std::bad_alloc from raycasting to them.
//
// Here the precomputation is an explicit update() that any construction path
// can call, every member is initialised at declaration, and the ray table
// starts empty rather than undefined. Forgetting update() now yields no rays,
// which is visible and harmless, instead of astronomical ones.
//
// Parameter loading is deliberately absent: it belongs at the ROS boundary.

#ifndef MGG_CORE_SENSOR_PARAMS_H_
#define MGG_CORE_SENSOR_PARAMS_H_

#include <vector>

#include <Eigen/Dense>

#include "mgg_core/types.h"

namespace mgg {

enum class SensorType { kCamera = 0, kLidar = 1 };
enum class CameraType { kFixed = 0, kRotating, kZoom, kRotatingZoom };

class SensorParams {
 public:
  // ------------------------------------------------------- configuration

  SensorType type = SensorType::kLidar;
  CameraType camera_type = CameraType::kFixed;

  /// Minimum range for map annotation (zoom camera only).
  double min_range = 0.0;
  /// Maximum range for volumetric gain.
  double max_range = 5.0;
  /// Offset from the body centre (odometry frame).
  Eigen::Vector3d center_offset = Eigen::Vector3d::Zero();
  /// Body to sensor, [yaw, pitch, roll] in radians, applied ZYX.
  Eigen::Vector3d rotations = Eigen::Vector3d::Zero();
  /// [horizontal, vertical] angles, radians.
  Eigen::Vector2d fov = Eigen::Vector2d::Zero();
  /// [horizontal, vertical] angular step for gain raycasting, radians.
  Eigen::Vector2d resolution = Eigen::Vector2d::Zero();
  /// Rays vertically and horizontally.
  int height = 0;
  int width = 0;
  /// Fraction of the full field of view that must be unknown for a viewpoint
  /// to count as a frontier.
  double frontier_percentage_threshold = 0.0;

  /// Recomputes the rotations and the frustum ray table from the fields
  /// above. Must be called after changing any of them; until it is, the ray
  /// table is empty and the rotations are identity.
  void update();

  /// True once update() has produced a usable ray table.
  bool isReady() const { return !frustum_endpoints_body_.empty(); }

  // ------------------------------------------------------------ queries

  /// Is `pos` (world frame) inside the sensor's field of view from `state`?
  bool isInsideFOV(const StateVec& state, const Eigen::Vector3d& pos) const;

  /// Ray endpoints in the world frame for a robot at `state`. Empty until
  /// update() has been called.
  void getFrustumEndpoints(const StateVec& state,
                           std::vector<Eigen::Vector3d>& endpoints) const;

  /// As above, with the rays scaled (used to shorten them in dark or
  /// low-visibility conditions).
  void getFrustumEndpoints(const StateVec& state,
                           std::vector<Eigen::Vector3d>& endpoints,
                           double range_scale) const;

  /// Frontier test: are the distinct unknown voxels seen from a viewpoint a
  /// large enough fraction, frontier_percentage_threshold, of the distinct
  /// voxels the sensor's rays reach in space that is all unknown? False
  /// until update().
  bool isFrontier(int num_unknown_voxels, double voxel_size) const;

  /// Distinct voxels of edge `voxel_size` that the ray table crosses from
  /// the centre of a voxel: the denominator of the frontier test. Computed
  /// once per sensor geometry and voxel size. Zero until update().
  double uniqueVoxelsFullFov(double voxel_size) const;

  /// The subset of this description the map layer needs.
  SensorModel model() const { return SensorModel{width, height, resolution}; }

 private:
  // Every one of these is initialised. The ROS 1 original left the rotations
  // undefined unless loadParams ran.
  Eigen::Matrix3d rot_body_to_sensor_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rot_sensor_to_body_ = Eigen::Matrix3d::Identity();
  /// Camera model only: inward normals of the four frustum planes.
  Eigen::Matrix<double, 3, 4> normal_vectors_ =
      Eigen::Matrix<double, 3, 4>::Zero();
  std::vector<Eigen::Vector3d> frustum_endpoints_body_;
};

}  // namespace mgg

#endif  // MGG_CORE_SENSOR_PARAMS_H_
