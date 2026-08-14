#include "mgg_core/sensor_params.h"

#include <cmath>

namespace mgg {
namespace {

/// Rotation about Z by `yaw`, matching how the planner treats StateVec[3].
Eigen::Matrix3d yawRotation(double yaw) {
  return Eigen::Matrix3d(
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

}  // namespace

void SensorParams::update() {
  // Body to sensor, ZYX as in the ROS 1 loadParams precompute.
  rot_body_to_sensor_ =
      Eigen::AngleAxisd(rotations[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rotations[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rotations[2], Eigen::Vector3d::UnitX());
  rot_sensor_to_body_ = rot_body_to_sensor_.inverse();

  // A zero resolution would divide by zero below; the ROS 1 code defaulted it
  // to one degree, so keep that.
  const double h_res =
      resolution[0] > 0.0 ? resolution[0] : (1.0 * M_PI / 180.0);
  const double v_res =
      resolution[1] > 0.0 ? resolution[1] : (1.0 * M_PI / 180.0);

  frustum_endpoints_body_.clear();

  if (type == SensorType::kCamera) {
    // Frustum as a pyramid: four corner rays, and inward normals for the
    // planes between them, used by isInsideFOV.
    const double h_2 = fov[0] / 2.0;
    const double v_2 = fov[1] / 2.0;
    Eigen::Matrix<double, 3, 4> edges;
    edges.col(0) = Eigen::Vector3d(std::cos(h_2), std::sin(h_2), std::sin(v_2));
    edges.col(1) = Eigen::Vector3d(std::cos(h_2), -std::sin(h_2), std::sin(v_2));
    edges.col(2) =
        Eigen::Vector3d(std::cos(h_2), -std::sin(h_2), -std::sin(v_2));
    edges.col(3) = Eigen::Vector3d(std::cos(h_2), std::sin(h_2), -std::sin(v_2));
    normal_vectors_.col(0) = edges.col(0).cross(edges.col(1));
    normal_vectors_.col(1) = edges.col(1).cross(edges.col(2));
    normal_vectors_.col(2) = edges.col(2).cross(edges.col(3));
    normal_vectors_.col(3) = edges.col(3).cross(edges.col(0));
  }

  // Ray table, identical for camera and lidar in the ROS 1 code.
  //
  // DELIBERATE BEHAVIOUR CHANGE from ROS 1. The original built rays as
  //     max_range * (cos dh, sin dh, sin dv)
  // which is not a unit-sphere parameterisation: the length came out as
  // max_range * sqrt(1 + sin^2 dv), so rays at the vertical extremes
  // overshot the nominal range by about 3% (20.66 m for a configured 20 m at
  // +/-15 degrees), and the sensor swept a subtly barrel-shaped volume rather
  // than a spherical cap. The correct spherical form scales the horizontal
  // components by cos(dv):
  const double h_lim = fov[0] / 2.0;
  const double v_lim = fov[1] / 2.0;
  for (double dv = -v_lim; dv < v_lim; dv += v_res) {
    for (double dh = -h_lim; dh < h_lim; dh += h_res) {
      const Eigen::Vector3d ray(max_range * std::cos(dv) * std::cos(dh),
                                max_range * std::cos(dv) * std::sin(dh),
                                max_range * std::sin(dv));
      frustum_endpoints_body_.push_back(rot_body_to_sensor_ * ray +
                                        center_offset);
    }
  }

  num_voxels_full_fov_ = (fov[0] / h_res) * (fov[1] / v_res) * max_range;
}

bool SensorParams::isInsideFOV(const StateVec& state,
                               const Eigen::Vector3d& pos) const {
  // World -> body -> sensor, then a range check followed by an angle check.
  const Eigen::Vector3d origin(state[0], state[1], state[2]);
  const Eigen::Vector3d pos_sensor =
      rot_sensor_to_body_ * yawRotation(-state[3]) * (pos - origin) -
      center_offset;
  const double range = pos_sensor.norm();

  if (range > max_range) return false;

  if (type == SensorType::kCamera) {
    for (int i = 0; i < 4; ++i) {
      if (pos_sensor.dot(normal_vectors_.col(i)) <= 0.0) return false;
    }
    return true;
  }
  if (type == SensorType::kLidar) {
    if (range < 1e-9) return true;
    const double h_angle = std::atan2(pos_sensor.y(), pos_sensor.x());
    const double v_angle = std::asin(pos_sensor.z() / range);
    return std::abs(h_angle) <= fov[0] / 2.0 &&
           std::abs(v_angle) <= fov[1] / 2.0;
  }
  return false;
}

void SensorParams::getFrustumEndpoints(
    const StateVec& state, std::vector<Eigen::Vector3d>& endpoints) const {
  getFrustumEndpoints(state, endpoints, 1.0);
}

void SensorParams::getFrustumEndpoints(
    const StateVec& state, std::vector<Eigen::Vector3d>& endpoints,
    double range_scale) const {
  const Eigen::Vector3d origin(state[0], state[1], state[2]);
  const Eigen::Matrix3d rot_world_to_body = yawRotation(state[3]);
  endpoints.clear();
  endpoints.reserve(frustum_endpoints_body_.size());
  for (const Eigen::Vector3d& p : frustum_endpoints_body_) {
    endpoints.push_back(origin + rot_world_to_body * p * range_scale);
  }
}

bool SensorParams::isFrontier(double num_unknown_voxels_normalized) const {
  if (num_voxels_full_fov_ <= 0.0) return false;
  return (num_unknown_voxels_normalized / num_voxels_full_fov_) >=
         frontier_percentage_threshold;
}

}  // namespace mgg
