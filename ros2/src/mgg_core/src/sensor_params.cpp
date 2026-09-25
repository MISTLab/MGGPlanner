#include "mgg_core/sensor_params.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <mutex>
#include <unordered_set>
#include <vector>

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

double SensorParams::uniqueVoxelsFullFov(double voxel_size) const {
  if (frustum_endpoints_body_.empty() || !std::isfinite(voxel_size) ||
      voxel_size <= 0.0) {
    return 0.0;
  }
  // Cells are packed into 21 bits per axis, offset to stay positive.
  constexpr std::int64_t kHalfSpan = std::int64_t(1) << 20;
  double reach = 0.0;
  for (const Eigen::Vector3d& p : frustum_endpoints_body_)
    reach = std::max(reach, p.cwiseAbs().maxCoeff());
  if (!(reach / voxel_size < double(kHalfSpan - 2))) return 0.0;

  // The ray table and the voxel size decide the count, and the planner asks
  // for it on every gain evaluation, so it is computed once per geometry.
  std::vector<double> key{voxel_size};
  key.reserve(1 + 3 * frustum_endpoints_body_.size());
  for (const Eigen::Vector3d& p : frustum_endpoints_body_)
    key.insert(key.end(), {p.x(), p.y(), p.z()});
  static std::mutex mutex;
  static std::map<std::vector<double>, double> counts;
  {
    const std::lock_guard<std::mutex> lock(mutex);
    const auto found = counts.find(key);
    if (found != counts.end()) return found->second;
  }

  // A 3-D DDA of each ray from the centre of voxel (0, 0, 0).
  std::unordered_set<std::uint64_t> cells;
  const auto pack = [&](const Eigen::Matrix<std::int64_t, 3, 1>& c) {
    return (std::uint64_t(c.x() + kHalfSpan) << 42) |
           (std::uint64_t(c.y() + kHalfSpan) << 21) |
           std::uint64_t(c.z() + kHalfSpan);
  };
  const Eigen::Vector3d origin = Eigen::Vector3d::Constant(0.5 * voxel_size);
  for (const Eigen::Vector3d& p : frustum_endpoints_body_) {
    Eigen::Matrix<std::int64_t, 3, 1> cell(0, 0, 0), step(0, 0, 0), last;
    Eigen::Vector3d next = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    Eigen::Vector3d delta = next;
    for (int axis = 0; axis < 3; ++axis) {
      last[axis] = std::int64_t(std::floor((origin[axis] + p[axis]) / voxel_size));
      if (p[axis] == 0.0) continue;
      step[axis] = p[axis] > 0.0 ? 1 : -1;
      next[axis] = 0.5 * voxel_size / std::abs(p[axis]);
      delta[axis] = voxel_size / std::abs(p[axis]);
    }
    cells.insert(pack(cell));
    while (cell != last) {
      int axis = 0;
      next.minCoeff(&axis);
      if (next[axis] > 1.0) break;
      cell[axis] += step[axis];
      next[axis] += delta[axis];
      cells.insert(pack(cell));
    }
  }
  const double count = double(cells.size());
  const std::lock_guard<std::mutex> lock(mutex);
  counts.emplace(std::move(key), count);
  return count;
}

bool SensorParams::isFrontier(int num_unknown_voxels, double voxel_size) const {
  const double full = uniqueVoxelsFullFov(voxel_size);
  if (full <= 0.0) return false;
  return num_unknown_voxels / full >= frontier_percentage_threshold;
}

}  // namespace mgg
