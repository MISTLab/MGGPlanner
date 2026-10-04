#include "mgg_ros/scan_input.h"

#include <chrono>
#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>

namespace mgg {
namespace {

bool hasFloatField(const sensor_msgs::msg::PointCloud2& cloud,
                   const std::string& name) {
  for (const auto& field : cloud.fields)
    if (field.name == name)
      return field.datatype == sensor_msgs::msg::PointField::FLOAT32;
  return false;
}

}  // namespace

std::optional<OdomScan> scanInOdom(const sensor_msgs::msg::PointCloud2& cloud,
                                   const tf2::BufferCore& buffer,
                                   const std::string& odom_frame) {
  if (cloud.header.frame_id.empty() || odom_frame.empty() ||
      !hasFloatField(cloud, "x") || !hasFloatField(cloud, "y") ||
      !hasFloatField(cloud, "z"))
    return std::nullopt;
  const tf2::TimePoint stamp(std::chrono::nanoseconds(
      static_cast<int64_t>(cloud.header.stamp.sec) * 1000000000LL +
      cloud.header.stamp.nanosec));
  geometry_msgs::msg::TransformStamped tf;
  try {
    // At the stamp only: lookupTransform throws instead of extrapolating.
    tf = buffer.lookupTransform(odom_frame, cloud.header.frame_id, stamp);
  } catch (const tf2::TransformException&) {
    return std::nullopt;
  }
  const auto& t = tf.transform.translation;
  const auto& r = tf.transform.rotation;
  const Eigen::Quaterniond rotation(r.w, r.x, r.y, r.z);
  if (!std::isfinite(rotation.norm()) || rotation.norm() < 1e-9)
    return std::nullopt;
  Eigen::Isometry3d odom_T_sensor = Eigen::Isometry3d::Identity();
  odom_T_sensor.linear() = rotation.normalized().toRotationMatrix();
  odom_T_sensor.translation() = Eigen::Vector3d(t.x, t.y, t.z);
  if (!odom_T_sensor.matrix().allFinite()) return std::nullopt;

  OdomScan scan;
  scan.origin = odom_T_sensor.translation();
  scan.points.reserve(static_cast<size_t>(cloud.width) * cloud.height);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"),
      z(cloud, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    // Organised clouds carry NaN where a ray had no return.
    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z))
      continue;
    scan.points.push_back(odom_T_sensor * Eigen::Vector3d(*x, *y, *z));
  }
  return scan;
}

}  // namespace mgg
