// Ground scan input (C5a): the cloud is placed with TF at its own stamp, the
// scan origin is the sensor frame's origin, and a cloud without TF at its
// stamp is dropped rather than placed with the latest transform.

#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/buffer_core.h>

#include "mgg_ros/scan_input.h"

using namespace mgg;

namespace {

builtin_interfaces::msg::Time stampAt(double t) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(std::floor(t));
  stamp.nanosec =
      static_cast<uint32_t>(std::llround((t - std::floor(t)) * 1e9));
  return stamp;
}

void addTransform(tf2::BufferCore& buffer, double t, double x, double yaw) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "odom";
  tf.header.stamp = stampAt(t);
  tf.child_frame_id = "lidar";
  tf.transform.translation.x = x;
  tf.transform.translation.z = 0.6;
  tf.transform.rotation.z = std::sin(yaw / 2);
  tf.transform.rotation.w = std::cos(yaw / 2);
  buffer.setTransform(tf, "test", false);
}

sensor_msgs::msg::PointCloud2 cloudAt(double t, const std::string& frame) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame;
  cloud.header.stamp = stampAt(t);
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(2);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"),
      z(cloud, "z");
  *x = 1;
  *y = 0;
  *z = 0;
  ++x;
  ++y;
  ++z;
  *x = NAN;
  *y = 0;
  *z = 0;
  return cloud;
}

}  // namespace

TEST(ScanInput, TfAtStampAndSensorOrigin) {
  tf2::BufferCore buffer;
  addTransform(buffer, 10.0, 1.0, 0.0);
  addTransform(buffer, 11.0, 2.0, 0.0);
  addTransform(buffer, 12.0, 3.0, M_PI / 2);

  const auto scan = scanInOdom(cloudAt(10.5, "lidar"), buffer, "odom");
  ASSERT_TRUE(scan);
  const Eigen::Vector3d origin_at_stamp(1.5, 0, 0.6);
  EXPECT_TRUE(scan->origin.isApprox(origin_at_stamp, 1e-9));
  // The non-finite return is dropped; the other is placed at the stamp.
  ASSERT_EQ(scan->points.size(), 1u);
  EXPECT_TRUE(scan->points[0].isApprox(Eigen::Vector3d(2.5, 0, 0.6), 1e-6));

  // Rotation at the stamp, not the latest yaw.
  const auto rotated = scanInOdom(cloudAt(12.0, "lidar"), buffer, "odom");
  ASSERT_TRUE(rotated);
  EXPECT_TRUE(rotated->points[0].isApprox(Eigen::Vector3d(3, 1, 0.6), 1e-6));

  // After the newest transform there is no TF at the stamp: no scan.
  const auto cloud_without_tf = cloudAt(12.5, "lidar");
  EXPECT_FALSE(scanInOdom(cloud_without_tf, buffer, "odom"));
  EXPECT_FALSE(scanInOdom(cloudAt(10.5, "unknown_lidar"), buffer, "odom"));
  // A zero stamp is TF's "latest": it must not place the cloud with the
  // newest dynamic transform.
  EXPECT_FALSE(scanInOdom(cloudAt(0.0, "lidar"), buffer, "odom"));
}
