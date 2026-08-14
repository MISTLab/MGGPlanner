// ROS 2 ingest for the OctoMap backend: PointCloud2 in, occupancy map out.
//
// This is the only ROS-aware part of mgg_map_octomap. OctomapMap itself stays
// free of ROS so it can be unit tested off-robot, which is the arrangement
// that lets the same map code serve ARGoS and a real robot.
//
// It replaces voxblox_ros::TsdfServer, which was the piece of voxblox that
// actually pulled ROS (and, transitively, rviz) into the planner's build.
//
// No PCL: sensor_msgs::PointCloud2Iterator reads the cloud directly, so this
// package does not inherit PCL's build weight for what amounts to a loop over
// three floats.

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "mgg_map_octomap/octomap_map.h"

namespace mgg {

class OctomapMapNode : public rclcpp::Node {
 public:
  OctomapMapNode() : rclcpp::Node("mgg_octomap_map") {
    OctomapConfig cfg;
    // Defaults mirror the ROS 1 voxblox_sim_config_sim.yaml where an
    // equivalent exists, so a port keeps the same spatial fidelity:
    // tsdf_voxel_size 0.20 -> resolution, max_ray_length_m -> max_range.
    cfg.resolution = declare_parameter("resolution", 0.20);
    cfg.max_range = declare_parameter("max_range", 20.0);
    cfg.probability_hit = declare_parameter("probability_hit", 0.7);
    cfg.probability_miss = declare_parameter("probability_miss", 0.4);
    cfg.clamping_min = declare_parameter("clamping_min", 0.12);
    cfg.clamping_max = declare_parameter("clamping_max", 0.97);
    cfg.occupancy_threshold = declare_parameter("occupancy_threshold", 0.5);
    cfg.occupied_dilation_voxels =
        static_cast<int>(declare_parameter("occupied_dilation_voxels", 0));

    world_frame_ = declare_parameter("world_frame", std::string("world"));
    // Matches the ROS 1 launch files, which remapped "pointcloud" onto
    // whatever the robot actually published.
    const std::string topic = declare_parameter("pointcloud_topic",
                                                std::string("pointcloud"));

    map_ = std::make_unique<OctomapMap>(cfg);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Sensor data is best-effort and high rate; a dropped scan is better than
    // a growing queue.
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          onCloud(*msg);
        });

    RCLCPP_INFO(get_logger(),
                "octomap map: resolution %.3f m, max_range %.1f m, world "
                "frame '%s', listening on '%s'",
                cfg.resolution, cfg.max_range, world_frame_.c_str(),
                topic.c_str());
  }

  MapInterface& map() { return *map_; }

 private:
  void onCloud(const sensor_msgs::msg::PointCloud2& msg) {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(world_frame_, msg.header.frame_id,
                                       msg.header.stamp,
                                       rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException& ex) {
      // Throttled: with a missing transform this fires at full sensor rate.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "dropping cloud, no transform %s -> %s: %s",
                           msg.header.frame_id.c_str(), world_frame_.c_str(),
                           ex.what());
      return;
    }

    const Eigen::Quaterniond q(
        tf.transform.rotation.w, tf.transform.rotation.x,
        tf.transform.rotation.y, tf.transform.rotation.z);
    const Eigen::Vector3d origin(tf.transform.translation.x,
                                 tf.transform.translation.y,
                                 tf.transform.translation.z);
    const Eigen::Matrix3d rot = q.normalized().toRotationMatrix();

    std::vector<Eigen::Vector3d> points;
    points.reserve(msg.width * msg.height);
    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      // Non-finite entries are normal in organised clouds (no return on that
      // ray) and would otherwise poison the octree bounds.
      if (!std::isfinite(*it_x) || !std::isfinite(*it_y) ||
          !std::isfinite(*it_z)) {
        continue;
      }
      points.emplace_back(rot * Eigen::Vector3d(*it_x, *it_y, *it_z) + origin);
    }
    if (points.empty()) return;

    map_->insertPointCloud(points, origin);
  }

  std::unique_ptr<OctomapMap> map_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  std::string world_frame_;
};

}  // namespace mgg

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mgg::OctomapMapNode>());
  rclcpp::shutdown();
  return 0;
}
