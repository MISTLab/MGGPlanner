// The ground local planner process (mgg_local_planner_node): drives
// LocalPlanningCore from the robot's own cloud, odometry, guidance, no-go
// zones, executor feedback and set_mode, and publishes certified local
// paths, their invalidations, status and the traversability grid.
//
// Name mggplanner_node (it reads the planner's parameter file), namespace
// <robot>/mgg/local_planner. Relative topics: local_path,
// local_path_feedback, local_path_invalidation, status, traversability,
// global_guidance, pointcloud, odometry, no_go_zones; service set_mode.
//
// Callbacks are serialized: one mutex guards the core. A scan without TF at
// its stamp is dropped and counted. The odometry frame is the first
// odometry header's frame; outputs are in it.

#ifndef MGG_ROS_LOCAL_PLANNER_NODE_H_
#define MGG_ROS_LOCAL_PLANNER_NODE_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_array.hpp>
#include <mgg_msgs/msg/global_guidance.hpp>
#include <mgg_msgs/msg/local_path.hpp>
#include <mgg_msgs/msg/local_path_feedback.hpp>
#include <mgg_msgs/msg/local_path_invalidation.hpp>
#include <mgg_msgs/msg/local_planner_status.hpp>
#include <mgg_msgs/srv/set_local_planner_mode.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mgg_ros/local_planning_core.h"

namespace mgg {

class LocalPlannerNode : public rclcpp::Node {
 public:
  using SetMode = mgg_msgs::srv::SetLocalPlannerMode;

  explicit LocalPlannerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // The subscription and service callbacks, public so a test can drive
  // them in a chosen order.
  void onPointCloud(const sensor_msgs::msg::PointCloud2& msg);
  void onOdometry(const nav_msgs::msg::Odometry& msg);
  void onFeedback(const mgg_msgs::msg::LocalPathFeedback& msg);
  void onGuidance(const mgg_msgs::msg::GlobalGuidance& msg);
  void onNoGoZones(const geometry_msgs::msg::PoseArray& msg);
  void onSetMode(const SetMode::Request& request, SetMode::Response& response);
  /// One planning cycle, as the planning_period_s timer runs it.
  void planCycle();

  /// The TF buffer scans and frame lookups read.
  tf2::BufferCore& tfBuffer() { return *tf_buffer_; }
  /// Scans dropped: before odometry, or without TF at their stamp.
  std::uint64_t droppedScans() const;
  /// For tests, between callbacks only: the core is not locked.
  const LocalPlanningCore& core() const { return core_; }

 private:
  LocalPlanningParams loadParameters();
  void runPlan();
  void publishInvalidations(bool plan_after);
  void publishGrid();
  std::optional<Eigen::Isometry3d> lookup(const std::string& frame) const;

  mutable std::mutex mutex_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double no_go_radius_m_ = 0;
  double planning_budget_s_ = 0;
  LocalPlanningCore core_;
  std::string odom_frame_;
  std::optional<StateVec> base_;
  std::uint64_t dropped_scans_ = 0;

  rclcpp::Publisher<mgg_msgs::msg::LocalPath>::SharedPtr path_pub_;
  rclcpp::Publisher<mgg_msgs::msg::LocalPathInvalidation>::SharedPtr
      invalidation_pub_;
  rclcpp::Publisher<mgg_msgs::msg::LocalPlannerStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<mgg_msgs::msg::LocalPathFeedback>::SharedPtr
      feedback_sub_;
  rclcpp::Subscription<mgg_msgs::msg::GlobalGuidance>::SharedPtr guidance_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr no_go_sub_;
  rclcpp::Service<SetMode>::SharedPtr set_mode_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mgg

#endif  // MGG_ROS_LOCAL_PLANNER_NODE_H_
