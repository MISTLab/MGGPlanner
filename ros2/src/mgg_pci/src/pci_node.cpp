#include "mgg_pci/pci_node.h"

#include <chrono>

namespace mgg_pci {

PciNode::PciNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mgg_pci", options),
      last_progress_time_(0, 0, RCL_ROS_TIME),
      path_start_time_(0, 0, RCL_ROS_TIME) {
  service_timeout_sec_ =
      declare_parameter("service_timeout_sec", service_timeout_sec_);
  bound_mode_ = static_cast<int>(declare_parameter("bound_mode", 0));
  world_frame_ = declare_parameter("world_frame", world_frame_);
  reach_distance_ = declare_parameter("reach_distance", reach_distance_);
  stuck_timeout_sec_ = declare_parameter("stuck_timeout_sec", stuck_timeout_sec_);
  const double auto_period = declare_parameter("auto_period_sec", 1.0);

  callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  planner_client_ = create_client<mgg_msgs::srv::PlannerSrv>(
      "mggplanner", rclcpp::ServicesQoS(), callback_group_);

  trigger_srv_ = create_service<std_srvs::srv::Trigger>(
      "pci_trigger",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        onTrigger(req, res);
      },
      rclcpp::ServicesQoS(), callback_group_);

  stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "pci_stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        onStop(req, res);
      },
      rclcpp::ServicesQoS(), callback_group_);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onOdometry(m); },
      sub_opts);

  // Latched: the path is a latest-value topic and a follower may start later.
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "command_path", rclcpp::QoS(1).transient_local());

  if (auto_period > 0.0) {
    auto_timer_ = create_timer(std::chrono::duration<double>(auto_period),
                               [this]() { tick(); }, callback_group_);
    RCLCPP_INFO(get_logger(), "auto watchdog: checking progress every %.1f s",
                auto_period);
  }
  RCLCPP_INFO(get_logger(), "pci ready; call pci_trigger to plan");
}

void PciNode::onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  std::unique_lock<std::mutex> lock(mutex_);
  have_odometry_ = true;
  current_pose_ = msg->pose.pose;

  if (!running_ || !path_in_progress_ || planning_in_progress_) {
    return;
  }

  const double dx = current_pose_.position.x - goal_pose_.position.x;
  const double dy = current_pose_.position.y - goal_pose_.position.y;
  const double dist_to_goal = std::sqrt(dx * dx + dy * dy);

  // Update progress tracking if the robot has moved noticeably
  const double moved_dx = current_pose_.position.x - last_progress_pos_.x;
  const double moved_dy = current_pose_.position.y - last_progress_pos_.y;
  if (std::sqrt(moved_dx * moved_dx + moved_dy * moved_dy) > 0.1) {
    last_progress_pos_ = current_pose_.position;
    last_progress_time_ = now();
  }

  if (dist_to_goal <= reach_distance_) {
    RCLCPP_INFO(get_logger(),
                "goal reached (dist=%.2f m <= %.2f m); requesting next plan",
                dist_to_goal, reach_distance_);
    path_in_progress_ = false;
    lock.unlock();
    planAndPublish();
  }
}

bool PciNode::requestPlan(std::vector<geometry_msgs::msg::Pose>& path,
                          std::string& error) {
  if (!planner_client_->wait_for_service(std::chrono::seconds(2))) {
    error = "planner service 'mggplanner' is not available";
    return false;
  }
  auto request = std::make_shared<mgg_msgs::srv::PlannerSrv::Request>();
  request->header.stamp = now();
  request->header.frame_id = world_frame_;
  request->bound_mode = bound_mode_;

  auto future = planner_client_->async_send_request(request);

  const auto status = future.wait_for(
      std::chrono::duration<double>(service_timeout_sec_));
  if (status != std::future_status::ready) {
    error = "planner did not answer within " +
            std::to_string(service_timeout_sec_) +
            " s (is the executor multi-threaded?)";
    return false;
  }

  const auto response = future.get();
  path = response->path;
  return true;
}

void PciNode::publishPath(const std::vector<geometry_msgs::msg::Pose>& path) {
  nav_msgs::msg::Path msg;
  msg.header.stamp = now();
  msg.header.frame_id = world_frame_;
  for (const auto& pose : path) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = msg.header;
    stamped.pose = pose;
    msg.poses.push_back(stamped);
  }
  path_pub_->publish(msg);
}

void PciNode::planAndPublish() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_in_progress_) return;
    planning_in_progress_ = true;
  }

  std::vector<geometry_msgs::msg::Pose> path;
  std::string error;
  const bool ok = requestPlan(path, error);

  std::lock_guard<std::mutex> lock(mutex_);
  planning_in_progress_ = false;

  if (!ok) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.c_str());
    return;
  }
  if (path.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "planner returned no path");
    path_in_progress_ = false;
    return;
  }

  publishPath(path);
  goal_pose_ = path.back();
  path_in_progress_ = true;
  last_progress_pos_ = current_pose_.position;
  last_progress_time_ = now();
  path_start_time_ = now();
}

void PciNode::tick() {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!running_ || planning_in_progress_) return;

  if (!path_in_progress_) {
    // If running in auto mode and not following any path, trigger planning
    lock.unlock();
    planAndPublish();
    return;
  }

  // Path is active: check if stuck or timed out
  if (last_progress_time_.nanoseconds() > 0) {
    const double stuck_duration = (now() - last_progress_time_).seconds();
    if (stuck_duration > stuck_timeout_sec_) {
      RCLCPP_WARN(get_logger(),
                  "robot made no progress for %.1f s (> %.1f s); replanning",
                  stuck_duration, stuck_timeout_sec_);
      path_in_progress_ = false;
      lock.unlock();
      planAndPublish();
    }
  }
}

void PciNode::onTrigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = true;
    path_in_progress_ = false;
  }
  planAndPublish();

  std::lock_guard<std::mutex> lock(mutex_);
  response->success = path_in_progress_;
  response->message = path_in_progress_
                          ? "started autonomous exploration"
                          : "failed to start path";
}

void PciNode::onStop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    path_in_progress_ = false;
  }
  publishPath({});
  response->success = true;
  response->message = "stopped";
  RCLCPP_INFO(get_logger(), "stopped");
}


}  // namespace mgg_pci
