#include "mgg_pci/pci_node.h"

#include <chrono>

namespace mgg_pci {

PciNode::PciNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mgg_pci", options) {
  service_timeout_sec_ =
      declare_parameter("service_timeout_sec", service_timeout_sec_);
  bound_mode_ = static_cast<int>(declare_parameter("bound_mode", 0));
  world_frame_ = declare_parameter("world_frame", world_frame_);
  const double auto_period = declare_parameter("auto_period_sec", 0.0);

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
    RCLCPP_INFO(get_logger(), "auto mode: replanning every %.1f s",
                auto_period);
  }
  RCLCPP_INFO(get_logger(), "pci ready; call pci_trigger to plan");
}

void PciNode::onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr) {
  have_odometry_ = true;
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

  // Blocking wait on the future from inside a service callback.
  //
  // This is exactly the shape that deadlocks on a SingleThreadedExecutor: the
  // executor thread is already inside onTrigger, so nothing is left to deliver
  // the planner's response and this waits out the full timeout. It works here
  // because the client, the services and the timer all sit in a reentrant
  // callback group and main() runs a MultiThreadedExecutor.
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

void PciNode::tick() {
  if (!running_) return;
  std::vector<geometry_msgs::msg::Pose> path;
  std::string error;
  if (!requestPlan(path, error)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.c_str());
    return;
  }
  if (path.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "planner returned no path");
    return;
  }
  publishPath(path);
}

void PciNode::onTrigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::vector<geometry_msgs::msg::Pose> path;
  std::string error;
  if (!requestPlan(path, error)) {
    response->success = false;
    response->message = error;
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return;
  }
  publishPath(path);
  running_ = true;
  response->success = !path.empty();
  response->message =
      "planner returned " + std::to_string(path.size()) + " poses";
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void PciNode::onStop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  running_ = false;
  // An empty path is the stop command to whatever is following it.
  publishPath({});
  response->success = true;
  response->message = "stopped";
  RCLCPP_INFO(get_logger(), "stopped");
}

}  // namespace mgg_pci
