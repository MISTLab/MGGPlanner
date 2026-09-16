#include "mgg_pci/pci_node.h"

#include <chrono>
#include <limits>

namespace mgg_pci {

double boundedRetryDelaySeconds(int attempt, double initial, double maximum) {
  double delay = initial;
  for (int n = 1; n < attempt && delay < maximum; ++n) {
    delay = std::min(maximum, delay * 2.0);
  }
  return delay;
}

PciNode::PciNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mgg_pci", options),
      last_progress_time_(0, 0, RCL_ROS_TIME),
      path_start_time_(0, 0, RCL_ROS_TIME),
      retry_not_before_(0, 0, RCL_ROS_TIME) {
  service_timeout_sec_ =
      declare_parameter("service_timeout_sec", service_timeout_sec_);
  bound_mode_ = static_cast<int>(declare_parameter("bound_mode", 0));
  world_frame_ = declare_parameter("world_frame", world_frame_);
  reach_distance_ = declare_parameter("reach_distance", reach_distance_);
  stuck_timeout_sec_ = declare_parameter("stuck_timeout_sec", stuck_timeout_sec_);
  bootstrap_distance_ = declare_parameter("bootstrap_distance", bootstrap_distance_);
  external_path_execution_ =
      declare_parameter("external_path_execution", external_path_execution_);
  empty_plan_retry_initial_sec_ = std::max(
      0.1, declare_parameter("empty_plan_retry_initial_sec",
                             empty_plan_retry_initial_sec_));
  empty_plan_retry_max_sec_ = std::max(
      empty_plan_retry_initial_sec_,
      declare_parameter("empty_plan_retry_max_sec", empty_plan_retry_max_sec_));
  max_empty_plans_before_stop_ =
      declare_parameter("max_empty_plans_before_stop", max_empty_plans_before_stop_);
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

  replan_srv_ = create_service<std_srvs::srv::Trigger>(
      "pci_replan",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        onReplan(req, res);
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
  status_pub_ = create_publisher<std_msgs::msg::String>(
      "status", rclcpp::QoS(1).transient_local());
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "command_path", rclcpp::QoS(1).transient_local());

  if (auto_period > 0.0) {
    auto_timer_ = create_timer(std::chrono::duration<double>(auto_period),
                               [this]() { tick(); }, callback_group_);
    RCLCPP_INFO(get_logger(), "auto watchdog: checking progress every %.1f s",
                auto_period);
  }
  RCLCPP_INFO(get_logger(), "pci ready; call pci_trigger to plan");
  if (external_path_execution_) {
    RCLCPP_INFO(get_logger(),
                "external path execution enabled; FollowPath owns arrival and watchdog decisions");
  }
}

bool PciNode::executeBootstrap() {
  if (!have_odometry_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                         "cannot bootstrap: no odometry received yet");
    return false;
  }
  const double qx = current_pose_.orientation.x;
  const double qy = current_pose_.orientation.y;
  const double qz = current_pose_.orientation.z;
  const double qw = current_pose_.orientation.w;
  const double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                1.0 - 2.0 * (qy * qy + qz * qz));

  const double x0 = current_pose_.position.x;
  const double y0 = current_pose_.position.y;
  const double z0 = current_pose_.position.z;

  std::vector<geometry_msgs::msg::Pose> path;
  for (double fraction : {0.33, 0.66, 1.0}) {
    geometry_msgs::msg::Pose p;
    p.position.x = x0 + bootstrap_distance_ * fraction * std::cos(yaw);
    p.position.y = y0 + bootstrap_distance_ * fraction * std::sin(yaw);
    p.position.z = z0;
    p.orientation = current_pose_.orientation;
    path.push_back(p);
  }

  has_bootstrapped_ = true;
  publishPath(path);
  publishStatus("exploring");
  goal_pose_ = path.back();
  path_in_progress_ = true;
  last_progress_pos_ = current_pose_.position;
  last_progress_time_ = now();
  path_start_time_ = now();

  RCLCPP_INFO(get_logger(),
              "bootstrapping: driving forward %.1f m (heading %.0f deg) to sweep local terrain",
              bootstrap_distance_, yaw * 180.0 / M_PI);
  return true;
}

void PciNode::onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  std::unique_lock<std::mutex> lock(mutex_);
  have_odometry_ = true;
  current_pose_ = msg->pose.pose;

  if (!running_ || !path_in_progress_ || planning_in_progress_ ||
      external_path_execution_) {
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
    stall_budget_.noteProgress();
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
    planner_client_->remove_pending_request(future);
    error = "planner did not answer within " +
            std::to_string(service_timeout_sec_) +
            " s (is the executor multi-threaded?)";
    return false;
  }

  const auto response = future.get();
  plan_status_ = response->status;
  path = response->path;
  return true;
}

void PciNode::publishStatus(const std::string& state) {
  std_msgs::msg::String msg;
  msg.data = "{\"state\":\"" + state + "\",\"stamp_ns\":" +
             std::to_string(now().nanoseconds()) + "}";
  status_pub_->publish(msg);
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

bool PciNode::pathEndpointMakesExternalProgress(
    const std::vector<geometry_msgs::msg::Pose>& path) const {
  const auto& endpoint = path.back();
  const double dx = endpoint.position.x - current_pose_.position.x;
  const double dy = endpoint.position.y - current_pose_.position.y;
  return std::hypot(dx, dy) > reach_distance_;
}

void PciNode::deferExternalRetry(const std::string& reason) {
  if (consecutive_empty_plans_ < std::numeric_limits<int>::max()) {
    ++consecutive_empty_plans_;
  }
  const double delay = boundedRetryDelaySeconds(
      consecutive_empty_plans_, empty_plan_retry_initial_sec_,
      empty_plan_retry_max_sec_);
  path_in_progress_ = false;
  waiting_for_plan_ = true;
  retry_not_before_ = now() + rclcpp::Duration::from_seconds(delay);
  publishStatus("waiting");
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                       "%s; retrying in %.1f s", reason.c_str(), delay);
}

void PciNode::planAndPublish() {
  if (exploration_completed_) {
    return;
  }
  uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_in_progress_ || !running_) return;
    generation = generation_;
    planning_in_progress_ = true;
  }

  std::vector<geometry_msgs::msg::Pose> path;
  std::string error;
  const bool ok = requestPlan(path, error);

  std::lock_guard<std::mutex> lock(mutex_);
  planning_in_progress_ = false;
  if (!running_ || generation != generation_) return;

  if (!ok) {
    if (!has_bootstrapped_ && bootstrap_distance_ > 0.0) {
      executeBootstrap();
      return;
    }
    if (external_path_execution_) {
      deferExternalRetry(error);
      return;
    }
    if (consecutive_empty_plans_ < std::numeric_limits<int>::max()) {
      ++consecutive_empty_plans_;
    }
    if (consecutive_empty_plans_ >= max_empty_plans_before_stop_) {
      running_ = false;
      path_in_progress_ = false;
      publishPath({});
      publishStatus("blocked");
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.c_str());
    return;
  }

  const bool insufficient_external_progress =
      external_path_execution_ && have_odometry_ && !path.empty() &&
      !pathEndpointMakesExternalProgress(path);
  if (path.empty() || insufficient_external_progress) {
    if (!has_bootstrapped_ && bootstrap_distance_ > 0.0) {
      // First attempt on standing start without mapped ground: bootstrap forward
      executeBootstrap();
      return;
    }

    if (external_path_execution_) {
      deferExternalRetry(insufficient_external_progress
                             ? "planner path makes no progress beyond the controller goal tolerance"
                             : "planner returned no path");
      return;
    }
    if (consecutive_empty_plans_ < std::numeric_limits<int>::max()) {
      ++consecutive_empty_plans_;
    }
    if (consecutive_empty_plans_ >= max_empty_plans_before_stop_) {
      exploration_completed_ = true;
      running_ = false;
      path_in_progress_ = false;
      publishPath({});
      publishStatus(plan_status_ == -3 ? "complete" : "blocked");
      RCLCPP_INFO(get_logger(),
                  "Exploration stopped: no reachable plan remains");
      return;
    }

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "planner returned no path (attempt %d/%d)",
                         consecutive_empty_plans_, max_empty_plans_before_stop_);
    path_in_progress_ = false;
    return;
  }

  consecutive_empty_plans_ = 0;
  waiting_for_plan_ = false;
  retry_not_before_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  has_bootstrapped_ = true;
  publishPath(path);
  publishStatus("exploring");
  goal_pose_ = path.back();
  path_in_progress_ = true;
  last_progress_pos_ = current_pose_.position;
  last_progress_time_ = now();
  path_start_time_ = now();
}

void PciNode::tick() {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!running_ || planning_in_progress_ || exploration_completed_) return;

  if (!path_in_progress_) {
    if (external_path_execution_ && waiting_for_plan_ &&
        now() < retry_not_before_) {
      return;
    }
    // If running in auto mode and not following any path, trigger planning
    lock.unlock();
    planAndPublish();
    return;
  }

  // FollowPath owns arrival and controller progress in this mode. PCI waits
  // for an explicit pci_replan after the action reaches a terminal result.
  if (external_path_execution_) return;

  // Path is active: check if stuck or timed out
  if (last_progress_time_.nanoseconds() > 0) {
    const double stuck_duration = (now() - last_progress_time_).seconds();
    if (stuck_duration > stuck_timeout_sec_ ||
        (now() - path_start_time_).seconds() > 6.0 * stuck_timeout_sec_) {
      RCLCPP_WARN(get_logger(),
                  "robot made no progress for %.1f s (> %.1f s); replanning",
                  stuck_duration, stuck_timeout_sec_);
      path_in_progress_ = false;
      if (stall_budget_.noteStall()) {
        running_ = false;
        publishPath({});
        publishStatus("blocked");
        return;
      }
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
    ++generation_;
    stall_budget_.noteProgress();
    publishStatus("starting");
    exploration_completed_ = false;
    consecutive_empty_plans_ = 0;
    path_in_progress_ = false;
    waiting_for_plan_ = false;
    retry_not_before_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  planAndPublish();

  std::lock_guard<std::mutex> lock(mutex_);
  response->success = running_ || exploration_completed_;
  response->message = path_in_progress_
                          ? "started autonomous exploration"
                          : waiting_for_plan_
                                ? "exploration active; waiting for a path"
                                : "failed to start path";
}

void PciNode::onReplan(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
      response->success = false;
      response->message = "exploration is not active";
      return;
    }
    if (planning_in_progress_) {
      response->success = false;
      response->message = "a planning cycle is already running";
      return;
    }
    exploration_completed_ = false;
    if (!waiting_for_plan_) consecutive_empty_plans_ = 0;
    path_in_progress_ = false;
    waiting_for_plan_ = false;
    retry_not_before_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  planAndPublish();

  std::lock_guard<std::mutex> lock(mutex_);
  response->success = path_in_progress_ ||
                      (external_path_execution_ && running_ && waiting_for_plan_);
  response->message = path_in_progress_
                          ? "published a fresh exploration path"
                          : waiting_for_plan_
                                ? "exploration active; waiting for a path"
                                : "failed to produce a fresh path";
}


void PciNode::onStop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    ++generation_;
    path_in_progress_ = false;
    waiting_for_plan_ = false;
  }
  publishPath({});
  publishStatus("stopped");
  response->success = true;
  response->message = "stopped";
  RCLCPP_INFO(get_logger(), "stopped");
}


}  // namespace mgg_pci
