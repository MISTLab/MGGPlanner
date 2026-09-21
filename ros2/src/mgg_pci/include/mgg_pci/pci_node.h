// Planner control interface: the thing that decides when to ask the planner
// for a path and what to do with the answer.
//
// Written fresh rather than ported. The ROS 1 role was filled by
// pci_general, which is a separate ROS 1 repository built around actionlib and
// a PCIManager abstraction; the surface actually needed is narrow (trigger,
// stop, path out) and a clean implementation serves the packaging goal better
// than a port.
//
// This is where the executor question from section 6 of ROS2_PORT_PLAN.md
// becomes concrete. The interface calls the planner's service from inside its
// own service callback. Under a SingleThreadedExecutor that deadlocks: the one
// executor thread is already occupied running the trigger callback, so it can
// never process the planner's response, and the wait times out. The callbacks
// therefore live in a reentrant group and main() spins a MultiThreadedExecutor.
// test_pci_deadlock demonstrates the failure directly.

#ifndef MGG_PCI_PCI_NODE_H_
#define MGG_PCI_PCI_NODE_H_

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mgg_msgs/srv/planner_srv.hpp>

namespace mgg_pci {

double boundedRetryDelaySeconds(int attempt, double initial, double maximum);
std::string retryStatusReason(const std::string& reason, double delay_seconds);
std::string statusJson(const std::string& state, std::int64_t stamp_ns,
                       const std::string& reason = "");

class StallBudget {
 public:
  explicit StallBudget(int limit = 3) : limit_(std::max(1, limit)) {}
  bool noteStall() { return ++count_ >= limit_; }
  void noteProgress() { count_ = 0; }
  int count() const { return count_; }

 private:
  int limit_;
  int count_ = 0;
};

class PciNode : public rclcpp::Node {
 public:
  explicit PciNode(const rclcpp::NodeOptions& options);

 private:
  void onTrigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onReplan(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
              std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void tick();

  /// Asks the planner for a path. Blocks until it answers or the timeout
  /// expires, which is only safe because of the executor arrangement above.
  bool requestPlan(std::vector<geometry_msgs::msg::Pose>& path,
                   std::string& error);

  void publishStatus(const std::string& state,
                     const std::string& reason = "");
  void publishPath(const std::vector<geometry_msgs::msg::Pose>& path);
  bool pathEndpointMakesExternalProgress(
      const std::vector<geometry_msgs::msg::Pose>& path) const;
  void deferExternalRetry(const std::string& reason);
  void planAndPublish();
  bool executeBootstrap();

  rclcpp::Client<mgg_msgs::srv::PlannerSrv>::SharedPtr planner_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replan_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr auto_timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::mutex mutex_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  /// mgg_ros::PlannerNode::kStatusComplete: nothing left to explore.
  static constexpr int kPlannerStatusComplete = -3;
  int plan_status_ = -1;
  StallBudget stall_budget_;
  uint64_t generation_ = 0;
  bool running_ = false;
  bool have_odometry_ = false;
  bool planning_in_progress_ = false;
  bool path_in_progress_ = false;
  bool waiting_for_plan_ = false;
  bool has_bootstrapped_ = false;
  bool exploration_completed_ = false;
  bool external_path_execution_ = false;
  int consecutive_empty_plans_ = 0;

  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose goal_pose_;
  geometry_msgs::msg::Point last_progress_pos_;
  rclcpp::Time last_progress_time_;
  rclcpp::Time path_start_time_;
  rclcpp::Time retry_not_before_;

  double service_timeout_sec_ = 60.0;
  double reach_distance_ = 0.3;
  double stuck_timeout_sec_ = 20.0;
  double bootstrap_distance_ = 3.0;
  double empty_plan_retry_initial_sec_ = 1.0;
  double empty_plan_retry_max_sec_ = 10.0;
  int max_empty_plans_before_stop_ = 3;
  int bound_mode_ = 0;
  std::string world_frame_ = "world";
};



}  // namespace mgg_pci

#endif  // MGG_PCI_PCI_NODE_H_
