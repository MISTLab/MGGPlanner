// Demonstrates why the executor choice is a requirement, not a preference.
//
// Section 6 of ROS2_PORT_PLAN.md flagged that the ROS 1 control interface
// calls the planner's service synchronously from inside its own callback, and
// that this pattern deadlocks under ROS 2's SingleThreadedExecutor. This test
// exercises the shape directly rather than asserting it in a comment: the same
// code deadlocks on one executor and succeeds on the other.

#include <chrono>
#include <atomic>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <mgg_msgs/srv/planner_srv.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mgg_pci/pci_node.h"

namespace {

using namespace std::chrono_literals;

/// Stands in for the planner: answers "inner" immediately.
class Inner : public rclcpp::Node {
 public:
  Inner() : rclcpp::Node("inner") {
    srv_ = create_service<std_srvs::srv::Trigger>(
        "inner",
        [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
           std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          res->success = true;
        });
  }
 private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};

/// Stands in for the control interface: its service handler calls "inner" and
/// blocks on the answer, which is the pattern under test.
class Outer : public rclcpp::Node {
 public:
  explicit Outer(bool reentrant) : rclcpp::Node("outer") {
    if (reentrant) {
      group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    }
    client_ = create_client<std_srvs::srv::Trigger>(
        "inner", rclcpp::ServicesQoS(), group_);
    srv_ = create_service<std_srvs::srv::Trigger>(
        "outer",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          auto fut = client_->async_send_request(
              std::make_shared<std_srvs::srv::Trigger::Request>());
          // Blocking wait from inside a service callback.
          res->success = fut.wait_for(3s) == std::future_status::ready;
          inner_answered = res->success;
        },
        rclcpp::ServicesQoS(), group_);
  }
  bool inner_answered = false;
 private:
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};

/// Drives one "outer" call under the given executor and reports whether the
/// nested call completed.
template <typename ExecutorT>
bool runNestedCall(bool reentrant) {
  auto inner = std::make_shared<Inner>();
  auto outer = std::make_shared<Outer>(reentrant);
  auto caller = std::make_shared<rclcpp::Node>("caller");
  auto client = caller->create_client<std_srvs::srv::Trigger>("outer");

  ExecutorT executor;
  executor.add_node(inner);
  executor.add_node(outer);
  std::thread spinner([&executor]() { executor.spin(); });

  bool answered = false;
  if (client->wait_for_service(3s)) {
    auto fut = client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    // The caller spins its own node, so it is never the bottleneck.
    if (rclcpp::spin_until_future_complete(caller, fut, 6s) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      answered = fut.get()->success;
    }
  }
  executor.cancel();
  spinner.join();
  return answered && outer->inner_answered;
}

// THE TRAP: one executor thread is already inside the outer callback, so the
// inner response can never be delivered and the wait times out.
TEST(PciExecutor, SingleThreadedExecutorDeadlocksOnANestedServiceCall) {
  EXPECT_FALSE(runNestedCall<rclcpp::executors::SingleThreadedExecutor>(false));
}

// THE FIX: reentrant callback group plus threads to run it, which is what
// PciNode and its main() use.
TEST(PciExecutor, MultiThreadedExecutorWithAReentrantGroupSucceeds) {
  EXPECT_TRUE(runNestedCall<rclcpp::executors::MultiThreadedExecutor>(true));
}

TEST(PciWatchdog, ThreeConsecutiveStallsExhaustTheBudget) {
  mgg_pci::StallBudget budget(3);
  EXPECT_FALSE(budget.noteStall());
  EXPECT_FALSE(budget.noteStall());
  EXPECT_TRUE(budget.noteStall());
  EXPECT_EQ(budget.count(), 3);
}

TEST(PciWatchdog, ProgressResetsConsecutiveStalls) {
  mgg_pci::StallBudget budget(3);
  EXPECT_FALSE(budget.noteStall());
  EXPECT_FALSE(budget.noteStall());
  budget.noteProgress();
  EXPECT_EQ(budget.count(), 0);
  EXPECT_FALSE(budget.noteStall());
}

TEST(PciRetry, EmptyPlanDelayUsesBoundedExponentialBackoff) {
  EXPECT_DOUBLE_EQ(mgg_pci::boundedRetryDelaySeconds(1, 1.0, 10.0), 1.0);
  EXPECT_DOUBLE_EQ(mgg_pci::boundedRetryDelaySeconds(2, 1.0, 10.0), 2.0);
  EXPECT_DOUBLE_EQ(mgg_pci::boundedRetryDelaySeconds(3, 1.0, 10.0), 4.0);
  EXPECT_DOUBLE_EQ(mgg_pci::boundedRetryDelaySeconds(4, 1.0, 10.0), 8.0);
  EXPECT_DOUBLE_EQ(mgg_pci::boundedRetryDelaySeconds(5, 1.0, 10.0), 10.0);
  EXPECT_DOUBLE_EQ(mgg_pci::boundedRetryDelaySeconds(40, 1.0, 10.0), 10.0);
}

class FakePlanner : public rclcpp::Node {
 public:
  FakePlanner(const std::string& ns,
              std::vector<std::vector<double>> path_x)
      : rclcpp::Node(
            "fake_planner",
            rclcpp::NodeOptions().arguments(
                {"--ros-args", "-r", "__ns:=" + ns})),
        path_x_(std::move(path_x)) {
    service_ = create_service<mgg_msgs::srv::PlannerSrv>(
        "mggplanner",
        [this](const std::shared_ptr<mgg_msgs::srv::PlannerSrv::Request>,
               std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response) {
          const int call = calls_.fetch_add(1);
          response->status = -3;
          const auto& points = path_x_.at(
              std::min(static_cast<size_t>(call), path_x_.size() - 1));
          for (double x : points) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = x;
            pose.orientation.w = 1.0;
            response->path.push_back(pose);
          }
        });
  }

  int calls() const { return calls_.load(); }

 private:
  std::atomic<int> calls_{0};
  std::vector<std::vector<double>> path_x_;
  rclcpp::Service<mgg_msgs::srv::PlannerSrv>::SharedPtr service_;
};

struct ExternalExecutionRig {
  explicit ExternalExecutionRig(const std::string& ns,
                                std::vector<std::vector<double>> path_x)
      : planner(std::make_shared<FakePlanner>(ns, std::move(path_x))),
        pci(std::make_shared<mgg_pci::PciNode>(
            rclcpp::NodeOptions()
                .arguments({"--ros-args", "-r", "__ns:=" + ns})
                .parameter_overrides(
                    {rclcpp::Parameter("external_path_execution", true),
                     rclcpp::Parameter("auto_period_sec", 0.0),
                     rclcpp::Parameter("bootstrap_distance", 0.0),
                     rclcpp::Parameter("service_timeout_sec", 2.0)}))),
        caller(std::make_shared<rclcpp::Node>(
            "pci_test_caller",
            rclcpp::NodeOptions().arguments(
                {"--ros-args", "-r", "__ns:=" + ns}))) {
    executor.add_node(planner);
    executor.add_node(pci);
    executor.add_node(caller);
    spinner = std::thread([this]() { executor.spin(); });
  }

  ~ExternalExecutionRig() {
    executor.cancel();
    if (spinner.joinable()) spinner.join();
  }

  std::shared_ptr<std_srvs::srv::Trigger::Response> call(
      const std::string& service_name) {
    auto client = caller->create_client<std_srvs::srv::Trigger>(service_name);
    if (!client->wait_for_service(2s)) return nullptr;
    auto future = client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(3s) != std::future_status::ready) return nullptr;
    return future.get();
  }

  void publishOdometry(double x = 0.0) {
    auto publisher = caller->create_publisher<nav_msgs::msg::Odometry>(
        "odometry", rclcpp::QoS(10));
    nav_msgs::msg::Odometry odometry;
    odometry.pose.pose.position.x = x;
    odometry.pose.pose.orientation.w = 1.0;
    for (int n = 0; n < 5; ++n) {
      publisher->publish(odometry);
      std::this_thread::sleep_for(20ms);
    }
  }

  std::shared_ptr<FakePlanner> planner;
  std::shared_ptr<mgg_pci::PciNode> pci;
  std::shared_ptr<rclcpp::Node> caller;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spinner;
};

TEST(PciExternalExecution, NearEndpointWaitsForExplicitReplan) {
  ExternalExecutionRig rig("/external_near_endpoint", {{1.0, 0.1}, {1.0}});
  rig.publishOdometry();

  const auto started = rig.call("pci_trigger");
  ASSERT_NE(started, nullptr);
  EXPECT_TRUE(started->success);
  EXPECT_NE(started->message.find("waiting for a path"), std::string::npos);
  EXPECT_EQ(rig.planner->calls(), 1);

  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(rig.planner->calls(), 1);
  const auto replanned = rig.call("pci_replan");
  ASSERT_NE(replanned, nullptr);
  EXPECT_TRUE(replanned->success);
  EXPECT_NE(replanned->message.find("published"), std::string::npos);
  EXPECT_EQ(rig.planner->calls(), 2);
}

TEST(PciExternalExecution, OdometryProximityCannotReplaceAcceptedPath) {
  ExternalExecutionRig rig("/external_arrival", {{1.0}, {2.0}});
  rig.publishOdometry();

  const auto started = rig.call("pci_trigger");
  ASSERT_NE(started, nullptr);
  ASSERT_TRUE(started->success);
  ASSERT_EQ(rig.planner->calls(), 1);

  // This is inside PCI's legacy 0.3 m reach distance. FollowPath has not yet
  // reported success, so external mode must retain the accepted path.
  rig.publishOdometry(0.9);
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(rig.planner->calls(), 1);

  const auto replanned = rig.call("pci_replan");
  ASSERT_NE(replanned, nullptr);
  EXPECT_TRUE(replanned->success);
  EXPECT_EQ(rig.planner->calls(), 2);
}

TEST(PciExternalExecution, RepeatedEmptyPlansRemainWaitingUntilManualStop) {
  ExternalExecutionRig rig("/external_empty_plan", {{}});
  rig.publishOdometry();

  for (const char* service : {"pci_trigger", "pci_replan", "pci_replan",
                              "pci_replan"}) {
    const auto response = rig.call(service);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->success);
    EXPECT_NE(response->message.find("waiting for a path"), std::string::npos);
  }
  EXPECT_EQ(rig.planner->calls(), 4);

  const auto stopped = rig.call("pci_stop");
  ASSERT_NE(stopped, nullptr);
  EXPECT_TRUE(stopped->success);
  const auto rejected = rig.call("pci_replan");
  ASSERT_NE(rejected, nullptr);
  EXPECT_FALSE(rejected->success);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
