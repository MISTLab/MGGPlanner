// Demonstrates why the executor choice is a requirement, not a preference.
//
// Section 6 of ROS2_PORT_PLAN.md flagged that the ROS 1 control interface
// calls the planner's service synchronously from inside its own callback, and
// that this pattern deadlocks under ROS 2's SingleThreadedExecutor. This test
// exercises the shape directly rather than asserting it in a comment: the same
// code deadlocks on one executor and succeeds on the other.

#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

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

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
