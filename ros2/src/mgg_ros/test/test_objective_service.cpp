#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <mgg_msgs/srv/plan_objective.hpp>

#include "mgg_ros/planner_node.h"

namespace {

using namespace std::chrono_literals;
using Service = mgg_msgs::srv::PlanObjective;

class ObjectiveService : public ::testing::Test {
 protected:
  void SetUp() override {
    planner = std::make_shared<mgg_ros::PlannerNode>(rclcpp::NodeOptions{});
    caller = std::make_shared<rclcpp::Node>("objective_service_test_client");
    client = caller->create_client<Service>("plan_objective");
    executor.add_node(planner);
    spinner = std::thread([this]() { executor.spin(); });
    ASSERT_TRUE(client->wait_for_service(3s));
  }

  void TearDown() override {
    executor.cancel();
    spinner.join();
  }

  Service::Response::SharedPtr call(Service::Request::SharedPtr request) {
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(caller, future, 3s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return nullptr;
    }
    return future.get();
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<mgg_ros::PlannerNode> planner;
  std::shared_ptr<rclcpp::Node> caller;
  rclcpp::Client<Service>::SharedPtr client;
  std::thread spinner;
};

TEST_F(ObjectiveService, ValidatesBeforeTouchingUnavailableMap) {
  auto invalid = std::make_shared<Service::Request>();
  invalid->objective = 255;
  auto response = call(invalid);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::UNSUPPORTED_OBJECTIVE);

  auto wrong_component = std::make_shared<Service::Request>();
  wrong_component->objective = Service::Request::NAVIGATE;
  wrong_component->component_id = "another-component";
  wrong_component->goal.orientation.w = 1.0;
  response = call(wrong_component);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::STALE_REVISION);

  auto valid = std::make_shared<Service::Request>();
  valid->objective = Service::Request::NAVIGATE;
  valid->goal.orientation.w = 1.0;
  response = call(valid);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
