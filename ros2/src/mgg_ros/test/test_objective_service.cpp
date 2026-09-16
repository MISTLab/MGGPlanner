#include <chrono>
#include <atomic>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <mgg_msgs/srv/plan_objective.hpp>
#include <mgg_msgs/msg/mapping_snapshot.hpp>
#include <mgg_msgs/srv/query_map_batch.hpp>

#include "mgg_ros/planner_node.h"

namespace mgg_ros {

class PlannerNodeTestPeer {
 public:
  static void acceptSnapshot(PlannerNode& node,
                             const mgg_msgs::msg::MappingSnapshot& snapshot) {
    node.onMappingSnapshot(
        std::make_shared<mgg_msgs::msg::MappingSnapshot>(snapshot));
  }

  static bool query(PlannerNode& node, mgg::FeasiblePath& path) {
    return node.queryIndexedMap(path);
  }

  static bool queryReady(const PlannerNode& node) {
    return node.indexed_map_client_ && node.indexed_map_client_->service_is_ready();
  }

  static bool hasQueryClient(const PlannerNode& node) {
    return static_cast<bool>(node.indexed_map_client_);
  }

  static void expireSnapshot(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.mapping_snapshot_received_ =
        std::chrono::steady_clock::now() - std::chrono::seconds(10);
  }
};

}  // namespace mgg_ros

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

TEST_F(ObjectiveService, AuthorityBindingDoesNotRequireIndexedQuery) {
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::hasQueryClient(*planner));
  mgg_msgs::msg::MappingSnapshot snapshot;
  snapshot.component_id = "component-a";
  snapshot.epoch = 7;
  snapshot.graph_revision = 8;
  snapshot.geometry_revision = std::string(64, 'a');
  snapshot.source_stamp.sec = 3;
  snapshot.component_from_navigation.rotation.w = 1.0;
  mgg_ros::PlannerNodeTestPeer::acceptSnapshot(*planner, snapshot);

  const auto request = [&snapshot]() {
    auto value = std::make_shared<Service::Request>();
    value->objective = Service::Request::RETURN_HOME;
    value->component_id = snapshot.component_id;
    value->goal.orientation.w = 1.0;
    value->map_epoch = snapshot.epoch;
    value->mapping_graph_revision = snapshot.graph_revision;
    value->geometry_revision = snapshot.geometry_revision;
    value->map_source_stamp = snapshot.source_stamp;
    return value;
  };

  auto response = call(request());
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED);

  auto wrong_component = request();
  wrong_component->component_id = "component-b";
  response = call(wrong_component);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::STALE_REVISION);

  auto wrong_graph = request();
  ++wrong_graph->mapping_graph_revision;
  response = call(wrong_graph);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::STALE_REVISION);

  auto wrong_geometry = request();
  wrong_geometry->geometry_revision = std::string(64, 'b');
  response = call(wrong_geometry);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::STALE_REVISION);

  mgg_ros::PlannerNodeTestPeer::expireSnapshot(*planner);
  response = call(request());
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::STALE_REVISION);
}

TEST(IndexedObjectiveService, MissingSnapshotFailsClosedWithoutNestedSpin) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("indexed_map_query_service",
                                    "/robot_1/mapping/query_batch");
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  auto caller = std::make_shared<rclcpp::Node>("indexed_objective_test_client");
  auto client = caller->create_client<Service>("plan_objective");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(planner);
  std::thread spinner([&executor]() { executor.spin(); });
  const bool ready = client->wait_for_service(3s);
  EXPECT_TRUE(ready);
  if (!ready) {
    executor.cancel();
    spinner.join();
    return;
  }

  auto request = std::make_shared<Service::Request>();
  request->objective = Service::Request::NAVIGATE;
  request->component_id = "component-a";
  request->goal.orientation.w = 1.0;
  request->map_epoch = 1;
  request->mapping_graph_revision = 2;
  request->geometry_revision = std::string(64, 'a');
  request->map_source_stamp.sec = 3;
  auto future = client->async_send_request(request);
  const auto result = rclcpp::spin_until_future_complete(caller, future, 3s);
  EXPECT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  if (result == rclcpp::FutureReturnCode::SUCCESS) {
    const auto response = future.get();
    EXPECT_NE(response, nullptr);
    if (response) {
      EXPECT_EQ(response->status, Service::Response::STALE_REVISION);
    }
  }

  executor.cancel();
  spinner.join();
}

TEST(IndexedObjectiveService, BatchedQueryIsBoundedAndUsesComponentFrame) {
  using Query = mgg_msgs::srv::QueryMapBatch;
  rclcpp::NodeOptions options;
  options.append_parameter_override("indexed_map_query_service",
                                    "/robot_1/mapping/query_batch");
  options.append_parameter_override("indexed_map_query_timeout_s", 0.05);
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  auto server = std::make_shared<rclcpp::Node>("indexed_query_test_server");
  std::atomic<bool> delay{false};
  std::vector<geometry_msgs::msg::Point> received;
  auto service = server->create_service<Query>(
      "/robot_1/mapping/query_batch",
      [&delay, &received](const Query::Request::SharedPtr request,
                          Query::Response::SharedPtr response) {
        received = request->samples;
        if (delay.load()) std::this_thread::sleep_for(200ms);
        response->status = Query::Response::OK;
        response->component_id = request->component_id;
        response->epoch = request->epoch;
        response->graph_revision = request->graph_revision;
        response->geometry_revision = request->geometry_revision;
        const auto n = request->samples.size();
        response->occupancy.assign(n, Query::Response::FREE);
        response->ground_z.assign(n, 0.0);
        response->roughness.assign(n, 0.0);
        response->clearance.assign(n, 100.0);
        response->step.assign(n, false);
        response->drop.assign(n, false);
      });
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions{}, 3);
  executor.add_node(planner);
  executor.add_node(server);
  std::thread spinner([&executor]() { executor.spin(); });
  const auto discovery_deadline = std::chrono::steady_clock::now() + 3s;
  while (!mgg_ros::PlannerNodeTestPeer::queryReady(*planner) &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::queryReady(*planner));

  mgg_msgs::msg::MappingSnapshot snapshot;
  snapshot.component_id = "component-a";
  snapshot.epoch = 7;
  snapshot.graph_revision = 8;
  snapshot.geometry_revision = std::string(64, 'a');
  const auto stamp = planner->now();
  snapshot.source_stamp.sec = static_cast<std::int32_t>(stamp.seconds());
  snapshot.source_stamp.nanosec =
      static_cast<std::uint32_t>(stamp.nanoseconds() % 1000000000LL);
  snapshot.component_from_navigation.translation.x = 10.0;
  snapshot.component_from_navigation.rotation.w = 1.0;
  mgg_ros::PlannerNodeTestPeer::acceptSnapshot(*planner, snapshot);

  mgg::FeasiblePath path;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.component_id = snapshot.component_id;
  path.map_epoch = snapshot.epoch;
  path.mapping_graph_revision = snapshot.graph_revision;
  path.geometry_revision = snapshot.geometry_revision;
  path.map_source_stamp_sec = snapshot.source_stamp.sec;
  path.map_source_stamp_nanosec = snapshot.source_stamp.nanosec;
  path.poses.push_back(mgg::StateVec(1.0, 0.0, 0.0, 0.0));
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_GE(received.size(), 2u);
  if (received.size() >= 2) {
    EXPECT_NEAR(received.front().x, 10.0, 1e-6);
    EXPECT_NEAR(received.back().x, 11.0, 1e-6);
  }

  delay = true;
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_EQ(path.status, mgg::PlanningStatus::kBlocked);
  executor.cancel();
  spinner.join();
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
