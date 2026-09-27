// The planner's authoritative configuration, independent of the map backend.
#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "mgg_ros/planner_node.h"
#include "mgg_msgs/msg/planner_config_state.hpp"

namespace mgg_ros {

using ConfigState = mgg_msgs::msg::PlannerConfigState;
using Region = mgg_msgs::srv::PlannerSetExplorationRegion;
using Target = mgg_msgs::srv::PlannerSetExplorationTarget;
using Leave = std_srvs::srv::SetBool;

class PlannerNodeTestPeer {
 public:
  // Read the fields the planner actually uses, not its publication cache.
  static ConfigState applied(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    ConfigState state;
    state.region_active = node.exploration_region_.has_value();
    if (state.region_active) {
      const auto& region = *node.exploration_region_;
      state.region_low.x = region.min_val.x();
      state.region_low.y = region.min_val.y();
      state.region_low.z = region.min_val.z();
      state.region_high.x = region.max_val.x();
      state.region_high.y = region.max_val.y();
      state.region_high.z = region.max_val.z();
    }
    state.target_active = node.exploration_target_.has_value();
    if (state.target_active) {
      state.target.x = node.exploration_target_->x();
      state.target.y = node.exploration_target_->y();
      state.target.z = node.exploration_target_->z();
    }
    state.fleet_enabled = static_cast<bool>(node.fleet_);
    state.leaving_fleet = node.fleet_ && node.fleet_->leaving();
    return state;
  }

  static void changeAndRecord(PlannerNode& node, int index, bool region,
                              std::vector<ConfigState>& history) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    if (region) {
      auto request = std::make_shared<Region::Request>();
      request->active = true;
      request->min.x = index;
      request->max.x = index + 1;
      request->max.y = 2;
      request->max.z = 3;
      auto response = std::make_shared<Region::Response>();
      node.onExplorationRegionRequest(request, response);
      EXPECT_TRUE(response->success);
    } else {
      auto request = std::make_shared<Target::Request>();
      request->active = true;
      request->target.x = index;
      request->target.y = -index;
      request->target.z = 5;
      auto response = std::make_shared<Target::Response>();
      node.onExplorationTargetRequest(request, response);
      EXPECT_TRUE(response->success);
    }
    history.push_back(applied(node));
  }

  static rclcpp::QoS configQoS(PlannerNode& node) {
    return node.planner_config_state_pub_->get_actual_qos();
  }

  static void setTarget(PlannerNode& node) {
    auto request = std::make_shared<Target::Request>();
    request->active = true;
    request->target.x = 42;
    auto response = std::make_shared<Target::Response>();
    node.onExplorationTargetRequest(request, response);
    EXPECT_TRUE(response->success);
  }

  static std::unique_lock<std::recursive_mutex> holdPlanner(PlannerNode& node) {
    return std::unique_lock<std::recursive_mutex>(node.planner_mutex_);
  }
};

namespace {

using namespace std::chrono_literals;

std::shared_ptr<PlannerNode> makeNode(const std::string& name,
                                      bool fleet_enabled = true) {
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=" + name,
                     "-r", "__ns:=/" + name + "/mgg"});
  options.parameter_overrides(
      {rclcpp::Parameter("map.backend", "mola_snapshot"),
       rclcpp::Parameter("map.mola.peer_root", "/nonexistent/config_state"),
       rclcpp::Parameter("fleet.enabled", fleet_enabled)});
  options.automatically_declare_parameters_from_overrides(true);
  return std::make_shared<PlannerNode>(options);
}

class PlannerConfigStateTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(PlannerConfigStateTest, StartupAdvertisesReliableLatchedRelativeTopic) {
  auto node = makeNode("config_topic");
  auto observer = std::make_shared<rclcpp::Node>("config_topic_observer");
  std::vector<rclcpp::TopicEndpointInfo> publishers;
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  do {
    publishers = observer->get_publishers_info_by_topic(
        "/config_topic/mgg/planner_config_state");
    if (!publishers.empty()) break;
    std::this_thread::sleep_for(10ms);
  } while (std::chrono::steady_clock::now() < deadline);
  ASSERT_EQ(publishers.size(), 1u);
  EXPECT_EQ(publishers.front().topic_type(), "mgg_msgs/msg/PlannerConfigState");
  const auto qos = publishers.front().qos_profile();
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
  EXPECT_EQ(PlannerNodeTestPeer::configQoS(*node).depth(), 1u);
}

void expectFields(const ConfigState& state, const ConfigState& expected) {
  EXPECT_EQ(state.region_active, expected.region_active);
  EXPECT_EQ(state.region_low, expected.region_low);
  EXPECT_EQ(state.region_high, expected.region_high);
  EXPECT_EQ(state.target_active, expected.target_active);
  EXPECT_EQ(state.target, expected.target);
  EXPECT_EQ(state.leaving_fleet, expected.leaving_fleet);
  EXPECT_EQ(state.fleet_enabled, expected.fleet_enabled);
}

// Construct the subscriber strictly after the planner has returned from startup.
struct Observer {
  explicit Observer(const std::shared_ptr<PlannerNode>& planner)
      : node(std::make_shared<rclcpp::Node>(
            std::string(planner->get_name()) + "_observer")) {
    subscription = node->create_subscription<ConfigState>(
        std::string(planner->get_namespace()) + "/planner_config_state",
        rclcpp::QoS(100).reliable().transient_local(),
        [this](ConfigState::ConstSharedPtr state) { states.push_back(*state); });
    executor.add_node(planner);
    executor.add_node(node);
  }

  bool waitFor(const std::function<bool()>& ready,
               std::chrono::milliseconds timeout = 3s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
      executor.spin_some();
      if (ready()) return true;
      std::this_thread::sleep_for(1ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return ready();
  }

  template <typename Service>
  std::shared_ptr<typename Service::Response> call(
      const std::string& name, const typename Service::Request& request) {
    auto client = node->create_client<Service>(name);
    if (!client->wait_for_service(3s)) {
      ADD_FAILURE() << "service not available: " << name;
      return std::make_shared<typename Service::Response>();
    }
    auto future = client->async_send_request(
        std::make_shared<typename Service::Request>(request));
    if (executor.spin_until_future_complete(future, 3s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      ADD_FAILURE() << "service timed out: " << name;
      return std::make_shared<typename Service::Response>();
    }
    return future.get();
  }

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<ConfigState>::SharedPtr subscription;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<ConfigState> states;
};

TEST_F(PlannerConfigStateTest, LateSubscriberReceivesInitialState) {
  auto planner = makeNode("config_startup");
  Observer observer(planner);
  ASSERT_TRUE(observer.waitFor([&] { return !observer.states.empty(); }));
  ASSERT_EQ(observer.states.size(), 1u);
  const auto& state = observer.states.back();
  EXPECT_NE(state.incarnation, 0u);
  EXPECT_EQ(state.generation, 1u);
  ConfigState expected;
  expected.fleet_enabled = true;
  expectFields(state, expected);
  EXPECT_GT(rclcpp::Time(state.stamp).nanoseconds(), 0);
  EXPECT_LE(rclcpp::Time(state.stamp), planner->now());
  expectFields(state, PlannerNodeTestPeer::applied(*planner));
}

TEST_F(PlannerConfigStateTest, SettersPublishAppliedValuesAndNoOpsKeepVersionAndStamp) {
  auto planner = makeNode("config_setters");
  Observer observer(planner);
  ASSERT_TRUE(observer.waitFor([&] { return !observer.states.empty(); }));
  const auto incarnation = observer.states.back().incarnation;
  ConfigState expected;
  expected.fleet_enabled = true;
  std::uint64_t generation = 1;
  const std::string prefix = planner->get_namespace();

  auto check = [&](const auto& request, const std::string& service,
                   bool changed) {
    const auto before = observer.states.back();
    const auto count = observer.states.size();
    using Service = std::conditional_t<
        std::is_same_v<std::decay_t<decltype(request)>, Region::Request>,
        Region, Target>;
    ASSERT_TRUE(observer.call<Service>(prefix + "/" + service, request)->success);
    ASSERT_TRUE(observer.waitFor([&] { return observer.states.size() > count; }));
    if (changed) ++generation;
    const auto& state = observer.states.back();
    EXPECT_EQ(state.generation, generation);
    EXPECT_EQ(state.incarnation, incarnation);
    expectFields(state, expected);
    expectFields(state, PlannerNodeTestPeer::applied(*planner));
    if (changed) {
      EXPECT_GT(rclcpp::Time(state.stamp), rclcpp::Time(before.stamp));
      EXPECT_LE(rclcpp::Time(state.stamp), planner->now());
    } else {
      EXPECT_EQ(state, before);
    }
  };

  Region::Request region;
  region.active = true;
  region.min.x = -1; region.min.y = -2; region.min.z = -3;
  region.max.x = 4; region.max.y = 5; region.max.z = 6;
  expected.region_active = true;
  expected.region_low = region.min;
  expected.region_high = region.max;
  check(region, "set_exploration_region", true);
  check(region, "set_exploration_region", false);
  region.max.z = 7;
  expected.region_high = region.max;
  check(region, "set_exploration_region", true);

  Target::Request target;
  target.active = true;
  target.target.x = 8; target.target.y = 9; target.target.z = 10;
  expected.target_active = true;
  expected.target = target.target;
  check(target, "set_exploration_target", true);
  check(target, "set_exploration_target", false);
  target.target.y = 11;
  expected.target = target.target;
  check(target, "set_exploration_target", true);

  // Ignored point payloads in clear requests do not become applied values.
  region.active = false;
  expected.region_active = false;
  expected.region_low = geometry_msgs::msg::Point();
  expected.region_high = geometry_msgs::msg::Point();
  check(region, "set_exploration_region", true);
  region.min.x = 999;
  check(region, "set_exploration_region", false);
  target.active = false;
  expected.target_active = false;
  expected.target = geometry_msgs::msg::Point();
  check(target, "set_exploration_target", true);
  target.target.x = 999;
  check(target, "set_exploration_target", false);
}

TEST_F(PlannerConfigStateTest, LeaveRejoinAndTheirNoOpsReflectTheCoordinator) {
  auto planner = makeNode("config_leave");
  Observer observer(planner);
  ASSERT_TRUE(observer.waitFor([&] { return !observer.states.empty(); }));
  const std::vector<bool> leaving{true, true, false, false};
  const std::vector<std::uint64_t> generations{2, 2, 3, 3};
  for (std::size_t i = 0; i < leaving.size(); ++i) {
    Leave::Request request;
    request.data = leaving[i];
    const auto before = observer.states.back();
    const auto count = observer.states.size();
    ASSERT_TRUE(observer.call<Leave>(
        "/config_leave/mgg/leave_fleet", request)->success);
    ASSERT_TRUE(observer.waitFor([&] { return observer.states.size() > count; }));
    const auto& state = observer.states.back();
    EXPECT_EQ(state.generation, generations[i]);
    EXPECT_EQ(state.incarnation, before.incarnation);
    EXPECT_TRUE(state.fleet_enabled);
    EXPECT_EQ(state.leaving_fleet, leaving[i]);
    expectFields(state, PlannerNodeTestPeer::applied(*planner));
    if (i % 2 == 1) EXPECT_EQ(state, before);
  }
}

TEST_F(PlannerConfigStateTest, FleetOffIsLatchedAndRefusedCallsPublishNothing) {
  auto planner = makeNode("config_fleet_off", false);
  Observer observer(planner);
  ASSERT_TRUE(observer.waitFor([&] { return !observer.states.empty(); }));
  expectFields(observer.states.back(), ConfigState());
  EXPECT_EQ(observer.states.back().generation, 1u);
  for (bool leave : {true, false}) {
    Leave::Request request;
    request.data = leave;
    EXPECT_FALSE(observer.call<Leave>(
        "/config_fleet_off/mgg/leave_fleet", request)->success);
  }
  EXPECT_FALSE(observer.waitFor([&] { return observer.states.size() > 1; }, 100ms));
}

TEST_F(PlannerConfigStateTest, InvalidSettersDoNotAdvanceOrPublishState) {
  auto planner = makeNode("config_invalid");
  Observer observer(planner);
  ASSERT_TRUE(observer.waitFor([&] { return !observer.states.empty(); }));
  Region::Request region;
  region.active = true;  // zero-volume region is invalid
  EXPECT_FALSE(observer.call<Region>(
      "/config_invalid/mgg/set_exploration_region", region)->success);
  Target::Request target;
  target.active = true;
  target.target.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(observer.call<Target>(
      "/config_invalid/mgg/set_exploration_target", target)->success);
  EXPECT_FALSE(observer.waitFor([&] { return observer.states.size() > 1; }, 100ms));
  expectFields(observer.states.back(), PlannerNodeTestPeer::applied(*planner));
}

TEST_F(PlannerConfigStateTest, TwoPlannersHaveDistinctNonzeroIncarnations) {
  auto first = makeNode("config_first");
  auto second = makeNode("config_second");
  Observer first_observer(first);
  Observer second_observer(second);
  ASSERT_TRUE(first_observer.waitFor([&] { return !first_observer.states.empty(); }));
  ASSERT_TRUE(second_observer.waitFor([&] { return !second_observer.states.empty(); }));
  const auto a = first_observer.states.back().incarnation;
  const auto b = second_observer.states.back().incarnation;
  EXPECT_NE(a, 0u);
  EXPECT_NE(b, 0u);
  EXPECT_NE(a, b);
}

TEST_F(PlannerConfigStateTest, SetterCannotPublishBeforeAcquiringPlannerMutex) {
  auto planner = makeNode("config_locked");
  Observer observer(planner);
  ASSERT_TRUE(observer.waitFor([&] { return !observer.states.empty(); }));
  auto lock = PlannerNodeTestPeer::holdPlanner(*planner);
  std::promise<void> started;
  auto entered = started.get_future();
  auto setter = std::async(std::launch::async, [&] {
    started.set_value();
    PlannerNodeTestPeer::setTarget(*planner);
  });
  entered.wait();
  EXPECT_FALSE(observer.waitFor([&] { return observer.states.size() > 1; }, 100ms));
  EXPECT_EQ(setter.wait_for(0ms), std::future_status::timeout);
  lock.unlock();
  setter.get();
  ASSERT_TRUE(observer.waitFor([&] { return observer.states.size() > 1; }));
  EXPECT_EQ(observer.states.back().generation, 2u);
  EXPECT_TRUE(observer.states.back().target_active);
  EXPECT_EQ(observer.states.back().target.x, 42);
  expectFields(observer.states.back(), PlannerNodeTestPeer::applied(*planner));
}

TEST_F(PlannerConfigStateTest, ConcurrentPublicationsMatchTheAppliedGeneration) {
  auto planner = makeNode("config_order");
  Observer observer(planner);
  ASSERT_TRUE(observer.waitFor([&] { return !observer.states.empty(); }));
  const auto incarnation = observer.states.back().incarnation;
  std::vector<ConfigState> history{PlannerNodeTestPeer::applied(*planner)};
  auto regions = std::async(std::launch::async, [&] {
    for (int i = 1; i <= 20; ++i) {
      PlannerNodeTestPeer::changeAndRecord(*planner, i, true, history);
      std::this_thread::sleep_for(1ms);
    }
  });
  auto targets = std::async(std::launch::async, [&] {
    for (int i = 1; i <= 20; ++i) {
      PlannerNodeTestPeer::changeAndRecord(*planner, i, false, history);
      std::this_thread::sleep_for(1ms);
    }
  });
  ASSERT_TRUE(observer.waitFor([&] {
    return observer.states.back().generation == 41;
  }));
  regions.get();
  targets.get();
  ASSERT_EQ(history.size(), 41u);
  ASSERT_GT(observer.states.size(), 2u);
  std::uint64_t previous = 0;
  for (const auto& state : observer.states) {
    ASSERT_GE(state.generation, 1u);
    ASSERT_LE(state.generation, history.size());
    EXPECT_GT(state.generation, previous);
    previous = state.generation;
    EXPECT_EQ(state.incarnation, incarnation);
    expectFields(state, history[state.generation - 1]);
  }
}

}  // namespace
}  // namespace mgg_ros
