#include <chrono>
#include <atomic>
#include <cmath>
#include <memory>
#include <sstream>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <mgg_msgs/srv/plan_objective.hpp>
#include <mgg_msgs/srv/planner_srv.hpp>
#include <mgg_msgs/msg/mapping_snapshot.hpp>
#include <mgg_msgs/srv/query_map_batch.hpp>

#include "mgg_ros/planner_node.h"

namespace mgg_ros {

class PlannerNodeTestPeer {
 public:
  static void configureBackboneTest(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.robot_params_.type = mgg::RobotType::kGroundRobot;
    node.robot_params_.size = Eigen::Vector3d(0.20, 0.20, 0.15);
    node.robot_params_.size_extension.setZero();
    node.robot_params_.size_extension_min.setZero();
    node.robot_params_.safety_extension.setZero();
    node.robot_params_.bound_mode = mgg::BoundModeType::kExactBound;
    node.planning_params_.max_ground_height = 0.30;
    node.planning_params_.max_step_height = 0.10;
    node.planning_params_.max_inclination = 0.52;
    node.global_vertex_spacing_ = 0.50;
  }

  static void acceptOdometry(PlannerNode& node, double x, double y, double z) {
    auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    msg->pose.pose.position.x = x;
    msg->pose.pose.position.y = y;
    msg->pose.pose.position.z = z;
    msg->pose.pose.orientation.w = 1.0;
    node.onOdometry(msg);
  }

  static void observeGroundSupport(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    // Vertical rays supply ground for every projection probe but leave parts
    // of the swept body volume unknown.  That distinction exercises strict
    // global-edge admission rather than treating support as free clearance.
    for (int repeat = 0; repeat < 10; ++repeat) {
      for (double x = -0.3; x <= 2.1; x += 0.10) {
        for (const double y : {-0.2, 0.0, 0.2}) {
          node.map_->insertPointCloud({Eigen::Vector3d(x, y, 0.0)},
                                      Eigen::Vector3d(x, y, 1.5));
        }
      }
    }
    ++node.map_revision_;
    node.updateGlobalGraph();
  }

  static void observeBodyCorridor(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    // Model a fully observed free body volume after the ground-only phase.
    node.map_->augmentFreeBox(Eigen::Vector3d(0.75, 0.0, 0.35),
                              Eigen::Vector3d(2.5, 0.8, 0.20));
    ++node.map_revision_;
    node.updateGlobalGraph();
  }

  static void observeDestinationSupport(PlannerNode& node, double x) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (int repeat = 0; repeat < 10; ++repeat) {
      for (const double dx : {-0.2, 0.0, 0.2}) {
        for (const double dy : {-0.2, 0.0, 0.2}) {
          node.map_->insertPointCloud(
              {Eigen::Vector3d(x + dx, dy, 0.0)},
              Eigen::Vector3d(x + dx, dy, 1.5));
        }
      }
    }
    node.map_->augmentFreeBox(Eigen::Vector3d(x, 0.0, 0.35),
                              Eigen::Vector3d(0.25, 0.25, 0.20));
    ++node.map_revision_;
  }

  static void configureExploreServiceScene(PlannerNode& node,
                                           bool support_root) {
    configureBackboneTest(node);
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.planning_params_.edge_length_min = 0.05;
    node.planning_params_.edge_length_max = 0.25;
    node.planning_params_.edge_overshoot = 0.0;
    node.planning_params_.num_vertices_max = 20;
    node.planning_params_.num_edges_max = 40;
    node.planning_params_.num_loops_max = 40;
    node.planning_params_.max_step_height = 0.30;
    node.planning_params_.path_interpolation_distance = 0.10;
    node.grid_params_.min_val = Eigen::Vector3d::Zero();
    node.grid_params_.max_val = Eigen::Vector3d(1.0, 0.0, 0.0);
    node.grid_params_.resolution = Eigen::Vector3d(0.50, 0.50, 0.20);
    node.global_space_.setBound(Eigen::Vector3d(-3.0, -3.0, -1.0),
                                Eigen::Vector3d(3.0, 3.0, 2.0));
    node.global_space_.min_extension.setZero();
    node.global_space_.max_extension.setZero();

    mgg::SensorParams sensor;
    sensor.type = mgg::SensorType::kLidar;
    sensor.max_range = 2.0;
    sensor.fov = Eigen::Vector2d(M_PI / 2.0, 0.20);
    sensor.resolution = Eigen::Vector2d(M_PI / 8.0, 0.20);
    sensor.frontier_percentage_threshold = 0.01;
    sensor.update();
    node.sensors_["test_lidar"] = sensor;
    node.planning_params_.exp_sensor_list = {"test_lidar"};

    // Cover every footprint box and offset ground probe used by the strict
    // global-edge check. Keep the lower face above the occupied floor.
    node.map_->augmentFreeBox(Eigen::Vector3d(0.60, 0.0, 0.40),
                              Eigen::Vector3d(2.4, 1.2, 0.60));
    for (int repeat = 0; repeat < 10; ++repeat) {
      for (double x = support_root ? -0.3 : 0.30; x <= 1.3; x += 0.10) {
        for (const double y : {-0.2, 0.0, 0.2}) {
          node.map_->insertPointCloud({Eigen::Vector3d(x, y, 0.0)},
                                      Eigen::Vector3d(x, y, 1.5));
        }
      }
    }
    node.map_->augmentFreeBox(Eigen::Vector3d(0.60, 0.0, 0.40),
                              Eigen::Vector3d(2.4, 1.2, 0.60));
    ++node.map_revision_;
  }

  static void addCorridorObstacle(PlannerNode& node, double x) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (int repeat = 0; repeat < 20; ++repeat) {
      node.map_->insertPointCloud({Eigen::Vector3d(x, 0.0, 0.30)},
                                  Eigen::Vector3d(x, 0.0, 1.5));
    }
    ++node.map_revision_;
  }

  static void configureGridServiceScene(PlannerNode& node,
                                        double body_offset_z = 0.0,
                                        double margin = 1.0) {
    configureBackboneTest(node);
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.robot_params_.center_offset = Eigen::Vector3d(0, 0, body_offset_z);
    node.grid_refinement_limits_.detour_margin_m = margin;
    // Functional service fixture, not a CPU latency benchmark. Core tests
    // exercise deadline expiry independently of DDS scheduling on the host.
    node.grid_refinement_limits_.timeout = std::chrono::milliseconds(1000);
    std::vector<Eigen::Vector3d> floor;
    for (int x = -8; x <= 32; ++x) {
      for (int y = -24; y <= 24; ++y) {
        floor.emplace_back(x * 0.05 + 0.025, y * 0.05 + 0.025, 0.025);
      }
    }
    for (int repeat = 0; repeat < 6; ++repeat) {
      node.map_->insertPointCloud(floor, Eigen::Vector3d(0.6, 0.0, 1.5));
    }
    // The known body volume is separate from the occupied supporting floor.
    node.map_->augmentFreeBox({0.6, 0.0, 0.5}, {3.2, 3.0, 0.8});
    ++node.map_revision_;
  }

  static void addGridObstacle(PlannerNode& node, double z, bool wall) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (int repeat = 0; repeat < 20; ++repeat) {
      const int bound = wall ? 26 : 0;
      for (int y = -bound; y <= bound; ++y) {
        node.map_->tree()->updateNode(
            octomap::point3d(0.9F, static_cast<float>(y * 0.05),
                             static_cast<float>(z)),
            true);
      }
    }
    ++node.map_revision_;
  }

  static bool makeGridVoxelUnknown(PlannerNode& node, double x, double y,
                                   double z) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    octomap::OcTreeKey key;
    if (!node.map_->tree()->coordToKeyChecked(
            octomap::point3d(static_cast<float>(x), static_cast<float>(y),
                             static_cast<float>(z)),
            key)) {
      return false;
    }
    if (!node.map_->tree()->updateNode(key, true)) return false;
    node.map_->tree()->deleteNode(key, node.map_->tree()->getTreeDepth());
    const bool removed = node.map_->tree()->search(key) == nullptr;
    if (removed) ++node.map_revision_;
    return removed;
  }

  static void setGridGeofence(PlannerNode& node, double xmin, double xmax,
                              double ymin, double ymax) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.planning_params_.geofence_checking_enable = true;
    node.geofence_->clear();
    mgg::Polygon2d polygon(std::vector<Eigen::Vector2d>{
        {xmin, ymin}, {xmin, ymax}, {xmax, ymax}, {xmax, ymin}, {xmin, ymin}});
    node.geofence_->addGeofenceArea(polygon);
  }

  static std::uint64_t localGraphRevision(const PlannerNode& node) {
    return node.local_graph_revision_;
  }

  static std::uint64_t localGraphMapRevision(const PlannerNode& node) {
    return node.local_graph_map_revision_;
  }

  static std::uint64_t globalGraphRevision(const PlannerNode& node) {
    return node.graph_revision_;
  }

  static std::string globalGraphDescription(const PlannerNode& node) {
    std::ostringstream out;
    for (const auto& entry : node.global_graph_->vertices_map_) {
      if (entry.second == nullptr) continue;
      out << "v" << entry.first << "=(" << entry.second->state.x() << ","
          << entry.second->state.y() << "," << entry.second->state.z() << ") ";
    }
    return out.str();
  }

  static int globalVertices(const PlannerNode& node) {
    return node.global_graph_->getNumVertices();
  }

  static int globalEdges(const PlannerNode& node) {
    return node.global_graph_->getNumEdges();
  }

  static mgg::StateVec globalVertexState(const PlannerNode& node, int id) {
    return node.global_graph_->getVertex(id)->state;
  }

  static bool initialAnchorSupported(const PlannerNode& node) {
    return node.initial_anchor_supported_;
  }

  static mgg::RouteCorridor planHome(const PlannerNode& node,
                                     const mgg::StateVec& goal) {
    mgg::PlanningRequest request;
    request.objective = mgg::ObjectiveKind::kReturnHome;
    request.component_id = node.component_id_;
    request.graph_revision = node.graph_revision_;
    request.map_revision = node.map_revision_;
    request.goal.pose = goal;
    mgg::TopologicalGoalPlanner planner(
        node.component_id_, node.graph_revision_, node.map_revision_, 1.0);
    return planner.plan(*node.global_graph_, node.current_state_, request);
  }

  static mgg::FeasiblePath refine(PlannerNode& node,
                                  const mgg::RouteCorridor& corridor) {
    return node.refineCorridor(corridor);
  }

  static mgg::ProjectedEdgeStatus globalEdgeStatus(
      PlannerNode& node, mgg::StateVec from, mgg::StateVec to) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    if (!node.projectStateToDrivingHeight(from) ||
        !node.projectStateToDrivingHeight(to)) {
      return mgg::ProjectedEdgeStatus::kHanging;
    }
    std::vector<Eigen::Vector3d> projected;
    return node.ground_->getProjectedEdgeStatus(
        from.head(3), to.head(3), node.robot_params_.getPlanningSize(), true,
        projected, false);
  }

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
using LegacyService = mgg_msgs::srv::PlannerSrv;

TEST(PlannerBackbone, CapturesHomeBeforeMotionAndConnectsOnlyMappedTerrain) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  mgg_ros::PlannerNodeTestPeer::configureBackboneTest(*planner);

  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 0.0, 0.0, 0.075);
  ASSERT_EQ(mgg_ros::PlannerNodeTestPeer::globalVertices(*planner), 1);
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::globalEdges(*planner), 0);
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::initialAnchorSupported(*planner));
  const mgg::StateVec provisional =
      mgg_ros::PlannerNodeTestPeer::globalVertexState(*planner, 0);
  EXPECT_NEAR(provisional.x(), 0.0, 1e-9);
  EXPECT_NEAR(provisional.y(), 0.0, 1e-9);

  // Motion through unknown space cannot turn the landmark into a free edge.
  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 0.60, 0.0, 0.075);
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::globalVertices(*planner), 1);
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::globalEdges(*planner), 0);
  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 1.20, 0.0, 0.075);
  mgg::StateVec authority_home = mgg::StateVec::Zero();
  authority_home.z() = 0.075;
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::planHome(*planner, authority_home).status,
            mgg::PlanningStatus::kUnreachable);

  // Ground support refines the anchor, but unknown body clearance still may
  // not create a global edge.
  mgg_ros::PlannerNodeTestPeer::observeGroundSupport(*planner);
  ASSERT_TRUE(mgg_ros::PlannerNodeTestPeer::initialAnchorSupported(*planner));
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::globalVertices(*planner), 1);
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::globalEdges(*planner), 0);

  // Once the body corridor is observed free, the reached pose connects to the
  // refined original anchor.
  mgg_ros::PlannerNodeTestPeer::observeBodyCorridor(*planner);
  mgg::StateVec observed_here = mgg::StateVec::Zero();
  observed_here.x() = 1.20;
  observed_here.z() = 0.075;
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::globalEdgeStatus(
                *planner, authority_home, observed_here),
            mgg::ProjectedEdgeStatus::kAdmissible);
  ASSERT_EQ(mgg_ros::PlannerNodeTestPeer::globalVertices(*planner), 2);
  ASSERT_EQ(mgg_ros::PlannerNodeTestPeer::globalEdges(*planner), 1);
  const mgg::StateVec home =
      mgg_ros::PlannerNodeTestPeer::globalVertexState(*planner, 0);
  EXPECT_NEAR(home.x(), 0.0, 0.11);
  EXPECT_NEAR(home.y(), 0.0, 0.11);
  EXPECT_NEAR(home.z(), 0.30, 0.11);

  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 1.80, 0.0, 0.075);
  ASSERT_EQ(mgg_ros::PlannerNodeTestPeer::globalVertices(*planner), 3);
  ASSERT_EQ(mgg_ros::PlannerNodeTestPeer::globalEdges(*planner), 2);

  // Authority home is a base pose, while graph vertices sit at driving
  // height.  The normal goal tolerance still selects the initial anchor.
  const mgg::RouteCorridor route =
      mgg_ros::PlannerNodeTestPeer::planHome(*planner, authority_home);
  EXPECT_EQ(route.status, mgg::PlanningStatus::kSucceeded) << route.reason;
  ASSERT_GE(route.poses.size(), 2u);
  EXPECT_NEAR(route.poses.back().x(), home.x(), 1e-9);
  EXPECT_NEAR(route.poses.back().y(), home.y(), 1e-9);
  const mgg::FeasiblePath feasible =
      mgg_ros::PlannerNodeTestPeer::refine(*planner, route);
  ASSERT_EQ(feasible.status, mgg::PlanningStatus::kSucceeded)
      << feasible.reason;
  ASSERT_FALSE(feasible.poses.empty());
  for (const auto& pose : feasible.poses) {
    EXPECT_NEAR(pose.z(), 0.075, 0.06);
  }
  EXPECT_NEAR(feasible.poses.back().z(), authority_home.z(), 0.06);

  // The Explore response boundary uses the same graph-to-base conversion.
  mgg::RouteCorridor explore = route;
  explore.request.objective = mgg::ObjectiveKind::kExplore;
  const mgg::FeasiblePath explore_response =
      mgg_ros::PlannerNodeTestPeer::refine(*planner, explore);
  ASSERT_EQ(explore_response.status, mgg::PlanningStatus::kSucceeded)
      << explore_response.reason;
  ASSERT_FALSE(explore_response.poses.empty());
  for (const auto& pose : explore_response.poses) {
    EXPECT_NEAR(pose.z(), 0.075, 0.06);
  }

}

TEST(PlannerBackbone, LegacyExploreRetainsHangingRootBootstrapPolicy) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  mgg_ros::PlannerNodeTestPeer::configureBackboneTest(*planner);
  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 0.0, 0.0, 0.075);
  ASSERT_FALSE(mgg_ros::PlannerNodeTestPeer::initialAnchorSupported(*planner));
  mgg_ros::PlannerNodeTestPeer::observeDestinationSupport(*planner, 0.50);

  // Response-boundary fixture for output already admitted by local graph
  // expansion, whose established bootstrap policy permits a hanging root.
  mgg::RouteCorridor explore;
  explore.status = mgg::PlanningStatus::kSucceeded;
  explore.request.objective = mgg::ObjectiveKind::kExplore;
  explore.poses.push_back(mgg::StateVec(0.50, 0.0, 0.30, 0.0));
  const mgg::FeasiblePath response =
      mgg_ros::PlannerNodeTestPeer::refine(*planner, explore);
  ASSERT_EQ(response.status, mgg::PlanningStatus::kSucceeded)
      << response.reason;
  ASSERT_EQ(response.poses.size(), 1u);
  EXPECT_NEAR(response.poses.front().z(), 0.075, 1e-9);

  // Explicit objectives do not inherit the hanging-root exception.
  mgg::RouteCorridor home = explore;
  home.request.objective = mgg::ObjectiveKind::kReturnHome;
  home.request.goal.pose = mgg::StateVec(0.50, 0.0, 0.075, 0.0);
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::refine(*planner, home).status,
            mgg::PlanningStatus::kBlocked);
}

class ObjectiveService : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("map.resolution", 0.05)});
    planner = std::make_shared<mgg_ros::PlannerNode>(options);
    caller = std::make_shared<rclcpp::Node>("objective_service_test_client");
    client = caller->create_client<Service>("plan_objective");
    legacy_client = caller->create_client<LegacyService>("mggplanner");
    executor.add_node(planner);
    spinner = std::thread([this]() { executor.spin(); });
    ASSERT_TRUE(client->wait_for_service(3s));
    ASSERT_TRUE(legacy_client->wait_for_service(3s));
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

  LegacyService::Response::SharedPtr callLegacy(
      LegacyService::Request::SharedPtr request) {
    auto future = legacy_client->async_send_request(request);
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
  rclcpp::Client<LegacyService>::SharedPtr legacy_client;
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

TEST_F(ObjectiveService,
       ColdStartLegacyExploreThenPinnedObjectiveKeepsBaseHeight) {
  mgg_ros::PlannerNodeTestPeer::configureExploreServiceScene(
      *planner, /*support_root=*/false);
  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 0.0, 0.0, 0.20);
  ASSERT_FALSE(mgg_ros::PlannerNodeTestPeer::initialAnchorSupported(*planner));

  auto legacy_request = std::make_shared<LegacyService::Request>();
  legacy_request->bound_mode = LegacyService::Request::EXACT_BOUND;
  const auto legacy = callLegacy(legacy_request);
  ASSERT_NE(legacy, nullptr);
  ASSERT_EQ(legacy->status, LegacyService::Response::FORWARD);
  ASSERT_GE(legacy->path.size(), 2u);

  auto pinned_request = std::make_shared<Service::Request>();
  pinned_request->objective = Service::Request::EXPLORE;
  pinned_request->graph_revision =
      mgg_ros::PlannerNodeTestPeer::localGraphRevision(*planner);
  pinned_request->map_revision =
      mgg_ros::PlannerNodeTestPeer::localGraphMapRevision(*planner);
  const auto pinned = call(pinned_request);
  ASSERT_NE(pinned, nullptr);
  ASSERT_EQ(pinned->status, Service::Response::SUCCEEDED) << pinned->reason;
  ASSERT_EQ(pinned->path.size(), legacy->path.size());
  for (std::size_t i = 0; i < legacy->path.size(); ++i) {
    EXPECT_NEAR(pinned->path[i].position.x, legacy->path[i].position.x, 1e-9);
    EXPECT_NEAR(pinned->path[i].position.y, legacy->path[i].position.y, 1e-9);
    EXPECT_NEAR(pinned->path[i].position.z, legacy->path[i].position.z, 1e-9);
  }
  // A second subtraction would put the supported destination below ground.
  EXPECT_GT(pinned->path.back().position.z, -0.05);
}

TEST_F(ObjectiveService, NewObstacleBlocksExplicitHomeThroughActualService) {
  mgg_ros::PlannerNodeTestPeer::configureExploreServiceScene(
      *planner, /*support_root=*/true);
  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 0.0, 0.0, 0.075);
  mgg::StateVec home_state = mgg::StateVec::Zero();
  home_state.z() = 0.075;
  mgg::StateVec first_state = home_state;
  first_state.x() = 0.60;
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::globalEdgeStatus(
                *planner, home_state, first_state),
            mgg::ProjectedEdgeStatus::kAdmissible);
  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 0.60, 0.0, 0.075);
  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 1.20, 0.0, 0.075);
  ASSERT_GE(mgg_ros::PlannerNodeTestPeer::globalVertices(*planner), 3);
  ASSERT_GE(mgg_ros::PlannerNodeTestPeer::globalEdges(*planner), 2);

  auto home = std::make_shared<Service::Request>();
  home->objective = Service::Request::RETURN_HOME;
  home->goal.position.z = 0.075;
  home->goal.orientation.w = 1.0;
  home->graph_revision =
      mgg_ros::PlannerNodeTestPeer::globalGraphRevision(*planner);
  SCOPED_TRACE(mgg_ros::PlannerNodeTestPeer::globalGraphDescription(*planner));
  auto response = call(home);
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED)
      << response->reason;
  ASSERT_GE(response->path.size(), 2u);

  mgg_ros::PlannerNodeTestPeer::addCorridorObstacle(*planner, 0.60);
  response = call(home);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED);
  EXPECT_TRUE(response->path.empty());
}

TEST_F(ObjectiveService, GridHomeDetoursOnObservedGroundAndBlocksWall) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureGridServiceScene(*planner);
  for (const double x : {0.0, 0.6, 1.2}) {
    Peer::acceptOdometry(*planner, x, 0.0, 0.075);
  }
  ASSERT_GE(Peer::globalVertices(*planner), 3);
  auto home = std::make_shared<Service::Request>();
  home->objective = Service::Request::RETURN_HOME;
  home->goal.position.z = 0.075;
  const double yaw = 0.7;
  home->goal.orientation.z = std::sin(yaw / 2);
  home->goal.orientation.w = std::cos(yaw / 2);
  home->graph_revision = Peer::globalGraphRevision(*planner);
  auto response = call(home);
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;

  // Block the edge between graph vertices, leaving both endpoints clear.
  Peer::addGridObstacle(*planner, 0.325, false);
  response = call(home);
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  ASSERT_GT(response->path.size(), 3u);
  bool detoured = false;
  for (const auto& pose : response->path) {
    detoured = detoured || std::abs(pose.position.y) > 0.15;
    EXPECT_NEAR(pose.position.z, 0.1, 1e-6);
  }
  EXPECT_TRUE(detoured);
  EXPECT_NEAR(response->path.front().position.x, 1.2, 1e-6);
  EXPECT_NEAR(response->path.back().position.x, 0.0, 1e-6);
  EXPECT_NEAR(response->path.back().orientation.z, std::sin(yaw / 2), 1e-6);
  EXPECT_NEAR(response->path.back().orientation.w, std::cos(yaw / 2), 1e-6);

  // The wall spans the observed floor and detour window; unknown space beyond
  // it must not be invented as a route around the ends.
  Peer::addGridObstacle(*planner, 0.325, true);
  response = call(home);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED) << response->reason;
  EXPECT_TRUE(response->path.empty());
}

TEST_F(ObjectiveService, GridBodyOffsetPreservesHeightAndChecksRaisedObstacle) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureGridServiceScene(*planner, 0.30, 0.0);
  for (const double x : {0.0, 0.6, 1.2}) {
    Peer::acceptOdometry(*planner, x, 0.0, 0.075);
  }
  auto home = std::make_shared<Service::Request>();
  home->objective = Service::Request::RETURN_HOME;
  home->goal.position.z = 0.075;
  home->goal.orientation.w = 1.0;
  home->graph_revision = Peer::globalGraphRevision(*planner);
  auto response = call(home);
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  ASSERT_GT(response->path.size(), 3u);
  for (const auto& pose : response->path) EXPECT_NEAR(pose.position.z, 0.1, 1e-6);

  // Above the unoffset box and its ground probes, but inside the real box
  // centered 30cm higher. The zero-margin search cannot sidestep this obstacle.
  Peer::addGridObstacle(*planner, 0.675, false);
  response = call(home);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED) << response->reason;
  EXPECT_TRUE(response->path.empty());
}

TEST_F(ObjectiveService, GridRejectsSingleUnknownBodyVoxelWithZeroOffset) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureGridServiceScene(*planner, 0.0, 0.0);
  for (const double x : {0.0, 0.6, 1.2}) {
    Peer::acceptOdometry(*planner, x, 0.0, 0.075);
  }
  auto home = std::make_shared<Service::Request>();
  home->objective = Service::Request::RETURN_HOME;
  home->goal.position.z = 0.075;
  home->goal.orientation.w = 1.0;
  home->graph_revision = Peer::globalGraphRevision(*planner);
  auto response = call(home);
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;

  // This is one body-volume key between graph vertices. The legacy 25%
  // tolerance accepts it; explicit refinement must treat any unknown as
  // blocked even when center_offset is zero.
  ASSERT_TRUE(Peer::makeGridVoxelUnknown(*planner, 0.9, 0.025, 0.325));
  response = call(home);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED) << response->reason;
  EXPECT_TRUE(response->path.empty());
}

TEST_F(ObjectiveService, GridGeofenceRejectsStationaryAndCrossingRoutes) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureGridServiceScene(*planner);
  for (const double x : {0.0, 0.6, 1.2}) {
    Peer::acceptOdometry(*planner, x, 0.0, 0.075);
  }
  auto goal = std::make_shared<Service::Request>();
  goal->objective = Service::Request::RETURN_HOME;
  goal->goal.position.x = 1.2;
  goal->goal.position.z = 0.075;
  goal->goal.orientation.w = 1.0;
  goal->graph_revision = Peer::globalGraphRevision(*planner);
  auto response = call(goal);
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;

  Peer::setGridGeofence(*planner, 1.1, 1.3, -0.15, 0.15);
  response = call(goal);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED) << response->reason;
  EXPECT_TRUE(response->path.empty());

  Peer::setGridGeofence(*planner, 0.85, 0.95, -1.3, 1.3);
  goal->goal.position.x = 0.0;
  response = call(goal);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED) << response->reason;
  EXPECT_TRUE(response->path.empty());
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
