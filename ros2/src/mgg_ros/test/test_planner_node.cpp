// The planner node's contract with its callers: an exploration cycle returns
// the whole lattice path, and an explicit objective returns the whole route
// over the global graph. Both are what PCI and a full-path controller
// execute; nothing here is windowed or truncated.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "mgg_ros/planner_node.h"

namespace mgg_ros {

class PlannerNodeTestPeer {
 public:
  static void configureGroundRobot(PlannerNode& node) {
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
    node.planning_params_.edge_length_min = 0.05;
    node.planning_params_.edge_length_max = 0.55;
    node.planning_params_.edge_overshoot = 0.0;
    node.planning_params_.num_vertices_max = 200;
    node.planning_params_.num_edges_max = 800;
    node.planning_params_.num_loops_max = 200;
    node.planning_params_.path_interpolation_distance = 0.10;
    node.planning_params_.traverse_length_max = 20.0;
    node.global_vertex_spacing_ = 0.50;
    node.grid_params_.min_val = Eigen::Vector3d(-1.0, -1.0, 0.0);
    node.grid_params_.max_val = Eigen::Vector3d(3.0, 1.0, 0.0);
    node.grid_params_.resolution = Eigen::Vector3d(0.50, 0.50, 0.20);
    node.global_space_.setBound(Eigen::Vector3d(-4.0, -4.0, -1.0),
                                Eigen::Vector3d(8.0, 4.0, 2.0));
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
    node.odometry_stale_s_ = 3600.0;
  }

  /// Mapped floor at z = 0 over [xmin, xmax] x [ymin, ymax], observed by
  /// vertical rays, and a free body volume above it. Space outside the
  /// rectangle stays unknown, which is what the lattice's frontiers look at.
  static void observeFloor(PlannerNode& node, double xmin, double xmax,
                           double ymin, double ymax) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    std::vector<Eigen::Vector3d> floor;
    for (double x = xmin; x <= xmax + 1e-9; x += 0.10) {
      for (double y = ymin; y <= ymax + 1e-9; y += 0.10) {
        floor.emplace_back(x, y, 0.0);
      }
    }
    for (int repeat = 0; repeat < 6; ++repeat) {
      for (const Eigen::Vector3d& p : floor) {
        node.cloud_map_->insertPointCloud({p}, Eigen::Vector3d(p.x(), p.y(), 1.5));
      }
    }
    node.cloud_map_->augmentFreeBox(
        Eigen::Vector3d(0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.40),
        Eigen::Vector3d(xmax - xmin, ymax - ymin, 0.60));
    ++node.map_revision_;
  }

  static void acceptOdometry(PlannerNode& node, double x, double y,
                             double stamp_s) {
    auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    msg->header.stamp.sec = static_cast<std::int32_t>(stamp_s);
    msg->pose.pose.position.x = x;
    msg->pose.pose.position.y = y;
    msg->pose.pose.position.z = 0.075;
    msg->pose.pose.orientation.w = 1.0;
    node.onOdometry(msg);
  }

  static int globalVertices(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.global_graph_->getNumVertices();
  }
  static int globalEdges(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.global_graph_->getNumEdges();
  }
  static void plan(PlannerNode& node,
                   std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response) {
    node.onPlanRequest(std::make_shared<mgg_msgs::srv::PlannerSrv::Request>(),
                       response);
  }
  static void objective(
      PlannerNode& node,
      std::shared_ptr<mgg_msgs::srv::PlanObjective::Request> request,
      std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> response) {
    node.onObjectiveRequest(request, response);
  }
};

namespace {

std::shared_ptr<PlannerNode> makeNode(const std::string& name) {
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=" + name});
  options.parameter_overrides({
      rclcpp::Parameter("map.backend", "cloud_octomap"),
      rclcpp::Parameter("map.resolution", 0.10),
      rclcpp::Parameter("PlanningParams.global_frame_id", "world"),
  });
  auto node = std::make_shared<PlannerNode>(options);
  PlannerNodeTestPeer::configureGroundRobot(*node);
  return node;
}

double pathLength(const std::vector<geometry_msgs::msg::Pose>& path) {
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += std::hypot(path[i].position.x - path[i - 1].position.x,
                         path[i].position.y - path[i - 1].position.y);
  }
  return length;
}

double maxStep(const std::vector<geometry_msgs::msg::Pose>& path) {
  double step = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    step = std::max(step, std::hypot(path[i].position.x - path[i - 1].position.x,
                                     path[i].position.y - path[i - 1].position.y));
  }
  return step;
}

}  // namespace

class PlannerNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(PlannerNodeTest, ExplorationReturnsTheWholeLatticePath) {
  auto node = makeNode("explore_whole");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);

  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);

  ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response->path.size(), 2u);
  // Starts at the robot, ends at a lattice leaf looking into unknown space:
  // the whole path, not a stub of it.
  EXPECT_LT(std::hypot(response->path.front().position.x,
                       response->path.front().position.y),
            0.30);
  EXPECT_GE(std::hypot(response->path.back().position.x,
                       response->path.back().position.y),
            1.0);
  EXPECT_NEAR(pathLength(response->path),
              std::hypot(response->path.back().position.x,
                         response->path.back().position.y),
              1.0);
  // Resampled at path_interpolation_distance: a controller sees a dense
  // path, not lattice corners.
  EXPECT_LE(maxStep(response->path), 0.25);
  // The accepted path joined the global graph (rrg.cpp:4538).
  EXPECT_GT(PlannerNodeTestPeer::globalVertices(*node), 2);

  // A second cycle plans afresh: it is answered from a new lattice, not by
  // the previous path handed back again.
  const auto first = response->path;
  PlannerNodeTestPeer::acceptOdometry(*node, first.back().position.x,
                                      first.back().position.y, 2.0);
  response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_LT(std::hypot(response->path.front().position.x - first.back().position.x,
                       response->path.front().position.y - first.back().position.y),
            0.30);
}

TEST_F(PlannerNodeTest, ReturnHomeIsTheWholeRouteOverTheGlobalGraph) {
  auto node = makeNode("return_home_whole");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  const int seeded = PlannerNodeTestPeer::globalVertices(*node);
  ASSERT_EQ(seeded, 1);

  // Drive 4 m east in half-metre steps: odometry wires the track into the
  // global graph (rrg.cpp:5247).
  double stamp = 2.0;
  for (double x = 0.5; x <= 4.0 + 1e-9; x += 0.5) {
    PlannerNodeTestPeer::acceptOdometry(*node, x, 0.0, stamp);
    stamp += 1.0;
  }
  ASSERT_GT(PlannerNodeTestPeer::globalVertices(*node), 4);
  ASSERT_GT(PlannerNodeTestPeer::globalEdges(*node), 3);

  auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
  request->objective = mgg_msgs::srv::PlanObjective::Request::RETURN_HOME;
  auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*node, request, response);

  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_NEAR(response->path.front().position.x, 4.0, 0.30);
  EXPECT_NEAR(response->path.back().position.x, 0.0, 0.30);
  EXPECT_NEAR(response->path.back().position.y, 0.0, 0.30);
  // The whole way home, straightened where the map vouches for it.
  EXPECT_NEAR(pathLength(response->path), 4.0, 0.60);
  EXPECT_LE(maxStep(response->path), 0.25);
}

TEST_F(PlannerNodeTest, NavigateRoutesToAGoalOnTheRoadmap) {
  auto node = makeNode("navigate_whole");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  double stamp = 2.0;
  for (double x = 0.5; x <= 4.0 + 1e-9; x += 0.5) {
    PlannerNodeTestPeer::acceptOdometry(*node, x, 0.0, stamp);
    stamp += 1.0;
  }
  // Back at the start: the goal is the far end of the track.
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, stamp);

  auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
  request->objective = mgg_msgs::srv::PlanObjective::Request::NAVIGATE;
  request->goal.position.x = 4.0;
  request->goal.orientation.w = 1.0;
  auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*node, request, response);

  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_NEAR(response->path.back().position.x, 4.0, 0.30);
  EXPECT_NEAR(pathLength(response->path), 4.0, 0.60);

  // A goal off the roadmap and off the map is refused, not truncated to a
  // prefix towards it.
  request->goal.position.x = 30.0;
  response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*node, request, response);
  EXPECT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::UNREACHABLE);
  EXPECT_TRUE(response->path.empty());
}

}  // namespace mgg_ros
