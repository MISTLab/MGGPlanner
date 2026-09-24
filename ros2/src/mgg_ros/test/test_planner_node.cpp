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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

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
  static void observeRaisedRing(PlannerNode& node, double inner, double outer,
                                double z) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (double x = -outer; x <= outer + 1e-9; x += 0.1) {
      for (double y = -outer; y <= outer + 1e-9; y += 0.1) {
        if (std::max(std::abs(x), std::abs(y)) < inner) continue;
        for (int repeat = 0; repeat < 8; ++repeat) {
          node.cloud_map_->insertPointCloud({Eigen::Vector3d(x, y, z)},
                                            Eigen::Vector3d(x, y, 1.5));
        }
      }
    }
    ++node.map_revision_;
  }
  /// A wall this robot has seen, across y = `y` from x0 to x1.
  static void observeWall(PlannerNode& node, double x0, double x1, double y) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (double x = x0; x <= x1 + 1e-9; x += 0.1) {
      for (double z = 0.1; z <= 0.6 + 1e-9; z += 0.1) {
        for (int repeat = 0; repeat < 6; ++repeat) {
          node.cloud_map_->insertPointCloud({Eigen::Vector3d(x, y, z)},
                                            Eigen::Vector3d(x, y - 1.0, z));
        }
      }
    }
    ++node.map_revision_;
  }
  static void setHangingRootReach(PlannerNode& node, double reach) {
    node.hanging_root_edge_length_max_ = reach;
  }
  static void plan(PlannerNode& node,
                   std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response) {
    node.onPlanRequest(std::make_shared<mgg_msgs::srv::PlannerSrv::Request>(),
                       response);
  }
  static void setDrivingHeight(PlannerNode& node, double height) {
    node.planning_params_.max_ground_height = height;
  }
  /// T_ours_theirs as a planar translation, as robot_poses.py sends it.
  static void receiveTransform(PlannerNode& node, const std::string& ours,
                               const std::string& theirs, double x, double y) {
    auto msg = std::make_shared<tf2_msgs::msg::TFMessage>();
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = ours;
    t.child_frame_id = theirs;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.rotation.w = 1.0;
    msg->transforms.push_back(t);
    node.onNeighbourTransforms(msg);
  }
  static mgg_msgs::msg::Graph ownGraph(PlannerNode& node) {
    return node.ownGraphMessage();
  }
  static void receiveGraph(PlannerNode& node, const mgg_msgs::msg::Graph& g) {
    node.onNeighbourGraph(std::make_shared<mgg_msgs::msg::Graph>(g));
  }
  /// The heights of the merged vertices of `robot_id`.
  static std::vector<double> neighbourHeights(PlannerNode& node,
                                              int robot_id) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    std::vector<double> heights;
    const auto found = node.global_graph_->vertex_by_robot_id_.find(robot_id);
    if (found == node.global_graph_->vertex_by_robot_id_.end()) return heights;
    for (const auto& entry : found->second) {
      if (entry.second != nullptr) heights.push_back(entry.second->state.z());
    }
    return heights;
  }
  static void objective(
      PlannerNode& node,
      std::shared_ptr<mgg_msgs::srv::PlanObjective::Request> request,
      std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> response) {
    node.onObjectiveRequest(request, response);
  }
};

namespace {

std::shared_ptr<PlannerNode> makeNode(
    const std::string& name, const std::string& frame = "world",
    std::vector<rclcpp::Parameter> extra = {}) {
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=" + name});
  std::vector<rclcpp::Parameter> parameters{
      rclcpp::Parameter("map.backend", "cloud_octomap"),
      rclcpp::Parameter("map.resolution", 0.10),
      rclcpp::Parameter("PlanningParams.global_frame_id", frame),
  };
  parameters.insert(parameters.end(), extra.begin(), extra.end());
  options.parameter_overrides(parameters);
  // As mggplanner_node does: the nested PlanningParams are read undeclared.
  options.automatically_declare_parameters_from_overrides(true);
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

TEST_F(PlannerNodeTest, NavigateInsideTheLatticeNeedsNoRoadmap) {
  auto node = makeNode("navigate_local");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  ASSERT_EQ(PlannerNodeTestPeer::globalVertices(*node), 1);

  auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
  request->objective = mgg_msgs::srv::PlanObjective::Request::NAVIGATE;
  request->goal.position.x = 2.3;
  request->goal.position.y = 0.7;
  request->goal.orientation.w = 1.0;
  auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*node, request, response);

  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  ASSERT_GE(response->path.size(), 2u);
  // The route ends at the exact goal, not at the nearest lattice cell.
  EXPECT_NEAR(response->path.back().position.x, 2.3, 1e-3);
  EXPECT_NEAR(response->path.back().position.y, 0.7, 1e-3);
  EXPECT_NEAR(pathLength(response->path), std::hypot(2.3, 0.7), 0.8);
}

TEST_F(PlannerNodeTest, BlindStartPlansFromThePhysicalAnchor) {
  // The lidar never sees the floor under the robot: the map holds ground
  // from 1.2 m outwards only. The root hangs at the physical driving height
  // and one edge may reach the first supported cell.
  auto node = makeNode("blind_start");
  PlannerNodeTestPeer::observeFloor(*node, 1.2, 5.0, -1.5, 1.5);
  PlannerNodeTestPeer::setHangingRootReach(*node, 2.0);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);

  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_LT(std::hypot(response->path.front().position.x,
                       response->path.front().position.y),
            0.30);
  EXPECT_GE(response->path.back().position.x, 1.2);

  // Without the allowance the same start has no admissible edge.
  auto strict = makeNode("blind_start_strict");
  PlannerNodeTestPeer::observeFloor(*strict, 1.2, 5.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*strict, 0.0, 0.0, 1.0);
  response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*strict, response);
  EXPECT_EQ(response->status, PlannerNode::kStatusNoPath);
}

namespace {

/// Two planners in separate odometry frames: robot 1 at the origin of
/// robot_0/odom, robot 2 five metres east, at the origin of robot_1/odom.
/// Each has mapped and driven its own four metres of one floor; robot 2 is a
/// taller platform. Shared over neighbour_transforms, as robot_poses.py
/// publishes them.
struct TwoPlanners {
  std::shared_ptr<PlannerNode> a;
  std::shared_ptr<PlannerNode> b;

  explicit TwoPlanners(const std::string& name) {
    const std::vector<rclcpp::Parameter> topic{
        rclcpp::Parameter("neighbour_pose_source", "topic")};
    auto with_id = [&topic](int id) {
      auto parameters = topic;
      parameters.emplace_back("PlanningParams.robot_id", id);
      return parameters;
    };
    a = makeNode(name + "_a", "robot_0/odom", with_id(1));
    b = makeNode(name + "_b", "robot_1/odom", with_id(2));
    PlannerNodeTestPeer::setDrivingHeight(*b, 0.45);
    PlannerNodeTestPeer::observeFloor(*a, -1.5, 6.0, -1.5, 1.5);
    PlannerNodeTestPeer::observeFloor(*b, -1.5, 6.0, -1.5, 1.5);
    double stamp = 1.0;
    for (double x = 0.0; x <= 4.0 + 1e-9; x += 0.5) {
      PlannerNodeTestPeer::acceptOdometry(*a, x, 0.0, stamp);
      PlannerNodeTestPeer::acceptOdometry(*b, x, 0.0, stamp);
      stamp += 1.0;
    }
    PlannerNodeTestPeer::acceptOdometry(*a, 0.0, 0.0, stamp);
  }

  void share() {
    PlannerNodeTestPeer::receiveTransform(*a, "robot_0/odom", "robot_1/odom",
                                          5.0, 0.0);
    PlannerNodeTestPeer::receiveGraph(*a, PlannerNodeTestPeer::ownGraph(*b));
  }
};

}  // namespace

TEST_F(PlannerNodeTest, NeighbourRoadmapMergesAtTheReceiversDrivingHeight) {
  TwoPlanners fleet("share");
  // Without a transform the roadmap is dropped, not merged at identity.
  PlannerNodeTestPeer::receiveGraph(*fleet.a,
                                    PlannerNodeTestPeer::ownGraph(*fleet.b));
  EXPECT_TRUE(PlannerNodeTestPeer::neighbourHeights(*fleet.a, 2).empty());

  // Sent at ground height by the taller robot, placed at this one's 0.30 m.
  const auto sent = PlannerNodeTestPeer::ownGraph(*fleet.b);
  ASSERT_GT(sent.vertices.size(), 4u);
  EXPECT_EQ(sent.header.frame_id, "robot_1/odom");
  for (const auto& v : sent.vertices) EXPECT_NEAR(v.pose.position.z, 0.0, 0.11);
  fleet.share();
  const auto heights = PlannerNodeTestPeer::neighbourHeights(*fleet.a, 2);
  ASSERT_EQ(heights.size(), sent.vertices.size());
  for (double z : heights) EXPECT_NEAR(z, 0.30, 0.11);
}

TEST_F(PlannerNodeTest, GoalInANeighboursMapRoutesOverItsRoadmap) {
  TwoPlanners fleet("goal_attach");
  auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
  request->objective = mgg_msgs::srv::PlanObjective::Request::NAVIGATE;
  // Beyond this robot's map, 0.3 m beside the ground robot 2 drove.
  request->goal.position.x = 8.7;
  request->goal.position.y = 0.3;
  request->goal.orientation.w = 1.0;

  // Alone, the goal has no mapped ground under it.
  auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*fleet.a, request, response);
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::UNREACHABLE);
  EXPECT_NE(response->reason.find("no mapped ground under the goal"),
            std::string::npos);

  // With robot 2's roadmap the route runs over it and ends on the goal.
  fleet.share();
  response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*fleet.a, request, response);
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_NEAR(response->path.front().position.x, 0.0, 0.30);
  EXPECT_NEAR(response->path.back().position.x, 8.7, 1e-3);
  EXPECT_NEAR(response->path.back().position.y, 0.3, 1e-3);
  EXPECT_NEAR(pathLength(response->path), 8.7, 1.0);
  // Over robot 2's edges the path stays at this robot's driving height:
  // the step and grade gate a controller applies passes it.
  for (const auto& pose : response->path) {
    EXPECT_NEAR(pose.position.z, 0.30, 0.11);
  }
}

TEST_F(PlannerNodeTest, NeighbourRoadmapDoesNotReachThroughAKnownWall) {
  TwoPlanners fleet("goal_wall");
  fleet.share();
  auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
  request->objective = mgg_msgs::srv::PlanObjective::Request::NAVIGATE;
  request->goal.position.x = 8.5;
  request->goal.position.y = 1.2;
  request->goal.orientation.w = 1.0;
  auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*fleet.a, request, response);
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;

  // Robot 1 has since seen a wall between robot 2's track and the goal.
  PlannerNodeTestPeer::observeWall(*fleet.a, 7.0, 10.0, 0.6);
  response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*fleet.a, request, response);
  EXPECT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::UNREACHABLE);
  EXPECT_TRUE(response->path.empty());
}

TEST_F(PlannerNodeTest, ARobotRestingInADipStillPlans) {
  // The floor around the robot is mapped a step higher than under it, so the
  // robot's own body box overlaps occupied voxels. Where it stands is not an
  // obstacle to it: the lattice still has admissible cells.
  auto node = makeNode("dip");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
  // A raised ring of floor 0.25 m up, from 0.3 m to 0.7 m out, all around.
  PlannerNodeTestPeer::observeRaisedRing(*node, 0.3, 0.7, 0.25);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response->path.size(), 2u);
}

}  // namespace mgg_ros
