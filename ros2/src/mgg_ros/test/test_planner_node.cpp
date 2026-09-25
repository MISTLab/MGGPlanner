// The planner node's contract with its callers: an exploration cycle returns
// the whole lattice path, and an explicit objective returns the whole route
// over the global graph. Both are what PCI and a full-path controller
// execute; nothing here is windowed or truncated.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "mgg_map_octomap/octomap_map.h"
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
  /// The floor is observed at the centre of every 0.1 m voxel from the one
  /// holding xmin (ymin) up: stepping 0.1 m from a bound drifts across voxel
  /// boundaries and leaves rows of floor unobserved, which the planner does
  /// not drive onto (min_observed_ground_fraction).
  static void observeFloor(PlannerNode& node, double xmin, double xmax,
                           double ymin, double ymax,
                           bool at_voxel_centres = true) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    std::vector<Eigen::Vector3d> floor;
    const auto first = [at_voxel_centres](double v) {
      return at_voxel_centres ? (std::floor(v / 0.10 + 1e-6) + 0.5) * 0.10
                              : v;
    };
    const double past = at_voxel_centres ? 0.05 : 0.0;
    for (double x = first(xmin); x <= xmax + past + 1e-9; x += 0.10) {
      for (double y = first(ymin); y <= ymax + past + 1e-9; y += 0.10) {
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

  static void acceptOdometry(
      PlannerNode& node, const nav_msgs::msg::Odometry::SharedPtr& msg) {
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
  /// A wall this robot has seen, across x = `x` from y0 to y1.
  static void observeWallAlongY(PlannerNode& node, double y0, double y1,
                                double x) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (double y = y0; y <= y1 + 1e-9; y += 0.1) {
      for (double z = 0.1; z <= 0.6 + 1e-9; z += 0.1) {
        for (int repeat = 0; repeat < 6; ++repeat) {
          node.cloud_map_->insertPointCloud({Eigen::Vector3d(x, y, z)},
                                            Eigen::Vector3d(x - 1.0, y, z));
        }
      }
    }
    ++node.map_revision_;
  }
  /// A vertex of `robot_id`'s merged roadmap joined to nothing, as a part of
  /// it cut off by the merge's step and grade filter would be.
  static void addDisconnectedNeighbourVertex(PlannerNode& node, int robot_id,
                                             double x, double y, double z) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    auto* vertex = new mgg::Vertex(node.global_graph_->generateVertexID(),
                                   mgg::StateVec(x, y, z, 0.0));
    vertex->robot_id = robot_id;
    node.global_graph_->addNeighbourVertex(vertex, 1000000);
  }
  /// Edges touching `robot_id`'s merged vertices.
  static std::size_t neighbourEdges(PlannerNode& node, int robot_id) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    std::size_t edges = 0;
    const auto found = node.global_graph_->vertex_by_robot_id_.find(robot_id);
    if (found == node.global_graph_->vertex_by_robot_id_.end()) return 0;
    for (const auto& entry : found->second) {
      const auto adjacent = node.global_graph_->edge_map_.find(entry.second->id);
      if (adjacent != node.global_graph_->edge_map_.end()) {
        edges += adjacent->second.size();
      }
    }
    return edges;
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
  /// shortcutAndResample on `path`; returns the routes sent unshortcut so
  /// far.
  static int shortcutAndResample(PlannerNode& node,
                                 std::vector<mgg::StateVec>& path,
                                 const mgg::PathOkFn& turns_ok) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.shortcutAndResample(path, turns_ok);
    return node.shortcut_turn_reverts_;
  }
  /// A robot `length` along x and `width` across, box and all.
  static void setRobotFootprint(PlannerNode& node, double length,
                                double width) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.robot_params_.size.x() = length;
    node.robot_params_.size.y() = width;
  }
  /// Gain counts unknown voxels only, as the deployed configurations
  /// (mgg_argos/config/*.yaml) have it.
  static void gainFromUnknownVoxelsOnly(PlannerNode& node) {
    node.planning_params_.free_voxel_gain = 0.0;
    node.planning_params_.occupied_voxel_gain = 0.0;
  }
  /// The controller's goal tolerance the planner assumes.
  static void setReachDistance(PlannerNode& node, double reach) {
    node.reach_distance_ = reach;
  }
  /// Gain counts only voxels inside this box (the global space).
  static void setGainSpace(PlannerNode& node, const Eigen::Vector3d& lo,
                           const Eigen::Vector3d& hi) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.global_space_.setBound(lo, hi);
  }
  /// The lattice spans `lo` to `hi` in x and y around the robot.
  static void setLattice(PlannerNode& node, const Eigen::Vector2d& lo,
                         const Eigen::Vector2d& hi) {
    node.grid_params_.min_val.head<2>() = lo;
    node.grid_params_.max_val.head<2>() = hi;
  }
  static void setSensorRange(PlannerNode& node, double range) {
    mgg::SensorParams& sensor = node.sensors_["test_lidar"];
    sensor.max_range = range;
    sensor.update();
  }
  /// Poses of the last lattice path dropped because it went nowhere.
  static int lastNowherePoses(PlannerNode& node) {
    return node.last_nowhere_poses_;
  }
  static void setMinObservedGround(PlannerNode& node, double fraction) {
    node.planning_params_.min_observed_ground_fraction = fraction;
  }
  static int pathsGoingNowhere(PlannerNode& node) {
    return node.paths_going_nowhere_;
  }
  static int lowGainRounds(PlannerNode& node) {
    return node.low_gain_rounds_;
  }
  /// Gain is counted at leaves only, as bistro.yaml has it.
  static void gainAtLeavesOnly(PlannerNode& node) {
    node.planning_params_.leafs_only_for_volumetric_gain = true;
  }
  static void setDepartureReverseAllowed(PlannerNode& node, bool allowed) {
    node.planning_params_.departure_reverse_allowed = allowed;
  }
  /// The robot at (x, y) facing `yaw`, at driving height over the map.
  static mgg::StateVec drivingState(PlannerNode& node, double x, double y,
                                    double yaw) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    mgg::StateVec state(x, y, 0.075, yaw);
    EXPECT_TRUE(node.projectToDrivingHeight(state));
    state[3] = yaw;
    return state;
  }
  static bool roomToTurn(PlannerNode& node, const mgg::StateVec& state) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return mgg::turnClear(*node.map_, node.robot_params_, state);
  }
  static bool straightDeparture(PlannerNode& node, const mgg::StateVec& start,
                                std::vector<mgg::StateVec>& path,
                                bool& reverse) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    mgg::Departure departure;
    const bool found = node.straightDeparture(start, departure);
    path = departure.path;
    reverse = departure.reverse;
    return found;
  }
  /// A chain of this robot's roadmap from the global graph's root through
  /// `points`, at driving height and facing `yaw`; its last vertex a
  /// frontier. Returns that vertex's id.
  static int addGlobalChainToFrontier(
      PlannerNode& node, const std::vector<Eigen::Vector2d>& points,
      double yaw = 0.0) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.seedGlobalGraph();
    mgg::Vertex* previous = node.global_graph_->getVertex(0);
    for (const Eigen::Vector2d& p : points) {
      auto* v = new mgg::Vertex(node.global_graph_->generateVertexID(),
                                mgg::StateVec(p.x(), p.y(),
                                              previous->state.z(), yaw));
      v->robot_id = static_cast<int>(node.planning_params_.robot_id);
      node.global_graph_->addVertex(v);
      node.global_graph_->addEdge(
          v, previous, (v->state - previous->state).head<3>().norm());
      previous = v;
    }
    previous->type = mgg::VertexType::kFrontier;
    ++node.graph_revision_;
    return previous->id;
  }
  /// The global planner runs as soon as the lattice has no frontier.
  static void consultGlobalPlannerAtOnce(PlannerNode& node) {
    node.auto_global_planner_low_gain_rounds_ = 0;
  }
  /// A global repositioning to `target_id` under way, and resumed while
  /// the robot is more than a metre from it.
  static void repositionTowards(PlannerNode& node, int target_id) {
    node.global_frontier_reach_m_ = 1.0;
    node.global_exploration_ongoing_ = true;
    node.current_global_vertex_id_ = target_id;
  }
  static int boxedInWithoutDeparture(PlannerNode& node) {
    return node.boxed_in_without_departure_;
  }
  static int boxedInDepartures(PlannerNode& node) {
    return node.boxed_in_departures_;
  }
  /// Routes to a goal sent although no route complied with the turn rule.
  static int routeSharpTurnFallbacks(PlannerNode& node) {
    return node.route_sharp_turn_fallbacks_;
  }
  /// Corners the last shortcut left, before resampling.
  static int shortcutCorners(PlannerNode& node) {
    return node.path_shortcut_corners_;
  }
  static void objective(
      PlannerNode& node,
      std::shared_ptr<mgg_msgs::srv::PlanObjective::Request> request,
      std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> response) {
    node.onObjectiveRequest(request, response);
  }
  /// The robot's keyframe trajectory comes from `source` from now on.
  static void setKeyframeSource(
      PlannerNode& node, std::unique_ptr<KeyframeTrajectorySource> source) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.keyframe_source_ = std::move(source);
  }
  /// The map in service is `component` at `epoch`, its component frame the
  /// planning frame.
  static void serveMap(PlannerNode& node, const std::string& component,
                       std::uint64_t epoch) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.mapping_snapshot_.component_id = component;
    node.mapping_snapshot_.epoch = epoch;
    node.mapping_snapshot_.component_from_navigation.rotation.w = 1.0;
    node.have_mapping_snapshot_ = true;
  }
  static void setRoadmapRebuildInterval(PlannerNode& node, double seconds) {
    node.roadmap_rebuild_min_interval_s_ = seconds;
  }
  static int roadmapRebuilds(PlannerNode& node) {
    return node.roadmap_rebuilds_;
  }
  static bool rebuildRoadmap(
      PlannerNode& node,
      PlannerNode::RoadmapRebuildTrigger trigger =
          PlannerNode::RoadmapRebuildTrigger::kSeedOnly) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.rebuildGlobalGraphFromKeyframes(trigger, "test");
  }
  static mgg::StateVec globalVertexState(PlannerNode& node, int id) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    const mgg::Vertex* vertex = node.findGlobalVertex(id);
    return vertex == nullptr ? mgg::StateVec::Constant(std::nan(""))
                             : vertex->state;
  }
  /// Whether this robot's global graph has a vertex within `within` of
  /// (x, y).
  static bool hasGlobalVertexNear(PlannerNode& node, double x, double y,
                                  double within) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (const auto& entry : node.global_graph_->vertices_map_) {
      if (entry.second != nullptr &&
          std::hypot(entry.second->state.x() - x,
                     entry.second->state.y() - y) <= within) {
        return true;
      }
    }
    return false;
  }
  /// A frontier of this robot's global graph within `within` of (x, y).
  static bool globalFrontierNear(PlannerNode& node, double x, double y,
                                 double within) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (const auto& entry : node.global_graph_->vertices_map_) {
      const mgg::Vertex* v = entry.second;
      if (v != nullptr && v->type == mgg::VertexType::kFrontier &&
          std::hypot(v->state.x() - x, v->state.y() - y) <= within) {
        return true;
      }
    }
    return false;
  }
  /// An own frontier at (x, y), at driving height, joined to nothing.
  static void addLoneGlobalFrontier(PlannerNode& node, double x, double y) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    mgg::StateVec state(x, y, 0.075, 0.0);
    EXPECT_TRUE(node.projectToDrivingHeight(state));
    auto* v = new mgg::Vertex(node.global_graph_->generateVertexID(), state);
    v->robot_id = static_cast<int>(node.planning_params_.robot_id);
    v->type = mgg::VertexType::kFrontier;
    node.global_graph_->addVertex(v);
    ++node.graph_revision_;
  }
  static int frontiersLostInRebuild(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.frontiersLostInRebuild();
  }
  static bool repositioningOngoing(PlannerNode& node) {
    return node.global_exploration_ongoing_;
  }
  static int repositioningTarget(PlannerNode& node) {
    return node.current_global_vertex_id_;
  }
  /// An accepted exploration path joins the global graph.
  static void addExplorationPath(PlannerNode& node,
                                 const std::vector<mgg::StateVec>& path) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.addRefPathToGraph(path);
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

TEST_F(PlannerNodeTest, APathEndingWithinTheGoalToleranceIsNoPath) {
  // Run 5, robot_3: a 2-pose path with no gain, sent 28 times and refused
  // each time by PCI as ending within its goal tolerance. Here every path
  // ends within the tolerance the planner is told: no path, and the round
  // counts towards global repositioning as one without gain does.
  auto node = makeNode("goes_nowhere");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::setReachDistance(*node, 10.0);
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_TRUE(response->path.empty())
      << "path of " << response->path.size() << " poses";
  EXPECT_EQ(response->status, PlannerNode::kStatusNoPath);
  EXPECT_EQ(PlannerNodeTestPeer::pathsGoingNowhere(*node), 1);
  EXPECT_EQ(PlannerNodeTestPeer::lowGainRounds(*node), 1);

  // With the global planner due, the robot is repositioned over the global
  // graph instead.
  PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{-0.5, 0.0}, {-1.0, 0.0}, {-1.5, 0.0}}, M_PI);
  PlannerNodeTestPeer::consultGlobalPlannerAtOnce(*node);
  response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_LT(response->path.back().position.x, -1.0);
}

TEST_F(PlannerNodeTest, WithGainAtLeavesOnlyAPlanWithPathsPulledBackStillSendsOne) {
  // Gain at leaves only, as deployed, and a wall across the far end of the
  // lattice: the leaves against it are pulled back to vertices with room,
  // which carry no gain of their own (5 paths pulled back). A path whose
  // gain lies beyond its end goes somewhere (PathGoesNowhere, core), and
  // the plan still sends one.
  auto node = makeNode("pulled_back_sent");
  PlannerNodeTestPeer::gainAtLeavesOnly(*node);
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
  PlannerNodeTestPeer::observeWallAlongY(*node, -1.5, 1.5, 3.15);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_EQ(PlannerNodeTestPeer::pathsGoingNowhere(*node), 0);
  EXPECT_GE(std::hypot(response->path.back().position.x,
                       response->path.back().position.y),
            1.0);
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
  // The robot faces away from home: the route starts with a turn about, on
  // mapped level floor with room, which the turn rule allows. The slope is
  // measured from the map; the global graph's vertices along the track are
  // in a line and would say nothing about it.
  EXPECT_EQ(PlannerNodeTestPeer::routeSharpTurnFallbacks(*node), 0);
}

TEST_F(PlannerNodeTest, ReturnHomeRoutesToTheHomeTheCallerSends) {
  // The caller's home is the home keyframe through its current map
  // correction; vertex 0 is where odometry started. They differ by the
  // correction (0.13 m on robot_0, SubT 2026-09-23), and the caller refuses
  // a route that does not end within 1 mm of the home it sent.
  auto node = makeNode("return_home_goal");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  double stamp = 2.0;
  for (double x = 0.5; x <= 4.0 + 1e-9; x += 0.5) {
    PlannerNodeTestPeer::acceptOdometry(*node, x, 0.0, stamp);
    stamp += 1.0;
  }

  auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
  request->objective = mgg_msgs::srv::PlanObjective::Request::RETURN_HOME;
  request->goal.position.x = 0.003;
  request->goal.position.y = 0.132;
  request->goal.position.z = 0.075;
  request->goal.orientation.w = 1.0;
  auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*node, request, response);
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_NEAR(response->path.back().position.x, 0.003, 1e-3);
  EXPECT_NEAR(response->path.back().position.y, 0.132, 1e-3);

  // A goal that is not finite leaves vertex 0 as the only home there is.
  request->goal.position.x = std::nan("");
  response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*node, request, response);
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_NEAR(response->path.back().position.x, 0.0, 1e-3);
  EXPECT_NEAR(response->path.back().position.y, 0.0, 1e-3);
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

TEST_F(PlannerNodeTest, AGoalSeededOnALowerLevelFindsTheFloorAboveIt) {
  // A 2-D goal is seeded at the robot's altitude. From a lower level that is
  // metres below the floor the goal is on (robot_0 at z -5.5 asking for a
  // goal near home, SubT 2026-09-23), and nothing is mapped below it.
  auto node = makeNode("goal_above");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  double stamp = 2.0;
  for (double x = 0.5; x <= 4.0 + 1e-9; x += 0.5) {
    PlannerNodeTestPeer::acceptOdometry(*node, x, 0.0, stamp);
    stamp += 1.0;
  }

  for (const double x : {0.7, 5.5}) {
    // One goal routed over the roadmap, one inside the local lattice.
    auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
    request->objective = mgg_msgs::srv::PlanObjective::Request::NAVIGATE;
    request->goal.position.x = x;
    request->goal.position.y = 0.3;
    request->goal.position.z = 0.075 - 5.5;
    request->goal.orientation.w = 1.0;
    auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
    PlannerNodeTestPeer::objective(*node, request, response);
    ASSERT_EQ(response->status,
              mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
        << "goal x " << x << ": " << response->reason;
    ASSERT_GE(response->path.size(), 2u);
    EXPECT_NEAR(response->path.back().position.x, x, 1e-3);
    EXPECT_NEAR(response->path.back().position.y, 0.3, 1e-3);
    // At driving height over the floor at z = 0, not down where it was asked.
    EXPECT_NEAR(response->path.back().position.z, 0.30, 0.15);
  }
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

  explicit TwoPlanners(const std::string& name, double transform_ttl_s = 5.0) {
    const std::vector<rclcpp::Parameter> topic{
        rclcpp::Parameter("neighbour_pose_source", "topic"),
        rclcpp::Parameter("neighbour_transform_ttl_sec", transform_ttl_s)};
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

TEST_F(PlannerNodeTest, ADisconnectedNearerVertexDoesNotHideAReachableOne) {
  TwoPlanners fleet("attach_reachable");
  fleet.share();
  // A stranded vertex of robot 2's roadmap right beside the goal; its track,
  // reachable, passes 0.3 m away.
  PlannerNodeTestPeer::addDisconnectedNeighbourVertex(*fleet.a, 2, 8.7, 0.35,
                                                      0.30);
  auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
  request->objective = mgg_msgs::srv::PlanObjective::Request::NAVIGATE;
  request->goal.position.x = 8.7;
  request->goal.position.y = 0.3;
  request->goal.orientation.w = 1.0;
  auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*fleet.a, request, response);
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  EXPECT_NEAR(response->path.back().position.x, 8.7, 1e-3);
  EXPECT_NEAR(response->path.back().position.y, 0.3, 1e-3);
}

TEST_F(PlannerNodeTest, AWithdrawnTransformMakesTheOldRoadmapUnusable) {
  // Long enough for the first plan to finish inside it on a slow host.
  TwoPlanners fleet("withdrawn", /*transform_ttl_s=*/1.0);
  fleet.share();
  auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
  request->objective = mgg_msgs::srv::PlanObjective::Request::NAVIGATE;
  request->goal.position.x = 8.7;
  request->goal.position.y = 0.3;
  request->goal.orientation.w = 1.0;
  auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*fleet.a, request, response);
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;

  // robot_poses stopped publishing (C-SLAM separated the robots, or the
  // source went quiet): no graph arrives, but the next plan must not use the
  // roadmap placed with the expired transform.
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));
  response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*fleet.a, request, response);
  EXPECT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::UNREACHABLE);
  EXPECT_TRUE(response->path.empty());

  // Placed again, it is joined again.
  fleet.share();
  response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*fleet.a, request, response);
  EXPECT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  EXPECT_NEAR(response->path.back().position.x, 8.7, 1e-3);
}

TEST_F(PlannerNodeTest, AWithdrawnRoadmapIsQuarantinedFromOrdinaryAttachment) {
  TwoPlanners fleet("quarantine", /*transform_ttl_s=*/1.0);
  fleet.share();
  ASSERT_GT(PlannerNodeTestPeer::neighbourEdges(*fleet.a, 2), 0u);
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));

  // A goal on robot 2's track, where this robot has no ground: refused once
  // the transform has expired, and the refusal withdraws the roadmap.
  auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
  request->objective = mgg_msgs::srv::PlanObjective::Request::NAVIGATE;
  request->goal.position.x = 8.5;
  request->goal.orientation.w = 1.0;
  auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*fleet.a, request, response);
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::UNREACHABLE);
  EXPECT_EQ(PlannerNodeTestPeer::neighbourEdges(*fleet.a, 2), 0u);

  // This robot now drives onto the stale roadmap's vertices: odometry joins
  // its states to the global graph (expandGraph), and the next objective
  // links the current state to it (connectStateToGraph). Neither may join a
  // quarantined vertex, and nothing routes over one.
  double stamp = 100.0;
  for (double x = 4.5; x <= 5.5 + 1e-9; x += 0.5) {
    PlannerNodeTestPeer::acceptOdometry(*fleet.a, x, 0.0, stamp);
    stamp += 1.0;
  }
  response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*fleet.a, request, response);
  EXPECT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::UNREACHABLE);
  EXPECT_EQ(PlannerNodeTestPeer::neighbourEdges(*fleet.a, 2), 0u);

  // The transform returns: the next merge joins the roadmap again.
  fleet.share();
  EXPECT_GT(PlannerNodeTestPeer::neighbourEdges(*fleet.a, 2), 0u);
  response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*fleet.a, request, response);
  EXPECT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  EXPECT_NEAR(response->path.back().position.x, 8.5, 1e-3);
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

TEST_F(PlannerNodeTest, ARobotAgainstAWallDepartsButItsPoseIsNoGoalLater) {
  // The robot stopped with its box touching a wall (robot_1 on the SubT
  // return, 2026-09-23). It is routed home from there, along a first
  // segment clear only for its centre line. Once it has driven away, the
  // same pose as a goal is refused: that segment was never checked for the
  // box, so it is not a roadmap edge (review r0).
  auto node = makeNode("wall_departure");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::observeWall(*node, 0.0, 3.0, 0.65);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  double stamp = 2.0;
  for (double x = 0.5; x <= 2.0 + 1e-9; x += 0.5) {
    PlannerNodeTestPeer::acceptOdometry(*node, x, 0.0, stamp);
    stamp += 1.0;
  }
  // The 0.2 m box at y = 0.52 reaches into the wall voxels from y = 0.6.
  PlannerNodeTestPeer::acceptOdometry(*node, 2.0, 0.52, stamp++);

  auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
  request->objective = mgg_msgs::srv::PlanObjective::Request::RETURN_HOME;
  request->goal.position.z = 0.075;
  request->goal.orientation.w = 1.0;
  auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*node, request, response);
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_NEAR(response->path.front().position.x, 2.0, 0.05);
  EXPECT_NEAR(response->path.front().position.y, 0.52, 0.05);
  EXPECT_NEAR(response->path.back().position.x, 0.0, 1e-3);
  EXPECT_NEAR(response->path.back().position.y, 0.0, 1e-3);

  PlannerNodeTestPeer::acceptOdometry(*node, 1.0, 0.0, stamp++);
  request->objective = mgg_msgs::srv::PlanObjective::Request::NAVIGATE;
  request->goal.position.x = 2.0;
  request->goal.position.y = 0.52;
  response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*node, request, response);
  EXPECT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::UNREACHABLE)
      << "route of " << response->path.size() << " poses";
  EXPECT_TRUE(response->path.empty());
}

TEST_F(PlannerNodeTest, ARobotRestingInADipStillPlans) {
  // The floor around the robot is mapped a step higher than under it, so the
  // robot's own body box overlaps occupied voxels. Where it stands is not an
  // obstacle to it: the lattice still has admissible cells.
  auto node = makeNode("dip");
  // As this scene was written: floor and ring stepped 0.1 m from their
  // bounds, which leaves rows of voxels unobserved, and the lattice leaves
  // the dip through them. The unobserved-ground check refuses exactly those
  // edges, so it is off here; a hole-free ring 0.25 m up all round, a step
  // over max_step_height, has no way out at all.
  PlannerNodeTestPeer::setMinObservedGround(*node, 0.0);
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5, false);
  // A raised ring of floor 0.25 m up, from 0.3 m to 0.7 m out, all around.
  PlannerNodeTestPeer::observeRaisedRing(*node, 0.3, 0.7, 0.25);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response->path.size(), 2u);
}

TEST_F(PlannerNodeTest, AResampledRouteThatFailsTheTurnCheckIsSentUnshortcut) {
  auto node = makeNode("shortcut_revert");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
  // A lattice staircase over open floor at driving height, which the
  // shortcut straightens.
  const std::vector<mgg::StateVec> lattice = {
      {0.0, 0.0, 0.30, 0.0}, {0.5, 0.0, 0.30, 0.0}, {1.0, 0.5, 0.30, 0.0},
      {1.5, 0.5, 0.30, 0.0}, {2.0, 1.0, 0.30, 0.0}};

  // With a check the route passes as it is resampled, it goes out
  // shortcut and resampled.
  std::vector<mgg::StateVec> kept = lattice;
  const mgg::PathOkFn anything = [](const mgg::PathType&) { return true; };
  EXPECT_EQ(PlannerNodeTestPeer::shortcutAndResample(*node, kept, anything),
            0);
  EXPECT_LT(PlannerNodeTestPeer::shortcutCorners(*node),
            static_cast<int>(lattice.size()));
  EXPECT_GT(kept.size(), lattice.size());

  // A check the lattice path and the shortcut pass but the resampled
  // route fails, as a turn check can once resampling moves where a turn's
  // measuring window ends (review r1): the lattice path goes out as it is.
  std::vector<mgg::StateVec> reverted = lattice;
  const std::size_t lattice_size = lattice.size();
  const mgg::PathOkFn coarse_only = [lattice_size](const mgg::PathType& p) {
    return p.size() <= lattice_size;
  };
  EXPECT_EQ(
      PlannerNodeTestPeer::shortcutAndResample(*node, reverted, coarse_only),
      1);
  EXPECT_EQ(PlannerNodeTestPeer::shortcutCorners(*node),
            static_cast<int>(lattice.size()));
  ASSERT_EQ(reverted.size(), lattice.size());
  for (std::size_t i = 0; i < lattice.size(); ++i) {
    EXPECT_TRUE(reverted[i].head<3>().isApprox(lattice[i].head<3>()))
        << "pose " << i;
  }
}

namespace {

/// A robot 0.6 m long and 0.2 m wide in a corridor 0.5 m wide from `from`
/// to `to` along x (walls at y = +-0.25) or, `along_y`, along y (walls at
/// x = +-0.25), on a floor mapped from -3 to 4 along it and -1.5 to 1.5
/// across: room to drive it along the corridor, not to turn it in place,
/// since its corners reach 0.32 m from its centre. With `across`, the
/// corridor runs along x and is 0.6 m wide (walls' faces at y = +-0.3), so
/// that the robot fits in it facing across, its ends against the walls.
std::shared_ptr<PlannerNode> boxedIn(const std::string& name, double from,
                                     double to, bool along_y = false,
                                     bool across = false) {
  auto node = makeNode(name);
  PlannerNodeTestPeer::setRobotFootprint(*node, 0.6, 0.2);
  if (along_y) {
    // At voxel centres: stepping 0.1 m from -3.0 drifts across a voxel
    // boundary and leaves a row of the floor unseen.
    PlannerNodeTestPeer::observeFloor(*node, -1.55, 1.55, -3.05, 4.05);
    PlannerNodeTestPeer::observeWallAlongY(*node, from, to, 0.25);
    PlannerNodeTestPeer::observeWallAlongY(*node, from, to, -0.25);
  } else {
    PlannerNodeTestPeer::observeFloor(*node, -3.0, 4.0, -1.5, 1.5);
    const double wall = across ? 0.35 : 0.25;
    PlannerNodeTestPeer::observeWall(*node, from, to, wall);
    PlannerNodeTestPeer::observeWall(*node, from, to, -wall);
  }
  return node;
}

/// How far along the corridor, and across it, `pose` is.
double alongCorridor(const mgg::StateVec& pose, bool along_y) {
  return along_y ? pose.y() : pose.x();
}
double acrossCorridor(const mgg::StateVec& pose, bool along_y) {
  return along_y ? pose.x() : pose.y();
}

}  // namespace

TEST_F(PlannerNodeTest, ABoxedInRobotDepartsStraightAheadWhenThereIsRoom) {
  // Run 4, robot_1: a Bunker in a pocket too tight to turn in was sent a
  // path that began with a turn, and DWB found no trajectory three times.
  // The corridor opens out 0.4 m ahead of the robot. Along y, the robot
  // faces +y: its box is checked turned with it, 0.2 m across the
  // corridor, not 0.6 m as a box aligned with the map would be (review r0).
  for (const bool along_y : {false, true}) {
    SCOPED_TRACE(along_y ? "along y" : "along x");
    const double yaw = along_y ? M_PI / 2.0 : 0.0;
    auto node = boxedIn(along_y ? "boxed_ahead_y" : "boxed_ahead", -2.5, 0.4,
                        along_y);
    const mgg::StateVec start =
        PlannerNodeTestPeer::drivingState(*node, 0.0, 0.0, yaw);
    ASSERT_FALSE(PlannerNodeTestPeer::roomToTurn(*node, start));
    std::vector<mgg::StateVec> path;
    bool reverse = true;
    ASSERT_TRUE(
        PlannerNodeTestPeer::straightDeparture(*node, start, path, reverse));
    EXPECT_FALSE(reverse);
    ASSERT_GE(path.size(), 2u);
    EXPECT_TRUE(path.front().head<3>().isApprox(start.head<3>()));
    // Out of the corridor to the first pose with room to turn, and no
    // farther.
    EXPECT_GE(alongCorridor(path.back(), along_y), 0.5);
    EXPECT_LE(alongCorridor(path.back(), along_y), 1.2);
    EXPECT_TRUE(PlannerNodeTestPeer::roomToTurn(*node, path.back()));
    EXPECT_FALSE(PlannerNodeTestPeer::roomToTurn(
        *node, path[path.size() - 2]));
    for (const mgg::StateVec& pose : path) {
      EXPECT_NEAR(acrossCorridor(pose, along_y), 0.0, 1e-9);
      EXPECT_DOUBLE_EQ(pose[3], yaw);
    }
  }
}

TEST_F(PlannerNodeTest, ABoxedInRobotReversesOutWhenOnlyBehindHasRoom) {
  // The corridor runs on 2.5 m ahead and opens out 0.4 m behind.
  for (const bool along_y : {false, true}) {
    SCOPED_TRACE(along_y ? "along y" : "along x");
    const double yaw = along_y ? M_PI / 2.0 : 0.0;
    auto node = boxedIn(along_y ? "boxed_behind_y" : "boxed_behind", -0.4,
                        2.5, along_y);
    const mgg::StateVec start =
        PlannerNodeTestPeer::drivingState(*node, 0.0, 0.0, yaw);
    ASSERT_FALSE(PlannerNodeTestPeer::roomToTurn(*node, start));
    std::vector<mgg::StateVec> path;
    bool reverse = false;
    ASSERT_TRUE(
        PlannerNodeTestPeer::straightDeparture(*node, start, path, reverse));
    EXPECT_TRUE(reverse);
    ASSERT_GE(path.size(), 2u);
    EXPECT_LE(alongCorridor(path.back(), along_y), -0.5);
    EXPECT_GE(alongCorridor(path.back(), along_y), -1.2);
    EXPECT_TRUE(PlannerNodeTestPeer::roomToTurn(*node, path.back()));
    // Driven backwards: every pose faces the way the robot faces.
    for (const mgg::StateVec& pose : path) {
      EXPECT_NEAR(acrossCorridor(pose, along_y), 0.0, 1e-9);
      EXPECT_DOUBLE_EQ(pose[3], yaw);
    }

    // A robot that may not reverse has no way out.
    PlannerNodeTestPeer::setDepartureReverseAllowed(*node, false);
    EXPECT_FALSE(
        PlannerNodeTestPeer::straightDeparture(*node, start, path, reverse));
    EXPECT_TRUE(path.empty());
  }
}

TEST_F(PlannerNodeTest, ABoxedInRobotWithNoRoomEitherWayGetsNoDeparture) {
  for (const bool along_y : {false, true}) {
    SCOPED_TRACE(along_y ? "along y" : "along x");
    auto node = boxedIn(along_y ? "boxed_both_y" : "boxed_both", -2.5, 2.5,
                        along_y);
    const mgg::StateVec start = PlannerNodeTestPeer::drivingState(
        *node, 0.0, 0.0, along_y ? M_PI / 2.0 : 0.0);
    std::vector<mgg::StateVec> path;
    bool reverse = false;
    EXPECT_FALSE(
        PlannerNodeTestPeer::straightDeparture(*node, start, path, reverse));
    EXPECT_TRUE(path.empty());
  }
}

TEST_F(PlannerNodeTest, ExplorationBoxedInSendsNoPathThatStartsWithATurn) {
  // Facing across the corridor, every exploration path starts with a
  // right-angle turn the robot has no room for, so none complies. Straight
  // ahead or back runs into a wall at once, and so does turning its ends
  // by 5 degrees: no path, rather than the fallback path that turns.
  auto node = boxedIn("boxed_explore", -2.5, 2.5, false, true);
  auto msg = std::make_shared<nav_msgs::msg::Odometry>();
  msg->header.stamp.sec = 1;
  msg->pose.pose.position.z = 0.075;
  msg->pose.pose.orientation.z = std::sin(M_PI / 4.0);
  msg->pose.pose.orientation.w = std::cos(M_PI / 4.0);
  PlannerNodeTestPeer::acceptOdometry(*node, msg);
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_TRUE(response->path.empty())
      << "path of " << response->path.size() << " poses";
  EXPECT_EQ(PlannerNodeTestPeer::boxedInWithoutDeparture(*node), 1);
  // A path may come on a later cycle: not exploration complete.
  EXPECT_EQ(response->status, PlannerNode::kStatusNoPath);
}

TEST_F(PlannerNodeTest, ABoxedInRobotGetsNoGlobalRouteThatStartsWithATurn) {
  // Review r0: with no local frontier for long enough the global planner is
  // consulted, and its route to a global frontier is kept when it turns
  // where it may not. Boxed in facing across the corridor, a route along it
  // to a frontier starts with a turn the robot has no room for, the very
  // path the boxed-in rule withholds. Neither the low-gain repositioning
  // nor one already under way may send it, and the robot is not told
  // exploration is complete: it gets no path, and its adapter's recovery
  // runs.
  auto node = boxedIn("boxed_global", -2.5, 2.5, false, true);
  auto msg = std::make_shared<nav_msgs::msg::Odometry>();
  msg->header.stamp.sec = 1;
  msg->pose.pose.position.z = 0.075;
  msg->pose.pose.orientation.z = std::sin(M_PI / 4.0);
  msg->pose.pose.orientation.w = std::cos(M_PI / 4.0);
  PlannerNodeTestPeer::acceptOdometry(*node, msg);
  const int frontier = PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{0.5, 0.0}, {1.0, 0.0}, {1.5, 0.0}, {2.0, 0.0}, {2.5, 0.0},
              {3.0, 0.0}, {3.5, 0.0}});
  PlannerNodeTestPeer::consultGlobalPlannerAtOnce(*node);

  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_TRUE(response->path.empty())
      << "path of " << response->path.size() << " poses";
  EXPECT_EQ(response->status, PlannerNode::kStatusNoPath);

  PlannerNodeTestPeer::repositionTowards(*node, frontier);
  response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_TRUE(response->path.empty())
      << "path of " << response->path.size() << " poses";
  EXPECT_EQ(response->status, PlannerNode::kStatusNoPath);
}

namespace {

/// The robot, 0.6 m long and 0.2 m wide, at the origin facing +x, at the
/// end of a dead-end corridor 0.5 m wide: walls at y = +-0.25 from x = -0.4
/// to 1.0, and a wall across the whole floor at x = 1.0. It is open behind.
/// The floor is mapped wide enough that no lattice viewpoint, facing +x as
/// the robot does, sees unknown space, and gain counts only unknown space:
/// the corridor is explored and no exploration path has gain. A global
/// frontier 2.5 m behind, facing away into the unmapped floor beyond
/// x = -3.5, is reached along the corridor's line. Returns its id.
std::shared_ptr<PlannerNode> exploredDeadEnd(const std::string& name,
                                             int& frontier) {
  auto node = makeNode(name);
  PlannerNodeTestPeer::setRobotFootprint(*node, 0.6, 0.2);
  PlannerNodeTestPeer::gainFromUnknownVoxelsOnly(*node);
  // At voxel centres, as boxedIn along y: a row of floor left unseen is
  // unknown ground, and a viewpoint that sees it has gain.
  PlannerNodeTestPeer::observeFloor(*node, -3.55, 1.05, -2.55, 2.55);
  PlannerNodeTestPeer::observeWall(*node, -0.4, 1.0, 0.25);
  PlannerNodeTestPeer::observeWall(*node, -0.4, 1.0, -0.25);
  PlannerNodeTestPeer::observeWallAlongY(*node, -2.5, 2.5, 1.0);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  frontier = PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{-0.5, 0.0}, {-1.0, 0.0}, {-1.5, 0.0}, {-2.0, 0.0}, {-2.5, 0.0}},
      M_PI);
  return node;
}

/// The response is a straight reverse departure out of exploredDeadEnd's
/// corridor: back along the robot's line to room to turn, every pose facing
/// +x as the robot does.
void expectReverseDepartureFromDeadEnd(
    PlannerNode& node,
    const mgg_msgs::srv::PlannerSrv::Response& response) {
  EXPECT_EQ(response.status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response.path.size(), 2u);
  EXPECT_LE(response.path.back().position.x, -0.5);
  EXPECT_GE(response.path.back().position.x, -1.2);
  for (const geometry_msgs::msg::Pose& pose : response.path) {
    EXPECT_NEAR(pose.position.y, 0.0, 1e-6);
    EXPECT_NEAR(pose.orientation.z, 0.0, 1e-6);
  }
  EXPECT_EQ(PlannerNodeTestPeer::boxedInDepartures(node), 1);
}

}  // namespace

TEST_F(PlannerNodeTest, APathGoingNowhereFromWhereTheRobotCannotTurnDepartsInstead) {
  // Review r0, M-5: the robot faces +x in a corridor too narrow to turn in
  // that opens out 0.4 m ahead. Every path ends within the goal tolerance
  // the planner is told, so none goes anywhere; with no room to turn it
  // departs straight ahead, as a boxed-in robot does, rather than getting
  // no path.
  auto node = boxedIn("nowhere_departs", -2.5, 0.4);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  const mgg::StateVec start =
      PlannerNodeTestPeer::drivingState(*node, 0.0, 0.0, 0.0);
  ASSERT_FALSE(PlannerNodeTestPeer::roomToTurn(*node, start));
  PlannerNodeTestPeer::setReachDistance(*node, 10.0);
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_EQ(PlannerNodeTestPeer::pathsGoingNowhere(*node), 1);
  EXPECT_EQ(PlannerNodeTestPeer::boxedInDepartures(*node), 1);
  EXPECT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_GE(response->path.back().position.x, 0.5);
  EXPECT_LE(response->path.back().position.x, 1.2);
  for (const geometry_msgs::msg::Pose& pose : response->path) {
    EXPECT_NEAR(pose.position.y, 0.0, 1e-6);
  }
}

TEST_F(PlannerNodeTest, ABunkerAtAnExploredRoomEntersACorridorTooNarrowToEndIn) {
  // Review r0, I-3: a Bunker (1.023 x 0.778 m) in an explored room, 1 m from
  // the mouth of a corridor 1.3 m wide and 6 m long (walls at y = +-0.65
  // from x = 0.5, the room's end wall across x = 0.5 either side of it).
  // The corridor is mapped to x = 3 and unknown beyond, so the gain lies in
  // it; no pose in it has viewpoint clearance (0.79 m). The room's clear
  // ends lead to no gain and go nowhere, so no path ends clear, and the
  // path into the corridor is sent, unclear, rather than no path.
  auto node = makeNode("narrow_corridor");
  PlannerNodeTestPeer::setRobotFootprint(*node, 1.023, 0.778);
  PlannerNodeTestPeer::gainFromUnknownVoxelsOnly(*node);
  PlannerNodeTestPeer::observeFloor(*node, -3.05, 3.05, -2.55, 2.55);
  PlannerNodeTestPeer::observeWall(*node, 0.5, 6.5, 0.65);
  PlannerNodeTestPeer::observeWall(*node, 0.5, 6.5, -0.65);
  PlannerNodeTestPeer::observeWallAlongY(*node, 0.7, 2.5, 0.5);
  PlannerNodeTestPeer::observeWallAlongY(*node, -2.5, -0.7, 0.5);
  // Within 1 m of the corridor's axis: from there the sensor, reaching
  // 2 m within 45 degrees of +x, sees only mapped floor across the room.
  PlannerNodeTestPeer::setLattice(*node, Eigen::Vector2d(-2.0, -1.0),
                                  Eigen::Vector2d(3.5, 1.0));
  PlannerNodeTestPeer::acceptOdometry(*node, -0.5, 0.0, 1.0);
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_GT(response->path.back().position.x, 1.0);
  EXPECT_NEAR(response->path.back().position.y, 0.0, 0.3);
  std::printf("corridor path: %zu poses to (%.2f, %.2f)\n",
              response->path.size(), response->path.back().position.x,
              response->path.back().position.y);
}

TEST_F(PlannerNodeTest, AClearTwoPosePathToNoGainLosesToThePathToGain) {
  // Review r0, M-5, robot_3's 2-pose, gain-0 path. The robot, 0.2 m
  // square, faces +x on mapped floor that ends at x = 0.75. To its right a
  // slot 0.5 m wide along y = -1 (walls at y = -0.75 up to x = 0.3, and at
  // y = -1.25) runs on into unmapped space, the only place gain counts
  // (x > 0.8, y < -0.8). The lattice is the robot's 0.5 m cells from
  // (0, -1) to (0.5, 0). Vertices in and at the slot see its unmapped part
  // but lack viewpoint clearance (0.29 m); the one at (0.5, 0) is clear and
  // sees none of it (the sensor reaches 1 m, 45 degrees either side of +x).
  // Clearance used to pick the one-edge path there: two poses, 0.5 m long,
  // leading to no gain, which went nowhere, and there was no path. That
  // clear end goes nowhere and is no clear end (review r0, I-3): no path
  // ends clear, and the path into the slot is sent, unclear.
  auto node = makeNode("gainless_two_pose");
  PlannerNodeTestPeer::gainFromUnknownVoxelsOnly(*node);
  PlannerNodeTestPeer::setSensorRange(*node, 1.0);
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 0.75, -1.5, 1.5);
  PlannerNodeTestPeer::observeWall(*node, -1.5, 0.3, -0.75);
  PlannerNodeTestPeer::observeWall(*node, -1.5, 0.75, -1.25);
  PlannerNodeTestPeer::setGainSpace(*node, Eigen::Vector3d(0.8, -4.0, -1.0),
                                    Eigen::Vector3d(8.0, -0.8, 2.0));
  PlannerNodeTestPeer::setLattice(*node, Eigen::Vector2d(0.0, -1.0),
                                  Eigen::Vector2d(0.5, 0.0));
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_LE(response->path.back().position.y, -0.5 + 1e-6);
  EXPECT_EQ(PlannerNodeTestPeer::pathsGoingNowhere(*node), 0);
}

TEST_F(PlannerNodeTest, AFailedGlobalSearchWithLocalGainLeftIsNotExplorationComplete) {
  // Review r0, I-2: every path goes nowhere (the goal tolerance the planner
  // is told reaches past the lattice), the round counts towards global
  // repositioning, and the global graph has no frontier. The lattice still
  // sees frontiers: no path, and PCI retries, not exploration complete.
  auto node = makeNode("nowhere_not_complete");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::setReachDistance(*node, 10.0);
  PlannerNodeTestPeer::consultGlobalPlannerAtOnce(*node);
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_TRUE(response->path.empty())
      << "path of " << response->path.size() << " poses";
  EXPECT_EQ(PlannerNodeTestPeer::pathsGoingNowhere(*node), 1);
  EXPECT_EQ(response->status, PlannerNode::kStatusNoPath);
}

TEST_F(PlannerNodeTest, ARobotInAnExploredDeadEndReversesOutForTheGlobalRoute) {
  // Review r1: a robot that drove into a corridor too narrow to turn in and
  // explored it has no local frontier, so the global planner is consulted.
  // Its route to the frontier behind starts with a 180-degree turn the
  // robot has no room for, and is withheld; the robot backs out straight
  // instead, as a boxed-in robot does, rather than getting no path.
  int frontier = -1;
  auto node = exploredDeadEnd("dead_end_low_gain", frontier);
  const mgg::StateVec start =
      PlannerNodeTestPeer::drivingState(*node, 0.0, 0.0, 0.0);
  ASSERT_FALSE(PlannerNodeTestPeer::roomToTurn(*node, start));
  PlannerNodeTestPeer::consultGlobalPlannerAtOnce(*node);

  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_EQ(PlannerNodeTestPeer::routeSharpTurnFallbacks(*node), 1);
  expectReverseDepartureFromDeadEnd(*node, *response);
}

TEST_F(PlannerNodeTest, AResumedRepositioningFromADeadEndReversesOutFirst) {
  // The same dead end with a global repositioning to the frontier behind
  // under way: the resumed route starts with the turn the robot has no room
  // for, and the robot backs out straight instead.
  int frontier = -1;
  auto node = exploredDeadEnd("dead_end_resume", frontier);
  PlannerNodeTestPeer::repositionTowards(*node, frontier);

  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_EQ(PlannerNodeTestPeer::routeSharpTurnFallbacks(*node), 1);
  expectReverseDepartureFromDeadEnd(*node, *response);
}

TEST_F(PlannerNodeTest, AResumedRouteWithheldWithNoWayOutIsNotExplorationComplete) {
  // Review r2: the robot, 0.6 m long, faces +x in an explored corridor
  // 0.5 m wide that runs 2.5 m on either side of it, too far for a
  // straight departure to reach room to turn. A repositioning to a
  // frontier behind it is under way; the resumed route starts with a
  // 180-degree turn, and is withheld with no departure. The lattice has no
  // gain and the global planner is due, but the robot is boxed in with no
  // way out: no second global search, whose frontier has meanwhile seen
  // everything and would make exploration complete, and no second
  // boxed-in count. No path, and the robot's own recovery runs.
  auto node = makeNode("withheld_no_way_out");
  PlannerNodeTestPeer::setRobotFootprint(*node, 0.6, 0.2);
  PlannerNodeTestPeer::gainFromUnknownVoxelsOnly(*node);
  // At voxel centres, and far enough that no lattice viewpoint, facing +x,
  // sees unknown space.
  PlannerNodeTestPeer::observeFloor(*node, -3.55, 5.55, -2.55, 2.55);
  PlannerNodeTestPeer::observeWall(*node, -2.5, 2.5, 0.25);
  PlannerNodeTestPeer::observeWall(*node, -2.5, 2.5, -0.25);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  // Facing +x, back up the corridor it has mapped: no gain on a re-check.
  const int frontier = PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{-0.5, 0.0}, {-1.0, 0.0}, {-1.5, 0.0}, {-2.0, 0.0}, {-2.5, 0.0}});
  PlannerNodeTestPeer::repositionTowards(*node, frontier);
  PlannerNodeTestPeer::consultGlobalPlannerAtOnce(*node);

  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_TRUE(response->path.empty())
      << "path of " << response->path.size() << " poses";
  EXPECT_EQ(response->status, PlannerNode::kStatusNoPath);
  EXPECT_EQ(PlannerNodeTestPeer::boxedInWithoutDeparture(*node), 1);
  EXPECT_EQ(PlannerNodeTestPeer::routeSharpTurnFallbacks(*node), 1);
}

/// A keyframe trajectory held in memory; `trajectory` may change between
/// reads.
struct TrajectoryInMemory : public KeyframeTrajectorySource {
  bool read(KeyframeTrajectory& out, std::string&) override {
    ++reads;
    out = trajectory;
    return true;
  }
  KeyframeTrajectory trajectory;
  int reads = 0;
};

/// Keyframes about every 0.25 m along `corners`, home first, on map
/// "component:test" epoch 0.
KeyframeTrajectory keyframesAlong(const std::vector<Eigen::Vector2d>& corners,
                                  std::uint64_t revision = 1) {
  KeyframeTrajectory trajectory;
  trajectory.component_id = "component:test";
  trajectory.revision = revision;
  const auto add = [&trajectory](const Eigen::Vector2d& p) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(p.x(), p.y(), 0.075);
    trajectory.poses.push_back(pose);
  };
  add(corners.front());
  for (std::size_t i = 1; i < corners.size(); ++i) {
    const Eigen::Vector2d leg = corners[i] - corners[i - 1];
    const int steps = std::max(1, static_cast<int>(std::round(leg.norm() / 0.25)));
    for (int k = 1; k <= steps; ++k) add(corners[i - 1] + leg * k / steps);
  }
  return trajectory;
}

KeyframeTrajectory keyframesAlongX(double x0, double x1,
                                   std::uint64_t revision = 1) {
  return keyframesAlong({{x0, 0.0}, {x1, 0.0}}, revision);
}

std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> returnHome(
    PlannerNode& node, double x, double y) {
  auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
  request->objective = mgg_msgs::srv::PlanObjective::Request::RETURN_HOME;
  request->goal.position.x = x;
  request->goal.position.y = y;
  request->goal.position.z = 0.075;
  request->goal.orientation.w = 1.0;
  auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(node, request, response);
  return response;
}

TEST_F(PlannerNodeTest, AGraphHoldingOnlyItsSeedIsRebuiltFromTheKeyframes) {
  // Run 5: the planner restarted 4 m from home and seeded its graph where
  // the robot stood, and Return Home had nothing to route over. With the
  // robot's keyframes the graph is rebuilt at once: vertex 0 is the home
  // keyframe, and the track leads back to it.
  auto node = makeNode("rebuild_seed_only");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::serveMap(*node, "component:test", 0);
  auto source = std::make_unique<TrajectoryInMemory>();
  source->trajectory = keyframesAlongX(0.0, 4.0);
  PlannerNodeTestPeer::setKeyframeSource(*node, std::move(source));

  PlannerNodeTestPeer::acceptOdometry(*node, 4.0, 0.0, 1.0);
  EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*node), 1);
  const mgg::StateVec home = PlannerNodeTestPeer::globalVertexState(*node, 0);
  EXPECT_NEAR(home.x(), 0.0, 1e-9);
  EXPECT_NEAR(home.y(), 0.0, 1e-9);
  EXPECT_GT(PlannerNodeTestPeer::globalVertices(*node), 4);

  const auto response = returnHome(*node, 0.0, 0.0);
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  EXPECT_NEAR(response->path.front().position.x, 4.0, 0.30);
  EXPECT_NEAR(response->path.back().position.x, 0.0, 1e-3);
  EXPECT_NEAR(response->path.back().position.y, 0.0, 1e-3);
}

TEST_F(PlannerNodeTest, KeyframesOfAnotherMapDoNotReplaceTheGraph) {
  auto node = makeNode("rebuild_other_map");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::serveMap(*node, "component:other", 0);
  auto source = std::make_unique<TrajectoryInMemory>();
  source->trajectory = keyframesAlongX(0.0, 4.0);
  PlannerNodeTestPeer::setKeyframeSource(*node, std::move(source));
  PlannerNodeTestPeer::acceptOdometry(*node, 4.0, 0.0, 1.0);
  EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*node), 0);
  EXPECT_EQ(PlannerNodeTestPeer::globalVertices(*node), 1);
  EXPECT_NEAR(PlannerNodeTestPeer::globalVertexState(*node, 0).x(), 4.0,
              1e-9);
}

TEST_F(PlannerNodeTest, APoseTheGraphCannotReachRebuildsItAndRoutesHome) {
  // The robot was driven round a wall, 3.5 m off its graph, with no
  // exploration path to grow it (run 5, robot_1 and robot_3 after the
  // restart): its pose links nowhere. The Return Home request rebuilds the
  // graph from its keyframes and routes back round the wall.
  auto node = makeNode("rebuild_unlinkable");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::observeWallAlongY(*node, -1.5, 0.6, 1.35);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(*node, {{0.5, 0.0}, {1.0, 0.0}});
  PlannerNodeTestPeer::acceptOdometry(*node, 4.5, 0.0, 2.0);
  PlannerNodeTestPeer::serveMap(*node, "component:test", 0);
  auto source = std::make_unique<TrajectoryInMemory>();
  source->trajectory = keyframesAlong(
      {{0.0, 0.0}, {0.8, 0.0}, {0.8, 1.1}, {2.0, 1.1}, {4.5, 0.0}});
  PlannerNodeTestPeer::setKeyframeSource(*node, std::move(source));
  // More than a seed: no rebuild on odometry.
  PlannerNodeTestPeer::acceptOdometry(*node, 4.5, 0.0, 3.0);
  ASSERT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*node), 0);

  const auto response = returnHome(*node, 0.0, 0.0);
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*node), 1);
  EXPECT_NEAR(response->path.back().position.x, 0.0, 1e-3);
}

TEST_F(PlannerNodeTest, ARebuildWhileResumingARepositioningGivesItUp) {
  // Review r0, C-1: a global repositioning is under way when the robot is
  // driven round a wall, off its graph. The resumed route cannot link the
  // robot's pose, which rebuilds the graph and frees the frontier the
  // route was going to. The repositioning is given up; nothing reads the
  // old graph afterwards (run under ASan), and no id of it is kept.
  auto node = makeNode("rebuild_while_resuming");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::observeWallAlongY(*node, -1.5, 0.6, 1.35);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  const int frontier = PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{0.5, 0.0}, {1.0, 0.0}});
  PlannerNodeTestPeer::repositionTowards(*node, frontier);
  PlannerNodeTestPeer::acceptOdometry(*node, 4.5, 0.0, 2.0);
  PlannerNodeTestPeer::serveMap(*node, "component:test", 0);
  auto source = std::make_unique<TrajectoryInMemory>();
  source->trajectory = keyframesAlong(
      {{0.0, 0.0}, {0.8, 0.0}, {0.8, 1.1}, {2.0, 1.1}, {4.5, 0.0}});
  PlannerNodeTestPeer::setKeyframeSource(*node, std::move(source));

  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*node), 1);
  EXPECT_FALSE(PlannerNodeTestPeer::repositioningOngoing(*node));
  EXPECT_EQ(PlannerNodeTestPeer::repositioningTarget(*node), -1);
  EXPECT_NE(response->status, PlannerNode::kStatusComplete);
}

/// The robot 3.5 m off its graph behind a wall (as in
/// APoseTheGraphCannotReachRebuildsItAndRoutesHome), its keyframes
/// running from home along `corners`.
std::shared_ptr<PlannerNode> robotBehindAWall(
    const std::string& name, const std::vector<Eigen::Vector2d>& corners,
    const std::vector<Eigen::Vector2d>& chain = {{0.5, 0.0}, {1.0, 0.0}}) {
  auto node = makeNode(name);
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::observeWallAlongY(*node, -1.5, 0.6, 1.35);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(*node, chain);
  PlannerNodeTestPeer::acceptOdometry(*node, 4.5, 0.0, 2.0);
  PlannerNodeTestPeer::serveMap(*node, "component:test", 0);
  auto source = std::make_unique<TrajectoryInMemory>();
  source->trajectory = keyframesAlong(corners);
  PlannerNodeTestPeer::setKeyframeSource(*node, std::move(source));
  return node;
}

const std::vector<Eigen::Vector2d> kRoundTheWall{
    {0.0, 0.0}, {0.8, 0.0}, {0.8, 1.1}, {2.0, 1.1}, {4.5, 0.0}};

TEST_F(PlannerNodeTest, ARobotBesideAGraphThatReachesItKeepsTheGraph) {
  // Review r0, I-1a: the robot stands 0.5 m from its graph with a wall
  // between (as a robot wedged against a wall beside its roadmap). Its
  // pose does not link, but the graph reaches there: no rebuild, and the
  // graph stays as it is.
  auto node = makeNode("rebuild_not_beside_graph");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::observeWall(*node, -1.5, 6.0, 0.25);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{0.5, 0.0}, {1.0, 0.0}, {1.5, 0.0}});
  PlannerNodeTestPeer::acceptOdometry(*node, 1.0, 0.5, 2.0);
  const int vertices = PlannerNodeTestPeer::globalVertices(*node);
  PlannerNodeTestPeer::serveMap(*node, "component:test", 0);
  auto source = std::make_unique<TrajectoryInMemory>();
  source->trajectory = keyframesAlong({{0.0, 0.5}, {1.0, 0.5}});
  PlannerNodeTestPeer::setKeyframeSource(*node, std::move(source));

  const auto response = returnHome(*node, 0.0, 0.0);
  EXPECT_NE(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED);
  EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*node), 0);
  EXPECT_EQ(PlannerNodeTestPeer::globalVertices(*node), vertices);
}

TEST_F(PlannerNodeTest, ARebuiltGraphThatDoesNotLinkThePoseIsNotSwappedIn) {
  // Review r0, I-1c: the keyframes stop short of where the robot is: the
  // rebuilt graph would not link it either, so the old graph is kept.
  auto node = robotBehindAWall("rebuild_not_linking",
                               {{0.0, 0.0}, {0.8, 0.0}});
  const int vertices = PlannerNodeTestPeer::globalVertices(*node);
  const auto response = returnHome(*node, 0.0, 0.0);
  EXPECT_NE(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED);
  EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*node), 0);
  EXPECT_EQ(PlannerNodeTestPeer::globalVertices(*node), vertices);
}

TEST_F(PlannerNodeTest, FrontiersSurviveARebuild) {
  // Review r0, I-1b: the old graph leads south from home to a frontier
  // off the robot's track. The rebuild carries it over with its path, so
  // the global planner can still reposition to it.
  auto node = robotBehindAWall("rebuild_keeps_frontiers", kRoundTheWall,
                               {{0.5, 0.0}, {0.5, -0.5}, {0.5, -1.0}});
  ASSERT_TRUE(PlannerNodeTestPeer::globalFrontierNear(*node, 0.5, -1.0, 1e-6));
  const auto response = returnHome(*node, 0.0, 0.0);
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*node), 1);
  EXPECT_TRUE(PlannerNodeTestPeer::globalFrontierNear(*node, 0.5, -1.0, 1e-6));
  EXPECT_EQ(PlannerNodeTestPeer::frontiersLostInRebuild(*node), 0);
}

TEST_F(PlannerNodeTest, AFrontierARebuildCannotCarryKeepsExplorationOpen) {
  // Review r0, I-1: a frontier joined to nothing the rebuilt graph meets is
  // lost by the rebuild. While it still looks into unknown space it is
  // remembered, and a failed global search is not exploration complete.
  auto node = robotBehindAWall("rebuild_lost_frontier", kRoundTheWall);
  PlannerNodeTestPeer::addLoneGlobalFrontier(*node, -1.2, -1.2);
  const auto response = returnHome(*node, 0.0, 0.0);
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  ASSERT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*node), 1);
  EXPECT_FALSE(PlannerNodeTestPeer::globalFrontierNear(*node, -1.2, -1.2, 0.1));
  EXPECT_EQ(PlannerNodeTestPeer::frontiersLostInRebuild(*node), 1);
}

TEST_F(PlannerNodeTest, ReturnHomeWithNoGoalRoutesToTheRebuiltHome) {
  // Review r0, M-3: the planner restarted away from home and seeded its
  // graph at (0.5, -1); Return Home with no finite goal routes to vertex 0.
  // The request rebuilds the graph, whose vertex 0 is the home keyframe
  // at (0, 0): the route goes there, not to the old seed.
  auto node = makeNode("rebuild_home_fallback");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::observeWallAlongY(*node, -1.5, 0.6, 1.35);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.5, -1.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(*node, {{0.5, -0.5}});
  PlannerNodeTestPeer::acceptOdometry(*node, 4.5, 0.0, 2.0);
  PlannerNodeTestPeer::serveMap(*node, "component:test", 0);
  auto source = std::make_unique<TrajectoryInMemory>();
  source->trajectory = keyframesAlong(kRoundTheWall);
  PlannerNodeTestPeer::setKeyframeSource(*node, std::move(source));

  const auto response = returnHome(*node, std::nan(""), std::nan(""));
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*node), 1);
  EXPECT_NEAR(response->path.back().position.x, 0.0, 1e-3);
  EXPECT_NEAR(response->path.back().position.y, 0.0, 1e-3);
}

TEST_F(PlannerNodeTest, AnExplorationPathThatCannotBeLinkedRebuildsTheGraph) {
  auto node = makeNode("rebuild_path_unlinked");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(*node, {{0.5, 0.0}, {1.0, 0.0}});
  PlannerNodeTestPeer::serveMap(*node, "component:test", 0);
  auto source = std::make_unique<TrajectoryInMemory>();
  source->trajectory = keyframesAlongX(0.0, 4.0);
  PlannerNodeTestPeer::setKeyframeSource(*node, std::move(source));
  const mgg::StateVec start = PlannerNodeTestPeer::drivingState(*node, 4.0, 0.0, 0.0);
  const mgg::StateVec end = PlannerNodeTestPeer::drivingState(*node, 5.5, 0.0, 0.0);
  PlannerNodeTestPeer::addExplorationPath(*node, {start, end});
  EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*node), 1);
  EXPECT_TRUE(PlannerNodeTestPeer::hasGlobalVertexNear(*node, 5.5, 0.0, 1e-6));
}

TEST_F(PlannerNodeTest, RebuildsAreRateLimitedAndSkipUnchangedKeyframes) {
  auto node = makeNode("rebuild_rate_limit");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::serveMap(*node, "component:test", 0);
  auto source = std::make_unique<TrajectoryInMemory>();
  source->trajectory = keyframesAlongX(0.0, 4.0);
  TrajectoryInMemory* keyframes = source.get();
  PlannerNodeTestPeer::setKeyframeSource(*node, std::move(source));

  EXPECT_TRUE(PlannerNodeTestPeer::rebuildRoadmap(*node));
  // Within the interval: not even read.
  EXPECT_FALSE(PlannerNodeTestPeer::rebuildRoadmap(*node));
  EXPECT_EQ(keyframes->reads, 1);
  PlannerNodeTestPeer::setRoadmapRebuildInterval(*node, 0.0);
  // The same keyframes on the same map: the same graph.
  EXPECT_FALSE(PlannerNodeTestPeer::rebuildRoadmap(*node));
  keyframes->trajectory = keyframesAlongX(0.0, 5.0, 2);
  EXPECT_TRUE(PlannerNodeTestPeer::rebuildRoadmap(*node));
  EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*node), 2);
}

TEST_F(PlannerNodeTest, EachRebuildTriggerHasItsOwnRateLimit) {
  // Review r0, M-7: a seed-only attempt does not hold back a rebuild for a
  // pose that cannot be linked within its interval.
  auto node = makeNode("rebuild_trigger_slots");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::serveMap(*node, "component:test", 0);
  auto source = std::make_unique<TrajectoryInMemory>();
  source->trajectory = keyframesAlongX(0.0, 4.0);
  PlannerNodeTestPeer::setKeyframeSource(*node, std::move(source));
  EXPECT_TRUE(PlannerNodeTestPeer::rebuildRoadmap(*node));
  EXPECT_FALSE(PlannerNodeTestPeer::rebuildRoadmap(*node));
  EXPECT_TRUE(PlannerNodeTestPeer::rebuildRoadmap(
      *node, PlannerNode::RoadmapRebuildTrigger::kPoseUnlinkable));
  EXPECT_FALSE(PlannerNodeTestPeer::rebuildRoadmap(
      *node, PlannerNode::RoadmapRebuildTrigger::kPoseUnlinkable));
  EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*node), 2);
}

}  // namespace mgg_ros
