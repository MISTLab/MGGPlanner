// Two planners explore one corridor together through the real message path
// (tour-exploration design §6, the mgg_ros integration test): one shared
// frame, roadmaps, bids and awards over ROS topics, and a simulated lidar
// that maps what each robot drives past. They split the frontiers, and the
// corridor is fully explored.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "native_scene_map.h"
#include "mgg_ros/planner_node.h"

namespace mgg_ros {

/// This binary's access to the node (test_planner_node.cpp has its own, with
/// more): the small ground robot and map helpers that test uses.
class PlannerNodeTestPeer {
 public:
  static bool hasSnapshotSubscription(const PlannerNode& node) {
    return node.mapping_snapshot_sub_ != nullptr;
  }
  static mgg_test::NativeSceneMap& sceneMap(PlannerNode& node) {
    return dynamic_cast<mgg_test::NativeSceneMap&>(*node.map_);
  }
  static void installSceneMap(PlannerNode& node) {
    // Disconnect callbacks before replacing their MOLA target.
    node.mapping_snapshot_sub_.reset();
    node.mola_map_ = nullptr;
    node.keyframe_source_.reset();
    node.map_ = std::make_unique<mgg_test::NativeSceneMap>(
        mgg_test::NativeSceneConfig{0.1});
    node.ground_ = std::make_unique<mgg::GroundProjection>(
        *node.map_, node.planning_params_);
  }
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
    node.planning_params_.free_voxel_gain = 0.0;
    node.planning_params_.occupied_voxel_gain = 0.0;
    node.global_vertex_spacing_ = 0.50;
    node.grid_params_.min_val = Eigen::Vector3d(-1.0, -1.0, 0.0);
    node.grid_params_.max_val = Eigen::Vector3d(3.0, 1.0, 0.0);
    node.grid_params_.resolution = Eigen::Vector3d(0.50, 0.50, 0.20);
    node.global_space_.setBound(Eigen::Vector3d(-12.0, -4.0, -1.0),
                                Eigen::Vector3d(12.0, 4.0, 2.0));
    node.global_space_.min_extension.setZero();
    node.global_space_.max_extension.setZero();
    // A lidar seeing all round: frontiers on every side.
    mgg::SensorParams sensor;
    sensor.type = mgg::SensorType::kLidar;
    sensor.max_range = 2.0;
    sensor.fov = Eigen::Vector2d(2.0 * M_PI, 0.20);
    sensor.resolution = Eigen::Vector2d(M_PI / 8.0, 0.20);
    sensor.frontier_percentage_threshold = 0.01;
    sensor.update();
    node.sensors_["test_lidar"] = sensor;
    node.planning_params_.exp_sensor_list = {"test_lidar"};
    node.odometry_stale_s_ = 3600.0;
  }

  /// Mapped floor at z = 0 over [xmin, xmax] x [ymin, ymax], seen from
  /// above, with a free body volume over it.
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
        sceneMap(node).insertPointCloud({p},
                                          Eigen::Vector3d(p.x(), p.y(), 1.5));
      }
    }
    sceneMap(node).augmentFreeBox(
        Eigen::Vector3d(0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.40),
        Eigen::Vector3d(xmax - xmin, ymax - ymin, 0.60));
    ++node.map_revision_;
  }

  /// Wall points, each seen from `origin_of(point)`.
  template <typename OriginFn>
  static void observeWall(PlannerNode& node,
                          const std::vector<Eigen::Vector3d>& points,
                          const OriginFn& origin_of) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (int repeat = 0; repeat < 6; ++repeat) {
      for (const Eigen::Vector3d& p : points) {
        sceneMap(node).insertPointCloud({p}, origin_of(p));
      }
    }
    ++node.map_revision_;
  }

  static void acceptOdometryPose(PlannerNode& node,
                                 const geometry_msgs::msg::Pose& pose,
                                 double stamp_s) {
    auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    msg->header.stamp.sec = static_cast<std::int32_t>(stamp_s);
    msg->header.stamp.nanosec = static_cast<std::uint32_t>(
        (stamp_s - std::floor(stamp_s)) * 1e9);
    msg->pose.pose = pose;
    msg->pose.pose.position.z = 0.075;
    node.onOdometry(msg);
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.applyLatestOdometry();
  }

  static void plan(PlannerNode& node,
                   std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response) {
    node.onPlanRequest(std::make_shared<mgg_msgs::srv::PlannerSrv::Request>(),
                       response);
  }
};

namespace {

// The corridor: floor at voxel centres from -9.95 to 9.95 along x and -0.65
// to 0.65 across, side walls at y = +-0.75, end walls at x = +-10.05. Each
// half is longer than the 3 m around its robot's track that odometry marks
// visited (event E1), so frontiers outlive a cycle and are auctioned.
constexpr double kFloorHalfLength = 9.95;
constexpr double kFloorHalfWidth = 0.65;
constexpr double kWallY = 0.75;
constexpr double kEndWallX = 10.05;
constexpr double kSensingRangeM = 2.0;
constexpr int kFloorColumns = 200;
constexpr int kMaxCycles = 80;

double voxelCentre(double v) { return std::floor(v * 10.0) / 10.0 + 0.05; }

/// What a robot at x sees: the floor and walls within kSensingRangeM along
/// the corridor. Records the floor columns seen.
void senseFrom(PlannerNode& node, double x, std::set<long>& seen) {
  const double x0 =
      std::max(-kFloorHalfLength, voxelCentre(x - kSensingRangeM));
  const double x1 = std::min(kFloorHalfLength, voxelCentre(x + kSensingRangeM));
  PlannerNodeTestPeer::observeFloor(node, x0, x1, -kFloorHalfWidth,
                                    kFloorHalfWidth);
  std::vector<Eigen::Vector3d> side;
  for (double wx = x0; wx <= x1 + 1e-9; wx += 0.1) {
    for (double z = 0.1; z <= 0.6 + 1e-9; z += 0.1) {
      side.emplace_back(wx, kWallY, z);
      side.emplace_back(wx, -kWallY, z);
    }
  }
  PlannerNodeTestPeer::observeWall(node, side, [](const Eigen::Vector3d& p) {
    return Eigen::Vector3d(p.x(), 0.0, p.z());
  });
  for (const double end : {-kEndWallX, kEndWallX}) {
    if (std::abs(end - x) > kSensingRangeM + 0.2) continue;
    std::vector<Eigen::Vector3d> wall;
    for (double wy = -kWallY; wy <= kWallY + 1e-9; wy += 0.1) {
      for (double z = 0.1; z <= 0.6 + 1e-9; z += 0.1) {
        wall.emplace_back(end, wy, z);
      }
    }
    PlannerNodeTestPeer::observeWall(node, wall,
                                     [end](const Eigen::Vector3d& p) {
                                       return Eigen::Vector3d(
                                           end - std::copysign(1.0, end),
                                           p.y(), p.z());
                                     });
  }
  for (double c = x0; c <= x1 + 1e-9; c += 0.1) {
    seen.insert(std::lround((c - 0.05) * 10.0));
  }
}

struct FleetRobot {
  std::shared_ptr<PlannerNode> node;
  double stamp = 1.0;
  bool complete = false;
  /// How far west and east it drove.
  double min_x = 0.0;
  double max_x = 0.0;
};

std::shared_ptr<PlannerNode> makeFleetNode(int robot_id) {
  rclcpp::NodeOptions options;
  options.arguments(
      {"--ros-args", "-r", "__node:=fleet_itest_" + std::to_string(robot_id),
       "-r", "tour_bid_out:=/fleet_itest/bids", "-r",
       "tour_bid_in:=/fleet_itest/bids", "-r",
       "tour_award_out:=/fleet_itest/awards", "-r",
       "tour_award_in:=/fleet_itest/awards", "-r",
       "neighbour_graph_out:=/fleet_itest/graphs", "-r",
       "neighbour_graph_in:=/fleet_itest/graphs"});
  options.parameter_overrides({
      rclcpp::Parameter("map.backend", "mola_snapshot"),
      rclcpp::Parameter("map.mola.peer_root", "/nonexistent/test_scene"),
      rclcpp::Parameter("map.resolution", 0.10),
      rclcpp::Parameter("PlanningParams.global_frame_id", "world"),
      rclcpp::Parameter("PlanningParams.robot_id", robot_id),
      // One shared frame: every robot at a zero offset.
      rclcpp::Parameter("neighbour_offsets",
                        std::vector<double>{1, 0, 0, 0, 2, 0, 0, 0}),
      rclcpp::Parameter("graph_publish_period_sec", 0.5),
      // In radio range from end to end of the corridor.
      rclcpp::Parameter("communication_range", 50.0),
      // This world is scaled to a 0.2 m robot on a 0.5 m lattice: clusters
      // a lattice cell apart are distinct places.
      rclcpp::Parameter("fleet.cluster_merge_radius_m", 0.5),
      rclcpp::Parameter("tour.min_cluster_gain", 0.0),
      rclcpp::Parameter("tour.recompute_interval_s", 0.0),
      rclcpp::Parameter("fleet.auction_interval_s", 0.3),
      // Three fleet ticks (kFleetTickPeriodS, 0.1 s): a peer that ticks just
      // after the call still answers in time.
      rclcpp::Parameter("fleet.bid_deadline_s", 0.3),
  });
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<PlannerNode>(options);
  PlannerNodeTestPeer::installSceneMap(*node);
  PlannerNodeTestPeer::configureGroundRobot(*node);
  return node;
}

}  // namespace

TEST(FleetExploration, SceneFixtureDisconnectsSnapshotSubscription) {
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter(
      "map.mola.peer_root", "/nonexistent/test_scene")});
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<PlannerNode>(options);
  EXPECT_TRUE(PlannerNodeTestPeer::hasSnapshotSubscription(*node));
  PlannerNodeTestPeer::installSceneMap(*node);
  EXPECT_FALSE(PlannerNodeTestPeer::hasSnapshotSubscription(*node));
  node.reset();
  rclcpp::shutdown();
}

TEST(FleetExploration, TwoPlannersSplitTheFrontiersAndExploreTheCorridor) {
  rclcpp::init(0, nullptr);
  std::array<FleetRobot, 2> robots;
  robots[0].node = makeFleetNode(1);
  robots[1].node = makeFleetNode(2);
  auto listener = std::make_shared<rclcpp::Node>("fleet_itest_listener");
  std::mutex awards_mutex;
  std::vector<mgg_msgs::msg::TourAward> awards;
  auto award_sub = listener->create_subscription<mgg_msgs::msg::TourAward>(
      "/fleet_itest/awards", rclcpp::QoS(100),
      [&](mgg_msgs::msg::TourAward::ConstSharedPtr msg) {
        const std::lock_guard<std::mutex> lock(awards_mutex);
        if (!msg->call) awards.push_back(*msg);
      });
  rclcpp::executors::MultiThreadedExecutor executor;
  for (const FleetRobot& robot : robots) executor.add_node(robot.node);
  executor.add_node(listener);
  std::thread spinner([&executor] { executor.spin(); });

  // Back to back in the middle of the corridor: robot 1 facing east, robot
  // 2 west. MGG adds no frontier within 1.5 m of a robot's track to its
  // roadmap, so each first finds frontiers ahead of it.
  std::set<long> seen;
  const std::array<double, 2> start_x{0.35, -0.35};
  for (std::size_t k = 0; k < robots.size(); ++k) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = start_x[k];
    pose.orientation.z = k == 0 ? 0.0 : 1.0;  // yaw 0 or pi
    pose.orientation.w = k == 0 ? 1.0 : 0.0;
    robots[k].min_x = robots[k].max_x = start_x[k];
    senseFrom(*robots[k].node, start_x[k], seen);
    PlannerNodeTestPeer::acceptOdometryPose(*robots[k].node, pose,
                                            robots[k].stamp);
    robots[k].stamp += 1.0;
  }
  // The planners hear each other and share their roadmaps.
  std::this_thread::sleep_for(std::chrono::seconds(2));

  int cycles = 0;
  for (; cycles < kMaxCycles && !(robots[0].complete && robots[1].complete);
       ++cycles) {
    for (FleetRobot& robot : robots) {
      if (robot.complete) continue;
      auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
      PlannerNodeTestPeer::plan(*robot.node, response);
      if (response->status == PlannerNode::kStatusComplete) {
        robot.complete = true;
        continue;
      }
      if (response->status != mgg_msgs::srv::PlannerSrv::Response::FORWARD ||
          response->path.empty()) {
        continue;
      }
      // Drive the path, mapping as it goes.
      double sensed_at = response->path.front().position.x;
      for (const geometry_msgs::msg::Pose& pose : response->path) {
        PlannerNodeTestPeer::acceptOdometryPose(*robot.node, pose,
                                                robot.stamp);
        robot.stamp += 0.2;
        robot.min_x = std::min(robot.min_x, pose.position.x);
        robot.max_x = std::max(robot.max_x, pose.position.x);
        if (std::abs(pose.position.x - sensed_at) >= 0.5) {
          senseFrom(*robot.node, pose.position.x, seen);
          sensed_at = pose.position.x;
        }
      }
      senseFrom(*robot.node, response->path.back().position.x, seen);
    }
    // Robots drive between plans, and the fleet assigns between drives:
    // wait for the next award (or two seconds) before planning again.
    std::size_t awards_before = 0;
    {
      const std::lock_guard<std::mutex> lock(awards_mutex);
      awards_before = awards.size();
    }
    const auto waited_from = std::chrono::steady_clock::now();
    for (;;) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      const std::lock_guard<std::mutex> lock(awards_mutex);
      if (awards.size() > awards_before ||
          std::chrono::steady_clock::now() - waited_from >
              std::chrono::seconds(2)) {
        break;
      }
    }
  }
  executor.cancel();
  spinner.join();

  EXPECT_TRUE(robots[0].complete) << "after " << cycles << " cycles";
  EXPECT_TRUE(robots[1].complete) << "after " << cycles << " cycles";
  EXPECT_EQ(seen.size(), static_cast<std::size_t>(kFloorColumns));
  // Each explored its own half: neither toured the frontiers the other held
  // or had explored, though its own map still shows them unknown.
  EXPECT_GT(robots[0].min_x, -1.5) << "robot 1 went west to " << robots[0].min_x;
  EXPECT_LT(robots[1].max_x, 1.5) << "robot 2 went east to " << robots[1].max_x;
  const std::lock_guard<std::mutex> lock(awards_mutex);
  ASSERT_FALSE(awards.empty());
  // The auction split them: no cluster in two bundles, and every cluster a
  // bidder won lies in its own half.
  std::size_t won = 0;
  for (const mgg_msgs::msg::TourAward& award : awards) {
    std::set<std::uint64_t> named;
    for (const mgg_msgs::msg::TourBundle& bundle : award.bundles) {
      for (const std::uint64_t id : bundle.clusters) {
        EXPECT_TRUE(named.insert(id).second)
            << "cluster " << id << " in two bundles of award "
            << award.auction_id;
        if (bundle.silent_s > 0.0) continue;
        for (const mgg_msgs::msg::TourCluster& cluster : award.clusters) {
          if (cluster.id != id) continue;
          ++won;
          if (bundle.robot_id == 1) {
            EXPECT_GT(cluster.position.x, -1.5);
          }
          if (bundle.robot_id == 2) {
            EXPECT_LT(cluster.position.x, 1.5);
          }
        }
      }
    }
  }
  EXPECT_GT(won, 0u);
  robots[0].node.reset();
  robots[1].node.reset();
  rclcpp::shutdown();
}

}  // namespace mgg_ros
