// The planner node's contract with its callers: an exploration cycle returns
// the whole lattice path, and an explicit objective returns the whole route
// over the global graph. Both are what PCI and a full-path controller
// execute; nothing here is windowed or truncated.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "mgg_map_octomap/octomap_map.h"
#include "mgg_ros/planner_node.h"
#include "mgg_ros/fleet_conversions.h"

namespace mgg_ros {

// Attempt snapshot retraction from another thread during a real gain scan.
class PublicationProbeMap : public mgg::MolaMap {
 public:
  PublicationProbeMap()
      : mgg::MolaMap(mgg::MolaMapConfig{"/tmp/frontier_lease_probe"}) {}
  std::future<void> writer;
  bool publication_blocked = false;
  void getScanStatusIterative(
      const Eigen::Vector3d& pos,
      const std::vector<Eigen::Vector3d>& endpoints, mgg::GainCounts& gain,
      std::vector<std::pair<Eigen::Vector3d, mgg::VoxelStatus>>& log,
      const mgg::SensorModel& sensor) override {
    std::promise<void> entered;
    auto ready = entered.get_future();
    writer = std::async(std::launch::async, [this, &entered]() {
      entered.set_value();
      requestSnapshot(mgg::MolaSnapshotRequest{});
    });
    ready.wait();
    publication_blocked =
        writer.wait_for(std::chrono::milliseconds(50)) == std::future_status::timeout;
    mgg::MolaMap::getScanStatusIterative(pos, endpoints, gain, log, sensor);
  }
};

class SlowScanMap : public mgg::OctomapMap {
 public:
  std::vector<double> scanned_x;
  void getScanStatusIterative(
      const Eigen::Vector3d& pos,
      const std::vector<Eigen::Vector3d>& endpoints, mgg::GainCounts& gain,
      std::vector<std::pair<Eigen::Vector3d, mgg::VoxelStatus>>& log,
      const mgg::SensorModel& sensor) override {
    scanned_x.push_back(pos.x());
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    mgg::OctomapMap::getScanStatusIterative(pos, endpoints, gain, log, sensor);
  }
};

class PlannerNodeTestPeer {
 public:
  static std::vector<mgg::FrontierCluster> frontierClusters(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.globalFrontierClusters();  // deliberately no caller-side lease
  }
  static void useMolaMap(PlannerNode& node, std::unique_ptr<mgg::MolaMap> map) {
    node.mola_map_ = map.get();
    node.cloud_map_ = nullptr;
    node.map_ = std::move(map);
    node.ground_ = std::make_unique<mgg::GroundProjection>(
        *node.map_, node.planning_params_);
  }
  static void useCloudMap(PlannerNode& node, std::unique_ptr<mgg::OctomapMap> map) {
    node.cloud_map_ = map.get();
    node.mola_map_ = nullptr;
    node.map_ = std::move(map);
    node.ground_ = std::make_unique<mgg::GroundProjection>(
        *node.map_, node.planning_params_);
  }
  static void resetFrontierGain(PlannerNode& node, int id) {
    auto* v = node.global_graph_->getVertex(id);
    v->type = mgg::VertexType::kFrontier;
    v->vol_gain.gain = 0.0;
  }
  static void setTourAside(PlannerNode& node, mgg::ClusterId id) {
    node.setTourClusterAside(id);
  }
  static void setFrontierOwner(PlannerNode& node, int id, int owner) {
    node.global_graph_->getVertex(id)->robot_id = owner;
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
  static std::uint64_t graphRevision(PlannerNode& node) {
    return node.graph_revision_;
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
  /// The no-go zones in force, planning frame.
  static std::vector<Eigen::Vector2d> noGoZones(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.no_go_zones_;
  }
  static rclcpp::CallbackGroupType noGoZonesCallbackGroupType(
      PlannerNode& node) {
    return node.no_go_zones_group_->type();
  }
  /// No-go zones at `points`, as SwarmDeck publishes them.
  static void receiveNoGoZones(PlannerNode& node, const std::string& frame,
                               const std::vector<Eigen::Vector2d>& points) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseArray>();
    msg->header.frame_id = frame;
    for (const Eigen::Vector2d& p : points) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = p.x();
      pose.position.y = p.y();
      pose.orientation.w = 1.0;
      msg->poses.push_back(pose);
    }
    node.onNoGoZones(msg);
  }
  static void setCommunicationRange(PlannerNode& node, double range) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.communication_range_ = range;
  }
  static bool isQuarantined(PlannerNode& node, int robot_id) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.global_graph_->isQuarantined(robot_id);
  }
  /// Quarantines `robot_id`'s roadmap, as an expired transform does.
  static void quarantine(PlannerNode& node, int robot_id) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.global_graph_->disconnectNeighbourGraph(robot_id);
  }
  static void release(PlannerNode& node, int robot_id) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.global_graph_->releaseNeighbourGraph(robot_id);
  }
  /// runGlobalPlanner to the best frontier, outside a plan request.
  static bool runGlobalPlanner(PlannerNode& node, std::string& reason) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.runGlobalPlanner(-1, reason);
  }
  static std::string completionWithheld(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.completionWithheld();
  }
  static void setGlobalSearchBudget(PlannerNode& node, double seconds) {
    node.planning_params_.global_search_time_budget_s = seconds;
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
  /// Space this robot has seen empty: a box of `size` centred on `center`.
  static void observeFreeBox(PlannerNode& node, const Eigen::Vector3d& center,
                             const Eigen::Vector3d& size) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.cloud_map_->augmentFreeBox(center, size);
    ++node.map_revision_;
  }
  static std::optional<mgg::StandingStart> standingStart(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.standingStart();
  }
  /// An edge's status as the node's lattice checks it, with its standing
  /// start.
  static mgg::ProjectedEdgeStatus edgeStatus(PlannerNode& node,
                                             const Eigen::Vector3d& from,
                                             const Eigen::Vector3d& to) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    mgg::GroundProjection ground(*node.map_, node.planning_params_);
    ground.setStandingStart(node.standingStart());
    std::vector<Eigen::Vector3d> path;
    return ground.getProjectedEdgeStatus(
        from, to, node.robot_params_.getPlanningSize(), false, path, false);
  }
  static void setHangingRootReach(PlannerNode& node, double reach) {
    node.hanging_root_edge_length_max_ = reach;
  }
  static void plan(PlannerNode& node,
                   std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response) {
    node.onPlanRequest(std::make_shared<mgg_msgs::srv::PlannerSrv::Request>(),
                       response);
  }
  /// A floor rising along +x at `grade` (rise over run) over [xmin, xmax] x
  /// [ymin, ymax], observed by vertical rays at the centre of every 0.1 m
  /// voxel but one in nine, (x, y) index 1 modulo 3 both ways. The
  /// columns left out stay wholly unknown, so no turning circle is
  /// observed (turnSpaceObserved), while the ground between is bridged.
  static void observeSparseSlope(PlannerNode& node, double xmin, double xmax,
                                 double ymin, double ymax, double grade) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    std::vector<Eigen::Vector3d> floor;
    const auto index = [](double v) {
      return static_cast<long>(std::floor(v / 0.10 + 1e-6));
    };
    for (long ix = index(xmin); ix <= index(xmax); ++ix) {
      for (long iy = index(ymin); iy <= index(ymax); ++iy) {
        if (((ix % 3) + 3) % 3 == 1 && ((iy % 3) + 3) % 3 == 1) continue;
        const double x = (ix + 0.5) * 0.10;
        floor.emplace_back(x, (iy + 0.5) * 0.10, grade * x);
      }
    }
    for (int repeat = 0; repeat < 6; ++repeat) {
      for (const Eigen::Vector3d& p : floor) {
        node.cloud_map_->insertPointCloud(
            {p}, Eigen::Vector3d(p.x(), p.y(), p.z() + 1.5));
      }
    }
    ++node.map_revision_;
  }
  /// The lattice's body may cross unknown space, as SwarmDeck deploys it.
  static void allowUnknownLatticeBody(PlannerNode& node) {
    node.allow_unknown_lattice_body_ = true;
  }
  /// Whether the robot has room to turn in place at `state`: roomToTurn,
  /// its turn space observed.
  static bool roomToTurnObserved(PlannerNode& node,
                                 const mgg::StateVec& state) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return mgg::roomToTurn(*node.map_, node.robot_params_,
                           node.planning_params_, state);
  }
  /// The test lidar's SensorParams::mount_height.
  static void setSensorMountHeight(PlannerNode& node, double height) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    mgg::SensorParams& sensor = node.sensors_.at("test_lidar");
    sensor.mount_height = height;
    sensor.update();
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
  /// The lattice's cell size in x and y.
  static void setLatticeResolution(PlannerNode& node, double resolution) {
    node.grid_params_.resolution.head<2>().setConstant(resolution);
  }
  /// The test lidar sees all round, so no direction has more gain for the
  /// way a viewpoint faces.
  static void seeAllRound(PlannerNode& node) {
    mgg::SensorParams& sensor = node.sensors_["test_lidar"];
    sensor.fov.x() = 2.0 * M_PI;
    sensor.update();
  }
  static void setExplorationTarget(PlannerNode& node,
                                   const Eigen::Vector3d& target) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.exploration_target_ = target;
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
  /// Whether the next lattice selection is to be scored without the
  /// direction penalty's bound (mgg::TurnBackHysteresis).
  static bool lastTurnedBack(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.turn_back_hysteresis_.lastTurnedBack();
  }
  /// The exploration target service, as SwarmDeck calls it.
  static void requestExplorationTarget(PlannerNode& node, bool active,
                                       const Eigen::Vector3d& target) {
    auto request = std::make_shared<
        mgg_msgs::srv::PlannerSetExplorationTarget::Request>();
    request->active = active;
    request->target.x = target.x();
    request->target.y = target.y();
    request->target.z = target.z();
    auto response = std::make_shared<
        mgg_msgs::srv::PlannerSetExplorationTarget::Response>();
    node.onExplorationTargetRequest(request, response);
  }
  static int lowGainHandoffs(PlannerNode& node) {
    return node.low_gain_handoffs_;
  }
  /// A path scoring under `voxels` unknown voxels is a low-gain round.
  static void setLowGainVoxels(PlannerNode& node, double voxels) {
    node.planning_params_.low_gain_voxels = voxels;
  }
  /// A global frontier takes over a low-gain path only when worth more than
  /// `voxels` unknown voxels.
  static void setLowGainHandoffMinVoxels(PlannerNode& node, double voxels) {
    node.planning_params_.low_gain_handoff_min_voxels = voxels;
  }
  /// The low-gain rounds are due at once; the tour is left as it is.
  static void lowGainRoundsDueAtOnce(PlannerNode& node) {
    node.auto_global_planner_low_gain_rounds_ = 0;
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
  /// The global planner runs as soon as the lattice has no frontier. That
  /// is the low-gain rule, which the tour replaces (tour-exploration design
  /// §2.4), so the tour is off.
  static void consultGlobalPlannerAtOnce(PlannerNode& node) {
    node.auto_global_planner_low_gain_rounds_ = 0;
    node.tour_params_.enabled = false;
  }
  /// The tour on or off, and the least cluster gain it takes: a frontier
  /// added by a test has no gain until the first re-check scores it.
  static void setTour(PlannerNode& node, bool enabled,
                      double min_cluster_gain) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.tour_params_.enabled = enabled;
    node.tour_params_.min_cluster_gain = min_cluster_gain;
    node.tour_planner_ = std::make_unique<mgg::TourPlanner>(node.tour_params_);
  }
  /// A tour that solves on any change at once.
  static void solveTourOnEveryChange(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.tour_params_.recompute_interval_s = 0.0;
    node.tour_planner_ = std::make_unique<mgg::TourPlanner>(node.tour_params_);
  }
  /// refreshTour alone, outside a plan request; its target or kNoCluster.
  static mgg::ClusterId refreshTour(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    std::string note;
    const std::optional<mgg::FrontierCluster> target = node.refreshTour(note);
    return target.has_value() ? target->id : mgg::kNoCluster;
  }
  /// The id of the global vertex at (x, y), or -1.
  static int globalVertexAt(PlannerNode& node, double x, double y) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (const auto& entry : node.global_graph_->vertices_map_) {
      if (entry.second != nullptr &&
          (entry.second->state.head<2>() - Eigen::Vector2d(x, y)).norm() <
              1e-6) {
        return entry.first;
      }
    }
    return -1;
  }
  /// Removes the global edge between `a` and `b`, a new revision.
  static void removeGlobalEdge(PlannerNode& node, int a, int b) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.global_graph_->removeEdge(node.findGlobalVertex(a),
                                   node.findGlobalVertex(b));
    ++node.graph_revision_;
  }
  /// Adds a global edge between `a` and `b` and nothing else, as a link
  /// that reuses a vertex does: no new vertex, and no revision.
  static void addGlobalEdgeOnly(PlannerNode& node, int a, int b) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    mgg::Vertex* u = node.findGlobalVertex(a);
    mgg::Vertex* v = node.findGlobalVertex(b);
    node.global_graph_->addEdge(u, v,
                                (u->state - v->state).head<3>().norm());
  }
  /// A vertex of no edge at (x, y) at the root's height, as a merged or
  /// quarantined remnant may be.
  static int addIsolatedGlobalVertex(PlannerNode& node, double x, double y) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.seedGlobalGraph();
    auto* v = new mgg::Vertex(
        node.global_graph_->generateVertexID(),
        mgg::StateVec(x, y, node.global_graph_->getVertex(0)->state.z(), 0.0));
    v->robot_id = static_cast<int>(node.planning_params_.robot_id);
    node.global_graph_->addVertex(v);
    ++node.graph_revision_;
    return v->id;
  }
  /// A global vertex's last scored gain.
  static void setVertexGain(PlannerNode& node, int id, double gain) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.findGlobalVertex(id)->vol_gain.gain = gain;
  }
  static void markGlobalFrontier(PlannerNode& node, int id) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.findGlobalVertex(id)->type = mgg::VertexType::kFrontier;
    ++node.graph_revision_;
  }
  /// A peer's reservation of (x, y), in the planning frame.
  static void receiveReservation(PlannerNode& node, double x, double y) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseArray>();
    msg->header.frame_id = node.world_frame_;
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation.w = 1.0;
    msg->poses.push_back(pose);
    node.onCoordinationExclusions(msg);
  }
  static int tourRoutesFailed(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.tour_routes_failed_;
  }
  /// Where the tour's target is; NaN when it has none.
  static Eigen::Vector3d tourTargetPosition(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    const mgg::TourPlan& plan = node.tour_planner_->plan();
    if (node.tour_planner_->target() == mgg::kNoCluster ||
        plan.clusters.empty()) {
      return Eigen::Vector3d::Constant(std::nan(""));
    }
    return plan.clusters.front().position;
  }
  static mgg::ClusterId tourTarget(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.tour_planner_->target();
  }
  static double tourTargetSince(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.tour_planner_->targetSince();
  }
  /// localPathServesTour with the robot at the origin facing `yaw`.
  static bool localPathServesTour(PlannerNode& node, double yaw,
                                  const Eigen::Vector3d& viewpoint,
                                  const Eigen::Vector3d& target) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.current_state_ = mgg::StateVec(0.0, 0.0, 0.0, yaw);
    return node.localPathServesTour(viewpoint, target);
  }
  static bool bestPathFromGlobalGraph(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.best_path_from_global_graph_;
  }
  /// This robot's bid as it would broadcast it.
  static mgg_msgs::msg::TourBid ownTourBidMsg(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    auto map_read = node.mapReadLease();
    mgg::TourBidData bid = node.ownTourBid();
    bid.robot_id = static_cast<int>(node.planning_params_.robot_id);
    bid.stamp_s = node.now().seconds();
    return toTourBidMsg(bid, node.world_frame_);
  }
  static void receiveTourBid(PlannerNode& node,
                             const mgg_msgs::msg::TourBid& msg) {
    node.onTourBid(std::make_shared<mgg_msgs::msg::TourBid>(msg));
  }
  static void receiveTourAward(PlannerNode& node,
                               const mgg_msgs::msg::TourAward& msg) {
    node.onTourAward(std::make_shared<mgg_msgs::msg::TourAward>(msg));
  }
  static std::vector<int> fleetGroup(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.fleet_->group(node.now().seconds());
  }
  static bool greedyRoute(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    std::string reason;
    return node.runGlobalPlanner(-1, reason);
  }
  static void claim(PlannerNode& node, int robot_id, const mgg::FleetCluster& cluster,
                    double heard_s) {
    mgg::TourBidData bid;
    bid.robot_id = robot_id;
    bid.auctioneer_id = robot_id;
    bid.stamp_s = heard_s;
    bid.clusters = {cluster};
    bid.costs_from_pose = {1.0};
    bid.costs_between = {0.0};
    bid.bundle = {cluster.id};
    node.fleet_->onBid(bid, heard_s);
  }
  static void applyAward(PlannerNode& node, const mgg::TourAwardData& award) {
    node.fleet_->onAward(award, node.now().seconds());
  }
  static std::vector<Eigen::Vector3d> fleetExclusions(PlannerNode& node) {
    return node.fleetExclusions();
  }
  static void localGainRemains(PlannerNode& node, bool remains) {
    node.local_gain_remains_now_ = remains;
  }
  static bool settleIdle(PlannerNode& node, std::string& summary, bool& complete) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.settleIdleRobot(summary, complete);
  }
  static mgg::FleetTickOutput fleetStep(PlannerNode& node, double now_s) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.fleet_->tick(now_s, [&node]() { return node.ownTourBid(); },
                             nullptr, nullptr);
  }
  static void fleetTick(PlannerNode& node, double now_s) {
    node.fleetTick(now_s);
  }
  static bool fleetHasAward(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.fleet_->hasAward();
  }
  /// Robot `robot_id` heard now, bidding nothing: this robot is in a group,
  /// whose auctioneer is the lower ID of the two.
  static void hearPeer(PlannerNode& node, int robot_id) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    mgg::TourBidData bid;
    bid.robot_id = robot_id;
    bid.auctioneer_id = robot_id;
    node.fleet_->onBid(bid, node.now().seconds());
  }
  static std::shared_ptr<mgg_msgs::srv::ReleaseClaims::Response>
  releaseClaims(PlannerNode& node, int robot_id) {
    auto request = std::make_shared<mgg_msgs::srv::ReleaseClaims::Request>();
    request->robot_id = robot_id;
    auto response = std::make_shared<mgg_msgs::srv::ReleaseClaims::Response>();
    node.onReleaseClaims(request, response);
    return response;
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

TEST_F(PlannerNodeTest, AMountedLidarStillFindsTheFrontierAhead) {
  // Lane mgg-sensor (review r0): gain rays cast from the lidar's mount
  // instead of the vertex (0.30 m over the floor here) must not make an
  // explored floor with unknown space ahead look finished. Below and above
  // the vertex, the plan goes forward, to the lattice's edge.
  for (const double mount : {0.0, 0.45, 1.2}) {
    SCOPED_TRACE(mount);
    auto node = makeNode("mounted_" + std::to_string(int(mount * 100)));
    PlannerNodeTestPeer::setSensorMountHeight(*node, mount);
    PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
    PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
    ASSERT_GE(response->path.size(), 2u);
    EXPECT_GE(std::hypot(response->path.back().position.x,
                         response->path.back().position.y),
              1.0);
  }
}

TEST_F(PlannerNodeTest, ASlopeWithNoRoomToTurnWithinReachGetsNoPathNotComplete) {
  // Review r1 (P1): the robot stands on an 11 degree slope whose every
  // turning circle holds an unobserved column: no vertex has room to turn,
  // the root included. Every path end is admitted on the slope without its
  // turn space observed, and none has room to turn within kDepartureMaxM
  // back along its path. The fallback for when no path ends clear sent the
  // best of them anyway; it must not. No path, retried, and not
  // exploration complete, not even with the global planner consulted.
  // The lattice is fine enough, and short enough to stay on the slope, for
  // every end's slope to be measured: grown outward from the robot, a 0.5 m
  // lattice on this holed slope holds three vertices.
  auto node = makeNode("slope_no_way_back");
  PlannerNodeTestPeer::setRobotFootprint(*node, 0.6, 0.2);
  PlannerNodeTestPeer::setLatticeResolution(*node, 0.25);
  PlannerNodeTestPeer::setLattice(*node, {-1.0, -1.0}, {2.0, 1.0});
  PlannerNodeTestPeer::allowUnknownLatticeBody(*node);
  PlannerNodeTestPeer::observeSparseSlope(*node, -1.5, 4.0, -1.5, 1.5, 0.2);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  const mgg::StateVec root =
      PlannerNodeTestPeer::drivingState(*node, 0.0, 0.0, 0.0);
  ASSERT_FALSE(PlannerNodeTestPeer::roomToTurnObserved(*node, root));
  for (int cycle = 0; cycle < 2; ++cycle) {
    SCOPED_TRACE(cycle);
    if (cycle == 1) PlannerNodeTestPeer::consultGlobalPlannerAtOnce(*node);
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    EXPECT_TRUE(response->path.empty())
        << "path of " << response->path.size() << " poses to ("
        << response->path.back().position.x << ", "
        << response->path.back().position.y << ")";
    EXPECT_EQ(response->status, PlannerNode::kStatusNoPath);
  }
}

TEST_F(PlannerNodeTest, ExplorationGoesTheWayTheRobotFaces) {
  // Run 6: a path back the way the robot came kept about 20 % of its score,
  // measured from the last path sent, and robots turned back into an
  // explored hangar. Paths are now measured from the robot's heading. Here
  // the floor and the lattice are the same all round the robot and its
  // lidar sees all round: whichever way it faces, it goes on that way, also
  // right after a path the other way. Toward an exploration target, the
  // target's bearing wins over the heading.
  auto node = makeNode("heading_reference");
  PlannerNodeTestPeer::observeFloor(*node, -2.5, 2.5, -2.5, 2.5);
  PlannerNodeTestPeer::setLattice(*node, {-2.0, -2.0}, {2.0, 2.0});
  PlannerNodeTestPeer::seeAllRound(*node);
  int stamp = 1;
  const auto plan_facing = [&](double yaw) {
    auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    msg->header.stamp.sec = stamp++;
    msg->pose.pose.position.z = 0.075;
    msg->pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg->pose.pose.orientation.w = std::cos(yaw / 2.0);
    PlannerNodeTestPeer::acceptOdometry(*node, msg);
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    EXPECT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
    return response->path.empty()
               ? Eigen::Vector2d(0.0, 0.0)
               : Eigen::Vector2d(response->path.back().position.x,
                                 response->path.back().position.y);
  };
  for (const double yaw : {0.0, M_PI, M_PI / 2.0, -M_PI / 2.0}) {
    SCOPED_TRACE(yaw);
    const Eigen::Vector2d end = plan_facing(yaw);
    const Eigen::Vector2d ahead(std::cos(yaw), std::sin(yaw));
    EXPECT_GT(end.dot(ahead), 1.0) << end.transpose();
  }
  // Facing east, with a target to the west.
  PlannerNodeTestPeer::setExplorationTarget(*node,
                                            Eigen::Vector3d(-10.0, 0.0, 0.0));
  EXPECT_LT(plan_facing(0.0).x(), -1.0);
}

TEST_F(PlannerNodeTest, TheTurnBackHysteresisFollowsThePathActuallySent) {
  // Review r0, M-1: the hysteresis recorded the lattice candidate before it
  // could be dropped as going nowhere or handed over to a global route. The
  // robot faces +x with an exploration target far to the west: the best
  // lattice path goes east, back from the target's bearing.
  const auto scene = [](const std::string& name) {
    auto node = makeNode(name);
    PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
    PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
    PlannerNodeTestPeer::requestExplorationTarget(*node, true, {-10.0, 0.0, 0.0});
    return node;
  };
  const auto plan = [](PlannerNode& node) {
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(node, response);
    return response;
  };
  {
    SCOPED_TRACE("sent");
    auto node = scene("hysteresis_sent");
    const auto response = plan(*node);
    ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
    ASSERT_GT(response->path.back().position.x, 0.5);
    EXPECT_TRUE(PlannerNodeTestPeer::lastTurnedBack(*node));
    // A new target, or an objective, supersedes it.
    PlannerNodeTestPeer::requestExplorationTarget(*node, true, {10.0, 0.0, 0.0});
    EXPECT_FALSE(PlannerNodeTestPeer::lastTurnedBack(*node));
    plan(*node);
    PlannerNodeTestPeer::requestExplorationTarget(*node, true, {-10.0, 0.0, 0.0});
    plan(*node);
    ASSERT_TRUE(PlannerNodeTestPeer::lastTurnedBack(*node));
    auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
    request->objective = mgg_msgs::srv::PlanObjective::Request::NAVIGATE;
    request->goal.position.x = 1.0;
    request->goal.orientation.w = 1.0;
    auto objective = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
    PlannerNodeTestPeer::objective(*node, request, objective);
    EXPECT_FALSE(PlannerNodeTestPeer::lastTurnedBack(*node));
  }
  {
    SCOPED_TRACE("dropped as going nowhere");
    auto node = scene("hysteresis_nowhere");
    PlannerNodeTestPeer::setReachDistance(*node, 10.0);
    const auto response = plan(*node);
    EXPECT_TRUE(response->path.empty());
    EXPECT_FALSE(PlannerNodeTestPeer::lastTurnedBack(*node));
  }
  {
    SCOPED_TRACE("handed over to a global route");
    auto node = scene("hysteresis_handover");
    PlannerNodeTestPeer::addGlobalChainToFrontier(
        *node, {{-0.5, 0.0}, {-1.0, 0.0}, {-1.5, 0.0}}, M_PI);
    PlannerNodeTestPeer::consultGlobalPlannerAtOnce(*node);
    PlannerNodeTestPeer::setLowGainVoxels(*node, 1e9);
    const auto response = plan(*node);
    ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
    ASSERT_TRUE(PlannerNodeTestPeer::bestPathFromGlobalGraph(*node));
    EXPECT_FALSE(PlannerNodeTestPeer::lastTurnedBack(*node));
  }
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

TEST_F(PlannerNodeTest, APathWithLittleGainCountsTowardsGlobalRepositioning) {
  // Run 8, robot_3: every lattice vertex was a frontier, so rounds with a
  // few voxels' gain never counted as low gain, and the robot lapped an
  // explored hangar. A path scoring under low_gain_voxels is a low-gain
  // round, and is still sent while the rounds are not due; one above it
  // counts back.
  auto node = makeNode("little_gain_counts");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::setLowGainVoxels(*node, 1e9);
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  EXPECT_GE(response->path.size(), 2u);
  EXPECT_EQ(PlannerNodeTestPeer::lowGainRounds(*node), 1);
  EXPECT_EQ(PlannerNodeTestPeer::lowGainHandoffs(*node), 0);

  PlannerNodeTestPeer::setLowGainVoxels(*node, 0.0);
  response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  EXPECT_EQ(PlannerNodeTestPeer::lowGainRounds(*node), 0);
}

TEST_F(PlannerNodeTest, ALowGainPathIsHandedOverToTheGlobalPlannerWhenDue) {
  // The low-gain rounds due, a low-gain lattice path is set aside for the
  // global planner: with a global frontier behind the robot worth more than
  // the threshold, it is routed there over the global graph instead. The
  // lattice path scores 721 and the frontier 938 (discounted), so a
  // threshold of 80 voxels at unknown_voxel_gain 10 lies between them.
  auto node = makeNode("low_gain_handoff");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{-0.5, 0.0}, {-1.0, 0.0}, {-1.5, 0.0}}, M_PI);
  PlannerNodeTestPeer::consultGlobalPlannerAtOnce(*node);
  PlannerNodeTestPeer::setLowGainVoxels(*node, 80.0);
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_TRUE(PlannerNodeTestPeer::bestPathFromGlobalGraph(*node));
  EXPECT_EQ(PlannerNodeTestPeer::lowGainHandoffs(*node), 1);
  EXPECT_EQ(PlannerNodeTestPeer::lowGainRounds(*node), 0);
}

TEST_F(PlannerNodeTest, TheHandoffTakesAGlobalFrontierByItsOwnMinimumGain) {
  // Review r0, I-1: the global frontier a low-gain path is handed over for
  // must be worth low_gain_handoff_min_voxels (50), not low_gain_voxels,
  // so that raising the local threshold does not reject more global
  // alternatives. The lattice path scores 721 and the frontier behind the
  // robot 938: with the local threshold far above both, the frontier is
  // still taken; with the handoff minimum above it, it is not.
  for (const bool minimum_above_frontier : {false, true}) {
    SCOPED_TRACE(minimum_above_frontier ? "minimum above" : "default minimum");
    auto node = makeNode(minimum_above_frontier ? "handoff_min_above"
                                                : "handoff_min_default");
    PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
    PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
    PlannerNodeTestPeer::addGlobalChainToFrontier(
        *node, {{-0.5, 0.0}, {-1.0, 0.0}, {-1.5, 0.0}}, M_PI);
    PlannerNodeTestPeer::consultGlobalPlannerAtOnce(*node);
    PlannerNodeTestPeer::setLowGainVoxels(*node, 1e9);
    if (minimum_above_frontier) {
      PlannerNodeTestPeer::setLowGainHandoffMinVoxels(*node, 1e9);
    }
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
    EXPECT_EQ(PlannerNodeTestPeer::bestPathFromGlobalGraph(*node),
              !minimum_above_frontier);
    EXPECT_EQ(PlannerNodeTestPeer::lowGainHandoffs(*node),
              minimum_above_frontier ? 0 : 1);
  }
}

TEST_F(PlannerNodeTest, ALowGainPathTowardTheTourTargetIsLeftToTheTour) {
  // Tour-exploration design §2.4: the tour's target decides when the robot
  // leaves local exploration. A low-gain lattice path toward the target is
  // kept, however many low-gain rounds are due; the greedy global planner
  // does not overrule the tour.
  auto node = makeNode("low_gain_tour");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{0.5, 0.0}, {1.0, 0.0}, {1.5, 0.0}, {2.0, 0.0}, {2.5, 0.0},
              {3.0, 0.0}, {3.5, 0.0}});
  PlannerNodeTestPeer::setTour(*node, true, 0.0);
  PlannerNodeTestPeer::lowGainRoundsDueAtOnce(*node);
  PlannerNodeTestPeer::setLowGainVoxels(*node, 1e9);
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_FALSE(PlannerNodeTestPeer::bestPathFromGlobalGraph(*node));
  EXPECT_NE(PlannerNodeTestPeer::tourTarget(*node), mgg::kNoCluster);
  EXPECT_EQ(PlannerNodeTestPeer::lowGainHandoffs(*node), 0);
  EXPECT_GT(response->path.back().position.x, 0.0);
}

TEST_F(PlannerNodeTest, ALowGainPathIsKeptWhenNoGlobalFrontierIsWorthMore) {
  // The rounds due, and the only global frontier, the one the lattice path
  // itself leaves (374), is worth less than the handoff minimum (50 voxels
  // at unknown_voxel_gain 10): it is not handed over for that, and the
  // low-gain path is sent after all, not exploration complete.
  auto node = makeNode("low_gain_kept");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::consultGlobalPlannerAtOnce(*node);
  PlannerNodeTestPeer::setLowGainVoxels(*node, 1e9);
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_EQ(PlannerNodeTestPeer::lowGainHandoffs(*node), 0);
  EXPECT_FALSE(PlannerNodeTestPeer::bestPathFromGlobalGraph(*node));
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

TEST_F(PlannerNodeTest, ANoGoZoneTurnsTheGlobalRouteAwayOrLeavesNone) {
  // SwarmDeck marks where a robot tripped its tilt guard (run 8: a Scout
  // was sent back over the same debris three times in 4 s). The robot
  // drove a loop: out along y = 0, back along y = 1. A zone on the outward
  // track sends the route to its far end round the loop; zones on both
  // leave no route; an empty set restores the direct one. A set in another
  // frame is ignored.
  auto node = makeNode("no_go_route", "world",
                       {rclcpp::Parameter("PlanningParams.no_go_radius_m", 0.3)});
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 2.5);
  double stamp = 1.0;
  const auto drive = [&](double x, double y) {
    PlannerNodeTestPeer::acceptOdometry(*node, x, y, stamp);
    stamp += 1.0;
  };
  for (double x = 0.0; x <= 4.0 + 1e-9; x += 0.5) drive(x, 0.0);
  drive(4.0, 0.5);
  for (double x = 4.0; x >= -1e-9; x -= 0.5) drive(x, 1.0);
  drive(0.0, 0.5);
  drive(0.0, 0.0);

  auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
  request->objective = mgg_msgs::srv::PlanObjective::Request::NAVIGATE;
  request->goal.position.x = 4.0;
  request->goal.orientation.w = 1.0;
  const auto navigate = [&]() {
    auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
    PlannerNodeTestPeer::objective(*node, request, response);
    return response;
  };
  const auto nearest = [](const auto& path, const Eigen::Vector2d& zone) {
    double d = 1e9;
    for (const auto& pose : path) {
      d = std::min(d, std::hypot(pose.position.x - zone.x(),
                                 pose.position.y - zone.y()));
    }
    return d;
  };
  auto response = navigate();
  ASSERT_EQ(response->status, mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  EXPECT_NEAR(pathLength(response->path), 4.0, 0.60);

  const Eigen::Vector2d outward(2.5, 0.0);
  PlannerNodeTestPeer::receiveNoGoZones(*node, "world", {outward});
  response = navigate();
  ASSERT_EQ(response->status, mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  EXPECT_GT(pathLength(response->path), 4.4);
  EXPECT_GT(nearest(response->path, outward), 0.4);

  PlannerNodeTestPeer::receiveNoGoZones(*node, "world",
                                        {outward, Eigen::Vector2d(2.5, 1.0)});
  response = navigate();
  EXPECT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::UNREACHABLE);
  EXPECT_TRUE(response->path.empty());

  // Another frame's set changes nothing.
  PlannerNodeTestPeer::receiveNoGoZones(*node, "elsewhere", {});
  EXPECT_EQ(navigate()->status,
            mgg_msgs::srv::PlanObjective::Response::UNREACHABLE);

  PlannerNodeTestPeer::receiveNoGoZones(*node, "world", {});
  response = navigate();
  ASSERT_EQ(response->status, mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  EXPECT_NEAR(pathLength(response->path), 4.0, 0.60);
}

TEST_F(PlannerNodeTest, OverlappingNoGoZoneMessagesEndWithTheLastSet) {
  // Review r0, I-4: the subscription was in the node's reentrant group, and
  // under the multithreaded executor an older replacement could take the
  // planner mutex after a newer one and put an obsolete set back, or clear
  // a new hazard. It has a mutually exclusive group of its own, so the
  // messages are handled one at a time, in the order taken. Published
  // back to back, alternating sets and clears end with the last message.
  auto node = makeNode("no_go_order", "world");
  EXPECT_EQ(PlannerNodeTestPeer::noGoZonesCallbackGroupType(*node),
            rclcpp::CallbackGroupType::MutuallyExclusive);
  auto publisher_node = std::make_shared<rclcpp::Node>("no_go_publisher");
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  auto zones_publisher =
      publisher_node->create_publisher<geometry_msgs::msg::PoseArray>(
          "no_go_zones", rclcpp::QoS(1).transient_local().reliable());
  for (int round = 0; round < 20; ++round) {
    for (int i = 0; i < 50; ++i) {
      geometry_msgs::msg::PoseArray msg;
      msg.header.frame_id = "world";
      if (i % 2 == 0) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = round * 100 + i;
        pose.orientation.w = 1.0;
        msg.poses.push_back(pose);
      }
      zones_publisher->publish(msg);
    }
    // The last message of the round sets one zone.
    geometry_msgs::msg::PoseArray last;
    last.header.frame_id = "world";
    geometry_msgs::msg::Pose pose;
    pose.position.x = -1.0 - round;
    pose.orientation.w = 1.0;
    last.poses.push_back(pose);
    zones_publisher->publish(last);
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    bool settled = false;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto zones = PlannerNodeTestPeer::noGoZones(*node);
      if (zones.size() == 1 && zones.front().x() == -1.0 - round) {
        settled = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_TRUE(settled) << "round " << round;
    // Nothing older arrives after it.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const auto zones = PlannerNodeTestPeer::noGoZones(*node);
    ASSERT_EQ(zones.size(), 1u) << "round " << round;
    EXPECT_EQ(zones.front().x(), -1.0 - round);
  }
  executor.cancel();
  spinner.join();
}

TEST_F(PlannerNodeTest, ARobotStandingInANoGoZoneIsRoutedOutOfIt) {
  // The zone is where the robot tripped, so it stands in it when the zone
  // arrives: its global routes may leave it.
  auto node = makeNode("no_go_leave", "world",
                       {rclcpp::Parameter("PlanningParams.no_go_radius_m", 0.3)});
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  double stamp = 1.0;
  for (double x = 0.0; x <= 4.0 + 1e-9; x += 0.5) {
    PlannerNodeTestPeer::acceptOdometry(*node, x, 0.0, stamp);
    stamp += 1.0;
  }
  PlannerNodeTestPeer::receiveNoGoZones(*node, "world", {{4.0, 0.0}});
  auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
  request->objective = mgg_msgs::srv::PlanObjective::Request::NAVIGATE;
  request->goal.position.x = 0.0;
  request->goal.orientation.w = 1.0;
  auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
  PlannerNodeTestPeer::objective(*node, request, response);
  ASSERT_EQ(response->status, mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  EXPECT_NEAR(response->path.back().position.x, 0.0, 0.30);
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

TEST_F(PlannerNodeTest, AQuarantinedRoadmapIsMergedAgainWhenItsTransformReturns) {
  // Run 8, robot_1: a global search outlasted the transform TTL and every
  // peer roadmap was quarantined. The transforms came back, but the peers
  // were beyond communication_range, so no roadmap of theirs was merged
  // again. The returning transform alone now merges the last roadmap
  // received back in.
  TwoPlanners fleet("readmit", /*transform_ttl_s=*/1.0);
  fleet.share();
  const std::size_t merged = PlannerNodeTestPeer::neighbourEdges(*fleet.a, 2);
  ASSERT_GT(merged, 0u);
  // Robot 2 drives out of range: its roadmaps are no longer taken.
  PlannerNodeTestPeer::setCommunicationRange(*fleet.a, 1.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*fleet.a, response);
  ASSERT_TRUE(PlannerNodeTestPeer::isQuarantined(*fleet.a, 2));
  EXPECT_EQ(PlannerNodeTestPeer::neighbourEdges(*fleet.a, 2), 0u);

  // Its next roadmap, out of range, changes nothing; its transform does.
  fleet.share();
  EXPECT_FALSE(PlannerNodeTestPeer::isQuarantined(*fleet.a, 2));
  EXPECT_GT(PlannerNodeTestPeer::neighbourEdges(*fleet.a, 2), 0u);
}

TEST_F(PlannerNodeTest, AQuarantinedNeighbourRoadmapWithholdsExplorationComplete) {
  // Run 8, robot_1: with every peer roadmap quarantined, its global graph
  // had no frontier, and it declared exploration complete. While any is
  // quarantined, a failed search is no path, retried.
  auto node = makeNode("quarantine_not_complete");
  PlannerNodeTestPeer::gainFromUnknownVoxelsOnly(*node);
  PlannerNodeTestPeer::observeFloor(*node, -3.55, 5.55, -2.55, 2.55);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(*node, {{-0.5, 0.0}});
  PlannerNodeTestPeer::addDisconnectedNeighbourVertex(*node, 5, 30.0, 0.0,
                                                      0.3);
  const auto plan = [&node]() {
    PlannerNodeTestPeer::consultGlobalPlannerAtOnce(*node);
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    EXPECT_TRUE(response->path.empty());
    return response->status;
  };
  PlannerNodeTestPeer::quarantine(*node, 5);
  EXPECT_EQ(plan(), PlannerNode::kStatusNoPath);
  PlannerNodeTestPeer::release(*node, 5);
  EXPECT_EQ(plan(), PlannerNode::kStatusComplete);
}

TEST_F(PlannerNodeTest, AGlobalSearchCutShortIsNotExplorationComplete) {
  // Two global frontiers in a room explored end to end: each re-check
  // demotes one. With no time for more than one re-check, the search is
  // cut short with nothing found: no path, retried. With time for both,
  // exploration is complete.
  auto node = makeNode("search_cut_short");
  PlannerNodeTestPeer::gainFromUnknownVoxelsOnly(*node);
  PlannerNodeTestPeer::observeFloor(*node, -3.55, 5.55, -2.55, 2.55);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  const auto plan = [&node]() {
    PlannerNodeTestPeer::consultGlobalPlannerAtOnce(*node);
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    EXPECT_TRUE(response->path.empty());
    return response->status;
  };
  PlannerNodeTestPeer::addGlobalChainToFrontier(*node, {{-0.5, 0.0}});
  PlannerNodeTestPeer::addGlobalChainToFrontier(*node, {{0.0, -0.5}});
  PlannerNodeTestPeer::setGlobalSearchBudget(*node, 0.0);
  EXPECT_EQ(plan(), PlannerNode::kStatusNoPath);
  PlannerNodeTestPeer::setGlobalSearchBudget(*node, 10.0);
  EXPECT_EQ(plan(), PlannerNode::kStatusComplete);
}

TEST_F(PlannerNodeTest, AFrontierFoundButNotRoutedToLeavesTheSearchInconclusive) {
  // Review r0, I-6: the robot stands on the best frontier of the global
  // graph, so the route to it fails ("already at the goal"), and another
  // frontier 1 m away is left unchecked by a zero time budget, or checked
  // with time to spare. Either way the search found a frontier it could not
  // route to: its failure is no answer, and exploration is not complete.
  for (const double budget : {0.0, 10.0}) {
    SCOPED_TRACE(budget);
    auto node = makeNode("found_not_routed_" + std::to_string(int(budget)));
    PlannerNodeTestPeer::observeFloor(*node, -1.5, 0.5, -0.5, 0.5);
    PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
    // The other frontier, deeper in the mapped floor, sees less unknown
    // space than the one the robot stands on; its last gain is lower too,
    // so the budget leaves it for later.
    const int other =
        PlannerNodeTestPeer::addGlobalChainToFrontier(*node, {{-1.0, 0.0}});
    PlannerNodeTestPeer::markGlobalFrontier(*node, 0);
    PlannerNodeTestPeer::setVertexGain(*node, 0, 1e6);
    PlannerNodeTestPeer::setVertexGain(*node, other, 1.0);
    PlannerNodeTestPeer::setGlobalSearchBudget(*node, budget);
    std::string reason;
    EXPECT_FALSE(PlannerNodeTestPeer::runGlobalPlanner(*node, reason));
    EXPECT_NE(reason.find("already at the goal"), std::string::npos) << reason;
    EXPECT_FALSE(PlannerNodeTestPeer::completionWithheld(*node).empty());
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

TEST_F(PlannerNodeTest, AStandingStartInItsLidarsBlindDiskIsNotBoxedIn) {
  // Run 6: robot_3 stood where it was placed for the whole run, robot_1
  // for 18 minutes. Their lidars never saw the ground within about 2 m of
  // them, and a wall (for robot_3, a peer's shadow) stood ahead: every way
  // out turned where they stood, over ground not observed, so each was
  // boxed in with no departure. Here the floor is observed from 0.6 m out,
  // and a wall 0.8 m ahead of the robot, which faces +y. Standing at its
  // start, as its keyframes show, it has room to turn and gets a path, on
  // this side of the wall. Side walls 0.4 m either side close the lattice's
  // diagonals: from the robot they are 45-degree turns, which the turn rule
  // allows anywhere.
  const auto scene = [](const std::string& name,
                        const KeyframeTrajectory& keyframes) {
    auto node = makeNode(name);
    PlannerNodeTestPeer::observeRaisedRing(*node, 0.6, 3.5, 0.0);
    PlannerNodeTestPeer::observeWall(*node, -1.0, 1.0, 0.8);
    PlannerNodeTestPeer::observeWallAlongY(*node, 0.2, 0.8, 0.4);
    PlannerNodeTestPeer::observeWallAlongY(*node, 0.2, 0.8, -0.4);
    PlannerNodeTestPeer::setHangingRootReach(*node, 1.2);
    PlannerNodeTestPeer::serveMap(*node, "component:test", 0);
    auto source = std::make_unique<TrajectoryInMemory>();
    source->trajectory = keyframes;
    PlannerNodeTestPeer::setKeyframeSource(*node, std::move(source));
    return node;
  };
  const auto facing_north = [](double x, int stamp) {
    auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    msg->header.stamp.sec = stamp;
    msg->pose.pose.position.x = x;
    msg->pose.pose.position.z = 0.075;
    msg->pose.pose.orientation.z = std::sin(M_PI / 4.0);
    msg->pose.pose.orientation.w = std::cos(M_PI / 4.0);
    return msg;
  };

  auto standing = scene("standing_start", keyframesAlong({{0.0, 0.0}}));
  PlannerNodeTestPeer::acceptOdometry(*standing, facing_north(0.0, 1));
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*standing, response);
  EXPECT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response->path.size(), 2u);
  const geometry_msgs::msg::Point& end = response->path.back().position;
  EXPECT_GE(std::hypot(end.x, end.y), 0.4) << end.x << ", " << end.y;
  EXPECT_LT(end.y, 0.8);
  EXPECT_EQ(PlannerNodeTestPeer::boxedInWithoutDeparture(*standing), 0);

  // The same place, reached from 0.6 m west: the robot has left its start,
  // and the ground it never saw is unobserved. Boxed in, with no way out.
  auto moved =
      scene("moved_from_start", keyframesAlong({{-0.6, 0.0}, {0.0, 0.0}}));
  PlannerNodeTestPeer::acceptOdometry(*moved, facing_north(-0.6, 1));
  PlannerNodeTestPeer::acceptOdometry(*moved, facing_north(0.0, 2));
  response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*moved, response);
  EXPECT_TRUE(response->path.empty())
      << "path of " << response->path.size() << " poses";
  EXPECT_EQ(PlannerNodeTestPeer::boxedInWithoutDeparture(*moved), 1);
}

TEST_F(PlannerNodeTest, APlannerRestartedBesideAnUnobservedDropIsNotAtItsStart) {
  // Review r0, I-1: a planner restarted mid-run takes its first odometry
  // wherever the robot then is, here 0.3 m from a drop whose bottom it
  // never saw (robot_0's run-5 ledge). The edge 0.2 m on to the ledge ends
  // on the floor, with the drop under the leading half of the 0.6 m body.
  // Without keyframes that show the robot never left its start (the
  // rebuild turned off, so no trajectory source; or a trajectory that left
  // home), no disk counts as observed ground, and the edge stays
  // unobserved. Only keyframes all at the robot's start exempt it.
  const auto scene = [](const std::string& name,
                        std::vector<rclcpp::Parameter> extra = {}) {
    auto node = makeNode(name, "world", std::move(extra));
    PlannerNodeTestPeer::setRobotFootprint(*node, 0.6, 0.2);
    PlannerNodeTestPeer::observeFloor(*node, -2.0, 0.3, -1.0, 1.0);
    // The drop: open space down to 0.5 m below the floor, nothing seen
    // under it.
    PlannerNodeTestPeer::observeFreeBox(*node, Eigen::Vector3d(1.0, 0.0, 0.1),
                                        Eigen::Vector3d(1.4, 2.0, 1.2));
    PlannerNodeTestPeer::setHangingRootReach(*node, 2.0);
    return node;
  };
  const auto onto_the_drop = [](PlannerNode& node) {
    return PlannerNodeTestPeer::edgeStatus(
        node, PlannerNodeTestPeer::drivingState(node, 0.0, 0.0, 0.0).head<3>(),
        PlannerNodeTestPeer::drivingState(node, 0.2, 0.0, 0.0).head<3>());
  };
  const auto with_keyframes = [](PlannerNode& node,
                                 const KeyframeTrajectory& keyframes) {
    PlannerNodeTestPeer::serveMap(node, "component:test", 0);
    auto source = std::make_unique<TrajectoryInMemory>();
    source->trajectory = keyframes;
    PlannerNodeTestPeer::setKeyframeSource(node, std::move(source));
  };

  auto no_rebuild = scene("restart_no_rebuild",
                          {rclcpp::Parameter("roadmap_rebuild.enable", false)});
  PlannerNodeTestPeer::acceptOdometry(*no_rebuild, 0.0, 0.0, 1.0);
  EXPECT_FALSE(PlannerNodeTestPeer::standingStart(*no_rebuild).has_value());
  EXPECT_EQ(onto_the_drop(*no_rebuild),
            mgg::ProjectedEdgeStatus::kGroundUnobserved);

  // Keyframes from home 1.5 m back to here: the robot left its start.
  auto left_home = scene("restart_left_home");
  with_keyframes(*left_home, keyframesAlong({{-1.5, 0.0}, {0.0, 0.0}}));
  PlannerNodeTestPeer::acceptOdometry(*left_home, 0.0, 0.0, 1.0);
  EXPECT_FALSE(PlannerNodeTestPeer::standingStart(*left_home).has_value());
  EXPECT_EQ(onto_the_drop(*left_home),
            mgg::ProjectedEdgeStatus::kGroundUnobserved);

  // Every keyframe here: the robot never left its start, and its disk
  // counts as observed ground.
  auto at_start = scene("restart_at_start");
  with_keyframes(*at_start, keyframesAlong({{0.0, 0.0}}));
  PlannerNodeTestPeer::acceptOdometry(*at_start, 0.0, 0.0, 1.0);
  ASSERT_TRUE(PlannerNodeTestPeer::standingStart(*at_start).has_value());
  EXPECT_NE(onto_the_drop(*at_start),
            mgg::ProjectedEdgeStatus::kGroundUnobserved);
  // Crept 0.3 m on, it still stands at its start; the disk stays where it
  // stood (review r0, M-3).
  PlannerNodeTestPeer::acceptOdometry(*at_start, -0.3, 0.0, 2.0);
  const auto crept = PlannerNodeTestPeer::standingStart(*at_start);
  ASSERT_TRUE(crept.has_value());
  EXPECT_LT(crept->center.norm(), 1e-9) << crept->center.transpose();
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

TEST_F(PlannerNodeTest, TheFirstFailedSearchAfterARebuildDroppedFrontiersIsNotComplete) {
  // Review r0, I-2: a rebuild replaces the global graph, and this robot's
  // frontiers go with it. The next low-gain round, in a room explored end
  // to end, found no frontier and made exploration complete. The first
  // failed global search after a rebuild that dropped any is no path; the
  // next one, with nothing found since, is exploration complete.
  auto node = makeNode("rebuild_dropped_frontiers");
  PlannerNodeTestPeer::gainFromUnknownVoxelsOnly(*node);
  PlannerNodeTestPeer::observeFloor(*node, -3.55, 5.55, -2.55, 2.55);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(*node, {{-0.5, 0.0}, {-1.0, 0.0}});
  PlannerNodeTestPeer::serveMap(*node, "component:test", 0);
  auto source = std::make_unique<TrajectoryInMemory>();
  source->trajectory = keyframesAlongX(0.0, 1.0);
  PlannerNodeTestPeer::setKeyframeSource(*node, std::move(source));
  ASSERT_TRUE(PlannerNodeTestPeer::rebuildRoadmap(
      *node, PlannerNode::RoadmapRebuildTrigger::kPoseUnlinkable));

  const auto plan = [&node]() {
    PlannerNodeTestPeer::consultGlobalPlannerAtOnce(*node);
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    EXPECT_TRUE(response->path.empty())
        << "path of " << response->path.size() << " poses";
    return response->status;
  };
  EXPECT_EQ(plan(), PlannerNode::kStatusNoPath);
  EXPECT_EQ(plan(), PlannerNode::kStatusComplete);
}

TEST_F(PlannerNodeTest, AQuarantinedRoadmapSurvivesARebuildAndReturnsWithItsTransform) {
  // Review r0, I-7: a roadmap rebuild replaced the global graph and with it
  // the quarantine, so neither the completion guard nor the re-admission on
  // the transform's return knew of the neighbour any more. Robot 2's
  // transform expires while it is out of range; the graph is rebuilt from
  // the keyframes; exploration may not be complete until the transform
  // returns, which merges the cached roadmap into the rebuilt graph.
  TwoPlanners fleet("rebuild_quarantine", /*transform_ttl_s=*/1.0);
  fleet.share();
  ASSERT_GT(PlannerNodeTestPeer::neighbourEdges(*fleet.a, 2), 0u);
  PlannerNodeTestPeer::setCommunicationRange(*fleet.a, 1.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*fleet.a, response);
  ASSERT_TRUE(PlannerNodeTestPeer::isQuarantined(*fleet.a, 2));

  PlannerNodeTestPeer::serveMap(*fleet.a, "component:test", 0);
  auto source = std::make_unique<TrajectoryInMemory>();
  source->trajectory = keyframesAlongX(0.0, 4.0);
  PlannerNodeTestPeer::setKeyframeSource(*fleet.a, std::move(source));
  ASSERT_TRUE(PlannerNodeTestPeer::rebuildRoadmap(
      *fleet.a, PlannerNode::RoadmapRebuildTrigger::kPoseUnlinkable));
  EXPECT_EQ(PlannerNodeTestPeer::neighbourEdges(*fleet.a, 2), 0u);
  EXPECT_FALSE(PlannerNodeTestPeer::completionWithheld(*fleet.a).empty());

  // Its next roadmap is out of range; its transform returns.
  PlannerNodeTestPeer::receiveGraph(*fleet.a,
                                    PlannerNodeTestPeer::ownGraph(*fleet.b));
  EXPECT_FALSE(PlannerNodeTestPeer::completionWithheld(*fleet.a).empty());
  PlannerNodeTestPeer::receiveTransform(*fleet.a, "robot_0/odom",
                                        "robot_1/odom", 5.0, 0.0);
  EXPECT_GT(PlannerNodeTestPeer::neighbourEdges(*fleet.a, 2), 0u);
  EXPECT_TRUE(PlannerNodeTestPeer::completionWithheld(*fleet.a).empty())
      << PlannerNodeTestPeer::completionWithheld(*fleet.a);
}

TEST_F(PlannerNodeTest, TheTourHeadsForItsTargetWithoutWaitingForLowGainRounds) {
  // exploredDeadEnd: the lattice has no gain, and the tour's only cluster
  // is the frontier 2.5 m behind the robot. The low-gain rule waits
  // auto_global_planner_low_gain_rounds (15) cycles and sends nothing; the
  // tour routes to the frontier at once (tour-exploration design §2.4).
  // That route starts with a turn the robot has no room for, so it backs
  // out toward the frontier first, as any withheld route does.
  for (const bool tour : {false, true}) {
    SCOPED_TRACE(tour ? "tour on" : "tour off");
    int frontier = -1;
    auto node =
        exploredDeadEnd(tour ? "tour_at_once" : "tour_off_waits", frontier);
    PlannerNodeTestPeer::setTour(*node, tour, 0.0);

    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    if (!tour) {
      EXPECT_EQ(response->status, PlannerNode::kStatusNoPath);
      EXPECT_EQ(PlannerNodeTestPeer::routeSharpTurnFallbacks(*node), 0);
      EXPECT_EQ(PlannerNodeTestPeer::tourTarget(*node), mgg::kNoCluster);
      continue;
    }
    EXPECT_NE(PlannerNodeTestPeer::tourTarget(*node), mgg::kNoCluster);
    EXPECT_EQ(PlannerNodeTestPeer::routeSharpTurnFallbacks(*node), 1);
    expectReverseDepartureFromDeadEnd(*node, *response);
  }
}

TEST_F(PlannerNodeTest, LocalExplorationTowardTheTourTargetIsKept) {
  // Floor mapped to x = 4 and unknown beyond: the lattice has gain ahead,
  // and so does the tour: a global frontier 3.5 m ahead, and the frontier
  // the accepted lattice path leaves in the global graph. The robot
  // explores locally toward its target rather than taking the global
  // route.
  auto node = makeNode("tour_local_toward");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{0.5, 0.0}, {1.0, 0.0}, {1.5, 0.0}, {2.0, 0.0}, {2.5, 0.0},
              {3.0, 0.0}, {3.5, 0.0}});
  PlannerNodeTestPeer::setTour(*node, true, 0.0);

  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_FALSE(PlannerNodeTestPeer::bestPathFromGlobalGraph(*node));
  EXPECT_NE(PlannerNodeTestPeer::tourTarget(*node), mgg::kNoCluster);
  EXPECT_GT(response->path.back().position.x, 0.0);
}

TEST_F(PlannerNodeTest, TheTourTargetIsJudgedInTheLatticeFrameAtTheRobotsHeading) {
  // Review r0, I-1: the lattice spans x in [-1, 3] and y in [-1, 1] along
  // the robot's heading (buildGridGraph turns it by the heading). Facing
  // +y, its box covers world x in [-1, 1] and y in [-1, 3].
  auto node = makeNode("tour_lattice_frame");
  PlannerNodeTestPeer::setLattice(*node, {-1.0, -1.0}, {3.0, 1.0});
  const double north = M_PI / 2.0;
  // (2, 0) is 2 m to the robot's right, outside the box, and a path north
  // heads 90 degrees away from it.
  EXPECT_FALSE(PlannerNodeTestPeer::localPathServesTour(
      *node, north, {0.0, 2.0, 0.0}, {2.0, 0.0, 0.0}));
  // (0, 2) is 2 m ahead, inside the box: any local path serves it.
  EXPECT_TRUE(PlannerNodeTestPeer::localPathServesTour(
      *node, north, {-1.0, 0.0, 0.0}, {0.0, 2.0, 0.0}));
  // Far ahead, served by a path heading its way.
  EXPECT_TRUE(PlannerNodeTestPeer::localPathServesTour(
      *node, north, {0.3, 2.0, 0.0}, {0.0, 20.0, 0.0}));
  // Facing +x, (2, 0) is inside the box.
  EXPECT_TRUE(PlannerNodeTestPeer::localPathServesTour(
      *node, 0.0, {0.0, 2.0, 0.0}, {2.0, 0.0, 0.0}));
}

TEST_F(PlannerNodeTest, AnEdgeAloneThatConnectsAClusterReachesTheTour) {
  // Review r0, I-4: the frontier at (6, 0) is cut off from the robot by a
  // missing edge between (4, 0) and (5, 0), so the tour has no reachable
  // cluster. Adding that edge, with no new vertex, makes the cluster
  // reachable; the tour's cached distances must not hide it.
  auto node = makeNode("tour_edge_only");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 7.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0}, {5.0, 0.0},
              {6.0, 0.0}});
  const int a = PlannerNodeTestPeer::globalVertexAt(*node, 4.0, 0.0);
  const int b = PlannerNodeTestPeer::globalVertexAt(*node, 5.0, 0.0);
  ASSERT_GE(a, 0);
  ASSERT_GE(b, 0);
  PlannerNodeTestPeer::removeGlobalEdge(*node, a, b);
  PlannerNodeTestPeer::setTour(*node, true, 0.0);
  PlannerNodeTestPeer::solveTourOnEveryChange(*node);

  EXPECT_EQ(PlannerNodeTestPeer::refreshTour(*node), mgg::kNoCluster);
  PlannerNodeTestPeer::addGlobalEdgeOnly(*node, a, b);
  EXPECT_NE(PlannerNodeTestPeer::refreshTour(*node), mgg::kNoCluster);
}

TEST_F(PlannerNodeTest, ATourTargetThatCannotBeRoutedToIsSetAside) {
  // Review r0, I-2: exploredDeadEnd, and a second frontier at (-0.5, 1.5)
  // beyond the corridor's wall, the tour's first choice. A vertex of no
  // edge was left at that very spot before it, and a route to a position
  // ends at the vertex found there first: the tour costs the frontier's
  // own vertex, the route cannot reach the one it finds. The failed target
  // is set aside, the tour takes the frontier behind instead, and nothing
  // changed that could make the first reachable: it is not tried again.
  int behind = -1;
  auto node = exploredDeadEnd("tour_route_fails", behind);
  PlannerNodeTestPeer::addIsolatedGlobalVertex(*node, -0.5, 1.5);
  PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{-0.5, 0.75}, {-0.5, 1.5}}, M_PI / 2.0);
  PlannerNodeTestPeer::setTour(*node, true, 0.0);

  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_EQ(PlannerNodeTestPeer::tourRoutesFailed(*node), 1);
  EXPECT_EQ(PlannerNodeTestPeer::tourTarget(*node), mgg::kNoCluster);
  for (int cycle = 0; cycle < 3; ++cycle) {
    SCOPED_TRACE("cycle " + std::to_string(cycle + 2));
    response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    EXPECT_EQ(PlannerNodeTestPeer::tourRoutesFailed(*node), 1);
    const Eigen::Vector3d target =
        PlannerNodeTestPeer::tourTargetPosition(*node);
    EXPECT_NEAR(target.x(), -2.5, 1e-6);
    EXPECT_NEAR(target.y(), 0.0, 1e-6);
  }
}

TEST_F(PlannerNodeTest, ATourTargetTheRobotStandsOnIsReachedNotAFailedRoute) {
  // Review r0, I-2: exploredDeadEnd with the root, where the robot stands,
  // a frontier too. The tour costs that cluster zero, and a route from the
  // robot's vertex to itself is "already at the goal". It counts as
  // reached: set aside with no route tried, and the tour takes the
  // frontier behind next.
  int behind = -1;
  auto node = exploredDeadEnd("tour_stands_on_target", behind);
  PlannerNodeTestPeer::markGlobalFrontier(*node, 0);
  PlannerNodeTestPeer::setTour(*node, true, 0.0);

  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_EQ(PlannerNodeTestPeer::tourRoutesFailed(*node), 0);
  EXPECT_EQ(PlannerNodeTestPeer::tourTarget(*node), mgg::kNoCluster);
  response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_EQ(PlannerNodeTestPeer::tourRoutesFailed(*node), 0);
  EXPECT_NEAR(PlannerNodeTestPeer::tourTargetPosition(*node).x(), -2.5, 1e-6);
}

TEST_F(PlannerNodeTest, AResumedTourRouteIsGivenUpWhenAPeerReservesItsTarget) {
  // Review r0, I-3: the lattice (x in [-1, 3]) explores ahead, and the
  // tour's target is a frontier 8 m behind, outside the lattice, taken
  // before the lattice's own frontier ahead joined the graph (the next
  // solve is a recompute interval away): the robot routes to it over the
  // global graph and resumes that route. A peer then reserves the
  // frontier; the next request gives the route up rather than resume it,
  // although refreshTour does not run while a route is resumed.
  auto node = makeNode("tour_resume_reserved");
  PlannerNodeTestPeer::setLattice(*node, {-1.0, -1.5}, {3.0, 1.5});
  PlannerNodeTestPeer::observeFloor(*node, -9.0, 4.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  const int frontier = PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{-1.0, 0.0}, {-2.0, 0.0}, {-3.0, 0.0}, {-4.0, 0.0},
              {-5.0, 0.0}, {-6.0, 0.0}, {-7.0, 0.0}, {-8.0, 0.0}},
      M_PI);
  PlannerNodeTestPeer::setTour(*node, true, 0.0);
  ASSERT_NE(PlannerNodeTestPeer::refreshTour(*node), mgg::kNoCluster);

  const auto plan = [&node]() {
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    return response;
  };
  auto response = plan();
  ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_TRUE(PlannerNodeTestPeer::bestPathFromGlobalGraph(*node));
  ASSERT_EQ(PlannerNodeTestPeer::repositioningTarget(*node), frontier);
  response = plan();  // resumed
  ASSERT_TRUE(PlannerNodeTestPeer::bestPathFromGlobalGraph(*node));
  ASSERT_EQ(PlannerNodeTestPeer::repositioningTarget(*node), frontier);

  PlannerNodeTestPeer::receiveReservation(*node, -8.0, 0.0);
  response = plan();
  EXPECT_FALSE(PlannerNodeTestPeer::repositioningOngoing(*node));
  EXPECT_FALSE(PlannerNodeTestPeer::bestPathFromGlobalGraph(*node));
  EXPECT_FALSE(PlannerNodeTestPeer::tourTargetPosition(*node).x() < -7.0);
}

TEST_F(PlannerNodeTest, AReachedTourTargetThatIsStillAFrontierIsReleasedOnce) {
  // The robot stands within global_frontier_reach_m of the frontier 3.5 m
  // ahead, which stays a frontier: it is released once, when first found
  // reached, and the free solve may take it again. After that it is kept
  // until it is explored, reassigned or cannot be routed to: releasing it
  // every cycle would solve every cycle and restamp the target's claim
  // (targetSince) each time.
  auto node = makeNode("tour_reached_once");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{0.5, 0.0}, {1.0, 0.0}, {1.5, 0.0}, {2.0, 0.0}, {2.5, 0.0},
              {3.0, 0.0}, {3.5, 0.0}});
  PlannerNodeTestPeer::setTour(*node, true, 0.0);

  const auto plan = [&node]() {
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  };
  plan();  // the first solve: the target is taken
  ASSERT_NE(PlannerNodeTestPeer::tourTarget(*node), mgg::kNoCluster);
  plan();  // found reached: released, and taken again
  const mgg::ClusterId target = PlannerNodeTestPeer::tourTarget(*node);
  ASSERT_NE(target, mgg::kNoCluster);
  const double since = PlannerNodeTestPeer::tourTargetSince(*node);
  for (int cycle = 0; cycle < 3; ++cycle) {
    plan();
    EXPECT_EQ(PlannerNodeTestPeer::tourTarget(*node), target);
    EXPECT_DOUBLE_EQ(PlannerNodeTestPeer::tourTargetSince(*node), since);
  }
}

TEST_F(PlannerNodeTest, APeersBidJoinsTheGroupOnlyWithATransformToIt) {
  TwoPlanners fleet("fleet_transform");
  const mgg_msgs::msg::TourBid bid = PlannerNodeTestPeer::ownTourBidMsg(*fleet.b);
  // No shared frame yet: robot 1 tours alone (tour-exploration design §4).
  PlannerNodeTestPeer::receiveTourBid(*fleet.a, bid);
  EXPECT_EQ(PlannerNodeTestPeer::fleetGroup(*fleet.a), std::vector<int>{1});
  // A transform appears: robot 2 joins robot 1's group.
  PlannerNodeTestPeer::receiveTransform(*fleet.a, "robot_0/odom",
                                        "robot_1/odom", 5.0, 0.0);
  PlannerNodeTestPeer::receiveTourBid(*fleet.a, bid);
  EXPECT_EQ(PlannerNodeTestPeer::fleetGroup(*fleet.a),
            (std::vector<int>{1, 2}));
}

TEST_F(PlannerNodeTest, AMalformedBidIsIgnored) {
  // Review Focus 1: one cost more than the bid has clusters.
  TwoPlanners fleet("fleet_malformed");
  PlannerNodeTestPeer::receiveTransform(*fleet.a, "robot_0/odom",
                                        "robot_1/odom", 5.0, 0.0);
  mgg_msgs::msg::TourBid bid = PlannerNodeTestPeer::ownTourBidMsg(*fleet.b);
  bid.costs_from_pose.push_back(1.0);
  PlannerNodeTestPeer::receiveTourBid(*fleet.a, bid);
  EXPECT_EQ(PlannerNodeTestPeer::fleetGroup(*fleet.a), std::vector<int>{1});
}

TEST_F(PlannerNodeTest, OnlyTheAuctioneerReleasesClaims) {
  TwoPlanners fleet("fleet_release");
  PlannerNodeTestPeer::receiveTransform(*fleet.a, "robot_0/odom",
                                        "robot_1/odom", 5.0, 0.0);
  PlannerNodeTestPeer::receiveTourBid(
      *fleet.a, PlannerNodeTestPeer::ownTourBidMsg(*fleet.b));
  PlannerNodeTestPeer::receiveTransform(*fleet.b, "robot_1/odom",
                                        "robot_0/odom", -5.0, 0.0);
  PlannerNodeTestPeer::receiveTourBid(
      *fleet.b, PlannerNodeTestPeer::ownTourBidMsg(*fleet.a));
  PlannerNodeTestPeer::hearPeer(*fleet.a, 3);
  EXPECT_TRUE(PlannerNodeTestPeer::releaseClaims(*fleet.a, 3)->success);
  const auto refused = PlannerNodeTestPeer::releaseClaims(*fleet.b, 3);
  EXPECT_FALSE(refused->success);
  EXPECT_NE(refused->message.find("robot 1"), std::string::npos)
      << refused->message;
}

TEST_F(PlannerNodeTest, TheAuctioneerCallsAndAwardsOnItsFleetTimer) {
  TwoPlanners fleet("fleet_award");
  PlannerNodeTestPeer::receiveTransform(*fleet.a, "robot_0/odom",
                                        "robot_1/odom", 5.0, 0.0);
  PlannerNodeTestPeer::receiveTourBid(
      *fleet.a, PlannerNodeTestPeer::ownTourBidMsg(*fleet.b));
  const double t0 = fleet.a->now().seconds();
  PlannerNodeTestPeer::fleetTick(*fleet.a, t0);
  EXPECT_FALSE(PlannerNodeTestPeer::fleetHasAward(*fleet.a));  // call out
  PlannerNodeTestPeer::fleetTick(*fleet.a, t0 + 1.1);  // past the deadline
  EXPECT_TRUE(PlannerNodeTestPeer::fleetHasAward(*fleet.a));
}

TEST_F(PlannerNodeTest, AnEmptyAwardAfterARebuildDroppedFrontiersIsNotYetComplete) {
  // TheFirstFailedSearchAfterARebuildDroppedFrontiersIsNotComplete, in a
  // group: the award answering this robot's request is its failed search.
  // The first one after the rebuild is no path; the next is exploration
  // complete.
  auto node = makeNode("fleet_rebuild_dropped");
  PlannerNodeTestPeer::gainFromUnknownVoxelsOnly(*node);
  PlannerNodeTestPeer::observeFloor(*node, -3.55, 5.55, -2.55, 2.55);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(*node, {{-0.5, 0.0}, {-1.0, 0.0}});
  PlannerNodeTestPeer::serveMap(*node, "component:test", 0);
  auto source = std::make_unique<TrajectoryInMemory>();
  source->trajectory = keyframesAlongX(0.0, 1.0);
  PlannerNodeTestPeer::setKeyframeSource(*node, std::move(source));
  ASSERT_TRUE(PlannerNodeTestPeer::rebuildRoadmap(
      *node, PlannerNode::RoadmapRebuildTrigger::kPoseUnlinkable));
  PlannerNodeTestPeer::setTour(*node, true, 0.0);
  PlannerNodeTestPeer::hearPeer(*node, 2);

  const auto plan = [&node]() {
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    EXPECT_TRUE(response->path.empty())
        << "path of " << response->path.size() << " poses";
    return response->status;
  };
  // Nothing on its tour and no local path: it asks for an auction.
  EXPECT_EQ(plan(), PlannerNode::kStatusNoPath);
  const double t0 = node->now().seconds();
  PlannerNodeTestPeer::fleetTick(*node, t0);        // the call
  PlannerNodeTestPeer::fleetTick(*node, t0 + 1.1);  // the award: nothing
  ASSERT_TRUE(PlannerNodeTestPeer::fleetHasAward(*node));
  EXPECT_EQ(plan(), PlannerNode::kStatusNoPath);
  EXPECT_EQ(plan(), PlannerNode::kStatusComplete);
}

TEST_F(PlannerNodeTest, InAPersistentChainOthersClaimsOverrideOurAwardedTour) {
  // 1 <-> 2 <-> 3: robot 1 cannot hear robot 3's overlapping claim.
  // Robot 2 can, and must respect it even when robot 1 awards it that area.
  const auto robot = [](int id) {
    auto node = makeNode("chain_" + std::to_string(id),
                         "chain_" + std::to_string(id) + "/odom",
                         {rclcpp::Parameter("PlanningParams.robot_id", id),
                          rclcpp::Parameter("neighbour_pose_source", "topic")});
    PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
    PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
    PlannerNodeTestPeer::setTour(*node, true, 0.0);
    PlannerNodeTestPeer::solveTourOnEveryChange(*node);
    return node;
  };
  auto a = robot(1), b = robot(2), c = robot(3);
  for (const auto& edge : std::vector<std::pair<int, int>>{
           {1, 2}, {2, 1}, {2, 3}, {3, 2}}) {
    auto& receiver = edge.first == 1 ? a : edge.first == 2 ? b : c;
    PlannerNodeTestPeer::receiveTransform(
        *receiver, "chain_" + std::to_string(edge.first) + "/odom",
        "chain_" + std::to_string(edge.second) + "/odom", 0.0, 0.0);
  }
  PlannerNodeTestPeer::addGlobalChainToFrontier(*b, {{3.0, 0.0}});
  PlannerNodeTestPeer::addGlobalChainToFrontier(*c, {{3.5, 0.0}});
  ASSERT_NE(PlannerNodeTestPeer::refreshTour(*c), mgg::kNoCluster);
  const auto b_content = PlannerNodeTestPeer::ownTourBidMsg(*b);
  ASSERT_EQ(b_content.clusters.size(), 1u);
  mgg_msgs::msg::TourAward award;
  award.header.frame_id = "chain_1/odom";
  award.auctioneer_id = 1;
  award.clusters = b_content.clusters;
  mgg_msgs::msg::TourBundle bundle;
  bundle.robot_id = 2;
  bundle.clusters = {b_content.clusters.front().id};
  award.bundles = {bundle};
  for (int round = 1; round <= 4; ++round) {
    PlannerNodeTestPeer::receiveTourBid(
        *b, PlannerNodeTestPeer::ownTourBidMsg(*a));
    const auto middle_bid = PlannerNodeTestPeer::ownTourBidMsg(*b);
    ASSERT_EQ(middle_bid.auctioneer_id, 1);
    PlannerNodeTestPeer::receiveTourBid(*a, middle_bid);
    PlannerNodeTestPeer::receiveTourBid(*c, middle_bid);
    const auto end_bid = PlannerNodeTestPeer::ownTourBidMsg(*c);
    ASSERT_EQ(end_bid.auctioneer_id, 3);
    PlannerNodeTestPeer::receiveTourBid(*b, end_bid);
    award.auction_id = round;
    award.header.stamp = a->now();
    PlannerNodeTestPeer::receiveTourAward(*b, award);
    ASSERT_TRUE(PlannerNodeTestPeer::fleetHasAward(*b));
    EXPECT_EQ(PlannerNodeTestPeer::fleetGroup(*a), (std::vector<int>{1, 2}));
    EXPECT_EQ(PlannerNodeTestPeer::fleetGroup(*c), (std::vector<int>{2, 3}));
    EXPECT_EQ(PlannerNodeTestPeer::refreshTour(*b), mgg::kNoCluster)
        << "round " << round << ": robot 3's claim precedes our bundle";
  }
}

TEST_F(PlannerNodeTest, APeerImportedFrontierWithUnknownVolumeFormsATourCluster) {
  TwoPlanners fleet("peer_frontier_gain");
  PlannerNodeTestPeer::gainFromUnknownVoxelsOnly(*fleet.a);
  // The frontier is at x=9 in a's frame, far from a's local lattice.
  // Graph/Vertex messages carry counts and the mark, but no scalar gain.
  auto graph = PlannerNodeTestPeer::ownGraph(*fleet.b);
  auto frontier = std::max_element(
      graph.vertices.begin(), graph.vertices.end(),
      [](const auto& a, const auto& b) {
        return a.pose.position.x < b.pose.position.x;
      });
  ASSERT_NE(frontier, graph.vertices.end());
  ASSERT_NEAR(frontier->pose.position.x, 4.0, 1e-6);
  frontier->is_frontier = true;
  PlannerNodeTestPeer::receiveTransform(*fleet.a, "robot_0/odom",
                                        "robot_1/odom", 5.0, 0.0);
  PlannerNodeTestPeer::setTour(*fleet.a, true, 1.0);
  for (int broadcast = 0; broadcast < 2; ++broadcast) {
    PlannerNodeTestPeer::receiveGraph(*fleet.a, graph);
    const auto bid = PlannerNodeTestPeer::ownTourBidMsg(*fleet.a);
    ASSERT_EQ(bid.clusters.size(), 1u);
    EXPECT_EQ(bid.clusters.front().owner_robot_id, 2);
    EXPECT_NEAR(bid.clusters.front().position.x, 9.0, 1e-6);
    EXPECT_GT(bid.clusters.front().gain, 1.0);
    EXPECT_NE(PlannerNodeTestPeer::refreshTour(*fleet.a), mgg::kNoCluster);
  }
}

TEST_F(PlannerNodeTest, AwardsRequireARecentInRangeBidFromTheirAuctioneer) {
  TwoPlanners fleet("award_range");
  PlannerNodeTestPeer::receiveTransform(*fleet.b, "robot_1/odom",
                                        "robot_0/odom", -5.0, 0.0);
  auto bid = PlannerNodeTestPeer::ownTourBidMsg(*fleet.a);
  bid.pose.position.x = 100.0;
  PlannerNodeTestPeer::receiveTourBid(*fleet.b, bid);
  mgg_msgs::msg::TourAward award;
  award.header.frame_id = "robot_0/odom";
  award.header.stamp = fleet.a->now();
  award.auctioneer_id = 1;
  award.auction_id = 1;
  for (bool call : {true, false}) {
    award.call = call;
    PlannerNodeTestPeer::receiveTourAward(*fleet.b, award);
    EXPECT_EQ(PlannerNodeTestPeer::fleetGroup(*fleet.b), std::vector<int>{2});
    EXPECT_EQ(PlannerNodeTestPeer::ownTourBidMsg(*fleet.b).auctioneer_id, 2);
    EXPECT_FALSE(PlannerNodeTestPeer::fleetHasAward(*fleet.b));
  }
  bid.pose.position.x = 0.0;
  PlannerNodeTestPeer::receiveTourBid(*fleet.b, bid);
  PlannerNodeTestPeer::receiveTourAward(*fleet.b, award);
  EXPECT_EQ(PlannerNodeTestPeer::fleetGroup(*fleet.b), (std::vector<int>{1, 2}));
  EXPECT_EQ(PlannerNodeTestPeer::ownTourBidMsg(*fleet.b).auctioneer_id, 1);
  EXPECT_TRUE(PlannerNodeTestPeer::fleetHasAward(*fleet.b));
}

TEST_F(PlannerNodeTest, ImportedFrontierScoringExcludesSnapshotPublication) {
  auto node = makeNode("frontier_map_lease");
  PlannerNodeTestPeer::observeFloor(*node, -1.0, 4.0, -1.0, 1.0);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  const int id = PlannerNodeTestPeer::addGlobalChainToFrontier(*node, {{3.0, 0.0}});
  PlannerNodeTestPeer::setFrontierOwner(*node, id, 2);
  auto map = std::make_unique<PublicationProbeMap>();
  auto* probe = map.get();
  PlannerNodeTestPeer::useMolaMap(*node, std::move(map));
  PlannerNodeTestPeer::frontierClusters(*node);
  ASSERT_TRUE(probe->writer.valid());
  EXPECT_TRUE(probe->publication_blocked);
  EXPECT_EQ(probe->writer.wait_for(std::chrono::seconds(1)),
            std::future_status::ready);
  probe->writer.get();
}

TEST_F(PlannerNodeTest, PeerFrontierRescoringDoesNotLapseATourSetAside) {
  TwoPlanners fleet("rescore_set_aside");
  PlannerNodeTestPeer::gainFromUnknownVoxelsOnly(*fleet.a);
  PlannerNodeTestPeer::observeFloor(*fleet.a, -3.55, 10.55, -3.55, 3.55);
  PlannerNodeTestPeer::setTour(*fleet.a, true, 0.0);
  PlannerNodeTestPeer::solveTourOnEveryChange(*fleet.a);
  PlannerNodeTestPeer::addGlobalChainToFrontier(*fleet.a, {{-2.0, 0.0}});
  auto graph = PlannerNodeTestPeer::ownGraph(*fleet.b);
  auto frontier = std::max_element(graph.vertices.begin(), graph.vertices.end(),
      [](const auto& a, const auto& b) { return a.pose.position.x < b.pose.position.x; });
  ASSERT_NE(frontier, graph.vertices.end());
  frontier->is_frontier = true;
  PlannerNodeTestPeer::receiveTransform(*fleet.a, "robot_0/odom", "robot_1/odom", 0.0, 0.0);
  PlannerNodeTestPeer::receiveGraph(*fleet.a, graph);
  const auto target = PlannerNodeTestPeer::refreshTour(*fleet.a);
  ASSERT_NE(target, mgg::kNoCluster);
  ASSERT_LT(PlannerNodeTestPeer::tourTargetPosition(*fleet.a).x(), 0.0);
  PlannerNodeTestPeer::setTourAside(*fleet.a, target);
  const auto revision = PlannerNodeTestPeer::graphRevision(*fleet.a);
  for (int broadcast = 0; broadcast < 3; ++broadcast) {
    PlannerNodeTestPeer::receiveGraph(*fleet.a, graph);
    const auto clusters = PlannerNodeTestPeer::frontierClusters(*fleet.a);
    ASSERT_EQ(clusters.size(), 1u);  // peer frontier was explored here
    EXPECT_EQ(PlannerNodeTestPeer::refreshTour(*fleet.a), mgg::kNoCluster);
    EXPECT_EQ(PlannerNodeTestPeer::graphRevision(*fleet.a), revision);
  }
  // Unlike re-marking a frontier, a genuinely new edge changes topology.
  const auto ends = std::minmax_element(graph.vertices.begin(), graph.vertices.end(),
      [](const auto& a, const auto& b) { return a.pose.position.x < b.pose.position.x; });
  mgg_msgs::msg::Edge edge;
  edge.source_id = ends.first->id;
  edge.target_id = ends.second->id;
  edge.weight = 4.0;
  graph.edges.push_back(edge);
  const int edges = PlannerNodeTestPeer::globalEdges(*fleet.a);
  PlannerNodeTestPeer::receiveGraph(*fleet.a, graph);
  EXPECT_EQ(PlannerNodeTestPeer::globalEdges(*fleet.a), edges + 1);
  EXPECT_GT(PlannerNodeTestPeer::graphRevision(*fleet.a), revision);
  EXPECT_EQ(PlannerNodeTestPeer::refreshTour(*fleet.a), target);
}

TEST_F(PlannerNodeTest, ImportedFrontierScoringContinuesAfterItsTimeBudget) {
  auto node = makeNode("score_budget");
  PlannerNodeTestPeer::observeFloor(*node, -1.0, 4.0, -1.0, 1.0);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::setTour(*node, true, 0.0);  // zero must not admit unscored peers
  for (double x : {3.0, 8.0, 13.0}) {
    int id = PlannerNodeTestPeer::addGlobalChainToFrontier(*node, {{x, 0.0}});
    PlannerNodeTestPeer::setFrontierOwner(*node, id, 2);
  }
  auto map = std::make_unique<SlowScanMap>();
  auto* slow = map.get();
  PlannerNodeTestPeer::useCloudMap(*node, std::move(map));
  auto clusters = PlannerNodeTestPeer::frontierClusters(*node);
  ASSERT_EQ(slow->scanned_x.size(), 1u);  // one scan may exceed the budget
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_GT(clusters.front().gain, 0.0);
  // A fresh zero-gain report for the first vertex cannot starve later ones.
  PlannerNodeTestPeer::resetFrontierGain(*node, clusters.front().representative_vertex_id);
  PlannerNodeTestPeer::frontierClusters(*node);
  PlannerNodeTestPeer::frontierClusters(*node);
  ASSERT_EQ(slow->scanned_x.size(), 3u);
  EXPECT_NE(slow->scanned_x[0], slow->scanned_x[1]);
  EXPECT_NE(slow->scanned_x[0], slow->scanned_x[2]);
  EXPECT_NE(slow->scanned_x[1], slow->scanned_x[2]);
  clusters = PlannerNodeTestPeer::frontierClusters(*node);
  EXPECT_EQ(clusters.size(), 3u);
  EXPECT_EQ(slow->scanned_x.size(), 4u);
}

TEST_F(PlannerNodeTest, AMissedCallAwardDoesNotCompleteAnUnconsideredRobot) {
  TwoPlanners fleet("missed_call");
  PlannerNodeTestPeer::receiveTransform(*fleet.b, "robot_1/odom", "robot_0/odom", -5.0, 0.0);
  mgg::FleetCoordinator leader(1, mgg::FleetParams{}, 0.2);
  const double t = fleet.b->now().seconds();
  leader.onBid(fromTourBidMsg(PlannerNodeTestPeer::ownTourBidMsg(*fleet.b),
                             Eigen::Isometry3d::Identity()), t);
  auto out = leader.tick(t, nullptr, nullptr, nullptr);
  ASSERT_TRUE(out.bid && out.award && out.award->call);
  PlannerNodeTestPeer::receiveTourBid(*fleet.b, toTourBidMsg(*out.bid, "robot_0/odom"));
  // The call is lost. This idle robot requests after collection started.
  bool complete = false;
  std::string note;
  ASSERT_TRUE(PlannerNodeTestPeer::settleIdle(*fleet.b, note, complete));
  ASSERT_FALSE(complete);
  auto reply = PlannerNodeTestPeer::fleetStep(*fleet.b, t + 0.1);
  ASSERT_TRUE(reply.bid);
  ASSERT_EQ(reply.bid->auction_id, 0u);
  ASSERT_TRUE(reply.bid->request_auction);
  leader.onBid(*reply.bid, t + 0.1);
  out = leader.tick(t + 1.1, nullptr, nullptr, nullptr);
  ASSERT_TRUE(out.award && !out.award->call);
  ASSERT_EQ(out.award->bundleOf(2), nullptr);
  PlannerNodeTestPeer::receiveTourAward(*fleet.b, toTourAwardMsg(*out.award, "robot_0/odom"));
  ASSERT_TRUE(PlannerNodeTestPeer::settleIdle(*fleet.b, note, complete));
  EXPECT_FALSE(complete) << "an award that omitted this robot cannot finish it";
  // Its periodic bid keeps requesting; the next round collects it.
  reply = PlannerNodeTestPeer::fleetStep(*fleet.b, t + 3.2);
  ASSERT_TRUE(reply.bid);
  EXPECT_TRUE(reply.bid->request_auction);
  leader.onBid(*reply.bid, t + 3.2);
  out = leader.tick(t + 3.2, nullptr, nullptr, nullptr);
  ASSERT_TRUE(out.award && out.award->call);
  PlannerNodeTestPeer::receiveTourAward(*fleet.b, toTourAwardMsg(*out.award, "robot_0/odom"));
  reply = PlannerNodeTestPeer::fleetStep(*fleet.b, t + 3.3);
  ASSERT_TRUE(reply.bid);
  leader.onBid(*reply.bid, t + 3.3);
  out = leader.tick(t + 4.3, nullptr, nullptr, nullptr);
  ASSERT_TRUE(out.award && !out.award->call);
  ASSERT_NE(out.award->bundleOf(1), nullptr);
  ASSERT_NE(out.award->bundleOf(2), nullptr);
  EXPECT_TRUE(out.award->bundleOf(2)->clusters.empty());
  PlannerNodeTestPeer::receiveTourAward(*fleet.b, toTourAwardMsg(*out.award, "robot_0/odom"));
  complete = false;
  ASSERT_TRUE(PlannerNodeTestPeer::settleIdle(*fleet.b, note, complete));
  EXPECT_TRUE(complete);
}

TEST_F(PlannerNodeTest, AnOutOfRangeBidDoesNotJoinTheFleet) {
  TwoPlanners fleet("bid_range");
  PlannerNodeTestPeer::receiveTransform(*fleet.a, "robot_0/odom", "robot_1/odom", 100.0, 0.0);
  PlannerNodeTestPeer::receiveTourBid(*fleet.a, PlannerNodeTestPeer::ownTourBidMsg(*fleet.b));
  EXPECT_EQ(PlannerNodeTestPeer::fleetGroup(*fleet.a), std::vector<int>{1});
}

TEST_F(PlannerNodeTest, AnAwardWithoutATransformIsNotApplied) {
  TwoPlanners fleet("award_transform");
  PlannerNodeTestPeer::receiveTransform(*fleet.b, "robot_1/odom", "robot_0/odom", -5.0, 0.0);
  PlannerNodeTestPeer::receiveTourBid(*fleet.b, PlannerNodeTestPeer::ownTourBidMsg(*fleet.a));
  mgg_msgs::msg::TourAward award;
  award.auctioneer_id = 1;
  award.auction_id = 1;
  award.header.stamp = fleet.a->now();
  award.header.frame_id = "unconnected/odom";
  PlannerNodeTestPeer::receiveTourAward(*fleet.b, award);
  EXPECT_FALSE(PlannerNodeTestPeer::fleetHasAward(*fleet.b));
  award.header.frame_id = "robot_0/odom";
  PlannerNodeTestPeer::receiveTourAward(*fleet.b, award);
  EXPECT_TRUE(PlannerNodeTestPeer::fleetHasAward(*fleet.b));
}

TEST_F(PlannerNodeTest, GreedyFallbackExcludesHeldAndPeerExploredClusters) {
  for (bool explored : {false, true}) {
    int frontier;
    auto node = exploredDeadEnd(explored ? "greedy_explored" : "greedy_held", frontier);
    PlannerNodeTestPeer::setTour(*node, false, 0.0);
    ASSERT_TRUE(PlannerNodeTestPeer::greedyRoute(*node));
    const auto clusters = PlannerNodeTestPeer::frontierClusters(*node);
    ASSERT_EQ(clusters.size(), 1u);
    const auto& c = clusters.front();
    mgg::FleetCluster held{c.id, c.owner_robot_id, c.position, c.gain};
    if (explored) {
      mgg::TourAwardData award;
      award.auctioneer_id = 0;
      award.stamp_s = node->now().seconds();
      award.explored = {held};
      PlannerNodeTestPeer::applyAward(*node, award);
    } else {
      PlannerNodeTestPeer::claim(*node, 2, held, node->now().seconds());
    }
    EXPECT_FALSE(PlannerNodeTestPeer::greedyRoute(*node));
  }
}

TEST_F(PlannerNodeTest, ASoloIdleRobotTakesOverTheOldestSilentClaimFirst) {
  auto node = makeNode("solo_takeover");
  const double now = node->now().seconds();
  PlannerNodeTestPeer::claim(*node, 2, {21, 2, {3.0, 0.0, 0.0}, 1000.0}, now - 30.0);
  PlannerNodeTestPeer::claim(*node, 3, {31, 3, {8.0, 0.0, 0.0}, 1000.0}, now - 20.0);
  ASSERT_EQ(PlannerNodeTestPeer::fleetGroup(*node), std::vector<int>{1});
  bool complete = false;
  std::string note;
  EXPECT_TRUE(PlannerNodeTestPeer::settleIdle(*node, note, complete));
  EXPECT_FALSE(complete);
  const auto excluded = PlannerNodeTestPeer::fleetExclusions(*node);
  ASSERT_EQ(excluded.size(), 1u);
  EXPECT_DOUBLE_EQ(excluded.front().x(), 8.0);
  EXPECT_TRUE(PlannerNodeTestPeer::settleIdle(*node, note, complete));
  EXPECT_TRUE(PlannerNodeTestPeer::fleetExclusions(*node).empty());
  EXPECT_FALSE(PlannerNodeTestPeer::settleIdle(*node, note, complete));
}

TEST_F(PlannerNodeTest, AnEmptyFleetAwardIsNotCompleteWhileLocalGainRemains) {
  auto node = makeNode("fleet_local_gain");
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::hearPeer(*node, 2);
  bool complete = false;
  std::string note;
  ASSERT_TRUE(PlannerNodeTestPeer::settleIdle(*node, note, complete));
  const double now = node->now().seconds();
  PlannerNodeTestPeer::fleetTick(*node, now);
  PlannerNodeTestPeer::fleetTick(*node, now + 1.1);
  ASSERT_TRUE(PlannerNodeTestPeer::fleetHasAward(*node));
  PlannerNodeTestPeer::localGainRemains(*node, true);
  ASSERT_TRUE(PlannerNodeTestPeer::settleIdle(*node, note, complete));
  EXPECT_FALSE(complete);
  PlannerNodeTestPeer::localGainRemains(*node, false);
  ASSERT_TRUE(PlannerNodeTestPeer::settleIdle(*node, note, complete));
  EXPECT_TRUE(complete);
}

TEST_F(PlannerNodeTest, ReleaseClaimsRejectsUnknownAndOwnRobotIds) {
  auto node = makeNode("invalid_release");
  const double now = node->now().seconds();
  PlannerNodeTestPeer::claim(*node, 2, {21, 2, {3.0, 0.0, 0.0}, 1000.0}, now - 30.0);
  EXPECT_FALSE(PlannerNodeTestPeer::releaseClaims(*node, 99)->success);
  EXPECT_FALSE(PlannerNodeTestPeer::releaseClaims(*node, 1)->success);
  EXPECT_EQ(PlannerNodeTestPeer::fleetExclusions(*node).size(), 1u);
}

TEST_F(PlannerNodeTest, ASoloReleaseReportsThatItAppliedLocally) {
  auto node = makeNode("local_release");
  const double now = node->now().seconds();
  PlannerNodeTestPeer::claim(*node, 2, {21, 2, {3.0, 0.0, 0.0}, 1000.0}, now - 30.0);
  ASSERT_EQ(PlannerNodeTestPeer::fleetGroup(*node), std::vector<int>{1});
  auto response = PlannerNodeTestPeer::releaseClaims(*node, 2);
  EXPECT_TRUE(response->success);
  EXPECT_TRUE(PlannerNodeTestPeer::fleetExclusions(*node).empty());
  EXPECT_NE(response->message.find("locally"), std::string::npos);
  EXPECT_NE(response->message.find("nothing forwarded"), std::string::npos);
}

}  // namespace mgg_ros
