// The planner node's contract with its callers: an exploration cycle returns
// the whole lattice path, and an explicit objective returns the whole route
// over the global graph. Both are what PCI and a full-path controller
// execute; nothing here is windowed or truncated.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "mgg_map_octomap/mola_map.h"
#include "mgg_map_octomap/octomap_map.h"
#include "mgg_ros/planner_node.h"
#include "mgg_msgs/srv/planner_set_exploration_region.hpp"
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

/// A MOLA planner product, as the mapping worker publishes it, of a level
/// floor: an occupied 0.2 m voxel layer at z index -1 over [x0, x1) x [y0,
/// y1) and free voxels above it, and 0.6 m round it, up to 0.8 m. The same layout as
/// test_mola_map.cpp's Publication, reduced to one revision.
class MolaFloorProduct {
 public:
  MolaFloorProduct(double x0, double x1, double y0, double y1) {
    static int sequence = 0;
    root_ = std::filesystem::temp_directory_path() /
            ("mgg-planner-mola-" + std::to_string(::getpid()) + "-" +
             std::to_string(sequence++));
    std::filesystem::create_directories(root_ / "mola" / "components");
    using json = nlohmann::json;
    struct Voxel { std::int64_t x, y, z; };
    std::vector<Voxel> occupied, free;
    const auto index = [](double v) {
      return static_cast<std::int64_t>(std::floor(v / 0.2 + 1e-9));
    };
    // Free space reaches 0.6 m beyond the floor on every side, so a body
    // at the floor's edge is not in unknown space.
    for (std::int64_t x = index(x0) - 3; x < index(x1) + 3; ++x) {
      for (std::int64_t y = index(y0) - 3; y < index(y1) + 3; ++y) {
        const bool floor = x >= index(x0) && x < index(x1) &&
                           y >= index(y0) && y < index(y1);
        if (floor) occupied.push_back({x, y, -1});
        for (std::int64_t z = 0; z <= 3; ++z) free.push_back({x, y, z});
      }
    }
    const std::string geometry(64, 'a');
    const std::string snapshot_id(64, '1');
    const std::uint64_t stamp = 1000;
    const std::size_t points = occupied.size();
    const json chunk{{"sha256", std::string(64, 'c')},
                     {"size_bytes", 16 + 12 * points},
                     {"point_count", points},
                     {"encoding", "application/vnd.swarmdeck.xyz-f32.v1"}};
    json submap{{"observed_at_ns", stamp}, {"chunks", json::array({chunk})}};
    submap["sensor_origins"] = json::array({json::array({0.0, 0.0, 0.0})});
    submap["ray_evidence"] = {{"return_semantics", "first_return"},
                              {"deskew", "not_required"},
                              {"origin_association", "single_capture"}};
    json manifest{{"map_id", "onboard"},
                  {"layer_id", "persistent_geometry"},
                  {"frame_id", "component_test"},
                  {"graph_revision", {{"component_id", "component:test"},
                                      {"epoch", 1}, {"revision", 0}}},
                  {"geometry_revision", geometry},
                  {"submaps", json::array({submap})},
                  {"chunks", json::array({chunk})},
                  {"tombstones", json::array()}};
    json canonical = manifest;
    canonical["schema"] = "swarmdeck.autonomy.v1";
    const json source{{"schema", "swarmdeck.autonomy.v1"},
                      {"snapshot_id", snapshot_id},
                      {"generated_at_ns", stamp},
                      {"manifests", json::array({manifest})}};
    const std::string source_bytes = source.dump();
    const std::string source_digest = sha256(source_bytes);
    const json metadata{
        {"schema", "swarmdeck.mola_planner_grid.v2"},
        {"graph_version", {{"component_id", "component:test"}, {"epoch", 1},
                           {"revision", 0}, {"digest", std::string(64, 'd')}}},
        {"identity", {{"geometry_revision", geometry},
                      {"native_geometry_digest", std::string(64, 'e')},
                      {"canonical_manifest_digest", sha256(canonical.dump())},
                      {"source_snapshot_id", snapshot_id},
                      {"source_sha256", source_digest},
                      {"reference_frame", "component_test"}}},
        {"source_stamp_ns", stamp},
        {"resolution_m", 0.2},
        {"ray_angular_resolution_rad", 0.08726646259971647},
        {"ray_step_fraction", 0.75},
        {"point_count", points},
        {"source_point_count", points},
        {"occupied_count", occupied.size()},
        {"free_count", free.size()},
        {"surface_count", occupied.size()},
        {"retired_count", 0},
        {"ray_steps", free.size()},
        {"qualified_ray_keyframes", 1}};
    const std::string metadata_bytes = metadata.dump();
    std::string grid("SDMGRID1", 8);
    append(grid, static_cast<std::uint32_t>(metadata_bytes.size()), 4);
    grid += metadata_bytes;
    for (const Voxel& v : occupied) {
      append(grid, v.x, 8); append(grid, v.y, 8); append(grid, v.z, 8);
    }
    for (const Voxel& v : free) {
      append(grid, v.x, 8); append(grid, v.y, 8); append(grid, v.z, 8);
    }
    for (const Voxel& v : occupied) {
      append(grid, v.x, 8); append(grid, v.y, 8);
      const double top = (v.z + 0.5) * 0.2;
      std::uint64_t raw = 0;
      std::memcpy(&raw, &top, sizeof(raw));
      append(grid, raw, 8);
    }
    const json index_json{
        {"version", 1},
        {"source_snapshot_id", snapshot_id},
        {"source_sha256", source_digest},
        {"generated_at_ns", stamp},
        {"artifacts",
         json::array({{{"component_id", "component:test"}, {"epoch", 1},
                       {"revision", 0}, {"geometry_revision", geometry},
                       {"manifest_sha256", std::string(64, '9')},
                       {"path", "components/native.mola"}, {"size_bytes", 1},
                       {"sha256", std::string(64, 'f')},
                       {"planner", {{"path", "components/native.sdpg"},
                                    {"size_bytes", grid.size()},
                                    {"sha256", sha256(grid)},
                                    {"source_sha256", source_digest},
                                    {"source_snapshot_id", snapshot_id}}}}})}};
    write(root_ / "mola" / "components" / "native.sdpg", grid);
    write(root_ / "mola" / "source.json", source_bytes);
    write(root_ / "mola" / "index.json", index_json.dump());
    request_ = {"component:test", 1, 0, geometry, stamp,
                Eigen::Isometry3d::Identity()};
  }
  ~MolaFloorProduct() { std::filesystem::remove_all(root_); }

  /// A MolaMap serving the product, once it has loaded it.
  std::unique_ptr<mgg::MolaMap> serve() const {
    mgg::MolaMapConfig config;
    config.peer_root = root_.string();
    config.snapshot_ttl_sec = 60.0;
    auto map = std::make_unique<mgg::MolaMap>(config);
    map->requestSnapshot(request_);
    for (int i = 0; i < 400 && !map->getStatus(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_TRUE(map->getStatus()) << map->lastError();
    return map;
  }

 private:
  static std::string sha256(const std::string& bytes) {
    std::array<unsigned char, EVP_MAX_MD_SIZE> output{};
    unsigned int size = 0;
    EVP_Digest(bytes.data(), bytes.size(), output.data(), &size, EVP_sha256(),
               nullptr);
    std::ostringstream text;
    text << std::hex << std::setfill('0');
    for (unsigned int i = 0; i < size; ++i) {
      text << std::setw(2) << static_cast<unsigned int>(output[i]);
    }
    return text.str();
  }
  template <typename T>
  static void append(std::string& bytes, T value, int size) {
    std::uint64_t raw = 0;
    std::memcpy(&raw, &value, sizeof(T));
    for (int i = 0; i < size; ++i) {
      bytes.push_back(static_cast<char>((raw >> (8 * i)) & 0xff));
    }
  }
  static void write(const std::filesystem::path& path,
                    const std::string& bytes) {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  }

  std::filesystem::path root_;
  mgg::MolaSnapshotRequest request_;
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
  /// runGlobalPlanner outside a plan request, optionally resuming a target.
  static bool runGlobalPlanner(PlannerNode& node, std::string& reason,
                               int target_id = -1) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.runGlobalPlanner(target_id, reason);
  }
  static bool isGlobalFrontier(PlannerNode& node, int id) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.global_graph_->getVertex(id)->type == mgg::VertexType::kFrontier;
  }
  static int buildLocalUnknownGain(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.buildLocalGraph();
    int unknown = 0;
    for (const auto& [id, vertex] : node.local_graph_->vertices_map_) {
      if (vertex) unknown += vertex->vol_gain.num_unknown_voxels;
    }
    return unknown;
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
  /// A plan request pinning `mode` for its cycle.
  static void planWithBoundMode(
      PlannerNode& node, mgg::BoundModeType mode,
      std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response) {
    auto request = std::make_shared<mgg_msgs::srv::PlannerSrv::Request>();
    request->bound_mode = static_cast<std::uint8_t>(mode);
    node.onPlanRequest(request, response);
  }
  /// The robot's configured bound mode and its size extension.
  static void setBound(PlannerNode& node, mgg::BoundModeType mode,
                       double extension) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.robot_params_.bound_mode = mode;
    node.robot_params_.size_extension = Eigen::Vector3d(extension, extension, 0.0);
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
  static std::shared_ptr<mgg_msgs::srv::PlannerSetExplorationRegion::Response>
  setExplorationRegion(PlannerNode& node, bool active,
                       const Eigen::Vector3d& min, const Eigen::Vector3d& max) {
    auto request =
        std::make_shared<mgg_msgs::srv::PlannerSetExplorationRegion::Request>();
    request->active = active;
    request->min.x = min.x();
    request->min.y = min.y();
    request->min.z = min.z();
    request->max.x = max.x();
    request->max.y = max.y();
    request->max.z = max.z();
    auto response =
        std::make_shared<mgg_msgs::srv::PlannerSetExplorationRegion::Response>();
    node.onExplorationRegionRequest(request, response);
    return response;
  }
  static std::vector<mgg::FrontierCluster> insideExplorationRegion(
      PlannerNode& node, std::vector<mgg::FrontierCluster> clusters) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.insideExplorationRegion(std::move(clusters));
  }
  static std::vector<mgg::FrontierCluster> tourCandidates(
      PlannerNode& node, std::vector<mgg::FrontierCluster> clusters) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.tourCandidates(std::move(clusters));
  }
  // Cache an empty tour: candidate IDs and graph revision stay unchanged,
  // so only an assignment/region change can require another solve.
  static void solveEmptyTour(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    const auto clusters = node.tourCandidates({});
    node.tour_planner_->solve(clusters, mgg::TourCostMatrix{},
                              node.graph_revision_,
                              node.tour_assignment_version_, 0.0);
  }
  static bool emptyTourNeedsSolve(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    const auto clusters = node.tourCandidates({});
    return node.tour_planner_->needsSolve(clusters, node.graph_revision_,
                                         node.tour_assignment_version_, 100.0);
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
  static void setFlightReach(PlannerNode& node, double reach_m) {
    auto msg = std::make_shared<std_msgs::msg::Float64>();
    msg->data = reach_m;
    node.onFlightReach(msg);
  }
  static double flightReach(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.flight_reach_m_;
  }
  static void receiveTourBid(PlannerNode& node,
                             const mgg_msgs::msg::TourBid& msg) {
    node.onTourBid(std::make_shared<mgg_msgs::msg::TourBid>(msg));
  }
  static void receiveTourAward(PlannerNode& node,
                               const mgg_msgs::msg::TourAward& msg) {
    node.onTourAward(std::make_shared<mgg_msgs::msg::TourAward>(msg));
  }
  static std::shared_ptr<std_srvs::srv::SetBool::Response> leaveFleet(
      PlannerNode& node, bool leave) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = leave;
    auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
    node.onLeaveFleet(request, response);
    return response;
  }
  static bool fleetLeaving(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.fleet_ && node.fleet_->leaving();
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
    bid.speed_mps = 1.0;
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
    bid.speed_mps = 1.0;
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
  /// Whether `trigger` has tried a rebuild, whatever came of it.
  static bool rebuildAttempted(PlannerNode& node,
                               PlannerNode::RoadmapRebuildTrigger trigger) {
    return node.roadmap_rebuild_attempted_[static_cast<int>(trigger)];
  }
  /// A global vertex of `robot_id` at (x, y, z), joined to `neighbours`;
  /// its id. Another robot's vertex enters as a merged one.
  static int addGlobalVertex(PlannerNode& node, int robot_id, double x,
                             double y, double z,
                             const std::vector<int>& neighbours) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    auto* vertex = new mgg::Vertex(node.global_graph_->generateVertexID(),
                                   mgg::StateVec(x, y, z, 0.0));
    vertex->robot_id = robot_id;
    if (robot_id == static_cast<int>(node.planning_params_.robot_id)) {
      node.global_graph_->addVertex(vertex);
    } else {
      node.global_graph_->addNeighbourVertex(vertex, vertex->id);
    }
    for (const int id : neighbours) {
      mgg::Vertex* other = node.findGlobalVertex(id);
      node.global_graph_->addEdge(
          vertex, other, (vertex->state - other->state).head<3>().norm());
    }
    ++node.graph_revision_;
    return vertex->id;
  }
  static int roadmapRebuildsRefused(PlannerNode& node) {
    return node.roadmap_rebuilds_refused_;
  }
  /// The callback group the flight_state subscription is in, found among
  /// the node's groups, or null.
  static rclcpp::CallbackGroup::SharedPtr flightStateGroup(PlannerNode& node) {
    rclcpp::CallbackGroup::SharedPtr found;
    node.get_node_base_interface()->for_each_callback_group(
        [&node, &found](const rclcpp::CallbackGroup::SharedPtr& group) {
          if (group->find_subscription_ptrs_if(
                  [&node](const rclcpp::SubscriptionBase::SharedPtr& sub) {
                    return sub == node.flight_state_sub_;
                  })) {
            found = group;
          }
        });
    return found;
  }
  static rclcpp::CallbackGroup::SharedPtr reentrantGroup(PlannerNode& node) {
    return node.callback_group_;
  }
  static std::optional<std::string> latestFlightState(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.latest_flight_state_;
  }
  /// The flight state SwarmDeck's adapter publishes on flight_state.
  static void setFlightState(PlannerNode& node, const std::string& state) {
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = state;
    node.onFlightState(msg);
  }
  static void setAerialRobot(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.robot_params_.type = mgg::RobotType::kAerialRobot;
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
  /// A lattice of one chain through `states`, every vertex seen (no
  /// frontier, no gain), as a lattice computed inside an exploration region
  /// leaves the vertices outside it.
  static void setSeenLattice(PlannerNode& node,
                             const std::vector<mgg::StateVec>& states) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.local_graph_->reset();
    mgg::Vertex* previous = nullptr;
    for (const mgg::StateVec& state : states) {
      auto* v = new mgg::Vertex(node.local_graph_->generateVertexID(), state);
      node.local_graph_->addVertex(v);
      if (previous != nullptr) {
        node.local_graph_->addEdge(
            v, previous, (v->state - previous->state).head<3>().norm());
      }
      previous = v;
    }
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

TEST_F(PlannerNodeTest, ASlopeWithNoRoomToTurnOnTheDefaultLatticeGetsNoPathNotComplete) {
  // Review r1 (P1), the original scene, kept alongside the finer one below
  // (review r0 of mgg-run8): the default 0.5 m lattice to x = 3. Grown
  // outward from the robot, it holds too few vertices round some ends to
  // fit the ground's slope there, and such an end read as unmeasured, not
  // on the slope: it needed its turn space observed to be clear, and the
  // fallback for when no path ends clear sent it with no way back. Where
  // the lattice cannot measure the slope the map does (groundSlope), and an
  // end the map puts on the slope needs a way back too. No path, retried,
  // and not exploration complete, not even with the global planner
  // consulted: an automatic repositioning's end needs a way back as a
  // lattice path's does (review r1, R1-3).
  auto node = makeNode("slope_no_way_back_default");
  PlannerNodeTestPeer::setRobotFootprint(*node, 0.6, 0.2);
  PlannerNodeTestPeer::allowUnknownLatticeBody(*node);
  PlannerNodeTestPeer::observeSparseSlope(*node, -1.5, 4.0, -1.5, 1.5, 0.2);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  const mgg::StateVec root =
      PlannerNodeTestPeer::drivingState(*node, 0.0, 0.0, 0.0);
  ASSERT_FALSE(PlannerNodeTestPeer::roomToTurnObserved(*node, root));
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_TRUE(response->path.empty())
      << "path of " << response->path.size() << " poses to ("
      << response->path.back().position.x << ", "
      << response->path.back().position.y << ")";
  EXPECT_EQ(response->status, PlannerNode::kStatusNoPath);
  PlannerNodeTestPeer::consultGlobalPlannerAtOnce(*node);
  response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_TRUE(response->path.empty())
      << "path of " << response->path.size() << " poses to ("
      << response->path.back().position.x << ", "
      << response->path.back().position.y << ")";
  EXPECT_EQ(response->status, PlannerNode::kStatusNoPath);
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

TEST_F(PlannerNodeTest, FromInsideANoGoZoneOnlyAnOutwardDepartureIsRouted) {
  // Review r0, I-3: standing in a zone disabled it for the whole route. The
  // robot stands in a zone near its east edge, on a straight track. A goal
  // to the west, through the zone's centre, is refused; one to the east,
  // straight out, is routed; and with the robot outside, a goal inside the
  // zone is refused.
  auto node = makeNode("no_go_inside", "world",
                       {rclcpp::Parameter("PlanningParams.no_go_radius_m", 0.5)});
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 8.0, -1.5, 1.5);
  double stamp = 1.0;
  for (double x = 0.0; x <= 7.0 + 1e-9; x += 0.5) {
    PlannerNodeTestPeer::acceptOdometry(*node, x, 0.0, stamp);
    stamp += 1.0;
  }
  // Zone at (4, 0), reach 0.6; the robot at 4.4.
  PlannerNodeTestPeer::acceptOdometry(*node, 4.4, 0.0, stamp++);
  PlannerNodeTestPeer::receiveNoGoZones(*node, "world", {{4.0, 0.0}});
  const auto navigate = [&node](double x) {
    auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
    request->objective = mgg_msgs::srv::PlanObjective::Request::NAVIGATE;
    request->goal.position.x = x;
    request->goal.orientation.w = 1.0;
    auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
    PlannerNodeTestPeer::objective(*node, request, response);
    return response;
  };
  auto west = navigate(0.0);
  EXPECT_EQ(west->status, mgg_msgs::srv::PlanObjective::Response::UNREACHABLE)
      << "path of " << west->path.size() << " poses";
  auto east = navigate(7.0 + 1e-3);
  EXPECT_EQ(east->status, mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << east->reason;
  // Outside, at the far west: the zone's centre is no goal.
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, stamp++);
  auto inside = navigate(4.0);
  EXPECT_EQ(inside->status, mgg_msgs::srv::PlanObjective::Response::UNREACHABLE)
      << "path of " << inside->path.size() << " poses";
}

/// A robot in a 1.2 m corridor from x = -1 to 5 on the cloud backend, or
/// on the mola_snapshot backend serving the same floor, with no-go zones
/// of 1.0 m radius (1.1 m reach), wider than the lattice (y in [-1, 1]).
std::shared_ptr<PlannerNode> corridorNode(const std::string& name, bool mola,
                                          std::unique_ptr<MolaFloorProduct>& product) {
  auto node = makeNode(name, "world",
                       {rclcpp::Parameter("PlanningParams.no_go_radius_m", 1.0)});
  if (mola) {
    product = std::make_unique<MolaFloorProduct>(-1.0, 5.0, -0.6, 0.6);
    PlannerNodeTestPeer::useMolaMap(*node, product->serve());
    PlannerNodeTestPeer::serveMap(*node, "component:test", 1);
    // The synthetic product's ground does not pass the observed-ground rule
    // (min_observed_ground_fraction, tested on its own); the zones are what
    // this scene is about.
    PlannerNodeTestPeer::setMinObservedGround(*node, 0.0);
  } else {
    PlannerNodeTestPeer::observeFloor(*node, -1.0, 5.0, -0.6, 0.6);
  }
  return node;
}

double furthestX(const std::vector<geometry_msgs::msg::Pose>& path) {
  double x = -1e9;
  for (const auto& pose : path) x = std::max(x, pose.position.x);
  return x;
}

/// The least distance of a pose of `path` from `centre`, in the plane.
double nearestTo(const std::vector<geometry_msgs::msg::Pose>& path,
                 const Eigen::Vector2d& centre) {
  double d = 1e9;
  for (const auto& pose : path) {
    d = std::min(d, std::hypot(pose.position.x - centre.x(),
                               pose.position.y - centre.y()));
  }
  return d;
}

TEST_F(PlannerNodeTest, LocalExplorationKeepsOutOfANoGoZoneOnEitherBackend) {
  // Review r0, I-5: only the mola backend's map knew the zones; the cloud
  // backend's lattice drove straight through them. The zone at x = 2.5
  // closes the corridor: on either backend no pose of the path comes
  // within its reach or gets past it, and without it the path goes on
  // beyond.
  for (const bool mola : {false, true}) {
    SCOPED_TRACE(mola ? "mola_snapshot" : "cloud_octomap");
    std::unique_ptr<MolaFloorProduct> product;
    auto node = corridorNode(mola ? "no_go_explore_mola" : "no_go_explore_cloud",
                             mola, product);
    PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
    EXPECT_GT(furthestX(response->path), 2.0);

    PlannerNodeTestPeer::receiveNoGoZones(*node, "world", {{2.5, 0.0}});
    response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    EXPECT_LT(furthestX(response->path), 2.5)
        << "path of " << response->path.size() << " poses";
    EXPECT_GE(nearestTo(response->path, {2.5, 0.0}), 1.1 - 1e-6);
  }
}

TEST_F(PlannerNodeTest, NavigateInTheLatticeKeepsOutOfANoGoZoneOnEitherBackend) {
  // Review r0, I-5: a goal inside the lattice, beyond a zone closing the
  // corridor, is refused on either backend; cleared, it is routed.
  for (const bool mola : {false, true}) {
    SCOPED_TRACE(mola ? "mola_snapshot" : "cloud_octomap");
    std::unique_ptr<MolaFloorProduct> product;
    auto node = corridorNode(mola ? "no_go_navigate_mola" : "no_go_navigate_cloud",
                             mola, product);
    PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
    auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
    request->objective = mgg_msgs::srv::PlanObjective::Request::NAVIGATE;
    request->goal.position.x = 2.8;
    request->goal.orientation.w = 1.0;
    request->component_id = "component:test";
    request->map_epoch = 1;
    PlannerNodeTestPeer::receiveNoGoZones(*node, "world", {{1.5, 0.0}});
    auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
    PlannerNodeTestPeer::objective(*node, request, response);
    EXPECT_EQ(response->status,
              mgg_msgs::srv::PlanObjective::Response::UNREACHABLE)
        << "path to x " << furthestX(response->path);
    PlannerNodeTestPeer::receiveNoGoZones(*node, "world", {});
    response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
    PlannerNodeTestPeer::objective(*node, request, response);
    EXPECT_EQ(response->status,
              mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
        << response->reason;
  }
}

TEST_F(PlannerNodeTest, ExplorationFromInsideANoGoZoneOnlyDepartsOnEitherBackend) {
  // Review r0, I-5 with I-3: the robot stands in the zone, west of its
  // centre, facing east into it. It is sent a departure west that ends out
  // of the zone; nothing goes east past where it stands.
  for (const bool mola : {false, true}) {
    SCOPED_TRACE(mola ? "mola_snapshot" : "cloud_octomap");
    std::unique_ptr<MolaFloorProduct> product;
    auto node = corridorNode(mola ? "no_go_depart_mola" : "no_go_depart_cloud",
                             mola, product);
    PlannerNodeTestPeer::acceptOdometry(*node, 1.8, 0.0, 1.0);
    PlannerNodeTestPeer::receiveNoGoZones(*node, "world", {{2.3, 0.0}});
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    // Review r1, R1-1: a departure is sent, not an empty path.
    ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
    ASSERT_GE(response->path.size(), 2u);
    EXPECT_LE(furthestX(response->path), 1.8 + 1e-3);
    EXPECT_GE(std::hypot(response->path.back().position.x - 2.3,
                         response->path.back().position.y),
              1.1 - 1e-6);
  }
}

TEST_F(PlannerNodeTest, TheNoGoReachFollowsTheRequestsBoundMode) {
  // Review r1, R1-2: the zones' reach (radius plus half the planning box)
  // was taken before the request's bound mode was applied, and the final
  // check ran after the configured mode was restored, so the lattice and
  // the check used the configured footprint, not the requested one. The
  // robot is 0.2 m with a 0.6 m extension; a zone of 0.3 m radius stands
  // 0.6 m beside the corridor's centre line, at (2, 0.6).
  const Eigen::Vector2d zone(2.0, 0.6);
  {
    SCOPED_TRACE("configured exact, requested extended: the reach grows");
    auto node = makeNode("no_go_bound_grow", "world",
                         {rclcpp::Parameter("PlanningParams.no_go_radius_m", 0.3)});
    PlannerNodeTestPeer::observeFloor(*node, -1.0, 5.0, -0.6, 0.6);
    PlannerNodeTestPeer::setBound(*node, mgg::BoundModeType::kExactBound, 0.6);
    PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
    PlannerNodeTestPeer::receiveNoGoZones(*node, "world", {zone});
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::planWithBoundMode(
        *node, mgg::BoundModeType::kExtendedBound, response);
    // Reach 0.3 + 0.4: the centre line near x = 2 is closed.
    EXPECT_GE(nearestTo(response->path, zone), 0.7 - 1e-6)
        << "path to x " << furthestX(response->path);
  }
  {
    SCOPED_TRACE("configured extended, requested exact: the reach shrinks");
    auto node = makeNode("no_go_bound_shrink", "world",
                         {rclcpp::Parameter("PlanningParams.no_go_radius_m", 0.3)});
    // A corridor one lattice row wide: the way past is the centre line.
    PlannerNodeTestPeer::observeFloor(*node, -1.0, 5.0, -0.3, 0.3);
    PlannerNodeTestPeer::setBound(*node, mgg::BoundModeType::kExtendedBound, 0.6);
    PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
    PlannerNodeTestPeer::receiveNoGoZones(*node, "world", {zone});
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::planWithBoundMode(
        *node, mgg::BoundModeType::kExactBound, response);
    // Reach 0.3 + 0.1: the centre line stays open past the zone.
    ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
    EXPECT_GT(furthestX(response->path), 2.5);
    EXPECT_GE(nearestTo(response->path, zone), 0.4 - 1e-6);
  }
}

TEST_F(PlannerNodeTest, ARestoredZoneSafeRouteThatTurnsWithoutRoomIsNotSent) {
  // Review r2, R2-2, the reviewer's graph at twice its scale (roadmap links
  // reach 0.55 m here): a zone of 2 m reach (1.7 m radius, 0.3 m
  // half-length) at the origin; the robot at S = (1, 0) faces north, with a
  // post 0.29 m south-west of it: no room to turn there, room everywhere
  // else. The roadmap holds S-A-C-T, zone-safe but turning 90 degrees at S,
  // and S-U-B-T, which starts north but re-enters the zone at B. A wall
  // across S-C and A-T leaves the route no shortcut. The zone-safe route is
  // found, the turn rule replaces it with S-U-B-T, and that is put back for
  // re-entering the zone. The flag that makes a route starting with a turn
  // the robot has no room for into a straight departure was left unset,
  // and S-A-C-T was sent. It must be a straight departure or no path.
  auto node = makeNode("no_go_restored_turn", "world",
                       {rclcpp::Parameter("PlanningParams.no_go_radius_m", 1.7)});
  PlannerNodeTestPeer::setRobotFootprint(*node, 0.6, 0.2);
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 5.0, -1.5, 4.5);
  PlannerNodeTestPeer::observeWall(*node, 0.85, 0.85, -0.25);
  PlannerNodeTestPeer::observeWallAlongY(*node, 1.3, 2.7, 2.5);
  // Only the lattice's own cells, all inside the zone: no path may end
  // there, so the global planner is consulted.
  PlannerNodeTestPeer::setLatticeResolution(*node, 0.25);
  PlannerNodeTestPeer::setLattice(*node, {-0.25, -0.25}, {0.25, 0.25});
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->header.stamp.sec = 1;
  odometry->pose.pose.position.x = 1.0;
  odometry->pose.pose.position.z = 0.075;
  odometry->pose.pose.orientation.z = std::sin(M_PI / 4.0);
  odometry->pose.pose.orientation.w = std::cos(M_PI / 4.0);
  PlannerNodeTestPeer::acceptOdometry(*node, odometry);
  const int a = PlannerNodeTestPeer::addIsolatedGlobalVertex(*node, 4.0, 0.0);
  const int c = PlannerNodeTestPeer::addIsolatedGlobalVertex(*node, 4.0, 4.0);
  const int t = PlannerNodeTestPeer::addIsolatedGlobalVertex(*node, 0.0, 4.0);
  const int u = PlannerNodeTestPeer::addIsolatedGlobalVertex(*node, 1.0, 4.0);
  const int b = PlannerNodeTestPeer::addIsolatedGlobalVertex(*node, 1.0, 1.0);
  PlannerNodeTestPeer::addGlobalEdgeOnly(*node, 0, a);
  PlannerNodeTestPeer::addGlobalEdgeOnly(*node, a, c);
  PlannerNodeTestPeer::addGlobalEdgeOnly(*node, c, t);
  PlannerNodeTestPeer::addGlobalEdgeOnly(*node, 0, u);
  PlannerNodeTestPeer::addGlobalEdgeOnly(*node, u, b);
  PlannerNodeTestPeer::addGlobalEdgeOnly(*node, b, t);
  PlannerNodeTestPeer::markGlobalFrontier(*node, t);
  PlannerNodeTestPeer::receiveNoGoZones(*node, "world", {{0.0, 0.0}});
  PlannerNodeTestPeer::consultGlobalPlannerAtOnce(*node);
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  EXPECT_FALSE(PlannerNodeTestPeer::bestPathFromGlobalGraph(*node));
  // A straight departure, ahead or back along the robot's heading (turned
  // by at most kDepartureMaxTurnRad), if any path at all.
  for (const auto& pose : response->path) {
    const double dx = pose.position.x - 1.0, dy = pose.position.y;
    if (std::hypot(dx, dy) < 1e-6) continue;
    EXPECT_LE(std::abs(dx) / std::hypot(dx, dy),
              std::sin(mgg::kDepartureMaxTurnRad) + 1e-6)
        << "pose at (" << pose.position.x << ", " << pose.position.y << ")";
  }
  EXPECT_EQ(PlannerNodeTestPeer::boxedInDepartures(*node) +
                PlannerNodeTestPeer::boxedInWithoutDeparture(*node),
            1);
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

TEST_F(PlannerNodeTest, TheDroneLeavesAndRejoinsTheFleetOnRequest) {
  auto node = makeNode("leave_fleet");
  auto response = PlannerNodeTestPeer::leaveFleet(*node, true);
  ASSERT_TRUE(response->success) << response->message;
  EXPECT_TRUE(PlannerNodeTestPeer::fleetLeaving(*node));
  const auto leaving = PlannerNodeTestPeer::fleetStep(*node, 1.0);
  ASSERT_TRUE(leaving.bid);
  EXPECT_TRUE(leaving.bid->leaving);
  response = PlannerNodeTestPeer::leaveFleet(*node, false);
  ASSERT_TRUE(response->success) << response->message;
  EXPECT_FALSE(PlannerNodeTestPeer::fleetLeaving(*node));
  const auto rejoined = PlannerNodeTestPeer::fleetStep(*node, 1.1);
  ASSERT_TRUE(rejoined.bid);
  EXPECT_FALSE(rejoined.bid->leaving);
}

TEST_F(PlannerNodeTest, LeaveFleetIsRefusedWhenFleetAssignmentIsOff) {
  auto node = makeNode("leave_fleet_off", "world",
                        {rclcpp::Parameter("fleet.enabled", false)});
  for (const bool leave : {true, false}) {
    const auto response = PlannerNodeTestPeer::leaveFleet(*node, leave);
    EXPECT_FALSE(response->success);
    EXPECT_FALSE(response->message.empty());
    EXPECT_FALSE(PlannerNodeTestPeer::fleetLeaving(*node));
  }
}

TEST_F(PlannerNodeTest, LeaveFleetServiceAcceptsBothTransitions) {
  auto node = makeNode("leave_fleet_service");
  auto client_node = std::make_shared<rclcpp::Node>("leave_fleet_client");
  auto client = client_node->create_client<std_srvs::srv::SetBool>("leave_fleet");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(client_node);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(3)));
  for (const bool leave : {true, false}) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = leave;
    auto future = client->async_send_request(request);
    ASSERT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(3)),
              rclcpp::FutureReturnCode::SUCCESS);
    const auto response = future.get();
    ASSERT_TRUE(response->success) << response->message;
    EXPECT_EQ(PlannerNodeTestPeer::fleetLeaving(*node), leave);
  }
}

TEST_F(PlannerNodeTest, ALeavingBidReleasesClaimsOnlyWithinRadioRange) {
  for (const bool initially_in_range : {true, false}) {
    SCOPED_TRACE(initially_in_range);
    TwoPlanners fleet("fleet_leave_range");
    PlannerNodeTestPeer::receiveTransform(*fleet.a, "robot_0/odom",
                                          "robot_1/odom", 5.0, 0.0);
    PlannerNodeTestPeer::setCommunicationRange(*fleet.a, 10.0);
    mgg::TourBidData bid;
    bid.robot_id = 2;
    bid.seq = 1;
    bid.stamp_s = fleet.a->now().seconds();
    bid.speed_mps = 1.0;
    bid.pose = mgg::StateVec(4, 0, 0, 0);  // transformed x = 9, within range
    bid.clusters = {{77, 2, Eigen::Vector3d(5, 0, 0), 1000.0}};
    bid.costs_from_pose = {1.0};
    bid.costs_between = {0.0};
    bid.bundle = {77};
    PlannerNodeTestPeer::receiveTourBid(*fleet.a,
                                        toTourBidMsg(bid, "robot_1/odom"));
    ASSERT_EQ(PlannerNodeTestPeer::fleetExclusions(*fleet.a).size(), 1u);
    ASSERT_EQ(PlannerNodeTestPeer::fleetGroup(*fleet.a),
              (std::vector<int>{1, 2}));

    mgg::TourBidData leaving;
    leaving.robot_id = 2;
    leaving.seq = 2;
    leaving.stamp_s = bid.stamp_s;
    leaving.leaving = true;
    leaving.pose = mgg::StateVec(initially_in_range ? 4 : 6, 0, 0, 0);
    PlannerNodeTestPeer::receiveTourBid(*fleet.a,
        toTourBidMsg(leaving, "robot_1/odom"));
    if (!initially_in_range) {
      EXPECT_EQ(PlannerNodeTestPeer::fleetExclusions(*fleet.a).size(), 1u);
      EXPECT_EQ(PlannerNodeTestPeer::fleetGroup(*fleet.a),
                (std::vector<int>{1, 2}));
      // A periodic exit is heard after returning into range.
      leaving.seq = 3;
      leaving.pose[0] = 4;
      PlannerNodeTestPeer::receiveTourBid(*fleet.a,
          toTourBidMsg(leaving, "robot_1/odom"));
    }
    EXPECT_TRUE(PlannerNodeTestPeer::fleetExclusions(*fleet.a).empty());
    EXPECT_EQ(PlannerNodeTestPeer::fleetGroup(*fleet.a), std::vector<int>{1});
  }
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
  // The leader knows no cluster; its bids carry a speed, as every bid does.
  const mgg::OwnBidFn leader_bid = [] {
    mgg::TourBidData bid;
    bid.speed_mps = 1.0;
    return bid;
  };
  const double t = fleet.b->now().seconds();
  leader.onBid(fromTourBidMsg(PlannerNodeTestPeer::ownTourBidMsg(*fleet.b),
                             Eigen::Isometry3d::Identity()), t);
  auto out = leader.tick(t, leader_bid, nullptr, nullptr);
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
  out = leader.tick(t + 1.1, leader_bid, nullptr, nullptr);
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
  out = leader.tick(t + 3.2, leader_bid, nullptr, nullptr);
  ASSERT_TRUE(out.award && out.award->call);
  PlannerNodeTestPeer::receiveTourAward(*fleet.b, toTourAwardMsg(*out.award, "robot_0/odom"));
  reply = PlannerNodeTestPeer::fleetStep(*fleet.b, t + 3.3);
  ASSERT_TRUE(reply.bid);
  leader.onBid(*reply.bid, t + 3.3);
  out = leader.tick(t + 4.3, leader_bid, nullptr, nullptr);
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

TEST_F(PlannerNodeTest, AnExplorationRegionKeepsTheTourInsideIt) {
  for (bool fleet_enabled : {false, true}) {
    SCOPED_TRACE(fleet_enabled);
    auto node = makeNode("exploration_region", "world",
                         {rclcpp::Parameter("fleet.enabled", fleet_enabled)});
    mgg::FrontierCluster pit;
    pit.id = 11;
    pit.position = Eigen::Vector3d(2.0, 0.0, -6.0);
    mgg::FrontierCluster corridor;
    corridor.id = 12;
    corridor.position = Eigen::Vector3d(20.0, 0.0, 1.0);
    const std::vector<mgg::FrontierCluster> both{pit, corridor};

    EXPECT_EQ(PlannerNodeTestPeer::insideExplorationRegion(*node, both).size(), 2u);
    auto response = PlannerNodeTestPeer::setExplorationRegion(
        *node, true, Eigen::Vector3d(-1, -3, -10), Eigen::Vector3d(5, 3, 0));
    ASSERT_TRUE(response->success) << response->message;
    const auto inside = PlannerNodeTestPeer::insideExplorationRegion(*node, both);
    ASSERT_EQ(inside.size(), 1u);
    EXPECT_EQ(inside[0].id, 11u);
    const auto candidates = PlannerNodeTestPeer::tourCandidates(*node, both);
    ASSERT_EQ(candidates.size(), 1u);
    EXPECT_EQ(candidates[0].id, 11u);

    response = PlannerNodeTestPeer::setExplorationRegion(
        *node, false, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    ASSERT_TRUE(response->success);
    EXPECT_EQ(PlannerNodeTestPeer::insideExplorationRegion(*node, both).size(), 2u);
    EXPECT_EQ(PlannerNodeTestPeer::tourCandidates(*node, both).size(), 2u);
  }
}

TEST_F(PlannerNodeTest, AnEmptyOrNonFiniteRegionIsRefused) {
  auto node = makeNode("exploration_region_bad");
  EXPECT_FALSE(PlannerNodeTestPeer::setExplorationRegion(
                   *node, true, Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(0, 2, 2))
                   ->success);
  EXPECT_FALSE(PlannerNodeTestPeer::setExplorationRegion(
                   *node, true, Eigen::Vector3d(0, 0, 0),
                   Eigen::Vector3d(std::nan(""), 1, 1))
                   ->success);
  EXPECT_FALSE(PlannerNodeTestPeer::setExplorationRegion(
                   *node, true, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 1, 1))
                   ->success);
}

TEST_F(PlannerNodeTest, RegionChangesInvalidateToursWithAndWithoutFleet) {
  for (bool fleet_enabled : {false, true}) {
    SCOPED_TRACE(fleet_enabled);
    auto node = makeNode("region_tour_invalidation", "world",
                         {rclcpp::Parameter("fleet.enabled", fleet_enabled)});
    PlannerNodeTestPeer::solveEmptyTour(*node);
    ASSERT_FALSE(PlannerNodeTestPeer::emptyTourNeedsSolve(*node));
    ASSERT_TRUE(PlannerNodeTestPeer::setExplorationRegion(
                    *node, true, Eigen::Vector3d(-1, -3, -10),
                    Eigen::Vector3d(5, 3, 0))->success);
    EXPECT_TRUE(PlannerNodeTestPeer::emptyTourNeedsSolve(*node));
    PlannerNodeTestPeer::solveEmptyTour(*node);
    ASSERT_FALSE(PlannerNodeTestPeer::emptyTourNeedsSolve(*node));
    EXPECT_FALSE(PlannerNodeTestPeer::setExplorationRegion(
                     *node, true, Eigen::Vector3d::Zero(),
                     Eigen::Vector3d::Zero())->success);
    EXPECT_FALSE(PlannerNodeTestPeer::emptyTourNeedsSolve(*node));
    ASSERT_TRUE(PlannerNodeTestPeer::setExplorationRegion(
                    *node, false, Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero())->success);
    EXPECT_TRUE(PlannerNodeTestPeer::emptyTourNeedsSolve(*node));
  }
}

TEST_F(PlannerNodeTest, RegionPlansPreserveOutsideFrontiersAndTheirBroadcast) {
  auto node = makeNode("region_preserves_frontiers", "world",
                       {rclcpp::Parameter("tour.min_cluster_gain", 0.0)});
  PlannerNodeTestPeer::observeFloor(*node, -3.55, 4.55, -1.55, 1.55);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::gainFromUnknownVoxelsOnly(*node);
  const int outside = PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{-0.5, 0.0}, {-1.0, 0.0}, {-2.0, 0.0}}, M_PI);
  const auto plan = [&]() {
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
  };
  // The second plan re-checks persistent frontiers before replacing the
  // previous local graph, as each subsequent exploration cycle does.
  plan();
  plan();
  ASSERT_TRUE(PlannerNodeTestPeer::isGlobalFrontier(*node, outside));
  const auto broadcastFrontier = [&]() {
    const auto graph = PlannerNodeTestPeer::ownGraph(*node);
    for (const auto& vertex : graph.vertices) {
      if (vertex.id == outside) return vertex.is_frontier;
    }
    return false;
  };
  ASSERT_TRUE(broadcastFrontier());
  const auto hasOutsideCandidate = [&]() {
    const auto candidates = PlannerNodeTestPeer::tourCandidates(
        *node, PlannerNodeTestPeer::frontierClusters(*node));
    for (const auto& cluster : candidates) {
      for (int id : cluster.member_vertex_ids) {
        if (id == outside) return true;
      }
    }
    return false;
  };
  ASSERT_TRUE(hasOutsideCandidate());
  ASSERT_TRUE(PlannerNodeTestPeer::setExplorationRegion(
                  *node, true, {100, 100, 100}, {110, 110, 110})->success);
  plan();
  EXPECT_TRUE(PlannerNodeTestPeer::isGlobalFrontier(*node, outside));
  EXPECT_TRUE(broadcastFrontier()) << "must not broadcast an operator exclusion as explored";
  EXPECT_TRUE(PlannerNodeTestPeer::tourCandidates(
                  *node, PlannerNodeTestPeer::frontierClusters(*node)).empty());

  ASSERT_TRUE(PlannerNodeTestPeer::setExplorationRegion(
                  *node, false, Eigen::Vector3d::Zero(),
                  Eigen::Vector3d::Zero())->success);
  EXPECT_TRUE(hasOutsideCandidate());
}

TEST_F(PlannerNodeTest, RegionLimitsLocalGainAndARefusedRequestPreservesIt) {
  auto node = makeNode("region_local_gain");
  PlannerNodeTestPeer::observeFloor(*node, -1.55, 4.55, -1.55, 1.55);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  ASSERT_GT(PlannerNodeTestPeer::buildLocalUnknownGain(*node), 0);
  ASSERT_TRUE(PlannerNodeTestPeer::setExplorationRegion(
                  *node, true, {100, 100, 100}, {110, 110, 110})->success);
  EXPECT_EQ(PlannerNodeTestPeer::buildLocalUnknownGain(*node), 0);
  EXPECT_FALSE(PlannerNodeTestPeer::setExplorationRegion(
                   *node, true, {0, 0, 0}, {0, 1, 1})->success);
  EXPECT_EQ(PlannerNodeTestPeer::buildLocalUnknownGain(*node), 0);
  mgg::FrontierCluster inside, outside;
  inside.id = 1;
  inside.position = {105, 105, 105};
  outside.id = 2;
  outside.position = {2, 0, 0};
  const auto kept = PlannerNodeTestPeer::insideExplorationRegion(
      *node, {inside, outside});
  ASSERT_EQ(kept.size(), 1u);
  EXPECT_EQ(kept.front().id, inside.id);
  ASSERT_TRUE(PlannerNodeTestPeer::setExplorationRegion(
                  *node, false, Eigen::Vector3d::Zero(),
                  Eigen::Vector3d::Zero())->success);
  EXPECT_GT(PlannerNodeTestPeer::buildLocalUnknownGain(*node), 0);
}

TEST_F(PlannerNodeTest, RegionRejectsGreedyAndResumedOutsideFrontiersWithoutDemotion) {
  auto node = makeNode("region_greedy_filter");
  PlannerNodeTestPeer::observeFloor(*node, -3.55, 4.55, -1.55, 1.55);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  const int outside = PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{-0.5, 0.0}, {-1.0, 0.0}, {-2.0, 0.0}}, M_PI);
  std::string reason;
  ASSERT_TRUE(PlannerNodeTestPeer::runGlobalPlanner(*node, reason)) << reason;
  ASSERT_TRUE(PlannerNodeTestPeer::setExplorationRegion(
                  *node, true, {100, 100, 100}, {110, 110, 110})->success);
  // The resumed path must not bypass the candidate predicate.
  EXPECT_FALSE(PlannerNodeTestPeer::runGlobalPlanner(*node, reason, outside));
  EXPECT_FALSE(PlannerNodeTestPeer::runGlobalPlanner(*node, reason));
  EXPECT_TRUE(PlannerNodeTestPeer::isGlobalFrontier(*node, outside));
  ASSERT_TRUE(PlannerNodeTestPeer::setExplorationRegion(
                  *node, false, Eigen::Vector3d::Zero(),
                  Eigen::Vector3d::Zero())->success);
  EXPECT_TRUE(PlannerNodeTestPeer::runGlobalPlanner(*node, reason)) << reason;
}

TEST_F(PlannerNodeTest, ABidCarriesTheRobotsSpeedReachAndHome) {
  auto node = makeNode("drone_bid");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  EXPECT_TRUE(std::isinf(PlannerNodeTestPeer::flightReach(*node)));
  PlannerNodeTestPeer::setFlightReach(*node, 180.0);
  PlannerNodeTestPeer::setFlightReach(*node, std::nan(""));  // ignored
  EXPECT_EQ(PlannerNodeTestPeer::flightReach(*node), 180.0);
  const mgg_msgs::msg::TourBid bid = PlannerNodeTestPeer::ownTourBidMsg(*node);
  EXPECT_EQ(bid.reach_m, 180.0);
  EXPECT_GT(bid.speed_mps, 0.0);
}

TEST_F(PlannerNodeTest, AFlightReachLeavesOutClustersItCouldNotReturnFrom) {
  // The frontier at (6, 0) is 6 m out and 6 m back home (vertex 0, where
  // the robot stands): within a reach of 20 m, beyond one of 10 m.
  auto node = makeNode("drone_reach_tour");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 7.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0}, {5.0, 0.0},
              {6.0, 0.0}});
  PlannerNodeTestPeer::setTour(*node, true, 0.0);
  PlannerNodeTestPeer::solveTourOnEveryChange(*node);
  const auto finite_costs = [&node]() {
    const mgg_msgs::msg::TourBid bid =
        PlannerNodeTestPeer::ownTourBidMsg(*node);
    return std::count_if(bid.costs_from_pose.begin(),
                         bid.costs_from_pose.end(),
                         [](double cost) { return std::isfinite(cost); });
  };

  EXPECT_NE(PlannerNodeTestPeer::refreshTour(*node), mgg::kNoCluster);
  EXPECT_GT(finite_costs(), 0);
  PlannerNodeTestPeer::setFlightReach(*node, 10.0);
  EXPECT_EQ(PlannerNodeTestPeer::refreshTour(*node), mgg::kNoCluster);
  EXPECT_EQ(finite_costs(), 0);
  PlannerNodeTestPeer::setFlightReach(*node, 20.0);
  EXPECT_NE(PlannerNodeTestPeer::refreshTour(*node), mgg::kNoCluster);
  EXPECT_GT(finite_costs(), 0);
}

TEST_F(PlannerNodeTest, ARegionsLatticeDoesNotDemoteAnOutsideFrontierItPasses) {
  // Task 16 review r1, m1: a path chosen on a lattice computed inside the
  // region runs through this robot's frontier outside it. Joining the
  // roadmap, it must not carry its region-limited gain onto that frontier.
  // Without a region the lattice's gain is the truth and does.
  for (const bool region : {true, false}) {
    SCOPED_TRACE(region ? "region" : "no region");
    auto node = makeNode(region ? "region_ref_path" : "no_region_ref_path");
    PlannerNodeTestPeer::observeFloor(*node, -3.55, 4.55, -1.55, 1.55);
    PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
    const int frontier = PlannerNodeTestPeer::addGlobalChainToFrontier(
        *node, {{-1.0, 0.0}, {-2.0, 0.0}}, M_PI);
    const mgg::StateVec root = PlannerNodeTestPeer::globalVertexState(*node, 0);
    const mgg::StateVec outside =
        PlannerNodeTestPeer::globalVertexState(*node, frontier);
    if (region) {
      ASSERT_TRUE(PlannerNodeTestPeer::setExplorationRegion(
                      *node, true, {-0.5, -1.5, -5.0}, {4.0, 1.5, 5.0})
                      ->success);
    }
    const mgg::StateVec middle(-1.0, 0.0, root.z(), M_PI);
    PlannerNodeTestPeer::setSeenLattice(*node, {root, middle, outside});
    PlannerNodeTestPeer::addExplorationPath(*node, {root, middle, outside});
    EXPECT_EQ(PlannerNodeTestPeer::isGlobalFrontier(*node, frontier), region);
  }
}

/// Odometry at (x, y, z), facing +x, moving along x at `speed`.
nav_msgs::msg::Odometry::SharedPtr odometryAt(double x, double y, double z,
                                              int stamp, double speed = 0.0) {
  auto msg = std::make_shared<nav_msgs::msg::Odometry>();
  msg->header.stamp.sec = stamp;
  msg->pose.pose.position.x = x;
  msg->pose.pose.position.y = y;
  msg->pose.pose.position.z = z;
  msg->pose.pose.orientation.w = 1.0;
  msg->twist.twist.linear.x = speed;
  return msg;
}

/// A robot at home whose graph runs from home to (4, 0), with keyframes
/// along the same track, and a wall across the track at x = `wall_x` seen
/// after the graph was built (none when NaN). An aerial robot flies it at
/// 0.4 m, in the observed air under the wall's top. The graph ends at
/// `graph_end` along x.
std::shared_ptr<PlannerNode> connectedGraphAndAWallAcrossTheTrack(
    const std::string& name, double wall_x, bool aerial,
    double graph_end = 4.0) {
  auto node = makeNode(name);
  const double z = aerial ? 0.4 : 0.075;
  if (aerial) PlannerNodeTestPeer::setAerialRobot(*node);
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, odometryAt(0.0, 0.0, z, 1));
  std::vector<Eigen::Vector2d> chain;
  for (double x = 0.5; x <= graph_end + 1e-9; x += 0.5) chain.emplace_back(x, 0.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(*node, chain);
  if (!std::isnan(wall_x)) {
    PlannerNodeTestPeer::observeWallAlongY(*node, -1.5, 1.5, wall_x);
  }
  PlannerNodeTestPeer::serveMap(*node, "component:test", 0);
  auto source = std::make_unique<TrajectoryInMemory>();
  source->trajectory = keyframesAlongX(0.0, 4.0);
  for (Eigen::Isometry3d& pose : source->trajectory.poses) {
    pose.translation().z() = z;
  }
  PlannerNodeTestPeer::setKeyframeSource(*node, std::move(source));
  return node;
}

TEST_F(PlannerNodeTest, ADronesRebuildThatWouldCutOffHomeOrSplitItsGraphIsRefused) {
  // Drone smoke test (drone scout Task 17, C): a keyframe rebuild replaced
  // a graph in which home was reachable with one of three components, home
  // alone in one, and Return Home failed for good. Rebuilt here, a wall
  // across the track refuses the edges there: at x = 0.25 home is cut off,
  // at x = 2.25 the track is split. The old graph, connected, is kept. A
  // ground robot's rebuild is the correction of a graph the map no longer
  // supports (review r0, P1) and replaces it as before.
  for (const bool aerial : {true, false}) {
    for (const double wall_x : {0.25, 2.25}) {
      SCOPED_TRACE(std::string(aerial ? "aerial" : "ground") + " wall " +
                   std::to_string(wall_x));
      auto node = connectedGraphAndAWallAcrossTheTrack(
          std::string(aerial ? "drone" : "ground") + "_rebuild_wall_" +
              std::to_string(int(wall_x * 100)),
          wall_x, aerial);
      EXPECT_EQ(PlannerNodeTestPeer::rebuildRoadmap(
                    *node, PlannerNode::RoadmapRebuildTrigger::kPathUnlinkable),
                !aerial);
      EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*node), aerial ? 0 : 1);
      EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuildsRefused(*node),
                aerial ? 1 : 0);
    }
  }
  // An exploration path the drone's graph cannot link, from (4, 0), with a
  // second wall at x = 3: the rebuilt graph links the path where home
  // cannot be reached from.
  auto cut_off = connectedGraphAndAWallAcrossTheTrack(
      "drone_rebuild_links_path_away_from_home", 3.0, true, 1.0);
  PlannerNodeTestPeer::addExplorationPath(
      *cut_off, {mgg::StateVec(4.0, 0.0, 0.4, 0.0),
                 mgg::StateVec(5.5, 0.0, 0.4, 0.0)});
  EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*cut_off), 0);
  EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuildsRefused(*cut_off), 1);
  // With the track clear, the drone's rebuild replaces the graph as before.
  auto clear = connectedGraphAndAWallAcrossTheTrack("drone_rebuild_clear",
                                                    std::nan(""), true);
  EXPECT_TRUE(PlannerNodeTestPeer::rebuildRoadmap(
      *clear, PlannerNode::RoadmapRebuildTrigger::kPathUnlinkable));
  EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuildsRefused(*clear), 0);
}

TEST_F(PlannerNodeTest, AnAerialPoseTheGraphCannotReachDoesNotRebuildIt) {
  // A drone relinks as it flies (odometry and its exploration paths); its
  // keyframes, from a pad on the floor, rebuild a graph with home cut off
  // (drone scout Task 17, C). A ground robot's pose off its graph still
  // rebuilds it (APoseTheGraphCannotReachRebuildsItAndRoutesHome).
  for (const bool aerial : {false, true}) {
    SCOPED_TRACE(aerial ? "aerial" : "ground");
    auto node = robotBehindAWall(
        aerial ? "aerial_unlinkable" : "ground_unlinkable", kRoundTheWall);
    if (aerial) PlannerNodeTestPeer::setAerialRobot(*node);
    returnHome(*node, 0.0, 0.0);
    EXPECT_EQ(PlannerNodeTestPeer::rebuildAttempted(
                  *node, PlannerNode::RoadmapRebuildTrigger::kPoseUnlinkable),
              !aerial);
    EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*node), aerial ? 0 : 1);
  }
}

TEST_F(PlannerNodeTest, ARegionsLatticeUpdatesAnInsideFrontierByItsWholeView) {
  // Review r0, P1: a viewpoint inside the region whose unknown space lies
  // outside it has no gain in the region's lattice. Joining the roadmap on
  // this robot's frontier, that gain must not demote it: the frontier
  // takes the gain of all it sees. Where it sees nothing unknown at all
  // (gain counted only within 0.3 m of it, all observed), it is explored,
  // and the update demotes it as without a region.
  for (const bool explored : {false, true}) {
    SCOPED_TRACE(explored ? "explored" : "looking outside the region");
    auto node = makeNode(explored ? "region_inside_explored"
                                  : "region_inside_looks_out");
    PlannerNodeTestPeer::observeFloor(*node, -3.55, 4.55, -1.55, 1.55);
    PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
    const int frontier = PlannerNodeTestPeer::addGlobalChainToFrontier(
        *node, {{-1.0, 0.0}, {-2.0, 0.0}}, M_PI);
    if (explored) {
      PlannerNodeTestPeer::setGainSpace(*node, {-0.3, -0.3, -0.25},
                                        {0.3, 0.3, 0.25});
    }
    const mgg::StateVec root = PlannerNodeTestPeer::globalVertexState(*node, 0);
    const mgg::StateVec inside =
        PlannerNodeTestPeer::globalVertexState(*node, frontier);
    ASSERT_TRUE(PlannerNodeTestPeer::setExplorationRegion(
                    *node, true, {-2.5, -0.5, -1.0}, {-1.5, 0.5, 1.5})
                    ->success);
    const mgg::StateVec middle(-1.0, 0.0, root.z(), M_PI);
    PlannerNodeTestPeer::setSeenLattice(*node, {root, middle, inside});
    PlannerNodeTestPeer::addExplorationPath(*node, {root, middle, inside});
    EXPECT_EQ(PlannerNodeTestPeer::isGlobalFrontier(*node, frontier),
              !explored);
  }
}

/// A drone's planner with aerial_home_height_m `anchor`, over an observed
/// floor with observed air up to 2 m (or over a map that has seen nothing).
std::shared_ptr<PlannerNode> droneOverAFloor(const std::string& name,
                                             double anchor,
                                             bool observed = true) {
  auto node = makeNode(name, "world",
                       {rclcpp::Parameter("aerial_home_height_m", anchor)});
  PlannerNodeTestPeer::setAerialRobot(*node);
  if (observed) {
    PlannerNodeTestPeer::observeFloor(*node, -1.5, 6.0, -1.5, 1.5);
    PlannerNodeTestPeer::observeFreeBox(*node, {2.25, 0.0, 1.2},
                                        {7.5, 3.0, 1.6});
  }
  return node;
}

TEST_F(PlannerNodeTest, ADronesPadStartAnchorsHomeWhereItFliesSoReachIsUsable) {
  // Review r0, P1: a drone's home, seeded on the pad, has its box in the
  // floor; no edge joins it, every cluster is unreachable from home, and a
  // finite reach leaves the drone no candidate. With aerial_home_height_m,
  // a drone SwarmDeck reports landed when home is seeded has its home that
  // high over it, where it takes off to, and its flight links there.
  for (const double anchor : {0.0, 1.0}) {
    SCOPED_TRACE(anchor);
    auto node = droneOverAFloor(
        "drone_pad_home_" + std::to_string(int(anchor * 10)), anchor);
    PlannerNodeTestPeer::setFlightState(*node, "landed");
    PlannerNodeTestPeer::acceptOdometry(*node, odometryAt(0.0, 0.0, 0.075, 1));
    const mgg::StateVec home = PlannerNodeTestPeer::globalVertexState(*node, 0);
    EXPECT_NEAR(home.x(), 0.0, 1e-9);
    EXPECT_NEAR(home.z(), 0.075 + anchor, 1e-9);
    PlannerNodeTestPeer::setFlightState(*node, "flying");
    int stamp = 2;
    PlannerNodeTestPeer::acceptOdometry(*node,
                                        odometryAt(0.0, 0.0, 1.075, stamp++));
    for (double x = 0.5; x <= 4.0 + 1e-9; x += 0.5) {
      PlannerNodeTestPeer::acceptOdometry(*node,
                                          odometryAt(x, 0.0, 1.075, stamp++));
    }
    const int far = PlannerNodeTestPeer::globalVertexAt(*node, 4.0, 0.0);
    if (anchor == 0.0) {
      EXPECT_EQ(PlannerNodeTestPeer::globalVertices(*node), 1);
      continue;
    }
    ASSERT_GE(far, 0);
    PlannerNodeTestPeer::markGlobalFrontier(*node, far);
    PlannerNodeTestPeer::setTour(*node, true, 0.0);
    PlannerNodeTestPeer::solveTourOnEveryChange(*node);
    PlannerNodeTestPeer::setFlightReach(*node, 20.0);
    EXPECT_NE(PlannerNodeTestPeer::refreshTour(*node), mgg::kNoCluster);
  }
}

TEST_F(PlannerNodeTest, ADronesHomeIsLiftedOnlyWhenSwarmDeckSaysItIsLanded) {
  // Review r1, P1: at rest and low is not landed. A drone hovering 0.4 m
  // over the floor, or standing where the map has seen nothing, gave no
  // evidence of the pad and had its home lifted into the air. Only the
  // flight state SwarmDeck publishes, landed when home is seeded, lifts
  // it; without one, or with any other, home is where the drone is.
  struct Case {
    const char* name;
    std::optional<std::string> state;
    double z;
    bool observed;
    bool lifted;
  };
  const std::vector<Case> cases{
      {"landed", std::string("landed"), 0.075, true, true},
      {"no_state_on_the_pad", std::nullopt, 0.075, true, false},
      {"low_hover", std::string("flying"), 0.4, true, false},
      {"low_hover_no_state", std::nullopt, 0.4, true, false},
      {"unknown_map", std::nullopt, 0.075, false, false},
      {"restart_while_flying", std::string("flying"), 1.3, true, false},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.name);
    auto node = droneOverAFloor(std::string("drone_home_") + c.name, 1.0,
                                c.observed);
    if (c.state) PlannerNodeTestPeer::setFlightState(*node, *c.state);
    PlannerNodeTestPeer::acceptOdometry(*node, odometryAt(0.0, 0.0, c.z, 1));
    EXPECT_NEAR(PlannerNodeTestPeer::globalVertexState(*node, 0).z(),
                c.z + (c.lifted ? 1.0 : 0.0), 1e-9);
  }
  // Landed reported after home was seeded lifts nothing afterwards.
  auto late = droneOverAFloor("drone_home_landed_late", 1.0);
  PlannerNodeTestPeer::acceptOdometry(*late, odometryAt(0.0, 0.0, 0.075, 1));
  PlannerNodeTestPeer::setFlightState(*late, "landed");
  PlannerNodeTestPeer::acceptOdometry(*late, odometryAt(0.0, 0.0, 0.075, 2));
  EXPECT_NEAR(PlannerNodeTestPeer::globalVertexState(*late, 0).z(), 0.075,
              1e-9);
}

TEST_F(PlannerNodeTest, ADronesRebuiltHomeIsLiftedAsItsSeedWas) {
  // The keyframe rebuild makes the first keyframe home. It is lifted when
  // home was seeded landed, and only then: a first keyframe recorded low
  // and moving, or a planner restarted in flight (whose keyframes may start
  // on the pad, recorded before it started), keeps it where it is.
  struct Case {
    const char* name;
    const char* state;
    double first_z;  // the first keyframe's and first odometry's height
    Eigen::Vector3d first_odometry;
    double speed;
    double home_z;
  };
  const std::vector<Case> cases{
      {"landed", "landed", 0.075, {0.0, 0.0, 0.075}, 0.0, 1.075},
      {"moving_first_keyframe", "flying", 0.3, {0.0, 0.0, 0.3}, 1.0, 0.3},
      {"restarted_in_flight", "flying", 0.075, {4.0, 0.0, 1.3}, 0.0, 0.075},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.name);
    auto node = droneOverAFloor(std::string("drone_rebuilt_home_") + c.name,
                                1.0);
    PlannerNodeTestPeer::serveMap(*node, "component:test", 0);
    auto source = std::make_unique<TrajectoryInMemory>();
    source->trajectory = keyframesAlongX(0.0, 4.0);
    for (Eigen::Isometry3d& pose : source->trajectory.poses) {
      pose.translation().z() = c.first_z;
    }
    PlannerNodeTestPeer::setKeyframeSource(*node, std::move(source));
    PlannerNodeTestPeer::setFlightState(*node, c.state);
    PlannerNodeTestPeer::acceptOdometry(
        *node, odometryAt(c.first_odometry.x(), c.first_odometry.y(),
                          c.first_odometry.z(), 1, c.speed));
    ASSERT_EQ(PlannerNodeTestPeer::roadmapRebuilds(*node), 1);
    const mgg::StateVec home = PlannerNodeTestPeer::globalVertexState(*node, 0);
    EXPECT_NEAR(home.x(), 0.0, 1e-9);
    EXPECT_NEAR(home.z(), c.home_z, 1e-9);
  }
}

TEST_F(PlannerNodeTest, ADronesRebuildJudgesHomeByTheRoutesItsGraphHas) {
  // Review r0, P1: whether home reaches a place is what a route over the
  // graph finds. Through a peer's vertex it does, though no edge of this
  // robot's joins them: a rebuild cutting home off is refused. Across an
  // edge a no-go zone blocks it does not: a rebuild dropping that place
  // takes nothing a route had.
  auto bridged = makeNode("drone_rebuild_peer_bridge");
  PlannerNodeTestPeer::setAerialRobot(*bridged);
  PlannerNodeTestPeer::observeFloor(*bridged, -1.5, 6.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*bridged, odometryAt(0.0, 0.0, 0.4, 1));
  const int peer =
      PlannerNodeTestPeer::addGlobalVertex(*bridged, 7, 0.5, 0.0, 0.4, {0});
  int previous = PlannerNodeTestPeer::addGlobalVertex(*bridged, 1, 1.0, 0.0,
                                                      0.4, {peer});
  for (double x = 1.5; x <= 4.0 + 1e-9; x += 0.5) {
    previous = PlannerNodeTestPeer::addGlobalVertex(*bridged, 1, x, 0.0, 0.4,
                                                    {previous});
  }
  PlannerNodeTestPeer::observeWallAlongY(*bridged, -1.5, 1.5, 0.25);
  PlannerNodeTestPeer::serveMap(*bridged, "component:test", 0);
  auto source = std::make_unique<TrajectoryInMemory>();
  source->trajectory = keyframesAlongX(0.0, 4.0);
  for (Eigen::Isometry3d& pose : source->trajectory.poses) {
    pose.translation().z() = 0.4;
  }
  PlannerNodeTestPeer::setKeyframeSource(*bridged, std::move(source));
  EXPECT_FALSE(PlannerNodeTestPeer::rebuildRoadmap(
      *bridged, PlannerNode::RoadmapRebuildTrigger::kPathUnlinkable));
  EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuildsRefused(*bridged), 1);

  // The track at 2.25 is split by a wall now, but a no-go zone there
  // already cut the graph's routes beyond 1.5 off from home.
  auto blocked = connectedGraphAndAWallAcrossTheTrack(
      "drone_rebuild_blocked_edge", 2.25, true);
  PlannerNodeTestPeer::receiveNoGoZones(*blocked, "world", {{2.25, 0.0}});
  EXPECT_TRUE(PlannerNodeTestPeer::rebuildRoadmap(
      *blocked, PlannerNode::RoadmapRebuildTrigger::kPathUnlinkable));
  EXPECT_EQ(PlannerNodeTestPeer::roadmapRebuildsRefused(*blocked), 0);
}

TEST_F(PlannerNodeTest, AClusterBehindTheRobotWithinReachStaysInTheTour) {
  // Review r0, P2: the robot at home faces +x; the frontier is 5 m behind
  // it, so 5 m out and 5 m back. With the tour's heading weight (2) its
  // cost is about 5 + 2 pi, but a reach of 12 m covers the 10 m flown.
  auto node = makeNode("reach_behind_the_robot");
  PlannerNodeTestPeer::observeFloor(*node, -6.0, 1.5, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{-1.0, 0.0}, {-2.0, 0.0}, {-3.0, 0.0}, {-4.0, 0.0}, {-5.0, 0.0}},
      M_PI);
  PlannerNodeTestPeer::setTour(*node, true, 0.0);
  PlannerNodeTestPeer::solveTourOnEveryChange(*node);
  PlannerNodeTestPeer::setFlightReach(*node, 12.0);
  EXPECT_NE(PlannerNodeTestPeer::refreshTour(*node), mgg::kNoCluster);
  PlannerNodeTestPeer::setFlightReach(*node, 9.0);
  EXPECT_EQ(PlannerNodeTestPeer::refreshTour(*node), mgg::kNoCluster);
}

TEST_F(PlannerNodeTest, APlannerWithoutAPositiveSpeedDoesNotStart) {
  // Review r1, P1: every bid is costed at v_max. A node that started with
  // none would bid what its peers refuse, and the loader's early return
  // left the planning parameters after it unloaded.
  for (const double v_max : {0.0, -1.0, std::nan(""),
                             std::numeric_limits<double>::infinity()}) {
    SCOPED_TRACE(v_max);
    EXPECT_THROW(makeNode("bad_speed", "world",
                          {rclcpp::Parameter("PlanningParams.v_max", v_max)}),
                 std::invalid_argument);
  }
  auto node = makeNode("good_speed", "world",
                       {rclcpp::Parameter("PlanningParams.v_max", 1.5)});
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  EXPECT_EQ(PlannerNodeTestPeer::ownTourBidMsg(*node).speed_mps, 1.5);
}

TEST_F(PlannerNodeTest, FlightStatesAreHandledInTheOrderTakenSoTheLatestSeedsHome) {
  // Review r2, P1: in the node's reentrant group an older "landed" could
  // take the planner mutex after a newer "flying" and put "landed" back,
  // and home was lifted for a drone in the air. The subscription has a
  // mutually exclusive group of its own, as no_go_zones has.
  auto node = droneOverAFloor("drone_flight_state_order", 1.0);
  const rclcpp::CallbackGroup::SharedPtr group =
      PlannerNodeTestPeer::flightStateGroup(*node);
  ASSERT_NE(group, nullptr);
  EXPECT_EQ(group->type(), rclcpp::CallbackGroupType::MutuallyExclusive);
  EXPECT_NE(group, PlannerNodeTestPeer::reentrantGroup(*node));

  // "landed" then "flying", delivered through the subscription, before
  // home is seeded: home stays where the drone is.
  auto publisher_node =
      std::make_shared<rclcpp::Node>("flight_state_publisher");
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  auto publisher = publisher_node->create_publisher<std_msgs::msg::String>(
      "flight_state", rclcpp::QoS(1).transient_local().reliable());
  std_msgs::msg::String state;
  state.data = "landed";
  publisher->publish(state);
  state.data = "flying";
  publisher->publish(state);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline &&
         PlannerNodeTestPeer::latestFlightState(*node) !=
             std::optional<std::string>("flying")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  // Nothing older arrives after it.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  executor.cancel();
  spinner.join();
  ASSERT_EQ(PlannerNodeTestPeer::latestFlightState(*node),
            std::optional<std::string>("flying"));
  PlannerNodeTestPeer::acceptOdometry(*node, odometryAt(0.0, 0.0, 1.3, 1));
  EXPECT_NEAR(PlannerNodeTestPeer::globalVertexState(*node, 0).z(), 1.3,
              1e-9);
}

}  // namespace mgg_ros
