#include "mgg_core/planning_cancellation.h"
// NAVIGATE objectives for a Bunker-sized ground robot (botman) on MOLA
// planning products at its deployed 0.10 m resolution: what the lattice,
// the goal link and the shortcut make of open floor, a wall and a ramp.
// The mola_snapshot backend is always built, so these run with and without
// OctoMap.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <rclcpp/rclcpp.hpp>

#include "mgg_map_octomap/mola_map.h"
#include "mgg_ros/planner_node.h"

namespace mgg_ros {

namespace {

std::string sha256(const std::string& bytes) {
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
void append(std::string& bytes, T value) {
  std::uint64_t raw = 0;
  std::memcpy(&raw, &value, sizeof(T));
  for (std::size_t i = 0; i < 8; ++i) {
    bytes.push_back(static_cast<char>((raw >> (8 * i)) & 0xff));
  }
}

void write(const std::filesystem::path& path, const std::string& bytes) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

/// How high over the ground a product's free space is observed, metres.
/// Botman's body reaches 1.57 m; above about 1.2 m its lidar's rays rarely
/// pass over open floor, so the top of the body box stands in unknown air,
/// as on the robot's own products (botman_0, 2026-10-01).
constexpr double kObservedFreeAboveM = 1.2;

/// An upright block standing on the ground over [x0, x1] x [y0, y1], `top`
/// metres above it.
struct Block {
  double x0, x1, y0, y1, top;
};

}  // namespace

/// A MOLA planning product at `resolution`: ground at `ground(x, y)` over
/// [x0, x1] x [y0, y1] (one occupied voxel, its measured surface the exact
/// height), observed free space up to `free_above` metres over it, and
/// blocks on it. Everything else is unknown.
class MolaTerrainProduct {
 public:
  MolaTerrainProduct(double resolution, double x0, double x1, double y0,
                     double y1, const std::function<double(double, double)>& ground,
                     const std::vector<Block>& blocks = {},
                     double free_above = kObservedFreeAboveM)
      : resolution_(resolution) {
    static int sequence = 0;
    root_ = std::filesystem::temp_directory_path() /
            ("mgg-navigation-mola-" + std::to_string(::getpid()) + "-" +
             std::to_string(sequence++));
    std::filesystem::create_directories(root_ / "mola" / "components");
    using json = nlohmann::json;
    struct Voxel {
      std::int64_t x, y, z;
      bool operator<(const Voxel& o) const {
        return std::tie(x, y, z) < std::tie(o.x, o.y, o.z);
      }
    };
    std::vector<Voxel> occupied, free;
    std::vector<double> surface;  // per occupied voxel, after sorting
    std::vector<std::pair<Voxel, double>> occupied_with_top;
    const auto index = [&](double v) {
      return static_cast<std::int64_t>(std::floor(v / resolution + 1e-9));
    };
    for (std::int64_t x = index(x0); x < index(x1); ++x) {
      for (std::int64_t y = index(y0); y < index(y1); ++y) {
        const double cx = (x + 0.5) * resolution, cy = (y + 0.5) * resolution;
        const double h = ground(cx, cy);
        if (!std::isfinite(h)) continue;
        const auto gz = static_cast<std::int64_t>(std::floor(h / resolution));
        occupied_with_top.push_back({{x, y, gz}, h});
        double block_top = -1.0;
        for (const Block& b : blocks) {
          if (cx >= b.x0 && cx < b.x1 && cy >= b.y0 && cy < b.y1) {
            block_top = std::max(block_top, b.top);
          }
        }
        const auto top_z = static_cast<std::int64_t>(
            std::floor((h + free_above) / resolution));
        for (std::int64_t z = gz + 1; z <= top_z; ++z) {
          const double zc = (z + 0.5) * resolution;
          if (block_top > 0.0 && zc - h <= block_top) {
            occupied_with_top.push_back({{x, y, z}, zc});
          } else {
            free.push_back({x, y, z});
          }
        }
      }
    }
    std::sort(occupied_with_top.begin(), occupied_with_top.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    for (const auto& [voxel, top] : occupied_with_top) {
      occupied.push_back(voxel);
      surface.push_back(top);
    }
    std::sort(free.begin(), free.end());

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
        {"resolution_m", resolution},
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
    const auto size32 = static_cast<std::uint32_t>(metadata_bytes.size());
    for (int i = 0; i < 4; ++i) grid.push_back(static_cast<char>((size32 >> (8 * i)) & 0xff));
    grid += metadata_bytes;
    for (const Voxel& v : occupied) { append(grid, v.x); append(grid, v.y); append(grid, v.z); }
    for (const Voxel& v : free) { append(grid, v.x); append(grid, v.y); append(grid, v.z); }
    for (std::size_t i = 0; i < occupied.size(); ++i) {
      append(grid, occupied[i].x);
      append(grid, occupied[i].y);
      append(grid, surface[i]);
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
  ~MolaTerrainProduct() { std::filesystem::remove_all(root_); }

  const std::filesystem::path& root() const { return root_; }
  const mgg::MolaSnapshotRequest& request() const { return request_; }
  double resolution() const { return resolution_; }

 private:
  double resolution_;
  std::filesystem::path root_;
  mgg::MolaSnapshotRequest request_;
};

class PlannerNodeTestPeer {
 public:
  /// Botman as deployed (deploy/mgg hardware.yaml with bunker.yaml): the
  /// 1.344 x 0.778 m box centred on the lidar, 0.61 m over the floor, 2 x
  /// 0.61 m tall, extended by 0.05 m; the lattice and terrain limits.
  static void configureBotman(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.robot_params_.type = mgg::RobotType::kGroundRobot;
    node.robot_params_.size = Eigen::Vector3d(1.344, 0.778, 1.22);
    node.robot_params_.size_extension = Eigen::Vector3d(0.05, 0.05, 0.05);
    node.robot_params_.size_extension_min.setZero();
    node.robot_params_.safety_extension.setZero();
    node.robot_params_.bound_mode = mgg::BoundModeType::kExtendedBound;
    node.robot_params_.center_offset.setZero();
    mgg::PlanningParams& p = node.planning_params_;
    p.rr_mode = mgg::RRModeType::kGraph;
    p.edge_length_min = 0.05;
    p.edge_length_max = 1.0;
    p.edge_overshoot = 0.0;
    p.num_vertices_max = 500;
    p.num_edges_max = 10000;
    p.num_loops_cutoff = 2000;
    p.num_loops_max = 200000;
    p.nearest_range = 0.6;
    p.nearest_range_z = 0.15;
    p.nearest_range_min = 0.05;
    p.nearest_range_max = 1.0;
    p.max_ground_height = 0.61 + 0.15 + 0.175;
    p.max_step_height = 0.15;
    p.max_inclination = 27.0 * M_PI / 180.0;
    p.max_footprint_tilt = 20.0 * M_PI / 180.0;
    p.max_footprint_step = 0.10;
    p.path_clearance_margin = 0.6;
    p.path_interpolation_distance = 0.25;
    p.traverse_length_max = 20.0;
    node.allow_unknown_lattice_body_ = true;
    node.hanging_root_edge_length_max_ = 1.0;
    node.grid_params_.min_val = Eigen::Vector3d(-6.0, -6.0, -0.2);
    node.grid_params_.max_val = Eigen::Vector3d(6.0, 6.0, 0.3);
    node.grid_params_.resolution = Eigen::Vector3d(0.4, 0.4, 0.1);
    node.global_space_.setBound(Eigen::Vector3d(-20.0, -20.0, -3.0),
                                Eigen::Vector3d(20.0, 20.0, 5.0));
    node.odometry_stale_s_ = 3600.0;
  }
  /// The product in service, as a heartbeat naming it would put it.
  static bool serve(PlannerNode& node, const MolaTerrainProduct& product) {
    node.mola_map_->requestSnapshot(product.request());
    for (int i = 0; i < 400 && !node.mola_map_->getStatus(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.mapping_snapshot_.component_id = "component:test";
    node.mapping_snapshot_.epoch = 1;
    node.mapping_snapshot_.component_from_navigation.rotation.w = 1.0;
    node.have_mapping_snapshot_ = true;
    return node.mola_map_->getStatus();
  }
  /// Odometry at the lidar, 0.61 m over the ground at (x, y), facing yaw.
  static void standAt(PlannerNode& node, double x, double y, double ground,
                      double yaw, double stamp_s) {
    auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    msg->header.stamp.sec = static_cast<std::int32_t>(stamp_s);
    msg->pose.pose.position.x = x;
    msg->pose.pose.position.y = y;
    msg->pose.pose.position.z = ground + 0.61;
    msg->pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg->pose.pose.orientation.w = std::cos(yaw / 2.0);
    node.onOdometry(msg);
  }
  static bool serveFixture(PlannerNode& node, const std::string& root) {
    nlohmann::json source;
    std::ifstream(root + "/mola/source.json") >> source;
    const auto& manifest = source.at("manifests").at(0);
    mgg::MolaSnapshotRequest request;
    request.component_id = manifest.at("graph_revision").at("component_id");
    request.epoch = manifest.at("graph_revision").at("epoch");
    request.graph_revision = manifest.at("graph_revision").at("revision");
    request.geometry_revision = manifest.at("geometry_revision");
    for (const auto& submap : manifest.at("submaps"))
      request.source_stamp_ns = std::max<std::uint64_t>(request.source_stamp_ns,
          submap.value("observed_at_ns", std::uint64_t{0}));
    node.mola_map_->requestSnapshot(request);
    for (int i=0; i<400 && !node.mola_map_->getStatus(); ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    node.mapping_snapshot_.component_id = request.component_id;
    node.mapping_snapshot_.epoch = request.epoch;
    node.mapping_snapshot_.component_from_navigation.rotation.w = 1;
    node.have_mapping_snapshot_ = true;
    mgg::SensorParams sensor;
    sensor.type = mgg::SensorType::kLidar;
    sensor.max_range = 20;
    sensor.fov = Eigen::Vector2d(2*M_PI, M_PI/3);
    sensor.resolution = Eigen::Vector2d(M_PI/36, M_PI/36);
    sensor.frontier_percentage_threshold = .05;
    sensor.update();
    node.sensors_["VLP16"] = sensor;
    node.planning_params_.exp_sensor_list = {"VLP16"};
    return node.mola_map_->getStatus();
  }
  static std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> explore(PlannerNode& node) {
    auto request = std::make_shared<mgg_msgs::srv::PlannerSrv::Request>();
    request->bound_mode = request->EXTENDED_BOUND;
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    node.onPlanRequest(request, response);
    return response;
  }
  static void objectiveIdentity(const PlannerNode& node, mgg_msgs::srv::PlanObjective::Request& request) {
    request.component_id = node.mapping_snapshot_.component_id;
    request.map_epoch = node.mapping_snapshot_.epoch;
  }
  static void forceGlobalFallback(PlannerNode& node, bool force) {
    node.grid_params_.min_val.x() = force ? -1 : -6;
    node.grid_params_.max_val.x() = force ? 1 : 6;
  }
  static void hardwarePolicy(PlannerNode& node) { node.allow_unknown_lattice_body_ = false; }
  static void shortcut(PlannerNode& node, std::vector<mgg::StateVec>& path) {
    node.shortcutAndResample(path, {}, {}, true);
  }
  static bool peersOpen(const PlannerNode& node) { return node.peer_edges_open_; }
  static bool peerDeadlineSet(const PlannerNode& node) {
    return node.peer_diagnosis_deadline_.has_value();
  }
  static bool peerBlocks(PlannerNode& node) {
    return node.peerBlocksSegment(Eigen::Vector3d(0, 0, 0.935),
                                 Eigen::Vector3d(2, 0, 0.935));
  }
  static void installPeerAndGraph(PlannerNode& node) {
    node.mola_map_->setTransientDiscs({Eigen::Vector2d(1, 0)}, 0.5, 60.0);
    node.global_graph_->reset();
    auto* a = new mgg::Vertex(0, mgg::StateVec(0, 0, 0.935, 0));
    auto* b = new mgg::Vertex(1, mgg::StateVec(2, 0, 0.935, 0));
    node.global_graph_->addVertex(a);
    node.global_graph_->addVertex(b);
    node.global_graph_->addEdge(a, b, 2.0);
  }
  static void nearRequestDeadline(PlannerNode& node) {
    node.lattice_deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(80);
    node.global_graph_->reset();
    for (int i = 0; i < 500; ++i) {
      node.global_graph_->addVertex(new mgg::Vertex(i, mgg::StateVec(i, 0, .935, 0)));
      if (i) node.global_graph_->addEdge(node.global_graph_->getVertex(i-1),
                                       node.global_graph_->getVertex(i), 1);
    }
    node.global_graph_->setEdgeBlocked([](const auto&, const auto&) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1)); return false;
    });
  }
  static bool requestExpired(const PlannerNode& node) {
    return std::chrono::steady_clock::now() >= *node.lattice_deadline_;
  }
  static bool diagnosisCutShort(const PlannerNode& node) { return node.peer_diagnosis_cut_short_; }
  static void diagnose(PlannerNode& node) {
    mgg::ShortestPathsReport report;
    node.diagnosePeerSearch(0, report);
  }
  static void setBudget(PlannerNode& node, double seconds) {
    node.lattice_time_budget_s_ = seconds;
  }
  static void objective(
      PlannerNode& node,
      std::shared_ptr<mgg_msgs::srv::PlanObjective::Request> request,
      std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> response) {
    node.onObjectiveRequest(request, response);
  }
};

namespace {

using Service = mgg_msgs::srv::PlanObjective;

std::shared_ptr<PlannerNode> botmanNode(const std::string& name,
                                        const MolaTerrainProduct& product) {
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=" + name});
  options.parameter_overrides({
      rclcpp::Parameter("map.backend", "mola_snapshot"),
      rclcpp::Parameter("map.resolution", product.resolution()),
      rclcpp::Parameter("map.mola.peer_root", product.root().string()),
      rclcpp::Parameter("map.mola.snapshot_ttl_sec", 60.0),
      rclcpp::Parameter("PlanningParams.global_frame_id", "world"),
  });
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<PlannerNode>(options);
  PlannerNodeTestPeer::configureBotman(*node);
  EXPECT_TRUE(PlannerNodeTestPeer::serve(*node, product));
  return node;
}

std::shared_ptr<Service::Response> navigate(PlannerNode& node, double x,
                                            double y, double z = 0.0) {
  auto request = std::make_shared<Service::Request>();
  request->objective = Service::Request::NAVIGATE;
  PlannerNodeTestPeer::objectiveIdentity(node, *request);
  request->goal.position.x = x;
  request->goal.position.y = y;
  request->goal.position.z = z;
  request->goal.orientation.w = 1.0;
  auto response = std::make_shared<Service::Response>();
  PlannerNodeTestPeer::objective(node, request, response);
  return response;
}

double pathLength(const std::vector<geometry_msgs::msg::Pose>& path) {
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += std::hypot(path[i].position.x - path[i - 1].position.x,
                         path[i].position.y - path[i - 1].position.y);
  }
  return length;
}

/// The largest heading change between consecutive segments, degrees.
double maxCornerDeg(const std::vector<geometry_msgs::msg::Pose>& path) {
  double worst = 0.0;
  Eigen::Vector2d previous = Eigen::Vector2d::Zero();
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector2d d(path[i].position.x - path[i - 1].position.x,
                            path[i].position.y - path[i - 1].position.y);
    if (d.norm() < 1e-6) continue;
    if (previous.norm() > 0.0) {
      worst = std::max(worst, std::abs(std::atan2(
                                  previous.x() * d.y() - previous.y() * d.x(),
                                  previous.dot(d))) *
                                  180.0 / M_PI);
    }
    previous = d;
  }
  return worst;
}

double flat(double, double) { return 0.0; }

}  // namespace

class PlannerNavigationTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(PlannerNavigationTest, HardwareNavigateFitsNorthDoorwayWithObservedBody) {
  MolaTerrainProduct product(.1, -5, 5, -3, 7, flat,
      {{-5, -.6, 1.5, 1.8, 2}, {.6, 5, 1.5, 1.8, 2}}, 2.0);
  auto node = botmanNode("hardware_doorway", product);
  PlannerNodeTestPeer::hardwarePolicy(*node);
  PlannerNodeTestPeer::standAt(*node, 0, 0, 0, M_PI/2, 1);
  const auto response = navigate(*node, 0, 4);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  EXPECT_LE(pathLength(response->path), 4.2);
}

TEST_F(PlannerNavigationTest, BotmanFixtureFullServicesBenchmark) {
  const char* root = std::getenv("MGG_NAV_BENCH_PRODUCT");
  if (!root || !*root) GTEST_SKIP() << "MGG_NAV_BENCH_PRODUCT is not set";
  const int repeat = std::getenv("MGG_SERVICE_BENCH_REPEAT")
      ? std::max(1, std::atoi(std::getenv("MGG_SERVICE_BENCH_REPEAT"))) : 3;
  for (int run = 0; run < repeat; ++run) for (bool allow_unknown : {false, true}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("map.backend", "mola_snapshot"),
        rclcpp::Parameter("map.resolution", .1),
        rclcpp::Parameter("map.mola.peer_root", std::string(root)),
        rclcpp::Parameter("roadmap_rebuild.robot_id", "botman_0"),
        rclcpp::Parameter("map.mola.snapshot_ttl_sec", 60.0)});
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<PlannerNode>(options);
    PlannerNodeTestPeer::configureBotman(*node);
    if (!allow_unknown) PlannerNodeTestPeer::hardwarePolicy(*node);
    ASSERT_TRUE(PlannerNodeTestPeer::serveFixture(*node, root));
    PlannerNodeTestPeer::standAt(*node, 0, 0, -.61, M_PI, run+1);
    for (const std::string mode : {"navigate_local", "navigate_global_fallback", "navigate_15m", "explore"}) {
      PlannerNodeTestPeer::forceGlobalFallback(*node, mode == "navigate_global_fallback");
      const auto started = std::chrono::steady_clock::now();
      int status = 0;
      std::size_t poses = 0;
      std::string reason;
      if (mode == "explore") {
        const auto result = PlannerNodeTestPeer::explore(*node);
        status = result->status; poses = result->path.size();
      } else {
        // Force the observed -2 m goal through the global route too;
        // separately measure the unobserved 15 m goal's clean refusal.
        const auto result = navigate(*node, mode == "navigate_15m" ? -15 : -2, 0);
        status = result->status; poses = result->path.size(); reason = result->reason;
      }
      const double elapsed = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
      nlohmann::json row{{"service",mode},{"repeat",run},{"allow_unknown_body",allow_unknown},
          {"request_budget_ms",500},{"total_ms",elapsed},{"status",status},{"poses",poses},
          {"reason",reason},{"x86_budget_met",elapsed <= (mode=="explore" ? 350 : 300)}};
      std::printf("SERVICE_BENCH %s\n", row.dump().c_str());
      EXPECT_LT(elapsed, 1000);  // hard envelope, not a claim of the x86 target
    }
  }
}

TEST_F(PlannerNavigationTest, HardwareShortcutDoesNotCrossUnknownBodyVolume) {
  MolaTerrainProduct product(.1, -4, 6, -4, 6, flat);
  auto node = botmanNode("hardware_shortcut", product);
  PlannerNodeTestPeer::hardwarePolicy(*node);
  std::vector<mgg::StateVec> path{{0, 0, .935, 0}, {0, 2, .935, 0}, {2, 2, .935, 0}};
  PlannerNodeTestPeer::shortcut(*node, path);
  double length = 0;
  for (std::size_t i = 1; i < path.size(); ++i)
    length += (path[i].head<3>() - path[i-1].head<3>()).norm();
  EXPECT_NEAR(length, 4.0, 1e-9);
}

TEST_F(PlannerNavigationTest, InterruptedPeerDiagnosisRestoresCollisionChecks) {
  MolaTerrainProduct product(0.1, -3, 4, -3, 3, flat);
  auto node = botmanNode("peer_diagnosis_budget", product);
  PlannerNodeTestPeer::installPeerAndGraph(*node);
  ASSERT_TRUE(PlannerNodeTestPeer::peerBlocks(*node));
  bool interrupted_in_diagnosis = false;
  {
    // Deterministically expire the request only once the diagnosis has
    // disabled peer edges, not during ordinary route search.
    mgg::PlanningCancellationScope deadline([&] {
      interrupted_in_diagnosis = PlannerNodeTestPeer::peersOpen(*node);
      return interrupted_in_diagnosis;
    });
    EXPECT_THROW(PlannerNodeTestPeer::diagnose(*node), mgg::PlanningInterrupted);
  }
  EXPECT_TRUE(interrupted_in_diagnosis);
  EXPECT_FALSE(PlannerNodeTestPeer::peersOpen(*node));
  EXPECT_FALSE(PlannerNodeTestPeer::peerDeadlineSet(*node));
  EXPECT_TRUE(PlannerNodeTestPeer::peerBlocks(*node));
  PlannerNodeTestPeer::diagnose(*node);
  EXPECT_TRUE(PlannerNodeTestPeer::peerBlocks(*node));
}

TEST_F(PlannerNavigationTest, ObjectiveInterruptionRestoresPeerDiagnosisDeadline) {
  MolaTerrainProduct product(.1, -3, 5, -3, 3, flat, {{1, 4, -3, 3, 2}});
  auto node = botmanNode("objective_peer_interrupt", product);
  PlannerNodeTestPeer::standAt(*node, 0, 0, 0, 0, 1);
  PlannerNodeTestPeer::setBudget(*node, 0);
  PlannerNodeTestPeer::installPeerAndGraph(*node);
  bool during_diagnosis = false;
  {
    mgg::PlanningCancellationScope interrupt([&] {
      during_diagnosis = PlannerNodeTestPeer::peerDeadlineSet(*node);
      return during_diagnosis;
    });
    const auto response = navigate(*node, 3, 0);
    EXPECT_EQ(response->status, Service::Response::BLOCKED);
    EXPECT_NE(response->reason.find("cancelled"), std::string::npos);
  }
  EXPECT_TRUE(during_diagnosis);
  EXPECT_FALSE(PlannerNodeTestPeer::peersOpen(*node));
  EXPECT_FALSE(PlannerNodeTestPeer::peerDeadlineSet(*node));
  EXPECT_TRUE(PlannerNodeTestPeer::peerBlocks(*node));
  // Another objective must enter ordinary peer handling, not the stale
  // diagnostic mode left by the interrupted request.
  const auto next = navigate(*node, 0, 0);
  EXPECT_TRUE(PlannerNodeTestPeer::peerBlocks(*node));
  EXPECT_FALSE(PlannerNodeTestPeer::peerDeadlineSet(*node));
}

TEST_F(PlannerNavigationTest, PeerDiagnosisYieldsBeforeTheRequestDeadline) {
  MolaTerrainProduct product(.1, -3, 4, -3, 3, flat);
  auto node = botmanNode("nested_budget", product);
  PlannerNodeTestPeer::nearRequestDeadline(*node);
  mgg::PlanningCancellationScope hard([&] { return PlannerNodeTestPeer::requestExpired(*node); });
  EXPECT_NO_THROW(PlannerNodeTestPeer::diagnose(*node));
  EXPECT_TRUE(PlannerNodeTestPeer::diagnosisCutShort(*node));
  EXPECT_FALSE(PlannerNodeTestPeer::requestExpired(*node));
}

TEST_F(PlannerNavigationTest, RequestBudgetRefusesWithoutPublishingAPartialRoute) {
  MolaTerrainProduct product(0.1, -8, 8, -8, 8,
                              [](double, double) { return -0.61; });
  auto node = botmanNode("budget_navigation", product);
  PlannerNodeTestPeer::standAt(*node, 0, 0, -0.61, 0, 1);
  PlannerNodeTestPeer::setBudget(*node, 1e-9);
  const auto started = std::chrono::steady_clock::now();
  const auto response = navigate(*node, 4, 0);
  EXPECT_EQ(response->status, Service::Response::UNREACHABLE);
  EXPECT_TRUE(response->path.empty());
  EXPECT_NE(response->reason.find("planning budget exceeded"), std::string::npos);
  EXPECT_NE(response->reason.find("ms"), std::string::npos);
  EXPECT_LT(std::chrono::steady_clock::now() - started, std::chrono::seconds(1));
}

TEST_F(PlannerNavigationTest, RequestBudgetPreservesOuterCancellation) {
  MolaTerrainProduct product(0.1, -8, 8, -8, 8,
                              [](double, double) { return -0.61; });
  auto node = botmanNode("cancel_navigation", product);
  PlannerNodeTestPeer::standAt(*node, 0, 0, -0.61, 0, 1);
  mgg::PlanningCancellationScope cancelled([] { return true; });
  const auto response = navigate(*node, 4, 0);
  EXPECT_EQ(response->status, Service::Response::BLOCKED);
  EXPECT_TRUE(response->path.empty());
  EXPECT_NE(response->reason.find("cancelled"), std::string::npos);
  EXPECT_EQ(response->reason.find("budget exceeded"), std::string::npos);
}

TEST_F(PlannerNavigationTest, ATwoMetreMoveOnOpenFloorComesOutStraight) {
  // The robot faces 0.3 rad, so its lattice is turned against the goal's
  // bearing (0.93 rad): the lattice route is a staircase, which the
  // shortcut must straighten.
  MolaTerrainProduct product(0.1, -4.0, 7.0, -4.0, 5.0, flat);
  auto node = botmanNode("botman_open_2m", product);
  PlannerNodeTestPeer::standAt(*node, 0.0, 0.0, 0.0, 0.3, 1.0);
  const auto response = navigate(*node, 1.2, 1.6);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  EXPECT_NEAR(response->path.back().position.x, 1.2, 0.05);
  EXPECT_NEAR(response->path.back().position.y, 1.6, 0.05);
  EXPECT_LE(pathLength(response->path), 2.1);
  EXPECT_LE(maxCornerDeg(response->path), 10.0);
}

TEST_F(PlannerNavigationTest, AGoalFourMetresAwayInObservedFreeSpaceSucceeds) {
  MolaTerrainProduct product(0.1, -4.0, 8.0, -4.0, 6.0, flat);
  auto node = botmanNode("botman_open_4m", product);
  PlannerNodeTestPeer::standAt(*node, 0.0, 0.0, 0.0, -0.2, 1.0);
  const auto response = navigate(*node, 3.3, 2.27);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  EXPECT_NEAR(response->path.back().position.x, 3.3, 0.05);
  EXPECT_NEAR(response->path.back().position.y, 2.27, 0.05);
  EXPECT_LE(pathLength(response->path), 4.2);
  EXPECT_LE(maxCornerDeg(response->path), 10.0);
}

TEST_F(PlannerNavigationTest, AGoalBehindAWallRoutesAroundIt) {
  // A wall 3 m wide across the way, 1.8 m tall; the goal is behind it.
  const Block wall{2.0, 2.3, -1.5, 1.5, 1.8};
  MolaTerrainProduct product(0.1, -3.0, 7.0, -4.0, 4.0, flat, {wall});
  auto node = botmanNode("botman_wall", product);
  PlannerNodeTestPeer::standAt(*node, 0.0, 0.0, 0.0, 0.0, 1.0);
  const auto response = navigate(*node, 4.5, 0.0);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  // No pose's body centre within half the planning width of the wall.
  for (const auto& pose : response->path) {
    const double dx = std::max({wall.x0 - pose.position.x, 0.0,
                                pose.position.x - wall.x1});
    const double dy = std::max({wall.y0 - pose.position.y, 0.0,
                                pose.position.y - wall.y1});
    EXPECT_GT(std::hypot(dx, dy), 0.414)
        << "pose at " << pose.position.x << ", " << pose.position.y;
  }
  // Retain the original path-quality bound with the oriented body and
  // turn rule intact, rather than relaxing it to accommodate the detour.
  std::printf("wall_path_length_m=%.6f\n", pathLength(response->path));
  EXPECT_LT(pathLength(response->path), 6.8);
}

TEST_F(PlannerNavigationTest, ARampRouteStaysWithinTheSlopeLimits) {
  // Floor, then a 15 degree ramp up from x = 1 to x = 4, then a deck.
  const double grade = std::tan(15.0 * M_PI / 180.0);
  const auto ramp = [grade](double x, double) {
    return std::clamp(x - 1.0, 0.0, 3.0) * grade;
  };
  MolaTerrainProduct product(0.1, -3.0, 7.0, -3.0, 3.0, ramp);
  auto node = botmanNode("botman_ramp", product);
  PlannerNodeTestPeer::standAt(*node, 0.0, 0.0, 0.0, 0.0, 1.0);
  const auto response = navigate(*node, 5.0, 0.4, ramp(5.0, 0.4));
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  const double limit = std::tan(27.0 * M_PI / 180.0);
  for (std::size_t i = 1; i < response->path.size(); ++i) {
    const auto& a = response->path[i - 1].position;
    const auto& b = response->path[i].position;
    const double run = std::hypot(b.x - a.x, b.y - a.y);
    if (run < 0.2) continue;
    EXPECT_LE(std::abs(b.z - a.z) / run, limit + 1e-6);
    // Driving height over the ground under it.
    EXPECT_NEAR(b.z - ramp(b.x, b.y), 0.935, 0.12);
  }
  EXPECT_LE(maxCornerDeg(response->path), 10.0);
}

}  // namespace mgg_ros
