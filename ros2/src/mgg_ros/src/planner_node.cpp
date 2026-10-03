#include "mgg_core/planning_cancellation.h"
#include "mgg_ros/planner_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <limits>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <random>
#include <regex>
#include <unordered_set>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.hpp>

#include "mgg_core/local_route.h"
#include "mgg_core/log.h"
#include "mgg_core/path_turns.h"
#include "mgg_core/trajectory.h"
#include "mgg_ros/conversions.h"
#include "mgg_ros/fleet_conversions.h"
#include "mgg_ros/param_loader.h"
#ifdef MGG_WITH_OCTOMAP
#include "mgg_map_octomap/octomap_map.h"
#endif

namespace mgg_ros {

namespace {

// rrg.cpp:5206 and 5252: how far the robot travels between joining the global
// graph, and rrg.cpp:5270: between recorded states with event E1.
constexpr double kOdoUpdateMinLength = 0.5;
constexpr double kMinLength = 1.0;
// rrg.cpp:5285: the roadmap within this of a recorded state is visited.
constexpr double kUpdateRadius = 3.0;
// rrg.cpp:5651: the current state links blind to a global vertex this close.
constexpr double kLinkRadius = 1.5;
// A goal only stands for a vertex it practically coincides with; any farther
// and it gets its own vertex with checked edges.
constexpr double kGoalLinkRadius = 0.1;
/// A cluster set aside from the tour (setTourClusterAside) returns once the
/// robot is this far from where it was set aside, unless it failed while
/// already at the target: moving away must not invite an immediate return.
constexpr double kTourSetAsideMoveM = 2.0;
/// It returns too once its gain has risen by more than this fraction of
/// its scored gain when set aside, and by more than tour.min_cluster_gain:
/// new evidence about the cluster itself.
constexpr double kTourSetAsideGainRise = 0.25;
/// A tour cluster only peer bodies kept the robot from is set aside this
/// long at most: a peer moves on, and one parked must not keep the robot
/// off the cluster for good (run 10b, robot_1 and robot_3 routed through
/// parked robot_2).
constexpr double kPeerBlockedAsideS = 5.0;
/// Peer bodies' centres are compared on a grid this fine, metres, for the
/// peer generation (refreshPeerGeneration): a move of a peer across a
/// roadmap edge's margin changes it, however small against its radius.
constexpr double kPeerGenerationCellM = 0.05;
/// A robot that has moved less than this from its first odometry, and whose
/// keyframes all lie within this of its home keyframe, has not left its
/// start (PlannerNode::standingStart).
constexpr double kStandingStartMoveM = 0.5;
/// seedGlobalGraph creates the global graph's root, home, as vertex 0.
constexpr int kHomeVertexId = 0;
/// The flight_state SwarmDeck's adapter publishes for a drone on the ground
/// (PlannerNode::home_seeded_landed_).
constexpr const char* kFlightStateLanded = "landed";

bool sameRoadmapSnapshot(const mgg::GraphExchange& a, const mgg::GraphExchange& b) {
  if (a.vertices.size() != b.vertices.size() || a.edges.size() != b.edges.size()) return false;
  for (std::size_t i = 0; i < a.vertices.size(); ++i) {
    const auto& x = a.vertices[i];
    const auto& y = b.vertices[i];
    if (x.id != y.id || x.robot_id != y.robot_id || x.state != y.state ||
        x.num_unknown_voxels != y.num_unknown_voxels ||
        x.num_free_voxels != y.num_free_voxels ||
        x.num_occupied_voxels != y.num_occupied_voxels ||
        x.is_frontier != y.is_frontier || x.visited != y.visited) return false;
  }
  for (std::size_t i = 0; i < a.edges.size(); ++i) {
    const auto& x = a.edges[i];
    const auto& y = b.edges[i];
    if (x.source_id != y.source_id || x.target_id != y.target_id || x.weight != y.weight)
      return false;
  }
  return true;
}

bool nearPathXY(const std::vector<mgg::StateVec>& path,
                const Eigen::Vector2d& point, double tolerance) {
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector2d a = path[i - 1].head<2>();
    const Eigen::Vector2d step = path[i].head<2>() - a;
    const double t = step.squaredNorm() > 1e-12
        ? std::clamp((point - a).dot(step) / step.squaredNorm(), 0.0, 1.0) : 0.0;
    if ((point - a - t * step).norm() <= tolerance) return true;
  }
  return false;
}

/// The in-service vertices of `graph` a route from home reaches: Dijkstra
/// over its edges, those a no-go zone blocks left out and other robots'
/// vertices passed through, as routeOverGlobalGraph searches (the caller
/// opens the edges peer bodies close). Home itself is always in it when the
/// graph has one.
std::unordered_set<int> reachedFromHome(mgg::GraphManager& graph) {
  std::unordered_set<int> reached;
  if (graph.vertices_map_.count(kHomeVertexId) == 0) return reached;
  reached.insert(kHomeVertexId);
  mgg::ShortestPathsReport report;
  if (graph.getNumVertices() < 2 ||
      !graph.findShortestPaths(kHomeVertexId, report) || !report.status) {
    return reached;
  }
  for (const auto& entry : graph.vertices_map_) {
    if (entry.second != nullptr && graph.inService(*entry.second) &&
        std::isfinite(mgg::reachedDistance(report, entry.first))) {
      reached.insert(entry.first);
    }
  }
  return reached;
}

/// Why a keyframe rebuild must not replace `current` with `rebuilt`, or
/// empty when it may. In the drone smoke test (drone scout Task 17, C) a
/// rebuild swapped a graph whose home Return Home reached for one of three
/// parts with home alone in one, and Return Home failed from then on. So
/// when a route from home reaches another of this robot's vertices in
/// `current` (reachedFromHome), a route from home in `rebuilt` must reach
/// `linked` (what the rebuild was for; without it, any other vertex), and
/// every such place: the rebuilt vertex nearest it, within
/// `match_radius`, where there is one.
std::string rebuildLosesHome(mgg::GraphManager& current,
                             mgg::GraphManager& rebuilt,
                             const mgg::Vertex* linked, int robot_id,
                             double match_radius) {
  std::vector<const mgg::Vertex*> places;
  for (const int id : reachedFromHome(current)) {
    const mgg::Vertex* vertex = current.vertices_map_.at(id);
    if (id != kHomeVertexId && vertex->robot_id == robot_id) {
      places.push_back(vertex);
    }
  }
  if (places.empty()) return "";
  const std::unordered_set<int> reached = reachedFromHome(rebuilt);
  if (reached.empty()) return "has no home";
  if (linked != nullptr) {
    if (reached.count(linked->id) == 0) {
      return "would not reach home from where it links";
    }
  } else if (reached.size() < 2) {
    return "would cut home off";
  }
  for (const mgg::Vertex* place : places) {
    mgg::StateVec state = place->state;
    mgg::Vertex* nearest = nullptr;
    if (rebuilt.getNearestVertexInRange(&state, match_radius, &nearest) &&
        nearest != nullptr && reached.count(nearest->id) == 0) {
      return "would cut off places the current graph connects to home";
    }
  }
  return "";
}

/// Restore temporary planner state on every exit, including interruption.
template <typename T>
struct RestoreScope {
  explicit RestoreScope(T& value) : value_(value), previous_(value) {}
  ~RestoreScope() { value_ = previous_; }
  RestoreScope(const RestoreScope&) = delete;
  RestoreScope& operator=(const RestoreScope&) = delete;
  T& value_;
  T previous_;
};

struct RequestActivity {
  std::atomic<bool>& active;
  explicit RequestActivity(std::atomic<bool>& flag) : active(flag) { active = true; }
  ~RequestActivity() { active = false; }
};

/// Sets a flag for a scope and restores it after.
struct FlagScope {
  explicit FlagScope(bool& flag) : flag_(flag), previous_(flag) { flag_ = true; }
  ~FlagScope() { flag_ = previous_; }
  FlagScope(const FlagScope&) = delete;
  FlagScope& operator=(const FlagScope&) = delete;
  bool& flag_;
  bool previous_;
};

/// Sets a request's lattice deadline `budget_s` from now, 0 none, and
/// restores the one before it.
struct DeadlineScope {
  DeadlineScope(std::optional<std::chrono::steady_clock::time_point>& deadline,
                double budget_s, const bool* diagnosing = nullptr)
      : deadline_(deadline), previous_(deadline),
        outer_(mgg::planning_cancelled), started_(std::chrono::steady_clock::now()),
        cancellation_([this] {
          if (outer_ && (*outer_)()) { cancelled_ = true; return true; }
          exhausted_ = deadline_ && std::chrono::steady_clock::now() >= *deadline_;
          if (exhausted_ && diagnosing_) interrupted_diagnosis_ = *diagnosing_;
          return exhausted_;
        }) {
    diagnosing_ = diagnosing;
    if (budget_s > 0.0 && !deadline_) {
      deadline_ = std::chrono::steady_clock::now() +
                  std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                      std::chrono::duration<double>(budget_s));
    }
  }
  ~DeadlineScope() { deadline_ = previous_; }
  DeadlineScope(const DeadlineScope&) = delete;
  DeadlineScope& operator=(const DeadlineScope&) = delete;
  std::optional<std::chrono::steady_clock::time_point>& deadline_;
  std::optional<std::chrono::steady_clock::time_point> previous_;
  const std::function<bool()>* outer_;
  std::chrono::steady_clock::time_point started_;
  bool interrupted_diagnosis_ = false;
  const bool* diagnosing_ = nullptr;
  bool exhausted_ = false;
  bool cancelled_ = false;
  mgg::PlanningCancellationScope cancellation_;
  std::string reason() const {
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - started_).count();
    return std::string(cancelled_ ? "cancelled" : "planning budget exceeded") +
           " after " + std::to_string(ms) + " ms";
  }
};

// Inner searches must yield with time left to validate/publish best-so-far.
// Reserve a fifth of remaining request time (at most 50 ms).
std::chrono::steady_clock::time_point innerDeadline(
    const std::optional<std::chrono::steady_clock::time_point>& request,
    double budget_s) {
  const auto now = std::chrono::steady_clock::now();
  auto deadline = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::max(0.0, budget_s)));
  if (request) {
    const auto remaining = std::max(*request - now, std::chrono::steady_clock::duration::zero());
    const auto reserve = std::min(remaining / 5,
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::milliseconds(50)));
    deadline = std::min(deadline, *request - reserve);
  }
  return deadline;
}

double secondsSince(const std::chrono::steady_clock::time_point& then) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - then)
      .count();
}

}  // namespace

PlannerNode::PlannerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mggplanner_node", options) {
  // Route mgg_core's warnings into the ROS log. The core deliberately does not
  // depend on rclcpp, so it emits through a sink instead.
  mgg::setLogSink([this](mgg::LogLevel level, const std::string& message) {
    switch (level) {
      case mgg::LogLevel::kError:
        RCLCPP_ERROR(get_logger(), "%s", message.c_str());
        break;
      case mgg::LogLevel::kWarn:
        RCLCPP_WARN(get_logger(), "%s", message.c_str());
        break;
      default:
        RCLCPP_INFO(get_logger(), "%s", message.c_str());
        break;
    }
  });

  planner_config_state_.incarnation = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count());
  RCLCPP_INFO(get_logger(), "planner configuration incarnation: %llu (wall-clock ns)",
              static_cast<unsigned long long>(planner_config_state_.incarnation));

  loadParameters();

  // The map: an octree built from point clouds, or the MOLA product a
  // mapping process publishes, placed in the planning frame by the
  // MappingSnapshot heartbeats. Either way the planner only sees
  // mgg::MapInterface.
  const double map_resolution = declareOrGet<double>(this, "map.resolution", 0.2);
  map_backend_ = declareOrGet<std::string>(this, "map.backend", map_backend_);
  if (map_backend_ == "cloud_octomap") {
#ifdef MGG_WITH_OCTOMAP
    mgg::OctomapConfig map_cfg;
    map_cfg.resolution = map_resolution;
    map_cfg.max_range = declareOrGet<double>(this, "map.max_range", 20.0);
    auto backend = std::make_unique<mgg::OctomapMap>(map_cfg);
    cloud_map_ = backend.get();
    map_ = std::move(backend);
#else
    throw std::invalid_argument(
        "map.backend cloud_octomap is not available: mgg_map_octomap was "
        "built with MGG_WITH_OCTOMAP=OFF; use mola_snapshot");
#endif
  } else if (map_backend_ == "mola_snapshot") {
    mgg::MolaMapConfig map_cfg;
    map_cfg.resolution = map_resolution;
    map_cfg.peer_root = declareOrGet<std::string>(this, "map.mola.peer_root", "");
    map_cfg.snapshot_ttl_sec = std::clamp(
        declareOrGet<double>(this, "map.mola.snapshot_ttl_sec", 3.0), 0.1, 60.0);
    map_cfg.max_snapshot_bytes = static_cast<std::size_t>(std::clamp(
        declareOrGet<std::int64_t>(this, "map.mola.max_snapshot_bytes", 67108864),
        std::int64_t{1024}, std::int64_t{64 * 1024 * 1024}));
    map_cfg.max_index_bytes = static_cast<std::size_t>(std::clamp(
        declareOrGet<std::int64_t>(this, "map.mola.max_index_bytes", 4194304),
        std::int64_t{1024}, std::int64_t{4 * 1024 * 1024}));
    map_cfg.max_grid_bytes = static_cast<std::size_t>(std::clamp(
        declareOrGet<std::int64_t>(this, "map.mola.max_grid_bytes", 268435456),
        std::int64_t{1024}, std::int64_t{256 * 1024 * 1024}));
    map_cfg.max_voxels = static_cast<std::size_t>(std::clamp(
        declareOrGet<std::int64_t>(this, "map.mola.max_voxels", 2000000),
        std::int64_t{1}, std::int64_t{2000000}));
    map_cfg.max_load_time = std::chrono::milliseconds(std::clamp(
        declareOrGet<std::int64_t>(this, "map.mola.max_load_ms", 2000),
        std::int64_t{1}, std::int64_t{10000}));
    // Map freshness, one line per grid installed (run-9 measurement):
    // steady-clock time from the authority heartbeat to installation, and
    // the node clock (sim time in simulation) less the product's newest
    // keyframe stamp. Captures no `this`: it runs on the map's worker.
    // The robot is the namespace's first segment (/robot_1/mgg), else the
    // node's name.
    std::string robot = get_namespace();
    const std::size_t first = robot.find_first_not_of('/');
    robot = first == std::string::npos ? "" : robot.substr(first);
    robot = robot.substr(0, robot.find('/'));
    if (robot.empty()) robot = get_name();
    map_cfg.on_install = [logger = get_logger(), clock = get_clock(),
                          robot](const mgg::MolaInstallation& installed) {
      char source_age[32] = "null";
      if (installed.source_stamp_ns > 0) {
        std::snprintf(source_age, sizeof(source_age), "%.3f",
                      clock->now().seconds() -
                          static_cast<double>(installed.source_stamp_ns) * 1e-9);
      }
      RCLCPP_INFO(logger,
                  "mapping_age {\"robot_id\":\"%s\",\"authority_to_mgg_age_s\":%.3f,"
                  "\"source_to_mgg_age_s\":%s}",
                  robot.c_str(), installed.authority_to_install_s, source_age);
    };
    auto backend = std::make_unique<mgg::MolaMap>(map_cfg);
    mola_map_ = backend.get();
    // A drone's box is the square its footprint circumscribes: held along
    // the map's grid and swept exactly (lane drone-door).
    mola_map_->setGridAlignedBody(robot_params_.type ==
                                  mgg::RobotType::kAerialRobot);
    map_ = std::move(backend);
    RCLCPP_INFO(get_logger(),
                "map backend '%s': source '%s', resolution %.3f m, TTL %.1f s",
                map_backend_.c_str(), map_cfg.peer_root.c_str(),
                map_cfg.resolution, map_cfg.snapshot_ttl_sec);
  } else {
    throw std::invalid_argument(
        "map.backend must be cloud_octomap or mola_snapshot");
  }
  ground_ = std::make_unique<mgg::GroundProjection>(*map_, planning_params_);
  geofence_ = std::make_unique<mgg::GeofenceManager>();
  local_graph_ = std::make_shared<mgg::GraphManager>();
  global_graph_ = std::make_shared<mgg::GraphManager>();
  local_graph_->setEdgeBlocked([this](const mgg::Vertex& a, const mgg::Vertex& b) {
    return robot_params_.type == mgg::RobotType::kAerialRobot &&
           peerBlocksSegment(a.state.head<3>(), b.state.head<3>());
  });
  global_graph_->setEdgeBlocked(
      [this](const mgg::Vertex& a, const mgg::Vertex& b) {
        return globalEdgeBlocked(a, b);
      });
  local_graph_->setRobotId(static_cast<int>(planning_params_.robot_id));
  global_graph_->setRobotId(static_cast<int>(planning_params_.robot_id));
  // The global graph expansion (rrg.cpp:80 to 82) samples in the local box
  // the lattice is built in, seeded as upstream's sampler was
  // (random_sampler.cpp:242).
  random_sampler_.setBound(grid_params_.min_val, grid_params_.max_val);
  random_sampler_.reset(std::random_device{}());

  aerial_peer_robot_ids_ = declareOrGet<std::vector<std::int64_t>>(
      this, "aerial_peer_robot_ids", std::vector<std::int64_t>{});
  std::string aerial_ids;
  for (const auto id : aerial_peer_robot_ids_) {
    if (id < 0 || id > std::numeric_limits<int>::max()) {
      throw std::invalid_argument("aerial_peer_robot_ids must be non-negative int32 robot IDs");
    }
    if (!aerial_ids.empty()) aerial_ids += ", ";
    aerial_ids += std::to_string(id);
  }
  RCLCPP_INFO(get_logger(), "aerial_peer_robot_ids: [%s]; ground receivers exclude these frontier owners",
              aerial_ids.c_str());

  // Inter-robot transforms. `static` reads fixed ones from
  // neighbour_offsets (bring-up with known spawn poses); `topic` takes live
  // estimates on neighbour_transforms, each T_ours_theirs from our planning
  // frame to a neighbour's, and holds them for neighbour_transform_ttl_sec.
  poses_ = std::make_unique<mgg::StaticPoseSource>();
  neighbour_pose_source_ = declareOrGet<std::string>(
      this, "neighbour_pose_source", neighbour_pose_source_);
  if (neighbour_pose_source_ != "static" && neighbour_pose_source_ != "topic") {
    throw std::invalid_argument("neighbour_pose_source must be static or topic");
  }
  neighbour_transform_ttl_s_ = std::max(
      0.1, declareOrGet<double>(this, "neighbour_transform_ttl_sec",
                                neighbour_transform_ttl_s_));
  const auto offsets = declareOrGet<std::vector<double>>(
      this, "neighbour_offsets", std::vector<double>{});
  if (offsets.size() % 4 != 0) {
    RCLCPP_ERROR(get_logger(),
                 "neighbour_offsets must be groups of 4 (robot_id, dx, dy, dz)"
                 "; got %zu values", offsets.size());
  } else {
    for (size_t i = 0; i + 3 < offsets.size(); i += 4) {
      poses_->setOffset(static_cast<int>(offsets[i]), offsets[i + 1],
                        offsets[i + 2], offsets[i + 3]);
      RCLCPP_INFO(get_logger(), "neighbour %d at (%.2f, %.2f, %.2f)",
                  static_cast<int>(offsets[i]), offsets[i + 1], offsets[i + 2],
                  offsets[i + 3]);
    }
  }

  cloud_tf_timeout_sec_ =
      declareOrGet<double>(this, "cloud_tf_timeout_sec", cloud_tf_timeout_sec_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  input_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions input_opts;
  input_opts.callback_group = input_callback_group_;
  planning_status_pub_ = create_publisher<std_msgs::msg::String>(
      "planning_status", rclcpp::QoS(1).transient_local());
  setAcquiringObservations(true);
  planning_status_timer_ = create_wall_timer(std::chrono::milliseconds(250),
      [this]() { publishPlanningStatus(); }, input_callback_group_);
  input_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
    std::unique_lock<std::recursive_mutex> lock(planner_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      applyPendingCancel();
      applyLatestOdometry();
      if (robot_params_.type == mgg::RobotType::kAerialRobot) refreshScoutingExclusions();
    }
  }, callback_group_);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onOdometry(m); },
      input_opts);

  if (cloud_map_ != nullptr) {
    rclcpp::QoS cloud_qos(rclcpp::KeepLast(10));
    cloud_qos.best_effort();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud", cloud_qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
          onPointCloud(m);
        },
        sub_opts);
  }
  if (mola_map_ != nullptr) {
    snapshot_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions snapshot_opts;
    snapshot_opts.callback_group = snapshot_callback_group_;
    mapping_snapshot_sub_ = create_subscription<mgg_msgs::msg::MappingSnapshot>(
        "mapping_snapshot", rclcpp::QoS(1).transient_local(),
        [this](mgg_msgs::msg::MappingSnapshot::ConstSharedPtr m) {
          onMappingSnapshot(m);
        },
        snapshot_opts);
  }

  neighbour_sub_ = create_subscription<mgg_msgs::msg::Graph>(
      "neighbour_graph_in", rclcpp::QoS(10),
      [this](mgg_msgs::msg::Graph::ConstSharedPtr m) { onNeighbourGraph(m); },
      sub_opts);
  if (neighbour_pose_source_ == "topic") {
    neighbour_transforms_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "neighbour_transforms", rclcpp::QoS(10),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
          onNeighbourTransforms(m);
        },
        sub_opts);
  }

  // Fleet coordination: frontiers a peer has reserved are skipped by the
  // selectors (the other-robot penalty of rrg.cpp:5804, made exact), and a
  // peer's body is neither an obstacle to map around nor a target.
  reservation_exclusion_radius_m_ = std::max(
      0.0, declareOrGet<double>(this, "reservation_exclusion_radius_m",
                                reservation_exclusion_radius_m_));
  reservation_exclusion_ttl_s_ = std::max(
      0.0, declareOrGet<double>(this, "reservation_exclusion_ttl_s",
                                reservation_exclusion_ttl_s_));
  fleet_coverage_radius_m_ = std::max(
      0.0, declareOrGet<double>(this, "fleet_coverage_radius_m",
                                fleet_coverage_radius_m_));
  fleet_coverage_max_link_checks_ = static_cast<int>(std::clamp<std::int64_t>(
      declareOrGet<std::int64_t>(this, "fleet_coverage_max_link_checks",
                                 fleet_coverage_max_link_checks_),
      0, std::numeric_limits<int>::max()));
  fleet_coverage_max_links_per_frontier_ =
      static_cast<int>(std::clamp<std::int64_t>(
          declareOrGet<std::int64_t>(this,
                                     "fleet_coverage_max_links_per_frontier",
                                     fleet_coverage_max_links_per_frontier_),
          1, std::numeric_limits<int>::max()));
  // A frontier's links are checked in one pass or not at all: a budget
  // smaller than one frontier's would never start one (review r4).
  if (fleet_coverage_max_link_checks_ < fleet_coverage_max_links_per_frontier_) {
    RCLCPP_ERROR(get_logger(),
                 "fleet_coverage_max_link_checks (%d) is below "
                 "fleet_coverage_max_links_per_frontier (%d): raised to %d",
                 fleet_coverage_max_link_checks_,
                 fleet_coverage_max_links_per_frontier_,
                 fleet_coverage_max_links_per_frontier_);
    fleet_coverage_max_link_checks_ = fleet_coverage_max_links_per_frontier_;
    fleet_coverage_link_budget_raised_ = true;
  }
  coordination_exclusions_sub_ =
      create_subscription<geometry_msgs::msg::PoseArray>(
          "coordination_exclusions", rclcpp::QoS(10),
          [this](geometry_msgs::msg::PoseArray::ConstSharedPtr m) {
            onCoordinationExclusions(m);
          },
          sub_opts);
  peer_body_radius_m_ = std::clamp(
      declareOrGet<double>(this, "peer_body_radius_m", peer_body_radius_m_),
      0.0, 5.0);
  peer_body_ttl_s_ = std::clamp(
      declareOrGet<double>(this, "peer_body_ttl_s", peer_body_ttl_s_), 0.1,
      30.0);
  scouting_exclusion_ttl_s_ = declareOrGet<double>(
      this, "scouting_exclusion_ttl_s", scouting_exclusion_ttl_s_);
  if (!std::isfinite(scouting_exclusion_ttl_s_) || scouting_exclusion_ttl_s_ <= 0.0) {
    throw std::invalid_argument("scouting_exclusion_ttl_s must be finite and positive");
  }
  aerial_peer_margin_m_ = declareOrGet<double>(this, "aerial_peer_margin_m", aerial_peer_margin_m_);
  if (!std::isfinite(aerial_peer_margin_m_) || aerial_peer_margin_m_ < 0) {
    throw std::invalid_argument("aerial_peer_margin_m must be finite and non-negative");
  }
  // Each message replaces the set, published under the planner mutex, so
  // they are handled one at a time, in the order taken, in a group of their
  // own, as the no-go zones' below: in the reentrant group, messages that
  // waited on a plan together took the mutex in any order, and an older
  // set could replace a newer one, with a fresh TTL (review r1, R3).
  peer_bodies_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions peer_bodies_opts;
  peer_bodies_opts.callback_group = peer_bodies_group_;
  peer_bodies_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "peer_bodies", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr m) {
        onPeerBodies(m);
      },
      peer_bodies_opts);
  aerial_peer_bodies_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "aerial_peer_bodies", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr m) { onAerialPeerBodies(m); },
      peer_bodies_opts);
  // Latched: SwarmDeck republishes the whole set on each change and owns
  // the zones' lifetime; a planner started later still gets the last set.
  // Each message replaces the set, so they are handled one at a time, in
  // the order taken, in a group of their own: in the reentrant group an
  // older message could take the planner mutex after a newer one and put
  // an obsolete set back (review r0, I-4).
  no_go_zones_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions no_go_opts;
  no_go_opts.callback_group = no_go_zones_group_;
  no_go_zones_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "no_go_zones", rclcpp::QoS(1).transient_local(),
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr m) {
        onNoGoZones(m);
      },
      no_go_opts);
  // Latched, as the producer publishes it (transient local, re-published
  // at least every second); in the no-go group, so one set replaces
  // another in the order taken.
  scouting_revision_pub_ = create_publisher<std_msgs::msg::UInt64>(
      "scouting_exclusion_revision", rclcpp::QoS(1).transient_local());
  scouting_exclusions_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "scouting_exclusions", rclcpp::QoS(1).transient_local(),
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr m) {
        onScoutingExclusions(m);
      },
      no_go_opts);
  no_go_discs_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "no_go_discs", rclcpp::QoS(1).transient_local(),
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr m) {
        onNoGoDiscs(m);
      },
      no_go_opts);
  // Latched: SwarmDeck's adapter publishes the drone's flight state on each
  // change; a planner started later gets the current one. Each state
  // replaces the last, so, as no_go_zones, they are handled one at a time,
  // in the order taken, in a group of their own: in the reentrant group an
  // older "landed" could take the planner mutex after a newer "flying" and
  // lift the home of a drone in the air (Task 17 review r2, P1).
  flight_state_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions flight_state_opts;
  flight_state_opts.callback_group = flight_state_group_;
  flight_state_sub_ = create_subscription<std_msgs::msg::String>(
      "flight_state", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        onFlightState(msg);
      },
      flight_state_opts);
  flight_reach_sub_ = create_subscription<std_msgs::msg::Float64>(
      "flight_reach_m", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        onFlightReach(msg);
      },
      sub_opts);

  graph_pub_ = create_publisher<mgg_msgs::msg::Graph>("neighbour_graph_out",
                                                      rclcpp::QoS(10));
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "graph_markers", rclcpp::QoS(1));
  // Latched: the best path is a latest-value topic, and a follower or RViz
  // started after the planner would otherwise see nothing until the next
  // cycle.
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "best_path", rclcpp::QoS(1).transient_local());
  // The tour (tour-exploration design §2), latched as best_path is.
  tour_planner_ = std::make_unique<mgg::TourPlanner>(tour_params_);
  tour_pub_ = create_publisher<nav_msgs::msg::Path>(
      "tour", rclcpp::QoS(1).transient_local());

  // Fleet frontier assignment (tour-exploration design §3). Each robot
  // publishes on tour_bid_out/tour_award_out and reads its peers' on
  // tour_bid_in/tour_award_in; a deployment remaps each pair to one shared
  // topic, as it does neighbour_graph_out/in.
  if (fleet_params_.enabled) {
    fleet_ = std::make_unique<mgg::FleetCoordinator>(
        static_cast<int>(planning_params_.robot_id), fleet_params_,
        tour_params_.commit_margin);
    tour_bid_pub_ = create_publisher<mgg_msgs::msg::TourBid>(
        "tour_bid_out", rclcpp::QoS(10));
    tour_award_pub_ = create_publisher<mgg_msgs::msg::TourAward>(
        "tour_award_out", rclcpp::QoS(10));
    tour_bid_sub_ = create_subscription<mgg_msgs::msg::TourBid>(
        "tour_bid_in", rclcpp::QoS(10),
        [this](mgg_msgs::msg::TourBid::ConstSharedPtr m) { onTourBid(m); },
        sub_opts);
    tour_award_sub_ = create_subscription<mgg_msgs::msg::TourAward>(
        "tour_award_in", rclcpp::QoS(10),
        [this](mgg_msgs::msg::TourAward::ConstSharedPtr m) {
          onTourAward(m);
        },
        sub_opts);
    fleet_timer_ = create_timer(
        std::chrono::duration<double>(kFleetTickPeriodS),
        [this]() {
          // The time is read under the lock, as every fleet entry point
          // reads it: a call or award handled between an earlier reading
          // and the tick would leave the tick's time behind the one the
          // coordinator last saw, which it takes for a clock reset (it
          // forgets the call waiting for this robot's bid).
          const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
          fleetTick(now().seconds());
        },
        callback_group_);
  }

  global_vertex_spacing_ =
      declareOrGet<double>(this, "global_vertex_spacing", 1.0);
  odometry_stale_s_ = std::max(
      0.1, declareOrGet<double>(this, "odometry_stale_s", odometry_stale_s_));
  auto_global_planner_low_gain_rounds_ = std::max(
      1, static_cast<int>(declareOrGet<std::int64_t>(
             this, "auto_global_planner_low_gain_rounds",
             auto_global_planner_low_gain_rounds_)));
  global_frontier_reach_m_ = std::max(
      0.5, declareOrGet<double>(this, "global_frontier_reach_m",
                                global_frontier_reach_m_));
  reach_distance_ = std::max(
      0.0, declareOrGet<double>(this, "reach_distance", reach_distance_));
  // Upstream's lattice only admits a cell whose whole body volume is known
  // free (rrg.cpp buildGridGraphExapnd, getBoxStatus with stop_at_unknown),
  // on a map that carves free space from every scan. A map that carves it
  // from keyframes only leaves a parked robot's surroundings largely
  // unobserved, so it would never take the first step. This admits a cell
  // whose body volume is partly unobserved; known occupied volume still
  // rejects it, and mapped ground under it and along the edge stays
  // mandatory. Off by default; simulation with a keyframe map turns it on.
  ground_exploration_lattice_budget_s_ = declareOrGet<double>(
      this, "ground_exploration_lattice_budget_s", mgg::kGroundExplorationLatticeBudgetS);
  if (!std::isfinite(ground_exploration_lattice_budget_s_) ||
      ground_exploration_lattice_budget_s_ <= 0)
    throw std::invalid_argument("ground_exploration_lattice_budget_s must be positive");
  allow_unknown_lattice_body_ = declareOrGet<bool>(
      this, "allow_unknown_lattice_body", allow_unknown_lattice_body_);
  // Old simulation configurations retain their explicitly relaxed control.
  // New hardware policy is opt-in and otherwise defaults to strict.
  unknown_body_policy_ = declareOrGet<std::string>(this, "unknown_body_policy",
      allow_unknown_lattice_body_ ? "legacy_relaxed" : "strict");
  unknown_body_sensor_ = declareOrGet<std::string>(this, "unknown_body_sensor", "VLP16");
  if (unknown_body_policy_ != "strict" && unknown_body_policy_ != "above_sensor_fov" &&
      unknown_body_policy_ != "legacy_relaxed")
    throw std::invalid_argument("unknown_body_policy must be strict, above_sensor_fov or legacy_relaxed");
  allow_unknown_lattice_body_ = unknown_body_policy_ == "legacy_relaxed";
  if (unknown_body_policy_ == "above_sensor_fov" &&
      !makeContext().unknown_body_above_center) {
    RCLCPP_ERROR(get_logger(),
        "unknown_body_policy=above_sensor_fov CANNOT APPLY: sensor '%s' must be an upright "
        "ground-robot lidar with finite full vertical FOV in (0, pi) and mount_height "
        "strictly above the planning body bottom; USING STRICT UNKNOWN POLICY",
        unknown_body_sensor_.c_str());
  }
  // The most one request (an objective, a plan request) spends sweeping
  // lattices, seconds: past it a sweep stops and the request plans over
  // what it built, or refuses with the budget named. A planning call longer
  // than the MOLA snapshot TTL (3 s) let the map expire under the next
  // request (botman, 2026-10-01). 0 is no bound.
  lattice_time_budget_s_ = std::max(
      0.0, declareOrGet<double>(this, "lattice_time_budget_s",
                                lattice_time_budget_s_));
  // A lidar does not see the ground under the robot: within its blind radius
  // a keyframe map holds no floor, so the graph root has no support until
  // the robot has driven away from where it stands. The root then sits at
  // the physical driving height (base height above the assumed floor) as a
  // hanging vertex, and one edge from it may reach up to this far to the
  // first supported vertex; every other check on that edge still applies.
  // Until the robot first moves, as its keyframe trajectory shows, the
  // ground within this reach of it counts as observed (standingStart()).
  hanging_root_edge_length_max_ = std::max(
      0.0, declareOrGet<double>(this, "hanging_root_edge_length_max",
                                hanging_root_edge_length_max_));
  // A drone's pad is on the floor, where its box meets the ground and no
  // edge joins it: its home is this far over the pad, at its take-off
  // height, when SwarmDeck reports it landed on flight_state as home is
  // seeded (seedGlobalGraph, rebuildGlobalGraphFromKeyframes).
  aerial_frontier_height_m_ = declareOrGet<double>(this, "aerial_frontier_height_m", aerial_frontier_height_m_);
  aerial_min_height_m_ = declareOrGet<double>(this, "aerial_min_height_m", aerial_min_height_m_);
  aerial_max_height_m_ = declareOrGet<double>(this, "aerial_max_height_m", aerial_max_height_m_);
  if (!std::isfinite(aerial_frontier_height_m_) || !std::isfinite(aerial_min_height_m_) ||
      !std::isfinite(aerial_max_height_m_) || aerial_min_height_m_ <= 0 ||
      aerial_max_height_m_ < aerial_min_height_m_) {
    throw std::invalid_argument("invalid aerial frontier height band");
  }
  aerial_home_height_m_ = std::max(
      0.0, declareOrGet<double>(this, "aerial_home_height_m",
                                aerial_home_height_m_));
  aerial_home_state_wait_s_ = declareOrGet<double>(
      this, "aerial_home_state_wait_s", aerial_home_state_wait_s_);
  if (!std::isfinite(aerial_home_state_wait_s_) ||
      aerial_home_state_wait_s_ < 0.0) {
    throw std::invalid_argument(
        "aerial_home_state_wait_s must be finite and non-negative");
  }
  // The global graph lives only in memory: a restarted planner, or a robot
  // driven far off its graph, is left with a graph that cannot reach where
  // the robot is or home (run 5, 2026-09-25). It is rebuilt from the
  // robot's keyframe trajectory: on start and whenever it holds only its
  // seed, and when neither the robot's pose nor an exploration path links
  // to it. The trajectory is the MOLA backend's graph solution, in the
  // robot's peer root beside the products the map is read from, whose last
  // path component is the robot's id there.
  roadmap_rebuild_min_interval_s_ = std::max(
      0.0, declareOrGet<double>(this, "roadmap_rebuild.min_interval_s",
                                roadmap_rebuild_min_interval_s_));
  roadmap_rebuild_params_.max_offset = std::clamp(
      declareOrGet<double>(this, "roadmap_rebuild.max_offset_m",
                           roadmap_rebuild_params_.max_offset),
      0.0, 2.0);
  roadmap_rebuild_params_.link_radius = std::max(
      0.0, declareOrGet<double>(this, "roadmap_rebuild.link_radius_m",
                                roadmap_rebuild_params_.link_radius));
  const bool rebuild_enabled =
      declareOrGet<bool>(this, "roadmap_rebuild.enable", true);
  std::string rebuild_off;
  if (!rebuild_enabled) {
    rebuild_off = "roadmap_rebuild.enable is false";
  } else if (mola_map_ == nullptr) {
    rebuild_off = "the map backend '" + map_backend_ + "' has no keyframes";
  } else {
    // A trailing separator would make the robot id the empty last
    // component.
    std::filesystem::path peer_root =
        std::filesystem::path(get_parameter("map.mola.peer_root").as_string())
            .lexically_normal();
    if (!peer_root.empty() && !peer_root.has_filename()) {
      peer_root = peer_root.parent_path();
    }
    const std::string robot = declareOrGet<std::string>(
        this, "roadmap_rebuild.robot_id", peer_root.filename().string());
    // Derived from the peer root when not given. A deployment whose peer
    // root is a product directory inside the robot's root (SwarmDeck's
    // <peer>/planning) gives both: derived, they would name the directory,
    // not the robot, and a file the bridge never writes.
    const std::string graph_solution = declareOrGet<std::string>(
        this, "roadmap_rebuild.graph_solution",
        (peer_root / "graph_solution.json").string());
    if (peer_root.empty()) {
      rebuild_off = "map.mola.peer_root is empty";
    } else if (robot.empty()) {
      rebuild_off = "no robot id (roadmap_rebuild.robot_id) for the peer "
                    "root " + peer_root.string();
    } else {
      keyframe_source_ = std::make_unique<GraphSolutionFile>(
          graph_solution, robot, std::size_t{64} * 1024 * 1024);
      RCLCPP_INFO(get_logger(),
                  "global graph rebuilds read %s's keyframes from %s",
                  robot.c_str(), graph_solution.c_str());
    }
  }
  if (keyframe_source_ == nullptr) {
    RCLCPP_INFO(get_logger(),
                "global graph rebuilds from the keyframes are off: %s",
                rebuild_off.c_str());
  }

  const double publish_period =
      declareOrGet<double>(this, "graph_publish_period_sec", 2.0);
  planner_config_state_pub_ = create_publisher<mgg_msgs::msg::PlannerConfigState>(
      "planner_config_state", rclcpp::QoS(1).reliable().transient_local());
  {
    const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
    publishPlannerConfigState();
  }

  // Advertise planner services only after the initial configuration is latched.
  if (fleet_) {
    release_claims_srv_ = create_service<mgg_msgs::srv::ReleaseClaims>(
        "release_claims",
        [this](const std::shared_ptr<mgg_msgs::srv::ReleaseClaims::Request> req,
               std::shared_ptr<mgg_msgs::srv::ReleaseClaims::Response> res) {
          onReleaseClaims(req, res);
        },
        rclcpp::ServicesQoS(), callback_group_);
  }
  cancel_srv_ = create_service<std_srvs::srv::Trigger>(
      "cancel_planning",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        cancelPlanning();
        response->success = true;
        response->message = "cancellation requested";
      }, rclcpp::ServicesQoS(), input_callback_group_);
  cancel_exploration_srv_ = create_service<std_srvs::srv::Trigger>(
      "cancel_exploration_planning",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        cancelExplorationPlanning();
        response->success = true;
        response->message = "exploration cancellation requested";
      }, rclcpp::ServicesQoS(), input_callback_group_);
  build_srv_ = create_service<std_srvs::srv::Trigger>(
      "build_local_graph",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        onBuildRequest(req, res);
      },
      rclcpp::ServicesQoS(), callback_group_);
  plan_srv_ = create_service<mgg_msgs::srv::PlannerSrv>(
      "mggplanner",
      [this](const std::shared_ptr<mgg_msgs::srv::PlannerSrv::Request> req,
             std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> res) {
        onPlanRequest(req, res);
      },
      rclcpp::ServicesQoS(), callback_group_);
  objective_srv_ = create_service<mgg_msgs::srv::PlanObjective>(
      "plan_objective",
      [this](const std::shared_ptr<mgg_msgs::srv::PlanObjective::Request> req,
             std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> res) {
        onObjectiveRequest(req, res);
      },
      rclcpp::ServicesQoS(), callback_group_);
  exploration_target_srv_ =
      create_service<mgg_msgs::srv::PlannerSetExplorationTarget>(
          "set_exploration_target",
          [this](const std::shared_ptr<
                     mgg_msgs::srv::PlannerSetExplorationTarget::Request>
                     req,
                 std::shared_ptr<
                     mgg_msgs::srv::PlannerSetExplorationTarget::Response>
                     res) { onExplorationTargetRequest(req, res); },
          rclcpp::ServicesQoS(), callback_group_);

  exploration_region_srv_ =
      create_service<mgg_msgs::srv::PlannerSetExplorationRegion>(
          "set_exploration_region",
          [this](const std::shared_ptr<
                     mgg_msgs::srv::PlannerSetExplorationRegion::Request>
                     req,
                 std::shared_ptr<
                     mgg_msgs::srv::PlannerSetExplorationRegion::Response>
                     res) { onExplorationRegionRequest(req, res); },
          rclcpp::ServicesQoS(), callback_group_);

  leave_fleet_srv_ = create_service<std_srvs::srv::SetBool>(
      "leave_fleet",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        onLeaveFleet(req, res);
      },
      rclcpp::ServicesQoS(), callback_group_);

  // Node::create_timer drives off get_clock(), the node's RCL_ROS_TIME clock.
  // That is correct in both deployments: on a real robot use_sim_time is false
  // and ROS time follows the system clock, while in simulation it follows
  // /clock. create_wall_timer would be wrong in the second case, running at
  // wall rate while the ARGoS bridge steps simulated time at its own pace.
  graph_timer_ = create_timer(
      std::chrono::duration<double>(publish_period),
      [this]() {
        std::unique_lock<std::recursive_mutex> lock(planner_mutex_, std::try_to_lock);
        if (!lock.owns_lock()) return;
        publishOwnGraph();
        publishMarkers();
      }, callback_group_);
  global_graph_update_timer_ = create_timer(
      std::chrono::duration<double>(mgg::kGlobalGraphUpdateTimerPeriod),
      [this]() { expandGlobalGraphTimerCallback(); }, callback_group_);

  // The one way that choice bites: use_sim_time true with nothing publishing
  // /clock leaves ROS time pinned at zero, so the timer never fires and the
  // node looks alive but silent. Deliberately a wall timer, since it has to
  // fire while ROS time is frozen. One shot.
  if (get_parameter("use_sim_time").as_bool()) {
    sim_time_check_ = create_wall_timer(std::chrono::seconds(5), [this]() {
      sim_time_check_->cancel();
      if (now().nanoseconds() == 0) {
        RCLCPP_WARN(get_logger(),
                    "use_sim_time is set but /clock has not been seen: ROS "
                    "time is still zero, so timers will never fire. Start the "
                    "simulation's clock publisher, or unset use_sim_time when "
                    "running on a robot.");
      }
    });
  }

  RCLCPP_INFO(get_logger(),
              "mggplanner ready: robot %u, frame '%s', grid %.2f x %.2f x %.2f",
              planning_params_.robot_id, world_frame_.c_str(),
              grid_params_.resolution.x(), grid_params_.resolution.y(),
              grid_params_.resolution.z());
}

void PlannerNode::addInputCallbackGroupsTo(rclcpp::Executor& executor) {
  executor.add_callback_group(input_callback_group_, get_node_base_interface());
  if (snapshot_callback_group_)
    executor.add_callback_group(snapshot_callback_group_, get_node_base_interface());
}

void PlannerNode::loadParameters() {
  ParamLoader p(this);
  if (!loadRobotParams(p, "RobotParams", robot_params_)) {
    throw std::invalid_argument(
        "RobotParams failed to load: physical_size and physical_center_offset "
        "must be set together, with positive finite dimensions and finite offsets; "
        "type and bound_mode must also be valid");
  }
  nominal_bound_mode_ = robot_params_.bound_mode;
  const bool planning_loaded =
      loadPlanningParams(p, "PlanningParams", planning_params_);
  // Every bid is costed in time at v_max (drone scout Task 17): a planner
  // without a speed would bid what its peers refuse. It does not start.
  if (!std::isfinite(planning_params_.v_max) ||
      !(planning_params_.v_max > 0.0)) {
    throw std::invalid_argument(
        "PlanningParams.v_max must be a positive, finite speed (m/s); got " +
        std::to_string(planning_params_.v_max));
  }
  if (!planning_loaded) {
    RCLCPP_ERROR(get_logger(), "PlanningParams failed to load");
  }
  if (!loadGridGraphParams(p, "BoundedSpaceParams/GridGraphLocal",
                           grid_params_)) {
    RCLCPP_WARN(get_logger(),
                "GridGraphLocal/resolution missing; using the default. If this "
                "config came from ROS 1, run tools/convert_config.py, which "
                "renames min_extension to resolution.");
  }
  if (!loadBoundedSpace(p, "BoundedSpaceParams/Global", global_space_)) {
    RCLCPP_WARN(get_logger(), "BoundedSpaceParams/Global missing");
  }
  if (!loadSensorSet(p, "SensorParams", sensors_)) {
    RCLCPP_WARN(get_logger(), "no sensors loaded from SensorParams");
  }
  loadTourParams(p, "tour", tour_params_);
  loadFleetParams(p, "fleet", fleet_params_);
  world_frame_ = planning_params_.global_frame_id;
  communication_range_ =
      declareOrGet<double>(this, "communication_range", 15.0);

  if (!p.missing().empty()) {
    RCLCPP_INFO(get_logger(),
                "%zu parameter(s) absent, defaults used (first: '%s')",
                p.missing().size(), p.missing().front().c_str());
  }
}

mgg::GainContext PlannerNode::makeGainContext() {
  mgg::GainContext ctx;
  ctx.map = map_.get();
  ctx.planning = &planning_params_;
  ctx.robot = &robot_params_;
  ctx.global_space = &global_space_;
  ctx.no_gain_zones = no_gain_zones_.empty() ? nullptr : &no_gain_zones_;
  ctx.sensors = &sensors_;
  return ctx;
}

mgg::ExpandContext PlannerNode::makeContext(bool include_own_body) {
  mgg::ExpandContext ctx;
  ctx.inclinations = &edge_inclinations_;
  ctx.map = map_.get();
  ctx.planning = &planning_params_;
  ctx.robot = &robot_params_;
  ctx.ground = ground_.get();
  ctx.geofence = geofence_.get();
  ctx.projected_graph = nullptr;  // visualisation only; not built here
  ctx.robot_id = static_cast<int>(planning_params_.robot_id);
  ctx.robot_box_size = robot_params_.getPlanningSize();
  ctx.allow_unknown_lattice_body = allow_unknown_lattice_body_;
  if (robot_params_.type == mgg::RobotType::kGroundRobot &&
      unknown_body_policy_ == "above_sensor_fov") {
    const auto sensor = sensors_.find(unknown_body_sensor_);
    if (sensor != sensors_.end()) {
      const auto& lidar = sensor->second;
      // Upright lidar geometry only. Missing/invalid configuration fails
      // closed to the strict body policy; never invent a sensor height/FOV.
      if (lidar.type == mgg::SensorType::kLidar && std::isfinite(lidar.mount_height) &&
          lidar.mount_height > std::max(0.0, planning_params_.max_ground_height +
              robot_params_.center_offset.z() - ctx.robot_box_size.z()/2) + 1e-9 &&
          std::isfinite(lidar.fov.y()) &&
          lidar.fov.y() > 0 && lidar.fov.y() < M_PI &&
          lidar.rotations.tail<2>().norm() < 1e-9) {
        ctx.unknown_body_above_center = lidar.mount_height -
            planning_params_.max_ground_height - robot_params_.center_offset.z();
      }
    }
  }
  if (include_own_body && robot_params_.physical_size &&
      robot_params_.physical_center_offset &&
      (ctx.unknown_body_above_center || allow_unknown_lattice_body_) &&
      unknown_body_policy_ != "strict" &&
      robot_params_.type == mgg::RobotType::kGroundRobot &&
      have_odometry_ && map_ && ground_ && map_->getStatus()) {
    const auto root = physicalAnchorAtDrivingHeight(current_state_);
    ctx.standing_body = mgg::OrientedBox{
        root.head<3>() + robot_params_.physicalOffsetForHeading(root[3]),
        root[3], robot_params_.physicalSize()};
    ctx.own_body_known_free = ownBodyKnownFree();
  }
  ctx.hanging_root_edge_length_max = hanging_root_edge_length_max_;
  // A hanging root sits at the physical driving height by construction;
  // projecting it again onto absent ground would only fail.
  ctx.preserve_hanging_root_start_height = hanging_root_edge_length_max_ > 0.0;
  ctx.root_footprint_exempt = true;
  ctx.root_is_robot = true;
  ctx.deadline = lattice_deadline_;
  if (peer_diagnosis_deadline_ && (!ctx.deadline || *peer_diagnosis_deadline_ < *ctx.deadline))
    ctx.deadline = peer_diagnosis_deadline_;
  // No-go zones close lattice edges on every backend, the robot's own
  // departure from one it stands in excepted (review r0, I-5).
  if (!no_go_.empty()) {
    ctx.projected_edge_admissible =
        [this](const std::vector<Eigen::Vector3d>& edge) {
          for (std::size_t i = 1; i < edge.size(); ++i) {
            if (noGoBlocksSegment(edge[i - 1], edge[i])) return false;
          }
          return true;
        };
  }
  return ctx;
}

std::shared_ptr<const mgg::KnownFreeBodyVolumes> PlannerNode::ownBodyKnownFree() {
  if (standing_start_scope_depth_ && plan_own_body_known_free_)
    return plan_own_body_known_free_;
  auto known = std::make_shared<mgg::KnownFreeBodyVolumes>();
  const auto root = physicalAnchorAtDrivingHeight(current_state_);
  known->add(*map_, {root.head<3>() + robot_params_.physicalOffsetForHeading(root[3]),
                    root[3], robot_params_.physicalSize()});
  // Each request reads one parser-validated session into a fresh volume.
  // Never concatenate poses with a prior read/session, even at equal epoch.
  KeyframeTrajectory trajectory;
  std::string error;
  if (keyframe_source_ && have_mapping_snapshot_ &&
      readOwnKeyframes(trajectory, error) &&
      trajectory.component_id == mapping_snapshot_.component_id &&
      trajectory.epoch == mapping_snapshot_.epoch) {
    const auto transform = navigationFromComponent();
    std::vector<mgg::StateVec> poses;
    // Only the last 20 metres need projection; addTrajectory bounds the
    // final segment exactly. Never connect the current pose to a stale tail.
    double distance = 0;
    for (std::size_t i = trajectory.poses.size(); i > 0; --i) {
      const auto pose = transform * trajectory.poses[i-1];
      if (i < trajectory.poses.size()) {
        const double segment = (trajectory.poses[i].translation() -
                                trajectory.poses[i-1].translation()).norm();
        if (segment > 1.0 + 1e-9) break;
        distance += segment;
      }
      mgg::StateVec state(pose.translation().x(), pose.translation().y(),
          pose.translation().z(), std::atan2(pose.linear()(1,0), pose.linear()(0,0)));
      poses.push_back(physicalAnchorAtDrivingHeight(state));
      if (distance >= 20) break;
    }
    std::reverse(poses.begin(), poses.end());
    known->addTrajectory(*map_, robot_params_, poses);
  }
  if (standing_start_scope_depth_) plan_own_body_known_free_ = known;
  return known;
}

mgg::ExpandContext PlannerNode::makeGlobalContext() {
  mgg::ExpandContext ctx = makeContext(false);
  ctx.unknown_body_above_center.reset();
  ctx.standing_body.reset();
  // Inclinations are keyed by local lattice ids; the roadmap has its own.
  ctx.inclinations = nullptr;
  // A roadmap edge must have been seen traversable (rrg.cpp:725).
  ctx.stop_at_unknown = true;
  // Vertex zero of the roadmap is home, not the robot.
  ctx.root_footprint_exempt = false;
  ctx.root_is_robot = false;
  // The roadmap keeps edges through a zone, which its searches leave out
  // while the zone stands (setEdgeBlocked): one refused here would be lost
  // for good.
  ctx.projected_edge_admissible = nullptr;
  return ctx;
}

mgg::Vertex* PlannerNode::findGlobalVertex(int id) const {
  const auto it = global_graph_->vertices_map_.find(id);
  return it == global_graph_->vertices_map_.end() ? nullptr : it->second;
}

mgg::Vertex* PlannerNode::linkRobotToGlobalGraph() {
  mgg::StateVec current = current_state_;
  if (!projectToDrivingHeight(current)) {
    current = physicalAnchorAtDrivingHeight(current_state_);
  }
  const int before = global_graph_->getNumVertices();
  const int edges_before = global_graph_->getNumEdges();
  mgg::Vertex* link = mgg::linkDeparture(*global_graph_, current,
                                         makeGlobalContext(), kLinkRadius)
                          .vertex;
  // A pose that reuses a vertex may still gain edges (connectStateToGraph).
  if (global_graph_->getNumVertices() != before ||
      global_graph_->getNumEdges() != edges_before) {
    ++graph_revision_;
  }
  return link;
}

void PlannerNode::setTourClusterAside(const mgg::FrontierCluster& cluster,
                                      double retry_s, bool at_target) {
  if (at_target) {
    auto& failure = tour_at_target_failures_[cluster.id];
    failure.retry_multiplier =
        std::min(4, std::max(1, 2 * failure.retry_multiplier));
    failure.gain = cluster.gain;
    retry_s *= failure.retry_multiplier;
  }
  tour_set_aside_[cluster.id] = TourSetAside{
      current_state_.head<3>(), cluster.gain, now().seconds(), retry_s,
      at_target};
  tour_planner_->releaseTarget();
}

void PlannerNode::noteGlobalGraphEdges() {
  const int edges = global_graph_->getNumEdges();
  if (edges == tour_graph_edges_) return;
  ++graph_revision_;
  tour_graph_edges_ = edges;
}

void PlannerNode::demoteFleetCoveredFrontiers() {
  // The walk to a peer's visited vertex may start with a link checked
  // against this robot's map, clear of what it has seen occupied. Only a
  // map in service can say: one without its snapshot knows nothing
  // occupied, and a demotion is for good (review r2, I-1). Without one,
  // the roadmap alone.
  auto map_read = mapReadLease();
  const mgg::ExpandContext ctx = makeGlobalContext();
  const bool map_serves = ctx.map != nullptr && ctx.map->getStatus();
  // A pass makes at most fleet_coverage_max_link_checks swept checks
  // (review r2, M-1), starting where the last one stopped (review r3, P2).
  // Each is swept with the nominal box, whatever this request's bound mode
  // (review r3, P1), and nothing of it is kept.
  mgg::RobotParams nominal = robot_params_;
  nominal.bound_mode = nominal_bound_mode_;
  mgg::FleetCoverageLinks links{&ctx, &fleet_coverage_cursor_,
                                nominal.getPlanningSize(),
                                fleet_coverage_max_link_checks_,
                                fleet_coverage_max_links_per_frontier_};
  const int demoted = mgg::demoteFleetCoveredFrontiers(
      *global_graph_, static_cast<int>(planning_params_.robot_id),
      fleet_coverage_radius_m_, map_serves ? &links : nullptr);
  if (links.deferred > 0) {
    RCLCPP_INFO(get_logger(),
                "fleet coverage: %d link check(s), the pass's budget, made; "
                "%d frontier(s) left for the next pass",
                links.checks, links.deferred);
  }
  if (demoted > 0) {
    RCLCPP_INFO(get_logger(),
                "%d frontier(s) covered by the fleet: a peer's visited vertex "
                "lies within %.1f m, joined by a walk of %.1f m",
                demoted, fleet_coverage_radius_m_,
                mgg::kFleetCoveragePathFactor * fleet_coverage_radius_m_);
  }
}

std::vector<mgg::FrontierCluster> PlannerNode::liftedPeerFrontiers() {
  auto map_read = mapReadLease();
  if (lifted_target_graph_.lock() != global_graph_) {
    lifted_target_vertices_.clear();
    lifted_targets_.clear();
    lifted_target_graph_ = global_graph_;
  }
  std::vector<mgg::FrontierCluster> candidates;
  AerialCounters& counts = aerial_counters_;
  counts.lifted_proposed = counts.lifted_rejected_gain = counts.lifted_rejected_cap =
      counts.lifted_rejected_merged = counts.lifted_rejected_no_anchor =
          counts.lifted_rejected_link = counts.lifted_rejected_scouting =
              counts.lifted_admitted = counts.lifted_senders_unplaced = 0;
  refreshScoutingExclusions();
  const double height = std::clamp(aerial_frontier_height_m_, aerial_min_height_m_, aerial_max_height_m_);
  for (const auto& [sender, snapshot] : neighbour_roadmaps_) {
    const auto frame = neighbour_frames_.find(sender);
    Eigen::Isometry3d transform;
    if (frame == neighbour_frames_.end() ||
        !refreshNeighbourTransform(sender, frame->second) ||
        !poses_->getRobotTransform(sender, transform)) {
      ++counts.lifted_senders_unplaced;
      continue;
    }
    for (const auto& v : snapshot.vertices) {
      if (!v.is_frontier || v.visited || !v.state.allFinite()) continue;
      ++counts.lifted_proposed;
      mgg::FrontierCluster c;
      c.owner_robot_id = sender;
      c.position = transform * v.state.head<3>();
      c.position.z() += height;
      c.gain = std::max(0, v.num_unknown_voxels) * planning_params_.unknown_voxel_gain +
          std::max(0, v.num_free_voxels) * planning_params_.free_voxel_gain +
          std::max(0, v.num_occupied_voxels) * planning_params_.occupied_voxel_gain;
      if (c.gain < tour_params_.min_cluster_gain) {
        ++counts.lifted_rejected_gain;
        continue;
      }
      if (scoutingExcludes(c.position)) {
        ++counts.lifted_rejected_scouting;
        continue;
      }
      c.id = mgg::makeClusterId(sender, c.position, tour_params_.cluster_id_cell_m);
      candidates.push_back(c);
    }
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
    return a.gain != b.gain ? a.gain > b.gain : a.id < b.id;
  });
  std::vector<LiftedTarget> targets;
  const auto box = robot_params_.getPlanningSize();
  for (auto c : candidates) {
    if (targets.size() == 64) {
      ++counts.lifted_rejected_cap;
      continue;
    }
    if (std::any_of(targets.begin(), targets.end(), [&](const auto& other) {
          return (c.position - other.cluster.position).norm() <= fleet_params_.cluster_merge_radius_m;
        })) {
      ++counts.lifted_rejected_merged;
      continue;
    }
    mgg::StateVec state(c.position.x(), c.position.y(), c.position.z(), 0);
    std::vector<mgg::Vertex*> anchors;
    global_graph_->getNearestVertices(&state, 5.0, &anchors);
    mgg::Vertex* anchor = nullptr;
    double best = std::numeric_limits<double>::infinity();
    bool own_anchor_near = false;
    for (auto* v : anchors) {
      if (!v || v->lifted_peer_target || !global_graph_->inService(*v) ||
          v->robot_id != static_cast<int>(planning_params_.robot_id)) continue;
      own_anchor_near = true;
      const double distance = (v->state.head<3>() - c.position).norm();
      if (distance >= best || map_->getStaticStrictPathStatus(
          v->state.head<3>() + robot_params_.center_offset,
          c.position + robot_params_.center_offset, box) != mgg::VoxelStatus::kFree) continue;
      best = distance;
      anchor = v;
    }
    if (anchor) {
      targets.push_back({c, anchor->id, best});
    } else if (own_anchor_near) {
      ++counts.lifted_rejected_link;
    } else {
      ++counts.lifted_rejected_no_anchor;
    }
  }
  counts.lifted_admitted = static_cast<int>(targets.size());
  const bool changed = targets.size() != lifted_targets_.size() ||
      !std::equal(targets.begin(), targets.end(), lifted_targets_.begin(),
          [](const auto& a, const auto& b) {
            return a.cluster.id == b.cluster.id &&
                a.cluster.owner_robot_id == b.cluster.owner_robot_id &&
                a.cluster.gain == b.cluster.gain &&
                a.cluster.position == b.cluster.position &&
                a.anchor == b.anchor && a.length == b.length;
          });
  if (changed) {
    withdrawLiftedTargets();
    for (std::size_t i = 0; i < targets.size(); ++i) {
      auto& target = targets[i];
      auto& c = target.cluster;
      const mgg::StateVec state(c.position.x(), c.position.y(), c.position.z(), 0);
      mgg::Vertex* endpoint = nullptr;
      if (i < lifted_target_vertices_.size()) {
        endpoint = findGlobalVertex(lifted_target_vertices_[i]);
        endpoint->state = state; // never in the nearest-neighbour index
      } else {
        endpoint = new mgg::Vertex(global_graph_->generateVertexID(), state);
        endpoint->robot_id = static_cast<int>(planning_params_.robot_id);
        endpoint->lifted_peer_target = true;
        global_graph_->addVertex(endpoint);
        lifted_target_vertices_.push_back(endpoint->id);
      }
      global_graph_->addEdge(findGlobalVertex(target.anchor), endpoint, target.length);
      c.representative_vertex_id = endpoint->id;
      c.member_vertex_ids = {endpoint->id};
    }
    lifted_targets_ = std::move(targets);
    ++graph_revision_;
  }
  std::vector<mgg::FrontierCluster> clusters;
  for (const auto& target : lifted_targets_) clusters.push_back(target.cluster);
  return clusters;
}

void PlannerNode::withdrawLiftedTargets() {
  // Remove both Boost edges and adjacency used by non-Dijkstra searches.
  // The pool remains bounded across withdrawal and re-admission.
  for (const int id : lifted_target_vertices_) {
    auto* v = findGlobalVertex(id);
    if (!v) continue;
    if (current_global_vertex_id_ == id) global_exploration_ongoing_ = false;
    const auto neighbours = global_graph_->edge_map_[id];
    for (const auto& [other, cost] : neighbours) {
      (void)cost;
      if (auto* u = findGlobalVertex(other)) global_graph_->removeEdge(v, u);
      auto& edges = global_graph_->edge_map_[other];
      edges.erase(std::remove_if(edges.begin(), edges.end(),
          [id](const auto& edge) { return edge.first == id; }), edges.end());
    }
    global_graph_->edge_map_[id].clear();
    v->type = mgg::VertexType::kUnvisited;
    v->vol_gain = mgg::VolumetricGain();
  }
  lifted_targets_.clear();
}

std::vector<mgg::FrontierCluster> PlannerNode::globalFrontierClusters() {
  demoteFleetCoveredFrontiers();
  // Counts and the frontier flag are the owner's latest map evidence.
  // Never replace them with this robot's unknown view of a peer's space.
  const auto score = globalFrontierGain();
  for (const auto& [id, vertex] : global_graph_->vertices_map_) {
    if (vertex && vertex->robot_id != static_cast<int>(planning_params_.robot_id)) {
      score(*vertex);
    }
  }
  std::vector<mgg::FrontierCluster> clusters = mgg::extractFrontierClusters(
      *global_graph_, fleet_params_.cluster_merge_radius_m,
      tour_params_.min_cluster_gain, tour_params_.cluster_id_cell_m,
      [this](const mgg::Vertex& v) {
        return !v.lifted_peer_target &&
               (v.robot_id == static_cast<int>(planning_params_.robot_id) ||
                (v.vol_gain.is_frontier && v.vol_gain.gain > 0.0));
      });
  if (robot_params_.type == mgg::RobotType::kAerialRobot) {
    auto peers = liftedPeerFrontiers();
    clusters.insert(clusters.end(), peers.begin(), peers.end());
  }
  cluster_ids_.stabilize(clusters, fleet_params_.cluster_merge_radius_m);
  return clusters;
}

mgg::FrontierCluster PlannerNode::tourValueCluster(const mgg::FrontierCluster& cluster) const {
  auto value_cluster = cluster;
  const auto* vertex = findGlobalVertex(cluster.representative_vertex_id);
  if (!fleet_ && robot_params_.type == mgg::RobotType::kAerialRobot &&
      vertex && vertex->lifted_peer_target) {
    value_cluster.owner_robot_id = static_cast<int>(planning_params_.robot_id);
  }
  return value_cluster;
}

int PlannerNode::capTourValues(mgg::TourCostMatrix& costs,
                               const std::vector<mgg::FrontierCluster>& clusters) const {
  auto value_clusters = clusters;
  for (auto& cluster : value_clusters) cluster = tourValueCluster(cluster);
  return mgg::capTourCostsByValue(costs, value_clusters, tour_params_.min_cluster_gain,
      static_cast<int>(planning_params_.robot_id), mgg::kGlobalOtherRobotPenalty);
}

bool PlannerNode::aerialLocalFrontiers() const {
  return std::any_of(local_graph_->vertices_map_.begin(), local_graph_->vertices_map_.end(),
      [](const auto& entry) {
        return entry.second && entry.second->type == mgg::VertexType::kFrontier;
      });
}

double PlannerNode::aerialGraphProgress(const Eigen::Vector3d& position) {
  auto map_read = mapReadLease();
  const auto* report = tour_distances_.from(*global_graph_, graph_revision_,
                                          kHomeVertexId, peer_generation_);
  if (!report || !position.allFinite() || !map_->getStatus()) return mgg::kUnreachableCost;
  mgg::StateVec state(position.x(), position.y(), position.z(), 0);
  std::vector<mgg::Vertex*> anchors;
  global_graph_->getNearestVertices(&state, 5.0, &anchors);
  double nearest = mgg::kUnreachableCost;
  double progress = mgg::kUnreachableCost;
  for (const auto* v : anchors) {
    if (!v || v->lifted_peer_target || !global_graph_->inService(*v) ||
        v->robot_id != static_cast<int>(planning_params_.robot_id)) continue;
    const double home = mgg::reachedDistance(*report, v->id);
    const double link = (v->state.head<3>() - position).norm();
    if (!std::isfinite(home) || link > nearest + 1e-6) continue;
    if (map_->getStaticStrictPathStatus(v->state.head<3>() + robot_params_.center_offset,
          position + robot_params_.center_offset, robot_params_.getPlanningSize()) !=
        mgg::VoxelStatus::kFree) continue;
    // Nearest certified anchor, not a straight-line distance from home.
    // Ties take the smaller progress, conservatively at branches/loops.
    if (link < nearest - 1e-6) progress = mgg::kUnreachableCost;
    nearest = link;
    progress = std::min(progress, home + link);
  }
  return progress;
}

double PlannerNode::aerialPeerProgress(int sender) {
  const auto received = aerial_peer_received_.find(sender);
  const auto snapshot = neighbour_roadmaps_.find(sender);
  const auto frame = neighbour_frames_.find(sender);
  Eigen::Isometry3d transform;
  if (received == aerial_peer_received_.end() ||
      secondsSince(received->second) > neighbour_transform_ttl_s_ ||
      snapshot == neighbour_roadmaps_.end() || frame == neighbour_frames_.end() ||
      !refreshNeighbourTransform(sender, frame->second) ||
      !poses_->getRobotTransform(sender, transform)) return mgg::kUnreachableCost;
  // Frontier vertices can be appended after trajectory vertices. Only the
  // latest visited vertex is evidence of peer progress, not vertices.back().
  const mgg::GraphExchangeVertex* latest = nullptr;
  for (const auto& v : snapshot->second.vertices) {
    if (v.visited && v.state.allFinite() && (!latest || v.id > latest->id)) latest = &v;
  }
  if (!latest) return mgg::kUnreachableCost;
  Eigen::Vector3d position = transform * latest->state.head<3>();
  position.z() += std::clamp(aerial_frontier_height_m_, aerial_min_height_m_, aerial_max_height_m_);
  return aerialGraphProgress(position);
}

double PlannerNode::aerialFleetFront() {
  double front = -1.0;
  aerial_front_peer_ = -1;
  aerial_front_is_lower_bound_ = false;
  for (const auto& [sender, snapshot] : neighbour_roadmaps_) {
    (void)snapshot;
    const double progress = aerialPeerProgress(sender);
    if (!std::isfinite(progress)) {
      aerial_front_is_lower_bound_ = true;
    } else if (progress > front ||
               (progress == front && sender < aerial_front_peer_)) {
      front = progress;
      aerial_front_peer_ = sender;
    }
  }
  return front < 0.0 ? mgg::kUnreachableCost : front;
}

void PlannerNode::biasAerialTourCosts(mgg::TourCostMatrix& costs,
                                     const std::vector<mgg::FrontierCluster>& clusters) {
  if (fleet_ || robot_params_.type != mgg::RobotType::kAerialRobot ||
      !std::isfinite(aerial_front_m_)) return;
  const auto* home = tour_distances_.from(*global_graph_, graph_revision_,
                                        kHomeVertexId, peer_generation_);
  if (!home) return;
  // At most five metres of first-leg preference, never a feasibility or
  // gain change. A nearby own frontier may still beat a distant scout target.
  constexpr double kFrontBiasM = 5.0;
  for (std::size_t i = 0; i < clusters.size(); ++i) {
    const double progress = mgg::reachedDistance(*home, clusters[i].representative_vertex_id);
    if (std::isfinite(progress) && std::isfinite(costs.from_robot[i])) {
      costs.from_robot[i] += std::clamp(aerial_front_m_ + kFrontBiasM - progress, 0.0, kFrontBiasM);
    }
  }
}

std::vector<mgg::FrontierCluster> PlannerNode::tourCandidates(
    std::vector<mgg::FrontierCluster> clusters) {
  aerial_fallback_remains_ = false;
  aerial_unknown_progress_clusters_.clear();
  refreshScoutingExclusions();
  if (!scouting_zones_.empty()) {
    const std::size_t before = clusters.size();
    clusters.erase(std::remove_if(clusters.begin(), clusters.end(),
                                  [this](const mgg::FrontierCluster& cluster) {
                                    return scoutingExcludes(cluster.position);
                                  }),
                   clusters.end());
    scouting_counters_.targets_refused += before - clusters.size();
  }
  const std::vector<Eigen::Vector3d> reserved = selectionExclusions();
  clusters.erase(
      std::remove_if(clusters.begin(), clusters.end(),
                     [&](const mgg::FrontierCluster& cluster) {
                       return std::any_of(
                           reserved.begin(), reserved.end(),
                           [&](const Eigen::Vector3d& point) {
                             return (cluster.position - point).norm() <=
                                    reservation_exclusion_radius_m_;
                           });
                     }),
      clusters.end());
  const int own_id = static_cast<int>(planning_params_.robot_id);
  if (!fleet_) {
    if (robot_params_.type == mgg::RobotType::kAerialRobot && !clusters.empty()) {
      const auto* link = linkRobotToGlobalGraph();
      if (!link) {
        RCLCPP_INFO(get_logger(), "aerial tour: own clusters unreachable: robot off graph");
        return {};
      }
      refreshPeerGeneration();
      relinkAerialHome(link->id);
      auto costs = mgg::computeTourCosts(*global_graph_, graph_revision_, tour_distances_,
          link->id, current_state_[3], clusters, tour_params_.heading_weight, peer_generation_);
      const auto before_cap = costs.from_robot;
      const std::vector<double> back = homeDistances(clusters);
      mgg::capTourCostsByReach(costs, back, flight_reach_m_);
      const auto before_value_cap = costs.from_robot;
      capTourValues(costs, clusters);
      std::vector<mgg::FrontierCluster> reachable;
      for (std::size_t i = 0; i < clusters.size(); ++i) {
        const bool reach_capped =
            std::isfinite(before_cap[i]) && !std::isfinite(before_value_cap[i]);
        const std::string reach_note =
            reach_capped ? noteReachCapReject(costs.distance_from_robot[i], back[i])
                         : std::string();
        if (std::isfinite(costs.from_robot[i])) reachable.push_back(clusters[i]);
        else if (clusters[i].owner_robot_id == own_id) {
          RCLCPP_INFO(get_logger(), "aerial tour: own cluster %016llx unreachable: %s",
              static_cast<unsigned long long>(clusters[i].id),
              std::isfinite(before_value_cap[i]) ? "below distance-discounted value floor" :
              reach_capped ? reach_note.c_str() :
              findGlobalVertex(clusters[i].representative_vertex_id) ?
                  "disconnected or search-time blocked" : "representative off graph");
        }
      }
      clusters = std::move(reachable);
    }
    const auto others = [own_id](const mgg::FrontierCluster& cluster) {
      return cluster.owner_robot_id != own_id;
    };
    const bool aerial = robot_params_.type == mgg::RobotType::kAerialRobot;
    if (!std::all_of(clusters.begin(), clusters.end(), others) ||
        (aerial && aerialLocalFrontiers())) {
      if (aerial) aerial_counters_.gate_own_or_local +=
          std::count_if(clusters.begin(), clusters.end(), others);
      clusters.erase(std::remove_if(clusters.begin(), clusters.end(), others),
                     clusters.end());
    } else if (aerial && !clusters.empty()) {
      const double drone = aerialGraphProgress(current_state_.head<3>());
      const auto* home = tour_distances_.from(*global_graph_, graph_revision_,
                                            kHomeVertexId, peer_generation_);
      clusters.erase(std::remove_if(clusters.begin(), clusters.end(), [&](const auto& c) {
        const double peer = aerialPeerProgress(c.owner_robot_id);
        const double target = home ? mgg::reachedDistance(*home, c.representative_vertex_id)
                                   : mgg::kUnreachableCost;
        const bool behind_peer = std::isfinite(target) && std::isfinite(peer) &&
                                 target + 1e-6 < peer;
        const bool behind_drone = std::isfinite(target) && std::isfinite(drone) &&
                                  target + 1e-6 < drone;
        aerial_counters_.gate_behind_peer += behind_peer;
        aerial_counters_.gate_behind_drone += behind_drone;
        if (behind_peer || behind_drone) return true;
        if (!std::isfinite(drone) || !std::isfinite(peer) || !std::isfinite(target)) {
          aerial_unknown_progress_clusters_.insert(c.id);
          ++aerial_counters_.gate_unknown_progress;
        } else {
          ++aerial_counters_.gate_known_forward;
        }
        return false;
      }), clusters.end());
    }
    clusters = insideExplorationRegion(std::move(clusters));
    aerial_fallback_remains_ = aerial &&
        std::any_of(clusters.begin(), clusters.end(), others);
    return clusters;
  }
  // Tour-exploration design §3.5: never a cluster another robot holds or a
  // peer explored; in a group with an award, the awarded bundle and the
  // frontiers this robot found itself that no award has named yet.
  const double now_s = now().seconds();
  const double radius = fleet_params_.cluster_merge_radius_m;
  const auto near = [radius](const mgg::FrontierCluster& cluster,
                             const std::vector<mgg::FleetCluster>& others) {
    return std::any_of(others.begin(), others.end(),
                       [&](const mgg::FleetCluster& other) {
                         return (other.position - cluster.position).norm() <=
                                radius;
                       });
  };
  const std::vector<mgg::FleetCluster> held = fleet_->claimedByOthers(now_s);
  const std::vector<mgg::FleetCluster>& bundle = fleet_->bundle();
  const std::vector<mgg::FleetCluster>& explored = fleet_->exploredElsewhere();
  const std::vector<mgg::FleetCluster>& awarded = fleet_->lastAwardClusters();
  const bool assigned = fleet_->inGroup(now_s) && fleet_->hasAward();
  clusters.erase(
      std::remove_if(clusters.begin(), clusters.end(),
                     [&](const mgg::FrontierCluster& cluster) {
                       if (near(cluster, explored)) return true;
                       // A chain's auctioneer may not hear a claim we do.
                       // Respect that claim even when our award overlaps it.
                       if (near(cluster, held)) return true;
                       if (near(cluster, bundle)) return false;
                       return assigned && (cluster.owner_robot_id != own_id ||
                                           near(cluster, awarded));
                     }),
      clusters.end());
  if (tour_fleet_assignment_version_ != fleet_->assignmentVersion()) {
    tour_fleet_assignment_version_ = fleet_->assignmentVersion();
    ++tour_assignment_version_;
  }
  return insideExplorationRegion(std::move(clusters));
}

std::optional<mgg::FrontierCluster> PlannerNode::refreshTour(
    std::string& note) {
  note.clear();
  aerial_fallback_remains_ = false;
  aerial_unknown_progress_clusters_.clear();
  if (!tour_params_.enabled || global_graph_->getNumVertices() <= 1) {
    if (global_graph_->getNumVertices() <= 1) tour_at_target_failures_.clear();
    tour_clusters_.clear();
    return std::nullopt;
  }
  noteGlobalGraphEdges();
  refreshPeerGeneration();
  // A home cut off is retried before the tour decides whether to solve: a
  // map revision alone does not solve it again.
  if (robot_params_.type == mgg::RobotType::kAerialRobot &&
      home_link_status_ != "connected") {
    if (mgg::Vertex* link = linkRobotToGlobalGraph()) relinkAerialHome(link->id);
  }
  std::vector<mgg::FrontierCluster> clusters = globalFrontierClusters();
  // Keep backoff only for existing clusters, including ones temporarily
  // reserved by peers. New evidence resets even an already-lapsed aside.
  for (auto it = tour_at_target_failures_.begin();
       it != tour_at_target_failures_.end();) {
    const auto cluster = std::find_if(
        clusters.begin(), clusters.end(),
        [&](const mgg::FrontierCluster& c) { return c.id == it->first; });
    const double gain = it->second.gain;
    const bool gain_rose = cluster != clusters.end() && gain > 0.0 &&
        cluster->gain - gain >
            std::max(kTourSetAsideGainRise * gain, tour_params_.min_cluster_gain);
    it = cluster == clusters.end() || gain_rose
             ? tour_at_target_failures_.erase(it) : std::next(it);
  }
  clusters = tourCandidates(std::move(clusters));
  const double now_s = now().seconds();
  // Clusters set aside stay out until something that could change the
  // outcome has: the robot's position, new evidence at the cluster (its
  // gain rising markedly), or the retry deadline (a clock that went
  // backwards counts as passed). Not any new graph revision: the roadmap
  // grows every cycle, and a cluster released at 103.5 s was offered and
  // released again at 106.2 s (run 10b, robot_1). At-target failures ignore
  // movement: it is not evidence that the frontier can now be observed.
  for (auto it = tour_set_aside_.begin(); it != tour_set_aside_.end();) {
    const TourSetAside& aside = it->second;
    const mgg::ClusterId id = it->first;
    const auto cluster = std::find_if(
        clusters.begin(), clusters.end(),
        [id](const mgg::FrontierCluster& c) { return c.id == id; });
    // A gain of zero is one never scored (as searchGlobalFrontier reads
    // it): its first score is no rise.
    const bool gain_rose =
        cluster != clusters.end() && aside.gain > 0.0 &&
        cluster->gain - aside.gain >
            std::max(kTourSetAsideGainRise * aside.gain,
                     tour_params_.min_cluster_gain);
    const bool lapsed =
        (!aside.at_target &&
         (current_state_.head<3>() - aside.position).norm() >
             kTourSetAsideMoveM) ||
        gain_rose || now_s < aside.at_s || now_s - aside.at_s >= aside.retry_s;
    it = lapsed ? tour_set_aside_.erase(it) : std::next(it);
  }
  clusters.erase(std::remove_if(clusters.begin(), clusters.end(),
                                [this](const mgg::FrontierCluster& cluster) {
                                  return tour_set_aside_.count(cluster.id) >
                                         0;
                                }),
                 clusters.end());
  // Known-forward targets have priority, but a temporary route set-aside
  // must not hide the unknown-progress fallback. Feasibility was checked
  // before this tier preference; it is not a new motion certificate.
  if (!fleet_ && robot_params_.type == mgg::RobotType::kAerialRobot &&
      std::any_of(clusters.begin(), clusters.end(), [this](const auto& c) {
        return aerial_unknown_progress_clusters_.count(c.id) == 0;
      })) {
    const auto before = clusters.size();
    clusters.erase(std::remove_if(clusters.begin(), clusters.end(), [this](const auto& c) {
      return aerial_unknown_progress_clusters_.count(c.id) != 0;
    }), clusters.end());
    aerial_counters_.gate_unknown_deferred += before - clusters.size();
  }
  // Reached: within global_frontier_reach_m of it, as a repositioning's
  // frontier is (onPlanRequest). The next solve chooses freely. A target
  // released as reached once is not released again (tour_reached_cluster_).
  for (const mgg::FrontierCluster& cluster : clusters) {
    if (cluster.id == tour_planner_->target() &&
        cluster.id != tour_reached_cluster_ &&
        (cluster.position - current_state_.head<3>()).norm() <=
            global_frontier_reach_m_) {
      tour_reached_cluster_ = cluster.id;
      tour_planner_->releaseTarget();
      break;
    }
  }
  // Without fleet assignment a cluster's worth is judged on every refresh,
  // on its current gain at its distance at the last solve: one crossing
  // the value floor either way solves the tour again at once, so a target
  // no longer worth its distance is not kept, nor one now worth it left
  // out, while the graph stays as it is (review r0, I-2).
  const int own_id = static_cast<int>(planning_params_.robot_id);
  if (!fleet_ && robot_params_.type == mgg::RobotType::kAerialRobot) {
    const double front = aerialFleetFront();
    if (front != aerial_front_m_) {
      aerial_front_m_ = front;
      ++tour_assignment_version_;
    }
  }
  const bool worth_changed =
      !fleet_ &&
      std::any_of(clusters.begin(), clusters.end(),
                  [&](const mgg::FrontierCluster& cluster) {
                    const auto distance =
                        tour_value_distances_.find(cluster.id);
                    if (distance == tour_value_distances_.end()) return false;
                    return mgg::tourClusterWorthItsDistance(
                               tourValueCluster(cluster), distance->second,
                               tour_params_.min_cluster_gain, own_id,
                               mgg::kGlobalOtherRobotPenalty) !=
                           (tour_value_worth_.count(cluster.id) > 0);
                  });
  if (worth_changed ||
      tour_planner_->needsSolve(clusters, graph_revision_,
                                tour_assignment_version_, now_s,
                                peer_generation_)) {
    mgg::Vertex* link = linkRobotToGlobalGraph();
    if (link == nullptr) {
      tour_clusters_.clear();
      note = "; tour: the robot's pose cannot be linked to the global graph";
      return std::nullopt;
    }
    noteGlobalGraphEdges();
    relinkAerialHome(link->id);
    const auto started = std::chrono::steady_clock::now();
    mgg::TourCostMatrix costs = mgg::computeTourCosts(
        *global_graph_, graph_revision_, tour_distances_, link->id,
        current_state_[3], clusters, tour_params_.heading_weight,
        peer_generation_);
    const auto before_reach_cap = costs.from_robot;
    const std::vector<double> back = homeDistances(clusters);
    mgg::capTourCostsByReach(costs, back, flight_reach_m_);
    if (fleet_ && robot_params_.type == mgg::RobotType::kAerialRobot) {
      for (std::size_t i = 0; i < clusters.size(); ++i) {
        if (clusters[i].owner_robot_id == own_id && !std::isfinite(costs.from_robot[i])) {
          const bool reach_capped = std::isfinite(before_reach_cap[i]);
          const std::string reach_note =
              reach_capped ? noteReachCapReject(costs.distance_from_robot[i], back[i])
                           : std::string();
          RCLCPP_INFO(get_logger(), "aerial tour: own cluster %016llx unreachable: %s",
              static_cast<unsigned long long>(clusters[i].id),
              reach_capped ? reach_note.c_str() :
              findGlobalVertex(clusters[i].representative_vertex_id) ?
                  "disconnected or search-time blocked" : "representative off graph");
        }
      }
    }
    // MGG weighed a global frontier's gain against its distance, and a
    // peer's frontier at a thousandth (rrg.cpp:5798 to 5804); the tour
    // takes only the clusters worth theirs. Run 12, robot_2: when its own
    // last cluster dipped under the floor the tour fell back to 24 peer
    // clusters and drove to one, then back when its own returned. With
    // fleet assignment the auction has costed distance and ownership
    // already, and an award the tour refused would leave the robot idle,
    // so only without.
    tour_value_distances_.clear();
    tour_value_worth_.clear();
    tour_value_left_out_ = 0;
    if (!fleet_) {
      for (std::size_t i = 0; i < clusters.size(); ++i) {
        if (std::isfinite(costs.from_robot[i])) {
          tour_value_distances_[clusters[i].id] =
              costs.distance_from_robot[i];
        }
      }
      tour_value_left_out_ = capTourValues(costs, clusters);
      for (std::size_t i = 0; i < clusters.size(); ++i) {
        if (std::isfinite(costs.from_robot[i])) {
          tour_value_worth_.insert(clusters[i].id);
        }
      }
    }
    biasAerialTourCosts(costs, clusters);
    tour_planner_->solve(clusters, costs, graph_revision_,
                         tour_assignment_version_, now_s, peer_generation_);
    if (robot_params_.type == mgg::RobotType::kAerialRobot) {
      mgg::ClusterId lifted_target = mgg::kNoCluster;
      for (const mgg::FrontierCluster& cluster : clusters) {
        if (cluster.id != tour_planner_->target()) continue;
        const mgg::Vertex* representative =
            findGlobalVertex(cluster.representative_vertex_id);
        if (representative != nullptr && representative->lifted_peer_target) {
          lifted_target = cluster.id;
        }
        break;
      }
      if (lifted_target != mgg::kNoCluster &&
          lifted_target != aerial_counters_.lifted_selected_target) {
        ++aerial_counters_.lifted_selected;
      }
      aerial_counters_.lifted_selected_target = lifted_target;
    }
    tour_solve_ms_ = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - started)
                         .count();
    if (tour_planner_->target() != tour_reached_cluster_) {
      tour_reached_cluster_ = mgg::kNoCluster;
    }
    publishTour();
  }
  tour_clusters_ = clusters;
  const mgg::TourPlan& plan = tour_planner_->plan();
  for (const mgg::FrontierCluster& cluster : tour_clusters_) {
    if (cluster.id != tour_planner_->target()) continue;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "; tour: %zu of %zu cluster(s), %.1f m%s, target %016llx "
                  "at (%.2f, %.2f, %.2f), costed and solved in %.1f ms",
                  plan.clusters.size(), tour_clusters_.size(), plan.cost,
                  plan.kept_target ? " (target kept)" : "",
                  static_cast<unsigned long long>(cluster.id),
                  cluster.position.x(), cluster.position.y(),
                  cluster.position.z(), tour_solve_ms_);
    note = buf;
    const mgg::Vertex* representative =
        findGlobalVertex(cluster.representative_vertex_id);
    if (representative != nullptr && representative->lifted_peer_target) {
      note += ", lifted from robot " + std::to_string(cluster.owner_robot_id) +
              "'s frontier";
    }
    return cluster;
  }
  if (!tour_clusters_.empty() && tour_value_left_out_ > 0) {
    note = "; tour: no reachable cluster worth its distance (" +
           std::to_string(tour_value_left_out_) + " of " +
           std::to_string(tour_clusters_.size()) +
           " below the value floor)";
    return std::nullopt;
  }
  note = tour_clusters_.empty() ? "; tour: no cluster"
                                : "; tour: no reachable cluster";
  if (!fleet_ && robot_params_.type == mgg::RobotType::kAerialRobot && aerialLocalFrontiers()) {
    note += "; own local frontiers take priority over lifted fallback";
  }
  if (robot_params_.type == mgg::RobotType::kAerialRobot &&
      home_link_status_.rfind("unreachable", 0) == 0) {
    note += " (home " + home_link_status_ + ")";
  }
  return std::nullopt;
}

bool PlannerNode::tourKeepsRoute(int vertex_id) {
  if (!tour_params_.enabled) return true;
  const mgg::ClusterId target = tour_planner_->target();
  if (target == mgg::kNoCluster) return true;
  const auto clusters = tourCandidates(globalFrontierClusters());
  if (!fleet_ && robot_params_.type == mgg::RobotType::kAerialRobot &&
      aerial_unknown_progress_clusters_.count(target) &&
      std::any_of(clusters.begin(), clusters.end(), [this](const auto& c) {
        return !aerial_unknown_progress_clusters_.count(c.id) && !tour_set_aside_.count(c.id);
      })) return false;
  for (const mgg::FrontierCluster& cluster : clusters) {
    if (cluster.id != target) continue;
    return std::find(cluster.member_vertex_ids.begin(),
                     cluster.member_vertex_ids.end(),
                     vertex_id) != cluster.member_vertex_ids.end();
  }
  return false;  // explored, reserved by a peer, or no longer this robot's
}

bool PlannerNode::localPathServesTour(const Eigen::Vector3d& viewpoint,
                                      const Eigen::Vector3d& target) const {
  const Eigen::Vector3d robot = current_state_.head<3>();
  // PCI checks the endpoint against its planar reach_distance, not how
  // far the path travels or how much closer it gets to the tour target.
  if ((viewpoint - robot).head<2>().norm() <= reach_distance_) return false;
  const Eigen::AngleAxisd to_lattice(-current_state_[3],
                                     Eigen::Vector3d::UnitZ());
  return mgg::localPathServesTarget(
      Eigen::Vector3d::Zero(), to_lattice * (viewpoint - robot),
      to_lattice * (target - robot), grid_params_.min_val,
      grid_params_.max_val, global_frontier_reach_m_);
}

void PlannerNode::publishTour() {
  nav_msgs::msg::Path msg;
  msg.header.stamp = now();
  msg.header.frame_id = world_frame_;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg.header;
  pose.pose = toPoseMsg(current_state_);
  msg.poses.push_back(pose);
  for (const mgg::FrontierCluster& cluster : tour_planner_->plan().clusters) {
    pose.pose = toPoseMsg(mgg::StateVec(cluster.position.x(),
                                        cluster.position.y(),
                                        cluster.position.z(), 0.0));
    msg.poses.push_back(pose);
  }
  tour_pub_->publish(msg);
}

bool PlannerNode::peerTransform(int robot_id, const std::string& frame,
                                Eigen::Isometry3d& t_ours_theirs) {
  if (!refreshNeighbourTransform(robot_id, frame)) return false;
  return poses_->getRobotTransform(robot_id, t_ours_theirs);
}

void PlannerNode::onTourBid(mgg_msgs::msg::TourBid::ConstSharedPtr msg) {
  if (!fleet_ || msg->robot_id == static_cast<int>(planning_params_.robot_id)) {
    return;
  }
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  Eigen::Isometry3d t_ours_theirs = Eigen::Isometry3d::Identity();
  if (!peerTransform(msg->robot_id, msg->header.frame_id, t_ours_theirs)) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
                          "bid from robot %d ignored: no transform to '%s'",
                          msg->robot_id, msg->header.frame_id.c_str());
    return;
  }
  const mgg::TourBidData bid = fromTourBidMsg(*msg, t_ours_theirs);
  // An exit carries a pose for the same simulated radio range as a bid,
  // but no tour or speed. It must not count as a normal bid receipt.
  if (bid.leaving) {
    if (bid.robot_id < 0 || bid.seq == 0 || !bid.pose.allFinite()) return;
    if (communication_range_ > 0.0 &&
        (bid.pose.head<3>() - current_state_.head<3>()).norm() >
            communication_range_) {
      return;
    }
    fleet_->onBid(bid, now().seconds());
    return;
  }
  if (!bid.wellFormed()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "ignoring a malformed bid from robot %d",
                         msg->robot_id);
    return;
  }
  if (communication_range_ > 0.0 &&
      (bid.pose.head<3>() - current_state_.head<3>()).norm() >
          communication_range_) {
    return;
  }
  const double now_s = now().seconds();
  fleet_bid_received_s_[bid.robot_id] = now_s;
  fleet_->onBid(bid, now_s);
}

void PlannerNode::onTourAward(mgg_msgs::msg::TourAward::ConstSharedPtr msg) {
  if (!fleet_ ||
      msg->auctioneer_id == static_cast<int>(planning_params_.robot_id)) {
    return;
  }
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  Eigen::Isometry3d t_ours_theirs = Eigen::Isometry3d::Identity();
  if (!peerTransform(msg->auctioneer_id, msg->header.frame_id,
                     t_ours_theirs)) {
    return;
  }
  const double now_s = now().seconds();
  const auto heard = fleet_bid_received_s_.find(msg->auctioneer_id);
  if (heard == fleet_bid_received_s_.end()) return;
  // Match the coordinator's clock-reset policy: future receipts age anew.
  if (heard->second > now_s) heard->second = now_s;
  if (now_s - heard->second > fleet_params_.peer_timeout_s) return;
  const std::uint64_t before = fleet_->assignmentVersion();
  fleet_->onAward(fromTourAwardMsg(*msg, t_ours_theirs), now_s);
  if (!msg->call && fleet_->assignmentVersion() != before) {
    RCLCPP_INFO(get_logger(),
                "fleet award %llu from robot %d: %zu cluster(s) for this "
                "robot",
                static_cast<unsigned long long>(msg->auction_id),
                msg->auctioneer_id, fleet_->bundle().size());
  }
}

void PlannerNode::onReleaseClaims(
    const std::shared_ptr<mgg_msgs::srv::ReleaseClaims::Request> request,
    std::shared_ptr<mgg_msgs::srv::ReleaseClaims::Response> response) {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (!fleet_) {
    response->success = false;
    response->message = "fleet assignment is off (fleet.enabled)";
    return;
  }
  const double now_s = now().seconds();
  if (fleet_->auctioneer(now_s) != static_cast<int>(planning_params_.robot_id)) {
    response->success = false;
    response->message = "not the auctioneer: robot " +
                        std::to_string(fleet_->auctioneer(now_s)) + " is";
    return;
  }
  if (!fleet_->releaseClaims(request->robot_id, now_s)) {
    response->success = false;
    response->message = request->robot_id == static_cast<int>(planning_params_.robot_id)
                            ? "cannot release this robot's own claims"
                            : "unknown robot " + std::to_string(request->robot_id);
    return;
  }
  response->success = true;
  response->message = "robot " + std::to_string(request->robot_id) +
      (fleet_->inGroup(now_s)
           ? "'s claims released; forwarded in the next award"
           : "'s claims released locally; no peers, nothing forwarded");
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void PlannerNode::fleetTick(double now_s) {
  if (!fleet_) return;
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (!have_odometry_ || home_state_wait_started_) return;
  auto map_read = mapReadLease();
  // Bids and auctions cost routes with one peer set, as a request does.
  std::optional<PeerBodyPin> peer_pin;
  pinPeerBodies(peer_pin);
  refreshPeerGeneration();
  const mgg::FleetTickOutput out =
      fleet_->tick(now_s, [this]() { return ownTourBid(); },
                   roadmapCostEstimate(), exploredByRoadmap());
  if (out.bid) tour_bid_pub_->publish(toTourBidMsg(*out.bid, world_frame_));
  if (!out.award) return;
  tour_award_pub_->publish(toTourAwardMsg(*out.award, world_frame_));
  if (out.award->call) return;
  std::size_t assigned = 0;
  for (const mgg::RobotBundle& bundle : out.award->bundles) {
    assigned += bundle.clusters.size();
  }
  RCLCPP_INFO(get_logger(),
              "fleet award %llu: %zu cluster(s) to %zu robot(s), %zu "
              "unassigned, %zu explored, %zu claim(s) released",
              static_cast<unsigned long long>(out.award->auction_id),
              assigned, out.award->bundles.size(), fleet_->lastUnassigned(),
              out.award->explored.size(),
              out.award->released_robot_ids.size());
}

void PlannerNode::relinkAerialHome(int robot_vertex_id) {
  if (robot_params_.type != mgg::RobotType::kAerialRobot) return;
  mgg::Vertex* home = findGlobalVertex(kHomeVertexId);
  if (home == nullptr || !global_graph_->inService(*home)) {
    home_link_status_ = "no home";
    return;
  }
  const auto key = std::make_tuple(
      static_cast<const mgg::GraphManager*>(global_graph_.get()),
      graph_revision_, map_revision_, peer_generation_);
  if (key == home_link_key_) return;
  home_link_key_ = key;
  if (robot_vertex_id == kHomeVertexId) {
    home_link_status_ = "connected";
    return;
  }
  const mgg::ShortestPathsReport* report = tour_distances_.from(
      *global_graph_, graph_revision_, robot_vertex_id, peer_generation_);
  if (report != nullptr &&
      std::isfinite(mgg::reachedDistance(*report, kHomeVertexId))) {
    home_link_status_ = "connected";
    return;
  }
  auto map_read = mapReadLease();
  const mgg::ExpandContext ctx = makeGlobalContext();
  // The same evidence Return Home asks of its goal (routeOverGlobalGraph).
  if (map_->getStaticStrictBoxStatus(
          home->state.head<3>() + robot_params_.center_offset,
          ctx.robot_box_size) != mgg::VoxelStatus::kFree) {
    home_link_status_ = "unreachable: home body not observed free";
    return;
  }
  ++home_relink_attempts_;
  mgg::ExpandGraphReport rep;
  mgg::expandGraphEdges(*global_graph_, home, rep, ctx);
  if (rep.num_edges_added == 0) {
    home_link_status_ = "unreachable: no admissible edge from home";
    return;
  }
  ++graph_revision_;
  ++home_relinks_;
  home_link_key_ = std::make_tuple(
      static_cast<const mgg::GraphManager*>(global_graph_.get()),
      graph_revision_, map_revision_, peer_generation_);
  const mgg::ShortestPathsReport* after = tour_distances_.from(
      *global_graph_, graph_revision_, robot_vertex_id, peer_generation_);
  const bool reached =
      after != nullptr &&
      std::isfinite(mgg::reachedDistance(*after, kHomeVertexId));
  home_link_status_ = reached ? "relinked" : "unreachable: relinked elsewhere";
  RCLCPP_INFO(get_logger(),
              "aerial home (%.2f, %.2f, %.2f) relinked: +%d edge(s); %s",
              home->state.x(), home->state.y(), home->state.z(),
              rep.num_edges_added,
              reached ? "reached from the robot" : "still not reached");
}

std::string PlannerNode::noteReachCapReject(double out_m, double back_m) {
  AerialCounters& counts = aerial_counters_;
  ++counts.reach_cap_rejects;
  if (!std::isfinite(back_m)) ++counts.reach_cap_home_unreachable;
  counts.last_reach_out_m = out_m;
  counts.last_reach_back_m = back_m;
  counts.last_reach_budget_m = flight_reach_m_;
  char note[192];
  if (std::isfinite(back_m)) {
    std::snprintf(note, sizeof(note),
                  "battery return reach cap (out %.1f m + back %.1f m > "
                  "reach %.1f m)",
                  out_m, back_m, flight_reach_m_);
  } else {
    std::snprintf(note, sizeof(note),
                  "battery return reach cap (out %.1f m, no way back to home: "
                  "%s; reach %.1f m)",
                  out_m, home_link_status_.c_str(), flight_reach_m_);
  }
  return note;
}

namespace {
/// A JSON number, or null when it is not finite.
std::string jsonNumber(double value) {
  if (!std::isfinite(value)) return "null";
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%.3f", value);
  return buf;
}
/// `text` as a JSON string, its quotes and backslashes escaped.
std::string jsonString(const std::string& text) {
  std::string out = "\"";
  for (const char c : text) {
    if (c == '"' || c == '\\') out += '\\';
    out += c;
  }
  return out + "\"";
}
}  // namespace

std::string PlannerNode::aerialStatusJson() const {
  const AerialCounters& c = aerial_counters_;
  const auto home_it = global_graph_->vertices_map_.find(kHomeVertexId);
  const mgg::Vertex* home =
      home_it == global_graph_->vertices_map_.end() ? nullptr : home_it->second;
  const auto home_edges = global_graph_->edge_map_.find(kHomeVertexId);
  std::size_t legacy_discs = 0;
  if (mola_map_ != nullptr) {
    legacy_discs = mola_map_->activeTransientDiscs().centres.size();
  }
  std::string j = "{\"robot_id\":" + std::to_string(planning_params_.robot_id);
  const auto recovery = mola_map_ != nullptr
      ? mola_map_->aerialRootRecoveryStats()
      : mgg::MolaMap::AerialRootRecoveryStats{};
  j += ",\"root_recovery\":{\"uses\":" + std::to_string(recovery.uses) +
       ",\"last_exempted_cells\":" + std::to_string(recovery.exempted_cells) +
       ",\"last_direction\":[" + jsonNumber(recovery.direction.x()) + "," +
       jsonNumber(recovery.direction.y()) + "," +
       jsonNumber(recovery.direction.z()) + "]}";
  j += ",\"home\":{\"status\":" + jsonString(home_link_status_) +
       ",\"x\":" + jsonNumber(home ? home->state.x() : NAN) +
       ",\"y\":" + jsonNumber(home ? home->state.y() : NAN) +
       ",\"z\":" + jsonNumber(home ? home->state.z() : NAN) +
       ",\"lifted\":" + (home_seeded_landed_ ? "true" : "false") +
       ",\"edges\":" +
       std::to_string(home_edges == global_graph_->edge_map_.end()
                          ? 0 : home_edges->second.size()) +
       ",\"reroots\":" + std::to_string(home_reroots_) +
       ",\"relink_attempts\":" + std::to_string(home_relink_attempts_) +
       ",\"relinks\":" + std::to_string(home_relinks_) + "}";
  j += ",\"flown\":{\"breadcrumbs\":" +
       std::to_string(flown_trail_counters_.added) +
       ",\"merged\":" + std::to_string(flown_trail_counters_.merged) +
       ",\"joined\":" + std::to_string(flown_trail_counters_.joined) +
       ",\"given_up\":" + std::to_string(flown_trail_counters_.expired) +
       ",\"capped\":" + std::to_string(flown_trail_counters_.capped) +
       ",\"pending\":" + std::to_string(flown_trail_.pending.size()) +
       ",\"chained\":" + (flown_trail_.head_vertex_id >= 0 ? "true" : "false") +
       ",\"map_corrections\":" + std::to_string(map_corrections_.load()) + "}";
  j += ",\"lifted\":{\"proposed\":" + std::to_string(c.lifted_proposed) +
       ",\"admitted\":" + std::to_string(c.lifted_admitted) +
       ",\"rejected\":{\"gain\":" + std::to_string(c.lifted_rejected_gain) +
       ",\"cap\":" + std::to_string(c.lifted_rejected_cap) +
       ",\"merged\":" + std::to_string(c.lifted_rejected_merged) +
       ",\"no_anchor\":" + std::to_string(c.lifted_rejected_no_anchor) +
       ",\"link\":" + std::to_string(c.lifted_rejected_link) +
       ",\"scouting\":" + std::to_string(c.lifted_rejected_scouting) +
       "},\"senders_unplaced\":" + std::to_string(c.lifted_senders_unplaced) +
       ",\"selected\":" + std::to_string(c.lifted_selected) + "}";
  j += ",\"progress_gate\":{\"own_or_local\":" + std::to_string(c.gate_own_or_local) +
       ",\"behind_peer\":" + std::to_string(c.gate_behind_peer) +
       ",\"behind_drone\":" + std::to_string(c.gate_behind_drone) +
       ",\"known_forward\":" + std::to_string(c.gate_known_forward) +
       ",\"unknown_progress\":" + std::to_string(c.gate_unknown_progress) +
       ",\"unknown_deferred\":" + std::to_string(c.gate_unknown_deferred) + "}";
  j += ",\"front_peer\":" + std::to_string(aerial_front_peer_) +
       ",\"front_is_lower_bound\":" + (aerial_front_is_lower_bound_ ? "true" : "false");
  j += ",\"cylinders\":{\"accepted\":" +
       std::to_string(c.cylinder_messages_accepted) +
       ",\"rejected_frame\":" + std::to_string(c.cylinder_messages_wrong_frame) +
       ",\"rejected_invalid\":" + std::to_string(c.cylinder_messages_invalid) +
       ",\"frame\":" + jsonString(c.cylinder_frame) +
       ",\"in_force\":" + std::to_string(activeAerialPeerBodies().size()) +
       ",\"legacy_discs_in_force\":" + std::to_string(legacy_discs) +
       ",\"segments_blocked\":{\"cylinder\":" +
       std::to_string(c.cylinder_segment_blocks) +
       ",\"legacy_disc\":" + std::to_string(c.disc_segment_blocks) + "}}";
  j += ",\"reach_cap\":{\"rejects\":" + std::to_string(c.reach_cap_rejects) +
       ",\"home_unreachable\":" + std::to_string(c.reach_cap_home_unreachable) +
       ",\"last_out_m\":" + jsonNumber(c.last_reach_out_m) +
       ",\"last_back_m\":" + jsonNumber(c.last_reach_back_m) +
       ",\"last_budget_m\":" + jsonNumber(c.last_reach_budget_m) +
       ",\"reach_m\":" + jsonNumber(flight_reach_m_) + "}";
  j += ",\"tour\":{\"in_reach_set_aside\":" +
       std::to_string(tour_in_reach_set_aside_) + "}";
  const ScoutingExclusionCounters& x = scouting_counters_;
  j += ",\"scouting_exclusions\":{\"in_force\":" +
       std::to_string(scouting_exclusion_centres_.size()) +
       ",\"accepted\":" + std::to_string(x.messages_accepted) +
       ",\"rejected_frame\":" + std::to_string(x.messages_wrong_frame) +
       ",\"rejected_invalid\":" + std::to_string(x.messages_invalid) +
       ",\"lapsed\":" + std::to_string(x.lapsed) +
       ",\"targets_refused\":" + std::to_string(x.targets_refused) +
       ",\"viewpoints_refused\":" + std::to_string(x.viewpoints_refused) +
       ",\"paths_refused\":" + std::to_string(x.paths_refused) + "}";
  return j + "}";
}

std::vector<double> PlannerNode::homeDistances(
    const std::vector<mgg::FrontierCluster>& clusters) {
  // The way back, from each representative to home: a peer body's margin
  // may be left but not entered, so the way out from home can be open where
  // the way back is not. computeTourCosts has solved from each
  // representative under this key already.
  std::vector<double> distances(clusters.size(), mgg::kUnreachableCost);
  for (std::size_t i = 0; i < clusters.size(); ++i) {
    const mgg::ShortestPathsReport* report = tour_distances_.from(
        *global_graph_, graph_revision_, clusters[i].representative_vertex_id,
        peer_generation_);
    if (report != nullptr) {
      distances[i] = mgg::reachedDistance(*report, kHomeVertexId);
    }
  }
  return distances;
}

mgg::TourBidData PlannerNode::ownTourBid() {
  mgg::TourBidData bid;
  bid.robot_id = static_cast<int>(planning_params_.robot_id);
  bid.auctioneer_id = fleet_->auctioneer(now().seconds());
  bid.pose = current_state_;
  bid.current_target = tour_planner_->target();
  bid.claim_stamp_s = tour_planner_->targetSince();
  bid.speed_mps = planning_params_.v_max;
  bid.reach_m = flight_reach_m_;
  if (const mgg::Vertex* home = findGlobalVertex(kHomeVertexId)) {
    bid.home = home->state.head<3>();
  }
  // §3.3 pool construction: the awarded clusters this robot's roadmap
  // shows explored.
  for (const mgg::FleetCluster& cluster : fleet_->lastAwardClusters()) {
    if (mgg::exploredInGraph(*global_graph_, cluster.position,
                             fleet_params_.cluster_merge_radius_m)) {
      bid.explored.push_back(cluster.id);
    }
  }
  if (global_graph_->getNumVertices() <= 1) return bid;
  std::vector<mgg::FrontierCluster> clusters = globalFrontierClusters();
  // Best gain first: what a peer would refuse as too large is cut from the
  // low-gain end.
  if (clusters.size() > mgg::kMaxBidClusters) {
    clusters.resize(mgg::kMaxBidClusters);
  }
  if (clusters.empty()) return bid;
  mgg::TourCostMatrix costs;
  refreshPeerGeneration();
  if (mgg::Vertex* link = linkRobotToGlobalGraph()) {
    relinkAerialHome(link->id);
    costs = mgg::computeTourCosts(*global_graph_, graph_revision_,
                                  tour_distances_, link->id,
                                  current_state_[3], clusters,
                                  /*heading_weight=*/0.0, peer_generation_);
    mgg::capTourCostsByReach(costs, homeDistances(clusters), flight_reach_m_);
  } else {
    costs.from_robot.assign(clusters.size(), mgg::kUnreachableCost);
    costs.between.assign(clusters.size(),
                         std::vector<double>(clusters.size(),
                                             mgg::kUnreachableCost));
  }
  for (std::size_t i = 0; i < clusters.size(); ++i) {
    const mgg::FrontierCluster& c = clusters[i];
    bid.clusters.push_back({c.id, c.owner_robot_id, c.position, c.gain});
    bid.costs_from_pose.push_back(costs.from_robot[i]);
    for (std::size_t j = 0; j < clusters.size(); ++j) {
      bid.costs_between.push_back(i == j ? 0.0 : costs.between[i][j]);
    }
  }
  return bid;
}

mgg::CostEstimateFn PlannerNode::roadmapCostEstimate() {
  return [this](const Eigen::Vector3d& from, const Eigen::Vector3d& to) {
    const mgg::StateVec from_state(from.x(), from.y(), from.z(), 0.0);
    const mgg::StateVec to_state(to.x(), to.y(), to.z(), 0.0);
    mgg::Vertex* a = nullptr;
    mgg::Vertex* b = nullptr;
    if (!global_graph_->getNearestVertex(&from_state, &a) ||
        !global_graph_->getNearestVertex(&to_state, &b) || a == nullptr ||
        b == nullptr) {
      return mgg::kUnreachableCost;
    }
    const mgg::ShortestPathsReport* report =
        tour_distances_.from(*global_graph_, graph_revision_, a->id,
                             peer_generation_);
    if (report == nullptr) return mgg::kUnreachableCost;
    return mgg::reachedDistance(*report, b->id) +
           (from - a->state.head<3>()).norm() +
           (to - b->state.head<3>()).norm();
  };
}

mgg::ExploredFn PlannerNode::exploredByRoadmap() {
  return [this](const Eigen::Vector3d& position) {
    return mgg::exploredInGraph(*global_graph_, position,
                                fleet_params_.cluster_merge_radius_m);
  };
}

std::vector<Eigen::Vector3d> PlannerNode::fleetExclusions() {
  std::vector<Eigen::Vector3d> points;
  if (!fleet_) return points;
  for (const mgg::FleetCluster& c : fleet_->claimedByOthers(now().seconds())) {
    points.push_back(c.position);
  }
  for (const mgg::FleetCluster& c : fleet_->exploredElsewhere()) {
    points.push_back(c.position);
  }
  return points;
}

bool PlannerNode::settleIdleRobot(std::string& summary, bool& complete) {
  if (fleet_ && fleet_->leaving()) return false;
  const double now_s = now().seconds();
  if (fleet_->inGroup(now_s)) {
    if (fleet_->requestAnswered()) {
      // As a failed global search is (review r0, I-2): not exploration
      // complete while the lattice still sees gain it cannot send a path
      // to, nor the first time after a graph rebuild dropped frontiers.
      if (local_gain_remains_now_) {
        summary +=
            "; the fleet's award leaves it nothing, but local gain remains: "
            "no path";
      } else if (frontiers_dropped_in_rebuild_ > 0) {
        summary += "; the fleet's award leaves it nothing, but a graph "
                   "rebuild dropped " +
                   std::to_string(frontiers_dropped_in_rebuild_) +
                   " frontier(s): no path";
        frontiers_dropped_in_rebuild_ = 0;
      } else if (const std::string withheld = completionWithheld();
                 !withheld.empty()) {
        summary += "; the fleet's award leaves it nothing, but " + withheld +
                   ": no path";
      } else {
        complete = true;
        summary +=
            "; exploration complete for this robot: the fleet's award "
            "leaves it nothing";
      }
    } else {
      if (!fleet_->awaitingAuction()) fleet_->requestAuction();
      summary += "; its bundle is done: waiting for the auction it asked for";
    }
    return true;
  }
  const int taken = fleet_->takeOverOldestClaim(now_s);
  if (taken < 0) return false;
  summary += "; took over silent robot " + std::to_string(taken) +
             "'s claims";
  return true;
}

mgg::MolaMap::ReadLease PlannerNode::mapReadLease() const {
  return mola_map_ != nullptr ? mola_map_->acquireReadLease()
                              : mgg::MolaMap::ReadLease{};
}

void PlannerNode::setAcquiringObservations(bool acquiring) {
  const bool was = acquiring_observations_.exchange(acquiring);
  if (acquiring && (!was || bootstrap_started_ns_.load() == 0)) {
    bootstrap_started_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }
}

void PlannerNode::publishPlanningStatus() {
  const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  std_msgs::msg::String status;
  const bool acquiring = acquiring_observations_.load();
  const auto planned = std::atomic_load(&last_planning_snapshot_);
  status.data = "{\"bootstrap_state\":" +
      jsonString(acquiring ? "acquiring observations" : "ready") +
      ",\"bootstrap_age_s\":" + jsonNumber(acquiring ?
          (stamp - bootstrap_started_ns_.load()) * 1e-9 : 0.0) +
      ",\"heartbeats_received\":" + std::to_string(heartbeats_received_.load()) +
      ",\"heartbeats_during_planning\":" + std::to_string(heartbeats_during_planning_.load()) +
      ",\"heartbeats_validated\":" + std::to_string(mola_map_ ? mola_map_->statRevalidationCount() : 0) +
      ",\"map_identity_changes\":" + std::to_string(map_identity_changes_.load()) +
      ",\"map_corrections\":" + std::to_string(map_corrections_.load()) +
      ",\"last_plan_component_id\":" + jsonString(planned ? planned->component_id : "") +
      ",\"last_plan_epoch\":" + std::to_string(planned ? planned->epoch : 0) +
      ",\"last_plan_geometry_revision\":" + jsonString(planned ? planned->geometry_revision : "") +
      ",\"map_authority_valid\":" + (mola_map_ && mola_map_->authorityValid() ? "true" : "false") +
      ",\"map_expiries\":" + std::to_string(mola_map_ ? mola_map_->expiryCount() : 0) +
      ",\"map_error\":" + jsonString(mola_map_ ? mola_map_->lastError() : "") +
      ",\"odometry_sample_age_s\":" + jsonNumber(odometry_sample_stamp_ns_.load() ?
          (now().nanoseconds() - odometry_sample_stamp_ns_.load()) * 1e-9 : NAN) +
      ",\"odometry_ingest_lag_s\":" + jsonNumber(odometry_ingest_lag_s_.load()) +
      ",\"cancellations\":" + std::to_string(cancellations_.load()) + "}";
  planning_status_pub_->publish(status);
}

void PlannerNode::refreshMapRevision() {
  if (mola_map_ == nullptr) return;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (latest_snapshot_) {
      mapping_snapshot_ = *latest_snapshot_;
      have_mapping_snapshot_ = true;
    }
  }
  if (const auto active = mola_map_->activeRequest()) {
    // Another component or epoch is another map: nothing checked against
    // the old one holds on it, and the graphs start again. A new
    // component_from_navigation alone (C-SLAM re-optimising where the
    // component lies in the planning frame) moves the same map rigidly:
    // the graphs move with it (followMapCorrection) and keep their place
    // in the map their edges were checked against. Resetting on it too
    // (mapvalid m8) wiped the drone's roadmap and flown trail seven times
    // in one SubT session (mgg-flown evidence, robot_4).
    const bool other_map =
        served_map_identity_ &&
        (served_map_identity_->component_id != active->component_id ||
         served_map_identity_->epoch != active->epoch);
    if (served_map_identity_ && !other_map &&
        !served_map_identity_->component_from_navigation.matrix().isApprox(
            active->component_from_navigation.matrix(), 1e-9)) {
      // p_component = T_old p_old = T_new p_new.
      followMapCorrection(active->component_from_navigation.inverse() *
                          served_map_identity_->component_from_navigation);
    }
    if (other_map) {
      global_graph_->reset();
      flown_trail_ = mgg::FlownTrail{};
      local_graph_->reset();
      // Lifted slots are graph vertex IDs, not reusable across an epoch.
      lifted_target_vertices_.clear();
      lifted_targets_.clear();
      global_graph_->setRobotId(static_cast<int>(planning_params_.robot_id));
      local_graph_->setRobotId(static_cast<int>(planning_params_.robot_id));
      ++graph_revision_;
      best_path_.clear();
      global_exploration_ongoing_ = false;
      standing_start_xy_.reset();
      left_standing_start_ = false;
      bootstrap_started_ns_ = 0;
      setAcquiringObservations(true);
      ++map_identity_changes_;
    }
    served_map_identity_ = *active;
    mapping_snapshot_.component_id = active->component_id;
    mapping_snapshot_.epoch = active->epoch;
    mapping_snapshot_.graph_revision = active->graph_revision;
    mapping_snapshot_.geometry_revision = active->geometry_revision;
    mapping_snapshot_.source_stamp = rclcpp::Time(static_cast<int64_t>(active->source_stamp_ns));
    const Eigen::Quaterniond q(active->component_from_navigation.linear());
    auto& t = mapping_snapshot_.component_from_navigation;
    t.translation.x = active->component_from_navigation.translation().x();
    t.translation.y = active->component_from_navigation.translation().y();
    t.translation.z = active->component_from_navigation.translation().z();
    t.rotation.x = q.x(); t.rotation.y = q.y();
    t.rotation.z = q.z(); t.rotation.w = q.w();
    have_mapping_snapshot_ = true;
  }
  if (have_odometry_ && robot_params_.type == mgg::RobotType::kGroundRobot) {
    Eigen::Vector3d floor = current_state_.head<3>() + robot_params_.center_offset;
    floor.z() -= robot_params_.size.z() / 2.0;
    mola_map_->setFootprintGroundSupport(floor, robot_params_.size.head<2>(), current_state_[3]);
  }
  const std::uint64_t generation = mola_map_->activeGeneration();
  if (generation == observed_map_generation_) return;
  observed_map_generation_ = generation;
  ++map_revision_;
}

void PlannerNode::followMapCorrection(const Eigen::Isometry3d& delta) {
  const int own = static_cast<int>(planning_params_.robot_id);
  // Lifted targets are peers' frontiers placed by their transforms, and
  // anchored on vertices that move: placed again by the next query.
  withdrawLiftedTargets();
  global_graph_->setEdgeRecertifier(
      [this](const mgg::Vertex& a, const mgg::Vertex& b, bool terrain_changed) {
        // Peers close edges for this search only, never erase map geometry.
        std::optional<mgg::MolaMap::TransientDiscPin> no_peers;
        if (mola_map_ != nullptr) {
          no_peers.emplace(*mola_map_, std::vector<Eigen::Vector2d>{}, 0.0);
        }
        mgg::ExpandGraphReport report;
        // Own aerial roadmap edges have no height band: keep flown heights.
        // EGO enforces the execution band; only lifted peer targets use MGG's
        // aerial_min/max_height_m.
        return mgg::correctedRoadmapEdgeTraversable(
            makeGlobalContext(), a, b, terrain_changed, report);
      });
  const int moved = mgg::transformRoadmap(*global_graph_, own, delta);
  mgg::transformFlownTrail(flown_trail_, delta);
  robot_state_hist_.transform(delta);
  // Neighbours' roadmaps are placed by their own transforms, and their
  // links to this robot's vertices were checked where those stood: cut
  // and merged again, with fresh links, when their transform or roadmap
  // next arrives (readmitQuarantinedNeighbours), as after a rebuild.
  int quarantined = 0;
  for (const auto& entry : neighbour_roadmaps_) {
    const auto merged = global_graph_->merged_graphs_.find(entry.first);
    if (merged != global_graph_->merged_graphs_.end() && merged->second &&
        !global_graph_->isQuarantined(entry.first)) {
      global_graph_->disconnectNeighbourGraph(entry.first);
      ++quarantined;
    }
    if (global_graph_->isQuarantined(entry.first)) {
      roadmaps_to_readmit_.insert(entry.first);
    }
  }
  // What was planned in the old placement is planned again.
  local_graph_->reset();
  local_graph_->setRobotId(own);
  best_path_.clear();
  global_exploration_ongoing_ = false;
  ++graph_revision_;
  ++map_corrections_;
  const Eigen::Vector3d shift = delta.translation();
  RCLCPP_INFO(get_logger(),
              "map placement corrected: the global graph follows it (%d "
              "vertices moved by (%.2f, %.2f, %.2f), turned %.3f rad; %zu "
              "flown samples pending; %d neighbour roadmap(s) to merge "
              "again); %lu correction(s) so far",
              moved, shift.x(), shift.y(), shift.z(),
              std::atan2(delta.linear()(1, 0), delta.linear()(0, 0)),
              flown_trail_.pending.size(), quarantined,
              static_cast<unsigned long>(map_corrections_.load()));
}

bool PlannerNode::projectToDrivingHeight(mgg::StateVec& state) const {
  if (robot_params_.type != mgg::RobotType::kGroundRobot) return true;
  Eigen::Vector3d pos(state[0], state[1], state[2]);
  mgg::VoxelStatus status;
  const double ground_height = ground_->projectSample(pos, status);
  if (status != mgg::VoxelStatus::kOccupied) return false;
  state[0] = pos[0];
  state[1] = pos[1];
  state[2] = pos[2] - (ground_height - planning_params_.max_ground_height);
  return true;
}

bool PlannerNode::projectGoalToDrivingHeight(mgg::StateVec& state) const {
  if (robot_params_.type != mgg::RobotType::kGroundRobot) return true;
  Eigen::Vector3d pos(state[0], state[1], state[2]);
  mgg::VoxelStatus status;
  const double ground_height = ground_->projectGoal(pos, status);
  if (status != mgg::VoxelStatus::kOccupied) return false;
  state[0] = pos[0];
  state[1] = pos[1];
  state[2] = pos[2] - (ground_height - planning_params_.max_ground_height);
  return true;
}

mgg::StateVec PlannerNode::physicalAnchorAtDrivingHeight(
    const mgg::StateVec& base_pose) const {
  // The base sits half the body height above the floor it stands on; the
  // driving height is max_ground_height above that floor.
  mgg::StateVec anchor = base_pose;
  if (robot_params_.type == mgg::RobotType::kGroundRobot) {
    anchor[2] += planning_params_.max_ground_height - robot_params_.size[2] / 2.0;
  }
  return anchor;
}

bool PlannerNode::readOwnKeyframes(KeyframeTrajectory& trajectory,
                                   std::string& error) {
  if (keyframe_source_->read(trajectory, error)) {
    keyframe_read_error_logged_at_.reset();
    return true;
  }
  const auto now = std::chrono::steady_clock::now();
  if (!keyframe_read_error_logged_at_ ||
      std::chrono::duration<double>(now - *keyframe_read_error_logged_at_)
              .count() >= kKeyframeReadErrorPeriodS) {
    keyframe_read_error_logged_at_ = now;
    ++keyframe_read_errors_logged_;
    RCLCPP_ERROR(get_logger(),
                 "cannot read this robot's keyframes from %s: %s; without "
                 "them it has no standing start and no roadmap rebuild "
                 "(%d such errors logged)",
                 keyframe_source_->location().c_str(), error.c_str(),
                 keyframe_read_errors_logged_);
  }
  return false;
}

PlannerNode::StandingStartScope::StandingStartScope(PlannerNode& node) : node(node) {
  ++node.standing_start_scope_depth_;
}

PlannerNode::StandingStartScope::~StandingStartScope() {
  if (--node.standing_start_scope_depth_ == 0) {
    node.plan_standing_start_.reset();
    node.plan_own_body_known_free_.reset();
    node.plan_reverse_edges_.clear();
  }
}

std::optional<mgg::StandingStart> PlannerNode::standingStart() {
  if (standing_start_scope_depth_ == 0) return readStandingStart();
  if (!plan_standing_start_) plan_standing_start_.emplace(readStandingStart());
  return *plan_standing_start_;
}

std::optional<mgg::StandingStart> PlannerNode::readStandingStart() {
  standing_start_unread_keyframes_.clear();
  if (robot_params_.type != mgg::RobotType::kGroundRobot ||
      !standing_start_xy_ || !(hanging_root_edge_length_max_ > 0.0) ||
      keyframe_source_ == nullptr || !have_mapping_snapshot_) {
    return std::nullopt;
  }
  KeyframeTrajectory trajectory;
  std::string error;
  if (!readOwnKeyframes(trajectory, error)) {
    standing_start_unread_keyframes_ = keyframe_source_->location();
    return std::nullopt;
  }
  if (trajectory.poses.empty() ||
      trajectory.component_id != mapping_snapshot_.component_id ||
      trajectory.epoch != mapping_snapshot_.epoch) {
    return std::nullopt;
  }
  const Eigen::Isometry3d navigation_from_component = navigationFromComponent();
  const Eigen::Vector2d home =
      (navigation_from_component * trajectory.poses.front().translation())
          .head<2>();
  for (const Eigen::Isometry3d& pose : trajectory.poses) {
    if (((navigation_from_component * pose.translation()).head<2>() - home)
            .norm() >= kStandingStartMoveM) {
      // The robot has left its start, whatever odometry says since.
      standing_start_xy_.reset();
      left_standing_start_ = true;
      return std::nullopt;
    }
  }
  if ((current_state_.head<2>() - home).norm() >= kStandingStartMoveM) {
    return std::nullopt;
  }
  // Round where it stood, the blind disk of its first scans: not where it
  // has crept to since, whose ground its lidar could see.
  return mgg::StandingStart{*standing_start_xy_, hanging_root_edge_length_max_};
}

bool PlannerNode::standingStartGoalAdmissible(const mgg::StateVec& goal) {
  auto arrival = goal;
  arrival[3] = mgg::kUnknownTurnHeading;
  const auto standing = standingStart();
  return !standing || standing->admitsGoal(goal.head<2>(), reach_distance_) ||
         mgg::observedArrivalDisk(*map_, robot_params_, planning_params_, arrival,
                                  reach_distance_);
}

Eigen::Isometry3d PlannerNode::navigationFromComponent() const {
  const auto& t = mapping_snapshot_.component_from_navigation.translation;
  const auto& q = mapping_snapshot_.component_from_navigation.rotation;
  Eigen::Isometry3d component_from_navigation = Eigen::Isometry3d::Identity();
  component_from_navigation.linear() =
      Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
  component_from_navigation.translation() = Eigen::Vector3d(t.x, t.y, t.z);
  return component_from_navigation.inverse();
}

std::vector<Eigen::Vector3d> PlannerNode::selectionExclusions() {
  if (have_coordination_exclusions_ &&
      secondsSince(coordination_exclusions_received_) <=
          reservation_exclusion_ttl_s_) {
    return coordination_exclusions_;
  }
  return {};
}

mgg::RecomputeGainFn PlannerNode::globalFrontierGain() {
  return [this](mgg::Vertex& vertex) {
    if (vertex.lifted_peer_target) return;
    if (vertex.robot_id != static_cast<int>(planning_params_.robot_id)) {
      auto& gain = vertex.vol_gain;
      const bool aerial_owner = robot_params_.type == mgg::RobotType::kGroundRobot &&
          std::find(aerial_peer_robot_ids_.begin(), aerial_peer_robot_ids_.end(),
                    vertex.robot_id) != aerial_peer_robot_ids_.end();
      gain.is_frontier = gain.is_frontier && !vertex.locally_explored && !aerial_owner;
      // The shared scorer feeds greedy repositioning, tour clusters and
      // fleet offers. Never re-check flight-altitude evidence as a ground
      // pose: its height band would be measured around the ceiling too.
      if (aerial_owner && vertex.type == mgg::VertexType::kFrontier) {
        vertex.type = mgg::VertexType::kUnvisited;
      }
      gain.gain = gain.is_frontier
          ? std::max(0, gain.num_unknown_voxels) * planning_params_.unknown_voxel_gain +
            std::max(0, gain.num_free_voxels) * planning_params_.free_voxel_gain +
            std::max(0, gain.num_occupied_voxels) * planning_params_.occupied_voxel_gain
          : 0.0;
      return;
    }
    // Upstream scored global frontiers against the world-fixed global bound
    // (computeVolumetricGainRayModelNoBound, rrg.cpp:3767). This port centres
    // its gain volume on the robot every cycle, so a frontier a street away
    // would count nothing; centre it on the frontier while it is scored.
    const RestoreScope restore_space(global_space_);
    global_space_.setCenter(vertex.state, /*use_extension=*/true);
    mgg::computeVolumetricGain(vertex.state, vertex.vol_gain,
                               makeGainContext());
  };
}

// ---------------------------------------------------------------------------
// Inputs

void PlannerNode::onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  const mgg::StateVec state = fromPoseMsg(msg->pose.pose);
  if (!state.allFinite()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "ignoring non-finite odometry");
    return;
  }
  const auto received = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (latest_odometry_ &&
        rclcpp::Time(msg->header.stamp) < rclcpp::Time(latest_odometry_->header.stamp)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "ignoring odometry older than latest accepted state");
      return;
    }
    latest_odometry_ = msg;
    latest_odometry_received_ = received;
    odometry_sample_stamp_ns_ = rclcpp::Time(msg->header.stamp).nanoseconds();
  }
  odometry_ingest_lag_s_.store(
      std::chrono::duration<double>(std::chrono::steady_clock::now() - received).count());
  // Ingestion only: applying odometry can expand the graph. The planner's
  // maintenance timer and request admission drain this slot on planner threads.
}

void PlannerNode::applyLatestOdometry() {
  if (mgg::planning_cancelled) {
    applyLatestOdometryImpl();
    return;
  }
  const auto generation = request_generation_.load();
  mgg::PlanningCancellationScope cancellation([this, generation] {
    return generation != request_generation_.load();
  });
  try {
    applyLatestOdometryImpl();
  } catch (const mgg::PlanningInterrupted&) {
    ++cancellations_;
    ++graph_revision_;
  }
}

void PlannerNode::applyLatestOdometryImpl() {
  nav_msgs::msg::Odometry::ConstSharedPtr msg;
  std::chrono::steady_clock::time_point received;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    msg = latest_odometry_;
    received = latest_odometry_received_;
  }
  if (!msg || msg == applied_odometry_) return;
  applied_odometry_ = msg;
  const auto stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  const mgg::StateVec state = fromPoseMsg(msg->pose.pose);
  last_odometry_stamp_ns_ = stamp_ns;
  current_state_ = state;
  current_tilt_ = tiltFromQuaternion(msg->pose.pose.orientation);
  if (!left_standing_start_) {
    if (!standing_start_xy_) standing_start_xy_ = state.head<2>();
    if ((state.head<2>() - *standing_start_xy_).norm() >= kStandingStartMoveM) {
      standing_start_xy_.reset();
      left_standing_start_ = true;
    }
  }
  have_odometry_ = true;
  last_odometry_received_ = received;

  auto map_read = mapReadLease();
  refreshMapRevision();
  forgetReverseExitIfOffRoute();
  expireReverseExitExclusions();
  seedGlobalGraph();
  if (home_state_wait_started_) return;
  // On start, or after the graph was lost, it holds only its seed.
  if (ownGlobalVertices() <= 1) {
    rebuildGlobalGraphFromKeyframes(RoadmapRebuildTrigger::kSeedOnly,
                                    "the global graph holds only its seed");
  }
  ingestOdometryIntoGlobalGraph();
}

void PlannerNode::onPointCloud(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  if (msg->data.empty() || cloud_map_ == nullptr) return;

  // Transform into world coordinates before projecting into the octree.
  // The sensor publishes in its own frame (e.g. "r0/lidar") and the map lives
  // in world_frame_.
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(
        world_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(cloud_tf_timeout_sec_));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "cannot transform point cloud from '%s' to '%s': %s",
                         msg->header.frame_id.c_str(), world_frame_.c_str(),
                         ex.what());
    return;
  }

  const Eigen::Vector3d origin(tf_msg.transform.translation.x,
                               tf_msg.transform.translation.y,
                               tf_msg.transform.translation.z);
  const Eigen::Quaterniond rot(
      tf_msg.transform.rotation.w, tf_msg.transform.rotation.x,
      tf_msg.transform.rotation.y, tf_msg.transform.rotation.z);

  std::vector<Eigen::Vector3d> points;
  points.reserve(msg->width * msg->height);
  sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    // Non-finite entries are normal in organised clouds (no return on that
    // ray) and would otherwise poison the octree bounds.
    if (!std::isfinite(*it_x) || !std::isfinite(*it_y) ||
        !std::isfinite(*it_z)) {
      continue;
    }
    points.emplace_back(rot * Eigen::Vector3d(*it_x, *it_y, *it_z) + origin);
  }
  if (!points.empty()) {
    const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
#ifdef MGG_WITH_OCTOMAP
    cloud_map_->insertPointCloud(points, origin);
    ++map_revision_;
#endif
  }
}

void PlannerNode::onMappingSnapshot(
    mgg_msgs::msg::MappingSnapshot::ConstSharedPtr msg) {
  static const std::regex kDigest("^[0-9a-fA-F]{64}$");
  const auto& t = msg->component_from_navigation.translation;
  const auto& q = msg->component_from_navigation.rotation;
  const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (msg->component_id.empty() ||
      !std::regex_match(msg->geometry_revision, kDigest) ||
      msg->source_stamp.sec < 0 || msg->source_stamp.nanosec >= 1000000000u ||
      !std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) ||
      !std::isfinite(q_norm) || std::abs(q_norm - 1.0) > 1e-4) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "ignoring invalid mapping snapshot");
    return;
  }
  Eigen::Isometry3d component_from_navigation = Eigen::Isometry3d::Identity();
  component_from_navigation.linear() =
      Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
  component_from_navigation.translation() = Eigen::Vector3d(t.x, t.y, t.z);
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    const auto active = mola_map_->activeRequest();
    const bool changed = latest_snapshot_
        ? latest_snapshot_->component_id != msg->component_id || latest_snapshot_->epoch != msg->epoch
        : active && (active->component_id != msg->component_id || active->epoch != msg->epoch);
    if (changed) {
      bootstrap_started_ns_ = 0;
      setAcquiringObservations(true);
    }
    latest_snapshot_ = msg;
  }
  ++heartbeats_received_;
  if (request_active_.load()) ++heartbeats_during_planning_;
  const std::string prior_error = mola_map_->lastError();
  if (!prior_error.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "MOLA map unavailable: %s", prior_error.c_str());
  }
  const std::uint64_t source_stamp_ns =
      static_cast<std::uint64_t>(msg->source_stamp.sec) * 1000000000ull +
      msg->source_stamp.nanosec;
  std::lock_guard<std::mutex> fence(cancellation_mutex_);
  mola_map_->requestSnapshot({msg->component_id, msg->epoch,
                              msg->graph_revision, msg->geometry_revision,
                              source_stamp_ns, component_from_navigation});
}

void PlannerNode::onCoordinationExclusions(
    geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
  if (msg->header.frame_id != world_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "ignoring coordination exclusions in frame '%s'",
                         msg->header.frame_id.c_str());
    return;
  }
  std::vector<Eigen::Vector3d> centers;
  centers.reserve(msg->poses.size());
  for (const auto& pose : msg->poses) {
    const auto& p = pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "ignoring invalid coordination exclusions");
      return;
    }
    centers.emplace_back(p.x, p.y, p.z);
  }
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  coordination_exclusions_ = std::move(centers);
  coordination_exclusions_received_ = std::chrono::steady_clock::now();
  have_coordination_exclusions_ = true;
}

void PlannerNode::onPeerBodies(
    geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
  if (mola_map_ == nullptr || peer_body_radius_m_ <= 0.0) return;
  if (msg->header.frame_id != world_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "ignoring peer bodies in frame '%s'",
                         msg->header.frame_id.c_str());
    return;
  }
  std::vector<Eigen::Vector2d> centres;
  centres.reserve(msg->poses.size());
  for (const auto& pose : msg->poses) {
    if (std::isfinite(pose.position.x) && std::isfinite(pose.position.y)) {
      centres.emplace_back(pose.position.x, pose.position.y);
    }
  }
  // Published between requests, never during one: a plan or objective
  // request plans with one peer set throughout (pinPeerBodies), review r0,
  // I5. The tour's route costs follow the set through the peer generation
  // (refreshPeerGeneration), not the graph revision.
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (robot_params_.type == mgg::RobotType::kAerialRobot && have_aerial_peer_bodies_) return;
  mola_map_->setTransientDiscs(std::move(centres), peer_body_radius_m_,
                               peer_body_ttl_s_);
}

void PlannerNode::refreshPeerGeneration() {
  if (robot_params_.type == mgg::RobotType::kAerialRobot) {
    std::vector<std::array<double, 4>> key;
    for (const auto& body : activeAerialPeerBodies()) {
      key.push_back({std::round(body.top.x() / kPeerGenerationCellM),
                     std::round(body.top.y() / kPeerGenerationCellM),
                     std::round(body.top.z() / kPeerGenerationCellM),
                     std::round(body.radius / kPeerGenerationCellM)});
    }
    std::sort(key.begin(), key.end());
    key.erase(std::unique(key.begin(), key.end()), key.end());
    if (key != aerial_peer_generation_key_) {
      aerial_peer_generation_key_ = std::move(key);
      ++peer_generation_;
    }
  }
  if (mola_map_ == nullptr) return;
  // The set in force: a request's pinned one, or, outside a request, the
  // one published and not expired.
  const mgg::MolaMap::TransientDiscSet peers =
      mola_map_->activeTransientDiscs();
  std::vector<std::array<long, 2>> key;
  key.reserve(peers.centres.size() + 1);
  for (const Eigen::Vector2d& centre : peers.centres) {
    key.push_back({std::lround(centre.x() / kPeerGenerationCellM),
                   std::lround(centre.y() / kPeerGenerationCellM)});
  }
  std::sort(key.begin(), key.end());
  if (!key.empty()) {
    key.push_back({std::lround(peers.radius_m / kPeerGenerationCellM), 0});
  }
  if (key == peer_generation_key_) return;
  peer_generation_key_ = std::move(key);
  ++peer_generation_;
}

void PlannerNode::onNoGoZones(
    geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
  if (msg->header.frame_id != world_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "ignoring no-go zones in frame '%s'",
                         msg->header.frame_id.c_str());
    return;
  }
  std::vector<Eigen::Vector2d> centres;
  centres.reserve(msg->poses.size());
  for (const auto& pose : msg->poses) {
    if (std::isfinite(pose.position.x) && std::isfinite(pose.position.y)) {
      centres.emplace_back(pose.position.x, pose.position.y);
    }
  }
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (mola_map_ != nullptr) {
    mola_map_->setNoGoDiscs(centres, planning_params_.no_go_radius_m);
  }
  if (centres != no_go_zones_) {
    no_go_zones_ = std::move(centres);
    refreshNoGoZones();
    // Route costs change with the edges the searches leave out.
    ++graph_revision_;
    RCLCPP_INFO(get_logger(), "%zu no-go zone(s) of %.2f m radius",
                no_go_zones_.size(), planning_params_.no_go_radius_m);
  }
}

void PlannerNode::onNoGoDiscs(
    geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
  if (msg->header.frame_id != world_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "ignoring no-go discs in frame '%s' (expected '%s')",
                         msg->header.frame_id.c_str(), world_frame_.c_str());
    return;
  }
  std::vector<Eigen::Vector2d> centres;
  std::vector<double> reaches;
  for (const auto& pose : msg->poses) {
    const auto& p = pose.position;
    // Reject the whole replacement, never silently drop a malformed margin.
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        !std::isfinite(p.z) || p.z <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "invalid no-go disc: need finite XY and a positive "
                           "finite centre-line reach in position.z; keeping previous set");
      return;
    }
    centres.emplace_back(p.x, p.y);
    reaches.push_back(p.z);
  }
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (mola_map_ != nullptr) mola_map_->setNoGoCentreLineDiscs(centres, reaches);
  if (centres != no_go_disc_centres_ || reaches != no_go_disc_reaches_) {
    no_go_disc_centres_ = std::move(centres);
    no_go_disc_reaches_ = std::move(reaches);
    refreshNoGoZones();
    ++graph_revision_;
  }
}

void PlannerNode::onScoutingExclusions(
    geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (msg->header.frame_id != world_frame_) {
    ++scouting_counters_.messages_wrong_frame;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "ignoring scouting exclusions in frame '%s' (expected "
                         "'%s'); keeping the set in force",
                         msg->header.frame_id.c_str(), world_frame_.c_str());
    return;
  }
  std::vector<Eigen::Vector2d> centres;
  std::vector<double> reaches;
  for (const auto& pose : msg->poses) {
    const auto& p = pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        p.z <= 0.0) {
      ++scouting_counters_.messages_invalid;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "invalid scouting exclusion: need finite XY and a "
                           "positive finite centre-line reach in position.z; "
                           "keeping the set in force");
      return;
    }
    centres.emplace_back(p.x, p.y);
    reaches.push_back(p.z);
  }
  ++scouting_counters_.messages_accepted;
  scouting_exclusions_received_ = std::chrono::steady_clock::now();
  if (centres == scouting_exclusion_centres_ &&
      reaches == scouting_exclusion_reaches_) {
    return;
  }
  scouting_exclusion_centres_ = std::move(centres);
  scouting_exclusion_reaches_ = std::move(reaches);
  scouting_zones_.set(scouting_exclusion_centres_, scouting_exclusion_reaches_);
  // The tour's candidates change: solve again.
  ++tour_assignment_version_;
  publishScoutingRevision();
  RCLCPP_INFO(get_logger(), "%zu scouting exclusion(s)",
              scouting_exclusion_centres_.size());
}

void PlannerNode::publishScoutingRevision() {
  if (robot_params_.type != mgg::RobotType::kAerialRobot) return;
  std_msgs::msg::UInt64 revision;
  revision.data = ++scouting_revision_;
  scouting_revision_pub_->publish(revision);
}

void PlannerNode::refreshScoutingExclusions() {
  if (scouting_exclusion_centres_.empty() ||
      secondsSince(scouting_exclusions_received_) <= scouting_exclusion_ttl_s_) {
    return;
  }
  scouting_exclusion_centres_.clear();
  scouting_exclusion_reaches_.clear();
  scouting_zones_.set({}, std::vector<double>{});
  ++scouting_counters_.lapsed;
  ++tour_assignment_version_;
  publishScoutingRevision();
  RCLCPP_WARN(get_logger(),
              "scouting exclusions lapsed: none received for %.1f s",
              scouting_exclusion_ttl_s_);
}

bool PlannerNode::scoutingPathAdmissible(const std::vector<mgg::StateVec>& path) {
  if (scouting_zones_.empty() || path.empty()) return true;
  std::vector<Eigen::Vector3d> points;
  points.reserve(path.size());
  for (const mgg::StateVec& state : path) points.push_back(state.head<3>());
  if (scouting_zones_.pathAdmissible(points)) return true;
  ++scouting_counters_.paths_refused;
  return false;
}

bool PlannerNode::scoutingExcludes(const Eigen::Vector3d& p) const {
  return !scouting_zones_.empty() && scouting_zones_.inside(p);
}

void PlannerNode::refreshNoGoZones() {
  // A centre line kept out of the reach keeps the body out of the disc.
  const Eigen::Vector3d box = robot_params_.getPlanningSize();
  auto centres = no_go_zones_;
  std::vector<double> reaches(centres.size(), planning_params_.no_go_radius_m +
                               0.5 * std::max(box.x(), box.y()));
  centres.insert(centres.end(), no_go_disc_centres_.begin(), no_go_disc_centres_.end());
  reaches.insert(reaches.end(), no_go_disc_reaches_.begin(), no_go_disc_reaches_.end());
  no_go_.set(std::move(centres), std::move(reaches));
}

bool PlannerNode::noGoAdmissible(const std::vector<mgg::StateVec>& path) {
  refreshNoGoZones();
  if (no_go_.empty() || path.empty()) return true;
  std::vector<Eigen::Vector3d> points;
  points.reserve(path.size());
  for (const mgg::StateVec& state : path) points.push_back(state.head<3>());
  return no_go_.pathAdmissible(points);
}

bool PlannerNode::noGoBlocksEdge(const mgg::Vertex& a,
                                 const mgg::Vertex& b) const {
  return noGoBlocksSegment(a.state.head<3>(), b.state.head<3>());
}

bool PlannerNode::noGoBlocksSegment(const Eigen::Vector3d& from,
                                    const Eigen::Vector3d& to) const {
  return !no_go_.empty() &&
         no_go_.blocksEdge(from, to, current_state_.head<3>());
}

bool PlannerNode::peerBlocksSegment(const Eigen::Vector3d& from,
                                    const Eigen::Vector3d& to) const {
  return peerBlockingSegment(from, to) != PeerBlock::kNone;
}

PlannerNode::PeerBlock PlannerNode::peerBlockingSegment(
    const Eigen::Vector3d& from, const Eigen::Vector3d& to) const {
  if (peer_edges_open_) return PeerBlock::kNone;
  if (robot_params_.type == mgg::RobotType::kAerialRobot) {
    const auto box = robot_params_.getPlanningSize();
    const Eigen::Vector3d start = from + robot_params_.center_offset;
    const Eigen::Vector3d end = to + robot_params_.center_offset;
    for (const auto& body : activeAerialPeerBodies()) {
      const double ceiling = body.top.z() + aerial_peer_margin_m_ + .5 * box.z();
      if (std::min(start.z(), end.z()) > ceiling) continue;
      // Clip to the portion at/below the top. The XY closest point on that
      // interval gives an exact cylinder sweep, not endpoint sampling.
      double lo = 0, hi = 1;
      if (start.z() > ceiling) lo = (ceiling - start.z()) / (end.z() - start.z());
      if (end.z() > ceiling) hi = (ceiling - start.z()) / (end.z() - start.z());
      const Eigen::Vector2d a = (start + lo * (end - start)).head<2>() - body.top.head<2>();
      const Eigen::Vector2d delta = ((hi - lo) * (end - start)).head<2>();
      const double t = delta.squaredNorm() > 0
          ? std::clamp(-a.dot(delta) / delta.squaredNorm(), 0.0, 1.0) : 0;
      const double reach = body.radius + aerial_peer_margin_m_ + .5 * box.head<2>().norm();
      const Eigen::Vector2d at_start = start.head<2>() - body.top.head<2>();
      const Eigen::Vector2d travel = end.head<2>() - start.head<2>();
      // The cylinder includes the peer's physical body, which can be
      // masked out of lidar. An inside start may exit monotonically away
      // or climb, but must never descend into that body. A point query
      // still reports the volume as blocked.
      if (start.z() <= ceiling && at_start.norm() <= reach &&
          end.z() >= start.z() - 1e-9 &&
          ((travel.squaredNorm() > 1e-12 && at_start.dot(travel) >= -1e-9) ||
           (travel.squaredNorm() <= 1e-12 && end.z() > start.z() + 1e-9))) continue;
      if ((a + t * delta).norm() <= reach) {
        ++aerial_counters_.cylinder_segment_blocks;
        return PeerBlock::kAerialCylinder;
      }
    }
  }
  if (mola_map_ == nullptr) return PeerBlock::kNone;
  // Where a roadmap edge is checked (global_graph.cpp edgeStatus): the
  // body's centre, half its planning box across.
  const Eigen::Vector3d box = robot_params_.getPlanningSize();
  if (!mola_map_->transientDiscsBlockSweep(
          from + robot_params_.center_offset, to + robot_params_.center_offset,
          0.5 * std::max(box.x(), box.y()))) {
    return PeerBlock::kNone;
  }
  ++aerial_counters_.disc_segment_blocks;
  return PeerBlock::kLegacyDisc;
}

void PlannerNode::onAerialPeerBodies(geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (robot_params_.type != mgg::RobotType::kAerialRobot) return;
  if (msg->header.frame_id != world_frame_) {
    ++aerial_counters_.cylinder_messages_wrong_frame;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "ignoring aerial peer bodies in the wrong frame");
    return;
  }
  std::vector<AerialPeerBody> bodies;
  for (const auto& pose : msg->poses) {
    const auto& p = pose.position;
    const auto& q = pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q.x) || q.x <= 0 || q.y != 0 || q.z != 0 || q.w != 0) {
      ++aerial_counters_.cylinder_messages_invalid;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "invalid aerial peer cylinder: need finite XY/top, positive radius, zero y/z/w marker; keeping previous set");
      return;
    }
    bodies.push_back({Eigen::Vector3d(p.x, p.y, p.z), q.x});
  }
  aerial_peer_bodies_ = std::move(bodies);
  aerial_peer_bodies_received_ = std::chrono::steady_clock::now();
  have_aerial_peer_bodies_ = true;
  ++aerial_counters_.cylinder_messages_accepted;
  aerial_counters_.cylinder_frame = msg->header.frame_id;
  // Once the spec-aware input is available, the legacy unbounded XY discs
  // must not prevent a drone from flying safely above a peer.
  if (mola_map_) mola_map_->setTransientDiscs({}, 0, peer_body_ttl_s_);
  refreshPeerGeneration();
}

std::vector<PlannerNode::AerialPeerBody> PlannerNode::activeAerialPeerBodies() const {
  if (pinned_aerial_peer_bodies_) return *pinned_aerial_peer_bodies_;
  if (!have_aerial_peer_bodies_ || secondsSince(aerial_peer_bodies_received_) > peer_body_ttl_s_) return {};
  return aerial_peer_bodies_;
}

PlannerNode::PeerBodyPin::PeerBodyPin(PlannerNode& owner) : node(owner) {
  auto bodies = node.activeAerialPeerBodies();
  previous = std::move(node.pinned_aerial_peer_bodies_);
  node.pinned_aerial_peer_bodies_ = std::move(bodies);
  if (node.mola_map_) {
    auto peers = node.mola_map_->activeTransientDiscs();
    ground.emplace(*node.mola_map_, std::move(peers.centres), peers.radius_m);
  }
}

PlannerNode::PeerBodyPin::~PeerBodyPin() {
  node.pinned_aerial_peer_bodies_ = std::move(previous);
}

void PlannerNode::pinPeerBodies(std::optional<PeerBodyPin>& pin) {
  pin.emplace(*this);
}

bool PlannerNode::peerAdmissible(const std::vector<mgg::StateVec>& path) const {
  if (robot_params_.type == mgg::RobotType::kAerialRobot && !path.empty()) {
    const Eigen::Vector3d current = current_state_.head<3>();
    const Eigen::Vector3d front = path.front().head<3>();
    const double gap_squared = (front - current).squaredNorm();
    if (gap_squared <= mgg::kDeltaLimit * mgg::kDeltaLimit) {
      // Roadmap linking may snap to a nearby vertex. The unrepresented
      // hop must obey the same outward, non-descending exit rule. An
      // identical front needs no hop (a point query would block it).
      if (gap_squared > 0.0 && peerBlocksSegment(current, front)) return false;
    } else if (peerBlocksSegment(front, front)) {
      return false;
    }
  }
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (peerBlocksSegment(path[i - 1].head<3>(), path[i].head<3>())) {
      return false;
    }
  }
  return true;
}

bool PlannerNode::peersInForce() const {
  if (robot_params_.type == mgg::RobotType::kAerialRobot && !activeAerialPeerBodies().empty()) return true;
  return mola_map_ != nullptr &&
         !mola_map_->activeTransientDiscs().centres.empty();
}

bool PlannerNode::diagnosePeerSearch(int source_id,
                                     mgg::ShortestPathsReport& rep) {
  ++peer_diagnoses_;
  const auto deadline = std::min(
      peer_diagnosis_deadline_.value_or(std::chrono::steady_clock::time_point::max()),
      innerDeadline(lattice_deadline_, planning_params_.global_search_time_budget_s));
  const FlagScope diagnosing(peer_diagnosis_in_progress_);
  const FlagScope open(peer_edges_open_);
  rep = mgg::ShortestPathsReport();
  global_graph_->findShortestPaths(source_id, rep, deadline);
  if (rep.cut_short) peer_diagnosis_cut_short_ = true;
  return !rep.cut_short;
}

bool PlannerNode::globalEdgeBlocked(const mgg::Vertex& a,
                                    const mgg::Vertex& b) {
  if (noGoBlocksEdge(a, b)) return true;
  // Run 10b: roadmap edges laid before a peer parked on them routed
  // robot_1 and robot_3 through robot_2. Closed for the search only, as a
  // no-go zone's are: the peer moves on, and the roadmap keeps the edge.
  const PeerBlock block = peerBlockingSegment(a.state.head<3>(), b.state.head<3>());
  if (block == PeerBlock::kNone) return false;
  peer_blocked_edges_.insert(std::minmax(a.id, b.id));
  if (block == PeerBlock::kAerialCylinder) {
    peer_cylinder_blocked_edges_.insert(std::minmax(a.id, b.id));
  }
  return true;
}

void PlannerNode::onNeighbourTransforms(
    tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  const auto now = std::chrono::steady_clock::now();
  for (const auto& t : msg->transforms) {
    if (t.header.frame_id != world_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "ignoring neighbour transform from '%s', not our "
                           "planning frame '%s'",
                           t.header.frame_id.c_str(), world_frame_.c_str());
      continue;
    }
    const auto& p = t.transform.translation;
    const auto& q = t.transform.rotation;
    const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !rotation.coeffs().allFinite() ||
        std::abs(rotation.norm() - 1.0) > 1e-3) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "ignoring an invalid neighbour transform to '%s'",
                           t.child_frame_id.c_str());
      continue;
    }
    Eigen::Isometry3d t_ours_theirs = Eigen::Isometry3d::Identity();
    t_ours_theirs.linear() = rotation.normalized().toRotationMatrix();
    t_ours_theirs.translation() = Eigen::Vector3d(p.x, p.y, p.z);
    neighbour_transforms_[t.child_frame_id] = {t_ours_theirs, now};
  }
  readmitQuarantinedNeighbours();
  if (robot_params_.type == mgg::RobotType::kAerialRobot) liftedPeerFrontiers();
}

void PlannerNode::readmitQuarantinedNeighbours() {
  if (roadmaps_to_readmit_.empty()) return;
  for (auto it = roadmaps_to_readmit_.begin();
       it != roadmaps_to_readmit_.end();) {
    const int robot = *it;
    const auto frame = neighbour_frames_.find(robot);
    const auto roadmap = neighbour_roadmaps_.find(robot);
    if (frame == neighbour_frames_.end() ||
        roadmap == neighbour_roadmaps_.end()) {
      // Nothing cached to merge again: the next roadmap received merges.
      ++it;
      continue;
    }
    if (!refreshNeighbourTransform(robot, frame->second)) {
      ++it;
      continue;
    }
    auto map_read = mapReadLease();
    const mgg::MergeResult r = mergeNeighbourRoadmap(roadmap->second);
    if (!r.merged || global_graph_->isQuarantined(robot)) {
      ++it;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
                           "robot %d's transform returned, but its roadmap "
                           "joins ours nowhere yet: it stays quarantined",
                           robot);
    } else {
      it = roadmaps_to_readmit_.erase(it);
      RCLCPP_INFO(get_logger(),
                  "robot %d's transform returned: its quarantined roadmap is "
                  "merged again (+%d edges)",
                  robot, r.edges_added);
    }
  }
}

std::string PlannerNode::completionWithheld() const {
  if (!fleet_ && robot_params_.type == mgg::RobotType::kAerialRobot &&
      aerial_fallback_remains_) {
    return "eligible forward or unknown-progress peer frontiers remain";
  }
  const std::size_t outstanding =
      std::max<std::size_t>(roadmaps_to_readmit_.size(),
                            static_cast<std::size_t>(
                                global_graph_->numQuarantined()));
  if (outstanding > 0) {
    return std::to_string(outstanding) +
           " neighbour roadmap(s) quarantined until their transforms return";
  }
  if (global_search_cut_short_) {
    return "the global search was cut short by its time budget";
  }
  if (global_frontier_not_routed_) {
    return "the global search found a frontier it could not route to";
  }
  // Retention's veto on a global route counts only while that target's
  // exclusion lives: a given-up target is unreachable from here, and a
  // lapsed one is back in the search (run14 review r3).
  if (global_target_refused_by_retention_ &&
      std::any_of(reverse_exit_exclusions_.begin(), reverse_exit_exclusions_.end(),
                  [this](const ReverseExitExclusion& exclusion) {
                    return !exclusion.given_up && reverseExitExclusionActive(exclusion) &&
                           (exclusion.position - *global_target_refused_by_retention_).norm() <=
                               reach_distance_ + mgg::kViewpointArrivalSlack;
                  })) {
    return "the route to its global frontier was refused a reverse exit";
  }
  return "";
}

bool PlannerNode::refreshNeighbourTransform(int sender,
                                            const std::string& sender_frame) {
  if (neighbour_pose_source_ != "topic") return true;
  const auto found = neighbour_transforms_.find(sender_frame);
  if (found == neighbour_transforms_.end() ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    found->second.received)
              .count() > neighbour_transform_ttl_s_) {
    poses_->clearTransform(sender);
    return false;
  }
  poses_->setTransform(sender, found->second.t_ours_theirs);
  return true;
}

void PlannerNode::withdrawUnplacedNeighbours() {
  if (robot_params_.type == mgg::RobotType::kAerialRobot) liftedPeerFrontiers();
  for (const auto& [robot, frame] : neighbour_frames_) {
    if (refreshNeighbourTransform(robot, frame)) continue;
    if (global_graph_->isQuarantined(robot) ||
        global_graph_->vertex_by_robot_id_.count(robot) == 0) {
      continue;
    }
    // Its roadmap was placed with a transform that is no longer current,
    // whether or not it is joined to ours: quarantined until a current one
    // places it again.
    const int cut = global_graph_->disconnectNeighbourGraph(robot);
    roadmaps_to_readmit_.insert(robot);
    ++graph_revision_;
    RCLCPP_INFO(get_logger(),
                "robot %d's transform was withdrawn: its roadmap is "
                "quarantined (%d edges cut) until it is placed again",
                robot, cut);
  }
}

mgg::ReceiverPlatform PlannerNode::receiverPlatform() const {
  mgg::ReceiverPlatform platform;
  platform.type = robot_params_.type;
  if (robot_params_.type == mgg::RobotType::kGroundRobot) {
    platform.driving_height = planning_params_.max_ground_height;
    platform.max_step_height = planning_params_.max_step_height;
    platform.max_inclination = planning_params_.max_inclination;
  }
  return platform;
}

mgg::MergeResult PlannerNode::mergeNeighbourRoadmap(
    const mgg::GraphExchange& incoming) {
  if (home_state_wait_started_) return {};
  const mgg::ExpandContext ctx = makeContext(false);
  const bool aerial = robot_params_.type == mgg::RobotType::kAerialRobot;
  Eigen::Isometry3d peer_transform = Eigen::Isometry3d::Identity();
  int sender = -1;
  if (aerial && !incoming.vertices.empty()) {
    refreshMapRevision();
    sender = incoming.vertices.front().robot_id;
    if (poses_->getRobotTransform(sender, peer_transform)) {
      const auto cached = aerial_merge_cache_.find(sender);
      if (cached != aerial_merge_cache_.end()) {
        const auto& old = cached->second;
        if (old.graph.lock() == global_graph_ && old.map == map_.get() &&
            old.map_revision == map_revision_ && !global_graph_->isQuarantined(sender) &&
            old.transform.matrix().isApprox(peer_transform.matrix(), 0.0) &&
            old.body_size == ctx.robot_box_size && old.center_offset == robot_params_.center_offset &&
            sameRoadmapSnapshot(old.snapshot, incoming)) {
          mgg::MergeResult unchanged;
          unchanged.merged = old.merged;
          return unchanged;
        }
      }
    }
  }
  // Peers occupy the roadmap only at search time, as for keyframe rebuilds.
  std::optional<mgg::MolaMap::TransientDiscPin> no_peers;
  if (aerial && mola_map_ != nullptr) {
    no_peers.emplace(*mola_map_, std::vector<Eigen::Vector2d>{}, 0.0);
  }
  // The merge asks whether the robot could actually drive between two graphs
  // before joining them; that judgement needs the map, so it is injected.
  const auto admissible = [this, &ctx](const Eigen::Vector3d& from,
                                       const Eigen::Vector3d& to) {
    if (robot_params_.type == mgg::RobotType::kAerialRobot) {
      return map_->getStaticStrictPathStatus(from + robot_params_.center_offset,
                                              to + robot_params_.center_offset,
                                              ctx.robot_box_size) ==
             mgg::VoxelStatus::kFree;
    }
    std::vector<Eigen::Vector3d> projected;
    return ground_->getProjectedEdgeStatus(from, to, ctx.robot_box_size, true,
                                           projected, false) ==
           mgg::ProjectedEdgeStatus::kAdmissible;
  };

  const int edges_before = global_graph_->getNumEdges();
  const mgg::MergeResult r = mgg::mergeNeighbourGraph(
      *global_graph_, incoming, *poses_, admissible, 5.0, receiverPlatform());
  if (r.vertices_added > 0 || global_graph_->getNumEdges() != edges_before ||
      r.vertices_replaced > 0 ||
      r.neighbour_restarted) {
    ++graph_revision_;
  }
  if (aerial && sender >= 0 && !r.transform_unavailable &&
      global_graph_->neighbour_placements_.count(sender) > 0) {
    aerial_merge_cache_[sender] = {incoming, global_graph_, map_.get(), map_revision_,
                                   peer_transform, ctx.robot_box_size,
                                   robot_params_.center_offset, r.merged};
  }
  return r;
}

void PlannerNode::onNeighbourGraph(mgg_msgs::msg::Graph::ConstSharedPtr msg) {
  if (msg->vertices.empty()) return;
  const int sender = msg->vertices.front().robot_id;
  if (sender == static_cast<int>(planning_params_.robot_id)) return;

  // Merging reads the map to decide reachability and writes the global graph.
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  auto map_read = mapReadLease();
  // The graph is in the sender's planning frame, which names the transform.
  neighbour_frames_[sender] = msg->header.frame_id;
  withdrawUnplacedNeighbours();

  // Communication range filter: robots must be within direct radio range.
  Eigen::Isometry3d t_ours_theirs = Eigen::Isometry3d::Identity();
  if (poses_->getRobotTransform(sender, t_ours_theirs)) {
    const auto& last_v = msg->vertices.back();
    const Eigen::Vector3d their_latest(last_v.pose.position.x,
                                       last_v.pose.position.y,
                                       last_v.pose.position.z);
    const Eigen::Vector3d their_world = t_ours_theirs * their_latest;
    const double dist = (their_world - current_state_.head<3>()).norm();
    if (communication_range_ > 0.0 && dist > communication_range_) {
      return;
    }
  }

  const mgg::GraphExchange incoming = fromGraphMsg(*msg);
  neighbour_roadmaps_[sender] = incoming;
  if (robot_params_.type == mgg::RobotType::kAerialRobot) {
    aerial_peer_received_[sender] = std::chrono::steady_clock::now();
  }
  // Ground coordinates cannot be aerial rendezvous. Keep their evidence as
  // targets only; never import a sender's edges into the aerial roadmap.
  if (robot_params_.type == mgg::RobotType::kAerialRobot) {
    liftedPeerFrontiers();
    return;
  }
  const mgg::MergeResult r = mergeNeighbourRoadmap(incoming);
  if (r.merged && !global_graph_->isQuarantined(sender)) {
    roadmaps_to_readmit_.erase(sender);
  }

  if (r.transform_unavailable) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "roadmap from robot %d ('%s') dropped: no %s "
                         "transform to it",
                         sender, msg->header.frame_id.c_str(),
                         neighbour_pose_source_ == "topic"
                             ? "current neighbour_transforms"
                             : "neighbour_offsets");
    return;
  }
  if (r.neighbour_restarted) {
    RCLCPP_INFO(get_logger(),
                "robot %d restarted its roadmap; the old one was cut out",
                sender);
  }
  if (r.vertices_replaced > 0) {
    RCLCPP_INFO(get_logger(),
                "robot %d's transform moved: %d merged vertices re-placed",
                sender, r.vertices_replaced);
  }
  if (r.edges_too_steep > 0) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
                         "robot %d: %d roadmap edge(s) beyond this robot's "
                         "step and grade limits not taken over",
                         sender, r.edges_too_steep);
  }
  if (r.newly_connected) {
    if (poses_->getRobotTransform(sender, t_ours_theirs) &&
        !incoming.vertices.empty()) {
      const Eigen::Vector3d their_world =
          t_ours_theirs * incoming.vertices.front().state.head<3>();
      recent_merges_.push_back(
          {now(), sender, current_state_.head<3>(), their_world});
    }
    publishMarkers();
    RCLCPP_INFO(get_logger(),
                "*** Swarm Graph Merge: Connected with robot %d (+%d vertices, +%d edges) ***",
                sender, r.vertices_added, r.edges_added);
  } else if (r.vertices_added > 0) {
    publishMarkers();
    RCLCPP_INFO(get_logger(),
                "roadmap update from robot %d: +%d new vertices, +%d edges",
                sender, r.vertices_added, r.edges_added);
  }
}

// ---------------------------------------------------------------------------
// The global graph between cycles

void PlannerNode::seedGlobalGraph() {
  if (!have_odometry_) return;
  if (global_graph_->getNumVertices() == 0) {
    if (robot_params_.type == mgg::RobotType::kAerialRobot &&
        aerial_home_height_m_ > 0.0 && !latest_flight_state_ &&
        !home_seeded_) {
      const auto now_time = now();
      if (!home_state_wait_started_ ||
          home_state_wait_started_->nanoseconds() == 0) {
        home_state_wait_started_ = now_time;
        // Before the first /clock, zero is not a deadline anchor. Either
        // odometry or the timer can first see valid time: re-arm the timer
        // from that non-zero anchor, rather than let its old expiry win.
        if (now_time.nanoseconds() != 0 && home_state_wait_timer_) {
          home_state_wait_timer_->reset();
        }
      }
      if ((now_time - *home_state_wait_started_).seconds() <
          aerial_home_state_wait_s_) {
        if (!home_state_wait_timer_ || home_state_wait_timer_->is_canceled()) {
          home_state_wait_timer_ = create_timer(
              std::chrono::duration<double>(aerial_home_state_wait_s_),
              [this]() {
                const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
                auto map_read = mapReadLease();
                seedGlobalGraph();
              }, callback_group_);
        }
        return;
      }
      RCLCPP_WARN(get_logger(),
                  "timed out waiting for %s after %.2f s; seeding home at "
                  "the pose, unlifted",
                  flight_state_sub_->get_topic_name(), aerial_home_state_wait_s_);
    }
    home_state_wait_started_.reset();
    if (home_state_wait_timer_) home_state_wait_timer_->cancel();
    // A lone root is a landmark, not a traversability claim: capture it from
    // the current odometry (after the bounded flight_state wait, if needed).
    // Until the map shows ground under it no edge attaches to it.
    // A drone SwarmDeck reports landed stands on its pad, where its box
    // meets the floor and no edge joins it: home is aerial_home_height_m
    // over it, where it takes off to. Without that report (timed out, or a
    // planner restarted in flight) home is where the drone is.
    // That is decided at the first seed only. A later one follows a map
    // reset (another component or epoch), usually in flight: vertex 0 is
    // where the drone is until the keyframe rebuild puts home back,
    // and that rebuild still lifts the pad its keyframes start on. Taking
    // the in-flight state then put home back on the pad, unlifted, its
    // box in the floor (mgg-flown evidence, robot_4).
    const bool first_seed = !home_seeded_;
    if (first_seed) {
      home_seeded_landed_ =
          robot_params_.type == mgg::RobotType::kAerialRobot &&
          aerial_home_height_m_ > 0.0 &&
          latest_flight_state_ == std::string(kFlightStateLanded);
      home_seeded_on_timeout_ =
          robot_params_.type == mgg::RobotType::kAerialRobot &&
          aerial_home_height_m_ > 0.0 && !latest_flight_state_;
      home_seed_pose_ = current_state_;
    } else {
      home_seeded_on_timeout_ = false;
    }
    home_seeded_ = true;
    mgg::StateVec root_state = current_state_;
    if (first_seed && home_seeded_landed_) {
      root_state[2] += aerial_home_height_m_;
    }
    global_root_supported_ = projectToDrivingHeight(root_state);
    if (!global_root_supported_) {
      root_state = physicalAnchorAtDrivingHeight(current_state_);
    }
    auto* root = new mgg::Vertex(0, root_state);
    root->robot_id = static_cast<int>(planning_params_.robot_id);
    root->type = mgg::VertexType::kVisited;
    root->is_hanging = !global_root_supported_;
    global_graph_->addVertex(root);
    last_state_marker_ = current_state_;
    last_state_marker_global_ = current_state_;
    ++graph_revision_;
    RCLCPP_INFO(get_logger(),
                "global graph seeded at (%.2f, %.2f, %.2f)%s%s", root_state[0],
                root_state[1], root_state[2],
                global_root_supported_ ? "" : " (awaiting mapped support)",
                !first_seed ? " (after a map reset)"
                : home_seeded_landed_
                    ? " (landed: aerial_home_height_m over the pad)"
                    : "");
    return;
  }
  if (global_root_supported_ || !map_->getStatus()) return;
  // The home coordinate stays put; only its height follows the ground once
  // the ground is seen.
  auto root_it = global_graph_->vertices_map_.find(0);
  if (root_it == global_graph_->vertices_map_.end() ||
      root_it->second == nullptr) {
    return;
  }
  mgg::StateVec supported_root = root_it->second->state;
  if (!projectToDrivingHeight(supported_root)) return;
  if (!global_graph_->updateVertexState(0, supported_root)) return;
  root_it->second->is_hanging = false;
  global_root_supported_ = true;
  ++graph_revision_;
  RCLCPP_INFO(get_logger(),
              "global graph root support observed at (%.2f, %.2f, %.2f)",
              supported_root[0], supported_root[1], supported_root[2]);
}

std::size_t PlannerNode::ownGlobalVertices() const {
  const auto own = global_graph_->vertex_by_robot_id_.find(
      static_cast<int>(planning_params_.robot_id));
  return own == global_graph_->vertex_by_robot_id_.end() ? 0
                                                          : own->second.size();
}

bool PlannerNode::globalGraphReaches(const mgg::StateVec& state) const {
  std::vector<mgg::Vertex*> near;
  mgg::StateVec query = state;
  if (!global_graph_->getNearestVertices(
          &query, planning_params_.edge_length_max, &near)) {
    return false;
  }
  const int own = static_cast<int>(planning_params_.robot_id);
  return std::any_of(near.begin(), near.end(),
                     [this, own](const mgg::Vertex* vertex) {
                       return vertex != nullptr && vertex->robot_id == own &&
                              global_graph_->inService(*vertex);
                     });
}

bool PlannerNode::rebuildGlobalGraphFromKeyframes(
    RoadmapRebuildTrigger trigger, const char* why,
    const std::function<mgg::Vertex*(mgg::GraphManager&)>& link_what_failed) {
  if (home_state_wait_started_ || keyframe_source_ == nullptr ||
      !have_mapping_snapshot_ ||
      !map_->getStatus()) {
    return false;
  }
  const int slot = static_cast<int>(trigger);
  const auto now = std::chrono::steady_clock::now();
  if (roadmap_rebuild_attempted_[slot] &&
      secondsSince(last_roadmap_rebuild_attempt_[slot]) <
          roadmap_rebuild_min_interval_s_) {
    return false;
  }
  roadmap_rebuild_attempted_[slot] = true;
  last_roadmap_rebuild_attempt_[slot] = now;
  KeyframeTrajectory trajectory;
  std::string error;
  if (!readOwnKeyframes(trajectory, error)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
                         "global graph not rebuilt (%s): %s", why,
                         error.c_str());
    return false;
  }
  if (trajectory.component_id != mapping_snapshot_.component_id ||
      trajectory.epoch != mapping_snapshot_.epoch) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 30000,
        "global graph not rebuilt (%s): the keyframes are of %s epoch %lu, "
        "the map in service is %s epoch %lu",
        why, trajectory.component_id.c_str(),
        static_cast<unsigned long>(trajectory.epoch),
        mapping_snapshot_.component_id.c_str(),
        static_cast<unsigned long>(mapping_snapshot_.epoch));
    return false;
  }
  // The same keyframes on the same map rebuild the same graph.
  const std::string inputs = trajectory.component_id + "/" +
                             std::to_string(trajectory.epoch) + "/" +
                             std::to_string(trajectory.revision) + "/" +
                             std::to_string(map_revision_);
  if (inputs == last_roadmap_rebuild_inputs_[slot]) return false;
  last_roadmap_rebuild_inputs_[slot] = inputs;

  // T_navigation_keyframe = T_component_navigation^-1 T_component_keyframe,
  // the same transform that places the map in the planning frame.
  const Eigen::Isometry3d navigation_from_component = navigationFromComponent();
  // Roll and pitch are kept: where the robot tipped, the map may show
  // nothing to tip on.
  std::vector<mgg::TrajectoryKeyframe> keyframes;
  keyframes.reserve(trajectory.poses.size());
  for (const Eigen::Isometry3d& pose : trajectory.poses) {
    const Eigen::Isometry3d keyframe = navigation_from_component * pose;
    const Eigen::Matrix3d r = keyframe.linear();
    mgg::TrajectoryKeyframe entry;
    entry.pose = mgg::StateVec(keyframe.translation().x(),
                               keyframe.translation().y(),
                               keyframe.translation().z(),
                               std::atan2(r(1, 0), r(0, 0)));
    entry.roll = std::atan2(r(2, 1), r(2, 2));
    entry.pitch = std::atan2(-r(2, 0), std::hypot(r(2, 1), r(2, 2)));
    keyframes.push_back(entry);
  }
  // The first keyframe is where the drone started. It is lifted as the
  // seed was, and only when the seed was: home seeded landed was its pad.
  // A planner started in flight cannot tell where its keyframes began.
  if (home_seeded_landed_ && !keyframes.empty()) {
    keyframes.front().pose[2] += aerial_home_height_m_;
  }

  auto rebuilt = std::make_shared<mgg::GraphManager>();
  rebuilt->setRobotId(static_cast<int>(planning_params_.robot_id));
  mgg::RoadmapRebuildParams params = roadmap_rebuild_params_;
  params.vertex_spacing = global_vertex_spacing_;
  // The keyframes are where the robot drove. A peer parked on them now
  // closes those edges for a search (globalEdgeBlocked, installed below),
  // not for the life of the rebuilt graph: its edges are checked against
  // the map with no peer bodies. Static occupancy and every body, step and
  // grade check still apply.
  std::optional<mgg::MolaMap::TransientDiscPin> no_peers;
  if (mola_map_ != nullptr) {
    no_peers.emplace(*mola_map_, std::vector<Eigen::Vector2d>{}, 0.0);
  }
  const mgg::RoadmapRebuildReport report = mgg::rebuildRoadmapFromTrajectory(
      *rebuilt, keyframes, makeGlobalContext(), params);
  no_peers.reset();
  if (!report.home_supported) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
                         "global graph not rebuilt (%s): no mapped ground "
                         "under the home keyframe at (%.2f, %.2f, %.2f)",
                         why, keyframes.front().pose.x(),
                         keyframes.front().pose.y(), keyframes.front().pose.z());
    return false;
  }
  mgg::Vertex* linked = nullptr;
  if (link_what_failed) {
    linked = link_what_failed(*rebuilt);
    if (linked == nullptr) {
      RCLCPP_WARN(get_logger(),
                  "global graph kept (%s): the graph rebuilt from %d "
                  "keyframes does not link it either (%d vertices, %d "
                  "components)",
                  why, report.keyframes, rebuilt->getNumVertices(),
                  report.components);
      return false;
    }
  }
  // Routes over the rebuilt graph leave out what they leave out over the
  // current one: no-go edges and, for the search only, edges a peer body
  // blocks (globalEdgeBlocked).
  rebuilt->setEdgeBlocked([this](const mgg::Vertex& a, const mgg::Vertex& b) {
    return globalEdgeBlocked(a, b);
  });
  // Only for an aerial robot: a ground robot's rebuild corrects a graph
  // whose connections the map may no longer support (a wall seen since),
  // and replaces it as before (review r0, P1). Both graphs are judged with
  // no-go edges out and edges a peer body closes open: a parked peer closes
  // them for a while, so it neither vetoes a rebuild nor hides the places
  // beyond it from the check.
  std::string loses_home;
  if (robot_params_.type == mgg::RobotType::kAerialRobot) {
    const FlagScope open(peer_edges_open_);
    loses_home = rebuildLosesHome(
        *global_graph_, *rebuilt, linked,
        static_cast<int>(planning_params_.robot_id),
        std::min(roadmap_rebuild_params_.link_radius,
                 planning_params_.edge_length_max));
  }
  if (!loses_home.empty()) {
    ++roadmap_rebuilds_refused_;
    RCLCPP_WARN(get_logger(),
                "global graph kept (%s): the graph rebuilt from %d keyframes "
                "%s (%d vertices, %d components, home's %d vertices); %d "
                "rebuild(s) refused so far",
                why, report.keyframes, loses_home.c_str(),
                rebuilt->getNumVertices(), report.components,
                report.home_component_vertices, roadmap_rebuilds_refused_);
    return false;
  }
  // The old graph's own frontiers go with it (none is carried over); the
  // next failed global search is then not exploration complete.
  int dropped = 0;
  const int own = static_cast<int>(planning_params_.robot_id);
  for (const auto& entry : global_graph_->vertices_map_) {
    const mgg::Vertex* vertex = entry.second;
    if (vertex != nullptr && vertex->robot_id == own &&
        vertex->type == mgg::VertexType::kFrontier &&
        global_graph_->inService(*vertex)) {
      ++dropped;
    }
  }
  if (dropped > 0) frontiers_dropped_in_rebuild_ = dropped;
  // Neighbours whose roadmap the old graph held, joined or quarantined.
  for (const auto& entry : neighbour_roadmaps_) {
    const auto merged = global_graph_->merged_graphs_.find(entry.first);
    if ((merged != global_graph_->merged_graphs_.end() && merged->second) ||
        global_graph_->isQuarantined(entry.first)) {
      roadmaps_to_readmit_.insert(entry.first);
    }
  }
  global_graph_ = rebuilt;
  // The flown trail's breadcrumbs were the old graph's; its samples still
  // waiting join the rebuilt one.
  flown_trail_.head_vertex_id = -1;
  flown_trail_.vertices_added = 0;
  global_root_supported_ = true;
  global_exploration_ongoing_ = false;
  current_global_vertex_id_ = -1;
  last_state_marker_ = current_state_;
  ++graph_revision_;
  ++roadmap_rebuilds_;
  // The rebuilt graph holds no neighbour roadmap: every one the old graph
  // held, joined or quarantined, is merged again from its cached copy, now
  // where its transform is current and when it returns otherwise (review
  // r0, I-7); until then exploration is not complete.
  readmitQuarantinedNeighbours();
  static const char* const kRefusals[8] = {
      "", "steep", "occupied", "unknown", "hanging", "cross slope",
      "footprint plane", "ground unobserved"};
  std::string refusals;
  const auto add_refusals = [&refusals](int count, const char* reason) {
    if (count == 0) return;
    if (!refusals.empty()) refusals += ", ";
    refusals += std::to_string(count) + " " + reason;
  };
  for (int s = 1; s < 8; ++s) {
    add_refusals(report.chain_refusals_by_status[s], kRefusals[s]);
  }
  add_refusals(report.chain_refusals_geofence, "geofence");
  add_refusals(report.chain_refusals_tilted, "tipped");
  const mgg::Vertex* home = global_graph_->getVertex(0);
  RCLCPP_INFO(get_logger(),
              "global graph rebuilt from %d keyframes (%s): home at (%.2f, "
              "%.2f, %.2f); %d vertices (%d beside their keyframe, %d "
              "keyframes without ground, %d tipped, %d spots without room), "
              "%d edges; "
              "along the track %d kept, %d refused%s%s%s, %d gaps; %d "
              "links; %d components, home's %d vertices; revision %lu; "
              "%.0f ms",
              report.keyframes, why, home->state.x(), home->state.y(),
              home->state.z(), report.vertices, report.offset_vertices,
              report.unsupported_keyframes, report.tilted_keyframes,
              report.boxed_vertices,
              global_graph_->getNumEdges(), report.chain_edges,
              report.chain_edges_refused, refusals.empty() ? "" : " (",
              refusals.c_str(), refusals.empty() ? "" : ")",
              report.chain_gaps, report.link_edges, report.components,
              report.home_component_vertices,
              static_cast<unsigned long>(trajectory.revision),
              1000.0 * report.elapsed_s);
  return true;
}

void PlannerNode::advanceFlownTrail(bool sample_pose) {
  if (robot_params_.type != mgg::RobotType::kAerialRobot || !have_odometry_ ||
      global_graph_->getNumVertices() == 0) {
    return;
  }
  mgg::FlownTrailParams params;
  params.spacing_m = global_vertex_spacing_;
  params.merge_radius_m = 0.5 * global_vertex_spacing_;
  const double now_s = 1e-9 * static_cast<double>(last_odometry_stamp_ns_);
  const bool sampled =
      sample_pose &&
      mgg::sampleFlownPose(flown_trail_, current_state_, now_s, params);
  const int discarded = mgg::trimFlownTrail(flown_trail_, params, now_s);
  flown_trail_counters_.expired += discarded;
  if (discarded > 0) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "flown trail broken: %d pending sample(s) discarded before certification "
        "(limit %d samples / %.0f s), map %s",
        discarded, params.max_pending, params.patience_s,
        map_->getStatus() ? "available" : "unavailable");
  }
  if (flown_trail_.pending.empty() || !map_->getStatus()) return;
  if (sample_pose && !sampled && discarded == 0 &&
      map_revision_ == flown_trail_map_revision_) {
    return;
  }
  flown_trail_map_revision_ = map_revision_;
  std::optional<mgg::MolaMap::TransientDiscPin> no_peers;
  if (mola_map_ != nullptr) {
    no_peers.emplace(*mola_map_, std::vector<Eigen::Vector2d>{}, 0.0);
  }
  const int vertices = global_graph_->getNumVertices();
  const int edges = global_graph_->getNumEdges();
  const mgg::FlownTrailReport report = mgg::addFlownBreadcrumbs(
      *global_graph_, flown_trail_, makeGlobalContext(), params, now_s);
  no_peers.reset();
  if (global_graph_->getNumVertices() != vertices ||
      global_graph_->getNumEdges() != edges) {
    ++graph_revision_;
  }
  flown_trail_counters_.added += report.added;
  flown_trail_counters_.merged += report.merged;
  flown_trail_counters_.joined += report.joined;
  flown_trail_counters_.expired += report.expired;
  flown_trail_counters_.capped += report.capped;
  if (report.expired > 0 || report.capped > 0) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "flown trail broken: %d sample(s) given up (their sweep not observed "
        "free within %.0f s, or more than %d waiting), %d past the %d-vertex "
        "cap; the next sample joins the global graph where it can",
        report.expired, params.patience_s, params.max_pending, report.capped,
        params.max_vertices);
  }
  if (report.added > 0 || report.joined > 0) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "flown trail: +%d breadcrumb(s), %d on existing vertices, %d joined "
        "anew; %zu waiting for the map; %d added in all (%d vertices, %d "
        "edges)",
        report.added, report.merged, report.joined,
        flown_trail_.pending.size(), flown_trail_.vertices_added,
        global_graph_->getNumVertices(), global_graph_->getNumEdges());
  }
}

void PlannerNode::ingestOdometryIntoGlobalGraph() {
  if (!have_odometry_ || global_graph_->getNumVertices() == 0) return;
  // Wherever the drone flies, by whatever it was sent: not only the
  // exploration paths, which teleoperation, an operator's goal or Return
  // Home never add.
  advanceFlownTrail(/*sample_pose=*/true);
  const bool add_state =
      (current_state_.head<3>() - last_state_marker_.head<3>()).norm() >=
      kOdoUpdateMinLength;
  const bool record_state =
      (current_state_.head<3>() - last_state_marker_global_.head<3>())
          .norm() >= kMinLength;
  if (!add_state && !record_state) return;

  // rrg.cpp:5247 to 5268: the robot's state joins the graph wired to every
  // reachable neighbour. Not before the home anchor has mapped support:
  // until then the root is a landmark no edge may attach to.
  if (add_state && global_root_supported_ && map_->getStatus()) {
    mgg::StateVec state = current_state_;
    if (projectToDrivingHeight(state)) {
      mgg::Vertex new_vertex(-1, state);
      mgg::ExpandGraphReport rep;
      mgg::expandGraph(*global_graph_, new_vertex, rep, makeGlobalContext());
      if (rep.status == mgg::ExpandGraphStatus::kSuccess) ++graph_revision_;
    }
  }
  if (add_state) last_state_marker_ = current_state_;

  // rrg.cpp:5270 to 5290: record the state and apply event E1.
  if (record_state) {
    robot_state_hist_.addState(current_state_);
    mgg::StateVec state = current_state_;
    global_graph_->updateVertexTypeInRange(state, kUpdateRadius);
    last_state_marker_global_ = current_state_;
  }
}

void PlannerNode::expandGlobalGraphTimerCallback() {
  const auto generation = request_generation_.load();
  std::unique_lock<std::recursive_mutex> lock(planner_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return;
  mgg::PlanningCancellationScope cancellation([this, generation] {
    return generation != request_generation_.load();
  });
  try {
    mgg::planningCheckpoint();
    expandGlobalGraphTimerCallbackImpl();
  } catch (const mgg::PlanningInterrupted&) {
    ++cancellations_;
    ++graph_revision_;
  }
}

void PlannerNode::expandGlobalGraphTimerCallbackImpl() {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  auto map_read = mapReadLease();
  refreshMapRevision();
  if (planner_trigger_count_ == 0 || home_state_wait_started_) return;
  if (!have_odometry_ || !map_->getStatus()) return;
  // The pass samples against one peer set, the one its skip key records.
  std::optional<PeerBodyPin> peer_pin;
  pinPeerBodies(peer_pin);
  refreshPeerGeneration();
  // The sampler draws around the unvisited clusters of the global graph and
  // tests against the map, the peer bodies and the robot's trail. When none
  // of those changed since a pass that added nothing, another pass spends
  // its whole budget (rrg.cpp:2608) rediscovering that nothing is
  // admissible: a parked robot would burn kGlobalGraphUpdateTimeBudget
  // every period indefinitely. A peer leaving opens space to sample
  // without changing the graph (review r1, R2).
  if (graph_revision_ == expansion_graph_revision_ &&
      map_revision_ == expansion_map_revision_ &&
      peer_generation_ == expansion_peer_generation_ &&
      (current_state_.head<3>() - expansion_state_.head<3>()).norm() <
          kOdometryStillM) {
    return;
  }
  expansion_graph_revision_ = graph_revision_;
  expansion_map_revision_ = map_revision_;
  expansion_peer_generation_ = peer_generation_;
  expansion_state_ = current_state_;

  const mgg::GlobalGraphExpansionReport report = mgg::expandGlobalGraph(
      *global_graph_, makeGlobalContext(), random_sampler_, robot_state_hist_,
      globalFrontierGain(), mgg::kGlobalGraphUpdateTimeBudget);
  global_space_.setCenter(current_state_, /*use_extension=*/true);
  if (report.vertices_added > 0) ++graph_revision_;
  if (report.vertices_added > 0 || report.edges_added > 0) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "global graph expansion: +%d vertices, +%d edges "
                         "(%d vertices, %d edges)",
                         report.vertices_added, report.edges_added,
                         global_graph_->getNumVertices(),
                         global_graph_->getNumEdges());
  }
}

void PlannerNode::addRefPathToGraph(const std::vector<mgg::StateVec>& path) {
  if (path.size() < 2 || global_graph_->getNumVertices() == 0) return;
  // Upstream added the lattice vertices themselves when it could
  // (rrg.cpp:4549), which carries the leaf's frontier mark and gain across.
  // The poses of a lattice path are lattice states, so look each one up; a
  // shortcut path is not, and enters as poses.
  std::vector<mgg::Vertex*> lattice;
  lattice.reserve(path.size());
  for (const mgg::StateVec& pose : path) {
    mgg::Vertex* vertex = nullptr;
    if (!local_graph_->getNearestVertexInRange(&pose, 1e-6, &vertex) ||
        vertex == nullptr) {
      lattice.clear();
      break;
    }
    lattice.push_back(vertex);
  }
  const mgg::ExpandContext ctx = makeGlobalContext();
  // The lattice's gain outside an exploration region counts nothing there
  // (buildLocalGraph): carried across, it would demote the roadmap's own
  // frontiers outside the region for good. Those vertices enter as poses.
  mgg::UsableVertexFn carries_gain;
  // Those inside it counted only what they see inside it, which may leave a
  // frontier at the boundary without gain (review r0, P1): they carry the
  // gain of all they see instead, scored as a frontier re-check scores it,
  // so an update that finds them explored still demotes them.
  std::vector<std::unique_ptr<mgg::Vertex>> unrestricted;
  if (exploration_region_) {
    carries_gain = [region = *exploration_region_](const mgg::Vertex& vertex) {
      return region.isInsideSpace(vertex.state.head<3>());
    };
    const mgg::RecomputeGainFn whole_view = globalFrontierGain();
    for (mgg::Vertex*& vertex : lattice) {
      if (!carries_gain(*vertex)) continue;
      auto rescored = std::make_unique<mgg::Vertex>(*vertex);
      // Scored on this robot's map: globalFrontierGain scores another
      // robot's vertex from its owner's reported counts instead.
      rescored->robot_id = static_cast<int>(planning_params_.robot_id);
      whole_view(*rescored);
      if (rescored->vol_gain.is_frontier) {
        rescored->type = mgg::VertexType::kFrontier;
      } else if (rescored->type == mgg::VertexType::kFrontier) {
        rescored->type = mgg::VertexType::kUnvisited;
      }
      vertex = rescored.get();
      unrestricted.push_back(std::move(rescored));
    }
    global_space_.setCenter(current_state_, /*use_extension=*/true);
  }
  std::vector<mgg::Vertex*> added_vertices;
  const auto add = [&]() {
    return lattice.empty()
               ? mgg::addRefPathToGraph(*global_graph_, path, ctx,
                                        global_vertex_spacing_,
                                        &added_vertices)
               : mgg::addRefPathToGraph(*global_graph_, lattice, ctx,
                                        global_vertex_spacing_,
                                        &added_vertices, carries_gain);
  };
  int before = global_graph_->getNumVertices();
  bool added = add();
  // Only when the graph reaches none of the path's poses: a path it
  // reaches but refused to link (a wedged start) is no reason to replace
  // it. The rebuilt graph replaces it only if the path joins it.
  if (!added &&
      std::none_of(path.begin(), path.end(),
                   [this](const mgg::StateVec& pose) {
                     return globalGraphReaches(pose);
                   })) {
    int rebuilt_before = 0;
    std::vector<mgg::Vertex*> rebuilt_added;
    const bool rebuilt = rebuildGlobalGraphFromKeyframes(
        RoadmapRebuildTrigger::kPathUnlinkable,
        "an exploration path could not be linked",
        [&](mgg::GraphManager& graph) -> mgg::Vertex* {
          rebuilt_before = graph.getNumVertices();
          const bool joined =
              lattice.empty()
                  ? mgg::addRefPathToGraph(graph, path, ctx,
                                           global_vertex_spacing_,
                                           &rebuilt_added)
                  : mgg::addRefPathToGraph(graph, lattice, ctx,
                                           global_vertex_spacing_,
                                           &rebuilt_added, carries_gain);
          return joined && !rebuilt_added.empty() ? rebuilt_added.front()
                                                  : nullptr;
        });
    if (rebuilt) {
      before = rebuilt_before;
      added_vertices = rebuilt_added;
      added = true;
    }
  }
  if (!added) {
    RCLCPP_WARN(get_logger(),
                "exploration path not added to the global graph: none of its "
                "%zu poses from (%.2f, %.2f, %.2f) could be linked",
                path.size(), path.front().x(), path.front().y(),
                path.front().z());
    return;
  }
  if (global_graph_->getNumVertices() != before) ++graph_revision_;
  // Where the path joined: at its start, or farther along when the start
  // could not be linked.
  const double joined_from_start_m =
      (added_vertices.front()->state.head<2>() - path.front().head<2>())
          .norm();
  RCLCPP_INFO(get_logger(),
              "global graph: +%d vertices from the exploration path, joined "
              "%.2f m from its start (%d vertices, %d edges)",
              global_graph_->getNumVertices() - before, joined_from_start_m,
              global_graph_->getNumVertices(), global_graph_->getNumEdges());
}

void PlannerNode::addFrontiers() {
  if (local_graph_->getNumVertices() < 2 ||
      global_graph_->getNumVertices() == 0) {
    return;
  }
  const int before = global_graph_->getNumVertices();
  // Upstream's kRangeCheck and kUpdateRadius (rrg.cpp:2447) were 1 m and 3 m
  // for an SMB whose trajectory was sampled every metre; scale them with the
  // trajectory spacing so a foot-bot configuration keeps the same proportions.
  const mgg::FrontierAdditionReport report = mgg::addFrontiers(
      *global_graph_, *local_graph_, makeGlobalContext(),
      [this, owner_gain = globalFrontierGain()](mgg::Vertex& vertex) {
        owner_gain(vertex);
        if (vertex.robot_id == static_cast<int>(planning_params_.robot_id) ||
            vertex.locally_explored) return;
        // addFrontiers calls this for peers only near the new local graph,
        // where this robot has fresh observations. Preserve owner counts.
        const RestoreScope restore_space(global_space_);
        global_space_.setCenter(vertex.state, /*use_extension=*/true);
        mgg::VolumetricGain local_gain;
        mgg::computeVolumetricGain(vertex.state, local_gain, makeGainContext());
        if (!local_gain.is_frontier) {
          vertex.locally_explored = true;
          vertex.vol_gain.is_frontier = false;
          vertex.vol_gain.gain = 0.0;
        }
      },
      global_vertex_spacing_, 1.0 * global_vertex_spacing_,
      3.0 * global_vertex_spacing_);
  global_space_.setCenter(current_state_, /*use_extension=*/true);
  if (global_graph_->getNumVertices() != before) ++graph_revision_;
  RCLCPP_INFO(get_logger(),
              "global graph: %d frontier(s) re-checked, %d demoted, %d "
              "peer frontier(s) left to their owners; %d local "
              "frontier(s) in %d cluster(s), %d path(s) added (%d vertices, "
              "%d edges)",
              report.global_frontiers_rechecked,
              report.global_frontiers_demoted,
              report.global_frontiers_left_to_owners, report.local_frontiers,
              report.clusters, report.paths_added,
              global_graph_->getNumVertices(), global_graph_->getNumEdges());
}

// ---------------------------------------------------------------------------
// Paths

void PlannerNode::shortcutAndResample(std::vector<mgg::StateVec>& path,
                                      const mgg::PathOkFn& turns_ok,
                                      const mgg::PathOkFn& corridor_ok,
                                      bool lattice_route) {
  path_shortcut_from_ = static_cast<int>(path.size());
  path_shortcut_corners_ = path_shortcut_from_;
  path_shortcut_to_ = path_shortcut_from_;
  if (path.size() <= 2) return;
  // What comes out of a graph is a walk along its edges: it steps between
  // vertices and reads as a staircase even across open floor. ROS 1 ran every
  // path it returned through improveFreePath and interpolatePath
  // (rrg.cpp:4160, 4176).
  //
  // Shortcut first, then resample. The other order interpolates points that
  // are about to be discarded, and leaves the corners the shortcut removed
  // still bent.
  // The plan's own projection, sharing ground lookups between the leaps
  // the pass tries; the map is held still by the caller's lease.
  mgg::GroundProjection shortcut_ground(*map_, planning_params_, true);
  shortcut_ground.setStandingStart(standingStart());
  mgg::ExpandContext ctx = makeContext();
  ctx.ground = &shortcut_ground;
  if (!lattice_route) {
    ctx.unknown_body_above_center.reset();
    ctx.standing_body.reset();
    ctx.own_body_known_free.reset();
  }
  const bool stop_at_unknown = !lattice_route || ctx.stop_at_unknown ||
                               !ctx.allow_unknown_lattice_body;
  const auto segment_free = [this, &ctx, stop_at_unknown](
                                const Eigen::Vector3d& from,
                                const Eigen::Vector3d& to) {
    // stop_at_unknown_voxel is true: a shortcut may only cross space already
    // known to be free. Upstream passes false here (rrg.cpp:4610), which
    // treats unknown space as passable; that is survivable there because its
    // shortcut only ever collapses a node when the segment leading to it is
    // under half a metre. Applied to a general shortcut it is not: a partly
    // explored map is mostly unknown, so every candidate line qualifies and
    // the path collapses to a straight run through whatever has not been
    // seen yet. A ground robot's segment also follows the terrain (steps,
    // inclination) as every graph edge does. Nor may it cross a no-go zone
    // the route went round.
    if (noGoBlocksSegment(from, to)) return false;
    // An exploration path's shortcut keeps to the scouting exclusions too.
    if (exploration_route_ && !scouting_zones_.empty() &&
        scouting_zones_.blocksEdge(from, to, current_state_.head<3>())) {
      return false;
    }
    if (robot_params_.type == mgg::RobotType::kAerialRobot &&
        peerBlocksSegment(from, to)) return false;
    if (robot_params_.type == mgg::RobotType::kGroundRobot) {
      return mgg::groundShortcutSegmentAdmissible(ctx, from, to,
                                                  stop_at_unknown);
    }
    return map_->getStrictPathStatus(from + robot_params_.center_offset,
                                      to + robot_params_.center_offset,
                                      ctx.robot_box_size) ==
           mgg::VoxelStatus::kFree;
  };
  mgg::PathType points;
  points.reserve(path.size());
  for (const mgg::StateVec& s : path) points.push_back(s.head(3));
  const mgg::PathType unshortcut = points;
  const bool unshortcut_ok = turns_ok && turns_ok(unshortcut);
  if (corridor_ok && !corridor_ok(unshortcut)) {
    path.clear();
    return;
  }
  const mgg::PathOkFn admissible = [&](const mgg::PathType& trial) {
    return (!unshortcut_ok || turns_ok(trial)) &&
           (!corridor_ok || corridor_ok(trial));
  };
  // The same bounded clearance measure as lattice selection; per-call
  // caching cannot outlive the map's read lease or retain transient
  // obstacles.
  mgg::SegmentClearanceFn clearance;
  if (robot_params_.type == mgg::RobotType::kGroundRobot &&
      planning_params_.path_clearance_margin > 0.0) {
    clearance = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
      return shortcut_ground.segmentClearance(a, b, ctx.robot_box_size);
    };
  }
  // The route as it will be sent: resampled at path_interpolation_distance
  // and, for a ground robot, each new pose at driving height over the
  // ground under it, as the leap's check projected it (a straight line
  // between a floor pose and a deck pose cuts under a ramp's crest).
  const auto resample = [&](const mgg::PathType& corners) {
    mgg::PathType resampled;
    if (!(planning_params_.path_interpolation_distance > 0.0) ||
        !mgg::interpolatePath(corners,
                              planning_params_.path_interpolation_distance,
                              resampled) ||
        resampled.size() < 2) {
      return corners;
    }
    // interpolatePath stops short of the last point by up to one step; the
    // route ends where it was planned to, which for an objective is the
    // exact goal.
    if ((resampled.back() - corners.back()).norm() > 1e-6) {
      resampled.push_back(corners.back());
    }
    if (robot_params_.type == mgg::RobotType::kGroundRobot) {
      for (std::size_t i = 1; i + 1 < resampled.size(); ++i) {
        const bool corner = std::any_of(
            corners.begin(), corners.end(), [&](const Eigen::Vector3d& c) {
              return (c - resampled[i]).norm() < 1e-9;
            });
        if (corner) continue;
        Eigen::Vector3d probe = resampled[i];
        mgg::VoxelStatus status = mgg::VoxelStatus::kUnknown;
        const double below = shortcut_ground.projectSample(probe, status);
        if (status == mgg::VoxelStatus::kOccupied) {
          resampled[i].z() -= below - planning_params_.max_ground_height;
        }
      }
    }
    return resampled;
  };
  // Leaps are judged on the route they make as it will be sent: resampling
  // moves where each turn's measuring window ends.
  const mgg::PathOkFn sent_admissible = [&](const mgg::PathType& trial) {
    return admissible(resample(trial));
  };
  points = robot_params_.type == mgg::RobotType::kGroundRobot
      ? mgg::shortcutPathKeepingClearance(points, segment_free, sent_admissible, clearance)
      : mgg::shortcutPath(points, segment_free, admissible);
  path_shortcut_corners_ = static_cast<int>(points.size());
  points = resample(points);
  // Resampling moves where each turn's measuring window ends, so the route
  // is checked again as it will be sent.
  if (!admissible(points)) {
    points = unshortcut;
    path_shortcut_corners_ = static_cast<int>(points.size());
    ++shortcut_turn_reverts_;
    RCLCPP_WARN(get_logger(),
                "shortcut route fails turn or reverse-corridor checks; sent "
                "unshortcut (%d so far)",
                shortcut_turn_reverts_);
  }
  path_shortcut_to_ = static_cast<int>(points.size());

  // Rebuild the states, keeping each point's heading pointing along the path
  // it is now on rather than along the edge it came from.
  std::vector<mgg::StateVec> rebuilt;
  rebuilt.reserve(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    const Eigen::Vector3d& here = points[i];
    const Eigen::Vector3d& ahead = points[i + 1 < points.size() ? i + 1 : i];
    const Eigen::Vector3d step = ahead - here;
    const double yaw = step.head(2).norm() > 1e-9
                           ? std::atan2(step.y(), step.x())
                           : (rebuilt.empty() ? path.front()[3]
                                              : rebuilt.back()[3]);
    rebuilt.emplace_back(here.x(), here.y(), here.z(), yaw);
  }
  path = rebuilt;
}

std::string PlannerNode::buildLocalGraph() {
  // Held for the whole cycle: the map must not change under a planner that is
  // ray-casting through it.
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  StandingStartScope standing_scope(*this);
  const DeadlineScope budget(lattice_deadline_,
      robot_params_.type == mgg::RobotType::kGroundRobot ? lattice_time_budget_s_ : 0.0);
  try {
  auto map_read = mapReadLease();
  refreshMapRevision();
  refreshScoutingExclusions();
  const FlagScope exploring(exploration_route_);
  best_path_.clear();
  best_path_from_global_graph_ = false;
  boxed_in_without_departure_now_ = false;
  local_gain_remains_now_ = false;
  low_gain_path_now_ = false;
  if (!have_odometry_) return "no odometry received yet";
  if (home_state_wait_started_) return "waiting for flight_state";
  if (!map_->getStatus()) {
    if (mola_map_ != nullptr) {
      const std::string detail = mola_map_->lastError();
      return detail.empty() ? "MOLA map snapshot is missing or stale"
                            : "MOLA map unavailable: " + detail;
    }
    return "map is empty; no point cloud received yet";
  }

  // Per-phase timing. A cycle that never returns says nothing about which of
  // the phases is responsible, and they scale with completely different
  // things: the sweep with the lattice volume, the gain with the number of
  // viewpoints times the sensor's ray count, the selection with the graph.
  using Clock = std::chrono::steady_clock;
  const auto t_start = Clock::now();
  seedGlobalGraph();

  // The last local graph's frontier paths go into the global graph before
  // that graph is thrown away (rrg.cpp:121, Rrg::reset).
  if (add_frontiers_to_global_graph_) {
    add_frontiers_to_global_graph_ = false;
    addFrontiers();
  }

  local_graph_->reset();
  // Inclinations are keyed by vertex id and the ids restart with the graph.
  edge_inclinations_.clear();
  mgg::StateVec root_state = current_state_;
  const bool root_hanging = !projectToDrivingHeight(root_state);
  if (root_hanging) root_state = physicalAnchorAtDrivingHeight(current_state_);
  auto* root = new mgg::Vertex(0, root_state);
  root->robot_id = static_cast<int>(planning_params_.robot_id);
  root->is_hanging = root_hanging;
  local_graph_->addVertex(root);
  ++planner_trigger_count_;
  const std::optional<mgg::StandingStart> standing = standingStart();
  const mgg::StandingStart* standing_on = standing ? &*standing : nullptr;
  // A robot that may stand blind at its start, and cannot tell, is told
  // of in every plan (runs 9 and 10).
  const std::string standing_note =
      standing_start_unread_keyframes_.empty()
          ? (standing ? "; standing start: active"
                      : left_standing_start_ ? "; standing start: expired"
                                             : "; standing start: inactive")
          : "; standing start: inactive (no keyframes at " +
                standing_start_unread_keyframes_ + ")";

  // The lattice checks each vertex's footprint from every edge that meets
  // it. Its ground lookups are shared for this plan only: the map is held
  // still by the lease above and the lock, and the cache goes with the plan.
  mgg::GroundProjection plan_ground(*map_, planning_params_,
                                    /*cache_footprint_ground=*/true);
  plan_ground.setStandingStart(standing);
  mgg::ExpandContext ctx = makeContext();
  ctx.ground = &plan_ground;
  if (robot_params_.type == mgg::RobotType::kGroundRobot && lattice_time_budget_s_ > 0) {
    const auto slice = innerDeadline(lattice_deadline_, ground_exploration_lattice_budget_s_);
    if (!ctx.deadline || slice < *ctx.deadline) ctx.deadline = slice;
  }
  const auto t_global = Clock::now();
  // The lattice is laid out around the root at driving height, where the
  // root vertex is; a ground robot's odometry origin sits lower than that.
  const mgg::GridGraphResult r = buildGridGraph(
      *local_graph_, root_state, grid_params_, ctx, latticeHeading());
  if (r.status == mgg::GridGraphStatus::kInvalidBounds) {
    return "grid bounds invalid: min_val must be <= 0, max_val >= 0 and "
           "resolution non-zero";
  }
  const auto t_grid = Clock::now();
  // Global space is defined in world frame and must remain static at world origin.
  mgg::GainContext gain_ctx = makeGainContext();
  // An operator region limits new exploration, not persistent frontier
  // re-checks: a temporary exclusion must never be broadcast as explored.
  gain_ctx.gain_region = exploration_region_ ? &*exploration_region_ : nullptr;
  const int evaluated = mgg::computeExplorationGain(
      *local_graph_, gain_ctx, planning_params_.leafs_only_for_volumetric_gain,
      planning_params_.cluster_vertices_for_gain);
  int frontiers = 0;
  std::int64_t band_unknown = 0, total_unknown = 0;
  bool total_available = true;
  std::int64_t free_voxels = 0, occupied_voxels = 0;
  std::uint64_t gain_rays = 0, gain_visits = 0;
  for (const auto& entry : local_graph_->vertices_map_) {
    if (entry.second == nullptr) continue;
    band_unknown += entry.second->vol_gain.num_unknown_voxels;
    if (entry.second->vol_gain.num_total_unknown_voxels < 0) total_available = false;
    else total_unknown += entry.second->vol_gain.num_total_unknown_voxels;
    free_voxels += entry.second->vol_gain.num_free_voxels;
    occupied_voxels += entry.second->vol_gain.num_occupied_voxels;
    gain_rays += entry.second->vol_gain.gain_rays_cast;
    gain_visits += entry.second->vol_gain.gain_voxel_visits;
  }
  // Frontiers given up as unreachable from here (retention refused three
  // times) are no local gain remaining: they must not hold completion back
  // (run14 review r2), here or in the fleet's settling.
  int outstanding_frontiers = 0;
  for (const auto& entry : local_graph_->vertices_map_) {
    if (entry.second != nullptr &&
        entry.second->type == mgg::VertexType::kFrontier) {
      ++frontiers;
      if (!reverseExitEndpointGivenUp(entry.second->state)) ++outstanding_frontiers;
    }
  }
  RCLCPP_INFO(get_logger(),
              "gain evidence: %s; band-unknown=%lld total-unknown=%s "
              "free=%lld occupied=%lld frontiers=%d viewpoints=%d "
              "scan-rays=%llu scan-visits=%llu ground-model-step-deg=%.2f "
              "ground-model-range-m=%.2f (0=sensor; summed per viewpoint; work backend-reported)",
              robot_params_.type == mgg::RobotType::kGroundRobot
                  ? "ground reachable-height band" : "aerial full 3D",
              static_cast<long long>(band_unknown),
              (total_available ? std::to_string(total_unknown) : "unavailable (pruned)").c_str(),
              static_cast<long long>(free_voxels), static_cast<long long>(occupied_voxels),
              frontiers, evaluated, static_cast<unsigned long long>(gain_rays),
              static_cast<unsigned long long>(gain_visits),
              planning_params_.ground_gain_angular_resolution_deg,
              planning_params_.ground_gain_max_range);
  // The graph's frontiers are worth keeping whether or not a path is chosen.
  add_frontiers_to_global_graph_ = local_graph_->getNumVertices() > 1;

  // The aerial tour must see this lattice's frontiers now, not a plan later
  // after it has already committed to a lifted target behind the fleet.
  if (robot_params_.type == mgg::RobotType::kAerialRobot && add_frontiers_to_global_graph_) {
    addFrontiers();
    add_frontiers_to_global_graph_ = false;
  }

  const auto t_gain = Clock::now();
  // Paths are penalised for leaving the way the robot faces
  // (path_direction_penalty): exploration goes on in the direction the
  // robot is already driving, rather than turning back for a similar gain
  // (run 6). Toward a target the robot has no route to yet, the direction is
  // the one to the target. It stays a preference: a path round an obstacle
  // that first heads away is discounted, not refused.
  const double selection_direction =
      exploration_target_.has_value()
          ? std::atan2(exploration_target_->y() - current_state_.y(),
                       exploration_target_->x() - current_state_.x())
          : current_state_[3];
  // A ground robot turns sharply only on level ground with room to turn in
  // place; a path that turns elsewhere is taken only when no other path
  // would be.
  int room_occupied = 0, room_unobserved = 0;
  const auto room_to_turn = [&](const mgg::StateVec& pose, bool allow_prior = true) {
    if (!mgg::turnClear(*map_, robot_params_, pose)) {
      ++room_occupied;
      return false;
    }
    if (!mgg::turnSpaceObserved(*map_, robot_params_, planning_params_, pose,
                               allow_prior ? standing_on : nullptr)) {
      ++room_unobserved;
      return false;
    }
    return true;
  };
  mgg::PathTurnCheck turn_check(*local_graph_, robot_params_, room_to_turn,
      nullptr, false, [this](const auto& a, const auto& b) {
        return mgg::turnTransitionClear(*map_, robot_params_, a, b);
      });
  turn_check.setRobotTilt(root_state.head<3>(), current_tilt_);
  // Where the lattice is too sparse to fit the ground, the map measures it.
  turn_check.setUnmeasuredSlope([this](const Eigen::Vector3d& position) {
    return mgg::groundSlope(
        *ground_, position,
        std::max(robot_params_.size.x(), robot_params_.size.y()));
  });
  mgg::PathTurnsFn turns_admissible;
  mgg::SharpTurnAllowedFn sharp_turn_allowed;
  // A path end on a slope, which needs no observed turn space, needs room
  // to turn within kDepartureMaxM back along its path (review r0, P1).
  mgg::SlopeEndRetreat slope_end_retreat;
  // Unlike outward root edges, an escape has no blind-start allowance.
  mgg::GroundProjection reverse_ground(*map_, planning_params_, true);
  const auto reverse_edge = [&](const mgg::StateVec& from, const mgg::StateVec& to) {
    return reverseExitEdge(reverse_ground, from, to);
  };
  std::map<std::pair<int, int>, bool> reverse_edges;

  if (robot_params_.type == mgg::RobotType::kGroundRobot) {
    turns_admissible = std::ref(turn_check);
    sharp_turn_allowed = [&turn_check](const mgg::Vertex& v) {
      return turn_check.sharpTurnAllowedAt(v.state.head<3>());
    };
    slope_end_retreat.admitted_on_slope = [this](const mgg::Vertex& v) {
      auto arrival = v.state;
      arrival[3] = mgg::kUnknownTurnHeading;
      return mgg::slopeExemptsTurnSpace(mgg::groundSlope(
                 *ground_, v.state.head<3>(),
                 std::max(robot_params_.size.x(), robot_params_.size.y()),
                 local_graph_.get())) &&
             !mgg::turnSpaceObserved(*map_, robot_params_, planning_params_,
                                     arrival);
    };
    slope_end_retreat.refuge_admissible = [this](const auto& path, std::size_t end,
                                                std::size_t refuge) {
      std::vector<mgg::StateVec> reverse;
      for (std::size_t i = end + 1; i-- > refuge;) reverse.push_back(path[i]->state);
      return refugeArrivalBandAdmissible(reverse);
    };
    slope_end_retreat.reverse_edge_admissible = [&](const mgg::Vertex& a,
                                                   const mgg::Vertex& b) {
      const auto key = std::make_pair(a.id, b.id);
      const auto found = reverse_edges.find(key);
      if (found != reverse_edges.end()) return found->second;
      return reverse_edges[key] = reverse_edge(a.state, b.state);
    };
  }
  // Right after a plan that turned back, the direction penalty is not
  // bounded: the robot does not turn straight back again for a similar
  // gain (mgg::pathTurnsBack).
  const mgg::PlanningParams selection_params =
      turn_back_hysteresis_.selectionParams(planning_params_);
  int retention_excluded = 0;
  const mgg::PathSelectionResult sel = mgg::selectBestPath(
      *local_graph_, selection_params, robot_params_, edge_inclinations_,
      map_->getResolution(), selection_direction, selectionExclusions(),
      reservation_exclusion_radius_m_,
      [this](const mgg::Vertex& v) {
        // On a slope, where the path-turn rule forbids turning, an end
        // needs no observed turn space.
        return mgg::viewpointClear(*map_, robot_params_, planning_params_,
                                   v.state, mgg::groundSlope(
                                       *ground_, v.state.head<3>(),
                                       std::max(robot_params_.size.x(),
                                                robot_params_.size.y()),
                                       local_graph_.get()));
      },
      turns_admissible, sharp_turn_allowed, reach_distance_,
      slope_end_retreat,
      [this, &standing, &retention_excluded](const mgg::Vertex& v) {
        if (no_go_.inside(v.state.head<3>())) return true;
        if (scoutingExcludes(v.state.head<3>())) {
          ++scouting_counters_.viewpoints_refused;
          return true;
        }
        if (reverseExitEndpointExcluded(v.state)) {
          ++retention_excluded;
          return true;
        }
        return standing && !standingStartGoalAdmissible(v.state);
      },
      // A path the final check would refuse is left out, so another is
      // chosen rather than none (review r1, R1-1).
      no_go_.empty() && scouting_zones_.empty()
          ? mgg::PathTurnsFn()
          : mgg::PathTurnsFn([this](const std::vector<mgg::Vertex*>& path) {
              std::vector<Eigen::Vector3d> points;
              points.reserve(path.size());
              for (const mgg::Vertex* v : path) points.push_back(v->state.head<3>());
              if (!no_go_.pathAdmissible(points)) return false;
              if (scouting_zones_.empty() || scouting_zones_.pathAdmissible(points)) {
                return true;
              }
              ++scouting_counters_.paths_refused;
              return false;
            }));
  for (const mgg::Vertex* v : sel.best_path) {
    if (v != nullptr) best_path_.push_back(v->state);
  }
  lattice_selection_direction_ = selection_direction;
  if (sel.sharp_turn_detour) {
    RCLCPP_INFO(get_logger(),
                "exploration path to (%.2f, %.2f, %.2f) goes the long way "
                "round: every shortest path turns on a slope or without room "
                "(%d search states%s)",
                best_path_.back().x(), best_path_.back().y(),
                best_path_.back().z(), sel.detour_states_expanded,
                sel.detour_search_capped ? ", capped" : "");
  }
  if (sel.sharp_turn_fallback) {
    ++sharp_turn_fallbacks_;
    // Whether a route turning only where it may was ruled out, or only not
    // found in time.
    char search[192];
    if (!sel.detour_searched) {
      std::snprintf(search, sizeof(search), "no route search");
    } else if (sel.detour_search_capped) {
      std::snprintf(search, sizeof(search),
                    "route search stopped at its cap of %d states with %d "
                    "route(s) found, none chosen; a compliant route may "
                    "exist",
                    mgg::kMaxDetourSearchStates, sel.detour_routes_found);
    } else {
      std::snprintf(search, sizeof(search),
                    "route search completed in %d states: %d route(s) "
                    "found, none chosen",
                    sel.detour_states_expanded, sel.detour_routes_found);
    }
    RCLCPP_WARN(get_logger(),
                "exploration path to (%.2f, %.2f, %.2f) turns sharply on a "
                "slope or without room to turn: no path turns only where it "
                "may; %s (%d such paths so far)",
                best_path_.back().x(), best_path_.back().y(),
                best_path_.back().z(), search, sharp_turn_fallbacks_);
  }
  if (sel.unclear_viewpoint) {
    ++unclear_viewpoints_selected_;
    RCLCPP_WARN(get_logger(),
                "exploration path ends without viewpoint clearance at (%.2f, "
                "%.2f, %.2f): no path ends clear (%d unclear viewpoints so "
                "far)",
                best_path_.back().x(), best_path_.back().y(),
                best_path_.back().z(), unclear_viewpoints_selected_);
  }

  // A best path that ends within the controller's goal tolerance, or leads
  // to no gain, takes the robot nowhere: run 5's Spot was sent a 2-pose
  // path with no gain 28 times, and its controller refused each one. It is
  // no path, so that the departure, the low-gain count and global
  // repositioning below run as they do when there is none.
  char nowhere[128] = "";
  const bool goes_nowhere =
      mgg::pathGoesNowhere(sel, current_state_.head<3>(), reach_distance_);
  if (goes_nowhere) {
    ++paths_going_nowhere_;
    last_nowhere_poses_ = static_cast<int>(best_path_.size());
    std::snprintf(nowhere, sizeof(nowhere),
                  "; best path goes nowhere (ends %.2f m away, gain %.1f of "
                  "%.1f): no path",
                  (best_path_.back().head<2>() - current_state_.head<2>())
                      .norm(),
                  sel.best_gain, sel.best_full_gain);
    best_path_.clear();
    path_shortcut_from_ = path_shortcut_corners_ = path_shortcut_to_ = 0;
  }

  // Boxed in: no path turns only where it may, goes anywhere, or has a
  // bounded way back from a narrow/slope end, and
  // the robot has no room to turn where it stands, so the fallback path
  // starts with a turn it cannot make. In run 4 a Bunker sent one in a
  // pocket got no valid trajectory from DWB three times and exploration was
  // blocked. It drives straight out instead, ahead or back, until it has
  // room; the next cycle plans from there. With no way out it gets no path,
  // and its adapter's own recovery runs.
  std::string boxed_in;
  const bool zone_escape = best_path_.empty() &&
                           no_go_.inside(root_state.head<3>());
  std::vector<Eigen::Vector3d> selected_points;
  for (const auto& pose : best_path_) selected_points.push_back(pose.head<3>());
  const bool is_boxed_in = !boxed_in_without_departure_now_ &&
                          (zone_escape || routeStartsWithTurnWithoutRoom(selected_points) || (
                           (sel.sharp_turn_fallback || goes_nowhere ||
                            (sel.best_path.empty() &&
                             sel.slope_ends_without_way_back > 0 &&
                             std::any_of(local_graph_->vertices_map_.begin(),
                                         local_graph_->vertices_map_.end(),
                                         [](const auto& entry) {
                                           return entry.second &&
                                               entry.second->vol_gain.gain > 0;
                                         })) ||
                            // Run 14: the candidates ran out where the
                            // robot has stood since they were refused.
                            (sel.best_path.empty() && retention_excluded > 0)) &&
                           turns_admissible &&
                           !mgg::roomToTurn(*map_, robot_params_, planning_params_,
                                            root_state, standing_on)));
  const bool stored_exit_tried =
      (is_boxed_in || sel.sharp_turn_fallback || best_path_.empty()) &&
      tryStoredReverseExit(root_state, boxed_in);
  if (!stored_exit_tried) boxed_in.clear();
  if (stored_exit_tried) {
    if (best_path_.empty()) boxed_in += departBoxedIn(root_state, "stored exit refused", false);
    path_shortcut_from_ = path_shortcut_corners_ = path_shortcut_to_ =
        static_cast<int>(best_path_.size());
  } else if (is_boxed_in) {
    const char* reason = zone_escape ? "no admissible end outside no-go zones"
        : goes_nowhere ? "its best path goes nowhere"
        : sel.best_path.empty() && sel.slope_ends_without_way_back > 0
            ? "no narrow or slope end has a way back"
        : sel.best_path.empty() && retention_excluded > 0
            ? "every other end was refused a reverse exit here"
            : "no path turns only where it may";
    boxed_in = departBoxedIn(root_state, reason);
    // Sent as it is: not a lattice path, and the roadmap keeps no edge the
    // robot drives backwards.
    path_shortcut_from_ = static_cast<int>(best_path_.size());
    path_shortcut_corners_ = path_shortcut_from_;
    path_shortcut_to_ = path_shortcut_from_;
  } else if (!goes_nowhere) {
    // rrg.cpp:4538: the accepted lattice path joins the global graph as its
    // vertices, before the shortcut turns it into poses.
    if (best_path_.size() >= 2) addRefPathToGraph(best_path_);
    // The shortcut must not make a turn the chosen path did not have.
    const double start_heading = current_state_[3];
    shortcutAndResample(best_path_, turns_admissible
        ? mgg::PathOkFn([&](const mgg::PathType& points) {
            return turn_check.admissible(points, start_heading);
          }) : mgg::PathOkFn(),
        turns_admissible ? mgg::PathOkFn([&](const mgg::PathType& points) {
            return reverseExitShortcutAdmissible(points);
          }) : mgg::PathOkFn(),
        /*lattice_route=*/true);
  }
  // Free cells with no vertices is the characteristic bring-up failure: the
  // lattice is finding space but every candidate is being turned away. The
  // reason breakdown is the only thing that separates a geometry mistake from
  // a genuinely blocked robot, so report it whenever it happens.
  char why[224] = "";
  if (r.vertices_added <= 16 && r.free_cells > 0) {
    std::snprintf(why, sizeof(why),
                  " (rejected: %d collision, %d no ground; edges: %d ok, "
                  "%d steep, %d occupied, %d unmapped, %d hanging, %d "
                  "cross-slope, %d footprint-plane, %d ground unobserved)",
                  r.rejected[static_cast<int>(mgg::ExpandGraphStatus::
                                                  kErrorCollisionEdge)],
                  r.no_ground, r.edge_status[0], r.edge_status[1],
                  r.edge_status[2], r.edge_status[3], r.edge_status[4],
                  r.edge_status[5], r.edge_status[6], r.edge_status[7]);
  }
  const auto t_end = Clock::now();
  const auto ms = [](Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };
  char timing[128];
  std::snprintf(timing, sizeof(timing),
                "; %.0f ms (global %.0f, grid %.0f, gain %.0f, select %.0f)",
                ms(t_start, t_end), ms(t_start, t_global), ms(t_global, t_grid),
                ms(t_grid, t_gain), ms(t_gain, t_end));
  char buf[896];
  std::snprintf(
      buf, sizeof(buf),
      "grid graph: %d free cells, %d vertices, %d edges%s%s; %d viewpoints, "
      "%d frontiers; best path %zu poses (%d lattice -> %d corners -> %d "
      "resampled), gain %.1f%s; viewpoint clearance: %d paths pulled back, "
      "%d without%s, %d narrow/slope ends without a way back; sharp turns: %d "
      "paths refused (%d on a slope, %d "
      "without room)%s%s%s%s; "
      "heading %.2f rad%s",
      r.free_cells, r.vertices_added, r.edges_added,
      r.hit_deadline ? " (stopped at its time budget)"
      : r.hit_limit  ? " (hit a size limit)"
                     : "",
      why, evaluated, frontiers,
      best_path_.size(), path_shortcut_from_, path_shortcut_corners_,
      path_shortcut_to_, sel.best_gain,
      sel.paths_rejected_steep > 0 ? " (some paths too steep)" : "",
      sel.paths_pulled_back, sel.paths_without_clear_viewpoint,
      sel.unclear_viewpoint ? ", none ends clear" : "",
      sel.slope_ends_without_way_back,
      sel.paths_with_sharp_turns, turn_check.refused_on_slope,
      turn_check.refused_without_room,
      sel.sharp_turn_detour ? ", detour" : "",
      sel.sharp_turn_fallback ? ", none complies" : "", nowhere,
      boxed_in.c_str(), selection_direction, timing);

  // rrg.cpp:2098 to 2120: rounds without a frontier among the leaves count
  // towards the global planner; a round with one counts back. A round with
  // frontiers but no path to send counts as one without: the lattice sees
  // gain it cannot reach, which is local gain left, not a finished
  // exploration, and not a reason never to reposition either. So does a
  // path scoring under low_gain_voxels (run 8: with every vertex a
  // frontier, robot_3 lapped an explored hangar on 2 to 150 voxels' gain).
  const double low_gain_score =
      planning_params_.low_gain_voxels * planning_params_.unknown_voxel_gain;
  if (local_graph_->getNumVertices() <= 1) {
    // Nothing was scored; this round says nothing about the frontier.
  } else if (frontiers == 0 || goes_nowhere || best_path_.empty()) {
    ++low_gain_rounds_;
    local_gain_remains_now_ =
        outstanding_frontiers > 0 || (goes_nowhere && sel.best_full_gain > 0.0);
  } else if (!is_boxed_in && sel.best_gain < low_gain_score) {
    ++low_gain_rounds_;
    low_gain_path_now_ = true;
    local_gain_remains_now_ = true;
  } else if (low_gain_rounds_ > 0) {
    --low_gain_rounds_;
  }
  // The lattice path as it will be sent, unless something replaces it
  // (recordSentPath).
  lattice_path_ = stored_exit_tried || is_boxed_in || goes_nowhere ? std::vector<mgg::StateVec>{}
                                               : best_path_;
  char room_note[128];
  std::snprintf(room_note, sizeof(room_note),
                "; room refusals: %d occupancy, %d missing observation",
                room_occupied, room_unobserved);
  std::string corner_note;
  if (turn_check.first_refused_corner) {
    const auto& corner = *turn_check.first_refused_corner;
    char first[192];
    std::snprintf(first, sizeof(first),
                  "; first refused corner (%.2f, %.2f, %.2f): %.1f deg turn, "
                  "%.1f deg slope, %s",
                  corner.position.x(), corner.position.y(), corner.position.z(),
                  corner.turn * 180.0 / M_PI, corner.slope * 180.0 / M_PI,
                  corner.on_slope ? "slope" : "no room");
    corner_note = first;
  }
  std::string arrival_note;
  if (standing && best_path_.empty()) {
    local_gain_remains_now_ = true;
    arrival_note = "; standing start: no admissible observed-arrival goal; "
                   "waiting for observed support/clearance";
  }
  mgg::planningCheckpoint();
  return std::string(buf) + standing_note + room_note + corner_note + arrival_note;
  } catch (const mgg::PlanningInterrupted&) {
    if (budget.previous_) throw;  // the outer request owns its refusal
    best_path_.clear();
    lattice_path_.clear();
    return budget.reason();
  }
}

std::string PlannerNode::departBoxedIn(const mgg::StateVec& root_state,
                                       const char* why, bool consult_stored) {
  std::string stored_note;
  if (consult_stored && tryStoredReverseExit(root_state, stored_note) &&
      !best_path_.empty()) return stored_note;
  char note[160];
  mgg::Departure departure;
  best_path_.clear();
  if (straightDeparture(root_state, departure)) {
    departure_sent_now_ = true;
    best_path_ = departure.path;
    ++boxed_in_departures_;
    const double length =
        (best_path_.back().head<2>() - best_path_.front().head<2>()).norm();
    char turn[48] = "";
    if (departure.turn != 0.0) {
      // The path's poses face the new heading; the controller turns while
      // it drives off rather than first (mgg::findDeparture, review r0 M-6).
      std::snprintf(turn, sizeof(turn), " at %+.0f deg to its heading",
                    departure.turn * 180.0 / M_PI);
    }
    std::snprintf(note, sizeof(note),
                  "; boxed in: straight departure %.2f m %s%s", length,
                  departure.reverse ? "in reverse" : "ahead", turn);
    RCLCPP_INFO(get_logger(),
                "boxed in at (%.2f, %.2f, %.2f): no room to turn and %s; "
                "departing %.2f m straight %s%s (%d departures so far)",
                root_state.x(), root_state.y(), root_state.z(), why, length,
                departure.reverse ? "back" : "ahead", turn,
                boxed_in_departures_);
  } else {
    ++boxed_in_without_departure_;
    boxed_in_without_departure_now_ = true;
    std::snprintf(note, sizeof(note),
                  "; boxed in: no straight departure, no path");
    RCLCPP_WARN(get_logger(),
                "boxed in at (%.2f, %.2f, %.2f): no room to turn, %s, and "
                "no room to turn within %.1f m straight ahead%s, turned by "
                "up to %.0f deg; no path (%d times so far)",
                root_state.x(), root_state.y(), root_state.z(), why,
                mgg::kDepartureMaxM,
                planning_params_.departure_reverse_allowed ? " or back" : "",
                mgg::kDepartureMaxTurnRad * 180.0 / M_PI,
                boxed_in_without_departure_);
  }
  return stored_note + note;
}

bool PlannerNode::straightDeparture(const mgg::StateVec& start,
                                    mgg::Departure& departure, bool arrival_band) {
  StandingStartScope standing_scope(*this);
  mgg::GroundProjection ground(*map_, planning_params_);
  ground.setStandingStart(standingStart());
  if (no_go_.inside(start.head<3>())) {
    // A bounded straight recovery, not a turn on the slope. Only a fully
    // outward path ending outside every zone can replace the empty choice.
    int endpoints_without_room = 0;
    const bool found = mgg::findDeparture(*map_, ground, robot_params_, planning_params_,
        start, departure, [this, &ground, &endpoints_without_room, arrival_band](const auto& path) {
          std::vector<Eigen::Vector3d> points;
          for (const auto& pose : path) points.push_back(pose.template head<3>());
          if (!no_go_.pathAdmissible(points)) return false;
          const auto& end = path.back();
          if (!standingStartGoalAdmissible(end)) return false;
          if (arrival_band) return refugeArrivalBandAdmissible(path, false);
          const double slope = mgg::groundSlope(ground, end.template head<3>(),
              std::max(robot_params_.size.x(), robot_params_.size.y()),
              local_graph_.get());
          if ((slope > mgg::kLevelGroundSlopeRad &&
               slope < mgg::kUnknownSlopeRad) ||
              mgg::roomToTurn(*map_, robot_params_, planning_params_, end,
                              ground.standingStart())) return true;
          ++endpoints_without_room;
          return false;
        }, 3.0, /*straight_only=*/true);
    if (endpoints_without_room > 0) {
      RCLCPP_INFO(get_logger(),
                  "zone escape: %d outside-zone endpoints refused for lack of "
                  "turning room", endpoints_without_room);
    }
    return found;
  }
  return mgg::findDeparture(*map_, ground, robot_params_, planning_params_,
                            start, departure, [this, &ground, arrival_band](const auto& path) {
                              if (arrival_band) return standingStartGoalAdmissible(path.back()) &&
                                  refugeArrivalBandAdmissible(path, false);
                              return standingStartGoalAdmissible(path.back()) &&
                                  mgg::roomToTurn(*map_, robot_params_, planning_params_,
                                                  path.back(), ground.standingStart());
                            });
}

bool PlannerNode::routeOverGlobalGraph(const mgg::StateVec goal,
                                       double goal_tolerance,
                                       std::vector<mgg::StateVec>& path,
                                       mgg::PathOkFn& turns_ok,
                                       std::string& reason) {
  path.clear();
  turns_ok = nullptr;
  last_route_starts_with_turn_without_room_ = false;
  last_route_blocked_by_peer_ = false;
  if (global_graph_->getNumVertices() == 0) {
    reason = "the global graph is empty";
    return false;
  }
  // Let's try to add the current state to the global graph (rrg.cpp:5643).
  // Without mapped ground under the robot its state is the physical anchor,
  // which links to a vertex it practically coincides with (the root at a
  // standing start) and otherwise needs a checked edge.
  mgg::StateVec current = current_state_;
  if (!projectToDrivingHeight(current)) {
    current = physicalAnchorAtDrivingHeight(current_state_);
  }
  // The drone departs from its latest breadcrumb: store those the map has
  // observed since.
  advanceFlownTrail(/*sample_pose=*/false);
  const mgg::ExpandContext ctx = makeGlobalContext();
  // An existing home/goal vertex is not free-space evidence. In particular,
  // home over the pad must be observed (or supplied as traversed-column
  // evidence by the map producer), and later occupied evidence wins.
  if (robot_params_.type == mgg::RobotType::kAerialRobot &&
      map_->getStaticStrictBoxStatus(goal.head<3>() + robot_params_.center_offset,
                                     ctx.robot_box_size) != mgg::VoxelStatus::kFree) {
    reason = "aerial goal body is not observed free";
    return false;
  }
  const int before = global_graph_->getNumVertices();
  // A departure clear only along its centre line joins this route and
  // nothing else: the route starts at the robot, then that vertex.
  mgg::DepartureLink departure =
      mgg::linkDeparture(*global_graph_, current, ctx, kLinkRadius);
  // Only when no vertex of the graph is within an edge of the robot: a
  // robot wedged beside a healthy graph (its box refused) keeps it. The
  // rebuilt graph replaces it only if the robot's pose links to it. Not
  // for an aerial robot, which relinks as it flies (odometry, exploration
  // paths): its keyframes start on the pad, whose floor its box sits in,
  // and a rebuild cut the drone's home off (drone scout Task 17, C).
  if (departure.vertex == nullptr &&
      robot_params_.type == mgg::RobotType::kGroundRobot &&
      !globalGraphReaches(current) &&
      rebuildGlobalGraphFromKeyframes(
          RoadmapRebuildTrigger::kPoseUnlinkable,
          "the current pose cannot be linked",
          [&current, &ctx](mgg::GraphManager& graph) {
            return mgg::linkDeparture(graph, current, ctx, kLinkRadius)
                .vertex;
          })) {
    departure = mgg::linkDeparture(*global_graph_, current, ctx, kLinkRadius);
  }
  mgg::Vertex* link_vertex = departure.vertex;
  if (global_graph_->getNumVertices() != before) ++graph_revision_;
  if (link_vertex == nullptr) {
    reason = "current pose cannot be linked to the global graph";
    return false;
  }
  // Dijkstra fills a parent for every vertex and leaves an unreached one as
  // its own parent, so reachability is read from that, not from presence.
  mgg::ShortestPathsReport rep;
  const auto search = [this, &rep, link_vertex]() {
    rep.status = false;
    if (peer_diagnosis_deadline_.has_value()) {
      // A route only asked for to tell whether peers alone stop one:
      // bounded as every peer diagnosis is (review r0, I4).
      diagnosePeerSearch(link_vertex->id, rep);
    } else {
      global_graph_->findShortestPaths(link_vertex->id, rep);
    }
  };
  const auto reaches = [&rep, link_vertex](const mgg::Vertex& vertex) {
    if (vertex.id == link_vertex->id) return true;
    if (!rep.status) return false;
    const auto parent = rep.parent_id_map.find(vertex.id);
    return parent != rep.parent_id_map.end() && parent->second != vertex.id;
  };
  // The goal is the graph vertex within `goal_tolerance` (a frontier or the
  // home root), or else the goal itself is linked into the graph with
  // checked edges, exactly where it was asked for: a goal is never replaced
  // by a nearby roadmap vertex.
  mgg::Vertex* goal_vertex = nullptr;
  mgg::StateVec goal_state = goal;
  bool exact_goal = false;
  // Query endpoints are deliberately absent from the rendezvous index.
  if (robot_params_.type == mgg::RobotType::kAerialRobot && goal_tolerance > 0) {
    for (const int id : lifted_target_vertices_) {
      auto* candidate = findGlobalVertex(id);
      if (candidate && candidate->lifted_peer_target &&
          !global_graph_->edge_map_[id].empty() &&
          (candidate->state.head<3>() - goal.head<3>()).norm() <= goal_tolerance) {
        goal_vertex = candidate;
        break;
      }
    }
  }
  if (!goal_vertex && (goal_tolerance <= 0.0 ||
      !global_graph_->getNearestVertexInRange(&goal, goal_tolerance,
                                              &goal_vertex) ||
      goal_vertex == nullptr)) {
    exact_goal = true;
    const int before_goal = global_graph_->getNumVertices();
    if (projectGoalToDrivingHeight(goal_state)) {
      goal_vertex = mgg::connectStateToGraph(
          *global_graph_, goal_state, ctx, kGoalLinkRadius,
          /*exact_state=*/true);
    } else {
      // Another robot may have driven there: its roadmap is the evidence of
      // ground this robot's map does not have. Only a part of it the robot
      // can reach will do; the lattice below cannot bridge to unmapped
      // ground.
      search();
      goal_vertex = attachGoalToNeighbourRoadmap(goal, reaches);
      if (goal_vertex == nullptr) {
        reason = "no mapped ground under the goal";
        return false;
      }
      goal_state = goal_vertex->state;
    }
    if (global_graph_->getNumVertices() != before_goal) ++graph_revision_;
  }
  search();
  // A connector is a query edge, not stored-roadmap evidence. The named
  // local sensor policy can join a mapped endpoint through upper unknown
  // air while every stored edge and arbitrary global expansion stay strict.
  // Dijkstra has already established reachability from the current link.
  bool query_goal_connector = false;
  if (exact_goal && (goal_vertex == nullptr || !reaches(*goal_vertex)) &&
      robot_params_.type == mgg::RobotType::kGroundRobot) {
    const auto local = makeContext();
    if (local.unknown_body_above_center) {
      std::vector<mgg::Vertex*> candidates;
      global_graph_->getNearestVertices(&goal_state, planning_params_.edge_length_max, &candidates);
      std::sort(candidates.begin(), candidates.end(), [&](const mgg::Vertex* a, const mgg::Vertex* b) {
        const auto cost = [&](const mgg::Vertex* v) {
          const auto found = rep.distance_map.find(v->id);
          return found == rep.distance_map.end() ? INFINITY :
              found->second + (v->state.head<3>()-goal_state.head<3>()).norm();
        };
        return cost(a) < cost(b);
      });
      const mgg::Vertex target(-1, goal_state);
      for (auto* candidate : candidates) {
        mgg::planningCheckpoint();
        if (!candidate || !reaches(*candidate)) continue;
        mgg::ExpandGraphReport checked;
        if (!mgg::roadmapEdgeTraversable(local, *candidate, target, checked)) continue;
        goal_vertex = candidate;
        query_goal_connector = true;
        break;
      }
    }
  }
  // No single roadmap edge reaches the goal, or the one that does joins a
  // part of the roadmap the robot cannot reach. Known space may still turn
  // or narrow between the two: lay the local planner's lattice out around
  // the goal and bridge from it to the roadmap the robot can use.
  if (exact_goal && (goal_vertex == nullptr || !reaches(*goal_vertex))) {
    const char* single_edge = goal_vertex == nullptr
                                  ? "goal cannot be linked to the global graph"
                                  : "the goal's roadmap link is not reachable";
    mgg::GoalLatticeReport lattice;
    const int before_lattice = global_graph_->getNumVertices();
    goal_vertex = mgg::connectGoalThroughLattice(
        *global_graph_, goal_state, grid_params_, ctx, latticeHeading(),
        reaches, &lattice);
    if (global_graph_->getNumVertices() != before_lattice) ++graph_revision_;
    if (goal_vertex == nullptr) {
      char why[256];
      std::snprintf(why, sizeof(why),
                    "%s; goal lattice: %d vertices in %d sweep(s), %d "
                    "reached from the goal, %d bridge checks%s, none onto "
                    "the robot's roadmap",
                    single_edge, lattice.lattice_vertices, lattice.passes,
                    lattice.reachable_vertices, lattice.bridge_checks,
                    lattice.hit_check_limit ? " (capped)" : "");
      reason = why;
      return false;
    }
    RCLCPP_INFO(get_logger(),
                "goal linked through a lattice around it: %d lattice "
                "vertices in %d sweep(s), %d bridge checks, %d joined the "
                "roadmap",
                lattice.lattice_vertices, lattice.passes,
                lattice.bridge_checks, lattice.chain_vertices);
    search();
  }
  if (goal_vertex == nullptr) {
    reason = "goal cannot be linked to the global graph";
    return false;
  }
  if (goal_vertex->id == link_vertex->id && !departure.query_local && !query_goal_connector) {
    reason = "already at the goal";
    return false;
  }
  if (!reaches(*goal_vertex)) {
    reason = "no route over the global graph reaches the goal";
    // Whether only peer bodies close the way: the same search with them
    // left out reaches the goal. Then the route is to be tried again. Only
    // with peers in force, and within the search budget: cut short, it is
    // retried as well (review r0, I4).
    if (peersInForce() && !peer_edges_open_) {
      if (!diagnosePeerSearch(link_vertex->id, rep)) {
        last_route_blocked_by_peer_ = true;
        reason += "; whether only a peer blocks it was cut short by the "
                  "search budget";
      } else if (reaches(*goal_vertex)) {
        last_route_blocked_by_peer_ = true;
        reason = "every route over the global graph to the goal is blocked "
                 "by a peer";
      }
    }
    return false;
  }
  std::vector<mgg::Vertex*> route;
  if (goal_vertex->id == link_vertex->id) {
    route.push_back(goal_vertex);
  } else {
    global_graph_->getShortestPath(goal_vertex->id, rep, true, route);
  }
  if (departure.query_local) path.push_back(current);
  // The robot departs from where it stands, or from its link vertex.
  const Eigen::Vector3d route_start =
      departure.query_local ? current.head<3>() : link_vertex->state.head<3>();
  refreshNoGoZones();
  // An exploration route keeps to the scouting exclusions as it does to the
  // no-go zones; an objective's never does.
  mgg::NoGoZones exploration_zones;
  const bool scouting = exploration_route_ && !scouting_zones_.empty();
  if (scouting) {
    std::vector<Eigen::Vector2d> centres = no_go_.centres();
    std::vector<double> reaches = no_go_.reaches();
    centres.insert(centres.end(), scouting_zones_.centres().begin(),
                   scouting_zones_.centres().end());
    reaches.insert(reaches.end(), scouting_zones_.reaches().begin(),
                   scouting_zones_.reaches().end());
    exploration_zones.set(std::move(centres), std::move(reaches));
  }
  const mgg::NoGoZones& zones = scouting ? exploration_zones : no_go_;
  const auto zones_admit = [&zones, &path](const std::vector<mgg::Vertex*>& r) {
    std::vector<Eigen::Vector3d> points;
    for (const mgg::StateVec& s : path) points.push_back(s.head<3>());
    for (const mgg::Vertex* v : r) points.push_back(v->state.head<3>());
    return zones.pathAdmissible(points);
  };
  // Dijkstra's edges are open either way, so its route may leave a zone
  // the robot stands in and come back: the route that departs outward only
  // and never re-enters is searched for instead (review r1, R1-1).
  if (!zones.empty() && !route.empty() && !zones_admit(route)) {
    route = mgg::zoneRespectingRoute(*global_graph_, link_vertex->id,
                                     goal_vertex->id, route_start, zones);
    // Only the exclusions stand in the way: say so (an objective would go).
    if (route.empty() && scouting &&
        (no_go_.empty() ||
         !mgg::zoneRespectingRoute(*global_graph_, link_vertex->id,
                                   goal_vertex->id, route_start, no_go_)
              .empty())) {
      ++scouting_counters_.paths_refused;
      path.clear();
      reason = "every exploration route to the goal enters a scouting exclusion";
      return false;
    }
  }
  if (!route.empty()) {
    const std::vector<mgg::Vertex*> before_turns = route;
    turns_ok = applyRouteTurnRule(*global_graph_, /*slope_from_map=*/true,
                                  path, route, "global route");
    // A turn-compliant detour must keep out of the zones too; the route it
    // replaced does. That route turns where it may not, as a fallback does,
    // and its first turn is judged below like a fallback's.
    if (!zones.empty() && !zones_admit(route) && zones_admit(before_turns)) {
      route = before_turns;
      ++route_sharp_turn_fallbacks_;
      RCLCPP_WARN(get_logger(),
                  "global route to (%.2f, %.2f, %.2f): the only route turning "
                  "where it may enters a no-go zone; the zone-safe route, "
                  "which does not turn only where it may, is kept (%d such "
                  "routes so far)",
                  route.back()->state.x(), route.back()->state.y(),
                  route.back()->state.z(), route_sharp_turn_fallbacks_);
    }
  }
  for (const mgg::Vertex* v : route) path.push_back(v->state);
  if (query_goal_connector) {
    path.push_back(goal_state);
    reason = "request-only goal connector: above_sensor_fov + physical own-body history";
    RCLCPP_INFO(get_logger(), "%s (vertex %d -> %.2f, %.2f); not stored",
                reason.c_str(), goal_vertex->id, goal_state.x(), goal_state.y());
  }
  // Whether the route kept starts with a turn the robot has no room for,
  // judged on that route, whichever branch kept it (review r2, R2-2): the
  // boxed-in guard (depart_instead_of_turning_route) reads it.
  {
    std::vector<Eigen::Vector3d> kept;
    kept.reserve(path.size());
    for (const mgg::StateVec& state : path) kept.push_back(state.head<3>());
    last_route_starts_with_turn_without_room_ =
        routeStartsWithTurnWithoutRoom(kept);
  }
  if (path.size() < 2) {
    reason = "no route over the global graph reaches the goal";
    return false;
  }
  // A route left open by the searches may still turn back into a zone the
  // robot is leaving, or end in one (review r0, I-3).
  std::vector<Eigen::Vector3d> points;
  points.reserve(path.size());
  for (const mgg::StateVec& state : path) points.push_back(state.head<3>());
  if (!no_go_.pathAdmissible(points)) {
    path.clear();
    turns_ok = nullptr;
    reason = "the route enters a no-go zone";
    return false;
  }
  if (scouting && !scouting_zones_.pathAdmissible(points)) {
    ++scouting_counters_.paths_refused;
    path.clear();
    turns_ok = nullptr;
    reason = "the exploration route enters a scouting exclusion";
    return false;
  }
  // The searches leave out the edges a peer body closes; the route as it
  // is sent, the robot's lead-in onto the roadmap and any route kept for
  // its turns included, is checked whole.
  for (std::size_t i = 1; i < points.size(); ++i) {
    if (peerBlocksSegment(points[i - 1], points[i])) {
      path.clear();
      turns_ok = nullptr;
      last_route_blocked_by_peer_ = true;
      reason = "the route is blocked by a peer";
      return false;
    }
  }
  return true;
}

mgg::Vertex* PlannerNode::attachGoalToNeighbourRoadmap(
    const mgg::StateVec& goal, const mgg::UsableVertexFn& reachable) {
  const int own_id = static_cast<int>(planning_params_.robot_id);
  std::vector<std::pair<double, mgg::Vertex*>> candidates;
  for (const auto& [robot_id, vertices] : global_graph_->vertex_by_robot_id_) {
    if (robot_id == own_id) continue;
    if (global_graph_->isQuarantined(robot_id)) continue;
    for (const auto& entry : vertices) {
      mgg::Vertex* vertex = entry.second;
      if (vertex == nullptr || vertex->is_hanging ||
          global_graph_->isRetired(vertex->id) || !reachable(*vertex)) {
        continue;
      }
      const Eigen::Vector3d gap = vertex->state.head<3>() - goal.head<3>();
      if (gap.head<2>().norm() > kLinkRadius) continue;
      // Planar reach first; height only separates floors.
      candidates.emplace_back(gap.norm(), vertex);
    }
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
  const mgg::ExpandContext ctx = makeGlobalContext();
  for (const auto& [distance, vertex] : candidates) {
    (void)distance;
    // The goal where it was asked for, on the ground that robot drove.
    const mgg::StateVec state(goal[0], goal[1], vertex->state[2], goal[3]);
    // Refused only through space this robot knows to be occupied, as a
    // blind link is (linkStateToGraph): unknown space is what it lacks.
    const Eigen::Vector3d& offset = ctx.robot->center_offset;
    if (map_->getPathStatus(vertex->state.head<3>() + offset,
                            state.head<3>() + offset, ctx.robot_box_size,
                            false) == mgg::VoxelStatus::kOccupied) {
      continue;
    }
    const double length = (state.head<3>() - vertex->state.head<3>()).norm();
    if (length < 1e-9) return vertex;
    auto* goal_vertex =
        new mgg::Vertex(global_graph_->generateVertexID(), state);
    goal_vertex->robot_id = own_id;
    goal_vertex->parent = vertex;
    goal_vertex->distance = vertex->distance + length;
    vertex->children.push_back(goal_vertex);
    global_graph_->addVertex(goal_vertex);
    global_graph_->addEdge(goal_vertex, vertex, length);
    RCLCPP_INFO(get_logger(),
                "goal (%.2f, %.2f) has no mapped ground here; attached to "
                "robot %d's roadmap %.2f m away",
                goal[0], goal[1], vertex->robot_id, length);
    return goal_vertex;
  }
  return nullptr;
}

double PlannerNode::latticeHeading() const {
  // A ground robot's lattice turns with it. An aerial one runs along the
  // map's voxels (lane drone-door): its body is held on that grid, and its
  // edges then cross a grid-aligned opening square-on, not obliquely by the
  // drone's yaw (a 0.9 m door leaves less than a voxel to spare).
  if (robot_params_.type != mgg::RobotType::kAerialRobot) return current_state_[3];
  double heading = 0.0;
  if (mola_map_ != nullptr && mola_map_->gridHeading(heading)) return heading;
  return 0.0;
}

bool PlannerNode::routeOverLocalLattice(const mgg::StateVec& goal,
                                        std::vector<mgg::StateVec>& path,
                                        mgg::PathOkFn& turns_ok,
                                        std::string& reason) {
  path.clear();
  turns_ok = nullptr;
  last_route_starts_with_turn_without_room_ = false;
  // The goal has to lie in the box the lattice is laid out in (heading
  // aside: the box is square in the shipped configurations).
  const Eigen::Vector3d offset = goal.head<3>() - current_state_.head<3>();
  if (offset.x() < grid_params_.min_val.x() ||
      offset.x() > grid_params_.max_val.x() ||
      offset.y() < grid_params_.min_val.y() ||
      offset.y() > grid_params_.max_val.y()) {
    reason = "goal is outside the local lattice";
    return false;
  }
  // One plan's shared footprint lookups, as in buildLocalGraph.
  mgg::GroundProjection plan_ground(*map_, planning_params_,
                                    /*cache_footprint_ground=*/true);
  plan_ground.setStandingStart(standingStart());
  mgg::PlanProfile profile;
  plan_ground.setProfile(&profile);
  mgg::ExpandContext ctx = makeContext();
  ctx.ground = &plan_ground;
  if (peer_diagnosis_deadline_) ctx.deadline = peer_diagnosis_deadline_;
  mgg::LocalRouteResult local = mgg::routeOverLocalLattice(
      *local_graph_, current_state_, goal, grid_params_, ctx, latticeHeading());
  local_route_profile_ = std::to_string(local.lattice.vertices_added) +
                         " vertices; " + profile.summary();
  if (!local.routed) {
    reason = local.reason;
    if (local.lattice.hit_deadline) {
      if (peer_diagnosis_deadline_) peer_diagnosis_cut_short_ = true;
      reason += " (the lattice stopped at its time budget, " +
                std::to_string(lattice_time_budget_s_) + " s)";
    }
    return false;
  }
  std::vector<mgg::Vertex*>& route = local.route;
  turns_ok = applyRouteTurnRule(*local_graph_, /*slope_from_map=*/false, {},
                                route, "local lattice route");
  for (const mgg::Vertex* v : route) path.push_back(v->state);
  if (!noGoAdmissible(path)) {
    path.clear();
    turns_ok = nullptr;
    reason = "the route through the local lattice enters a no-go zone";
    return false;
  }
  return true;
}

bool PlannerNode::startPathAfterChassisSpin(std::vector<mgg::StateVec>& path,
                                             bool lattice_route) {
  if (robot_params_.type != mgg::RobotType::kGroundRobot || path.size() < 2 ||
      robot_params_.physicalOffsetForHeading(0).head<2>().norm() < 1e-9) return true;
  const auto refused = [&](const char* check, const mgg::StateVec& from,
                           const mgg::StateVec& to) {
    RCLCPP_WARN(get_logger(), "post-spin join refused: %s; root=(%.6f,%.6f,%.6f,%.6f) "
        "from=(%.6f,%.6f,%.6f,%.6f) to=(%.6f,%.6f,%.6f,%.6f)", check,
        current_state_[0],current_state_[1],current_state_[2],current_state_[3],
        from[0],from[1],from[2],from[3],to[0],to[1],to[2],to[3]);
    return false;
  };
  const double first_yaw = std::atan2(path[1].y()-path[0].y(),path[1].x()-path[0].x());
  if (std::abs(std::remainder(first_yaw-current_state_[3],2*M_PI)) <=
      mgg::kSharpTurnRad + 1e-9) return true;
  mgg::StateVec incoming = current_state_;
  if (!projectToDrivingHeight(incoming)) incoming = physicalAnchorAtDrivingHeight(current_state_);
  const auto standing = standingStart();
  const double slope = mgg::groundSlope(*ground_, incoming.head<3>(),
      std::max(robot_params_.size.x(),robot_params_.size.y()),local_graph_.get());
  if (slope > mgg::kLevelGroundSlopeRad && current_tilt_ >= 4*M_PI/180) return refused("start slope",incoming,path[1]);
  if (!mgg::roomToTurn(*map_,robot_params_,planning_params_,incoming,
                       standing ? &*standing : nullptr)) return refused("incoming turn room",incoming,path[1]);
  const Eigen::Vector2d centre = incoming.head<2>() +
      robot_params_.physicalOffsetForHeading(incoming[3]).head<2>();
  const Eigen::Vector2d offset = robot_params_.physicalOffsetForHeading(0).head<2>();
  auto ctx = lattice_route ? makeContext() : makeGlobalContext();
  // The first point is the reference AFTER the stationary spin. Join only
  // the first forward point on the initial route leg (no unchecked leap
  // over a corner). The outgoing yaw solves the offset-reference geometry.
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (i > 1) {
      const auto leg = (path[i]-path[0]).head<2>().eval();
      if (std::abs(std::remainder(std::atan2(leg.y(),leg.x())-first_yaw,2*M_PI)) > 1e-3)
        return refused("initial leg corner",incoming,path[i]);
    }
    const Eigen::Vector2d delta = path[i].head<2>()-centre;
    const double length = delta.norm();
    if (length <= offset.norm()+.01) continue;
    const double yaw = std::atan2(delta.y(),delta.x()) + std::asin(offset.y()/length);
    mgg::StateVec post = incoming;
    post.head<2>() = centre-robot_params_.physicalOffsetForHeading(yaw).head<2>();
    post[3] = yaw;
    if (!mgg::groundShortcutSegmentAdmissible(ctx,post.head<3>(),path[i].head<3>(),
          !lattice_route || ctx.stop_at_unknown || !ctx.allow_unknown_lattice_body))
      return refused("projected body/terrain sweep",post,path[i]);
    std::vector<mgg::StateVec> adjusted{post};
    adjusted.insert(adjusted.end(),path.begin()+i,path.end());
    if (!noGoAdmissible(adjusted)) return refused("no-go",post,path[i]);
    if (!peerAdmissible(adjusted)) return refused("peer",post,path[i]);
    mgg::PathTurnCheck turns(*local_graph_, robot_params_,
        [this, standing](const auto& pose) {
          return mgg::roomToTurn(*map_,robot_params_,planning_params_,pose,
                                 standing ? &*standing : nullptr);
        }, [this](const Eigen::Vector3d& at) {
          return mgg::groundSlope(*ground_,at,
              std::max(robot_params_.size.x(),robot_params_.size.y()),local_graph_.get());
        }, false, [this](const auto& a,const auto& b) {
          return mgg::turnTransitionClear(*map_,robot_params_,a,b);
        });
    turns.setRobotTilt(post.head<3>(), current_tilt_);
    mgg::PathType points;
    for (const auto& pose : adjusted) points.push_back(pose.head<3>());
    // The stationary start spin was checked above. Check every remaining
    // corner, including a changed arrival heading at the route join.
    if (!turns.admissible(points,yaw)) return refused("remaining corner",post,path[i]);
    // recordSentPath must still recognise the selected lattice route after
    // its reference is moved by the stationary chassis spin.
    if (lattice_route && path == lattice_path_) lattice_path_ = adjusted;
    path = std::move(adjusted);
    return true;
  }
  return refused("no forward join",incoming,path.back());
}

bool PlannerNode::routeStartsWithTurnWithoutRoom(
    const std::vector<Eigen::Vector3d>& points) {
  if (robot_params_.type != mgg::RobotType::kGroundRobot || points.size() < 2) {
    return false;
  }
  const double start_heading = current_state_[3];
  // Over the robot's length, as PathTurnCheck measures turns.
  const std::vector<double> turns = mgg::pathTurns(
      points, start_heading,
      std::max(robot_params_.size.x(), robot_params_.size.y()));
  mgg::StateVec start = current_state_;
  if (!projectToDrivingHeight(start)) start = physicalAnchorAtDrivingHeight(current_state_);
  const std::optional<mgg::StandingStart> standing = standingStart();
  if (turns.empty() || turns.front() <= mgg::kSharpTurnRad + 1e-9) return false;
  const auto* prior = !storedReverseExitApplies(start) && standing ? &*standing : nullptr;
  if (!mgg::roomToTurn(*map_, robot_params_, planning_params_, start, prior)) return true;
  return false;  // start spin keeps the incoming chassis centre fixed
}

mgg::PathOkFn PlannerNode::applyRouteTurnRule(
    mgg::GraphManager& graph, bool slope_from_map,
    const std::vector<mgg::StateVec>& lead_in,
    std::vector<mgg::Vertex*>& route, const char* route_name) {
  if (robot_params_.type != mgg::RobotType::kGroundRobot || route.empty()) {
    return nullptr;
  }
  mgg::SlopeFn slope;
  if (slope_from_map) {
    // Over the robot's length, the radius terrainSlope fits a lattice over.
    const double radius =
        std::max(robot_params_.size.x(), robot_params_.size.y());
    slope = [this, radius](const Eigen::Vector3d& position) {
      return mgg::groundSlope(*ground_, position, radius, local_graph_.get());
    };
  }
  const std::optional<mgg::StandingStart> standing = standingStart();
  const auto check = std::make_shared<mgg::PathTurnCheck>(
      graph, robot_params_,
      [this, standing](const mgg::StateVec& pose) {
        return mgg::roomToTurn(*map_, robot_params_, planning_params_, pose,
                               standing ? &*standing : nullptr);
      },
      slope, false, [this](const auto& a, const auto& b) {
        return mgg::turnTransitionClear(*map_, robot_params_, a, b);
      });
  const double start_heading = current_state_[3];
  std::vector<Eigen::Vector3d> lead_in_points;
  for (const mgg::StateVec& s : lead_in) lead_in_points.push_back(s.head<3>());
  // Where the route starts is where the robot stands.
  check->setRobotTilt(lead_in_points.empty() ? route.front()->state.head<3>()
                                             : lead_in_points.front(),
                      current_tilt_);
  const mgg::RouteTurnChoice choice = mgg::chooseTurnCompliantRoute(
      graph, *check, lead_in_points, start_heading, route,
      mgg::kMaxDetourSearchStates);
  const Eigen::Vector3d end = route.back()->state.head<3>();
  if (choice.detour) {
    RCLCPP_INFO(get_logger(),
                "%s to (%.2f, %.2f, %.2f) goes the long way round: the "
                "shortest turns on a slope or without room (%d search "
                "states)",
                route_name, end.x(), end.y(), end.z(),
                choice.states_expanded);
  } else if (choice.fallback) {
    ++route_sharp_turn_fallbacks_;
    std::vector<Eigen::Vector3d> points = lead_in_points;
    for (const mgg::Vertex* v : route) points.push_back(v->state.head<3>());
    last_route_starts_with_turn_without_room_ =
        routeStartsWithTurnWithoutRoom(points);
    RCLCPP_WARN(get_logger(),
                "%s to (%.2f, %.2f, %.2f) turns sharply on a slope or "
                "without room to turn: no route turns only where it may; "
                "route search %s in %d states (%d such routes so far)",
                route_name, end.x(), end.y(), end.z(),
                choice.capped ? "stopped at its cap, a compliant route may "
                                "exist,"
                              : "completed",
                choice.states_expanded, route_sharp_turn_fallbacks_);
  }
  return [check, start_heading](const mgg::PathType& points) {
    return check->admissible(points, start_heading);
  };
}

bool PlannerNode::runGlobalPlanner(int target_id, std::string& reason,
                                   double min_gain) {
  StandingStartScope standing_scope(*this);
  best_path_.clear();
  best_path_from_global_graph_ = false;
  global_search_cut_short_ = false;
  global_frontier_not_routed_ = false;
  global_target_refused_by_retention_.reset();
  global_route_at_target_ = false;
  last_route_blocked_by_peer_ = false;
  refreshScoutingExclusions();
  const FlagScope exploring(exploration_route_);
  if (global_graph_->getNumVertices() <= 1) {
    // rrg.cpp:5582.
    reason = "the global graph holds no frontier to reposition to";
    return false;
  }
  // A frontier the fleet has covered is neither resumed nor chosen.
  demoteFleetCoveredFrontiers();
  const auto inside_region = [this](const mgg::Vertex& vertex) {
    if (scoutingExcludes(vertex.state.head<3>())) {
      ++scouting_counters_.targets_refused;
      return false;
    }
    return !reverseExitEndpointExcluded(vertex.state) &&
           (!exploration_region_ || exploration_region_->isInsideSpace(vertex.state.head<3>()));
  };
  if (target_id >= 0) {
    const auto* target = findGlobalVertex(target_id);
    if (target && reverseExitEndpointExcluded(target->state)) {
      global_target_refused_by_retention_ = target->state.head<3>();
      reason = "target excluded after reverse-exit retention refusal";
      return false;
    }
  }
  mgg::Vertex* target = nullptr;
  if (target_id >= 0) {
    // Resuming the repositioning that was under way (rrg.cpp:5556).
    target = findGlobalVertex(target_id);
    if (target == nullptr ||
        (target->type != mgg::VertexType::kFrontier &&
         !(target->lifted_peer_target && !global_graph_->edge_map_[target_id].empty())) ||
        !inside_region(*target)) {
      target = nullptr;  // reached, demoted or excluded: choose anew
    }
  }
  if (target == nullptr) {
    mgg::StateVec current = current_state_;
    if (!projectToDrivingHeight(current)) {
      current = physicalAnchorAtDrivingHeight(current_state_);
    }
    const int before = global_graph_->getNumVertices();
    mgg::Vertex* link_vertex =
        mgg::linkDeparture(*global_graph_, current, makeGlobalContext(),
                           kLinkRadius)
            .vertex;
    if (global_graph_->getNumVertices() != before) ++graph_revision_;
    if (link_vertex == nullptr) {
      reason = "current pose cannot be linked to the global graph";
      return false;
    }
    // A peer's reservation, and with fleet assignment the clusters other
    // robots hold or peers explored (tour-exploration design §3.5): the
    // greedy fallback takes none of them either.
    std::vector<Eigen::Vector3d> excluded = selectionExclusions();
    const std::vector<Eigen::Vector3d> fleet_excluded = fleetExclusions();
    excluded.insert(excluded.end(), fleet_excluded.begin(),
                    fleet_excluded.end());
    const double exclusion_radius =
        fleet_ ? std::max(reservation_exclusion_radius_m_,
                          fleet_params_.cluster_merge_radius_m)
               : reservation_exclusion_radius_m_;
    const Eigen::Vector3d robot_position = current_state_.head<3>();
    const mgg::GlobalFrontierReport report = mgg::searchGlobalFrontier(
        *global_graph_, link_vertex->id,
        static_cast<int>(planning_params_.robot_id), globalFrontierGain(),
        excluded, exclusion_radius,
        exploration_target_.has_value() ? &*exploration_target_ : nullptr,
        planning_params_.global_search_time_budget_s, &robot_position,
        reach_distance_, inside_region,
        lattice_deadline_ ? std::optional(innerDeadline(lattice_deadline_,
            planning_params_.global_search_time_budget_s)) : std::nullopt);
    global_space_.setCenter(current_state_, /*use_extension=*/true);
    // Cut short, the search is no answer whether or not it found a
    // frontier: the one it found may yet fail to route (review r0, I-6).
    global_search_cut_short_ = report.cut_short();
    if (report.best_frontier == nullptr) {
      // rrg.cpp:5628 and 5759: no frontier, or none the graph can reach.
      char why[224];
      std::snprintf(why, sizeof(why),
                    "%d global frontier(s), none reachable with gain (%d "
                    "own re-checked, %d demoted%s)",
                    report.frontiers, report.rechecked, report.demoted,
                    report.cut_short()
                        ? (", " + std::to_string(report.unchecked) +
                           " left unchecked by the time budget")
                              .c_str()
                        : "");
      reason = why;
      if (report.within_reach > 0) {
        // Still holding gain, but PCI would consider their goals reached:
        // these frontiers were skipped, not explored. Retry, not complete.
        global_frontier_not_routed_ = true;
        global_exploration_ongoing_ = false;
        current_global_vertex_id_ = -1;
        reason += "; " + std::to_string(report.within_reach) +
                  " frontier(s) within the controller goal tolerance";
      }
      // Frontiers only peer bodies keep the robot from are still to be
      // explored: the search is retried, not exploration complete. Only
      // with peers in force, and within the search budget: cut short, the
      // search is retried as well (review r0, I4).
      mgg::ShortestPathsReport open;
      if (peersInForce() && !report.cut_short() &&
          !diagnosePeerSearch(link_vertex->id, open)) {
        global_frontier_not_routed_ = true;
        last_route_blocked_by_peer_ = true;
        reason += "; whether frontiers lie behind a peer was cut short by "
                  "the search budget";
      } else if (open.status) {
        for (const auto& [id, vertex] : global_graph_->vertices_map_) {
          // Only a frontier the search could take: one outside the
          // exploration region is left out, peer or no peer.
          if (vertex == nullptr || vertex->type != mgg::VertexType::kFrontier ||
              !global_graph_->inService(*vertex) ||
              vertex->vol_gain.gain <= 0.0 || !inside_region(*vertex)) {
            continue;
          }
          const auto parent = open.parent_id_map.find(id);
          if (!open.status || parent == open.parent_id_map.end() ||
              parent->second == id) {
            continue;
          }
          const bool is_excluded = std::any_of(
              excluded.begin(), excluded.end(),
              [&](const Eigen::Vector3d& point) {
                return (vertex->state.head<3>() - point).norm() <=
                       exclusion_radius;
              });
          if (is_excluded) continue;
          global_frontier_not_routed_ = true;
          last_route_blocked_by_peer_ = true;
          reason += "; the frontiers left are blocked by a peer";
          break;
        }
      }
      return false;
    }
    if (report.best_gain < min_gain) {
      char why[192];
      std::snprintf(why, sizeof(why),
                    "the best of %d global frontier(s) has gain %.1f, under "
                    "the low-gain threshold %.1f",
                    report.frontiers, report.best_gain, min_gain);
      reason = why;
      return false;
    }
    RCLCPP_INFO(get_logger(),
                "global planner: %d frontier(s), %d feasible; repositioning to "
                "(%.2f, %.2f, %.2f), gain %.1f over %.1f m",
                report.frontiers, report.feasible,
                report.best_frontier->state.x(),
                report.best_frontier->state.y(),
                report.best_frontier->state.z(), report.best_gain,
                report.best_distance);
    target = report.best_frontier;
  }
  // Route to it over the global graph (rrg.cpp:5846), whole. Routing may
  // rebuild the graph (the robot's pose not linking), which frees `target`:
  // its state and id are copied first, and a repositioning whose graph was
  // replaced under it is given up rather than kept on an id of the old one.
  const mgg::StateVec target_state = target->state;
  const int target_vertex_id = target->id;
  target = nullptr;
  const int rebuilds_before = roadmap_rebuilds_;
  mgg::PathOkFn turns_ok;
  const bool routed = routeOverGlobalGraph(target_state, 1e-3, best_path_,
                                           turns_ok, reason);
  // A frontier found but not routed to is still a frontier: the search's
  // failure is no answer, not "none left".
  if (!routed || roadmap_rebuilds_ != rebuilds_before) {
    global_frontier_not_routed_ = true;
  }
  if (roadmap_rebuilds_ != rebuilds_before) {
    best_path_.clear();
    global_exploration_ongoing_ = false;
    current_global_vertex_id_ = -1;
    reason = "the global graph was rebuilt from the keyframes while routing "
             "to the frontier" +
             std::string(routed ? "" : " (" + reason + ")") +
             "; the repositioning is given up";
    return false;
  }
  if (!routed) return false;
  if (!best_path_.empty() && !standingStartGoalAdmissible(best_path_.back())) {
    best_path_.clear();
    global_frontier_not_routed_ = true;
    global_exploration_ongoing_ = false;
    current_global_vertex_id_ = -1;
    reason = "the goal intersects the standing-start arrival band";
    return false;
  }
  shortcutAndResample(best_path_, turns_ok);
  // A repositioning's end on a slope, where the robot may not turn, needs
  // a way back as a lattice path's does (review r1, R1-3): the route is cut
  // back to its last end with one, or not taken.
  if (robot_params_.type == mgg::RobotType::kGroundRobot &&
      !last_route_starts_with_turn_without_room_ && best_path_.size() >= 2) {
    const double radius =
        std::max(robot_params_.size.x(), robot_params_.size.y());
    // The map's slope at each pose, from nine ground points round it, is
    // noisy where the ground is patchy: a pose is taken as on the slope
    // when the map puts any pose within a robot's length along the route
    // on one.
    std::vector<double> along(best_path_.size(), 0.0);
    std::vector<bool> sloped(best_path_.size(), false);
    for (std::size_t i = 0; i < best_path_.size(); ++i) {
      if (i > 0) {
        along[i] = along[i - 1] + (best_path_[i].head<3>() -
                                   best_path_[i - 1].head<3>()).norm();
      }
      sloped[i] = mgg::slopeExemptsTurnSpace(
          mgg::groundSlope(*ground_, best_path_[i].head<3>(), radius,
                           local_graph_.get()));
    }
    const auto on_slope = [&](std::size_t i) {
      auto arrival = best_path_[i];
      arrival[3] = mgg::kUnknownTurnHeading;
      bool near_slope = false;
      for (std::size_t j = 0; j < best_path_.size() && !near_slope; ++j) {
        near_slope = sloped[j] && std::abs(along[j] - along[i]) <= radius;
      }
      return !mgg::viewpointClear(*map_, robot_params_, planning_params_,
                                  best_path_[i]) ||
             (near_slope && !mgg::turnSpaceObserved(*map_, robot_params_,
                                                   planning_params_,
                                                   arrival));
    };
    mgg::GroundProjection reverse_ground(*map_, planning_params_, true);
    std::map<std::pair<std::size_t, std::size_t>, bool> reverse_edges;
    if (!mgg::cutBackToWayBack(
            best_path_, on_slope,
            [this](std::size_t i) {
              return refugeArrivalBandAdmissible({best_path_.rbegin(), best_path_.rend() - i});
            }, planning_params_.departure_reverse_allowed,
            [&](std::size_t a, std::size_t b) {
              const auto key = std::make_pair(a, b);
              const auto found = reverse_edges.find(key);
              if (found != reverse_edges.end()) return found->second;
              return reverse_edges[key] = reverseExitEdge(
                  reverse_ground, best_path_[a], best_path_[b]);
            }, planning_params_.reverse_exit_max_length)) {
      best_path_.clear();
      global_frontier_not_routed_ = true;
      reason = "the route ends on a slope or in a narrow passage with no way back";
      return false;
    }
  }
  // Ground ends have already passed the single slope/narrow way-back
  // predicate; pulling them back for clearance again would discard valid
  // targets inside passages. Aerial routes retain the clearance cutback.
  if (robot_params_.type != mgg::RobotType::kGroundRobot &&
      best_path_.size() >= 2) {
    mgg::pullBackToClearViewpoint(best_path_, [this](const mgg::StateVec& pose) {
      return mgg::viewpointClear(*map_, robot_params_, planning_params_, pose);
    });
  }
  if (!best_path_.empty() && !standingStartGoalAdmissible(best_path_.back())) {
    best_path_.clear();
    global_frontier_not_routed_ = true;
    global_exploration_ongoing_ = false;
    current_global_vertex_id_ = -1;
    reason = "the goal intersects the standing-start arrival band";
    return false;
  }
  // Check the final end, after shortcutting and all viewpoint cutbacks.
  // Run 11: a nearby high-gain tour target repeatedly replaced a useful
  // local path with a route PCI rejected as already reached. The same
  // planar tolerance applies to non-tour repositioning as well.
  if (best_path_.empty() ||
      (best_path_.back().head<2>() - current_state_.head<2>()).norm() <=
          reach_distance_) {
    best_path_.clear();
    global_frontier_not_routed_ = true;
    global_route_at_target_ =
        (target_state.head<2>() - current_state_.head<2>()).norm() <=
            reach_distance_;
    global_exploration_ongoing_ = false;
    current_global_vertex_id_ = -1;
    reason = "the global route makes no progress beyond the controller goal "
             "tolerance";
    return false;
  }
  best_path_from_global_graph_ = true;
  // rrg.cpp:5838 to 5843: this frontier is the target until it is reached.
  current_global_vertex_id_ = target_vertex_id;
  global_exploration_ongoing_ = true;
  return true;
}

// ---------------------------------------------------------------------------
// Services

void PlannerNode::onBuildRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  const auto generation = request_generation_.load();
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  applyPendingCancel();
  auto map_read = mapReadLease();
  const bool admitted = map_read.hasSnapshot();
  const auto map_generation = mola_map_ ? mola_map_->activeGeneration() : 0;
  mgg::PlanningCancellationScope cancellation([this, generation, admitted, map_generation] {
    return generation != request_generation_.load() ||
        (admitted && (!mola_map_->authorityValid() ||
                      mola_map_->activeGeneration() != map_generation));
  });
  try {
    mgg::planningCheckpoint();
    applyLatestOdometry();
    response->message = buildLocalGraph();
    response->success = local_graph_->getNumVertices() > 1;
    publishPath();
    publishMarkers();
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  } catch (const mgg::PlanningInterrupted&) {
    ++cancellations_;
    ++graph_revision_;
    best_path_.clear();
    response->success = false;
    response->message = "planning cancelled";
  }
}

void PlannerNode::cancelPlanning() {
  // No planner work on the dedicated input thread, even when currently idle.
  std::lock_guard<std::mutex> fence(cancellation_mutex_);
  ++request_generation_;
  pending_cancel_clear_ = true;
}

void PlannerNode::applyPendingCancel() {
  // Caller holds planner_mutex_. Consume before admitting new work, or on the
  // maintenance tick after background work releases the planner lock.
  std::lock_guard<std::mutex> fence(cancellation_mutex_);
  if (!pending_cancel_clear_.exchange(false)) return;
  const bool had_path = !best_path_.empty();
  best_path_.clear();
  global_exploration_ongoing_ = false;
  if (exploration_target_) {
    exploration_target_.reset();
    publishPlannerConfigState();
  }
  if (had_path) publishPathUnderCancellationFence();
}

void PlannerNode::cancelExplorationPlanning() {
  // Fence only exploration requests (including those waiting for the planner).
  // Do not clear shared path state: it may belong to a newer operator objective.
  std::lock_guard<std::mutex> fence(cancellation_mutex_);
  ++exploration_generation_;
}

void PlannerNode::onPlanRequest(
    const std::shared_ptr<mgg_msgs::srv::PlannerSrv::Request> request,
    std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response) {
  const auto generation = request_generation_.load();
  const auto exploration_generation = exploration_generation_.load();
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  applyPendingCancel();
  RequestActivity activity(request_active_);
  auto map_read = mapReadLease();
  if (mola_map_) {
    if (const auto snapshot = mola_map_->activeRequest()) {
      std::atomic_store(&last_planning_snapshot_,
          std::make_shared<const mgg::MolaSnapshotRequest>(*snapshot));
      RCLCPP_INFO(get_logger(), "plan identity: component=%s epoch=%llu geometry_revision=%s",
          snapshot->component_id.c_str(), static_cast<unsigned long long>(snapshot->epoch),
          snapshot->geometry_revision.c_str());
    }
  }
  const bool admitted = map_read.hasSnapshot();
  const auto map_generation = mola_map_ ? mola_map_->activeGeneration() : 0;
  const auto bound = robot_params_.bound_mode;
  mgg::PlanningCancellationScope cancellation([this, generation, exploration_generation,
                                               admitted, map_generation]() {
    return request_generation_.load() != generation ||
           exploration_generation_.load() != exploration_generation ||
           (admitted && (!mola_map_->authorityValid() ||
                         mola_map_->activeGeneration() != map_generation));
  });
  try {
    mgg::planningCheckpoint();
    const bool computed = onPlanRequestImpl(request, response);
    std::lock_guard<std::mutex> fence(cancellation_mutex_);
    mgg::planningCheckpoint();
    // This is the commit point. No cancellation checkpoint may run after it:
    // a later cancel belongs after this answer, not to an unsent computation.
    if (computed) {
      recordSentPath();
      if (!best_path_.empty()) setAcquiringObservations(false);
      publishPathUnderCancellationFence();
      if (planner_config_state_.last_plan_generation != planner_config_state_.generation) {
        planner_config_state_.last_plan_generation = planner_config_state_.generation;
        publishPlannerConfigState();
      }
    }
  } catch (const mgg::PlanningInterrupted&) {
    ++cancellations_;
    ++graph_revision_;
    local_graph_->reset();
    best_path_.clear();
    // A route probe also pre-empts exploration. Keep its target/repositioning;
    // explicit general cancellation clears them through applyPendingCancel().
    response->path.clear();
    response->status = mgg_msgs::srv::PlannerSrv::Response::CANCELLED;
    RCLCPP_INFO(get_logger(), "planning cancelled: superseded or map authority expired/changed");
  }
  robot_params_.bound_mode = bound;
}

void PlannerNode::onObjectiveRequest(
    const std::shared_ptr<mgg_msgs::srv::PlanObjective::Request> request,
    std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> response) {
  std::uint64_t generation;
  {
    std::lock_guard<std::mutex> fence(cancellation_mutex_);
    generation = ++request_generation_;
  }
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  applyPendingCancel();
  RequestActivity activity(request_active_);
  auto map_read = mapReadLease();
  if (mola_map_) {
    if (const auto snapshot = mola_map_->activeRequest()) {
      std::atomic_store(&last_planning_snapshot_,
          std::make_shared<const mgg::MolaSnapshotRequest>(*snapshot));
      RCLCPP_INFO(get_logger(), "plan identity: component=%s epoch=%llu geometry_revision=%s",
          snapshot->component_id.c_str(), static_cast<unsigned long long>(snapshot->epoch),
          snapshot->geometry_revision.c_str());
    }
  }
  const bool admitted = map_read.hasSnapshot();
  const auto map_generation = mola_map_ ? mola_map_->activeGeneration() : 0;
  const auto* outer_cancel = mgg::planning_cancelled;
  bool outer_cancelled = false;
  mgg::PlanningCancellationScope cancellation([this, generation, admitted, map_generation, outer_cancel, &outer_cancelled]() {
    outer_cancelled = outer_cancelled || (outer_cancel && (*outer_cancel)());
    return outer_cancelled || request_generation_.load() != generation ||
           (admitted && (!mola_map_->authorityValid() ||
                         mola_map_->activeGeneration() != map_generation));
  });
  try {
    mgg::planningCheckpoint();
    // NAVIGATE is also used for route probes. The adapter explicitly clears
    // exploration before operator objectives; probes must preserve its state.
    onObjectiveRequestImpl(request, response);
    std::lock_guard<std::mutex> fence(cancellation_mutex_);
    mgg::planningCheckpoint();
  } catch (const mgg::PlanningInterrupted&) {
    ++cancellations_;
    ++graph_revision_;
    local_graph_->reset();
    best_path_.clear();
    global_exploration_ongoing_ = false;
    response->path.clear();
    const bool superseded = request_generation_.load() != generation ||
                            outer_cancelled;
    response->status = superseded ? mgg_msgs::srv::PlanObjective::Response::BLOCKED
                                : mgg_msgs::srv::PlanObjective::Response::STALE_REVISION;
    response->reason = "planning cancelled: superseded or map authority expired/changed";
    refreshMapRevision();
    if (mola_map_) {
      response->component_id = mapping_snapshot_.component_id;
      response->map_epoch = mapping_snapshot_.epoch;
      response->mapping_graph_revision = mapping_snapshot_.graph_revision;
      response->geometry_revision = mapping_snapshot_.geometry_revision;
      response->map_source_stamp = mapping_snapshot_.source_stamp;
    }
  }
}

bool PlannerNode::onPlanRequestImpl(
    const std::shared_ptr<mgg_msgs::srv::PlannerSrv::Request> request,
    std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response) {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  applyLatestOdometry();
  refreshMapRevision();
  StandingStartScope standing_scope(*this);
  const DeadlineScope budget(lattice_deadline_,
      robot_params_.type == mgg::RobotType::kGroundRobot ? lattice_time_budget_s_ : 0.0);
  const auto request_bound_mode = robot_params_.bound_mode;
  try {
  // One peer set for the whole request (review r0, I5).
  std::optional<PeerBodyPin> peer_pin;
  pinPeerBodies(peer_pin);
  departure_sent_now_ = false;
  stored_reverse_sent_now_ = false;
  lattice_path_.clear();
  peer_blocked_edges_.clear();
  peer_cylinder_blocked_edges_.clear();
  refreshScoutingExclusions();
  withdrawUnplacedNeighbours();
  response->planning_bound_mode = request->bound_mode;
  if (home_state_wait_started_) {
    response->status = kStatusNotReady;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "plan request refused: waiting for flight_state");
    return false;
  }
  if (!have_odometry_ || !map_->getStatus()) {
    response->status = kStatusNotReady;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "plan request refused: %s",
                         have_odometry_ ? "acquiring observations" : "no odometry");
    setAcquiringObservations(true);
    return false;
  }
  // The planner's state is its last odometry message; a plan from where the
  // robot was is completed at once by the controller where the robot is.
  if (secondsSince(last_odometry_received_) > odometry_stale_s_) {
    response->status = kStatusNotReady;
    RCLCPP_WARN(get_logger(), "plan request refused: odometry is %.1f s old",
                secondsSince(last_odometry_received_));
    return false;
  }
  const auto scouting_before = scouting_counters_;
  ++plan_requests_;
  expireReverseExitExclusions();
  for (const ReverseExitExclusion& exclusion : reverse_exit_exclusions_) {
    if (!exclusion.given_up && exclusion.excluded_through_request + 1 == plan_requests_) {
      RCLCPP_INFO(get_logger(),
                  "retention exclusion lifted after %llu plans: (%.2f, %.2f, %.2f) judged "
                  "again (%d of %d refusals)",
                  static_cast<unsigned long long>(kReverseExitExclusionPlans),
                  exclusion.position.x(), exclusion.position.y(), exclusion.position.z(),
                  exclusion.refusals, kReverseExitExclusionGiveUpRefusals);
    }
  }
  // A caller may pin the bound mode for this cycle, e.g. to squeeze through a
  // gap it would normally refuse.
  const RestoreScope restore_bound_mode(robot_params_.bound_mode);
  const mgg::BoundModeType previous = robot_params_.bound_mode;
  robot_params_.bound_mode =
      static_cast<mgg::BoundModeType>(request->bound_mode);
  // The zones' reach is half the planning box out, the box this request
  // plans with, for the lattice, the routes and the final check alike
  // (review r1, R1-2).
  refreshNoGoZones();
  // Every cycle plans afresh; the previous path was the previous answer.
  best_path_.clear();
  best_path_from_global_graph_ = false;

  std::string summary;
  std::string reason;
  std::optional<mgg::FrontierCluster> aerial_chosen_target;
  bool complete = false;
  // rrg.cpp:1229 to 1240: a global repositioning under way is resumed until
  // the robot is within reach of its frontier; then local exploration
  // takes over again.
  bool resume_global = false;
  if (global_exploration_ongoing_) {
    const mgg::Vertex* target = findGlobalVertex(current_global_vertex_id_);
    if (target != nullptr &&
        (current_state_.head<3>() - target->state.head<3>()).norm() >
            global_frontier_reach_m_ &&
        tourKeepsRoute(current_global_vertex_id_)) {
      resume_global = true;
    } else {
      global_exploration_ongoing_ = false;
    }
  }
  // A route over the global graph is kept when no route turns only where it
  // may. One whose first turn the robot has no room to make is the path a
  // boxed-in robot is not sent (buildLocalGraph): the repositioning is given
  // up and, as a boxed-in robot does, it departs straight ahead or back
  // instead, or gets no path when it has no departure.
  const auto depart_instead_of_turning_route = [this](std::string& note) {
    if (!last_route_starts_with_turn_without_room_) return false;
    best_path_from_global_graph_ = false;
    global_exploration_ongoing_ = false;
    mgg::StateVec root_state = current_state_;
    if (!projectToDrivingHeight(root_state)) {
      root_state = physicalAnchorAtDrivingHeight(current_state_);
    }
    note = departBoxedIn(root_state,
                         "the global route starts with a sharp turn");
    return true;
  };
  std::string resumed;
  // A resumed route withheld with no departure: the robot is boxed in with
  // no way out, as buildLocalGraph's boxed_in_without_departure_now_ says,
  // which buildLocalGraph resets.
  bool withheld_without_departure = false;
  if (resume_global) {
    auto map_read = mapReadLease();
    refreshMapRevision();
    std::string departure;
    if (!runGlobalPlanner(current_global_vertex_id_, reason)) {
      global_exploration_ongoing_ = false;
      // Kept in the summary of the local plan made instead: a route a peer
      // blocks is retried, not missing.
      resumed = "; global repositioning abandoned: " + reason;
      summary = resumed.substr(2);
    } else if (depart_instead_of_turning_route(departure)) {
      withheld_without_departure = best_path_.empty();
      resumed =
          "; global repositioning given up: its route starts with a turn "
          "the robot has no room for" +
          departure;
      summary = resumed.substr(2);
    } else {
      summary = "resuming global repositioning";
    }
  }
  if (best_path_.empty()) {
    const int departures_before = boxed_in_departures_;
    summary = buildLocalGraph() + resumed;
    // A boxed-in robot sent a straight departure (buildLocalGraph) keeps it.
    const bool departed = boxed_in_departures_ != departures_before;
    // Tour-exploration design §2.4: the tour's target, not low_gain_rounds,
    // decides when the robot leaves local exploration for the global graph:
    // as soon as the lattice has no path toward the target.
    std::optional<mgg::FrontierCluster> tour_target;
    if (tour_params_.enabled && local_graph_->getNumVertices() > 1) {
      auto map_read = mapReadLease();
      std::string tour_note;
      tour_target = refreshTour(tour_note);
      summary += tour_note;
    }
    if (tour_target && low_gain_path_now_ &&
        (tour_target->position - current_state_.head<3>()).norm() <=
            global_frontier_reach_m_) {
      setTourClusterAside(*tour_target, tour_params_.route_retry_s);
      tour_target.reset();
      summary += "; nearby tour target reached with low local gain";
    }
    bool tour_decided = false;
    if (tour_target.has_value() && !departed &&
        !boxed_in_without_departure_now_ && !withheld_without_departure) {
      if (!best_path_.empty() &&
          localPathServesTour(best_path_.back().head<3>(),
                              tour_target->position)) {
        tour_decided = true;
        summary += "; exploring locally toward the tour's target";
      } else if (robot_params_.type == mgg::RobotType::kAerialRobot &&
                 (tour_target->id == tour_reached_cluster_ ||
                  tour_at_target_failures_.count(tour_target->id) > 0) &&
                 (tour_target->position - current_state_.head<3>()).norm() <=
                     global_frontier_reach_m_) {
        // A reached cluster reselected now, or retried after an at-target
        // aside: the failure history survives the reached marker clearing.
        // drone-r4: keep local exploration instead of routing back inside
        // reach, even on the first request after a retry deadline lapses.
        // Fresh targets still route at once, even with no local path;
        // ground tours retain their closer approach to the representative.
        setTourClusterAside(*tour_target, tour_params_.route_retry_s, true);
        ++tour_in_reach_set_aside_;
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 10000,
            "tour: in-reach target set aside (at-target backoff), total %llu",
            static_cast<unsigned long long>(tour_in_reach_set_aside_));
        summary += "; in-reach tour target set aside";
        if (!best_path_.empty()) summary += "; exploring locally";
      } else {
        auto map_read = mapReadLease();
        const std::vector<mgg::StateVec> local_path = best_path_;
        std::string departure;
        const mgg::Vertex* source = linkRobotToGlobalGraph();
        if (source != nullptr &&
            source->id == tour_target->representative_vertex_id) {
          // The robot stands on the target's representative: reached, and
          // local exploration has nothing here. The tour costs it zero and
          // a route cannot leave from it, so it is set aside, not routed to.
          setTourClusterAside(*tour_target, tour_params_.route_retry_s);
          summary += "; the robot stands on the tour's target: reached";
        } else if (runGlobalPlanner(tour_target->representative_vertex_id,
                                    reason)) {
          // The low-gain rounds restart only once the tour sends something
          // the robot can drive (review r0, I1).
          if (!depart_instead_of_turning_route(departure)) {
            tour_decided = true;
            low_gain_rounds_ = 0;
            summary += "; routing to the tour's target over the global graph";
          } else if (!best_path_.empty() &&
                     (best_path_.back().head<2>() - current_state_.head<2>())
                             .norm() > reach_distance_) {
            tour_decided = true;
            low_gain_rounds_ = 0;
            summary +=
                "; the route to the tour's target starts with a turn the "
                "robot has no room for" +
                departure;
          } else {
            // Run 10b, robot_0: a route that starts with a turn the robot
            // has no room for, and no departure out, is no route at all.
            // Nor is a departure ending within PCI's tolerance (run 11).
            // The tour fails as it does when no route exists: the target is
            // set aside and the local path already checked is kept. Without
            // one, the choices without the tour follow; they judge their
            // own routes' first turns, not this one's.
            best_path_.clear();
            ++tour_routes_failed_;
            setTourClusterAside(*tour_target, tour_params_.route_retry_s);
            boxed_in_without_departure_now_ = false;
            summary +=
                "; the route to the tour's target starts with a turn the "
                "robot has no room for and it has no straight departure "
                "beyond the controller goal tolerance: the target is set aside";
            if (!local_path.empty()) {
              best_path_ = local_path;
              best_path_from_global_graph_ = false;
              summary += ", the local path kept";
            }
          }
        } else {
          // No route after all: the next solve chooses again, and this
          // cycle keeps what local exploration found.
          best_path_ = local_path;
          best_path_from_global_graph_ = false;
          // Blocked only by peer bodies, the target is set aside briefly:
          // the peer may move on.
          ++tour_routes_failed_;
          setTourClusterAside(
              *tour_target,
              last_route_blocked_by_peer_
                  ? std::min(kPeerBlockedAsideS, tour_params_.route_retry_s)
                  : tour_params_.route_retry_s,
              global_route_at_target_);
          summary += "; no route to the tour's target, set aside: " + reason;
        }
      }
    }
    // A tour that decided nothing has no target this cycle: its target was
    // set aside, and the fleet and the greedy planner choose without it
    // (review r0, I1).
    if (tour_decided) {
      tour_at_target_failures_.erase(tour_target->id);
      aerial_chosen_target = tour_target;
    }
    if (!tour_decided) tour_target.reset();
    // A low-gain lattice path, once the low-gain rounds are due and the
    // tour has not decided, is set aside for the fleet and the global
    // planner below, as no path would be; it is sent only when neither
    // gives the robot anywhere to go.
    std::vector<mgg::StateVec> low_gain_path;
    if (!tour_decided && low_gain_path_now_ && !best_path_.empty() &&
        local_graph_->getNumVertices() > 1 &&
        low_gain_rounds_ >= auto_global_planner_low_gain_rounds_) {
      low_gain_path.swap(best_path_);
    }
    // An empty lattice is a map that does not yet show the robot's
    // surroundings, not an explored one: PCI retries as the map grows. The
    // global planner is consulted once the lattice existed and saw nothing
    // new for long enough.
    const bool low_gain =
        best_path_.empty() && local_graph_->getNumVertices() > 1 &&
        low_gain_rounds_ >= auto_global_planner_low_gain_rounds_;
    // Design §3.5: with fleet assignment, a robot with nothing on its tour
    // and no local path asks for an auction, or takes a silent robot's
    // claims over, before the greedy fallback is consulted.
    const bool fleet_decided =
        !tour_decided && fleet_ && tour_params_.enabled &&
        !tour_target.has_value() && best_path_.empty() &&
        local_graph_->getNumVertices() > 1 && !departed &&
        !boxed_in_without_departure_now_ && !withheld_without_departure &&
        settleIdleRobot(summary, complete);
    if (tour_decided || fleet_decided) {
      // The tour or the fleet decided this cycle.
    } else if (low_gain &&
               (boxed_in_without_departure_now_ ||
                withheld_without_departure)) {
      // Boxed in with no way out: the global planner's route would start
      // with the turn the robot cannot make, and failing to find one would
      // not make exploration complete. No path, and the robot's own
      // recovery runs.
      summary += "; boxed in: no global repositioning";
    } else if (low_gain) {
      // No leaf with gain for long enough: the global planner routes to the
      // best global frontier (rrg.cpp:2119, mggplanner.cpp:217).
      auto map_read = mapReadLease();
      RCLCPP_INFO(get_logger(),
                  "global repositioning triggered: %d low-gain rounds "
                  "(threshold %d), %s; local frontier interest=%s",
                  low_gain_rounds_, auto_global_planner_low_gain_rounds_,
                  low_gain_path.empty() ? "no local path" : "low-gain local path",
                  local_gain_remains_now_ ? "yes" : "no");
      low_gain_rounds_ = 0;
      std::string departure;
      // A low-gain path set aside is handed over only for a frontier worth
      // low_gain_handoff_min_voxels.
      const double min_gain =
          low_gain_path.empty()
              ? 0.0
              : planning_params_.low_gain_handoff_min_voxels *
                    planning_params_.unknown_voxel_gain;
      if (!runGlobalPlanner(-1, reason, min_gain)) {
        if (local_gain_remains_now_) {
          // The lattice still sees gain it cannot send a path to; that is
          // not a finished exploration. No path, and PCI retries.
          summary += "; no global route (" + reason +
                     "), but local gain remains: no path";
        } else if (frontiers_dropped_in_rebuild_ > 0) {
          // A graph rebuild dropped this robot's frontiers since: the
          // first failed search after it is not exploration complete
          // (review r0, I-2). No path, and the next one may be.
          summary += "; no global route (" + reason + "), but a graph " +
                     "rebuild dropped " +
                     std::to_string(frontiers_dropped_in_rebuild_) +
                     " frontier(s): no path";
          frontiers_dropped_in_rebuild_ = 0;
        } else if (const std::string withheld = completionWithheld();
                   !withheld.empty()) {
          // Frontiers this robot cannot see now are not explored (run 8,
          // robot_1): no path, retried.
          summary += "; no global route (" + reason + "), but " + withheld +
                     ": no path";
        } else {
          complete = true;
          summary += "; exploration complete: " + reason;
        }
      } else if (depart_instead_of_turning_route(departure)) {
        summary +=
            "; no local gain, and the global route starts with a turn the "
            "robot has no room for" +
            departure;
      } else {
        summary += "; no local gain, repositioning over the global graph";
      }
    }
    if (!low_gain_path.empty()) {
      if (best_path_.empty()) {
        best_path_.swap(low_gain_path);
        best_path_from_global_graph_ = false;
        summary += "; keeps its low-gain local path";
      } else {
        ++low_gain_handoffs_;
        summary += "; low-gain local path handed over";
      }
    }
  }
  if (!departure_sent_now_ &&
      !startPathAfterChassisSpin(best_path_, !best_path_from_global_graph_)) {
    best_path_.clear();
    summary += "; post-spin reference cannot join the route safely";
  }
  summary += clearInadmissibleBestPath();
  if (!peer_blocked_edges_.empty()) {
    summary += "; " + std::to_string(peer_blocked_edges_.size()) +
               " global graph edge(s) blocked by peers";
    if (robot_params_.type == mgg::RobotType::kAerialRobot) {
      summary += " (" + std::to_string(peer_cylinder_blocked_edges_.size()) +
                 " by aerial cylinders, " +
                 std::to_string(peer_blocked_edges_.size() -
                                peer_cylinder_blocked_edges_.size()) +
                 " by legacy discs)";
    }
  }
  // Capture/recheck the escape for the actual route and bound mode sent.
  {
    auto map_read = mapReadLease();
    summary += rememberReverseExit();
  }
  robot_params_.bound_mode = previous;
  refreshNoGoZones();
  enforceSafeCompletion(complete);
  if (mola_map_ && robot_params_.type == mgg::RobotType::kGroundRobot &&
      acquiring_observations_.load() && best_path_.empty() &&
      local_graph_->getNumVertices() <= 1) {
    setAcquiringObservations(true);
    complete = false;
    summary += "; acquiring observations";
  }
  const bool scouting_refused = robot_params_.type == mgg::RobotType::kAerialRobot &&
      !scouting_zones_.empty() && best_path_.empty() &&
      (scouting_counters_.paths_refused > scouting_before.paths_refused ||
       scouting_counters_.viewpoints_refused > scouting_before.viewpoints_refused);
  // Only actual path/viewpoint refusals trigger the short retry, not lifted
  // target prefilters (which may be ineligible for other reasons). Mixed
  // physical refusals still retry: proving exclusion-only would require a
  // second search with exclusions disabled. Never weaken the real search.
  if (scouting_refused) complete = false;
  response->status = !best_path_.empty()
                         ? mgg_msgs::srv::PlannerSrv::Response::FORWARD
                     : complete ? kStatusComplete
                     : scouting_refused ? mgg_msgs::srv::PlannerSrv::Response::SCOUTING_EXCLUDED
                     : mola_map_ && robot_params_.type == mgg::RobotType::kGroundRobot &&
                         acquiring_observations_.load() ? kStatusNotReady
                                : kStatusNoPath;
  for (const mgg::StateVec& s : best_path_) {
    response->path.push_back(toPoseMsg(s));
  }
  publishMarkers();
  RCLCPP_INFO(get_logger(), "plan request: %s", summary.c_str());
  if (robot_params_.type == mgg::RobotType::kAerialRobot) {
    const auto* representative = aerial_chosen_target
        ? findGlobalVertex(aerial_chosen_target->representative_vertex_id)
        : best_path_from_global_graph_ ? findGlobalVertex(current_global_vertex_id_) : nullptr;
    const bool lifted = representative && representative->lifted_peer_target;
    const Eigen::Vector3d target = aerial_chosen_target ? aerial_chosen_target->position
        : representative ? representative->state.head<3>().eval()
        : best_path_.empty() ? current_state_.head<3>().eval() : best_path_.back().head<3>().eval();
    const double progress = best_path_.empty() ? mgg::kUnreachableCost : aerialGraphProgress(target);
    const std::string distance = std::isfinite(progress) ? std::to_string(progress) : "unknown";
    const double fleet_front = aerialFleetFront();
    const std::string fleet_distance = std::isfinite(fleet_front) ? std::to_string(fleet_front) : "unknown";
    RCLCPP_INFO(get_logger(), "aerial choice: target=%s target_progress_m=%s fleet_front_m=%s "
        "front_peer=%d front_quality=%s reason=%s",
        best_path_.empty() ? "none" : lifted ? "lifted" : "own",
        distance.c_str(), fleet_distance.c_str(), aerial_front_peer_,
        !std::isfinite(fleet_front) ? "unknown" : aerial_front_is_lower_bound_ ? "lower_bound" : "placed_peers",
        complete ? "exploration complete" : best_path_.empty() ? "no admissible path"
        : aerial_chosen_target ? (fleet_ ? "fleet-assigned tour target"
            : lifted ? (aerial_unknown_progress_clusters_.count(aerial_chosen_target->id)
                ? "unknown-progress fallback; no available known-forward target"
                : "not behind peer or drone; bounded graph-front bias")
            : std::isfinite(fleet_front) ? "own tour target; bounded graph-front bias"
                                         : "own tour target; no placed fresh peer front")
        : best_path_from_global_graph_ ? "global fallback or resumed route"
        : "local exploration before peer fallback");
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "aerial_status %s",
                         aerialStatusJson().c_str());
  }
  return true;
  } catch (const mgg::PlanningInterrupted&) {
    robot_params_.bound_mode = request_bound_mode;
    refreshNoGoZones();
    if (budget.cancelled_) throw;
    best_path_.clear();
    lattice_path_.clear();
    response->path.clear();
    response->status = kStatusNotReady;
    RCLCPP_WARN(get_logger(), "plan request refused: %s", budget.reason().c_str());
    return false;
  }
}

std::string PlannerNode::clearInadmissibleBestPath() {
  if (!best_path_.empty() && !standingStartGoalAdmissible(best_path_.back())) {
    best_path_.clear();
    return "; the goal intersects the standing-start arrival band: no path";
  }
  if (!noGoAdmissible(best_path_)) {
    // The last net: whatever produced it, nothing is sent into a zone. It
    // runs with the request's bound mode still applied.
    best_path_.clear();
    return "; the path enters a no-go zone: no path";
  }
  if (!scoutingPathAdmissible(best_path_)) {
    // Exploration only: a plan never takes the robot into a scouting
    // exclusion, though it may lead it out of one.
    best_path_.clear();
    return "; the path enters a scouting exclusion: no path";
  }
  if (!peerAdmissible(best_path_)) {
    // And nothing is sent through a peer body of the set this request
    // planned with.
    best_path_.clear();
    return "; the path meets a peer body: no path";
  }
  return {};
}

void PlannerNode::enforceSafeCompletion(bool& complete) {
  if (best_path_.empty() &&
      (standingStart() || std::any_of(reverse_exit_exclusions_.begin(),
                                      reverse_exit_exclusions_.end(),
                                      [this](const ReverseExitExclusion& exclusion) {
                                        // A given-up end withholds nothing:
                                        // it is unreachable from here.
                                        return !exclusion.given_up &&
                                               reverseExitExclusionActive(exclusion);
                                      }))) {
    complete = false;
  }
}

bool PlannerNode::endpointNeedsReverseExit(const mgg::StateVec& pose) const {
  auto arrival = pose;
  arrival[3] = mgg::kUnknownTurnHeading;
  const double slope = mgg::groundSlope(*ground_, pose.head<3>(),
      std::max(robot_params_.size.x(), robot_params_.size.y()), local_graph_.get());
  return !mgg::viewpointClear(*map_, robot_params_, planning_params_, pose, slope) ||
         (mgg::slopeExemptsTurnSpace(slope) &&
          !mgg::turnSpaceObserved(*map_, robot_params_, planning_params_, arrival));
}

bool PlannerNode::reverseExitShortcutAdmissible(const mgg::PathType& points) {
  std::vector<mgg::StateVec> route;
  for (const auto& point : points) route.emplace_back(point.x(), point.y(), point.z(), 0.0);
  mgg::GroundProjection ground(*map_, planning_params_, true);
  const bool found = mgg::cutBackToWayBack(route,
      [&](std::size_t i) { return endpointNeedsReverseExit(route[i]); },
      [&](std::size_t i) {
        return refugeArrivalBandAdmissible({route.rbegin(), route.rend() - i});
      },
      planning_params_.departure_reverse_allowed,
      [&](std::size_t a, std::size_t b) { return reverseExitEdge(ground, route[a], route[b]); },
      planning_params_.reverse_exit_max_length);
  return found && route.size() == points.size();
}

bool PlannerNode::refugeArrivalBandAdmissible(const std::vector<mgg::StateVec>& path,
                                               bool require_level) const {
  if (path.size() < 2) return false;
  // Accepted discretisation: overlapping turn circles at <= 0.1 m arc
  // steps, including both band ends. This is not a continuous-space proof.
  constexpr double kArrivalBandStepM = 0.1;
  const auto room = [&](const mgg::StateVec& pose) {
    return (!require_level || mgg::groundSlope(*ground_, pose.head<3>(),
        std::max(robot_params_.size.x(), robot_params_.size.y())) <= mgg::kLevelGroundSlopeRad) &&
        mgg::roomToTurn(*map_, robot_params_, planning_params_, pose, nullptr);
  };

  double remaining = reach_distance_ + mgg::kViewpointArrivalSlack;
  for (std::size_t i = path.size() - 1; i > 0 && remaining > 1e-9; --i) {
    const auto delta = (path[i - 1] - path[i]).eval();
    const double length = delta.head<3>().norm();
    if (length < 1e-9) continue;
    const double checked = std::min(length, remaining);
    const int steps = std::max(1, static_cast<int>(std::ceil(checked / kArrivalBandStepM)));
    for (int sample = 0; sample <= steps; ++sample) {
      mgg::StateVec pose = path[i] + delta * (checked * sample / steps / length);
      // Retreat paths carry lattice/build-time yaws; infer chassis yaw
      // opposite reverse travel. Straight departures already carry their
      // actual chassis yaw (and can be forward), so retain it there.
      if (require_level) pose[3] = std::atan2(delta.y(), delta.x());
      if (!room(pose)) return false;
    }
    remaining -= checked;
  }
  return remaining <= 1e-9;  // no extrapolating beyond the known corridor
}

bool PlannerNode::reverseExitEdgeWithoutPeers(const mgg::StateVec& from,
                                              const mgg::StateVec& to) const {
  // Peer bodies are transient discs in the map as well as the sweep check:
  // pin an empty set over the request's, and use fresh ground lookups.
  std::optional<mgg::MolaMap::TransientDiscPin> no_peers;
  if (mola_map_ != nullptr) no_peers.emplace(*mola_map_, std::vector<Eigen::Vector2d>{}, 0.0);
  mgg::GroundProjection ground(*map_, planning_params_, true);
  return mgg::reverseExitEdgeAdmissible(*map_, ground, robot_params_, planning_params_,
      from, to, [this](const auto& a, const auto& b) { return no_go_.pathAdmissible({a, b}); });
}

bool PlannerNode::reverseExitEdge(const mgg::GroundProjection& ground,
                                  const mgg::StateVec& from,
                                  const mgg::StateVec& to) const {
  const std::uint64_t generation = mola_map_ ? mola_map_->activeGeneration() : map_revision_;
  if (generation != plan_reverse_generation_) {
    plan_reverse_edges_.clear();
    plan_reverse_generation_ = generation;
  }
  const std::array<double, 6> key{from.x(), from.y(), from.z(), to.x(), to.y(), to.z()};
  if (standing_start_scope_depth_ > 0) {
    const auto found = plan_reverse_edges_.find(key);
    if (found != plan_reverse_edges_.end()) return found->second;
  }
  const bool allowed = mgg::reverseExitEdgeAdmissible(*map_, ground, robot_params_, planning_params_,
      from, to, [this](const auto& a, const auto& b) {
        return no_go_.pathAdmissible({a, b}) &&
               !peerBlocksSegment(a - robot_params_.center_offset,
                                  b - robot_params_.center_offset);
      });
  if (standing_start_scope_depth_ > 0) plan_reverse_edges_[key] = allowed;
  return allowed;
}

bool PlannerNode::reverseExitEndpointGivenUp(const mgg::StateVec& pose) const {
  return std::any_of(reverse_exit_exclusions_.begin(), reverse_exit_exclusions_.end(),
      [&](const ReverseExitExclusion& exclusion) {
        return exclusion.given_up &&
               (pose.head<3>() - exclusion.position).norm() <=
                   reach_distance_ + mgg::kViewpointArrivalSlack;
      });
}

bool PlannerNode::reverseExitEndpointExcluded(const mgg::StateVec& pose) const {
  return std::any_of(reverse_exit_exclusions_.begin(), reverse_exit_exclusions_.end(),
      [&](const ReverseExitExclusion& exclusion) {
        return reverseExitExclusionActive(exclusion) &&
               (pose.head<3>() - exclusion.position).norm() <=
                   reach_distance_ + mgg::kViewpointArrivalSlack;
      });
}

std::string PlannerNode::excludeReverseExitEndpoints(const std::vector<Eigen::Vector3d>& ends) {
  expireReverseExitExclusions();
  if (reverse_exit_exclusions_.empty()) {
    reverse_exit_exclusions_anchor_ = current_state_.head<2>();
    reverse_exit_exclusions_map_ = have_mapping_snapshot_
        ? std::make_pair(mapping_snapshot_.component_id, mapping_snapshot_.epoch)
        : std::make_pair(std::string(), std::uint64_t{0});
  }
  const double tolerance = reach_distance_ + mgg::kViewpointArrivalSlack;
  std::vector<std::size_t> refused_now;  // one count per refusal, per exclusion
  std::string note;
  for (const Eigen::Vector3d& end : ends) {
    const auto found = std::find_if(reverse_exit_exclusions_.begin(), reverse_exit_exclusions_.end(),
        [&](const ReverseExitExclusion& exclusion) {
          return (exclusion.position - end).norm() <= tolerance;
        });
    if (found == reverse_exit_exclusions_.end()) {
      reverse_exit_exclusions_.push_back(
          {end, plan_requests_ + kReverseExitExclusionPlans, 1, false});
      refused_now.push_back(reverse_exit_exclusions_.size() - 1);
      continue;
    }
    const std::size_t index = static_cast<std::size_t>(found - reverse_exit_exclusions_.begin());
    if (std::find(refused_now.begin(), refused_now.end(), index) != refused_now.end() ||
        found->given_up) {
      continue;
    }
    refused_now.push_back(index);
    found->excluded_through_request = plan_requests_ + kReverseExitExclusionPlans;
    if (++found->refusals >= kReverseExitExclusionGiveUpRefusals) {
      found->given_up = true;
      char given_up[192];
      std::snprintf(given_up, sizeof(given_up),
                    "; retention exclusion given up: unreachable from here "
                    "((%.2f, %.2f, %.2f), %d refusals)",
                    found->position.x(), found->position.y(), found->position.z(),
                    found->refusals);
      RCLCPP_WARN(get_logger(), "%s", given_up + 2);
      note += given_up;
    }
  }
  return note;
}

void PlannerNode::expireReverseExitExclusions() {
  if (reverse_exit_exclusions_.empty()) return;
  // Not on every map revision: those arrive with each scan, and forgetting
  // at that rate brought back run 14's alternation between two refused
  // candidates. A new component or epoch re-anchors the coordinates.
  const auto map = have_mapping_snapshot_
      ? std::make_pair(mapping_snapshot_.component_id, mapping_snapshot_.epoch)
      : std::make_pair(std::string(), std::uint64_t{0});
  if ((current_state_.head<2>() - reverse_exit_exclusions_anchor_).norm() >
          kReverseExitExclusionMoveM ||
      map != reverse_exit_exclusions_map_) {
    reverse_exit_exclusions_.clear();
  }
}

std::string PlannerNode::rememberReverseExit() {
  if (best_path_.empty() || stored_reverse_sent_now_) return {};
  const std::string refusal = retainReverseExit(best_path_, departure_sent_now_);
  if (refusal.empty()) return {};
  std::vector<Eigen::Vector3d> refused_ends{best_path_.back().head<3>()};
  if (best_path_from_global_graph_) {
    if (const auto* target = findGlobalVertex(current_global_vertex_id_)) {
      refused_ends.push_back(target->state.head<3>());
      // A refused global route vetoes completion while its exclusion lives;
      // a refused local path is no global routing failure.
      global_target_refused_by_retention_ = target->state.head<3>();
    }
  }
  const std::string given_up = excludeReverseExitEndpoints(refused_ends);
  if (tour_planner_) {
    for (const auto& cluster : tour_clusters_) {
      const bool serves_target = best_path_from_global_graph_
          ? cluster.representative_vertex_id == current_global_vertex_id_
          : localPathServesTour(best_path_.back().head<3>(), cluster.position);
      if (cluster.id == tour_planner_->target() && serves_target) {
        setTourClusterAside(cluster, tour_params_.route_retry_s);
        break;
      }
    }
  }
  best_path_.clear();
  best_path_from_global_graph_ = false;
  global_exploration_ongoing_ = false;
  std::string note = "; reverse exit retention refused: " + refusal +
                     "; endpoint excluded for " + std::to_string(kReverseExitExclusionPlans) +
                     " plans (" + std::to_string(reverse_exit_exclusions_.size()) +
                     " excluded)";
  RCLCPP_WARN(get_logger(), "%s", note.c_str() + 2);
  note += given_up;
  // Run 14: returning no path here left a robot that cannot turn choosing
  // the same refused candidates forever. Fall back in this request, as the
  // boxed-in branch does (M-2): the stored exit, then a validated departure.
  mgg::StateVec root_state = current_state_;
  if (!projectToDrivingHeight(root_state)) {
    root_state = physicalAnchorAtDrivingHeight(current_state_);
  }
  std::string fallback;
  if (tryStoredReverseExit(root_state, fallback)) {
    if (best_path_.empty()) fallback += departBoxedIn(root_state, "stored exit refused", false);
  } else {
    // A robot with room to turn is not boxed in: the exclusion moves it on.
    const std::optional<mgg::StandingStart> standing = standingStart();
    fallback = mgg::roomToTurn(*map_, robot_params_, planning_params_, root_state,
                               standing ? &*standing : nullptr)
        ? std::string()
        : departBoxedIn(root_state, "its path's reverse exit retention was refused", false);
  }
  note += fallback;
  if (!best_path_.empty()) note += clearInadmissibleBestPath();
  if (best_path_.empty() && note.find("no path") == std::string::npos) note += "; no path";
  return note;
}

std::string PlannerNode::retainReverseExit(const std::vector<mgg::StateVec>& path,
                                            bool departure) {
  if (path.empty()) return {};
  const bool extending = currentPoseNeedsStoredExit();
  if (!extending) {
    stored_reverse_retreating_ = false;
    stored_reverse_exit_.clear();
  }
  // Short departures independently validate an opening within kDepartureMaxM.
  // Keep any old corridor until odometry leaves it; a lost old refuge must not
  // veto that independently validated escape (M-2).
  if (departure || path.size() < 2 ||
      robot_params_.type != mgg::RobotType::kGroundRobot ||
      !planning_params_.departure_reverse_allowed) return {};
  const auto& end = path.back();
  if (!extending && !endpointNeedsReverseExit(end)) return {};
  std::vector<mgg::StateVec> backtrack(path.rbegin(), path.rend());
  if (extending) {
    std::size_t nearest = 0;
    for (std::size_t i = 1; i < stored_reverse_exit_.size(); ++i) {
      if ((backtrack.back().head<3>() - stored_reverse_exit_[i].head<3>()).squaredNorm() <
          (backtrack.back().head<3>() - stored_reverse_exit_[nearest].head<3>()).squaredNorm()) nearest = i;
    }
    backtrack.insert(backtrack.end(), stored_reverse_exit_.begin() + nearest + 1,
                     stored_reverse_exit_.end());
    auto entry_refuge = std::min_element(reverse_exit_entry_path_.begin(), reverse_exit_entry_path_.end(),
        [&](const auto& a, const auto& b) {
          return (a.template head<3>() - backtrack.back().head<3>()).squaredNorm() <
                 (b.template head<3>() - backtrack.back().head<3>()).squaredNorm();
        });
    while (entry_refuge != reverse_exit_entry_path_.begin()) backtrack.push_back(*--entry_refuge);
  }
  mgg::GroundProjection ground(*map_, planning_params_, true);
  std::vector<mgg::StateVec> reverse{end};
  double length = 0.0;
  const double limit = planning_params_.reverse_exit_max_length > 0.0
      ? planning_params_.reverse_exit_max_length : mgg::kDepartureMaxM;
  for (std::size_t i = 1; i < backtrack.size(); ++i) {
    const auto& to = backtrack[i];
    const auto& from = backtrack[i - 1];
    length += (from.head<3>() - to.head<3>()).norm();
    if (length > limit + 1e-9 || !reverseExitEdge(ground, from, to)) break;
    reverse.back()[3] = std::atan2(from.y() - to.y(), from.x() - to.x());
    reverse.push_back(to);
    reverse.back()[3] = reverse[reverse.size() - 2][3];
    // While extending, turn room ahead of the current root cannot rescue a
    // cancellation before that opening. Preserve the refuge behind the root.
    if ((!extending || i >= path.size() - 1) && refugeArrivalBandAdmissible(reverse)) {
      if (extending) retainEntryPastRefuge(reverse);
      else reverse_exit_entry_path_ = path;
      stored_reverse_exit_ = std::move(reverse);
      stored_reverse_retreating_ = false;
      return {};
    }
  }
  return "no validated refuge within the length bound on current map";
}

void PlannerNode::forgetReverseExitIfOffRoute() {
  if (stored_reverse_exit_.size() < 2) return;
  const double tolerance = reach_distance_ + mgg::kViewpointArrivalSlack;
  bool at_refuge = false;
  if (stored_reverse_retreating_ &&
      (current_state_.head<2>() - stored_reverse_exit_.back().head<2>()).norm() <= tolerance) {
    mgg::StateVec start = current_state_;
    // Controller goal tolerance may stop the robot on the corridor side of
    // the refuge. Proximity alone does not make a turn safe at that pose.
    at_refuge = projectToDrivingHeight(start) &&
        mgg::roomToTurn(*map_, robot_params_, planning_params_, start, nullptr);
  }
  if (at_refuge || !nearPathXY(reverse_exit_entry_path_, current_state_.head<2>(), tolerance)) {
    stored_reverse_exit_.clear();
    reverse_exit_entry_path_.clear();
    stored_reverse_retreating_ = false;
  }
}

bool PlannerNode::storedReverseExitApplies(const mgg::StateVec& pose) const {
  return nearPathXY(stored_reverse_exit_, pose.head<2>(),
                     reach_distance_ + mgg::kViewpointArrivalSlack);
}

bool PlannerNode::currentPoseNeedsStoredExit() const {
  mgg::StateVec start = current_state_;
  if (!projectToDrivingHeight(start)) start = physicalAnchorAtDrivingHeight(current_state_);
  return storedReverseExitApplies(start) &&
      !mgg::roomToTurn(*map_, robot_params_, planning_params_, start, nullptr);
}

void PlannerNode::retainEntryPastRefuge(const std::vector<mgg::StateVec>& path) {
  // Keep the entry prefix beyond the refuge for a later band extension, but
  // discard the outward portion already passed on this retreat.
  const auto refuge = std::min_element(reverse_exit_entry_path_.begin(), reverse_exit_entry_path_.end(),
      [&](const auto& a, const auto& b) {
        return (a.template head<3>() - path.back().head<3>()).squaredNorm() <
               (b.template head<3>() - path.back().head<3>()).squaredNorm();
      });
  reverse_exit_entry_path_.erase(refuge, reverse_exit_entry_path_.end());
  reverse_exit_entry_path_.insert(reverse_exit_entry_path_.end(), path.rbegin(), path.rend());
}

void PlannerNode::keepReverseDeparture(const std::vector<mgg::StateVec>& path) {
  stored_reverse_exit_ = path;
  retainEntryPastRefuge(path);
  stored_reverse_retreating_ = true;
}

bool PlannerNode::tryStoredReverseExit(const mgg::StateVec& start, std::string& note) {
  std::vector<mgg::StateVec> path;
  std::string unusable_here;
  if (!validateStoredReverseExit(start, path, note, &unusable_here)) return false;
  if (!unusable_here.empty()) {
    // Run 14: a corridor that cannot be driven from here is no escape, and
    // keeping it only vetoes every new path (extension mode). New paths
    // from this pose are judged as fresh ones, under the same 6 m bound.
    stored_reverse_exit_.clear();
    reverse_exit_entry_path_.clear();
    stored_reverse_retreating_ = false;
    // Every current exclusion was judged in extension mode, against this
    // corridor. The next retention judges fresh; refusals re-accumulate.
    std::string dropped = "; stored reverse exit dropped: " + unusable_here;
    if (!reverse_exit_exclusions_.empty()) {
      dropped += "; " + std::to_string(reverse_exit_exclusions_.size()) +
                 " retention exclusion(s) judged against it cleared";
      reverse_exit_exclusions_.clear();
    }
    RCLCPP_WARN(get_logger(), "%s", dropped.c_str() + 2);
    note += dropped;
  }
  best_path_ = std::move(path);
  best_path_from_global_graph_ = false;
  global_exploration_ongoing_ = false;
  if (!best_path_.empty()) {
    keepReverseDeparture(best_path_);
    stored_reverse_sent_now_ = true;
    departure_sent_now_ = true;
    ++boxed_in_departures_;
  }
  return true;
}

bool PlannerNode::validateStoredReverseExit(const mgg::StateVec& start,
    std::vector<mgg::StateVec>& path, std::string& note, std::string* unusable_here) {
  path.clear();
  if (!storedReverseExitApplies(start)) return false;
  if (mgg::roomToTurn(*map_, robot_params_, planning_params_, start, nullptr)) {
    note = "; stored reverse exit not needed: projected current pose has observed turn room";
    return false;
  }
  // A static refusal: nothing but the map, the zones or the pose changing
  // makes the corridor drivable from here.
  const auto refused = [&](const char* reason, bool static_here = false) {
    note = std::string("; stored reverse exit refused: ") + reason;
    RCLCPP_WARN(get_logger(), "%s", note.c_str() + 2);
    if (static_here && unusable_here != nullptr) *unusable_here = reason;
    return true;  // tried; exploration may still try a validated departure
  };
  if (!planning_params_.departure_reverse_allowed) return refused("reverse disabled");
  // Connect the actual (possibly early) arrival to the entry corridor. Drop
  // passed poses, and give a lateral arrival at least the existing minimum
  // departure length to blend back onto it rather than turn in place.
  std::size_t nearest = 0;
  for (std::size_t i = 1; i < stored_reverse_exit_.size(); ++i) {
    if ((start.head<3>() - stored_reverse_exit_[i].head<3>()).squaredNorm() <
        (start.head<3>() - stored_reverse_exit_[nearest].head<3>()).squaredNorm()) nearest = i;
  }
  std::size_t join = std::min(nearest + 1, stored_reverse_exit_.size() - 1);
  while (join + 1 < stored_reverse_exit_.size() &&
         (start.head<2>() - stored_reverse_exit_[join].head<2>()).norm() < mgg::kDepartureMinM) ++join;
  std::vector<mgg::StateVec> reverse{start};
  reverse.insert(reverse.end(), stored_reverse_exit_.begin() + join, stored_reverse_exit_.end());
  if (!refugeArrivalBandAdmissible(reverse)) {
    auto refuge = std::min_element(reverse_exit_entry_path_.begin(), reverse_exit_entry_path_.end(),
        [&](const auto& a, const auto& b) {
          return (a.template head<3>() - reverse.back().head<3>()).squaredNorm() <
                 (b.template head<3>() - reverse.back().head<3>()).squaredNorm();
        });
    while (refuge != reverse_exit_entry_path_.begin() && !refugeArrivalBandAdmissible(reverse)) {
      reverse.push_back(*--refuge);
    }
  }
  const Eigen::Vector2d centre = start.head<2>() +
      robot_params_.physicalOffsetForHeading(start[3]).head<2>();
  const Eigen::Vector2d delta = reverse[1].head<2>() - centre;
  const auto physical_offset = robot_params_.physicalOffsetForHeading(0).head<2>().eval();
  if (delta.norm() <= physical_offset.norm() + .01)
    return refused("entry connector is too short for chassis spin", true);
  const double heading = std::atan2(delta.y(), delta.x()) + M_PI -
      std::asin(physical_offset.y()/delta.norm());
  const double turn = std::remainder(heading - start[3], 2.0 * M_PI);
  if (std::abs(turn) > mgg::kDepartureMaxTurnRad + 1e-9) {
    return refused("entry heading mismatch", true);
  }
  // The same angular envelope/steps as findDeparture, without excusing the
  // standing body. No separate rotate-in-place pose is emitted.
  mgg::OrientedBox body;
  body.size = robot_params_.getPlanningSize();
  const int turns = std::max(1, static_cast<int>(std::ceil(std::abs(turn) / mgg::kDepartureTurnStepRad)));
  for (int i = 0; i <= turns; ++i) {
    body.heading = start[3] + turn * i / turns;
    const auto reference = mgg::referenceAfterChassisSpin(robot_params_, start, body.heading);
    const Eigen::Vector3d box_center = reference.head<3>() +
        robot_params_.offsetForHeading(body.heading);
    if (mgg::orientedBoxPathStatus(*map_, box_center,
        box_center, body, false, nullptr) != mgg::VoxelStatus::kFree) {
      return refused("entry heading has no swept body clearance", true);
    }
  }
  reverse.front() = mgg::referenceAfterChassisSpin(robot_params_, start, heading);
  mgg::GroundProjection ground(*map_, planning_params_, true);
  double length = 0.0;
  const double limit = planning_params_.reverse_exit_max_length > 0.0
      ? planning_params_.reverse_exit_max_length : mgg::kDepartureMaxM;
  for (std::size_t i = 1; i < reverse.size(); ++i) {
    length += (reverse[i].head<3>() - reverse[i - 1].head<3>()).norm();
    if (length > limit + 1e-9) return refused("length bound exceeded");
    if (!reverseExitEdge(ground, reverse[i - 1], reverse[i])) {
      // One more sweep, without peers, tells a peer standing in the
      // corridor (transient: keep it) from a static failure.
      if (reverseExitEdgeWithoutPeers(reverse[i - 1], reverse[i])) {
        return refused("current reverse edge is blocked by a peer");
      }
      return refused("current reverse edge fails terrain, clearance or no-go validation", true);
    }
  }
  if (!refugeArrivalBandAdmissible(reverse)) return refused("refuge arrival band lacks observed level turn room");
  // Edge sweeps alone do not certify the chassis transition at a corner,
  // especially the new actual-arrival connector. Keep the same sharp-turn
  // veto, with travel heading opposite the entry body heading.
  mgg::PathTurnCheck reverse_turns(*local_graph_, robot_params_,
      [this](const auto& pose) {
        return mgg::roomToTurn(*map_, robot_params_, planning_params_, pose);
      }, [this, &ground](const Eigen::Vector3d& at) {
        return mgg::groundSlope(ground, at,
            std::max(robot_params_.size.x(), robot_params_.size.y()));
      }, true, [this](const auto& a, const auto& b) {
        return mgg::turnTransitionClear(*map_, robot_params_, a, b);
      });
  mgg::PathType points;
  for (const auto& pose : reverse) points.push_back(pose.head<3>());
  if (!reverse_turns.admissible(points, heading + M_PI)) {
    return refused("reverse corner fails current slope or turn-room checks");
  }
  path = std::move(reverse);
  note = "; stored reverse exit revalidated: reversing to the refuge";
  RCLCPP_INFO(get_logger(), "%s (%.2f m)", note.c_str() + 2, length);
  return true;
}

void PlannerNode::recordSentPath() {
  // The hysteresis follows the path actually sent: the lattice path when
  // it is what goes out, else nothing (review r0, M-1).
  const bool lattice_sent =
      !best_path_.empty() && !best_path_from_global_graph_ &&
      best_path_.size() == lattice_path_.size() &&
      std::equal(best_path_.begin(), best_path_.end(), lattice_path_.begin(),
                 [](const mgg::StateVec& a, const mgg::StateVec& b) {
                   return a == b;
                 });
  if (!lattice_sent) {
    turn_back_hysteresis_.reset();
  } else {
    std::vector<Eigen::Vector3d> points;
    for (const mgg::StateVec& state : best_path_) {
      points.push_back(state.head<3>());
    }
    turn_back_hysteresis_.record(points, lattice_selection_direction_,
                                 planning_params_);
    if (turn_back_hysteresis_.lastTurnedBack()) ++paths_turning_back_;
  }
  lattice_path_.clear();
}

void PlannerNode::publishPlannerConfigState() {
  mgg_msgs::msg::PlannerConfigState state;
  state.incarnation = planner_config_state_.incarnation;
  state.generation = planner_config_state_.generation;
  state.stamp = planner_config_state_.stamp;
  state.last_plan_generation = planner_config_state_.last_plan_generation;
  state.region_active = exploration_region_.has_value();
  if (exploration_region_) {
    const auto& region = *exploration_region_;
    state.region_low.x = region.min_val.x();
    state.region_low.y = region.min_val.y();
    state.region_low.z = region.min_val.z();
    state.region_high.x = region.max_val.x();
    state.region_high.y = region.max_val.y();
    state.region_high.z = region.max_val.z();
  }
  state.target_active = exploration_target_.has_value();
  if (exploration_target_) {
    state.target.x = exploration_target_->x();
    state.target.y = exploration_target_->y();
    state.target.z = exploration_target_->z();
  }
  state.fleet_enabled = static_cast<bool>(fleet_);
  state.leaving_fleet = fleet_ && fleet_->leaving();
  // Metadata is copied unchanged above, so only applied fields can differ.
  // Generation zero is the unpublished initial state, even with fleet off.
  if (state.generation == 0 || state != planner_config_state_) {
    ++state.generation;
    state.stamp = now();
  }
  planner_config_state_ = state;
  planner_config_state_pub_->publish(state);
}

void PlannerNode::onExplorationTargetRequest(
    const std::shared_ptr<mgg_msgs::srv::PlannerSetExplorationTarget::Request>
        request,
    std::shared_ptr<mgg_msgs::srv::PlannerSetExplorationTarget::Response>
        response) {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  applyPendingCancel();
  // The direction paths are scored against changes with the target.
  turn_back_hysteresis_.reset();
  if (!request->active) {
    if (exploration_target_.has_value()) {
      RCLCPP_INFO(get_logger(), "exploration target cleared");
    }
    exploration_target_.reset();
    publishPlannerConfigState();
    response->success = true;
    return;
  }
  const Eigen::Vector3d target(request->target.x, request->target.y,
                               request->target.z);
  if (!target.allFinite()) {
    response->success = false;
    response->message = "exploration target is not finite";
    return;
  }
  exploration_target_ = target;
  RCLCPP_INFO(get_logger(), "exploring toward (%.2f, %.2f, %.2f)", target.x(),
              target.y(), target.z());
  publishPlannerConfigState();
  response->success = true;
}

void PlannerNode::onLeaveFleet(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (!fleet_) {
    response->success = false;
    response->message = "fleet assignment is off";
    return;
  }
  if (request->data) {
    fleet_->leave(now().seconds());
    response->message = "left the fleet; claims released";
  } else {
    fleet_->rejoin();
    response->message = "rejoined the fleet";
  }
  ++tour_assignment_version_;
  publishPlannerConfigState();
  response->success = true;
}

void PlannerNode::onExplorationRegionRequest(
    const std::shared_ptr<mgg_msgs::srv::PlannerSetExplorationRegion::Request>
        request,
    std::shared_ptr<mgg_msgs::srv::PlannerSetExplorationRegion::Response>
        response) {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (!request->active) {
    if (exploration_region_) {
      const auto& min = exploration_region_->min_val;
      const auto& max = exploration_region_->max_val;
      RCLCPP_INFO(get_logger(),
                  "exploration region cleared: min (%.2f, %.2f, %.2f), "
                  "max (%.2f, %.2f, %.2f)",
                  min.x(), min.y(), min.z(), max.x(), max.y(), max.z());
    } else {
      RCLCPP_INFO(get_logger(), "exploration region cleared (none was active)");
    }
    exploration_region_.reset();
    ++tour_assignment_version_;
    publishPlannerConfigState();
    response->success = true;
    response->message = "exploration region cleared";
    return;
  }
  const Eigen::Vector3d min(request->min.x, request->min.y, request->min.z);
  const Eigen::Vector3d max(request->max.x, request->max.y, request->max.z);
  if (!min.allFinite() || !max.allFinite() ||
      !(min.array() < max.array()).all()) {
    response->success = false;
    response->message = "exploration region must be finite with min < max";
    return;
  }
  mgg::BoundedSpaceParams region;
  region.type = mgg::BoundedSpaceType::kCuboid;
  region.setBound(min, max);
  region.setCenter(Eigen::Vector3d(0, 0, 0), /*use_extension=*/false);
  exploration_region_ = region;
  RCLCPP_INFO(get_logger(),
              "exploration region set: min (%.2f, %.2f, %.2f), "
              "max (%.2f, %.2f, %.2f)",
              min.x(), min.y(), min.z(), max.x(), max.y(), max.z());
  // The tour's candidates change: solve again.
  ++tour_assignment_version_;
  publishPlannerConfigState();
  response->success = true;
  response->message = "exploring inside the region only";
}

void PlannerNode::onFlightReach(const std_msgs::msg::Float64::SharedPtr msg) {
  if (std::isnan(msg->data) || msg->data < 0.0) return;
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (msg->data != flight_reach_m_) ++tour_assignment_version_;
  flight_reach_m_ = msg->data;
}

void PlannerNode::onFlightState(const std_msgs::msg::String::SharedPtr msg) {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (home_state_wait_started_) {
    auto map_read = mapReadLease();
    // A late state must not lift home if the deadline passed before the
    // timer callback could acquire the planner mutex.
    seedGlobalGraph();
    latest_flight_state_ = msg->data;
    seedGlobalGraph();
  } else {
    latest_flight_state_ = msg->data;
  }
  rerootHomeOnLateLandedState();
}

void PlannerNode::rerootHomeOnLateLandedState() {
  if (!home_seeded_on_timeout_ || !latest_flight_state_) return;
  // Decided once, on the first state after the timeout seed: a later
  // "landed" may be anywhere the drone flew to.
  home_seeded_on_timeout_ = false;
  if (*latest_flight_state_ != std::string(kFlightStateLanded)) return;
  mgg::Vertex* home = findGlobalVertex(kHomeVertexId);
  if (home == nullptr) return;
  // drone-r3: SwarmDeck's adapter connected 50 s after the planner's wait
  // timed out. Home stayed on the pad, its box in the floor, joined to no
  // edge: every return cost was infinite and the reach cap refused every
  // cluster, while Return Home to the take-off height routed.
  const double moved =
      (current_state_.head<3>() - home_seed_pose_.head<3>()).norm();
  if (left_standing_start_ || moved >= kStandingStartMoveM) {
    RCLCPP_WARN(get_logger(),
                "late flight_state 'landed' after the home was seeded "
                "unlifted, but the drone moved %.2f m since: home stays at "
                "(%.2f, %.2f, %.2f)",
                moved, home->state.x(), home->state.y(), home->state.z());
    return;
  }
  const Eigen::Vector3d was = home->state.head<3>();
  mgg::StateVec lifted = current_state_;
  lifted[2] += aerial_home_height_m_;
  // Its edges were checked where it was.
  const auto neighbours = global_graph_->edge_map_[kHomeVertexId];
  for (const auto& [other, cost] : neighbours) {
    (void)cost;
    if (auto* u = findGlobalVertex(other)) global_graph_->removeEdge(home, u);
    auto& edges = global_graph_->edge_map_[other];
    edges.erase(std::remove_if(edges.begin(), edges.end(),
                               [](const auto& edge) {
                                 return edge.first == kHomeVertexId;
                               }),
                edges.end());
  }
  global_graph_->edge_map_[kHomeVertexId].clear();
  for (auto& [sender, placement] : global_graph_->neighbour_placements_) {
    (void)sender;
    for (auto it = placement.merge_owned_edges.begin();
         it != placement.merge_owned_edges.end();) {
      it = it->first == kHomeVertexId || it->second == kHomeVertexId
               ? placement.merge_owned_edges.erase(it)
               : std::next(it);
    }
  }
  global_graph_->updateVertexState(kHomeVertexId, lifted);
  home->is_hanging = false;
  home_seeded_landed_ = true;
  // Rebuilds from the same keyframes now lift the first one.
  for (std::string& inputs : last_roadmap_rebuild_inputs_) inputs.clear();
  ++graph_revision_;
  ++home_reroots_;
  RCLCPP_INFO(get_logger(),
              "late flight_state 'landed': home re-rooted at take-off height "
              "(%.2f, %.2f, %.2f), was (%.2f, %.2f, %.2f), %zu edge(s) cut",
              lifted.x(), lifted.y(), lifted.z(), was.x(), was.y(), was.z(),
              neighbours.size());
}

std::vector<mgg::FrontierCluster> PlannerNode::insideExplorationRegion(
    std::vector<mgg::FrontierCluster> clusters) const {
  if (!exploration_region_) return clusters;
  clusters.erase(std::remove_if(clusters.begin(), clusters.end(),
                                [this](const mgg::FrontierCluster& cluster) {
                                  return !exploration_region_->isInsideSpace(
                                      cluster.position);
                                }),
                 clusters.end());
  return clusters;
}

void PlannerNode::onObjectiveRequestImpl(
    const std::shared_ptr<mgg_msgs::srv::PlanObjective::Request> request,
    std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> response) {
  using Service = mgg_msgs::srv::PlanObjective;
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  applyLatestOdometry();
  refreshMapRevision();
  StandingStartScope standing_scope(*this);
  const DeadlineScope budget(lattice_deadline_,
      robot_params_.type == mgg::RobotType::kGroundRobot ? lattice_time_budget_s_ : 0.0,
      &peer_diagnosis_in_progress_);
  // Every answer is logged with the objective, where the robot is and the
  // goal it asked for: a refusal alone does not say which objective it
  // answered or where it was going (run 5, 2026-09-25).
  std::string route_note;
  struct LogAnswer {
    std::function<void()> log;
    ~LogAnswer() { log(); }
  } log_answer{[this, &request, &response, &route_note]() {
    const char* objective =
        request->objective == Service::Request::NAVIGATE      ? "NAVIGATE"
        : request->objective == Service::Request::RETURN_HOME ? "RETURN_HOME"
                                                              : "UNKNOWN";
    const auto& g = request->goal.position;
    if (response->status == Service::Response::SUCCEEDED ||
        response->status == Service::Response::DEPARTURE_FIRST) {
      RCLCPP_INFO(get_logger(),
                  "objective %s route (robot at %.2f, %.2f; goal %.2f, %.2f, "
                  "%.2f): %zu poses %s",
                  objective, current_state_.x(), current_state_.y(), g.x, g.y,
                  g.z, response->path.size(), route_note.c_str());
      return;
    }
    const char* status =
        response->status == Service::Response::UNREACHABLE      ? "unreachable"
        : response->status == Service::Response::STALE_REVISION ? "stale map"
        : response->status == Service::Response::BLOCKED        ? "blocked"
                                                          : "unsupported";
    RCLCPP_WARN(get_logger(),
                "objective %s refused (robot at %.2f, %.2f; goal %.2f, %.2f, "
                "%.2f): %s: %s",
                objective, current_state_.x(), current_state_.y(), g.x, g.y,
                g.z, status, response->reason.c_str());
  }};
  try {
  local_route_profile_.clear();
  // One peer set for the whole request (review r0, I5).
  std::optional<PeerBodyPin> peer_pin;
  pinPeerBodies(peer_pin);
  // An objective supersedes exploration's last path.
  turn_back_hysteresis_.reset();
  refreshNoGoZones();
  withdrawUnplacedNeighbours();
  auto map_read = mapReadLease();
  refreshMapRevision();
  // The route is planned on the map the caller names; a request for another
  // map is answered with the one in service so the caller can tell.
  if (mola_map_ != nullptr) {
    response->component_id = mapping_snapshot_.component_id;
    response->map_epoch = mapping_snapshot_.epoch;
    response->mapping_graph_revision = mapping_snapshot_.graph_revision;
    response->geometry_revision = mapping_snapshot_.geometry_revision;
    response->map_source_stamp = mapping_snapshot_.source_stamp;
    if (!have_mapping_snapshot_ ||
        request->component_id != mapping_snapshot_.component_id ||
        request->map_epoch != mapping_snapshot_.epoch) {
      response->status = Service::Response::STALE_REVISION;
      response->reason = "the requested map is not the one in service";
      return;
    }
  }
  if (home_state_wait_started_) {
    response->status = Service::Response::BLOCKED;
    response->reason = "waiting for flight_state";
    return;
  }
  if (!have_odometry_ || !map_->getStatus()) {
    response->status = Service::Response::BLOCKED;
    response->reason = have_odometry_ ? "acquiring observations" : "no odometry";
    setAcquiringObservations(true);
    return;
  }
  if (secondsSince(last_odometry_received_) > odometry_stale_s_) {
    response->status = Service::Response::BLOCKED;
    response->reason = "odometry is stale";
    return;
  }
  mgg::StateVec goal;
  double tolerance = 0.0;
  // Return Home with no finite goal routes to vertex 0, which a rebuild
  // (the robot's pose not linking) moves to the home keyframe.
  bool home_is_vertex_zero = false;
  if (request->objective == Service::Request::RETURN_HOME) {
    // The caller's home is the home keyframe through its current map
    // correction, and it refuses a route that does not end there. Vertex 0
    // is where odometry started, which any correction moves off that home,
    // so it stands in only for a goal that is not finite. Tolerance 0 links
    // the goal itself into the graph.
    goal = fromPoseMsg(request->goal);
    if (!goal.allFinite()) {
      const mgg::Vertex* home = findGlobalVertex(0);
      if (home == nullptr) {
        response->status = Service::Response::UNREACHABLE;
        response->reason = "no home recorded yet";
        return;
      }
      goal = home->state;
      tolerance = 1e-3;
      home_is_vertex_zero = true;
    }
  } else if (request->objective == Service::Request::NAVIGATE) {
    goal = fromPoseMsg(request->goal);
    if (!goal.allFinite()) {
      response->status = Service::Response::UNSUPPORTED_OBJECTIVE;
      response->reason = "goal is not finite";
      return;
    }
  } else {
    response->status = Service::Response::UNSUPPORTED_OBJECTIVE;
    response->reason = "objective must be NAVIGATE or RETURN_HOME";
    return;
  }
  seedGlobalGraph();
  std::vector<mgg::StateVec> route;
  std::string reason;
  // A goal within the local box is a local plan: the grid graph is the
  // paper's local planner and is finer than the roadmap. Anything farther,
  // or a goal the lattice cannot reach, routes over the global graph.
  std::string local_reason;
  mgg::PathOkFn turns_ok;
  const bool local =
      request->objective == Service::Request::NAVIGATE &&
      routeOverLocalLattice(goal, route, turns_ok, local_reason);
  const int rebuilds_before = roadmap_rebuilds_;
  peer_diagnosis_cut_short_ = false;
  bool routed =
      local || routeOverGlobalGraph(goal, tolerance, route, turns_ok, reason);
  if (home_is_vertex_zero && roadmap_rebuilds_ != rebuilds_before) {
    // The goal was the old vertex 0 (the seed where the planner started);
    // the rebuilt graph's vertex 0 is the home keyframe.
    goal = findGlobalVertex(0)->state;
    routed = routeOverGlobalGraph(goal, tolerance, route, turns_ok, reason);
  }
  if (!routed) {
    if (!local_reason.empty()) reason += "; local lattice: " + local_reason;
    if (!local_route_profile_.empty()) {
      reason += " [" + local_route_profile_ + "]";
    }
    // A goal only peer bodies keep the robot from, wherever they stop it
    // (linking the goal, the lattice round it, the roadmap search, the
    // route's check), is BLOCKED, which the caller retries until its
    // deadline, not UNREACHABLE (review r0, I2): the same routing with no
    // peer bodies finds a route.
    // The routing's own diagnosis only says whether a search without peers
    // reaches the goal, not that a route there is admissible (a no-go zone
    // may still hold the goal, review r1, R1): only its being cut short
    // decides here, as retryable. Otherwise the whole routing runs again
    // without peers, bound by the search budget too (review r0, I4).
    if (peersInForce()) {
      if (peer_diagnosis_cut_short_) {
        response->status = Service::Response::BLOCKED;
        response->reason =
            "blocked by a peer, or not: the check was cut short by the "
            "search budget: " +
            reason;
        return;
      }
      std::optional<mgg::MolaMap::TransientDiscPin> no_peers;
      if (mola_map_) no_peers.emplace(*mola_map_, std::vector<Eigen::Vector2d>{}, 0.0);
      const FlagScope diagnosing(peer_diagnosis_in_progress_);
      const FlagScope open(peer_edges_open_);
      const RestoreScope restore_deadline(peer_diagnosis_deadline_);
      peer_diagnosis_cut_short_ = false;
      peer_diagnosis_deadline_ = innerDeadline(
          lattice_deadline_, planning_params_.global_search_time_budget_s);
      std::vector<mgg::StateVec> open_route;
      mgg::PathOkFn open_turns_ok;
      std::string open_reason;
      const bool open_routed =
          (request->objective == Service::Request::NAVIGATE &&
           routeOverLocalLattice(goal, open_route, open_turns_ok,
                                 open_reason)) ||
          (!peer_diagnosis_cut_short_ &&
           routeOverGlobalGraph(goal, tolerance, open_route, open_turns_ok,
                                open_reason));
      if (open_routed) {
        response->status = Service::Response::BLOCKED;
        response->reason = "blocked by a peer: " + reason;
        return;
      }
      if (peer_diagnosis_cut_short_) {
        response->status = Service::Response::BLOCKED;
        response->reason =
            "blocked by a peer, or not: the check was cut short by the "
            "search budget: " +
            reason;
        return;
      }
      reason += "; without peers: " + open_reason;
    }
    response->status = Service::Response::UNREACHABLE;
    response->reason = reason;
    return;
  }
  shortcutAndResample(route, turns_ok, nullptr, /*lattice_route=*/local);
  mgg::PathType objective_points;
  for (const auto& pose : route) objective_points.push_back(pose.head<3>());
  // Check the actual route being sent, even without retained escape memory.
  // No SUCCEEDED objective may start with a turn at a room-less root.
  if (routeStartsWithTurnWithoutRoom(objective_points)) {
    mgg::StateVec start = current_state_;
    if (!projectToDrivingHeight(start)) start = physicalAnchorAtDrivingHeight(current_state_);
    std::string exit_note;
    std::vector<mgg::StateVec> exit_path;
    if (validateStoredReverseExit(start, exit_path, exit_note) && !exit_path.empty()) {
      keepReverseDeparture(exit_path);
      response->status = Service::Response::DEPARTURE_FIRST;
      response->reason = "stored reverse exit revalidated to refuge; request the objective again from there";
      route_note = response->reason;
      for (const auto& pose : exit_path) response->path.push_back(toPoseMsg(pose));
    } else {
      if (exit_note.empty()) exit_note = "; stored reverse exit unavailable at projected current pose";
      mgg::Departure departure;
      if (straightDeparture(start, departure, /*arrival_band=*/true)) {
        response->status = Service::Response::DEPARTURE_FIRST;
        response->reason = std::string("validated departure ") +
            (departure.reverse ? "in reverse" : "ahead") +
            "; request the objective again from its end" + exit_note;
        route_note = response->reason;
        for (const auto& pose : departure.path) response->path.push_back(toPoseMsg(pose));
      } else {
        response->status = Service::Response::BLOCKED;
        response->reason = "objective requires a departure" + exit_note +
            "; validated departure refused: no terrain-clear leg with turn room within " +
            std::to_string(mgg::kDepartureMaxM) + " m";
      }
    }
    return;  // keep the remaining escape; never command an in-place turn
  }
  if (!startPathAfterChassisSpin(route, local)) {
    response->status = Service::Response::UNREACHABLE;
    response->reason = "post-spin reference cannot join the route safely";
    return;
  }
  if (!route.empty() && !standingStartGoalAdmissible(route.back())) {
    response->status = Service::Response::UNREACHABLE;
    response->reason = "the goal intersects the standing-start arrival band";
    return;
  }
  if (!noGoAdmissible(route)) {
    response->status = Service::Response::UNREACHABLE;
    response->reason = "the route enters a no-go zone";
    return;
  }
  if (!peerAdmissible(route)) {
    response->status = Service::Response::BLOCKED;
    response->reason = "blocked by a peer: the route meets a peer body";
    return;
  }
  if (!route.empty() && currentPoseNeedsStoredExit()) {
    const std::string refusal = retainReverseExit(route, false);
    if (!refusal.empty()) {
      response->reason = "escape corridor not retained: " + refusal;
      RCLCPP_WARN(get_logger(), "%s", response->reason.c_str());
    }
  } else {
    stored_reverse_exit_.clear();
  }
  response->status = Service::Response::SUCCEEDED;
  for (const mgg::StateVec& s : route) response->path.push_back(toPoseMsg(s));
  char note[160];
  std::snprintf(note, sizeof(note),
                "to (%.2f, %.2f) over the %s (%d corners)", goal.x(), goal.y(),
                local ? "local lattice" : "global graph",
                path_shortcut_corners_);
  route_note = note;
  if (!local && !reason.empty()) {
    route_note += "; " + reason;
    response->reason = reason;
  }
  if (local && !local_route_profile_.empty()) {
    route_note += " [" + local_route_profile_ + "]";
  }
  mgg::planningCheckpoint();
  } catch (const mgg::PlanningInterrupted&) {
    if (budget.cancelled_) {
      response->path.clear();
      response->status = Service::Response::BLOCKED;
      response->reason = budget.reason();
      throw;
    }
    response->path.clear();
    response->status = (budget.cancelled_ || budget.interrupted_diagnosis_)
        ? mgg_msgs::srv::PlanObjective::Response::BLOCKED
        : mgg_msgs::srv::PlanObjective::Response::UNREACHABLE;
    response->reason = budget.reason();
    RCLCPP_WARN(get_logger(), "%s", response->reason.c_str());
  }
}

// ---------------------------------------------------------------------------
// Outputs

void PlannerNode::publishPath() {
  std::lock_guard<std::mutex> fence(cancellation_mutex_);
  mgg::planningCheckpoint();
  publishPathUnderCancellationFence();
}

void PlannerNode::publishPathUnderCancellationFence() {
  nav_msgs::msg::Path msg;
  msg.header.stamp = now();
  msg.header.frame_id = world_frame_;
  for (const mgg::StateVec& s : best_path_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = toPoseMsg(s);
    msg.poses.push_back(pose);
  }
  path_pub_->publish(msg);
}

mgg_msgs::msg::Graph PlannerNode::ownGraphMessage() {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (global_graph_->getNumVertices() == 0) return {};
  // Exchanged at ground height: a neighbour of another platform drives at
  // its own height above the same floor (mergeNeighbourGraph).
  auto msg = toGraphMsg(*global_graph_,
                        static_cast<int>(planning_params_.robot_id),
                        receiverPlatform().driving_height);
  msg.header.stamp = now();
  msg.header.frame_id = world_frame_;
  return msg;
}

void PlannerNode::publishOwnGraph() {
  // Runs on a timer, so it can land in the middle of a planning cycle
  // rewriting the very graph it is serialising; ownGraphMessage locks.
  const auto msg = ownGraphMessage();
  if (msg.vertices.empty()) return;
  graph_pub_->publish(msg);
}

void PlannerNode::publishMarkers() {
  if (marker_pub_->get_subscription_count() == 0) return;
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);

  visualization_msgs::msg::MarkerArray array;
  const auto edges_of = [this](mgg::GraphManager& graph, const char* ns,
                               float r, float g, float b, float width) {
    visualization_msgs::msg::Marker edges;
    edges.header.frame_id = world_frame_;
    edges.header.stamp = now();
    edges.ns = ns;
    edges.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges.action = visualization_msgs::msg::Marker::ADD;
    edges.scale.x = width;
    edges.color.r = r;
    edges.color.g = g;
    edges.color.b = b;
    edges.color.a = 0.8;
    edges.pose.orientation.w = 1.0;
    std::pair<mgg::Graph::GraphType::edge_iterator,
              mgg::Graph::GraphType::edge_iterator> range;
    graph.graph_->getEdgeIterator(range);
    for (auto it = range.first; it != range.second; ++it) {
      const auto property = graph.graph_->getEdgeProperty(it);
      const mgg::Vertex* u = graph.getVertex(std::get<0>(property));
      const mgg::Vertex* v = graph.getVertex(std::get<1>(property));
      if (u == nullptr || v == nullptr) continue;
      geometry_msgs::msg::Point a, b;
      a.x = u->state[0]; a.y = u->state[1]; a.z = u->state[2];
      b.x = v->state[0]; b.y = v->state[1]; b.z = v->state[2];
      edges.points.push_back(a);
      edges.points.push_back(b);
    }
    return edges;
  };
  const auto points_of = [this](mgg::GraphManager& graph, const char* ns,
                                mgg::VertexType only, bool filter, float r,
                                float g, float b, float size) {
    visualization_msgs::msg::Marker points;
    points.header.frame_id = world_frame_;
    points.header.stamp = now();
    points.ns = ns;
    points.type = visualization_msgs::msg::Marker::POINTS;
    points.action = visualization_msgs::msg::Marker::ADD;
    points.scale.x = size;
    points.scale.y = size;
    points.color.r = r;
    points.color.g = g;
    points.color.b = b;
    points.color.a = 1.0;
    points.pose.orientation.w = 1.0;
    for (const auto& entry : graph.vertices_map_) {
      if (entry.second == nullptr) continue;
      if (filter && entry.second->type != only) continue;
      geometry_msgs::msg::Point p;
      p.x = entry.second->state[0];
      p.y = entry.second->state[1];
      p.z = entry.second->state[2];
      points.points.push_back(p);
    }
    return points;
  };

  // The local lattice: where the planner considered standing and where it
  // believes it can drive, which is the part that shows a graph split by an
  // obstacle or stranded in a corner.
  array.markers.push_back(points_of(*local_graph_, "local_graph",
                                    mgg::VertexType::kUnvisited, false, 0.0f,
                                    1.0f, 0.0f, 0.15f));
  array.markers.push_back(
      edges_of(*local_graph_, "local_graph_edges", 0.0f, 0.6f, 1.0f, 0.03f));
  auto frontiers = points_of(*local_graph_, "frontiers",
                             mgg::VertexType::kFrontier, true, 1.0f, 0.2f,
                             0.2f, 0.25f);
  if (!frontiers.points.empty()) array.markers.push_back(frontiers);

  // The global graph: the roadmap and its frontiers.
  if (global_graph_->getNumVertices() > 0) {
    array.markers.push_back(edges_of(*global_graph_, "global_graph_edges",
                                     1.0f, 0.8f, 0.2f, 0.05f));
    auto global_frontiers = points_of(*global_graph_, "global_frontiers",
                                      mgg::VertexType::kFrontier, true, 1.0f,
                                      0.0f, 0.6f, 0.3f);
    if (!global_frontiers.points.empty()) {
      array.markers.push_back(global_frontiers);
    }
  }

  // Swarm graph merge beacons.
  const rclcpp::Time current_time = now();
  recent_merges_.erase(
      std::remove_if(recent_merges_.begin(), recent_merges_.end(),
                     [&](const MergeEvent& ev) {
                       return (current_time - ev.stamp).seconds() > 3.0;
                     }),
      recent_merges_.end());
  if (!recent_merges_.empty()) {
    visualization_msgs::msg::Marker merge_marker;
    merge_marker.header.frame_id = world_frame_;
    merge_marker.header.stamp = current_time;
    merge_marker.ns = "graph_merges";
    merge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    merge_marker.action = visualization_msgs::msg::Marker::ADD;
    merge_marker.scale.x = 0.08;
    merge_marker.color.g = 1.0;
    merge_marker.color.b = 1.0;
    merge_marker.color.a = 1.0;
    merge_marker.pose.orientation.w = 1.0;
    for (const auto& ev : recent_merges_) {
      const double r = 0.6;
      geometry_msgs::msg::Point a, b, c, d;
      a.x = ev.their_pos.x() - r; a.y = ev.their_pos.y(); a.z = ev.their_pos.z();
      b.x = ev.their_pos.x() + r; b.y = ev.their_pos.y(); b.z = ev.their_pos.z();
      c.x = ev.their_pos.x(); c.y = ev.their_pos.y() - r; c.z = ev.their_pos.z();
      d.x = ev.their_pos.x(); d.y = ev.their_pos.y() + r; d.z = ev.their_pos.z();
      merge_marker.points.push_back(a);
      merge_marker.points.push_back(b);
      merge_marker.points.push_back(c);
      merge_marker.points.push_back(d);
    }
    array.markers.push_back(merge_marker);
  }

  marker_pub_->publish(array);
}

}  // namespace mgg_ros
