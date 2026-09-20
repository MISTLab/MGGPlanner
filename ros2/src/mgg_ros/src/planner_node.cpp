#include "mgg_ros/planner_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <future>
#include <limits>
#include <map>
#include <random>
#include <regex>
#include <stdexcept>
#include <tuple>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.hpp>

#include "mgg_core/log.h"
#include "mgg_core/trajectory.h"
#include "mgg_ros/conversions.h"
#include "mgg_ros/param_loader.h"

namespace mgg_ros {

namespace {

std::string boundedObjectiveFailure(const std::string& reason) {
  constexpr std::size_t kLimit = 220;
  constexpr std::size_t kTail = 100;
  std::string compact = reason;
  const std::size_t grid = compact.find(" [grid evidence:");
  const std::size_t footprint =
      compact.find(" [first footprint rejection:");
  if (grid != std::string::npos && footprint != std::string::npos &&
      grid < footprint) {
    compact = compact.substr(0, grid) + compact.substr(footprint);
  }
  if (compact.size() <= kLimit) return compact;
  return compact.substr(0, kLimit - kTail - 3) + "..." +
         compact.substr(compact.size() - kTail);
}

bool hasSkippableObjectiveWaypointRejection(
    const mgg::RouteCorridor& corridor, const std::string& reason) {
  constexpr char kPrefix[] = "route corridor waypoint[";
  constexpr char kSuffix[] = "] rejected:";
  if (corridor.status != mgg::PlanningStatus::kSucceeded ||
      (corridor.request.objective != mgg::ObjectiveKind::kReturnHome &&
       corridor.request.objective != mgg::ObjectiveKind::kNavigate) ||
      corridor.poses.empty() ||
      reason.compare(0, sizeof(kPrefix) - 1, kPrefix) != 0) {
    return false;
  }
  if (!corridor.request.goal.pose.allFinite() ||
      std::any_of(corridor.poses.begin(), corridor.poses.end(),
                  [](const mgg::StateVec& pose) { return !pose.allFinite(); })) {
    return false;
  }
  std::size_t cursor = sizeof(kPrefix) - 1;
  std::size_t index = 0;
  const std::size_t digits_begin = cursor;
  while (cursor < reason.size() && reason[cursor] >= '0' &&
         reason[cursor] <= '9') {
    const std::size_t digit = static_cast<std::size_t>(reason[cursor] - '0');
    if (index > (std::numeric_limits<std::size_t>::max() - digit) / 10u) {
      return false;
    }
    index = index * 10u + digit;
    ++cursor;
  }
  if (cursor == digits_begin ||
      reason.compare(cursor, sizeof(kSuffix) - 1, kSuffix) != 0 ||
      index >= corridor.poses.size()) {
    return false;
  }
  cursor += sizeof(kSuffix) - 1;
  while (cursor < reason.size() && reason[cursor] == ' ') ++cursor;
  const auto has_projection_class = [&reason, cursor](const char* value) {
    return reason.compare(cursor, std::char_traits<char>::length(value), value) ==
           0;
  };
  if (!has_projection_class("no mapped ground support") &&
      !has_projection_class("body intersects occupied space") &&
      !has_projection_class("body includes unknown space") &&
      !has_projection_class("geofence violation")) {
    return false;
  }
  // A partial corridor stores its exact local proxy as the last pose. It is a
  // mandatory endpoint, while all earlier graph poses are refinement hints.
  return !corridor.partial || index + 1u < corridor.poses.size();
}

mgg::RouteCorridor objectiveLocalEndpointCorridor(
    const mgg::RouteCorridor& corridor) {
  mgg::RouteCorridor endpoint = corridor;
  if (endpoint.partial && !endpoint.poses.empty()) {
    const mgg::StateVec local_proxy = endpoint.poses.back();
    endpoint.poses.clear();
    endpoint.poses.push_back(local_proxy);
  } else {
    // A full corridor's endpoint remains request-owned. Clearing a matching
    // final graph pose makes the grid planner project and validate that exact
    // objective goal again before searching.
    endpoint.poses.clear();
  }
  return endpoint;
}

bool objectiveGridWindowFits(const mgg::RouteCorridor& corridor,
                             const mgg::StateVec& current,
                             double resolution, double margin,
                             std::size_t max_cells) {
  if (!current.allFinite() || !std::isfinite(resolution) || resolution <= 0.0 ||
      !std::isfinite(margin) || margin < 0.0 || max_cells == 0) {
    return false;
  }
  std::vector<mgg::StateVec> waypoints;
  waypoints.reserve(corridor.poses.size() + 2);
  waypoints.push_back(current);
  for (const mgg::StateVec& pose : corridor.poses) {
    if (!pose.allFinite()) return false;
    if ((waypoints.back().head<3>() - pose.head<3>())
            .cwiseAbs().maxCoeff() > 1e-6) {
      waypoints.push_back(pose);
    }
  }
  if (!corridor.partial) {
    const mgg::StateVec& goal = corridor.request.goal.pose;
    if (!goal.allFinite()) return false;
    if ((waypoints.back().head<3>() - goal.head<3>())
            .cwiseAbs().maxCoeff() > 1e-6) {
      waypoints.push_back(goal);
    }
  }
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    const long double width =
        std::ceil((std::abs(static_cast<long double>(waypoints[i].x()) -
                            waypoints[i - 1].x()) +
                   2.0L * margin) /
                  resolution) +
        3.0L;
    const long double height =
        std::ceil((std::abs(static_cast<long double>(waypoints[i].y()) -
                            waypoints[i - 1].y()) +
                   2.0L * margin) /
                  resolution) +
        3.0L;
    if (!std::isfinite(width) || !std::isfinite(height) || width < 1.0L ||
        height < 1.0L ||
        width > static_cast<long double>(max_cells) ||
        height > static_cast<long double>(max_cells) ||
        width * height > static_cast<long double>(max_cells)) {
      return false;
    }
  }
  return true;
}

double widestObjectiveGridMargin(const mgg::RouteCorridor& corridor,
                                 const mgg::StateVec& current,
                                 const mgg::GridRefinementLimits& limits,
                                 double maximum_margin) {
  const double minimum_margin = limits.detour_margin_m;
  if (!std::isfinite(maximum_margin) || maximum_margin <= minimum_margin ||
      !objectiveGridWindowFits(corridor, current, limits.resolution_m,
                               minimum_margin, limits.max_cells)) {
    return minimum_margin;
  }
  const double resolution = limits.resolution_m;
  const long double steps_value = std::floor(
      (static_cast<long double>(maximum_margin) - minimum_margin) /
          resolution +
      1e-9L);
  if (!std::isfinite(steps_value) || steps_value < 0.0L ||
      steps_value >
          static_cast<long double>(std::numeric_limits<std::size_t>::max())) {
    return minimum_margin;
  }
  const std::size_t steps = static_cast<std::size_t>(steps_value);
  std::size_t low = 0;
  std::size_t high = steps;
  while (low < high) {
    const std::size_t middle = low + (high - low + 1) / 2;
    const double candidate = minimum_margin + middle * resolution;
    if (objectiveGridWindowFits(corridor, current, resolution, candidate,
                                limits.max_cells)) {
      low = middle;
    } else {
      high = middle - 1;
    }
  }
  double selected = minimum_margin + low * resolution;
  if (selected < maximum_margin &&
      objectiveGridWindowFits(corridor, current, resolution, maximum_margin,
                              limits.max_cells)) {
    selected = maximum_margin;
  }
  return selected;
}

/// XY arc length of an emitted local section, measured from the pose the route
/// started at. Only XY is used: the graph plane and the navigation base plane
/// differ solely in Z, so this length is the same in both.
double emittedSectionXyLength(const mgg::StateVec& start,
                              const std::vector<mgg::StateVec>& poses) {
  double length = 0.0;
  Eigen::Vector2d previous = start.head<2>();
  for (const mgg::StateVec& pose : poses) {
    length += (pose.head<2>() - previous).norm();
    previous = pose.head<2>();
  }
  return length;
}

/// Replays the bounded objective window walk to report how many global route
/// poses a section of the given XY length actually consumed. The window walk
/// and this replay share one arc-length coordinate, so a section that the
/// indexed authority shortened resumes exactly where it stopped instead of
/// where its horizon had asked to stop.
std::size_t objectiveRouteIndexAfterLength(
    const std::vector<mgg::StateVec>& poses, std::size_t begin,
    const mgg::StateVec& start, double length) {
  std::size_t index = std::min(begin, poses.size());
  double remaining = std::isfinite(length) ? std::max(0.0, length) : 0.0;
  Eigen::Vector2d previous = start.head<2>();
  while (index < poses.size()) {
    const double distance = (poses[index].head<2>() - previous).norm();
    if (!std::isfinite(distance) || distance > remaining + 1e-6) break;
    remaining -= distance;
    previous = poses[index].head<2>();
    ++index;
  }
  return index;
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

  loadParameters();

  const double map_resolution = declareOrGet<double>(this, "map.resolution", 0.2);
  map_backend_ = declareOrGet<std::string>(this, "map.backend", map_backend_);
  indexed_map_query_service_ = declareOrGet<std::string>(
      this, "indexed_map_query_service", indexed_map_query_service_);
  if (map_backend_ == "mola_snapshot" && indexed_map_query_service_.empty()) {
    throw std::invalid_argument(
        "map.backend=mola_snapshot requires indexed_map_query_service for "
        "exact route validation");
  }
  if (map_backend_ == "cloud_octomap") {
    mgg::OctomapConfig map_cfg;
    map_cfg.resolution = map_resolution;
    map_cfg.max_range = declareOrGet<double>(this, "map.max_range", 20.0);
    auto backend = std::make_unique<mgg::OctomapMap>(map_cfg);
    cloud_map_ = backend.get();
    map_ = std::move(backend);
  } else if (map_backend_ == "mola_snapshot") {
    mgg::MolaMapConfig map_cfg;
    map_cfg.resolution = map_resolution;
    map_cfg.peer_root = declareOrGet<std::string>(this, "map.mola.peer_root", "");
    map_cfg.snapshot_ttl_sec = std::clamp(
        declareOrGet<double>(this, "map.mola.snapshot_ttl_sec", 3.0), 0.1, 60.0);
    map_cfg.max_snapshot_bytes = static_cast<std::size_t>(std::clamp(
        declareOrGet<std::int64_t>(this, "map.mola.max_snapshot_bytes", 4194304),
        std::int64_t{1024}, std::int64_t{4 * 1024 * 1024}));
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
    auto backend = std::make_unique<mgg::MolaMap>(map_cfg);
    mola_map_ = backend.get();
    map_ = std::move(backend);
    RCLCPP_INFO(get_logger(),
                "map backend '%s': source '%s', resolution %.3f m, TTL %.1f s",
                map_backend_.c_str(), map_cfg.peer_root.c_str(), map_cfg.resolution,
                map_cfg.snapshot_ttl_sec);
  } else {
    throw std::invalid_argument("map.backend must be cloud_octomap or mola_snapshot");
  }
  const std::string body_evidence_policy = declareOrGet<std::string>(
      this, "objective_body_evidence_policy", "strict_volume");
  if (body_evidence_policy == "observed_ground") {
    if ((map_backend_ != "cloud_octomap" && map_backend_ != "mola_snapshot") ||
        !get_parameter("use_sim_time").as_bool() ||
        robot_params_.type != mgg::RobotType::kGroundRobot) {
      throw std::invalid_argument(
          "objective_body_evidence_policy=observed_ground requires a "
          "simulated ground robot using a qualified map backend");
    }
    observed_ground_body_evidence_ = true;
    if (cloud_map_ != nullptr) cloud_map_->setTrackMeasuredSurfaceZ(true);
  } else if (body_evidence_policy != "strict_volume") {
    throw std::invalid_argument(
        "objective_body_evidence_policy must be strict_volume or "
        "observed_ground");
  }
  const std::string ground_evidence_policy = declareOrGet<std::string>(
      this, "objective_ground_evidence_policy", "observed_ground");
  if (ground_evidence_policy == "provisional_unknown") {
    if (!observed_ground_body_evidence_ ||
        (map_backend_ != "cloud_octomap" && map_backend_ != "mola_snapshot") ||
        !get_parameter("use_sim_time").as_bool() ||
        robot_params_.type != mgg::RobotType::kGroundRobot) {
      throw std::invalid_argument(
          "objective_ground_evidence_policy=provisional_unknown requires "
          "objective_body_evidence_policy=observed_ground on a simulated "
          "ground robot using a qualified map backend");
    }
    provisional_unknown_ground_ = true;
  } else if (ground_evidence_policy != "observed_ground") {
    throw std::invalid_argument(
        "objective_ground_evidence_policy must be observed_ground or "
        "provisional_unknown");
  }
  ground_ = std::make_unique<mgg::GroundProjection>(*map_, planning_params_);
  const double grid_resolution_floor = map_->getResolution();
  grid_refinement_limits_.resolution_m = std::clamp(
      declareOrGet<double>(this, "grid_refinement_resolution_m", 0.25),
      grid_resolution_floor, std::max(2.0, grid_resolution_floor));
  grid_refinement_limits_.detour_margin_m = std::clamp(
      declareOrGet<double>(this, "grid_refinement_margin_m", 1.0), 0.0,
      10.0);
  grid_refinement_limits_.max_cells = static_cast<std::size_t>(std::clamp(
      declareOrGet<std::int64_t>(this, "grid_refinement_max_cells", 4096),
      std::int64_t{16}, std::int64_t{65536}));
  grid_refinement_limits_.max_expansions =
      static_cast<std::size_t>(std::clamp(
          declareOrGet<std::int64_t>(this, "grid_refinement_max_expansions",
                                     2048),
          std::int64_t{1}, std::int64_t{65536}));
  grid_refinement_limits_.timeout = std::chrono::milliseconds(std::clamp(
      declareOrGet<std::int64_t>(this, "grid_refinement_timeout_ms", 50),
      std::int64_t{1}, std::int64_t{5000}));
  objective_grid_limits_ = grid_refinement_limits_;
  objective_grid_limits_.detour_margin_m = std::clamp(
      declareOrGet<double>(this, "objective_grid_margin_m", 4.0), 0.0, 25.0);
  objective_grid_max_margin_m_ = std::clamp(
      declareOrGet<double>(this, "objective_grid_max_margin_m",
                           objective_grid_limits_.detour_margin_m),
      objective_grid_limits_.detour_margin_m, 25.0);
  objective_grid_limits_.max_cells = static_cast<std::size_t>(std::clamp(
      declareOrGet<std::int64_t>(this, "objective_grid_max_cells", 32768),
      std::int64_t{64}, std::int64_t{262144}));
  objective_grid_limits_.max_expansions = static_cast<std::size_t>(std::clamp(
      declareOrGet<std::int64_t>(this, "objective_grid_max_expansions", 16384),
      std::int64_t{1}, std::int64_t{262144}));
  objective_grid_limits_.timeout = std::chrono::milliseconds(std::clamp(
      declareOrGet<std::int64_t>(this, "objective_grid_timeout_ms", 500),
      std::int64_t{1}, std::int64_t{5000}));
  const double requested_partial_progress = declareOrGet<double>(
      this, "partial_route_min_progress_m", partial_route_min_progress_m_);
  partial_route_min_progress_m_ =
      std::isfinite(requested_partial_progress)
          ? std::clamp(requested_partial_progress, 0.10, 5.0)
          : 1.0;
  odometry_height_error_max_m_ = std::clamp(
      declareOrGet<double>(this, "odometry_height_error_max_m",
                           odometry_height_error_max_m_),
      0.0, 1.0);
  hazard_prefix_standoff_m_ = declareOrGet<double>(
      this, "hazard_prefix_standoff_m", hazard_prefix_standoff_m_);
  hazard_prefix_standoff_m_ =
      std::isfinite(hazard_prefix_standoff_m_)
          ? std::clamp(hazard_prefix_standoff_m_, 0.0, 10.0)
          : 0.0;
  // The parameter keeps its original name; see step_measurement_margin_m_ for
  // what it now covers and why the default is 0.02 m.
  const double requested_step_margin = declareOrGet<double>(
      this, "footprint_step_measurement_tolerance_m",
      step_measurement_margin_m_);
  step_measurement_margin_m_ =
      std::isfinite(requested_step_margin)
          ? std::clamp(requested_step_margin, 0.0, 0.05)
          : 0.02;
  const double requested_start_support = declareOrGet<double>(
      this, "objective_start_support_max_distance_m",
      objective_start_support_max_distance_m_);
  objective_start_support_max_distance_m_ =
      std::isfinite(requested_start_support)
          ? std::clamp(requested_start_support, 0.0, 5.0)
          : 3.0;
  objective_grid_limits_.start_connector_max_distance_m =
      objective_start_support_max_distance_m_;
  {
    mgg::BlockedCorridorLimits blocked;
    blocked.max_entries = static_cast<std::size_t>(std::clamp(
        declareOrGet<std::int64_t>(this, "blocked_corridor_max_entries", 64),
        std::int64_t{0}, std::int64_t{4096}));
    const double requested_cell = declareOrGet<double>(
        this, "blocked_corridor_cell_size_m", 0.5);
    blocked.cell_size_m = std::isfinite(requested_cell)
                              ? std::clamp(requested_cell, 0.05, 5.0)
                              : 0.5;
    const double requested_ttl =
        declareOrGet<double>(this, "blocked_corridor_ttl_s", 10.0);
    blocked.ttl_s = std::isfinite(requested_ttl)
                        ? std::clamp(requested_ttl, 0.0, 600.0)
                        : 10.0;
    // A mark also dies once this many new map revisions have arrived: new
    // measurements are the only thing that can change the verdict that
    // produced it.
    blocked.revision_window = static_cast<std::uint64_t>(std::clamp(
        declareOrGet<std::int64_t>(this, "blocked_corridor_revision_window", 8),
        std::int64_t{1}, std::int64_t{100000}));
    blocked_corridors_.setLimits(blocked);
  }
  const double requested_route_horizon = declareOrGet<double>(
      this, "objective_route_horizon_m", objective_route_horizon_m_);
  objective_route_horizon_m_ =
      std::isfinite(requested_route_horizon)
          ? std::clamp(requested_route_horizon, 1.0, 50.0)
          : 8.0;
  const double requested_route_progress_tolerance = declareOrGet<double>(
      this, "objective_route_progress_tolerance_m", 1.0);
  objective_route_progress_tolerance_m_ =
      std::isfinite(requested_route_progress_tolerance)
          ? std::clamp(requested_route_progress_tolerance, 0.1, 3.0)
          : 1.0;
  const double requested_odometry_stale_s =
      declareOrGet<double>(this, "odometry_stale_s", 5.0);
  odometry_stale_s_ = std::isfinite(requested_odometry_stale_s)
                          ? std::clamp(requested_odometry_stale_s, 0.1, 60.0)
                          : 5.0;
  objective_route_max_poses_ = static_cast<std::size_t>(std::clamp(
      declareOrGet<std::int64_t>(this, "objective_route_max_poses", 4096),
      std::int64_t{2}, std::int64_t{65536}));
  geofence_ = std::make_unique<mgg::GeofenceManager>();
  local_graph_ = std::make_shared<mgg::GraphManager>();
  global_graph_ = std::make_shared<mgg::GraphManager>();
  local_graph_->setRobotId(static_cast<int>(planning_params_.robot_id));
  global_graph_->setRobotId(static_cast<int>(planning_params_.robot_id));
  route_instance_id_ = std::to_string(planning_params_.robot_id) + "-" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());

  // Static inter-robot transforms for bring-up. Phase 8 swaps this for a
  // Swarm-SLAM backed PoseSource; the merge takes either.
  poses_ = std::make_unique<mgg::StaticPoseSource>();
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

  if (cloud_map_ != nullptr) {
    cloud_tf_timeout_sec_ =
        declareOrGet<double>(this, "cloud_tf_timeout_sec", cloud_tf_timeout_sec_);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  planning_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onOdometry(m); },
      sub_opts);

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

  neighbour_sub_ = create_subscription<mgg_msgs::msg::Graph>(
      "neighbour_graph_in", rclcpp::QoS(10),
      [this](mgg_msgs::msg::Graph::ConstSharedPtr m) { onNeighbourGraph(m); },
      sub_opts);

  reservation_exclusion_radius_m_ = std::max(
      0.0, declareOrGet<double>(this, "reservation_exclusion_radius_m",
                                reservation_exclusion_radius_m_));
  reservation_exclusion_ttl_s_ = std::max(
      0.0, declareOrGet<double>(this, "reservation_exclusion_ttl_s",
                                reservation_exclusion_ttl_s_));
  coordination_exclusions_sub_ =
      create_subscription<geometry_msgs::msg::PoseArray>(
          "coordination_exclusions", rclcpp::QoS(10),
          [this](geometry_msgs::msg::PoseArray::ConstSharedPtr m) {
            onCoordinationExclusions(m);
          },
          sub_opts);
  peer_body_radius_m_ = std::clamp(
      declareOrGet<double>(this, "peer_body_radius_m", peer_body_radius_m_), 0.0,
      5.0);
  peer_body_ttl_s_ = std::clamp(
      declareOrGet<double>(this, "peer_body_ttl_s", peer_body_ttl_s_), 0.1, 30.0);
  peer_bodies_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "peer_bodies", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr m) {
        onPeerBodies(m);
      },
      sub_opts);

  indexed_map_query_timeout_s_ = std::max(
      0.01, declareOrGet<double>(this, "indexed_map_query_timeout_s",
                                 indexed_map_query_timeout_s_));
  indexed_map_snapshot_ttl_s_ = std::max(
      0.0, declareOrGet<double>(this, "indexed_map_snapshot_ttl_s",
                                indexed_map_snapshot_ttl_s_));
  indexed_map_sample_spacing_m_ = std::max(
      0.02, declareOrGet<double>(this, "indexed_map_sample_spacing_m",
                                 indexed_map_sample_spacing_m_));
  indexed_map_max_roughness_m_ = std::max(
      0.0, declareOrGet<double>(this, "indexed_map_max_roughness_m",
                                indexed_map_max_roughness_m_));
  indexed_map_ground_tolerance_m_ = std::clamp(
      declareOrGet<double>(this, "indexed_map_ground_tolerance_m",
                           indexed_map_ground_tolerance_m_),
      0.0, 0.50);
  indexed_map_max_source_age_s_ = std::max(
      0.0, declareOrGet<double>(this, "indexed_map_max_source_age_s",
                                indexed_map_max_source_age_s_));
  mapping_snapshot_sub_ = create_subscription<mgg_msgs::msg::MappingSnapshot>(
      "mapping_snapshot", rclcpp::QoS(1).transient_local(),
      [this](mgg_msgs::msg::MappingSnapshot::ConstSharedPtr m) {
        onMappingSnapshot(m);
      },
      sub_opts);
  if (!indexed_map_query_service_.empty()) {
    indexed_map_client_ = create_client<mgg_msgs::srv::QueryMapBatch>(
        indexed_map_query_service_, rclcpp::ServicesQoS(), callback_group_);
  }

  graph_pub_ = create_publisher<mgg_msgs::msg::Graph>("neighbour_graph_out",
                                                      rclcpp::QoS(10));
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "graph_markers", rclcpp::QoS(1));
  // Latched: the best path is a latest-value topic, and a follower or RViz
  // started after the planner would otherwise see nothing until the next
  // cycle.
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "best_path", rclcpp::QoS(1).transient_local());

  build_srv_ = create_service<std_srvs::srv::Trigger>(
      "build_local_graph",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        onBuildRequest(req, res);
      },
      rclcpp::ServicesQoS(), planning_callback_group_);

  const double requested_global_spacing =
      declareOrGet<double>(this, "global_vertex_spacing", 1.0);
  global_vertex_spacing_ = std::isfinite(requested_global_spacing)
                               ? std::clamp(requested_global_spacing, 0.01, 100.0)
                               : 1.0;
  pending_global_max_samples_ = static_cast<std::size_t>(std::clamp(
      declareOrGet<std::int64_t>(this, "global_backbone_pending_max_samples",
                                 128),
      std::int64_t{8}, std::int64_t{4096}));
  const double requested_pending_length = declareOrGet<double>(
      this, "global_backbone_pending_max_length_m", 128.0);
  pending_global_max_length_m_ =
      std::isfinite(requested_pending_length)
          ? std::clamp(requested_pending_length, global_vertex_spacing_,
                       10000.0)
          : 128.0;
  pending_global_drain_max_samples_ = static_cast<std::size_t>(std::clamp(
      declareOrGet<std::int64_t>(this, "global_backbone_drain_max_samples", 32),
      std::int64_t{1}, std::int64_t{512}));

  plan_srv_ = create_service<mgg_msgs::srv::PlannerSrv>(
      "mggplanner",
      [this](const std::shared_ptr<mgg_msgs::srv::PlannerSrv::Request> req,
             std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> res) {
        onPlanRequest(req, res);
      },
      rclcpp::ServicesQoS(), planning_callback_group_);

  objective_srv_ = create_service<mgg_msgs::srv::PlanObjective>(
      "plan_objective",
      [this](const std::shared_ptr<mgg_msgs::srv::PlanObjective::Request> req,
             std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> res) {
        onObjectiveRequest(req, res);
      },
      rclcpp::ServicesQoS(), planning_callback_group_);
  refine_objective_route_srv_ =
      create_service<mgg_msgs::srv::RefineObjectiveRoute>(
          "refine_objective_route",
          [this](const std::shared_ptr<
                     mgg_msgs::srv::RefineObjectiveRoute::Request> req,
                 std::shared_ptr<
                     mgg_msgs::srv::RefineObjectiveRoute::Response> res) {
            onRefineObjectiveRoute(req, res);
          },
          rclcpp::ServicesQoS(), planning_callback_group_);
  validate_objective_route_srv_ =
      create_service<mgg_msgs::srv::ValidateObjectiveRoute>(
          "validate_objective_route",
          [this](const std::shared_ptr<
                     mgg_msgs::srv::ValidateObjectiveRoute::Request> req,
                 std::shared_ptr<
                     mgg_msgs::srv::ValidateObjectiveRoute::Response> res) {
            onValidateObjectiveRoute(req, res);
          },
          rclcpp::ServicesQoS(), planning_callback_group_);

  const double publish_period =
      declareOrGet<double>(this, "graph_publish_period_sec", 2.0);
  // Node::create_timer drives off get_clock(), the node's RCL_ROS_TIME clock.
  // That is correct in both deployments: on a real robot use_sim_time is false
  // and ROS time follows the system clock, while in simulation it follows
  // /clock. create_wall_timer would be wrong in the second case, running at
  // wall rate while the ARGoS bridge steps simulated time at its own pace.
  graph_timer_ = create_timer(
      std::chrono::duration<double>(publish_period),
      [this]() { publishOwnGraph(); publishMarkers(); }, callback_group_);

  // The global graph expansion (rrg.cpp:80 to 82) samples in the local box
  // the lattice is built in, seeded as upstream's sampler was
  // (random_sampler.cpp:242).
  random_sampler_.setBound(grid_params_.min_val, grid_params_.max_val);
  random_sampler_.reset(std::random_device{}());
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

void PlannerNode::loadParameters() {
  ParamLoader p(this);
  if (!loadRobotParams(p, "RobotParams", robot_params_)) {
    RCLCPP_ERROR(get_logger(), "RobotParams failed to load");
  }
  if (!loadPlanningParams(p, "PlanningParams", planning_params_)) {
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
  world_frame_ = planning_params_.global_frame_id;
  component_id_ = declareOrGet<std::string>(this, "component_id", world_frame_);
  mission_id_ = declareOrGet<std::string>(this, "mission_id", "");
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


mgg::ExpandContext PlannerNode::makeContext() {
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
  ctx.hanging_root_edge_length_max =
      objective_start_support_max_distance_m_;
  return ctx;
}

mgg::ExpandContext PlannerNode::makeGlobalContext() {
  mgg::ExpandContext ctx = makeContext();
  ctx.inclinations = nullptr;
  // A roadmap edge must have been seen traversable (rrg.cpp:725).
  ctx.stop_at_unknown = true;
  return ctx;
}

std::vector<Eigen::Vector3d> PlannerNode::selectionExclusions() {
  std::vector<Eigen::Vector3d> exclusions;
  if (have_coordination_exclusions_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    coordination_exclusions_received_)
              .count() <= reservation_exclusion_ttl_s_) {
    exclusions = coordination_exclusions_;
  }
  const double now_s = steadyNowSeconds();
  while (!rejected_explore_leaves_.empty() &&
         now_s - rejected_explore_leaves_.front().second >
             rejected_explore_leaf_ttl_s_) {
    rejected_explore_leaves_.pop_front();
  }
  for (const auto& rejected : rejected_explore_leaves_) {
    exclusions.push_back(rejected.first);
  }
  return exclusions;
}

mgg::RecomputeGainFn PlannerNode::globalFrontierGain() {
  return [this](mgg::Vertex& vertex) {
    // Upstream scored global frontiers against the world-fixed global bound
    // (computeVolumetricGainRayModelNoBound, rrg.cpp:3767). This port centres
    // its gain volume on the robot every cycle, so a frontier a street away
    // would count nothing; centre it on the frontier while it is scored.
    global_space_.setCenter(vertex.state, /*use_extension=*/true);
    mgg::computeVolumetricGain(vertex.state, vertex.vol_gain,
                               makeGainContext());
  };
}

void PlannerNode::addRefPathToGraph(const std::vector<mgg::StateVec>& path) {
  if (path.size() < 2) return;
  // Upstream added the lattice vertices themselves when it could
  // (rrg.cpp:4549), which carries the leaf's frontier mark and gain across.
  // The corridor poses are lattice states, so look each one up.
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
  // A path that is not the lattice's own enters the roadmap at the height
  // every other vertex has: max_ground_height above mapped ground, the
  // projection expandGraph applies to a sample (rrg.cpp:4869 stopped at the
  // first hanging vertex; a pose without mapped ground ends the path here).
  // Route poses otherwise arrive at odometry base height or at base plus a
  // constant offset, and a flat street then holds vertices 0.12 to 0.30 m
  // above ground with false steps between consecutive route poses.
  std::vector<mgg::StateVec> projected;
  if (lattice.empty()) {
    projected.reserve(path.size());
    for (mgg::StateVec pose : path) {
      if (!projectStateToDrivingHeight(pose, /*preserve_xy=*/true)) break;
      projected.push_back(pose);
    }
    if (projected.size() < 2) {
      RCLCPP_WARN(get_logger(),
                  "exploration path not added to the global graph: no mapped "
                  "ground under its start at (%.2f, %.2f, %.2f)",
                  path.front().x(), path.front().y(), path.front().z());
      return;
    }
  }
  const int before = global_graph_->getNumVertices();
  const mgg::ExpandContext ctx = makeGlobalContext();
  const bool added =
      lattice.empty()
          ? mgg::addRefPathToGraph(*global_graph_, projected, ctx,
                                   global_vertex_spacing_)
          : mgg::addRefPathToGraph(*global_graph_, lattice, ctx,
                                   global_vertex_spacing_);
  if (!added) {
    RCLCPP_WARN(get_logger(),
                "exploration path not added to the global graph: its start "
                "at (%.2f, %.2f, %.2f) could not be linked",
                path.front().x(), path.front().y(), path.front().z());
    return;
  }
  if (global_graph_->getNumVertices() != before) ++graph_revision_;
  RCLCPP_INFO(get_logger(),
              "global graph: +%d vertices from the exploration path (%d "
              "vertices, %d edges)",
              global_graph_->getNumVertices() - before,
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
      *global_graph_, *local_graph_, makeGlobalContext(), globalFrontierGain(),
      global_vertex_spacing_, 1.0 * global_vertex_spacing_,
      3.0 * global_vertex_spacing_);
  global_space_.setCenter(current_state_, /*use_extension=*/true);
  if (global_graph_->getNumVertices() != before) ++graph_revision_;
  RCLCPP_INFO(get_logger(),
              "global graph: %d frontier(s) re-checked, %d demoted; %d local "
              "frontier(s) in %d cluster(s), %d path(s) added (%d vertices, "
              "%d edges)",
              report.global_frontiers_rechecked,
              report.global_frontiers_demoted, report.local_frontiers,
              report.clusters, report.paths_added,
              global_graph_->getNumVertices(), global_graph_->getNumEdges());
}

void PlannerNode::expandGlobalGraphTimerCallback() {
  // Reads the map and writes the global graph, like updateGlobalGraph.
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  auto map_read = mola_map_ != nullptr ? mola_map_->acquireReadLease()
                                       : mgg::MolaMap::ReadLease{};
  refreshMolaRevision();
  // rrg.cpp:2548: nothing to grow before the first plan.
  if (planner_trigger_count_ == 0) return;
  if (!have_odometry_ || !map_->getStatus()) return;

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

void PlannerNode::ingestOdometryIntoGlobalGraph() {
  if (!have_odometry_ || global_graph_->getNumVertices() == 0) return;
  constexpr double kOdoUpdateMinLength = 0.5;  // rrg.cpp:5206 and 5252
  constexpr double kMinLength = 1.0;           // rrg.cpp:5270
  const bool add_state =
      (current_state_.head<3>() - last_state_marker_.head<3>()).norm() >=
      kOdoUpdateMinLength;
  const bool record_state =
      (current_state_.head<3>() - last_state_marker_global_.head<3>())
          .norm() >= kMinLength;
  if (!add_state && !record_state) return;

  // Get position from odometry to add more vertices to the graph for homing
  // (rrg.cpp:5247 to 5268). The trajectory backbone (updateGlobalGraph) is
  // the chain of rrg.cpp:5204 to 5245; this is the checked expandGraph
  // alongside it, which wires the state to every reachable neighbour. Not
  // before the home anchor has mapped support: until then the root is a
  // landmark no edge may attach to (updateGlobalGraph), and expandGraph's
  // hanging-root allowance is a local lattice bootstrap, not a claim about
  // the ground under home.
  if (add_state && initial_anchor_supported_ && map_->getStatus()) {
    auto map_read = mola_map_ != nullptr ? mola_map_->acquireReadLease()
                                         : mgg::MolaMap::ReadLease{};
    // rrg.cpp:5259: the state at driving height. expandGraph drops a ground
    // robot's state onto the mapped terrain and refuses it where there is
    // none, so the blind offset is only the starting point of that.
    mgg::Vertex new_vertex(-1, physicalAnchorAtDrivingHeight(current_state_));
    mgg::ExpandGraphReport rep;
    mgg::expandGraph(*global_graph_, new_vertex, rep, makeGlobalContext());
    if (rep.status == mgg::ExpandGraphStatus::kSuccess) ++graph_revision_;
  }
  if (add_state) last_state_marker_ = current_state_;

  // rrg.cpp:5270 to 5290: record the state and apply event E1.
  if (record_state) {
    constexpr double kUpdateRadius = 3.0;
    robot_state_hist_.addState(current_state_);
    mgg::StateVec state = current_state_;
    global_graph_->updateVertexTypeInRange(state, kUpdateRadius);  // E1
    last_state_marker_global_ = current_state_;
  }
}

void PlannerNode::shortcutObjectiveCorridor(mgg::RouteCorridor& corridor) {
  if (corridor.status != mgg::PlanningStatus::kSucceeded ||
      corridor.poses.size() < 3) {
    return;
  }
  const Eigen::Vector3d footprint = robot_params_.getPlanningSize();
  const Eigen::Vector3d offset = robot_params_.center_offset;
  const auto segment_free = [this, &footprint, &offset](
                                const Eigen::Vector3d& from,
                                const Eigen::Vector3d& to) {
    // A shortcut may only cross space already seen traversable: unknown
    // volume blocks it, and a ground robot's segment follows the terrain
    // (steps, inclination) as every roadmap edge does.
    if (robot_params_.type == mgg::RobotType::kGroundRobot) {
      std::vector<Eigen::Vector3d> projected;
      return ground_->getProjectedEdgeStatus(
                 from + offset, to + offset, footprint,
                 /*stop_at_unknown_voxel=*/true, projected,
                 /*is_hanging=*/false) == mgg::ProjectedEdgeStatus::kAdmissible;
    }
    return map_->getPathStatus(from, to, footprint, true) ==
           mgg::VoxelStatus::kFree;
  };
  mgg::PathType points;
  points.reserve(corridor.poses.size());
  for (const mgg::StateVec& pose : corridor.poses) points.push_back(pose.head<3>());
  const mgg::PathType cut = mgg::shortcutPath(points, segment_free);
  if (cut.size() >= points.size()) return;
  // shortcutPath keeps a subsequence of its input, so each kept point maps
  // back to the pose it came from; interior headings follow the new segments.
  std::vector<mgg::StateVec> poses;
  poses.reserve(cut.size());
  std::size_t source = 0;
  for (const Eigen::Vector3d& point : cut) {
    while (source < corridor.poses.size() &&
           (corridor.poses[source].head<3>() - point).norm() > 1e-9) {
      ++source;
    }
    if (source >= corridor.poses.size()) return;  // not a subsequence: keep the route
    poses.push_back(corridor.poses[source]);
  }
  for (std::size_t i = 1; i + 1 < poses.size(); ++i) {
    const Eigen::Vector2d step = poses[i + 1].head<2>() - poses[i].head<2>();
    if (step.norm() > 1e-6) poses[i][3] = std::atan2(step.y(), step.x());
  }
  RCLCPP_INFO(get_logger(), "objective route shortcut: %zu poses -> %zu",
              corridor.poses.size(), poses.size());
  corridor.poses = std::move(poses);
}

mgg::RouteCorridor PlannerNode::runGlobalPlanner(mgg::PlanningRequest& core,
                                                 TopologicalRetry& retry) {
  mgg::RouteCorridor corridor;
  corridor.request = core;
  corridor.status = mgg::PlanningStatus::kUnreachable;
  if (global_graph_->getNumVertices() <= 1) {
    // rrg.cpp:5582.
    corridor.reason = "exploration complete: the global graph holds no "
                      "frontier to reposition to";
    return corridor;
  }
  // Let's try to add the current state to the global graph (rrg.cpp:5643).
  mgg::StateVec current = current_state_;
  if (!projectStateToDrivingHeight(current)) {
    current = physicalAnchorAtDrivingHeight(current_state_);
  }
  const int before = global_graph_->getNumVertices();
  constexpr double kRadiusLimit = 1.5;  // rrg.cpp:5651
  mgg::Vertex* link_vertex = mgg::connectStateToGraph(
      *global_graph_, current, makeGlobalContext(), kRadiusLimit);
  if (global_graph_->getNumVertices() != before) ++graph_revision_;
  if (link_vertex == nullptr) {
    corridor.reason = "current pose cannot be linked to the global graph";
    return corridor;
  }
  const mgg::GlobalFrontierReport report = mgg::searchGlobalFrontier(
      *global_graph_, link_vertex->id,
      static_cast<int>(planning_params_.robot_id), globalFrontierGain(),
      selectionExclusions(), reservation_exclusion_radius_m_);
  global_space_.setCenter(current_state_, /*use_extension=*/true);
  if (report.best_frontier == nullptr) {
    // rrg.cpp:5628 and 5759: no frontier, or none the graph can reach.
    char reason[192];
    std::snprintf(reason, sizeof(reason),
                  "exploration complete: %d global frontier(s), none "
                  "reachable with gain (%d re-checked out)",
                  report.frontiers, report.demoted);
    corridor.reason = reason;
    return corridor;
  }
  RCLCPP_INFO(get_logger(),
              "global planner: %d frontier(s), %d feasible; repositioning to "
              "(%.2f, %.2f, %.2f), gain %.1f over %.1f m",
              report.frontiers, report.feasible,
              report.best_frontier->state.x(), report.best_frontier->state.y(),
              report.best_frontier->state.z(), report.best_gain,
              report.best_distance);
  // Route to it over the global graph with the shared topological stage
  // (rrg.cpp:5846). The goal is a graph vertex, so the explicit objectives'
  // binding tolerance resolves it exactly.
  core.goal.pose = report.best_frontier->state;
  core.goal.landmark_id.clear();
  selected_explore_leaf_ = report.best_frontier->state.head<3>();
  mgg::TopologicalGoalPlanner global_planner(
      core.component_id, core.graph_revision, core.map_revision, 1.0);
  retry.valid = true;
  retry.graph = global_graph_.get();
  retry.current = current;
  retry.request = core;
  retry.goal_tolerance = 1.0;
  retry.minimum_partial_progress = 0.0;
  corridor = global_planner.plan(*global_graph_, current, core,
                                 blockedCorridorView());
  corridor.request = core;
  shortcutObjectiveCorridor(corridor);
  return corridor;
}

void PlannerNode::planExploreCorridor(mgg::PlanningRequest& core,
                                      const std::string& summary,
                                      mgg::RouteCorridor& corridor,
                                      TopologicalRetry& retry,
                                      bool& from_global_graph) {
  from_global_graph = false;
  if (have_explore_selection_) {
    // The selector's leaf is the objective goal from here on, so the window,
    // the response and the continuation token all agree on it.
    core.goal.pose = explore_target_;
    core.goal.landmark_id.clear();
    // A lattice vertex is an exact graph state. Bind it tightly so the stage
    // cannot substitute a neighbouring vertex for the chosen leaf.
    const double explore_tolerance =
        std::max(1e-3, 0.5 * map_->getResolution());
    mgg::TopologicalGoalPlanner explore_planner(
        core.component_id, core.graph_revision, core.map_revision,
        explore_tolerance);
    retry.valid = true;
    retry.graph = local_graph_.get();
    retry.current = explore_root_;
    retry.request = core;
    retry.goal_tolerance = explore_tolerance;
    retry.minimum_partial_progress = 0.0;
    corridor = explore_planner.plan(*local_graph_, explore_root_, core,
                                    blockedCorridorView());
    corridor.request = core;
    return;
  }
  // No leaf with gain: every local path is blocked, excluded or sees nothing
  // new. Upstream triggered the global planner here (rrg.cpp:2119 and
  // mggplanner.cpp:217) rather than returning no path.
  RCLCPP_INFO(get_logger(),
              "no local gain (%s); asking the global graph for a frontier",
              summary.empty() ? "no selection" : summary.c_str());
  corridor = runGlobalPlanner(core, retry);
  from_global_graph = corridor.status == mgg::PlanningStatus::kSucceeded;
}

void PlannerNode::refreshMolaRevision() {
  if (mola_map_ == nullptr) return;
  const std::uint64_t generation = mola_map_->activeGeneration();
  if (generation == observed_mola_generation_) return;
  observed_mola_generation_ = generation;
  ++map_revision_;
}

void PlannerNode::stageGlobalBreadcrumbs(const mgg::StateVec& state) {
  if (!have_global_sampling_anchor_) {
    global_sampling_anchor_ = state;
    have_global_sampling_anchor_ = true;
    last_global_odometry_ = state;
    return;
  }

  const auto loseHistory = [this](const char* reason) {
    if (global_backbone_history_lost_) return;
    global_backbone_history_lost_ = true;
    global_backbone_history_lost_reason_ = reason;
    RCLCPP_ERROR(get_logger(),
                 "global trajectory history lost: %s (%zu samples, %.1f m); "
                 "Return Home will remain blocked",
                 global_backbone_history_lost_reason_.c_str(),
                 pending_global_breadcrumbs_.size(), pending_global_length_m_);
  };
  const auto append = [this](const mgg::StateVec& sample,
                             double path_length, const auto& lose_history) {
    if (global_backbone_history_lost_) return false;
    if (!sample.allFinite() || !std::isfinite(path_length) ||
        path_length <= 0.0) {
      lose_history("invalid pending breadcrumb geometry");
      return false;
    }
    const bool count_full =
        pending_global_breadcrumbs_.size() >= pending_global_max_samples_;
    const bool length_full =
        pending_global_length_m_ + path_length >
        pending_global_max_length_m_ + 1e-9;
    if (count_full || length_full) {
      lose_history(count_full ? "pending breadcrumb count limit exceeded"
                              : "pending breadcrumb length limit exceeded");
      return false;
    }
    pending_global_breadcrumbs_.push_back({sample, path_length});
    pending_global_length_m_ += path_length;
    return true;
  };

  const auto interpolate = [](const mgg::StateVec& a,
                              const mgg::StateVec& b, double fraction) {
    mgg::StateVec sample = a + fraction * (b - a);
    const double yaw_delta =
        std::atan2(std::sin(b[3] - a[3]), std::cos(b[3] - a[3]));
    sample[3] = a[3] + fraction * yaw_delta;
    return sample;
  };

  const Eigen::Vector3d motion =
      state.head<3>() - last_global_odometry_.head<3>();
  if (!motion.allFinite()) {
    loseHistory("odometry displacement is not finite");
    return;
  }
  const double last_motion_norm = last_global_motion_.norm();
  const double motion_norm = motion.norm();
  if (!std::isfinite(last_motion_norm) || !std::isfinite(motion_norm)) {
    loseHistory("odometry displacement exceeds numeric range");
    return;
  }
  if (!global_backbone_history_lost_ && last_motion_norm > 1e-6 &&
      motion_norm > 1e-6) {
    const double cosine = std::clamp(
        (last_global_motion_ / last_motion_norm).dot(motion / motion_norm),
        -1.0, 1.0);
    const double turn = std::acos(cosine);
    const Eigen::Vector3d from_anchor_delta =
        last_global_odometry_.head<3>() - global_sampling_anchor_.head<3>();
    const double from_anchor = from_anchor_delta.norm();
    if (!from_anchor_delta.allFinite() || !std::isfinite(from_anchor)) {
      loseHistory("breadcrumb progress exceeds numeric range");
      return;
    }
    // Keep meaningful corners that fall between regular samples. The minimum
    // displacement prevents stationary pose noise from manufacturing vertices.
    if (turn >= M_PI / 6.0 &&
        from_anchor >= global_vertex_spacing_ * 0.25) {
      if (append(last_global_odometry_, from_anchor, loseHistory)) {
        global_sampling_anchor_ = last_global_odometry_;
      }
    }
  }

  while (!global_backbone_history_lost_) {
    const Eigen::Vector3d delta =
        state.head<3>() - global_sampling_anchor_.head<3>();
    const double distance = delta.norm();
    if (!delta.allFinite() || !std::isfinite(distance)) {
      loseHistory("breadcrumb progress exceeds numeric range");
      break;
    }
    if (distance < global_vertex_spacing_) break;
    const double fraction = global_vertex_spacing_ / distance;
    if (!std::isfinite(fraction) || fraction <= 0.0 || fraction > 1.0 + 1e-9) {
      loseHistory("breadcrumb interpolation made no finite progress");
      break;
    }
    const mgg::StateVec sample =
        interpolate(global_sampling_anchor_, state, fraction);
    if ((sample.head<3>().array() ==
         global_sampling_anchor_.head<3>().array())
            .all()) {
      loseHistory("breadcrumb interpolation made no numeric progress");
      break;
    }
    if (!append(sample, global_vertex_spacing_, loseHistory)) break;
    global_sampling_anchor_ = sample;
  }

  if (motion_norm > 1e-6) last_global_motion_ = motion;
  last_global_odometry_ = state;
}

void PlannerNode::onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  const mgg::StateVec state = fromPoseMsg(msg->pose.pose);
  if (!state.allFinite()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "ignoring non-finite odometry");
    return;
  }

  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  const std::int64_t stamp_ns =
      static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(msg->header.stamp.nanosec);
  if (have_odometry_ && stamp_ns < last_odometry_stamp_ns_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "ignoring odometry %.3f s older than the current state",
                         static_cast<double>(last_odometry_stamp_ns_ - stamp_ns) *
                             1e-9);
    return;
  }
  last_odometry_stamp_ns_ = stamp_ns;
  current_state_ = state;
  have_odometry_ = true;
  last_odometry_received_ = now();
  if (!have_initial_state_) {
    initial_state_ = state;
    have_initial_state_ = true;
  }
  stageGlobalBreadcrumbs(state);
  refreshMolaRevision();
  updateGlobalGraph();
  ingestOdometryIntoGlobalGraph();
}

std::string PlannerNode::staleOdometryReason() const {
  if (!have_odometry_ || !last_odometry_received_) return "";
  const double age_s =
      static_cast<double>(now().nanoseconds() -
                          last_odometry_received_->nanoseconds()) *
      1e-9;
  if (!(age_s > odometry_stale_s_)) return "";
  char reason[128];
  std::snprintf(reason, sizeof(reason),
                "odometry or planning map is unavailable: last odometry "
                "%.1f s old",
                age_s);
  return reason;
}

void PlannerNode::onPointCloud(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  if (cloud_map_ == nullptr || msg->data.empty()) return;

  // Transform into world coordinates before projecting into the octree.
  // The sensor publishes in its own frame (e.g. "r0/lidar") and the map lives
  // in world_frame_.
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(world_frame_, msg->header.frame_id,
                                         msg->header.stamp,
                                         rclcpp::Duration::from_seconds(0.1));
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
    cloud_map_->insertPointCloud(points, origin);
    ++map_revision_;
    // A new map revision may admit the first blocked chronological breadcrumb.
    if (have_odometry_ &&
        (!initial_anchor_supported_ || !pending_global_breadcrumbs_.empty())) {
      updateGlobalGraph();
    }
  }
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
    const auto& q = pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6 ||
        std::abs(q.z) > 1e-6 || std::abs(q.w - 1.0) > 1e-6) {
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
  mola_map_->setTransientDiscs(std::move(centres), peer_body_radius_m_,
                               peer_body_ttl_s_);
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
                         "ignoring invalid indexed mapping snapshot");
    return;
  }
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  component_from_navigation_ = Eigen::Isometry3d::Identity();
  component_from_navigation_.linear() =
      Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
  component_from_navigation_.translation() = Eigen::Vector3d(t.x, t.y, t.z);
  mapping_snapshot_ = *msg;
  mapping_snapshot_received_ = std::chrono::steady_clock::now();
  have_mapping_snapshot_ = true;
  if (mola_map_ != nullptr) {
    refreshMolaRevision();
    const std::string prior_error = mola_map_->lastError();
    if (!prior_error.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "MOLA map unavailable: %s", prior_error.c_str());
    }
    const std::uint64_t source_stamp_ns =
        static_cast<std::uint64_t>(msg->source_stamp.sec) * 1000000000ull +
        msg->source_stamp.nanosec;
    mola_map_->requestSnapshot({msg->component_id, msg->epoch,
                                msg->graph_revision, msg->geometry_revision,
                                source_stamp_ns, component_from_navigation_});
  }
}

void PlannerNode::onNeighbourGraph(mgg_msgs::msg::Graph::ConstSharedPtr msg) {
  if (msg->vertices.empty()) return;
  const int sender = msg->vertices.front().robot_id;
  if (sender == static_cast<int>(planning_params_.robot_id)) return;

  // Merging reads the map to decide reachability and writes the global graph.
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  auto map_read = mola_map_ != nullptr ? mola_map_->acquireReadLease()
                                       : mgg::MolaMap::ReadLease{};
  refreshMolaRevision();

  // Communication range filter: robots must be within direct radio range.
  Eigen::Isometry3d t_ours_theirs = Eigen::Isometry3d::Identity();
  if (poses_->getRobotTransform(sender, t_ours_theirs) && !msg->vertices.empty()) {
    const auto& last_v = msg->vertices.back();
    const Eigen::Vector3d their_latest(last_v.pose.position.x,
                                       last_v.pose.position.y,
                                       last_v.pose.position.z);
    const Eigen::Vector3d their_world = t_ours_theirs * their_latest;
    const Eigen::Vector3d our_world(current_state_[0], current_state_[1], current_state_[2]);
    const double dist = (their_world - our_world).norm();
    if (communication_range_ > 0.0 && dist > communication_range_) {
      return;
    }
  }


  const mgg::GraphExchange incoming = fromGraphMsg(*msg);
  const mgg::ExpandContext ctx = makeContext();
  // The merge asks whether the robot could actually drive between two graphs
  // before joining them; that judgement needs the map, so it is injected.
  const auto admissible = [this, &ctx](const Eigen::Vector3d& from,
                                       const Eigen::Vector3d& to) {
    if (robot_params_.type == mgg::RobotType::kAerialRobot) {
      return map_->getPathStatus(from, to, ctx.robot_box_size, true) ==
             mgg::VoxelStatus::kFree;
    }
    std::vector<Eigen::Vector3d> projected;
    return ground_->getProjectedEdgeStatus(from, to, ctx.robot_box_size, true,
                                           projected, false) ==
           mgg::ProjectedEdgeStatus::kAdmissible;
  };

  const mgg::MergeResult r = mgg::mergeNeighbourGraph(
      *global_graph_, incoming, *poses_, admissible);

  if (r.vertices_added > 0 || r.edges_added > 0) ++graph_revision_;

  if (r.transform_unavailable) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "no transform for robot %d; set neighbour_offsets or "
                         "supply a SLAM-backed PoseSource", sender);
    return;
  }
  if (r.newly_connected) {
    if (poses_->getRobotTransform(sender, t_ours_theirs) && !incoming.vertices.empty()) {
      const Eigen::Vector3d their_local(incoming.vertices.front().state[0],
                                        incoming.vertices.front().state[1],
                                        incoming.vertices.front().state[2]);
      const Eigen::Vector3d their_world = t_ours_theirs * their_local;
      const Eigen::Vector3d our_pos(current_state_[0], current_state_[1], current_state_[2]);
      recent_merges_.push_back({now(), sender, our_pos, their_world});
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



std::string PlannerNode::buildLocalGraph(bool strict_projected_endpoints) {
  // Held for the whole cycle: the map must not change under a planner that is
  // ray-casting through it. Point clouds arriving meanwhile queue up, and the
  // subscription's best-effort depth decides how many survive.
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  auto map_read = mola_map_ != nullptr ? mola_map_->acquireReadLease()
                                       : mgg::MolaMap::ReadLease{};
  refreshMolaRevision();
  if (!have_odometry_) return "no odometry received yet";
  if (!map_->getStatus()) {
    if (mola_map_ != nullptr) {
      const std::string detail = mola_map_->lastError();
      return detail.empty() ? "MOLA map snapshot is missing or stale"
                            : "MOLA map unavailable: " + detail;
    }
    return "map is empty; no point cloud received yet";
  }

  // Per-phase timing. A cycle that never returns says nothing about which of
  // the three phases is responsible, and they scale with completely different
  // things: the sweep with the lattice volume, the gain with the number of
  // viewpoints times the sensor's ray count, the selection with the graph.
  using Clock = std::chrono::steady_clock;
  const auto t_start = Clock::now();
  updateGlobalGraph();

  // The last local graph's frontier paths go into the global graph before
  // that graph is thrown away (rrg.cpp:121, Rrg::reset).
  if (add_frontiers_to_global_graph_) {
    add_frontiers_to_global_graph_ = false;
    addFrontiers();
  }

  local_graph_->reset();
  have_explore_selection_ = false;
  // Inclinations are keyed by vertex id and the ids restart with the graph.
  edge_inclinations_.clear();
  mgg::StateVec root_state = current_state_;
  bool root_hanging = false;
  if (robot_params_.type == mgg::RobotType::kGroundRobot) {
    const bool preserve_physical_root =
        map_backend_ == "mola_snapshot" && observed_ground_body_evidence_ &&
        provisional_unknown_ground_;
    if (preserve_physical_root) {
      root_state = physicalAnchorAtDrivingHeight(current_state_);
      root_hanging = true;
    } else {
      Eigen::Vector3d pos(root_state[0], root_state[1], root_state[2]);
      mgg::VoxelStatus vs;
      const double ground_height = ground_->projectSample(pos, vs);
      if (vs == mgg::VoxelStatus::kOccupied) {
        root_state[2] =
            pos[2] - (ground_height - planning_params_.max_ground_height);
      } else {
        root_hanging = true;
        // Odometry locates the base, whereas graph states locate the raised
        // collision box. Preserve this offset inside the sensor blind spot.
        root_state[2] += planning_params_.max_ground_height -
                         robot_params_.size[2] / 2.0;
      }
    }
  }
  auto* root = new mgg::Vertex(0, root_state);
  root->robot_id = static_cast<int>(planning_params_.robot_id);
  // A single lidar return beneath the physical pose does not prove the
  // near-field connector between the robot and the first observed floor. In
  // the qualified simulation policy, keep only this local root eligible for
  // the existing bounded hanging-edge bootstrap. The qualified MOLA branch
  // preserves its odometry-derived physical height above; all child vertices
  // and all known-hazard checks remain strict.
  root->is_hanging =
      root_hanging ||
      (provisional_unknown_ground_ &&
       robot_params_.type == mgg::RobotType::kGroundRobot);
  local_graph_->addVertex(root);



  mgg::ExpandContext ctx = makeContext();
  ctx.strict_projected_endpoint = strict_projected_endpoints;
  // This policy is constructor-qualified to simulated ground robots. Offer
  // sparse MOLA lattice cells to the existing ground and edge checks even
  // when their body volume is not fully ray-observed; known occupied cells
  // still fail the prefilter. Hardware retains the strict default.
  ctx.allow_unknown_lattice_body = observed_ground_body_evidence_;
  ctx.preserve_hanging_root_start_height =
      map_backend_ == "mola_snapshot" && observed_ground_body_evidence_ &&
      provisional_unknown_ground_ &&
      robot_params_.type == mgg::RobotType::kGroundRobot;
  if (observed_ground_body_evidence_ &&
      robot_params_.type == mgg::RobotType::kGroundRobot) {
    // Explore bypasses explicit-objective grid refinement, so apply the same
    // known-terrain and occupied-body veto before graph alternatives are
    // selected. Graph expansion supplies centre-offset coordinates.
    ctx.projected_edge_admissible = [this](
        const std::vector<Eigen::Vector3d>& projected) {
      std::vector<Eigen::Vector3d> driving;
      driving.reserve(projected.size());
      for (const Eigen::Vector3d& point : projected) {
        driving.push_back(point - robot_params_.center_offset);
      }
      return objectiveTerrainPathSupported(driving);
    };
  }
  const auto t_global = Clock::now();
  const mgg::GridGraphResult r = buildGridGraph(
      *local_graph_, root_state, grid_params_, ctx, current_state_[3]);
  ++local_graph_revision_;
  local_graph_map_revision_ = map_revision_;
  ++planner_trigger_count_;  // rrg.cpp:1332

  if (r.status == mgg::GridGraphStatus::kInvalidBounds) {
    return "grid bounds invalid: min_val must be <= 0, max_val >= 0 and "
           "resolution non-zero";
  }
  const auto t_grid = Clock::now();
  // The gain evaluation needs the sampling volume centred on the robot, since
  // it rejects voxels outside it.
  global_space_.setCenter(current_state_, /*use_extension=*/true);

  mgg::GainContext gain_ctx = makeGainContext();
  const int evaluated = mgg::computeExplorationGain(
      *local_graph_, gain_ctx, planning_params_.leafs_only_for_volumetric_gain,
      planning_params_.cluster_vertices_for_gain);

  int frontiers = 0;
  for (const auto& entry : local_graph_->vertices_map_) {
    if (entry.second != nullptr &&
        entry.second->type == mgg::VertexType::kFrontier) {
      ++frontiers;
    }
  }

  const auto t_gain = Clock::now();
  const mgg::PathSelectionResult sel = mgg::selectBestPath(
      *local_graph_, planning_params_, robot_params_, edge_inclinations_,
      map_->getResolution(), exploring_direction_, selectionExclusions(),
      reservation_exclusion_radius_m_);

  best_path_.clear();
  path_shortcut_from_ = 0;
  path_shortcut_corners_ = 0;
  path_shortcut_to_ = 0;
  for (const mgg::Vertex* v : sel.best_path) {
    if (v != nullptr) best_path_.push_back(v->state);
  }
  selected_explore_leaf_.reset();
  if (!best_path_.empty()) {
    selected_explore_leaf_ = best_path_.back().head<3>();
  }
  const std::vector<mgg::StateVec> selected_lattice_path = best_path_;
  // The gain selector owns the exploration target. The shared topological
  // stage plans the corridor to it, so the exact lattice root and the chosen
  // leaf are what Explore hands across that boundary.
  have_explore_selection_ = !selected_lattice_path.empty();
  explore_root_ = root_state;
  explore_target_ = have_explore_selection_ ? selected_lattice_path.back()
                                            : root_state;
  // A graph with a path worth taking has frontiers worth remembering
  // (rrg.cpp:2147); they are added before this graph is rebuilt.
  if (have_explore_selection_) add_frontiers_to_global_graph_ = true;


  // What comes out of the graph is a walk along lattice edges: it steps
  // between cell centres and reads as a staircase even across open floor. ROS 1
  // ran every path it returned through improveFreePath and interpolatePath
  // (rrg.cpp:4160, 4176) and this port was publishing the raw walk, which is
  // why the paths looked erratic in a map with nothing in them to avoid.
  //
  // Shortcut first, then resample. The other order interpolates points that
  // are about to be discarded, and leaves the corners the shortcut removed
  // still bent.
  if (best_path_.size() > 2) {
    const mgg::ExpandContext path_ctx = makeContext();
    const auto segment_free = [this, &path_ctx](const Eigen::Vector3d& from,
                                                const Eigen::Vector3d& to) {
      // stop_at_unknown_voxel is true: a shortcut may only cross space
      // already known to be free.
      //
      // Upstream passes false here (rrg.cpp:4610), which treats unknown space
      // as passable. That is survivable there because its shortcut only ever
      // collapses a node when the segment leading to it is under half a metre,
      // so the leap is short. Applied to a general shortcut it is not: a
      // partly explored map is mostly unknown, so every candidate line
      // qualifies and the path collapses to a straight run from the robot to
      // the goal - measured, 15 lattice points became 2 - straight through
      // whatever has not been seen yet. The lattice detour stays wherever the
      // map cannot yet vouch for the straight line, which near a frontier is
      // exactly where it should.
      return map_->getPathStatus(from, to, path_ctx.robot_box_size, true) ==
             mgg::VoxelStatus::kFree;
    };
    mgg::PathType points;
    points.reserve(best_path_.size());
    for (const mgg::StateVec& s : best_path_) points.push_back(s.head(3));

    const size_t before = points.size();
    points = mgg::shortcutPath(points, segment_free);
    path_shortcut_corners_ = static_cast<int>(points.size());
    mgg::PathType resampled;
    if (planning_params_.path_interpolation_distance > 0.0 &&
        mgg::interpolatePath(points,
                             planning_params_.path_interpolation_distance,
                             resampled) &&
        resampled.size() >= 2) {
      points = resampled;
    }
    path_shortcut_from_ = static_cast<int>(before);
    path_shortcut_to_ = static_cast<int>(points.size());

    // Rebuild the states, keeping each point's heading pointing along the path
    // it is now on rather than along the lattice edge it came from.
    std::vector<mgg::StateVec> rebuilt;
    rebuilt.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
      const Eigen::Vector3d& here = points[i];
      const Eigen::Vector3d& ahead = points[i + 1 < points.size() ? i + 1 : i];
      const Eigen::Vector3d step = ahead - here;
      const double yaw = step.head(2).norm() > 1e-9
                             ? std::atan2(step.y(), step.x())
                             : (rebuilt.empty() ? best_path_.front()[3]
                                                : rebuilt.back()[3]);
      rebuilt.emplace_back(here.x(), here.y(), here.z(), yaw);
    }
    best_path_ = rebuilt;
  }
  if (observed_ground_body_evidence_ && !best_path_.empty()) {
    // Shortcutting and interpolation happen after graph admission. If either
    // operation crosses incompatible terrain, retain the selected lattice
    // route whose individual edges were already checked during expansion.
    if (!retainTerrainSafeExplorationPath(
            selected_lattice_path,
            [this](const std::vector<Eigen::Vector3d>& path) {
              return objectiveTerrainPathSupported(path);
            },
            best_path_)) {
      path_shortcut_corners_ = path_shortcut_from_;
      path_shortcut_to_ = path_shortcut_from_;
    }
  }
  if (best_path_.size() >= 2) {
    // Remember where this path is heading, so the next cycle penalises
    // doubling back.
    std::vector<Eigen::Vector3d> points;
    points.reserve(best_path_.size());
    for (const mgg::StateVec& s : best_path_) points.push_back(s.head(3));
    exploring_direction_ = mgg::estimateDirectionFromPath(points);
  }
  publishPath();
  publishMarkers();

  // A very low graph acceptance ratio is the characteristic support failure:
  // the lattice finds free body boxes while projection or swept edges reject
  // nearly every candidate. Keep this aggregate; per-sample logs would flood
  // the fleet and perturb planning timing.
  char why[192] = "";
  const bool low_acceptance =
      r.free_cells > 0 &&
      static_cast<std::int64_t>(r.vertices_added) * 20 < r.free_cells;
  if (low_acceptance) {
    std::snprintf(why, sizeof(why),
                  " (rejected: %d collision, %d no ground, projected body "
                  "%d occupied/%d unknown; edges: %d ok, "
                  "%d steep, %d occupied, %d unmapped, %d hanging)",
                  r.rejected[static_cast<int>(mgg::ExpandGraphStatus::
                                                  kErrorCollisionEdge)],
                  r.no_ground, r.projected_endpoint_occupied,
                  r.projected_endpoint_unknown, r.edge_status[0], r.edge_status[1],
                  r.edge_status[2], r.edge_status[3], r.edge_status[4]);
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
  char buf[640];
  std::snprintf(
      buf, sizeof(buf),
      "grid graph: %d free cells, %d vertices, %d edges%s%s; %d viewpoints, "
      "%d frontiers; best path %zu poses (%d lattice -> %d corners -> %d "
      "resampled), gain %.1f%s, "
      "heading %.2f rad%s",
      r.free_cells, r.vertices_added, r.edges_added,
      r.hit_limit ? " (hit a size limit)" : "", why, evaluated, frontiers,
      best_path_.size(), path_shortcut_from_, path_shortcut_corners_,
      path_shortcut_to_, sel.best_gain,
      sel.paths_rejected_steep > 0 ? " (some paths too steep)" : "",
      exploring_direction_, timing);
  return std::string(buf);
}

bool PlannerNode::projectStateToDrivingHeight(mgg::StateVec& state,
                                              bool preserve_xy,
                                              bool accept_ground_above_sample) const {
  if (robot_params_.type != mgg::RobotType::kGroundRobot) return true;
  const Eigen::Vector2d requested_xy = state.head<2>();
  Eigen::Vector3d pos(state[0], state[1], state[2]);
  mgg::VoxelStatus status;
  const double ground_height = ground_->projectSample(pos, status);
  if (status != mgg::VoxelStatus::kOccupied || !std::isfinite(ground_height) ||
      (!accept_ground_above_sample && ground_height < 0.0)) {
    return false;
  }
  if (preserve_xy &&
      (pos.head<2>() - requested_xy).cwiseAbs().maxCoeff() > 1e-3) {
    return false;
  }
  state[0] = preserve_xy ? requested_xy.x() : pos[0];
  state[1] = preserve_xy ? requested_xy.y() : pos[1];
  const double projected_z =
      pos[2] - (ground_height - planning_params_.max_ground_height);
  if (!std::isfinite(projected_z)) return false;
  state[2] = projected_z;
  return state.allFinite();
}

bool PlannerNode::resolveNavigateGoalDrivingHeight(
    mgg::StateVec& state) const {
  if (robot_params_.type != mgg::RobotType::kGroundRobot) {
    return projectStateToDrivingHeight(state, true);
  }
  if (!state.allFinite() || !ground_ || !map_ ||
      !std::isfinite(robot_params_.size.z()) || robot_params_.size.z() < 0.0 ||
      !std::isfinite(planning_params_.max_ground_height) ||
      !std::isfinite(ground_->max_projection_length) ||
      ground_->max_projection_length <= 0.0) {
    return false;
  }
  const double resolution = map_->getResolution();
  if (!std::isfinite(resolution) || resolution <= 0.0) return false;
  const double map_extent = resolution * 32768.0;
  if (!std::isfinite(map_extent) || map_extent <= 0.0 ||
      (state.head<3>().cwiseAbs().array() >= map_extent).any()) {
    return false;
  }
  if (projectStateToDrivingHeight(state, true)) return true;

  // A UI goal carries navigation-base Z, which is often the robot's current
  // height even when distant road terrain has risen gradually. Search upward
  // in bounded vertical bands for the first observed surface at the exact XY.
  // The resulting pose is still only an A* endpoint: footprint, body,
  // geofence, step, inclination, and swept-edge checks must connect it from
  // the current pose before any command path is emitted.
  constexpr std::size_t kMaxHeightProbes = 64;
  const mgg::StateVec requested = state;
  const double navigation_to_driving =
      planning_params_.max_ground_height - robot_params_.size.z() / 2.0;
  const double band = std::max(0.20, 2.0 * resolution);
  const double search_limit = ground_->max_projection_length;
  const double stride =
      std::max(band, search_limit / double(kMaxHeightProbes - 1));
  const double requested_ground = requested.z() - robot_params_.size.z() / 2.0;
  if (!std::isfinite(navigation_to_driving) || !std::isfinite(stride) ||
      !std::isfinite(requested_ground)) {
    return false;
  }

  for (std::size_t probe = 0; probe < kMaxHeightProbes; ++probe) {
    const double lift = std::min(search_limit, probe * stride);
    mgg::StateVec candidate = requested;
    candidate.z() += navigation_to_driving + lift;
    if (!candidate.allFinite() ||
        (candidate.head<3>().cwiseAbs().array() >= map_extent).any()) {
      return false;
    }
    if (projectStateToDrivingHeight(candidate, true, true)) {
      const double resolved_ground =
          candidate.z() - planning_params_.max_ground_height;
      const double rise = resolved_ground - requested_ground;
      if (std::isfinite(rise) && rise >= -1e-6 &&
          rise <= search_limit + 1e-6) {
        state = candidate;
        return true;
      }
    }
    if (lift >= search_limit) break;
  }
  return false;
}

mgg::StateVec PlannerNode::physicalAnchorAtDrivingHeight(
    const mgg::StateVec& base_pose) const {
  mgg::StateVec anchor = base_pose;
  if (robot_params_.type == mgg::RobotType::kGroundRobot) {
    anchor[2] += planning_params_.max_ground_height -
                 robot_params_.size[2] / 2.0;
  }
  return anchor;
}

bool PlannerNode::validateObjectiveStartSupport(
    const mgg::StateVec& anchor, const mgg::StateVec& supported,
    std::vector<mgg::StateVec>& checked, bool tolerate_unknown_body,
    bool physical_anchor) const {
  checked.clear();
  if (robot_params_.type != mgg::RobotType::kGroundRobot || !ground_ ||
      !anchor.allFinite() || !supported.allFinite() ||
      !std::isfinite(objective_start_support_max_distance_m_) ||
      objective_start_support_max_distance_m_ <= 0.0) {
    if (objective_start_support_failure_.empty())
      objective_start_support_failure_ = "connector configuration or state invalid";
    return false;
  }
  const double distance =
      (supported.head<3>() - anchor.head<3>()).norm();
  if (!std::isfinite(distance) || distance <= 1e-9 ||
      distance > objective_start_support_max_distance_m_ + 1e-9) {
    if (objective_start_support_failure_.empty())
      objective_start_support_failure_ = "connector exceeds bounded distance";
    return false;
  }

  // Missing ground may bridge one physical anchor to a mapped endpoint, but
  // it never makes occupancy free: the full swept body follows the selected
  // objective evidence policy and always rejects occupied or invalid queries.
  std::vector<Eigen::Vector3d> projected;
  const Eigen::Vector3d footprint_body = robot_params_.getPlanningSize();
  Eigen::Vector3d body = footprint_body;
  if (observed_ground_body_evidence_ &&
      robot_params_.type == mgg::RobotType::kGroundRobot) {
    const double xy_diagonal = footprint_body.head<2>().norm();
    body.x() = xy_diagonal;
    body.y() = xy_diagonal;
  }
  const bool qualified_mola_body_envelope =
      map_backend_ == "mola_snapshot" && observed_ground_body_evidence_ &&
      provisional_unknown_ground_ &&
      robot_params_.type == mgg::RobotType::kGroundRobot;
  if (ground_->getProjectedEdgeStatus(
          anchor.head<3>(), supported.head<3>(),
          qualified_mola_body_envelope ? footprint_body : body,
          /*stop_at_unknown_voxel=*/false, projected,
          /*is_hanging=*/true) != mgg::ProjectedEdgeStatus::kAdmissible ||
      projected.size() < 2) {
    if (objective_start_support_failure_.empty() ||
        objective_start_support_failure_ == "connector exceeds bounded distance")
      objective_start_support_failure_ = "projected connector inadmissible";
    return false;
  }

  const Eigen::Vector3d center_offset = robot_params_.center_offset;
  checked.reserve(projected.size());
  for (std::size_t point_index = 0; point_index < projected.size();
       ++point_index) {
    const Eigen::Vector3d& point = projected[point_index];
    // Odometry proves that the robot stands on the anchor itself. A kerb a
    // few millimetres over the step limit under one wheel must not refuse
    // the connector; every point the robot has yet to reach, the height
    // steps and the whole swept body remain mandatory.
    const bool stationary_physical_pose =
        physical_anchor && point_index == 0 &&
        (point - anchor.head<3>()).cwiseAbs().maxCoeff() <= 1e-6;
    if (!stationary_physical_pose &&
        !objectiveFootprintTerrainSupported(point, footprint_body)) {
      checked.clear();
      if (objective_start_support_failure_.empty() ||
          objective_start_support_failure_ ==
              "connector exceeds bounded distance") {
        objective_start_support_failure_ =
            "connector footprint intersects incompatible known terrain";
      }
      return false;
    }
    // A hanging connector has no mapped terrain with which to justify a
    // gradual elevation change. Any observed floor outside the platform step
    // cap therefore refuses the connector, even when the end-to-end chord
    // would make that change look like a shallow ramp.
    if (std::abs(point.z() - anchor.z()) > stepLimitWithMargin() + 1e-6) {
      checked.clear();
      if (objective_start_support_failure_.empty() ||
          objective_start_support_failure_ == "connector exceeds bounded distance")
        objective_start_support_failure_ = "connector height delta exceeds step cap";
      return false;
    }
    mgg::StateVec pose = mgg::StateVec::Zero();
    pose.head<3>() = point;
    checked.push_back(pose);
  }
  if ((checked.front().head<3>() - anchor.head<3>()).cwiseAbs().maxCoeff() >
          1e-6 ||
      (checked.back().head<3>() - supported.head<3>())
              .cwiseAbs()
              .maxCoeff() > 1e-6) {
    checked.clear();
    if (objective_start_support_failure_.empty() ||
        objective_start_support_failure_ == "connector exceeds bounded distance")
      objective_start_support_failure_ = "projected connector changed an endpoint";
    return false;
  }
  for (std::size_t i = 1; i < checked.size(); ++i) {
    const Eigen::Vector3d from = checked[i - 1].head<3>() + center_offset;
    const Eigen::Vector3d to = checked[i].head<3>() + center_offset;
    if (planning_params_.geofence_checking_enable &&
        (!geofence_ ||
         geofence_->getPathStatus(from.head<2>(), to.head<2>(),
                                  body.head<2>()) ==
             mgg::GeofenceManager::CoordinateStatus::kViolated)) {
      checked.clear();
      return false;
    }
    const mgg::VoxelStatus swept =
        objectiveSweptBodyStatus(from, to, body, tolerate_unknown_body);
    if (swept != mgg::VoxelStatus::kFree) {
      const bool collect_detail =
          objective_start_support_failure_.empty() ||
          objective_start_support_failure_ ==
              "connector exceeds bounded distance";
      if (collect_detail) {
        const double resolution = map_->getResolution();
        const double length = (to - from).norm();
        Eigen::Vector3d first = 0.5 * (from + to);
        if (std::isfinite(resolution) && resolution > 0.0 &&
            std::isfinite(length)) {
          const std::size_t steps = static_cast<std::size_t>(
              std::max(1.0, std::ceil(length / resolution)));
          const Eigen::Vector3d step =
              (to - from) / static_cast<double>(steps);
          const Eigen::Vector3d swept_size = body + step.cwiseAbs();
          for (std::size_t sample = 0; sample < steps; ++sample) {
            const Eigen::Vector3d point =
                from + (static_cast<double>(sample) + 0.5) * step;
            if (objectiveBodyStatus(point, swept_size) !=
                mgg::VoxelStatus::kFree) {
              first = point;
              break;
            }
          }
        }
        char detail[160];
        std::snprintf(
            detail, sizeof(detail),
            "%s swept body %s near (%.2f, %.2f, %.2f)",
            observed_ground_body_evidence_ ? "occupied-only" : "strict",
            swept == mgg::VoxelStatus::kOccupied ? "occupied" : "unknown",
            first.x(), first.y(), first.z());
        objective_start_support_failure_ = detail;
      }
      checked.clear();
      return false;
    }
  }
  return true;
}

mgg::VoxelStatus PlannerNode::objectiveBodyStatus(
    const Eigen::Vector3d& center, const Eigen::Vector3d& body,
    bool tolerate_unknown) const {
  return observed_ground_body_evidence_ || tolerate_unknown
             ? map_->getBoxStatus(center, body, false)
             : map_->getStrictBoxStatus(center, body);
}

mgg::VoxelStatus PlannerNode::objectiveSweptBodyStatus(
    const Eigen::Vector3d& from, const Eigen::Vector3d& to,
    const Eigen::Vector3d& body, bool tolerate_unknown) const {
  if (!observed_ground_body_evidence_ && tolerate_unknown) {
    // graph_expansion's own lattice rule: occupied space blocks the sweep,
    // unobserved space does not.
    return map_->getPathStatus(from, to, body, false);
  }
  if (observed_ground_body_evidence_) {
    const Eigen::Vector3d planning_body = robot_params_.getPlanningSize();
    if (map_backend_ == "mola_snapshot" && provisional_unknown_ground_ &&
        robot_params_.type == mgg::RobotType::kGroundRobot) {
      return map_->getOccupiedOnlyCylinderPathStatus(
          from, to, 0.5 * planning_body.head<2>().norm(), planning_body.z());
    }
    Eigen::Vector3d legacy_body = planning_body;
    const double diagonal = planning_body.head<2>().norm();
    legacy_body.x() = diagonal;
    legacy_body.y() = diagonal;
    return map_->getOccupiedOnlyPathStatus(from, to, legacy_body);
  }
  return map_->getStrictPathStatus(from, to, body);
}

bool PlannerNode::objectiveFootprintTerrainSupported(
    const Eigen::Vector3d& driving_pose,
    const Eigen::Vector3d& body) const {
  return objectiveFootprintTerrainStatus(driving_pose, body) ==
         mgg::GridProjectionStatus::kSupported;
}

mgg::GridProjectionStatus PlannerNode::objectiveFootprintTerrainStatus(
    const Eigen::Vector3d& driving_pose,
    const Eigen::Vector3d& body) const {
  const auto record_failure = [this](const std::string& detail) {
    if (objective_footprint_failure_.empty()) {
      objective_footprint_failure_ = detail;
    }
  };
  // This extra terrain contract is intentionally limited to simulation's
  // observed-ground policy. Explicit objectives and Explore both use it;
  // strict-volume hardware planning retains its existing semantics.
  if (!observed_ground_body_evidence_ ||
      robot_params_.type != mgg::RobotType::kGroundRobot) {
    return mgg::GridProjectionStatus::kSupported;
  }
  if (!driving_pose.allFinite() || !body.allFinite() ||
      (body.array() < 0.0).any() ||
      !std::isfinite(planning_params_.max_ground_height) ||
      !std::isfinite(planning_params_.max_step_height) ||
      planning_params_.max_step_height < 0.0 || !ground_ ||
      !std::isfinite(ground_->max_projection_length) ||
      ground_->max_projection_length <= 0.0) {
    record_failure("invalid footprint query configuration");
    return mgg::GridProjectionStatus::kBodyUnknown;
  }
  const double resolution = map_->getResolution();
  const double radius = 0.5 * body.head<2>().norm();
  if (!std::isfinite(resolution) || resolution <= 0.0 ||
      !std::isfinite(radius)) {
    record_failure("invalid footprint resolution or radius");
    return mgg::GridProjectionStatus::kBodyUnknown;
  }

  // Enumerate the map's own XY cells. MOLA can rotate its component grid
  // relative to navigation coordinates, so the backend returns the complete
  // transformed set rather than only one lattice phase.
  const Eigen::Vector2d body_center =
      driving_pose.head<2>() + robot_params_.center_offset.head<2>();
  constexpr int kMaxFootprintSamples = 4096;
  std::vector<mgg::XYCellCenter> footprint_cells;
  if (!map_->getCircleIntersectingXYCellCenters(
          body_center, radius, kMaxFootprintSamples, footprint_cells)) {
    record_failure("footprint cell grid unavailable or exceeds its bound");
    return mgg::GridProjectionStatus::kBodyUnknown;
  }
  const int samples = static_cast<int>(footprint_cells.size());
  const double nominal_ground_z =
      driving_pose.z() - planning_params_.max_ground_height;
  struct FootprintGroundHit {
    double z;
    std::int64_t grid_x;
    std::int64_t grid_y;
    bool connected;
  };
  std::vector<FootprintGroundHit> ground_hits;
  ground_hits.reserve(footprint_cells.size());
  bool needs_connected_support = false;
  // The platform limit plus the terrain measurement margin; see
  // step_measurement_margin_m_.
  const double footprint_step_limit = stepLimitWithMargin() + 1e-6;
  double first_unsupported_delta = 0.0;

  // getRayStatus(..., false) deliberately treats unobserved cells as
  // traversable and its legacy API also returns Free when ray setup fails.
  // Validate the entire bounded query volume once through the box API first;
  // Unknown here means invalid geometry or an exceeded map work bound.
  const double query_extent = radius + std::sqrt(0.5) * resolution;
  const Eigen::Vector3d query_center(
      body_center.x(), body_center.y(),
      driving_pose.z() - 0.5 * ground_->max_projection_length);
  const Eigen::Vector3d query_size(2.0 * query_extent, 2.0 * query_extent,
                                   ground_->max_projection_length);
  if (map_->getBoxStatus(query_center, query_size, false) ==
      mgg::VoxelStatus::kUnknown) {
    record_failure("footprint query unavailable");
    return mgg::GridProjectionStatus::kBodyUnknown;
  }
  for (std::size_t cell_index = 0; cell_index < footprint_cells.size();
       ++cell_index) {
      const mgg::XYCellCenter& cell = footprint_cells[cell_index];
      const Eigen::Vector2d& cell_center = cell.center;
      Eigen::Vector3d start(cell_center.x(), cell_center.y(), driving_pose.z());
      const Eigen::Vector3d end =
          start - Eigen::Vector3d(0.0, 0.0, ground_->max_projection_length);
      Eigen::Vector3d hit;
      const mgg::VoxelStatus ray =
          map_->getGroundRayStatus(start, end, false, hit);
      // Lateral absence of a hit is neutral: the centre ray already proves
      // support for ordinary poses, while sparse simulated sensors cannot
      // certify every footprint column. Known terrain is a veto when it rises
      // or drops beyond the conservative platform step envelope.
      if (ray == mgg::VoxelStatus::kUnknown) {
        record_failure("footprint ray unavailable");
        return mgg::GridProjectionStatus::kBodyUnknown;
      }
      if (ray != mgg::VoxelStatus::kOccupied) continue;
      if (!hit.allFinite()) {
        record_failure("non-finite footprint ground hit");
        return mgg::GridProjectionStatus::kBodyUnknown;
      }
      const double delta = hit.z() - nominal_ground_z;
      const bool center_compatible =
          std::abs(delta) <= footprint_step_limit;
      if (!center_compatible && !needs_connected_support) {
        first_unsupported_delta = delta;
        needs_connected_support = true;
      }
      ground_hits.push_back(
          {hit.z(), cell.grid_x, cell.grid_y, center_compatible});
  }
  // Preserve the original all-to-centre check as the common fast path.  If a
  // footprint spans more than one individually traversable rise, accept it
  // only when every measured outlier has a chain of measured four-neighbour
  // support back to terrain within the centre's step envelope.  Missing rays
  // never create support or bridge a gap.
  if (needs_connected_support) {
    if (!std::isfinite(planning_params_.max_inclination) ||
        planning_params_.max_inclination < 0.0) {
      record_failure("invalid footprint query configuration");
      return mgg::GridProjectionStatus::kBodyUnknown;
    }
    std::map<std::pair<std::int64_t, std::int64_t>, std::size_t> ground_hit_at;
    for (std::size_t i = 0; i < ground_hits.size(); ++i) {
      ground_hit_at.emplace(
          std::make_pair(ground_hits[i].grid_x, ground_hits[i].grid_y), i);
    }
    std::vector<std::size_t> pending;
    pending.reserve(ground_hits.size());
    for (std::size_t i = 0; i < ground_hits.size(); ++i) {
      if (ground_hits[i].connected) pending.push_back(i);
    }
    constexpr std::int64_t kDx[] = {-1, 1, 0, 0};
    constexpr std::int64_t kDy[] = {0, 0, -1, 1};
    for (std::size_t head = 0; head < pending.size(); ++head) {
      const FootprintGroundHit& current = ground_hits[pending[head]];
      for (int direction = 0; direction < 4; ++direction) {
        const auto found = ground_hit_at.find(std::make_pair(
            current.grid_x + kDx[direction],
            current.grid_y + kDy[direction]));
        if (found == ground_hit_at.end()) continue;
        const std::size_t neighbour_index = found->second;
        if (ground_hits[neighbour_index].connected) continue;
        const double dz =
            std::abs(ground_hits[neighbour_index].z - current.z);
        const double inclination = std::atan2(dz, resolution);
        if (dz > footprint_step_limit &&
            inclination > planning_params_.max_inclination + 1e-6) {
          continue;
        }
        ground_hits[neighbour_index].connected = true;
        pending.push_back(neighbour_index);
      }
    }
    if (std::any_of(ground_hits.begin(), ground_hits.end(),
                    [nominal_ground_z, footprint_step_limit](const FootprintGroundHit& hit) {
                      return !hit.connected &&
                             std::abs(hit.z - nominal_ground_z) >
                                 footprint_step_limit;
                    })) {
      char detail[160];
      std::snprintf(detail, sizeof(detail),
                    "known %s %.3f m exceeds step limit %.3f m plus %.3f m "
                    "measurement tolerance",
                    first_unsupported_delta > 0.0 ? "rise" : "drop",
                    std::abs(first_unsupported_delta),
                    planning_params_.max_step_height,
                    step_measurement_margin_m_);
      record_failure(detail);
      return mgg::GridProjectionStatus::kNoGround;
    }
  }
  return samples > 0 ? mgg::GridProjectionStatus::kSupported
                     : mgg::GridProjectionStatus::kBodyUnknown;
}

bool PlannerNode::objectiveTerrainPathSupported(
    const std::vector<Eigen::Vector3d>& driving_path) const {
  if (!observed_ground_body_evidence_ ||
      robot_params_.type != mgg::RobotType::kGroundRobot) {
    return true;
  }
  if (driving_path.empty()) return false;
  const Eigen::Vector3d footprint_body = robot_params_.getPlanningSize();
  const Eigen::Vector3d physical_anchor =
      physicalAnchorAtDrivingHeight(current_state_).head<3>();
  for (std::size_t i = 0; i < driving_path.size(); ++i) {
    const bool physical_start =
        i == 0 &&
        (driving_path[i].head<2>() - physical_anchor.head<2>())
                .cwiseAbs().maxCoeff() <= 1e-6 &&
        std::abs(driving_path[i].z() - physical_anchor.z()) <=
            planning_params_.max_step_height + map_->getResolution() + 1e-6;
    if (!physical_start &&
        !objectiveFootprintTerrainSupported(driving_path[i], footprint_body)) {
      return false;
    }
    if (i == 0) continue;
    if (std::abs(driving_path[i].z() - driving_path[i - 1].z()) >
        stepLimitWithMargin() + 1e-6) {
      return false;
    }
    const Eigen::Vector3d from =
        driving_path[i - 1] + robot_params_.center_offset;
    const Eigen::Vector3d to =
        driving_path[i] + robot_params_.center_offset;
    if (objectiveSweptBodyStatus(from, to, footprint_body) !=
        mgg::VoxelStatus::kFree) {
      return false;
    }
  }
  return true;
}

bool PlannerNode::retainTerrainSafeExplorationPath(
    const std::vector<mgg::StateVec>& selected_lattice_path,
    const std::function<bool(const std::vector<Eigen::Vector3d>&)>&
        terrain_supported,
    std::vector<mgg::StateVec>& candidate) {
  std::vector<Eigen::Vector3d> driving;
  driving.reserve(candidate.size());
  for (const mgg::StateVec& state : candidate) {
    driving.push_back(state.head<3>());
  }
  if (terrain_supported && terrain_supported(driving)) return true;
  candidate = selected_lattice_path;
  return false;
}

void PlannerNode::updateGlobalGraph() {
  auto map_read = mola_map_ != nullptr ? mola_map_->acquireReadLease()
                                       : mgg::MolaMap::ReadLease{};
  refreshMolaRevision();
  if (!have_odometry_ || !have_initial_state_) return;

  if (global_graph_->getNumVertices() == 0) {
    // A lone root is a landmark, not a traversability claim.  Capture it from
    // the first finite odometry immediately, before the first point cloud or
    // command can move the robot away from home.  Until support is observed no
    // outgoing edge is admitted below.
    mgg::StateVec root_state = initial_state_;
    if (robot_params_.type == mgg::RobotType::kGroundRobot) {
      root_state[2] += planning_params_.max_ground_height -
                       robot_params_.size[2] / 2.0;
    } else {
      initial_anchor_supported_ = true;
    }
    auto* root = new mgg::Vertex(0, root_state);
    root->robot_id = static_cast<int>(planning_params_.robot_id);
    root->type = mgg::VertexType::kVisited;
    root->is_hanging = !initial_anchor_supported_;
    global_graph_->addVertex(root);
    last_own_global_vertex_id_ = 0;
    ++graph_revision_;
    RCLCPP_INFO(get_logger(),
                "global graph captured initial anchor at (%.2f, %.2f, %.2f)%s",
                root_state[0], root_state[1], root_state[2],
                initial_anchor_supported_ ? "" : " (awaiting mapped support)");
  }

  // A failed support or edge query can only change after the map changes.
  // Odometry may continue appending behind that head, but it must not turn one
  // unchanged unknown cell into repeated map work or skip ahead in the queue.
  if (global_backbone_blocked_on_map_ &&
      global_backbone_blocked_map_revision_ == map_revision_) {
    return;
  }
  global_backbone_blocked_on_map_ = false;

  if (!initial_anchor_supported_) {
    auto root_it = global_graph_->vertices_map_.find(0);
    auto* root = root_it == global_graph_->vertices_map_.end()
                     ? nullptr
                     : root_it->second;
    if (root == nullptr) {
      global_backbone_history_lost_ = true;
      global_backbone_history_lost_reason_ =
          "initial graph vertex is unavailable";
      RCLCPP_ERROR(get_logger(), "global trajectory history lost: %s",
                   global_backbone_history_lost_reason_.c_str());
      return;
    }
    mgg::StateVec supported_root = initial_state_;
    if (projectStateToDrivingHeight(supported_root)) {
      if (!global_graph_->updateVertexState(0, supported_root)) {
        global_backbone_history_lost_ = true;
        global_backbone_history_lost_reason_ =
            "initial graph vertex is unavailable";
        RCLCPP_ERROR(get_logger(), "global trajectory history lost: %s",
                     global_backbone_history_lost_reason_.c_str());
        return;
      }
      root->is_hanging = false;
      initial_anchor_supported_ = true;
      ++graph_revision_;
      RCLCPP_INFO(get_logger(),
                  "global graph initial anchor support observed at "
                  "(%.2f, %.2f, %.2f)",
                  supported_root[0], supported_root[1], supported_root[2]);
    } else {
      // The fixed home coordinate must never move to a distant floor hit.
      // Once odometry proves that the robot physically left that coordinate,
      // a bounded connector may attach the retained anchor to the first
      // mapped-supported breadcrumb. The connector still rejects occupied or
      // unknown body volume and measured steps/drops.
      std::size_t selected = pending_global_breadcrumbs_.size();
      mgg::StateVec bridge_end = mgg::StateVec::Zero();
      std::vector<mgg::StateVec> bridge;
      for (std::size_t i = 0; i < pending_global_breadcrumbs_.size(); ++i) {
        mgg::StateVec candidate = pending_global_breadcrumbs_[i].state;
        if (!projectStateToDrivingHeight(candidate)) continue;
        std::vector<mgg::StateVec> checked;
        if (validateObjectiveStartSupport(root->state, candidate, checked)) {
          selected = i;
          bridge_end = candidate;
          bridge = std::move(checked);
          break;
        }
      }
      if (selected == pending_global_breadcrumbs_.size()) {
        global_backbone_blocked_on_map_ = true;
        global_backbone_blocked_map_revision_ = map_revision_;
        return;
      }
      double bridge_length = 0.0;
      for (std::size_t i = 1; i < bridge.size(); ++i) {
        bridge_length +=
            (bridge[i].head<3>() - bridge[i - 1].head<3>()).norm();
      }
      auto* v = new mgg::Vertex(global_graph_->generateVertexID(), bridge_end);
      v->robot_id = static_cast<int>(planning_params_.robot_id);
      v->type = mgg::VertexType::kVisited;
      v->parent = root;
      v->distance = bridge_length;
      root->children.push_back(v);
      global_graph_->addVertex(v);
      global_graph_->addEdge(v, root, bridge_length);
      last_own_global_vertex_id_ = v->id;
      root->is_hanging = false;
      initial_anchor_supported_ = true;
      for (std::size_t i = 0; i <= selected; ++i) {
        pending_global_length_m_ = std::max(
            0.0, pending_global_length_m_ -
                     pending_global_breadcrumbs_.front().path_length);
        pending_global_breadcrumbs_.pop_front();
      }
      ++graph_revision_;
      RCLCPP_INFO(get_logger(),
                  "global graph retained physical home anchor and connected "
                  "it to mapped support at (%.2f, %.2f, %.2f)",
                  bridge_end[0], bridge_end[1], bridge_end[2]);
    }
  }

  const mgg::ExpandContext ctx = makeContext();
  const auto admitEdge = [this, &ctx](const Eigen::Vector3d& from,
                                      const Eigen::Vector3d& to,
                                      double& distance,
                                      const char*& blocked_reason) {
    distance = (to - from).norm();
    blocked_reason = "unknown";
    if (robot_params_.type == mgg::RobotType::kGroundRobot) {
      std::vector<Eigen::Vector3d> projected_edge;
      const auto status = ground_->getProjectedEdgeStatus(
          from, to, ctx.robot_box_size, /*stop_at_unknown_voxel=*/true,
          projected_edge, /*is_hanging=*/false);
      if (status != mgg::ProjectedEdgeStatus::kAdmissible) {
        if (status == mgg::ProjectedEdgeStatus::kSteep) {
          blocked_reason = "steep";
        } else if (status == mgg::ProjectedEdgeStatus::kOccupied) {
          blocked_reason = "occupied";
        } else if (status == mgg::ProjectedEdgeStatus::kHanging) {
          blocked_reason = "no ground";
        }
        return false;
      }
      distance = 0.0;
      for (std::size_t i = 1; i < projected_edge.size(); ++i) {
        distance += (projected_edge[i] - projected_edge[i - 1]).norm();
      }
      return true;
    }
    const auto status = map_->getPathStatus(
        from, to, ctx.robot_box_size, /*stop_at_unknown=*/true);
    if (status == mgg::VoxelStatus::kOccupied) blocked_reason = "occupied";
    return status == mgg::VoxelStatus::kFree;
  };

  std::size_t drained = 0;
  while (!pending_global_breadcrumbs_.empty() &&
         drained < pending_global_drain_max_samples_) {
    const PendingGlobalBreadcrumb& pending = pending_global_breadcrumbs_.front();
    mgg::StateVec state = pending.state;
    if (!projectStateToDrivingHeight(state)) {
      global_backbone_blocked_on_map_ = true;
      global_backbone_blocked_map_revision_ = map_revision_;
      if (!global_backbone_blockage_reported_) {
        RCLCPP_WARN(get_logger(),
                    "global trajectory blocked on no ground at (%.2f, %.2f); "
                    "retaining %zu chronological breadcrumb(s)",
                    state.x(), state.y(), pending_global_breadcrumbs_.size());
        global_backbone_blockage_reported_ = true;
      }
      break;
    }

    auto parent_it =
        global_graph_->vertices_map_.find(last_own_global_vertex_id_);
    mgg::Vertex* parent_vertex =
        parent_it == global_graph_->vertices_map_.end() ? nullptr
                                                       : parent_it->second;
    if (parent_vertex == nullptr ||
        parent_vertex->robot_id != static_cast<int>(planning_params_.robot_id)) {
      global_backbone_history_lost_ = true;
      global_backbone_history_lost_reason_ =
          "last owned graph vertex is unavailable";
      RCLCPP_ERROR(get_logger(), "global trajectory history lost: %s",
                   global_backbone_history_lost_reason_.c_str());
      return;
    }

    const Eigen::Vector3d origin = parent_vertex->state.head<3>();
    const Eigen::Vector3d here = state.head<3>();
    double edge_distance = 0.0;
    const char* blocked_reason = "unknown";
    if (!admitEdge(origin, here, edge_distance, blocked_reason)) {
      global_backbone_blocked_on_map_ = true;
      global_backbone_blocked_map_revision_ = map_revision_;
      if (!global_backbone_blockage_reported_) {
        RCLCPP_WARN(get_logger(),
                    "global trajectory blocked on %s at (%.2f, %.2f); "
                    "retaining %zu chronological breadcrumb(s)",
                    blocked_reason, here.x(), here.y(),
                    pending_global_breadcrumbs_.size());
        global_backbone_blockage_reported_ = true;
      }
      break;
    }

    // Repeated passes should reuse an owned vertex rather than grow the graph
    // forever. The chronological predecessor-to-head transition has already
    // passed above; a separate strict 3D transition check prevents an XY-near
    // vertex across a wall or on another floor from becoming an unchecked
    // alias. Reuse changes no parent, child, or graph edge.
    mgg::Vertex* reused_vertex = nullptr;
    std::vector<mgg::Vertex*> nearby;
    if (global_graph_->getNearestVertices(
            &state, global_vertex_spacing_ * 0.25, &nearby)) {
      double nearest_distance = std::numeric_limits<double>::max();
      for (mgg::Vertex* candidate : nearby) {
        if (candidate == nullptr ||
            candidate->robot_id !=
                static_cast<int>(planning_params_.robot_id)) {
          continue;
        }
        const double candidate_distance =
            (candidate->state.head<3>() - here).norm();
        double reuse_edge_distance = 0.0;
        const char* reuse_blocked_reason = "unknown";
        if (candidate_distance < nearest_distance &&
            admitEdge(here, candidate->state.head<3>(), reuse_edge_distance,
                      reuse_blocked_reason)) {
          nearest_distance = candidate_distance;
          reused_vertex = candidate;
        }
      }
    }

    // A breadcrumb is a place the robot has stood, so its vertex is visited:
    // a frontier the robot has driven onto is no frontier, and addFrontiers
    // reads these vertices as the robot state history upstream kept
    // separately (rrg.cpp:2457). Event E1 itself, which marks everything
    // within 3 m of the robot visited every metre of travel (rrg.cpp:5285),
    // runs in ingestOdometryIntoGlobalGraph.
    if (reused_vertex != nullptr) {
      reused_vertex->type = mgg::VertexType::kVisited;
      last_own_global_vertex_id_ = reused_vertex->id;
    } else {
      auto* v = new mgg::Vertex(global_graph_->generateVertexID(), state);
      v->robot_id = static_cast<int>(planning_params_.robot_id);
      v->type = mgg::VertexType::kVisited;
      v->parent = parent_vertex;
      v->distance = parent_vertex->distance + edge_distance;
      parent_vertex->children.push_back(v);
      global_graph_->addVertex(v);
      global_graph_->addEdge(v, parent_vertex, edge_distance);
      last_own_global_vertex_id_ = v->id;
      ++graph_revision_;
    }
    pending_global_length_m_ =
        std::max(0.0, pending_global_length_m_ - pending.path_length);
    pending_global_breadcrumbs_.pop_front();
    ++drained;
    global_backbone_blockage_reported_ = false;
  }
}


void PlannerNode::onBuildRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  auto map_read = mola_map_ != nullptr ? mola_map_->acquireReadLease()
                                       : mgg::MolaMap::ReadLease{};
  response->message = buildLocalGraph();
  response->success = local_graph_->getNumVertices() > 1;
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void PlannerNode::onPlanRequest(
    const std::shared_ptr<mgg_msgs::srv::PlannerSrv::Request> request,
    std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response) {
  std::unique_lock<std::recursive_mutex> lock(planner_mutex_);
  auto map_read = mola_map_ != nullptr ? mola_map_->acquireReadLease()
                                       : mgg::MolaMap::ReadLease{};
  refreshMolaRevision();
  const std::uint64_t mola_generation_at_start =
      mola_map_ != nullptr ? mola_map_->activeGeneration() : 0;
  // A caller may pin the bound mode for this cycle, e.g. to squeeze through a
  // gap it would normally refuse.
  const mgg::BoundModeType previous = robot_params_.bound_mode;
  robot_params_.bound_mode =
      static_cast<mgg::BoundModeType>(request->bound_mode);
  // Capture receipt freshness and the requested body bound before graph
  // construction and gain evaluation. Those bounded operations hold
  // planner_mutex_, so a current snapshot must not expire merely because its
  // callback cannot refresh the receipt time.
  const IndexedQueryContext query_context = indexedQueryContext();

  best_path_.clear();
  const std::string summary = buildLocalGraph();
  robot_params_.bound_mode = previous;

  mgg::PlanningRequest core;
  core.objective = mgg::ObjectiveKind::kExplore;
  core.component_id = component_id_;
  core.graph_revision = local_graph_revision_;
  core.map_revision = map_revision_;
  if (query_context.mapping_snapshot_fresh_at_capture) {
    const auto& snapshot = query_context.mapping_snapshot;
    core.component_id = snapshot.component_id;
    core.map_epoch = snapshot.epoch;
    core.mapping_graph_revision = snapshot.graph_revision;
    core.geometry_revision = snapshot.geometry_revision;
    core.map_source_stamp_sec = snapshot.source_stamp.sec;
    core.map_source_stamp_nanosec = snapshot.source_stamp.nanosec;
  }
  // The legacy service shares the unified Explore corridor: the gain selector
  // picks the target, the topological stage plans the route to it. It refines
  // with the objective profile, the same budget PlanObjective gives Explore,
  // because the legacy 50 ms profile was never exercised by an Explore
  // corridor before this stage was shared and a full terrain-projected
  // refinement does not fit it. It has no route continuation, so no horizon
  // window is taken here.
  mgg::RouteCorridor corridor;
  corridor.request = core;
  TopologicalRetry explore_retry;
  bool corridor_from_global_graph = false;
  planExploreCorridor(core, summary, corridor, explore_retry,
                      corridor_from_global_graph);
  mgg::FeasiblePath feasible =
      refineCorridor(corridor, &objective_grid_limits_);
  const bool allow_explore_height_refinement =
      map_backend_ == "mola_snapshot" && provisional_unknown_ground_ &&
      observed_ground_body_evidence_;
  map_read.allowPublication();
  lock.unlock();
  if (feasible.status == mgg::PlanningStatus::kSucceeded) {
    queryIndexedMap(feasible, query_context,
                    allow_explore_height_refinement,
                    allow_explore_height_refinement);
  }
  lock.lock();
  map_read.reacquirePublication();
  const bool mola_ready = mola_map_ == nullptr || mola_map_->getStatus();
  refreshMolaRevision();
  if (mola_map_ != nullptr &&
      feasible.status == mgg::PlanningStatus::kSucceeded &&
      (!mola_ready ||
       mola_map_->activeGeneration() != mola_generation_at_start)) {
    feasible.status = mgg::PlanningStatus::kStaleRevision;
    feasible.reason = "MOLA map snapshot changed during planning";
    feasible.poses.clear();
    feasible.partial = false;
    feasible.indexed_map_validated = false;
  }
  const bool indexed_snapshot_missing =
      indexed_map_client_ &&
      (!query_context.have_mapping_snapshot ||
       !query_context.mapping_snapshot_fresh_at_capture);
  const bool indexed_route_rejected =
      indexed_map_client_ && corridor.status == mgg::PlanningStatus::kSucceeded &&
      feasible.status != mgg::PlanningStatus::kSucceeded;
  if (feasible.status != mgg::PlanningStatus::kSucceeded) {
    // A failed local terrain/current-pose refinement is just as terminal as a
    // failed indexed query.  Keeping the selector's graph-height path here
    // bypassed the rejection whenever the optional indexed service was off.
    best_path_.clear();
    // A refusal on evidence, as opposed to a stale or missing map, is a
    // verdict on this leaf: leave it out of the next selections.
    const bool interrupted =
        feasible.reason.find("deadline") != std::string::npos ||
        feasible.reason.find("cancelled") != std::string::npos;
    if (selected_explore_leaf_ && !interrupted &&
        (feasible.status == mgg::PlanningStatus::kBlocked ||
         feasible.status == mgg::PlanningStatus::kUnreachable)) {
      rejected_explore_leaves_.emplace_back(*selected_explore_leaf_,
                                            steadyNowSeconds());
      while (rejected_explore_leaves_.size() > 32u) {
        rejected_explore_leaves_.pop_front();
      }
    }
  }

  if (feasible.status == mgg::PlanningStatus::kSucceeded &&
      !corridor_from_global_graph) {
    // The accepted exploration path joins the global graph (rrg.cpp:4549).
    // A corridor taken from the global graph is already in it.
    addRefPathToGraph(corridor.poses);
  }

  const auto& response_path =
      feasible.status == mgg::PlanningStatus::kSucceeded ? feasible.poses
                                                         : best_path_;

  response->planning_bound_mode = request->bound_mode;
  response->status = !have_odometry_ || !map_->getStatus() ? -1
      : indexed_snapshot_missing || indexed_route_rejected ? -2
      : response_path.empty() ? (local_graph_->getNumVertices() <= 1 ? -2 : -3)
      : mgg_msgs::srv::PlannerSrv::Response::FORWARD;
  for (const mgg::StateVec& s : response_path) {
    response->path.push_back(toPoseMsg(s));
  }

  std::string final_rejection;
  if (feasible.status != mgg::PlanningStatus::kSucceeded &&
      !feasible.reason.empty()) {
    final_rejection = "; final route rejected: " +
                      boundedObjectiveFailure(feasible.reason);
  }
  RCLCPP_INFO(get_logger(), "plan request: %s%s", summary.c_str(),
              final_rejection.c_str());
}

PlannerNode::IndexedQueryContext PlannerNode::indexedQueryContext() const {
  IndexedQueryContext context;
  context.route_start = current_state_;
  context.component_from_navigation = component_from_navigation_;
  context.body = robot_params_.getPlanningSize();
  context.physical_size = robot_params_.size;
  context.center_offset = robot_params_.center_offset;
  context.robot_type = robot_params_.type;
  context.max_step_height = planning_params_.max_step_height;
  context.step_measurement_margin = step_measurement_margin_m_;
  context.max_inclination = planning_params_.max_inclination;
  context.graph_to_base =
      planning_params_.max_ground_height - robot_params_.size.z() / 2.0;
  context.max_provisional_ground_prefix =
      objective_start_support_max_distance_m_;
  context.observed_ground_body_evidence = observed_ground_body_evidence_;
  context.provisional_unknown_ground = provisional_unknown_ground_;
  context.preserve_physical_start_height =
      map_backend_ == "mola_snapshot" && observed_ground_body_evidence_ &&
      provisional_unknown_ground_ &&
      robot_params_.type == mgg::RobotType::kGroundRobot;
  context.have_mapping_snapshot = have_mapping_snapshot_;
  context.mapping_snapshot_fresh_at_capture =
      have_mapping_snapshot_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    mapping_snapshot_received_).count() <=
          indexed_map_snapshot_ttl_s_;
  context.mapping_snapshot = mapping_snapshot_;
  return context;
}

bool PlannerNode::queryIndexedMap(mgg::FeasiblePath& path,
                                  const IndexedQueryContext& context,
                                  bool allow_height_refinement,
                                  bool allow_prefix_truncation,
                                  bool allow_bounded_unknown_tail,
                                  bool allow_continuable_prefix,
                                  bool* truncated_to_validated_prefix,
                                  Eigen::Vector3d* hazard_ahead) {
  path.indexed_map_validated = false;
  if (truncated_to_validated_prefix) *truncated_to_validated_prefix = false;
  if (!indexed_map_client_) return true;
  const auto fail = [&path](mgg::PlanningStatus status,
                            const std::string& reason) {
    path.status = status;
    path.poses.clear();
    path.partial = false;
    path.reason = reason;
    return false;
  };
  const auto query_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(indexed_map_query_timeout_s_);
  const bool qualified_height_refinement_allowed =
      allow_height_refinement &&
      context.observed_ground_body_evidence &&
      context.provisional_unknown_ground &&
      context.robot_type == mgg::RobotType::kGroundRobot;
  const bool prefix_truncation_allowed =
      allow_prefix_truncation && qualified_height_refinement_allowed;
  const bool bounded_unknown_tail_allowed =
      allow_bounded_unknown_tail && qualified_height_refinement_allowed;
  // A committed Navigate/Home objective may outrun the measured floor. Driving
  // the strictly validated part of the section is never weaker than the route
  // the native planner already approved, so this needs no provisional-ground
  // qualification: it only ever emits samples that passed every check below,
  // and only when the rejection was the absence of terrain evidence rather
  // than evidence of a hazard.
  const bool continuable_prefix_allowed =
      allow_continuable_prefix &&
      context.robot_type != mgg::RobotType::kAerialRobot;
  bool height_refinement_available = qualified_height_refinement_allowed;
  static const std::regex kDigest("^[0-9a-fA-F]{64}$");
  if (path.component_id.empty() ||
      !std::regex_match(path.geometry_revision, kDigest) ||
      path.map_source_stamp_sec < 0 ||
      path.map_source_stamp_nanosec >= 1000000000u ||
      (path.map_source_stamp_sec == 0 && path.map_source_stamp_nanosec == 0)) {
    return fail(mgg::PlanningStatus::kStaleRevision,
                "indexed mapping snapshot key or source stamp is invalid");
  }
  if (!context.have_mapping_snapshot ||
      !context.mapping_snapshot_fresh_at_capture) {
    return fail(mgg::PlanningStatus::kStaleRevision,
                "indexed mapping authority is unavailable or expired");
  }
  if (context.mapping_snapshot.component_id != path.component_id ||
      context.mapping_snapshot.epoch != path.map_epoch ||
      context.mapping_snapshot.graph_revision != path.mapping_graph_revision ||
      context.mapping_snapshot.geometry_revision != path.geometry_revision ||
      context.mapping_snapshot.source_stamp.sec != path.map_source_stamp_sec ||
      context.mapping_snapshot.source_stamp.nanosec !=
          path.map_source_stamp_nanosec) {
    return fail(mgg::PlanningStatus::kStaleRevision,
                "indexed mapping authority changed during planning");
  }
  const rclcpp::Time source_time(path.map_source_stamp_sec,
                                 path.map_source_stamp_nanosec,
                                 get_clock()->get_clock_type());
  const double source_age = (now() - source_time).seconds();
  if (!std::isfinite(source_age) || source_age < -0.5 ||
      (indexed_map_max_source_age_s_ > 0.0 &&
       source_age > indexed_map_max_source_age_s_)) {
    return fail(mgg::PlanningStatus::kStaleRevision,
                "indexed mapping source stamp is outside its deadline");
  }
  if (!indexed_map_client_->service_is_ready()) {
    return fail(mgg::PlanningStatus::kBlocked,
                "indexed map query service is unavailable");
  }
  // The key captured at the start of the plan stays the key of every batch:
  // the indexed map server keeps answering a superseded key for a grace
  // period, and a route binds to the geometry it was checked against. Only
  // another component or epoch, or a moved authority transform, makes the
  // captured geometry the wrong one. A revision, geometry revision or source
  // stamp that advanced during the query (a product lands every 2 to 6 s per
  // robot while the fleet drives) is not a frame change; failing on it made
  // exploration crawl on 2026-09-18 (retry backoff to 10 s per plan).
  const auto mapping_authority_is_current = [this, &context]() {
    const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
    const auto& snapshot = context.mapping_snapshot;
    return have_mapping_snapshot_ &&
           mapping_snapshot_.component_id == snapshot.component_id &&
           mapping_snapshot_.epoch == snapshot.epoch &&
           component_from_navigation_.matrix().isApprox(
               context.component_from_navigation.matrix(), 1e-9);
  };
  constexpr std::size_t kMaxIndexedMapSamples = 4096;
  // The least XY progress a validated prefix must make to be emitted as a
  // section; anything shorter is refused with the rejection that cut it.
  constexpr double kMinSectionProgressM = 0.3;

  while (true) {
    if (path.poses.size() >= kMaxIndexedMapSamples) {
      return fail(mgg::PlanningStatus::kBlocked,
                  "indexed map query sample budget exceeded");
    }
    auto request = std::make_shared<mgg_msgs::srv::QueryMapBatch::Request>();
    request->component_id = path.component_id;
    request->epoch = path.map_epoch;
    request->graph_revision = path.mapping_graph_revision;
    request->geometry_revision = path.geometry_revision;
    request->source_stamp.sec = path.map_source_stamp_sec;
    request->source_stamp.nanosec = path.map_source_stamp_nanosec;
    const Eigen::Vector3d body = context.body;
    const Eigen::Vector3d component_body =
        context.component_from_navigation.linear().cwiseAbs() * body;
    const Eigen::Vector3d component_up =
        context.component_from_navigation.linear() * Eigen::Vector3d::UnitZ();
    if (!body.allFinite() || (body.array() <= 0.0).any() ||
        !component_body.allFinite() || (component_body.array() <= 0.0).any() ||
        !context.physical_size.allFinite() ||
        (context.physical_size.array() <= 0.0).any() ||
        !context.center_offset.allFinite() ||
        context.center_offset.head<2>().norm() > 1e-9 ||
        !component_up.allFinite() ||
        !mgg::authorityTiltAcceptable(
            context.component_from_navigation.linear()) ||
        !std::isfinite(indexed_map_ground_tolerance_m_) ||
        !std::isfinite(context.max_step_height) ||
        context.max_step_height < 0.0 ||
        !std::isfinite(context.step_measurement_margin) ||
        context.step_measurement_margin < 0.0 ||
        !std::isfinite(context.max_inclination) ||
        context.max_inclination < 0.0 ||
      !std::isfinite(context.graph_to_base)) {
      return fail(mgg::PlanningStatus::kBlocked,
                  "indexed map body, terrain limits, or gravity alignment is "
                  "invalid");
    }
    // The one number every measured rise or drop is judged against, here and
    // in the indexed authority: the platform limit plus the terrain
    // measurement margin (see step_measurement_margin_m_).
    const double max_step_with_margin =
        context.max_step_height + context.step_measurement_margin;
    request->body_size.x = component_body.x();
    request->body_size.y = component_body.y();
    request->body_size.z = component_body.z();
    request->max_step_m = max_step_with_margin;
    request->max_drop_m = max_step_with_margin;
    request->stop_at_unknown = !context.observed_ground_body_evidence;

    std::vector<mgg::StateVec> route;
    route.reserve(path.poses.size() + 1);
    route.push_back(context.route_start);
    route.insert(route.end(), path.poses.begin(), path.poses.end());
    std::vector<double> expected_ground_z;
    std::vector<mgg::StateVec> dense_base_states;
    for (std::size_t segment = 1; segment < route.size(); ++segment) {
      const mgg::StateVec& a = route[segment - 1];
      const mgg::StateVec& b = route[segment];
      // Qualified Explore terrain is indexed over XY. Keep its sample lattice
      // stable when a bounded terrain correction changes only Z; the adjacent
      // rise/inclination checks and native swept-body recheck still validate
      // that corrected height. Every other caller retains full 3-D spacing.
      const Eigen::Vector3d segment_delta = b.head<3>() - a.head<3>();
      const double distance =
          qualified_height_refinement_allowed
              ? segment_delta.head<2>().norm()
              : segment_delta.norm();
      const double scaled_steps = distance / indexed_map_sample_spacing_m_;
      if (!std::isfinite(scaled_steps) ||
          scaled_steps > static_cast<double>(kMaxIndexedMapSamples)) {
        return fail(mgg::PlanningStatus::kBlocked,
                    "indexed map query sample budget exceeded");
      }
      // Subtracting translated world coordinates can put an exact lattice
      // multiple a few ulps above its integer (for example 100.3 - 100.0).
      // Snap only within the floating-point error implied by those coordinate
      // magnitudes; genuine over-spacing still receives another sample.
      const double coordinate_scale =
          std::max({1.0, a.head<3>().cwiseAbs().maxCoeff(),
                    b.head<3>().cwiseAbs().maxCoeff()}) /
          indexed_map_sample_spacing_m_;
      const double nearest_steps = std::round(scaled_steps);
      const double integer_tolerance =
          16.0 * std::numeric_limits<double>::epsilon() *
          std::max({1.0, std::abs(scaled_steps), coordinate_scale});
      const double stable_scaled_steps =
          std::abs(scaled_steps - nearest_steps) <= integer_tolerance
              ? nearest_steps
              : scaled_steps;
      const std::size_t steps = std::max<std::size_t>(
          1, static_cast<std::size_t>(std::ceil(stable_scaled_steps)));
      const std::size_t first_step = segment == 1 ? 0 : 1;
      const std::size_t additions = steps - first_step + 1;
      if (additions > kMaxIndexedMapSamples ||
          request->samples.size() > kMaxIndexedMapSamples - additions) {
        return fail(mgg::PlanningStatus::kBlocked,
                    "indexed map query sample budget exceeded");
      }
      const double yaw_delta =
          std::atan2(std::sin(b[3] - a[3]), std::cos(b[3] - a[3]));
      for (std::size_t i = first_step; i <= steps; ++i) {
        const double fraction = static_cast<double>(i) / steps;
        mgg::StateVec base = a + fraction * (b - a);
        const double yaw = a[3] + fraction * yaw_delta;
        base[3] = std::atan2(std::sin(yaw), std::cos(yaw));
        Eigen::Vector3d navigation = base.head<3>() + context.center_offset;
        if (context.robot_type == mgg::RobotType::kGroundRobot) {
          navigation.z() += context.graph_to_base;
        }
        const Eigen::Vector3d component =
            context.component_from_navigation * navigation;
        geometry_msgs::msg::Point sample;
        sample.x = component.x();
        sample.y = component.y();
        sample.z = component.z();
        request->samples.push_back(sample);
        dense_base_states.push_back(base);
        if (context.robot_type == mgg::RobotType::kGroundRobot) {
          Eigen::Vector3d contact = base.head<3>();
          contact.x() += context.center_offset.x();
          contact.y() += context.center_offset.y();
          contact.z() -= context.physical_size.z() / 2.0;
          expected_ground_z.push_back(
              (context.component_from_navigation * contact).z());
        }
      }
    }
    if (request->samples.empty()) {
      return fail(mgg::PlanningStatus::kBlocked,
                  "indexed map query has no route samples");
    }
    if (dense_base_states.size() != request->samples.size()) {
      return fail(mgg::PlanningStatus::kBlocked,
                  "indexed map query route sampling is inconsistent");
    }
    std::vector<double> dense_route_distance(request->samples.size(), 0.0);
    std::vector<double> dense_xy_progress(request->samples.size(), 0.0);
    for (std::size_t i = 1; i < request->samples.size(); ++i) {
      const Eigen::Vector3d previous(request->samples[i - 1].x,
                                     request->samples[i - 1].y,
                                     request->samples[i - 1].z);
      const Eigen::Vector3d current(request->samples[i].x,
                                    request->samples[i].y,
                                    request->samples[i].z);
      const Eigen::Vector3d delta = current - previous;
      dense_route_distance[i] = dense_route_distance[i - 1] + delta.norm();
      dense_xy_progress[i] =
          dense_xy_progress[i - 1] + delta.head<2>().norm();
    }

    // The caller releases planner_mutex_ before this bounded wait. Snapshot and
    // odometry callbacks therefore cannot occupy executor threads while blocked
    // behind the planning callback. The exact authority is rechecked below.
    const auto remaining_query_time =
        query_deadline - std::chrono::steady_clock::now();
    if (remaining_query_time <= std::chrono::steady_clock::duration::zero()) {
      return fail(mgg::PlanningStatus::kBlocked, "indexed map query timed out");
    }
    auto future = indexed_map_client_->async_send_request(request);
    if (future.wait_for(remaining_query_time) != std::future_status::ready) {
      indexed_map_client_->remove_pending_request(future);
      return fail(mgg::PlanningStatus::kBlocked, "indexed map query timed out");
    }
    mgg_msgs::srv::QueryMapBatch::Response::SharedPtr response;
    try {
      response = future.get();
    } catch (const std::exception& e) {
      return fail(mgg::PlanningStatus::kBlocked,
                  std::string("indexed map query failed: ") + e.what());
    }
    if (!response) {
      return fail(mgg::PlanningStatus::kBlocked,
                  "indexed map query returned no response");
    }
    if (response->status != mgg_msgs::srv::QueryMapBatch::Response::OK) {
      return fail(
          response->status == mgg_msgs::srv::QueryMapBatch::Response::STALE
              ? mgg::PlanningStatus::kStaleRevision
              : mgg::PlanningStatus::kBlocked,
          response->detail.empty() ? "indexed map query rejected the snapshot"
                                   : response->detail);
    }
    if (response->component_id != request->component_id ||
        response->epoch != request->epoch ||
        response->graph_revision != request->graph_revision ||
        response->geometry_revision != request->geometry_revision) {
      return fail(mgg::PlanningStatus::kStaleRevision,
                  "indexed map query returned a different snapshot");
    }
    const std::size_t count = request->samples.size();
    if (response->occupancy.size() != count || response->ground_z.size() != count ||
        response->roughness.size() != count || response->clearance.size() != count ||
        response->step.size() != count || response->drop.size() != count) {
      return fail(mgg::PlanningStatus::kBlocked,
                  "indexed map query returned malformed result arrays");
    }
    if (context.robot_type == mgg::RobotType::kGroundRobot &&
        expected_ground_z.size() != count) {
      return fail(mgg::PlanningStatus::kBlocked,
                  "indexed map ground does not support the emitted body "
                  "height");
    }
    // The emitted heights follow the robot's odometry, whose height drifts
    // while the mapped ground does not (0.16 m over two hours refused every
    // goal, 2026-09-18). Such a drift is one constant offset between the
    // expected heights and the mapped ground at the robot's own position and
    // along the whole route. Only that uniform case is corrected: the median
    // over the samples ahead is the estimate, and every finite sample,
    // including the one under the robot, must agree with it within the
    // tolerance. A dip or a rise that begins ahead of the robot, or a floor
    // that is odd only under the robot, is not uniform and is still judged
    // below. Offsets within the step limit are left to the bounded height
    // refinement that already handles them.
    if (context.robot_type == mgg::RobotType::kGroundRobot) {
      std::vector<double> ahead;
      std::vector<double> all;
      for (std::size_t i = 0; i < count; ++i) {
        if (std::isfinite(response->ground_z[i]) &&
            std::isfinite(expected_ground_z[i])) {
          const double offset = expected_ground_z[i] - response->ground_z[i];
          all.push_back(offset);
          if (dense_xy_progress[i] > 1e-9) ahead.push_back(offset);
        }
      }
      if (ahead.size() >= 3u) {
        std::nth_element(ahead.begin(), ahead.begin() + ahead.size() / 2,
                         ahead.end());
        const double bias = ahead[ahead.size() / 2];
        const bool uniform = std::all_of(
            all.begin(), all.end(), [&](double offset) {
              return std::abs(offset - bias) <=
                     indexed_map_ground_tolerance_m_ + 1e-9;
            });
        // The plain limit, not the measured-terrain one: below it a uniform
        // offset is absorbed by the ground tolerance band, above it it is
        // odometry error, and a margin here would open a band in between
        // where neither applies.
        if (uniform && std::abs(bias) > context.max_step_height &&
            std::abs(bias) <= odometry_height_error_max_m_) {
          for (double& expected : expected_ground_z) expected -= bias;
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                               "odometry height is %.3f m off the mapped "
                               "ground along the route; expected heights "
                               "corrected",
                               bias);
        }
      }
    }
    bool have_measured_terrain = false;
    bool have_positive_progress_terrain = false;
    bool used_provisional_missing_terrain = false;
    bool provisional_gap_open = false;
    bool have_previous_finite_ground = false;
    double provisional_gap_distance = 0.0;
    double route_xy_progress = 0.0;
    double previous_finite_ground = 0.0;
    Eigen::Vector2d previous_finite_xy = Eigen::Vector2d::Zero();
    std::vector<double> refined_base_z(
        count, std::numeric_limits<double>::quiet_NaN());
    std::optional<std::size_t> prefix_end;
    std::optional<std::size_t> first_refinement_sample;
    // Route length at each sample, every sample that could end a prefix, and
    // the sample being judged: what a hazard needs to keep its standoff.
    std::vector<double> sample_xy_progress(count, 0.0);
    std::vector<std::size_t> prefix_candidates;
    std::size_t examined_sample = 0;
    // Why a route was rejected decides whether its validated prefix may still
    // be driven. Absent terrain evidence ahead (unknown occupancy, an
    // unmeasured interval, an over-long provisional connector) is a horizon
    // that later observations can move. Every other rejection is evidence of a
    // hazard and fails the whole section closed.
    struct Rejection {
      std::string reason;
      bool unmeasured_ahead = false;
    };
    const auto hazard = [](std::string reason) {
      return Rejection{std::move(reason), false};
    };
    const auto unmeasured = [](std::string reason) {
      return Rejection{std::move(reason), true};
    };
    const auto validate_route = [&]() -> std::optional<Rejection> {
      for (std::size_t i = 0; i < count; ++i) {
        if (i > 0) {
          const Eigen::Vector3d previous(request->samples[i - 1].x,
                                         request->samples[i - 1].y,
                                         request->samples[i - 1].z);
          const Eigen::Vector3d current(request->samples[i].x,
                                        request->samples[i].y,
                                        request->samples[i].z);
          const Eigen::Vector3d delta = current - previous;
          provisional_gap_distance += delta.norm();
          route_xy_progress += delta.head<2>().norm();
        }
        sample_xy_progress[i] = route_xy_progress;
        examined_sample = i;
        const bool occupied = response->occupancy[i] ==
                              mgg_msgs::srv::QueryMapBatch::Response::OCCUPIED;
        const bool unknown = response->occupancy[i] ==
                             mgg_msgs::srv::QueryMapBatch::Response::UNKNOWN;
        const bool occupancy_supported =
            response->occupancy[i] ==
                mgg_msgs::srv::QueryMapBatch::Response::FREE ||
            (context.observed_ground_body_evidence && unknown);
        const bool finite_ground = std::isfinite(response->ground_z[i]);
        const bool finite_roughness = std::isfinite(response->roughness[i]);
        const bool paired_missing_terrain =
            std::isnan(response->ground_z[i]) &&
            std::isnan(response->roughness[i]);
        const bool provisional_gap_within_bound =
            std::isfinite(context.max_provisional_ground_prefix) &&
            context.max_provisional_ground_prefix > 0.0 &&
            std::isfinite(provisional_gap_distance) &&
            provisional_gap_distance <=
                context.max_provisional_ground_prefix + 1e-9;
        const bool provisional_missing_terrain =
            context.provisional_unknown_ground && paired_missing_terrain &&
            provisional_gap_within_bound;
        const bool unknown_clearance_allowed =
            context.observed_ground_body_evidence &&
            (unknown || provisional_missing_terrain) &&
            std::isnan(response->clearance[i]);
        const auto unsupported = [i](const char* detail) {
          return std::string(
                     "indexed map terrain or clearance is unsupported at "
                     "sample ") +
                 std::to_string(i) + ": " + detail;
        };
        if (occupied || !occupancy_supported) {
          // Unknown space is a horizon. Occupied space is a wall. The sample
          // and its position make the verdict checkable against the map.
          char reason[192];
          std::snprintf(reason, sizeof(reason),
                        "indexed map route is occupied or unknown: %s at "
                        "sample %zu (%.2f, %.2f, %.2f)",
                        occupied ? "occupied" : "unknown", i,
                        request->samples[i].x, request->samples[i].y,
                        request->samples[i].z);
          return occupied ? hazard(reason) : unmeasured(reason);
        }
        if (context.robot_type != mgg::RobotType::kAerialRobot &&
            !std::isfinite(response->clearance[i]) &&
            !unknown_clearance_allowed) {
          return hazard(unsupported("clearance is unavailable"));
        }
        if (context.robot_type != mgg::RobotType::kAerialRobot &&
            std::isfinite(response->clearance[i]) &&
            response->clearance[i] < component_body.z()) {
          return hazard(unsupported("clearance is below body height"));
        }
        if (context.robot_type != mgg::RobotType::kAerialRobot &&
            (response->step[i] || response->drop[i])) {
          return hazard(unsupported(response->step[i] ? "step reported"
                                                      : "drop reported"));
        }
        if (context.robot_type != mgg::RobotType::kAerialRobot) {
          if (!finite_ground || !finite_roughness) {
            if (!provisional_missing_terrain) {
              if (context.provisional_unknown_ground &&
                  paired_missing_terrain && !provisional_gap_within_bound) {
                return unmeasured(
                    "indexed map provisional terrain connector exceeds its "
                    "bound");
              }
              return unmeasured(unsupported("ground or roughness is "
                                            "unavailable"));
            }
            // Native MOLA edge checks support this bounded connector. Explore
            // still requires later measured support before emitting a prefix;
            // qualified Navigate/Home may retain a bounded unknown tail.
            provisional_gap_open = true;
            used_provisional_missing_terrain = true;
            continue;
          }
          const bool closes_provisional_connector =
              context.provisional_unknown_ground && provisional_gap_open;
          if (closes_provisional_connector &&
              !provisional_gap_within_bound) {
            return unmeasured(
                "indexed map provisional terrain connector exceeds its "
                "bound");
          }
          if (response->roughness[i] > indexed_map_max_roughness_m_) {
            return hazard(unsupported("roughness exceeds its limit"));
          }
          const double ground_mismatch =
              std::abs(expected_ground_z[i] - response->ground_z[i]);
          const Eigen::Vector3d component_ground(
              request->samples[i].x, request->samples[i].y,
              response->ground_z[i]);
          const Eigen::Vector3d navigation_ground =
              context.component_from_navigation.inverse() * component_ground;
          const double fitted_base_z =
              navigation_ground.z() + context.physical_size.z() / 2.0;
          if (!std::isfinite(fitted_base_z)) {
            return hazard("indexed map ground refinement is non-finite");
          }
          // Keep already-compatible poses at the exact height validated by
          // the native planner. Samples that need a bounded correction move
          // only to the nearest edge of the indexed agreement band. Missing
          // intervals remain NaN here and are interpolated between these
          // exact anchors below.
          refined_base_z[i] = dense_base_states[i].z();
          // The samples travel as float32 and the heights are voxel-quantized,
          // so a mismatch of exactly the tolerance arrives as 0.1000000015 and
          // was refused ("mismatch 0.100000 m exceeds 0.100000 m", the Spot,
          // 2026-09-20, twice). A micron is below anything the map measures.
          if (ground_mismatch > indexed_map_ground_tolerance_m_ + 1e-6) {
            const bool physical_start_sample =
                qualified_height_refinement_allowed &&
                context.preserve_physical_start_height &&
                dense_route_distance[i] <= 1e-9 &&
                (dense_base_states[i].head<3>() -
                 context.route_start.head<3>())
                        .cwiseAbs()
                        .maxCoeff() <= 1e-6;
            const bool physical_start_mismatch =
                physical_start_sample &&
                ground_mismatch <= max_step_with_margin + 1e-6;
            const bool bounded_refinement =
                height_refinement_available && dense_xy_progress[i] > 1e-9 &&
                ground_mismatch <= max_step_with_margin + 1e-6;
            if (!physical_start_mismatch && !bounded_refinement) {
              char detail[192];
              std::snprintf(
                  detail, sizeof(detail),
                  "indexed map ground does not support the emitted body "
                  "height at sample %zu: mismatch %.6f m exceeds %.6f m",
                  i, ground_mismatch, indexed_map_ground_tolerance_m_);
              return hazard(std::string(detail));
            }
            if (bounded_refinement) {
              const double agreement_band = indexed_map_ground_tolerance_m_;
              refined_base_z[i] = std::clamp(
                  dense_base_states[i].z(),
                  fitted_base_z - agreement_band,
                  fitted_base_z + agreement_band);
              if (!first_refinement_sample) first_refinement_sample = i;
            }
          }
          const Eigen::Vector2d current_xy(request->samples[i].x,
                                           request->samples[i].y);
          if (have_previous_finite_ground) {
            const double dz =
                std::abs(response->ground_z[i] - previous_finite_ground);
            const double inclination =
                std::atan2(dz, (current_xy - previous_finite_xy).norm());
            const bool incompatible_gap_height =
                closes_provisional_connector &&
                dz > max_step_with_margin + 1e-6;
            const bool incompatible_observed_height =
                !closes_provisional_connector &&
                dz > max_step_with_margin + 1e-6 &&
                inclination > context.max_inclination + 1e-6;
            if (incompatible_gap_height || incompatible_observed_height) {
              return hazard(
                  "indexed map route exceeds platform step or inclination "
                  "limits");
            }
          } else if (closes_provisional_connector &&
                     (expected_ground_z.empty() ||
                      std::abs(response->ground_z[i] -
                               expected_ground_z.front()) >
                          max_step_with_margin + 1e-6)) {
            return hazard(
                "indexed map route exceeds platform step or inclination "
                "limits");
          }
          previous_finite_ground = response->ground_z[i];
          previous_finite_xy = current_xy;
          have_previous_finite_ground = true;
          have_measured_terrain = true;
          if (route_xy_progress > 1e-9) {
            have_positive_progress_terrain = true;
          }
          provisional_gap_open = false;
          provisional_gap_distance = 0.0;
          const double endpoint_displacement =
              (dense_base_states[i].head<2>() -
               context.route_start.head<2>()).norm();
          if (endpoint_displacement + 1e-9 >=
              partial_route_min_progress_m_) {
            prefix_end = i;
            prefix_candidates.push_back(i);
          }
        }
      }
      if (context.robot_type != mgg::RobotType::kAerialRobot) {
        if (provisional_gap_open && !bounded_unknown_tail_allowed) {
          return unmeasured(
              "indexed map route ends without measured terrain support");
        }
        if (!have_measured_terrain && !bounded_unknown_tail_allowed) {
          return unmeasured(
              "indexed map route has no measured terrain support");
        }
        if (used_provisional_missing_terrain &&
            !have_positive_progress_terrain &&
            !bounded_unknown_tail_allowed) {
          return unmeasured(
              "indexed map provisional terrain has no positive-progress "
              "support");
        }
      }
      return std::nullopt;
    };

    const std::optional<Rejection> route_failure = validate_route();
    std::size_t accepted_count = count;
    bool used_validated_prefix = false;
    if (route_failure) {
      // Qualified Explore may shorten after any rejection. A committed
      // Navigate/Home section may shorten only when terrain evidence ran out
      // ahead; its caller then resumes the same objective from this endpoint.
      bool hazard_standoff_prefix = false;
      if (continuable_prefix_allowed && !route_failure->unmeasured_ahead &&
          hazard_prefix_standoff_m_ > 0.0 && examined_sample < count) {
        for (auto candidate = prefix_candidates.rbegin();
             candidate != prefix_candidates.rend(); ++candidate) {
          if (sample_xy_progress[examined_sample] -
                  sample_xy_progress[*candidate] + 1e-9 >=
              hazard_prefix_standoff_m_) {
            prefix_end = *candidate;
            hazard_standoff_prefix = true;
            break;
          }
        }
        if (hazard_standoff_prefix && hazard_ahead != nullptr) {
          *hazard_ahead = dense_base_states[examined_sample].head<3>();
        }
      }
      const bool continuable =
          continuable_prefix_allowed &&
          (route_failure->unmeasured_ahead || hazard_standoff_prefix);
      if ((!prefix_truncation_allowed && !continuable) || !prefix_end ||
          !path.speed_limits.empty()) {
        return fail(mgg::PlanningStatus::kBlocked, route_failure->reason);
      }
      // A prefix that makes no progress is not a section. Emitting it as a
      // partial success sent the robot a route that ended where it stood,
      // the controller completed it at once, the continuation found the
      // robot short of the section goal and replanned, and the fresh plan
      // emitted the same empty prefix: two to three replans a second with
      // the robot standing still (robot_0 and robot_1 on benchbot,
      // 2026-09-18 and again 2026-09-19 after this was reverted with
      // unrelated changes). Refusing with the rejection reason lets the
      // caller route around the marked hazard or report it.
      if (sample_xy_progress[*prefix_end] < kMinSectionProgressM) {
        return fail(mgg::PlanningStatus::kBlocked, route_failure->reason);
      }
      accepted_count = *prefix_end + 1;
      used_validated_prefix = true;
      path.partial = true;
      path.reason.clear();
      if (truncated_to_validated_prefix) {
        *truncated_to_validated_prefix = true;
      }
    }
    const bool needs_height_refinement =
        first_refinement_sample && *first_refinement_sample < accepted_count;
    if (!mapping_authority_is_current()) {
      return fail(mgg::PlanningStatus::kStaleRevision,
                  "indexed mapping authority changed during query");
    }
    if (needs_height_refinement) {
      if (!height_refinement_available || !path.speed_limits.empty()) {
        return fail(mgg::PlanningStatus::kBlocked,
                    "indexed map ground refinement is unavailable");
      }
      std::vector<mgg::StateVec> refined(
          dense_base_states.begin(),
          dense_base_states.begin() + accepted_count);
      for (std::size_t i = 0; i < accepted_count; ++i) {
        if (std::isfinite(refined_base_z[i]) && dense_xy_progress[i] > 1e-9) {
          refined[i].z() = refined_base_z[i];
        }
      }
      for (std::size_t begin = 0; begin < accepted_count;) {
        if (std::isfinite(refined_base_z[begin])) {
          ++begin;
          continue;
        }
        std::size_t end = begin;
        while (end < accepted_count &&
               !std::isfinite(refined_base_z[end])) {
          ++end;
        }
        if (end >= accepted_count) {
          if (!bounded_unknown_tail_allowed || begin == 0) {
            return fail(
                mgg::PlanningStatus::kBlocked,
                "indexed map route ends without measured terrain support");
          }
          // Unknown terrain cannot define a new slope. Carry the last checked
          // driving height through this bounded tail, then require the native
          // swept-body check and repeated pinned indexed query below.
          const double inherited_z = refined[begin - 1].z();
          for (std::size_t i = begin; i < accepted_count; ++i) {
            refined[i].z() = inherited_z;
          }
          break;
        }
        const std::size_t anchor = begin == 0 ? 0 : begin - 1;
        const double span = dense_route_distance[end] -
                            dense_route_distance[anchor];
        const double anchor_correction =
            refined[anchor].z() - dense_base_states[anchor].z();
        const double end_correction =
            refined[end].z() - dense_base_states[end].z();
        for (std::size_t i = begin; i < end; ++i) {
          if (i == 0 || dense_xy_progress[i] <= 1e-9) continue;
          const double fraction =
              span > 1e-12
                  ? std::clamp((dense_route_distance[i] -
                                dense_route_distance[anchor]) /
                                   span,
                               0.0, 1.0)
                  : 0.0;
          refined[i].z() += anchor_correction +
                            fraction * (end_correction - anchor_correction);
        }
        begin = end;
      }
      for (std::size_t i = 1; i < refined.size(); ++i) {
        const double rise = std::abs(refined[i].z() - refined[i - 1].z());
        const double run =
            (refined[i].head<2>() - refined[i - 1].head<2>()).norm();
        const double inclination = std::atan2(rise, run);
        if (rise > max_step_with_margin + 1e-6 &&
            inclination > context.max_inclination + 1e-6) {
          return fail(mgg::PlanningStatus::kBlocked,
                      "indexed map refined route exceeds platform step or "
                      "inclination limits");
        }
      }
      if (refined.size() < 2) {
        return fail(mgg::PlanningStatus::kBlocked,
                    "indexed map ground refinement has no emitted route");
      }
      // Height correction changes the exact 3-D sweep that the native graph
      // approved. Recheck the corrected swept-body envelope against the same
      // pinned native map before asking the indexed authority to validate it
      // a second time. Terrain remains the indexed authority's responsibility
      // on that second pass.
      const double refined_body_radius = 0.5 * context.body.head<2>().norm();
      for (std::size_t i = 1; i < refined.size(); ++i) {
        if (std::chrono::steady_clock::now() > query_deadline) {
          return fail(mgg::PlanningStatus::kBlocked,
                      "indexed map query timed out");
        }
        Eigen::Vector3d from = refined[i - 1].head<3>() + context.center_offset;
        Eigen::Vector3d to = refined[i].head<3>() + context.center_offset;
        from.z() += context.graph_to_base;
        to.z() += context.graph_to_base;
        if (map_->getOccupiedOnlyCylinderPathStatus(
                from, to, refined_body_radius, context.body.z()) !=
            mgg::VoxelStatus::kFree) {
          return fail(mgg::PlanningStatus::kBlocked,
                      "indexed map refined route violates native body "
                      "constraints");
        }
      }
      if (std::chrono::steady_clock::now() > query_deadline) {
        return fail(mgg::PlanningStatus::kBlocked,
                    "indexed map query timed out");
      }
      path.poses.assign(refined.begin() + 1, refined.end());
      height_refinement_available = false;
      continue;
    }
    if (used_validated_prefix) {
      if (accepted_count < 2) {
        return fail(mgg::PlanningStatus::kBlocked,
                    "indexed map validated prefix has no emitted route");
      }
      path.poses.assign(dense_base_states.begin() + 1,
                        dense_base_states.begin() + accepted_count);
    }
    path.indexed_map_validated = true;
    return true;
  }
}

mgg::FeasiblePath PlannerNode::refineCorridor(
    const mgg::RouteCorridor& corridor,
    const mgg::GridRefinementLimits* limits) {
  mgg::FeasiblePath result;
  result.status = corridor.status;
  result.mission_id = corridor.request.mission_id;
  result.component_id = corridor.request.component_id;
  result.graph_revision = corridor.request.graph_revision;
  result.map_revision = corridor.request.map_revision;
  result.map_epoch = corridor.request.map_epoch;
  result.mapping_graph_revision = corridor.request.mapping_graph_revision;
  result.geometry_revision = corridor.request.geometry_revision;
  result.map_source_stamp_sec = corridor.request.map_source_stamp_sec;
  result.map_source_stamp_nanosec = corridor.request.map_source_stamp_nanosec;
  result.partial = corridor.partial;
  result.reason = corridor.reason;
  objective_start_support_failure_.clear();
  objective_footprint_failure_.clear();
  if (corridor.status != mgg::PlanningStatus::kSucceeded) return result;

  result.poses = corridor.poses;

  const Eigen::Vector3d footprint_body = robot_params_.getPlanningSize();
  Eigen::Vector3d body = footprint_body;
  if (observed_ground_body_evidence_ &&
      robot_params_.type == mgg::RobotType::kGroundRobot) {
    const double xy_diagonal = footprint_body.head<2>().norm();
    body.x() = xy_diagonal;
    body.y() = xy_diagonal;
  }
  // Explore shares this stage's structure and every known-hazard veto, but
  // keeps the body-evidence policy its lattice has always used: a frontier is
  // adjacent to unknown space, so requiring a fully observed body volume would
  // stop the robot from ever reaching one. Explicit objectives stay strict.
  const bool tolerate_unknown_body =
      corridor.request.objective == mgg::ObjectiveKind::kExplore;
  // Projection and traversal revisit the same grid poses many times during a
  // refinement. The map is held stable for this RPC, so cache the relatively
  // expensive footprint terrain query for this refinement only. A later RPC
  // always observes newly inserted terrain.
  std::map<std::tuple<double, double, double>, mgg::GridProjectionStatus>
      footprint_terrain_cache;
  const auto footprintTerrainStatus =
      [this, &footprint_body, &footprint_terrain_cache](
          const Eigen::Vector3d& driving_pose) {
        if (!observed_ground_body_evidence_ ||
            robot_params_.type != mgg::RobotType::kGroundRobot) {
          return mgg::GridProjectionStatus::kSupported;
        }
        if (!driving_pose.allFinite()) {
          return mgg::GridProjectionStatus::kBodyUnknown;
        }
        const auto key = std::make_tuple(
            driving_pose.x(), driving_pose.y(), driving_pose.z());
        const auto found = footprint_terrain_cache.find(key);
        if (found != footprint_terrain_cache.end()) return found->second;
        const mgg::GridProjectionStatus status =
            objectiveFootprintTerrainStatus(driving_pose, footprint_body);
        footprint_terrain_cache.emplace(key, status);
        return status;
      };
  const Eigen::Vector3d center_offset = robot_params_.center_offset;
  const mgg::StateVec current_anchor =
      physicalAnchorAtDrivingHeight(current_state_);
  const mgg::Vertex* home_vertex = global_graph_->getVertex(0);
  const bool have_connected_home_anchor =
      initial_anchor_supported_ && home_vertex != nullptr;
  const mgg::StateVec home_anchor =
      have_connected_home_anchor ? home_vertex->state : mgg::StateVec::Zero();
  const double home_association_tolerance =
      std::clamp(2.0 * map_->getResolution(), 0.01, 0.25);
  const double requested_home_delta =
      (corridor.request.goal.pose.head<3>() - initial_state_.head<3>())
          .cwiseAbs()
          .maxCoeff();
  const bool request_targets_home_anchor =
      corridor.request.objective == mgg::ObjectiveKind::kReturnHome &&
      have_connected_home_anchor &&
      (requested_home_delta <= 1e-3 ||
       (!corridor.request.goal.landmark_id.empty() &&
        requested_home_delta <= home_association_tolerance));
  const auto samePosition = [](const mgg::StateVec& a,
                               const mgg::StateVec& b) {
    return (a.head<3>() - b.head<3>()).cwiseAbs().maxCoeff() <= 1e-6;
  };
  const auto isPhysicalCurrent =
      [this, &current_anchor](const mgg::StateVec& state) {
        return (state.head<2>() - current_anchor.head<2>())
                       .cwiseAbs().maxCoeff() <= 1e-6 &&
               (std::abs(state.z() - current_state_.z()) <= 1e-6 ||
                std::abs(state.z() - current_anchor.z()) <=
                    planning_params_.max_step_height + map_->getResolution() +
                        1e-6);
      };
  const auto geofenceContains = [this, &body](const Eigen::Vector3d& center) {
    if (!planning_params_.geofence_checking_enable) return true;
    if (!geofence_) return false;
    return geofence_->getBoxStatus(center.head<2>(), body.head<2>()) !=
           mgg::GeofenceManager::CoordinateStatus::kViolated;
  };
  const auto geofenceAllows = [this, &body](const Eigen::Vector3d& from,
                                            const Eigen::Vector3d& to) {
    if (!planning_params_.geofence_checking_enable) return true;
    if (!geofence_) return false;
    if ((from.head<2>() - to.head<2>()).norm() <= 1e-9) {
      return geofence_->getBoxStatus(from.head<2>(), body.head<2>()) !=
             mgg::GeofenceManager::CoordinateStatus::kViolated;
    }
    return geofence_->getPathStatus(
               from.head<2>(), to.head<2>(), body.head<2>()) !=
           mgg::GeofenceManager::CoordinateStatus::kViolated;
  };
  const auto project = [this, body, footprint_body, center_offset,
                        current_anchor, home_anchor,
                        have_connected_home_anchor, request_targets_home_anchor,
                        tolerate_unknown_body,
                        &corridor, &samePosition, &isPhysicalCurrent,
                        &geofenceContains,
                        &footprintTerrainStatus](mgg::StateVec& state) {
    bool physical_anchor_fallback = false;
    bool provisional_unknown_height = false;
    const bool physical_current = isPhysicalCurrent(state);
    const bool at_physical_current_xy =
        (state.head<2>() - current_anchor.head<2>())
            .cwiseAbs().maxCoeff() <= 1e-6;
    const bool qualified_mola_ground =
        observed_ground_body_evidence_ && provisional_unknown_ground_ &&
        map_backend_ == "mola_snapshot" &&
        robot_params_.type == mgg::RobotType::kGroundRobot;
    if (robot_params_.type == mgg::RobotType::kGroundRobot) {
      const bool exact_navigate_goal =
          corridor.request.objective == mgg::ObjectiveKind::kNavigate &&
          (state.head<2>() - corridor.request.goal.pose.head<2>())
                  .cwiseAbs().maxCoeff() <= 1e-6;
      const bool exact_explicit_goal =
          (corridor.request.objective == mgg::ObjectiveKind::kNavigate ||
           corridor.request.objective == mgg::ObjectiveKind::kReturnHome) &&
          (state.head<2>() - corridor.request.goal.pose.head<2>())
                  .cwiseAbs().maxCoeff() <= 1e-6;
      const bool resolve_exact_navigate_height =
          exact_navigate_goal && !at_physical_current_xy;
      const bool height_projected = ground_ &&
          (qualified_mola_ground && physical_current
               ? (state = current_anchor, true)
               : resolve_exact_navigate_height
               ? resolveNavigateGoalDrivingHeight(state)
               : projectStateToDrivingHeight(state, exact_explicit_goal));
      if (!height_projected) {
        if (provisional_unknown_ground_) {
          // Unknown terrain carries no height evidence. Grid cells inherit
          // their parent's already checked driving plane; resetting every
          // cell to the robot's initial plane creates a false step after a
          // gradual known climb. The physical pose still uses odometry as its
          // anchor when ground is hidden beneath the body.
          if (at_physical_current_xy ||
              corridor.request.objective != mgg::ObjectiveKind::kNavigate) {
            state[2] = current_anchor[2];
          } else {
            provisional_unknown_height = true;
          }
          physical_anchor_fallback = samePosition(state, current_anchor);
        } else {
        // Only two unsupported coordinates carry physical provenance: the
        // robot's current footprint and the retained initial Home anchor after
        // that anchor gained a checked connection. Arbitrary unsupported
        // goals and intermediate grid cells still fail closed.
        bool physical_anchor = false;
        if (samePosition(state, current_state_) ||
            samePosition(state, current_anchor)) {
          state = current_anchor;
          physical_anchor = true;
        } else if (have_connected_home_anchor &&
                   (samePosition(state, initial_state_) ||
                    samePosition(state, home_anchor) ||
                    (request_targets_home_anchor &&
                     samePosition(state, corridor.request.goal.pose)))) {
          state = home_anchor;
        } else {
          return mgg::GridProjectionStatus::kNoGround;
        }
        const Eigen::Vector3d anchor_center =
            state.head<3>() + center_offset;
        // The robot's own pose is never the reason a plan is refused: the
        // outgoing edge checks below decide whether it can leave. The
        // retained Home anchor keeps its occupancy veto.
        if (!physical_anchor &&
            map_->getBoxStatus(anchor_center, body,
                               /*stop_at_unknown_voxel=*/false) ==
                mgg::VoxelStatus::kOccupied) {
          return mgg::GridProjectionStatus::kBodyOccupied;
        }
        if (!geofenceContains(anchor_center)) {
          return mgg::GridProjectionStatus::kGeofenceViolation;
        }
        return mgg::GridProjectionStatus::kSupported;
        }
      }
    }
    Eigen::Vector3d center = state.head<3>() + center_offset;
    // Odometry proves that the robot already occupies this one exact pose.
    // Permit it to seed a route even when a conservative terrain footprint
    // query rejects the stationary pose (a kerb a few millimetres over the
    // step limit under one wheel) or when its body box holds occupied space
    // it is evidently not colliding with. Unknown body volume and geofence
    // checks below, and the complete outgoing edge checks, remain mandatory:
    // they decide whether the robot can leave, not whether it may stand.
    if (!physical_anchor_fallback) {
      const mgg::GridProjectionStatus footprint_status =
          footprintTerrainStatus(state.head<3>());
      if (footprint_status != mgg::GridProjectionStatus::kSupported &&
          !(physical_current &&
            footprint_status == mgg::GridProjectionStatus::kNoGround)) {
        return footprint_status == mgg::GridProjectionStatus::kBodyUnknown
                   ? footprint_status
                   : mgg::GridProjectionStatus::kNoGround;
      }
    }
    const mgg::VoxelStatus body_status =
        qualified_mola_ground
            ? objectiveSweptBodyStatus(center, center, footprint_body)
            : objectiveBodyStatus(center, body, tolerate_unknown_body);
    if (body_status == mgg::VoxelStatus::kOccupied) {
      if (!physical_current) return mgg::GridProjectionStatus::kBodyOccupied;
    } else if (body_status != mgg::VoxelStatus::kFree) {
      return mgg::GridProjectionStatus::kBodyUnknown;
    }
    if (!geofenceContains(center)) {
      return mgg::GridProjectionStatus::kGeofenceViolation;
    }
    return provisional_unknown_height
               ? mgg::GridProjectionStatus::kProvisionalUnknown
               : mgg::GridProjectionStatus::kSupported;
  };
  const auto traverse =
      [this, body, footprint_body, center_offset, current_anchor, home_anchor,
       have_connected_home_anchor, tolerate_unknown_body, &samePosition,
       &isPhysicalCurrent,
       &geofenceAllows,
       &footprintTerrainStatus, &corridor](const mgg::StateVec& a,
                                const mgg::StateVec& b,
                        std::vector<mgg::StateVec>& checked) {
        checked.clear();
        const bool a_physical_current = isPhysicalCurrent(a);
        const bool b_physical_current = isPhysicalCurrent(b);
        const bool a_provenance =
            samePosition(a, current_anchor) ||
            (have_connected_home_anchor && samePosition(a, home_anchor));
        const bool b_provenance =
            samePosition(b, current_anchor) ||
            (have_connected_home_anchor && samePosition(b, home_anchor));
        mgg::StateVec projected_a = a;
        mgg::StateVec projected_b = b;
        const bool a_anchor =
            !provisional_unknown_ground_ && a_provenance &&
            !projectStateToDrivingHeight(projected_a);
        const bool b_anchor =
            !provisional_unknown_ground_ && b_provenance &&
            !projectStateToDrivingHeight(projected_b);
        if (robot_params_.type == mgg::RobotType::kGroundRobot &&
            (a_anchor || b_anchor)) {
          if (a_anchor && b_anchor) {
            if (!samePosition(a, b)) return false;
            const Eigen::Vector3d center = a.head<3>() + center_offset;
            if (!geofenceAllows(center, center) ||
                map_->getBoxStatus(center, body,
                                   /*stop_at_unknown_voxel=*/false) ==
                    mgg::VoxelStatus::kOccupied) {
              return false;
            }
            checked = {a, b};
            return true;
          }
          const mgg::StateVec& anchor = a_anchor ? a : b;
          const mgg::StateVec& mapped = a_anchor ? b : a;
          const bool anchor_is_physical_current =
              a_anchor ? a_physical_current : b_physical_current;
          mgg::StateVec projected_mapped = mapped;
          if (!projectStateToDrivingHeight(projected_mapped) ||
              !samePosition(projected_mapped, mapped) ||
              !validateObjectiveStartSupport(anchor, mapped, checked,
                                             tolerate_unknown_body,
                                             anchor_is_physical_current)) {
            checked.clear();
            return false;
          }
          if (b_anchor) std::reverse(checked.begin(), checked.end());
          return true;
        }
        const Eigen::Vector3d body_from = a.head<3>() + center_offset;
        const Eigen::Vector3d body_to = b.head<3>() + center_offset;
        if (!geofenceAllows(body_from, body_to)) return false;
        if (robot_params_.type == mgg::RobotType::kAerialRobot) {
          if (map_->getPathStatus(body_from, body_to, body,
                                  !tolerate_unknown_body) !=
              mgg::VoxelStatus::kFree) {
            return false;
          }
          checked = {a, b};
          return true;
        }
        if (!ground_) return false;
        std::vector<Eigen::Vector3d> projected;
        // GroundProjection owns the planner/driving-height convention.  The
        // robot center offset is applied only to the additional collision
        // sweep below; passing an offset pose to GroundProjection would shift
        // the returned driving pose by -center_offset.z.
        const bool qualified_mola_ground =
            observed_ground_body_evidence_ && provisional_unknown_ground_ &&
            map_backend_ == "mola_snapshot";
        if (ground_->getProjectedEdgeStatus(
                a.head<3>(), b.head<3>(),
                qualified_mola_ground ? footprint_body : body,
                /*stop_at_unknown_voxel=*/!observed_ground_body_evidence_ &&
                    !tolerate_unknown_body,
                projected, provisional_unknown_ground_,
                qualified_mola_ground && a_physical_current) !=
                mgg::ProjectedEdgeStatus::kAdmissible ||
            projected.size() < 2) {
          return false;
        }
        if (qualified_mola_ground) {
          if (a_physical_current) projected.front() = a.head<3>();
          if (b_physical_current) projected.back() = b.head<3>();
        }
        if (provisional_unknown_ground_) {
          double inherited_z = a.z();
          for (std::size_t projected_index = 0;
               projected_index < projected.size(); ++projected_index) {
            Eigen::Vector3d& driving_pose = projected[projected_index];
            const bool preserved_physical_endpoint =
                qualified_mola_ground &&
                ((projected_index == 0 && a_physical_current) ||
                 (projected_index + 1 == projected.size() &&
                  b_physical_current));
            if (preserved_physical_endpoint) {
              driving_pose = projected_index == 0 ? a.head<3>() : b.head<3>();
              inherited_z = driving_pose.z();
              continue;
            }
            mgg::StateVec supported = mgg::StateVec::Zero();
            supported.head<3>() = driving_pose;
            if (projectStateToDrivingHeight(supported, true)) {
              driving_pose = supported.head<3>();
            } else if (corridor.request.objective !=
                       mgg::ObjectiveKind::kNavigate) {
              driving_pose.z() = current_anchor.z();
            } else {
              driving_pose.z() = inherited_z;
            }
            inherited_z = driving_pose.z();
          }
        }
        checked.reserve(projected.size());
        for (std::size_t projected_index = 0;
             projected_index < projected.size(); ++projected_index) {
          const Eigen::Vector3d& driving_pose = projected[projected_index];
          const bool physical_endpoint =
              (projected_index == 0 && a_physical_current) ||
              (projected_index + 1 == projected.size() && b_physical_current);
          const mgg::GridProjectionStatus footprint_status =
              footprintTerrainStatus(driving_pose);
          if (footprint_status != mgg::GridProjectionStatus::kSupported &&
              !(physical_endpoint &&
                footprint_status == mgg::GridProjectionStatus::kNoGround)) {
            checked.clear();
            return false;
          }
          // The legacy edge rule permits a rise above the step cap whenever
          // its average angle is below max_inclination. In observed-ground
          // objective mode that can classify a 15 cm kerb sampled over one
          // 30 cm map interval as a ramp. Keep an explicit per-sample height
          // bound; this is intentionally conservative for ramps whose rise
          // over one map sample exceeds the platform's step capability.
          if (observed_ground_body_evidence_ && !checked.empty() &&
              std::abs(driving_pose.z() - checked.back().z()) >
                  stepLimitWithMargin() + 1e-6) {
            checked.clear();
            return false;
          }
          mgg::StateVec pose = mgg::StateVec::Zero();
          pose.head<3>() = driving_pose;
          checked.push_back(pose);
        }
        if ((checked.front().head<3>() - a.head<3>()).cwiseAbs().maxCoeff() >
                1e-6 ||
            (checked.back().head<3>() - b.head<3>()).cwiseAbs().maxCoeff() >
                1e-6) {
          return false;
        }
        for (std::size_t i = 1; i < checked.size(); ++i) {
          const Eigen::Vector3d checked_from =
              checked[i - 1].head<3>() + center_offset;
          const Eigen::Vector3d checked_to =
              checked[i].head<3>() + center_offset;
          if (!geofenceAllows(checked_from, checked_to) ||
              objectiveSweptBodyStatus(checked_from, checked_to, body,
                                       tolerate_unknown_body) !=
                  mgg::VoxelStatus::kFree) {
            return false;
          }
        }
        return true;
      };
  mgg::GridRefinementLimits widened_objective_limits;
  const mgg::GridRefinementLimits* active_limits =
      limits != nullptr ? limits : &grid_refinement_limits_;
  if (limits != nullptr &&
      (corridor.request.objective == mgg::ObjectiveKind::kNavigate ||
       corridor.request.objective == mgg::ObjectiveKind::kReturnHome ||
       corridor.request.objective == mgg::ObjectiveKind::kExplore)) {
    widened_objective_limits = *limits;
    widened_objective_limits.detour_margin_m = widestObjectiveGridMargin(
        corridor, current_state_, *limits, objective_grid_max_margin_m_);
    active_limits = &widened_objective_limits;
  }
  mgg::BoundedGridPlanner grid_planner(current_state_, *active_limits, project,
                                       traverse);
  result = grid_planner.refine(corridor);
  if (result.status != mgg::PlanningStatus::kSucceeded &&
      !objective_start_support_failure_.empty()) {
    result.reason += " [start connector: " +
                     objective_start_support_failure_ + "]";
  }
  if (result.status != mgg::PlanningStatus::kSucceeded &&
      !objective_footprint_failure_.empty()) {
    result.reason += " [first footprint rejection: " +
                     objective_footprint_failure_ + "]";
  }
  if (result.status != mgg::PlanningStatus::kSucceeded) return result;
  convertPathToNavigationBase(result);
  return result;
}

void PlannerNode::convertPathToNavigationBase(mgg::FeasiblePath& path) const {
  if (robot_params_.type != mgg::RobotType::kGroundRobot) return;
  const double graph_to_base =
      planning_params_.max_ground_height - robot_params_.size[2] / 2.0;
  for (auto& pose : path.poses) pose[2] -= graph_to_base;
}

void PlannerNode::onValidateObjectiveRoute(
    const std::shared_ptr<mgg_msgs::srv::ValidateObjectiveRoute::Request>& request,
    std::shared_ptr<mgg_msgs::srv::ValidateObjectiveRoute::Response> response) {
  using Response = mgg_msgs::srv::ValidateObjectiveRoute::Response;
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  auto map_read = mola_map_ != nullptr ? mola_map_->acquireReadLease()
                                       : mgg::MolaMap::ReadLease{};
  refreshMolaRevision();
  response->map_revision = map_revision_;
  const auto validation_deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  const auto unavailable = [&response](const std::string& reason) {
    response->status = Response::UNAVAILABLE;
    response->reason = reason;
  };
  const auto invalid = [&response](const std::string& reason) {
    response->status = Response::INVALID;
    response->reason = reason;
  };
  // A controller reports an execution-blocking hazard by submitting its
  // remaining route here. An INVALID verdict therefore also marks the cached
  // topological corridor nearest the hazard, so the next objective request
  // selects a different corridor rather than the one the robot cannot follow.
  const auto invalid_at = [this, &invalid](const std::string& reason,
                                          const Eigen::Vector3d& hazard) {
    invalid(reason);
    blockCachedRouteNear(hazard);
  };
  if (!provisional_unknown_ground_ ||
      (map_backend_ != "cloud_octomap" && map_backend_ != "mola_snapshot") ||
      !map_ || !map_->getStatus() || !have_odometry_) {
    unavailable("provisional route validation is unavailable");
    return;
  }
  const std::string& active_component =
      have_mapping_snapshot_ ? mapping_snapshot_.component_id : component_id_;
  if (mission_id_.empty() || request->mission_id != mission_id_ ||
      active_component.empty() || request->component_id != active_component ||
      request->frame_id != world_frame_) {
    unavailable("route authority does not match this planner");
    return;
  }
  if (request->path.size() < 2 || request->path.size() > 2048 ||
      !std::isfinite(request->lookahead_m) || request->lookahead_m < 0.5 ||
      request->lookahead_m > 10.0) {
    unavailable("route or lookahead is outside validation bounds");
    return;
  }

  std::vector<mgg::StateVec> route;
  route.reserve(request->path.size());
  const double graph_to_base =
      planning_params_.max_ground_height - robot_params_.size.z() / 2.0;
  for (const auto& pose : request->path) {
    mgg::StateVec state = fromPoseMsg(pose);
    if (!state.allFinite()) {
      unavailable("route contains a non-finite pose");
      return;
    }
    state.z() += graph_to_base;
    route.push_back(state);
  }

  std::size_t closest_segment = 0;
  double closest_t = 0.0;
  double closest_distance = std::numeric_limits<double>::infinity();
  double closest_s = 0.0;
  std::vector<double> route_s(route.size(), 0.0);
  for (std::size_t i = 1; i < route.size(); ++i) {
    route_s[i] = route_s[i - 1] +
                 (route[i].head<2>() - route[i - 1].head<2>()).norm();
  }
  const Eigen::Vector2d current = current_state_.head<2>();
  for (std::size_t i = 1; i < route.size(); ++i) {
    const Eigen::Vector2d a = route[i - 1].head<2>();
    const Eigen::Vector2d delta = route[i].head<2>() - a;
    const double denom = delta.squaredNorm();
    const double t = denom > 1e-12
                         ? std::clamp((current - a).dot(delta) / denom, 0.0, 1.0)
                         : 0.0;
    const double distance = (current - (a + t * delta)).norm();
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_segment = i - 1;
      closest_t = t;
      closest_s = route_s[i - 1] + t * std::sqrt(denom);
    }
  }
  const double association_limit =
      std::clamp(robot_params_.getPlanningSize().head<2>().norm(), 0.5, 2.0);
  if (!std::isfinite(closest_distance) || closest_distance > association_limit) {
    unavailable("robot is not associated with the submitted route");
    return;
  }
  const double ambiguity_tolerance =
      std::max(0.10, map_->getResolution());
  for (std::size_t i = 1; i < route.size(); ++i) {
    const Eigen::Vector2d a = route[i - 1].head<2>();
    const Eigen::Vector2d delta = route[i].head<2>() - a;
    const double denom = delta.squaredNorm();
    const double t = denom > 1e-12
                         ? std::clamp((current - a).dot(delta) / denom, 0.0, 1.0)
                         : 0.0;
    const double candidate_s = route_s[i - 1] + t * std::sqrt(denom);
    if (std::abs(candidate_s - closest_s) <= association_limit) continue;
    if ((current - (a + t * delta)).norm() <=
        closest_distance + ambiguity_tolerance) {
      unavailable("robot progress is ambiguous on the submitted route");
      return;
    }
  }

  std::vector<mgg::StateVec> ahead;
  ahead.push_back(route[closest_segment] +
                  closest_t * (route[closest_segment + 1] -
                               route[closest_segment]));
  double remaining = request->lookahead_m;
  for (std::size_t i = closest_segment + 1;
       i < route.size() && remaining > 1e-9; ++i) {
    const double length =
        (route[i].head<3>() - ahead.back().head<3>()).norm();
    if (!std::isfinite(length)) {
      unavailable("route segment length is invalid");
      return;
    }
    if (length <= remaining + 1e-9) {
      if (length > 1e-9) ahead.push_back(route[i]);
      remaining -= length;
    } else if (length > 1e-12) {
      mgg::StateVec boundary = ahead.back();
      boundary.head<3>() +=
          (remaining / length) * (route[i].head<3>() - ahead.back().head<3>());
      ahead.push_back(boundary);
      remaining = 0.0;
    }
    if (ahead.size() > 128 ||
        std::chrono::steady_clock::now() > validation_deadline) {
      unavailable("remaining route validation exceeded its work bound");
      return;
    }
  }

  const Eigen::Vector3d footprint = robot_params_.getPlanningSize();
  Eigen::Vector3d body = footprint;
  const double diagonal = footprint.head<2>().norm();
  body.x() = diagonal;
  body.y() = diagonal;
  const Eigen::Vector3d center_offset = robot_params_.center_offset;
  const bool qualified_mola_body_envelope =
      map_backend_ == "mola_snapshot" && observed_ground_body_evidence_ &&
      provisional_unknown_ground_ &&
      robot_params_.type == mgg::RobotType::kGroundRobot;
  const auto point_body_status =
      [this, &body, qualified_mola_body_envelope](
          const Eigen::Vector3d& center) {
        return qualified_mola_body_envelope
                   ? objectiveSweptBodyStatus(center, center, body)
                   : objectiveBodyStatus(center, body);
      };
  const mgg::StateVec physical_anchor =
      physicalAnchorAtDrivingHeight(current_state_);
  const auto isPhysicalCurrent = [this, &physical_anchor](
                                     const Eigen::Vector3d& pose) {
    return (pose.head<2>() - physical_anchor.head<2>())
                   .cwiseAbs().maxCoeff() <= 1e-6 &&
           (std::abs(pose.z() - current_state_.z()) <= 1e-6 ||
            std::abs(pose.z() - physical_anchor.z()) <=
                planning_params_.max_step_height + map_->getResolution() +
                    1e-6);
  };
  if (ahead.size() < 2) {
    const Eigen::Vector3d pose = ahead.front().head<3>();
    const bool physical_current = isPhysicalCurrent(pose);
    const auto footprint_status =
        objectiveFootprintTerrainStatus(pose, footprint);
    const mgg::VoxelStatus box = point_body_status(pose + center_offset);
    const bool geofence_invalid =
        planning_params_.geofence_checking_enable &&
        (!geofence_ ||
         geofence_->getBoxStatus((pose + center_offset).head<2>(),
                                 body.head<2>()) ==
             mgg::GeofenceManager::CoordinateStatus::kViolated);
    if (box == mgg::VoxelStatus::kOccupied || geofence_invalid ||
        (footprint_status == mgg::GridProjectionStatus::kNoGround &&
         !physical_current)) {
      invalid_at("stationary route intersects a known hazard", pose);
    } else if (footprint_status == mgg::GridProjectionStatus::kBodyUnknown ||
               box == mgg::VoxelStatus::kUnknown) {
      unavailable("stationary route query was unavailable");
    } else {
      response->status = Response::VALID;
      response->reason = "route has no remaining validation segment";
    }
    return;
  }
  std::size_t exact_height_samples = 0;
  for (std::size_t segment = 1; segment < ahead.size(); ++segment) {
    std::vector<Eigen::Vector3d> projected;
    mgg::ProjectedEdgeStatus terrain = mgg::ProjectedEdgeStatus::kUnknown;
    if (qualified_mola_body_envelope) {
      // This route already passed the indexed terrain authority at these
      // exact heights. Densify it for current-map footprint and capsule checks
      // without borrowing a lateral native floor ray that would undo the
      // bounded indexed correction.
      const Eigen::Vector3d start = ahead[segment - 1].head<3>();
      const Eigen::Vector3d end = ahead[segment].head<3>();
      const Eigen::Vector3d delta = end - start;
      const double length = delta.norm();
      const double resolution = map_->getResolution();
      const double inclination =
          std::atan2(std::abs(delta.z()), delta.head<2>().norm());
      if (!std::isfinite(length) || !std::isfinite(resolution) ||
          resolution <= 0.0 ||
          (std::abs(delta.z()) > stepLimitWithMargin() + 1e-6 &&
           inclination > planning_params_.max_inclination + 1e-6)) {
        terrain = mgg::ProjectedEdgeStatus::kSteep;
      } else {
        constexpr std::size_t kMaxExactHeightSamples = 4096;
        const double scaled_steps = length / (2.0 * resolution);
        if (!std::isfinite(scaled_steps) ||
            scaled_steps > double(kMaxExactHeightSamples)) {
          unavailable("remaining route validation exceeded its work bound");
          return;
        }
        const std::size_t steps = std::max<std::size_t>(
            1, static_cast<std::size_t>(std::ceil(scaled_steps)));
        if (steps + 1 > kMaxExactHeightSamples - exact_height_samples) {
          unavailable("remaining route validation exceeded its work bound");
          return;
        }
        projected.reserve(steps + 1);
        for (std::size_t i = 0; i <= steps; ++i) {
          projected.push_back(start +
                              (double(i) / double(steps)) * delta);
        }
        exact_height_samples += projected.size();
        terrain = mgg::ProjectedEdgeStatus::kAdmissible;
      }
    } else {
      terrain = ground_->getProjectedEdgeStatus(
          ahead[segment - 1].head<3>(), ahead[segment].head<3>(), body, false,
          projected, true);
    }
    if (terrain == mgg::ProjectedEdgeStatus::kUnknown) {
      unavailable("map query was unavailable");
      return;
    }
    if (terrain != mgg::ProjectedEdgeStatus::kAdmissible || projected.size() < 2) {
      invalid_at("remaining route intersects known terrain",
                 ahead[segment].head<3>());
      return;
    }
    if (provisional_unknown_ground_ && !qualified_mola_body_envelope) {
      double inherited_z = ahead[segment - 1].z();
      for (Eigen::Vector3d& pose : projected) {
        mgg::StateVec supported = mgg::StateVec::Zero();
        supported.head<3>() = pose;
        if (projectStateToDrivingHeight(supported, true)) {
          pose = supported.head<3>();
        } else {
          pose.z() = inherited_z;
        }
        inherited_z = pose.z();
      }
    }
    Eigen::Vector3d previous;
    bool have_previous = false;
    for (std::size_t projected_index = 0;
         projected_index < projected.size(); ++projected_index) {
      if (std::chrono::steady_clock::now() > validation_deadline) {
        unavailable("remaining route validation exceeded its time bound");
        return;
      }
      const Eigen::Vector3d& pose = projected[projected_index];
      const bool physical_current =
          segment == 1 && projected_index == 0 && isPhysicalCurrent(pose);
      const Eigen::Vector3d center = pose + center_offset;
      const mgg::VoxelStatus box = point_body_status(center);
      if (box == mgg::VoxelStatus::kOccupied) {
        invalid_at("remaining route intersects known occupied space", pose);
        return;
      }
      const auto footprint_status =
          objectiveFootprintTerrainStatus(pose, footprint);
      if (footprint_status == mgg::GridProjectionStatus::kNoGround &&
          !physical_current) {
        invalid_at("remaining route footprint intersects known terrain",
                   pose);
        return;
      }
      if (box == mgg::VoxelStatus::kUnknown ||
          footprint_status == mgg::GridProjectionStatus::kBodyUnknown) {
        unavailable("remaining route map query was unavailable");
        return;
      }
      if (planning_params_.geofence_checking_enable &&
          (!geofence_ ||
           geofence_->getBoxStatus(center.head<2>(), body.head<2>()) ==
               mgg::GeofenceManager::CoordinateStatus::kViolated)) {
        invalid_at("remaining route violates the geofence", pose);
        return;
      }
      if (have_previous &&
          std::abs(pose.z() - previous.z()) > stepLimitWithMargin() + 1e-6) {
        invalid_at("remaining route exceeds the platform step limit", pose);
        return;
      }
      previous = pose;
      have_previous = true;
    }
    for (std::size_t i = 1; i < projected.size(); ++i) {
      const Eigen::Vector3d from = projected[i - 1] + center_offset;
      const Eigen::Vector3d to = projected[i] + center_offset;
      if (planning_params_.geofence_checking_enable &&
          (!geofence_ ||
           geofence_->getPathStatus(from.head<2>(), to.head<2>(),
                                    body.head<2>()) ==
               mgg::GeofenceManager::CoordinateStatus::kViolated)) {
        invalid_at("remaining route crosses the geofence", from);
        return;
      }
      const mgg::VoxelStatus swept = objectiveSweptBodyStatus(from, to, body);
      if (swept == mgg::VoxelStatus::kUnknown) {
        unavailable("remaining route sweep query was unavailable");
        return;
      }
      if (swept == mgg::VoxelStatus::kOccupied) {
        invalid_at("remaining route sweep intersects known occupied space",
                   from);
        return;
      }
      if (std::chrono::steady_clock::now() > validation_deadline) {
        unavailable("remaining route validation exceeded its time bound");
        return;
      }
    }
  }
  response->status = Response::VALID;
  response->reason = "remaining route has no known hazard";
}

PlannerNode::ObjectiveIndexedFlags PlannerNode::objectiveIndexedQueryFlags(
    mgg::ObjectiveKind objective) const {
  // Joining the shared corridor stage must not move Explore's indexed-query
  // policy: it remains the only objective allowed to emit a shorter validated
  // prefix, and the explicit objectives remain the only ones allowed a bounded
  // unknown tail.
  const bool qualified_mola_ground =
      map_backend_ == "mola_snapshot" && provisional_unknown_ground_ &&
      observed_ground_body_evidence_ &&
      robot_params_.type == mgg::RobotType::kGroundRobot;
  ObjectiveIndexedFlags flags;
  flags.prefix_truncation =
      qualified_mola_ground && objective == mgg::ObjectiveKind::kExplore;
  flags.bounded_unknown_tail =
      qualified_mola_ground && (objective == mgg::ObjectiveKind::kNavigate ||
                                objective == mgg::ObjectiveKind::kReturnHome);
  flags.height_refinement =
      flags.prefix_truncation || flags.bounded_unknown_tail;
  return flags;
}

double PlannerNode::steadyNowSeconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

mgg::BlockedCorridorView PlannerNode::blockedCorridorView() const {
  mgg::BlockedCorridorView view;
  view.registry = &blocked_corridors_;
  view.map_revision = map_revision_;
  view.now_s = steadyNowSeconds();
  return view;
}

void PlannerNode::blockCorridorSegment(const mgg::StateVec& from,
                                       const mgg::StateVec& to) {
  blocked_corridors_.block(from, to, map_revision_, steadyNowSeconds());
}

void PlannerNode::blockCachedRouteNear(const Eigen::Vector3d& hazard) {
  if (!cached_objective_route_) return;
  blockRouteNear(cached_objective_route_->global_poses, hazard);
}

void PlannerNode::blockRouteNear(const std::vector<mgg::StateVec>& route,
                                 const Eigen::Vector3d& hazard) {
  if (!hazard.allFinite() || route.size() < 2) return;
  std::size_t best = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < route.size(); ++i) {
    const Eigen::Vector2d a = route[i - 1].head<2>();
    const Eigen::Vector2d delta = route[i].head<2>() - a;
    const double denom = delta.squaredNorm();
    const double t = denom > 1e-12
                         ? std::clamp((hazard.head<2>() - a).dot(delta) / denom,
                                      0.0, 1.0)
                         : 0.0;
    const double distance = (hazard.head<2>() - (a + t * delta)).norm();
    if (distance < best_distance) {
      best_distance = distance;
      best = i;
    }
  }
  if (!std::isfinite(best_distance)) return;
  blockCorridorSegment(route[best - 1], route[best]);
}

bool PlannerNode::sliceObjectiveRouteWindow(
    const mgg::PlanningRequest& core, mgg::RouteCorridor& corridor,
    std::vector<mgg::StateVec>& global_objective_path,
    std::unique_ptr<CachedObjectiveRoute>& pending_objective_route) {
  global_objective_path.clear();
  pending_objective_route.reset();
  if (corridor.status != mgg::PlanningStatus::kSucceeded ||
      corridor.poses.empty()) {
    return false;
  }
  if (corridor.poses.size() > objective_route_max_poses_) {
    corridor.status = mgg::PlanningStatus::kBlocked;
    corridor.reason = "persistent objective route exceeds the bounded pose limit";
    corridor.poses.clear();
    return false;
  }
  global_objective_path = corridor.poses;
  global_objective_path.back()[3] = core.goal.pose[3];
  mgg::RouteCorridor local = corridor;
  local.poses.clear();
  double remaining = objective_route_horizon_m_;
  mgg::StateVec previous = current_state_;
  std::size_t next_index = 0;
  while (next_index < corridor.poses.size()) {
    const mgg::StateVec& target = corridor.poses[next_index];
    const double distance = (target.head<2>() - previous.head<2>()).norm();
    if (distance > remaining + 1e-9) {
      // The section's proxy is an interpolated point of the route. Where the
      // horizon happens to fall on a measured kerb or step the whole section
      // would be refused, so step the proxy back along the route, half a
      // metre at a time down to the useful-progress bound, until its
      // footprint no longer meets measured incompatible terrain. Unknown
      // ground is not a reason to step back: the provisional policy and the
      // validated prefix own that horizon. A few footprint checks, no search.
      const Eigen::Vector3d direction =
          (target.head<3>() - previous.head<3>()) / distance;
      const Eigen::Vector3d footprint = robot_params_.getPlanningSize();
      double reach = remaining;
      mgg::StateVec endpoint = previous;
      for (;;) {
        endpoint.head<3>() = previous.head<3>() + direction * reach;
        endpoint[3] = target[3];
        // Route poses already sit at driving height, like the graph they
        // come from, so the footprint is checked there directly.
        bool measured_hazard = false;
        if (robot_params_.type == mgg::RobotType::kGroundRobot) {
          const Eigen::Vector3d driving = endpoint.head<3>();
          measured_hazard =
              objectiveFootprintTerrainStatus(driving, footprint) ==
                  mgg::GridProjectionStatus::kNoGround ||
              objectiveSweptBodyStatus(driving + robot_params_.center_offset,
                                       driving + robot_params_.center_offset,
                                       footprint) == mgg::VoxelStatus::kOccupied;
        }
        // Never below the previous route pose, and never below the
        // useful-progress bound from where the robot stands.
        const double next = std::max(0.0, reach - 0.5);
        if (!measured_hazard || reach <= 0.0 ||
            (endpoint.head<2>() - current_state_.head<2>()).norm() <=
                partial_route_min_progress_m_ + 1e-9) {
          break;
        }
        reach = next;
      }
      if (reach < remaining) {
        RCLCPP_INFO(get_logger(),
                    "section proxy stepped back from %.1f m to %.1f m along "
                    "the route: measured terrain at the horizon",
                    remaining, reach);
      }
      // A proxy stepped all the way back onto the previous route pose is
      // that pose: the section ends there rather than listing it twice.
      if (local.poses.empty() ||
          (local.poses.back().head<3>() - endpoint.head<3>()).norm() > 1e-6) {
        local.poses.push_back(endpoint);
      }
      break;
    }
    local.poses.push_back(target);
    remaining = std::max(0.0, remaining - distance);
    previous = target;
    ++next_index;
    if (remaining <= 1e-9) break;
  }
  const bool more = next_index < corridor.poses.size();
  local.partial = more;
  if (more) {
    local.request.goal.pose = local.poses.back();
    local.request.goal.landmark_id.clear();
    pending_objective_route = std::make_unique<CachedObjectiveRoute>();
    pending_objective_route->id =
        route_instance_id_ + "-" + std::to_string(++route_sequence_);
    pending_objective_route->mission_id = core.mission_id;
    pending_objective_route->component_id = core.component_id;
    pending_objective_route->objective = core.objective;
    pending_objective_route->graph_revision = core.graph_revision;
    pending_objective_route->exact_goal = core.goal;
    pending_objective_route->global_poses = global_objective_path;
    pending_objective_route->next_index = next_index;
    pending_objective_route->expected_endpoint = local.poses.back();
  }
  corridor = std::move(local);
  return true;
}

void PlannerNode::onObjectiveRequest(
    const std::shared_ptr<mgg_msgs::srv::PlanObjective::Request> request,
  std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> response) {
  std::unique_lock<std::recursive_mutex> lock(planner_mutex_);
  const std::uint64_t objective_generation =
      ++objective_request_generation_;
  auto map_read = mola_map_ != nullptr ? mola_map_->acquireReadLease()
                                       : mgg::MolaMap::ReadLease{};
  refreshMolaRevision();
  const std::uint64_t mola_generation_at_start =
      mola_map_ != nullptr ? mola_map_->activeGeneration() : 0;
  const IndexedQueryContext query_context = indexedQueryContext();
  mgg::PlanningRequest core;
  core.mission_id = request->mission_id;
  core.objective = static_cast<mgg::ObjectiveKind>(request->objective);
  core.goal.pose = fromPoseMsg(request->goal);
  core.goal.landmark_id = request->goal_landmark_id;
  core.component_id = request->component_id.empty() ? component_id_
                                                    : request->component_id;
  core.map_epoch = request->map_epoch;
  core.mapping_graph_revision = request->mapping_graph_revision;
  core.geometry_revision = request->geometry_revision;
  core.map_source_stamp_sec = request->map_source_stamp.sec;
  core.map_source_stamp_nanosec = request->map_source_stamp.nanosec;
  // A new operator objective supersedes the only cached rolling route. Its
  // opaque token can never become current again within this node instance.
  cached_objective_route_.reset();
  mgg::RouteCorridor corridor;
  corridor.request = core;
  bool explicit_objective_planned = false;
  bool provisional_physical_home = false;
  bool explore_corridor_from_global_graph = false;
  TopologicalRetry topological_retry;
  const bool local_objective = core.objective == mgg::ObjectiveKind::kExplore;
  const std::uint64_t active_graph_revision =
      local_objective ? local_graph_revision_ : graph_revision_;
  const std::uint64_t active_map_revision =
      local_objective && request->graph_revision != 0
          ? local_graph_map_revision_
          : map_revision_;
  const bool supported = request->objective <=
                         mgg_msgs::srv::PlanObjective::Request::RETURN_HOME;
  const bool explicit_goal =
      core.objective == mgg::ObjectiveKind::kNavigate ||
      core.objective == mgg::ObjectiveKind::kReturnHome;
  static const std::regex kDigest("^[0-9a-fA-F]{64}$");
  const bool mapping_stamp_valid =
      core.map_source_stamp_sec >= 0 &&
      core.map_source_stamp_nanosec < 1000000000u &&
      (core.map_source_stamp_sec != 0 || core.map_source_stamp_nanosec != 0);
  const bool mapping_authority_fresh =
      query_context.mapping_snapshot_fresh_at_capture;
  const auto& captured_snapshot = query_context.mapping_snapshot;
  const bool mapping_key_matches =
      mapping_authority_fresh &&
      core.component_id == captured_snapshot.component_id &&
      core.map_epoch == captured_snapshot.epoch &&
      core.mapping_graph_revision == captured_snapshot.graph_revision &&
      core.geometry_revision == captured_snapshot.geometry_revision &&
      core.map_source_stamp_sec == captured_snapshot.source_stamp.sec &&
      core.map_source_stamp_nanosec == captured_snapshot.source_stamp.nanosec;
  const bool request_has_mapping_key =
      core.map_epoch != 0 || core.mapping_graph_revision != 0 ||
      !core.geometry_revision.empty() || core.map_source_stamp_sec != 0 ||
      core.map_source_stamp_nanosec != 0;
  const bool require_mapping_key =
      indexed_map_client_ || have_mapping_snapshot_ || request_has_mapping_key;
  if (!supported || !current_state_.allFinite() ||
      (explicit_goal && !core.goal.pose.allFinite())) {
    corridor.status = mgg::PlanningStatus::kUnsupportedObjective;
    corridor.reason = "objective and goal must be supported and finite";
  } else if (require_mapping_key &&
             (!mapping_stamp_valid ||
              !std::regex_match(core.geometry_revision, kDigest) ||
              !mapping_key_matches)) {
    corridor.status = mgg::PlanningStatus::kStaleRevision;
    corridor.reason = "mapping authority snapshot is missing, invalid, or stale";
  } else if ((!require_mapping_key && core.component_id != component_id_) ||
             (request->graph_revision != 0 &&
              request->graph_revision != active_graph_revision) ||
             (request->map_revision != 0 &&
              request->map_revision != active_map_revision) ||
             (local_objective && request->graph_revision != 0 &&
              local_graph_map_revision_ != map_revision_)) {
    corridor.status = mgg::PlanningStatus::kStaleRevision;
    corridor.reason = "component or planning snapshot revision is stale";
  } else if (core.objective == mgg::ObjectiveKind::kReturnHome &&
             global_backbone_history_lost_) {
    corridor.status = mgg::PlanningStatus::kBlocked;
    corridor.reason = "global trajectory history was lost: " +
                      global_backbone_history_lost_reason_;
  } else if (!have_odometry_ || !map_->getStatus()) {
    corridor.status = mgg::PlanningStatus::kBlocked;
    corridor.reason = "odometry or planning map is unavailable";
  } else if (const std::string stale = staleOdometryReason(); !stale.empty()) {
    corridor.status = mgg::PlanningStatus::kBlocked;
    corridor.reason = stale;
  } else {
    // A zero revision requests a fresh local snapshot. Explicit revisions bind
    // the already-built snapshot and never rebuild underneath the request.
    std::string summary;
    if (core.objective == mgg::ObjectiveKind::kExplore &&
        request->graph_revision == 0) {
      // Explore retains its legacy frontier policy. Explicit objectives do
      // not rebuild or score a local gain graph.
      summary = buildLocalGraph();
    }
    core.graph_revision = local_objective ? local_graph_revision_
                                          : graph_revision_;
    core.map_revision = map_revision_;
    if (core.objective == mgg::ObjectiveKind::kExplore) {
      // Explore's utility/gain selector still chooses the target frontier.
      // The corridor to it comes from the same topological stage that
      // Navigate and ReturnHome use, so all three share terrain decisions,
      // the bounded local window and route continuation. When the local
      // graph has nothing worth going to, the same stage routes over the
      // global graph to the best global frontier instead.
      planExploreCorridor(core, summary, corridor, topological_retry,
                          explore_corridor_from_global_graph);
    } else {
      // All explicit objectives use the persistent graph snapshot. Missing or
      // disconnected topology cannot suppress Navigate's bounded map A*.
      mgg::GraphManager& graph = *global_graph_;
      mgg::StateVec graph_current = current_state_;
      mgg::PlanningRequest graph_request = core;
      const bool current_has_mapped_support =
          projectStateToDrivingHeight(graph_current);
      if (!current_has_mapped_support) {
        // Odometry is physical provenance for this exact footprint. It may
        // seed topology at the configured driving height, but refinement must
        // still find and strictly validate a bounded connector to mapped
        // support before any motion path can be returned.
        graph_current = physicalAnchorAtDrivingHeight(current_state_);
      }
      const mgg::Vertex* home_root = global_graph_->getVertex(0);
      const double home_delta =
          have_initial_state_
              ? (core.goal.pose.head<3>() - initial_state_.head<3>())
                    .cwiseAbs().maxCoeff()
              : std::numeric_limits<double>::infinity();
      const double home_association_tolerance =
          std::clamp(2.0 * map_->getResolution(), 0.01, 0.25);
      const bool requests_physical_home =
          core.objective == mgg::ObjectiveKind::kReturnHome &&
          have_initial_state_ && home_root != nullptr &&
          (home_delta <= 1e-3 ||
          (!core.goal.landmark_id.empty() &&
            home_delta <= home_association_tolerance));
      provisional_physical_home =
          provisional_unknown_ground_ && requests_physical_home;
      bool home_goal_supported = true;
      if (core.objective == mgg::ObjectiveKind::kReturnHome &&
          !projectStateToDrivingHeight(graph_request.goal.pose, true)) {
        home_goal_supported = false;
        if ((initial_anchor_supported_ || provisional_unknown_ground_) &&
            requests_physical_home) {
          graph_request.goal.pose = home_root->state;
          home_goal_supported = true;
        }
      }
      if (!home_goal_supported && !current_has_mapped_support) {
        corridor.request = core;
        corridor.status = mgg::PlanningStatus::kUnreachable;
        char reason[192];
        std::snprintf(reason, sizeof(reason),
                      "current pose rejected: no mapped ground support at "
                      "(%.2f, %.2f, %.2f)",
                      current_state_.x(), current_state_.y(), current_state_.z());
        corridor.reason = reason;
      } else if (!home_goal_supported) {
        corridor.request = core;
        corridor.status = mgg::PlanningStatus::kUnreachable;
        char reason[192];
        std::snprintf(reason, sizeof(reason),
                      "goal rejected: no mapped ground support at "
                      "(%.2f, %.2f, %.2f)",
                      graph_request.goal.pose.x(), graph_request.goal.pose.y(),
                      graph_request.goal.pose.z());
        corridor.reason = reason;
      } else {
        // A supported nearby goal must bind at its projected driving height.
        // A distant unsupported goal may still receive a checked local proxy;
        // its exact terrain is assessed only when a later horizon reaches it.
        mgg::StateVec projected_goal = graph_request.goal.pose;
        const bool navigate_goal_is_current =
            core.objective == mgg::ObjectiveKind::kNavigate &&
            (projected_goal.head<2>() - current_state_.head<2>())
                    .cwiseAbs().maxCoeff() <= 1e-6;
        const bool resolve_exact_navigate_height =
            core.objective == mgg::ObjectiveKind::kNavigate &&
            !navigate_goal_is_current;
        const bool projected_goal_supported =
            resolve_exact_navigate_height
                ? resolveNavigateGoalDrivingHeight(projected_goal)
                : projectStateToDrivingHeight(projected_goal, true);
        if (projected_goal_supported) {
          graph_request.goal.pose = projected_goal;
        }
        const double explicit_minimum_progress =
            core.objective == mgg::ObjectiveKind::kNavigate
                ? partial_route_min_progress_m_
                : 0.0;
        mgg::TopologicalGoalPlanner objective_planner(
            core.component_id, core.graph_revision, core.map_revision, 1.0,
            explicit_minimum_progress);
        explicit_objective_planned = true;
        topological_retry.valid = true;
        topological_retry.graph = &graph;
        topological_retry.current = graph_current;
        topological_retry.request = graph_request;
        topological_retry.goal_tolerance = 1.0;
        topological_retry.minimum_partial_progress = explicit_minimum_progress;
        corridor = objective_planner.plan(graph, graph_current, graph_request,
                                          blockedCorridorView());
        if (core.objective == mgg::ObjectiveKind::kNavigate &&
            robot_params_.type == mgg::RobotType::kGroundRobot &&
            !projected_goal_supported && corridor.poses.size() >= 2u &&
            (corridor.poses.back().head<2>() - core.goal.pose.head<2>())
                    .cwiseAbs().maxCoeff() <= 1e-6) {
          // An unsupported UI goal owns an exact navigation-base pose, while
          // graph vertices and rolling proxies use driving height.  Keep the
          // optimistic connector on its preceding graph driving plane until a
          // later local window observes terrain near the goal.  Interpolating
          // the request's base Z here fabricates a long slope and a false step
          // at the first horizon.  corridor.request below still retains the
          // exact caller-owned XYZ and yaw for final-window binding.
          corridor.poses.back().z() =
              corridor.poses[corridor.poses.size() - 2u].z();
        }
        // Graph lookup uses driving height, while refinement and the response
        // retain the caller's exact base-pose goal.
        corridor.request = core;
        shortcutObjectiveCorridor(corridor);
      }
    }
  }

  // All three objectives take the same bounded first section out of their
  // persistent topological route, and mint the same continuation token.
  std::vector<mgg::StateVec> global_objective_path;
  std::unique_ptr<CachedObjectiveRoute> pending_objective_route;
  sliceObjectiveRouteWindow(core, corridor, global_objective_path,
                            pending_objective_route);

  // All three objectives now refine against the same bounded grid profile.
  const mgg::GridRefinementLimits* objective_limits = &objective_grid_limits_;
  mgg::RouteCorridor primary = corridor;
  const bool objective_graph_primary =
      explicit_objective_planned &&
      (core.objective == mgg::ObjectiveKind::kNavigate ||
       core.objective == mgg::ObjectiveKind::kReturnHome) &&
      corridor.status == mgg::PlanningStatus::kSucceeded &&
      !corridor.poses.empty();
  const bool direct_primary =
      explicit_objective_planned &&
      ((core.objective == mgg::ObjectiveKind::kNavigate &&
        !objective_graph_primary) ||
       (provisional_physical_home &&
        corridor.status == mgg::PlanningStatus::kUnreachable &&
        corridor.poses.empty() &&
        (core.goal.pose.head<2>() - current_state_.head<2>()).norm() <=
            objective_route_horizon_m_)) &&
      (corridor.status == mgg::PlanningStatus::kSucceeded ||
       corridor.status == mgg::PlanningStatus::kUnreachable);
  if (direct_primary) {
    // Navigate searches directly only when topology provides no usable graph
    // corridor. ReturnHome follows its persistent graph whenever one exists;
    // a physical Home without one retains the bounded provisional escape.
    primary.status = mgg::PlanningStatus::kSucceeded;
    primary.poses.clear();
    primary.partial = false;
    primary.reason.clear();
  }
  // A corridor that came from the topological stage can be marked blocked and
  // replaced. A direct search has no corridor to mark.
  const bool topological_corridor_primary =
      topological_retry.valid && !direct_primary &&
      corridor.status == mgg::PlanningStatus::kSucceeded &&
      !corridor.poses.empty();
  const auto objective_refinement_started = std::chrono::steady_clock::now();
  mgg::FeasiblePath path = refineCorridor(primary, objective_limits);
  if (topological_corridor_primary &&
      path.status != mgg::PlanningStatus::kSucceeded) {
    // Blocked-corridor feedback. Mark the corridor segment the bounded grid
    // stage rejected, then ask the topological stage once for an alternative
    // route inside the same deadline. Shared by Explore, Navigate and Home.
    const std::size_t from_index = path.blocked_from_index;
    std::size_t to_index = path.blocked_to_index;
    if (!global_objective_path.empty() &&
        to_index >= global_objective_path.size()) {
      to_index = global_objective_path.size() - 1u;
    }
    if (path.blocked_segment_identified &&
        from_index != mgg::kNoCorridorIndex &&
        from_index < global_objective_path.size() &&
        to_index < global_objective_path.size() && from_index != to_index) {
      blockCorridorSegment(global_objective_path[from_index],
                           global_objective_path[to_index]);
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - objective_refinement_started);
      if (elapsed < objective_grid_limits_.timeout) {
        mgg::TopologicalGoalPlanner alternative_planner(
            core.component_id, core.graph_revision, core.map_revision,
            topological_retry.goal_tolerance,
            topological_retry.minimum_partial_progress);
        mgg::RouteCorridor alternative = alternative_planner.plan(
            *topological_retry.graph, topological_retry.current,
            topological_retry.request, blockedCorridorView());
        alternative.request = core;
        const auto sameRoute = [](const std::vector<mgg::StateVec>& a,
                                  const std::vector<mgg::StateVec>& b) {
          if (a.size() != b.size()) return false;
          for (std::size_t i = 0; i < a.size(); ++i) {
            if ((a[i].head<3>() - b[i].head<3>()).cwiseAbs().maxCoeff() >
                1e-9) {
              return false;
            }
          }
          return true;
        };
        if (alternative.status == mgg::PlanningStatus::kSucceeded &&
            !alternative.poses.empty() &&
            !sameRoute(alternative.poses, global_objective_path)) {
          std::vector<mgg::StateVec> alternative_global;
          std::unique_ptr<CachedObjectiveRoute> alternative_pending;
          if (sliceObjectiveRouteWindow(core, alternative, alternative_global,
                                        alternative_pending)) {
            mgg::GridRefinementLimits remaining = objective_grid_limits_;
            remaining.timeout -= elapsed;
            mgg::FeasiblePath rerouted = refineCorridor(alternative, &remaining);
            if (rerouted.status == mgg::PlanningStatus::kSucceeded) {
              path = std::move(rerouted);
              corridor = std::move(alternative);
              primary = corridor;
              global_objective_path = std::move(alternative_global);
              pending_objective_route = std::move(alternative_pending);
            }
          }
        }
      }
    }
  }
  const std::string primary_failure_reason = path.reason;
  if (objective_graph_primary &&
      path.status != mgg::PlanningStatus::kSucceeded) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - objective_refinement_started);
    if (elapsed < objective_grid_limits_.timeout) {
      mgg::RouteCorridor direct = corridor;
      direct.status = mgg::PlanningStatus::kSucceeded;
      direct.poses.clear();
      direct.partial = false;
      direct.reason.clear();
      mgg::GridRefinementLimits remaining = objective_grid_limits_;
      remaining.timeout -= elapsed;
      mgg::FeasiblePath fallback = refineCorridor(direct, &remaining);
      if (fallback.status == mgg::PlanningStatus::kSucceeded) {
        fallback.partial = corridor.partial;
      }
      if (fallback.status != mgg::PlanningStatus::kSucceeded) {
        fallback.reason =
            "graph corridor: " +
            boundedObjectiveFailure(primary_failure_reason) +
            " [direct fallback: " +
            boundedObjectiveFailure(fallback.reason) + "]";
      }
      path = std::move(fallback);
    }
  }
  if (!objective_graph_primary &&
      path.status != mgg::PlanningStatus::kSucceeded &&
      hasSkippableObjectiveWaypointRejection(primary, path.reason)) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - objective_refinement_started);
    if (elapsed < objective_grid_limits_.timeout) {
      mgg::RouteCorridor endpoint = objectiveLocalEndpointCorridor(primary);
      mgg::GridRefinementLimits remaining = objective_grid_limits_;
      remaining.timeout -= elapsed;
      mgg::FeasiblePath fallback = refineCorridor(endpoint, &remaining);
      if (fallback.status != mgg::PlanningStatus::kSucceeded) {
        fallback.reason =
            "objective graph corridor: " +
            boundedObjectiveFailure(primary_failure_reason) +
            " [local endpoint fallback: " +
            boundedObjectiveFailure(fallback.reason) + "]";
      }
      path = std::move(fallback);
    }
  }
  const ObjectiveIndexedFlags indexed_flags =
      objectiveIndexedQueryFlags(core.objective);
  const bool explicit_objective =
      core.objective == mgg::ObjectiveKind::kNavigate ||
      core.objective == mgg::ObjectiveKind::kReturnHome;
  // A committed objective whose section the indexed authority shortens resumes
  // along this exact route. Retain the route the grid planner approved before
  // validation so a direct Navigate without a graph corridor still has one.
  const std::vector<mgg::StateVec> validated_section_route =
      explicit_objective && path.status == mgg::PlanningStatus::kSucceeded
          ? path.poses
          : std::vector<mgg::StateVec>{};
  bool truncated_to_prefix = false;
  Eigen::Vector3d hazard_ahead =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  map_read.allowPublication();
  lock.unlock();
  if (path.status == mgg::PlanningStatus::kSucceeded) {
    queryIndexedMap(path, query_context, indexed_flags.height_refinement,
                    indexed_flags.prefix_truncation,
                    indexed_flags.bounded_unknown_tail, explicit_objective,
                    &truncated_to_prefix, &hazard_ahead);
  }
  if (mola_map_ != nullptr) {
    lock.lock();
    map_read.reacquirePublication();
    // The section stops short of a hazard on its route. Mark that corridor
    // segment, so the continuation asks the topological stage for a way
    // around it instead of the same route again.
    if (hazard_ahead.allFinite()) {
      blockRouteNear(global_objective_path, hazard_ahead);
    }
    const bool mola_ready = mola_map_->getStatus();
    refreshMolaRevision();
    if (path.status == mgg::PlanningStatus::kSucceeded &&
        (!mola_ready ||
         mola_map_->activeGeneration() != mola_generation_at_start)) {
      path.status = mgg::PlanningStatus::kStaleRevision;
      path.reason = "MOLA map snapshot changed during planning";
      path.poses.clear();
      path.partial = false;
      path.indexed_map_validated = false;
    }
  }
  if (!lock.owns_lock()) lock.lock();
  if (objective_generation != objective_request_generation_) {
    path.status = mgg::PlanningStatus::kBlocked;
    path.reason = "objective was superseded during route validation";
    path.poses.clear();
    path.partial = false;
    path.indexed_map_validated = false;
  }
  if (objective_generation == objective_request_generation_ &&
      path.status == mgg::PlanningStatus::kSucceeded && path.partial &&
      truncated_to_prefix && explicit_objective && !path.poses.empty()) {
    // The indexed authority accepted only a validated prefix of this section.
    // The committed destination does not move, so the continuation resumes
    // where the emitted prefix ends rather than where the horizon asked to
    // stop. A section that was the final one now needs a continuation too.
    if (!lock.owns_lock()) lock.lock();
    if (global_objective_path.empty() && !validated_section_route.empty()) {
      // No graph corridor existed, so the direct route the grid planner already
      // approved to the exact goal becomes the continuation topology. Global
      // route poses carry graph driving height, while a refined path is a
      // navigation-base path.
      global_objective_path = validated_section_route;
      if (robot_params_.type == mgg::RobotType::kGroundRobot) {
        for (mgg::StateVec& pose : global_objective_path) {
          pose.z() += query_context.graph_to_base;
        }
      }
    }
    if (global_objective_path.empty()) {
      path.status = mgg::PlanningStatus::kBlocked;
      path.reason = "validated objective prefix has no continuation route";
      path.poses.clear();
      path.partial = false;
      path.indexed_map_validated = false;
    } else {
      if (!pending_objective_route) {
        pending_objective_route = std::make_unique<CachedObjectiveRoute>();
        pending_objective_route->id =
            route_instance_id_ + "-" + std::to_string(++route_sequence_);
        pending_objective_route->mission_id = core.mission_id;
        pending_objective_route->component_id = core.component_id;
        pending_objective_route->objective = core.objective;
        pending_objective_route->graph_revision = core.graph_revision;
        pending_objective_route->exact_goal = core.goal;
        pending_objective_route->global_poses = global_objective_path;
      }
      const std::size_t resume_index = std::min(
          objectiveRouteIndexAfterLength(
              pending_objective_route->global_poses, 0,
              query_context.route_start,
              emittedSectionXyLength(query_context.route_start, path.poses)),
          pending_objective_route->global_poses.size() - 1u);
      pending_objective_route->next_index = resume_index;
      pending_objective_route->expected_endpoint = path.poses.back();
    }
  }
  if (objective_generation == objective_request_generation_ &&
      path.status == mgg::PlanningStatus::kSucceeded && path.partial &&
      pending_objective_route) {
    if (!lock.owns_lock()) lock.lock();
    cached_objective_route_ = std::move(pending_objective_route);
  } else if (objective_generation == objective_request_generation_) {
    if (!lock.owns_lock()) lock.lock();
    cached_objective_route_.reset();
  }
  if (objective_generation == objective_request_generation_ &&
      core.objective == mgg::ObjectiveKind::kExplore &&
      path.status == mgg::PlanningStatus::kSucceeded &&
      !explore_corridor_from_global_graph && !global_objective_path.empty()) {
    // The accepted exploration path joins the global graph (rrg.cpp:4549):
    // the whole lattice corridor at driving height, not just the section
    // returned now. A corridor taken from the global graph is already in it.
    if (!lock.owns_lock()) lock.lock();
    addRefPathToGraph(global_objective_path);
  }
  response->status = static_cast<std::uint8_t>(path.status);
  response->component_id = path.component_id;
  response->graph_revision = path.graph_revision;
  response->map_revision = path.map_revision;
  response->map_epoch = path.map_epoch;
  response->mapping_graph_revision = path.mapping_graph_revision;
  response->geometry_revision = path.geometry_revision;
  response->map_source_stamp.sec = path.map_source_stamp_sec;
  response->map_source_stamp.nanosec = path.map_source_stamp_nanosec;
  response->partial = path.status == mgg::PlanningStatus::kSucceeded &&
                      path.partial;
  response->indexed_map_validated =
      path.status == mgg::PlanningStatus::kSucceeded &&
      path.indexed_map_validated;
  response->reason = path.reason;
  for (const auto& pose : path.poses) response->path.push_back(toPoseMsg(pose));
  if (path.status == mgg::PlanningStatus::kSucceeded && !path.poses.empty()) {
    // What the caller is handed against what the continuation will expect:
    // a section the controller completes where the robot stands (2026-09-20,
    // three cycles) has to show up here as ending near the current state.
    RCLCPP_INFO(get_logger(),
                "objective section: %zu poses (%.2f, %.2f) -> (%.2f, %.2f), "
                "%.2f m from the robot at (%.2f, %.2f), route %zu poses%s",
                path.poses.size(), path.poses.front().x(),
                path.poses.front().y(), path.poses.back().x(),
                path.poses.back().y(),
                (path.poses.back().head<2>() - current_state_.head<2>()).norm(),
                current_state_.x(), current_state_.y(),
                global_objective_path.size(), path.partial ? " (partial)" : "");
  }
  if (!global_objective_path.empty()) {
    mgg::FeasiblePath display;
    display.poses = global_objective_path;
    convertPathToNavigationBase(display);
    for (const auto& pose : display.poses)
      response->global_path.push_back(toPoseMsg(pose));
    if (core.objective != mgg::ObjectiveKind::kExplore) {
      // Navigate and ReturnHome own an exact caller-supplied goal already
      // expressed in the navigation base frame. Explore's goal is a graph
      // vertex the selector chose, so it takes the same conversion as the
      // rest of the corridor.
      response->global_path.back() = toPoseMsg(core.goal.pose);
    }
  }
  if (cached_objective_route_ && path.status == mgg::PlanningStatus::kSucceeded &&
      path.partial) {
    response->route_id = cached_objective_route_->id;
  }
}

void PlannerNode::onRefineObjectiveRoute(
    const std::shared_ptr<mgg_msgs::srv::RefineObjectiveRoute::Request> request,
    std::shared_ptr<mgg_msgs::srv::RefineObjectiveRoute::Response> response) {
  std::unique_lock<std::recursive_mutex> lock(planner_mutex_);
  auto map_read = mola_map_ != nullptr ? mola_map_->acquireReadLease()
                                       : mgg::MolaMap::ReadLease{};
  refreshMolaRevision();
  const std::uint64_t mola_generation_at_start =
      mola_map_ != nullptr ? mola_map_->activeGeneration() : 0;
  const IndexedQueryContext query_context = indexedQueryContext();
  auto finish = [&](mgg::PlanningStatus status, const std::string& reason) {
    response->status = static_cast<std::uint8_t>(status);
    response->component_id = cached_objective_route_
                                 ? cached_objective_route_->component_id
                                 : component_id_;
    // An exploration route is pinned to the local lattice snapshot, not to the
    // persistent graph revision the explicit objectives bind.
    response->graph_revision =
        cached_objective_route_ && cached_objective_route_->objective ==
                                       mgg::ObjectiveKind::kExplore
            ? local_graph_revision_
            : graph_revision_;
    response->map_revision = map_revision_;
    if (have_mapping_snapshot_) {
      response->map_epoch = mapping_snapshot_.epoch;
      response->mapping_graph_revision = mapping_snapshot_.graph_revision;
      response->geometry_revision = mapping_snapshot_.geometry_revision;
      response->map_source_stamp = mapping_snapshot_.source_stamp;
    }
    response->reason = reason;
  };
  if (!cached_objective_route_ || request->route_id.empty() ||
      request->route_id != cached_objective_route_->id ||
      request->mission_id != cached_objective_route_->mission_id ||
      request->component_id != cached_objective_route_->component_id) {
    finish(mgg::PlanningStatus::kBlocked,
           "objective route token is unknown, expired, or superseded");
    return;
  }
  const std::string refining_route_id = cached_objective_route_->id;
  const bool native_revision_matches =
      (request->graph_revision == 0 ||
       request->graph_revision == cached_objective_route_->graph_revision) &&
      (request->map_revision == 0 || request->map_revision == map_revision_);
  const bool request_has_mapping_key =
      request->map_epoch != 0 || request->mapping_graph_revision != 0 ||
      !request->geometry_revision.empty() || request->map_source_stamp.sec != 0 ||
      request->map_source_stamp.nanosec != 0;
  const bool require_mapping_key =
      indexed_map_client_ || have_mapping_snapshot_ || request_has_mapping_key;
  const bool mapping_authority_fresh =
      query_context.mapping_snapshot_fresh_at_capture;
  const auto& captured_snapshot = query_context.mapping_snapshot;
  const bool mapping_key_matches =
      !require_mapping_key ||
      (mapping_authority_fresh &&
       request->component_id == captured_snapshot.component_id &&
       request->map_epoch == captured_snapshot.epoch &&
       request->mapping_graph_revision == captured_snapshot.graph_revision &&
       request->geometry_revision == captured_snapshot.geometry_revision &&
       request->map_source_stamp.sec == captured_snapshot.source_stamp.sec &&
       request->map_source_stamp.nanosec ==
           captured_snapshot.source_stamp.nanosec);
  if (!native_revision_matches || !mapping_key_matches) {
    finish(mgg::PlanningStatus::kStaleRevision,
           "current planning or mapping snapshot does not match");
    return;
  }
  if (!have_odometry_ || !current_state_.allFinite() || !map_->getStatus()) {
    finish(mgg::PlanningStatus::kBlocked,
           "odometry or planning map is unavailable");
    return;
  }
  if (const std::string stale = staleOdometryReason(); !stale.empty()) {
    finish(mgg::PlanningStatus::kBlocked, stale);
    return;
  }
  const double endpoint_distance_m =
      (current_state_.head<2>() -
       cached_objective_route_->expected_endpoint.head<2>())
          .norm();
  if (endpoint_distance_m > objective_route_progress_tolerance_m_) {
    // The distance and the odometry age tell a section that was driven from
    // one the controller completed where the robot already stood.
    const double odometry_age_s =
        last_odometry_received_
            ? static_cast<double>(now().nanoseconds() -
                                  last_odometry_received_->nanoseconds()) *
                  1e-9
            : 0.0;
    char detail[160];
    std::snprintf(detail, sizeof(detail),
                  "robot is outside the completed objective route window "
                  "(%.2f m from the section endpoint, odometry %.1f s old)",
                  endpoint_distance_m, odometry_age_s);
    finish(mgg::PlanningStatus::kBlocked, detail);
    return;
  }
  if (cached_objective_route_->next_index >=
      cached_objective_route_->global_poses.size()) {
    finish(mgg::PlanningStatus::kBlocked, "objective route is already complete");
    cached_objective_route_.reset();
    return;
  }

  const std::size_t begin = cached_objective_route_->next_index;
  std::size_t next_index = begin;
  double remaining = objective_route_horizon_m_;
  mgg::StateVec previous = current_state_;
  std::vector<mgg::StateVec> local_poses;
  while (next_index < cached_objective_route_->global_poses.size()) {
    const mgg::StateVec& target =
        cached_objective_route_->global_poses[next_index];
    const double distance =
        (target.head<2>() - previous.head<2>()).norm();
    if (distance > remaining + 1e-9) {
      mgg::StateVec endpoint = previous;
      endpoint.head<3>() +=
          (target.head<3>() - previous.head<3>()) * (remaining / distance);
      endpoint[3] = target[3];
      local_poses.push_back(endpoint);
      break;
    }
    local_poses.push_back(target);
    remaining = std::max(0.0, remaining - distance);
    previous = target;
    ++next_index;
    if (remaining <= 1e-9) break;
  }
  const bool more = next_index < cached_objective_route_->global_poses.size();
  mgg::RouteCorridor corridor;
  corridor.status = mgg::PlanningStatus::kSucceeded;
  corridor.partial = more;
  corridor.request.mission_id = cached_objective_route_->mission_id;
  corridor.request.objective = cached_objective_route_->objective;
  corridor.request.component_id = cached_objective_route_->component_id;
  corridor.request.graph_revision = cached_objective_route_->graph_revision;
  corridor.request.map_revision = map_revision_;
  corridor.request.map_epoch = request->map_epoch;
  corridor.request.mapping_graph_revision = request->mapping_graph_revision;
  corridor.request.geometry_revision = request->geometry_revision;
  corridor.request.map_source_stamp_sec = request->map_source_stamp.sec;
  corridor.request.map_source_stamp_nanosec = request->map_source_stamp.nanosec;
  corridor.request.goal = more
                              ? mgg::PlanningGoal{
                                    local_poses.back(), ""}
                              : cached_objective_route_->exact_goal;
  corridor.poses = std::move(local_poses);
  const auto objective_refinement_started = std::chrono::steady_clock::now();
  mgg::FeasiblePath path = refineCorridor(corridor, &objective_grid_limits_);
  if (path.status != mgg::PlanningStatus::kSucceeded &&
      path.blocked_segment_identified) {
    // Blocked-corridor feedback for a continuation. Section pose j is global
    // pose begin + j, including the interpolated horizon endpoint, which
    // stands in for the global pose it was heading towards.
    const std::vector<mgg::StateVec>& route =
        cached_objective_route_->global_poses;
    const std::size_t from_index =
        path.blocked_from_index == mgg::kNoCorridorIndex
            ? (begin == 0 ? mgg::kNoCorridorIndex : begin - 1u)
            : begin + path.blocked_from_index;
    std::size_t to_index = begin + path.blocked_to_index;
    if (path.blocked_to_index != mgg::kNoCorridorIndex && !route.empty() &&
        to_index >= route.size()) {
      to_index = route.size() - 1u;
    }
    if (from_index != mgg::kNoCorridorIndex && from_index < route.size() &&
        path.blocked_to_index != mgg::kNoCorridorIndex &&
        to_index < route.size() && from_index != to_index) {
      blockCorridorSegment(route[from_index], route[to_index]);
    }
  }
  const std::string primary_failure_reason = path.reason;
  if (path.status != mgg::PlanningStatus::kSucceeded) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - objective_refinement_started);
    if (elapsed < objective_grid_limits_.timeout) {
      mgg::RouteCorridor endpoint = objectiveLocalEndpointCorridor(corridor);
      mgg::GridRefinementLimits remaining = objective_grid_limits_;
      remaining.timeout -= elapsed;
      mgg::FeasiblePath fallback = refineCorridor(endpoint, &remaining);
      if (fallback.status != mgg::PlanningStatus::kSucceeded) {
        fallback.reason =
            "objective graph corridor: " +
            boundedObjectiveFailure(primary_failure_reason) +
            " [local endpoint fallback: " +
            boundedObjectiveFailure(fallback.reason) + "]";
      }
      path = std::move(fallback);
    }
  }
  const mgg::StateVec section_start = current_state_;
  const std::vector<mgg::StateVec> route_poses =
      cached_objective_route_->global_poses;
  bool truncated_to_prefix = false;
  map_read.allowPublication();
  lock.unlock();
  if (path.status == mgg::PlanningStatus::kSucceeded) {
    const bool qualified_explicit_ground =
        map_backend_ == "mola_snapshot" && provisional_unknown_ground_ &&
        observed_ground_body_evidence_ &&
        robot_params_.type == mgg::RobotType::kGroundRobot;
    queryIndexedMap(path, query_context, qualified_explicit_ground,
                    /*allow_prefix_truncation=*/false,
                    qualified_explicit_ground,
                    /*allow_continuable_prefix=*/true, &truncated_to_prefix);
  }
  lock.lock();
  if (mola_map_ != nullptr) {
    map_read.reacquirePublication();
    const bool mola_ready = mola_map_->getStatus();
    refreshMolaRevision();
    if (path.status == mgg::PlanningStatus::kSucceeded &&
        (!mola_ready ||
         mola_map_->activeGeneration() != mola_generation_at_start)) {
      path.status = mgg::PlanningStatus::kStaleRevision;
      path.reason = "MOLA map snapshot changed during planning";
      path.poses.clear();
      path.partial = false;
    }
  }
  if (path.status == mgg::PlanningStatus::kSucceeded &&
      (!cached_objective_route_ ||
       cached_objective_route_->id != refining_route_id)) {
    path.status = mgg::PlanningStatus::kBlocked;
    path.reason = "objective route was superseded during validation";
    path.poses.clear();
    path.partial = false;
    path.indexed_map_validated = false;
  }
  if (path.status == mgg::PlanningStatus::kSucceeded) {
    if (path.partial && truncated_to_prefix && !path.poses.empty() &&
        !route_poses.empty()) {
      // Only a validated prefix of this section was accepted. The committed
      // destination is unchanged, so resume from the emitted endpoint. A final
      // section shortened this way keeps its cached route instead of retiring.
      cached_objective_route_->next_index = std::min(
          objectiveRouteIndexAfterLength(
              route_poses, begin, section_start,
              emittedSectionXyLength(section_start, path.poses)),
          route_poses.size() - 1u);
      cached_objective_route_->expected_endpoint = path.poses.back();
    } else if (path.partial) {
      cached_objective_route_->next_index = next_index;
      cached_objective_route_->expected_endpoint =
          corridor.request.goal.pose;
    } else {
      cached_objective_route_.reset();
    }
  }
  response->status = static_cast<std::uint8_t>(path.status);
  response->component_id = path.component_id;
  response->graph_revision = path.graph_revision;
  response->map_revision = path.map_revision;
  response->map_epoch = path.map_epoch;
  response->mapping_graph_revision = path.mapping_graph_revision;
  response->geometry_revision = path.geometry_revision;
  response->map_source_stamp.sec = path.map_source_stamp_sec;
  response->map_source_stamp.nanosec = path.map_source_stamp_nanosec;
  response->partial = path.status == mgg::PlanningStatus::kSucceeded && path.partial;
  response->indexed_map_validated =
      path.status == mgg::PlanningStatus::kSucceeded && path.indexed_map_validated;
  response->reason = path.reason;
  for (const auto& pose : path.poses) response->path.push_back(toPoseMsg(pose));
  if (path.status == mgg::PlanningStatus::kSucceeded && !path.poses.empty()) {
    RCLCPP_INFO(get_logger(),
                "objective continuation: %zu poses (%.2f, %.2f) -> (%.2f, %.2f), "
                "%.2f m from the robot at (%.2f, %.2f)%s",
                path.poses.size(), path.poses.front().x(),
                path.poses.front().y(), path.poses.back().x(),
                path.poses.back().y(),
                (path.poses.back().head<2>() - current_state_.head<2>()).norm(),
                current_state_.x(), current_state_.y(),
                path.partial ? " (partial)" : "");
  }
}

void PlannerNode::publishPath() {
  nav_msgs::msg::Path msg;
  msg.header.stamp = now();
  msg.header.frame_id = world_frame_;
  mgg::FeasiblePath navigation_path;
  navigation_path.poses = best_path_;
  convertPathToNavigationBase(navigation_path);
  for (const mgg::StateVec& s : navigation_path.poses) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = toPoseMsg(s);
    msg.poses.push_back(pose);
  }
  path_pub_->publish(msg);
}

void PlannerNode::publishOwnGraph() {
  // Runs on a timer, so it can land in the middle of a planning cycle
  // rewriting the very graph it is serialising.
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (global_graph_->getNumVertices() == 0) return;
  auto msg = toGraphMsg(*global_graph_,
                        static_cast<int>(planning_params_.robot_id));
  if (msg.vertices.empty()) return;
  msg.header.stamp = now();
  msg.header.frame_id = world_frame_;
  graph_pub_->publish(msg);
}

void PlannerNode::publishMarkers() {
  if (marker_pub_->get_subscription_count() == 0) return;
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);

  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker vertices;
  vertices.header.frame_id = world_frame_;
  vertices.header.stamp = now();
  vertices.ns = "local_graph";
  vertices.type = visualization_msgs::msg::Marker::POINTS;
  vertices.action = visualization_msgs::msg::Marker::ADD;
  vertices.scale.x = 0.15;
  vertices.scale.y = 0.15;
  vertices.color.g = 1.0;
  vertices.color.a = 1.0;
  vertices.pose.orientation.w = 1.0;
  for (const auto& entry : local_graph_->vertices_map_) {
    if (entry.second == nullptr) continue;
    geometry_msgs::msg::Point p;
    p.x = entry.second->state[0];
    p.y = entry.second->state[1];
    p.z = entry.second->state[2];
    vertices.points.push_back(p);
  }
  array.markers.push_back(vertices);

  // The edges, as a LINE_LIST. The vertices alone say where the planner
  // considered standing; the edges say where it believes it can drive, which
  // is the part that shows a graph split by an obstacle or stranded in a
  // corner. The ARGoS bridge draws these too.
  visualization_msgs::msg::Marker edges;
  edges.header.frame_id = world_frame_;
  edges.header.stamp = now();
  edges.ns = "local_graph_edges";
  edges.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges.action = visualization_msgs::msg::Marker::ADD;
  edges.scale.x = 0.03;
  edges.color.b = 1.0;
  edges.color.g = 0.6;
  edges.color.a = 0.6;
  edges.pose.orientation.w = 1.0;
  std::pair<mgg::Graph::GraphType::edge_iterator,
            mgg::Graph::GraphType::edge_iterator> edge_range;
  local_graph_->graph_->getEdgeIterator(edge_range);
  for (auto it = edge_range.first; it != edge_range.second; ++it) {
    const auto property = local_graph_->graph_->getEdgeProperty(it);
    const mgg::Vertex* u = local_graph_->getVertex(std::get<0>(property));
    const mgg::Vertex* v = local_graph_->getVertex(std::get<1>(property));
    if (u == nullptr || v == nullptr) continue;
    geometry_msgs::msg::Point a, b;
    a.x = u->state[0]; a.y = u->state[1]; a.z = u->state[2];
    b.x = v->state[0]; b.y = v->state[1]; b.z = v->state[2];
    edges.points.push_back(a);
    edges.points.push_back(b);
  }
  array.markers.push_back(edges);

  // Frontiers, as POINTS. Shows candidate exploration targets.
  visualization_msgs::msg::Marker frontiers;
  frontiers.header.frame_id = world_frame_;
  frontiers.header.stamp = now();
  frontiers.ns = "frontiers";
  frontiers.type = visualization_msgs::msg::Marker::POINTS;
  frontiers.action = visualization_msgs::msg::Marker::ADD;
  frontiers.scale.x = 0.25;
  frontiers.scale.y = 0.25;
  frontiers.color.r = 1.0;
  frontiers.color.g = 0.2;
  frontiers.color.b = 0.2;
  frontiers.color.a = 1.0;
  frontiers.pose.orientation.w = 1.0;
  for (const auto& entry : local_graph_->vertices_map_) {
    if (entry.second != nullptr &&
        entry.second->type == mgg::VertexType::kFrontier) {
      geometry_msgs::msg::Point p;
      p.x = entry.second->state[0];
      p.y = entry.second->state[1];
      p.z = entry.second->state[2];
      frontiers.points.push_back(p);
    }
  }
  if (!frontiers.points.empty()) {
    array.markers.push_back(frontiers);
  }

  // Swarm global graph edges, as a LINE_LIST.
  if (global_graph_->getNumVertices() > 0) {
    visualization_msgs::msg::Marker global_edges;
    global_edges.header.frame_id = world_frame_;
    global_edges.header.stamp = now();
    global_edges.ns = "global_graph_edges";
    global_edges.type = visualization_msgs::msg::Marker::LINE_LIST;
    global_edges.action = visualization_msgs::msg::Marker::ADD;
    global_edges.scale.x = 0.05;
    global_edges.color.r = 1.0;
    global_edges.color.g = 0.8;
    global_edges.color.b = 0.2;
    global_edges.color.a = 1.0;
    global_edges.pose.orientation.w = 1.0;
    std::pair<mgg::Graph::GraphType::edge_iterator,
              mgg::Graph::GraphType::edge_iterator> g_edge_range;
    global_graph_->graph_->getEdgeIterator(g_edge_range);
    for (auto it = g_edge_range.first; it != g_edge_range.second; ++it) {
      const auto property = global_graph_->graph_->getEdgeProperty(it);
      const mgg::Vertex* u = global_graph_->getVertex(std::get<0>(property));
      const mgg::Vertex* v = global_graph_->getVertex(std::get<1>(property));
      if (u == nullptr || v == nullptr) continue;
      geometry_msgs::msg::Point a, b;
      a.x = u->state[0]; a.y = u->state[1]; a.z = u->state[2];
      b.x = v->state[0]; b.y = v->state[1]; b.z = v->state[2];
      global_edges.points.push_back(a);
      global_edges.points.push_back(b);
    }
    if (!global_edges.points.empty()) {
      array.markers.push_back(global_edges);
    }
  }

  // Swarm graph merge beacons & connecting lines
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
    merge_marker.color.r = 0.0;
    merge_marker.color.g = 1.0;
    merge_marker.color.b = 1.0;
    merge_marker.color.a = 1.0;
    merge_marker.pose.orientation.w = 1.0;

    for (const auto& ev : recent_merges_) {
      // Horizontal beacon cross at the rendezvous position
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
