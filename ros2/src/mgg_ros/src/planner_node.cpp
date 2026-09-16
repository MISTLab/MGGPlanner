#include "mgg_ros/planner_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <future>
#include <regex>
#include <stdexcept>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.hpp>

#include "mgg_core/log.h"
#include "mgg_core/trajectory.h"
#include "mgg_ros/conversions.h"
#include "mgg_ros/param_loader.h"

namespace mgg_ros {

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
  const double requested_start_support = declareOrGet<double>(
      this, "objective_start_support_max_distance_m",
      objective_start_support_max_distance_m_);
  objective_start_support_max_distance_m_ =
      std::isfinite(requested_start_support)
          ? std::clamp(requested_start_support, 0.0, 5.0)
          : 3.0;
  objective_grid_limits_.start_connector_max_distance_m =
      objective_start_support_max_distance_m_;
  geofence_ = std::make_unique<mgg::GeofenceManager>();
  local_graph_ = std::make_shared<mgg::GraphManager>();
  global_graph_ = std::make_shared<mgg::GraphManager>();
  local_graph_->setRobotId(static_cast<int>(planning_params_.robot_id));
  global_graph_->setRobotId(static_cast<int>(planning_params_.robot_id));

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
  current_state_ = state;
  have_odometry_ = true;
  if (!have_initial_state_) {
    initial_state_ = state;
    have_initial_state_ = true;
  }
  stageGlobalBreadcrumbs(state);
  refreshMolaRevision();
  updateGlobalGraph();
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

  local_graph_->reset();
  // Inclinations are keyed by vertex id and the ids restart with the graph.
  edge_inclinations_.clear();
  mgg::StateVec root_state = current_state_;
  bool root_hanging = false;
  if (robot_params_.type == mgg::RobotType::kGroundRobot) {
    Eigen::Vector3d pos(root_state[0], root_state[1], root_state[2]);
    mgg::VoxelStatus vs;
    const double ground_height = ground_->projectSample(pos, vs);
    if (vs == mgg::VoxelStatus::kOccupied) {
      root_state[2] = pos[2] - (ground_height - planning_params_.max_ground_height);
    } else {
      root_hanging = true;
      // Odometry locates the base, whereas graph states locate the raised
      // collision box. Preserve this offset even inside the sensor blind spot.
      root_state[2] += planning_params_.max_ground_height - robot_params_.size[2] / 2.0;
    }
  }
  auto* root = new mgg::Vertex(0, root_state);
  root->robot_id = static_cast<int>(planning_params_.robot_id);
  root->is_hanging = root_hanging;
  local_graph_->addVertex(root);



  mgg::ExpandContext ctx = makeContext();
  ctx.strict_projected_endpoint = strict_projected_endpoints;
  const auto t_global = Clock::now();
  const mgg::GridGraphResult r = buildGridGraph(
      *local_graph_, current_state_, grid_params_, ctx, current_state_[3]);
  ++local_graph_revision_;
  local_graph_map_revision_ = map_revision_;

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
  std::vector<Eigen::Vector3d> exclusions;
  if (have_coordination_exclusions_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    coordination_exclusions_received_)
              .count() <= reservation_exclusion_ttl_s_) {
    exclusions = coordination_exclusions_;
  }
  const mgg::PathSelectionResult sel = mgg::selectBestPath(
      *local_graph_, planning_params_, robot_params_, edge_inclinations_,
      map_->getResolution(), exploring_direction_, exclusions,
      reservation_exclusion_radius_m_);

  best_path_.clear();
  path_shortcut_from_ = 0;
  path_shortcut_corners_ = 0;
  path_shortcut_to_ = 0;
  for (const mgg::Vertex* v : sel.best_path) {
    if (v != nullptr) best_path_.push_back(v->state);
  }


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
                                              bool preserve_xy) const {
  if (robot_params_.type != mgg::RobotType::kGroundRobot) return true;
  const Eigen::Vector2d requested_xy = state.head<2>();
  Eigen::Vector3d pos(state[0], state[1], state[2]);
  mgg::VoxelStatus status;
  const double ground_height = ground_->projectSample(pos, status);
  if (status != mgg::VoxelStatus::kOccupied || ground_height < 0.0) {
    return false;
  }
  if (preserve_xy &&
      (pos.head<2>() - requested_xy).cwiseAbs().maxCoeff() > 1e-3) {
    return false;
  }
  state[0] = preserve_xy ? requested_xy.x() : pos[0];
  state[1] = preserve_xy ? requested_xy.y() : pos[1];
  state[2] = pos[2] - (ground_height - planning_params_.max_ground_height);
  return true;
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
    std::vector<mgg::StateVec>& checked) const {
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
  // it never makes occupancy free: the full swept body remains a strict,
  // unknown-rejecting query.
  std::vector<Eigen::Vector3d> projected;
  const Eigen::Vector3d body = robot_params_.getPlanningSize();
  if (ground_->getProjectedEdgeStatus(
          anchor.head<3>(), supported.head<3>(), body,
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
  for (const Eigen::Vector3d& point : projected) {
    // A hanging connector has no mapped terrain with which to justify a
    // gradual elevation change. Any observed floor outside the platform step
    // cap therefore refuses the connector, even when the end-to-end chord
    // would make that change look like a shallow ramp.
    if (std::abs(point.z() - anchor.z()) >
        planning_params_.max_step_height + 1e-6) {
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
    const mgg::VoxelStatus swept = map_->getStrictPathStatus(from, to, body);
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
            if (map_->getStrictBoxStatus(point, swept_size) !=
                mgg::VoxelStatus::kFree) {
              first = point;
              break;
            }
          }
        }
        char detail[160];
        std::snprintf(
            detail, sizeof(detail),
            "strict swept body %s near (%.2f, %.2f, %.2f)",
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

    if (reused_vertex != nullptr) {
      last_own_global_vertex_id_ = reused_vertex->id;
    } else {
      auto* v = new mgg::Vertex(global_graph_->generateVertexID(), state);
      v->robot_id = static_cast<int>(planning_params_.robot_id);
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

  best_path_.clear();
  const std::string summary = buildLocalGraph();
  robot_params_.bound_mode = previous;

  mgg::PlanningRequest core;
  core.objective = mgg::ObjectiveKind::kExplore;
  core.component_id = component_id_;
  core.graph_revision = local_graph_revision_;
  core.map_revision = map_revision_;
  const bool mapping_authority_fresh =
      have_mapping_snapshot_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    mapping_snapshot_received_).count() <=
          indexed_map_snapshot_ttl_s_;
  if (mapping_authority_fresh) {
    core.component_id = mapping_snapshot_.component_id;
    core.map_epoch = mapping_snapshot_.epoch;
    core.mapping_graph_revision = mapping_snapshot_.graph_revision;
    core.geometry_revision = mapping_snapshot_.geometry_revision;
    core.map_source_stamp_sec = mapping_snapshot_.source_stamp.sec;
    core.map_source_stamp_nanosec = mapping_snapshot_.source_stamp.nanosec;
  }
  mgg::RouteCorridor corridor;
  corridor.request = core;
  corridor.poses = best_path_;
  corridor.status = best_path_.empty() ? mgg::PlanningStatus::kUnreachable
                                       : mgg::PlanningStatus::kSucceeded;
  mgg::FeasiblePath feasible = refineCorridor(corridor);
  const IndexedQueryContext query_context = indexedQueryContext();
  map_read = mgg::MolaMap::ReadLease{};
  lock.unlock();
  if (feasible.status == mgg::PlanningStatus::kSucceeded) {
    queryIndexedMap(feasible, query_context);
  }
  lock.lock();
  map_read = mola_map_ != nullptr ? mola_map_->acquireReadLease()
                                  : mgg::MolaMap::ReadLease{};
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
      (!have_mapping_snapshot_ ||
       std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                     mapping_snapshot_received_).count() >
           indexed_map_snapshot_ttl_s_);
  const bool indexed_route_rejected =
      indexed_map_client_ && corridor.status == mgg::PlanningStatus::kSucceeded &&
      feasible.status != mgg::PlanningStatus::kSucceeded;
  if (feasible.status != mgg::PlanningStatus::kSucceeded) {
    // A failed local terrain/current-pose refinement is just as terminal as a
    // failed indexed query.  Keeping the selector's graph-height path here
    // bypassed the rejection whenever the optional indexed service was off.
    best_path_.clear();
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

  RCLCPP_INFO(get_logger(), "plan request: %s", summary.c_str());
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
  context.max_inclination = planning_params_.max_inclination;
  context.graph_to_base =
      planning_params_.max_ground_height - robot_params_.size.z() / 2.0;
  context.have_mapping_snapshot = have_mapping_snapshot_;
  context.mapping_snapshot = mapping_snapshot_;
  context.mapping_snapshot_received = mapping_snapshot_received_;
  return context;
}

bool PlannerNode::queryIndexedMap(mgg::FeasiblePath& path,
                                  const IndexedQueryContext& context) {
  path.indexed_map_validated = false;
  if (!indexed_map_client_) return true;
  const auto fail = [&path](mgg::PlanningStatus status,
                            const std::string& reason) {
    path.status = status;
    path.poses.clear();
    path.partial = false;
    path.reason = reason;
    return false;
  };
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
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    context.mapping_snapshot_received).count() >
          indexed_map_snapshot_ttl_s_) {
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
      (component_up - Eigen::Vector3d::UnitZ()).norm() > 1e-6 ||
      !std::isfinite(indexed_map_ground_tolerance_m_) ||
      !std::isfinite(context.max_step_height) ||
      context.max_step_height < 0.0 ||
      !std::isfinite(context.max_inclination) ||
      context.max_inclination < 0.0 ||
      !std::isfinite(context.graph_to_base)) {
    return fail(mgg::PlanningStatus::kBlocked,
                "indexed map body, terrain limits, or gravity alignment is invalid");
  }
  request->body_size.x = component_body.x();
  request->body_size.y = component_body.y();
  request->body_size.z = component_body.z();
  request->stop_at_unknown = true;

  std::vector<mgg::StateVec> route;
  route.reserve(path.poses.size() + 1);
  route.push_back(context.route_start);
  route.insert(route.end(), path.poses.begin(), path.poses.end());
  std::vector<double> expected_ground_z;
  for (std::size_t segment = 1; segment < route.size(); ++segment) {
    const Eigen::Vector3d a = route[segment - 1].head(3);
    const Eigen::Vector3d b = route[segment].head(3);
    const double distance = (b - a).norm();
    const std::size_t steps = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(distance /
                                              indexed_map_sample_spacing_m_)));
    for (std::size_t i = segment == 1 ? 0 : 1; i <= steps; ++i) {
      const Eigen::Vector3d base =
          a + (b - a) * (static_cast<double>(i) / steps);
      Eigen::Vector3d navigation = base + context.center_offset;
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
      if (context.robot_type == mgg::RobotType::kGroundRobot) {
        Eigen::Vector3d contact = base;
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

  // The caller releases planner_mutex_ before this bounded wait. Snapshot and
  // odometry callbacks therefore cannot occupy executor threads while blocked
  // behind the planning callback. The exact authority is rechecked below.
  auto future = indexed_map_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(
          indexed_map_query_timeout_s_)) != std::future_status::ready) {
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
  for (std::size_t i = 0; i < count; ++i) {
    if (response->occupancy[i] !=
        mgg_msgs::srv::QueryMapBatch::Response::FREE) {
      return fail(mgg::PlanningStatus::kBlocked,
                  "indexed map route is occupied or unknown");
    }
    if (context.robot_type != mgg::RobotType::kAerialRobot &&
        (!std::isfinite(response->ground_z[i]) ||
         !std::isfinite(response->roughness[i]) ||
         response->roughness[i] > indexed_map_max_roughness_m_ ||
         !std::isfinite(response->clearance[i]) ||
        response->clearance[i] < component_body.z() || response->step[i] ||
         response->drop[i])) {
      return fail(mgg::PlanningStatus::kBlocked,
                  "indexed map terrain or clearance is unsupported");
    }
    if (context.robot_type != mgg::RobotType::kAerialRobot) {
      if (expected_ground_z.size() != count ||
          std::abs(expected_ground_z[i] - response->ground_z[i]) >
          indexed_map_ground_tolerance_m_ + 1e-9) {
        return fail(mgg::PlanningStatus::kBlocked,
                    "indexed map ground does not support the emitted body height");
      }
      if (i > 0) {
        const double dx = request->samples[i].x - request->samples[i - 1].x;
        const double dy = request->samples[i].y - request->samples[i - 1].y;
        const double dz =
            std::abs(response->ground_z[i] - response->ground_z[i - 1]);
        const double inclination = std::atan2(dz, std::hypot(dx, dy));
        if (dz > context.max_step_height + 1e-6 &&
            inclination > context.max_inclination + 1e-6) {
          return fail(mgg::PlanningStatus::kBlocked,
                      "indexed map route exceeds platform step or inclination limits");
        }
      }
    }
  }
  {
    const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
    const auto& snapshot = context.mapping_snapshot;
    if (!have_mapping_snapshot_ ||
        mapping_snapshot_.component_id != snapshot.component_id ||
        mapping_snapshot_.epoch != snapshot.epoch ||
        mapping_snapshot_.graph_revision != snapshot.graph_revision ||
        mapping_snapshot_.geometry_revision != snapshot.geometry_revision ||
        mapping_snapshot_.source_stamp.sec != snapshot.source_stamp.sec ||
        mapping_snapshot_.source_stamp.nanosec != snapshot.source_stamp.nanosec ||
        !component_from_navigation_.matrix().isApprox(
            context.component_from_navigation.matrix(), 1e-9)) {
      return fail(mgg::PlanningStatus::kStaleRevision,
                  "indexed mapping authority changed during query");
    }
  }
  path.indexed_map_validated = true;
  return true;
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
  if (corridor.status != mgg::PlanningStatus::kSucceeded) return result;

  result.poses = corridor.poses;
  if (corridor.request.objective == mgg::ObjectiveKind::kExplore) {
    // Explore corridors come only from buildLocalGraph's selector.  Their
    // edges have already passed graph_expansion's ground/body policy, which
    // deliberately permits a hanging local root and unknown beyond observed
    // rays during cold start.  Rechecking with the strict explicit-objective
    // policy here deadlocks a stationary robot whose lidar cannot see beneath
    // itself. The caller applies the optional indexed query after releasing
    // planner_mutex_.
    convertPathToNavigationBase(result);
    return result;
  }

  const Eigen::Vector3d body = robot_params_.getPlanningSize();
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
  const auto project = [this, body, center_offset, current_anchor, home_anchor,
                        have_connected_home_anchor, request_targets_home_anchor,
                        &corridor, &samePosition,
                        &geofenceContains](mgg::StateVec& state) {
    if (robot_params_.type == mgg::RobotType::kGroundRobot) {
      const bool exact_explicit_goal =
          (corridor.request.objective == mgg::ObjectiveKind::kNavigate ||
           corridor.request.objective == mgg::ObjectiveKind::kReturnHome) &&
          (state.head<2>() - corridor.request.goal.pose.head<2>())
                  .cwiseAbs().maxCoeff() <= 1e-6;
      if (!ground_ ||
          !projectStateToDrivingHeight(state, exact_explicit_goal)) {
        // Only two unsupported coordinates carry physical provenance: the
        // robot's current footprint and the retained initial Home anchor after
        // that anchor gained a checked connection. Arbitrary unsupported
        // goals and intermediate grid cells still fail closed.
        if (samePosition(state, current_state_) ||
            samePosition(state, current_anchor)) {
          state = current_anchor;
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
        if (map_->getBoxStatus(anchor_center, body,
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
    const Eigen::Vector3d center = state.head<3>() + center_offset;
    const mgg::VoxelStatus body_status = map_->getStrictBoxStatus(center, body);
    if (body_status == mgg::VoxelStatus::kOccupied) {
      return mgg::GridProjectionStatus::kBodyOccupied;
    }
    if (body_status != mgg::VoxelStatus::kFree) {
      return mgg::GridProjectionStatus::kBodyUnknown;
    }
    if (!geofenceContains(center)) {
      return mgg::GridProjectionStatus::kGeofenceViolation;
    }
    return mgg::GridProjectionStatus::kSupported;
  };
  const auto traverse =
      [this, body, center_offset, current_anchor, home_anchor,
       have_connected_home_anchor, &samePosition,
       &geofenceAllows](const mgg::StateVec& a, const mgg::StateVec& b,
                        std::vector<mgg::StateVec>& checked) {
        checked.clear();
        const bool a_provenance =
            samePosition(a, current_anchor) ||
            (have_connected_home_anchor && samePosition(a, home_anchor));
        const bool b_provenance =
            samePosition(b, current_anchor) ||
            (have_connected_home_anchor && samePosition(b, home_anchor));
        mgg::StateVec projected_a = a;
        mgg::StateVec projected_b = b;
        const bool a_anchor =
            a_provenance && !projectStateToDrivingHeight(projected_a);
        const bool b_anchor =
            b_provenance && !projectStateToDrivingHeight(projected_b);
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
          mgg::StateVec projected_mapped = mapped;
          if (!projectStateToDrivingHeight(projected_mapped) ||
              !samePosition(projected_mapped, mapped) ||
              !validateObjectiveStartSupport(anchor, mapped, checked)) {
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
          if (map_->getStrictPathStatus(body_from, body_to, body) !=
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
        if (ground_->getProjectedEdgeStatus(a.head<3>(), b.head<3>(), body,
                                            true, projected, false) !=
                mgg::ProjectedEdgeStatus::kAdmissible ||
            projected.size() < 2) {
          return false;
        }
        checked.reserve(projected.size());
        for (const Eigen::Vector3d& driving_pose : projected) {
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
              map_->getStrictPathStatus(checked_from, checked_to, body) !=
                  mgg::VoxelStatus::kFree) {
            return false;
          }
        }
        return true;
      };
  const mgg::GridRefinementLimits& active_limits =
      limits != nullptr ? *limits : grid_refinement_limits_;
  mgg::BoundedGridPlanner grid_planner(current_state_, active_limits, project,
                                       traverse);
  result = grid_planner.refine(corridor);
  if (result.status != mgg::PlanningStatus::kSucceeded &&
      !objective_start_support_failure_.empty()) {
    result.reason += " [start connector: " +
                     objective_start_support_failure_ + "]";
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

void PlannerNode::onObjectiveRequest(
    const std::shared_ptr<mgg_msgs::srv::PlanObjective::Request> request,
  std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> response) {
  std::unique_lock<std::recursive_mutex> lock(planner_mutex_);
  auto map_read = mola_map_ != nullptr ? mola_map_->acquireReadLease()
                                       : mgg::MolaMap::ReadLease{};
  refreshMolaRevision();
  const std::uint64_t mola_generation_at_start =
      mola_map_ != nullptr ? mola_map_->activeGeneration() : 0;
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
  mgg::RouteCorridor corridor;
  corridor.request = core;
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
      have_mapping_snapshot_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    mapping_snapshot_received_).count() <=
          indexed_map_snapshot_ttl_s_;
  const bool mapping_key_matches =
      mapping_authority_fresh &&
      core.component_id == mapping_snapshot_.component_id &&
      core.map_epoch == mapping_snapshot_.epoch &&
      core.mapping_graph_revision == mapping_snapshot_.graph_revision &&
      core.geometry_revision == mapping_snapshot_.geometry_revision &&
      core.map_source_stamp_sec == mapping_snapshot_.source_stamp.sec &&
      core.map_source_stamp_nanosec == mapping_snapshot_.source_stamp.nanosec;
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
      corridor.request = core;
      corridor.poses = best_path_;
      corridor.status = best_path_.empty() ? mgg::PlanningStatus::kUnreachable
                                           : mgg::PlanningStatus::kSucceeded;
      corridor.reason = best_path_.empty() ? summary : "";
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
      bool home_goal_supported = true;
      if (core.objective == mgg::ObjectiveKind::kReturnHome &&
          !projectStateToDrivingHeight(graph_request.goal.pose, true)) {
        home_goal_supported = false;
        const mgg::Vertex* root = global_graph_->getVertex(0);
        const double home_delta =
            have_initial_state_
                ? (core.goal.pose.head<3>() - initial_state_.head<3>())
                      .cwiseAbs()
                      .maxCoeff()
                : std::numeric_limits<double>::infinity();
        const double home_association_tolerance =
            std::clamp(2.0 * map_->getResolution(), 0.01, 0.25);
        const bool requests_physical_home =
            have_initial_state_ && root != nullptr &&
            (home_delta <= 1e-3 ||
             (!core.goal.landmark_id.empty() &&
              home_delta <= home_association_tolerance));
        if (initial_anchor_supported_ && requests_physical_home) {
          graph_request.goal.pose = root->state;
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
        if (projectStateToDrivingHeight(projected_goal, true)) {
          graph_request.goal.pose = projected_goal;
        }
        mgg::TopologicalGoalPlanner objective_planner(
            core.component_id, core.graph_revision, core.map_revision, 1.0,
            core.objective == mgg::ObjectiveKind::kNavigate
                ? partial_route_min_progress_m_
                : 0.0);
        corridor =
            objective_planner.plan(graph, graph_current, graph_request);
        // Graph lookup uses driving height, while refinement and the response
        // retain the caller's exact base-pose goal.
        corridor.request = core;
      }
    }
  }

  const mgg::GridRefinementLimits* objective_limits =
      core.objective == mgg::ObjectiveKind::kNavigate
          ? &objective_grid_limits_
          : nullptr;
  const auto objective_refinement_started = std::chrono::steady_clock::now();
  mgg::FeasiblePath path = refineCorridor(corridor, objective_limits);
  if (core.objective == mgg::ObjectiveKind::kNavigate &&
      path.status != mgg::PlanningStatus::kSucceeded &&
      !corridor.poses.empty()) {
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
      path = refineCorridor(direct, &remaining);
    }
  }
  const IndexedQueryContext query_context = indexedQueryContext();
  map_read = mgg::MolaMap::ReadLease{};
  lock.unlock();
  if (path.status == mgg::PlanningStatus::kSucceeded) {
    queryIndexedMap(path, query_context);
  }
  if (mola_map_ != nullptr) {
    lock.lock();
    map_read = mola_map_->acquireReadLease();
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
