#include "mgg_ros/planner_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <random>
#include <regex>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.hpp>

#include "mgg_core/log.h"
#include "mgg_core/path_turns.h"
#include "mgg_core/trajectory.h"
#include "mgg_ros/conversions.h"
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
    auto backend = std::make_unique<mgg::MolaMap>(map_cfg);
    mola_map_ = backend.get();
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
  local_graph_->setRobotId(static_cast<int>(planning_params_.robot_id));
  global_graph_->setRobotId(static_cast<int>(planning_params_.robot_id));
  // The global graph expansion (rrg.cpp:80 to 82) samples in the local box
  // the lattice is built in, seeded as upstream's sampler was
  // (random_sampler.cpp:242).
  random_sampler_.setBound(grid_params_.min_val, grid_params_.max_val);
  random_sampler_.reset(std::random_device{}());

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
  if (mola_map_ != nullptr) {
    mapping_snapshot_sub_ = create_subscription<mgg_msgs::msg::MappingSnapshot>(
        "mapping_snapshot", rclcpp::QoS(1).transient_local(),
        [this](mgg_msgs::msg::MappingSnapshot::ConstSharedPtr m) {
          onMappingSnapshot(m);
        },
        sub_opts);
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
  peer_bodies_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "peer_bodies", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr m) {
        onPeerBodies(m);
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

  build_srv_ = create_service<std_srvs::srv::Trigger>(
      "build_local_graph",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        onBuildRequest(req, res);
      },
      rclcpp::ServicesQoS(), callback_group_);

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
  // Upstream's lattice only admits a cell whose whole body volume is known
  // free (rrg.cpp buildGridGraphExapnd, getBoxStatus with stop_at_unknown),
  // on a map that carves free space from every scan. A map that carves it
  // from keyframes only leaves a parked robot's surroundings largely
  // unobserved, so it would never take the first step. This admits a cell
  // whose body volume is partly unobserved; known occupied volume still
  // rejects it, and mapped ground under it and along the edge stays
  // mandatory. Off by default; simulation with a keyframe map turns it on.
  allow_unknown_lattice_body_ = declareOrGet<bool>(
      this, "allow_unknown_lattice_body", allow_unknown_lattice_body_);
  // A lidar does not see the ground under the robot: within its blind radius
  // a keyframe map holds no floor, so the graph root has no support until
  // the robot has driven away from where it stands. The root then sits at
  // the physical driving height (base height above the assumed floor) as a
  // hanging vertex, and one edge from it may reach up to this far to the
  // first supported vertex; every other check on that edge still applies.
  hanging_root_edge_length_max_ = std::max(
      0.0, declareOrGet<double>(this, "hanging_root_edge_length_max",
                                hanging_root_edge_length_max_));

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
  ctx.allow_unknown_lattice_body = allow_unknown_lattice_body_;
  ctx.hanging_root_edge_length_max = hanging_root_edge_length_max_;
  // A hanging root sits at the physical driving height by construction;
  // projecting it again onto absent ground would only fail.
  ctx.preserve_hanging_root_start_height = hanging_root_edge_length_max_ > 0.0;
  ctx.root_footprint_exempt = true;
  return ctx;
}

mgg::ExpandContext PlannerNode::makeGlobalContext() {
  mgg::ExpandContext ctx = makeContext();
  // Inclinations are keyed by local lattice ids; the roadmap has its own.
  ctx.inclinations = nullptr;
  // A roadmap edge must have been seen traversable (rrg.cpp:725).
  ctx.stop_at_unknown = true;
  // Vertex zero of the roadmap is home, not the robot.
  ctx.root_footprint_exempt = false;
  return ctx;
}

mgg::Vertex* PlannerNode::findGlobalVertex(int id) const {
  const auto it = global_graph_->vertices_map_.find(id);
  return it == global_graph_->vertices_map_.end() ? nullptr : it->second;
}

mgg::MolaMap::ReadLease PlannerNode::mapReadLease() const {
  return mola_map_ != nullptr ? mola_map_->acquireReadLease()
                              : mgg::MolaMap::ReadLease{};
}

void PlannerNode::refreshMapRevision() {
  if (mola_map_ == nullptr) return;
  const std::uint64_t generation = mola_map_->activeGeneration();
  if (generation == observed_map_generation_) return;
  observed_map_generation_ = generation;
  ++map_revision_;
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
    // Upstream scored global frontiers against the world-fixed global bound
    // (computeVolumetricGainRayModelNoBound, rrg.cpp:3767). This port centres
    // its gain volume on the robot every cycle, so a frontier a street away
    // would count nothing; centre it on the frontier while it is scored.
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
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  // The subscription is reentrant and every callback waits on the planner
  // mutex, so messages can be handled out of order: an older one must not
  // overwrite a newer state.
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
  last_odometry_received_ = std::chrono::steady_clock::now();

  auto map_read = mapReadLease();
  refreshMapRevision();
  seedGlobalGraph();
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
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  Eigen::Isometry3d component_from_navigation = Eigen::Isometry3d::Identity();
  component_from_navigation.linear() =
      Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
  component_from_navigation.translation() = Eigen::Vector3d(t.x, t.y, t.z);
  mapping_snapshot_ = *msg;
  have_mapping_snapshot_ = true;
  refreshMapRevision();
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
  mola_map_->setTransientDiscs(std::move(centres), peer_body_radius_m_,
                               peer_body_ttl_s_);
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
    ++graph_revision_;
    RCLCPP_INFO(get_logger(),
                "robot %d's transform was withdrawn: its roadmap is "
                "quarantined (%d edges cut) until it is placed again",
                robot, cut);
  }
}

mgg::ReceiverPlatform PlannerNode::receiverPlatform() const {
  mgg::ReceiverPlatform platform;
  if (robot_params_.type == mgg::RobotType::kGroundRobot) {
    platform.driving_height = planning_params_.max_ground_height;
    platform.max_step_height = planning_params_.max_step_height;
    platform.max_inclination = planning_params_.max_inclination;
  }
  return platform;
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
      *global_graph_, incoming, *poses_, admissible, 5.0, receiverPlatform());
  if (r.vertices_added > 0 || r.edges_added > 0 || r.vertices_replaced > 0 ||
      r.neighbour_restarted) {
    ++graph_revision_;
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
    // A lone root is a landmark, not a traversability claim: capture it from
    // the first odometry, before anything moves the robot away from home.
    // Until the map shows ground under it no edge attaches to it.
    mgg::StateVec root_state = current_state_;
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
                "global graph seeded at (%.2f, %.2f, %.2f)%s", root_state[0],
                root_state[1], root_state[2],
                global_root_supported_ ? "" : " (awaiting mapped support)");
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

void PlannerNode::ingestOdometryIntoGlobalGraph() {
  if (!have_odometry_ || global_graph_->getNumVertices() == 0) return;
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
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  auto map_read = mapReadLease();
  refreshMapRevision();
  if (planner_trigger_count_ == 0) return;
  if (!have_odometry_ || !map_->getStatus()) return;
  // The sampler draws around the unvisited clusters of the global graph and
  // tests against the map and the robot's trail. When none of those changed
  // since a pass that added nothing, another pass spends its whole budget
  // (rrg.cpp:2608) rediscovering that nothing is admissible: a parked robot
  // would burn kGlobalGraphUpdateTimeBudget every period indefinitely.
  if (graph_revision_ == expansion_graph_revision_ &&
      map_revision_ == expansion_map_revision_ &&
      (current_state_.head<3>() - expansion_state_.head<3>()).norm() <
          kOdometryStillM) {
    return;
  }
  expansion_graph_revision_ = graph_revision_;
  expansion_map_revision_ = map_revision_;
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
  const int before = global_graph_->getNumVertices();
  const mgg::ExpandContext ctx = makeGlobalContext();
  std::vector<mgg::Vertex*> added_vertices;
  const bool added =
      lattice.empty()
          ? mgg::addRefPathToGraph(*global_graph_, path, ctx,
                                   global_vertex_spacing_, &added_vertices)
          : mgg::addRefPathToGraph(*global_graph_, lattice, ctx,
                                   global_vertex_spacing_, &added_vertices);
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

// ---------------------------------------------------------------------------
// Paths

void PlannerNode::shortcutAndResample(std::vector<mgg::StateVec>& path) {
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
  const mgg::ExpandContext ctx = makeContext();
  const auto segment_free = [this, &ctx](const Eigen::Vector3d& from,
                                         const Eigen::Vector3d& to) {
    // stop_at_unknown_voxel is true: a shortcut may only cross space already
    // known to be free. Upstream passes false here (rrg.cpp:4610), which
    // treats unknown space as passable; that is survivable there because its
    // shortcut only ever collapses a node when the segment leading to it is
    // under half a metre. Applied to a general shortcut it is not: a partly
    // explored map is mostly unknown, so every candidate line qualifies and
    // the path collapses to a straight run through whatever has not been
    // seen yet. A ground robot's segment also follows the terrain (steps,
    // inclination) as every graph edge does.
    if (robot_params_.type == mgg::RobotType::kGroundRobot) {
      std::vector<Eigen::Vector3d> projected;
      return ground_->getProjectedEdgeStatus(from, to, ctx.robot_box_size,
                                             true, projected, false) ==
             mgg::ProjectedEdgeStatus::kAdmissible;
    }
    return map_->getPathStatus(from, to, ctx.robot_box_size, true) ==
           mgg::VoxelStatus::kFree;
  };
  mgg::PathType points;
  points.reserve(path.size());
  for (const mgg::StateVec& s : path) points.push_back(s.head(3));
  points = mgg::shortcutPath(points, segment_free);
  path_shortcut_corners_ = static_cast<int>(points.size());
  mgg::PathType resampled;
  if (planning_params_.path_interpolation_distance > 0.0 &&
      mgg::interpolatePath(points, planning_params_.path_interpolation_distance,
                           resampled) &&
      resampled.size() >= 2) {
    // interpolatePath stops short of the last point by up to one step; the
    // route ends where it was planned to, which for an objective is the
    // exact goal.
    if ((resampled.back() - points.back()).norm() > 1e-6) {
      resampled.push_back(points.back());
    }
    points = resampled;
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
  auto map_read = mapReadLease();
  refreshMapRevision();
  best_path_.clear();
  best_path_from_global_graph_ = false;
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

  const mgg::ExpandContext ctx = makeContext();
  const auto t_global = Clock::now();
  // The lattice is laid out around the root at driving height, where the
  // root vertex is; a ground robot's odometry origin sits lower than that.
  const mgg::GridGraphResult r = buildGridGraph(
      *local_graph_, root_state, grid_params_, ctx, current_state_[3]);
  if (r.status == mgg::GridGraphStatus::kInvalidBounds) {
    return "grid bounds invalid: min_val must be <= 0, max_val >= 0 and "
           "resolution non-zero";
  }
  const auto t_grid = Clock::now();
  // Global space is defined in world frame and must remain static at world origin.
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
  // The graph's frontiers are worth keeping whether or not a path is chosen.
  add_frontiers_to_global_graph_ = local_graph_->getNumVertices() > 1;

  const auto t_gain = Clock::now();
  // Toward a target the robot has no route to yet, the direction the paths
  // are penalised for leaving is the one to the target, not the last path's.
  // path_direction_penalty keeps it a preference: a path round an obstacle
  // that first heads away is discounted, not refused.
  const double selection_direction =
      exploration_target_.has_value()
          ? std::atan2(exploration_target_->y() - current_state_.y(),
                       exploration_target_->x() - current_state_.x())
          : exploring_direction_;
  // A ground robot turns sharply only on level ground with room to turn in
  // place; a path that turns elsewhere is taken only when no other path
  // would be.
  mgg::PathTurnCheck turn_check(
      *local_graph_, robot_params_, [this](const mgg::Vertex& v) {
        return mgg::turnClear(*map_, robot_params_, v.state);
      });
  mgg::PathTurnsFn turns_admissible;
  if (robot_params_.type == mgg::RobotType::kGroundRobot) {
    turns_admissible = std::ref(turn_check);
  }
  const mgg::PathSelectionResult sel = mgg::selectBestPath(
      *local_graph_, planning_params_, robot_params_, edge_inclinations_,
      map_->getResolution(), selection_direction, selectionExclusions(),
      reservation_exclusion_radius_m_,
      [this](const mgg::Vertex& v) {
        return mgg::viewpointClear(*map_, robot_params_, planning_params_,
                                   v.state);
      },
      turns_admissible);
  for (const mgg::Vertex* v : sel.best_path) {
    if (v != nullptr) best_path_.push_back(v->state);
  }
  if (sel.sharp_turn_fallback) {
    ++sharp_turn_fallbacks_;
    RCLCPP_WARN(get_logger(),
                "exploration path to (%.2f, %.2f, %.2f) turns sharply on a "
                "slope or without room to turn: no path turns only where it "
                "may (%d such paths so far)",
                best_path_.back().x(), best_path_.back().y(),
                best_path_.back().z(), sharp_turn_fallbacks_);
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

  // rrg.cpp:4538: the accepted lattice path joins the global graph as its
  // vertices, before the shortcut turns it into poses.
  if (best_path_.size() >= 2) addRefPathToGraph(best_path_);
  shortcutAndResample(best_path_);
  if (best_path_.size() >= 2) {
    // Remember where this path is heading, so the next cycle penalises
    // doubling back.
    std::vector<Eigen::Vector3d> points;
    points.reserve(best_path_.size());
    for (const mgg::StateVec& s : best_path_) points.push_back(s.head(3));
    exploring_direction_ = mgg::estimateDirectionFromPath(points);
  }

  // Free cells with no vertices is the characteristic bring-up failure: the
  // lattice is finding space but every candidate is being turned away. The
  // reason breakdown is the only thing that separates a geometry mistake from
  // a genuinely blocked robot, so report it whenever it happens.
  char why[160] = "";
  if (r.vertices_added == 0 && r.free_cells > 0) {
    std::snprintf(why, sizeof(why),
                  " (rejected: %d collision, %d no ground; edges: %d ok, "
                  "%d steep, %d occupied, %d unmapped, %d hanging, %d "
                  "cross-slope)",
                  r.rejected[static_cast<int>(mgg::ExpandGraphStatus::
                                                  kErrorCollisionEdge)],
                  r.no_ground, r.edge_status[0], r.edge_status[1],
                  r.edge_status[2], r.edge_status[3], r.edge_status[4],
                  r.edge_status[5]);
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
  char buf[768];
  std::snprintf(
      buf, sizeof(buf),
      "grid graph: %d free cells, %d vertices, %d edges%s%s; %d viewpoints, "
      "%d frontiers; best path %zu poses (%d lattice -> %d corners -> %d "
      "resampled), gain %.1f%s; viewpoint clearance: %d paths pulled back, "
      "%d without%s; sharp turns: %d paths refused (%d on a slope, %d "
      "without room)%s; "
      "heading %.2f rad%s",
      r.free_cells, r.vertices_added, r.edges_added,
      r.hit_limit ? " (hit a size limit)" : "", why, evaluated, frontiers,
      best_path_.size(), path_shortcut_from_, path_shortcut_corners_,
      path_shortcut_to_, sel.best_gain,
      sel.paths_rejected_steep > 0 ? " (some paths too steep)" : "",
      sel.paths_pulled_back, sel.paths_without_clear_viewpoint,
      sel.unclear_viewpoint ? ", none ends clear" : "",
      sel.paths_with_sharp_turns, turn_check.refused_on_slope,
      turn_check.refused_without_room,
      sel.sharp_turn_fallback ? ", none complies" : "", exploring_direction_,
      timing);

  // rrg.cpp:2098 to 2120: rounds without a frontier among the leaves count
  // towards the global planner; a round with one counts back.
  if (local_graph_->getNumVertices() <= 1) {
    // Nothing was scored; this round says nothing about the frontier.
  } else if (frontiers == 0) {
    ++low_gain_rounds_;
  } else if (low_gain_rounds_ > 0) {
    --low_gain_rounds_;
  }
  return std::string(buf);
}

bool PlannerNode::routeOverGlobalGraph(const mgg::StateVec& goal,
                                       double goal_tolerance,
                                       std::vector<mgg::StateVec>& path,
                                       std::string& reason) {
  path.clear();
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
  const mgg::ExpandContext ctx = makeGlobalContext();
  const int before = global_graph_->getNumVertices();
  mgg::Vertex* link_vertex =
      mgg::connectStateToGraph(*global_graph_, current, ctx, kLinkRadius,
                               /*exact_state=*/false);
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
    global_graph_->findShortestPaths(link_vertex->id, rep);
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
  if (goal_tolerance <= 0.0 ||
      !global_graph_->getNearestVertexInRange(&goal, goal_tolerance,
                                              &goal_vertex) ||
      goal_vertex == nullptr) {
    exact_goal = true;
    const int before_goal = global_graph_->getNumVertices();
    if (projectToDrivingHeight(goal_state)) {
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
        *global_graph_, goal_state, grid_params_, ctx, current_state_[3],
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
  if (goal_vertex->id == link_vertex->id) {
    reason = "already at the goal";
    return false;
  }
  if (!reaches(*goal_vertex)) {
    reason = "no route over the global graph reaches the goal";
    return false;
  }
  global_graph_->getShortestPath(goal_vertex->id, rep, true, path);
  if (path.size() < 2) {
    reason = "no route over the global graph reaches the goal";
    return false;
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

bool PlannerNode::routeOverLocalLattice(const mgg::StateVec& goal,
                                        std::vector<mgg::StateVec>& path,
                                        std::string& reason) {
  path.clear();
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
  local_graph_->reset();
  edge_inclinations_.clear();
  mgg::StateVec root_state = current_state_;
  const bool root_hanging = !projectToDrivingHeight(root_state);
  if (root_hanging) root_state = physicalAnchorAtDrivingHeight(current_state_);
  auto* root = new mgg::Vertex(0, root_state);
  root->robot_id = static_cast<int>(planning_params_.robot_id);
  root->is_hanging = root_hanging;
  local_graph_->addVertex(root);
  const mgg::ExpandContext ctx = makeContext();
  const mgg::GridGraphResult r = buildGridGraph(
      *local_graph_, root_state, grid_params_, ctx, current_state_[3]);
  if (r.status == mgg::GridGraphStatus::kInvalidBounds ||
      local_graph_->getNumVertices() <= 1) {
    reason = "the local lattice holds no admissible cell";
    return false;
  }
  mgg::StateVec goal_state = goal;
  if (!projectToDrivingHeight(goal_state)) {
    reason = "no mapped ground under the goal";
    return false;
  }
  mgg::Vertex* goal_vertex = mgg::connectStateToGraph(
      *local_graph_, goal_state, ctx, kGoalLinkRadius, /*exact_state=*/true);
  if (goal_vertex == nullptr) {
    reason = "goal cannot be linked to the local lattice";
    return false;
  }
  mgg::ShortestPathsReport rep;
  if (!local_graph_->findShortestPaths(0, rep) || !rep.status ||
      (goal_vertex->id != 0 &&
       rep.parent_id_map.find(goal_vertex->id) == rep.parent_id_map.end())) {
    reason = "no route through the local lattice reaches the goal";
    return false;
  }
  local_graph_->getShortestPath(goal_vertex->id, rep, true, path);
  if (path.size() < 2) {
    reason = "already at the goal";
    return false;
  }
  return true;
}

bool PlannerNode::runGlobalPlanner(int target_id, std::string& reason) {
  best_path_.clear();
  best_path_from_global_graph_ = false;
  if (global_graph_->getNumVertices() <= 1) {
    // rrg.cpp:5582.
    reason = "the global graph holds no frontier to reposition to";
    return false;
  }
  mgg::Vertex* target = nullptr;
  if (target_id >= 0) {
    // Resuming the repositioning that was under way (rrg.cpp:5556).
    target = findGlobalVertex(target_id);
    if (target == nullptr || target->type != mgg::VertexType::kFrontier) {
      target = nullptr;  // it was reached or demoted meanwhile: choose anew
    }
  }
  if (target == nullptr) {
    mgg::StateVec current = current_state_;
    if (!projectToDrivingHeight(current)) {
      current = physicalAnchorAtDrivingHeight(current_state_);
    }
    const int before = global_graph_->getNumVertices();
    mgg::Vertex* link_vertex = mgg::connectStateToGraph(
        *global_graph_, current, makeGlobalContext(), kLinkRadius,
        /*exact_state=*/false);
    if (global_graph_->getNumVertices() != before) ++graph_revision_;
    if (link_vertex == nullptr) {
      reason = "current pose cannot be linked to the global graph";
      return false;
    }
    const mgg::GlobalFrontierReport report = mgg::searchGlobalFrontier(
        *global_graph_, link_vertex->id,
        static_cast<int>(planning_params_.robot_id), globalFrontierGain(),
        selectionExclusions(), reservation_exclusion_radius_m_,
        exploration_target_.has_value() ? &*exploration_target_ : nullptr);
    global_space_.setCenter(current_state_, /*use_extension=*/true);
    if (report.best_frontier == nullptr) {
      // rrg.cpp:5628 and 5759: no frontier, or none the graph can reach.
      char why[192];
      std::snprintf(why, sizeof(why),
                    "%d global frontier(s), none reachable with gain (%d "
                    "re-checked out)",
                    report.frontiers, report.demoted);
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
  // Route to it over the global graph (rrg.cpp:5846), whole.
  if (!routeOverGlobalGraph(target->state, 1e-3, best_path_, reason)) {
    return false;
  }
  shortcutAndResample(best_path_);
  // The frontier is a lattice leaf of an earlier cycle, which may stand
  // against a wall: the route ends where the robot has room (viewpointClear)
  // when any pose along it has, and at the frontier as before otherwise.
  if (best_path_.size() >= 2 &&
      !mgg::pullBackToClearViewpoint(
          best_path_, [this](const mgg::StateVec& pose) {
            return mgg::viewpointClear(*map_, robot_params_, planning_params_,
                                       pose);
          })) {
    ++unclear_viewpoints_selected_;
    RCLCPP_WARN(get_logger(),
                "global route ends without viewpoint clearance at (%.2f, "
                "%.2f, %.2f): no pose along it is clear (%d unclear "
                "viewpoints so far)",
                best_path_.back().x(), best_path_.back().y(),
                best_path_.back().z(), unclear_viewpoints_selected_);
  }
  best_path_from_global_graph_ = true;
  // rrg.cpp:5838 to 5843: this frontier is the target until it is reached.
  current_global_vertex_id_ = target->id;
  global_exploration_ongoing_ = true;
  return true;
}

// ---------------------------------------------------------------------------
// Services

void PlannerNode::onBuildRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  response->message = buildLocalGraph();
  response->success = local_graph_->getNumVertices() > 1;
  publishPath();
  publishMarkers();
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void PlannerNode::onPlanRequest(
    const std::shared_ptr<mgg_msgs::srv::PlannerSrv::Request> request,
    std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response) {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  withdrawUnplacedNeighbours();
  response->planning_bound_mode = request->bound_mode;
  if (!have_odometry_ || !map_->getStatus()) {
    response->status = kStatusNotReady;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "plan request refused: %s",
                         have_odometry_ ? "no map" : "no odometry");
    return;
  }
  // The planner's state is its last odometry message; a plan from where the
  // robot was is completed at once by the controller where the robot is.
  if (secondsSince(last_odometry_received_) > odometry_stale_s_) {
    response->status = kStatusNotReady;
    RCLCPP_WARN(get_logger(), "plan request refused: odometry is %.1f s old",
                secondsSince(last_odometry_received_));
    return;
  }
  // A caller may pin the bound mode for this cycle, e.g. to squeeze through a
  // gap it would normally refuse.
  const mgg::BoundModeType previous = robot_params_.bound_mode;
  robot_params_.bound_mode =
      static_cast<mgg::BoundModeType>(request->bound_mode);
  // Every cycle plans afresh; the previous path was the previous answer.
  best_path_.clear();
  best_path_from_global_graph_ = false;

  std::string summary;
  std::string reason;
  bool complete = false;
  // rrg.cpp:1229 to 1240: a global repositioning under way is resumed until
  // the robot is within reach of its frontier; then local exploration
  // takes over again.
  bool resume_global = false;
  if (global_exploration_ongoing_) {
    const mgg::Vertex* target = findGlobalVertex(current_global_vertex_id_);
    if (target != nullptr &&
        (current_state_.head<3>() - target->state.head<3>()).norm() >
            global_frontier_reach_m_) {
      resume_global = true;
    } else {
      global_exploration_ongoing_ = false;
    }
  }
  if (resume_global) {
    auto map_read = mapReadLease();
    refreshMapRevision();
    if (runGlobalPlanner(current_global_vertex_id_, reason)) {
      summary = "resuming global repositioning";
    } else {
      global_exploration_ongoing_ = false;
      summary = "global repositioning abandoned: " + reason;
    }
  }
  if (best_path_.empty()) {
    summary = buildLocalGraph();
    // An empty lattice is a map that does not yet show the robot's
    // surroundings, not an explored one: PCI retries as the map grows. The
    // global planner is consulted once the lattice existed and saw nothing
    // new for long enough.
    if (best_path_.empty() && local_graph_->getNumVertices() > 1 &&
        low_gain_rounds_ >= auto_global_planner_low_gain_rounds_) {
      // No leaf with gain for long enough: the global planner routes to the
      // best global frontier (rrg.cpp:2119, mggplanner.cpp:217).
      auto map_read = mapReadLease();
      low_gain_rounds_ = 0;
      if (runGlobalPlanner(-1, reason)) {
        summary += "; no local gain, repositioning over the global graph";
      } else {
        complete = true;
        summary += "; exploration complete: " + reason;
      }
    }
  }
  robot_params_.bound_mode = previous;

  response->status = !best_path_.empty()
                         ? mgg_msgs::srv::PlannerSrv::Response::FORWARD
                     : complete ? kStatusComplete
                                : kStatusNoPath;
  for (const mgg::StateVec& s : best_path_) {
    response->path.push_back(toPoseMsg(s));
  }
  publishPath();
  publishMarkers();
  RCLCPP_INFO(get_logger(), "plan request: %s", summary.c_str());
}

void PlannerNode::onExplorationTargetRequest(
    const std::shared_ptr<mgg_msgs::srv::PlannerSetExplorationTarget::Request>
        request,
    std::shared_ptr<mgg_msgs::srv::PlannerSetExplorationTarget::Response>
        response) {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (!request->active) {
    if (exploration_target_.has_value()) {
      RCLCPP_INFO(get_logger(), "exploration target cleared");
    }
    exploration_target_.reset();
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
  response->success = true;
}

void PlannerNode::onObjectiveRequest(
    const std::shared_ptr<mgg_msgs::srv::PlanObjective::Request> request,
    std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> response) {
  using Service = mgg_msgs::srv::PlanObjective;
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
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
  if (!have_odometry_ || !map_->getStatus()) {
    response->status = Service::Response::BLOCKED;
    response->reason = have_odometry_ ? "map unavailable" : "no odometry";
    return;
  }
  if (secondsSince(last_odometry_received_) > odometry_stale_s_) {
    response->status = Service::Response::BLOCKED;
    response->reason = "odometry is stale";
    return;
  }
  mgg::StateVec goal;
  double tolerance = 0.0;
  if (request->objective == Service::Request::RETURN_HOME) {
    const mgg::Vertex* home = findGlobalVertex(0);
    if (home == nullptr) {
      response->status = Service::Response::UNREACHABLE;
      response->reason = "no home recorded yet";
      return;
    }
    goal = home->state;
    tolerance = 1e-3;
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
  const bool local = request->objective == Service::Request::NAVIGATE &&
                     routeOverLocalLattice(goal, route, local_reason);
  if (!local && !routeOverGlobalGraph(goal, tolerance, route, reason)) {
    if (!local_reason.empty()) reason += "; local lattice: " + local_reason;
    response->status = Service::Response::UNREACHABLE;
    response->reason = reason;
    RCLCPP_WARN(get_logger(), "objective refused: %s", reason.c_str());
    return;
  }
  shortcutAndResample(route);
  response->status = Service::Response::SUCCEEDED;
  for (const mgg::StateVec& s : route) response->path.push_back(toPoseMsg(s));
  RCLCPP_INFO(get_logger(),
              "objective route: %zu poses to (%.2f, %.2f) over the %s (%d "
              "corners)",
              route.size(), goal.x(), goal.y(),
              local ? "local lattice" : "global graph", path_shortcut_corners_);
}

// ---------------------------------------------------------------------------
// Outputs

void PlannerNode::publishPath() {
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
