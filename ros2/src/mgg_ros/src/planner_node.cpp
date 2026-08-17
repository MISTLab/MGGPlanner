#include "mgg_ros/planner_node.h"

#include <chrono>
#include <cmath>
#include <cstdio>
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

  mgg::OctomapConfig map_cfg;
  map_cfg.resolution = declareOrGet<double>(this, "map.resolution", 0.2);
  map_cfg.max_range = declareOrGet<double>(this, "map.max_range", 20.0);
  map_ = std::make_unique<mgg::OctomapMap>(map_cfg);
  ground_ = std::make_unique<mgg::GroundProjection>(*map_, planning_params_);
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

  rclcpp::QoS cloud_qos(rclcpp::KeepLast(10));
  cloud_qos.best_effort();
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", cloud_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
        onPointCloud(m);
      },
      sub_opts);

  neighbour_sub_ = create_subscription<mgg_msgs::msg::Graph>(
      "neighbour_graph_in", rclcpp::QoS(10),
      [this](mgg_msgs::msg::Graph::ConstSharedPtr m) { onNeighbourGraph(m); },
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

  plan_srv_ = create_service<mgg_msgs::srv::PlannerSrv>(
      "mggplanner",
      [this](const std::shared_ptr<mgg_msgs::srv::PlannerSrv::Request> req,
             std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> res) {
        onPlanRequest(req, res);
      },
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
  return ctx;
}

void PlannerNode::onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  current_state_ = fromPoseMsg(msg->pose.pose);
  have_odometry_ = true;

  // The global graph is a trajectory backbone, so it has to be laid down as
  // the robot drives rather than once a planning cycle. A new vertex only
  // attaches to a parent within global_vertex_spacing * 5, and a cycle can be
  // forty seconds apart: the robot covers several metres in that time, ends up
  // beyond the attachment radius of everything, and the graph stays at the one
  // seed vertex for the whole run. That is silent - the seed is published, so
  // the exchange looks healthy - and it means two robots' graphs are two
  // isolated points that can never come close enough to merge. Measured: four
  // robots, 0.65 Hz of graph traffic, one vertex per message, zero merges.
  //
  // Only the distance test runs at odometry rate. The rest is behind it and
  // fires every global_vertex_spacing of travel, so the map queries and the
  // sweep over existing vertices happen a couple of times a metre.
  const Eigen::Vector3d here(current_state_[0], current_state_[1],
                             current_state_[2]);
  if (have_global_anchor_ &&
      (here - last_global_anchor_).norm() < global_vertex_spacing_) {
    return;
  }
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  updateGlobalGraph();
}

void PlannerNode::onPointCloud(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  if (msg->data.empty()) return;

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
    map_->insertPointCloud(points, origin);
  }
}

void PlannerNode::onNeighbourGraph(mgg_msgs::msg::Graph::ConstSharedPtr msg) {
  if (msg->vertices.empty()) return;
  const int sender = msg->vertices.front().robot_id;
  if (sender == static_cast<int>(planning_params_.robot_id)) return;

  // Merging reads the map to decide reachability and writes the global graph.
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);

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



std::string PlannerNode::buildLocalGraph() {
  // Held for the whole cycle: the map must not change under a planner that is
  // ray-casting through it. Point clouds arriving meanwhile queue up, and the
  // subscription's best-effort depth decides how many survive.
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (!have_odometry_) return "no odometry received yet";
  if (!map_->getStatus()) return "map is empty; no point cloud received yet";

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
    }
  }
  auto* root = new mgg::Vertex(0, root_state);
  root->robot_id = static_cast<int>(planning_params_.robot_id);
  root->is_hanging = root_hanging;
  local_graph_->addVertex(root);



  const mgg::ExpandContext ctx = makeContext();
  const auto t_global = Clock::now();
  const mgg::GridGraphResult r = buildGridGraph(
      *local_graph_, current_state_, grid_params_, ctx, current_state_[3]);

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
      map_->getResolution(), exploring_direction_);

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

  // Free cells with no vertices is the characteristic bring-up failure: the
  // lattice is finding space but every candidate is being turned away. The
  // reason breakdown is the only thing that separates a geometry mistake from
  // a genuinely blocked robot, so report it whenever it happens.
  char why[128] = "";
  if (r.vertices_added == 0 && r.free_cells > 0) {
    std::snprintf(why, sizeof(why),
                  " (rejected: %d collision, %d no ground; edges: %d ok, "
                  "%d steep, %d occupied, %d unmapped, %d hanging)",
                  r.rejected[static_cast<int>(mgg::ExpandGraphStatus::
                                                  kErrorCollisionEdge)],
                  r.no_ground, r.edge_status[0], r.edge_status[1],
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

void PlannerNode::updateGlobalGraph() {
  if (!have_odometry_) return;

  mgg::StateVec state = current_state_;
  // Drop the pose onto the terrain first, so global vertices sit where the
  // robot could actually stand. Matches what searchHomingPath does before
  // linking the current state in.
  if (robot_params_.type == mgg::RobotType::kGroundRobot) {
    Eigen::Vector3d pos(state[0], state[1], state[2]);
    mgg::VoxelStatus vs;
    const double ground_height = ground_->projectSample(pos, vs);
    if (vs != mgg::VoxelStatus::kOccupied) return;  // nothing to stand on yet
    state[0] = pos[0];
    state[1] = pos[1];
    state[2] = pos[2] - (ground_height - planning_params_.max_ground_height);
  }

  if (global_graph_->getNumVertices() == 0) {
    auto* root = new mgg::Vertex(0, state);
    root->robot_id = static_cast<int>(planning_params_.robot_id);
    global_graph_->addVertex(root);
    last_global_anchor_ = Eigen::Vector3d(state[0], state[1], state[2]);
    have_global_anchor_ = true;
    RCLCPP_INFO(get_logger(), "global graph seeded at (%.2f, %.2f, %.2f)",
                state[0], state[1], state[2]);
    return;
  }

  const mgg::ExpandContext ctx = makeContext();
  const auto is_admissible = [this, &ctx](const Eigen::Vector3d& from,
                                          const Eigen::Vector3d& to) {
    // Check if the path between vertices is clear of walls/obstacles
    return map_->getPathStatus(from, to, ctx.robot_box_size, /*stop_at_unknown=*/false) !=
           mgg::VoxelStatus::kOccupied;
  };


  const Eigen::Vector3d here(state[0], state[1], state[2]);

  // First, verify we are at least global_vertex_spacing_ from any existing vertex
  // of this robot to avoid cluttering the graph with near-duplicates.
  double min_dist_to_any = std::numeric_limits<double>::max();
  for (const auto& entry : global_graph_->vertices_map_) {
    if (entry.second == nullptr) continue;
    if (entry.second->robot_id != static_cast<int>(planning_params_.robot_id)) {
      continue;
    }
    const Eigen::Vector3d vpos(entry.second->state[0], entry.second->state[1],
                               entry.second->state[2]);
    const double d = (vpos - here).norm();
    if (d < min_dist_to_any) {
      min_dist_to_any = d;
    }
  }
  if (min_dist_to_any < global_vertex_spacing_) return;

  // Find candidate vertices of this robot sorted by Euclidean distance,
  // and choose the closest one whose connecting edge is collision-free (admissible).
  std::vector<std::pair<double, mgg::Vertex*>> candidates;
  for (const auto& entry : global_graph_->vertices_map_) {
    if (entry.second == nullptr) continue;
    if (entry.second->robot_id != static_cast<int>(planning_params_.robot_id)) {
      continue;
    }
    const Eigen::Vector3d vpos(entry.second->state[0], entry.second->state[1],
                               entry.second->state[2]);
    const double d = (vpos - here).norm();
    if (d <= global_vertex_spacing_ * 5.0) {
      candidates.emplace_back(d, entry.second);
    }
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  mgg::Vertex* parent_vertex = nullptr;
  double edge_distance = 0.0;
  for (const auto& cand : candidates) {
    const Eigen::Vector3d origin(cand.second->state[0], cand.second->state[1],
                                 cand.second->state[2]);
    if (is_admissible(origin, here)) {
      parent_vertex = cand.second;
      edge_distance = cand.first;
      break;
    }
  }

  // If no candidate has an admissible line of sight (e.g. turned a blind wall corner),
  // do NOT create an edge cutting through the wall.
  if (parent_vertex == nullptr) {
    // Nothing in range with a clear line to it. Expected briefly around a
    // blind corner, and the right answer is to wait rather than run an edge
    // through the wall - but if it persists the backbone has stalled and the
    // robot will never share anything, so say so.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                         "global graph has no reachable parent within %.1f m of "
                         "(%.2f, %.2f); it has %d vertices and is not growing",
                         global_vertex_spacing_ * 5.0, here.x(), here.y(),
                         global_graph_->getNumVertices());
    return;
  }

  auto* v = new mgg::Vertex(global_graph_->generateVertexID(), state);
  v->robot_id = static_cast<int>(planning_params_.robot_id);
  v->parent = parent_vertex;
  v->distance = parent_vertex->distance + edge_distance;
  parent_vertex->children.push_back(v);
  global_graph_->addVertex(v);
  global_graph_->addEdge(v, parent_vertex, edge_distance);
  last_global_anchor_ = here;
  have_global_anchor_ = true;
}


void PlannerNode::onBuildRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  response->message = buildLocalGraph();
  response->success = local_graph_->getNumVertices() > 1;
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void PlannerNode::onPlanRequest(
    const std::shared_ptr<mgg_msgs::srv::PlannerSrv::Request> request,
    std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response) {
  // A caller may pin the bound mode for this cycle, e.g. to squeeze through a
  // gap it would normally refuse.
  const mgg::BoundModeType previous = robot_params_.bound_mode;
  robot_params_.bound_mode =
      static_cast<mgg::BoundModeType>(request->bound_mode);

  const std::string summary = buildLocalGraph();
  robot_params_.bound_mode = previous;

  response->planning_bound_mode = request->bound_mode;
  response->status = best_path_.empty()
                         ? mgg_msgs::srv::PlannerSrv::Response::HOMING
                         : mgg_msgs::srv::PlannerSrv::Response::FORWARD;
  for (const mgg::StateVec& s : best_path_) {
    response->path.push_back(toPoseMsg(s));
  }

  RCLCPP_INFO(get_logger(), "plan request: %s", summary.c_str());
}

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
