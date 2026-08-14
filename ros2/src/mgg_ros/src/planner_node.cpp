#include "mgg_ros/planner_node.h"

#include <cstdio>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "mgg_core/log.h"
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

  build_srv_ = create_service<std_srvs::srv::Trigger>(
      "build_local_graph",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        onBuildRequest(req, res);
      },
      rclcpp::ServicesQoS(), callback_group_);

  global_vertex_spacing_ =
      declareOrGet<double>(this, "global_vertex_spacing", 1.0);

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
}

void PlannerNode::onPointCloud(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  // Points arrive in the sensor frame; without odometry there is nothing to
  // place them against.
  if (!have_odometry_) return;
  const Eigen::Vector3d origin(current_state_[0], current_state_[1],
                               current_state_[2]);
  std::vector<Eigen::Vector3d> points;
  points.reserve(msg->width * msg->height);
  sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    if (!std::isfinite(*it_x) || !std::isfinite(*it_y) ||
        !std::isfinite(*it_z)) {
      continue;
    }
    points.emplace_back(origin + Eigen::Vector3d(*it_x, *it_y, *it_z));
  }
  if (!points.empty()) map_->insertPointCloud(points, origin);
}

void PlannerNode::onNeighbourGraph(mgg_msgs::msg::Graph::ConstSharedPtr msg) {
  if (msg->vertices.empty()) return;
  const int sender = msg->vertices.front().robot_id;
  if (sender == static_cast<int>(planning_params_.robot_id)) return;

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
  if (r.vertices_added > 0 || r.edges_added > 0) {
    RCLCPP_INFO(get_logger(),
                "merged robot %d: +%d vertices, +%d edges, %d updated%s",
                sender, r.vertices_added, r.edges_added, r.vertices_updated,
                r.merged ? "" : " (not yet connected)");
  }
}

std::string PlannerNode::buildLocalGraph() {
  if (!have_odometry_) return "no odometry received yet";
  if (!map_->getStatus()) return "map is empty; no point cloud received yet";

  updateGlobalGraph();

  local_graph_->reset();
  auto* root = new mgg::Vertex(0, current_state_);
  root->robot_id = static_cast<int>(planning_params_.robot_id);
  local_graph_->addVertex(root);

  const mgg::ExpandContext ctx = makeContext();
  const mgg::GridGraphResult r = buildGridGraph(
      *local_graph_, current_state_, grid_params_, ctx, current_state_[3]);

  if (r.status == mgg::GridGraphStatus::kInvalidBounds) {
    return "grid bounds invalid: min_val must be <= 0, max_val >= 0 and "
           "resolution non-zero";
  }
  // The gain evaluation needs the sampling volume centred on the robot, since
  // it rejects voxels outside it.
  global_space_.setCenter(current_state_, /*use_extension=*/true);

  mgg::GainContext gain_ctx = makeGainContext();
  const int evaluated = mgg::computeExplorationGain(
      *local_graph_, gain_ctx, planning_params_.leafs_only_for_volumetric_gain,
      planning_params_.cluster_vertices_for_gain);

  // Best frontier, which is what the path selection will consume once
  // evaluateGraph and getBestPath are ported.
  const mgg::Vertex* best = nullptr;
  int frontiers = 0;
  for (const auto& entry : local_graph_->vertices_map_) {
    const mgg::Vertex* v = entry.second;
    if (v == nullptr) continue;
    if (v->type == mgg::VertexType::kFrontier) ++frontiers;
    if (best == nullptr || v->vol_gain.gain > best->vol_gain.gain) best = v;
  }

  char buf[288];
  std::snprintf(buf, sizeof(buf),
                "grid graph: %d free cells, %d vertices, %d edges%s; "
                "%d viewpoints evaluated, %d frontiers, best gain %.1f at "
                "(%.2f, %.2f, %.2f)",
                r.free_cells, r.vertices_added, r.edges_added,
                r.hit_limit ? " (hit a size limit)" : "", evaluated, frontiers,
                best ? best->vol_gain.gain : 0.0,
                best ? best->state[0] : 0.0, best ? best->state[1] : 0.0,
                best ? best->state[2] : 0.0);
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
    RCLCPP_INFO(get_logger(), "global graph seeded at (%.2f, %.2f, %.2f)",
                state[0], state[1], state[2]);
    return;
  }

  mgg::Vertex* nearest = nullptr;
  if (!global_graph_->getNearestVertex(&state, &nearest) ||
      nearest == nullptr) {
    return;
  }
  const Eigen::Vector3d origin(nearest->state[0], nearest->state[1],
                               nearest->state[2]);
  const Eigen::Vector3d here(state[0], state[1], state[2]);
  const double d = (here - origin).norm();
  // Only extend once the robot has actually moved, or the graph fills with
  // near-duplicate vertices every cycle.
  if (d < global_vertex_spacing_) return;

  auto* v = new mgg::Vertex(global_graph_->generateVertexID(), state);
  v->robot_id = static_cast<int>(planning_params_.robot_id);
  v->parent = nearest;
  v->distance = nearest->distance + d;
  nearest->children.push_back(v);
  global_graph_->addVertex(v);
  global_graph_->addEdge(v, nearest, d);
}

void PlannerNode::onBuildRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  response->message = buildLocalGraph();
  response->success = local_graph_->getNumVertices() > 1;
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void PlannerNode::publishOwnGraph() {
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
  marker_pub_->publish(array);
}

}  // namespace mgg_ros
