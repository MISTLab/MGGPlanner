#include "mgg_ros/local_planner_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <tf2/exceptions.h>

#include "mgg_ros/conversions.h"
#include "mgg_ros/exploration_event.h"
#include "mgg_ros/local_events.h"
#include "mgg_ros/local_path_conversions.h"
#include "mgg_ros/param_loader.h"
#include "mgg_ros/scan_input.h"

namespace mgg {
namespace {

using Clock = std::chrono::steady_clock;

rclcpp::QoS latched() { return rclcpp::QoS(1).reliable().transient_local(); }
rclcpp::QoS reliable() { return rclcpp::QoS(10).reliable(); }

}  // namespace

LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mggplanner_node", options),
      tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
      core_(loadParameters()) {
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  robot_id_ = robotIdFromNamespace(get_namespace(), get_name());
  core_.setFrameLookup(
      [this](const std::string& frame) { return lookup(frame); });

  path_pub_ =
      create_publisher<mgg_msgs::msg::LocalPath>("local_path", reliable());
  invalidation_pub_ = create_publisher<mgg_msgs::msg::LocalPathInvalidation>(
      "local_path_invalidation", reliable());
  status_pub_ =
      create_publisher<mgg_msgs::msg::LocalPlannerStatus>("status", latched());
  grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("traversability",
                                                             latched());

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", rclcpp::QoS(2).best_effort(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        onPointCloud(*msg);
      });
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        onOdometry(*msg);
      });
  feedback_sub_ = create_subscription<mgg_msgs::msg::LocalPathFeedback>(
      "local_path_feedback", reliable(),
      [this](mgg_msgs::msg::LocalPathFeedback::ConstSharedPtr msg) {
        onFeedback(*msg);
      });
  guidance_sub_ = create_subscription<mgg_msgs::msg::GlobalGuidance>(
      "global_guidance", latched(),
      [this](mgg_msgs::msg::GlobalGuidance::ConstSharedPtr msg) {
        onGuidance(*msg);
      });
  no_go_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "no_go_zones", latched(),
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
        onNoGoZones(*msg);
      });
  set_mode_srv_ = create_service<SetMode>(
      "set_mode", [this](const std::shared_ptr<SetMode::Request> request,
                         std::shared_ptr<SetMode::Response> response) {
        onSetMode(*request, *response);
      });

  const double period = mgg_ros::declareOrGet<double>(this, "planning_period_s",
                                                      kLocalPlanningPeriodS);
  if (!std::isfinite(period) || period < 0)
    throw std::invalid_argument(
        "planning_period_s must be finite and non-negative");
  // Zero leaves cycles to the caller (tests); production uses the default.
  if (period > 0) {
    timer_ =
        create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::duration<double>(period)),
                          [this] { planCycle(); });
  }
  RCLCPP_INFO(get_logger(), "local planner epoch %llu, planning every %.2f s",
              static_cast<unsigned long long>(core_.epoch()), period);
}

LocalPlanningParams LocalPlannerNode::loadParameters() {
  LocalPlanningParams params;
  mgg_ros::ParamLoader p(this);
  if (!mgg_ros::loadRobotParams(p, "RobotParams", params.robot))
    throw std::invalid_argument("RobotParams failed to load");
  if (!mgg_ros::loadPlanningParams(p, "PlanningParams", params.planning))
    throw std::invalid_argument("PlanningParams failed to load");
  std::vector<std::string> sensors;
  if (p.get("SensorParams/sensor_list", sensors) && !sensors.empty()) {
    if (!mgg_ros::loadSensorParams(p, "SensorParams/" + sensors.front(),
                                   params.sensor))
      throw std::invalid_argument("SensorParams/" + sensors.front() +
                                  " failed to load");
  } else {
    RCLCPP_WARN(get_logger(),
                "no SensorParams/sensor_list: local gain is unavailable");
  }
  params.window.resolution = mgg_ros::declareOrGet<double>(
      this, "local_map.resolution", params.window.resolution);
  params.ground_recheck_s = mgg_ros::declareOrGet<double>(
      this, "ground_recheck_s", params.ground_recheck_s);
  // A fresh epoch per process: the executor tells a restarted planner's
  // paths from the old process's by it, and sequences restart at 1.
  params.epoch = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
  no_go_radius_m_ = params.planning.no_go_radius_m;
  planning_budget_s_ = kLocalPlanningBudgetS + 2 * params.ground_recheck_s;
  return params;
}

std::optional<Eigen::Isometry3d> LocalPlannerNode::lookup(
    const std::string& frame) const {
  if (odom_frame_.empty()) return std::nullopt;
  if (frame == odom_frame_) return Eigen::Isometry3d::Identity();
  geometry_msgs::msg::TransformStamped tf;
  try {
    // Latest: goals, guidance and zones follow every map-to-odometry
    // correction, re-resolved each cycle.
    tf = tf_buffer_->lookupTransform(odom_frame_, frame, tf2::TimePointZero);
  } catch (const tf2::TransformException&) {
    return std::nullopt;
  }
  const auto& t = tf.transform.translation;
  const auto& r = tf.transform.rotation;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() =
      Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized().toRotationMatrix();
  T.translation() = Eigen::Vector3d(t.x, t.y, t.z);
  return T;
}

void LocalPlannerNode::onOdometry(const nav_msgs::msg::Odometry& msg) {
  const std::lock_guard<std::mutex> lock(mutex_);
  const StateVec base = mgg_ros::fromPoseMsg(msg.pose.pose);
  if (!base.allFinite() || msg.header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "ignoring odometry without a frame or finite pose");
    return;
  }
  if (odom_frame_.empty()) {
    odom_frame_ = msg.header.frame_id;
  } else if (msg.header.frame_id != odom_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "ignoring odometry in '%s'; planning in '%s'",
                         msg.header.frame_id.c_str(), odom_frame_.c_str());
    return;
  }
  base_ = base;
  const MapChange change = core_.onOdometry(base);
  if (change.everything) {
    publishGrid();
    // A reset withdraws everything: report it now, not next cycle.
    publishInvalidations(false);
    runPlan();
    return;
  }
  publishInvalidations(true);
}

void LocalPlannerNode::onPointCloud(const sensor_msgs::msg::PointCloud2& msg) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (odom_frame_.empty() || !base_) {
    ++dropped_scans_;
    return;
  }
  const auto scan = scanInOdom(msg, *tf_buffer_, odom_frame_);
  if (!scan) {
    ++dropped_scans_;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "dropped %llu scan(s): no TF from '%s' to '%s' at the scan stamp",
        static_cast<unsigned long long>(dropped_scans_),
        msg.header.frame_id.c_str(), odom_frame_.c_str());
    return;
  }
  core_.onScan(*scan, *base_);
  publishGrid();
  publishInvalidations(true);
}

void LocalPlannerNode::onFeedback(const mgg_msgs::msg::LocalPathFeedback& msg) {
  const std::lock_guard<std::mutex> lock(mutex_);
  core_.onFeedback(fromFeedbackMsg(msg));
}

void LocalPlannerNode::onGuidance(const mgg_msgs::msg::GlobalGuidance& msg) {
  const std::lock_guard<std::mutex> lock(mutex_);
  try {
    core_.setGuidance(fromGuidanceMsg(msg));
  } catch (const std::invalid_argument& error) {
    RCLCPP_WARN(get_logger(), "ignoring guidance: %s", error.what());
  }
}

void LocalPlannerNode::onNoGoZones(const geometry_msgs::msg::PoseArray& msg) {
  const std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Eigen::Vector2d> centres;
  for (const auto& pose : msg.poses)
    if (std::isfinite(pose.position.x) && std::isfinite(pose.position.y))
      centres.emplace_back(pose.position.x, pose.position.y);
  // As PlannerNode: a centre line kept out of the reach keeps the body out.
  const Eigen::Vector3d box = core_.params().robot.getPlanningSize();
  NoGoZones zones;
  zones.set(std::move(centres),
            no_go_radius_m_ + 0.5 * std::max(box.x(), box.y()));
  core_.setNoGoZones(zones, msg.header.frame_id);
  // A path the new zones withdraw is reported, and replaced, at once.
  publishInvalidations(true);
}

void LocalPlannerNode::onSetMode(const SetMode::Request& request,
                                 SetMode::Response& response) {
  const std::lock_guard<std::mutex> lock(mutex_);
  response.epoch = core_.epoch();
  LocalModeRequest mode;
  LocalModeResponse result;
  bool continuation = false;
  try {
    mode = fromModeRequest(request);
    // As the core decides it: the request keeps the current session.
    continuation = mode.session_id == core_.sessionId();
    result = core_.setMode(mode);
  } catch (const std::invalid_argument& error) {
    response.accepted = false;
    response.reason = error.what();
    return;
  }
  response.accepted = result.accepted;
  response.reason = result.reason;
  response.epoch = result.epoch;
  if (!result.accepted) return;
  // Requests are service calls, not recorded topics: replay re-issues these.
  // Losslessly: a route too long for one log line is split into parts that
  // share the request's identity, pose and stamp.
  const double seconds = now().seconds();
  for (auto& part : localRequestParts(
           localRequestPayload(mode, continuation, base_, seconds),
           kLocalRequestPartBytes))
    emitEvent("local_request", std::move(part), seconds);
  runPlan();
}

void LocalPlannerNode::planCycle() {
  const std::lock_guard<std::mutex> lock(mutex_);
  runPlan();
}

void LocalPlannerNode::runPlan() {
  const auto deadline =
      Clock::now() + std::chrono::duration_cast<Clock::duration>(
                         std::chrono::duration<double>(planning_budget_s_));
  auto result = core_.plan(deadline);
  if (result.path) {
    result.path->frame_id = odom_frame_;
    result.path->stamp_ns = static_cast<std::uint64_t>(now().nanoseconds());
    path_pub_->publish(toLocalPathMsg(*result.path));
  }
  publishInvalidations(false);
  mgg_msgs::msg::LocalPlannerStatus status;
  status.status = statusName(result.status);
  status.reason = result.reason;
  status.map_revision = core_.map().revision();
  status.session_id = core_.sessionId();
  status.request_id = core_.requestId();
  status.stamp = now();
  status.guidance_sequence_id = core_.guidanceSequenceId();
  status_pub_->publish(status);
  if (core_.sessionId().empty() || core_.requestId().empty()) return;
  if (core_.mode() != LocalMode::kIdle)
    emitEvent("local_plan",
              localPlanPayload(result, core_.sessionId(), core_.requestId(),
                               status.map_revision),
              now().seconds());
  const std::string status_event = status.status + '\n' + status.reason +
                                   '\n' + status.session_id + '\n' +
                                   status.request_id;
  if (status_event != last_status_event_) {
    last_status_event_ = status_event;
    emitEvent("local_status", localStatusPayload(status), now().seconds());
  }
}

void LocalPlannerNode::emitEvent(const std::string& kind,
                                 const nlohmann::json& payload,
                                 double seconds) {
  // Sim time once /clock runs; before it, consumers place the wall time.
  const std::optional<double> stamp =
      seconds > 0 ? std::optional<double>(seconds) : std::nullopt;
  std::string line;
  try {
    line = mgg_ros::explorationEventLine(kind, robot_id_, payload, stamp);
  } catch (const std::invalid_argument& error) {
    RCLCPP_WARN(get_logger(), "dropped %s event: %s", kind.c_str(),
                error.what());
    return;
  }
  RCLCPP_INFO(get_logger(), "%s", line.c_str());
}

void LocalPlannerNode::publishInvalidations(bool plan_after) {
  const auto invalidations = core_.takeInvalidations();
  for (const auto& invalidation : invalidations) {
    mgg_msgs::msg::LocalPathInvalidation msg;
    msg.header.frame_id = odom_frame_;
    msg.header.stamp = now();
    msg.session_id = invalidation.session_id;
    msg.epoch = invalidation.epoch;
    msg.sequence_id = invalidation.sequence_id;
    msg.map_revision = invalidation.map_revision;
    msg.reason = invalidation.reason;
    invalidation_pub_->publish(msg);
  }
  // A withdrawn path is replanned at once, not at the next period.
  if (plan_after && !invalidations.empty()) runPlan();
}

void LocalPlannerNode::publishGrid() {
  const auto cells = core_.ground().occupancy();
  const auto window = core_.map().window();
  if (cells.empty() || window.isEmpty()) return;
  const double resolution = core_.map().getResolution();
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = odom_frame_;
  grid.header.stamp = now();
  grid.info.map_load_time = grid.header.stamp;
  grid.info.resolution = static_cast<float>(resolution);
  grid.info.width =
      static_cast<uint32_t>(std::lround(window.sizes().x() / resolution));
  grid.info.height =
      static_cast<uint32_t>(std::lround(window.sizes().y() / resolution));
  if (static_cast<size_t>(grid.info.width) * grid.info.height != cells.size())
    return;
  grid.info.origin.position.x = core_.ground().origin().x();
  grid.info.origin.position.y = core_.ground().origin().y();
  grid.info.origin.position.z = window.min().z();
  grid.info.origin.orientation.w = 1;
  grid.data = cells;
  grid_pub_->publish(grid);
}

std::uint64_t LocalPlannerNode::droppedScans() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return dropped_scans_;
}

}  // namespace mgg
