// The ground local planner process: odometry jumps reset ground admissions
// as well as voxels, sustained obstacle churn beside an unchanged corridor
// does not starve it, outputs echo the current session and request, and
// accepted requests and planning cycles are logged as native events.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <rcutils/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "local_planner_scene.h"
#include "mgg_ros/conversions.h"
#include "mgg_ros/local_planner_node.h"

using namespace mgg;
using mgg_msgs::msg::LocalPath;
using mgg_msgs::msg::LocalPathInvalidation;
using mgg_msgs::msg::LocalPlannerStatus;
using mgg_test::basePose;
using mgg_test::drivingPoint;

namespace {

builtin_interfaces::msg::Time stampAt(double t) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(std::floor(t));
  stamp.nanosec =
      static_cast<uint32_t>(std::llround((t - std::floor(t)) * 1e9));
  return stamp;
}

std::vector<rclcpp::Parameter> sceneParameters() {
  const auto p = mgg_test::sceneParams();
  return {
      rclcpp::Parameter("RobotParams.type", "kGroundRobot"),
      rclcpp::Parameter("RobotParams.size",
                        std::vector<double>{p.robot.size.x(), p.robot.size.y(),
                                            p.robot.size.z()}),
      rclcpp::Parameter("RobotParams.size_extension",
                        std::vector<double>{0, 0, 0}),
      rclcpp::Parameter("RobotParams.size_extension_min",
                        std::vector<double>{0, 0, 0}),
      rclcpp::Parameter("RobotParams.safety_extension",
                        std::vector<double>{0, 0, 0}),
      rclcpp::Parameter("RobotParams.bound_mode", "kExactBound"),
      rclcpp::Parameter("PlanningParams.max_ground_height",
                        p.planning.max_ground_height),
      rclcpp::Parameter("PlanningParams.max_inclination",
                        p.planning.max_inclination),
      rclcpp::Parameter("PlanningParams.max_negative_inclination",
                        p.planning.max_negative_inclination),
      rclcpp::Parameter("PlanningParams.min_observed_ground_fraction",
                        p.planning.min_observed_ground_fraction),
      rclcpp::Parameter("PlanningParams.v_max", p.planning.v_max),
      // Zone reach = radius + half the 1 m body: 0.8 m, which leaves the
      // corridor a detour.
      rclcpp::Parameter("PlanningParams.no_go_radius_m", 0.3),
      rclcpp::Parameter("SensorParams.sensor_list",
                        std::vector<std::string>{"lidar"}),
      rclcpp::Parameter("SensorParams.lidar.type", "kLidar"),
      rclcpp::Parameter("SensorParams.lidar.max_range", p.sensor.max_range),
      rclcpp::Parameter(
          "SensorParams.lidar.fov",
          std::vector<double>{p.sensor.fov.x(), p.sensor.fov.y()}),
      rclcpp::Parameter("SensorParams.lidar.resolution",
                        std::vector<double>{p.sensor.resolution.x(),
                                            p.sensor.resolution.y()}),
      // Tests run each cycle explicitly.
      rclcpp::Parameter("planning_period_s", 0.0),
  };
}

/// One node in its own namespace, a probe recording its outputs, and the
/// corridor it sees.
struct Harness {
  std::shared_ptr<LocalPlannerNode> node;
  rclcpp::Node::SharedPtr probe;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<LocalPath> paths;
  std::vector<LocalPathInvalidation> invalidations;
  std::vector<LocalPlannerStatus> statuses;
  std::vector<nav_msgs::msg::OccupancyGrid> grids;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  mgg_test::Corridor corridor;
  double t = 100.0;

  explicit Harness(const std::string& ns) {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/" + ns});
    options.parameter_overrides(sceneParameters());
    options.automatically_declare_parameters_from_overrides(true);
    node = std::make_shared<LocalPlannerNode>(options);
    probe = std::make_shared<rclcpp::Node>(ns + "_probe");
    const std::string root = "/" + ns + "/";
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    subscriptions.push_back(probe->create_subscription<LocalPath>(
        root + "local_path", rclcpp::QoS(100).reliable(),
        [this](const LocalPath& m) { paths.push_back(m); }));
    subscriptions.push_back(probe->create_subscription<LocalPathInvalidation>(
        root + "local_path_invalidation", rclcpp::QoS(100).reliable(),
        [this](const LocalPathInvalidation& m) {
          invalidations.push_back(m);
        }));
    subscriptions.push_back(probe->create_subscription<LocalPlannerStatus>(
        root + "status", latched,
        [this](const LocalPlannerStatus& m) { statuses.push_back(m); }));
    subscriptions.push_back(
        probe->create_subscription<nav_msgs::msg::OccupancyGrid>(
            root + "traversability", latched,
            [this](const nav_msgs::msg::OccupancyGrid& m) {
              grids.push_back(m);
            }));
    executor.add_node(probe);
  }

  size_t received() const {
    return paths.size() + invalidations.size() + statuses.size() + grids.size();
  }
  /// Delivers everything published so far: until 50 ms pass without news.
  void settle() {
    using Clock = std::chrono::steady_clock;
    const auto give_up = Clock::now() + std::chrono::seconds(3);
    auto quiet_since = Clock::now();
    size_t seen = received();
    while (Clock::now() < give_up &&
           Clock::now() - quiet_since < std::chrono::milliseconds(50)) {
      executor.spin_some(std::chrono::milliseconds(5));
      if (received() != seen) {
        seen = received();
        quiet_since = Clock::now();
      }
    }
  }

  void odometry(const StateVec& base) {
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "odom";
    odom.header.stamp = stampAt(t);
    odom.child_frame_id = "base_link";
    odom.pose.pose = mgg_ros::toPoseMsg(base);
    node->onOdometry(odom);
  }

  /// Odometry, the lidar's TF at this stamp, and its scan of the corridor.
  void step(const StateVec& base) {
    odometry(base);
    const Eigen::Vector3d origin = mgg_test::lidarOrigin(base);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "odom";
    tf.header.stamp = stampAt(t);
    tf.child_frame_id = "lidar";
    tf.transform.translation.x = origin.x();
    tf.transform.translation.y = origin.y();
    tf.transform.translation.z = origin.z();
    tf.transform.rotation.w = 1;
    node->tfBuffer().setTransform(tf, "test", false);
    const auto points = corridor.scan(origin);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "lidar";
    cloud.header.stamp = stampAt(t);
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"),
        z(cloud, "z");
    for (const auto& p : points) {
      *x = static_cast<float>(p.x() - origin.x());
      *y = static_cast<float>(p.y() - origin.y());
      *z = static_cast<float>(p.z() - origin.z());
      ++x;
      ++y;
      ++z;
    }
    node->onPointCloud(cloud);
  }

  LocalPlannerNode::SetMode::Response setMode(
      const std::string& session, const std::string& request,
      const Eigen::Vector3d& goal,
      const std::vector<Eigen::Vector3d>& route = {}) {
    LocalPlannerNode::SetMode::Request req;
    req.session_id = session;
    req.request_id = request;
    req.mode = LocalPlannerNode::SetMode::Request::FOLLOW_ROUTE;
    req.goal.x = goal.x();
    req.goal.y = goal.y();
    req.goal.z = goal.z();
    req.tolerance_m = 0.3;
    for (const auto& p : route) {
      geometry_msgs::msg::Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      req.route.push_back(point);
    }
    LocalPlannerNode::SetMode::Response res;
    node->onSetMode(req, res);
    return res;
  }
};

int8_t cellAt(const nav_msgs::msg::OccupancyGrid& grid, double x, double y) {
  const int ix = static_cast<int>(
      std::floor((x - grid.info.origin.position.x) / grid.info.resolution));
  const int iy = static_cast<int>(
      std::floor((y - grid.info.origin.position.y) / grid.info.resolution));
  if (ix < 0 || iy < 0 || ix >= static_cast<int>(grid.info.width) ||
      iy >= static_cast<int>(grid.info.height))
    return -2;
  return grid.data[iy * grid.info.width + ix];
}

std::vector<std::string>* g_event_lines = nullptr;

void captureEventLine(const rcutils_log_location_t*, int, const char*,
                      rcutils_time_point_value_t, const char* format,
                      va_list* args) {
  if (!g_event_lines) return;
  va_list copy;
  va_copy(copy, *args);
  std::vector<char> buffer(32 * 1024);
  std::vsnprintf(buffer.data(), buffer.size(), format, copy);
  va_end(copy);
  const std::string line(buffer.data());
  const auto at = line.find("SDEVT1 ");
  if (at != std::string::npos) g_event_lines->push_back(line.substr(at + 7));
}

/// The node's native event lines, as RCLCPP_INFO writes them.
struct EventCapture {
  rcutils_logging_output_handler_t previous;
  std::vector<std::string> lines;
  EventCapture() : previous(rcutils_logging_get_output_handler()) {
    g_event_lines = &lines;
    rcutils_logging_set_output_handler(captureEventLine);
  }
  ~EventCapture() {
    rcutils_logging_set_output_handler(previous);
    g_event_lines = nullptr;
  }
  std::vector<nlohmann::json> of(const std::string& kind) const {
    std::vector<nlohmann::json> events;
    for (const auto& line : lines) {
      auto event = nlohmann::json::parse(line);
      if (event["kind"] == kind) events.push_back(std::move(event));
    }
    return events;
  }
};

std::vector<StateVec> posesOf(const LocalPath& path) {
  std::vector<StateVec> poses;
  for (const auto& p : path.poses) poses.push_back(mgg_ros::fromPoseMsg(p));
  return poses;
}

}  // namespace

class LocalPlannerNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(LocalPlannerNodeTest, ResetClearsGroundAndMovesOrigin) {
  Harness h("reset_clears_ground");
  const StateVec base = basePose(0.1, 0.1);
  // Observe until the ground 3 m ahead is admitted.
  int8_t admitted = -1;
  for (int i = 0; i < 40 && admitted != 0; ++i) {
    h.step(base);
    h.t += 0.1;
    h.settle();
    if (!h.grids.empty()) admitted = cellAt(h.grids.back(), 3.1, 0.1);
  }
  ASSERT_EQ(admitted, 0);
  ASSERT_TRUE(h.setMode("s1", "r1", drivingPoint(6.1, 0.1)).accepted);
  h.node->planCycle();
  h.settle();
  ASSERT_FALSE(h.paths.empty());
  const auto old_origin = h.grids.back().info.origin.position;
  const size_t invalidations_before = h.invalidations.size();
  uint64_t certified_revision = 0;
  for (const auto& path : h.paths)
    certified_revision = std::max(certified_revision, path.map_revision);

  // Odometry jumps 3 m: the window moves, and nothing certified survives.
  h.odometry(basePose(3.1, 0.1));
  h.settle();
  ASSERT_FALSE(h.grids.empty());
  const auto& grid = h.grids.back();
  EXPECT_NE(grid.info.origin.position.x, old_origin.x);
  const int8_t stale_cell = cellAt(grid, 3.1, 0.1);
  EXPECT_EQ(stale_cell, -1);
  ASSERT_FALSE(h.statuses.empty());
  EXPECT_EQ(h.statuses.back().status, LocalPlannerStatus::WAITING_FOR_MAP);
  std::set<uint64_t> invalidated;
  for (const auto& inv : h.invalidations) invalidated.insert(inv.sequence_id);
  bool all_paths_invalidated = true;
  for (const auto& path : h.paths)
    all_paths_invalidated &= invalidated.count(path.sequence_id) > 0;
  EXPECT_TRUE(all_paths_invalidated);
  // The reset's invalidations carry the reset's own revision (the status
  // after the reset reports it; no scan has arrived since), newer than any
  // revision a path was certified on.
  ASSERT_GT(h.invalidations.size(), invalidations_before);
  const uint64_t reset_revision = h.statuses.back().map_revision;
  EXPECT_GT(reset_revision, certified_revision);
  for (size_t i = invalidations_before; i < h.invalidations.size(); ++i)
    EXPECT_EQ(h.invalidations[i].map_revision, reset_revision);
}

TEST_F(LocalPlannerNodeTest, MovingChurnRecoversWithinOneCycle) {
  Harness h("moving_churn");
  StateVec base = basePose(0.1, 0.1);
  for (int i = 0; i < 12; ++i) {
    h.step(base);
    h.t += 0.1;
  }
  std::vector<Eigen::Vector3d> route;
  for (int x = 1; x <= 30; ++x) route.push_back(drivingPoint(x + 0.1, 0.1));
  ASSERT_TRUE(h.setMode("s1", "r1", drivingPoint(30.1, 0.1), route).accepted);
  h.node->planCycle();
  h.settle();
  ASSERT_FALSE(h.paths.empty());
  LocalPath executing = h.paths.back();
  double progress = mgg_test::progressAlong(posesOf(executing), base.head<2>());
  size_t paths_seen = h.paths.size(),
         invalidations_seen = h.invalidations.size();

  int extensions = 0, unsafe_outputs = 0, breaks_beside_corridor = 0,
      intrusion_breaks = 0;
  double recovery_s = 0;
  std::optional<double> lost_at;
  Eigen::AlignedBox3d intrusion;
  // 10 Hz side-obstacle changes for 5 s while driving at 0.5 m/s. For one
  // second the obstacle stands on the route instead: that withdrawal is
  // real, and the planner must recover from it within a cycle.
  for (int step = 0; step < 50; ++step) {
    h.t += 0.1;
    const bool intruding = step >= 25 && step < 35;
    const double x0 = base.x() + 2.0 + 0.4 * (step % 3);
    h.corridor.obstacle = Eigen::AlignedBox3d(
        Eigen::Vector3d(x0, 1.3, -0.1), Eigen::Vector3d(x0 + 0.4, 1.7, 1.0));
    if (step == 25)
      intrusion =
          Eigen::AlignedBox3d(Eigen::Vector3d(base.x() + 2.5, -0.1, -0.1),
                              Eigen::Vector3d(base.x() + 2.9, 0.3, 1.0));
    if (intruding) h.corridor.obstacle = intrusion;
    progress += 0.05;
    const StateVec along = mgg_test::poseAt(posesOf(executing), progress);
    base = basePose(along.x(), along.y(), along[3]);
    mgg_msgs::msg::LocalPathFeedback fb;
    fb.session_id = "s1";
    fb.epoch = executing.epoch;
    fb.sequence_id = executing.sequence_id;
    fb.state = mgg_msgs::msg::LocalPathFeedback::EXECUTING;
    fb.progress_m = progress;
    fb.speed_mps = 0.5;
    h.node->onFeedback(fb);
    h.step(base);
    if (step % 5 == 4) h.node->planCycle();
    h.settle();
    for (; invalidations_seen < h.invalidations.size(); ++invalidations_seen)
      if (h.invalidations[invalidations_seen].sequence_id ==
          executing.sequence_id) {
        if (step < 25) ++breaks_beside_corridor;
        if (intruding) ++intrusion_breaks;
        if (!lost_at) lost_at = h.t;
      }
    for (; paths_seen < h.paths.size(); ++paths_seen) {
      const auto& path = h.paths[paths_seen];
      if (path.kind == LocalPath::EXTEND) ++extensions;
      const auto poses = posesOf(path);
      bool unsafe = false;
      for (double s = 0; s <= mgg_test::pathLength(poses) && !unsafe;
           s += 0.05) {
        const StateVec p = mgg_test::poseAt(poses, s);
        for (const auto& solid :
             {*h.corridor.obstacle, h.corridor.solids[1], h.corridor.solids[2]})
          unsafe |= mgg_test::footprintOverlaps(p, {1.0, 0.6}, solid);
      }
      unsafe_outputs += unsafe;
      if (lost_at) {
        recovery_s = std::max(recovery_s, h.t - *lost_at);
        lost_at.reset();
      }
      executing = path;
      progress = mgg_test::progressAlong(poses, base.head<2>());
    }
  }
  if (lost_at) recovery_s = std::max(recovery_s, h.t - *lost_at);
  // The intrusion did withdraw the driven path, so recovery was measured.
  EXPECT_GT(intrusion_breaks, 0);
  EXPECT_LE(recovery_s, 0.5);
  EXPECT_GT(extensions, 5);
  EXPECT_EQ(unsafe_outputs, 0);
  // Until the intrusion the corridor never changed: churn beside it must not
  // withdraw the path being driven (each withdrawal is a BREAK).
  EXPECT_EQ(breaks_beside_corridor, 0);
  // The churn did reach the retained paths' dependencies.
  EXPECT_GT(h.node->core().retainedRechecks(), 0u);
}

TEST_F(LocalPlannerNodeTest, SessionAndRequestEcho) {
  Harness h("session_echo");
  const StateVec base = basePose(0.1, 0.1);
  for (int i = 0; i < 12; ++i) {
    h.step(base);
    h.t += 0.1;
  }
  const auto first_mode = h.setMode("s1", "r1", drivingPoint(6.1, 0.1));
  ASSERT_TRUE(first_mode.accepted) << first_mode.reason;
  h.node->planCycle();
  h.settle();
  ASSERT_FALSE(h.paths.empty());
  const LocalPath old_path = h.paths.back();
  EXPECT_EQ(old_path.session_id, "s1");
  EXPECT_EQ(old_path.epoch, first_mode.epoch);

  const std::string new_session = "s2", latest_request = "r2b";
  ASSERT_TRUE(h.setMode(new_session, "r2a", drivingPoint(6.1, 0.1)).accepted);
  ASSERT_TRUE(
      h.setMode(new_session, latest_request, drivingPoint(6.1, 0.1)).accepted);
  // A delayed feedback of the old session.
  mgg_msgs::msg::LocalPathFeedback old_fb;
  old_fb.session_id = "s1";
  old_fb.epoch = old_path.epoch;
  old_fb.sequence_id = old_path.sequence_id;
  old_fb.state = mgg_msgs::msg::LocalPathFeedback::EXECUTING;
  old_fb.progress_m = 2.0;
  h.node->onFeedback(old_fb);
  const bool old_feedback_applied = h.node->core().progress() != 0.0;

  h.node->planCycle();
  h.settle();
  ASSERT_FALSE(h.paths.empty());
  ASSERT_FALSE(h.statuses.empty());
  const auto& path = h.paths.back();
  const auto& status = h.statuses.back();
  EXPECT_EQ(path.session_id, new_session);
  EXPECT_EQ(path.request_id, latest_request);
  // The new session starts afresh; it never extends the old session's path.
  const auto first_new = std::find_if(
      h.paths.begin(), h.paths.end(),
      [&](const LocalPath& p) { return p.session_id == new_session; });
  ASSERT_NE(first_new, h.paths.end());
  EXPECT_EQ(first_new->kind, LocalPath::START);
  EXPECT_EQ(status.session_id, new_session);
  EXPECT_EQ(status.request_id, latest_request);
  EXPECT_FALSE(old_feedback_applied);
}

TEST_F(LocalPlannerNodeTest, NoGoUpdateWithdrawsDrivenPath) {
  Harness h("no_go_update");
  const StateVec base = basePose(0.1, 0.1);
  for (int i = 0; i < 12; ++i) {
    h.step(base);
    h.t += 0.1;
  }
  ASSERT_TRUE(h.setMode("s1", "r1", drivingPoint(6.1, 0.1)).accepted);
  h.settle();
  ASSERT_FALSE(h.paths.empty());
  const LocalPath driven = h.paths.back();
  const Eigen::Vector2d zone(3.1, 0.1);
  ASSERT_TRUE(mgg_test::pathCrosses(posesOf(driven), zone, 0.8));
  const size_t paths_before = h.paths.size();

  // A zone on the driven path, same session: withdrawn and replaced at
  // once, without waiting for a planning cycle.
  geometry_msgs::msg::PoseArray zones;
  zones.header.frame_id = "odom";
  zones.poses.resize(1);
  zones.poses[0].position.x = zone.x();
  zones.poses[0].position.y = zone.y();
  h.node->onNoGoZones(zones);
  h.settle();
  ASSERT_FALSE(h.invalidations.empty());
  EXPECT_EQ(h.invalidations.back().sequence_id, driven.sequence_id);
  EXPECT_EQ(h.invalidations.back().session_id, "s1");
  ASSERT_GT(h.paths.size(), paths_before);
  const auto& detour = h.paths.back();
  EXPECT_EQ(detour.kind, LocalPath::BREAK);
  EXPECT_EQ(detour.extends_sequence_id, driven.sequence_id);
  EXPECT_FALSE(mgg_test::pathCrosses(posesOf(detour), zone, 0.8));
}

TEST_F(LocalPlannerNodeTest, SetModeEmitsLocalRequest) {
  Harness h("set_mode_events");
  const StateVec base = basePose(0.1, 0.1);
  for (int i = 0; i < 12; ++i) {
    h.step(base);
    h.t += 0.1;
  }
  EventCapture capture;
  // An idle planner has no session: no planning-cycle event.
  h.node->planCycle();
  EXPECT_TRUE(capture.of("local_plan").empty());

  LocalPlannerNode::SetMode::Request req;
  req.session_id = "s1";
  req.request_id = "r1";
  req.mode = LocalPlannerNode::SetMode::Request::FOLLOW_ROUTE;
  req.frame_id = "odom";
  req.goal.x = 6.1;
  req.goal.y = 0.1;
  req.goal.z = drivingPoint(6.1, 0.1).z();
  req.tolerance_m = 0.3;
  LocalPlannerNode::SetMode::Response res;
  h.node->onSetMode(req, res);
  ASSERT_TRUE(res.accepted) << res.reason;
  auto requests = capture.of("local_request");
  ASSERT_EQ(requests.size(), 1u);
  const auto& event = requests.front();
  EXPECT_EQ(event["robot_id"], "set_mode_events");
  EXPECT_EQ(event["payload"]["request_id"], req.request_id);
  EXPECT_EQ(event["payload"]["session_id"], req.session_id);
  EXPECT_FALSE(event["payload"]["continuation"].get<bool>());
  EXPECT_NEAR(event["payload"]["pose"]["x"].get<double>(), base.x(), 1e-9);
  EXPECT_EQ(event["payload"]["mode"], "follow_route");
  // The accepted request planned at once: one cycle, one status change.
  const auto plans = capture.of("local_plan");
  ASSERT_EQ(plans.size(), 1u);
  EXPECT_EQ(plans.back()["payload"]["request_id"], req.request_id);
  EXPECT_EQ(plans.back()["payload"]["outcome"], "path");
  EXPECT_GE(plans.back()["payload"]["path_length_m"].get<double>(), 0.0);
  const auto statuses = capture.of("local_status");
  ASSERT_EQ(statuses.size(), 1u);
  EXPECT_EQ(statuses.back()["payload"]["status"], "moving");

  // Same session: a continuation. A refused request is not an event.
  req.request_id = "r2";
  h.node->onSetMode(req, res);
  ASSERT_TRUE(res.accepted);
  req.session_id = "";
  req.request_id = "r3";
  h.node->onSetMode(req, res);
  ASSERT_FALSE(res.accepted);
  requests = capture.of("local_request");
  ASSERT_EQ(requests.size(), 2u);
  EXPECT_EQ(requests.back()["payload"]["request_id"], "r2");
  EXPECT_TRUE(requests.back()["payload"]["continuation"].get<bool>());
}
