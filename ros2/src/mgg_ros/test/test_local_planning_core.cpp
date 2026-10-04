// The ROS-free local planning cycle the node and the replay runner share:
// session-fenced feedback, no-go zones, inputs in other frames re-transformed
// every cycle, and retained paths withdrawn when the map changes under them.

#include <algorithm>
#include <chrono>
#include <optional>

#include <gtest/gtest.h>

#include "local_planner_scene.h"
#include "mgg_ros/local_planning_core.h"

using namespace mgg;
using mgg_test::basePose;
using mgg_test::drivingPoint;

namespace {

std::chrono::steady_clock::time_point soon(double s = 1.0) {
  return std::chrono::steady_clock::now() +
         std::chrono::duration_cast<std::chrono::steady_clock::duration>(
             std::chrono::duration<double>(s));
}

/// Scans until the ground under the corridor ahead is certified.
void observe(LocalPlanningCore& core, const mgg_test::Corridor& corridor,
             const StateVec& base, int scans = 12) {
  core.onOdometry(base);
  for (int i = 0; i < scans; ++i) core.onScan(corridor.odomScan(base), base);
}

LocalModeRequest follow(const std::string& session, const std::string& request,
                        const Eigen::Vector3d& goal,
                        const std::string& frame = {}) {
  LocalModeRequest r;
  r.session_id = session;
  r.request_id = request;
  r.mode = LocalMode::kFollowRoute;
  r.frame_id = frame;
  r.goal = goal;
  r.tolerance_m = 0.3;
  return r;
}

}  // namespace

TEST(LocalPlanningCore, FeedbackIsSessionFenced) {
  LocalPlanningCore core(mgg_test::sceneParams());
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(6.1, 0.1))).accepted);
  LocalFeedback old_session_fb;
  old_session_fb.session_id = "s0";
  old_session_fb.epoch = core.epoch();
  old_session_fb.sequence_id = 1;
  old_session_fb.executing = true;
  old_session_fb.progress_m = 2.5;
  core.onFeedback(old_session_fb);
  EXPECT_EQ(core.progress(), 0.0);
  LocalFeedback old_epoch_fb = old_session_fb;
  old_epoch_fb.session_id = "s1";
  old_epoch_fb.epoch = core.epoch() + 1;
  core.onFeedback(old_epoch_fb);
  EXPECT_EQ(core.progress(), 0.0);
  LocalFeedback fb = old_epoch_fb;
  fb.epoch = core.epoch();
  fb.progress_m = 1.25;
  core.onFeedback(fb);
  EXPECT_DOUBLE_EQ(core.progress(), fb.progress_m);
  EXPECT_EQ(core.executingSequenceId(), 1u);
}

TEST(LocalPlanningCore, NoGoZonesBlockPath) {
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  const Eigen::Vector2d zone(3.1, 0.1);
  const double reach = 0.8;
  LocalPlanningCore core(mgg_test::sceneParams());
  observe(core, corridor, base);
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(6.1, 0.1))).accepted);
  const auto open = core.plan(soon());
  ASSERT_TRUE(open.path) << open.reason;
  // The straight corridor route runs through the zone.
  ASSERT_TRUE(mgg_test::pathCrosses(open.path->poses, zone, reach));

  NoGoZones zones;
  zones.set({zone}, reach);
  core.setNoGoZones(zones);
  ASSERT_TRUE(
      core.setMode(follow("s2", "r2", drivingPoint(6.1, 0.1))).accepted);
  const auto result = core.plan(soon());
  ASSERT_TRUE(result.path) << result.reason;
  EXPECT_FALSE(mgg_test::pathCrosses(result.path->poses, zone, reach));
}

TEST(LocalPlanningCore, GoalFramesFollowTransforms) {
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  LocalPlanningCore core(mgg_test::sceneParams());
  observe(core, corridor, base);

  // An empty frame is the odometry frame; no lookup is needed.
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(5.1, 0.1))).accepted);
  const auto odom_goal = core.plan(soon());
  ASSERT_TRUE(odom_goal.path) << odom_goal.reason;
  EXPECT_TRUE(odom_goal.path->reaches_goal);
  EXPECT_LT((odom_goal.path->poses.back().head<2>() - Eigen::Vector2d(5.1, 0.1))
                .norm(),
            0.31);

  // A map-frame goal moves with a later map-to-odometry correction.
  Eigen::Isometry3d odom_T_map = Eigen::Isometry3d::Identity();
  bool have_map = true;
  core.setFrameLookup(
      [&](const std::string& frame) -> std::optional<Eigen::Isometry3d> {
        if (frame != "map" || !have_map) return std::nullopt;
        return odom_T_map;
      });
  ASSERT_TRUE(
      core.setMode(follow("s2", "r2", drivingPoint(5.1, 0.1), "map")).accepted);
  const auto before = core.plan(soon());
  ASSERT_TRUE(before.path) << before.reason;
  EXPECT_LT(
      (before.path->poses.back().head<2>() - Eigen::Vector2d(5.1, 0.1)).norm(),
      0.31);
  odom_T_map.translation() = Eigen::Vector3d(0, 1.0, 0);
  const auto after = core.plan(soon());
  ASSERT_TRUE(after.path) << after.reason;
  EXPECT_TRUE(after.path->reaches_goal);
  EXPECT_LT(
      (after.path->poses.back().head<2>() - Eigen::Vector2d(5.1, 1.1)).norm(),
      0.31);

  // No transform: blocked, naming the frame, for goals and for no-go zones.
  have_map = false;
  const auto missing = core.plan(soon());
  EXPECT_FALSE(missing.path);
  EXPECT_EQ(missing.status, LocalStatus::kBlocked);
  EXPECT_EQ(missing.reason, "no_transform:map");
  ASSERT_TRUE(
      core.setMode(follow("s3", "r3", drivingPoint(5.1, 0.1))).accepted);
  NoGoZones zones;
  zones.set({Eigen::Vector2d(20, 20)}, 0.5);
  core.setNoGoZones(zones, "map");
  const auto zones_missing = core.plan(soon());
  EXPECT_FALSE(zones_missing.path);
  EXPECT_EQ(zones_missing.status, LocalStatus::kBlocked);
  EXPECT_EQ(zones_missing.reason, "no_transform:map");
}

TEST(LocalPlanningCore, ObstacleOnRetainedPathInvalidates) {
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  LocalPlanningCore core(mgg_test::sceneParams());
  observe(core, corridor, base);
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(6.1, 0.1))).accepted);
  const auto first = core.plan(soon());
  ASSERT_TRUE(first.path) << first.reason;
  EXPECT_TRUE(core.takeInvalidations().empty());

  // A box across the corridor, on the path.
  corridor.obstacle = Eigen::AlignedBox3d(Eigen::Vector3d(3.1, -2.5, -0.1),
                                          Eigen::Vector3d(3.5, 2.5, 1.0));
  for (int i = 0; i < 4; ++i) core.onScan(corridor.odomScan(base), base);
  const auto invalidations = core.takeInvalidations();
  ASSERT_FALSE(invalidations.empty());
  EXPECT_EQ(invalidations.front().sequence_id, first.path->sequence_id);
  EXPECT_EQ(invalidations.front().session_id, "s1");
  EXPECT_EQ(invalidations.front().epoch, core.epoch());
  EXPECT_GT(core.retainedRechecks(), 0u);

  // The next path breaks from the current pose.
  const auto next = core.plan(soon());
  if (next.path) {
    EXPECT_EQ(next.path->kind, LocalPathKind::kBreak);
    EXPECT_EQ(next.path->extends_sequence_id, first.path->sequence_id);
  }
}

TEST(LocalPlanningCore, NoGoUpdateWithdrawsRetainedPath) {
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  const Eigen::Vector2d zone(3.1, 0.1);
  const double reach = 0.8;
  LocalPlanningCore core(mgg_test::sceneParams());
  observe(core, corridor, base);
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(6.1, 0.1))).accepted);
  const auto first = core.plan(soon());
  ASSERT_TRUE(first.path) << first.reason;
  ASSERT_TRUE(mgg_test::pathCrosses(first.path->poses, zone, reach));

  // Same session: the new zone withdraws the path being driven at once.
  NoGoZones zones;
  zones.set({zone}, reach);
  core.setNoGoZones(zones);
  auto invalidations = core.takeInvalidations();
  ASSERT_EQ(invalidations.size(), 1u);
  EXPECT_EQ(invalidations[0].sequence_id, first.path->sequence_id);
  EXPECT_EQ(invalidations[0].session_id, "s1");
  const auto detour = core.plan(soon());
  ASSERT_TRUE(detour.path) << detour.reason;
  EXPECT_EQ(detour.path->kind, LocalPathKind::kBreak);
  EXPECT_EQ(detour.path->extends_sequence_id, first.path->sequence_id);
  EXPECT_FALSE(mgg_test::pathCrosses(detour.path->poses, zone, reach));

  // Zones re-placed by a later map-to-odometry correction withdraw too.
  Eigen::Isometry3d odom_T_map = Eigen::Isometry3d::Identity();
  bool have_map = true;
  core.setFrameLookup(
      [&](const std::string& frame) -> std::optional<Eigen::Isometry3d> {
        if (frame != "map" || !have_map) return std::nullopt;
        return odom_T_map;
      });
  NoGoZones outside;
  outside.set({Eigen::Vector2d(3.1, 4.1)}, reach);
  core.setNoGoZones(outside, "map");
  EXPECT_TRUE(core.takeInvalidations().empty());
  ASSERT_TRUE(
      core.setMode(follow("s2", "r2", drivingPoint(6.1, 0.1))).accepted);
  const auto straight = core.plan(soon());
  ASSERT_TRUE(straight.path) << straight.reason;
  ASSERT_TRUE(mgg_test::pathCrosses(straight.path->poses, zone, reach));
  odom_T_map.translation() = Eigen::Vector3d(0, -4.0, 0);
  const auto replaced = core.plan(soon());
  invalidations = core.takeInvalidations();
  ASSERT_EQ(invalidations.size(), 1u);
  EXPECT_EQ(invalidations[0].sequence_id, straight.path->sequence_id);
  ASSERT_TRUE(replaced.path) << replaced.reason;
  EXPECT_EQ(replaced.path->kind, LocalPathKind::kBreak);
  EXPECT_FALSE(mgg_test::pathCrosses(replaced.path->poses, zone, reach));

  // Zones without a transform cannot vouch for any path.
  have_map = false;
  core.setNoGoZones(outside, "map");
  invalidations = core.takeInvalidations();
  ASSERT_EQ(invalidations.size(), 1u);
  EXPECT_EQ(invalidations[0].sequence_id, replaced.path->sequence_id);
  EXPECT_EQ(invalidations[0].reason, "no_transform:map");
}

TEST(LocalPlanningCore, PendingDescendantsWithdrawRetainedPath) {
  mgg_test::Corridor corridor;
  LocalPlanningCore core(mgg_test::sceneParams());
  // The ground flood is seeded where the robot stood first; it then drives
  // 2 m in steps short of an odometry reset.
  observe(core, corridor, basePose(0.1, 0.1));
  for (double x = 0.35; x < 2.1; x += 0.25)
    observe(core, corridor, basePose(x, 0.1), 2);
  const StateVec base = basePose(2.1, 0.1);
  observe(core, corridor, base);
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(7.1, 0.1))).accepted);
  const auto path = core.plan(soon());
  ASSERT_TRUE(path.path) << path.reason;
  ASSERT_TRUE(core.takeInvalidations().empty());
  ASSERT_FALSE(path.path->edge_dependencies.empty());

  // An obstacle on the flood seed, behind the robot and out of the path's
  // reach: the seed's withdrawal withdraws every descendant column, and
  // with no recheck budget they stay pending.
  core.setGroundRecheckBudget(0);
  corridor.obstacle = Eigen::AlignedBox3d(Eigen::Vector3d(-0.3, -0.3, -0.1),
                                          Eigen::Vector3d(0.1, 0.5, 0.9));
  MapChange changes;
  for (int i = 0; i < 4; ++i) {
    const auto change = core.onScan(corridor.odomScan(base), base);
    changes.boxes.insert(changes.boxes.end(), change.boxes.begin(),
                         change.boxes.end());
    changes.everything |= change.everything;
  }
  ASSERT_FALSE(changes.boxes.empty());
  ASSERT_FALSE(changes.everything);
  for (const auto& dep : path.path->edge_dependencies)
    ASSERT_FALSE(changeReaches(changes, dep));
  ASSERT_TRUE(std::any_of(path.path->edge_dependencies.begin(),
                          path.path->edge_dependencies.end(),
                          [&](const Eigen::AlignedBox3d& dep) {
                            return core.ground().pending(dep);
                          }));

  const auto invalidations = core.takeInvalidations();
  ASSERT_EQ(invalidations.size(), 1u);
  EXPECT_EQ(invalidations[0].sequence_id, path.path->sequence_id);
}
