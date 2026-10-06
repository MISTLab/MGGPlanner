// The ROS-free local planning cycle the node and the replay runner share:
// session-fenced feedback, no-go zones, inputs in other frames re-transformed
// every cycle, and retained paths withdrawn when the map changes under them.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
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

/// A lidar 0.6 m over the floor whose lowest ring dips 22.5 degrees, as the
/// robots' do: it never sees the floor within 1.45 m of itself, nor the
/// body band under its lowest ring. Rings fall every 0.1 m of floor out to
/// 12 m, then every 0.01 rad from 0.15 rad down to the walls and ceiling.
OdomScan blindScan(const mgg_test::Corridor& corridor, const StateVec& base) {
  OdomScan scan;
  scan.origin = mgg_test::lidarOrigin(base);
  const double height = scan.origin.z() - mgg_test::kFloorTop;
  std::vector<double> elevations;
  for (double range = height / std::tan(0.3927); range <= 12; range += 0.1)
    elevations.push_back(-std::atan2(height, range));
  for (double elevation = -0.15; elevation <= 0.3927; elevation += 0.01)
    elevations.push_back(elevation);
  mgg::test::SyntheticScene scene{corridor.solids};
  for (const double elevation : elevations) {
    for (int a = 0; a < 720; ++a) {
      const double azimuth = 2 * M_PI * (a + 0.5) / 720;
      const Eigen::Vector3d direction(std::cos(elevation) * std::cos(azimuth),
                                      std::cos(elevation) * std::sin(azimuth),
                                      std::sin(elevation));
      double nearest = std::numeric_limits<double>::infinity();
      for (const auto& solid : scene.solids) {
        const auto d = mgg::test::rayBoxDistance(scan.origin, direction, solid);
        if (d && *d < nearest) nearest = *d;
      }
      if (nearest <= 20) scan.points.push_back(scan.origin + nearest * direction);
    }
  }
  return scan;
}

/// Rays from `origin` down onto every 0.1 m of floor within 0.6 m along x
/// and across the corridor: a view from above, into what the robot's own
/// lidar cannot see.
OdomScan surveyScan(const mgg_test::Corridor& corridor,
                    const Eigen::Vector3d& origin) {
  OdomScan scan;
  scan.origin = origin;
  for (double x = origin.x() - 0.6; x <= origin.x() + 0.6; x += 0.1) {
    for (double y = -mgg_test::kWallY + 0.05; y < mgg_test::kWallY; y += 0.1) {
      const Eigen::Vector3d direction =
          (Eigen::Vector3d(x, y, mgg_test::kFloorTop) - origin).normalized();
      double nearest = std::numeric_limits<double>::infinity();
      for (const auto& solid : corridor.solids) {
        const auto d = mgg::test::rayBoxDistance(origin, direction, solid);
        if (d && *d < nearest) nearest = *d;
      }
      if (nearest <= 20) scan.points.push_back(origin + nearest * direction);
    }
  }
  return scan;
}

void observeBlind(LocalPlanningCore& core, const mgg_test::Corridor& corridor,
                  const StateVec& base, int scans = 2) {
  core.onOdometry(base);
  for (int i = 0; i < scans; ++i) core.onScan(blindScan(corridor, base), base);
}

/// The scene's parameters with a standing disk of `radius` (the lidar's
/// ground blind radius is 1.45 m).
LocalPlanningParams standingParams(double radius = 2.0) {
  auto params = mgg_test::sceneParams();
  params.hanging_root_edge_length_max = radius;
  return params;
}

/// The executor driving `path` from `from` to `to` metres along it, every
/// 0.25 m: feedback, odometry and a blind scan at each step. Returns the
/// pose reached.
StateVec drive(LocalPlanningCore& core, const mgg_test::Corridor& corridor,
               const LocalPathPlan& path, double from, double to) {
  StateVec moved = basePose(0, 0);
  for (double progress = from + 0.25; progress <= to + 1e-9; progress += 0.25) {
    LocalFeedback feedback;
    feedback.session_id = path.session_id;
    feedback.epoch = core.epoch();
    feedback.sequence_id = path.sequence_id;
    feedback.executing = true;
    feedback.progress_m = progress;
    feedback.speed_mps = 0.6;
    core.onFeedback(feedback);
    const StateVec at = mgg_test::poseAt(path.poses, progress);
    moved = basePose(at.x(), at.y(), at[3]);
    core.onOdometry(moved);
    core.onScan(blindScan(corridor, moved), moved);
  }
  return moved;
}

int admittedCells(const LocalPlanningCore& core) {
  const auto cells = core.ground().occupancy();
  return static_cast<int>(std::count(cells.begin(), cells.end(), 0));
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

TEST(LocalPlanningCore, TwoDimensionalObjectiveReachesGoal) {
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  LocalPlanningCore core(mgg_test::sceneParams());
  observe(core, corridor, base);
  // As the adapter sends a 2-D objective: seeded at the robot's current
  // altitude (its base), never pre-projected to the driving height.
  const Eigen::Vector3d objective(5.1, 0.1, mgg_test::kBaseZ);
  ASSERT_TRUE(core.setMode(follow("s1", "r1", objective)).accepted);
  const auto result = core.plan(soon());
  ASSERT_TRUE(result.path) << result.reason;
  EXPECT_TRUE(result.path->reaches_goal);
  EXPECT_NEAR(result.path->poses.back().z(), mgg_test::kDrivingZ, 1e-9);
  EXPECT_LT(
      (result.path->poses.back().head<2>() - objective.head<2>()).norm(),
      0.31);
}

TEST(LocalPlanningCore, GlobalGroundEstimateDiffersFromLocal) {
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  LocalPlanningCore core(mgg_test::sceneParams());
  observe(core, corridor, base);
  // The guidance map puts this floor 0.35 m higher than the local map does.
  const double offset = 0.35;
  auto request = follow("s1", "r1",
                        Eigen::Vector3d(14.1, 0.1, mgg_test::kBaseZ + offset));
  request.route = {Eigen::Vector3d(7.1, 0.1, mgg_test::kDrivingZ + offset),
                   Eigen::Vector3d(14.1, 0.1, mgg_test::kDrivingZ + offset)};
  ASSERT_TRUE(core.setMode(request).accepted);
  const auto aim = core.plan(soon());
  ASSERT_TRUE(aim.path) << aim.reason;
  EXPECT_FALSE(aim.path->reaches_goal);
  EXPECT_NEAR(aim.path->poses.back().z(), mgg_test::kDrivingZ, 1e-9);
  EXPECT_LT((aim.path->poses.back().head<2>() - Eigen::Vector2d(7.1, 0.1))
                .norm(),
            0.31);

  ASSERT_TRUE(core.setMode(follow("s2", "r2",
                                  Eigen::Vector3d(6.1, 0.1,
                                                  mgg_test::kBaseZ + offset)))
                  .accepted);
  const auto goal = core.plan(soon());
  ASSERT_TRUE(goal.path) << goal.reason;
  EXPECT_TRUE(goal.path->reaches_goal);
  EXPECT_NEAR(goal.path->poses.back().z(), mgg_test::kDrivingZ, 1e-9);
}

TEST(LocalPlanningCore, UnobservedGoalGroundWaitsForMap) {
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  LocalPlanningCore core(mgg_test::sceneParams());
  observe(core, corridor, base);
  // Behind the corridor wall: inside the window, never observed.
  ASSERT_TRUE(core.setMode(follow("s1", "r1",
                                  Eigen::Vector3d(3.1, 4.1, mgg_test::kBaseZ)))
                  .accepted);
  const auto result = core.plan(soon());
  EXPECT_FALSE(result.path);
  EXPECT_EQ(result.status, LocalStatus::kWaitingForMap);
  EXPECT_EQ(result.reason, "goal_ground_unknown");
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

  // A box across the corridor, on the path where the body reaches before
  // the commitment ends (1.2 m on at 0.6 m/s, the body 0.5 m ahead).
  ASSERT_NEAR(first.path->commit_length_m, 1.2, 1e-6);
  corridor.obstacle = Eigen::AlignedBox3d(Eigen::Vector3d(1.5, -2.5, -0.1),
                                          Eigen::Vector3d(1.9, 2.5, 1.0));
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

// p1a-acc-2: 99 live breaks, each a stop, came from map changes beyond the
// commitment (robot_2: an occupied segment 3 m on; a 1.2 m commitment still
// passed). The suffix alone is withdrawn: no invalidation, an immediate
// replan from the commitment, and motion goes on.
TEST(LocalPlanningCore, SuffixObstacleKeepsTheCommitment) {
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  LocalPlanningCore core(mgg_test::sceneParams());
  observe(core, corridor, base);
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(6.1, 0.1))).accepted);
  const auto first = core.plan(soon());
  ASSERT_TRUE(first.path) << first.reason;
  EXPECT_FALSE(core.takeReplanRequest());
  // On the path 3 m on, leaving a gap on the left wide enough for the body
  // between the refused rings the layer keeps round the box and the wall.
  const Eigen::AlignedBox3d box(Eigen::Vector3d(3.1, -2.5, -0.1),
                                Eigen::Vector3d(3.5, 0.3, 1.0));
  ASSERT_TRUE(mgg_test::pathCrosses(first.path->poses, {3.3, 0.1}, 0.5));
  corridor.obstacle = box;
  for (int i = 0; i < 4; ++i) core.onScan(corridor.odomScan(base), base);
  EXPECT_TRUE(core.takeInvalidations().empty());
  EXPECT_EQ(core.suffixWithdrawals(), 1u);
  EXPECT_TRUE(core.takeReplanRequest());
  EXPECT_FALSE(core.takeReplanRequest());
  const auto next = core.plan(soon());
  ASSERT_TRUE(next.path) << next.reason;
  EXPECT_EQ(next.path->kind, LocalPathKind::kExtend);
  EXPECT_EQ(next.path->extends_sequence_id, first.path->sequence_id);
  const auto rest = remainingCommitment(*first.path, 0);
  ASSERT_GE(next.path->prefix_length, rest.poses.size());
  for (size_t i = 0; i < rest.poses.size(); ++i)
    EXPECT_EQ(next.path->poses[i], rest.poses[i]);
  for (const auto& pose : next.path->poses)
    EXPECT_FALSE(mgg_test::footprintOverlaps(pose, {1.0, 0.6}, box));
}

// p1a-acc-2: all 109 live splice rejections were "wrong parent". However
// many reports naming an older path cross a publication in flight, none
// moves the tip back; only the executor's explicit refusal does, and it
// covers the refused path's undecided predecessors and its descendants, as
// the executor's splicer (one current path, decided in order) refuses them.
TEST(LocalPlanningCore, StaleFeedbackCannotMoveTheTipBack) {
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  LocalPlanningCore core(mgg_test::sceneParams());
  observe(core, corridor, base);
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(6.1, 0.1))).accepted);
  const auto report = [&](std::uint64_t sequence, bool refused = false) {
    LocalFeedback feedback;
    feedback.session_id = "s1";
    feedback.epoch = core.epoch();
    feedback.sequence_id = sequence;
    feedback.executing = sequence != 0 && !refused;
    feedback.refused = refused;
    core.onFeedback(feedback);
  };
  const auto plan = [&]() {
    auto result = core.plan(soon());
    EXPECT_TRUE(result.path) << result.reason;
    return result.path ? *result.path : LocalPathPlan{};
  };
  const auto p1 = plan();
  report(p1.sequence_id);
  const auto p2 = plan();
  EXPECT_EQ(p2.extends_sequence_id, p1.sequence_id);
  // Queued reports naming p1 that crossed p2 in flight, any number of them.
  for (int i = 0; i < 4; ++i) report(p1.sequence_id);
  EXPECT_EQ(core.executingSequenceId(), p2.sequence_id);
  EXPECT_EQ(core.acknowledgedSequenceId(), p1.sequence_id);
  const auto p3 = plan();
  EXPECT_EQ(p3.extends_sequence_id, p2.sequence_id);  // not a sibling of p2
  // p2 accepted while p3 is in flight; a later stale report naming p1 is
  // older than the acknowledged path and changes nothing.
  report(p2.sequence_id);
  report(p1.sequence_id);
  EXPECT_EQ(core.acknowledgedSequenceId(), p2.sequence_id);
  EXPECT_EQ(core.executingSequenceId(), p3.sequence_id);
  const auto p4 = plan();
  EXPECT_EQ(p4.extends_sequence_id, p3.sequence_id);
  // The executor refused p3 (so p4, extending it, is refused too) while
  // still on p2: the next plan extends p2.
  report(p2.sequence_id);
  EXPECT_EQ(core.executingSequenceId(), p4.sequence_id);
  report(p3.sequence_id, true);
  EXPECT_EQ(core.executingSequenceId(), p2.sequence_id);
  const auto p5 = plan();
  EXPECT_EQ(p5.extends_sequence_id, p2.sequence_id);
  // A refusal of p4 as it arrives changes nothing more: p5 stays in flight.
  report(p4.sequence_id, true);
  EXPECT_EQ(core.executingSequenceId(), p5.sequence_id);
  report(p5.sequence_id);
  // The executor finished p5: IDLE, with p6 in flight, which it refuses
  // (it extends p5). An IDLE sent before a START arrived does not undo it
  // either; the executor's acceptance of it is its report naming it.
  const auto p6 = plan();
  EXPECT_EQ(p6.extends_sequence_id, p5.sequence_id);
  report(0);
  EXPECT_EQ(core.executingSequenceId(), p6.sequence_id);
  report(p6.sequence_id, true);
  EXPECT_EQ(core.executingSequenceId(), 0u);
  const auto start = plan();
  EXPECT_EQ(start.kind, LocalPathKind::kStart);
  report(0);
  EXPECT_EQ(core.executingSequenceId(), start.sequence_id);
  report(start.sequence_id);
  EXPECT_EQ(core.acknowledgedSequenceId(), start.sequence_id);
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

TEST(LocalPlanningCore, RetentionDoesNotInvalidateExtensionChain) {
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  LocalPlanningCore core(mgg_test::sceneParams());
  observe(core, corridor, base);
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(6.1, 0.1))).accepted);
  const auto revision = core.map().revision();
  // No feedback prunes the chain early: cross the retention boundary twice.
  const std::uint64_t last_sequence = kMaxRetained + 2;
  for (std::uint64_t sequence = 1; sequence <= last_sequence; ++sequence) {
    const auto result = core.plan(soon());
    ASSERT_TRUE(result.path) << result.reason;
    EXPECT_EQ(result.path->sequence_id, sequence);
    EXPECT_EQ(result.path->kind, sequence == 1 ? LocalPathKind::kStart
                                              : LocalPathKind::kExtend);
    EXPECT_EQ(result.path->extends_sequence_id, sequence - 1);
    EXPECT_EQ(core.map().revision(), revision);
    EXPECT_TRUE(core.takeInvalidations().empty()) << "sequence " << sequence;
  }
  // Retirement really bounds the retained set, without disabling genuine
  // withdrawals of its remaining paths (including non-executing ancestors).
  NoGoZones zones;
  zones.set({Eigen::Vector2d(3.1, 0.1)}, 0.8);
  core.setNoGoZones(zones);
  const auto invalidations = core.takeInvalidations();
  ASSERT_EQ(invalidations.size(), kMaxRetained);
  EXPECT_EQ(invalidations.front().sequence_id,
            last_sequence - kMaxRetained + 1);
  EXPECT_EQ(invalidations.back().sequence_id, last_sequence);
}

TEST(LocalPlanningCore, RetentionInvalidatesFeedbackConfirmedPath) {
  // Explicit refusals reproduce the rejected-sibling case. One-shot feedback
  // also has to survive intervening publications without another executor ack.
  for (const bool repeat_feedback : {false, true}) {
    SCOPED_TRACE(repeat_feedback);
    mgg_test::Corridor corridor;
    const StateVec base = basePose(0.1, 0.1);
    LocalPlanningCore core(mgg_test::sceneParams());
    observe(core, corridor, base);
    ASSERT_TRUE(
        core.setMode(follow("s1", "r1", drivingPoint(6.1, 0.1))).accepted);
    const auto first = core.plan(soon());
    ASSERT_TRUE(first.path) << first.reason;
    ASSERT_EQ(first.path->sequence_id, 1u);
    LocalFeedback feedback;
    feedback.session_id = "s1";
    feedback.epoch = core.epoch();
    feedback.sequence_id = first.path->sequence_id;
    feedback.executing = true;
    core.onFeedback(feedback);
    for (size_t i = 0; i < kMaxRetained; ++i) {
      // The executor refuses each new path and stays on the first.
      if (repeat_feedback && i > 0) {
        LocalFeedback refusal = feedback;
        refusal.executing = false;
        refusal.refused = true;
        refusal.sequence_id = core.executingSequenceId();
        core.onFeedback(refusal);
        core.onFeedback(feedback);
      }
      const auto next = core.plan(soon());
      ASSERT_TRUE(next.path) << next.reason;
      EXPECT_EQ(next.path->kind, LocalPathKind::kExtend);
      if (repeat_feedback)
        EXPECT_EQ(next.path->extends_sequence_id, first.path->sequence_id);
      if (i + 1 < kMaxRetained)
        EXPECT_TRUE(core.takeInvalidations().empty());
    }
    const auto invalidations = core.takeInvalidations();
    ASSERT_EQ(invalidations.size(), 1u);
    EXPECT_EQ(invalidations[0].sequence_id, first.path->sequence_id);
    EXPECT_EQ(invalidations[0].session_id, "s1");
    EXPECT_EQ(invalidations[0].epoch, core.epoch());
    EXPECT_EQ(invalidations[0].reason, "retention limit");
    const auto next = core.plan(soon());
    ASSERT_TRUE(next.path) << next.reason;
    EXPECT_TRUE(core.takeInvalidations().empty());
  }
}

TEST(LocalPlanningCore, RetentionFeedbackStopIsSessionAndEpochFenced) {
  for (const std::string stop_kind : {"current", "old_session", "old_epoch"}) {
    SCOPED_TRACE(stop_kind);
    mgg_test::Corridor corridor;
    const StateVec base = basePose(0.1, 0.1);
    LocalPlanningCore core(mgg_test::sceneParams());
    observe(core, corridor, base);
    ASSERT_TRUE(
        core.setMode(follow("s1", "r1", drivingPoint(6.1, 0.1))).accepted);
    const auto first = core.plan(soon());
    ASSERT_TRUE(first.path) << first.reason;
    LocalFeedback feedback;
    feedback.session_id = "s1";
    feedback.epoch = core.epoch();
    feedback.sequence_id = first.path->sequence_id;
    feedback.executing = true;
    core.onFeedback(feedback);
    feedback.executing = false;
    if (stop_kind == "old_session") feedback.session_id = "old";
    if (stop_kind == "old_epoch") ++feedback.epoch;
    core.onFeedback(feedback);
    for (size_t i = 0; i < kMaxRetained; ++i) {
      const auto next = core.plan(soon());
      ASSERT_TRUE(next.path) << next.reason;
    }
    const auto invalidations = core.takeInvalidations();
    if (stop_kind == "current") {
      EXPECT_TRUE(invalidations.empty());
    } else {
      ASSERT_EQ(invalidations.size(), 1u);
      EXPECT_EQ(invalidations[0].sequence_id, first.path->sequence_id);
      EXPECT_EQ(invalidations[0].reason, "retention limit");
    }
  }
}

TEST(LocalPlanningCore, StandingDiskStartsBlindAnywhere) {
  // Legacy's standing start, centred on the robot every cycle: wherever it
  // stands, its lidar blind to the floor within 1.45 m, it departs. Without
  // the disk it stays where it stands.
  mgg_test::Corridor corridor;
  for (const StateVec& base : {basePose(0.1, 0.1), basePose(-2.1, 0.1, 0.6),
                               basePose(5.9, 0.7, -2.0)}) {
    for (const double radius : {0.0, 2.0}) {
      SCOPED_TRACE(::testing::Message() << "at " << base.transpose()
                                        << ", radius " << radius);
      LocalPlanningCore core(standingParams(radius));
      core.setGroundRecheckBudget(5);
      observeBlind(core, corridor, base);
      ASSERT_TRUE(core.setMode(follow("s1", "r1",
                                      drivingPoint(base.x() + 4.5, 0.1)))
                      .accepted);
      const auto result = core.plan(soon(5));
      if (radius == 0) {
        EXPECT_FALSE(core.standingStart());
        EXPECT_EQ(admittedCells(core), 0);
        EXPECT_FALSE(result.path) << result.reason;
        continue;
      }
      ASSERT_TRUE(core.standingStart());
      EXPECT_TRUE(core.standingStart()->center.isApprox(base.head<2>()));
      EXPECT_DOUBLE_EQ(core.standingStart()->radius, 2.0);
      ASSERT_TRUE(result.path) << result.reason;
      EXPECT_EQ(result.status, LocalStatus::kMoving);
      EXPECT_TRUE(result.path->reaches_goal);
    }
  }
}

TEST(LocalPlanningCore, StandingDiskKeepsObservedDrops) {
  // A pit across the corridor 0.8 to 1.2 m ahead, in the robot's blind
  // ring and its disk, observed from above (surveyScan): still refused.
  // The same scans over an unbroken floor give a path to the goal.
  for (const bool pit : {false, true}) {
    SCOPED_TRACE(pit ? "pit" : "floor");
    mgg_test::Corridor corridor;
    const Eigen::AlignedBox3d hole(Eigen::Vector3d(0.9, -2.5, -1.1),
                                   Eigen::Vector3d(1.3, 2.5, 0.0));
    if (pit) {
      Eigen::AlignedBox3d beyond = corridor.solids.front();
      corridor.solids.front().max().x() = hole.min().x();
      beyond.min().x() = hole.max().x();
      corridor.solids.push_back(beyond);
      // Its bottom, 1 m down.
      corridor.solids.emplace_back(
          Eigen::Vector3d(-30, -10, mgg_test::kFloorTop - 1.2),
          Eigen::Vector3d(40, 10, mgg_test::kFloorTop - 1.0));
    }
    LocalPlanningCore core(standingParams());
    core.setGroundRecheckBudget(5);
    const StateVec base = basePose(0.1, 0.1);
    observeBlind(core, corridor, base);
    core.onScan(surveyScan(corridor, Eigen::Vector3d(1.1, 0.1, 1.9)), base);
    ASSERT_TRUE(
        core.setMode(follow("s1", "r1", drivingPoint(4.6, 0.1))).accepted);
    const auto result = core.plan(soon(5));
    ASSERT_TRUE(core.standingStart());
    if (!pit) {
      ASSERT_TRUE(result.path) << result.reason;
      EXPECT_TRUE(result.path->reaches_goal);
      continue;
    }
    if (result.path) {
      EXPECT_FALSE(result.path->reaches_goal);
      for (const auto& pose : result.path->poses)
        EXPECT_FALSE(mgg_test::footprintOverlaps(pose, {1.0, 0.6}, hole));
    }
  }
}

TEST(LocalPlanningCore, UnknownBeyondTheStandingDiskStillRefuses) {
  // A disk smaller than the blind ring: the unseen ground between its edge
  // and the observed floor stays unknown, and nothing crosses it.
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  LocalPlanningCore core(standingParams(1.0));
  core.setGroundRecheckBudget(5);
  observeBlind(core, corridor, base);
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(4.6, 0.1))).accepted);
  const auto result = core.plan(soon(5));
  ASSERT_TRUE(core.standingStart());
  EXPECT_EQ(core.ground().verdict({0.3, 0.1}), GroundVerdict::kAdmitted);
  EXPECT_NE(core.ground().verdict({1.35, 0.1}), GroundVerdict::kAdmitted);
  EXPECT_FALSE(result.path) << result.reason;
}

TEST(LocalPlanningCore, StandingDiskSurvivesSessionStopAndRestart) {
  // The live failure: a session ended, guidance went idle, and Explore could
  // not restart. The disk belongs to the local core alone: after IDLE, a
  // drive and a new session, the robot departs again.
  mgg_test::Corridor corridor;
  LocalPlanningCore core(standingParams());
  core.setGroundRecheckBudget(5);
  observeBlind(core, corridor, basePose(0.1, 0.1));
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(4.6, 0.1))).accepted);
  const auto first = core.plan(soon(5));
  ASSERT_TRUE(first.path) << first.reason;
  const StateVec stopped = drive(core, corridor, *first.path, 0, 1.0);

  LocalModeRequest idle;
  idle.session_id = "s1";
  idle.request_id = "r-idle";
  idle.mode = LocalMode::kIdle;
  ASSERT_TRUE(core.setMode(idle).accepted);
  LocalGuidance none;
  none.session_id = "s1";
  none.sequence_id = 9;
  none.reason = "idle";
  core.setGuidance(none);
  EXPECT_EQ(core.plan(soon(5)).status, LocalStatus::kNoLocalTarget);
  observeBlind(core, corridor, stopped);

  ASSERT_TRUE(
      core.setMode(follow("s2", "r2", drivingPoint(6.1, 0.1))).accepted);
  const auto restarted = core.plan(soon(5));
  ASSERT_TRUE(restarted.path) << restarted.reason;
  EXPECT_EQ(restarted.path->session_id, "s2");
  EXPECT_EQ(restarted.path->kind, LocalPathKind::kStart);
  ASSERT_TRUE(core.standingStart());
  EXPECT_TRUE(core.standingStart()->center.isApprox(
      (stopped.head<2>()), 1e-9));
}

TEST(LocalPlanningCore, MovingStandingDiskKeepsTheExecutingPath) {
  // The disk follows the robot each cycle across its blind departure; no
  // path is withdrawn, and every next path extends the executing one.
  mgg_test::Corridor corridor;
  LocalPlanningCore core(standingParams());
  core.setGroundRecheckBudget(5);
  observeBlind(core, corridor, basePose(0.1, 0.1));
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(6.1, 0.1))).accepted);
  const auto first = core.plan(soon(5));
  ASSERT_TRUE(first.path) << first.reason;
  // The deployed recheck budget from here: each move's ground work fits it.
  core.setGroundRecheckBudget(LocalPlanningParams{}.ground_recheck_s);
  LocalPathPlan executing = *first.path;
  double progress = 0;
  for (int step = 0; step < 8; ++step) {
    SCOPED_TRACE(::testing::Message() << "step " << step);
    const StateVec at = drive(core, corridor, executing, progress, progress + 0.25);
    progress += 0.25;
    EXPECT_TRUE(core.takeInvalidations().empty());
    const auto next = core.plan(soon(5));
    ASSERT_TRUE(core.standingStart());
    EXPECT_TRUE(core.standingStart()->center.isApprox(at.head<2>(), 1e-9));
    EXPECT_EQ(core.ground().pendingCount(), 0);
    EXPECT_TRUE(core.takeInvalidations().empty());
    if (next.path) {
      EXPECT_EQ(next.path->kind, LocalPathKind::kExtend);
      EXPECT_EQ(next.path->extends_sequence_id, executing.sequence_id);
      executing = *next.path;
      progress = 0;
    }
  }
}

TEST(LocalPlanningCore, RetainedPathKeepsTheDiskItWasCertifiedWith) {
  // The executing departure was certified with the disk round its start.
  // The disk then moves 2.2 m aside with the robot's odometry, off that
  // departure's blind ground, and the next path is refused. A map change
  // reaching the departure rechecks it with its own disk: it stays.
  mgg_test::Corridor corridor;
  LocalPlanningCore core(standingParams());
  core.setGroundRecheckBudget(5);
  observeBlind(core, corridor, basePose(0.1, 0.1));
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(4.6, 0.1))).accepted);
  const auto first = core.plan(soon(5));
  ASSERT_TRUE(first.path) << first.reason;
  LocalFeedback feedback;
  feedback.session_id = "s1";
  feedback.epoch = core.epoch();
  feedback.sequence_id = first.path->sequence_id;
  feedback.executing = true;
  core.onFeedback(feedback);
  StateVec aside = basePose(0.1, 0.1);
  for (const double y : {-0.4, -0.9, -1.4, -1.9, -2.1}) {
    aside = basePose(0.1, y);
    core.onOdometry(aside);
  }
  const auto next = core.plan(soon(5));
  ASSERT_TRUE(core.standingStart());
  ASSERT_FALSE(core.standingStart()->covers(first.path->poses.front().head<2>()));
  if (next.path) {
    LocalFeedback refusal = feedback;
    refusal.sequence_id = next.path->sequence_id;
    refusal.executing = false;
    refusal.refused = true;
    core.onFeedback(refusal);
  }
  ASSERT_EQ(core.executingSequenceId(), first.path->sequence_id);
  ASSERT_TRUE(core.takeInvalidations().empty());
  // A cable 1.2 m over the floor beside the departure, 1.5 m along it:
  // a change it depends on, harmless, and nothing seen of its blind start.
  OdomScan ahead;
  ahead.origin = mgg_test::lidarOrigin(aside);
  for (double x = 1.3; x <= 1.7; x += 0.1)
    ahead.points.emplace_back(x, 0.7, 1.1);
  const auto rechecks = core.retainedRechecks();
  core.onScan(ahead, aside);
  ASSERT_GT(core.retainedRechecks(), rechecks);
  for (const auto& invalidation : core.takeInvalidations())
    EXPECT_NE(invalidation.sequence_id, first.path->sequence_id);
}

TEST(LocalPlanningCore, MapChangeWithdrawsAPathAfterTheDiskMoved) {
  // The executing path keeps the disk it was certified with, but a real map
  // change that makes its commitment unsafe still withdraws it.
  mgg_test::Corridor corridor;
  LocalPlanningCore core(standingParams());
  core.setGroundRecheckBudget(5);
  observeBlind(core, corridor, basePose(0.1, 0.1));
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(6.1, 0.1))).accepted);
  const auto first = core.plan(soon(5));
  ASSERT_TRUE(first.path) << first.reason;
  const StateVec at = drive(core, corridor, *first.path, 0, 0.5);
  const auto next = core.plan(soon(5));
  ASSERT_TRUE(next.path) << next.reason;
  ASSERT_TRUE(core.standingStart());
  ASSERT_FALSE(core.standingStart()->center.isApprox(
      first.path->poses.front().head<2>(), 1e-3));
  ASSERT_TRUE(core.takeInvalidations().empty());
  // A box across the corridor 1 m ahead, within the commitment (blindScan
  // sees solids only).
  corridor.solids.emplace_back(Eigen::Vector3d(at.x() + 0.9, -2.5, -0.1),
                               Eigen::Vector3d(at.x() + 1.3, 2.5, 1.0));
  for (int i = 0; i < 3; ++i) core.onScan(blindScan(corridor, at), at);
  const auto invalidations = core.takeInvalidations();
  ASSERT_FALSE(invalidations.empty());
  EXPECT_EQ(invalidations.back().sequence_id, next.path->sequence_id);
  EXPECT_EQ(invalidations.back().reason,
            "map change withdrew the path's certification");
}
