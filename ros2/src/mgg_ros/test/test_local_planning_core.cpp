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

void observeBlind(LocalPlanningCore& core, const mgg_test::Corridor& corridor,
                  const StateVec& base, int scans = 2) {
  core.onOdometry(base);
  for (int i = 0; i < scans; ++i) core.onScan(blindScan(corridor, base), base);
}

/// Guidance without a target, carrying the guidance planner's standing
/// start block.
LocalGuidance standingGuidance(const std::string& session,
                               std::uint64_t sequence, bool valid,
                               const Eigen::Vector2d& center,
                               double radius = 2.0, std::uint64_t boot = 7) {
  LocalGuidance g;
  g.session_id = session;
  g.sequence_id = sequence;
  g.kind = GuidanceKind::kNone;
  g.standing_start.valid = valid;
  g.standing_start.center = Eigen::Vector3d(center.x(), center.y(), 0);
  g.standing_start.radius = radius;
  g.standing_start.boot = boot;
  return g;
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

TEST(LocalPlanningCore, StandingStartMovesFromRest) {
  // Scans that leave the floor round the robot unseen. Without the guidance
  // planner's proof the robot stays where it stands; with it, it departs.
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  LocalPlanningCore core(mgg_test::sceneParams());
  core.setGroundRecheckBudget(5);
  observeBlind(core, corridor, base);
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(4.6, 0.1))).accepted);
  EXPECT_EQ(admittedCells(core), 0);
  const auto blind = core.plan(soon(5));
  EXPECT_FALSE(blind.path) << blind.reason;

  core.setGuidance(standingGuidance("s1", 1, true, {0.1, 0.1}));
  ASSERT_TRUE(core.standingStart());
  EXPECT_GT(admittedCells(core), 0);
  const auto moving = core.plan(soon(5));
  ASSERT_TRUE(moving.path) << moving.reason;
  EXPECT_EQ(moving.status, LocalStatus::kMoving);
  EXPECT_TRUE(moving.path->reaches_goal);
  EXPECT_GE(mgg_test::pathLength(moving.path->poses), 4.2);
}

TEST(LocalPlanningCore, StandingStartHoldsAcrossItsDiskThenExpires) {
  // The robot drives its departure across the blind disk, past 0.5 m to its
  // edge: the executing path stays certified until the anchor leaves the
  // disk. Then the prior is revoked for good, and the path is re-certified
  // without it.
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  LocalPlanningCore core(mgg_test::sceneParams());
  core.setGroundRecheckBudget(5);
  observeBlind(core, corridor, base);
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(4.6, 0.1))).accepted);
  core.setGuidance(standingGuidance("s1", 1, true, {0.1, 0.1}));
  const auto departure = core.plan(soon(5));
  ASSERT_TRUE(departure.path) << departure.reason;
  const auto& poses = departure.path->poses;
  ASSERT_TRUE(core.takeInvalidations().empty());
  bool left = false;
  for (double progress = 0.25; progress <= 2.6; progress += 0.25) {
    const StateVec at = mgg_test::poseAt(poses, progress);
    LocalFeedback feedback;
    feedback.session_id = "s1";
    feedback.epoch = core.epoch();
    feedback.sequence_id = departure.path->sequence_id;
    feedback.executing = true;
    feedback.progress_m = progress;
    feedback.speed_mps = 0.6;
    core.onFeedback(feedback);
    const StateVec moved = basePose(at.x(), at.y(), at[3]);
    const auto rechecks = core.retainedRechecks();
    core.onOdometry(moved);
    core.onScan(blindScan(corridor, moved), moved);
    const bool inside =
        (at.head<2>() - Eigen::Vector2d(0.1, 0.1)).norm() <= 2.0;
    EXPECT_TRUE(core.takeInvalidations().empty()) << "at " << progress;
    EXPECT_EQ(core.standingStart().has_value(), inside) << "at " << progress;
    if (!inside && !left) {
      left = true;
      EXPECT_GT(core.retainedRechecks(), rechecks);
    }
  }
  ASSERT_TRUE(left);
  // Left, it never stands again, whatever the guidance says.
  core.setGuidance(standingGuidance("s1", 2, true, {0.1, 0.1}));
  EXPECT_FALSE(core.standingStart());
}

TEST(LocalPlanningCore, StandingStartRevocationWithdraws) {
  // Expiry: the guidance planner's proof lapses (valid=false). Admissions
  // and the path that relied on the prior are withdrawn, and the same
  // unseen ground refuses again: a revocation after the prior was applied
  // is final for its boot, whatever valid block follows.
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  LocalPlanningCore core(mgg_test::sceneParams());
  core.setGroundRecheckBudget(5);
  observeBlind(core, corridor, base);
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(4.6, 0.1))).accepted);
  core.setGuidance(standingGuidance("s1", 1, true, {0.1, 0.1}));
  const auto first = core.plan(soon(5));
  ASSERT_TRUE(first.path) << first.reason;
  ASSERT_TRUE(core.takeInvalidations().empty());

  core.setGuidance(standingGuidance("s1", 2, false, {0.1, 0.1}));
  EXPECT_FALSE(core.standingStart());
  const auto invalidations = core.takeInvalidations();
  ASSERT_EQ(invalidations.size(), 1u);
  EXPECT_EQ(invalidations.front().sequence_id, first.path->sequence_id);
  EXPECT_EQ(admittedCells(core), 0);
  const auto refused = core.plan(soon(5));
  EXPECT_FALSE(refused.path) << refused.reason;

  core.setGuidance(standingGuidance("s1", 3, true, {0.1, 0.1}));
  EXPECT_FALSE(core.standingStart());
  EXPECT_EQ(admittedCells(core), 0);
  const auto still = core.plan(soon(5));
  EXPECT_FALSE(still.path) << still.reason;
  // Nor does a new session bring it back.
  ASSERT_TRUE(
      core.setMode(follow("s2", "r2", drivingPoint(4.6, 0.1))).accepted);
  core.setGuidance(standingGuidance("s2", 4, true, {0.1, 0.1}));
  EXPECT_FALSE(core.standingStart());
}

TEST(LocalPlanningCore, StandingStartRevokedBeforeItsFirstApplicationMayApply) {
  // A revocation before any prior was applied (the proof not yet there) is
  // not final: the first valid block then applies.
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  LocalPlanningCore core(mgg_test::sceneParams());
  core.setGroundRecheckBudget(5);
  observeBlind(core, corridor, base);
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(4.6, 0.1))).accepted);
  core.setGuidance(standingGuidance("s1", 1, false, {0.1, 0.1}));
  EXPECT_FALSE(core.standingStart());
  EXPECT_FALSE(core.plan(soon(5)).path);
  core.setGuidance(standingGuidance("s1", 2, true, {0.1, 0.1}));
  ASSERT_TRUE(core.standingStart());
  const auto moving = core.plan(soon(5));
  EXPECT_TRUE(moving.path) << moving.reason;
}

TEST(LocalPlanningCore, StandingStartIgnoresStaleAndRepeatedBlocks) {
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  {
    // Before any application: a valid block older than a revocation does
    // not apply; a newer valid one does.
    LocalPlanningCore core(mgg_test::sceneParams());
    observeBlind(core, corridor, base);
    ASSERT_TRUE(
        core.setMode(follow("s1", "r1", drivingPoint(4.6, 0.1))).accepted);
    core.setGuidance(standingGuidance("s1", 5, false, {0.1, 0.1}));
    core.setGuidance(standingGuidance("s1", 4, true, {0.1, 0.1}));
    EXPECT_FALSE(core.standingStart());
    core.setGuidance(standingGuidance("s1", 5, true, {0.1, 0.1}));
    EXPECT_FALSE(core.standingStart());  // a repeated identity
    core.setGuidance(standingGuidance("s1", 6, true, {0.1, 0.1}));
    EXPECT_TRUE(core.standingStart());
  }
  {
    // Applied: a repeated identity revokes nothing, and a valid block older
    // than the revocation that followed does not undo it.
    LocalPlanningCore core(mgg_test::sceneParams());
    observeBlind(core, corridor, base);
    ASSERT_TRUE(
        core.setMode(follow("s1", "r1", drivingPoint(4.6, 0.1))).accepted);
    core.setGuidance(standingGuidance("s1", 1, true, {0.1, 0.1}));
    ASSERT_TRUE(core.standingStart());
    core.setGuidance(standingGuidance("s1", 1, false, {0.1, 0.1}));
    EXPECT_TRUE(core.standingStart());
    core.setGuidance(standingGuidance("s1", 3, false, {0.1, 0.1}));
    EXPECT_FALSE(core.standingStart());
    core.setGuidance(standingGuidance("s1", 2, true, {0.1, 0.1}));
    EXPECT_FALSE(core.standingStart());
  }
}

TEST(LocalPlanningCore, StandingStartIsHeldToTheBootAndDiskFirstApplied) {
  // Once applied, another boot can neither replace, enlarge, shift nor
  // revoke the disk, and its own boot's later blocks only keep or revoke
  // it: the disk stays where it was first placed.
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  LocalPlanningCore core(mgg_test::sceneParams());
  observeBlind(core, corridor, base);
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(4.6, 0.1))).accepted);
  core.setGuidance(standingGuidance("s1", 1, true, {0.1, 0.1}, 2.0, 7));
  const auto applied = core.standingStart();
  ASSERT_TRUE(applied);
  const auto same = [&] {
    const auto now = core.standingStart();
    return now && now->center == applied->center &&
           now->radius == applied->radius;
  };
  core.takeInvalidations();
  core.setGuidance(standingGuidance("s1", 2, true, {1.1, 0.1}, 4.0, 8));
  EXPECT_TRUE(same());
  core.setGuidance(standingGuidance("s1", 3, true, {-0.9, 0.1}, 3.0, 6));
  EXPECT_TRUE(same());
  core.setGuidance(standingGuidance("s1", 4, false, {0.1, 0.1}, 2.0, 8));
  EXPECT_TRUE(same());
  core.setGuidance(standingGuidance("s1", 5, true, {1.1, 0.1}, 4.0, 7));
  EXPECT_TRUE(same());
  EXPECT_TRUE(core.takeInvalidations().empty());
  core.setGuidance(standingGuidance("s1", 6, false, {0.1, 0.1}, 2.0, 7));
  EXPECT_FALSE(core.standingStart());
}

TEST(LocalPlanningCore, EarlyGuidanceKeepsTheNewestStandingBlock) {
  // Guidance answering before the session starts, out of order: the held
  // message is the newest, so a stale valid block cannot override the
  // revocation that superseded it.
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  LocalPlanningCore core(mgg_test::sceneParams());
  observeBlind(core, corridor, base);
  core.setGuidance(standingGuidance("s1", 2, false, {0.1, 0.1}));
  core.setGuidance(standingGuidance("s1", 1, true, {0.1, 0.1}));
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(4.6, 0.1))).accepted);
  EXPECT_FALSE(core.standingStart());
  EXPECT_EQ(core.guidanceSequenceId(), 2u);
  core.setGuidance(standingGuidance("s1", 3, true, {0.1, 0.1}));
  EXPECT_TRUE(core.standingStart());
  // A held block older than one already accepted is fenced at setMode.
  core.setGuidance(standingGuidance("s2", 2, true, {0.1, 0.1}));
  ASSERT_TRUE(
      core.setMode(follow("s2", "r2", drivingPoint(4.6, 0.1))).accepted);
  EXPECT_FALSE(core.standingStart());
  core.setGuidance(standingGuidance("s2", 4, true, {0.1, 0.1}));
  EXPECT_TRUE(core.standingStart());
}

TEST(LocalPlanningCore, RestartAwayFromTheDiskHasNoStandingStart) {
  // A local planner restarted mid-run beside the disk of a latched proof:
  // its own odometry shows the robot is not in it, so no prior, ever.
  mgg_test::Corridor corridor;
  const StateVec away = basePose(3.1, 0.1);
  LocalPlanningCore core(mgg_test::sceneParams());
  observeBlind(core, corridor, away);
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(4.6, 0.1))).accepted);
  core.setGuidance(standingGuidance("s1", 1, true, {0.1, 0.1}));
  EXPECT_FALSE(core.standingStart());
  EXPECT_FALSE(core.plan(soon(5)).path);
  core.onOdometry(basePose(2.6, 0.1));
  core.onOdometry(basePose(1.6, 0.1));
  core.setGuidance(standingGuidance("s1", 2, true, {0.1, 0.1}));
  EXPECT_FALSE(core.standingStart());
}

TEST(LocalPlanningCore, OdometryJumpRevokesStandingStartForGood) {
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  LocalPlanningCore core(mgg_test::sceneParams());
  observeBlind(core, corridor, base);
  ASSERT_TRUE(
      core.setMode(follow("s1", "r1", drivingPoint(4.6, 0.1))).accepted);
  core.setGuidance(standingGuidance("s1", 1, true, {0.1, 0.1}));
  ASSERT_TRUE(core.standingStart());
  // A jump that stays in the disk is still no proof of standing there.
  core.onOdometry(basePose(1.3, 0.1));
  EXPECT_FALSE(core.standingStart());
  core.setGuidance(standingGuidance("s1", 2, true, {0.1, 0.1}));
  EXPECT_FALSE(core.standingStart());
}

TEST(LocalPlanningCore, LeavingTheDiskBetweenSessionsEndsTheStandingStart) {
  // Applied in s1; then no block is held (a new session before its
  // guidance, or IDLE) while the robot drives out of the disk and back in
  // steps short of a jump. A valid block of the same boot arriving after
  // that, even one issued before the drive, never re-applies it.
  mgg_test::Corridor corridor;
  const StateVec base = basePose(0.1, 0.1);
  const auto outAndBack = [](LocalPlanningCore& core) {
    for (double x = 0.35; x <= 2.6; x += 0.25) core.onOdometry(basePose(x, 0.1));
    for (double x = 2.35; x >= 0.1; x -= 0.25) core.onOdometry(basePose(x, 0.1));
    core.onOdometry(basePose(0.1, 0.1));
  };
  LocalModeRequest idle;
  idle.session_id = "s1";
  idle.request_id = "r-idle";
  idle.mode = LocalMode::kIdle;
  for (const bool use_idle : {false, true}) {
    SCOPED_TRACE(use_idle ? "IDLE" : "session change");
    LocalPlanningCore core(mgg_test::sceneParams());
    observeBlind(core, corridor, base);
    ASSERT_TRUE(
        core.setMode(follow("s1", "r1", drivingPoint(4.6, 0.1))).accepted);
    core.setGuidance(standingGuidance("s1", 1, true, {0.1, 0.1}));
    ASSERT_TRUE(core.standingStart());
    if (use_idle) {
      ASSERT_TRUE(core.setMode(idle).accepted);
    } else {
      ASSERT_TRUE(
          core.setMode(follow("s2", "r2", drivingPoint(4.6, 0.1))).accepted);
    }
    EXPECT_FALSE(core.standingStart());
    outAndBack(core);
    const std::string session = use_idle ? "s1" : "s2";
    if (use_idle)
      ASSERT_TRUE(
          core.setMode(follow("s1", "r3", drivingPoint(4.6, 0.1))).accepted);
    core.setGuidance(standingGuidance(session, 2, true, {0.1, 0.1}));
    EXPECT_FALSE(core.standingStart());
    core.setGuidance(standingGuidance(session, 3, true, {0.1, 0.1}));
    EXPECT_FALSE(core.standingStart());
  }
  {
    // Control: no drive in between, so the next session's block applies.
    LocalPlanningCore core(mgg_test::sceneParams());
    observeBlind(core, corridor, base);
    ASSERT_TRUE(
        core.setMode(follow("s1", "r1", drivingPoint(4.6, 0.1))).accepted);
    core.setGuidance(standingGuidance("s1", 1, true, {0.1, 0.1}));
    ASSERT_TRUE(
        core.setMode(follow("s2", "r2", drivingPoint(4.6, 0.1))).accepted);
    EXPECT_FALSE(core.standingStart());
    core.setGuidance(standingGuidance("s2", 2, true, {0.1, 0.1}));
    EXPECT_TRUE(core.standingStart());
  }
}
