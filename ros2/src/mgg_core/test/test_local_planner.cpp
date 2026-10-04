#include <gtest/gtest.h>

#include "local_planner_fixture.h"
#include "mgg_core/local_route.h"
#include "mgg_core/planning_cancellation.h"
using namespace mgg;

TEST(LocalPlanner, CorridorAndSplice) {
  mgg_test::LocalScene s;
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  auto first = planner.plan(in, {});
  ASSERT_TRUE(first.path) << first.reason;
  EXPECT_TRUE(first.candidate_count && *first.candidate_count > 0);
  EXPECT_GE(pathLength(*first.path), 6);
  EXPECT_LE(pathLength(*first.path), 10);
  in.executing_path = first.path;
  in.progress_m = 0.5;
  in.pose = first.path->poses[2];
  auto next = planner.plan(in, {});
  ASSERT_TRUE(next.path) << next.reason;
  auto prefix = committedPrefix(*first.path, in.progress_m,
                                commitmentLength(in.speed_mps, in.braking));
  ASSERT_EQ(next.path->prefix_length, prefix.poses.size());
  for (size_t i = 0; i < prefix.poses.size(); ++i)
    EXPECT_EQ(next.path->poses[i], prefix.poses[i]);
  EXPECT_EQ(next.path->kind, LocalPathKind::kExtend);
  EXPECT_EQ(next.path->extends_sequence_id, first.path->sequence_id);
  EXPECT_EQ(next.path->session_id, in.session_id);
  EXPECT_TRUE(planner.pathStillCertified(*next.path, 0));
  in.executing_invalid = true;
  auto broken = planner.plan(in, {});
  ASSERT_TRUE(broken.path) << broken.reason;
  EXPECT_EQ(broken.path->kind, LocalPathKind::kBreak);
  EXPECT_EQ(broken.path->poses.front(), in.pose);
}
TEST(LocalPlanner, StatusCompleteness) {
  mgg_test::LocalScene s;
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  s.layer.reset(in.pose.head<3>(), 0);
  auto pending = planner.plan(in, {});
  EXPECT_EQ(pending.status, LocalStatus::kWaitingForMap);
  EXPECT_FALSE(pending.checks_complete);
  s.layer.recheck(std::chrono::steady_clock::now() + std::chrono::seconds(20));
  in.target.reset();
  auto empty = planner.plan(in, {});
  EXPECT_EQ(empty.status, LocalStatus::kNoLocalTarget) << empty.reason;
  EXPECT_TRUE(empty.checks_complete);
  in.target = Eigen::Vector3d(7, 0, 0.5);
  s.map.wall = true;
  auto wall = planner.plan(in, MapChange{1, {}, true});
  EXPECT_EQ(wall.status, LocalStatus::kBlocked) << wall.reason;
  EXPECT_FALSE(wall.reason.empty());
}
TEST(LocalPlanner, PendingElsewhereStillMoves) {
  mgg_test::LocalScene s;
  MapChange change{2,
                   {Eigen::AlignedBox3d(Eigen::Vector3d(-7.9, -7.9, -0.1),
                                        Eigen::Vector3d(-7.7, -7.7, 0.1))},
                   false};
  s.layer.withdraw(change);
  ASSERT_GT(s.layer.pendingCount(), 0);
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto result = planner.plan(s.inputs(), change);
  ASSERT_TRUE(result.path) << result.reason;
  EXPECT_EQ(result.status, LocalStatus::kMoving);
  auto in = s.inputs();
  in.target.reset();
  EXPECT_NE(planner.plan(in, {}).status, LocalStatus::kNoLocalTarget);
}
TEST(LocalPlanner, CertificationNotFallback) {
  mgg_test::LocalScene s;
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  s.map.narrow = true;
  in.pose = {2, 0, .5, 0};
  in.target = Eigen::Vector3d(6, 0, .5);
  ASSERT_TRUE(planner.plan(in, {}).path);
  in.no_go_zones.set({{4, 0}}, .8);
  ASSERT_FALSE(in.no_go_zones.inside(in.pose.head<3>()));
  EXPECT_FALSE(planner.plan(in, {}).path);
  in.no_go_zones.set({}, 0);
  s.map.narrow = false;
  s.cache.flushAll();
  in = s.inputs();
  in.target.reset();
  s.map.boundary_unknown = true;
  EXPECT_FALSE(planner.plan(in, {}).path);
  // Translation is collision-free, but facing backwards requires a spin.
  LocalPathPlan sharp;
  sharp.poses = {{2, 0, .5, 0}, {1.75, 0, .5, M_PI}, {1.5, 0, .5, M_PI}};
  sharp.reverse = {false, false, false};
  s.map.narrow = true;
  s.cache.flushAll();
  GroundProjection ground(s.map, s.planning);
  ExpandContext ctx;
  ctx.map = &s.map;
  ctx.ground = &ground;
  ctx.robot = &s.robot;
  ctx.planning = &s.planning;
  ctx.robot_box_size = s.robot.getPlanningSize();
  ASSERT_TRUE(groundShortcutSegmentAdmissible(
      ctx, sharp.poses.front().head<3>(), sharp.poses.back().head<3>(), true));
  ASSERT_FALSE(roomToTurn(s.map, s.robot, s.planning, sharp.poses.front()));
  EXPECT_FALSE(planner.pathStillCertified(sharp, 0));
  s.map.narrow = false;
  s.cache.flushAll();
  ASSERT_TRUE(roomToTurn(s.map, s.robot, s.planning, sharp.poses.front()));
  EXPECT_TRUE(planner.pathStillCertified(sharp, 0));
}
TEST(LocalPlanner, FinalNotIntermediateGoal) {
  mgg_test::LocalScene s;
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  in.speed_mps = 0.3;
  in.target = Eigen::Vector3d(1, 0, 0.5);
  auto short_terminal = planner.plan(in, {});
  ASSERT_TRUE(short_terminal.path) << short_terminal.reason;
  EXPECT_TRUE(short_terminal.path->reaches_goal);
  EXPECT_LT(pathLength(*short_terminal.path), 6);
  in.target = Eigen::Vector3d(20, 0, 0.5);
  in.coarse_route = {{7, 0, 0.5}, {20, 0, 0.5}};
  auto exit = planner.plan(in, {});
  ASSERT_TRUE(exit.path) << exit.reason;
  EXPECT_FALSE(exit.path->reaches_goal);
  in.speed_mps = 0;
  in.goal_tolerance_m = 0.2;
  in.target = Eigen::Vector3d(0.2, 0, 0.5);
  auto at = planner.plan(in, {});
  ASSERT_TRUE(at.path);
  EXPECT_TRUE(at.path->reaches_goal);
  EXPECT_EQ(at.path->poses.size(), 1u);
  in.target = Eigen::Vector3d(0.201, 0, 0.5);
  auto beyond = planner.plan(in, {});
  ASSERT_TRUE(beyond.path);
  EXPECT_GT(beyond.path->poses.size(), 1u);
  // Nonterminal endpoint near an intermediate waypoint is not the final goal.
  EXPECT_FALSE(exit.path->reaches_goal);
}

TEST(LocalPlanner, BrakingCapAndTooShortCandidate) {
  mgg_test::LocalScene s;
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  in.speed_mps = 1;
  in.braking.deceleration_mps2 = 0.3;
  in.target = Eigen::Vector3d(2.5, 0, 0.5);
  auto capped = planner.plan(in, {});
  ASSERT_TRUE(capped.path) << capped.reason;
  ASSERT_TRUE(capped.speed_cap_mps);
  EXPECT_LT(*capped.speed_cap_mps, in.speed_mps);
  EXPECT_GE(capped.path->commit_length_m,
            commitmentLength(in.speed_mps, in.braking));
  in.target = Eigen::Vector3d(1, 0, 0.5);
  EXPECT_FALSE(planner.plan(in, {}).path);
  in.speed_mps = 0;
  auto stopped = planner.plan(in, {});
  ASSERT_TRUE(stopped.path) << stopped.reason;
  EXPECT_FALSE(stopped.speed_cap_mps);
  EXPECT_GT(stopped.path->commit_length_m, in.braking.margin_m);
}

TEST(LocalPlanner, DirtyOrPendingPrefixWithdrawsImmediately) {
  mgg_test::LocalScene s;
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto first = planner.plan(s.inputs(), {});
  ASSERT_TRUE(first.path);
  EXPECT_TRUE(planner.pathStillCertified(*first.path, 0));
  MapChange change{3,
                   {Eigen::AlignedBox3d(Eigen::Vector3d(1, -.1, -.1),
                                        Eigen::Vector3d(1.2, .1, .1))},
                   false};
  s.layer.withdraw(change);
  s.cache.withdraw(change);
  EXPECT_FALSE(planner.pathStillCertified(*first.path, 0));
  auto in = s.inputs();
  in.executing_path = first.path;
  auto pending = planner.plan(in, change);
  EXPECT_FALSE(pending.path);
  EXPECT_FALSE(pending.checks_complete);
  s.layer.recheck(std::chrono::steady_clock::now() + std::chrono::seconds(20));
  s.map.wall = true;
  EXPECT_FALSE(planner.pathStillCertified(*first.path, 0));
}

TEST(LocalPlanner, EndpointBeyondGoalToleranceIsNotTerminal) {
  mgg_test::LocalScene s;
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  in.target = Eigen::Vector3d(7.3, 7.3, 0.5);
  in.goal_tolerance_m = 0.1;
  auto beyond = planner.plan(in, {});
  ASSERT_TRUE(beyond.path) << beyond.reason;
  EXPECT_FALSE(beyond.path->reaches_goal);
  const double gap = (beyond.path->poses.back().head<3>() - *in.target).norm();
  EXPECT_GT(gap, in.goal_tolerance_m);
  in.goal_tolerance_m = gap + 1e-6;
  auto at = planner.plan(in, {});
  ASSERT_TRUE(at.path) << at.reason;
  EXPECT_TRUE(at.path->reaches_goal);
}

TEST(LocalPlanner, LocalGainIsUncachedAndNotWindowBoundary) {
  mgg_test::LocalScene s;
  s.sensor.max_range = 2;
  s.sensor.update();
  s.map.interior_unknown = true;
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  in.target.reset();
  auto frontier = planner.plan(in, {});
  EXPECT_EQ(frontier.reason, "certified local path");
  ASSERT_TRUE(frontier.path) << frontier.reason;
  EXPECT_EQ(frontier.status, LocalStatus::kMoving);
  EXPECT_GE(pathLength(*frontier.path), 6);
  EXPECT_LE(pathLength(*frontier.path), 10);
  s.map.interior_unknown = false;
  s.map.boundary_unknown = true;
  // No withdrawal/flush is needed to refresh gain: it is not cached.
  const auto warm_edges = s.cache.edges().size();
  ASSERT_GT(warm_edges, 0u);
  auto observed = planner.plan(in, {});
  EXPECT_GE(s.cache.edges().size(), warm_edges);
  EXPECT_FALSE(observed.path);
  EXPECT_EQ(observed.status, LocalStatus::kNoLocalTarget) << observed.reason;
}

TEST(LocalPlanner, StationaryTerminalWhileSlowingDown) {
  mgg_test::LocalScene s;
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  auto previous = planner.plan(in, {});
  ASSERT_TRUE(previous.path);
  in.executing_path = previous.path;
  in.target = Eigen::Vector3d(.2, 0, .5);
  in.goal_tolerance_m = .2;
  for (double speed : {.2, 1e-6}) {
    in.speed_mps = speed;
    auto terminal = planner.plan(in, {});
    ASSERT_TRUE(terminal.path) << terminal.reason;
    EXPECT_EQ(terminal.status, LocalStatus::kMoving);
    EXPECT_TRUE(terminal.path->reaches_goal);
    ASSERT_EQ(terminal.path->poses.size(), 1u);
    EXPECT_EQ(terminal.path->poses.front(), in.pose);
    EXPECT_EQ(terminal.path->kind, LocalPathKind::kBreak);
    EXPECT_EQ(terminal.path->commit_length_m, 0);
    EXPECT_EQ(terminal.path->prefix_length, 1u);
    ASSERT_TRUE(terminal.speed_cap_mps);
    EXPECT_EQ(*terminal.speed_cap_mps, 0);
  }
}

TEST(LocalPlanner, ScoringBudgetKeepsCertifiedCandidate) {
  mgg_test::LocalScene s;
  s.map.unknown_beside_corridor = true;
  s.map.delay_voxel_queries = true;
  s.sensor.max_range = 20;
  s.sensor.fov = {2 * M_PI, M_PI / 6};
  s.sensor.resolution = {M_PI / 36, M_PI / 36};
  s.sensor.frontier_percentage_threshold = .05;
  s.sensor.update();
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  in.target.reset();
  auto result = planner.plan(in, {});
  EXPECT_EQ(result.status, LocalStatus::kMoving) << result.reason;
  ASSERT_TRUE(result.path) << result.reason;
  EXPECT_FALSE(result.checks_complete);
  EXPECT_GE(pathLength(*result.path), 6);
  EXPECT_LE(pathLength(*result.path), 10);
  EXPECT_LT(result.cycle_ms,
            350);  // Cooperative checks, not an Orin qualification.
  s.map.delay_voxel_queries = false;
  EXPECT_TRUE(planner.pathStillCertified(*result.path, 0));
}

TEST(LocalPlanner, ScoringDoesNotSwallowCancellation) {
  mgg_test::LocalScene s;
  s.map.interior_unknown = true;
  s.map.delay_voxel_queries = true;
  s.sensor.max_range = 2;
  s.sensor.update();
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  in.target.reset();
  bool cancelled = false;
  PlanningCancellationScope cancellation([&] {
    if (!cancelled && s.map.voxel_queries >= 1024) {
      cancelled = true;
      return true;
    }
    return false;
  });
  auto result = planner.plan(in, {});
  EXPECT_TRUE(cancelled);
  EXPECT_FALSE(result.path);
  EXPECT_FALSE(result.checks_complete);
}
