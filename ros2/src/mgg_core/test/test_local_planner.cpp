#include <gtest/gtest.h>

#include <algorithm>

#include "local_planner_fixture.h"
#include "mgg_core/graph_expansion.h"
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
  in.target.reset();
  {
    // Unobserved startup: nothing round the robot is observed, so nothing
    // was searched. Never "observed and empty".
    mgg_test::LocalScene blind;
    blind.map.blind_radius = 1.5;
    blind.layer.reset({0, 0, 0.5}, 0);
    blind.layer.recheck(std::chrono::steady_clock::now() +
                        std::chrono::seconds(20));
    LocalPlanner blind_planner(blind.map, blind.layer, blind.cache,
                               blind.planning, blind.robot, blind.sensor);
    auto unobserved = blind_planner.plan(in, MapChange{1, {}, true});
    EXPECT_EQ(unobserved.status, LocalStatus::kWaitingForMap)
        << unobserved.reason;
    EXPECT_FALSE(unobserved.checks_complete);
  }
  {
    // Not every viewpoint scored within the slice: not exhausted.
    mgg_test::LocalScene slow;
    slow.map.delay_voxel_queries = true;
    slow.sensor.max_range = 20;
    slow.sensor.fov = {2 * M_PI, M_PI / 6};
    slow.sensor.resolution = {M_PI / 36, M_PI / 36};
    slow.sensor.update();
    LocalPlanner slow_planner(slow.map, slow.layer, slow.cache, slow.planning,
                              slow.robot, slow.sensor);
    auto unscored = slow_planner.plan(in, {});
    EXPECT_EQ(unscored.status, LocalStatus::kWaitingForMap) << unscored.reason;
    EXPECT_EQ(unscored.reason, "not every viewpoint was scored");
    EXPECT_FALSE(unscored.checks_complete);
    ASSERT_TRUE(unscored.viewpoints_offered && unscored.viewpoints_scored);
    EXPECT_LT(*unscored.viewpoints_scored, *unscored.viewpoints_offered);
  }
  {
    // Fully observed and enclosed: walls round the robot, every cell seen.
    mgg_test::LocalScene closed;
    for (const double side : {-1.0, 1.0}) {
      closed.map.solids.emplace_back(Eigen::Vector3d(side * 0.75 - 0.1, -1, 0.1),
                                     Eigen::Vector3d(side * 0.75 + 0.1, 1, 0.9));
      closed.map.solids.emplace_back(Eigen::Vector3d(-1, side * 0.4 - 0.1, 0.1),
                                     Eigen::Vector3d(1, side * 0.4 + 0.1, 0.9));
    }
    LocalPlanner closed_planner(closed.map, closed.layer, closed.cache,
                                closed.planning, closed.robot, closed.sensor);
    auto enclosed = closed_planner.plan(in, {});
    EXPECT_EQ(enclosed.status, LocalStatus::kNoLocalTarget) << enclosed.reason;
    EXPECT_TRUE(enclosed.checks_complete);
    ASSERT_TRUE(enclosed.viewpoints_offered && enclosed.viewpoints_scored);
    EXPECT_EQ(*enclosed.viewpoints_scored, *enclosed.viewpoints_offered);
  }
  {
    // Gain beyond reach of a useful path: blocked, not "no gain".
    mgg_test::LocalScene boxed;
    boxed.map.unknown_beside_corridor = true;
    boxed.map.solids.emplace_back(Eigen::Vector3d(1.6, -1, 0.1),
                                  Eigen::Vector3d(1.8, 1, 0.9));
    boxed.map.solids.emplace_back(Eigen::Vector3d(-1.8, -1, 0.1),
                                  Eigen::Vector3d(-1.6, 1, 0.9));
    LocalPlanner boxed_planner(boxed.map, boxed.layer, boxed.cache,
                               boxed.planning, boxed.robot, boxed.sensor);
    auto short_gain = boxed_planner.plan(in, {});
    EXPECT_FALSE(short_gain.path);
    ASSERT_TRUE(short_gain.candidate_count);
    EXPECT_GT(*short_gain.candidate_count, 0u);
    EXPECT_EQ(short_gain.status, LocalStatus::kBlocked) << short_gain.reason;
    EXPECT_EQ(short_gain.reason.rfind("gain candidates", 0), 0u)
        << short_gain.reason;
  }
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

TEST(LocalPlanner, GroundGoalHeightIsAHint) {
  mgg_test::LocalScene s;
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  // A 2-D objective seeded at the base's height (floor 0, half the body
  // up), not at the 0.5 m driving height the lattice links at.
  in.target = Eigen::Vector3d(5, 0, 0.1);
  auto base_height = planner.plan(in, {});
  ASSERT_TRUE(base_height.path) << base_height.reason;
  EXPECT_TRUE(base_height.path->reaches_goal);
  EXPECT_NEAR(base_height.path->poses.back().z(), 0.5, 1e-9);
  EXPECT_LE((base_height.path->poses.back().head<2>() - Eigen::Vector2d(5, 0))
                .norm(),
            in.goal_tolerance_m);
  // The objective itself is never rewritten.
  EXPECT_EQ(*in.target, Eigen::Vector3d(5, 0, 0.1));

  // At the goal, a hint height off the driving height is still terminal.
  in.speed_mps = 0;
  in.target = Eigen::Vector3d(0.1, 0, 0.1);
  auto at = planner.plan(in, {});
  ASSERT_TRUE(at.path) << at.reason;
  EXPECT_TRUE(at.path->reaches_goal);
  EXPECT_EQ(at.path->poses.size(), 1u);
}

TEST(LocalPlanner, GlobalGroundEstimateDiffersFromLocal) {
  mgg_test::LocalScene s;
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  // Guidance planned on a map whose floor sits 0.4 m above the local one.
  in.target = Eigen::Vector3d(6, 0, 0.9);
  auto goal = planner.plan(in, {});
  ASSERT_TRUE(goal.path) << goal.reason;
  EXPECT_TRUE(goal.path->reaches_goal);
  EXPECT_NEAR(goal.path->poses.back().z(), 0.5, 1e-9);
  // Beyond the window, the route aim is placed on the local ground too.
  in.target = Eigen::Vector3d(20, 0, 0.9);
  in.coarse_route = {{7, 0, 0.9}, {20, 0, 0.9}};
  auto aim = planner.plan(in, {});
  ASSERT_TRUE(aim.path) << aim.reason;
  EXPECT_FALSE(aim.path->reaches_goal);
  EXPECT_NEAR(aim.path->poses.back().z(), 0.5, 1e-9);
  EXPECT_LT((aim.path->poses.back().head<2>() - Eigen::Vector2d(7, 0)).norm(),
            0.2);
}

TEST(LocalPlanner, UnknownGoalGroundWaitsForMap) {
  mgg_test::LocalScene s;
  s.map.unknown_ground = true;
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  in.target = Eigen::Vector3d(5.5, 2.5, 0.1);
  auto unknown = planner.plan(in, {});
  EXPECT_FALSE(unknown.path);
  EXPECT_EQ(unknown.status, LocalStatus::kWaitingForMap);
  EXPECT_EQ(unknown.reason, "goal_ground_unknown");
  // A route aim on unobserved ground falls back to the farthest observed one.
  in.target = Eigen::Vector3d(20, 0, 0.5);
  in.coarse_route = {{7, 0, 0.5}, {5.5, 2.5, 0.5}, {20, 0, 0.5}};
  auto aim = planner.plan(in, {});
  ASSERT_TRUE(aim.path) << aim.reason;
  EXPECT_LT((aim.path->poses.back().head<2>() - Eigen::Vector2d(7, 0)).norm(),
            0.2);
  in.coarse_route = {{5.5, 2.5, 0.5}, {20, 0, 0.5}};
  auto none = planner.plan(in, {});
  EXPECT_FALSE(none.path);
  EXPECT_EQ(none.status, LocalStatus::kWaitingForMap);
  EXPECT_EQ(none.reason, "goal_ground_unknown");
}

TEST(LocalPlanner, RealSensorBandUnknownIsGain) {
  // The deployed VLP16 and ground gain model: unknown confined to the floor
  // band beyond 4 m to either side is a frontier (0.5 m of unknown), not
  // a fraction of the sensor's full 360 x 45 degree, 20 m ray table.
  mgg_test::LocalScene s;
  s.map.band_unknown_beyond_y = 4;
  s.sensor = SensorParams();
  s.sensor.type = SensorType::kLidar;
  s.sensor.fov = {2 * M_PI, 0.7854};
  s.sensor.resolution = {5 * M_PI / 180, 5 * M_PI / 180};
  s.sensor.max_range = 20;
  s.sensor.frontier_percentage_threshold = 0.05;
  s.sensor.mount_height = 0.72;
  s.sensor.update();
  s.planning.ground_gain_angular_resolution_deg = 7.5;
  s.planning.ground_gain_max_range = 10;
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  in.target.reset();
  auto result = planner.plan(in, {});
  ASSERT_TRUE(result.candidate_count);
  EXPECT_GT(*result.candidate_count, 0u) << result.reason;
  ASSERT_TRUE(result.path) << result.reason;
  EXPECT_EQ(result.status, LocalStatus::kMoving);
  EXPECT_GE(pathLength(*result.path), 6);
}

TEST(LocalPlanner, StandingStartDepartsAcrossItsBlindDisk) {
  mgg_test::LocalScene s;
  s.map.blind_radius = 1.5;
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  // Without the prior the robot stands on unobserved ground: no departure.
  auto blind = planner.plan(in, MapChange{1, {}, true});
  EXPECT_FALSE(blind.path);
  const StandingStart standing{Eigen::Vector2d::Zero(), 2.0};
  planner.setStandingStart(standing);
  s.cache.flushAll();
  auto departure = planner.plan(in, MapChange{2, {}, true});
  ASSERT_TRUE(departure.path) << departure.reason;
  EXPECT_EQ(departure.status, LocalStatus::kMoving);
  EXPECT_TRUE(departure.path->reaches_goal);
  EXPECT_GE(pathLength(*departure.path), 6.5);
  // Re-certified from points along the departure, still in the disk.
  EXPECT_TRUE(planner.pathStillCertified(*departure.path, 0.6));
  // Legacy's first-goal arrival protection: never stop in the disk.
  auto inside = in;
  inside.target = Eigen::Vector3d(1.7, 0, 0.5);
  EXPECT_FALSE(planner.plan(inside, {}).path);
  // An occupied obstacle in the unseen body band blocks the departure.
  s.map.solids.emplace_back(Eigen::Vector3d(0.9, -8, 0.3),
                            Eigen::Vector3d(1.1, 8, 0.5));
  s.cache.flushAll();
  auto obstacle = planner.plan(in, MapChange{3, {}, true});
  EXPECT_FALSE(obstacle.path) << obstacle.reason;
  EXPECT_FALSE(planner.pathStillCertified(*departure.path, 0));
  s.map.solids.clear();
  // An observed drop in the disk is refused even with the prior.
  s.map.pit = Eigen::AlignedBox2d(Eigen::Vector2d(0.8, -8),
                                  Eigen::Vector2d(1.2, 8));
  s.cache.flushAll();
  auto drop = planner.plan(in, MapChange{4, {}, true});
  EXPECT_FALSE(drop.path) << drop.reason;
  s.map.pit.reset();
  // Revoked, the same unknown floor and body band refuse again.
  planner.setStandingStart(std::nullopt);
  s.cache.flushAll();
  auto revoked = planner.plan(in, MapChange{5, {}, true});
  EXPECT_FALSE(revoked.path) << revoked.reason;
  EXPECT_FALSE(planner.pathStillCertified(*departure.path, 0));
}

TEST(LocalPlanner, StandingStartUnknownBodyStaysInItsDisk) {
  // The disk is fixed round where the robot started (0, 0), radius 2; the
  // robot has since moved to x = 1.5, still in it. Its hanging departure
  // may reach 2 m from there, to x = 3.5: the unknown body volume it may
  // pass is the disk's, never a strip beyond the disk's edge, whatever the
  // ground under it.
  const StandingStart standing{Eigen::Vector2d::Zero(), 2.0};
  const Eigen::AlignedBox3d outside(Eigen::Vector3d(2.2, -8, 0.3),
                                    Eigen::Vector3d(2.6, 8, 0.6));
  const Eigen::AlignedBox3d inside(Eigen::Vector3d(1.6, -8, 0.3),
                                   Eigen::Vector3d(1.8, 8, 0.6));
  const auto line = [](const std::vector<double>& xs) {
    LocalPathPlan path;
    for (const double x : xs) path.poses.push_back(StateVec(x, 0, 0.5, 0));
    path.reverse.assign(path.poses.size(), false);
    return path;
  };
  for (const bool beyond : {true, false}) {
    SCOPED_TRACE(beyond ? "strip beyond the disk" : "strip in the disk");
    mgg_test::LocalScene s;
    s.map.unseen_volumes.push_back(beyond ? outside : inside);
    const double root_x = beyond ? 1.5 : 1.0;
    s.layer.setStandingStart(standing);
    s.layer.reset({root_x, 0, 0.5}, 0);
    s.layer.recheck(std::chrono::steady_clock::now() +
                    std::chrono::seconds(20));

    // Expansion: the root's hanging edge across the strip.
    GroundProjection ground(s.map, s.planning, true);
    ground.setStandingStart(standing);
    ExpandContext ctx;
    ctx.map = &s.map;
    ctx.ground = &ground;
    ctx.planning = &s.planning;
    ctx.robot = &s.robot;
    ctx.robot_box_size = s.robot.getPlanningSize();
    ctx.stop_at_unknown = true;
    ctx.root_is_robot = true;
    ctx.hanging_root_edge_length_max = standing.radius;
    ctx.preserve_hanging_root_start_height = true;
    ctx.hanging_root_unknown_body_disk = standing;
    GraphManager graph;
    auto* root = new Vertex(0, StateVec(root_x, 0, 0.5, 0));
    root->is_hanging = true;
    graph.addVertex(root);
    Vertex candidate(1, StateVec(root_x + 2.0, 0, 0.5, 0));
    ExpandGraphReport report;
    expandGraph(graph, candidate, report, ctx);
    EXPECT_EQ(report.num_vertices_added, beyond ? 0 : 1);

    // Certification: the same departure, then on to observed ground.
    LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot,
                         s.sensor);
    planner.setStandingStart(standing);
    EXPECT_EQ(planner.pathStillCertified(line({root_x, root_x + 2.0, 7.0}), 0),
              !beyond);

    // The whole cycle, to a goal past the strip.
    auto in = s.inputs();
    in.pose = StateVec(root_x, 0, 0.5, 0);
    auto result = planner.plan(in, MapChange{1, {}, true});
    EXPECT_EQ(result.path.has_value(), !beyond) << result.reason;
    // Occupied in the disk blocks, as anywhere.
    if (!beyond) {
      s.map.solids.push_back(inside);
      s.cache.flushAll();
      EXPECT_FALSE(
          planner.pathStillCertified(line({root_x, root_x + 2.0, 7.0}), 0));
      EXPECT_FALSE(planner.plan(in, MapChange{2, {}, true}).path);
    }
  }
}

TEST(LocalPlanner, StandingStartTurnsSeeOnlyWholeDiskCellsAsObserved) {
  // The disk round the origin, radius 2. The robot at (1.5, 0.5) facing +x
  // turns about to drive off along -x: its turning circle (0.54 m) reaches
  // the cell at (1.7, 0.9), whose centre lies in the disk but whose square
  // reaches past its edge, and the cell at (1.3, 0.9), wholly in it. Its
  // straight sweep along -x meets neither. Ground is observed throughout.
  const StandingStart standing{Eigen::Vector2d::Zero(), 2.0};
  EXPECT_TRUE(standing.covers({1.7, 0.9}));
  EXPECT_FALSE(standing.coversCell({1.7, 0.9}, 0.2));
  EXPECT_TRUE(standing.coversCell({1.3, 0.9}, 0.2));
  const auto column = [](double x, double y) {
    return Eigen::AlignedBox3d(Eigen::Vector3d(x - 0.1, y - 0.1, 0.3),
                               Eigen::Vector3d(x + 0.1, y + 0.1, 0.7));
  };
  const StateVec at(1.5, 0.5, 0.5, 0);
  LocalPathPlan turn;
  for (const double x : {1.5, -0.5, -5.0})
    turn.poses.push_back(StateVec(x, 0.5, 0.5, x == 1.5 ? 0 : M_PI));
  turn.reverse.assign(turn.poses.size(), false);

  // Nothing unseen: the turn and the departure certify.
  {
    mgg_test::LocalScene s;
    LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot,
                         s.sensor);
    planner.setStandingStart(standing);
    EXPECT_TRUE(planner.pathStillCertified(turn, 0));
  }
  // An unseen body column straddling the disk's edge in the turning circle:
  // observed for the legacy policy, never for the v2 one, and the turn is
  // refused.
  {
    mgg_test::LocalScene s;
    s.map.unseen_volumes.push_back(column(1.7, 0.9));
    EXPECT_TRUE(turnSpaceObserved(s.map, s.robot, s.planning, at, &standing,
                                  StandingTurnBody::kCellCentreInDisk));
    EXPECT_FALSE(turnSpaceObserved(s.map, s.robot, s.planning, at, &standing,
                                   StandingTurnBody::kWholeCellInDisk));
    LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot,
                         s.sensor);
    planner.setStandingStart(standing);
    EXPECT_FALSE(planner.pathStillCertified(turn, 0));
  }
  // The same unseen column wholly in the disk passes; made occupied, it
  // blocks the turn.
  {
    mgg_test::LocalScene s;
    s.map.unseen_volumes.push_back(column(1.3, 0.9));
    EXPECT_TRUE(turnSpaceObserved(s.map, s.robot, s.planning, at, &standing,
                                  StandingTurnBody::kWholeCellInDisk));
    LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot,
                         s.sensor);
    planner.setStandingStart(standing);
    EXPECT_TRUE(planner.pathStillCertified(turn, 0));
    s.map.unseen_volumes.clear();
    s.map.solids.push_back(column(1.3, 0.9));
    s.cache.flushAll();
    EXPECT_FALSE(roomToTurn(s.map, s.robot, s.planning, at, &standing,
                            StandingTurnBody::kWholeCellInDisk));
    EXPECT_FALSE(planner.pathStillCertified(turn, 0));
  }
}

// Spec §4.4 (v2-motionfix): only an unsafe commitment breaks it. The rest of
// a path may lose its certificate while what the executor may still drive
// keeps it; the next path then extends from the rest of that commitment.
TEST(LocalPlanner, SuffixLossKeepsTheCommitment) {
  mgg_test::LocalScene s;
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  const auto first = planner.plan(in, {});
  ASSERT_TRUE(first.path) << first.reason;
  ASSERT_NEAR(first.path->commit_length_m, 1.2, 1e-6);
  ASSERT_GT(pathLength(*first.path), 4);
  for (const auto& p : first.path->poses) ASSERT_NEAR(p.y(), 0, 1e-6);
  // On the path, beyond where the body reaches by the commitment's end.
  s.map.solids.emplace_back(Eigen::Vector3d(3.0, -0.3, 0.1),
                            Eigen::Vector3d(3.4, 0.3, 0.9));
  s.cache.flushAll();
  EXPECT_FALSE(planner.pathStillCertified(*first.path, 0));
  EXPECT_TRUE(planner.commitmentStillCertified(*first.path, 0));
  // Faster now: this cycle's two-second horizon reaches the obstacle, the
  // commitment the executor is held to does not. Extend, never BREAK.
  in.executing_path = first.path;
  in.speed_mps = 1.0;
  ASSERT_GT(commitmentLength(in.speed_mps, in.braking) + 0.5, 3.0);
  const auto next = planner.plan(in, {});
  ASSERT_TRUE(next.path) << next.reason;
  EXPECT_EQ(next.path->kind, LocalPathKind::kExtend);
  EXPECT_EQ(next.path->extends_sequence_id, first.path->sequence_id);
  const auto rest = remainingCommitment(*first.path, 0);
  ASSERT_EQ(next.path->prefix_length, rest.poses.size());
  for (size_t i = 0; i < rest.poses.size(); ++i)
    EXPECT_EQ(next.path->poses[i], rest.poses[i]);
  EXPECT_TRUE(planner.pathStillCertified(*next.path, 0));

  // An obstacle the body meets within the commitment breaks it.
  s.map.solids.emplace_back(Eigen::Vector3d(1.3, -0.3, 0.1),
                            Eigen::Vector3d(1.5, 0.3, 0.9));
  s.cache.flushAll();
  EXPECT_FALSE(planner.commitmentStillCertified(*first.path, 0));
  in.speed_mps = 0.6;
  const auto broken = planner.plan(in, {});
  if (broken.path) EXPECT_EQ(broken.path->kind, LocalPathKind::kBreak);
}

// Spec §4.3 as amended (v2-motionfix): 6-10 m is preferred, not a veto. With
// no 6 m path (p1a-acc-2 robot_2: 208 routes to gain, all under 5.03 m),
// the best-ranked certified one of at least a commitment is published.
TEST(LocalPlanner, ShorterCertifiedPathWhenNoSixMetreOne) {
  mgg_test::LocalScene s;
  s.map.unknown_beside_corridor = true;
  s.map.solids.emplace_back(Eigen::Vector3d(2.8, -1, 0.1),
                            Eigen::Vector3d(3.0, 1, 0.9));
  s.map.solids.emplace_back(Eigen::Vector3d(-3.0, -1, 0.1),
                            Eigen::Vector3d(-2.8, 1, 0.9));
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto in = s.inputs();
  in.target.reset();
  const auto result = planner.plan(in, {});
  ASSERT_TRUE(result.path) << result.reason;
  EXPECT_EQ(result.status, LocalStatus::kMoving);
  EXPECT_EQ(result.reason, "certified local path");
  EXPECT_FALSE(result.path->reaches_goal);
  const double length = pathLength(*result.path);
  EXPECT_LT(length, kPreferredPathM);
  EXPECT_GE(length + 1e-9, commitmentLength(in.speed_mps, in.braking));
  ASSERT_TRUE(result.refusals);
  EXPECT_TRUE(planner.pathStillCertified(*result.path, 0));
  // With a 6 m path available, it is preferred (CorridorAndSplice).
}

// MGG is the single terrain authority (v2-motionfix): Nav2's costmap carries
// the layer's refusals and MPPI checks the footprint against them, so a
// certified path must keep every footprint off them. A 0.38 m kerb beside
// the corridor refuses the columns next to it; the planner's own checks
// pass a body box riding over it, not the costmap.
TEST(LocalPlanner, CertifiedFootprintsStayOffRefusedTerrain) {
  auto tops = mgg_test::localFloor();
  for (int x = 10; x < 25; ++x)
    for (int y = 2; y < 4; ++y) tops[{x, y}] = 0.38;  // x 2-5 m, y 0.4-0.8 m
  struct KerbMap : mgg_test::TerrainFixture {
    using TerrainFixture::TerrainFixture;
    std::optional<Eigen::AlignedBox3d> windowBounds() const override {
      return Eigen::AlignedBox3d(Eigen::Vector3d(-8, -8, -2),
                                 Eigen::Vector3d(8, 8, 4));
    }
  } map(0.2, tops);
  const auto planning = mgg_test::localPlanning();
  const auto robot = mgg_test::localRobot();
  SensorParams sensor;
  sensor.fov = {6.28, 0.2};
  sensor.resolution = {0.4, 0.2};
  sensor.update();
  GroundLayer layer(map, planning, robot);
  layer.reset({0, 0, 0.5}, 0);
  layer.recheck(std::chrono::steady_clock::now() + std::chrono::seconds(20));
  ASSERT_EQ(layer.verdict({3.1, 0.1}), GroundVerdict::kRefusedStepGrade);
  ASSERT_EQ(layer.verdict({3.1, -0.1}), GroundVerdict::kAdmitted);
  CertificationCache cache{dependencyHalos(robot, planning, 0.2, 5)};
  LocalPlanner planner(map, layer, cache, planning, robot, sensor);
  LocalPathPlan straight;
  for (int i = 0; i <= 28; ++i) {
    straight.poses.emplace_back(i * 0.25, 0, 0.5, 0);
    straight.reverse.push_back(false);
  }
  EXPECT_FALSE(planner.pathStillCertified(straight, 0));
  mgg_test::LocalScene scene;
  auto in = scene.inputs();
  const auto result = planner.plan(in, {});
  ASSERT_TRUE(result.path) << result.reason;
  const Eigen::Vector2d size = robot.getPlanningSize().head<2>();
  for (const auto& pose : result.path->poses)
    EXPECT_FALSE(layer.refusedUnder(pose.head<2>(), pose[3], size))
        << pose.transpose();
  // Round the kerb, not along it.
  EXPECT_TRUE(std::any_of(
      result.path->poses.begin(), result.path->poses.end(),
      [](const StateVec& p) {
        return p.x() > 2 && p.x() < 5 && p.y() < -0.1;
      }));
}
