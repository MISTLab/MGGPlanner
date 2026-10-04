#include <gtest/gtest.h>

#include "local_planner_fixture.h"
using namespace mgg;
TEST(RecentTrack, ReverseBoundAndPermission) {
  RecentTrack track;
  for (int i = 0; i <= 60; ++i) track.add(StateVec(i * .25, 0, .5, 0));
  auto back = track.backFrom(StateVec(15, 0, .5, 0), 3);
  ASSERT_GT(back.size(), 1u);
  double length = 0;
  for (size_t i = 1; i < back.size(); ++i) {
    length += (back[i] - back[i - 1]).head<3>().norm();
    EXPECT_LE(std::abs(back[i].y()), .05);
  }
  EXPECT_LE(length, 3);
  EXPECT_TRUE(track.backFrom(StateVec(15, 1, .5, 0), 3).empty());
  EXPECT_TRUE(track.backFrom(StateVec(0, 0, .5, 0), 3).empty());
  mgg_test::LocalScene s;
  auto in = s.inputs();
  in.pose = {2, 0, .5, 0};
  in.target = Eigen::Vector3d(-1, 0, .5);
  s.map.narrow = true;
  s.map.wall = true;
  s.map.wall_x = 2.7;
  ASSERT_FALSE(roomToTurn(s.map, s.robot, s.planning, in.pose));
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  for (int i = 0; i <= 8; ++i) planner.recordPose(StateVec(i * .25, 0, .5, 0));
  auto allowed = planner.plan(in, {});
  ASSERT_TRUE(allowed.path) << allowed.reason;
  EXPECT_LE(pathLength(*allowed.path), 3 * s.robot.size.x());
  for (size_t i = 0; i < allowed.path->poses.size(); ++i) {
    EXPECT_TRUE(allowed.path->reverse[i]);
    EXPECT_LE(std::abs(allowed.path->poses[i].y()), .05);
  }
  s.planning.departure_reverse_allowed = false;
  LocalPlanner disallowed(s.map, s.layer, s.cache, s.planning, s.robot,
                          s.sensor);
  for (int i = 0; i <= 8; ++i)
    disallowed.recordPose(StateVec(i * .25, 0, .5, 0));
  auto result = disallowed.plan(in, {});
  if (result.path) {
    for (bool reverse : result.path->reverse) {
      EXPECT_FALSE(reverse);
    }
  }
  s.map.narrow = false;
  s.planning.departure_reverse_allowed = true;
  in.target =
      Eigen::Vector3d(6, 0, .5);  // Behind a wall, but room at the root.
  in.speed_mps = 0;  // A .25 m reverse would otherwise fail the stopping bound.
  s.cache.flushAll();
  ASSERT_TRUE(roomToTurn(s.map, s.robot, s.planning, in.pose));
  LocalPlanner can_turn(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  for (int i = 0; i <= 8; ++i) can_turn.recordPose(StateVec(i * .25, 0, .5, 0));
  auto blocked = can_turn.plan(in, {});
  EXPECT_FALSE(blocked.path);
  EXPECT_EQ(blocked.status, LocalStatus::kBlocked);
}

TEST(RecentTrack, ContinuesCertifiedReverseEscape) {
  mgg_test::LocalScene s;
  s.map.narrow = true;
  s.map.wall = true;
  s.map.wall_x = 2.7;
  auto in = s.inputs();
  in.pose = {2, 0, .5, 0};
  in.target = Eigen::Vector3d(-1, 0, .5);
  in.speed_mps = .2;
  LocalPlanner planner(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  for (int i = 0; i <= 8; ++i) planner.recordPose(StateVec(i * .25, 0, .5, 0));
  auto first = planner.plan(in, {});
  ASSERT_TRUE(first.path) << first.reason;
  ASSERT_TRUE(first.path->reverse.front());
  const StateVec refuge = first.path->poses.back();
  for (int cycle = 0; cycle < 3; ++cycle) {
    in.executing_path = first.path;
    in.progress_m = .25;
    in.pose = first.path->poses[1];
    planner.recordPose(in.pose);  // Track now includes the motion in reverse.
    auto next = planner.plan(in, {});
    EXPECT_EQ(next.status, LocalStatus::kMoving) << next.reason;
    ASSERT_TRUE(next.path) << next.reason;
    EXPECT_EQ(next.path->kind, LocalPathKind::kExtend);
    EXPECT_EQ(next.path->extends_sequence_id, first.path->sequence_id);
    EXPECT_EQ(next.path->poses.back(), refuge);
    EXPECT_LT(pathLength(*next.path), pathLength(*first.path));
    auto prefix =
        committedPrefix(*first.path, .25, commitmentLength(.2, in.braking));
    ASSERT_GE(next.path->poses.size(), prefix.poses.size());
    for (size_t i = 0; i < prefix.poses.size(); ++i)
      EXPECT_EQ(next.path->poses[i], prefix.poses[i]);
    for (bool reverse : next.path->reverse) {
      EXPECT_TRUE(reverse);
    }
    first = next;
  }
  in.executing_path = first.path;
  in.progress_m = 0;
  in.no_go_zones.set({refuge.head<2>()}, .3);
  EXPECT_FALSE(planner.plan(in, {}).path);
  in.no_go_zones.set({}, 0);
  in.pose = refuge;
  in.progress_m = pathLength(*first.path);
  in.speed_mps = 0;
  auto forward = planner.plan(in, {});
  ASSERT_TRUE(forward.path) << forward.reason;
  EXPECT_EQ(forward.path->kind, LocalPathKind::kBreak);
  for (bool reverse : forward.path->reverse) {
    EXPECT_FALSE(reverse);
  }
}
