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
  s.map.wall = false;
  s.cache.flushAll();
  LocalPlanner can_turn(s.map, s.layer, s.cache, s.planning, s.robot, s.sensor);
  auto forward = can_turn.plan(in, {});
  ASSERT_TRUE(forward.path) << forward.reason;
  for (bool reverse : forward.path->reverse) {
    EXPECT_FALSE(reverse);
  }
}
