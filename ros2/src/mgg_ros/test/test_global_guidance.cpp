#include <gtest/gtest.h>
#include "mgg_ros/global_guidance.h"

TEST(GlobalGuidance, TargetAndComplete) {
  mgg::GlobalGuidance guidance;
  mgg_msgs::srv::SetLocalPlannerMode::Request mode;
  mode.mode = mode.EXPLORE;
  mode.session_id = "mode-session";
  mode.request_id = "request";
  ASSERT_TRUE(guidance.setMode(mode));
  std::vector<geometry_msgs::msg::Point> global_route(2);
  global_route.back().x = 12;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 500;
  const auto target = guidance.message(global_route, false, "world", "target", stamp);
  EXPECT_EQ(target.header.stamp, stamp);
  EXPECT_EQ(target.route, global_route);
  EXPECT_EQ(target.target, global_route.back());
  EXPECT_EQ(target.kind, target.TARGET);
  EXPECT_EQ(target.session_id, mode.session_id);
  const auto done = guidance.message({}, true, "world", "exhausted", stamp);
  EXPECT_EQ(done.kind, done.COMPLETE);
  EXPECT_GT(done.sequence_id, target.sequence_id);
  EXPECT_TRUE(done.route.empty());
}

TEST(GlobalGuidance, StatusIdentityAndMode) {
  mgg::GlobalGuidance guidance;
  mgg_msgs::srv::SetLocalPlannerMode::Request mode;
  mode.mode = mode.EXPLORE;
  mode.session_id = "new";
  mode.request_id = "latest";
  ASSERT_TRUE(guidance.setMode(mode));
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 500;
  const auto target = guidance.message(std::vector<geometry_msgs::msg::Point>(2),
                                       false, "world", "target", stamp);
  mgg_msgs::msg::LocalPlannerStatus status;
  status.stamp = stamp;
  status.status = status.BLOCKED;
  status.session_id = "old";
  status.request_id = mode.request_id;
  EXPECT_FALSE(guidance.setsTargetAside(status));
  status.session_id = mode.session_id;
  status.request_id = "old-request";
  EXPECT_FALSE(guidance.setsTargetAside(status));
  status.request_id = mode.request_id;
  EXPECT_TRUE(guidance.setsTargetAside(status));  // optional sequence absent
  // Zero can mean the local node had not yet received any guidance. A
  // queued status from before this TARGET must not set that target aside.
  status.stamp.nanosec = stamp.nanosec - 1;
  EXPECT_FALSE(guidance.setsTargetAside(status));
  status.stamp.sec = stamp.sec - 1;
  status.stamp.nanosec = 999999999;
  EXPECT_FALSE(guidance.setsTargetAside(status));
  status.stamp = stamp;
  EXPECT_TRUE(guidance.setsTargetAside(status));
  ++status.stamp.sec;
  EXPECT_TRUE(guidance.setsTargetAside(status));
  status.guidance_sequence_id = target.sequence_id + 1;
  EXPECT_FALSE(guidance.setsTargetAside(status));
  status.guidance_sequence_id = target.sequence_id;
  EXPECT_TRUE(guidance.setsTargetAside(status));
  status.status = status.WAITING_FOR_MAP;
  EXPECT_FALSE(guidance.setsTargetAside(status));
  mode.mode = mode.IDLE;
  ASSERT_TRUE(guidance.setMode(mode));
  status.status = status.BLOCKED;
  EXPECT_FALSE(guidance.setsTargetAside(status));
  mode.mode = 99;
  EXPECT_FALSE(guidance.setMode(mode));
}

TEST(GlobalGuidance, CarriesTheStandingStartBlock) {
  mgg::GlobalGuidance guidance;
  mgg_msgs::srv::SetLocalPlannerMode::Request mode;
  mode.mode = mode.EXPLORE;
  mode.session_id = "session";
  mode.request_id = "request";
  ASSERT_TRUE(guidance.setMode(mode));
  builtin_interfaces::msg::Time stamp;
  const auto standing = guidance.message(
      {}, false, "world", "discovery", stamp,
      mgg::StandingStart{Eigen::Vector2d(1.5, -2), 2.5}, 42);
  EXPECT_TRUE(standing.standing_start.valid);
  EXPECT_DOUBLE_EQ(standing.standing_start.center.x, 1.5);
  EXPECT_DOUBLE_EQ(standing.standing_start.center.y, -2);
  EXPECT_DOUBLE_EQ(standing.standing_start.radius, 2.5);
  EXPECT_EQ(standing.standing_start.boot, 42u);
  // Every kind carries it; without the proof it is invalid, same identity.
  const auto revoked =
      guidance.message({}, true, "world", "done", stamp, std::nullopt, 42);
  EXPECT_EQ(revoked.kind, revoked.COMPLETE);
  EXPECT_FALSE(revoked.standing_start.valid);
  EXPECT_EQ(revoked.standing_start.boot, 42u);
  EXPECT_GT(revoked.sequence_id, standing.sequence_id);
}
