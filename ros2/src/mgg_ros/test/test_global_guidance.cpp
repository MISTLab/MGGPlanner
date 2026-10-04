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
  const auto target = guidance.message(global_route, false, "world", "target");
  EXPECT_EQ(target.route, global_route);
  EXPECT_EQ(target.target, global_route.back());
  EXPECT_EQ(target.kind, target.TARGET);
  EXPECT_EQ(target.session_id, mode.session_id);
  const auto done = guidance.message({}, true, "world", "exhausted");
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
  const auto target = guidance.message(std::vector<geometry_msgs::msg::Point>(2),
                                       false, "world", "target");
  mgg_msgs::msg::LocalPlannerStatus status;
  status.status = status.BLOCKED;
  status.session_id = "old";
  status.request_id = mode.request_id;
  EXPECT_FALSE(guidance.setsTargetAside(status));
  status.session_id = mode.session_id;
  status.request_id = "old-request";
  EXPECT_FALSE(guidance.setsTargetAside(status));
  status.request_id = mode.request_id;
  EXPECT_TRUE(guidance.setsTargetAside(status));  // optional sequence absent
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
