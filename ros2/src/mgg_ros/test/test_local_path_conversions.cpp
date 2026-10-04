// The ground local planner's wire: LocalPath round trips with its identity,
// and the status names are exactly LocalPlannerStatus's constants (C2a).

#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <mgg_msgs/msg/local_planner_status.hpp>

#include "mgg_ros/local_path_conversions.h"

using namespace mgg;
using mgg_msgs::msg::LocalPlannerStatus;

TEST(LocalPathConversions, RoundTripAndStatus) {
  EXPECT_EQ(statusName(LocalStatus::kMoving),
            std::string(LocalPlannerStatus::MOVING));
  EXPECT_EQ(statusName(LocalStatus::kWaitingForMap),
            std::string(LocalPlannerStatus::WAITING_FOR_MAP));
  EXPECT_EQ(statusName(LocalStatus::kNoLocalTarget),
            std::string(LocalPlannerStatus::NO_LOCAL_TARGET));
  EXPECT_EQ(statusName(LocalStatus::kBlocked),
            std::string(LocalPlannerStatus::BLOCKED));

  LocalPathPlan in;
  in.frame_id = "robot_0/odom";
  in.session_id = "session-a";
  in.request_id = "request-7";
  in.stamp_ns = 12'345'678'901ULL;
  in.epoch = 99;
  in.sequence_id = 4;
  in.extends_sequence_id = 3;
  in.map_revision = 17;
  in.guidance_sequence_id = 5;
  in.kind = LocalPathKind::kExtend;
  in.prefix_length = 2;
  in.commit_length_m = 1.25;
  in.poses = {StateVec(0, 0, 0.5, 0.1), StateVec(0.25, 0.1, 0.5, 0.3),
              StateVec(0.5, 0.2, 0.5, -0.2)};
  in.reverse = {false, false, true};
  in.reaches_goal = true;
  const auto msg = toLocalPathMsg(in);
  EXPECT_EQ(msg.kind, mgg_msgs::msg::LocalPath::EXTEND);
  EXPECT_EQ(msg.header.frame_id, in.frame_id);
  const auto decoded = fromLocalPathMsg(msg);
  EXPECT_EQ(decoded.session_id, in.session_id);
  EXPECT_EQ(decoded.request_id, in.request_id);
  EXPECT_EQ(decoded.frame_id, in.frame_id);
  EXPECT_EQ(decoded.stamp_ns, in.stamp_ns);
  EXPECT_EQ(decoded.epoch, in.epoch);
  EXPECT_EQ(decoded.sequence_id, in.sequence_id);
  EXPECT_EQ(decoded.extends_sequence_id, in.extends_sequence_id);
  EXPECT_EQ(decoded.map_revision, in.map_revision);
  EXPECT_EQ(decoded.guidance_sequence_id, in.guidance_sequence_id);
  EXPECT_EQ(decoded.kind, in.kind);
  EXPECT_EQ(decoded.prefix_length, in.prefix_length);
  EXPECT_DOUBLE_EQ(decoded.commit_length_m, in.commit_length_m);
  ASSERT_EQ(decoded.poses.size(), in.poses.size());
  for (size_t i = 0; i < in.poses.size(); ++i)
    EXPECT_TRUE(decoded.poses[i].isApprox(in.poses[i], 1e-12));
  EXPECT_EQ(decoded.reverse, in.reverse);
  EXPECT_TRUE(decoded.reaches_goal);

  auto bad = msg;
  bad.kind = 7;
  EXPECT_THROW(fromLocalPathMsg(bad), std::invalid_argument);

  mgg_msgs::msg::LocalPathFeedback fb;
  fb.session_id = "session-a";
  fb.epoch = 99;
  fb.sequence_id = 4;
  fb.state = mgg_msgs::msg::LocalPathFeedback::EXECUTING;
  fb.progress_m = 1.5;
  fb.deceleration_mps2 = 0.4;
  const auto feedback = fromFeedbackMsg(fb);
  EXPECT_EQ(feedback.session_id, fb.session_id);
  EXPECT_TRUE(feedback.executing);
  EXPECT_DOUBLE_EQ(feedback.progress_m, 1.5);
  fb.state = mgg_msgs::msg::LocalPathFeedback::IDLE;
  EXPECT_FALSE(fromFeedbackMsg(fb).executing);

  mgg_msgs::msg::GlobalGuidance g;
  g.header.frame_id = "map";
  g.session_id = "session-a";
  g.sequence_id = 8;
  g.kind = mgg_msgs::msg::GlobalGuidance::TARGET;
  g.target.x = 3;
  g.route.resize(2);
  g.route[1].y = 2;
  const auto guidance = fromGuidanceMsg(g);
  EXPECT_EQ(guidance.kind, GuidanceKind::kTarget);
  EXPECT_EQ(guidance.frame_id, "map");
  EXPECT_EQ(guidance.route.size(), 2u);
  EXPECT_DOUBLE_EQ(guidance.route[1].y(), 2);
  EXPECT_FALSE(guidance.standing_start.valid);
  g.standing_start.valid = true;
  g.standing_start.center.x = -1;
  g.standing_start.radius = 1.5;
  g.standing_start.boot = 11;
  const auto standing = fromGuidanceMsg(g).standing_start;
  EXPECT_TRUE(standing.valid);
  EXPECT_DOUBLE_EQ(standing.center.x(), -1);
  EXPECT_DOUBLE_EQ(standing.radius, 1.5);
  EXPECT_EQ(standing.boot, 11u);
  g.kind = 9;
  EXPECT_THROW(fromGuidanceMsg(g), std::invalid_argument);

  mgg_msgs::srv::SetLocalPlannerMode::Request req;
  req.session_id = "session-b";
  req.request_id = "request-9";
  req.mode = mgg_msgs::srv::SetLocalPlannerMode::Request::FOLLOW_ROUTE;
  req.frame_id = "map";
  req.goal.x = 4;
  req.tolerance_m = 0.5;
  const auto mode = fromModeRequest(req);
  EXPECT_EQ(mode.session_id, req.session_id);
  EXPECT_EQ(mode.request_id, req.request_id);
  EXPECT_EQ(mode.mode, LocalMode::kFollowRoute);
  EXPECT_EQ(mode.frame_id, "map");
  EXPECT_DOUBLE_EQ(mode.goal.x(), 4);
  req.mode = 3;
  EXPECT_THROW(fromModeRequest(req), std::invalid_argument);
}
