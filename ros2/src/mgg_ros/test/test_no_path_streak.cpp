#include <gtest/gtest.h>
#include "mgg_ros/no_path_streak.h"

TEST(NoPathStreak, SixIdenticalAnswersThenResetBySuccessOrMissingIdentity) {
  mgg_ros::NoPathStreak streak;
  Eigen::Vector4d pose(24.46, -57.62, 0, 0.71);
  for (int i = 1; i <= 6; ++i)
    EXPECT_EQ(streak.note("geometry", pose, true), i == 6);
  EXPECT_FALSE(streak.note("geometry", pose, false));
  EXPECT_FALSE(streak.note("geometry", pose, true));
  for (int i = 0; i < 10; ++i) EXPECT_FALSE(streak.note("", pose, true));
}

TEST(NoPathStreak, ChangesResetAndToleranceUsesFixedAnchorNotPreviousAnswer) {
  mgg_ros::NoPathStreak streak;
  Eigen::Vector4d pose = Eigen::Vector4d::Zero();
  for (int i = 0; i < 5; ++i) EXPECT_FALSE(streak.note("a", pose, true));
  EXPECT_FALSE(streak.note("b", pose, true));
  pose.x() = 0.11;
  EXPECT_FALSE(streak.note("b", pose, true));
  pose[3] = 0.11;
  EXPECT_FALSE(streak.note("b", pose, true));
  for (int i = 0; i < 10; ++i) {
    pose.x() += 0.06;
    EXPECT_FALSE(streak.note("b", pose, true));
  }
}

TEST(NoPathStreak, SmallJitterAndWrappedHeadingCountAsSamePose) {
  mgg_ros::NoPathStreak streak;
  Eigen::Vector4d pose(0, 0, 0, M_PI - 0.02);
  EXPECT_FALSE(streak.note("a", pose, true));
  pose.x() = 0.09;
  pose[3] = -M_PI + 0.02;
  for (int i = 2; i <= 6; ++i)
    EXPECT_EQ(streak.note("a", pose, true), i == 6);
}
