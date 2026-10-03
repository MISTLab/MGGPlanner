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

TEST(NoPathStreak, NoGoAndScoutingChangesStartANewStreak) {
  // ground12 review P2-1: the identity covered only map geometry and pose,
  // so a robot waiting on a no-go zone about to expire was blocked.
  mgg_ros::NoPathStreak streak;
  const Eigen::Vector4d pose(1, 2, 0, 0);
  const auto key = [](std::uint64_t scouting, std::vector<Eigen::Vector2d> zones) {
    return "geometry:" + mgg_ros::noPathPlanningInputs(
        scouting, zones, std::vector<double>(zones.size(), 0.8));
  };
  for (int i = 0; i < 5; ++i) EXPECT_FALSE(streak.note(key(3, {{4, 5}}), pose, true));
  // The zone expires: the same map and pose is a new input.
  for (int i = 1; i <= 6; ++i) EXPECT_EQ(streak.note(key(3, {}), pose, true), i == 6);
  // New scouting exclusions.
  for (int i = 1; i <= 6; ++i) EXPECT_EQ(streak.note(key(4, {}), pose, true), i == 6);
  EXPECT_NE(key(4, {{4, 5}}), key(4, {{4, 5.5}}));
}
