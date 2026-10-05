#include <gtest/gtest.h>

#include "mgg_core/local_path.h"
using namespace mgg;
TEST(LocalPath, BrakingCouplesCommitment) {
  BrakingBounds b;
  b.deceleration_mps2 = 0.2;
  b.latency_s = 0.4;
  const double v = 2;
  double commit_m = commitmentLength(v, b);
  EXPECT_GE(commit_m, v * v / (2 * b.deceleration_mps2) +
                          v * (b.latency_s + b.planning_latency_s) +
                          b.margin_m);
  EXPECT_GE(commit_m, 1.5 * v);
  EXPECT_LT(commitmentSpeedCap(2 * v, b), v);
  EXPECT_GE(commitmentSpeedCap(2 * v, b), 0);
}
TEST(LocalPath, ExactPrefixAndSplice) {
  LocalPathPlan path;
  path.sequence_id = 7;
  for (int i = 0; i <= 40; ++i) {
    path.poses.emplace_back(i * .25, 0, 0, 0);
    path.reverse.push_back(false);
  }
  auto prefix = committedPrefix(path, .6, 1.2);
  EXPECT_EQ(prefix.poses.front(), path.poses[2]);
  EXPECT_EQ(prefix.poses.back(), path.poses[8]);
  LocalPathPlan extension;
  extension.poses = {prefix.poses.back(), StateVec(3, 0, 0, 0)};
  extension.reverse = {false, false};
  auto joined = splicePath(prefix, extension);
  EXPECT_EQ(extension.poses.front(), prefix.poses.back());
  EXPECT_EQ(joined.poses.size(), prefix.poses.size() + 1);
  extension.poses.front()[0] += .01;
  EXPECT_THROW(splicePath(prefix, extension), std::invalid_argument);
}
TEST(LocalPath, RemainingCommitmentEndsAtTheCommitment) {
  LocalPathPlan path;
  for (int i = 0; i <= 40; ++i) {
    path.poses.emplace_back(i * .25, 0, 0, 0);
    path.reverse.push_back(false);
  }
  path.commit_length_m = 1.5;
  // From progress 0.6 to the commitment's end at 1.5 m, original vertices.
  auto rest = remainingCommitment(path, .6);
  EXPECT_EQ(rest.poses.front(), path.poses[2]);
  EXPECT_EQ(rest.poses.back(), path.poses[6]);
  // Past the commitment: the segment the robot is on, nothing more.
  rest = remainingCommitment(path, 2.1);
  EXPECT_EQ(rest.poses.front(), path.poses[8]);
  EXPECT_EQ(rest.poses.back(), path.poses[9]);
}
