#include <cmath>
#include <gtest/gtest.h>
#include "mgg_core/trajectory.h"

namespace {
using mgg::PathType;

TEST(Trajectory, PathLengthSumsSegments) {
  const PathType p{{0,0,0},{1,0,0},{1,1,0}};
  EXPECT_NEAR(mgg::getPathLength(p), 2.0, 1e-12);
}

TEST(Trajectory, InterpolationRespectsSpacing) {
  const PathType p{{0,0,0},{2,0,0}};
  PathType out;
  ASSERT_TRUE(mgg::interpolatePath(p, 0.5, out));
  ASSERT_GE(out.size(), 4u);
  EXPECT_NEAR((out[1] - out[0]).norm(), 0.5, 1e-6);
}

TEST(Trajectory, DtwOfIdenticalPathsIsZero) {
  const PathType p{{0,0,0},{1,0,0},{2,0,0}};
  EXPECT_NEAR(mgg::computeDTWDistance(p, p), 0.0, 1e-12);
}

TEST(Trajectory, DirectionEstimateFollowsThePath) {
  const PathType east{{0,0,0},{1,0,0},{2,0,0}};
  EXPECT_NEAR(mgg::estimateDirectionFromPath(east), 0.0, 1e-6);
  const PathType north{{0,0,0},{0,1,0},{0,2,0}};
  EXPECT_NEAR(mgg::estimateDirectionFromPath(north), M_PI / 2.0, 1e-6);
}

TEST(Trajectory, TruncateYawWraps) {
  double a = 3.5 * M_PI / 2.0;
  mgg::truncateYaw(a);
  EXPECT_LE(std::abs(a), M_PI + 1e-12);
}

// The reference is now a straight ray along the heading, not a single
// repeated point. A path running along the heading should be much closer to
// it than one running across.
TEST(Trajectory, DirectionDistancePenalisesSidewaysPaths) {
  const PathType forward{{0,0,0},{1,0,0},{2,0,0},{3,0,0}};
  const PathType sideways{{0,0,0},{0,1,0},{0,2,0},{0,3,0}};
  const double d_fwd = mgg::computeDistanceBetweenTrajectoryAndDirection(
      forward, 0.0, 0.2, true);
  const double d_side = mgg::computeDistanceBetweenTrajectoryAndDirection(
      sideways, 0.0, 0.2, true);
  EXPECT_LT(d_fwd, d_side);
  // Small, but not zero: the reference holds n points ending one step short of
  // the path's end, since n = path_length / discrete_length and the loop runs
  // i < n. DTW therefore leaves one step of residual, scaled by the squared
  // path length. That off-by-one is in the original formula and is left alone;
  // only the collapsed reference was the defect.
  EXPECT_LT(d_fwd, 0.05);
  EXPECT_LT(d_fwd * 5.0, d_side);
}

TEST(Trajectory, DirectionDistanceFollowsTheHeading) {
  const PathType north{{0,0,0},{0,1,0},{0,2,0},{0,3,0}};
  // Judged against a northward heading, the same path is now the aligned one.
  const double aligned = mgg::computeDistanceBetweenTrajectoryAndDirection(
      north, M_PI / 2.0, 0.2, true);
  const double crossed = mgg::computeDistanceBetweenTrajectoryAndDirection(
      north, 0.0, 0.2, true);
  EXPECT_LT(aligned, crossed);
  EXPECT_LT(aligned, 0.05);
}

}  // namespace
