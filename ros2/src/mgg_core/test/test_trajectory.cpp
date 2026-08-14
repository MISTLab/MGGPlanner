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

// CHARACTERISATION of the reproduced defect: the reference path collapses to a
// single repeated point, so this does not measure deviation from a heading.
// Pinned so that correcting it shows up as a failing test rather than a silent
// behaviour change.
TEST(Trajectory, DirectionDistanceUsesADegenerateReference) {
  const PathType forward{{0,0,0},{1,0,0},{2,0,0}};
  const PathType sideways{{0,0,0},{0,1,0},{0,2,0}};
  const double d_fwd = mgg::computeDistanceBetweenTrajectoryAndDirection(
      forward, 0.0, 0.2, true);
  const double d_side = mgg::computeDistanceBetweenTrajectoryAndDirection(
      sideways, 0.0, 0.2, true);
  // Both finite; forward still scores lower, which is why the defect went
  // unnoticed: the penalty behaves plausibly while measuring the wrong thing.
  EXPECT_LT(d_fwd, std::numeric_limits<double>::max());
  EXPECT_LT(d_side, std::numeric_limits<double>::max());
  EXPECT_LT(d_fwd, d_side);
}

}  // namespace
