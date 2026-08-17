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

// --- shortcutPath -----------------------------------------------------------

namespace {

/// Everything is traversable.
bool alwaysFree(const mgg::VectorType&, const mgg::VectorType&) { return true; }

/// A wall along x = 1, blocking any segment that crosses it away from y = 0.
/// Sampled rather than solved so the test exercises the same shape of check a
/// real map does.
bool blockedByWall(const mgg::VectorType& a, const mgg::VectorType& b) {
  for (int i = 0; i <= 20; ++i) {
    const mgg::VectorType p = a + (b - a) * (i / 20.0);
    if (std::abs(p.x() - 1.0) < 0.05 && p.y() > 0.5) return false;
  }
  return true;
}

}  // namespace

TEST(Trajectory, ShortcutCollapsesAStaircase) {
  // The characteristic grid-graph output: a diagonal walked as alternating
  // axis-aligned steps.
  mgg::PathType staircase;
  for (int i = 0; i < 5; ++i) {
    staircase.emplace_back(i * 0.25, i * 0.25, 0.0);
    staircase.emplace_back((i + 1) * 0.25, i * 0.25, 0.0);
  }
  const mgg::PathType out = mgg::shortcutPath(staircase, alwaysFree);
  // With nothing in the way the whole staircase is one straight run.
  EXPECT_EQ(out.size(), 2u);
  EXPECT_EQ(out.front(), staircase.front());
  EXPECT_EQ(out.back(), staircase.back());
}

TEST(Trajectory, ShortcutKeepsTheDetourItNeeds) {
  // Around a wall: the middle point is what makes the path legal, so it has to
  // survive.
  const mgg::PathType path = {{0.0, 1.0, 0.0},
                              {0.5, 0.0, 0.0},
                              {1.5, 0.0, 0.0},
                              {2.0, 1.0, 0.0}};
  const mgg::PathType out = mgg::shortcutPath(path, blockedByWall);
  EXPECT_GT(out.size(), 2u) << "shortcut cut straight through the wall";
  EXPECT_EQ(out.front(), path.front());
  EXPECT_EQ(out.back(), path.back());
  for (size_t i = 1; i < out.size(); ++i) {
    EXPECT_TRUE(blockedByWall(out[i - 1], out[i]))
        << "segment " << i << " of the shortcut path is not traversable";
  }
}

TEST(Trajectory, ShortcutLeavesShortPathsAlone) {
  const mgg::PathType two = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  EXPECT_EQ(mgg::shortcutPath(two, alwaysFree).size(), 2u);
  EXPECT_TRUE(mgg::shortcutPath({}, alwaysFree).empty());
}

TEST(Trajectory, ShortcutWithoutAPredicateIsIdentity) {
  // A caller with no map to check against must get its path back untouched
  // rather than a straight line through whatever is there.
  const mgg::PathType path = {{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {2.0, 0.0, 0.0}};
  EXPECT_EQ(mgg::shortcutPath(path, nullptr).size(), path.size());
}
