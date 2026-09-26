// Tests for the tour planner (tour-exploration design §2.3, §2.4): when the
// tour is solved again, the commitment rule, and whether a local path
// serves the target.

#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/tour_planner.h"

namespace {

using mgg::ClusterId;
using mgg::FrontierCluster;
using mgg::TourCostMatrix;
using mgg::TourPlan;
using mgg::TourPlanner;

FrontierCluster named(ClusterId id, double x) {
  FrontierCluster c;
  c.id = id;
  c.owner_robot_id = 1;
  c.representative_vertex_id = static_cast<int>(id);
  c.position = Eigen::Vector3d(x, 0.0, 0.0);
  c.member_vertex_ids = {static_cast<int>(id)};
  return c;
}

/// Two clusters, `a` and `b` from the robot, `apart` between them.
TourCostMatrix twoClusters(double a, double b, double apart) {
  TourCostMatrix costs;
  costs.from_robot = {a, b};
  costs.between = {{0.0, apart}, {apart, 0.0}};
  return costs;
}

TEST(TourPlanner, TheFirstSolveTargetsTheTourFirstCluster) {
  TourPlanner planner{mgg::TourParams{}};
  const std::vector<FrontierCluster> clusters{named(1, 10.0), named(2, -11.0)};
  EXPECT_TRUE(planner.needsSolve(clusters, 1, 0, 0.0));
  EXPECT_EQ(planner.target(), mgg::kNoCluster);
  const TourPlan& plan =
      planner.solve(clusters, twoClusters(10.0, 11.0, 30.0), 1, 0, 0.0);
  ASSERT_EQ(plan.clusters.size(), 2u);
  EXPECT_EQ(plan.clusters.front().id, 1u);
  EXPECT_EQ(planner.target(), 1u);
  EXPECT_DOUBLE_EQ(plan.cost, 40.0);
  EXPECT_FALSE(plan.kept_target);
  EXPECT_DOUBLE_EQ(planner.targetSince(), 0.0);
}

TEST(TourPlanner, TheCommitmentMarginPreventsFlipFlop) {
  TourPlanner planner{mgg::TourParams{}};  // commit_margin 0.2
  const std::vector<FrontierCluster> clusters{named(1, 10.0), named(2, -11.0)};
  planner.solve(clusters, twoClusters(10.0, 11.0, 30.0), 1, 0, 0.0);
  ASSERT_EQ(planner.target(), 1u);

  // Starting at b is now 2.5 % cheaper (39 against 40): not enough.
  const TourPlan& kept =
      planner.solve(clusters, twoClusters(10.0, 9.0, 30.0), 2, 0, 1.0);
  EXPECT_EQ(planner.target(), 1u);
  EXPECT_TRUE(kept.kept_target);
  EXPECT_DOUBLE_EQ(kept.cost, 40.0);
  EXPECT_DOUBLE_EQ(planner.targetSince(), 0.0);

  // Starting at b saves 44 % (39 against 70): the target switches.
  const TourPlan& switched =
      planner.solve(clusters, twoClusters(40.0, 9.0, 30.0), 3, 0, 2.0);
  EXPECT_EQ(planner.target(), 2u);
  EXPECT_FALSE(switched.kept_target);
  EXPECT_DOUBLE_EQ(planner.targetSince(), 2.0);
}

TEST(TourPlanner, SolvesOnChangeAtMostEveryIntervalAndAtOnceWhenTheTargetGoes) {
  mgg::TourParams params;
  params.recompute_interval_s = 1.0;
  TourPlanner planner(params);
  const std::vector<FrontierCluster> clusters{named(1, 10.0), named(2, -11.0)};
  planner.solve(clusters, twoClusters(10.0, 11.0, 30.0), 1, 0, 0.0);

  EXPECT_FALSE(planner.needsSolve(clusters, 1, 0, 5.0));  // nothing changed
  EXPECT_FALSE(planner.needsSolve(clusters, 2, 0, 0.5));  // too soon
  EXPECT_TRUE(planner.needsSolve(clusters, 2, 0, 1.0));   // graph changed
  EXPECT_TRUE(planner.needsSolve(clusters, 1, 1, 1.0));   // assignment changed
  EXPECT_TRUE(planner.needsSolve({named(1, 10.0)}, 1, 0, 1.0));  // set changed
  // The target's cluster is gone: at once.
  EXPECT_TRUE(planner.needsSolve({named(2, -11.0)}, 1, 0, 0.1));
  // The target was reached and released: at once.
  planner.releaseTarget();
  EXPECT_EQ(planner.target(), mgg::kNoCluster);
  EXPECT_TRUE(planner.needsSolve(clusters, 1, 0, 0.1));
}

TEST(TourPlanner, AnUnreachableClusterIsLeftOut) {
  TourPlanner planner{mgg::TourParams{}};
  const std::vector<FrontierCluster> clusters{named(1, 10.0), named(2, -11.0)};
  const TourPlan& plan = planner.solve(
      clusters, twoClusters(10.0, mgg::kUnreachableCost, 30.0), 1, 0, 0.0);
  ASSERT_EQ(plan.clusters.size(), 1u);
  EXPECT_EQ(planner.target(), 1u);
  const TourPlan& none = planner.solve(
      clusters,
      twoClusters(mgg::kUnreachableCost, mgg::kUnreachableCost, 30.0), 2, 0,
      1.0);
  EXPECT_TRUE(none.clusters.empty());
  EXPECT_EQ(planner.target(), mgg::kNoCluster);
}

TEST(LocalPathServesTarget, AheadOrInsideTheLatticeOnly) {
  const Eigen::Vector3d lo(-1.0, -1.0, 0.0);
  const Eigen::Vector3d hi(3.0, 1.0, 0.0);
  const Eigen::Vector3d robot = Eigen::Vector3d::Zero();
  const Eigen::Vector3d far(20.0, 0.0, 0.0);
  EXPECT_TRUE(mgg::localPathServesTarget(robot, {2.0, 0.5, 0.0}, far, lo, hi));
  EXPECT_FALSE(mgg::localPathServesTarget(robot, {-2.0, 0.0, 0.0}, far, lo, hi));
  EXPECT_FALSE(mgg::localPathServesTarget(robot, {0.0, 2.0, 0.0}, far, lo, hi));
  EXPECT_FALSE(mgg::localPathServesTarget(robot, robot, far, lo, hi));
  // A target inside the lattice box is served by whatever local path.
  EXPECT_TRUE(mgg::localPathServesTarget(robot, {-1.0, 0.0, 0.0},
                                         {2.0, 0.5, 0.0}, lo, hi));
}

}  // namespace
