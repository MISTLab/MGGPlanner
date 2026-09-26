// Tests for the claims robots hold on frontier clusters (tour-exploration
// design §4): they outlive silence until the TTL, and are released early
// when explored, by an idle robot's take-over, or by the operator.

#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/fleet_claims.h"

namespace {

using mgg::ClaimRegistry;
using mgg::FleetCluster;

FleetCluster cluster(mgg::ClusterId id, int owner, double x) {
  FleetCluster c;
  c.id = id;
  c.owner_robot_id = owner;
  c.position = Eigen::Vector3d(x, 0.0, 0.0);
  return c;
}

TEST(ClaimRegistry, AClaimLastsUntilItsRobotHasBeenSilentForTheTtl) {
  ClaimRegistry claims;
  claims.record(2, {cluster(21, 2, 10.0)}, 100.0);
  EXPECT_TRUE(claims.expire(100.0 + 599.0, 600.0).empty());
  ASSERT_NE(claims.find(2), nullptr);
  // Heard again: the TTL counts from here.
  claims.heard(2, 300.0);
  EXPECT_TRUE(claims.expire(850.0, 600.0).empty());
  EXPECT_EQ(claims.expire(901.0, 600.0), std::vector<int>{2});
  EXPECT_EQ(claims.find(2), nullptr);
}

TEST(ClaimRegistry, RecordingKeepsTheLatestHeardTimeAndDropsAnEmptyBundle) {
  ClaimRegistry claims;
  claims.record(2, {cluster(21, 2, 10.0)}, 200.0);
  claims.record(2, {cluster(21, 2, 10.0), cluster(22, 2, 12.0)}, 150.0);
  ASSERT_NE(claims.find(2), nullptr);
  EXPECT_EQ(claims.find(2)->clusters.size(), 2u);
  EXPECT_DOUBLE_EQ(claims.find(2)->last_heard_s, 200.0);
  claims.record(2, {}, 250.0);
  EXPECT_EQ(claims.find(2), nullptr);
  // Hearing a robot that holds nothing records nothing.
  claims.heard(5, 10.0);
  EXPECT_TRUE(claims.robots().empty());
}

TEST(ClaimRegistry, ExploredClustersLeaveEveryClaim) {
  ClaimRegistry claims;
  claims.record(2, {cluster(21, 2, 10.0), cluster(22, 2, 20.0)}, 0.0);
  claims.record(3, {cluster(31, 3, -10.0)}, 0.0);
  const int dropped = claims.dropExplored(
      [](const Eigen::Vector3d& p) { return p.x() < 0.0 || p.x() > 15.0; });
  EXPECT_EQ(dropped, 2);
  ASSERT_NE(claims.find(2), nullptr);
  ASSERT_EQ(claims.find(2)->clusters.size(), 1u);
  EXPECT_EQ(claims.find(2)->clusters[0].id, 21u);
  EXPECT_EQ(claims.find(3), nullptr);
  EXPECT_EQ(claims.dropExplored(nullptr), 0);
}

TEST(ClaimRegistry, TheOperatorReleasesAClaim) {
  ClaimRegistry claims;
  claims.record(2, {cluster(21, 2, 10.0)}, 0.0);
  EXPECT_TRUE(claims.release(2));
  EXPECT_FALSE(claims.release(2));
  EXPECT_EQ(claims.find(2), nullptr);
}

TEST(ClaimRegistry, TheLongestSilentCandidateIsReleasedFirst) {
  ClaimRegistry claims;
  claims.record(2, {cluster(21, 2, 10.0)}, 50.0);
  claims.record(3, {cluster(31, 3, 20.0)}, 20.0);
  claims.record(4, {cluster(41, 4, 30.0)}, 10.0);
  // Robot 4 has been silent longest but is not a candidate (in the group).
  EXPECT_EQ(claims.releaseOldest({2, 3}), 3);
  EXPECT_EQ(claims.releaseOldest({2, 3}), 2);
  EXPECT_EQ(claims.releaseOldest({2, 3}), -1);
  EXPECT_NE(claims.find(4), nullptr);
}

TEST(ClaimRegistry, ClustersByHolder) {
  ClaimRegistry claims;
  claims.record(1, {cluster(11, 1, 1.0)}, 0.0);
  claims.record(2, {cluster(21, 2, 2.0), cluster(22, 2, 3.0)}, 0.0);
  claims.record(3, {cluster(31, 3, 4.0)}, 0.0);
  EXPECT_EQ(claims.robots(), (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(claims.clustersOf({2, 3}).size(), 3u);
  const std::vector<FleetCluster> others = claims.clustersExcept(1);
  ASSERT_EQ(others.size(), 3u);
  for (const FleetCluster& c : others) EXPECT_NE(c.owner_robot_id, 1);
}

// Keeping a pre-reset timestamp would delay expiry until the old clock
// catches up. Rebase once, then age normally in the new clock epoch.
TEST(ClaimRegistry, ExpiryAfterClockRollbackRestartsTheTtl) {
  ClaimRegistry claims;
  claims.record(2, {cluster(21, 2, 10.0)}, 10000.0);
  EXPECT_TRUE(claims.expire(10010.0, 600.0).empty());
  EXPECT_TRUE(claims.expire(10.0, 600.0).empty());
  ASSERT_NE(claims.find(2), nullptr);
  EXPECT_DOUBLE_EQ(claims.find(2)->last_heard_s, 10.0);
  EXPECT_TRUE(claims.expire(609.0, 600.0).empty());
  EXPECT_TRUE(claims.expire(610.0, 600.0).empty());
  EXPECT_EQ(claims.expire(611.0, 600.0), std::vector<int>{2});
  EXPECT_EQ(claims.find(2), nullptr);
}

// Taking max(old, receipt) would ignore contact in the new clock epoch.
TEST(ClaimRegistry, HearingAfterClockRollbackUsesTheLocalReceiptTime) {
  ClaimRegistry claims;
  claims.record(2, {cluster(21, 2, 10.0)}, 10000.0);
  claims.heard(2, 10.0);
  ASSERT_NE(claims.find(2), nullptr);
  EXPECT_DOUBLE_EQ(claims.find(2)->last_heard_s, 10.0);
  EXPECT_TRUE(claims.expire(610.0, 600.0).empty());
  EXPECT_EQ(claims.expire(611.0, 600.0), std::vector<int>{2});
  EXPECT_EQ(claims.find(2), nullptr);
}

TEST(ClaimRegistry, AFutureAwardTimeIsRebasedBeforeItsClaimExpires) {
  ClaimRegistry claims;
  claims.record(2, {cluster(21, 2, 10.0)}, 100.0);
  claims.record(2, {cluster(22, 2, 12.0)}, 10000.0);
  ASSERT_NE(claims.find(2), nullptr);
  EXPECT_DOUBLE_EQ(claims.find(2)->last_heard_s, 10000.0);
  EXPECT_TRUE(claims.expire(100.0, 600.0).empty());
  ASSERT_NE(claims.find(2), nullptr);
  EXPECT_DOUBLE_EQ(claims.find(2)->last_heard_s, 100.0);
  ASSERT_EQ(claims.find(2)->clusters.size(), 1u);
  EXPECT_EQ(claims.find(2)->clusters[0].id, 22u);
  EXPECT_TRUE(claims.expire(700.0, 600.0).empty());
  EXPECT_EQ(claims.expire(701.0, 600.0), std::vector<int>{2});
  EXPECT_EQ(claims.find(2), nullptr);
}

}  // namespace
