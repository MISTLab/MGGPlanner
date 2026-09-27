// Tests for the fleet's exchange types (tour-exploration design §3.3, §3.4).

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mgg_core/fleet_types.h"

namespace {

using mgg::FleetCluster;
using mgg::TourAwardData;
using mgg::TourBidData;

FleetCluster cluster(mgg::ClusterId id, double x) {
  FleetCluster c;
  c.id = id;
  c.owner_robot_id = 1;
  c.position = Eigen::Vector3d(x, 0.0, 0.0);
  c.gain = 1000.0;
  return c;
}

TourBidData twoClusterBid() {
  TourBidData bid;
  bid.robot_id = 1;
  bid.auctioneer_id = 1;
  bid.pose = mgg::StateVec(0.0, 0.0, 0.0, 0.0);
  bid.clusters = {cluster(11, 3.0), cluster(12, 7.0)};
  bid.costs_from_pose = {3.0, 7.0};
  bid.costs_between = {0.0, 4.0, 4.0, 0.0};
  return bid;
}

TEST(FleetTypes, WellFormedRejectsInconsistentOrInvalidBids) {
  // Review Focus 1: a peer's bid arrives over a link nobody controls.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const TourBidData bid = twoClusterBid();
  EXPECT_TRUE(bid.wellFormed());
  TourBidData b = bid;
  b.costs_from_pose.push_back(1.0);
  EXPECT_FALSE(b.wellFormed());
  b = bid;
  b.costs_between.pop_back();
  EXPECT_FALSE(b.wellFormed());
  b = bid;
  b.costs_between[1] = nan;
  EXPECT_FALSE(b.wellFormed());
  b = bid;
  b.costs_from_pose[0] = -1.0;
  EXPECT_FALSE(b.wellFormed());
  b = bid;
  b.pose[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(b.wellFormed());
  b = bid;
  b.clusters[0].id = mgg::kNoCluster;
  EXPECT_FALSE(b.wellFormed());
  b = bid;
  b.clusters[1].position.x() = nan;
  EXPECT_FALSE(b.wellFormed());
  // Unreachable is a cost like any other.
  b = bid;
  b.costs_between[1] = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(b.wellFormed());
  // A robot with nothing to report still bids.
  TourBidData empty;
  empty.auctioneer_id = empty.robot_id;
  EXPECT_TRUE(empty.wellFormed());
}

TEST(FleetTypes, WellFormedRejectsUnnamedBundleAndExploredEntries) {
  const TourBidData bid = twoClusterBid();
  TourBidData b = bid;
  b.bundle = {11, mgg::kNoCluster};
  EXPECT_FALSE(b.wellFormed());
  b = bid;
  b.explored = {mgg::kNoCluster};
  EXPECT_FALSE(b.wellFormed());
  // No current target is kNoCluster.
  b = bid;
  b.current_target = mgg::kNoCluster;
  EXPECT_TRUE(b.wellFormed());
}

TEST(FleetTypes, WellFormedCapsEveryListAtTheBidClusterLimit) {
  const auto bidWith = [](std::size_t n) {
    TourBidData bid;
    bid.auctioneer_id = bid.robot_id;
    for (std::size_t i = 0; i < n; ++i) {
      bid.clusters.push_back(cluster(i + 1, static_cast<double>(i)));
    }
    bid.costs_from_pose.assign(n, 0.0);
    bid.costs_between.assign(n * n, 0.0);
    return bid;
  };
  EXPECT_TRUE(bidWith(mgg::kMaxBidClusters).wellFormed());
  EXPECT_FALSE(bidWith(mgg::kMaxBidClusters + 1).wellFormed());
  TourBidData b;
  b.auctioneer_id = b.robot_id;
  b.bundle.assign(mgg::kMaxBidClusters, 11);
  b.explored.assign(mgg::kMaxBidClusters, 12);
  EXPECT_TRUE(b.wellFormed());
  b.bundle.push_back(11);
  EXPECT_FALSE(b.wellFormed());
  b.bundle.pop_back();
  b.explored.push_back(12);
  EXPECT_FALSE(b.wellFormed());
}

TEST(FleetTypes, CostBetweenReadsTheRowMajorMatrix) {
  TourBidData bid = twoClusterBid();
  bid.costs_between = {0.0, 4.0, 5.0, 0.0};
  EXPECT_DOUBLE_EQ(bid.costBetween(0, 1), 4.0);
  EXPECT_DOUBLE_EQ(bid.costBetween(1, 0), 5.0);
}

TEST(FleetTypes, AnAwardFindsBundlesAndClustersById) {
  TourAwardData award;
  award.clusters = {cluster(11, 3.0), cluster(21, 9.0)};
  award.bundles = {{1, {11}, 0.0}, {2, {21}, 4.5}};
  ASSERT_NE(award.bundleOf(2), nullptr);
  EXPECT_DOUBLE_EQ(award.bundleOf(2)->silent_s, 4.5);
  EXPECT_EQ(award.bundleOf(3), nullptr);
  ASSERT_NE(award.cluster(21), nullptr);
  EXPECT_DOUBLE_EQ(award.cluster(21)->position.x(), 9.0);
  EXPECT_EQ(award.cluster(99), nullptr);
}

TEST(FleetTypes, ADronesSpeedReachAndHomeMustBeValid) {
  mgg::TourBidData bid;
  EXPECT_TRUE(bid.wellFormed());
  EXPECT_TRUE(std::isinf(bid.reach_m));
  bid.speed_mps = -1.0;
  EXPECT_FALSE(bid.wellFormed());
  bid.speed_mps = 4.0;
  bid.reach_m = std::nan("");
  EXPECT_FALSE(bid.wellFormed());
  bid.reach_m = 120.0;
  bid.home = Eigen::Vector3d(0.0, std::numeric_limits<double>::infinity(), 0.0);
  EXPECT_FALSE(bid.wellFormed());
  bid.home = Eigen::Vector3d::Zero();
  EXPECT_TRUE(bid.wellFormed());
}

}  // namespace
