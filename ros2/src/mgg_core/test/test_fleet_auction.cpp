// Tests for the auctioneer's cluster pool and sequential single-item
// auction (tour-exploration design §3.3, §3.4).

#include <cmath>
#include <limits>
#include <map>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/fleet_auction.h"
#include "mgg_core/tour_solver.h"

namespace {

using mgg::AuctionBidder;
using mgg::AuctionResult;
using mgg::ClusterPool;
using mgg::FleetCluster;
using mgg::TourBidData;

FleetCluster cluster(mgg::ClusterId id, int owner, double x, double y = 0.0) {
  FleetCluster c;
  c.id = id;
  c.owner_robot_id = owner;
  c.position = Eigen::Vector3d(x, y, 0.0);
  c.gain = 1000.0;
  return c;
}

double euclid(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return (a - b).norm();
}

/// A bid from `robot` at (x, 0) listing `clusters` at straight-line costs.
TourBidData bidAt(int robot, double x, const std::vector<FleetCluster>& clusters) {
  TourBidData bid;
  bid.robot_id = robot;
  bid.pose = mgg::StateVec(x, 0.0, 0.0, 0.0);
  bid.clusters = clusters;
  for (const FleetCluster& c : clusters) {
    bid.costs_from_pose.push_back(euclid(bid.pose.head<3>(), c.position));
  }
  for (const FleetCluster& a : clusters) {
    for (const FleetCluster& b : clusters) {
      bid.costs_between.push_back(euclid(a.position, b.position));
    }
  }
  return bid;
}

/// A bidder at `at` on a line of clusters at `xs`.
AuctionBidder onLine(int robot, double at, const std::vector<double>& xs) {
  AuctionBidder b;
  b.robot_id = robot;
  for (const double x : xs) b.from_pose.push_back(std::abs(x - at));
  for (const double x : xs) {
    b.between.emplace_back();
    for (const double y : xs) b.between.back().push_back(std::abs(x - y));
  }
  return b;
}

/// A place along one of two corridors that meet at the origin.
struct CorridorPoint {
  char corridor;
  double along;
};

double corridorDistance(const CorridorPoint& p, const CorridorPoint& q) {
  return p.corridor == q.corridor ? std::abs(p.along - q.along)
                                  : p.along + q.along;
}

/// A bidder standing where the corridors meet.
AuctionBidder atJunction(int robot, const std::vector<CorridorPoint>& points) {
  AuctionBidder b;
  b.robot_id = robot;
  for (const CorridorPoint& p : points) b.from_pose.push_back(p.along);
  for (const CorridorPoint& p : points) {
    b.between.emplace_back();
    for (const CorridorPoint& q : points) {
      b.between.back().push_back(corridorDistance(p, q));
    }
  }
  return b;
}

/// The best open tour over `indices` for `b`.
double bundleCost(const AuctionBidder& b, const std::vector<int>& indices) {
  std::vector<double> from;
  std::vector<std::vector<double>> between;
  for (const int i : indices) {
    from.push_back(b.from_pose[i]);
    between.emplace_back();
    for (const int j : indices) between.back().push_back(b.between[i][j]);
  }
  return mgg::solveOpenTour(from, between).cost;
}

/// Every assignment of `n` clusters to the bidders, each bundle toured
/// optimally: the least total.
double bruteForceMinSum(const std::vector<AuctionBidder>& bidders,
                        std::size_t n) {
  const std::size_t m = bidders.size();
  std::size_t assignments = 1;
  for (std::size_t c = 0; c < n; ++c) assignments *= m;
  double best = mgg::kUnreachableCost;
  for (std::size_t code = 0; code < assignments; ++code) {
    std::vector<std::vector<int>> parts(m);
    std::size_t rest = code;
    for (std::size_t c = 0; c < n; ++c) {
      parts[rest % m].push_back(static_cast<int>(c));
      rest /= m;
    }
    double total = 0.0;
    for (std::size_t r = 0; r < m; ++r) total += bundleCost(bidders[r], parts[r]);
    best = std::min(best, total);
  }
  return best;
}

double auctionCost(const std::vector<AuctionBidder>& bidders,
                   const AuctionResult& result) {
  double total = 0.0;
  for (const AuctionBidder& b : bidders) {
    total += mgg::openTourCost(result.bundles.at(b.robot_id), b.from_pose,
                               b.between);
  }
  return total;
}

TEST(ClusterPool, MergesClustersWithinTheRadiusUnderTheOwnersName) {
  // Robot 2 lists robot 1's frontier, merged into its roadmap, under its own
  // quantization: one place, two names.
  const TourBidData a =
      bidAt(1, 0.0, {cluster(100, 1, 10.0), cluster(101, 1, 30.0)});
  const TourBidData b =
      bidAt(2, 20.0, {cluster(200, 1, 10.5), cluster(201, 2, 25.0)});
  const ClusterPool pool = mgg::buildClusterPool({a, b}, {}, 2.0, nullptr);
  ASSERT_EQ(pool.clusters.size(), 3u);
  const int p = pool.indexOf(200);
  ASSERT_GE(p, 0);
  EXPECT_EQ(pool.clusters[p].id, 100u);
  EXPECT_EQ(pool.indexOf(100), p);
  EXPECT_EQ(pool.bid_to_pool[0][0], p);
  EXPECT_EQ(pool.bid_to_pool[1][0], p);
  EXPECT_NE(pool.bid_to_pool[1][1], p);
  EXPECT_EQ(pool.indexNear(Eigen::Vector3d(29.0, 0.0, 0.0), 2.0),
            pool.indexOf(101));
  EXPECT_EQ(pool.indexNear(Eigen::Vector3d(50.0, 0.0, 0.0), 2.0), -1);
}

TEST(ClusterPool, DropsClustersExploredByAnyRobot) {
  TourBidData a =
      bidAt(1, 0.0, {cluster(100, 1, 10.0), cluster(300, 3, -10.0)});
  TourBidData b = bidAt(2, 20.0, {cluster(201, 2, 25.0)});
  b.explored = {100};  // robot 2's roadmap shows robot 1's cluster explored
  // The auctioneer's own roadmap shows everything behind it explored.
  const auto explored_here = [](const Eigen::Vector3d& p) {
    return p.x() < 0.0;
  };
  const ClusterPool pool =
      mgg::buildClusterPool({a, b}, {}, 2.0, explored_here);
  ASSERT_EQ(pool.clusters.size(), 1u);
  EXPECT_EQ(pool.clusters[0].id, 201u);
  EXPECT_EQ(pool.dropped_explored.size(), 2u);
  EXPECT_EQ(pool.bid_to_pool[0], (std::vector<int>{-1, -1}));
  EXPECT_EQ(pool.bid_to_pool[1], (std::vector<int>{0}));
}

TEST(ClusterPool, HoldsClaimedClustersNoBidNames) {
  const ClusterPool pool = mgg::buildClusterPool(
      {bidAt(1, 0.0, {cluster(100, 1, 10.0)})}, {cluster(400, 4, 40.0)}, 2.0,
      nullptr);
  EXPECT_EQ(pool.clusters.size(), 2u);
  EXPECT_GE(pool.indexOf(400), 0);
  EXPECT_EQ(pool.bid_to_pool.size(), 1u);
}

TEST(ClusterPool, BidderCostsComeFromTheBidOrTheEstimate) {
  const TourBidData a = bidAt(1, 0.0, {cluster(100, 1, 10.0)});
  const TourBidData b = bidAt(2, 20.0, {cluster(201, 2, 25.0)});
  const ClusterPool pool = mgg::buildClusterPool({a, b}, {}, 2.0, nullptr);
  const int p100 = pool.indexOf(100);
  const int p201 = pool.indexOf(201);
  const auto doubled = [](const Eigen::Vector3d& from,
                          const Eigen::Vector3d& to) {
    return 2.0 * (from - to).norm();
  };
  const AuctionBidder bidder =
      mgg::bidderCosts(a, pool.bid_to_pool[0], pool, doubled);
  EXPECT_DOUBLE_EQ(bidder.from_pose[p100], 10.0);  // from its bid
  EXPECT_DOUBLE_EQ(bidder.from_pose[p201], 50.0);  // estimated
  EXPECT_DOUBLE_EQ(bidder.between[p100][p201], 30.0);
  EXPECT_DOUBLE_EQ(bidder.between[p201][p100], 30.0);
  EXPECT_DOUBLE_EQ(bidder.between[p100][p100], 0.0);
  EXPECT_EQ(bidder.current_target, -1);
  // Without an estimate, what the bid does not give is unreachable.
  EXPECT_EQ(mgg::bidderCosts(a, pool.bid_to_pool[0], pool, nullptr)
                .from_pose[p201],
            mgg::kUnreachableCost);
  TourBidData targeting = a;
  targeting.current_target = 100;
  EXPECT_EQ(mgg::bidderCosts(targeting, pool.bid_to_pool[0], pool, doubled)
                .current_target,
            p100);
}

TEST(SequentialAuction, MatchesTheBruteForceOptimumOnLines) {
  {
    const std::vector<double> xs{1.0, 2.0, 3.0, 7.0, 8.0, 9.0};
    const std::vector<AuctionBidder> bidders{onLine(1, 0.0, xs),
                                             onLine(2, 10.0, xs)};
    const AuctionResult result = mgg::runSequentialAuction(
        bidders, std::vector<bool>(xs.size(), false), 0.2, 0.0);
    EXPECT_NEAR(auctionCost(bidders, result),
                bruteForceMinSum(bidders, xs.size()), 1e-9);
    EXPECT_NEAR(auctionCost(bidders, result), 6.0, 1e-9);
    EXPECT_TRUE(result.unassigned.empty());
  }
  {
    const std::vector<double> xs{2.0, 4.0, 12.0, 14.0, 18.0};
    const std::vector<AuctionBidder> bidders{
        onLine(1, 0.0, xs), onLine(2, 10.0, xs), onLine(3, 20.0, xs)};
    const AuctionResult result = mgg::runSequentialAuction(
        bidders, std::vector<bool>(xs.size(), false), 0.2, 0.0);
    EXPECT_NEAR(auctionCost(bidders, result),
                bruteForceMinSum(bidders, xs.size()), 1e-9);
    EXPECT_NEAR(auctionCost(bidders, result), 10.0, 1e-9);
  }
}

TEST(SequentialAuction, TwoCorridorsGoOnePerRobot) {
  // Both robots where two corridors meet, five clusters down each.
  std::vector<CorridorPoint> points;
  for (const double along : {2.0, 4.0, 6.0, 8.0, 10.0}) points.push_back({'A', along});
  for (const double along : {2.0, 4.0, 6.0, 8.0, 10.0}) points.push_back({'B', along});
  const std::vector<AuctionBidder> bidders{atJunction(1, points),
                                           atJunction(2, points)};
  const AuctionResult result = mgg::runSequentialAuction(
      bidders, std::vector<bool>(points.size(), false), 0.2, 0.3);
  const std::vector<int>& one = result.bundles.at(1);
  const std::vector<int>& two = result.bundles.at(2);
  ASSERT_EQ(one.size(), 5u);
  ASSERT_EQ(two.size(), 5u);
  for (const int i : one) EXPECT_EQ(points[i].corridor, points[one.front()].corridor);
  for (const int i : two) EXPECT_EQ(points[i].corridor, points[two.front()].corridor);
  EXPECT_NE(points[one.front()].corridor, points[two.front()].corridor);
  // Each tours its corridor outward.
  EXPECT_DOUBLE_EQ(points[one.front()].along, 2.0);
  EXPECT_DOUBLE_EQ(points[one.back()].along, 10.0);
}

TEST(SequentialAuction, TheBalancePenaltyHandsAnEqualClusterToTheLighterBundle) {
  // Robot 1 keeps a target 10 m away; cluster 1 lies 3 m past it. Robot 2
  // is 3 m from cluster 1. Equal marginal costs: the tie goes to robot 1
  // without the balance penalty, to robot 2 with it.
  AuctionBidder heavy;
  heavy.robot_id = 1;
  heavy.from_pose = {10.0, 13.0};
  heavy.between = {{0.0, 3.0}, {3.0, 0.0}};
  heavy.current_target = 0;
  AuctionBidder light;
  light.robot_id = 2;
  light.from_pose = {30.0, 3.0};
  light.between = {{0.0, 3.0}, {3.0, 0.0}};
  const AuctionResult none =
      mgg::runSequentialAuction({heavy, light}, {false, false}, 0.2, 0.0);
  EXPECT_EQ(none.bundles.at(1), (std::vector<int>{0, 1}));
  EXPECT_TRUE(none.bundles.at(2).empty());
  const AuctionResult balanced =
      mgg::runSequentialAuction({heavy, light}, {false, false}, 0.2, 0.3);
  EXPECT_EQ(balanced.bundles.at(1), std::vector<int>{0});
  EXPECT_EQ(balanced.bundles.at(2), std::vector<int>{1});
}

TEST(SequentialAuction, CommitmentKeepsATargetUnlessAnotherRobotIsMuchCloser) {
  AuctionBidder holder;
  holder.robot_id = 1;
  holder.from_pose = {5.0};
  holder.between = {{0.0}};
  holder.current_target = 0;
  AuctionBidder rival;
  rival.robot_id = 2;
  rival.from_pose = {4.5};  // 10 % closer: within the 20 % margin
  rival.between = {{0.0}};
  const AuctionResult kept =
      mgg::runSequentialAuction({holder, rival}, {false}, 0.2, 0.0);
  EXPECT_EQ(kept.bundles.at(1), std::vector<int>{0});
  EXPECT_TRUE(kept.bundles.at(2).empty());
  rival.from_pose = {3.9};  // 22 % closer
  const AuctionResult lost =
      mgg::runSequentialAuction({holder, rival}, {false}, 0.2, 0.0);
  EXPECT_TRUE(lost.bundles.at(1).empty());
  EXPECT_EQ(lost.bundles.at(2), std::vector<int>{0});
}

TEST(SequentialAuction, AContestedTargetStaysWithTheCloserRobot) {
  // After a reconnection both robots were heading for the same cluster.
  AuctionBidder a;
  a.robot_id = 1;
  a.from_pose = {6.0};
  a.between = {{0.0}};
  a.current_target = 0;
  AuctionBidder b = a;
  b.robot_id = 2;
  b.from_pose = {5.0};
  const AuctionResult result =
      mgg::runSequentialAuction({a, b}, {false}, 0.2, 0.0);
  EXPECT_TRUE(result.bundles.at(1).empty());
  EXPECT_EQ(result.bundles.at(2), std::vector<int>{0});
}

TEST(SequentialAuction, FixedAndUnreachableClustersAreNotAwarded) {
  AuctionBidder b = onLine(1, 0.0, {1.0, 2.0, 3.0});
  b.from_pose[2] = mgg::kUnreachableCost;
  for (int i = 0; i < 3; ++i) {
    if (i != 2) {
      b.between[i][2] = mgg::kUnreachableCost;
      b.between[2][i] = mgg::kUnreachableCost;
    }
  }
  const AuctionResult result =
      mgg::runSequentialAuction({b}, {true, false, false}, 0.2, 0.3);
  EXPECT_EQ(result.bundles.at(1), std::vector<int>{1});
  EXPECT_EQ(result.unassigned, std::vector<int>{2});
}

}  // namespace
