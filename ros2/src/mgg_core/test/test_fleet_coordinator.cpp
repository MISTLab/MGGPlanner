// Tests for the fleet coordinator (tour-exploration design §3, §4): the
// auctioneer election, one-round auctions over an in-memory radio, late
// bids and missed awards, explored clusters, merging groups, silent claims
// and their release. Time is simulated.

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/fleet_coordinator.h"

namespace {

using mgg::ClusterId;
using mgg::FleetCluster;
using mgg::FleetCoordinator;
using mgg::FleetParams;
using mgg::TourAwardData;
using mgg::TourBidData;

FleetCluster cluster(ClusterId id, int owner, double x, double y = 0.0) {
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

std::vector<ClusterId> idsOf(const std::vector<FleetCluster>& clusters) {
  std::vector<ClusterId> ids;
  for (const FleetCluster& c : clusters) ids.push_back(c.id);
  std::sort(ids.begin(), ids.end());
  return ids;
}

/// A robot standing still at (x, y) that knows `known`, with straight-line
/// costs, and bids its bundle's first cluster as its current target.
struct SimRobot {
  SimRobot(int robot_id, double x, double y, const FleetParams& params)
      : id(robot_id),
        position(x, y, 0.0),
        coordinator(std::make_unique<FleetCoordinator>(robot_id, params, 0.2)) {}

  TourBidData ownBid() const {
    TourBidData bid;
    bid.pose = mgg::StateVec(position.x(), position.y(), 0.0, 0.0);
    bid.clusters = known;
    for (const FleetCluster& c : known) {
      bid.costs_from_pose.push_back(euclid(position, c.position));
    }
    for (const FleetCluster& a : known) {
      for (const FleetCluster& b : known) {
        bid.costs_between.push_back(euclid(a.position, b.position));
      }
    }
    bid.current_target = front();
    bid.explored = explored;
    return bid;
  }
  ClusterId front() const {
    const auto& bundle = coordinator->bundle();
    return bundle.empty() ? mgg::kNoCluster : bundle.front().id;
  }

  int id;
  Eigen::Vector3d position;
  std::vector<FleetCluster> known;
  std::vector<ClusterId> explored;
  std::unique_ptr<FleetCoordinator> coordinator;
};

/// Ticks every robot and delivers what each sends to every robot that hears
/// it, all in one frame. `deaf` holds (from, to) pairs that do not get
/// through; results (not calls) are logged with the time they were sent.
struct Radio {
  explicit Radio(std::vector<SimRobot*> members) : robots(std::move(members)) {}

  std::vector<SimRobot*> robots;
  std::set<std::pair<int, int>> deaf;
  std::vector<std::pair<double, TourAwardData>> awards;

  void cut(int a, int b) {
    deaf.insert({a, b});
    deaf.insert({b, a});
  }
  void restore(int a, int b) {
    deaf.erase({a, b});
    deaf.erase({b, a});
  }
  void step(double now) {
    std::vector<std::pair<int, mgg::FleetTickOutput>> sent;
    for (SimRobot* r : robots) {
      sent.emplace_back(r->id,
                        r->coordinator->tick(
                            now, [r] { return r->ownBid(); }, euclid, nullptr));
    }
    for (const auto& [from, out] : sent) {
      if (out.award && !out.award->call) awards.emplace_back(now, *out.award);
      for (SimRobot* r : robots) {
        if (r->id == from || deaf.count({from, r->id}) > 0) continue;
        if (out.bid) r->coordinator->onBid(*out.bid, now);
        if (out.award) r->coordinator->onAward(*out.award, now);
      }
    }
  }
  void runUntil(double& now, double until, double dt = 0.1) {
    for (; now < until - 1e-9; now += dt) step(now);
  }
};

TEST(FleetCoordinator, TheLowestIdIsTheAuctioneerAndHandsOverWhenItFallsSilent) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 10.0, 0.0, params),
      r3(3, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 12.0)};
  r3.known = {cluster(31, 3, 22.0)};
  Radio radio{{&r1, &r2, &r3}};
  double now = 0.0;
  radio.runUntil(now, 8.0);
  for (const SimRobot* r : radio.robots) {
    EXPECT_EQ(r->coordinator->auctioneer(now), 1);
  }
  ASSERT_FALSE(radio.awards.empty());
  for (const auto& [sent, award] : radio.awards) EXPECT_EQ(award.auctioneer_id, 1);

  // Robot 1 falls silent: after fleet.peer_timeout_s the next lowest takes
  // over at the next auction.
  radio.cut(1, 2);
  radio.cut(1, 3);
  const std::size_t before = radio.awards.size();
  radio.runUntil(now, 20.0);
  EXPECT_EQ(r2.coordinator->auctioneer(now), 2);
  EXPECT_EQ(r3.coordinator->auctioneer(now), 2);
  EXPECT_EQ(r1.coordinator->group(now), std::vector<int>{1});
  bool from_two = false;
  for (std::size_t i = before; i < radio.awards.size(); ++i) {
    EXPECT_NE(radio.awards[i].second.auctioneer_id, 3);
    from_two |= radio.awards[i].second.auctioneer_id == 2;
  }
  EXPECT_TRUE(from_two);
}

TEST(FleetCoordinator, AnAwardSplitsTheClustersAndEveryRobotAppliesItsBundle) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  // More than fleet.cluster_merge_radius_m apart: four clusters.
  r1.known = {cluster(11, 1, 2.0), cluster(12, 1, 6.0)};
  r2.known = {cluster(21, 2, 18.0), cluster(22, 2, 14.0)};
  Radio radio{{&r1, &r2}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  EXPECT_EQ(idsOf(r1.coordinator->bundle()), (std::vector<ClusterId>{11, 12}));
  EXPECT_EQ(idsOf(r2.coordinator->bundle()), (std::vector<ClusterId>{21, 22}));
  ASSERT_FALSE(radio.awards.empty());
  std::set<ClusterId> named;
  for (const mgg::RobotBundle& bundle : radio.awards.back().second.bundles) {
    for (const ClusterId id : bundle.clusters) EXPECT_TRUE(named.insert(id).second);
  }
  // Each knows what the other holds.
  EXPECT_EQ(idsOf(r1.coordinator->claimedByOthers(now)),
            (std::vector<ClusterId>{21, 22}));
}

TEST(FleetCoordinator, ALateBidKeepsItsPreviousBundle) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 18.0)};
  Radio radio{{&r1, &r2}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  ASSERT_EQ(r2.front(), 21u);

  // Robot 2's bids stop reaching robot 1 for three seconds (it stays in the
  // group: fleet.peer_timeout_s is 5 s), while a new cluster appears.
  radio.deaf.insert({2, 1});
  r1.known.push_back(cluster(13, 1, 6.0));
  const std::size_t first = radio.awards.size();
  radio.runUntil(now, 8.0);
  ASSERT_GT(radio.awards.size(), first);
  const TourAwardData& award = radio.awards.back().second;
  const mgg::RobotBundle* held = award.bundleOf(2);
  ASSERT_NE(held, nullptr);
  EXPECT_GT(held->silent_s, 0.0);
  EXPECT_EQ(held->clusters, std::vector<ClusterId>{21});
  EXPECT_EQ(r2.front(), 21u);
  EXPECT_EQ(idsOf(r1.coordinator->bundle()), (std::vector<ClusterId>{11, 13}));
}

TEST(FleetCoordinator, AMissedAwardKeepsThePreviousBundle) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 18.0)};
  Radio radio{{&r1, &r2}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  const std::vector<ClusterId> before = idsOf(r2.coordinator->bundle());
  ASSERT_EQ(before, std::vector<ClusterId>{21});

  // Robot 2 hears neither calls nor awards for three seconds, while its own
  // new cluster makes robot 1 auction.
  radio.deaf.insert({1, 2});
  r2.known.push_back(cluster(22, 2, 14.0));
  const std::size_t first = radio.awards.size();
  radio.runUntil(now, 8.0);
  ASSERT_GT(radio.awards.size(), first);
  EXPECT_EQ(idsOf(r2.coordinator->bundle()), before);

  // Hearing again, it follows the next award.
  radio.deaf.erase({1, 2});
  r1.known.push_back(cluster(14, 1, 5.0));
  const std::size_t second = radio.awards.size();
  radio.runUntil(now, 14.0);
  ASSERT_GT(radio.awards.size(), second);
  const mgg::RobotBundle* latest = radio.awards.back().second.bundleOf(2);
  ASSERT_NE(latest, nullptr);
  std::vector<ClusterId> expected = latest->clusters;
  std::sort(expected.begin(), expected.end());
  EXPECT_EQ(idsOf(r2.coordinator->bundle()), expected);
}

TEST(FleetCoordinator, ClustersInAPeersExploredSpaceAreDropped) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0), cluster(12, 1, 10.0)};
  r2.known = {cluster(21, 2, 18.0)};
  r2.explored = {12};  // robot 2's roadmap shows it explored
  Radio radio{{&r1, &r2}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  ASSERT_FALSE(radio.awards.empty());
  const TourAwardData& award = radio.awards.back().second;
  for (const mgg::RobotBundle& bundle : award.bundles) {
    for (const ClusterId id : bundle.clusters) EXPECT_NE(id, 12u);
  }
  EXPECT_EQ(idsOf(award.explored), std::vector<ClusterId>{12});
  EXPECT_EQ(idsOf(r1.coordinator->exploredElsewhere()),
            std::vector<ClusterId>{12});
  EXPECT_EQ(idsOf(r1.coordinator->bundle()), std::vector<ClusterId>{11});
}

TEST(FleetCoordinator, AClusterReportedExploredStaysOutOfLaterAuctions) {
  // Robot 2 explored ground robot 1 marked a frontier and reported it once.
  // Robot 1's own map does not show robot 2's ground, so robot 1 keeps
  // marking the frontier and bidding it.
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0), cluster(22, 1, 14.0)};
  r2.known = {cluster(21, 2, 18.0)};
  r2.explored = {22};
  Radio radio{{&r1, &r2}};
  double now = 0.0;
  radio.runUntil(now, 3.0);
  r2.explored.clear();
  r1.known.push_back(cluster(13, 1, 6.0));  // another auction
  const std::size_t first = radio.awards.size();
  radio.runUntil(now, 8.0);
  ASSERT_GT(radio.awards.size(), first);
  for (std::size_t i = first; i < radio.awards.size(); ++i) {
    for (const mgg::RobotBundle& bundle : radio.awards[i].second.bundles) {
      for (const ClusterId id : bundle.clusters) EXPECT_NE(id, 22u);
    }
  }
}

TEST(FleetCoordinator, TwoGroupsMergingSettleWithoutNeedlessTargetChanges) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 5.0, 0.0, params),
      r3(3, 30.0, 0.0, params), r4(4, 35.0, 0.0, params);
  r1.known = {cluster(11, 1, -2.0)};
  r2.known = {cluster(21, 2, 7.0)};
  r3.known = {cluster(31, 3, 28.0)};
  r4.known = {cluster(41, 4, 37.0)};
  Radio radio{{&r1, &r2, &r3, &r4}};
  for (const int a : {1, 2}) {
    for (const int b : {3, 4}) radio.cut(a, b);
  }
  double now = 0.0;
  radio.runUntil(now, 8.0);
  EXPECT_EQ(r3.coordinator->auctioneer(now), 3);
  EXPECT_EQ(r1.coordinator->auctioneer(now), 1);
  std::map<int, ClusterId> targets;
  for (const SimRobot* r : radio.robots) {
    targets[r->id] = r->front();
    ASSERT_NE(r->front(), mgg::kNoCluster) << "robot " << r->id;
  }

  // The groups come into contact: robot 1, the lowest ID of the merged
  // group, runs the auctions; robot 3 stops as soon as it hears robot 1.
  for (const int a : {1, 2}) {
    for (const int b : {3, 4}) radio.restore(a, b);
  }
  const double merged_at = now;
  const std::size_t first = radio.awards.size();
  radio.runUntil(now, 16.0);
  bool everyone = false;
  for (std::size_t i = first; i < radio.awards.size(); ++i) {
    const auto& [sent, award] = radio.awards[i];
    if (sent >= merged_at + params.auction_interval_s) {
      EXPECT_EQ(award.auctioneer_id, 1);
    }
    everyone |= award.auctioneer_id == 1 && award.bundleOf(3) != nullptr &&
                award.bundleOf(4) != nullptr && award.bundleOf(2) != nullptr;
  }
  EXPECT_TRUE(everyone);
  for (const SimRobot* r : radio.robots) {
    EXPECT_EQ(r->coordinator->auctioneer(now), 1);
    EXPECT_EQ(r->front(), targets[r->id]) << "robot " << r->id;
  }
}

TEST(FleetCoordinator, ASilentPeersClaimsPersistUntilTheTtl) {
  FleetParams params;
  params.claim_ttl_s = 600.0;  // SwarmDeck's SubT simulation value
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 18.0)};
  Radio radio{{&r1, &r2}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  ASSERT_EQ(r2.front(), 21u);

  radio.cut(1, 2);
  const double silent_from = now;
  radio.runUntil(now, 300.0, 1.0);
  // Long out of the group, robot 2 still holds its cluster.
  EXPECT_EQ(r1.coordinator->group(now), std::vector<int>{1});
  EXPECT_EQ(idsOf(r1.coordinator->claimedByOthers(now)),
            std::vector<ClusterId>{21});
  radio.runUntil(now, silent_from + params.claim_ttl_s + 5.0, 1.0);
  EXPECT_TRUE(r1.coordinator->claimedByOthers(now).empty());
}

TEST(FleetCoordinator, IdleRobotsTakeOverTheLongestSilentClaimFirst) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 10.0, 0.0, params),
      r3(3, 20.0, 0.0, params), r4(4, 30.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 12.0)};
  r3.known = {cluster(31, 3, 22.0)};
  r4.known = {cluster(41, 4, 32.0)};
  Radio radio{{&r1, &r2, &r3, &r4}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  ASSERT_EQ(r4.front(), 41u);
  // Robot 4 falls silent first, robot 3 two seconds later.
  for (const int other : {1, 2, 3}) radio.cut(4, other);
  radio.runUntil(now, 7.0);
  for (const int other : {1, 2}) radio.cut(3, other);
  radio.runUntil(now, 15.0);

  // Robots 1 and 2 finish their bundles with nothing left to explore.
  r1.known.clear();
  r2.known.clear();
  r1.coordinator->requestAuction();
  r2.coordinator->requestAuction();
  const std::size_t first = radio.awards.size();
  radio.runUntil(now, 22.0);
  ASSERT_GT(radio.awards.size(), first);
  bool released_four = false;
  for (std::size_t i = first; i < radio.awards.size(); ++i) {
    const auto& ids = radio.awards[i].second.released_robot_ids;
    released_four |= std::find(ids.begin(), ids.end(), 4) != ids.end();
  }
  EXPECT_TRUE(released_four);
  EXPECT_TRUE(r1.front() == 41u || r2.front() == 41u);
  // Robot 3's claim, younger, stays.
  const std::vector<ClusterId> held = idsOf(r1.coordinator->claimedByOthers(now));
  EXPECT_TRUE(std::find(held.begin(), held.end(), 31u) != held.end());
}

TEST(FleetCoordinator, ARobotAloneTakesOverTheLongestSilentClaimFirst) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 10.0, 0.0, params),
      r3(3, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 12.0)};
  r3.known = {cluster(31, 3, 22.0)};
  Radio radio{{&r1, &r2, &r3}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  radio.cut(3, 1);
  radio.cut(3, 2);
  radio.runUntil(now, 7.0);
  radio.cut(2, 1);
  radio.runUntil(now, 20.0);
  ASSERT_EQ(r1.coordinator->group(now), std::vector<int>{1});
  EXPECT_EQ(r1.coordinator->takeOverOldestClaim(now), 3);
  EXPECT_EQ(r1.coordinator->takeOverOldestClaim(now), 2);
  EXPECT_EQ(r1.coordinator->takeOverOldestClaim(now), -1);
  EXPECT_TRUE(r1.coordinator->claimedByOthers(now).empty());
}

TEST(FleetCoordinator, WithoutATransformARobotIsAloneAndJoinsWhenOneAppears) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 18.0)};
  Radio radio{{&r1, &r2}};
  // The ROS layer drops messages from a robot it holds no transform to.
  radio.cut(1, 2);
  double now = 0.0;
  radio.runUntil(now, 5.0);
  EXPECT_EQ(r1.coordinator->group(now), std::vector<int>{1});
  EXPECT_FALSE(r1.coordinator->hasAward());
  EXPECT_TRUE(r1.coordinator->bundle().empty());
  EXPECT_TRUE(radio.awards.empty());

  radio.restore(1, 2);  // a transform appears
  radio.runUntil(now, 10.0);
  EXPECT_EQ(r1.coordinator->group(now), (std::vector<int>{1, 2}));
  EXPECT_TRUE(r1.coordinator->hasAward());
  ASSERT_FALSE(radio.awards.empty());
  const mgg::RobotBundle* theirs = radio.awards.back().second.bundleOf(2);
  ASSERT_NE(theirs, nullptr);
  EXPECT_EQ(theirs->clusters, std::vector<ClusterId>{21});
}

TEST(FleetCoordinator, AnAuctioneerThatRestartsStillHasItsAwardsApplied) {
  // Review Focus 5: a restarted planner numbers its auctions from 1 again.
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 18.0)};
  Radio radio{{&r1, &r2}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  ASSERT_FALSE(radio.awards.empty());

  r1.coordinator = std::make_unique<FleetCoordinator>(1, params, 0.2);
  r1.known.push_back(cluster(13, 1, 6.0));
  const std::size_t first = radio.awards.size();
  radio.runUntil(now, 10.0);
  ASSERT_GT(radio.awards.size(), first);
  EXPECT_EQ(radio.awards[first].second.auction_id,
            radio.awards.front().second.auction_id);
  const std::vector<ClusterId> held = idsOf(r2.coordinator->claimedByOthers(now));
  EXPECT_TRUE(std::find(held.begin(), held.end(), 13u) != held.end());
}

TEST(FleetCoordinator, AnOperatorReleaseIsForwardedInTheNextAward) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 10.0, 0.0, params),
      r3(3, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 12.0)};
  r3.known = {cluster(31, 3, 22.0)};
  Radio radio{{&r1, &r2, &r3}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  radio.cut(3, 1);
  radio.cut(3, 2);
  radio.runUntil(now, 15.0);
  ASSERT_FALSE(r2.coordinator->claimedByOthers(now).empty());

  // Only the auctioneer answers the operator.
  EXPECT_FALSE(r2.coordinator->releaseClaims(3, now));
  EXPECT_TRUE(r1.coordinator->releaseClaims(3, now));
  const std::size_t first = radio.awards.size();
  radio.runUntil(now, 20.0);
  bool forwarded = false;
  for (std::size_t i = first; i < radio.awards.size(); ++i) {
    const auto& ids = radio.awards[i].second.released_robot_ids;
    forwarded |= std::find(ids.begin(), ids.end(), 3) != ids.end();
  }
  EXPECT_TRUE(forwarded);
  for (const FleetCluster& c : r2.coordinator->claimedByOthers(now)) {
    EXPECT_NE(c.id, 31u);
  }
  for (const FleetCluster& c : r1.coordinator->claimedByOthers(now)) {
    EXPECT_NE(c.id, 31u);
  }
}

}  // namespace
