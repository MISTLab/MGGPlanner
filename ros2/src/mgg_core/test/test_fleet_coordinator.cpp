// Tests for the fleet coordinator (tour-exploration design §3, §4): the
// auctioneer election, one-round auctions over an in-memory radio, late
// bids and missed awards, explored clusters, merging groups, silent claims
// and their release. Time is simulated.

#include <algorithm>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <tuple>
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

TourAwardData makeCall(int auctioneer, std::uint64_t auction_id,
                       double stamp_s) {
  TourAwardData call;
  call.auction_id = auction_id;
  call.auctioneer_id = auctioneer;
  call.stamp_s = stamp_s;
  call.call = true;
  return call;
}

TourAwardData makeAward(int auctioneer, std::uint64_t auction_id,
                        double stamp_s, std::vector<mgg::RobotBundle> bundles,
                        std::vector<FleetCluster> clusters) {
  TourAwardData award;
  award.auction_id = auction_id;
  award.auctioneer_id = auctioneer;
  award.stamp_s = stamp_s;
  award.bundles = std::move(bundles);
  award.clusters = std::move(clusters);
  return award;
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
    bid.robot_id = id;
    bid.auctioneer_id = id;  // tick fills in the current election
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
  std::vector<std::pair<double, TourAwardData>> calls;
  std::vector<std::pair<double, TourBidData>> bids;

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
      if (out.award) {
        EXPECT_TRUE(out.award->wellFormed()) << "from robot " << from;
      }
      if (out.award && !out.award->call) awards.emplace_back(now, *out.award);
      if (out.award && out.award->call) calls.emplace_back(now, *out.award);
      if (out.bid) bids.emplace_back(now, *out.bid);
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


bool holds(const std::vector<int>& robots, int robot_id) {
  return std::find(robots.begin(), robots.end(), robot_id) != robots.end();
}

// A clock reset (a simulation restart) moves now_s backwards. The last time
// a silent robot was heard, from before the reset, must neither keep its
// claim alive nor keep it in the group while the auctioneer's awards keep
// naming its claim.
TEST(FleetCoordinator, AfterAClockRollbackASilentRobotsClaimAgesFromTheReset) {
  FleetParams params;
  params.claim_ttl_s = 600.0;
  FleetCoordinator coordinator(2, params, 0.2);
  // Robot 1 is the auctioneer; robot 3 holds cluster 31 and is silent for
  // `silent_s` as robot 1 knows it.
  const auto award = [](std::uint64_t auction_id, double stamp_s,
                        double silent_s) {
    TourAwardData a;
    a.auction_id = auction_id;
    a.auctioneer_id = 1;
    a.stamp_s = stamp_s;
    a.clusters = {cluster(31, 3, 22.0)};
    a.bundles = {mgg::RobotBundle{2, {}, 0.0},
                 mgg::RobotBundle{3, {31}, silent_s}};
    return a;
  };
  TourBidData bid;
  bid.robot_id = 3;
  bid.auctioneer_id = 3;
  coordinator.onBid(bid, 10000.0);
  std::uint64_t auction_id = 1;
  coordinator.onAward(award(auction_id++, 10000.0, 0.0), 10000.0);
  ASSERT_EQ(coordinator.group(10000.0), (std::vector<int>{1, 2, 3}));
  ASSERT_EQ(idsOf(coordinator.claimedByOthers(10000.0)),
            std::vector<ClusterId>{31});

  // The clocks reset to 10 s and robot 3 is not heard again.
  const double reset = 10.0;
  for (double now = reset; now <= reset + params.claim_ttl_s; now += 2.0) {
    coordinator.onAward(award(auction_id++, now, now - reset), now);
    coordinator.tick(now, nullptr, nullptr, nullptr);
    EXPECT_EQ(holds(coordinator.group(now), 3),
              now - reset <= params.peer_timeout_s)
        << "at " << now;
    EXPECT_EQ(idsOf(coordinator.claimedByOthers(now)),
              std::vector<ClusterId>{31})
        << "at " << now;
  }
  const double expired = reset + params.claim_ttl_s + 2.0;
  coordinator.onAward(award(auction_id++, expired, expired - reset), expired);
  coordinator.tick(expired, nullptr, nullptr, nullptr);
  EXPECT_TRUE(coordinator.claimedByOthers(expired).empty());
  EXPECT_EQ(coordinator.group(expired), (std::vector<int>{1, 2}));
}


// The coordinator's own timers must not wait for the old clock either: after
// a reset, an auction being collected is awarded within one bid deadline,
// periodic bids resume within one auction interval, and auctions are called
// again.
TEST(FleetCoordinator, AfterAClockRollbackBidsAndAuctionsResume) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 18.0)};
  Radio radio{{&r1, &r2}};
  double now = 10000.0;
  radio.runUntil(now, 10005.0);
  ASSERT_FALSE(radio.awards.empty());

  // A new cluster makes robot 1 call an auction; the clocks reset to 10 s
  // while it collects the bids.
  r1.known.push_back(cluster(13, 1, 6.0));
  const std::size_t calls_before = radio.calls.size();
  for (; radio.calls.size() == calls_before && now < 10010.0; now += 0.1) {
    radio.step(now);
  }
  ASSERT_GT(radio.calls.size(), calls_before);
  const double reset = 10.0;
  now = reset;
  const std::size_t awards_before = radio.awards.size();
  const std::size_t bids_before = radio.bids.size();
  const std::size_t calls_at_reset = radio.calls.size();

  radio.runUntil(now, reset + params.auction_interval_s + 0.15);
  ASSERT_GT(radio.awards.size(), awards_before);
  EXPECT_LE(radio.awards[awards_before].first,
            reset + params.bid_deadline_s + 0.15);
  EXPECT_EQ(idsOf(r1.coordinator->bundle()), (std::vector<ClusterId>{11, 13}));
  std::set<int> bidders;
  for (std::size_t i = bids_before; i < radio.bids.size(); ++i) {
    bidders.insert(radio.bids[i].second.robot_id);
  }
  EXPECT_EQ(bidders, (std::set<int>{1, 2}));

  // A new cluster of robot 2's is auctioned in the new clock epoch.
  r2.known.push_back(cluster(22, 2, 14.0));
  radio.runUntil(now, reset + 4.0 * params.auction_interval_s);
  EXPECT_GT(radio.calls.size(), calls_at_reset);
  EXPECT_EQ(idsOf(r2.coordinator->bundle()), (std::vector<ClusterId>{21, 22}));
}


// Peers refuse a bid whose bundle or explored list is longer than
// kMaxBidClusters, so this robot's own bid never carries more: the bundle
// keeps its first clusters (visited next), and a longer explored list goes
// out in turns so that no report is left out for good.
TEST(FleetCoordinator, OwnBidsStayWithinTheBidClusterLimit) {
  const FleetParams params;
  FleetCoordinator coordinator(2, params, 0.2);
  const std::size_t n = mgg::kMaxBidClusters + 1;
  TourAwardData award;
  award.auction_id = 1;
  award.auctioneer_id = 1;
  award.stamp_s = 0.0;
  mgg::RobotBundle mine{2, {}, 0.0};
  for (std::size_t i = 0; i < n; ++i) {
    const ClusterId id = static_cast<ClusterId>(100 + i);
    award.clusters.push_back(cluster(id, 1, static_cast<double>(i)));
    mine.clusters.push_back(id);
  }
  award.bundles = {mine};
  coordinator.onAward(award, 0.0);
  ASSERT_EQ(coordinator.bundle().size(), n);

  std::vector<ClusterId> explored;
  for (std::size_t i = 0; i < n; ++i) {
    explored.push_back(static_cast<ClusterId>(5000 + i));
  }
  const auto own_bid = [&explored] {
    TourBidData bid;
    bid.robot_id = 2;
    bid.auctioneer_id = 1;
    bid.explored = explored;
    return bid;
  };
  std::vector<TourBidData> bids;
  for (double now = 0.0; now < 2.0 * params.auction_interval_s + 0.05;
       now += 0.1) {
    const mgg::FleetTickOutput out =
        coordinator.tick(now, own_bid, nullptr, nullptr);
    if (out.bid) bids.push_back(*out.bid);
  }
  ASSERT_GE(bids.size(), 2u);
  std::set<ClusterId> reported;
  for (const TourBidData& bid : bids) {
    EXPECT_TRUE(bid.wellFormed());
    ASSERT_EQ(bid.bundle.size(), mgg::kMaxBidClusters);
    EXPECT_TRUE(std::equal(bid.bundle.begin(), bid.bundle.end(),
                           mine.clusters.begin()));
    EXPECT_EQ(bid.explored.size(), mgg::kMaxBidClusters);
    reported.insert(bid.explored.begin(), bid.explored.end());
  }
  EXPECT_EQ(reported.size(), n);

  // At the limit nothing is cut.
  explored.pop_back();
  const mgg::FleetTickOutput out =
      coordinator.tick(10.0, own_bid, nullptr, nullptr);
  ASSERT_TRUE(out.bid.has_value());
  EXPECT_EQ(out.bid->explored, explored);
}

// An award naming kNoCluster (a broken peer) must not put it in this robot's
// bundle, claims or award clusters, whence its next bid would carry it and
// be refused.
TEST(FleetCoordinator, AnAwardsUnnamedClustersAreIgnored) {
  const FleetParams params;
  FleetCoordinator coordinator(2, params, 0.2);
  TourAwardData award;
  award.auction_id = 1;
  award.auctioneer_id = 1;
  award.stamp_s = 0.0;
  award.clusters = {cluster(mgg::kNoCluster, 1, 1.0), cluster(21, 2, 2.0),
                    cluster(31, 3, 3.0)};
  award.bundles = {mgg::RobotBundle{2, {mgg::kNoCluster, 21}, 0.0},
                   mgg::RobotBundle{3, {31, mgg::kNoCluster}, 0.0}};
  award.explored = {cluster(mgg::kNoCluster, 1, 40.0), cluster(41, 4, 50.0)};
  coordinator.onAward(award, 0.0);
  EXPECT_EQ(idsOf(coordinator.bundle()), std::vector<ClusterId>{21});
  EXPECT_EQ(idsOf(coordinator.claimedByOthers(0.0)),
            std::vector<ClusterId>{31});
  EXPECT_EQ(idsOf(coordinator.lastAwardClusters()),
            (std::vector<ClusterId>{21, 31}));
  EXPECT_EQ(idsOf(coordinator.exploredElsewhere()),
            std::vector<ClusterId>{41});
  const mgg::FleetTickOutput out =
      coordinator.tick(0.0, nullptr, nullptr, nullptr);
  ASSERT_TRUE(out.bid.has_value());
  EXPECT_TRUE(out.bid->wellFormed());
  EXPECT_EQ(out.bid->bundle, std::vector<ClusterId>{21});
}

// An award is known by its round, (auctioneer, stamp). A round no newer
// than one already applied from the same auctioneer is a replay or arrived
// out of order: applying it would restore a superseded assignment.
TEST(FleetCoordinator, AReplayedOrReorderedAwardIsNotApplied) {
  FleetCoordinator coordinator(2, FleetParams{}, 0.2);
  const std::vector<FleetCluster> known = {
      cluster(21, 2, 2.0), cluster(22, 2, 4.0), cluster(23, 2, 6.0),
      cluster(24, 2, 8.0)};
  const auto award = [&known](std::uint64_t auction_id, double stamp_s,
                              ClusterId id) {
    return makeAward(1, auction_id, stamp_s, {mgg::RobotBundle{2, {id}, 0.0}},
                     known);
  };
  const TourAwardData a = award(1, 1.0, 21);
  coordinator.onAward(a, 1.0);
  coordinator.onAward(award(2, 3.0, 22), 3.0);
  coordinator.onAward(a, 3.5);  // A, B, then A again
  EXPECT_EQ(idsOf(coordinator.bundle()), std::vector<ClusterId>{22});
  // Round 4 overtakes round 3 on the way.
  coordinator.onAward(award(4, 6.0, 23), 6.1);
  coordinator.onAward(award(3, 5.0, 24), 6.2);
  EXPECT_EQ(idsOf(coordinator.bundle()), std::vector<ClusterId>{23});
}

// A restarted auctioneer numbers its auctions from 1 again: a call reusing
// an auction ID already answered, with a new stamp, is a new round.
TEST(FleetCoordinator, ACallReusingAnAuctionIdWithANewStampIsAnswered) {
  FleetCoordinator coordinator(2, FleetParams{}, 0.2);
  TourAwardData call = makeCall(1, 7, 1.0);
  coordinator.onAward(call, 1.0);
  mgg::FleetTickOutput out = coordinator.tick(1.0, nullptr, nullptr, nullptr);
  ASSERT_TRUE(out.bid.has_value());
  EXPECT_EQ(out.bid->auction_id, 7u);

  call.stamp_s = 2.0;
  coordinator.onAward(call, 2.0);
  out = coordinator.tick(2.0, nullptr, nullptr, nullptr);
  ASSERT_TRUE(out.bid.has_value());
  EXPECT_EQ(out.bid->auction_id, 7u);
  // The same round delivered twice is answered once.
  coordinator.onAward(call, 2.5);
  EXPECT_FALSE(coordinator.tick(2.5, nullptr, nullptr, nullptr).bid);
}

/// A well-formed bid from `robot_id` naming nothing: a peer heard.
TourBidData emptyBid(int robot_id) {
  TourBidData bid;
  bid.robot_id = robot_id;
  bid.auctioneer_id = robot_id;
  return bid;
}

// §3.5: the request is answered by an award of the auctioneer asked, even
// when the award of the round the robot first bid in was lost.
TEST(FleetCoordinator, ARequestIsAnsweredByALaterAwardWhenTheFirstIsLost) {
  FleetCoordinator coordinator(2, FleetParams{}, 0.2);
  coordinator.requestAuction();
  coordinator.onAward(makeCall(1, 100, 0.0), 0.0);
  ASSERT_TRUE(coordinator.tick(0.0, nullptr, nullptr, nullptr).bid);
  // Round 100's award is lost.
  coordinator.onAward(makeCall(1, 101, 2.0), 2.0);
  const mgg::FleetTickOutput out =
      coordinator.tick(2.0, nullptr, nullptr, nullptr);
  ASSERT_TRUE(out.bid.has_value());
  EXPECT_TRUE(out.bid->request_auction);
  EXPECT_TRUE(coordinator.awaitingAuction());
  EXPECT_FALSE(coordinator.requestAnswered());
  coordinator.onAward(
      makeAward(1, 101, 3.0, {mgg::RobotBundle{2, {}, 0.0}}, {}), 3.0);
  EXPECT_FALSE(coordinator.awaitingAuction());
  EXPECT_TRUE(coordinator.requestAnswered());
}

// The auctioneer asked falls silent: the request goes to the next one, here
// this robot, whose award answers it.
TEST(FleetCoordinator, ARequestFollowsTheAuctioneerWhenItChanges) {
  const FleetParams params;
  FleetCoordinator coordinator(2, params, 0.2);
  coordinator.onBid(emptyBid(3), 0.0);
  coordinator.requestAuction();
  coordinator.onAward(makeCall(1, 100, 0.0), 0.0);
  ASSERT_TRUE(coordinator.tick(0.0, nullptr, nullptr, nullptr).bid);
  for (double now = 0.1; now < 12.0; now += 0.1) {
    coordinator.onBid(emptyBid(3), now);  // robot 1 is not heard again
    coordinator.tick(now, nullptr, nullptr, nullptr);
    if (now < params.peer_timeout_s) {
      EXPECT_TRUE(coordinator.awaitingAuction()) << "at " << now;
    }
  }
  EXPECT_EQ(coordinator.auctioneer(12.0), 2);
  EXPECT_FALSE(coordinator.awaitingAuction());
  EXPECT_TRUE(coordinator.requestAnswered());
}

/// A well-formed bid from `robot_id` standing at (x, 0) that knows `known`
/// and holds `bundle`, answering `auction_id`.
TourBidData bidFrom(int robot_id, double x, std::vector<FleetCluster> known,
                    std::uint64_t auction_id = 0,
                    std::vector<ClusterId> bundle = {}) {
  SimRobot robot(robot_id, x, 0.0, FleetParams{});
  robot.known = std::move(known);
  TourBidData bid = robot.ownBid();
  bid.robot_id = robot_id;
  bid.auction_id = auction_id;
  bid.bundle = std::move(bundle);
  bid.current_target = bid.bundle.empty() ? mgg::kNoCluster : bid.bundle[0];
  return bid;
}

// §3.4: a bid received at or after fleet.bid_deadline_s is late, even when
// the auctioneer has not ticked past the deadline yet; it is not collected.
// One received before the deadline is.
TEST(FleetCoordinator, ABidReceivedAfterTheDeadlineIsNotCollected) {
  const FleetParams params;
  const auto collected = [&params](double received_s) {
    FleetCoordinator coordinator(1, params, 0.2);
    coordinator.onBid(emptyBid(2), 0.0);
    const mgg::FleetTickOutput call =
        coordinator.tick(0.0, nullptr, nullptr, nullptr);
    EXPECT_TRUE(call.award && call.award->call);
    if (!call.award) return false;
    coordinator.onBid(
        bidFrom(2, 20.0, {cluster(21, 2, 18.0)}, call.award->auction_id),
        received_s);
    const mgg::FleetTickOutput out = coordinator.tick(
        std::max(received_s, params.bid_deadline_s) + 0.1, nullptr, nullptr,
        nullptr);
    EXPECT_TRUE(out.award && !out.award->call);
    if (!out.award) return false;
    return out.award->bundleOf(2) != nullptr;
  };
  EXPECT_TRUE(collected(params.bid_deadline_s - 0.0625));
  EXPECT_FALSE(collected(params.bid_deadline_s));
  // Received after the deadline, before the auctioneer's next tick.
  EXPECT_FALSE(collected(params.bid_deadline_s + 0.1));
}

// §4: a peer's bid names what it holds, its current target and bundle. An
// auctioneer that knows the claim only from that bid (it restarted, or its
// group just merged) keeps it fixed when the peer misses the call, rather
// than awarding it to another robot.
TEST(FleetCoordinator, AClaimKnownOnlyFromABidIsNotAwardedElsewhere) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 18.0)};
  Radio radio{{&r1, &r2}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  ASSERT_EQ(r2.front(), 21u);

  // Robot 1 restarts knowing robot 2's frontier too. Robot 2 does not hear
  // it, so never answers its calls; its periodic bids get through.
  r1.coordinator = std::make_unique<FleetCoordinator>(1, params, 0.2);
  r1.known.push_back(cluster(21, 2, 18.0));
  radio.deaf.insert({1, 2});
  const std::size_t first = radio.awards.size();
  radio.runUntil(now, 12.0);
  ASSERT_GT(radio.awards.size(), first);
  for (std::size_t i = first; i < radio.awards.size(); ++i) {
    const mgg::RobotBundle* own = radio.awards[i].second.bundleOf(1);
    ASSERT_NE(own, nullptr);
    EXPECT_EQ(own->clusters, std::vector<ClusterId>{11});
  }
  EXPECT_EQ(idsOf(r1.coordinator->bundle()), std::vector<ClusterId>{11});
  EXPECT_EQ(idsOf(r1.coordinator->claimedByOthers(now)),
            std::vector<ClusterId>{21});
  EXPECT_EQ(r2.front(), 21u);
}

// A bundle cut at kMaxBidClusters may go on beyond the cut: the claim keeps
// what it held there. Ids the bid and the awards do not name are skipped.
TEST(FleetCoordinator, AClaimFromABidKeepsWhatATruncatedBundleLeavesOut) {
  FleetCoordinator coordinator(1, FleetParams{}, 0.2);
  std::vector<FleetCluster> held;
  mgg::RobotBundle bundle{2, {}, 0.0};
  for (std::size_t i = 0; i < mgg::kMaxBidClusters + 1; ++i) {
    held.push_back(cluster(static_cast<ClusterId>(100 + i), 2,
                           static_cast<double>(i)));
    bundle.clusters.push_back(held.back().id);
  }
  // Robot 1 learned robot 2's claim from robot 0's award.
  coordinator.onAward(makeAward(0, 1, 0.0, {bundle}, held), 0.0);
  ASSERT_EQ(coordinator.claimedByOthers(0.0).size(), held.size());

  std::vector<ClusterId> sent(bundle.clusters.begin(),
                              bundle.clusters.begin() + mgg::kMaxBidClusters);
  TourBidData cut = bidFrom(2, 0.0, {}, 0, sent);
  cut.stamp_s = 1.0;
  coordinator.onBid(cut, 1.0);
  EXPECT_EQ(coordinator.claimedByOthers(1.0).size(), held.size());

  // A bundle within the limit is the whole claim; an unknown ID is skipped.
  TourBidData whole =
      bidFrom(2, 0.0, {cluster(900, 2, 50.0)}, 0, {100, 900, 5000});
  whole.stamp_s = 2.0;
  coordinator.onBid(whole, 2.0);
  EXPECT_EQ(idsOf(coordinator.claimedByOthers(2.0)),
            (std::vector<ClusterId>{100, 900}));
}

// A peer reports a cluster of the settled award explored in one periodic
// bid, with no new cluster and no request: that makes an auction due, and
// the report is kept for it, which drops the cluster.
TEST(FleetCoordinator, AnExploredReportAfterASettledAwardIsAuctioned) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0), cluster(12, 1, 6.0)};
  r2.known = {cluster(21, 2, 18.0)};
  Radio radio{{&r1, &r2}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  ASSERT_EQ(idsOf(r1.coordinator->bundle()), (std::vector<ClusterId>{11, 12}));
  // Let one auction interval pass with nothing new.
  const std::size_t settled = radio.awards.size();
  radio.runUntil(now, 5.0 + 2.0 * params.auction_interval_s);
  ASSERT_EQ(radio.awards.size(), settled);

  r2.explored = {12};
  const std::size_t bids = radio.bids.size();
  bool reported = false;
  for (; !reported && now < 20.0; now += 0.1) {
    radio.step(now);
    for (std::size_t i = bids; i < radio.bids.size(); ++i) {
      reported |= radio.bids[i].second.robot_id == 2 &&
                  radio.bids[i].second.auction_id == 0;
    }
  }
  ASSERT_TRUE(reported);
  r2.explored.clear();  // reported once
  radio.runUntil(now, now + 2.0 * params.auction_interval_s);
  ASSERT_GT(radio.awards.size(), settled);
  EXPECT_EQ(idsOf(radio.awards.back().second.explored),
            std::vector<ClusterId>{12});
  EXPECT_EQ(idsOf(r1.coordinator->bundle()), std::vector<ClusterId>{11});
}

// The auctioneer's broadcast bids and its own collected bids each send an
// explored list longer than kMaxBidClusters in turns of their own: every
// report reaches the peers and every report reaches this robot's pools,
// whatever the rhythm of bids and auctions.
TEST(FleetCoordinator, TheAuctioneersExploredReportsAllGoOut) {
  const FleetParams params;
  FleetCoordinator coordinator(1, params, 0.2);
  const std::size_t chunks = 3;
  std::vector<ClusterId> explored;
  for (std::size_t i = 0; i < chunks * mgg::kMaxBidClusters; ++i) {
    explored.push_back(static_cast<ClusterId>(5000 + i));
  }
  // One of this robot's own clusters in each turn's share of the list.
  SimRobot self(1, 0.0, 0.0, params);
  std::set<ClusterId> expected;
  for (std::size_t k = 0; k < chunks; ++k) {
    const ClusterId id = explored[k * mgg::kMaxBidClusters];
    self.known.push_back(cluster(id, 1, 10.0 * static_cast<double>(k + 1)));
    expected.insert(id);
  }
  const auto own_bid = [&self, &explored] {
    TourBidData bid = self.ownBid();
    bid.explored = explored;
    return bid;
  };
  std::set<ClusterId> broadcast;
  std::set<ClusterId> pooled;
  // A peer keeps asking for auctions; this robot ticks every two seconds.
  for (double now = 0.0; now < 24.0; now += 2.0) {
    TourBidData peer = emptyBid(2);
    peer.request_auction = true;
    coordinator.onBid(peer, now);
    const mgg::FleetTickOutput out =
        coordinator.tick(now, own_bid, euclid, nullptr);
    if (out.bid) broadcast.insert(out.bid->explored.begin(),
                                  out.bid->explored.end());
    if (out.award && !out.award->call) {
      for (const FleetCluster& c : out.award->explored) pooled.insert(c.id);
    }
  }
  EXPECT_EQ(broadcast.size(), explored.size());
  EXPECT_EQ(pooled, expected);
}

// An award with a non-finite position, a non-finite or negative silence, a
// cluster in two robots' bundles or a bundle ID no cluster entry names is
// refused whole: nothing this robot holds or knows changes. (kNoCluster
// entries alone are filtered instead: AnAwardsUnnamedClustersAreIgnored.)
TEST(FleetCoordinator, AMalformedAwardIsRefusedWhole) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const TourAwardData valid = makeAward(
      1, 1, 1.0,
      {mgg::RobotBundle{2, {21}, 0.0}, mgg::RobotBundle{3, {31}, 0.0}},
      {cluster(21, 2, 2.0), cluster(31, 3, 3.0), cluster(22, 2, 4.0)});
  // A later round that moves robot 2 to cluster 22, and what breaks it.
  TourAwardData next = valid;
  next.auction_id = 2;
  next.stamp_s = 2.0;
  next.bundles[0].clusters = {22};
  std::vector<std::pair<const char*, TourAwardData>> broken;
  const auto variant = [&](const char* what, auto&& change) {
    TourAwardData award = next;
    change(award);
    broken.emplace_back(what, award);
  };
  variant("NaN position", [&](TourAwardData& a) {
    a.clusters[2].position.x() = nan;
  });
  variant("infinite explored position", [&](TourAwardData& a) {
    a.explored = {cluster(41, 4, inf)};
  });
  variant("NaN silence",
          [&](TourAwardData& a) { a.bundles[1].silent_s = nan; });
  variant("infinite silence",
          [&](TourAwardData& a) { a.bundles[1].silent_s = inf; });
  variant("negative silence",
          [&](TourAwardData& a) { a.bundles[1].silent_s = -1.0; });
  variant("a cluster in two bundles",
          [](TourAwardData& a) { a.bundles[0].clusters = {22, 31}; });
  variant("an unlisted bundle ID",
          [](TourAwardData& a) { a.bundles[0].clusters = {22, 99}; });

  for (const auto& [what, award] : broken) {
    FleetCoordinator coordinator(2, FleetParams{}, 0.2);
    coordinator.onAward(valid, 1.0);
    const std::uint64_t version = coordinator.assignmentVersion();
    coordinator.onAward(award, 2.0);
    EXPECT_EQ(idsOf(coordinator.bundle()), std::vector<ClusterId>{21}) << what;
    EXPECT_EQ(idsOf(coordinator.claimedByOthers(2.0)),
              std::vector<ClusterId>{31})
        << what;
    EXPECT_EQ(idsOf(coordinator.lastAwardClusters()),
              (std::vector<ClusterId>{21, 22, 31}))
        << what;
    EXPECT_TRUE(coordinator.exploredElsewhere().empty()) << what;
    EXPECT_EQ(coordinator.assignmentVersion(), version) << what;
  }
  // The same round, well formed, is applied.
  FleetCoordinator coordinator(2, FleetParams{}, 0.2);
  coordinator.onAward(valid, 1.0);
  coordinator.onAward(next, 2.0);
  EXPECT_EQ(idsOf(coordinator.bundle()), std::vector<ClusterId>{22});
}

// SubT radio is often a chain. With 1-2 and 2-3 in contact and 1-3 not,
// robot 2 follows robot 1 and says so in its bids, so robot 3, which does
// not hear robot 1, runs its own auctions instead of waiting for robot 2:
// every robot has an award.
TEST(FleetCoordinator, InAChainEveryRobotHasAnAward) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 10.0, 0.0, params),
      r3(3, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 12.0)};
  r3.known = {cluster(31, 3, 22.0)};
  Radio radio{{&r1, &r2, &r3}};
  radio.cut(1, 3);
  double now = 0.0;
  radio.runUntil(now, 10.0);
  EXPECT_EQ(r1.coordinator->auctioneer(now), 1);
  EXPECT_EQ(r2.coordinator->auctioneer(now), 1);
  EXPECT_EQ(r3.coordinator->auctioneer(now), 3);
  EXPECT_EQ(r1.front(), 11u);
  EXPECT_EQ(r2.front(), 21u);
  EXPECT_EQ(r3.front(), 31u);
  // Robot 3 respects the claim it knows from robot 2's bids.
  EXPECT_EQ(idsOf(r3.coordinator->claimedByOthers(now)),
            std::vector<ClusterId>{21});
}

// Nothing changes for a long time: no auction runs, and still nobody stops
// following the auctioneer.
TEST(FleetCoordinator, ASettledGroupKeepsItsAuctioneerThroughALongQuiet) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 10.0, 0.0, params),
      r3(3, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 12.0)};
  r3.known = {cluster(31, 3, 22.0)};
  Radio radio{{&r1, &r2, &r3}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  const std::size_t awards = radio.awards.size();
  const std::size_t calls = radio.calls.size();
  radio.runUntil(now, 120.0);
  EXPECT_EQ(radio.awards.size(), awards);
  EXPECT_EQ(radio.calls.size(), calls);
  for (const SimRobot* r : radio.robots) {
    EXPECT_EQ(r->coordinator->auctioneer(now), 1) << "robot " << r->id;
  }
}

// The auctioneer restarts: its peers keep following it, and nobody else
// calls an auction meanwhile.
TEST(FleetCoordinator, ARestartedAuctioneerIsStillFollowed) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 10.0, 0.0, params),
      r3(3, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 12.0)};
  r3.known = {cluster(31, 3, 22.0)};
  Radio radio{{&r1, &r2, &r3}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  r1.coordinator = std::make_unique<FleetCoordinator>(1, params, 0.2);
  r1.known.push_back(cluster(13, 1, 6.0));
  const std::size_t awards = radio.awards.size();
  const std::size_t calls = radio.calls.size();
  radio.runUntil(now, 15.0);
  ASSERT_GT(radio.awards.size(), awards);
  for (std::size_t i = calls; i < radio.calls.size(); ++i) {
    EXPECT_EQ(radio.calls[i].second.auctioneer_id, 1);
  }
  for (std::size_t i = awards; i < radio.awards.size(); ++i) {
    EXPECT_EQ(radio.awards[i].second.auctioneer_id, 1);
  }
  for (const SimRobot* r : radio.robots) {
    EXPECT_EQ(r->coordinator->auctioneer(now), 1) << "robot " << r->id;
  }
  EXPECT_EQ(idsOf(r1.coordinator->bundle()), (std::vector<ClusterId>{11, 13}));
}

// A claim is as fresh as the message it came from. Robot 2's bid sent
// before the award that moved it from 21 to 22 arrives after that award: it
// must not restore 21, or the next auction, which robot 2's answer misses,
// would give 22 away while robot 2 tours it.
TEST(FleetCoordinator, ABidOlderThanTheAwardDoesNotReplaceItsClaim) {
  const FleetParams params;
  FleetCoordinator coordinator(1, params, 0.2);
  SimRobot self(1, 0.0, 0.0, params);
  const auto own_bid = [&self] { return self.ownBid(); };
  const auto bid2 = [](double stamp_s, std::vector<FleetCluster> known,
                       std::uint64_t auction_id,
                       std::vector<ClusterId> bundle) {
    TourBidData bid = bidFrom(2, 20.0, std::move(known), auction_id,
                              std::move(bundle));
    bid.stamp_s = stamp_s;
    bid.auctioneer_id = 1;
    return bid;
  };
  coordinator.onBid(bid2(0.0, {cluster(21, 2, 18.0)}, 0, {21}), 0.0);
  const mgg::FleetTickOutput call =
      coordinator.tick(0.0, own_bid, euclid, nullptr);
  ASSERT_TRUE(call.award && call.award->call);
  // Robot 2 answers; 21 is gone from its map meanwhile.
  coordinator.onBid(
      bid2(0.5, {cluster(22, 2, 14.0)}, call.award->auction_id, {21}), 0.5);
  const mgg::FleetTickOutput award =
      coordinator.tick(1.0, own_bid, euclid, nullptr);
  ASSERT_TRUE(award.award && !award.award->call);
  ASSERT_NE(award.award->bundleOf(2), nullptr);
  ASSERT_EQ(award.award->bundleOf(2)->clusters, std::vector<ClusterId>{22});

  // Robot 2's bid of 0.8, sent before it heard the award, arrives late.
  coordinator.onBid(
      bid2(0.8, {cluster(21, 2, 18.0), cluster(22, 2, 14.0)}, 0, {21}), 1.5);
  EXPECT_EQ(idsOf(coordinator.claimedByOthers(1.5)),
            std::vector<ClusterId>{22});

  // Robot 1 now knows 22 too and auctions; robot 2 misses the call.
  self.known = {cluster(22, 2, 14.0)};
  std::vector<TourAwardData> awards;
  for (double now = 2.0; now < 4.55; now += 0.5) {
    const mgg::FleetTickOutput out =
        coordinator.tick(now, own_bid, euclid, nullptr);
    if (out.award && !out.award->call) awards.push_back(*out.award);
  }
  ASSERT_FALSE(awards.empty());
  for (const TourAwardData& later : awards) {
    const mgg::RobotBundle* own = later.bundleOf(1);
    ASSERT_NE(own, nullptr);
    EXPECT_TRUE(own->clusters.empty());
    const mgg::RobotBundle* held = later.bundleOf(2);
    ASSERT_NE(held, nullptr);
    EXPECT_EQ(held->clusters, std::vector<ClusterId>{22});
  }
}

// After a clock reset the source stamp of a claim is pulled back to now, so
// the next bid, stamped on the new clock, replaces the claim.
TEST(FleetCoordinator, AfterAClockRollbackTheNextBidReplacesAClaim) {
  FleetCoordinator coordinator(1, FleetParams{}, 0.2);
  TourBidData before = bidFrom(2, 20.0, {cluster(21, 2, 18.0)}, 0, {21});
  before.stamp_s = 10000.0;
  coordinator.onBid(before, 10000.0);
  ASSERT_EQ(idsOf(coordinator.claimedByOthers(10000.0)),
            std::vector<ClusterId>{21});
  TourBidData after = bidFrom(2, 20.0, {cluster(22, 2, 14.0)}, 0, {22});
  after.stamp_s = 10.5;
  coordinator.onBid(emptyBid(3), 10.0);  // any message at the new time
  coordinator.onBid(after, 10.5);
  EXPECT_EQ(idsOf(coordinator.claimedByOthers(10.5)),
            std::vector<ClusterId>{22});
}

// Two partitions that auctioned apart meet: robots 2 and 3 both claim 50.
// Neither answers robot 1's call, so both claims are fixed. The award names
// 50 once, under the newer claim (the lower ID on a tie), keeps it from the
// bidders, and a follower applies the award.
TEST(FleetCoordinator, OverlappingFixedClaimsAreAwardedToOneHolder) {
  const FleetParams params;
  const auto run = [&params](double stamp2, double stamp3) {
    FleetCoordinator coordinator(1, params, 0.2);
    SimRobot self(1, 0.0, 0.0, params);
    self.known = {cluster(11, 1, 2.0), cluster(50, 2, 10.0)};
    const auto claim = [](int robot_id, double stamp_s) {
      TourBidData bid =
          bidFrom(robot_id, 20.0, {cluster(50, 2, 10.0)}, 0, {50});
      bid.stamp_s = stamp_s;
      bid.auctioneer_id = robot_id;  // each led its own partition
      return bid;
    };
    coordinator.onBid(claim(2, stamp2), 0.0);
    coordinator.onBid(claim(3, stamp3), 0.0);
    const auto own_bid = [&self] { return self.ownBid(); };
    EXPECT_TRUE(coordinator.tick(0.0, own_bid, euclid, nullptr).award);
    const mgg::FleetTickOutput out =
        coordinator.tick(params.bid_deadline_s, own_bid, euclid, nullptr);
    EXPECT_TRUE(out.award && !out.award->call);
    return out.award.value_or(TourAwardData{});
  };
  for (const auto& [stamp2, stamp3, holder] :
       std::vector<std::tuple<double, double, int>>{{0.0, 0.0, 2},
                                                    {0.0, -0.5, 2},
                                                    {-0.5, 0.0, 3}}) {
    const TourAwardData award = run(stamp2, stamp3);
    EXPECT_TRUE(award.wellFormed());
    int named = 0;
    for (const mgg::RobotBundle& bundle : award.bundles) {
      const bool holds =
          std::find(bundle.clusters.begin(), bundle.clusters.end(), 50u) !=
          bundle.clusters.end();
      named += holds ? 1 : 0;
      if (holds) {
        EXPECT_EQ(bundle.robot_id, holder);
      }
    }
    EXPECT_EQ(named, 1);
    ASSERT_NE(award.bundleOf(1), nullptr);
    EXPECT_EQ(award.bundleOf(1)->clusters, std::vector<ClusterId>{11});

    FleetCoordinator follower(4, params, 0.2);
    follower.onAward(award, 1.5);
    EXPECT_TRUE(follower.hasAward());
    EXPECT_EQ(idsOf(follower.lastAwardClusters()),
              (std::vector<ClusterId>{11, 50}));
  }
}

// A call is answered once per round: after calls A and B of one
// auctioneer, A delivered again is not answered.
TEST(FleetCoordinator, AReplayedCallIsNotAnsweredAgain) {
  FleetCoordinator coordinator(2, FleetParams{}, 0.2);
  const TourAwardData a = makeCall(1, 7, 1.0);
  coordinator.onAward(a, 1.0);
  mgg::FleetTickOutput out = coordinator.tick(1.0, nullptr, nullptr, nullptr);
  ASSERT_TRUE(out.bid.has_value());
  EXPECT_EQ(out.bid->auction_id, 7u);
  coordinator.onAward(makeCall(1, 8, 1.5), 1.5);
  out = coordinator.tick(1.5, nullptr, nullptr, nullptr);
  ASSERT_TRUE(out.bid.has_value());
  EXPECT_EQ(out.bid->auction_id, 8u);
  coordinator.onAward(a, 1.8);
  EXPECT_FALSE(coordinator.tick(1.8, nullptr, nullptr, nullptr).bid);
}

// The tour solves again when the claims it respects change: a bid that
// adds, replaces or removes a claim advances assignmentVersion, and one
// that repeats it (a refresh of when its robot was heard) does not.
TEST(FleetCoordinator, BidClaimChangesAdvanceTheAssignmentVersion) {
  FleetCoordinator coordinator(1, FleetParams{}, 0.2);
  const auto bid = [](double stamp_s, std::vector<ClusterId> bundle) {
    TourBidData b =
        bidFrom(2, 20.0, {cluster(21, 2, 18.0), cluster(22, 2, 14.0)}, 0,
                std::move(bundle));
    b.stamp_s = stamp_s;
    return b;
  };
  std::uint64_t version = coordinator.assignmentVersion();
  coordinator.onBid(bid(1.0, {21}), 1.0);
  EXPECT_GT(coordinator.assignmentVersion(), version) << "added";
  version = coordinator.assignmentVersion();
  coordinator.onBid(bid(2.0, {21}), 2.0);
  EXPECT_EQ(coordinator.assignmentVersion(), version) << "unchanged";
  coordinator.onBid(bid(3.0, {22}), 3.0);
  EXPECT_GT(coordinator.assignmentVersion(), version) << "replaced";
  version = coordinator.assignmentVersion();
  coordinator.onBid(bid(4.0, {}), 4.0);
  EXPECT_GT(coordinator.assignmentVersion(), version) << "removed";
}

/// Robot 2's bid holding `bundle`, stamped `stamp_s`, following robot 1.
TourBidData claimOf2(double stamp_s, std::vector<ClusterId> bundle) {
  TourBidData bid =
      bidFrom(2, 20.0, {cluster(21, 2, 18.0), cluster(22, 2, 14.0)}, 0,
              std::move(bundle));
  bid.stamp_s = stamp_s;
  bid.auctioneer_id = 1;
  return bid;
}

// A release is the claim's latest state. Robot 2's bid of 1 s, delayed in
// transit, arrives after the operator released robot 2's claims at 20 s:
// it must not bring the claim back.
TEST(FleetCoordinator, ADelayedBidDoesNotUndoAnOperatorRelease) {
  FleetCoordinator coordinator(1, FleetParams{}, 0.2);
  coordinator.onBid(claimOf2(0.0, {21}), 0.0);
  ASSERT_EQ(idsOf(coordinator.claimedByOthers(0.0)),
            std::vector<ClusterId>{21});
  ASSERT_TRUE(coordinator.releaseClaims(2, 20.0));
  coordinator.onBid(claimOf2(1.0, {21}), 21.0);
  EXPECT_TRUE(coordinator.claimedByOthers(21.0).empty());
  // A bid sent after the release claims again.
  coordinator.onBid(claimOf2(22.0, {22}), 22.0);
  EXPECT_EQ(idsOf(coordinator.claimedByOthers(22.0)),
            std::vector<ClusterId>{22});
}

// The same for a robot alone taking the oldest silent claim over.
TEST(FleetCoordinator, ADelayedBidDoesNotUndoATakeOver) {
  FleetCoordinator coordinator(1, FleetParams{}, 0.2);
  coordinator.onBid(claimOf2(0.0, {21}), 0.0);
  ASSERT_EQ(coordinator.group(20.0), std::vector<int>{1});
  ASSERT_EQ(coordinator.takeOverOldestClaim(20.0), 2);
  coordinator.onBid(claimOf2(1.0, {21}), 21.0);
  EXPECT_TRUE(coordinator.claimedByOthers(21.0).empty());
}

// And for the take-over an auction makes for an idle bidder (§4 release 2):
// robot 1, idle, gets robot 2's cluster; robot 2's delayed bid must not
// leave it claimed by robot 2 as well.
TEST(FleetCoordinator, ADelayedBidDoesNotUndoAnAuctionsTakeOver) {
  const FleetParams params;
  FleetCoordinator coordinator(1, params, 0.2);
  coordinator.onBid(claimOf2(0.0, {21}), 0.0);
  bool released = false;
  for (double now = 0.0; now < 12.0 && !released; now += 0.5) {
    coordinator.onBid(emptyBid(3), now);  // keeps robot 1 in a group
    const mgg::FleetTickOutput out =
        coordinator.tick(now, nullptr, euclid, nullptr);
    if (out.award && !out.award->call) {
      const auto& ids = out.award->released_robot_ids;
      released = std::find(ids.begin(), ids.end(), 2) != ids.end();
    }
  }
  ASSERT_TRUE(released);
  EXPECT_EQ(idsOf(coordinator.bundle()), std::vector<ClusterId>{21});
  // Sent at 4 s, after the first award, before robot 2 fell silent.
  coordinator.onBid(claimOf2(4.0, {21}), 12.5);
  EXPECT_TRUE(coordinator.claimedByOthers(12.5).empty());
}

// An award releasing robot 2's claims, stamped before robot 2's latest bid,
// arrives after that bid: the bid is newer and its claim stays. A release
// award newer than the bid removes it.
TEST(FleetCoordinator, AReleaseAwardOlderThanABidKeepsTheBidsClaim) {
  FleetCoordinator coordinator(3, FleetParams{}, 0.2);
  const std::vector<FleetCluster> clusters = {cluster(21, 2, 18.0)};
  coordinator.onAward(
      makeAward(1, 1, 10.0, {mgg::RobotBundle{2, {21}, 0.0}}, clusters),
      10.0);
  coordinator.onBid(claimOf2(30.0, {22}), 30.0);
  ASSERT_EQ(idsOf(coordinator.claimedByOthers(30.0)),
            std::vector<ClusterId>{22});
  TourAwardData release = makeAward(1, 2, 20.0, {}, {});
  release.released_robot_ids = {2};
  coordinator.onAward(release, 31.0);
  EXPECT_EQ(idsOf(coordinator.claimedByOthers(31.0)),
            std::vector<ClusterId>{22});
  release.auction_id = 3;
  release.stamp_s = 32.0;
  coordinator.onAward(release, 32.0);
  EXPECT_TRUE(coordinator.claimedByOthers(32.0).empty());
}

}  // namespace

TEST(FleetCoordinator, AnOmittedBidderKeepsRequestingUntilNamedInAnAward) {
  FleetCoordinator leader(1, FleetParams{}, 0.2), follower(2, FleetParams{}, 0.2);
  leader.onBid(emptyBid(2), 0.0);
  follower.onBid(emptyBid(1), 0.0);
  leader.tick(0.0, nullptr, nullptr, nullptr);  // call lost
  follower.requestAuction();
  auto reply = follower.tick(0.1, nullptr, nullptr, nullptr);
  ASSERT_TRUE(reply.bid);
  leader.onBid(*reply.bid, 0.1);  // not an answer to the current call
  auto out = leader.tick(1.1, nullptr, nullptr, nullptr);
  ASSERT_TRUE(out.award);
  ASSERT_EQ(out.award->bundleOf(2), nullptr);
  follower.onAward(*out.award, 1.1);
  EXPECT_FALSE(follower.requestAnswered());
  EXPECT_TRUE(follower.awaitingAuction());
  reply = follower.tick(3.2, nullptr, nullptr, nullptr);
  ASSERT_TRUE(reply.bid);
  EXPECT_TRUE(reply.bid->request_auction);
  leader.onBid(*reply.bid, 3.2);
  out = leader.tick(3.2, nullptr, nullptr, nullptr);
  ASSERT_TRUE(out.award && out.award->call);
  follower.onAward(*out.award, 3.2);
  reply = follower.tick(3.3, nullptr, nullptr, nullptr);
  ASSERT_TRUE(reply.bid);
  leader.onBid(*reply.bid, 3.3);
  out = leader.tick(4.3, nullptr, nullptr, nullptr);
  ASSERT_TRUE(out.award && !out.award->call);
  ASSERT_NE(out.award->bundleOf(1), nullptr);
  ASSERT_NE(out.award->bundleOf(2), nullptr);
  EXPECT_TRUE(out.award->bundleOf(2)->clusters.empty());
  follower.onAward(*out.award, 4.3);
  EXPECT_TRUE(follower.requestAnswered());
}
