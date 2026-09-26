#include "mgg_core/fleet_coordinator.h"

#include <algorithm>
#include <utility>

namespace mgg {
namespace {

std::vector<ClusterId> idsOf(const std::vector<FleetCluster>& clusters) {
  std::vector<ClusterId> ids;
  ids.reserve(clusters.size());
  for (const FleetCluster& c : clusters) ids.push_back(c.id);
  return ids;
}

/// kNoCluster names no cluster (a broken peer's award): dropped, or this
/// robot's next bid would carry it and peers would refuse the bid.
std::vector<FleetCluster> namedOnly(const std::vector<FleetCluster>& clusters) {
  std::vector<FleetCluster> named;
  named.reserve(clusters.size());
  for (const FleetCluster& c : clusters) {
    if (c.id != kNoCluster) named.push_back(c);
  }
  return named;
}

bool isMember(const std::vector<int>& members, int robot_id) {
  return std::find(members.begin(), members.end(), robot_id) != members.end();
}

}  // namespace

FleetCoordinator::FleetCoordinator(int robot_id, const FleetParams& params,
                                   double commit_margin)
    : robot_id_(robot_id), params_(params), commit_margin_(commit_margin) {}

std::vector<int> FleetCoordinator::group(double now_s) const {
  std::vector<int> members{robot_id_};
  for (const auto& [robot_id, heard_s] : last_heard_s_) {
    if (robot_id != robot_id_ && now_s - heard_s <= params_.peer_timeout_s) {
      members.push_back(robot_id);
    }
  }
  std::sort(members.begin(), members.end());
  return members;
}

int FleetCoordinator::auctioneer(double now_s) const {
  for (const int robot_id : group(now_s)) {
    if (robot_id == robot_id_) return robot_id;
    const auto named = follows_.find(robot_id);
    if (named == follows_.end() || named->second == robot_id) return robot_id;
  }
  return robot_id_;
}

void FleetCoordinator::rebaseFutureTimes(double now_s) {
  for (auto& [robot_id, heard_s] : last_heard_s_) {
    heard_s = std::min(heard_s, now_s);
  }
  // Timers too, or bids, awards and calls would wait for the old clock.
  last_bid_s_ = std::min(last_bid_s_, now_s);
  last_auction_s_ = std::min(last_auction_s_, now_s);
  if (collecting_) {
    collecting_->started_s = std::min(collecting_->started_s, now_s);
  }
  for (auto& [robot_id, stamp_s] : claim_stamp_s_) {
    stamp_s = std::min(stamp_s, now_s);
  }
  if (now_s < round_heard_s_) {
    applied_stamp_s_.clear();
    answered_call_s_.clear();
    called_round_.reset();
    round_heard_s_ = now_s;
  }
}

void FleetCoordinator::noteHeard(int robot_id, double now_s) {
  // The receipt time itself, even when earlier than the one kept: after a
  // clock reset the kept time would hold the robot in the group.
  last_heard_s_[robot_id] = now_s;
  claims_.heard(robot_id, now_s);
}

void FleetCoordinator::onBid(const TourBidData& bid, double now_s) {
  rebaseFutureTimes(now_s);
  if (bid.robot_id == robot_id_ || !bid.wellFormed()) return;
  noteHeard(bid.robot_id, now_s);
  follows_[bid.robot_id] = bid.auctioneer_id;
  recordBidClaim(bid, now_s);
  noteReportedExplored(bid.explored);
  last_bids_[bid.robot_id] = bid;
  if (bid.request_auction) peer_requested_ = true;
  // Collected only before the deadline, as the local receipt time tells: a
  // late bid, like a missing one, leaves its bidder with what it holds.
  if (collecting_ && bid.auction_id == collecting_->auction_id &&
      now_s - collecting_->started_s < params_.bid_deadline_s) {
    collecting_->bids[bid.robot_id] = bid;
  }
}

void FleetCoordinator::recordBidClaim(const TourBidData& bid, double now_s) {
  const auto source = claim_stamp_s_.find(bid.robot_id);
  if (source != claim_stamp_s_.end() && bid.stamp_s <= source->second) return;
  claim_stamp_s_[bid.robot_id] = bid.stamp_s;
  std::vector<ClusterId> ids;
  if (bid.current_target != kNoCluster) ids.push_back(bid.current_target);
  for (const ClusterId id : bid.bundle) {
    if (std::find(ids.begin(), ids.end(), id) == ids.end()) ids.push_back(id);
  }
  const Claim* held = claims_.find(bid.robot_id);
  const auto named = [](const std::vector<FleetCluster>& clusters,
                        ClusterId id) -> const FleetCluster* {
    for (const FleetCluster& c : clusters) {
      if (c.id == id) return &c;
    }
    return nullptr;
  };
  std::vector<FleetCluster> clusters;
  for (const ClusterId id : ids) {
    const FleetCluster* c = named(bid.clusters, id);
    if (c == nullptr) c = named(award_clusters_, id);
    if (c == nullptr && held != nullptr) c = named(held->clusters, id);
    if (c != nullptr) clusters.push_back(*c);
  }
  if (held != nullptr && bid.bundle.size() >= kMaxBidClusters) {
    for (const FleetCluster& c : held->clusters) {
      if (named(clusters, c.id) == nullptr) clusters.push_back(c);
    }
  }
  // Added, replaced or removed, not only heard again: the tour solves anew.
  const auto same = [](const FleetCluster& a, const FleetCluster& b) {
    return a.id == b.id && a.position == b.position;
  };
  const bool changed =
      held == nullptr
          ? !clusters.empty()
          : !std::equal(held->clusters.begin(), held->clusters.end(),
                        clusters.begin(), clusters.end(), same);
  claims_.record(bid.robot_id, std::move(clusters), now_s);
  if (changed) ++assignment_version_;
}

void FleetCoordinator::noteReportedExplored(
    const std::vector<ClusterId>& ids) {
  for (const ClusterId id : ids) {
    if (std::find(reported_explored_.begin(), reported_explored_.end(), id) !=
        reported_explored_.end()) {
      continue;
    }
    reported_explored_.push_back(id);
    if (std::any_of(award_clusters_.begin(), award_clusters_.end(),
                    [id](const FleetCluster& c) { return c.id == id; })) {
      explored_reported_ = true;
    }
  }
  if (reported_explored_.size() > kMaxExploredElsewhere) {
    reported_explored_.erase(
        reported_explored_.begin(),
        reported_explored_.begin() +
            static_cast<std::ptrdiff_t>(reported_explored_.size() -
                                        kMaxExploredElsewhere));
  }
}

void FleetCoordinator::onAward(const TourAwardData& award, double now_s) {
  rebaseFutureTimes(now_s);
  if (award.auctioneer_id == robot_id_ || !award.wellFormed()) return;
  noteHeard(award.auctioneer_id, now_s);
  follows_[award.auctioneer_id] = award.auctioneer_id;
  // A robot that left the sender's group, or whose group just merged with a
  // lower ID, ignores it.
  if (award.auctioneer_id != auctioneer(now_s)) return;
  round_heard_s_ = now_s;
  if (award.call) {
    // A replayed call does not displace a newer one not yet answered.
    if (!called_round_ || called_round_->auctioneer_id != award.auctioneer_id ||
        award.stamp_s > called_round_->stamp_s) {
      called_auction_ = award.auction_id;
      called_round_ = Round{award.auctioneer_id, award.stamp_s};
    }
    return;
  }
  const auto applied = applied_stamp_s_.find(award.auctioneer_id);
  if (applied != applied_stamp_s_.end() && award.stamp_s <= applied->second) {
    return;
  }
  applyAward(award, now_s);
}

void FleetCoordinator::requestAuction() {
  requested_ = true;
  request_sent_ = false;
  answered_ = false;
  request_auctioneer_ = -1;
}

FleetTickOutput FleetCoordinator::tick(double now_s, const OwnBidFn& own_bid,
                                       const CostEstimateFn& estimate,
                                       const ExploredFn& explored) {
  rebaseFutureTimes(now_s);
  FleetTickOutput out;
  bool claims_changed = !claims_.expire(now_s, params_.claim_ttl_s).empty();
  if (explored && claims_.dropExplored(explored) > 0) claims_changed = true;
  if (claims_changed) ++assignment_version_;

  const std::vector<int> members = group(now_s);
  const int elected = auctioneer(now_s);
  const bool leader = members.size() > 1 && elected == robot_id_;
  const bool requesting = requested_ && !answered_;
  // The auctioneer asked is gone: ask the new one.
  if (requesting && request_sent_ && request_auctioneer_ != elected) {
    request_sent_ = false;
  }

  const auto makeBid = [&](std::uint64_t auction_id,
                           std::size_t& explored_turn) {
    TourBidData bid = own_bid ? own_bid() : TourBidData{};
    bid.robot_id = robot_id_;
    bid.seq = ++seq_;
    bid.stamp_s = now_s;
    bid.auction_id = auction_id;
    bid.auctioneer_id = elected;
    bid.bundle = idsOf(bundle_);
    capBidLists(bid, explored_turn);
    bid.request_auction = bid.request_auction || requesting;
    own_cluster_ids_.clear();
    for (const FleetCluster& c : bid.clusters) own_cluster_ids_.insert(c.id);
    return bid;
  };

  // §3.3: bid on a call, every auction interval so peers hear this robot,
  // and at once when its bundle is done.
  bool called = false;
  if (called_round_) {
    const auto answered = answered_call_s_.find(called_round_->auctioneer_id);
    called = answered == answered_call_s_.end() ||
             called_round_->stamp_s > answered->second;
  }
  const bool periodic = now_s - last_bid_s_ >= params_.auction_interval_s;
  const bool request_now = requesting && !request_sent_;
  if (called || periodic || request_now) {
    out.bid = makeBid(called ? called_auction_ : 0, broadcast_explored_turn_);
    last_bid_s_ = now_s;
    if (called) {
      answered_call_s_[called_round_->auctioneer_id] = called_round_->stamp_s;
    }
    if (requesting) {
      request_sent_ = true;
      request_auctioneer_ = elected;
    }
  }

  if (!leader) {
    collecting_.reset();
    return out;
  }
  if (collecting_) {
    if (now_s - collecting_->started_s < params_.bid_deadline_s) return out;
    TourAwardData award = computeAward(now_s, estimate, explored);
    auctioned_members_ = collecting_->members;
    auctioned_signature_ = collecting_->signature;
    collecting_.reset();
    last_auction_s_ = now_s;
    peer_requested_ = false;
    // What followers would refuse is not applied here either.
    if (!award.wellFormed()) return out;
    applyAward(award, now_s);
    out.award = std::move(award);
    return out;
  }
  // §3.2: a cluster appeared or disappeared in the pool, a robot joined or
  // left, a robot asked, or the operator released a claim.
  const std::set<ClusterId> signature = poolSignature(members);
  const bool due = !has_award_ || members != auctioned_members_ ||
                   signature != auctioned_signature_ || peer_requested_ ||
                   requesting || !pending_releases_.empty() ||
                   explored_reported_;
  if (!due || now_s - last_auction_s_ < params_.auction_interval_s) return out;
  Collection collection;
  collection.auction_id =
      (static_cast<std::uint64_t>(robot_id_ & 0xFFFF) << 40) |
      ++auction_counter_;
  collection.started_s = now_s;
  collection.members = members;
  collection.signature = signature;
  collection.bids[robot_id_] =
      makeBid(collection.auction_id, collected_explored_turn_);
  TourAwardData call;
  call.auction_id = collection.auction_id;
  call.auctioneer_id = robot_id_;
  call.stamp_s = now_s;
  call.call = true;
  collecting_ = std::move(collection);
  out.award = std::move(call);
  return out;
}

TourAwardData FleetCoordinator::computeAward(double now_s,
                                             const CostEstimateFn& estimate,
                                             const ExploredFn& explored) {
  const Collection& collection = *collecting_;
  std::vector<TourBidData> bids;
  std::set<int> bidders;
  for (const auto& [robot_id, bid] : collection.bids) {
    if (!isMember(collection.members, robot_id)) continue;
    bids.push_back(bid);
    bidders.insert(robot_id);
    // The explored reports of bids since the last auction go with this
    // robot's own bid (buildClusterPool drops every ID a bid reports).
    if (robot_id == robot_id_) {
      bids.back().explored.insert(bids.back().explored.end(),
                                  reported_explored_.begin(),
                                  reported_explored_.end());
    }
  }
  reported_explored_.clear();
  explored_reported_ = false;
  // Robots that did not bid keep what they hold (§3.4 step 3): members whose
  // bid missed the deadline, and silent robots.
  std::set<int> holders;
  std::set<int> silent;
  for (const int robot_id : claims_.robots()) {
    if (robot_id == robot_id_ || bidders.count(robot_id) > 0) continue;
    holders.insert(robot_id);
    if (!isMember(collection.members, robot_id)) silent.insert(robot_id);
  }
  std::vector<int> released = pending_releases_;
  pending_releases_.clear();

  // A cluster once reported explored stays out of the pool. A frontier its
  // owner explored is demoted everywhere by the owner's next broadcast
  // (graph_merge takes the owner's mark both ways), but one another robot
  // explored is not: the owner's map does not show that robot's ground, so
  // the owner keeps marking it, bidding it and broadcasting it as a
  // frontier, which re-marks every merged copy of it.
  const ExploredFn explored_anywhere = [this,
                                        &explored](const Eigen::Vector3d& p) {
    if (explored && explored(p)) return true;
    return std::any_of(explored_elsewhere_.begin(), explored_elsewhere_.end(),
                       [&](const FleetCluster& e) {
                         return (e.position - p).norm() <=
                                params_.cluster_merge_radius_m;
                       });
  };

  ClusterPool pool;
  AuctionResult result;
  std::vector<FleetCluster> freed;
  for (int attempt = 0; attempt < 2; ++attempt) {
    const std::vector<FleetCluster> held = claims_.clustersOf(holders);
    std::vector<FleetCluster> listed = held;
    listed.insert(listed.end(), freed.begin(), freed.end());
    pool = buildClusterPool(bids, listed, params_.cluster_merge_radius_m,
                            explored_anywhere);
    std::vector<bool> fixed(pool.clusters.size(), false);
    for (const FleetCluster& c : held) {
      int p = pool.indexOf(c.id);
      if (p < 0) p = pool.indexNear(c.position, params_.cluster_merge_radius_m);
      if (p >= 0) fixed[p] = true;
    }
    std::vector<AuctionBidder> auction_bidders;
    for (std::size_t k = 0; k < bids.size(); ++k) {
      auction_bidders.push_back(
          bidderCosts(bids[k], pool.bid_to_pool[k], pool, estimate));
    }
    result = runSequentialAuction(auction_bidders, fixed, commit_margin_,
                                  params_.balance_weight);
    // §3.5 and §4 release 2: a bidder left with nothing takes over the
    // claim of the robot silent longest, one per auction.
    const bool idle = std::any_of(
        bidders.begin(), bidders.end(), [&result](int robot_id) {
          const auto found = result.bundles.find(robot_id);
          return found == result.bundles.end() || found->second.empty();
        });
    if (attempt > 0 || !idle || silent.empty()) break;
    std::set<int> candidates;
    for (const int robot_id : silent) {
      if (holders.count(robot_id) > 0) candidates.insert(robot_id);
    }
    const Claim* oldest = nullptr;
    for (const int robot_id : candidates) {
      const Claim* claim = claims_.find(robot_id);
      if (claim != nullptr &&
          (oldest == nullptr || claim->last_heard_s < oldest->last_heard_s)) {
        oldest = claim;
      }
    }
    if (oldest == nullptr) break;
    freed = oldest->clusters;
    const int taken = claims_.releaseOldest(candidates);
    noteReleased(taken, now_s);
    holders.erase(taken);
    released.push_back(taken);
    ++assignment_version_;
  }
  last_unassigned_ = result.unassigned.size();

  TourAwardData award;
  award.auction_id = collection.auction_id;
  award.auctioneer_id = robot_id_;
  award.stamp_s = now_s;
  std::set<ClusterId> named;
  const auto name = [&](int p) {
    const FleetCluster& c = pool.clusters[p];
    if (named.insert(c.id).second) award.clusters.push_back(c);
    return c.id;
  };
  // The auction result names every collected bidder, even an empty bundle:
  // that explicit entry lets an idle bidder distinguish an answer from omission.
  for (const auto& [robot_id, indices] : result.bundles) {
    RobotBundle bundle;
    bundle.robot_id = robot_id;
    for (const int p : indices) bundle.clusters.push_back(name(p));
    award.bundles.push_back(std::move(bundle));
  }
  // Claims can overlap (partitions that auctioned apart, a claim known from
  // a bid): each fixed pool cluster goes to one holder, the one whose claim
  // came from the newest message, the lower ID on a tie.
  const auto held_index = [&](const FleetCluster& c) {
    const int p = pool.indexOf(c.id);
    return p >= 0 ? p : pool.indexNear(c.position,
                                       params_.cluster_merge_radius_m);
  };
  const auto source_s = [this](int robot_id) {
    const auto found = claim_stamp_s_.find(robot_id);
    return found == claim_stamp_s_.end() ? kNever : found->second;
  };
  std::map<int, int> fixed_holder;
  for (const int holder : holders) {
    const Claim* claim = claims_.find(holder);
    if (claim == nullptr) continue;
    for (const FleetCluster& c : claim->clusters) {
      const int p = held_index(c);
      if (p < 0) continue;  // explored meanwhile
      const auto owner = fixed_holder.find(p);
      if (owner == fixed_holder.end()) {
        fixed_holder[p] = holder;
      } else if (source_s(holder) > source_s(owner->second)) {
        owner->second = holder;
      }
    }
  }
  for (const int holder : holders) {
    const Claim* claim = claims_.find(holder);
    if (claim == nullptr) continue;
    RobotBundle bundle;
    bundle.robot_id = holder;
    bundle.silent_s = std::max(0.0, now_s - claim->last_heard_s);
    for (const FleetCluster& c : claim->clusters) {
      const int p = held_index(c);
      if (p < 0 || fixed_holder[p] != holder) continue;
      const ClusterId id = pool.clusters[p].id;
      if (std::find(bundle.clusters.begin(), bundle.clusters.end(), id) ==
          bundle.clusters.end()) {
        bundle.clusters.push_back(name(p));
      }
    }
    award.bundles.push_back(std::move(bundle));
  }
  award.explored = pool.dropped_explored;
  award.released_robot_ids = std::move(released);
  return award;
}

void FleetCoordinator::applyAward(const TourAwardData& award, double now_s) {
  applied_stamp_s_[award.auctioneer_id] = award.stamp_s;
  has_award_ = true;
  award_clusters_ = namedOnly(award.clusters);
  noteExplored(namedOnly(award.explored));
  for (const int robot_id : award.released_robot_ids) {
    // A bid newer than this award claimed again since.
    const auto source = claim_stamp_s_.find(robot_id);
    if (source != claim_stamp_s_.end() && award.stamp_s < source->second) {
      continue;
    }
    claims_.release(robot_id);
    claim_stamp_s_[robot_id] = award.stamp_s;
  }
  for (const RobotBundle& bundle : award.bundles) {
    std::vector<FleetCluster> clusters;
    for (const ClusterId id : bundle.clusters) {
      if (id == kNoCluster) continue;
      if (const FleetCluster* c = award.cluster(id)) clusters.push_back(*c);
    }
    if (bundle.robot_id == robot_id_) {
      bundle_ = std::move(clusters);
      continue;
    }
    // A bid newer than this award named the robot's claim since.
    const auto source = claim_stamp_s_.find(bundle.robot_id);
    if (source != claim_stamp_s_.end() && award.stamp_s < source->second) {
      continue;
    }
    claim_stamp_s_[bundle.robot_id] = award.stamp_s;
    // Last heard as the auctioneer knows it, so a silent robot's claim ages
    // here as it does there; never after now, so that after a clock reset
    // awards naming a silent robot do not keep its claim in the future.
    double heard_s = now_s - std::max(0.0, bundle.silent_s);
    const auto known = last_heard_s_.find(bundle.robot_id);
    if (known != last_heard_s_.end()) {
      heard_s = std::max(heard_s, known->second);
    }
    heard_s = std::min(heard_s, now_s);
    claims_.record(bundle.robot_id, std::move(clusters), heard_s);
  }
  // A robot the award does not name (its bid missed the deadline) keeps its
  // previous bundle.
  if (requested_ && request_sent_ &&
      award.auctioneer_id == request_auctioneer_ &&
      award.bundleOf(robot_id_) != nullptr) {
    answered_ = true;
  }
  if (!bundle_.empty()) {
    requested_ = false;
    answered_ = false;
    request_sent_ = false;
    request_auctioneer_ = -1;
  }
  ++assignment_version_;
}

bool FleetCoordinator::releaseClaims(int robot_id, double now_s) {
  rebaseFutureTimes(now_s);
  if (auctioneer(now_s) != robot_id_ || robot_id == robot_id_ ||
      (last_heard_s_.count(robot_id) == 0 && claim_stamp_s_.count(robot_id) == 0)) {
    return false;
  }
  claims_.release(robot_id);
  noteReleased(robot_id, now_s);
  pending_releases_.push_back(robot_id);
  ++assignment_version_;
  return true;
}

int FleetCoordinator::takeOverOldestClaim(double now_s) {
  rebaseFutureTimes(now_s);
  const std::vector<int> members = group(now_s);
  std::set<int> silent;
  for (const int robot_id : claims_.robots()) {
    if (!isMember(members, robot_id)) silent.insert(robot_id);
  }
  const int taken = claims_.releaseOldest(silent);
  if (taken >= 0) {
    noteReleased(taken, now_s);
    ++assignment_version_;
  }
  return taken;
}

void FleetCoordinator::noteReleased(int robot_id, double now_s) {
  const auto source = claim_stamp_s_.find(robot_id);
  claim_stamp_s_[robot_id] = source == claim_stamp_s_.end()
                                 ? now_s
                                 : std::max(source->second, now_s);
}

std::vector<FleetCluster> FleetCoordinator::claimedByOthers(double now_s) {
  rebaseFutureTimes(now_s);
  if (!claims_.expire(now_s, params_.claim_ttl_s).empty()) {
    ++assignment_version_;
  }
  return claims_.clustersExcept(robot_id_);
}

void FleetCoordinator::capBidLists(TourBidData& bid,
                                   std::size_t& explored_turn) {
  if (bid.bundle.size() > kMaxBidClusters) bid.bundle.resize(kMaxBidClusters);
  const std::size_t n = bid.explored.size();
  if (n <= kMaxBidClusters) return;
  const std::size_t first = explored_turn % n;
  std::rotate(bid.explored.begin(),
              bid.explored.begin() + static_cast<std::ptrdiff_t>(first),
              bid.explored.end());
  bid.explored.resize(kMaxBidClusters);
  explored_turn = (first + kMaxBidClusters) % n;
}

void FleetCoordinator::noteExplored(const std::vector<FleetCluster>& clusters) {
  for (const FleetCluster& c : clusters) {
    const bool known = std::any_of(
        explored_elsewhere_.begin(), explored_elsewhere_.end(),
        [&](const FleetCluster& e) {
          return (e.position - c.position).norm() <=
                 params_.cluster_merge_radius_m;
        });
    if (!known) explored_elsewhere_.push_back(c);
  }
  if (explored_elsewhere_.size() > kMaxExploredElsewhere) {
    explored_elsewhere_.erase(
        explored_elsewhere_.begin(),
        explored_elsewhere_.begin() +
            static_cast<std::ptrdiff_t>(explored_elsewhere_.size() -
                                        kMaxExploredElsewhere));
  }
}

std::set<ClusterId> FleetCoordinator::poolSignature(
    const std::vector<int>& members) const {
  std::set<ClusterId> ids = own_cluster_ids_;
  for (const int robot_id : members) {
    const auto bid = last_bids_.find(robot_id);
    if (bid == last_bids_.end()) continue;
    for (const FleetCluster& c : bid->second.clusters) ids.insert(c.id);
  }
  return ids;
}

}  // namespace mgg
