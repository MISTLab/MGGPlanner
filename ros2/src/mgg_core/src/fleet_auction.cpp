#include "mgg_core/fleet_auction.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "mgg_core/tour_solver.h"

namespace mgg {

int ClusterPool::indexOf(ClusterId id) const {
  for (std::size_t p = 0; p < member_ids.size(); ++p) {
    if (std::find(member_ids[p].begin(), member_ids[p].end(), id) !=
        member_ids[p].end()) {
      return static_cast<int>(p);
    }
  }
  return -1;
}

int ClusterPool::indexNear(const Eigen::Vector3d& position,
                           double radius_m) const {
  int best = -1;
  double best_distance = radius_m;
  for (std::size_t p = 0; p < clusters.size(); ++p) {
    const double distance = (clusters[p].position - position).norm();
    if (distance <= best_distance) {
      best_distance = distance;
      best = static_cast<int>(p);
    }
  }
  return best;
}

ClusterPool buildClusterPool(const std::vector<TourBidData>& bids,
                             const std::vector<FleetCluster>& held,
                             double merge_radius_m,
                             const ExploredFn& explored) {
  struct Entry {
    FleetCluster cluster;
    int bid = -1;
    int index = -1;
    bool owner_report = false;
  };
  std::vector<Entry> entries;
  for (std::size_t b = 0; b < bids.size(); ++b) {
    for (std::size_t i = 0; i < bids[b].clusters.size(); ++i) {
      const FleetCluster& c = bids[b].clusters[i];
      entries.push_back({c, static_cast<int>(b), static_cast<int>(i),
                         c.owner_robot_id == bids[b].robot_id});
    }
  }
  for (const FleetCluster& c : held) entries.push_back({c, -1, -1, false});
  // Owners' own reports first, then by ID, so a place's name does not depend
  // on the order the bids arrived in. Equal keys keep bids as given, then
  // held clusters.
  std::stable_sort(entries.begin(), entries.end(),
                   [](const Entry& a, const Entry& b) {
                     if (a.owner_report != b.owner_report) {
                       return a.owner_report;
                     }
                     return a.cluster.id < b.cluster.id;
                   });

  ClusterPool merged;
  std::vector<int> entry_pool(entries.size(), -1);
  for (std::size_t k = 0; k < entries.size(); ++k) {
    const FleetCluster& c = entries[k].cluster;
    // An ID already pooled joins its cluster wherever this report puts it:
    // one ID is never two pool clusters, so never awarded twice.
    int p = merged.indexOf(c.id);
    if (p >= 0) {
      if ((merged.clusters[p].position - c.position).norm() >
          merge_radius_m) {
        ++merged.conflicting_reports;
      }
    } else {
      p = merged.indexNear(c.position, merge_radius_m);
    }
    if (p < 0) {
      p = static_cast<int>(merged.clusters.size());
      merged.clusters.push_back(c);
      merged.member_ids.emplace_back();
    } else {
      merged.clusters[p].gain = std::max(merged.clusters[p].gain, c.gain);
    }
    std::vector<ClusterId>& members = merged.member_ids[p];
    if (std::find(members.begin(), members.end(), c.id) == members.end()) {
      members.push_back(c.id);
    }
    entry_pool[k] = p;
  }

  std::unordered_set<ClusterId> explored_ids;
  for (const TourBidData& bid : bids) {
    explored_ids.insert(bid.explored.begin(), bid.explored.end());
  }
  ClusterPool pool;
  pool.conflicting_reports = merged.conflicting_reports;
  std::vector<int> reindex(merged.clusters.size(), -1);
  for (std::size_t p = 0; p < merged.clusters.size(); ++p) {
    const bool named_explored = std::any_of(
        merged.member_ids[p].begin(), merged.member_ids[p].end(),
        [&explored_ids](ClusterId id) { return explored_ids.count(id) > 0; });
    if (named_explored ||
        (explored && explored(merged.clusters[p].position))) {
      pool.dropped_explored.push_back(merged.clusters[p]);
      continue;
    }
    reindex[p] = static_cast<int>(pool.clusters.size());
    pool.clusters.push_back(merged.clusters[p]);
    pool.member_ids.push_back(merged.member_ids[p]);
  }
  pool.bid_to_pool.resize(bids.size());
  for (std::size_t b = 0; b < bids.size(); ++b) {
    pool.bid_to_pool[b].assign(bids[b].clusters.size(), -1);
  }
  for (std::size_t k = 0; k < entries.size(); ++k) {
    if (entries[k].bid < 0) continue;
    pool.bid_to_pool[entries[k].bid][entries[k].index] =
        reindex[entry_pool[k]];
  }
  return pool;
}

AuctionBidder bidderCosts(const TourBidData& bid,
                          const std::vector<int>& bid_to_pool,
                          const ClusterPool& pool,
                          const CostEstimateFn& estimate) {
  const std::size_t n = pool.clusters.size();
  AuctionBidder bidder;
  bidder.robot_id = bid.robot_id;
  bidder.from_pose.assign(n, kUnreachableCost);
  bidder.between.assign(n, std::vector<double>(n, kUnreachableCost));
  // The bid's own clusters at each pool cluster.
  std::vector<std::vector<std::size_t>> mine(n);
  for (std::size_t i = 0; i < bid.clusters.size() && i < bid_to_pool.size();
       ++i) {
    if (bid_to_pool[i] >= 0) mine[bid_to_pool[i]].push_back(i);
  }
  const auto estimated = [&estimate](const Eigen::Vector3d& from,
                                     const Eigen::Vector3d& to) {
    if (!estimate) return kUnreachableCost;
    const double cost = estimate(from, to);
    return std::isnan(cost) || cost < 0.0 ? kUnreachableCost : cost;
  };
  const Eigen::Vector3d at = bid.pose.head<3>();
  for (std::size_t p = 0; p < n; ++p) {
    if (mine[p].empty()) {
      bidder.from_pose[p] = estimated(at, pool.clusters[p].position);
      continue;
    }
    for (const std::size_t i : mine[p]) {
      bidder.from_pose[p] = std::min(bidder.from_pose[p],
                                     bid.costs_from_pose[i]);
    }
  }
  for (std::size_t p = 0; p < n; ++p) {
    bidder.between[p][p] = 0.0;
    for (std::size_t q = p + 1; q < n; ++q) {
      double cost = kUnreachableCost;
      if (!mine[p].empty() && !mine[q].empty()) {
        for (const std::size_t i : mine[p]) {
          for (const std::size_t j : mine[q]) {
            cost = std::min({cost, bid.costBetween(i, j),
                             bid.costBetween(j, i)});
          }
        }
      } else {
        cost = estimated(pool.clusters[p].position, pool.clusters[q].position);
      }
      bidder.between[p][q] = cost;
      bidder.between[q][p] = cost;
    }
  }
  if (bid.current_target != kNoCluster) {
    bidder.current_target = pool.indexOf(bid.current_target);
  }
  // A bidder that must keep flight to get home bids on nothing it could not
  // reach and still return from (drone scout design §4.3).
  if (std::isfinite(bid.reach_m)) {
    for (std::size_t p = 0; p < n; ++p) {
      const double back = estimated(pool.clusters[p].position, bid.home);
      if (bidder.from_pose[p] + back > bid.reach_m) {
        bidder.from_pose[p] = kUnreachableCost;
      }
    }
  }
  // Costs become time at the bidder's own speed, so a fast scout wins the
  // far clusters a slow robot would take long to reach. Every bid carries
  // one (TourBidData::wellFormed; the node bids at v_max); one without has
  // no time to compare and bids on nothing.
  if (!(std::isfinite(bid.speed_mps) && bid.speed_mps > 0.0)) {
    bidder.from_pose.assign(n, kUnreachableCost);
    return bidder;
  }
  for (std::size_t p = 0; p < n; ++p) {
    if (std::isfinite(bidder.from_pose[p])) {
      bidder.from_pose[p] /= bid.speed_mps;
    }
    for (std::size_t q = 0; q < n; ++q) {
      if (std::isfinite(bidder.between[p][q])) {
        bidder.between[p][q] /= bid.speed_mps;
      }
    }
  }
  return bidder;
}

AuctionResult runSequentialAuction(const std::vector<AuctionBidder>& input,
                                   const std::vector<bool>& fixed,
                                   double commit_margin,
                                   double balance_weight) {
  std::vector<const AuctionBidder*> bidders;
  for (const AuctionBidder& b : input) bidders.push_back(&b);
  std::sort(bidders.begin(), bidders.end(),
            [](const AuctionBidder* a, const AuctionBidder* b) {
              return a->robot_id < b->robot_id;
            });
  const std::size_t n = fixed.size();
  std::vector<bool> taken = fixed;
  std::map<int, std::vector<int>> order;
  std::map<int, bool> keeps_first;
  std::map<int, double> tour_cost;
  for (const AuctionBidder* b : bidders) {
    order[b->robot_id];
    keeps_first[b->robot_id] = false;
    tour_cost[b->robot_id] = 0.0;
  }

  // 1. Each bidder keeps its current target unless another's cost to it is
  // lower by more than the commit margin; of two heading for one cluster,
  // the closer keeps it (the lower ID on a tie).
  for (std::size_t t = 0; t < n; ++t) {
    if (taken[t]) continue;
    const AuctionBidder* keeper = nullptr;
    for (const AuctionBidder* b : bidders) {
      if (b->current_target == static_cast<int>(t) &&
          std::isfinite(b->from_pose[t]) &&
          (keeper == nullptr || b->from_pose[t] < keeper->from_pose[t])) {
        keeper = b;
      }
    }
    if (keeper == nullptr) continue;
    double best_other = kUnreachableCost;
    for (const AuctionBidder* b : bidders) {
      if (b != keeper) best_other = std::min(best_other, b->from_pose[t]);
    }
    if (best_other < (1.0 - commit_margin) * keeper->from_pose[t]) continue;
    order[keeper->robot_id].push_back(static_cast<int>(t));
    keeps_first[keeper->robot_id] = true;
    tour_cost[keeper->robot_id] = keeper->from_pose[t];
    taken[t] = true;
  }

  // 2. Sequential single-item auction over the rest.
  for (;;) {
    const AuctionBidder* winner = nullptr;
    int item = -1;
    Insertion where;
    double best_bid = kUnreachableCost;
    for (const AuctionBidder* b : bidders) {
      for (std::size_t c = 0; c < n; ++c) {
        // A cluster the bidder cannot reach from its pose (or, with limited
        // reach, not reach and return from: bidderCosts) is not its
        // candidate, however cheap the leg from one it already won.
        if (taken[c] || !std::isfinite(b->from_pose[c])) continue;
        const Insertion insertion = cheapestInsertion(
            order[b->robot_id], static_cast<int>(c), b->from_pose, b->between,
            keeps_first[b->robot_id]);
        if (!std::isfinite(insertion.added_cost)) continue;
        const double bid =
            insertion.added_cost + balance_weight * tour_cost[b->robot_id];
        if (bid < best_bid) {
          best_bid = bid;
          winner = b;
          item = static_cast<int>(c);
          where = insertion;
        }
      }
    }
    if (winner == nullptr) break;
    std::vector<int>& bundle = order[winner->robot_id];
    bundle.insert(bundle.begin() + where.position, item);
    taken[item] = true;
    tour_cost[winner->robot_id] =
        openTourCost(bundle, winner->from_pose, winner->between);
  }

  // Each bundle in the order its robot will tour it (§2.3), a kept target
  // first.
  AuctionResult result;
  for (const AuctionBidder* b : bidders) {
    std::vector<int> bundle = order[b->robot_id];
    if (bundle.size() > 1) {
      std::vector<double> from;
      std::vector<std::vector<double>> between;
      for (const int i : bundle) {
        from.push_back(b->from_pose[i]);
        between.emplace_back();
        for (const int j : bundle) between.back().push_back(b->between[i][j]);
      }
      const OpenTour tour =
          solveOpenTour(from, between, keeps_first[b->robot_id] ? 0 : -1);
      if (tour.order.size() == bundle.size()) {
        std::vector<int> ordered;
        for (const int k : tour.order) ordered.push_back(bundle[k]);
        bundle = std::move(ordered);
      }
    }
    result.bundles[b->robot_id] = std::move(bundle);
  }
  for (std::size_t c = 0; c < n; ++c) {
    if (!taken[c]) result.unassigned.push_back(static_cast<int>(c));
  }
  return result;
}

}  // namespace mgg
