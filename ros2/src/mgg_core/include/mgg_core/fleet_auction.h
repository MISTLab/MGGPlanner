// The frontier auction's pool and award (tour-exploration design §3.3,
// §3.4), computed by the group's auctioneer from one round of bids.
//
// Pool: every bid's clusters and the clusters held by robots that did not
// bid, merged when closer than fleet.cluster_merge_radius_m (robots name one
// place differently: a robot names a peer's merged frontier by its own
// quantization), under the owner's own name when the owner bid it. One ID is
// one pool cluster: a report of an ID already pooled joins it wherever it
// lies. The owner's report of its own cluster places it, otherwise the first
// report in the pool's order (owners' reports first, then by ID, then bids
// as given, then held clusters). A cluster any bidder reports explored, or
// at which the auctioneer's own roadmap shows explored space, is dropped.
//
// Award: each bidder keeps its current target unless another bidder's cost
// to it is lower by more than tour.commit_margin; the rest go by sequential
// single-item auction. Each round every bidder bids for every open cluster
// its marginal tour cost (cheapest insertion into its bundle's tour) plus
// fleet.balance_weight times its bundle's tour cost; the lowest bid wins,
// ties to the lower robot ID. Held clusters are fixed: nobody gets them.

#ifndef MGG_CORE_FLEET_AUCTION_H_
#define MGG_CORE_FLEET_AUCTION_H_

#include <map>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/fleet_types.h"

namespace mgg {

struct ClusterPool {
  std::vector<FleetCluster> clusters;
  /// Every ID merged into each pool cluster.
  std::vector<std::vector<ClusterId>> member_ids;
  /// Per bid, in the order given: the pool index of each of its clusters, or
  /// -1 when it was dropped as explored.
  std::vector<std::vector<int>> bid_to_pool;
  /// The clusters dropped as explored.
  std::vector<FleetCluster> dropped_explored;
  /// Reports of a pooled ID farther than the merge radius from where the
  /// pool places it (a stale report or transform); they join it anyway.
  std::size_t conflicting_reports = 0;

  /// The pool cluster `id` was merged into, or -1.
  int indexOf(ClusterId id) const;
  /// The pool cluster nearest `position` within `radius_m`, or -1.
  int indexNear(const Eigen::Vector3d& position, double radius_m) const;
};

/// §3.3 pool construction from `bids` and the `held` clusters of robots that
/// did not bid. `explored` is the auctioneer's own check; null checks
/// nothing.
ClusterPool buildClusterPool(const std::vector<TourBidData>& bids,
                             const std::vector<FleetCluster>& held,
                             double merge_radius_m,
                             const ExploredFn& explored);

/// One bidder's costs over the pool.
struct AuctionBidder {
  int robot_id = 0;
  /// Per pool cluster, from the bidder's pose.
  std::vector<double> from_pose;
  /// Pool x pool, symmetric.
  std::vector<std::vector<double>> between;
  /// The pool index of its current target, or -1.
  int current_target = -1;
};

/// The bid's costs over the pool: its own where it lists the cluster (the
/// least of its entries merged into one), otherwise `estimate` on the
/// auctioneer's roadmap (kUnreachableCost without one).
AuctionBidder bidderCosts(const TourBidData& bid,
                          const std::vector<int>& bid_to_pool,
                          const ClusterPool& pool,
                          const CostEstimateFn& estimate);

struct AuctionResult {
  /// Pool indices each bidder won, in its tour order; every bidder has an
  /// entry, and a kept target stays first.
  std::map<int, std::vector<int>> bundles;
  /// Pool indices neither fixed nor reachable by any bidder.
  std::vector<int> unassigned;
};

/// §3.4 steps 1 and 2 over the pool clusters not `fixed`. A bidder bids
/// only on clusters its from_pose cost is finite for.
AuctionResult runSequentialAuction(const std::vector<AuctionBidder>& bidders,
                                   const std::vector<bool>& fixed,
                                   double commit_margin,
                                   double balance_weight);

}  // namespace mgg

#endif  // MGG_CORE_FLEET_AUCTION_H_
