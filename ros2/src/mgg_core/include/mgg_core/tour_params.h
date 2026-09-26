// Parameters of tour-based exploration and fleet frontier assignment
// (ros2/docs/2026-09-25-tour-exploration-design.md §5).
//
// Plain structs the host fills in, as params.h is; mgg_ros loads them from
// the `tour.` and `fleet.` ROS parameters (loadTourParams, loadFleetParams)
// and clamps them here, so a mistyped YAML value cannot divide by zero or
// switch commitment off by accident.

#ifndef MGG_CORE_TOUR_PARAMS_H_
#define MGG_CORE_TOUR_PARAMS_H_

#include <algorithm>
#include <cmath>

namespace mgg {

struct TourParams {
  /// Use the tour instead of low-gain-triggered greedy repositioning.
  bool enabled = true;
  /// Clusters whose representative's gain is below this are dropped. Tuned
  /// in the SubT simulation; the starting value is ten unknown voxels at the
  /// deployed unknown_voxel_gain of 60.
  double min_cluster_gain = 600.0;
  /// Grid a representative's position is quantized on for its stable ID,
  /// metres.
  double cluster_id_cell_m = 1.0;
  /// Cost of the first leg's heading change, metres per radian. Tuned; the
  /// starting value makes a U-turn cost about 6 m of driving.
  double heading_weight = 2.0;
  /// Minimum interval between tour solves, seconds.
  double recompute_interval_s = 1.0;
  /// Fraction of the remaining tour cost a new first cluster must save to
  /// replace the current target.
  double commit_margin = 0.2;
  /// A cluster the robot could not be routed to is left out of the tour
  /// for at most this long, seconds; sooner when the robot moves or the
  /// graph changes.
  double route_retry_s = 30.0;
};

struct FleetParams {
  /// Bid, run and follow frontier auctions.
  bool enabled = true;
  /// Clusters closer than this are one: in the tour's clustering, in the
  /// auction pool, and when matching a cluster to a claim, metres.
  double cluster_merge_radius_m = 2.0;
  /// Balance penalty on a bidder's bundle tour cost. Tuned.
  double balance_weight = 0.3;
  /// Minimum interval between auctions, and between periodic bids, seconds.
  double auction_interval_s = 2.0;
  /// How long the auctioneer waits for bids, seconds.
  double bid_deadline_s = 1.0;
  /// A robot not heard for this long leaves the group (its claims stay,
  /// see claim_ttl_s), seconds.
  double peer_timeout_s = 5.0;
  /// Claim lifetime of a silent peer, seconds (SwarmDeck SubT sim: 600).
  double claim_ttl_s = 1800.0;
};

/// Smallest ID cell and merge radius, metres: below this a cluster's ID
/// changes with every centimetre its representative moves.
inline constexpr double kMinClusterCellM = 0.05;

namespace detail {
inline double finiteOr(double value, double fallback) {
  return std::isfinite(value) ? value : fallback;
}
}  // namespace detail

/// Non-finite values take the default; lengths, gains and times are at least
/// their floor; commit_margin lies in [0, 0.95].
inline void clampTourParams(TourParams& p) {
  const TourParams d;
  p.min_cluster_gain =
      std::max(0.0, detail::finiteOr(p.min_cluster_gain, d.min_cluster_gain));
  p.cluster_id_cell_m = std::max(
      kMinClusterCellM, detail::finiteOr(p.cluster_id_cell_m,
                                         d.cluster_id_cell_m));
  p.heading_weight =
      std::max(0.0, detail::finiteOr(p.heading_weight, d.heading_weight));
  p.recompute_interval_s = std::max(
      0.0, detail::finiteOr(p.recompute_interval_s, d.recompute_interval_s));
  p.commit_margin = std::clamp(
      detail::finiteOr(p.commit_margin, d.commit_margin), 0.0, 0.95);
  p.route_retry_s =
      std::max(0.0, detail::finiteOr(p.route_retry_s, d.route_retry_s));
}

inline void clampFleetParams(FleetParams& p) {
  const FleetParams d;
  p.cluster_merge_radius_m = std::max(
      kMinClusterCellM, detail::finiteOr(p.cluster_merge_radius_m,
                                         d.cluster_merge_radius_m));
  p.balance_weight =
      std::max(0.0, detail::finiteOr(p.balance_weight, d.balance_weight));
  p.auction_interval_s = std::max(
      0.0, detail::finiteOr(p.auction_interval_s, d.auction_interval_s));
  p.bid_deadline_s =
      std::max(0.0, detail::finiteOr(p.bid_deadline_s, d.bid_deadline_s));
  p.peer_timeout_s =
      std::max(0.1, detail::finiteOr(p.peer_timeout_s, d.peer_timeout_s));
  p.claim_ttl_s =
      std::max(0.0, detail::finiteOr(p.claim_ttl_s, d.claim_ttl_s));
}

}  // namespace mgg

#endif  // MGG_CORE_TOUR_PARAMS_H_
