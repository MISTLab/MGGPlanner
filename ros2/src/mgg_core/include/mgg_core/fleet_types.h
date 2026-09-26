// What robots exchange for fleet frontier assignment (tour-exploration
// design §3): bids, awards and the clusters they name, as plain structs.
// mgg_ros converts them to and from mgg_msgs/TourBid and TourAward at the
// frame boundary, as it does the roadmap exchange (graph_merge.h), so the
// auction runs in unit tests without ROS.
//
// Positions are in the holder's planning frame: a received bid or award has
// been placed with the neighbour transform before it gets here. Costs are
// frame-free.

#ifndef MGG_CORE_FLEET_TYPES_H_
#define MGG_CORE_FLEET_TYPES_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/frontier_clusters.h"
#include "mgg_core/types.h"

namespace mgg {

/// A bid listing more clusters than this is refused (wellFormed): its cost
/// matrix alone would be eight megabytes.
inline constexpr std::size_t kMaxBidClusters = 1024;

/// A frontier cluster as bids and awards carry it.
struct FleetCluster {
  ClusterId id = kNoCluster;
  int owner_robot_id = 0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double gain = 0.0;
};

/// One robot's bid (§3.3).
struct TourBidData {
  int robot_id = 0;
  std::uint64_t seq = 0;
  double stamp_s = 0.0;
  /// The auction it answers; 0 for a periodic bid.
  std::uint64_t auction_id = 0;
  StateVec pose = StateVec::Zero();
  /// Every frontier cluster the bidder knows.
  std::vector<FleetCluster> clusters;
  /// Per cluster, from the bidder's pose; kUnreachableCost when unreachable.
  std::vector<double> costs_from_pose;
  /// clusters x clusters, row-major.
  std::vector<double> costs_between;
  ClusterId current_target = kNoCluster;
  double claim_stamp_s = 0.0;
  std::vector<ClusterId> bundle;
  /// Awarded clusters the bidder's roadmap shows explored.
  std::vector<ClusterId> explored;
  /// The bidder's bundle is done and it asks for an auction (§3.2, §3.5).
  bool request_auction = false;

  double costBetween(std::size_t i, std::size_t j) const {
    return costs_between[i * clusters.size() + j];
  }
  /// At most kMaxBidClusters clusters, bundle entries and explored entries;
  /// cost arrays sized to the clusters; a finite pose and finite cluster
  /// positions; costs may be +inf, never NaN or negative. The IDs in
  /// clusters, bundle and explored must not be kNoCluster; current_target
  /// may be (no target). IDs are not checked against each other.
  bool wellFormed() const;
};

/// One robot's awarded clusters, in tour order (§3.4).
struct RobotBundle {
  int robot_id = 0;
  std::vector<ClusterId> clusters;
  /// How long the auctioneer has not heard this robot; 0 when it bid on
  /// time.
  double silent_s = 0.0;
};

/// An auction call (call = true, nothing else set) or an award (§3.4).
struct TourAwardData {
  std::uint64_t auction_id = 0;
  int auctioneer_id = 0;
  double stamp_s = 0.0;
  bool call = false;
  /// Every cluster the bundles name.
  std::vector<FleetCluster> clusters;
  std::vector<RobotBundle> bundles;
  /// Clusters dropped as explored by a robot of the group.
  std::vector<FleetCluster> explored;
  /// Silent robots whose claims were released (§4).
  std::vector<int> released_robot_ids;

  const RobotBundle* bundleOf(int robot_id) const;
  const FleetCluster* cluster(ClusterId id) const;
};

/// Whether the holder's own map or roadmap shows a position explored.
using ExploredFn = std::function<bool(const Eigen::Vector3d& position)>;
/// The auctioneer's estimate of a cost a bid does not give, on its merged
/// roadmap (§3.3); kUnreachableCost when it has none.
using CostEstimateFn =
    std::function<double(const Eigen::Vector3d& from, const Eigen::Vector3d& to)>;

}  // namespace mgg

#endif  // MGG_CORE_FLEET_TYPES_H_
