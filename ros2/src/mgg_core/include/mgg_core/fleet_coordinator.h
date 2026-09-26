// Fleet frontier assignment inside MGG (tour-exploration design §3, §4): who
// is in this robot's group, who runs the auction, and the one-round call,
// bid and award exchange. No central server: each connected group runs its
// own.
//
// A robot's group is itself and every robot it heard (a bid or an award)
// within fleet.peer_timeout_s. The ROS layer delivers only messages from
// robots it holds a neighbour transform to, placed in this robot's frame, so
// a robot without a shared frame is alone and tours alone. The auctioneer is
// the group's lowest robot ID that is a candidate: this robot itself, a
// robot not yet heard naming an auctioneer, or one whose latest message
// names itself (its bid's auctioneer_id is its own ID, or it sent a call or
// award). A robot whose bid names another follows another auctioneer and is
// no candidate: in a chain 1-2-3 where 3 does not hear 1, robot 2 follows 1
// and robot 3 auctions itself, respecting the claims it knows, rather than
// wait for robot 2. Every bid names the auctioneer its sender follows.
// The auctioneer calls an auction when the pool's clusters,
// the membership, a robot's request or an operator release changed, at most
// every fleet.auction_interval_s; takes one bid per member for
// fleet.bid_deadline_s; and awards bundles to the bidders, with the claims
// of members that did not bid and of silent robots fixed.
//
// Silence is not failure (§4): a silent robot's claims stay excluded for
// others until fleet.claim_ttl_s after it was last heard, a robot finds them
// explored, an idle robot takes the oldest over, or the operator releases
// them.

#ifndef MGG_CORE_FLEET_COORDINATOR_H_
#define MGG_CORE_FLEET_COORDINATOR_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include "mgg_core/fleet_auction.h"
#include "mgg_core/fleet_claims.h"
#include "mgg_core/fleet_types.h"
#include "mgg_core/tour_params.h"

namespace mgg {

/// This robot's bid content now: pose, clusters and costs, current target
/// and claim stamp, explored clusters. The coordinator fills in the rest.
using OwnBidFn = std::function<TourBidData()>;

struct FleetTickOutput {
  /// To broadcast to peers.
  std::optional<TourBidData> bid;
  /// A call or an award, to broadcast to peers.
  std::optional<TourAwardData> award;
};

/// Clusters peers explored, remembered at most this many (the oldest go).
inline constexpr std::size_t kMaxExploredElsewhere = 4096;

class FleetCoordinator {
 public:
  FleetCoordinator(int robot_id, const FleetParams& params,
                   double commit_margin);

  /// A peer's bid, in this robot's frame. Malformed bids are dropped. A bid
  /// answering this robot's call is collected only if received before
  /// fleet.bid_deadline_s has passed. The bid's current target and bundle
  /// replace the bidder's claim (§4) if the bid is newer than the message
  /// the claim came from: a bid sent before an award arrives after it
  /// changes nothing but the time the bidder was heard. The clusters
  /// it reports explored are kept for this robot's next auction (at most
  /// kMaxExploredElsewhere, the oldest go); one the last award named makes
  /// an auction due.
  void onBid(const TourBidData& bid, double now_s);
  /// A peer's call or award, in this robot's frame. A malformed one (a
  /// non-finite position, a non-finite or negative silence, a bundle ID no
  /// cluster entry lists, a cluster in two bundles) is dropped whole;
  /// kNoCluster entries are only filtered out. Only this robot's current
  /// auctioneer is followed. A call or award is known by its round,
  /// (auctioneer, stamp): a call is answered once per round, and an award is
  /// applied only when its round is newer than the last one applied from
  /// that auctioneer (a replayed or overtaken award is stale). A restarted
  /// auctioneer reuses auction IDs but not stamps.
  void onAward(const TourAwardData& award, double now_s);
  /// One step. Returns this robot's bid when an auction was called, every
  /// fleet.auction_interval_s, and at once when it requests an auction; as
  /// auctioneer, a call when one is due and the award once the bid deadline
  /// has passed (applied here too). `own_bid` is called only for a bid.
  FleetTickOutput tick(double now_s, const OwnBidFn& own_bid,
                       const CostEstimateFn& estimate,
                       const ExploredFn& explored);

  /// §3.5: this robot's bundle is done; its bids ask for an auction. Any
  /// award the auctioneer asked applies after the request went out answers
  /// it; when the auctioneer changes first, the request goes to the new one.
  void requestAuction();
  /// A request is out and no award has answered it yet.
  bool awaitingAuction() const { return requested_ && !answered_; }
  /// The auctioneer answered the request with an award that left this robot
  /// nothing.
  bool requestAnswered() const { return requested_ && answered_; }
  /// §4 release 3, on the auctioneer only: `robot_id`'s claims are released
  /// here and forwarded in the next award. False when this robot is not the
  /// group's auctioneer.
  bool releaseClaims(int robot_id, double now_s);
  /// §3.5 and §4 release 2 for a robot alone: the claim of the robot silent
  /// longest is released so this robot may take its clusters over. Returns
  /// that robot's ID, or -1 when no silent robot holds one.
  int takeOverOldestClaim(double now_s);

  /// This robot and every robot heard within fleet.peer_timeout_s, sorted.
  /// A robot last heard after `now_s` (the clock was reset) counts as just
  /// heard.
  std::vector<int> group(double now_s) const;
  /// The group's lowest candidate (see the top of this file).
  int auctioneer(double now_s) const;
  bool inGroup(double now_s) const { return group(now_s).size() > 1; }
  bool hasAward() const { return has_award_; }
  /// This robot's bundle from the last award it applied.
  const std::vector<FleetCluster>& bundle() const { return bundle_; }
  /// Every cluster other robots hold, stale claims expired first.
  std::vector<FleetCluster> claimedByOthers(double now_s);
  /// Clusters awards reported explored by some robot.
  const std::vector<FleetCluster>& exploredElsewhere() const {
    return explored_elsewhere_;
  }
  /// The clusters the last applied award named.
  const std::vector<FleetCluster>& lastAwardClusters() const {
    return award_clusters_;
  }
  /// Pool clusters no bidder could reach in this robot's last auction.
  std::size_t lastUnassigned() const { return last_unassigned_; }
  /// Changes whenever this robot's bundle or the claims it respects may
  /// have changed; the tour solves again.
  std::uint64_t assignmentVersion() const { return assignment_version_; }

 private:
  static constexpr double kNever = -std::numeric_limits<double>::infinity();

  /// A call's or award's identity: its auctioneer and stamp.
  struct Round {
    int auctioneer_id = -1;
    double stamp_s = kNever;
    bool operator==(const Round& other) const {
      return auctioneer_id == other.auctioneer_id && stamp_s == other.stamp_s;
    }
    bool operator!=(const Round& other) const { return !(*this == other); }
  };

  struct Collection {
    std::uint64_t auction_id = 0;
    double started_s = 0.0;
    std::vector<int> members;
    std::set<ClusterId> signature;
    std::map<int, TourBidData> bids;
  };

  /// Clock rollback (a simulation restart moves now_s backwards): a time
  /// after `now_s` becomes `now_s`, as ClaimRegistry::expire does for claims,
  /// so a silent robot counts as just heard once and then ages normally, and
  /// the next bid, award or call waits at most one interval or deadline. The
  /// rounds applied and answered are forgotten, as their stamps are of the
  /// old clock, and a claim's source stamp in the future becomes `now_s`.
  /// Called first by every member that takes the time.
  void rebaseFutureTimes(double now_s);
  /// `robot_id` was heard at the local receipt time `now_s`.
  void noteHeard(int robot_id, double now_s);
  /// The bidder holds its current target and bundle, as named by its own
  /// clusters, the last award or its claim; IDs none names are skipped. A
  /// bundle cut at kMaxBidClusters keeps the rest of the claim. Only a bid
  /// newer than the claim's source replaces it.
  void recordBidClaim(const TourBidData& bid, double now_s);
  /// Keeps explored IDs a bid reports for the next auction.
  void noteReportedExplored(const std::vector<ClusterId>& ids);
  void noteExplored(const std::vector<FleetCluster>& clusters);
  /// Cuts this robot's own bid to what peers accept (kMaxBidClusters per
  /// list): the bundle keeps its first clusters, the ones visited next; a
  /// longer explored list is sent in turns, the next entries each bid from
  /// `explored_turn`, so every report goes out. Broadcast bids and the bids
  /// this robot collects in its own auctions take turns of their own.
  void capBidLists(TourBidData& bid, std::size_t& explored_turn);
  void applyAward(const TourAwardData& award, double now_s);
  TourAwardData computeAward(double now_s, const CostEstimateFn& estimate,
                             const ExploredFn& explored);
  /// The cluster IDs the members' latest bids name: the pool's makeup.
  std::set<ClusterId> poolSignature(const std::vector<int>& members) const;

  int robot_id_;
  FleetParams params_;
  double commit_margin_;

  std::map<int, double> last_heard_s_;
  std::map<int, TourBidData> last_bids_;
  // The auctioneer each robot's latest bid, call or award names.
  std::map<int, int> follows_;
  // Per robot, the stamp of the bid or award its claim last came from. A bid
  // replaces the claim only when newer; an award unless older (an award
  // wins a tie: a bid sent at its stamp did not know it).
  std::map<int, double> claim_stamp_s_;
  ClaimRegistry claims_;
  std::vector<FleetCluster> bundle_;
  std::vector<FleetCluster> award_clusters_;
  std::vector<FleetCluster> explored_elsewhere_;
  bool has_award_ = false;
  std::uint64_t assignment_version_ = 0;
  std::size_t last_unassigned_ = 0;

  // The stamp of the last award applied, per auctioneer; the local time a
  // round was last heard (a later now_s before it means a clock reset).
  std::map<int, double> applied_stamp_s_;
  double round_heard_s_ = kNever;

  // Bidding.
  std::uint64_t seq_ = 0;
  double last_bid_s_ = kNever;
  std::uint64_t called_auction_ = 0;
  std::optional<Round> called_round_;
  std::optional<Round> answered_round_;
  std::set<ClusterId> own_cluster_ids_;
  std::size_t broadcast_explored_turn_ = 0;
  std::size_t collected_explored_turn_ = 0;
  bool requested_ = false;
  bool request_sent_ = false;
  bool answered_ = false;
  // The auctioneer the request went to.
  int request_auctioneer_ = -1;

  // Auctioneering.
  std::optional<Collection> collecting_;
  std::uint64_t auction_counter_ = 0;
  double last_auction_s_ = kNever;
  std::vector<int> auctioned_members_;
  std::set<ClusterId> auctioned_signature_;
  bool peer_requested_ = false;
  // Cluster IDs bids reported explored since the last auction, oldest
  // first, and whether one of them was named by the last award.
  std::vector<ClusterId> reported_explored_;
  bool explored_reported_ = false;
  std::vector<int> pending_releases_;
};

}  // namespace mgg

#endif  // MGG_CORE_FLEET_COORDINATOR_H_
