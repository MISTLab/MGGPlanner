// The clusters other robots hold (tour-exploration design §4). A robot's
// claim is its last awarded bundle; nobody else gets those clusters. Silence
// is not failure: a claim stays after its robot falls silent, until
// fleet.claim_ttl_s after it was last heard, and is released earlier when a
// robot of the group finds a cluster explored (it simply stops being a
// frontier), when idle robots take the oldest silent claims over, or by the
// operator.

#ifndef MGG_CORE_FLEET_CLAIMS_H_
#define MGG_CORE_FLEET_CLAIMS_H_

#include <map>
#include <set>
#include <vector>

#include "mgg_core/fleet_types.h"

namespace mgg {

struct Claim {
  int robot_id = 0;
  std::vector<FleetCluster> clusters;
  /// When this robot, or the auctioneer it learned the claim from, last
  /// heard the holder.
  double last_heard_s = 0.0;
};

class ClaimRegistry {
 public:
  /// `robot_id` holds `clusters` as of an award, heard at `heard_s` (the
  /// later of this and any earlier time is kept). An empty bundle is no
  /// claim.
  void record(int robot_id, std::vector<FleetCluster> clusters,
              double heard_s);
  /// `robot_id` was heard: its claim, if any, is fresh again. `heard_s` is
  /// the local receipt time, not a peer timestamp; it replaces the previous
  /// time even when the local clock has moved backwards.
  void heard(int robot_id, double heard_s);
  /// Drops the claims of robots silent for more than `ttl_s`; returns their
  /// IDs. A last-heard time in the future (clock rollback or a future award
  /// time) is rebased to `now_s` without expiring on this call, granting a
  /// full TTL in the current clock epoch.
  std::vector<int> expire(double now_s, double ttl_s);
  /// Removes every claimed cluster at which `explored` holds; returns how
  /// many.
  int dropExplored(const ExploredFn& explored);
  /// The operator's release. False when there was no claim.
  bool release(int robot_id);
  /// Releases the claim of the candidate silent longest (the lower ID on a
  /// tie) and returns its robot ID, or -1 when no candidate holds one.
  int releaseOldest(const std::set<int>& candidates);

  const Claim* find(int robot_id) const;
  /// Holders in ascending robot ID.
  std::vector<int> robots() const;
  /// The claimed clusters of `robots`, in ascending holder ID, each claim's
  /// in its bundle order.
  std::vector<FleetCluster> clustersOf(const std::set<int>& robots) const;
  /// Every claimed cluster not held by `robot_id`, in the same order.
  std::vector<FleetCluster> clustersExcept(int robot_id) const;

 private:
  std::map<int, Claim> claims_;
};

}  // namespace mgg

#endif  // MGG_CORE_FLEET_CLAIMS_H_
