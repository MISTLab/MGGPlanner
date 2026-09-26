#include "mgg_core/fleet_claims.h"

#include <algorithm>
#include <utility>

namespace mgg {

void ClaimRegistry::record(int robot_id, std::vector<FleetCluster> clusters,
                           double heard_s) {
  const auto found = claims_.find(robot_id);
  const double heard = found == claims_.end()
                           ? heard_s
                           : std::max(found->second.last_heard_s, heard_s);
  if (clusters.empty()) {
    if (found != claims_.end()) claims_.erase(found);
    return;
  }
  claims_[robot_id] = Claim{robot_id, std::move(clusters), heard};
}

void ClaimRegistry::heard(int robot_id, double heard_s) {
  const auto found = claims_.find(robot_id);
  if (found == claims_.end()) return;
  found->second.last_heard_s = heard_s;
}

std::vector<int> ClaimRegistry::expire(double now_s, double ttl_s) {
  std::vector<int> expired;
  for (auto it = claims_.begin(); it != claims_.end();) {
    if (now_s < it->second.last_heard_s) {
      // A clock reset (or future award time) must not pin the claim to the
      // old clock. Give it a full TTL starting in the current epoch.
      it->second.last_heard_s = now_s;
      ++it;
    } else if (now_s - it->second.last_heard_s > ttl_s) {
      expired.push_back(it->first);
      it = claims_.erase(it);
    } else {
      ++it;
    }
  }
  return expired;
}

int ClaimRegistry::dropExplored(const ExploredFn& explored) {
  if (!explored) return 0;
  int dropped = 0;
  for (auto it = claims_.begin(); it != claims_.end();) {
    std::vector<FleetCluster>& clusters = it->second.clusters;
    const std::size_t before = clusters.size();
    clusters.erase(std::remove_if(clusters.begin(), clusters.end(),
                                  [&explored](const FleetCluster& c) {
                                    return explored(c.position);
                                  }),
                   clusters.end());
    dropped += static_cast<int>(before - clusters.size());
    if (clusters.empty()) {
      it = claims_.erase(it);
    } else {
      ++it;
    }
  }
  return dropped;
}

bool ClaimRegistry::release(int robot_id) {
  return claims_.erase(robot_id) > 0;
}

int ClaimRegistry::releaseOldest(const std::set<int>& candidates) {
  auto oldest = claims_.end();
  for (auto it = claims_.begin(); it != claims_.end(); ++it) {
    if (candidates.count(it->first) == 0) continue;
    if (oldest == claims_.end() ||
        it->second.last_heard_s < oldest->second.last_heard_s) {
      oldest = it;
    }
  }
  if (oldest == claims_.end()) return -1;
  const int robot_id = oldest->first;
  claims_.erase(oldest);
  return robot_id;
}

const Claim* ClaimRegistry::find(int robot_id) const {
  const auto found = claims_.find(robot_id);
  return found == claims_.end() ? nullptr : &found->second;
}

std::vector<int> ClaimRegistry::robots() const {
  std::vector<int> ids;
  for (const auto& entry : claims_) ids.push_back(entry.first);
  return ids;
}

std::vector<FleetCluster> ClaimRegistry::clustersOf(
    const std::set<int>& robots) const {
  std::vector<FleetCluster> clusters;
  for (const auto& [robot_id, claim] : claims_) {
    if (robots.count(robot_id) == 0) continue;
    clusters.insert(clusters.end(), claim.clusters.begin(),
                    claim.clusters.end());
  }
  return clusters;
}

std::vector<FleetCluster> ClaimRegistry::clustersExcept(int robot_id) const {
  std::vector<FleetCluster> clusters;
  for (const auto& [holder, claim] : claims_) {
    if (holder == robot_id) continue;
    clusters.insert(clusters.end(), claim.clusters.begin(),
                    claim.clusters.end());
  }
  return clusters;
}

}  // namespace mgg
