#include "mgg_core/fleet_types.h"

#include <algorithm>
#include <cmath>
#include <set>

namespace mgg {

bool TourBidData::wellFormed() const {
  const std::size_t n = clusters.size();
  if (n > kMaxBidClusters || bundle.size() > kMaxBidClusters ||
      explored.size() > kMaxBidClusters) {
    return false;
  }
  if (costs_from_pose.size() != n || costs_between.size() != n * n) {
    return false;
  }
  if (!pose.allFinite()) return false;
  if (!std::isfinite(speed_mps) || !(speed_mps > 0.0)) return false;
  if (std::isnan(reach_m) || reach_m < 0.0 || !home.allFinite()) return false;
  for (const FleetCluster& cluster : clusters) {
    if (cluster.id == kNoCluster || !cluster.position.allFinite()) {
      return false;
    }
  }
  const auto unnamed = [](ClusterId id) { return id == kNoCluster; };
  if (std::any_of(bundle.begin(), bundle.end(), unnamed) ||
      std::any_of(explored.begin(), explored.end(), unnamed)) {
    return false;
  }
  const auto valid = [](double cost) {
    return !std::isnan(cost) && cost >= 0.0;
  };
  return std::all_of(costs_from_pose.begin(), costs_from_pose.end(), valid) &&
         std::all_of(costs_between.begin(), costs_between.end(), valid);
}

const RobotBundle* TourAwardData::bundleOf(int robot_id) const {
  for (const RobotBundle& bundle : bundles) {
    if (bundle.robot_id == robot_id) return &bundle;
  }
  return nullptr;
}

const FleetCluster* TourAwardData::cluster(ClusterId id) const {
  for (const FleetCluster& c : clusters) {
    if (c.id == id) return &c;
  }
  return nullptr;
}

bool TourAwardData::wellFormed() const {
  const auto finite = [](const FleetCluster& c) {
    return c.id == kNoCluster || c.position.allFinite();
  };
  if (!std::all_of(clusters.begin(), clusters.end(), finite) ||
      !std::all_of(explored.begin(), explored.end(), finite)) {
    return false;
  }
  std::set<ClusterId> assigned;
  for (const RobotBundle& bundle : bundles) {
    if (!std::isfinite(bundle.silent_s) || bundle.silent_s < 0.0) return false;
    std::set<ClusterId> ids(bundle.clusters.begin(), bundle.clusters.end());
    ids.erase(kNoCluster);
    for (const ClusterId id : ids) {
      if (cluster(id) == nullptr || !assigned.insert(id).second) return false;
    }
  }
  return true;
}

}  // namespace mgg
