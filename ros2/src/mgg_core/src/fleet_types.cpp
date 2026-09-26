#include "mgg_core/fleet_types.h"

#include <algorithm>
#include <cmath>

namespace mgg {

bool TourBidData::wellFormed() const {
  const std::size_t n = clusters.size();
  if (n > kMaxBidClusters) return false;
  if (costs_from_pose.size() != n || costs_between.size() != n * n) {
    return false;
  }
  if (!pose.allFinite()) return false;
  for (const FleetCluster& cluster : clusters) {
    if (cluster.id == kNoCluster || !cluster.position.allFinite()) {
      return false;
    }
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

}  // namespace mgg
