#include "mgg_core/tour_planner.h"

#include <algorithm>
#include <cmath>

#include "mgg_core/tour_solver.h"

namespace mgg {

bool TourPlanner::needsSolve(const std::vector<FrontierCluster>& clusters,
                             std::uint64_t graph_revision,
                             std::uint64_t assignment_version,
                             double now_s,
                             std::uint64_t peer_generation) const {
  if (!solved_ || released_) return true;
  std::set<ClusterId> ids;
  for (const FrontierCluster& cluster : clusters) ids.insert(cluster.id);
  if (target_ != kNoCluster && ids.count(target_) == 0) return true;
  const bool changed = graph_revision != graph_revision_ ||
                       peer_generation != peer_generation_ ||
                       assignment_version != assignment_version_ ||
                       ids != cluster_ids_;
  // A clock that went backwards (a simulation reset under use_sim_time)
  // counts as the interval elapsed.
  const bool interval_elapsed =
      now_s < solved_at_s_ ||
      now_s - solved_at_s_ >= params_.recompute_interval_s;
  return changed && interval_elapsed;
}

const TourPlan& TourPlanner::solve(const std::vector<FrontierCluster>& clusters,
                                   const TourCostMatrix& costs,
                                   std::uint64_t graph_revision,
                                   std::uint64_t assignment_version,
                                   double now_s,
                                   std::uint64_t peer_generation) {
  solved_ = true;
  released_ = false;
  graph_revision_ = graph_revision;
  peer_generation_ = peer_generation;
  assignment_version_ = assignment_version;
  solved_at_s_ = now_s;
  cluster_ids_.clear();
  for (const FrontierCluster& cluster : clusters) cluster_ids_.insert(cluster.id);

  const OpenTour best = solveOpenTour(costs.from_robot, costs.between);
  OpenTour chosen = best;
  bool kept = false;
  int current = -1;
  for (std::size_t i = 0; i < clusters.size(); ++i) {
    if (clusters[i].id == target_) current = static_cast<int>(i);
  }
  if (current >= 0 && !best.order.empty() && best.order.front() != current) {
    const OpenTour committed =
        solveOpenTour(costs.from_robot, costs.between, current);
    if (!committed.order.empty() && std::isfinite(committed.cost) &&
        !(best.cost < (1.0 - params_.commit_margin) * committed.cost)) {
      chosen = committed;
      kept = true;
    }
  }

  plan_ = TourPlan{};
  plan_.cost = chosen.cost;
  plan_.kept_target = kept;
  for (const int index : chosen.order) plan_.clusters.push_back(clusters[index]);
  const ClusterId next =
      plan_.clusters.empty() ? kNoCluster : plan_.clusters.front().id;
  if (next != target_) target_since_s_ = now_s;
  target_ = next;
  return plan_;
}

void TourPlanner::releaseTarget() {
  target_ = kNoCluster;
  released_ = true;
}

bool localPathServesTarget(const Eigen::Vector3d& robot,
                           const Eigen::Vector3d& viewpoint,
                           const Eigen::Vector3d& target,
                           const Eigen::Vector3d& lattice_min,
                           const Eigen::Vector3d& lattice_max,
                           const double reach_m) {
  const Eigen::Vector2d offset = (target - robot).head<2>();
  // Planar: the robot's pose and the lattice's viewpoints are at different
  // heights (its base, their driving height).
  const double from_robot = offset.norm();
  const double from_viewpoint = (target - viewpoint).head<2>().norm();
  const bool progress =
      from_robot - from_viewpoint >=
          std::min(kTowardTargetMinProgressM,
                   kTowardTargetMinFraction * from_robot) ||
      (from_viewpoint <= reach_m && from_viewpoint <= from_robot);
  if (!progress) return false;
  if (offset.x() >= lattice_min.x() && offset.x() <= lattice_max.x() &&
      offset.y() >= lattice_min.y() && offset.y() <= lattice_max.y()) {
    return true;
  }
  const Eigen::Vector2d step = (viewpoint - robot).head<2>();
  if (step.norm() < 1e-6 || offset.norm() < 1e-6) return false;
  const double angle =
      std::abs(std::remainder(std::atan2(step.y(), step.x()) -
                                  std::atan2(offset.y(), offset.x()),
                              2.0 * M_PI));
  return angle <= kTowardTargetMaxAngleRad;
}

}  // namespace mgg
