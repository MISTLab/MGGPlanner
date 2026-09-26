// The per-robot tour (tour-exploration design §2.3, §2.4): when it is solved
// again, and the commitment that keeps the robot on its target.
//
// The robot's current target is the first cluster of its tour. It is kept
// until reached, explored (its cluster vanishes) or reassigned (the
// assignment version changes and the cluster leaves the robot's set); a new
// first cluster replaces it only when it lowers the remaining tour cost by
// more than tour.commit_margin.

#ifndef MGG_CORE_TOUR_PLANNER_H_
#define MGG_CORE_TOUR_PLANNER_H_

#include <cstdint>
#include <set>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/frontier_clusters.h"
#include "mgg_core/tour_costs.h"
#include "mgg_core/tour_params.h"

namespace mgg {

struct TourPlan {
  /// In tour order; the first is the current target.
  std::vector<FrontierCluster> clusters;
  double cost = 0.0;
  /// The commitment kept a target the solver would have replaced.
  bool kept_target = false;
};

class TourPlanner {
 public:
  explicit TourPlanner(const TourParams& params) : params_(params) {}

  /// §2.3: at once before the first solve, when the target's cluster is
  /// gone, or after releaseTarget; otherwise when the graph revision, the
  /// assignment version or the set of cluster IDs changed, and at least
  /// tour.recompute_interval_s after the last solve. A `now_s` before the
  /// last solve (the clock went backwards, as a simulation reset under
  /// use_sim_time does) counts as the interval elapsed.
  bool needsSolve(const std::vector<FrontierCluster>& clusters,
                  std::uint64_t graph_revision,
                  std::uint64_t assignment_version, double now_s) const;
  /// Solves the tour over `clusters` with `costs` (indexed alike) and applies
  /// the commitment rule.
  const TourPlan& solve(const std::vector<FrontierCluster>& clusters,
                        const TourCostMatrix& costs,
                        std::uint64_t graph_revision,
                        std::uint64_t assignment_version, double now_s);
  /// The target was reached, or no route to it exists: the next solve
  /// chooses freely.
  void releaseTarget();

  ClusterId target() const { return target_; }
  /// When the current target was taken (the bid's claim stamp).
  double targetSince() const { return target_since_s_; }
  const TourPlan& plan() const { return plan_; }

 private:
  TourParams params_;
  TourPlan plan_;
  ClusterId target_ = kNoCluster;
  double target_since_s_ = 0.0;
  bool solved_ = false;
  bool released_ = false;
  std::uint64_t graph_revision_ = 0;
  std::uint64_t assignment_version_ = 0;
  std::set<ClusterId> cluster_ids_;
  double solved_at_s_ = 0.0;
};

/// A local path's viewpoint serves the target when it heads within this of
/// the target's bearing from the robot.
inline constexpr double kTowardTargetMaxAngleRad = 1.0471975511965976;  // pi/3

/// §2.4: whether the local exploration path ending at `viewpoint` serves the
/// tour's `target`: the target lies within the lattice box around the robot
/// (`lattice_min`/`lattice_max`, offsets in x and y, as the grid graph lays
/// it out), or the viewpoint lies within kTowardTargetMaxAngleRad of the
/// target's bearing.
bool localPathServesTarget(const Eigen::Vector3d& robot,
                           const Eigen::Vector3d& viewpoint,
                           const Eigen::Vector3d& target,
                           const Eigen::Vector3d& lattice_min,
                           const Eigen::Vector3d& lattice_max);

}  // namespace mgg

#endif  // MGG_CORE_TOUR_PLANNER_H_
