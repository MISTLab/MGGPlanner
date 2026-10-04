#ifndef MGG_CORE_LOCAL_PLANNER_H_
#define MGG_CORE_LOCAL_PLANNER_H_
#include <cstddef>
#include <optional>

#include "mgg_core/certification_cache.h"
#include "mgg_core/ground_layer.h"
#include "mgg_core/local_path.h"
#include "mgg_core/local_planner_status.h"
#include "mgg_core/no_go_zones.h"
#include "mgg_core/recent_track.h"
#include "mgg_core/sensor_params.h"
namespace mgg {
inline constexpr double kLocalPlanningPeriodS = 0.5;
inline constexpr double kLocalPlanningBudgetS = 0.3;
struct LocalPlanInputs {
  // All positions are driving-height references in the same odometry frame.
  StateVec pose = StateVec::Zero();
  double speed_mps = 0, progress_m = 0;
  std::string session_id, request_id;
  uint64_t map_revision = 0, epoch = 0, sequence_id = 0,
           guidance_sequence_id = 0;
  std::optional<LocalPathPlan> executing_path;
  bool executing_invalid = false;
  std::optional<Eigen::Vector3d> target;
  std::vector<Eigen::Vector3d> coarse_route;
  double goal_tolerance_m = 0.2;
  NoGoZones no_go_zones;
  BrakingBounds braking;
};
struct LocalPlanResult {
  std::optional<LocalPathPlan> path;
  LocalStatus status = LocalStatus::kBlocked;
  std::string reason;
  bool checks_complete = false;
  double cycle_ms = 0;
  // Driver exposes this cap through its feedback/control boundary. It does
  // not replace the executor's instantaneous remaining-distance fence.
  std::optional<double> speed_cap_mps;
  // Destinations considered (the goal, or scored gain viewpoints); set only
  // when the search reached candidate selection. Telemetry, not policy.
  std::optional<std::size_t> candidate_count;
};
class LocalPlanner {
 public:
  LocalPlanner(const MapInterface&, const GroundLayer&, CertificationCache&,
               const PlanningParams&, const RobotParams&, const SensorParams&);
  LocalPlanResult plan(const LocalPlanInputs&, const MapChange&);
  // Driver withdraws map/ground/cache changes before this check. No pending
  // admission is reusable, even if the old path carries a dependency box.
  bool pathStillCertified(const LocalPathPlan&, double progress_m);
  // Node calls for every odometry sample, not only each planning cycle.
  void recordPose(const StateVec& pose) { track_.add(pose); }
  void reset() {
    track_.clear();
    cache_.flushAll();
  }

 private:
  LocalPlanResult search(const LocalPlanInputs&,
                         std::chrono::steady_clock::time_point);
  bool certify(LocalPathPlan&, const NoGoZones&);
  Eigen::AlignedBox3d dependency(const StateVec&, const StateVec&) const;
  double localGain(const StateVec&, const Eigen::AlignedBox3d&) const;
  const MapInterface& map_;
  const GroundLayer& layer_;
  CertificationCache& cache_;
  PlanningParams planning_;
  RobotParams robot_;
  SensorParams sensor_;
  DependencyHalos halos_;
  RecentTrack track_;
  NoGoZones zones_;
  uint64_t sequence_ = 0;
};
}  // namespace mgg
#endif
