#ifndef MGG_CORE_LOCAL_PLANNER_H_
#define MGG_CORE_LOCAL_PLANNER_H_
#include <chrono>
#include <cstddef>
#include <functional>
#include <optional>

#include "mgg_core/certification_cache.h"
#include "mgg_core/ground_layer.h"
#include "mgg_core/local_path.h"
#include "mgg_core/local_planner_status.h"
#include "mgg_core/no_go_zones.h"
#include "mgg_core/path_turns.h"
#include "mgg_core/recent_track.h"
#include "mgg_core/sensor_params.h"
namespace mgg {
inline constexpr double kLocalPlanningPeriodS = 0.5;
inline constexpr double kLocalPlanningBudgetS = 0.3;
/// Spec §4.3 as amended (v2-motionfix): a path of 6-10 m is preferred.
/// Without one, the best-ranked certified path at least one commitment long.
inline constexpr double kPreferredPathM = 6;
inline constexpr double kMaxPathM = 10;
/// Why the routed candidates of one search were not published. Telemetry,
/// not policy.
struct CandidateRefusals {
  /// No turn-compliant route reached the candidate.
  std::size_t unrouted = 0;
  /// Shorter than one commitment, or shorter than 6 m once a shorter
  /// certified path was kept.
  std::size_t too_short = 0;
  /// Its route did not pass the final certificate.
  std::size_t uncertified = 0;
};
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
  /// The driver's cycle deadline; the search also keeps its own budget
  /// (kLocalPlanningBudgetS). Both are budgets: a certified shorter path in
  /// hand is still published when either runs out. A cancellation scope
  /// around plan() (planning_cancelled) is external and publishes nothing.
  std::chrono::steady_clock::time_point deadline =
      std::chrono::steady_clock::time_point::max();
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
  // Gain viewpoints the search would score, and how many it scored within
  // its scoring slice; set only when gain scoring ran. Telemetry, not
  // policy: fewer scored than offered is "not every viewpoint scored".
  std::optional<std::size_t> viewpoints_offered, viewpoints_scored;
  // Set when the search reached candidate routing.
  std::optional<CandidateRefusals> refusals;
};
class LocalPlanner {
 public:
  LocalPlanner(const MapInterface&, const GroundLayer&, CertificationCache&,
               const PlanningParams&, const RobotParams&, const SensorParams&);
  LocalPlanResult plan(const LocalPlanInputs&, const MapChange&);
  // Driver withdraws map/ground/cache changes before this check. No pending
  // admission is reusable, even if the old path carries a dependency box.
  // The whole rest of the path, from progress_m on.
  bool pathStillCertified(const LocalPathPlan&, double progress_m);
  // Only the rest of its commitment (commit_length_m from the path's start):
  // all the executor may drive without a newer path. Failing this breaks
  // the commitment; failing only pathStillCertified withdraws the suffix.
  bool commitmentStillCertified(const LocalPathPlan&, double progress_m);
  // Node calls for every odometry sample, not only each planning cycle.
  void recordPose(const StateVec& pose) { track_.add(pose); }
  void reset() {
    track_.clear();
    cache_.flushAll();
  }
  /// The robot stands in the standing disk (StandingStart, which the v2
  /// local core centres on the robot): unobserved ground in it counts as
  /// ground, and from a root in the disk a departure may hang across it up
  /// to the disk's radius (hanging_root_edge_length_max) to observed
  /// ground, through the body band the lidar cannot see where that lies
  /// wholly in the disk
  /// (StandingStart::coversCell; occupied still blocks, and unknown body
  /// volume past the disk's edge blocks as anywhere). Turns use it as
  /// legacy does, and a path may not end in it (StandingStart::admitsGoal)
  /// unless its arrival disk is observed. The driver withdraws the cache
  /// where the disk was and is on a change.
  void setStandingStart(const std::optional<StandingStart>& standing) {
    standing_ = standing;
  }

 private:
  LocalPlanResult search(const LocalPlanInputs&,
                         std::chrono::steady_clock::time_point);
  bool certify(LocalPathPlan&, const NoGoZones&);
  /// Whether the planning footprint at `position` (driving height), facing
  /// `heading`, stays off every column the ground layer exports as refused.
  bool footprintOffRefusedTerrain(const Eigen::Vector3d& position,
                                  double heading) const;
  /// No planning footprint along the path, at most a map cell apart and
  /// turning on the spot at its vertices included, overlaps a column the
  /// ground layer exports as refused.
  bool footprintsOffRefusedTerrain(const LocalPathPlan&) const;
  Eigen::AlignedBox3d dependency(const StateVec&, const StateVec&) const;
  double localGain(const StateVec&, const Eigen::AlignedBox3d&) const;
  /// Whether a root at `pose` departs as a hanging root: in the standing
  /// start's disk, where the lidar sees neither the ground nor the body
  /// band near the robot, whether or not some ground under it is observed.
  bool hangingRoot(const StateVec& pose) const;
  /// Where too little ground is observed to measure the slope, the standing
  /// start's disk is level: its unobserved ground is the robot's floor.
  SlopeFn standingSlope() const;
  /// A path's last pose may not stop in the standing start's disk.
  bool standingArrivalAdmissible(const StateVec& end, double tolerance) const;
  const MapInterface& map_;
  const GroundLayer& layer_;
  CertificationCache& cache_;
  PlanningParams planning_;
  RobotParams robot_;
  /// The sensor's gain-only copy (groundGainSensor); visibility policy
  /// never reads it.
  SensorParams gain_sensor_;
  std::optional<StandingStart> standing_;
  DependencyHalos halos_;
  RecentTrack track_;
  NoGoZones zones_;
  uint64_t sequence_ = 0;
  /// plan()'s caller's cancellation, while plan() runs.
  const std::function<bool()>* external_cancelled_ = nullptr;
};
}  // namespace mgg
#endif
