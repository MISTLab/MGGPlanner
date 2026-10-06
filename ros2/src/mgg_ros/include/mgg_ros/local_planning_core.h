// The ground local planning cycle without ROS: rolling voxels, the certified
// ground layer, the certification cache and the receding local planner, plus
// the session state the local planner node and its replay runner share.
//
// Every input arrives through one of the calls below, already in the
// odometry frame except where a frame is named: the driver (the node, or
// mgg_local_replay) owns transport and TF. Calls are not thread-safe; the
// driver serializes them.
//
// Poses passed to onScan/onOdometry are odometry base poses (x, y, z, yaw).
// The core plans at MGG's driving height, max_ground_height above the floor
// under the base; targets, routes and output paths use that height.

#ifndef MGG_ROS_LOCAL_PLANNING_CORE_H_
#define MGG_ROS_LOCAL_PLANNING_CORE_H_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "mgg_core/certification_cache.h"
#include "mgg_core/ground_layer.h"
#include "mgg_core/local_planner.h"
#include "mgg_map_octomap/rolling_voxel_map.h"

namespace mgg {

/// Published paths kept for re-certification. Paired with the adapter's
/// local_path.py RETAINED_PATHS ancestor-invalidation window.
inline constexpr std::size_t kMaxRetained = 8;

/// One scan in the odometry frame: its returns and the sensor's origin, both
/// from TF at the scan stamp.
struct OdomScan {
  std::vector<Eigen::Vector3d> points;
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
};

/// SetLocalPlannerMode's modes, same values as the service constants.
enum class LocalMode : std::uint8_t {
  kIdle = 0,
  kExplore = 1,
  kFollowRoute = 2
};

struct LocalModeRequest {
  std::string session_id, request_id;
  LocalMode mode = LocalMode::kIdle;
  /// Frame of goal and route; empty is the odometry frame.
  std::string frame_id;
  Eigen::Vector3d goal = Eigen::Vector3d::Zero();
  double tolerance_m = 0.2;
  std::vector<Eigen::Vector3d> route;
};

struct LocalModeResponse {
  bool accepted = false;
  std::string reason;
  std::uint64_t epoch = 0;
};

/// GlobalGuidance's kinds, same values as the message constants.
enum class GuidanceKind : std::uint8_t {
  kNone = 0,
  kTarget = 1,
  kComplete = 2
};

struct LocalGuidance {
  std::string session_id;
  std::uint64_t sequence_id = 0;
  GuidanceKind kind = GuidanceKind::kNone;
  /// Frame of target and route; empty is the odometry frame.
  std::string frame_id;
  Eigen::Vector3d target = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> route;
  std::string reason;
};

struct LocalFeedback {
  std::string session_id;
  std::uint64_t epoch = 0, sequence_id = 0;
  /// False when the executor drives no local path (state IDLE).
  bool executing = false;
  /// The executor refused path sequence_id (state REFUSED).
  bool refused = false;
  double progress_m = 0, speed_mps = 0;
  /// Braking bounds; non-positive or non-finite values keep the current ones.
  double deceleration_mps2 = 0, latency_s = 0, speed_cap_mps = 0;
};

/// A published path that is no longer certified.
struct LocalInvalidation {
  std::string session_id;
  std::uint64_t epoch = 0, sequence_id = 0, map_revision = 0;
  std::string reason;
};

struct LocalPlanningParams {
  RobotParams robot;
  PlanningParams planning;
  /// The sensor local gain is scored with.
  SensorParams sensor;
  RollingWindowParams window;
  /// The process epoch every output carries.
  std::uint64_t epoch = 0;
  /// Ground recheck budget after each map change and before each plan.
  double ground_recheck_s = 0.02;
  /// An odometry step longer than this resets map, ground, cache and track.
  double reset_jump_m = 1.0;
  /// The standing disk's radius (StandingStart), the planner node's
  /// hanging_root_edge_length_max: the lidar's ground blind radius. Each
  /// planning cycle centres the disk on the robot. 0 turns it off.
  double hanging_root_edge_length_max = 0;
};

/// odom_T_frame for a named frame, or nullopt when there is no transform.
using FrameLookup =
    std::function<std::optional<Eigen::Isometry3d>(const std::string& frame)>;

class LocalPlanningCore {
 public:
  /// Throws std::invalid_argument unless params.robot is a ground robot.
  explicit LocalPlanningCore(const LocalPlanningParams& params);

  /// Integrates one scan with the robot at base pose `robot`.
  MapChange onScan(const OdomScan& scan, const StateVec& robot);
  /// Moves the window and records the track; a jump resets everything.
  MapChange onOdometry(const StateVec& robot);
  LocalModeResponse setMode(const LocalModeRequest& request);
  /// The latest guidance of the current session wins. Guidance of another
  /// session is held until setMode starts that session (the global planner
  /// may answer before the local planner hears of the session).
  void setGuidance(const LocalGuidance& guidance);
  /// Zones in `frame_id` (empty: odometry), re-transformed every cycle.
  /// Retained paths entering them, or every retained path when the zones
  /// have no transform, are invalidated at once (takeInvalidations).
  void setNoGoZones(const NoGoZones& zones, const std::string& frame_id = {});
  /// Ignored unless it is for the current session and epoch.
  ///
  /// The executing path, which the next plan extends, follows from two
  /// monotone facts and nothing else: the acknowledged path (the executor's
  /// newest EXECUTING report; IDLE clears it, an older path is a stale
  /// report) and the publications it explicitly refused (REFUSED). It is
  /// the newest publication in flight (newer than every path ever
  /// acknowledged) and not refused, else the acknowledged path. Reports
  /// naming an older path, however many cross a publication in flight,
  /// move nothing back; only a refusal does. A refusal of path N also covers every undecided older
  /// publication (the executor decides paths in order and reports each
  /// acceptance before its next decision) and their descendants.
  void onFeedback(const LocalFeedback& feedback);
  /// Resolves frames other than the odometry frame. Without it, only
  /// odometry-frame inputs can be planned with.
  void setFrameLookup(FrameLookup lookup) { lookup_ = std::move(lookup); }
  /// Replaces params.ground_recheck_s for later calls (replay, tests).
  void setGroundRecheckBudget(double seconds);
  /// One planning cycle. A path it returns carries the epoch, the next
  /// sequence id and the session; the core then treats it as executing
  /// (onFeedback). It first centres the standing disk on the robot
  /// (standingStart).
  LocalPlanResult plan(std::chrono::steady_clock::time_point deadline);
  /// Whether a map change withdrew only the suffix of a retained path,
  /// beyond its still certified commitment, since the last call or plan:
  /// no invalidation (motion goes on within the commitment), but plan now.
  bool takeReplanRequest();
  /// Invalidations produced since the last call: paths withdrawn by map
  /// changes or resets, or the last feedback-confirmed executing path evicted
  /// by retention pruning. Retiring other paths is silent. Plan immediately
  /// after a non-empty result.
  std::vector<LocalInvalidation> takeInvalidations();

  const LocalPlanningParams& params() const { return params_; }
  std::uint64_t epoch() const { return params_.epoch; }
  const std::string& sessionId() const { return session_id_; }
  const std::string& requestId() const { return request_id_; }
  LocalMode mode() const { return mode_; }
  std::uint64_t guidanceSequenceId() const {
    return guidance_ ? guidance_->sequence_id : 0;
  }
  /// Progress along acknowledgedSequenceId(), as the executor reported it.
  double progress() const { return progress_; }
  std::uint64_t executingSequenceId() const { return executing_sequence_; }
  /// The path the executor last reported driving; 0 when idle.
  std::uint64_t acknowledgedSequenceId() const {
    return acknowledged_sequence_;
  }
  const RollingVoxelMap& map() const { return map_; }
  const GroundLayer& ground() const { return layer_; }
  /// Scans dropped because their robot pose or origin was not finite.
  std::uint64_t droppedScans() const { return dropped_scans_; }
  /// Retained paths re-certified because a map change reached them.
  std::uint64_t retainedRechecks() const { return retained_rechecks_; }
  /// Retained paths whose suffix a map change withdrew, their commitment
  /// still certified (takeReplanRequest).
  std::uint64_t suffixWithdrawals() const { return suffix_withdrawals_; }
  /// The standing disk applied now, in the odometry frame: legacy's
  /// standing start, round where the robot was at the last planning cycle,
  /// of radius hanging_root_edge_length_max. The ground layer seeds it as
  /// ground and the planner departs from a hanging root across it; observed
  /// drops, steps and occupied space still block. nullopt before the first
  /// cycle or with a zero radius.
  std::optional<StandingStart> standingStart() const { return standing_; }

 private:
  struct Retained {
    LocalPathPlan path;
    bool invalid = false;
    /// Only its commitment is still certified (takeReplanRequest).
    bool suffix_withdrawn = false;
    /// The executor refused it, or a path it extends (onFeedback).
    bool refused = false;
    /// The standing disk it was certified with: map changes recheck it
    /// with this disk, not the one that moved with the robot since.
    std::optional<StandingStart> standing;
  };
  StateVec anchorOf(const StateVec& base) const;
  double floorUnder(const StateVec& anchor) const;
  /// Places the window on `anchor`; resets on a jump. Returns the change.
  MapChange place(const StateVec& anchor, bool* reset);
  void resetAll(const StateVec& anchor, MapChange& change);
  void afterChange(const MapChange& change);
  /// Centres the standing disk on the anchor. Not a map change: the ground
  /// layer and the cache are withdrawn where the disk was and is, retained
  /// paths keep the disk they were certified with.
  void applyStandingDisk();
  /// The zones in the odometry frame; nullopt (naming `failed`) without a
  /// transform. Empty zones need none.
  std::optional<NoGoZones> zonesInOdom(std::string& failed) const;
  void withdrawAgainstZones(const std::optional<NoGoZones>& zones,
                            const std::string& failed);
  void invalidate(std::uint64_t sequence, const std::string& reason);
  /// The executing path from the acknowledged path and the refusals.
  void updateExecutingSequence();
  /// Progress along a retained path: the executor's, for the acknowledged
  /// path, else from its start.
  double progressOn(std::uint64_t sequence) const;
  /// Dependencies of the rest of the executing path's commitment, which
  /// ground rechecks take first.
  std::vector<Eigen::AlignedBox3d> commitmentDependencies() const;
  /// Ground recheck slices of ground_recheck_s_ each, ending no later than
  /// `deadline`: the commitment's pending work first, and a second slice
  /// while any of it is still pending.
  void recheckGround(std::chrono::steady_clock::time_point deadline);
  void clearSession();
  std::optional<Eigen::Isometry3d> resolve(const std::string& frame,
                                           std::string& failed) const;

  LocalPlanningParams params_;
  RollingVoxelMap map_;
  GroundLayer layer_;
  CertificationCache cache_;
  LocalPlanner planner_;
  FrameLookup lookup_;
  double ground_recheck_s_;

  bool have_pose_ = false;
  StateVec anchor_ = StateVec::Zero();

  std::string session_id_, request_id_;
  LocalMode mode_ = LocalMode::kIdle;
  LocalModeRequest request_;
  std::optional<LocalGuidance> guidance_;
  std::optional<LocalGuidance> early_guidance_;
  std::optional<StandingStart> standing_;
  NoGoZones zones_;
  std::string zones_frame_;

  std::map<std::uint64_t, Retained> retained_;
  std::uint64_t next_sequence_ = 1;
  /// What the next plan extends (onFeedback).
  std::uint64_t executing_sequence_ = 0;
  /// The executor's newest EXECUTING report; 0 after IDLE.
  std::uint64_t acknowledged_sequence_ = 0;
  /// The newest path the executor ever reported EXECUTING: publications
  /// after it are in flight.
  std::uint64_t acknowledged_high_ = 0;
  bool replan_requested_ = false;
  double progress_ = 0, speed_mps_ = 0;
  BrakingBounds braking_;
  std::vector<LocalInvalidation> invalidations_;
  std::uint64_t dropped_scans_ = 0;
  std::uint64_t retained_rechecks_ = 0;
  std::uint64_t suffix_withdrawals_ = 0;
};

}  // namespace mgg

#endif  // MGG_ROS_LOCAL_PLANNING_CORE_H_
