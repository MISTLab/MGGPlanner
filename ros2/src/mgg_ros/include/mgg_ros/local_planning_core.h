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

/// GlobalGuidance's standing_start block (mgg_msgs/StandingStart): the
/// guidance planner's proof that the robot stands in the disk where it
/// started. Center is in the guidance frame.
struct GuidanceStandingStart {
  bool valid = false;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 0;
  /// The guidance planner's incarnation; with LocalGuidance::sequence_id,
  /// the block's identity.
  std::uint64_t boot = 0;
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
  GuidanceStandingStart standing_start;
};

struct LocalFeedback {
  std::string session_id;
  std::uint64_t epoch = 0, sequence_id = 0;
  /// False when the executor drives no local path (state IDLE).
  bool executing = false;
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
  /// may answer before the local planner hears of the session); a held
  /// message never replaces a newer one, by (standing_start.boot,
  /// sequence_id).
  ///
  /// The standing_start block is fenced by its identity before it changes
  /// anything: a block no newer, by (boot, sequence_id), than the newest
  /// accepted is ignored (stale or repeated), and once a disk is applied
  /// only that disk's boot is heard. The current session's latest accepted
  /// valid block is applied to the ground layer and the planner while the
  /// robot's anchor stays in its disk, placed in the odometry frame at
  /// first application (needing a transform then) and never moved or
  /// enlarged after. It ends for good, never applied again by this core,
  /// when the anchor leaves the disk, odometry jumps, or an accepted block
  /// revokes it (valid false) after it was applied; a revocation before
  /// the first application is not final. A new session drops the block
  /// until that session's guidance brings one. Every change withdraws the
  /// ground layer, the certification cache and every retained path, as a
  /// map change of everything does.
  void setGuidance(const LocalGuidance& guidance);
  /// Zones in `frame_id` (empty: odometry), re-transformed every cycle.
  /// Retained paths entering them, or every retained path when the zones
  /// have no transform, are invalidated at once (takeInvalidations).
  void setNoGoZones(const NoGoZones& zones, const std::string& frame_id = {});
  /// Ignored unless it is for the current session and epoch.
  void onFeedback(const LocalFeedback& feedback);
  /// Resolves frames other than the odometry frame. Without it, only
  /// odometry-frame inputs can be planned with.
  void setFrameLookup(FrameLookup lookup) { lookup_ = std::move(lookup); }
  /// Replaces params.ground_recheck_s for later calls (replay, tests).
  void setGroundRecheckBudget(double seconds);
  /// One planning cycle. A path it returns carries the epoch, the next
  /// sequence id and the session; the core then treats it as executing
  /// until feedback names another path.
  LocalPlanResult plan(std::chrono::steady_clock::time_point deadline);
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
  double progress() const { return progress_; }
  std::uint64_t executingSequenceId() const { return executing_sequence_; }
  const RollingVoxelMap& map() const { return map_; }
  const GroundLayer& ground() const { return layer_; }
  /// Scans dropped because their robot pose or origin was not finite.
  std::uint64_t droppedScans() const { return dropped_scans_; }
  /// Retained paths re-certified because a map change reached them.
  std::uint64_t retainedRechecks() const { return retained_rechecks_; }
  /// The standing start applied now, in the odometry frame.
  std::optional<StandingStart> standingStart() const {
    return standing_applied_ ? std::optional<StandingStart>(standing_->odom)
                             : std::nullopt;
  }

 private:
  struct Retained {
    LocalPathPlan path;
    bool invalid = false;
  };
  /// The disk first applied, with the boot that proved it.
  struct AppliedStandingStart {
    std::uint64_t boot = 0;
    StandingStart odom;
  };
  /// An accepted standing_start block, with its guidance's frame.
  struct StandingBlock {
    GuidanceStandingStart block;
    std::string frame_id;
  };
  StateVec anchorOf(const StateVec& base) const;
  double floorUnder(const StateVec& anchor) const;
  /// Places the window on `anchor`; resets on a jump. Returns the change.
  MapChange place(const StateVec& anchor, bool* reset);
  void resetAll(const StateVec& anchor, MapChange& change);
  void afterChange(const MapChange& change);
  /// Takes the guidance's standing_start block unless the identity fence
  /// refuses it (setGuidance); ends an applied standing start it revokes.
  void acceptStandingStart(const LocalGuidance& guidance);
  /// Whether a block is valid with a finite centre and positive radius.
  static bool usable(const GuidanceStandingStart& block);
  /// Applies, keeps or revokes the standing start (setGuidance).
  void refreshStandingStart();
  /// The zones in the odometry frame; nullopt (naming `failed`) without a
  /// transform. Empty zones need none.
  std::optional<NoGoZones> zonesInOdom(std::string& failed) const;
  void withdrawAgainstZones(const std::optional<NoGoZones>& zones,
                            const std::string& failed);
  void invalidate(std::uint64_t sequence, const std::string& reason);
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
  /// The current session's latest accepted block.
  std::optional<StandingBlock> standing_block_;
  /// The newest (boot, sequence_id) a block was accepted with.
  std::optional<std::pair<std::uint64_t, std::uint64_t>> standing_identity_;
  /// Set at first application, then fixed.
  std::optional<AppliedStandingStart> standing_;
  bool standing_applied_ = false;
  /// The anchor left the disk, odometry jumped, or an applied standing
  /// start was revoked: never stand again.
  bool standing_start_ended_ = false;
  NoGoZones zones_;
  std::string zones_frame_;

  std::map<std::uint64_t, Retained> retained_;
  std::uint64_t next_sequence_ = 1;
  std::uint64_t executing_sequence_ = 0;
  /// Last executor acknowledgement, independent of optimistic publications.
  std::uint64_t fed_back_sequence_ = 0;
  double progress_ = 0, speed_mps_ = 0;
  BrakingBounds braking_;
  std::vector<LocalInvalidation> invalidations_;
  std::uint64_t dropped_scans_ = 0;
  std::uint64_t retained_rechecks_ = 0;
};

}  // namespace mgg

#endif  // MGG_ROS_LOCAL_PLANNING_CORE_H_
