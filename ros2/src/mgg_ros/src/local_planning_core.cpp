#include "mgg_ros/local_planning_core.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "mgg_core/planning_cancellation.h"

namespace mgg {
namespace {

using Clock = std::chrono::steady_clock;

Clock::time_point after(double seconds) {
  return Clock::now() + std::chrono::duration_cast<Clock::duration>(
                            std::chrono::duration<double>(seconds));
}

const LocalPlanningParams& validated(const LocalPlanningParams& params) {
  if (params.robot.type != RobotType::kGroundRobot)
    throw std::invalid_argument(
        "LocalPlanningCore plans for ground robots only");
  if (!std::isfinite(params.ground_recheck_s) || params.ground_recheck_s < 0 ||
      !std::isfinite(params.reset_jump_m) || params.reset_jump_m <= 0)
    throw std::invalid_argument(
        "LocalPlanningCore: ground_recheck_s must be finite and non-negative, "
        "reset_jump_m finite and positive");
  return params;
}

void merge(MapChange& into, const MapChange& change) {
  into.revision = change.revision;
  into.everything |= change.everything;
  into.boxes.insert(into.boxes.end(), change.boxes.begin(), change.boxes.end());
}

bool finite(const std::vector<Eigen::Vector3d>& points) {
  return std::all_of(points.begin(), points.end(),
                     [](const Eigen::Vector3d& p) { return p.allFinite(); });
}

std::vector<Eigen::Vector3d> transformed(
    const Eigen::Isometry3d& T, const std::vector<Eigen::Vector3d>& points) {
  std::vector<Eigen::Vector3d> out;
  out.reserve(points.size());
  for (const auto& p : points) out.push_back(T * p);
  return out;
}

}  // namespace

LocalPlanningCore::LocalPlanningCore(const LocalPlanningParams& params)
    : params_(validated(params)),
      map_(params_.window),
      layer_(map_, params_.planning, params_.robot),
      cache_(dependencyHalos(
          params_.robot, params_.planning, map_.getResolution(),
          GroundProjection(map_, params_.planning).max_projection_length)),
      planner_(map_, layer_, cache_, params_.planning, params_.robot,
               params_.sensor),
      ground_recheck_s_(params_.ground_recheck_s) {}

StateVec LocalPlanningCore::anchorOf(const StateVec& base) const {
  // The base sits half the body height over its floor; MGG drives at
  // max_ground_height over that floor (PlannerNode does the same).
  StateVec anchor = base;
  anchor[2] += params_.planning.max_ground_height - params_.robot.size.z() / 2;
  return anchor;
}

double LocalPlanningCore::floorUnder(const StateVec& anchor) const {
  return anchor.z() - params_.planning.max_ground_height;
}

void LocalPlanningCore::resetAll(const StateVec& anchor, MapChange& change) {
  merge(change, map_.reset(anchor.head<3>()));
  layer_.reset(anchor.head<3>(), floorUnder(anchor));
  cache_.flushAll();
  planner_.reset();
  // After the reset: the invalidations carry the revision that withdrew them.
  for (auto& entry : retained_)
    invalidate(entry.first, "odometry jump reset the local map");
}

MapChange LocalPlanningCore::place(const StateVec& anchor, bool* reset) {
  MapChange change;
  change.revision = map_.revision();
  const bool jump =
      !have_pose_ ||
      (anchor.head<3>() - anchor_.head<3>()).norm() > params_.reset_jump_m;
  if (jump) resetAll(anchor, change);
  if (reset) *reset = jump;
  anchor_ = anchor;
  have_pose_ = true;
  return change;
}

MapChange LocalPlanningCore::onOdometry(const StateVec& robot) {
  if (!robot.allFinite()) return MapChange{map_.revision(), {}, false};
  const StateVec anchor = anchorOf(robot);
  bool reset = false;
  MapChange change = place(anchor, &reset);
  if (!reset) {
    merge(change, map_.recenter(anchor.head<3>()));
    layer_.recenter(anchor.head<3>(), floorUnder(anchor));
  }
  planner_.recordPose(anchor);
  afterChange(change);
  return change;
}

MapChange LocalPlanningCore::onScan(const OdomScan& scan,
                                    const StateVec& robot) {
  if (!robot.allFinite() || !scan.origin.allFinite()) {
    ++dropped_scans_;
    return MapChange{map_.revision(), {}, false};
  }
  const StateVec anchor = anchorOf(robot);
  bool reset = false;
  MapChange change = place(anchor, &reset);
  std::vector<Eigen::Vector3d> points;
  points.reserve(scan.points.size());
  for (const auto& p : scan.points)
    if (p.allFinite()) points.push_back(p);
  merge(change, map_.insertScan(points, scan.origin, anchor.head<3>()));
  layer_.recenter(anchor.head<3>(), floorUnder(anchor));
  afterChange(change);
  return change;
}

void LocalPlanningCore::afterChange(const MapChange& change) {
  const bool changed = change.everything || !change.boxes.empty();
  if (changed) {
    layer_.withdraw(change);
    cache_.withdraw(change);
  }
  // Recheck now, so retained paths are judged on completed certification
  // where the budget allows; whatever stays pending fails them below.
  layer_.recheck(after(ground_recheck_s_));
  // Ground withdrawals reach beyond the change: a withdrawn flood parent
  // withdraws its descendants, and so does a scroll or a re-seed. Whatever
  // is still pending counts as changed for the paths depending on it.
  const bool pending = layer_.pendingCount() > 0;
  if (!changed && !pending) return;
  for (auto& [sequence, retained] : retained_) {
    if (retained.invalid) continue;
    const auto& deps = retained.path.edge_dependencies;
    const auto reaches = [&](const Eigen::AlignedBox3d& box) {
      return changed && changeReaches(change, box);
    };
    bool reached = change.everything || deps.empty() ||
                   std::any_of(deps.begin(), deps.end(), reaches);
    if (!reached && pending) {
      Eigen::AlignedBox3d all;
      for (const auto& box : deps) all.extend(box);
      reached = layer_.pending(all) &&
                std::any_of(deps.begin(), deps.end(),
                            [&](const Eigen::AlignedBox3d& box) {
                              return layer_.pending(box);
                            });
    }
    if (!reached) continue;
    ++retained_rechecks_;
    const double progress = sequence == executing_sequence_ ? progress_ : 0.0;
    if (!planner_.pathStillCertified(retained.path, progress))
      invalidate(sequence, "map change withdrew the path's certification");
  }
}

void LocalPlanningCore::setGroundRecheckBudget(double seconds) {
  if (!std::isfinite(seconds) || seconds < 0)
    throw std::invalid_argument("ground recheck budget must be finite, >= 0");
  ground_recheck_s_ = seconds;
}

void LocalPlanningCore::invalidate(std::uint64_t sequence,
                                   const std::string& reason) {
  auto found = retained_.find(sequence);
  if (found == retained_.end() || found->second.invalid) return;
  found->second.invalid = true;
  invalidations_.push_back(LocalInvalidation{
      session_id_, params_.epoch, sequence, map_.revision(), reason});
}

void LocalPlanningCore::clearSession() {
  retained_.clear();
  executing_sequence_ = 0;
  fed_back_sequence_ = 0;
  progress_ = 0;
  speed_mps_ = 0;
  guidance_.reset();
}

LocalModeResponse LocalPlanningCore::setMode(const LocalModeRequest& request) {
  LocalModeResponse response;
  response.epoch = params_.epoch;
  if (request.session_id.empty() || request.request_id.empty()) {
    response.reason = "session_id and request_id are required";
    return response;
  }
  if (request.mode != LocalMode::kIdle && request.mode != LocalMode::kExplore &&
      request.mode != LocalMode::kFollowRoute) {
    response.reason = "unknown mode";
    return response;
  }
  if (request.mode == LocalMode::kFollowRoute &&
      (!request.goal.allFinite() || !std::isfinite(request.tolerance_m) ||
       request.tolerance_m < 0 || !finite(request.route))) {
    response.reason = "FOLLOW_ROUTE needs a finite goal, route and tolerance";
    return response;
  }
  const bool continuation = request.session_id == session_id_;
  // A new session never extends, nor hears feedback for, the old one's paths.
  if (!continuation || request.mode == LocalMode::kIdle) clearSession();
  session_id_ = request.session_id;
  request_id_ = request.request_id;
  mode_ = request.mode;
  request_ = request;
  if (!continuation && early_guidance_ &&
      early_guidance_->session_id == session_id_)
    guidance_ = early_guidance_;
  early_guidance_.reset();
  response.accepted = true;
  response.reason = continuation ? "session continued" : "session started";
  return response;
}

void LocalPlanningCore::setGuidance(const LocalGuidance& guidance) {
  if (guidance.session_id.empty()) return;
  if (guidance.session_id != session_id_) {
    early_guidance_ = guidance;
    return;
  }
  guidance_ = guidance;
}

void LocalPlanningCore::setNoGoZones(const NoGoZones& zones,
                                     const std::string& frame_id) {
  zones_ = zones;
  zones_frame_ = frame_id;
  std::string failed;
  withdrawAgainstZones(zonesInOdom(failed), failed);
}

std::optional<NoGoZones> LocalPlanningCore::zonesInOdom(
    std::string& failed) const {
  if (zones_.empty()) return NoGoZones{};
  const auto T = resolve(zones_frame_, failed);
  if (!T) return std::nullopt;
  std::vector<Eigen::Vector2d> centres;
  for (const auto& c : zones_.centres())
    centres.push_back((*T * Eigen::Vector3d(c.x(), c.y(), 0)).head<2>());
  NoGoZones zones;
  zones.set(std::move(centres), zones_.reaches());
  return zones;
}

void LocalPlanningCore::withdrawAgainstZones(
    const std::optional<NoGoZones>& zones, const std::string& failed) {
  for (auto& [sequence, retained] : retained_) {
    if (retained.invalid) continue;
    if (!zones) {
      // Zones that cannot be placed cannot vouch for any path.
      invalidate(sequence, "no_transform:" + failed);
      continue;
    }
    if (zones->empty()) continue;
    const double progress = sequence == executing_sequence_ ? progress_ : 0.0;
    const auto remaining =
        committedPrefix(retained.path, progress, pathLength(retained.path));
    std::vector<Eigen::Vector3d> points;
    for (const auto& p : remaining.poses) points.push_back(p.head<3>());
    if (!zones->pathAdmissible(points))
      invalidate(sequence, "path enters a no-go zone");
  }
}

void LocalPlanningCore::onFeedback(const LocalFeedback& feedback) {
  if (session_id_.empty() || feedback.session_id != session_id_ ||
      feedback.epoch != params_.epoch || !std::isfinite(feedback.progress_m) ||
      feedback.progress_m < 0)
    return;
  if (feedback.executing && feedback.sequence_id != 0) {
    fed_back_sequence_ = feedback.sequence_id;
    executing_sequence_ = feedback.sequence_id;
    progress_ = feedback.progress_m;
    // The executor has moved on from every older path.
    retained_.erase(retained_.begin(),
                    retained_.lower_bound(executing_sequence_));
  } else {
    fed_back_sequence_ = 0;
    executing_sequence_ = 0;
    progress_ = 0;
  }
  if (std::isfinite(feedback.speed_mps))
    speed_mps_ = std::abs(feedback.speed_mps);
  if (std::isfinite(feedback.deceleration_mps2) &&
      feedback.deceleration_mps2 > 0)
    braking_.deceleration_mps2 = feedback.deceleration_mps2;
  if (std::isfinite(feedback.latency_s) && feedback.latency_s > 0)
    braking_.latency_s = feedback.latency_s;
}

std::optional<Eigen::Isometry3d> LocalPlanningCore::resolve(
    const std::string& frame, std::string& failed) const {
  if (frame.empty()) return Eigen::Isometry3d::Identity();
  std::optional<Eigen::Isometry3d> T;
  if (lookup_) T = lookup_(frame);
  if (!T || !T->matrix().allFinite()) {
    failed = frame;
    return std::nullopt;
  }
  return T;
}

LocalPlanResult LocalPlanningCore::plan(Clock::time_point deadline) {
  const auto start = Clock::now();
  LocalPlanResult result;
  const auto finish = [&]() -> LocalPlanResult {
    result.cycle_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    return result;
  };
  if (!have_pose_) {
    result.status = LocalStatus::kWaitingForMap;
    result.reason = "no odometry";
    return finish();
  }
  if (mode_ == LocalMode::kIdle) {
    result.status = LocalStatus::kNoLocalTarget;
    result.reason = "idle";
    return finish();
  }
  layer_.recheck(std::min(deadline, after(ground_recheck_s_)));

  LocalPlanInputs in;
  std::string failed;
  const auto blocked = [&]() -> LocalPlanResult {
    result.status = LocalStatus::kBlocked;
    result.reason = "no_transform:" + failed;
    return finish();
  };
  {
    // Never plan, nor keep a path, without the zones. Their placement can
    // move with every map-to-odometry correction: recheck retained paths.
    const auto zones = zonesInOdom(failed);
    withdrawAgainstZones(zones, failed);
    if (!zones) return blocked();
    in.no_go_zones = *zones;
  }
  const bool guided = guidance_ && guidance_->kind == GuidanceKind::kTarget;
  std::optional<Eigen::Isometry3d> guidance_T;
  if (guided) {
    guidance_T = resolve(guidance_->frame_id, failed);
    if (!guidance_T) return blocked();
  }
  if (mode_ == LocalMode::kFollowRoute) {
    const auto T = resolve(request_.frame_id, failed);
    if (!T) return blocked();
    in.target = *T * request_.goal;
    in.goal_tolerance_m = request_.tolerance_m;
    // The guidance planner's current route to the objective, else the
    // route the request carried.
    in.coarse_route = guided ? transformed(*guidance_T, guidance_->route)
                             : transformed(*T, request_.route);
  } else if (guided) {
    in.target = *guidance_T * guidance_->target;
    in.coarse_route = transformed(*guidance_T, guidance_->route);
  }

  in.pose = anchor_;
  in.speed_mps = speed_mps_;
  in.session_id = session_id_;
  in.request_id = request_id_;
  in.map_revision = map_.revision();
  in.epoch = params_.epoch;
  in.sequence_id = next_sequence_;
  in.guidance_sequence_id = guidance_ ? guidance_->sequence_id : 0;
  const auto executing = retained_.find(executing_sequence_);
  if (executing != retained_.end()) {
    in.executing_path = executing->second.path;
    in.executing_invalid = executing->second.invalid;
    in.progress_m = progress_;
  }
  in.braking = braking_;
  {
    PlanningCancellationScope scope(
        [deadline] { return Clock::now() >= deadline; });
    result = planner_.plan(in, MapChange{map_.revision(), {}, false});
  }
  if (result.path) {
    const std::uint64_t sequence = next_sequence_++;
    retained_[sequence] = Retained{*result.path, false};
    // Until feedback names another path, the executor drives this one.
    executing_sequence_ = sequence;
    progress_ = 0;
    while (retained_.size() > kMaxRetained) {
      // Publication changes the planning assumption, not the last executor
      // acknowledgement. Fence its path even if feedback skips a cycle;
      // retiring other ancestors silently avoids stopping newer paths.
      if (retained_.begin()->first == fed_back_sequence_)
        invalidate(retained_.begin()->first, "retention limit");
      retained_.erase(retained_.begin());
    }
  }
  return finish();
}

std::vector<LocalInvalidation> LocalPlanningCore::takeInvalidations() {
  std::vector<LocalInvalidation> out;
  out.swap(invalidations_);
  return out;
}

}  // namespace mgg
