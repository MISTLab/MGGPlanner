#include "mgg_core/local_planner.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

#include "mgg_core/departure.h"
#include "mgg_core/gain.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/local_route.h"
#include "mgg_core/planning_cancellation.h"
#include "mgg_core/voxel_walk.h"
namespace mgg {
namespace {
using Clock = std::chrono::steady_clock;
std::vector<Eigen::Vector3d> points(const LocalPathPlan& path) {
  std::vector<Eigen::Vector3d> out;
  for (const auto& p : path.poses) out.push_back(p.head<3>());
  return out;
}
bool interior(const Eigen::AlignedBox3d& window, const Eigen::Vector3d& p,
              double margin = 0.6) {
  return p.x() >= window.min().x() + margin &&
         p.x() <= window.max().x() - margin &&
         p.y() >= window.min().y() + margin &&
         p.y() <= window.max().y() - margin && p.z() >= window.min().z() &&
         p.z() <= window.max().z();
}
/// Whether ground is observed under `p`, at driving height.
bool groundObserved(const GroundProjection& ground, const Eigen::Vector3d& p) {
  Eigen::Vector3d sample = p;
  VoxelStatus status = VoxelStatus::kUnknown;
  ground.projectSample(sample, status);
  return status == VoxelStatus::kOccupied;
}
}  // namespace
LocalPlanner::LocalPlanner(const MapInterface& map, const GroundLayer& layer,
                           CertificationCache& cache,
                           const PlanningParams& planning,
                           const RobotParams& robot, const SensorParams& sensor)
    : map_(map),
      layer_(layer),
      cache_(cache),
      planning_(planning),
      robot_(robot),
      gain_sensor_(groundGainSensor(sensor, planning).value_or(sensor)),
      halos_(dependencyHalos(
          robot, planning, map.getResolution(),
          GroundProjection(map, planning).max_projection_length)) {
  planning_.edge_length_min = 0.1;
  planning_.edge_length_max = 0.6;
  planning_.nearest_range = 0.61;
  planning_.nearest_range_min = 0.1;
  planning_.nearest_range_max = 0.61;
  planning_.edge_overshoot = 0;
  planning_.num_vertices_max = 3000;
  planning_.num_edges_max = 30000;
  planning_.num_loops_cutoff = 10000;
  planning_.num_loops_max = 20000;
}
Eigen::AlignedBox3d LocalPlanner::dependency(const StateVec& a,
                                             const StateVec& b) const {
  auto box = edgeDependency(a.head<3>(), b.head<3>(), halos_);
  box.extend(turnRoomDependency(a.head<3>(), halos_));
  box.extend(turnRoomDependency(b.head<3>(), halos_));
  box.extend(slopeDependency(a.head<3>(), halos_));
  box.extend(slopeDependency(b.head<3>(), halos_));
  return box;
}
bool LocalPlanner::certify(LocalPathPlan& path, const NoGoZones& zones) {
  if (path.poses.empty() || path.reverse.size() != path.poses.size())
    return false;
  for (const auto& p : path.poses)
    if (!p.allFinite()) return false;
  const auto window = map_.windowBounds();
  if (window)
    for (const auto& p : path.poses)
      if (!window->contains(p.head<3>())) return false;
  if (!zones.pathAdmissible(points(path))) return false;
  GroundProjection ground(map_, planning_, true);
  ground.setStandingStart(standing_);
  ExpandContext ctx;
  ctx.map = &map_;
  ctx.ground = &ground;
  ctx.planning = &planning_;
  ctx.robot = &robot_;
  ctx.robot_box_size = robot_.getPlanningSize();
  ctx.stop_at_unknown = true;
  GraphManager graph;
  auto room = cache_.turnRoom([&](const StateVec& p) {
    return roomToTurn(map_, robot_, planning_, p,
                      standing_ ? &*standing_ : nullptr);
  });
  auto slope = cache_.slope([&](const Eigen::Vector3d& p) {
    return groundSlope(ground, p, robot_.size.head<2>().maxCoeff(), nullptr);
  });
  const bool reversing = path.reverse.front();
  for (bool r : path.reverse)
    if (r != reversing) return false;
  PathTurnCheck turns(graph, robot_, room, slope, reversing,
                      [&](const StateVec& a, const StateVec& b) {
                        return turnTransitionClear(map_, robot_, a, b);
                      });
  turns.setUnmeasuredSlope(standingSlope());
  if (!turns.admissible(points(path),
                        path.poses.front()[3] + (reversing ? M_PI : 0)))
    return false;
  path.edge_dependencies.clear();
  if (path.poses.size() == 1) {
    if (layer_.pending(dependency(path.poses.front(), path.poses.front())))
      return false;
    OrientedBox body;
    body.heading = path.poses.front()[3];
    body.size = robot_.getPlanningSize();
    const Eigen::Vector3d center =
        path.poses.front().head<3>() + robot_.offsetForHeading(body.heading);
    return layer_.verdict(path.poses.front().head<2>()) ==
               GroundVerdict::kAdmitted &&
           orientedBoxPathStatus(map_, center, center, body, true, nullptr) ==
               VoxelStatus::kFree;
  }
  // A departure from a hanging root is the lattice's hanging-root edge:
  // to the last pose over observed ground within the disk's radius of the
  // root (hanging_root_edge_length_max), or the whole path when it stays
  // within that (a committed prefix; publication keeps ends out of the
  // disk). Every resampled segment of it is certified as that edge is
  // (unobserved ground in the disk, unknown body volume; occupied blocks),
  // and the whole path's terrain with the root's floor as evidence. Beyond
  // it, the strict checks.
  const auto pts = points(path);
  std::size_t hanging_end = 0;
  if (!reversing && hangingRoot(path.poses.front())) {
    std::size_t reach = 1;
    while (reach < pts.size() &&
           (pts[reach] - pts.front()).norm() <= standing_->radius + 1e-9)
      ++reach;
    if (reach == pts.size()) hanging_end = reach - 1;
    for (std::size_t k = reach - 1; k > 0 && hanging_end == 0; --k)
      if (groundObserved(ground, pts[k])) hanging_end = k;
    if (hanging_end == 0) return false;
    for (std::size_t k = 1; k < hanging_end; ++k)
      if (!standing_->covers(pts[k].head<2>()) &&
          !groundObserved(ground, pts[k]))
        return false;
  }
  for (size_t i = 1; i < path.poses.size(); ++i) {
    planningCheckpoint();
    const auto& a = path.poses[i - 1];
    const auto& b = path.poses[i];
    auto dep = dependency(a, b);
    if (layer_.pending(dep)) return false;
    if (reversing) {
      if (!planning_.departure_reverse_allowed ||
          !reverseExitEdgeAdmissible(map_, ground, robot_, planning_, a, b))
        return false;
    } else if (!groundShortcutSegmentAdmissible(
                   ctx, pts[i - 1], pts[i], i > hanging_end, i <= hanging_end,
                   i == 1 && hanging_end > 0 ? &pts.front() : nullptr))
      return false;
    path.edge_dependencies.push_back(dep);
  }
  return ground.groundStepsAdmissible(pts,
                                      hanging_end > 0 ? &pts.front() : nullptr);
}
bool LocalPlanner::hangingRoot(const StateVec& pose) const {
  return standing_ && standing_->covers(pose.head<2>());
}
SlopeFn LocalPlanner::standingSlope() const {
  return [standing = standing_](const Eigen::Vector3d& p) {
    return standing && standing->covers(p.head<2>()) ? 0.0 : kUnknownSlopeRad;
  };
}
bool LocalPlanner::standingArrivalAdmissible(const StateVec& end,
                                             double tolerance) const {
  if (!standing_ || standing_->admitsGoal(end.head<2>(), tolerance)) return true;
  StateVec arrival = end;
  arrival[3] = kUnknownTurnHeading;
  return observedArrivalDisk(map_, robot_, planning_, arrival, tolerance);
}
bool LocalPlanner::pathStillCertified(const LocalPathPlan& path,
                                      double progress) {
  if (path.reverse.size() != path.poses.size()) return false;
  auto remaining = committedPrefix(path, progress, pathLength(path));
  return certify(remaining, zones_);
}
double LocalPlanner::localGain(const StateVec& state,
                               const Eigen::AlignedBox3d& window) const {
  Eigen::Vector3d origin = state.head<3>();
  std::vector<Eigen::Vector3d> endpoints;
  if (gain_sensor_.mount_height > 0) {
    gain_sensor_.getMountedFrustumEndpoints(
        state, state.z() - planning_.max_ground_height, origin, endpoints);
  } else {
    gain_sensor_.getFrustumEndpoints(state, endpoints);
  }
  std::unordered_set<VoxelKey, VoxelKeyHash> unknown;
  PlanningCheckpointThrottle checkpoint;
  for (const auto& end : endpoints) {
    planningCheckpoint();
    walkVoxels(origin, end, map_.getResolution(), 100000,
               [&](const VoxelIndex& k) {
                 checkpoint.check();
                 const VoxelKey key{static_cast<int32_t>(k.x),
                                    static_cast<int32_t>(k.y),
                                    static_cast<int32_t>(k.z)};
                 auto p = centerOf(key, map_.getResolution());
                 if (!window.contains(p)) return false;
                 auto status = map_.getVoxelStatus(p);
                 if (status == VoxelStatus::kOccupied) return false;
                 if (status == VoxelStatus::kUnknown && interior(window, p) &&
                     p.z() >= state.z() - planning_.max_ground_height &&
                     p.z() <= state.z() - planning_.max_ground_height +
                                  robot_.size.z() +
                                  planning_.ground_frontier_height_margin)
                   unknown.insert(key);
                 return true;
               });
  }
  return groundGainFrontier(unknown.size(), map_.getResolution())
             ? unknown.size()
             : 0;
}
LocalPlanResult LocalPlanner::plan(const LocalPlanInputs& in,
                                   const MapChange& change) {
  const auto start = Clock::now();
  const auto deadline =
      start + std::chrono::duration_cast<Clock::duration>(
                  std::chrono::duration<double>(kLocalPlanningBudgetS));
  cache_.withdraw(change);
  if (change.everything) track_.clear();
  // No-go policy is not part of cache keys. Flushing avoids stale admissions
  // even when setMode/setNoGoZones changes policy without a map revision.
  if (zones_.centres() != in.no_go_zones.centres() ||
      zones_.reaches() != in.no_go_zones.reaches())
    cache_.flushAll();
  zones_ = in.no_go_zones;
  track_.add(in.pose);
  LocalPlanResult result;
  const auto* parent = planning_cancelled;
  PlanningCancellationScope scope(
      [&] { return Clock::now() >= deadline || (parent && (*parent)()); });
  try {
    result = search(in, deadline);
    if (result.path)
      result.path->map_revision = std::max(in.map_revision, change.revision);
  } catch (const std::invalid_argument& error) {
    result.reason = error.what();
  } catch (const PlanningInterrupted&) {
    result.reason =
        "local search budget exhausted; reachability not established";
    result.status = LocalStatus::kBlocked;
    result.checks_complete = false;
  }
  result.cycle_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - start).count();
  return result;
}
LocalPlanResult LocalPlanner::search(const LocalPlanInputs& in,
                                     Clock::time_point deadline) {
  LocalPlanResult result;
  result.reason = "no certified local route";
  if (!in.pose.allFinite() || !std::isfinite(in.speed_mps) ||
      !std::isfinite(in.progress_m) || !std::isfinite(in.goal_tolerance_m) ||
      in.goal_tolerance_m < 0 || (in.target && !in.target->allFinite())) {
    result.reason = "invalid local inputs";
    return result;
  }
  if (!map_.getStatus()) {
    result.status = LocalStatus::kWaitingForMap;
    result.reason = "map is unavailable";
    return result;
  }
  // At rest, reserve a driving horizon too: a margin-only commitment would
  // leave the executor's braking fence permanently at zero speed.
  const double commit = commitmentLength(
      std::max(std::abs(in.speed_mps), planning_.v_max), in.braking);
  auto window = map_.windowBounds().value_or(
      Eigen::AlignedBox3d(in.pose.head<3>() - Eigen::Vector3d(8, 8, 3),
                          in.pose.head<3>() + Eigen::Vector3d(8, 8, 3)));
  GroundProjection ground(map_, planning_, true);
  ground.setStandingStart(standing_);
  // A goal's height is a hint (a 2-D objective is seeded at the robot's
  // altitude; guidance carries the global map's ground). Driving height over
  // the local ground at the same XY, or nothing: never a guessed height.
  const auto onLocalGround =
      [&](const Eigen::Vector3d& p) -> std::optional<Eigen::Vector3d> {
    Eigen::Vector3d sample = p;
    VoxelStatus status = VoxelStatus::kUnknown;
    const double below = ground.projectGoal(sample, status);
    if (status != VoxelStatus::kOccupied || !std::isfinite(below))
      return std::nullopt;
    return Eigen::Vector3d(p.x(), p.y(),
                           p.z() - below + planning_.max_ground_height);
  };
  // in.target placed on the local ground, for linking and terminal checks;
  // unset where the local map has no ground for it.
  const std::optional<Eigen::Vector3d> target =
      in.target ? onLocalGround(*in.target) : std::nullopt;
  LocalPathPlan identity;
  identity.session_id = in.session_id;
  identity.request_id = in.request_id;
  identity.epoch = in.epoch;
  identity.map_revision = in.map_revision;
  identity.guidance_sequence_id = in.guidance_sequence_id;
  identity.sequence_id = in.sequence_id ? in.sequence_id : ++sequence_;
  const bool same = in.executing_path &&
                    in.executing_path->session_id == in.session_id &&
                    in.executing_path->epoch == in.epoch;
  identity.kind = same ? LocalPathKind::kBreak : LocalPathKind::kStart;
  if (same) identity.extends_sequence_id = in.executing_path->sequence_id;
  LocalPathPlan prefix;
  if (same && !in.executing_invalid &&
      in.executing_path->reverse.size() == in.executing_path->poses.size()) {
    prefix = committedPrefix(*in.executing_path, in.progress_m, commit);
    if (!certify(prefix, in.no_go_zones)) prefix = LocalPathPlan{};
  }
  bool selection_complete = true;
  auto publish = [&](LocalPathPlan path) -> bool {
    // Legacy's first-goal arrival protection: never stop in the disk.
    if (path.poses.size() > 1 &&
        !standingArrivalAdmissible(path.poses.back(), in.goal_tolerance_m))
      return false;
    if (!certify(path, in.no_go_zones)) return false;
    const double length = pathLength(path);
    const double speed = std::abs(in.speed_mps);
    const double stopping =
        speed * speed / (2 * in.braking.deceleration_mps2) +
        speed * (in.braking.latency_s + in.braking.planning_latency_s) +
        in.braking.margin_m;
    // A stationary terminal is a stop request, not a new motion segment.
    // Accept it even while slowing down; the executor fences velocity and
    // rechecks goal tolerance. Moving terminals still need stopping space.
    const bool stationary_terminal =
        path.reaches_goal && path.poses.size() == 1;
    if (!stationary_terminal && speed > 0 && length + 1e-9 < stopping)
      return false;
    path.commit_length_m = std::min(length, commit);
    // Preserve the envelope, never silently shorten it to two seconds.
    // A cap lets subsequent cycles return to the nominal two-second horizon.
    if (stationary_terminal)
      result.speed_cap_mps = 0;
    else if (speed > 0 && stopping > 2 * speed)
      result.speed_cap_mps =
          commitmentSpeedCap(std::min(length, 2 * speed), in.braking);
    if (!stationary_terminal && !prefix.poses.empty())
      path.commit_length_m = std::max(path.commit_length_m, pathLength(prefix));
    if (path.prefix_length == 0) {
      path.prefix_length =
          committedPrefix(path, 0, path.commit_length_m).poses.size();
    }
    result.path = std::move(path);
    result.status = LocalStatus::kMoving;
    result.reason = result.path->reaches_goal ? "final goal certified"
                                              : "certified local path";
    result.checks_complete = selection_complete;
    return true;
  };
  if (target &&
      (in.pose.head<3>() - *target).norm() <= in.goal_tolerance_m + 1e-9) {
    auto path = identity;
    path.poses = {in.pose};
    path.reverse = {false};
    path.reaches_goal = true;
    if (publish(path)) return result;
  }
  if (!prefix.poses.empty() && prefix.reverse.front()) {
    // Re-certify the original escape to its original refuge. The newly
    // recorded reverse odometry must not become a fresh backwards escape,
    // nor may a forward suffix be appended to a reverse commitment.
    if (!roomToTurn(map_, robot_, planning_, in.pose,
                    standing_ ? &*standing_ : nullptr)) {
      auto remaining = committedPrefix(*in.executing_path, in.progress_m,
                                       pathLength(*in.executing_path));
      auto continuation = identity;
      continuation.kind = LocalPathKind::kExtend;
      continuation.poses = std::move(remaining.poses);
      continuation.reverse = std::move(remaining.reverse);
      continuation.prefix_length = prefix.poses.size();
      continuation.reaches_goal =
          target && (continuation.poses.back().head<3>() - *target).norm() <=
                        in.goal_tolerance_m;
      if (continuation.poses.size() > 1 && publish(continuation)) return result;
    }
    // At the refuge (or after failed revalidation), start forward from the
    // current pose with BREAK instead of attempting a mixed-direction splice.
    prefix = LocalPathPlan{};
  }
  const StateVec root = prefix.poses.empty() ? in.pose : prefix.poses.back();
  if (layer_.pending(dependency(root, root))) {
    result.status = LocalStatus::kWaitingForMap;
    result.reason = "all departures depend on pending map";
    return result;
  }
  std::optional<Eigen::Vector3d> aim;
  bool aim_ground_unknown = false;
  if (in.target && interior(window, *in.target)) {
    aim = target;
    aim_ground_unknown = !aim;
  } else {
    // The farthest route point in the window with local ground under it.
    for (auto p = in.coarse_route.rbegin(); p != in.coarse_route.rend(); ++p) {
      if (!p->allFinite() || !interior(window, *p)) continue;
      aim = onLocalGround(*p);
      aim_ground_unknown = !aim;
      if (aim) break;
    }
  }
  if (aim_ground_unknown) {
    // Blocks only this target; the guidance planner may select elsewhere.
    result.status = LocalStatus::kWaitingForMap;
    result.reason = "goal_ground_unknown";
    return result;
  }
  if (!aim && !gain_sensor_.isReady()) {
    result.reason = "gain sensor has no configured rays";
    return result;
  }
  if (aim) {
    StateVec goal;
    goal << *aim, root[3];
    if (layer_.pending(dependency(goal, goal))) {
      result.status = LocalStatus::kWaitingForMap;
      result.reason = "target dependency is pending";
      return result;
    }
  }
  ExpandContext ctx;
  ctx.map = &map_;
  ctx.planning = &planning_;
  ctx.robot = &robot_;
  ctx.ground = &ground;
  ctx.robot_box_size = robot_.getPlanningSize();
  ctx.edge_verdicts = &cache_.edges();
  ctx.stop_at_unknown = true;
  ctx.strict_projected_endpoint = true;
  ctx.projected_edge_admissible =
      [&](const std::vector<Eigen::Vector3d>& edge) {
        if (!in.no_go_zones.pathAdmissible(edge)) return false;
        for (size_t i = 1; i < edge.size(); ++i) {
          StateVec a, b;
          a << edge[i - 1], 0;
          b << edge[i], 0;
          if (layer_.pending(dependency(a, b))) return false;
        }
        return true;
      };
  // The standing start's hanging root, as PlannerNode::makeContext and
  // buildLocalGraph make it: one edge out of the root may cross the blind
  // disk to observed ground, through the body band its lidar cannot see.
  const bool hanging_root = hangingRoot(root);
  if (hanging_root) {
    ctx.hanging_root_edge_length_max = standing_->radius;
    ctx.hanging_root_unknown_body = true;
    ctx.preserve_hanging_root_start_height = true;
    ctx.root_is_robot = true;
  }
  ctx.deadline =
      std::min(deadline, Clock::now() + std::chrono::milliseconds(170));
  GridGraphParams grid;
  grid.world_aligned = true;
  grid.resolution = {0.4, 0.4, 0.4};
  grid.min_val = window.min() - root.head<3>();
  grid.max_val = window.max() - root.head<3>();
  grid.min_val.x() += 0.6;
  grid.min_val.y() += 0.6;
  grid.max_val.x() -= 0.6;
  grid.max_val.y() -= 0.6;
  grid.min_val.z() = 0;
  grid.max_val.z() = 0;
  if ((grid.min_val.array() > 0).any() || (grid.max_val.array() < 0).any()) {
    result.reason = "splice root outside planning interior";
    return result;
  }
  GraphManager graph;
  auto* root_vertex = new Vertex(graph.generateVertexID(), root);
  root_vertex->is_hanging = hanging_root;
  graph.addVertex(root_vertex);
  const auto lattice = buildGridGraph(graph, root, grid, ctx, 0);
  auto room = cache_.turnRoom([&](const StateVec& p) {
    return roomToTurn(map_, robot_, planning_, p,
                      standing_ ? &*standing_ : nullptr);
  });
  auto slope = cache_.slope([&](const Eigen::Vector3d& p) {
    return groundSlope(ground, p, robot_.size.head<2>().maxCoeff(), nullptr);
  });
  PathTurnCheck turns(graph, robot_, room, slope);
  turns.setUnmeasuredSlope(standingSlope());
  struct Candidate {
    int id;
    double score;
  };
  std::vector<Candidate> candidates;
  if (aim) {
    StateVec goal;
    goal << *aim, root[3];
    auto* exact =
        linkGoalToLattice(graph, goal, ctx, [](const Vertex&) { return true; });
    if (exact) candidates.push_back({exact->id, 0});
  } else {
    // Score useful-length candidates first, then keep the best *completed*
    // scores within a soft slice. Leave time to route and certify them.
    std::vector<Vertex*> viewpoints;
    for (const auto& entry : graph.vertices_map_) {
      const auto& v = *entry.second;
      if (v.id != 0 && interior(window, v.state.head<3>()) &&
          !layer_.pending(dependency(v.state, v.state)))
        viewpoints.push_back(entry.second);
    }
    const double retained_length = pathLength(prefix);
    const auto priority = [&](const Vertex* v) {
      const Eigen::Vector2d delta = (v->state - root).head<2>();
      const double turn = std::abs(
          std::remainder(std::atan2(delta.y(), delta.x()) - root[3], 2 * M_PI));
      return std::abs(delta.norm() + retained_length - 7.0) + .25 * turn;
    };
    std::sort(viewpoints.begin(), viewpoints.end(),
              [&](const Vertex* a, const Vertex* b) {
                const double pa = priority(a), pb = priority(b);
                return pa == pb ? a->id < b->id : pa < pb;
              });
    const auto scoring_deadline =
        std::min(Clock::now() + std::chrono::milliseconds(100),
                 deadline - std::chrono::milliseconds(100));
    const auto* parent = planning_cancelled;
    bool parent_interrupted = false;
    {
      PlanningCancellationScope scoring_scope([&] {
        if (parent && (*parent)()) {
          parent_interrupted = true;
          return true;
        }
        return Clock::now() >= scoring_deadline;
      });
      try {
        for (const Vertex* v : viewpoints) {
          planningCheckpoint();
          const double gain = localGain(v->state, window);
          if (gain > 0) candidates.push_back({v->id, -gain});
        }
      } catch (const PlanningInterrupted&) {
        if (parent_interrupted) throw;
        selection_complete = false;
      }
    }
    // Only the scoring slice may be recovered. Never swallow cancellation or
    // the outer cycle deadline and publish a partial safety certificate.
    planningCheckpoint();
    std::sort(candidates.begin(), candidates.end(),
              [](const auto& a, const auto& b) {
                return a.score == b.score ? a.id < b.id : a.score < b.score;
              });
  }
  result.candidate_count = candidates.size();
  std::vector<int> destinations;
  for (const auto& c : candidates) destinations.push_back(c.id);
  auto routes = findTurnCompliantRoutes(
      graph, root[3], turns.window(), destinations,
      [&](const Vertex& v) {
        return turns.sharpTurnAllowedAt(v.state.head<3>());
      },
      20000);
  std::size_t too_short = 0;  // candidates refused only for their length
  for (const auto& candidate : candidates) {
    planningCheckpoint();
    auto found = routes.to.find(candidate.id);
    if (found == routes.to.end() || !turns(found->second.path)) continue;
    auto path = identity;
    path.poses = {root};
    path.reverse = {false};
    // Shortcut only when the whole candidate and native-spacing terrain
    // checks still pass; no sharp-turn fallback to a geometric shortest path.
    std::vector<StateVec> route;
    for (const auto* v : found->second.path) route.push_back(v->state);
    for (size_t i = 0; i + 1 < route.size();) {
      size_t next = i + 1;
      for (size_t j = route.size() - 1; j > i + 1; --j) {
        LocalPathPlan shortcut = path;
        shortcut.poses.push_back(route[j]);
        shortcut.reverse.push_back(false);
        if (certify(shortcut, in.no_go_zones)) {
          next = j;
          break;
        }
      }
      const StateVec start = path.poses.back();
      const auto end = route[next];
      const double distance = (end - start).head<3>().norm();
      const int steps =
          std::max(1, static_cast<int>(std::ceil(distance / 0.25)));
      for (int k = 1; k <= steps; ++k) {
        StateVec p = start + (end - start) * (double(k) / steps);
        p[3] = std::atan2(end.y() - start.y(), end.x() - start.x());
        path.poses.push_back(p);
        path.reverse.push_back(false);
      }
      i = next;
    }
    if (!prefix.poses.empty()) path = splicePath(prefix, path);
    if (pathLength(path) > 10) {
      // Clip only the newly generated tail, never the exact retained prefix.
      double along = 0;
      size_t keep = 1;
      for (; keep < path.poses.size(); ++keep) {
        along += (path.poses[keep] - path.poses[keep - 1]).head<3>().norm();
        if (along > 10) break;
      }
      if (keep < path.prefix_length) continue;
      path.poses.resize(keep);
      path.reverse.resize(keep);
    }
    path.reaches_goal =
        target && (path.poses.back().head<3>() - *target).norm() <=
                      in.goal_tolerance_m + 1e-9;
    if (!path.reaches_goal && pathLength(path) < 6) {
      ++too_short;
      continue;
    }
    if (publish(path)) return result;
  }
  // Reverse is an escape to the first turn refuge, never a new reverse plan
  // through unexplored space. Validate all of it against today's map.
  if (planning_.departure_reverse_allowed && !room(in.pose)) {
    auto back = track_.backFrom(in.pose, 3 * robot_.size.x());
    auto path = identity;
    for (const auto& p : back) {
      path.poses.push_back(p);
      path.reverse.push_back(true);
      if (path.poses.size() > 1 && room(p)) {
        path.reaches_goal =
            target && (p.head<3>() - *target).norm() <= in.goal_tolerance_m;
        if (publish(path)) return result;
        break;
      }
    }
  }
  result.checks_complete = selection_complete && !lattice.hit_limit &&
                           !routes.capped && layer_.pendingCount() == 0;
  // Exhaustion needs evidence: every viewpoint scored, none with gain, and
  // a lattice that left the root unless observed space encloses it.
  const auto unknown_departures =
      lattice.no_ground + lattice.unknown_cells +
      lattice.projected_endpoint_unknown +
      lattice.edge_status[static_cast<int>(ProjectedEdgeStatus::kUnknown)] +
      lattice.edge_status[static_cast<int>(ProjectedEdgeStatus::kHanging)] +
      lattice.edge_status[static_cast<int>(
          ProjectedEdgeStatus::kGroundUnobserved)];
  const bool unobserved_surroundings =
      graph.getNumVertices() == 1 &&
      (hanging_root || unknown_departures > 0 ||
       layer_.verdict(root.head<2>()) != GroundVerdict::kAdmitted);
  if (!in.target && layer_.pendingCount() > 0) {
    result.status = LocalStatus::kWaitingForMap;
    result.reason = "pending map prevents exhaustion";
  } else if (!in.target && !candidates.empty()) {
    // Gain remains: a route or length refusal is not an empty window.
    result.status = LocalStatus::kBlocked;
    result.reason = too_short == candidates.size()
                        ? "gain candidates are shorter than a useful path"
                        : "gain candidates have no certified local route";
  } else if (!in.target && !selection_complete) {
    result.status = LocalStatus::kWaitingForMap;
    result.reason = "not every viewpoint was scored";
  } else if (!in.target && unobserved_surroundings) {
    result.status = LocalStatus::kWaitingForMap;
    result.reason = "no certified ground around the robot";
    result.checks_complete = false;
  } else if (!in.target && result.checks_complete) {
    result.status = LocalStatus::kNoLocalTarget;
    result.reason = graph.getNumVertices() == 1
                        ? "observed surroundings enclose the robot"
                        : "observed local window has no gain";
  } else {
    result.status = LocalStatus::kBlocked;
    result.reason =
        result.checks_complete
            ? "target has no certified local route"
            : "bounded search incomplete; not physical unreachability";
  }
  return result;
}
}  // namespace mgg
