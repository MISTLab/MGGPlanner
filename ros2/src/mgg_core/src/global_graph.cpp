#include "mgg_core/global_graph.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>

#include "mgg_core/departure.h"
#include "mgg_core/trajectory.h"

namespace mgg {

RobotStateHistory::RobotStateHistory() { reset(); }

RobotStateHistory::~RobotStateHistory() {
  if (kd_tree_) kd_free(kd_tree_);
}

void RobotStateHistory::reset() {
  if (kd_tree_) kd_free(kd_tree_);
  kd_tree_ = kd_create(3);
  state_hist_.clear();
}

void RobotStateHistory::addState(const StateVec& state) {
  state_hist_.push_back(state);
  StateVec* stored = &state_hist_.back();
  kd_insert3(kd_tree_, stored->x(), stored->y(), stored->z(), stored);
}

bool RobotStateHistory::getNearestStates(
    const StateVec& state, double range,
    std::vector<const StateVec*>* s_res) const {
  // The kd-tree library cannot deal with an empty tree (rrg.cpp:6245).
  if (state_hist_.empty()) return false;
  kdres* neighbors =
      kd_nearest_range3(kd_tree_, state.x(), state.y(), state.z(), range);
  const int neighbors_size = kd_res_size(neighbors);
  if (neighbors_size <= 0) {
    kd_res_free(neighbors);  // upstream returned without freeing (rrg.cpp:6249)
    return false;
  }
  s_res->clear();
  for (int i = 0; i < neighbors_size; ++i) {
    s_res->push_back(static_cast<const StateVec*>(kd_res_item_data(neighbors)));
    if (kd_res_next(neighbors) <= 0) break;
  }
  kd_res_free(neighbors);
  return true;
}

namespace {

/// rrg.cpp:4831 and 4832.
constexpr double kDeltaLimit = 0.1;
constexpr double kRadiusLimit = 0.5;

/// A pose to fold in, with the local vertex it came from when there is one.
struct RefPose {
  StateVec state = StateVec::Zero();
  const Vertex* source = nullptr;
};

/// `box_size` swept along the straight segment between two driving-height
/// states runs through space the map knows to be occupied.
bool throughKnownObstacle(const ExpandContext& ctx, const StateVec& from,
                          const StateVec& to,
                          const Eigen::Vector3d& box_size) {
  return ctx.map->getPathStatus(from.head<3>() + ctx.robot->center_offset,
                                to.head<3>() + ctx.robot->center_offset,
                                box_size, false) == VoxelStatus::kOccupied;
}

/// rrg.cpp:4817 to 4862, the "add root vertex first" block shared by
/// addRefPathToGraph and connectStateToGraph (rrg.cpp:5471 to 5510): the
/// nearest vertex itself when the state sits on it, a blind edge when it is
/// within `blind_radius`, otherwise a checked expandGraph.
Vertex* linkStateToGraph(GraphManager& graph, const StateVec& state,
                         const ExpandContext& ctx, double blind_radius,
                         bool exact_state) {
  Vertex* nearest_vertex = nullptr;
  if (!graph.getNearestVertex(&state, &nearest_vertex) ||
      nearest_vertex == nullptr) {
    return nullptr;
  }
  if ((state.head<3>() - nearest_vertex->state.head<3>()).squaredNorm() <=
      (exact_state ? 0.0 : kDeltaLimit * kDeltaLimit)) {
    return nearest_vertex;
  }
  // "@TODO: find better way to do this. Blindly add a link/vertex to the
  // graph." (rrg.cpp:4839). The state is where the robot stands or where a
  // verified path starts, a step from a vertex it drove through. Upstream
  // took the geometrically nearest vertex; here the nearest one whose
  // segment is not known to cross an obstacle, so a vertex on the far side
  // of a thin wall cannot become the link. Unknown space still passes, as
  // it does for every lattice edge.
  const double radius = std::max(blind_radius, ctx.planning->edge_length_min);
  std::vector<Vertex*> candidates;
  if (graph.getNearestVertices(&state, radius, &candidates)) {
    std::sort(candidates.begin(), candidates.end(),
              [&state](const Vertex* a, const Vertex* b) {
                return (a->state.head<3>() - state.head<3>()).norm() <
                       (b->state.head<3>() - state.head<3>()).norm();
              });
    for (Vertex* candidate : candidates) {
      if (candidate == nullptr ||
          throughKnownObstacle(ctx, candidate->state, state,
                               ctx.robot_box_size)) {
        continue;
      }
      const double direction_norm =
          (state.head<3>() - candidate->state.head<3>()).norm();
      auto* new_vertex = new Vertex(graph.generateVertexID(), state);
      new_vertex->robot_id = ctx.robot_id;
      new_vertex->parent = candidate;
      new_vertex->distance = candidate->distance + direction_norm;
      candidate->children.push_back(new_vertex);
      graph.addVertex(new_vertex);
      graph.addEdge(new_vertex, candidate, direction_norm);
      return new_vertex;
    }
  }
  ExpandGraphReport rep;
  Vertex candidate(-1, state);
  expandGraph(graph, candidate, rep, ctx);
  if (rep.status != ExpandGraphStatus::kSuccess) return nullptr;
  // expandGraph may clip a long edge. A partial extension is not an exact
  // objective endpoint, even though it remains useful roadmap geometry.
  if (exact_state &&
      (rep.vertex_added->state.head<3>() - state.head<3>()).squaredNorm() >
          1e-12) {
    return nullptr;
  }
  return rep.vertex_added;
}

/// Links one pose of a verified path where the graph reaches it: within one
/// edge of a vertex, through linkStateToGraph. Farther away, expandGraph
/// would clip its edge short of the pose and the chain would continue from
/// the clipped vertex over an unchecked edge, so the pose is not tried.
Vertex* linkPathPose(GraphManager& graph, const StateVec& state,
                     const ExpandContext& ctx) {
  Vertex* nearest_vertex = nullptr;
  if (!graph.getNearestVertex(&state, &nearest_vertex) ||
      nearest_vertex == nullptr ||
      (state.head<3>() - nearest_vertex->state.head<3>()).norm() >
          ctx.planning->edge_length_max) {
    return nullptr;
  }
  return linkStateToGraph(graph, state, ctx, kRadiusLimit, false);
}

/// A vertex of this robot already standing on `state`, if there is one: a
/// replan from the same place repeats the same lattice, and upstream re-added
/// it every cycle (rrg.cpp:4882). Not applied to other robots' vertices,
/// whose placement carries the inter-robot transform error.
Vertex* ownVertexAt(GraphManager& graph, const StateVec& state, int robot_id) {
  Vertex* nearest = nullptr;
  if (!graph.getNearestVertexInRange(&state, kDeltaLimit, &nearest) ||
      nearest == nullptr || nearest->robot_id != robot_id) {
    return nullptr;
  }
  return nearest;
}

bool addRefPath(GraphManager& graph, const std::vector<RefPose>& poses,
                const ExpandContext& ctx, double vertex_spacing,
                std::vector<Vertex*>* path_vertices) {
  if (poses.empty()) return false;

  // The whole path is collision free already and starts from the root
  // vertex. Only its first vertex has to be linked to the existing graph
  // (rrg.cpp:4812). Upstream dropped the path when that failed. But the root
  // is where the robot stood, and a robot resting against a wall or tilted
  // on a ramp crest stands where its body box overlaps the map: the start
  // then links nowhere, the graph stops growing, every later start is
  // farther from it and none links again (SubT finals, 2026-09-23). The
  // poses after the root are lattice states whose bodies were checked, so
  // the path joins at the first pose the graph reaches, and the unlinked
  // prefix stays out of the graph. Every pose is tried before spacing
  // drops any: the only pose that links may be one spacing would drop.
  std::size_t first = 0;
  Vertex* parent_vertex = nullptr;
  for (; first < poses.size(); ++first) {
    parent_vertex = linkPathPose(graph, poses[first].state, ctx);
    if (parent_vertex != nullptr) break;
  }
  if (parent_vertex == nullptr) return false;
  if (first > 0 && poses[first].source != nullptr &&
      parent_vertex->robot_id == ctx.robot_id &&
      parent_vertex->type != VertexType::kVisited) {
    parent_vertex->type = poses[first].source->type;
    parent_vertex->vol_gain = poses[first].source->vol_gain;
  }

  // From the linked pose on, keep about one pose per vertex_spacing.
  // Upstream's path was the lattice walk itself, one vertex per grid cell
  // (rrg.cpp:4867); the port's corridor may have been resampled more densely
  // than the roadmap needs. The final pose is the viewpoint the path was
  // chosen for and is always kept.
  std::vector<RefPose> kept;
  kept.push_back(poses[first]);
  for (std::size_t i = first + 1; i < poses.size(); ++i) {
    const double gap =
        (poses[i].state.head<3>() - kept.back().state.head<3>()).norm();
    const bool last = i + 1 == poses.size();
    if (gap >= vertex_spacing || (last && gap > 1e-9)) kept.push_back(poses[i]);
  }

  // Add all remaining vertices of the path (rrg.cpp:4864).
  std::vector<Vertex*> vertex_list;
  vertex_list.push_back(parent_vertex);
  for (std::size_t i = 1; i < kept.size(); ++i) {
    const double direction_norm =
        (kept[i].state.head<3>() - parent_vertex->state.head<3>()).norm();
    Vertex* new_vertex = ownVertexAt(graph, kept[i].state, ctx.robot_id);
    if (new_vertex == parent_vertex) continue;
    if (new_vertex == nullptr) {
      new_vertex = new Vertex(graph.generateVertexID(), kept[i].state);
      new_vertex->robot_id = ctx.robot_id;
      new_vertex->parent = parent_vertex;
      new_vertex->distance = parent_vertex->distance + direction_norm;
      parent_vertex->children.push_back(new_vertex);
      graph.addVertex(new_vertex);
    }
    if (kept[i].source != nullptr &&
        new_vertex->type != VertexType::kVisited) {
      new_vertex->type = kept[i].source->type;
      new_vertex->vol_gain = kept[i].source->vol_gain;
    }
    if (!graph.graph_->edgeExists(new_vertex->id, parent_vertex->id)) {
      graph.addEdge(new_vertex, parent_vertex, direction_norm);
    }
    vertex_list.push_back(new_vertex);
    parent_vertex = new_vertex;
  }

  // Build edges around vertices if possible to get better paths
  // (rrg.cpp:4895). The path itself is trusted; the extra edges are checked.
  for (Vertex* vertex : vertex_list) {
    ExpandGraphReport rep;
    expandGraphEdges(graph, vertex, rep, ctx);
  }

  // Add intermediate vertices along long segments to densify the graph
  // (rrg.cpp:4922, intp_len 1.0 m there; vertex_spacing here). Evenly spaced
  // rather than a run of full steps plus a remainder, and only once a segment
  // holds at least two spacings, so a segment just over the spacing does not
  // gain a vertex a few centimetres from its end.
  if (vertex_spacing > 0.0) {
    for (std::size_t i = 0; i + 1 < vertex_list.size(); ++i) {
      const Eigen::Vector3d start = vertex_list[i]->state.head<3>();
      const Eigen::Vector3d end = vertex_list[i + 1]->state.head<3>();
      const double edge_length = (end - start).norm();
      const int n_intp =
          static_cast<int>(std::floor(edge_length / vertex_spacing));
      if (n_intp < 2) continue;
      const double segment = edge_length / n_intp;
      const Eigen::Vector3d edge_vec = (end - start) / edge_length;
      Vertex* prev_vertex = vertex_list[i];
      for (int j = 1; j < n_intp; ++j) {
        const Eigen::Vector3d new_v = start + j * segment * edge_vec;
        StateVec new_state;
        new_state << new_v[0], new_v[1], new_v[2], vertex_list[i]->state[3];
        Vertex* new_vertex = ownVertexAt(graph, new_state, ctx.robot_id);
        if (new_vertex == prev_vertex || new_vertex == vertex_list[i + 1]) {
          continue;
        }
        if (new_vertex == nullptr) {
          new_vertex = new Vertex(graph.generateVertexID(), new_state);
          new_vertex->robot_id = ctx.robot_id;
          graph.addVertex(new_vertex);
        }
        if (!graph.graph_->edgeExists(new_vertex->id, prev_vertex->id)) {
          graph.addEdge(new_vertex, prev_vertex, segment);
        }
        prev_vertex = new_vertex;
      }
      if (prev_vertex != vertex_list[i + 1] &&
          !graph.graph_->edgeExists(prev_vertex->id, vertex_list[i + 1]->id)) {
        graph.addEdge(prev_vertex, vertex_list[i + 1], segment);
      }
    }
  }

  if (path_vertices != nullptr) *path_vertices = vertex_list;
  return true;
}

}  // namespace

Vertex* connectStateToGraph(GraphManager& graph, const StateVec& state,
                            const ExpandContext& ctx,
                            double dist_ignore_collision_check,
                            bool exact_state) {
  Vertex* linked = linkStateToGraph(
      graph, state, ctx, dist_ignore_collision_check, exact_state);
  if (linked == nullptr) return nullptr;
  // rrg.cpp:5484 and 5497: edges from the linked vertex to whatever else is
  // reachable around it, so the route out of here is not just the chain in.
  ExpandGraphReport rep;
  expandGraphEdges(graph, linked, rep, ctx);
  return linked;
}

DepartureLink linkDeparture(GraphManager& graph, const StateVec& state,
                            const ExpandContext& ctx, double link_radius) {
  DepartureLink link;
  link.vertex = connectStateToGraph(graph, state, ctx, link_radius,
                                    /*exact_state=*/false);
  if (link.vertex != nullptr) return link;
  // The robot's own pose is where it stands, even when its box touches a
  // wall it stopped against: every box sweep out of there was refused and
  // the robot could not be routed anywhere (robot_1 and robot_2 on the SubT
  // return, 2026-09-23). It may depart along a centre line clear of known
  // obstacles, which still refuses a vertex behind a wall. That link was
  // never checked for the robot's box, so it stays out of the graph: no
  // later route, goal link or neighbour may use it (review r0).
  const double radius = std::max(link_radius, ctx.planning->edge_length_min);
  std::vector<Vertex*> candidates;
  if (!graph.getNearestVertices(&state, radius, &candidates)) return link;
  std::sort(candidates.begin(), candidates.end(),
            [&state](const Vertex* a, const Vertex* b) {
              return (a->state.head<3>() - state.head<3>()).squaredNorm() <
                     (b->state.head<3>() - state.head<3>()).squaredNorm();
            });
  for (Vertex* candidate : candidates) {
    if (candidate == nullptr || !graph.inService(*candidate) ||
        throughKnownObstacle(ctx, candidate->state, state,
                             Eigen::Vector3d::Zero())) {
      continue;
    }
    link.vertex = candidate;
    link.query_local = true;
    return link;
  }
  return link;
}

bool addRefPathToGraph(GraphManager& graph, const std::vector<StateVec>& path,
                       const ExpandContext& ctx, double vertex_spacing,
                       std::vector<Vertex*>* path_vertices) {
  std::vector<RefPose> poses;
  poses.reserve(path.size());
  for (const StateVec& state : path) poses.push_back({state, nullptr});
  return addRefPath(graph, poses, ctx, vertex_spacing, path_vertices);
}

bool addRefPathToGraph(GraphManager& graph,
                       const std::vector<Vertex*>& path,
                       const ExpandContext& ctx, double vertex_spacing,
                       std::vector<Vertex*>* path_vertices) {
  std::vector<RefPose> poses;
  poses.reserve(path.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (path[i] == nullptr) return false;
    // Don't add the part of the path after the first hanging vertex
    // (rrg.cpp:4868).
    if (i > 0 && path[i]->is_hanging) break;
    poses.push_back({path[i]->state, path[i]});
  }
  return addRefPath(graph, poses, ctx, vertex_spacing, path_vertices);
}

namespace {

/// A base pose at driving height over mapped ground, as the planner takes
/// the robot's own pose. False when the map shows no ground under it, or
/// only ground more than a step above the base: a probe that starts inside
/// a wall or a table beside the robot finds its top, which the robot's base
/// was not standing on.
bool keyframeAtDrivingHeight(const ExpandContext& ctx, StateVec& state) {
  if (ctx.robot->type != RobotType::kGroundRobot) return true;
  Eigen::Vector3d position = state.head<3>();
  VoxelStatus status = VoxelStatus::kUnknown;
  const double ground = ctx.ground->projectSample(position, status);
  if (status != VoxelStatus::kOccupied || !std::isfinite(ground) ||
      ground < -ctx.planning->max_step_height) {
    return false;
  }
  state[0] = position[0];
  state[1] = position[1];
  state[2] = position[2] - (ground - ctx.planning->max_ground_height);
  return true;
}

}  // namespace

bool drivenEdgeTraversable(const ExpandContext& ctx, const Vertex& from,
                           const Vertex& to, ExpandGraphReport& rep) {
  // The robot physically drove this trajectory, so the volume its body
  // passed through was traversable, whether or not the map observed it: a
  // keyframe map carves free space only from its keyframes and leaves most
  // of the body volume near the ground unobserved, and requiring it known
  // refused nine in ten of the run-5 trajectory's edges. Unobserved space
  // therefore does not block, as it does not for the local lattice whose
  // paths the roadmap is otherwise made of. Everything that makes a
  // stretch undrivable still refuses it: mapped ground under the whole
  // edge, the slope, the cross slope, the footprint plane, observed ground
  // ahead, and known obstacles in the body's sweep. So segments where the
  // robot tipped, climbed a rock or went over a ledge still drop out.
  // The body is the robot's box turned along the edge: the map-aligned box
  // grown to hold it split the trajectory wherever the robot drove near a
  // wall.
  const Eigen::Vector3d origin = from.state.head<3>();
  const Eigen::Vector3d direction = to.state.head<3>() - origin;
  const double d_norm = direction.norm();
  const Eigen::Vector3d overshoot =
      d_norm > 1e-12
          ? Eigen::Vector3d(direction / d_norm * ctx.planning->edge_overshoot)
          : Eigen::Vector3d::Zero();
  const Eigen::Vector3d start = origin + ctx.robot->center_offset - overshoot;
  Eigen::Vector3d end = origin + ctx.robot->center_offset + direction;
  if (to.id != 0) end += overshoot;
  if (ctx.planning->geofence_checking_enable && ctx.geofence != nullptr &&
      ctx.geofence->getPathStatus(
          start.head<2>(), end.head<2>(), ctx.robot_box_size.head<2>()) ==
          GeofenceManager::CoordinateStatus::kViolated) {
    rep.status = ExpandGraphStatus::kErrorGeofenceViolated;
    return false;
  }
  OrientedBox body;
  body.heading = std::atan2(direction.y(), direction.x());
  body.size = ctx.robot_box_size;
  if (ctx.robot->type != RobotType::kGroundRobot) {
    return orientedBoxPathStatus(*ctx.map, start, end, body, false,
                                 nullptr) == VoxelStatus::kFree;
  }
  EdgeBodyCheck check;
  check.sweep = [&ctx, &body](const Eigen::Vector3d& a,
                              const Eigen::Vector3d& b) {
    return orientedBoxPathStatus(*ctx.map, a, b, body, false, nullptr);
  };
  std::vector<Eigen::Vector3d> projected;
  const ProjectedEdgeStatus status = ctx.ground->getProjectedEdgeStatus(
      start, end, ctx.robot_box_size, false, projected, false, false, &check,
      EdgeTravel::kBothWays);
  ++rep.edge_status[static_cast<int>(status)];
  return status == ProjectedEdgeStatus::kAdmissible;
}

RoadmapRebuildReport rebuildRoadmapFromTrajectory(
    GraphManager& graph, const std::vector<TrajectoryKeyframe>& keyframes,
    const ExpandContext& ctx, const RoadmapRebuildParams& params) {
  using Clock = std::chrono::steady_clock;
  const Clock::time_point started = Clock::now();
  RoadmapRebuildReport report;
  report.keyframes = static_cast<int>(keyframes.size());
  const auto finish = [&report, started]() {
    report.elapsed_s =
        std::chrono::duration<double>(Clock::now() - started).count();
    return report;
  };
  if (keyframes.empty() || graph.getNumVertices() != 0) return finish();

  // The keyframes with mapped ground under them, as base poses: every
  // vertex is dropped onto the ground from the robot's base, not from its
  // driving height, which for a tall robot can lie above an overhang it
  // drove under.
  // Where the robot tipped past max_inclination (the latest keyframe
  // aside): tipped keyframes are set apart before the ground is looked
  // for, since the ground under a tipped robot is not where its base says.
  // No edge may pass within the robot's half diagonal of them, and the
  // keyframe after them starts a new piece of the chain.
  const auto tipped = [&ctx](const TrajectoryKeyframe& keyframe) {
    return std::max(std::abs(keyframe.roll), std::abs(keyframe.pitch)) >
           ctx.planning->max_inclination;
  };
  struct Supported {
    TrajectoryKeyframe keyframe;
    /// The robot tipped since the previous supported keyframe.
    bool after_tipping = false;
    /// The latest keyframe, tipped.
    bool tipped = false;
  };
  std::vector<Supported> supported;
  supported.reserve(keyframes.size());
  std::vector<Eigen::Vector2d> tipped_at;
  StateVec home_state = StateVec::Zero();
  bool tipped_since = false;
  for (std::size_t i = 0; i < keyframes.size(); ++i) {
    const bool latest = i + 1 == keyframes.size();
    const bool is_tipped = i > 0 && tipped(keyframes[i]);
    if (is_tipped) {
      ++report.tilted_keyframes;
      if (!latest) {
        tipped_at.push_back(keyframes[i].pose.head<2>());
        tipped_since = true;
        continue;
      }
    }
    StateVec state = keyframes[i].pose;
    if (!keyframeAtDrivingHeight(ctx, state)) {
      ++report.unsupported_keyframes;
      if (i == 0) return finish();
      continue;
    }
    if (i == 0) home_state = state;
    supported.push_back({keyframes[i], tipped_since, is_tipped});
    tipped_since = false;
  }
  report.home_supported = true;

  const double tipped_clearance =
      0.5 * ctx.robot_box_size.head<2>().norm();
  const auto passes_tipping = [&tipped_at, tipped_clearance](
                                  const StateVec& a, const StateVec& b) {
    const Eigen::Vector2d from = a.head<2>();
    const Eigen::Vector2d along = b.head<2>() - from;
    const double length_sq = along.squaredNorm();
    for (const Eigen::Vector2d& p : tipped_at) {
      const double t =
          length_sq > 1e-12
              ? std::clamp((p - from).dot(along) / length_sq, 0.0, 1.0)
              : 0.0;
      if ((from + t * along - p).norm() < tipped_clearance) return true;
    }
    return false;
  };

  auto* home = new Vertex(0, home_state);
  home->robot_id = ctx.robot_id;
  home->type = VertexType::kVisited;
  graph.addVertex(home);
  std::vector<Vertex*> vertices{home};
  Vertex* previous = home;
  // The base pose the previous vertex was placed from.
  StateVec previous_base = supported.front().keyframe.pose;

  // Offsets across the direction of travel, nearest first.
  std::vector<double> offsets{0.0};
  const double offset_step = ctx.map->getResolution() > 0.0
                                 ? ctx.map->getResolution()
                                 : 0.2;
  for (double o = offset_step; o <= params.max_offset + 1e-9;
       o += offset_step) {
    offsets.push_back(o);
    offsets.push_back(-o);
  }
  const auto has_room = [&ctx](const StateVec& state) {
    return ctx.map->getBoxStatus(state.head<3>() + ctx.robot->center_offset,
                                 ctx.robot_box_size,
                                 false) != VoxelStatus::kOccupied;
  };
  // A vertex at `target` or beside it, chained to `previous` unless
  // `chain` is false (the robot tipped since). A robot against a wall or
  // wedged stood where its box overlaps the map's obstacles; beside that
  // spot there may be room, and every edge is checked in full wherever the
  // vertex goes.
  const auto place = [&](const StateVec& target, bool chain) {
    Eigen::Vector2d across(previous_base.y() - target.y(),
                           target.x() - previous_base.x());
    across = across.norm() > 1e-9 ? Eigen::Vector2d(across.normalized())
                                  : Eigen::Vector2d(0.0, 1.0);
    bool placed = false;
    bool linked = false;
    bool tried = false;
    bool near_tipping = false;
    StateVec chosen = target;
    ExpandGraphReport first_refusal;
    for (const double offset : offsets) {
      StateVec candidate = target;
      candidate[0] += offset * across.x();
      candidate[1] += offset * across.y();
      if (!keyframeAtDrivingHeight(ctx, candidate) || !has_room(candidate)) {
        continue;
      }
      if (!placed) {
        chosen = candidate;
        placed = true;
      }
      if (!chain) break;
      const double d =
          (candidate.head<3>() - previous->state.head<3>()).norm();
      if (d > ctx.planning->edge_length_max ||
          d <= ctx.planning->edge_length_min) {
        continue;
      }
      if (passes_tipping(previous->state, candidate)) {
        near_tipping = true;
        continue;
      }
      const Vertex probe(-1, candidate);
      ExpandGraphReport rep;
      const bool passes = drivenEdgeTraversable(ctx, *previous, probe, rep);
      if (!tried) first_refusal = rep;
      tried = true;
      if (passes) {
        chosen = candidate;
        linked = true;
        break;
      }
    }
    if (!placed) {
      ++report.boxed_vertices;
      return false;
    }
    previous_base = target;
    if ((chosen.head<2>() - target.head<2>()).norm() > 1e-9) {
      ++report.offset_vertices;
    }
    const double d = (chosen.head<3>() - previous->state.head<3>()).norm();
    auto* vertex = new Vertex(graph.generateVertexID(), chosen);
    vertex->robot_id = ctx.robot_id;
    vertex->type = VertexType::kVisited;
    // Distance travelled along the trajectory: the way round between two
    // vertices, which the link rule below reads.
    vertex->distance = previous->distance + d;
    graph.addVertex(vertex);
    vertices.push_back(vertex);
    if (linked) {
      vertex->parent = previous;
      previous->children.push_back(vertex);
      graph.addEdge(vertex, previous, d);
      ++report.chain_edges;
    } else if (!chain || (near_tipping && !tried)) {
      ++report.chain_edges_refused;
      ++report.chain_refusals_tilted;
    } else if (!tried) {
      ++report.chain_gaps;
    } else {
      ++report.chain_edges_refused;
      if (first_refusal.status == ExpandGraphStatus::kErrorGeofenceViolated) {
        ++report.chain_refusals_geofence;
      }
      for (int s = 1; s < 8; ++s) {
        report.chain_refusals_by_status[s] += first_refusal.edge_status[s];
      }
    }
    previous = vertex;
    return true;
  };

  // No chain edge across a stretch where the robot tipped (until a vertex
  // after it is placed), nor into the latest keyframe when it is tipped.
  bool break_pending = false;
  for (std::size_t i = 1; i < supported.size(); ++i) {
    const StateVec& state = supported[i].keyframe.pose;
    const bool latest = i + 1 == supported.size();
    if (supported[i].after_tipping) break_pending = true;
    if (break_pending || supported[i].tipped) {
      if (place(state, false)) break_pending = false;
      continue;
    }
    const double gap = (state.head<3>() - previous_base.head<3>()).norm();
    if ((gap < params.vertex_spacing && !latest) ||
        gap <= ctx.planning->edge_length_min) {
      continue;
    }
    // Keyframes far apart (a fast stretch): vertices in between, about a
    // spacing apart and no edge longer than edge_length_max.
    int pieces = static_cast<int>(
        std::ceil(gap / ctx.planning->edge_length_max - 1e-9));
    if (params.vertex_spacing > 0.0) {
      pieces = std::max(
          pieces, static_cast<int>(std::floor(gap / params.vertex_spacing)));
    }
    pieces = std::max(1, pieces);
    const StateVec from = previous_base;
    for (int k = 1; k < pieces; ++k) {
      StateVec between = from + (state - from) * (double(k) / pieces);
      between[3] = state[3];
      place(between, true);
    }
    place(state, true);
  }

  // Every vertex to the others within link_radius, nearest first.
  const double reach =
      std::min(params.link_radius, ctx.planning->edge_length_max);
  for (Vertex* vertex : vertices) {
    std::vector<Vertex*> near;
    if (!graph.getNearestVertices(&vertex->state, reach, &near)) continue;
    const Eigen::Vector3d at = vertex->state.head<3>();
    std::sort(near.begin(), near.end(),
              [&at](const Vertex* a, const Vertex* b) {
                return (a->state.head<3>() - at).squaredNorm() <
                       (b->state.head<3>() - at).squaredNorm();
              });
    for (Vertex* other : near) {
      if (other == nullptr || other == vertex) continue;
      const double d = (other->state.head<3>() - at).norm();
      if (d <= ctx.planning->edge_length_min || d >= reach) continue;
      if (graph.graph_->edgeExists(vertex->id, other->id)) continue;
      // As expandGraphEdges (rrg.cpp:907): a long way round and a height
      // change is a different floor, not a shortcut.
      if (std::abs(other->distance - vertex->distance) >
              ctx.planning->nearest_range_max &&
          std::abs(other->state[2] - vertex->state[2]) >
              ctx.planning->nearest_range_z) {
        continue;
      }
      if (passes_tipping(vertex->state, other->state)) continue;
      ExpandGraphReport rep;
      if (!drivenEdgeTraversable(ctx, *vertex, *other, rep)) continue;
      graph.addEdge(vertex, other, d);
      ++report.link_edges;
    }
  }
  report.vertices = graph.getNumVertices();

  // Connected parts, by a walk over the edge lists.
  std::unordered_map<int, int> component_of;
  for (Vertex* start : vertices) {
    if (component_of.count(start->id) > 0) continue;
    const int component = report.components++;
    int size = 0;
    std::vector<int> stack{start->id};
    component_of[start->id] = component;
    while (!stack.empty()) {
      const int id = stack.back();
      stack.pop_back();
      ++size;
      const auto edges = graph.edge_map_.find(id);
      if (edges == graph.edge_map_.end()) continue;
      for (const auto& [neighbour, cost] : edges->second) {
        (void)cost;
        if (component_of.emplace(neighbour, component).second) {
          stack.push_back(neighbour);
        }
      }
    }
    if (start == home) report.home_component_vertices = size;
  }
  return finish();
}

Vertex* connectGoalThroughLattice(GraphManager& graph, const StateVec& goal,
                                  const GridGraphParams& grid,
                                  const ExpandContext& ctx, double heading,
                                  const UsableVertexFn& usable,
                                  GoalLatticeReport* report) {
  GoalLatticeReport scratch_report;
  GoalLatticeReport& out = report != nullptr ? *report : scratch_report;
  out = GoalLatticeReport{};

  // The lattice itself is scratch; only the path through it that reaches the
  // roadmap is kept. Rooted at the goal, so every lattice vertex it reaches
  // has a verified way back to the goal.
  GraphManager lattice;
  auto* root = new Vertex(0, goal);
  root->robot_id = ctx.robot_id;
  lattice.addVertex(root);
  // One sweep offers every cell once, linked to whichever lattice vertex is
  // nearest at that moment, so a cell round a corner from the goal is offered
  // before anything that could reach it exists. Sweep again while the last
  // sweep added vertices: a cell that already has one is refused as a short
  // edge, so each pass only reaches round the next turn. The whole lattice
  // stays within one sweep's num_vertices_max.
  for (int pass = 0; pass < kMaxGoalLatticePasses; ++pass) {
    const GridGraphResult built =
        buildGridGraph(lattice, goal, grid, ctx, heading);
    if (built.status != GridGraphStatus::kOk) return nullptr;
    ++out.passes;
    if (built.vertices_added == 0 ||
        lattice.getNumVertices() >= ctx.planning->num_vertices_max) {
      break;
    }
  }
  out.lattice_vertices = lattice.getNumVertices();

  // Nearest the goal first, by distance through the lattice.
  ShortestPathsReport lattice_paths;
  std::vector<std::pair<double, Vertex*>> reachable{{0.0, root}};
  if (lattice.getNumVertices() > 1 &&
      lattice.findShortestPaths(0, lattice_paths) && lattice_paths.status) {
    for (const auto& entry : lattice.vertices_map_) {
      Vertex* vertex = entry.second;
      if (vertex == nullptr || vertex->id == 0) continue;
      const auto parent = lattice_paths.parent_id_map.find(vertex->id);
      // Dijkstra leaves an unreached vertex as its own parent.
      if (parent == lattice_paths.parent_id_map.end() ||
          parent->second == vertex->id) {
        continue;
      }
      reachable.emplace_back(lattice_paths.distance_map.at(vertex->id),
                             vertex);
    }
  }
  std::stable_sort(reachable.begin(), reachable.end(),
                   [](const auto& a, const auto& b) { return a.first < b.first; });
  out.reachable_vertices = static_cast<int>(reachable.size());

  const double reach = ctx.planning->edge_length_max;
  for (const auto& [lattice_distance, vertex] : reachable) {
    (void)lattice_distance;
    std::vector<Vertex*> candidates;
    if (!graph.getNearestVertices(&vertex->state, reach, &candidates)) continue;
    const Eigen::Vector3d at = vertex->state.head<3>();
    std::sort(candidates.begin(), candidates.end(),
              [&at](const Vertex* a, const Vertex* b) {
                return (a->state.head<3>() - at).squaredNorm() <
                       (b->state.head<3>() - at).squaredNorm();
              });
    for (Vertex* candidate : candidates) {
      if (candidate == nullptr || candidate->is_hanging) continue;
      if (usable && !usable(*candidate)) continue;
      const double gap = (candidate->state.head<3>() - at).norm();
      if (gap >= reach) continue;
      if (gap > 1e-9) {
        if (out.bridge_checks >= kMaxGoalBridgeChecks) {
          out.hit_check_limit = true;
          return nullptr;
        }
        ++out.bridge_checks;
        ExpandGraphReport rep;
        if (!roadmapEdgeTraversable(ctx, *vertex, *candidate, rep)) continue;
      }
      // Roadmap vertex, bridge, then the lattice path back to the goal.
      std::vector<StateVec> chain{candidate->state};
      std::vector<StateVec> through;
      if (vertex->id == 0) {
        through.push_back(goal);
      } else {
        lattice.getShortestPath(vertex->id, lattice_paths,
                                /*source_to_target_order=*/false, through);
      }
      chain.insert(chain.end(), through.begin(), through.end());
      // A spacing of zero keeps every lattice vertex: dropping one would
      // replace two checked lattice edges with an unchecked chord.
      std::vector<Vertex*> added;
      if (!addRefPathToGraph(graph, chain, ctx, /*vertex_spacing=*/0.0,
                             &added) ||
          added.empty()) {
        return nullptr;
      }
      out.chain_vertices = static_cast<int>(added.size());
      Vertex* goal_vertex = added.back();
      // An own roadmap vertex within kDeltaLimit of the goal would have taken
      // its place; that is not the requested endpoint.
      if ((goal_vertex->state.head<3>() - goal.head<3>()).squaredNorm() >
          1e-12) {
        return nullptr;
      }
      return goal_vertex;
    }
  }
  return nullptr;
}

std::vector<int> performShortestPathsClustering(
    GraphManager& graph, const ShortestPathsReport& rep,
    std::vector<Vertex*>& vertices, double dist_threshold,
    double principle_path_min_length, bool refinement_enable) {
  // Assume long paths are principal paths. Go over them one by one from the
  // longest, group by the normalised DTW distance, then refine by choosing the
  // closest valid principal path (rrg.cpp:5374).
  std::sort(vertices.begin(), vertices.end(),
            [&graph, &rep](const Vertex* a, const Vertex* b) {
              return graph.getShortestDistance(a->id, rep) >
                     graph.getShortestDistance(b->id, rep);
            });

  std::vector<PathType> cluster_paths;
  std::vector<int> cluster_ids;
  for (Vertex* vertex : vertices) {
    PathType path_cur;
    graph.getShortestPath(vertex->id, rep, true, path_cur);
    bool found_a_neighbour = false;
    for (std::size_t j = 0; j < cluster_paths.size(); ++j) {
      if (computeDistanceBetweenTwoTrajectories(path_cur, cluster_paths[j]) <=
          dist_threshold) {
        vertex->cluster_id = cluster_ids[j];
        found_a_neighbour = true;
        break;
      }
    }
    if (!found_a_neighbour) {
      // No neighbour: this path starts a new cluster as its principal path.
      cluster_paths.push_back(path_cur);
      cluster_ids.push_back(vertex->id);
      vertex->cluster_id = vertex->id;
    }
  }
  if (!refinement_enable) return cluster_ids;

  // Refinement: drop short principal paths, then give every vertex the
  // closest remaining one (rrg.cpp:5412).
  std::vector<PathType> cluster_paths_refine;
  std::vector<int> cluster_ids_refine;
  for (std::size_t j = 0; j < cluster_paths.size(); ++j) {
    if (getPathLength(cluster_paths[j]) >= principle_path_min_length) {
      cluster_paths_refine.push_back(cluster_paths[j]);
      cluster_ids_refine.push_back(cluster_ids[j]);
    }
  }
  for (Vertex* vertex : vertices) {
    PathType path_cur;
    graph.getShortestPath(vertex->id, rep, true, path_cur);
    double dist_min = std::numeric_limits<double>::infinity();
    for (std::size_t j = 0; j < cluster_paths_refine.size(); ++j) {
      const double dist_score = computeDistanceBetweenTwoTrajectories(
          path_cur, cluster_paths_refine[j]);
      if (dist_score < dist_min) {
        dist_min = dist_score;
        vertex->cluster_id = cluster_ids_refine[j];
      }
    }
  }
  return cluster_ids_refine;
}

FrontierAdditionReport addFrontiers(GraphManager& global_graph,
                                    GraphManager& local_graph,
                                    const ExpandContext& ctx,
                                    const RecomputeGainFn& recompute_gain,
                                    double vertex_spacing, double range_check,
                                    double update_radius) {
  FrontierAdditionReport report;

  // 2) Re-update the previous frontiers: still a frontier, or now
  // surrounded by known space (rrg.cpp:2411 to 2426). Upstream re-checked
  // every one; a merged peer's frontier is re-checked only near the new
  // local graph, where this robot's map has changed, and otherwise left to
  // its owner, whose next broadcast carries its type. Re-checked on this
  // robot's map, where the peer's explored space is unknown, they were never
  // demoted, and in run 6 their 529 re-checks took robot_3 4.3 s a plan.
  for (auto& entry : global_graph.vertices_map_) {
    Vertex* vertex = entry.second;
    if (vertex == nullptr || vertex->type != VertexType::kFrontier ||
        !global_graph.inService(*vertex)) {
      continue;
    }
    if (vertex->robot_id != ctx.robot_id) {
      std::vector<Vertex*> near_local;
      if (!local_graph.getNearestVertices(&vertex->state, update_radius,
                                          &near_local)) {
        ++report.global_frontiers_left_to_owners;
        continue;
      }
    }
    ++report.global_frontiers_rechecked;
    if (recompute_gain) recompute_gain(*vertex);
    if (!vertex->vol_gain.is_frontier) {
      vertex->type = VertexType::kUnvisited;
      ++report.global_frontiers_demoted;
    }
  }

  // 1) Potential frontiers at leaf vertices of the newly sampled local graph
  // (rrg.cpp:2428). Gain evaluation typed the frontiers; the leaves come
  // from the shortest-path tree.
  ShortestPathsReport local_rep;
  if (!local_graph.findShortestPaths(local_rep)) return report;
  local_graph.findLeafVertices(local_rep);
  std::vector<Vertex*> leaf_vertices;
  local_graph.getLeafVertices(leaf_vertices);
  std::vector<Vertex*> frontier_vertices;
  for (Vertex* vertex : leaf_vertices) {
    if (vertex != nullptr && vertex->type == VertexType::kFrontier) {
      frontier_vertices.push_back(vertex);
    }
  }
  report.local_frontiers = static_cast<int>(frontier_vertices.size());
  if (frontier_vertices.empty()) return report;

  // 3) Cluster the frontier paths and keep each cluster's principal path
  // (rrg.cpp:2443).
  const std::vector<int> cluster_ids = performShortestPathsClustering(
      local_graph, local_rep, frontier_vertices);
  report.clusters = static_cast<int>(cluster_ids.size());

  // 4) Add each principal path unless the area already has vertices, or the
  // robot has already passed through it (rrg.cpp:2449 to 2471).
  for (const int cluster_id : cluster_ids) {
    Vertex* leaf = local_graph.getVertex(cluster_id);
    if (leaf == nullptr) continue;
    Vertex* nearest_vertex = nullptr;
    if (global_graph.getNearestVertexInRange(&leaf->state, range_check,
                                             &nearest_vertex)) {
      continue;
    }
    std::vector<Vertex*> nearby;
    if (global_graph.getNearestVertices(&leaf->state, update_radius,
                                        &nearby)) {
      const bool passed = std::any_of(
          nearby.begin(), nearby.end(), [](const Vertex* vertex) {
            return vertex != nullptr && vertex->type == VertexType::kVisited;
          });
      if (passed) continue;
    }
    std::vector<Vertex*> path;
    local_graph.getShortestPath(cluster_id, local_rep, true, path);
    if (path.size() < 2) continue;
    // Only keep the frontier for the leaf vertex; the rest of the path is
    // ordinary roadmap (rrg.cpp:2463).
    for (auto pa = path.begin(); pa != path.end() - 1; ++pa) {
      (*pa)->type = VertexType::kUnvisited;
    }
    if (addRefPathToGraph(global_graph, path, ctx, vertex_spacing)) {
      ++report.paths_added;
    }
  }
  return report;
}

GlobalFrontierReport searchGlobalFrontier(
    GraphManager& graph, int source_id, int robot_id,
    const RecomputeGainFn& recompute_gain,
    const std::vector<Eigen::Vector3d>& excluded, double exclusion_radius,
    const Eigen::Vector3d* target) {
  GlobalFrontierReport report;

  // Re-check all frontiers against the current map (rrg.cpp:5612 to 5625).
  std::vector<Vertex*> global_frontiers;
  for (auto& entry : graph.vertices_map_) {
    Vertex* vertex = entry.second;
    if (vertex == nullptr || vertex->type != VertexType::kFrontier ||
        !graph.inService(*vertex)) {
      continue;
    }
    if (recompute_gain) recompute_gain(*vertex);
    if (!vertex->vol_gain.is_frontier) {
      vertex->type = VertexType::kUnvisited;
      ++report.demoted;
      continue;
    }
    global_frontiers.push_back(vertex);
  }
  report.frontiers = static_cast<int>(global_frontiers.size());
  if (global_frontiers.empty()) return report;

  // Dijkstra from the current vertex to all (rrg.cpp:5669).
  if (graph.vertices_map_.find(source_id) == graph.vertices_map_.end()) {
    return report;
  }
  ShortestPathsReport frontier_graph_rep;
  if (!graph.findShortestPaths(source_id, frontier_graph_rep)) return report;

  // Exploration gain of every reachable frontier (rrg.cpp:5766 to 5818).
  for (Vertex* frontier : global_frontiers) {
    const auto distance = frontier_graph_rep.distance_map.find(frontier->id);
    if (distance == frontier_graph_rep.distance_map.end() ||
        !std::isfinite(distance->second) ||
        distance->second >= std::numeric_limits<double>::max() / 2.0) {
      continue;  // not connected to where the robot stands
    }
    const bool is_excluded = std::any_of(
        excluded.begin(), excluded.end(),
        [frontier, exclusion_radius](const Eigen::Vector3d& center) {
          return (frontier->state.head<3>() - center).norm() <=
                 exclusion_radius;
        });
    if (is_excluded) continue;
    ++report.feasible;
    double exp_gain = frontier->vol_gain.gain *
                      std::exp(-kGlobalDistancePenalty * distance->second);
    if (frontier->robot_id != robot_id) exp_gain *= kGlobalOtherRobotPenalty;
    if (target != nullptr) {
      exp_gain *= std::exp(-kGlobalTargetPenalty *
                           (frontier->state.head<3>() - *target).norm());
    }
    if (exp_gain > report.best_gain) {
      report.best_gain = exp_gain;
      report.best_frontier = frontier;
      report.best_distance = distance->second;
    }
  }
  return report;
}

bool sampleVertex(RandomSampler& sampler, const StateVec& root_state,
                  const ExpandContext& ctx, Vertex& vertex) {
  // rrg.cpp:455 to 503, the RandomSampler overload.
  StateVec state = StateVec::Zero();
  bool hanging = false;
  bool found = false;

  int while_thres = 1000;  // magic number (rrg.cpp:461)
  while (!found && while_thres--) {
    hanging = false;
    sampler.generate(root_state, state);
    // rrg.cpp:468 rejected draws outside the world-fixed global bound, less
    // half a robot. The port's local graph has no such bound (its global
    // space is re-centred on the robot for gain scoring), so neither does
    // this; the geofence still applies to every edge in expandGraph.

    if (ctx.robot->type == RobotType::kGroundRobot) {
      Eigen::Vector3d sample = state.head<3>() + ctx.robot->center_offset;
      VoxelStatus vs;
      const double ground_dist = ctx.ground->projectSample(sample, vs);
      // Nothing within reach below the draw. The one-argument overload
      // rejected this (rrg.cpp:426); the three-argument one shifted the state
      // by the -1 sentinel, which no caller could use.
      if (vs == VoxelStatus::kFree || !std::isfinite(ground_dist)) continue;
      if (vs == VoxelStatus::kUnknown) hanging = true;
      sample[2] -= (ground_dist - ctx.planning->max_ground_height);
      state[0] = sample[0] - ctx.robot->center_offset[0];
      state[1] = sample[1] - ctx.robot->center_offset[1];
      state[2] -= (ground_dist - ctx.planning->max_ground_height);
    }

    // Check if the surrounding area is free (rrg.cpp:487). The unknown
    // policy is the one the local lattice applies to its cells.
    if (ctx.map->getBoxStatus(state.head<3>() + ctx.robot->center_offset,
                              ctx.robot_box_size,
                              !ctx.allow_unknown_lattice_body) ==
        VoxelStatus::kFree) {
      found = true;
    }
  }
  vertex.state = state;
  vertex.is_hanging = hanging;
  return found;
}

GlobalGraphExpansionReport expandGlobalGraph(
    GraphManager& global_graph, const ExpandContext& ctx,
    RandomSampler& sampler, const RobotStateHistory& robot_state_hist,
    const RecomputeGainFn& compute_gain, double time_budget_s) {
  // rrg.cpp:2535 to 2671.
  using Clock = std::chrono::steady_clock;
  const Clock::time_point time_lim = Clock::now();
  const auto elapsed = [time_lim]() {
    return std::chrono::duration<double>(Clock::now() - time_lim).count();
  };
  GlobalGraphExpansionReport report;

  // Extract unvisited vertices in the global graph (rrg.cpp:2565).
  std::vector<Vertex*> unvisited_vertices;
  for (auto& entry : global_graph.vertices_map_) {
    Vertex* vertex = entry.second;
    if (vertex != nullptr && vertex->type == VertexType::kUnvisited &&
        global_graph.inService(*vertex)) {
      unvisited_vertices.push_back(vertex);
    }
  }
  report.unvisited_vertices = static_cast<int>(unvisited_vertices.size());
  if (unvisited_vertices.empty()) {
    report.elapsed_s = elapsed();
    return report;
  }

  // Randomly choose a vertex, then group all nearby vertices within a local
  // box; repeat until the boxes cover every unvisited vertex (rrg.cpp:2574
  // to 2606). Each box is represented by the centroid of its vertices.
  constexpr double kLocalBoxRadiusSq = kLocalBoxRadius * kLocalBoxRadius;
  std::vector<Eigen::Vector3d> cluster_centroids;
  std::vector<Vertex*> unvisited_vertices_remain;
  while (!unvisited_vertices.empty()) {
    unvisited_vertices_remain.clear();
    const Eigen::Vector3d seed =
        unvisited_vertices[sampler.index(unvisited_vertices.size())]
            ->state.head<3>();
    Eigen::Vector3d cluster_center = Eigen::Vector3d::Zero();
    int num_vertices_in_cluster = 0;
    for (Vertex* vertex : unvisited_vertices) {
      if ((vertex->state.head<3>() - seed).squaredNorm() <=
          kLocalBoxRadiusSq) {
        cluster_center += vertex->state.head<3>();
        ++num_vertices_in_cluster;
      } else {
        unvisited_vertices_remain.push_back(vertex);
      }
    }
    cluster_centroids.push_back(cluster_center / num_vertices_in_cluster);
    unvisited_vertices.swap(unvisited_vertices_remain);
  }
  report.clusters = static_cast<int>(cluster_centroids.size());

  // Expand the global graph: one sample around each centroid per pass, for
  // as long as the budget lasts (rrg.cpp:2608 to 2668).
  while (elapsed() < time_budget_s) {
    ++report.passes;
    for (const Eigen::Vector3d& centroid : cluster_centroids) {
      const StateVec centroid_state(centroid.x(), centroid.y(), centroid.z(),
                                    0.0);
      Vertex new_vertex(-1, StateVec::Zero());
      if (!sampleVertex(sampler, centroid_state, ctx, new_vertex)) continue;
      if (new_vertex.is_hanging) continue;
      // rrg.cpp:2623 to 2631 dropped the sample onto the ground a second
      // time; sampleVertex has already done so, and expandGraph does it
      // again before checking the edge.

      // Only expand samples in sparse areas, not yet passed by the robot and
      // not close to any frontier (rrg.cpp:2632 to 2654).
      std::vector<const StateVec*> s_res;
      if (robot_state_hist.getNearestStates(new_vertex.state, kSparseRadius,
                                            &s_res)) {
        continue;
      }
      std::vector<Vertex*> v_res;
      if (global_graph.getNearestVertices(&new_vertex.state, kSparseRadius,
                                          &v_res)) {
        continue;
      }
      std::vector<Vertex*> f_res;
      if (global_graph.getNearestVertices(&new_vertex.state,
                                          kOverlappedFrontierRadius, &f_res) &&
          std::any_of(f_res.begin(), f_res.end(), [](const Vertex* vertex) {
            return vertex != nullptr &&
                   vertex->type == VertexType::kFrontier;
          })) {
        continue;
      }

      ++report.samples;
      ExpandGraphReport rep;
      expandGraph(global_graph, new_vertex, rep, ctx);
      if (rep.status != ExpandGraphStatus::kSuccess ||
          rep.vertex_added == nullptr) {
        continue;
      }
      // rrg.cpp:2660: the volumetric gain says whether the new vertex looks
      // into unknown space, which makes it a global frontier.
      if (compute_gain) compute_gain(*rep.vertex_added);
      if (rep.vertex_added->vol_gain.is_frontier) {
        rep.vertex_added->type = VertexType::kFrontier;
        ++report.frontiers_added;
      }
      report.vertices_added += rep.num_vertices_added;
      report.edges_added += rep.num_edges_added;
    }
  }

  report.elapsed_s = elapsed();
  return report;
}

}  // namespace mgg
