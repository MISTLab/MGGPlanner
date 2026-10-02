#include "mgg_core/planning_cancellation.h"
#include "mgg_core/local_route.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "mgg_core/departure.h"
#include "mgg_core/global_graph.h"
#include "mgg_core/ground_projection.h"
#include "mgg_core/path_turns.h"

namespace mgg {

StateVec localRouteRoot(const ExpandContext& ctx, const StateVec& robot_pose,
                        bool& hanging) {
  StateVec root = robot_pose;
  hanging = false;
  if (ctx.robot->type != RobotType::kGroundRobot) return root;
  Eigen::Vector3d pos = robot_pose.head<3>();
  VoxelStatus status = VoxelStatus::kUnknown;
  const double ground_height = ctx.ground->projectSample(pos, status);
  if (status == VoxelStatus::kOccupied) {
    root[0] = pos.x();
    root[1] = pos.y();
    root[2] = pos.z() - (ground_height - ctx.planning->max_ground_height);
    return root;
  }
  // The base sits half the body height above the floor it stands on; the
  // driving height is max_ground_height above that floor.
  hanging = true;
  root = robot_pose;
  root[2] += ctx.planning->max_ground_height - ctx.robot->size[2] / 2.0;
  return root;
}

namespace {

// Lazy Dijkstra on the ground-relative eight-neighbour lattice. An edge is
// evaluated only when its source is popped, and the search ends only when
// the goal is popped, never when it is first discovered. No heuristic.
Vertex* lazyGroundLattice(GraphManager& graph, Vertex* root,
                          const StateVec& goal, const GridGraphParams& grid,
                          const ExpandContext& ctx, GridGraphResult& result) {
  if ((grid.resolution.array() <= 0).any() ||
      (grid.min_val.array() > 0).any() || (grid.max_val.array() < 0).any()) {
    result.status = GridGraphStatus::kInvalidBounds;
    return nullptr;
  }
  PlanProfile* profile = ctx.ground->profile();
  ProfileScope timed(profile ? &profile->lattice : nullptr);
  PlanningParams planning = *ctx.planning;
  planning.rr_mode = RRModeType::kTree;  // this loop, not expandGraph, wires neighbours
  ExpandContext lazy = ctx;
  lazy.planning = &planning;
  // Match buildGridGraph's precheck, including its occupied AABB fallback.
  // Explicit strict endpoint checks supplied by the caller remain in force.
  const auto endpoint_admissible = [&](const StateVec& from, const StateVec& to) {
    ProfileScope timed_precheck(profile ? &profile->cell_prechecks : nullptr);
    OrientedBox body;
    body.heading = std::atan2(to.y() - from.y(), to.x() - from.x());
    body.size = ctx.robot_box_size;
    const Eigen::Vector3d center = to.head<3>() + ctx.robot->offsetForHeading(body.heading);
    if (ctx.unknown_body_above_center)
      return orientedBoxPathStatus(*ctx.map, center, center, body, !ctx.allow_unknown_lattice_body, nullptr,
                                    false, ctx.unknown_body_above_center, ctx.own_body_known_free.get()) == VoxelStatus::kFree;
    auto status = ctx.map->getBoxStatus(center, body.size, !ctx.allow_unknown_lattice_body);
    if (status == VoxelStatus::kUnknown && ctx.own_body_known_free)
      status = orientedBoxPathStatus(*ctx.map, center, center, body, true,
          nullptr, false, std::nullopt, ctx.own_body_known_free.get());
    if (status == VoxelStatus::kOccupied && !ctx.map->dynamicBoxBlocked(center, body.size))
      status = orientedBoxPathStatus(*ctx.map, center, center, body,
                                      !ctx.allow_unknown_lattice_body, nullptr);
    return status == VoxelStatus::kFree;
  };
  EdgeVerdictCache verdicts;
  lazy.edge_verdicts = &verdicts;
  lazy.edge_cost = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    const double yaw = std::atan2(b.y() - a.y(), b.x() - a.x());
    const Eigen::Vector3d offset = ctx.robot->offsetForHeading(yaw);
    double cost = ctx.ground->clearanceCost(a + offset, b + offset, ctx.robot_box_size, 1);
    if (ctx.robot->center_offset.head<2>().squaredNorm() > 0.0) {
      const Eigen::Vector3d reverse = ctx.robot->offsetForHeading(yaw + M_PI);
      cost = std::max(cost, ctx.ground->clearanceCost(
          b + reverse, a + reverse, ctx.robot_box_size, 1));
    }
    // NAVIGATE prefers clearance without accepting metre-scale detours for
    // centimetres of extra gap. Retain a nonnegative weighted Dijkstra
    // cost (at most 1.1 times length), independently of exploration's weight.
    // One midpoint per short stencil edge is a soft preference only; hard
    // sweeps and the shortcut minimum still check their full sample sets.
    const double length = (b - a).norm();
    return length + 0.025 * (cost - length);
  };
  using Cell = std::pair<int, int>;
  using Entry = std::pair<double, int>;
  std::map<Cell, std::vector<Vertex*>> cells{{{0, 0}, {root}}};
  std::unordered_map<int, Cell> cell_of{{root->id, {0, 0}}};
  std::unordered_map<int, double> distance{{root->id, 0.0}};
  std::unordered_set<int> settled;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  open.emplace(0.0, root->id);
  Vertex* goal_vertex = nullptr;
  const double c = std::cos(root->state[3]), s = std::sin(root->state[3]);
  int loops = 0;
  const auto count = [&](const ExpandGraphReport& rep) {
    result.vertices_added += rep.num_vertices_added;
    result.edges_added += rep.num_edges_added;
    result.free_cells += rep.num_vertices_added;
    if (rep.no_ground) ++result.no_ground;
    for (int i = 0; i < 8; ++i) result.edge_status[i] += rep.edge_status[i];
  };
  const auto connect_existing = [&](Vertex* from, Vertex* to) {
    if (from == to || graph.graph_->edgeExists(from->id, to->id)) return;
    ExpandGraphReport rep;
    if (latticeEdgeTraversable(lazy, *from, *to, rep)) {
      graph.addEdge(from, to, lazy.edge_cost(from->state.head<3>(), to->state.head<3>()));
      ++result.edges_added;
    }
    count(rep);
  };
  while (!open.empty()) {
    planningCheckpoint();
    if (ctx.deadline && std::chrono::steady_clock::now() >= *ctx.deadline) {
      result.hit_limit = result.hit_deadline = true;
      if (profile) profile->budget_exhausted = true;
      return nullptr;
    }
    const auto [cost, id] = open.top();
    open.pop();
    if (cost != distance.at(id) || !settled.insert(id).second) continue;
    Vertex* at = graph.getVertex(id);
    if (at == goal_vertex ||
        (at->state.head<3>() - goal.head<3>()).squaredNorm() <= 1e-12) return at;
    if (++loops > planning.num_loops_max) { result.hit_limit = true; break; }

    // Any settled reachable vertex within the ordinary edge reach may
    // link the exact goal. A later cheaper arrival still relaxes its cost.
    if ((at->state.head<3>() - goal.head<3>()).norm() <= planning.edge_length_max) {
      ProfileScope link_time(profile ? &profile->goal_link : nullptr);
      if (goal_vertex) {
        connect_existing(at, goal_vertex);
      } else if (endpoint_admissible(at->state, goal)) {
        Vertex candidate(-1, goal);
        ExpandGraphReport rep;
        expandGraphFrom(graph, candidate, at, rep, lazy, true);
        count(rep);
        if (rep.vertex_added &&
            (rep.vertex_added->state.head<3>() - goal.head<3>()).squaredNorm() <= 1e-12) {
          goal_vertex = rep.vertex_added;
        } else if (rep.vertex_added) {
          return nullptr;
        }
      }
    }
    const Cell current = cell_of.at(id);
    // A physical root may overlap old occupied cells: a checked escape
    // must END clear, so one pitch can be too short even when a longer
    // edge leaves safely. Preserve ordinary root departure reach; only
    // non-root cells use the eight-neighbour stencil.
    const int reach_x = id == 0 ? static_cast<int>(std::ceil(std::min(
        planning.edge_length_max, std::max(-grid.min_val.x(), grid.max_val.x())) /
        grid.resolution.x())) : 1;
    const int reach_y = id == 0 ? static_cast<int>(std::ceil(std::min(
        planning.edge_length_max, std::max(-grid.min_val.y(), grid.max_val.y())) /
        grid.resolution.y())) : 1;
    for (int dx = -reach_x; dx <= reach_x; ++dx) {
      for (int dy = -reach_y; dy <= reach_y; ++dy) {
        planningCheckpoint();
        if (dx == 0 && dy == 0) continue;
        const Cell next{current.first + dx, current.second + dy};
        const double x = next.first * grid.resolution.x();
        const double y = next.second * grid.resolution.y();
        if (x < grid.min_val.x() || x > grid.max_val.x() ||
            y < grid.min_val.y() || y > grid.max_val.y()) continue;
        StateVec state(root->state.x() + c * x - s * y,
                       root->state.y() + s * x + c * y,
                       at->state.z(), root->state[3]);
        if ((state.head<2>() - at->state.head<2>()).norm() > planning.edge_length_max)
          continue;
        Eigen::Vector3d projected = state.head<3>();
        VoxelStatus support;
        const double height = ctx.ground->projectSample(projected, support);
        if (support != VoxelStatus::kOccupied) continue;
        state.head<3>() = projected;
        state.z() -= height - planning.max_ground_height;
        // Like LatticeColumnGround, merge only within max_step_height.
        // The same XY can be reached later on a ramp/deck above the floor.
        Vertex* existing = nullptr;
        const auto found = cells.find(next);
        if (found != cells.end()) for (Vertex* vertex : found->second) {
          if (std::abs(vertex->state.z() - state.z()) <= planning.max_step_height) {
            existing = vertex;
            break;
          }
        }
        if (existing) {
          if (!settled.count(existing->id)) connect_existing(at, existing);
          continue;
        }
        if (graph.getNumVertices() >= planning.num_vertices_max ||
            graph.getNumEdges() >= planning.num_edges_max) {
          result.hit_limit = true;
          continue;
        }
        if (!endpoint_admissible(at->state, state)) continue;
        // expandGraphFrom clips before projecting. Offer the raw XY step
        // at the parent's height, as the eager lattice does, so projection
        // cannot clip a slope edge off the cell whose key we store.
        StateVec raw = state;
        raw.z() = at->state.z();
        Vertex candidate(-1, raw);
        ExpandGraphReport rep;
        expandGraphFrom(graph, candidate, at, rep, lazy, true);
        count(rep);
        if (rep.vertex_added) {
          cells[next].push_back(rep.vertex_added);
          cell_of.emplace(rep.vertex_added->id, next);
        }
      }
    }
    const auto edges = graph.edge_map_.find(id);
    if (edges == graph.edge_map_.end()) continue;
    for (const auto& [to, weight] : edges->second) {
      if (settled.count(to)) continue;
      const double proposed = cost + weight;
      const auto known = distance.find(to);
      if (known == distance.end() || proposed < known->second) {
        distance[to] = proposed;
        open.emplace(proposed, to);
      }
    }
  }
  return nullptr;
}

}  // namespace

LocalRouteResult routeOverLocalLattice(GraphManager& graph,
                                       const StateVec& robot_pose,
                                       const StateVec& goal,
                                       const GridGraphParams& grid,
                                       const ExpandContext& ctx) {
  LocalRouteResult result;
  graph.reset();
  if (ctx.inclinations != nullptr) ctx.inclinations->clear();
  bool root_hanging = false;
  const StateVec root_state = localRouteRoot(ctx, robot_pose, root_hanging);
  auto* root = new Vertex(0, root_state);
  root->robot_id = ctx.robot_id;
  root->is_hanging = root_hanging;
  graph.addVertex(root);
  PlanProfile* const profile = ctx.ground ? ctx.ground->profile() : nullptr;
  StateVec goal_state = goal;
  if (ctx.robot->type == RobotType::kGroundRobot) {
    Eigen::Vector3d pos = goal.head<3>();
    VoxelStatus status = VoxelStatus::kUnknown;
    const double ground_height = ctx.ground->projectGoal(pos, status);
    if (status != VoxelStatus::kOccupied) {
      result.reason = "no mapped ground under the goal";
      return result;
    }
    goal_state[0] = pos.x();
    goal_state[1] = pos.y();
    goal_state[2] = pos.z() - (ground_height - ctx.planning->max_ground_height);
  }
  Vertex* goal_vertex = nullptr;
  if (ctx.robot->type == RobotType::kGroundRobot) {
    if ((grid.resolution.array() <= 0).any() ||
        (grid.min_val.array() > 0).any() || (grid.max_val.array() < 0).any()) {
      result.lattice.status = GridGraphStatus::kInvalidBounds;
      result.reason = "invalid local lattice bounds";
      return result;
    }
    // A lattice-policy shortcut can establish the complete straight route
    // before spending time on cells unrelated to this point goal. Demand
    // the configured soft clearance margin, not just absence of collision.
    {
      ProfileScope timed(profile ? &profile->goal_link : nullptr);
      const auto from = root_state.head<3>().eval();
      const auto to = goal_state.head<3>().eval();
      const double yaw = std::atan2(to.y() - from.y(), to.x() - from.x());
      const Eigen::Vector3d offset = ctx.robot->offsetForHeading(yaw);
      const auto turns = pathTurns({from, to}, root_state[3],
          std::max(ctx.robot->size.x(), ctx.robot->size.y()));
      const bool start_turn_ok = turns.empty() || turns.front() <= kSharpTurnRad + 1e-9 ||
          (roomToTurn(*ctx.map, *ctx.robot, *ctx.planning, root_state,
                      ctx.ground->standingStart()) &&
           groundSlope(*ctx.ground, from,
               std::max(ctx.robot->size.x(), ctx.robot->size.y()), &graph) <=
               kLevelGroundSlopeRad);
      if (start_turn_ok && (to - from).norm() > 1e-9 &&
          groundShortcutSegmentAdmissible(ctx, from, to,
              ctx.stop_at_unknown || !ctx.allow_unknown_lattice_body) &&
          ctx.ground->segmentClearance(from + offset, to + offset, ctx.robot_box_size) + 1e-9 >=
              std::min(ctx.planning->path_clearance_margin, 1.0)) {
        goal_vertex = new Vertex(graph.generateVertexID(), goal_state);
        goal_vertex->robot_id = ctx.robot_id;
        goal_vertex->parent = root;
        goal_vertex->distance = (to - from).norm();
        root->children.push_back(goal_vertex);
        graph.addVertex(goal_vertex);
        graph.addEdge(root, goal_vertex,
                      ctx.ground->clearanceCost(from + offset, to + offset, ctx.robot_box_size));
        result.lattice.vertices_added = result.lattice.edges_added = 1;
      }
    }
    if (!goal_vertex) {
      goal_vertex = lazyGroundLattice(graph, root, goal_state, grid, ctx, result.lattice);
    }
  } else {
    // Aerial planning retains the full 3D lattice and its original linker.
    result.lattice = buildGridGraph(graph, root_state, grid, ctx, robot_pose[3]);
    if (result.lattice.status == GridGraphStatus::kInvalidBounds ||
        graph.getNumVertices() <= 1) {
      result.reason = "the local lattice holds no admissible cell";
      return result;
    }
    goal_vertex = linkGoalToLattice(graph, goal_state, ctx, {});
  }
  if (goal_vertex == nullptr) {
    result.reason = "goal cannot be linked to the local lattice";
    return result;
  }
  ShortestPathsReport rep;
  bool searched = false;
  {
    ProfileScope timed(profile ? &profile->search : nullptr);
    searched = graph.findShortestPaths(0, rep);
  }
  if (!searched || !rep.status ||
      (goal_vertex->id != 0 &&
       rep.parent_id_map.find(goal_vertex->id) == rep.parent_id_map.end())) {
    result.reason = "no route through the local lattice reaches the goal";
    return result;
  }
  graph.getShortestPath(goal_vertex->id, rep, true, result.route);
  if (result.route.size() < 2) {
    result.route.clear();
    result.reason = "already at the goal";
    return result;
  }
  result.routed = true;
  return result;
}

bool groundShortcutSegmentAdmissible(const ExpandContext& ctx,
                                     const Eigen::Vector3d& from,
                                     const Eigen::Vector3d& to,
                                     bool stop_at_unknown) {
  OrientedBox body;
  body.heading = std::atan2(to.y() - from.y(), to.x() - from.x());
  body.size = ctx.robot_box_size;
  EdgeBodyCheck check;
  check.sweep = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return orientedBoxPathStatus(*ctx.map, a, b, body, stop_at_unknown,
                                 nullptr, true, ctx.unknown_body_above_center, ctx.own_body_known_free.get());
  };
  std::vector<Eigen::Vector3d> projected;
  const Eigen::Vector3d offset = ctx.robot->offsetForHeading(body.heading);
  // Driven from `from` to `to`: the shortcut is the path itself.
  if (ctx.planning->geofence_checking_enable && ctx.geofence &&
      ctx.geofence->getPathStatus((from + offset).head<2>(),
          (to + offset).head<2>(), ctx.robot_box_size.head<2>()) ==
              GeofenceManager::CoordinateStatus::kViolated) return false;
  return ctx.ground->getProjectedEdgeStatus(
             from + offset, to + offset,
             ctx.robot_box_size, stop_at_unknown, projected, false, false,
             &check, EdgeTravel::kForward) == ProjectedEdgeStatus::kAdmissible &&
         (!ctx.projected_edge_admissible || ctx.projected_edge_admissible(projected));
}

Vertex* linkGoalToLattice(GraphManager& graph, const StateVec& goal_state,
                          const ExpandContext& ctx,
                          const std::function<bool(const Vertex&)>& reached) {
  if (ctx.robot->type == RobotType::kAerialRobot) {
    return connectStateToGraph(graph, goal_state, ctx, kLocalGoalLinkRadius,
                               /*exact_state=*/true);
  }
  // A point goal is not a previously driven pose: even sub-cell links
  // require the lattice's complete oriented body and terrain checks.
  std::vector<Vertex*> around;
  if (!graph.getNearestVertices(&goal_state, ctx.planning->edge_length_max,
                                &around)) {
    return nullptr;
  }
  std::sort(around.begin(), around.end(),
            [&goal_state](const Vertex* a, const Vertex* b) {
              return (a->state.head<3>() - goal_state.head<3>()).squaredNorm() <
                     (b->state.head<3>() - goal_state.head<3>()).squaredNorm();
            });
  int tried = 0;
  for (Vertex* parent : around) {
    planningCheckpoint();
    if (parent == nullptr || (reached && !reached(*parent))) continue;
    if ((parent->state.head<3>() - goal_state.head<3>()).squaredNorm() <= 1e-12) {
      return parent;
    }
    if (tried++ >= kGoalLinkCandidates) break;
    ExpandGraphReport rep;
    Vertex candidate(-1, goal_state);
    expandGraphFrom(graph, candidate, parent, rep, ctx,
                    /*allow_short_edge=*/true);
    if (rep.status != ExpandGraphStatus::kSuccess || rep.vertex_added == nullptr) {
      continue;
    }
    // Within edge_length_max nothing is clipped; the projected endpoint is
    // the goal's own driving height.
    if ((rep.vertex_added->state.head<3>() - goal_state.head<3>())
            .squaredNorm() <= 1e-12) {
      return rep.vertex_added;
    }
  }
  return nullptr;
}

}  // namespace mgg
