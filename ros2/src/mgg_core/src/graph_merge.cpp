#include "mgg_core/graph_merge.h"

#include <cstdio>
#include <string>
#include <utility>

#include "mgg_core/log.h"

namespace mgg {
namespace {

/// Applies the inter-robot transform to a neighbour's state. Unlike the ROS 1
/// version this carries z and yaw through the rotation, so robots need not
/// share a heading. The state arrives at ground height and leaves at this
/// robot's driving height above that ground.
StateVec placeState(const StateVec& state,
                    const Eigen::Isometry3d& t_ours_theirs,
                    double driving_height) {
  const Eigen::Vector3d p_theirs(state[0], state[1], state[2]);
  const Eigen::Vector3d p_ours = t_ours_theirs * p_theirs;
  // Yaw of the composed rotation about z.
  const Eigen::Matrix3d r =
      t_ours_theirs.linear() *
      Eigen::Matrix3d(Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()));
  const double yaw = std::atan2(r(1, 0), r(0, 0));
  return StateVec(p_ours.x(), p_ours.y(), p_ours.z() + driving_height, yaw);
}

/// Within this robot's step and grade limits, the rule ground projection
/// applies to its own edges (ground_projection.cpp).
bool climbable(const Vertex& a, const Vertex& b,
               const ReceiverPlatform& platform) {
  const Eigen::Vector3d ray = b.state.head<3>() - a.state.head<3>();
  const double rise = std::abs(ray.z());
  return rise <= platform.max_step_height + 1e-6 ||
         std::atan2(rise, ray.head<2>().norm()) <= platform.max_inclination;
}

Vertex* makeVertex(GraphManager& graph, const GraphExchangeVertex& v,
                   const StateVec& state) {
  auto* vertex = new Vertex(graph.generateVertexID(), state);
  vertex->vol_gain.num_unknown_voxels = v.num_unknown_voxels;
  vertex->vol_gain.num_occupied_voxels = v.num_occupied_voxels;
  vertex->vol_gain.num_free_voxels = v.num_free_voxels;
  vertex->vol_gain.is_frontier = v.is_frontier;
  vertex->robot_id = v.robot_id;
  if (v.is_frontier) vertex->type = VertexType::kFrontier;
  return vertex;
}

void refreshVertex(Vertex* vertex, const GraphExchangeVertex& v) {
  vertex->vol_gain.num_unknown_voxels = v.num_unknown_voxels;
  vertex->vol_gain.num_occupied_voxels = v.num_occupied_voxels;
  vertex->vol_gain.num_free_voxels = v.num_free_voxels;
  vertex->vol_gain.is_frontier = v.is_frontier;
  vertex->robot_id = v.robot_id;
  if (v.is_frontier) vertex->type = VertexType::kFrontier;
}

/// Looks a neighbour vertex up without the operator[] insertion that made the
/// ROS 1 version return nullptr for unknown ids.
Vertex* findNeighbourVertex(GraphManager& graph, int robot_id, int vertex_id) {
  auto robot_it = graph.vertex_by_robot_id_.find(robot_id);
  if (robot_it == graph.vertex_by_robot_id_.end()) return nullptr;
  auto vertex_it = robot_it->second.find(vertex_id);
  if (vertex_it == robot_it->second.end()) return nullptr;
  return vertex_it->second;
}

/// Adds the incoming edges, skipping any whose endpoints are not both present
/// and any this robot could not climb. Always deduplicated: an edge may
/// already be in the graph from an earlier snapshot, and GraphManager::addEdge
/// would append a second adjacency entry for it.
int addEdges(GraphManager& graph, const GraphExchange& incoming, int robot_id,
             const ReceiverPlatform& platform, int& unresolved,
             int& too_steep) {
  int added = 0;
  for (const GraphExchangeEdge& e : incoming.edges) {
    Vertex* source = findNeighbourVertex(graph, robot_id, e.source_id);
    Vertex* target = findNeighbourVertex(graph, robot_id, e.target_id);
    if (source == nullptr || target == nullptr) {
      ++unresolved;
      continue;
    }
    if (!climbable(*source, *target, platform)) {
      ++too_steep;
      continue;
    }
    graph.addNeighbourEdge(source, target, e.weight);
    ++added;
  }
  return added;
}

/// The nearest vertex within `radius` that is not `robot_id`'s.
Vertex* nearestOtherRobotVertex(GraphManager& graph, const StateVec& state,
                                int robot_id, double radius) {
  std::vector<Vertex*> candidates;
  if (!graph.getNearestVertices(&state, radius, &candidates)) return nullptr;
  Vertex* nearest = nullptr;
  double best = radius;
  for (Vertex* candidate : candidates) {
    if (candidate == nullptr || candidate->robot_id == robot_id) continue;
    const double d = (candidate->state.head<3>() - state.head<3>()).norm();
    if (d <= best) {
      best = d;
      nearest = candidate;
    }
  }
  return nearest;
}

/// The neighbour restarted when a vertex it sent before is missing from this
/// complete snapshot, or has moved in its own frame.
bool neighbourRestarted(const GraphManager::NeighbourPlacement& placement,
                        const GraphExchange& incoming) {
  std::unordered_map<int, const GraphExchangeVertex*> by_id;
  by_id.reserve(incoming.vertices.size());
  for (const GraphExchangeVertex& v : incoming.vertices) by_id[v.id] = &v;
  for (const auto& [id, sent] : placement.sent_states) {
    const auto found = by_id.find(id);
    if (found == by_id.end()) return true;
    if ((found->second->state.head<2>() - sent.head<2>()).norm() >
        kNeighbourRestartToleranceM) {
      return true;
    }
  }
  return false;
}

/// Moves the neighbour's merged vertices to where `t_ours_theirs` puts them,
/// if it moves any by more than kNeighbourReplaceToleranceM. One pass over
/// the neighbour's vertices; the index is rebuilt once, only on a move.
int replaceNeighbourVertices(GraphManager& graph, int robot_id,
                             GraphManager::NeighbourPlacement& placement,
                             const Eigen::Isometry3d& t_ours_theirs,
                             double driving_height) {
  const auto merged = graph.vertex_by_robot_id_.find(robot_id);
  if (merged == graph.vertex_by_robot_id_.end()) return 0;
  bool moved = false;
  for (const auto& [id, sent] : placement.sent_states) {
    const auto vertex = merged->second.find(id);
    if (vertex == merged->second.end() || vertex->second == nullptr) continue;
    const StateVec placed = placeState(sent, t_ours_theirs, driving_height);
    if ((placed.head<3>() - vertex->second->state.head<3>()).norm() >
        kNeighbourReplaceToleranceM) {
      moved = true;
      break;
    }
  }
  if (!moved) return 0;
  int replaced = 0;
  for (const auto& [id, sent] : placement.sent_states) {
    const auto vertex = merged->second.find(id);
    if (vertex == merged->second.end() || vertex->second == nullptr) continue;
    vertex->second->state = placeState(sent, t_ours_theirs, driving_height);
    ++replaced;
  }
  graph.rebuildNearestIndex();
  return replaced;
}

}  // namespace

void StaticPoseSource::setTransform(int robot_id,
                                    const Eigen::Isometry3d& t_ours_theirs) {
  transforms_[robot_id] = t_ours_theirs;
}

void StaticPoseSource::setOffset(int robot_id, double dx, double dy,
                                 double dz) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(dx, dy, dz);
  transforms_[robot_id] = t;
}

void StaticPoseSource::clearTransform(int robot_id) {
  transforms_.erase(robot_id);
}

bool StaticPoseSource::getRobotTransform(
    int robot_id, Eigen::Isometry3d& t_ours_theirs) const {
  auto it = transforms_.find(robot_id);
  if (it == transforms_.end()) return false;
  t_ours_theirs = it->second;
  return true;
}

MergeResult mergeNeighbourGraph(GraphManager& global_graph,
                                const GraphExchange& incoming,
                                const PoseSource& poses,
                                const EdgeAdmissibleFn& is_admissible,
                                double rendezvous_radius,
                                const ReceiverPlatform& platform) {
  MergeResult result;
  if (incoming.vertices.empty()) return result;
  const int neighbour_id = incoming.vertices.front().robot_id;

  // Before any guard: a restarted planner's first snapshots hold only its new
  // root, and must still cut out what was merged from its old run.
  auto placement = global_graph.neighbour_placements_.find(neighbour_id);
  if (placement != global_graph.neighbour_placements_.end() &&
      neighbourRestarted(placement->second, incoming)) {
    // Its vertex ids now name other places: what came from the old run is
    // cut out before anything of the new one is merged.
    global_graph.retireNeighbourGraph(neighbour_id);
    result.neighbour_restarted = true;
    placement = global_graph.neighbour_placements_.end();
  }

  // Same guard as the ROS 1 version: nothing useful to connect to yet.
  if (global_graph.vertices_map_.size() < 2 || incoming.vertices.size() < 2) {
    return result;
  }

  Eigen::Isometry3d t_ours_theirs;
  if (!poses.getRobotTransform(neighbour_id, t_ours_theirs)) {
    // With a SLAM-backed PoseSource this is the normal state of affairs until
    // the two robots have actually seen the same place.
    result.transform_unavailable = true;
    return result;
  }

  const double driving_height = platform.driving_height;

  if (placement != global_graph.neighbour_placements_.end()) {
    result.vertices_replaced = replaceNeighbourVertices(
        global_graph, neighbour_id, placement->second, t_ours_theirs,
        driving_height);
    if (result.vertices_replaced > 0) {
      // Every edge was judged where its vertices stood before: the links
      // joining its roadmap to the rest against this robot's map, its own
      // edges against this robot's step and grade limits, which a tilted
      // transform changes. All are dropped; the rendezvous is looked for
      // again and its edges re-read from this complete snapshot.
      global_graph.cutNeighbourEdges(neighbour_id);
      global_graph.merged_graphs_[neighbour_id] = false;
    }
  }

  const bool already_merged = global_graph.merged_graphs_[neighbour_id];
  if (!already_merged) {
    // Hunt for a rendezvous: an incoming vertex close enough to one of ours
    // that the robot could actually drive between them.
    for (const GraphExchangeVertex& v : incoming.vertices) {
      Vertex* existing = findNeighbourVertex(global_graph, v.robot_id, v.id);
      const StateVec state =
          existing != nullptr ? existing->state
                              : placeState(v.state, t_ours_theirs,
                                           driving_height);
      Vertex* nearest = existing != nullptr
                            ? nearestOtherRobotVertex(global_graph, state,
                                                      neighbour_id,
                                                      rendezvous_radius)
                            : nullptr;
      if (existing == nullptr &&
          !global_graph.getNearestVertexInRange(&state, rendezvous_radius,
                                                &nearest)) {
        continue;
      }
      if (nearest == nullptr) continue;
      const Eigen::Vector3d origin(nearest->state[0], nearest->state[1],
                                   nearest->state[2]);
      const Eigen::Vector3d target(state[0], state[1], state[2]);
      if (!is_admissible(origin, target)) continue;

      global_graph.merged_graphs_[neighbour_id] = true;
      if (existing != nullptr) {
        global_graph.addNeighbourEdge(existing, nearest,
                                      (target - origin).norm());
        continue;
      }
      Vertex* new_vertex = makeVertex(global_graph, v, state);
      global_graph.addNeighbourVertex(new_vertex, v.id);
      // Form a tree as the first step, as the original did.
      new_vertex->parent = nearest;
      new_vertex->distance = nearest->distance + (target - origin).norm();
      nearest->children.push_back(new_vertex);
      global_graph.addEdge(new_vertex, nearest, (target - origin).norm());
      ++result.vertices_added;
    }
  }

  result.merged = global_graph.merged_graphs_[neighbour_id];
  result.newly_connected = !already_merged && result.merged;
  if (!result.merged) return result;
  // Joined again with a current transform: a quarantine ends here.
  global_graph.releaseNeighbourGraph(neighbour_id);

  // Connected: take everything else the neighbour knows.
  auto& sent_states =
      global_graph.neighbour_placements_[neighbour_id].sent_states;
  for (const GraphExchangeVertex& v : incoming.vertices) {
    Vertex* existing = findNeighbourVertex(global_graph, v.robot_id, v.id);
    if (existing == nullptr) {
      StateVec state = placeState(v.state, t_ours_theirs, driving_height);
      global_graph.addNeighbourVertex(makeVertex(global_graph, v, state), v.id);
      ++result.vertices_added;
    } else {
      refreshVertex(existing, v);
      ++result.vertices_updated;
    }
    sent_states[v.id] = v.state;
  }

  result.edges_added =
      addEdges(global_graph, incoming, neighbour_id, platform,
               result.edges_unresolved, result.edges_too_steep);

  if (result.edges_unresolved > 0) {
    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "merge with robot %d: skipped %d edge(s) naming a vertex "
                  "absent from the same message",
                  neighbour_id, result.edges_unresolved);
    logWarn(buf);
  }
  return result;
}

}  // namespace mgg
