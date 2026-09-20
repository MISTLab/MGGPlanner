#include "mgg_core/global_graph.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "mgg_core/trajectory.h"

namespace mgg {
namespace {

/// rrg.cpp:4831 and 4832.
constexpr double kDeltaLimit = 0.1;
constexpr double kRadiusLimit = 0.5;

/// A pose to fold in, with the local vertex it came from when there is one.
struct RefPose {
  StateVec state = StateVec::Zero();
  const Vertex* source = nullptr;
};

/// The straight segment between two driving-height states runs through
/// space the map knows to be occupied.
bool throughKnownObstacle(const ExpandContext& ctx, const StateVec& from,
                          const StateVec& to) {
  return ctx.map->getPathStatus(from.head<3>() + ctx.robot->center_offset,
                                to.head<3>() + ctx.robot->center_offset,
                                ctx.robot_box_size, false) ==
         VoxelStatus::kOccupied;
}

/// rrg.cpp:4817 to 4862, the "add root vertex first" block shared by
/// addRefPathToGraph and connectStateToGraph (rrg.cpp:5471 to 5510): the
/// nearest vertex itself when the state sits on it, a blind edge when it is
/// within `blind_radius`, otherwise a checked expandGraph.
Vertex* linkStateToGraph(GraphManager& graph, const StateVec& state,
                         const ExpandContext& ctx, double blind_radius) {
  Vertex* nearest_vertex = nullptr;
  if (!graph.getNearestVertex(&state, &nearest_vertex) ||
      nearest_vertex == nullptr) {
    return nullptr;
  }
  if ((state.head<3>() - nearest_vertex->state.head<3>()).norm() <=
      kDeltaLimit) {
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
          throughKnownObstacle(ctx, candidate->state, state)) {
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
  return rep.status == ExpandGraphStatus::kSuccess ? rep.vertex_added
                                                    : nullptr;
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

  // Keep about one pose per vertex_spacing. Upstream's path was the lattice
  // walk itself, one vertex per grid cell (rrg.cpp:4867); the port's corridor
  // may have been resampled more densely than the roadmap needs. The final
  // pose is the viewpoint the path was chosen for and is always kept.
  std::vector<RefPose> kept;
  kept.push_back(poses.front());
  for (std::size_t i = 1; i < poses.size(); ++i) {
    const double gap =
        (poses[i].state.head<3>() - kept.back().state.head<3>()).norm();
    const bool last = i + 1 == poses.size();
    if (gap >= vertex_spacing || (last && gap > 1e-9)) kept.push_back(poses[i]);
  }

  // The whole path is collision free already and starts from the root
  // vertex. Only the first vertex has to be linked to the existing graph
  // (rrg.cpp:4812).
  Vertex* parent_vertex =
      linkStateToGraph(graph, kept.front().state, ctx, kRadiusLimit);
  if (parent_vertex == nullptr) return false;

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
                            double dist_ignore_collision_check) {
  Vertex* linked =
      linkStateToGraph(graph, state, ctx, dist_ignore_collision_check);
  if (linked == nullptr) return nullptr;
  // rrg.cpp:5484 and 5497: edges from the linked vertex to whatever else is
  // reachable around it, so the route out of here is not just the chain in.
  ExpandGraphReport rep;
  expandGraphEdges(graph, linked, rep, ctx);
  return linked;
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

  // 2) Re-update all previous frontiers in the graph: still a frontier, or
  // now surrounded by known space (rrg.cpp:2411 to 2426).
  for (auto& entry : global_graph.vertices_map_) {
    Vertex* vertex = entry.second;
    if (vertex == nullptr || vertex->type != VertexType::kFrontier) continue;
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
    const std::vector<Eigen::Vector3d>& excluded, double exclusion_radius) {
  GlobalFrontierReport report;

  // Re-check all frontiers against the current map (rrg.cpp:5612 to 5625).
  std::vector<Vertex*> global_frontiers;
  for (auto& entry : graph.vertices_map_) {
    Vertex* vertex = entry.second;
    if (vertex == nullptr || vertex->type != VertexType::kFrontier) continue;
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
    if (exp_gain > report.best_gain) {
      report.best_gain = exp_gain;
      report.best_frontier = frontier;
      report.best_distance = distance->second;
    }
  }
  return report;
}

}  // namespace mgg
