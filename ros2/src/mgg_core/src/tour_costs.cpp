#include "mgg_core/tour_costs.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace mgg {

const ShortestPathsReport* GraphDistanceCache::from(GraphManager& graph,
                                                    std::uint64_t revision,
                                                    int source_id,
                                                    std::uint64_t peer_generation) {
  if (!valid_ || &graph != graph_ || revision != revision_ ||
      peer_generation != peer_generation_) {
    reports_.clear();
    failed_.clear();
    graph_ = &graph;
    revision_ = revision;
    peer_generation_ = peer_generation;
    valid_ = true;
  }
  const auto found = reports_.find(source_id);
  if (found != reports_.end()) return &found->second;
  if (failed_.count(source_id) > 0) return nullptr;
  // Graph::findDijkstraShortestPaths prints to stdout for an unknown
  // source; ask the vertex map first.
  if (graph.vertices_map_.find(source_id) == graph.vertices_map_.end()) {
    failed_.insert(source_id);
    return nullptr;
  }
  ShortestPathsReport report;
  ++solves_;
  if (!graph.findShortestPaths(source_id, report) || !report.status) {
    failed_.insert(source_id);
    return nullptr;
  }
  return &reports_.emplace(source_id, std::move(report)).first->second;
}

double reachedDistance(const ShortestPathsReport& report, int target_id) {
  if (!report.status) return kUnreachableCost;
  if (target_id == report.source_id) return 0.0;
  const auto parent = report.parent_id_map.find(target_id);
  const auto distance = report.distance_map.find(target_id);
  // Dijkstra leaves an unreached vertex as its own parent at the largest
  // double (global_graph.cpp searchGlobalFrontier reads it the same way).
  if (parent == report.parent_id_map.end() ||
      distance == report.distance_map.end() || parent->second == target_id ||
      !std::isfinite(distance->second) ||
      distance->second >= std::numeric_limits<double>::max() / 2.0) {
    return kUnreachableCost;
  }
  return distance->second;
}

double firstLegHeadingChange(GraphManager& graph,
                             const ShortestPathsReport& from_robot,
                             int target_id, double robot_yaw) {
  if (target_id == from_robot.source_id ||
      !std::isfinite(reachedDistance(from_robot, target_id))) {
    return 0.0;
  }
  std::vector<int> path;
  graph.getShortestPath(target_id, from_robot,
                        /*source_to_target_order=*/true, path);
  if (path.size() < 2) return 0.0;
  const auto source = graph.vertices_map_.find(path.front());
  if (source == graph.vertices_map_.end() || source->second == nullptr) {
    return 0.0;
  }
  const Eigen::Vector2d origin = source->second->state.head<2>();
  Eigen::Vector2d toward = Eigen::Vector2d::Zero();
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto vertex = graph.vertices_map_.find(path[i]);
    if (vertex == graph.vertices_map_.end() || vertex->second == nullptr) {
      continue;
    }
    toward = vertex->second->state.head<2>() - origin;
    if (toward.norm() >= kFirstLegLookaheadM) break;
  }
  if (toward.norm() < 1e-6) return 0.0;
  const double bearing = std::atan2(toward.y(), toward.x());
  return std::abs(std::remainder(bearing - robot_yaw, 2.0 * M_PI));
}

TourCostMatrix computeTourCosts(GraphManager& graph, std::uint64_t revision,
                                GraphDistanceCache& cache,
                                int source_vertex_id, double robot_yaw,
                                const std::vector<FrontierCluster>& clusters,
                                double heading_weight,
                                std::uint64_t peer_generation) {
  const std::size_t n = clusters.size();
  TourCostMatrix costs;
  costs.from_robot.assign(n, kUnreachableCost);
  costs.between.assign(n, std::vector<double>(n, kUnreachableCost));
  for (std::size_t i = 0; i < n; ++i) costs.between[i][i] = 0.0;

  const ShortestPathsReport* robot =
      cache.from(graph, revision, source_vertex_id, peer_generation);
  if (robot != nullptr) {
    for (std::size_t i = 0; i < n; ++i) {
      const int target = clusters[i].representative_vertex_id;
      const double distance = reachedDistance(*robot, target);
      if (!std::isfinite(distance)) continue;
      costs.from_robot[i] =
          distance + heading_weight * firstLegHeadingChange(graph, *robot,
                                                            target, robot_yaw);
    }
  }
  // A peer body's margin may be left but not entered, so reachability can
  // differ by direction; the solver's matrix is symmetric. A leg costs its
  // worse direction, unreachable when either is (review r0, M1).
  std::vector<const ShortestPathsReport*> from_cluster(n, nullptr);
  for (std::size_t i = 0; i < n; ++i) {
    from_cluster[i] = cache.from(
        graph, revision, clusters[i].representative_vertex_id, peer_generation);
  }
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      if (from_cluster[i] == nullptr || from_cluster[j] == nullptr) continue;
      const double there = reachedDistance(
          *from_cluster[i], clusters[j].representative_vertex_id);
      const double back = reachedDistance(
          *from_cluster[j], clusters[i].representative_vertex_id);
      const double distance = std::isfinite(there) && std::isfinite(back)
                                  ? std::max(there, back)
                                  : kUnreachableCost;
      costs.between[i][j] = distance;
      costs.between[j][i] = distance;
    }
  }
  return costs;
}

}  // namespace mgg
