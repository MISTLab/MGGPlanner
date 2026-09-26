#include "mgg_core/frontier_clusters.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace mgg {
namespace {

std::uint64_t cellBits(double coordinate, double cell_m) {
  double cell = std::floor(coordinate / cell_m);
  if (!std::isfinite(cell)) cell = 0.0;
  cell = std::clamp(cell, -32768.0, 32767.0);
  return static_cast<std::uint64_t>(static_cast<std::int64_t>(cell) + 32768) &
         0xFFFFu;
}

}  // namespace

ClusterId makeClusterId(int robot_id, const Eigen::Vector3d& position,
                        double cell_m) {
  const std::uint64_t robot =
      (static_cast<std::uint64_t>(robot_id) & 0x7FFFu) + 1u;
  return (robot << 48) | (cellBits(position.x(), cell_m) << 32) |
         (cellBits(position.y(), cell_m) << 16) |
         cellBits(position.z(), cell_m);
}

std::vector<FrontierCluster> extractFrontierClusters(GraphManager& graph,
                                                     double merge_radius_m,
                                                     double min_cluster_gain,
                                                     double cluster_id_cell_m,
    const std::function<bool(const Vertex&)>& eligible) {
  std::vector<Vertex*> frontiers;
  for (auto& entry : graph.vertices_map_) {
    Vertex* vertex = entry.second;
    if (vertex == nullptr || vertex->type != VertexType::kFrontier ||
        !graph.inService(*vertex) || (eligible && !eligible(*vertex))) {
      continue;
    }
    frontiers.push_back(vertex);
  }
  // Best gain first; ties by id, so the grouping does not follow hash order.
  std::sort(frontiers.begin(), frontiers.end(),
            [](const Vertex* a, const Vertex* b) {
              if (a->vol_gain.gain != b->vol_gain.gain) {
                return a->vol_gain.gain > b->vol_gain.gain;
              }
              return a->id < b->id;
            });

  const double radius_sq = merge_radius_m * merge_radius_m;
  std::vector<bool> grouped(frontiers.size(), false);
  std::vector<FrontierCluster> clusters;
  for (std::size_t i = 0; i < frontiers.size(); ++i) {
    if (grouped[i]) continue;
    const Vertex* representative = frontiers[i];
    FrontierCluster cluster;
    cluster.owner_robot_id = representative->robot_id;
    cluster.representative_vertex_id = representative->id;
    cluster.position = representative->state.head<3>();
    cluster.gain = representative->vol_gain.gain;
    for (std::size_t j = i; j < frontiers.size(); ++j) {
      if (grouped[j] ||
          (frontiers[j]->state.head<3>() - cluster.position).squaredNorm() >
              radius_sq) {
        continue;
      }
      grouped[j] = true;
      frontiers[j]->cluster_id = representative->id;
      cluster.member_vertex_ids.push_back(frontiers[j]->id);
    }
    if (!(cluster.gain >= min_cluster_gain)) continue;  // also drops NaN
    cluster.id = makeClusterId(cluster.owner_robot_id, cluster.position,
                               cluster_id_cell_m);
    clusters.push_back(std::move(cluster));
  }
  std::unordered_set<ClusterId> used;
  for (FrontierCluster& cluster : clusters) {
    while (!used.insert(cluster.id).second) ++cluster.id;
  }
  return clusters;
}

void ClusterIdRegistry::stabilize(std::vector<FrontierCluster>& clusters,
                                  double match_radius_m) {
  std::vector<bool> reused(known_.size(), false);
  std::vector<bool> renamed(clusters.size(), false);
  std::unordered_set<ClusterId> used;
  // Old names first, so a newcomer cannot take the name of a cluster that
  // stayed.
  for (std::size_t c = 0; c < clusters.size(); ++c) {
    int best = -1;
    double best_distance = match_radius_m;
    for (std::size_t k = 0; k < known_.size(); ++k) {
      if (reused[k]) continue;
      const double distance =
          (known_[k].position - clusters[c].position).norm();
      if (distance <= best_distance) {
        best_distance = distance;
        best = static_cast<int>(k);
      }
    }
    if (best < 0) continue;
    reused[best] = true;
    renamed[c] = true;
    clusters[c].id = known_[best].id;
    used.insert(clusters[c].id);
  }
  for (std::size_t c = 0; c < clusters.size(); ++c) {
    if (renamed[c]) continue;
    while (!used.insert(clusters[c].id).second) ++clusters[c].id;
  }
  known_.clear();
  for (const FrontierCluster& cluster : clusters) {
    known_.push_back({cluster.id, cluster.position});
  }
}

bool exploredInGraph(GraphManager& graph, const Eigen::Vector3d& position,
                     double radius_m) {
  const StateVec state(position.x(), position.y(), position.z(), 0.0);
  std::vector<Vertex*> nearby;
  if (!graph.getNearestVertices(&state, radius_m, &nearby)) return false;
  bool covered = false;
  for (const Vertex* vertex : nearby) {
    if (vertex == nullptr || !graph.inService(*vertex)) continue;
    if (vertex->type == VertexType::kFrontier) return false;
    covered = true;
  }
  return covered;
}

}  // namespace mgg
