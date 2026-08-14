#include "mgg_core/gain.h"

#include <algorithm>
#include <list>

#include "mgg_core/log.h"

namespace mgg {
namespace {

bool inNoGainZone(const GainContext& ctx, const Eigen::Vector3d& voxel) {
  if (ctx.no_gain_zones == nullptr) return false;
  for (const BoundedSpaceParams& zone : *ctx.no_gain_zones) {
    if (zone.isInsideSpace(voxel)) return true;
  }
  return false;
}

}  // namespace

void computeVolumetricGain(
    const StateVec& state, VolumetricGain& gain, const GainContext& ctx,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>* voxel_log) {
  gain.reset();
  if (ctx.map == nullptr || ctx.planning == nullptr ||
      ctx.sensors == nullptr || ctx.global_space == nullptr) {
    return;
  }

  const Eigen::Vector3d origin(state[0], state[1], state[2]);

  for (const std::string& sensor_name : ctx.planning->exp_sensor_list) {
    auto it = ctx.sensors->find(sensor_name);
    if (it == ctx.sensors->end()) {
      logWarn("gain: no sensor named '" + sensor_name + "'");
      continue;
    }
    const SensorParams& sensor = it->second;

    std::vector<Eigen::Vector3d> endpoints;
    sensor.getFrustumEndpoints(state, endpoints);
    if (endpoints.empty()) {
      logWarn("gain: sensor '" + sensor_name +
              "' has no rays; was update() called?");
      continue;
    }

    GainCounts raw;
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> visited;
    ctx.map->getScanStatus(origin, endpoints, raw, visited, sensor.model());

    int unknown = 0, free = 0, occupied = 0;
    for (const auto& entry : visited) {
      const Eigen::Vector3d& voxel = entry.first;
      // Only count what lies inside the region the robot may explore, and
      // outside any zone declared uninteresting.
      if (!ctx.global_space->isInsideSpace(voxel)) continue;
      if (inNoGainZone(ctx, voxel)) continue;
      switch (entry.second) {
        case VoxelStatus::kUnknown: ++unknown; break;
        case VoxelStatus::kFree: ++free; break;
        case VoxelStatus::kOccupied: ++occupied; break;
      }
      if (voxel_log != nullptr) voxel_log->emplace_back(voxel, entry.second);
    }

    gain.num_unknown_voxels += unknown;
    gain.num_free_voxels += free;
    gain.num_occupied_voxels += occupied;
    gain.gain += unknown * ctx.planning->unknown_voxel_gain +
                 free * ctx.planning->free_voxel_gain +
                 occupied * ctx.planning->occupied_voxel_gain;

    // Scaled to metres, as the ROS 1 code did, so the threshold is
    // resolution independent.
    if (sensor.isFrontier(unknown * ctx.map->getResolution())) {
      gain.is_frontier = true;
    }
  }
}

int computeExplorationGain(GraphManager& graph, const GainContext& ctx,
                           bool only_leaf_vertices, bool clustering) {
  // Leaves first: they sit at the edge of the graph and are the likeliest
  // frontiers, so with clustering on they seed the clusters.
  std::list<int> pending;
  for (const auto& entry : graph.vertices_map_) {
    if (entry.second == nullptr) continue;
    if (entry.second->is_leaf_vertex) {
      pending.push_front(entry.first);
    } else {
      pending.push_back(entry.first);
    }
  }
  // The ROS 1 loop indexed vertices_map_ by position, "for (i = 0; i <
  // getNumVertices(); ++i) vertex_map[i]->is_leaf_vertex". operator[] inserts
  // a null for a missing key and that arrow then dereferences it, so any gap
  // in the ids, or a Boost vertex count above the map size, was a segfault.
  // Iterating the map avoids both.

  int evaluated = 0;
  while (!pending.empty()) {
    const int v_id = pending.front();
    pending.pop_front();

    auto it = graph.vertices_map_.find(v_id);
    if (it == graph.vertices_map_.end() || it->second == nullptr) continue;
    Vertex* v = it->second;

    if (only_leaf_vertices && !v->is_leaf_vertex) continue;

    computeVolumetricGain(v->state, v->vol_gain, ctx);
    ++evaluated;

    if (clustering) {
      // Neighbours inherit this gain rather than recomputing it. Raycasting
      // every vertex is the expensive part of a planning cycle.
      std::vector<Vertex*> nearby;
      if (graph.getNearestVertices(&v->state, ctx.planning->clustering_radius,
                                   &nearby)) {
        for (Vertex* n : nearby) {
          if (n == nullptr || n == v) continue;
          auto pos = std::find(pending.begin(), pending.end(), n->id);
          if (pos != pending.end()) {
            n->vol_gain = v->vol_gain;
            if (n->vol_gain.is_frontier) n->type = VertexType::kFrontier;
            pending.erase(pos);
          }
        }
      }
    }

    if (v->vol_gain.is_frontier) v->type = VertexType::kFrontier;
  }
  return evaluated;
}

}  // namespace mgg
