#include "mgg_core/gain.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <list>
#include <map>
#include <utility>

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

/// Where the ground under a vertex's surroundings has been mapped. The
/// support of a column is the first occupied voxel below the vertex's
/// height, as the planner's ground projection would find it, searched down
/// to `depth` below the vertex. Columns are cached per viewpoint.
class ColumnSupport {
 public:
  ColumnSupport(const MapInterface& map, const Eigen::Vector3d& origin,
                double depth)
      : map_(map), origin_(origin), depth_(depth),
        resolution_(map.getResolution()) {}

  /// Whether the voxel centred at `voxel` lies under mapped ground: under
  /// its own column's support, or, for a column whose floor has a one-voxel
  /// gap, under the support of all four edge neighbours. Where no ground is
  /// mapped over it, a stairwell or ground falling away, it does not.
  bool isUnderGround(const Eigen::Vector3d& voxel) {
    if (below(voxel, 0.0, 0.0)) return true;
    return below(voxel, resolution_, 0.0) && below(voxel, -resolution_, 0.0) &&
           below(voxel, 0.0, resolution_) && below(voxel, 0.0, -resolution_);
  }

 private:
  bool below(const Eigen::Vector3d& voxel, double dx, double dy) {
    const double x = voxel.x() + dx, y = voxel.y() + dy;
    // Voxel centres repeat at multiples of the resolution; a millimetre key
    // tells columns apart in any frame.
    const std::pair<long long, long long> key(std::llround(x * 1000.0),
                                              std::llround(y * 1000.0));
    auto it = support_z_.find(key);
    if (it == support_z_.end()) {
      Eigen::Vector3d end;
      const VoxelStatus status = map_.getRayStatus(
          Eigen::Vector3d(x, y, origin_.z()),
          Eigen::Vector3d(x, y, origin_.z() - depth_), false, end);
      const double z = status == VoxelStatus::kOccupied
                           ? end.z()
                           : -std::numeric_limits<double>::infinity();
      it = support_z_.emplace(key, z).first;
    }
    // Below the support voxel, not in it.
    return voxel.z() < it->second - 0.5 * resolution_;
  }

  const MapInterface& map_;
  const Eigen::Vector3d origin_;
  const double depth_;
  const double resolution_;
  std::map<std::pair<long long, long long>, double> support_z_;
};

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

    // Each voxel once per viewpoint: a voxel is revealed once however many
    // rays cross it. getScanStatus logs it once per ray, and near the
    // viewpoint hundreds of rays share a voxel (0.77 of the logged count was
    // unique on the captured Bistro grids, diag-viewpoint 2026-09-24).
    GainCounts raw;
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> visited;
    const bool ground_robot =
        ctx.robot != nullptr && ctx.robot->type == RobotType::kGroundRobot;
    // Ground robots only explore the traversable ground layer: voxels high
    // above the vertex, or deep below it, are irrelevant to them.
    const double max_h_above = std::max(ctx.planning->robot_height * 2.5, 1.2);
    if (ground_robot) {
      // A lidar leaves unknown gaps in walls, and rays through them counted
      // the space behind: 0.39 of the count at the captured plan ends. Wall
      // evidence is a return above the floor (a vertex rides
      // max_ground_height over it) and within the gain band; a gap is
      // inferred only between two such returns in a column.
      const double floor_z = origin.z() - ctx.planning->max_ground_height;
      const WallBand wall{floor_z + ctx.map->getResolution(),
                          origin.z() + max_h_above};
      ctx.map->getVisibleScanStatus(origin, endpoints, wall, raw, visited,
                                    sensor.model());
    } else {
      ctx.map->getScanStatusIterative(origin, endpoints, raw, visited,
                                      sensor.model());
    }

    // Below the vertex, the band reaches max(2 max_ground_height, 1 m). A
    // vertex rides max_ground_height above its ground, and nothing under
    // mapped ground can be seen: 0.10 of the count at the captured plan
    // ends lay under the floor (diag-viewpoint, 2026-09-24).
    const double max_h_below =
        std::max(ctx.planning->max_ground_height * 2.0, 1.0);
    const double floor_depth =
        ctx.planning->max_ground_height + ctx.map->getResolution();
    ColumnSupport support(*ctx.map, origin,
                          max_h_below + ctx.map->getResolution());

    int unknown = 0, free = 0, occupied = 0;
    for (const auto& entry : visited) {
      const Eigen::Vector3d& voxel = entry.first;
      // Only count what lies inside the region the robot may explore, and
      // outside any zone declared uninteresting.
      if (!ctx.global_space->isInsideSpace(voxel)) continue;
      if (inNoGainZone(ctx, voxel)) continue;

      if (ground_robot) {
        if (voxel.z() - origin.z() > max_h_above ||
            origin.z() - voxel.z() > max_h_below) {
          continue;
        }
        // Deeper than a voxel under the vertex's own floor: counted unless
        // mapped ground lies over it. The band once ended one voxel under
        // that floor, and a ramp down or a stairwell lost its lower space.
        if (origin.z() - voxel.z() > floor_depth &&
            support.isUnderGround(voxel)) {
          continue;
        }
      }

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
    const double unknown_scaled = unknown * ctx.map->getResolution();
    if (sensor.isFrontier(unknown_scaled) ||
        (ctx.robot != nullptr && ctx.robot->type == RobotType::kGroundRobot &&
         unknown_scaled >= 0.5)) {
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
