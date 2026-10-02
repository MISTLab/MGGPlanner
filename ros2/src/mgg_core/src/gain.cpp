#include "mgg_core/planning_cancellation.h"
#include "mgg_core/gain.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <list>
#include <optional>
#include <unordered_map>
#include <utility>

#include "mgg_core/log.h"

namespace mgg {
namespace {

bool inNoGainZone(const GainContext& ctx, const Eigen::Vector3d& voxel) {
  if (ctx.no_gain_zones == nullptr) return false;
  for (const BoundedSpaceParams& zone : *ctx.no_gain_zones) {
    planningCheckpoint();
    if (zone.isInsideSpace(voxel)) return true;
  }
  return false;
}

bool outsideGainRegion(const GainContext& ctx, const Eigen::Vector3d& voxel) {
  return ctx.gain_region != nullptr && !ctx.gain_region->isInsideSpace(voxel);
}

// Bound the existing inclusion region conservatively in navigation axes.
// This is only traversal pruning; exact rotated/spherical inclusion remains
// in the scoring loop below, as do no-gain exclusions.
void intersectGainBounds(const BoundedSpaceParams& space, ScanBounds& bounds) {
  Eigen::Vector3d low, high;
  if (space.type == BoundedSpaceType::kSphere) {
    const double radius = space.radiusTotal() > 0 ? space.radiusTotal() : space.radius;
    low = space.getCenter() - Eigen::Vector3d::Constant(radius);
    high = space.getCenter() + Eigen::Vector3d::Constant(radius);
  } else {
    const bool totals = space.minValTotal() != space.maxValTotal();
    const Eigen::Vector3d a = totals ? space.minValTotal() : space.min_val;
    const Eigen::Vector3d b = totals ? space.maxValTotal() : space.max_val;
    const Eigen::Matrix3d rotation = space.getRotationMatrix().transpose();
    const Eigen::Vector3d center = space.getCenter() + rotation * ((a + b) * 0.5);
    const Eigen::Vector3d half = rotation.cwiseAbs() * ((b - a) * 0.5);
    low = center - half;
    high = center + half;
  }
  bounds.low = bounds.low.cwiseMax(low);
  bounds.high = bounds.high.cwiseMin(high);
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
    Column& column = at(voxel.x(), voxel.y());
    if (below(voxel, column.support_z)) return true;
    if (!column.enclosed_known) {
      // The lowest of the four neighbours' supports; -inf once one has none.
      double lowest = std::numeric_limits<double>::infinity();
      for (const auto& [dx, dy] : {std::pair<double, double>{resolution_, 0.0},
                                   {-resolution_, 0.0},
                                   {0.0, resolution_},
                                   {0.0, -resolution_}}) {
        lowest = std::min(lowest,
                          at(voxel.x() + dx, voxel.y() + dy).support_z);
        if (std::isinf(lowest)) break;
      }
      // Elements of an unordered_map keep their address when it rehashes.
      column.enclosed_z = lowest;
      column.enclosed_known = true;
    }
    return below(voxel, column.enclosed_z);
  }

 private:
  struct Column {
    double support_z = -std::numeric_limits<double>::infinity();
    double enclosed_z = -std::numeric_limits<double>::infinity();
    bool enclosed_known = false;
  };

  /// Below the support voxel centred at `support_z`, not in it.
  bool below(const Eigen::Vector3d& voxel, double support_z) const {
    return voxel.z() < support_z - 0.5 * resolution_;
  }

  Column& at(double x, double y) {
    // Voxel centres repeat at multiples of the resolution; a millimetre key
    // tells columns apart in any frame.
    const std::uint64_t key =
        (std::uint64_t(std::llround(x * 1000.0)) << 32) ^
        (std::uint64_t(std::llround(y * 1000.0)) & 0xffffffffULL);
    const auto found = columns_.find(key);
    if (found != columns_.end()) return found->second;
    Column column;
    Eigen::Vector3d end;
    if (map_.getRayStatus(Eigen::Vector3d(x, y, origin_.z()),
                          Eigen::Vector3d(x, y, origin_.z() - depth_), false,
                          end) == VoxelStatus::kOccupied) {
      column.support_z = end.z();
    }
    return columns_.emplace(key, column).first->second;
  }

  const MapInterface& map_;
  const Eigen::Vector3d origin_;
  const double depth_;
  const double resolution_;
  std::unordered_map<std::uint64_t, Column> columns_;
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

  // The vertex, which for a ground robot rides max_ground_height over its
  // ground. The gain band is measured from it.
  const Eigen::Vector3d origin(state[0], state[1], state[2]);
  const bool ground_robot =
      ctx.robot != nullptr && ctx.robot->type == RobotType::kGroundRobot;

  for (const std::string& sensor_name : ctx.planning->exp_sensor_list) {
    planningCheckpoint();
    auto it = ctx.sensors->find(sensor_name);
    if (it == ctx.sensors->end()) {
      logWarn("gain: no sensor named '" + sensor_name + "'");
      continue;
    }
    // A private model for ranking only: never mutate the sensor used by
    // observed-body/FOV policy. Zero overrides retain its original ray table.
    std::optional<SensorParams> gain_sensor;
    const double step_deg = ctx.planning->ground_gain_angular_resolution_deg;
    const double range = ctx.planning->ground_gain_max_range;
    if (ground_robot && ((std::isfinite(step_deg) && step_deg > 0) ||
                         (std::isfinite(range) && range > 0))) {
      gain_sensor = it->second;
      if (std::isfinite(step_deg) && step_deg > 0) {
        const double step = std::min(step_deg, 180.0) * M_PI / 180.0;
        for (int axis = 0; axis < 2; ++axis) {
          const double real_step = gain_sensor->resolution[axis] > 0
              ? gain_sensor->resolution[axis] : M_PI / 180.0;
          gain_sensor->resolution[axis] = std::max(real_step, step);
        }
      }
      if (std::isfinite(range) && range > 0)
        gain_sensor->max_range = std::min(gain_sensor->max_range, range);
      gain_sensor->update();
    }
    const SensorParams& sensor = gain_sensor ? *gain_sensor : it->second;

    // A ground robot's sensor sees from where it is mounted: rays from the
    // vertex started 0.2 m low for a Bunker's lidar (diag-sensor, run 7).
    Eigen::Vector3d ray_origin = origin;
    std::vector<Eigen::Vector3d> endpoints;
    if (ground_robot && sensor.mount_height > 0.0) {
      sensor.getMountedFrustumEndpoints(
          state, origin.z() - ctx.planning->max_ground_height, ray_origin,
          endpoints);
    } else {
      sensor.getFrustumEndpoints(state, endpoints);
    }
    if (endpoints.empty()) {
      logWarn("gain: sensor '" + sensor_name +
              "' has no rays; was update() called?");
      continue;
    }

    // Interest follows the robot's reachable level, not unknown ceiling air
    // in a tall room. Keep the downward band: ramps and open stairwells can
    // reveal useful space below the vertex's own floor.
    const double band_top = ground_robot
        ? origin.z() - ctx.planning->max_ground_height + ctx.robot->size.z() +
              std::max(0.0, ctx.planning->ground_frontier_height_margin)
        : std::numeric_limits<double>::infinity();
    const double max_h_below =
        std::max(ctx.planning->max_ground_height * 2.0, 1.0);
    GainCounts raw;
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> visited;
    if (ground_robot && !ctx.planning->ground_gain_full_scan) {
      ScanBounds bounds;
      bounds.low.z() = origin.z() - max_h_below;
      bounds.high.z() = band_top;
      intersectGainBounds(*ctx.global_space, bounds);
      if (ctx.gain_region) intersectGainBounds(*ctx.gain_region, bounds);
      ctx.map->getScanStatusInBounds(ray_origin, endpoints, raw, visited,
                                     sensor.model(), bounds);
      gain.num_total_unknown_voxels = -1;  // unavailable, not zero unknown
    } else {
      ctx.map->getScanStatusIterative(ray_origin, endpoints, raw, visited,
                                      sensor.model());
    }
    gain.gain_rays_cast += raw.rays_cast;
    gain.gain_voxel_visits += raw.voxel_visits;
    const double floor_depth =
        ctx.planning->max_ground_height + ctx.map->getResolution();
    ColumnSupport support(*ctx.map, origin,
                          max_h_below + ctx.map->getResolution());

    int unknown = 0, free = 0, occupied = 0;
    PlanningCheckpointThrottle checkpoint;
    for (const auto& entry : visited) {
      checkpoint.check();
      const Eigen::Vector3d& voxel = entry.first;
      // Only count what lies inside the region the robot may explore, and
      // outside any zone declared uninteresting.
      if (!ctx.global_space->isInsideSpace(voxel)) continue;
      if (inNoGainZone(ctx, voxel)) continue;
      if (outsideGainRegion(ctx, voxel)) continue;

      if (ground_robot) {
        if (origin.z() - voxel.z() > max_h_below) continue;
        // Deeper than a voxel under the vertex's own floor: counted unless
        // mapped ground lies over it. The band once ended one voxel under
        // that floor, and a ramp down or a stairwell lost its lower space.
        if (origin.z() - voxel.z() > floor_depth &&
            support.isUnderGround(voxel)) {
          continue;
        }
      }

      if (gain.num_total_unknown_voxels >= 0 && entry.second == VoxelStatus::kUnknown)
        ++gain.num_total_unknown_voxels;
      if (voxel.z() > band_top) continue;

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

    // A ground robot's vertex is a frontier with 0.5 m of unknown, in
    // voxels times their edge. Otherwise the distinct unknown voxels are
    // measured against the distinct voxels the sensor's rays reach in space
    // that is all unknown. The ray-length denominator of the ROS 1 code
    // suited a count of every voxel of every ray; against distinct voxels it
    // can deny a frontier even in space that is all unknown.
    const double resolution = ctx.map->getResolution();
    if (ground_robot ? unknown * resolution >= 0.5
                     : sensor.isFrontier(unknown, resolution)) {
      gain.is_frontier = true;
    }
  }
  // Known space (including nonzero free/occupied weights) must not compete
  // with a door frontier or keep the low-gain/global handoff from firing.
  if (ground_robot && !gain.is_frontier) gain.gain = 0.0;
}

int computeExplorationGain(GraphManager& graph, const GainContext& ctx,
                           bool only_leaf_vertices, bool clustering) {
  const bool ground_robot =
      ctx.robot != nullptr && ctx.robot->type == RobotType::kGroundRobot;
  if (ground_robot && clustering) {
    logInfo("ground gain: clustering skipped; frontier evidence is viewpoint-local");
    clustering = false;
  }
  // Leaves first: they sit at the edge of the graph and are the likeliest
  // frontiers, so with clustering on they seed the clusters.
  std::list<int> pending;
  for (const auto& entry : graph.vertices_map_) {
    planningCheckpoint();
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
    planningCheckpoint();
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
          planningCheckpoint();
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
    else if (ground_robot && v->type == VertexType::kFrontier)
      v->type = VertexType::kUnvisited;
  }
  return evaluated;
}

}  // namespace mgg
