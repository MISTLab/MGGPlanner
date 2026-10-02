// Volumetric gain: how much unmapped space a viewpoint would reveal.
//
// This is what makes MGG an exploration planner rather than a graph builder.
// Ported from Rrg::computeVolumetricGainRayModel and
// Rrg::computeExplorationGain.

#ifndef MGG_CORE_GAIN_H_
#define MGG_CORE_GAIN_H_

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/graph_base.h"
#include "mgg_core/graph_manager.h"
#include "mgg_core/map_interface.h"
#include "mgg_core/params.h"
#include "mgg_core/sensor_params.h"

namespace mgg {

/// What gain evaluation needs from its surroundings.
struct GainContext {
  MapInterface* map = nullptr;
  const PlanningParams* planning = nullptr;
  const RobotParams* robot = nullptr;
  /// Voxels outside this volume are ignored, so a viewpoint is not rewarded
  /// for seeing space the robot is not allowed to explore.
  const BoundedSpaceParams* global_space = nullptr;
  /// Regions that contribute no gain even when unknown. Optional.
  const std::vector<BoundedSpaceParams>* no_gain_zones = nullptr;
  /// When set, only voxels inside this volume count: the operator's
  /// "Explore here" region (drone scout design §3.5). Optional.
  const BoundedSpaceParams* gain_region = nullptr;
  /// Sensors named by planning->exp_sensor_list.
  const std::unordered_map<std::string, SensorParams>* sensors = nullptr;
};


/// Gain of a single viewpoint.
///
/// Casts the sensor's rays, tallies what they pass through, discards anything
/// outside the global bounds or inside a no-gain zone, and scores the rest
/// with the per-type weights. Sets is_frontier when enough unknown volume is
/// visible. A ground robot's rays start at its sensor, mount_height over
/// the ground under the vertex when that is set
/// (SensorParams::getMountedFrustumEndpoints). Ground gain/frontiers count
/// only up to robot.size.z() + ground_frontier_height_margin above its floor;
/// non-frontiers score zero. Below the floor, mapped support suppresses gain;
/// the lower limit remains max(2 max_ground_height, 1 m) under the vertex.
/// num_total_unknown_voxels is -1 (unavailable) with production band pruning.
/// ground_gain_full_scan enables the unpruned diagnostic/reference scan of
/// the configured gain model, recording unknown before the upper cutoff.
/// Ground-only angular/range overrides coarsen/shorten a private sensor copy;
/// zero preserves the real sensor. Sparse counts/ranking are approximate;
/// pruning remains exact for each configured model. Aerial rays are unchanged.
///
/// `voxel_log`, when given, receives every counted voxel for visualisation.
void computeVolumetricGain(
    const StateVec& state, VolumetricGain& gain, const GainContext& ctx,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>* voxel_log = nullptr);

/// Gain for every vertex of `graph`.
///
/// With `clustering`, a vertex's gain is shared with its neighbours within
/// clustering_radius instead of being recomputed, which is the expensive part
/// of a planning cycle. Ground robots never share gains: a door frontier
/// must not be copied into an explored room. `only_leaf_vertices` skips
/// interior vertices.
///
/// Returns how many viewpoints were actually evaluated.
int computeExplorationGain(GraphManager& graph, const GainContext& ctx,
                           bool only_leaf_vertices, bool clustering);

}  // namespace mgg

#endif  // MGG_CORE_GAIN_H_
