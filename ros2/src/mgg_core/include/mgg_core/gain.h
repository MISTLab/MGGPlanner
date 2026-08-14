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
  /// Voxels outside this volume are ignored, so a viewpoint is not rewarded
  /// for seeing space the robot is not allowed to explore.
  const BoundedSpaceParams* global_space = nullptr;
  /// Regions that contribute no gain even when unknown. Optional.
  const std::vector<BoundedSpaceParams>* no_gain_zones = nullptr;
  /// Sensors named by planning->exp_sensor_list.
  const std::unordered_map<std::string, SensorParams>* sensors = nullptr;
};

/// Gain of a single viewpoint.
///
/// Casts the sensor's rays, tallies what they pass through, discards anything
/// outside the global bounds or inside a no-gain zone, and scores the rest
/// with the per-type weights. Sets is_frontier when enough unknown volume is
/// visible.
///
/// `voxel_log`, when given, receives every counted voxel for visualisation.
void computeVolumetricGain(
    const StateVec& state, VolumetricGain& gain, const GainContext& ctx,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>* voxel_log = nullptr);

/// Gain for every vertex of `graph`.
///
/// With `clustering`, a vertex's gain is shared with its neighbours within
/// clustering_radius instead of being recomputed, which is the expensive part
/// of a planning cycle. `only_leaf_vertices` skips interior vertices.
///
/// Returns how many viewpoints were actually evaluated.
int computeExplorationGain(GraphManager& graph, const GainContext& ctx,
                           bool only_leaf_vertices, bool clustering);

}  // namespace mgg

#endif  // MGG_CORE_GAIN_H_
