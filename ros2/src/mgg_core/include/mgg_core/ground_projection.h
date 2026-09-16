// Dropping samples and edges onto the ground, for wheeled and legged robots.
//
// Ported from Rrg::projectSample and Rrg::getProjectedEdgeStatus. Both depend
// only on the map and the planning parameters, so they move across cleanly.
//
// Two defects in the originals are fixed here:
//
//   * The enum value was spelled kOccipied.
//   * getProjectedEdgeStatus dropped a trailing point with
//     "projected_edge.erase(projected_edge.end())". Erasing end() is
//     undefined behaviour: std::vector::erase requires a dereferenceable
//     iterator, which end() is not. libstdc++ happens to treat it as
//     pop_back(), so it worked, but a build with _GLIBCXX_DEBUG asserts on
//     it. It is pop_back() here.

#ifndef MGG_CORE_GROUND_PROJECTION_H_
#define MGG_CORE_GROUND_PROJECTION_H_

#include <vector>

#include <Eigen/Dense>

#include "mgg_core/map_interface.h"
#include "mgg_core/params.h"
#include "mgg_core/types.h"

namespace mgg {

enum class ProjectedEdgeStatus {
  kAdmissible = 0,
  kSteep,      ///< exceeds max_inclination
  kOccupied,   ///< hits an obstacle
  kUnknown,    ///< passes through unmapped space
  kHanging,    ///< no ground beneath it
};

class GroundProjection {
 public:
  GroundProjection(const MapInterface& map, const PlanningParams& params)
      : map_(map), params_(params) {}

  /// How far below `sample` the ground lies.
  ///
  /// Casts downward from `sample` and from four offsets around it. Returns the
  /// distance to the ground and sets `status` to kOccupied when ground was
  /// found, kUnknown when every ray ran into unmapped space, or kFree when
  /// nothing was hit within max_projection_length (returning -1).
  ///
  /// `sample` is updated to the x,y of whichever offset found ground, matching
  /// the original.
  double projectSample(Eigen::Vector3d& sample, VoxelStatus& status) const;

  /// Whether a robot could drive from `start` to `end` once both are dropped
  /// onto the ground, filling `projected_edge_out` with the ground-following
  /// polyline when it can.
  ///
  /// PRECONDITION, inherited from the ROS 1 code and easy to miss: `start` and
  /// `end` must ALREADY sit at driving height, i.e. max_ground_height above
  /// the ground. Intermediate samples are height-corrected as they are
  /// generated, but the endpoint is appended as given, without correction. Feed
  /// it raw endpoints and the final segment jumps by whatever the height
  /// mismatch is, and the edge is rejected as kSteep. Rrg::expandGraph
  /// satisfies this by projecting the new state before calling in.
  ProjectedEdgeStatus getProjectedEdgeStatus(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end,
      const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel,
      std::vector<Eigen::Vector3d>& projected_edge_out, bool is_hanging,
      bool preserve_start_height = false) const;

  /// How far down projectSample looks. Was a bare 5.0 in the original.
  double max_projection_length = 5.0;

 private:
  const MapInterface& map_;
  const PlanningParams& params_;
};

}  // namespace mgg

#endif  // MGG_CORE_GROUND_PROJECTION_H_
