// Builds the 2.5D traversability raster from a native MOLA planner grid.
//
// Rule, per raster cell (see buildTraversabilityRaster):
//   ground    the lowest surface max_z among the voxels the cell covers. A
//             column may hold the road and a table top above it; the road
//             is the ground and the table top an obstacle in the body band.
//   obstacle  any occupied voxel whose top lies above ground + step band and
//             whose bottom lies below ground + body band: matter the robot
//             can neither roll over nor pass beneath.
//   inflated  a free cell within the body radius of an obstacle cell; the
//             body may not fit there at this resolution.
//   unknown   no voxel the cell covers has a surface; or, with a clearance
//             requirement, the free voxels above the ground do not reach it.
//   free      everything else.
// Kerbs and steps are not obstacles: they are ground height differences
// between adjacent cells, which the planner judges against the platform's
// step and drop limits. A bridge deck above a road is an obstacle for the
// road cell and the deck is not traversable: the raster is 2.5D.

#ifndef MGG_MAP_OCTOMAP_TRAVERSABILITY_BUILDER_H_
#define MGG_MAP_OCTOMAP_TRAVERSABILITY_BUILDER_H_

#include <memory>

#include <Eigen/Dense>

#include "mgg_core/traversability_raster.h"
#include "mgg_map_octomap/native_mola_grid.h"

namespace mgg {

struct RasterParams {
  /// Raster cell size, metres.
  double cell_size_m = 0.5;
  /// Lateral inflation of obstacle cells: a free cell is inflated when any
  /// point of an obstacle cell lies within this distance of its centre.
  double body_radius_m = 0.0;
  /// Step band: matter up to this height above the ground is terrain the
  /// platform rolls over (the platform's max_step_height).
  double max_step_height_m = 0.0;
  /// Body band: matter between the step band and this height above the
  /// ground is an obstacle; above it the robot passes beneath.
  double body_height_m = 0.0;
  /// Zero disables. Otherwise a cell is free only when its free voxels cover
  /// the column from the ground up to this height: unobserved air above
  /// observed ground is then unknown rather than free.
  double min_clearance_m = 0.0;
  /// Measurement tolerance on the step band, metres.
  double step_tolerance_m = 0.01;

  bool operator==(const RasterParams& other) const {
    return cell_size_m == other.cell_size_m &&
           body_radius_m == other.body_radius_m &&
           max_step_height_m == other.max_step_height_m &&
           body_height_m == other.body_height_m &&
           min_clearance_m == other.min_clearance_m &&
           step_tolerance_m == other.step_tolerance_m;
  }
  bool operator!=(const RasterParams& other) const {
    return !(*this == other);
  }
};

/// Builds the raster in the frame `raster_from_grid` maps into: every voxel
/// centre and surface height is transformed before it is binned, so a
/// navigation-frame raster comes from the inverse of the authority's
/// component_from_navigation. Returns nullptr when the grid holds no
/// occupied voxel, the parameters are invalid, the transform tilts more than
/// the authority tolerance, or the raster would exceed the cell bound.
std::shared_ptr<const TraversabilityRaster> buildTraversabilityRaster(
    const NativeMolaGrid& grid, const RasterParams& params,
    const Eigen::Isometry3d& raster_from_grid = Eigen::Isometry3d::Identity());

}  // namespace mgg

#endif  // MGG_MAP_OCTOMAP_TRAVERSABILITY_BUILDER_H_
