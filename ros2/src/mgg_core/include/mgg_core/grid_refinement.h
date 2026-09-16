// Bounded transport-free grid refinement for explicit topological routes.

#ifndef MGG_CORE_GRID_REFINEMENT_H_
#define MGG_CORE_GRID_REFINEMENT_H_

#include <chrono>
#include <cstddef>
#include <functional>

#include "mgg_core/planning_stages.h"

namespace mgg {

struct GridRefinementLimits {
  double resolution_m = 0.25;
  double detour_margin_m = 1.0;
  double start_connector_max_distance_m = 0.0;
  std::size_t max_cells = 4096;
  std::size_t max_expansions = 2048;
  std::chrono::milliseconds timeout{50};
};

/// Result of projecting an explicit-route state and checking the full body.
/// Keeping the rejection class in the callback result lets callers report the
/// failed check without repeating any map query.
enum class GridProjectionStatus {
  kSupported,
  kNoGround,
  kBodyOccupied,
  kBodyUnknown,
  kGeofenceViolation,
};

/// Projects a pose onto supported terrain and verifies that the robot's full
/// footprint fits there. The input z is the nearest parent's driving height;
/// implementations must fail rather than select an unrelated stacked surface.
using GridProjectState = std::function<GridProjectionStatus(StateVec&)>;

/// Verifies a swept segment, including occupancy, footprint, terrain step,
/// incline, and geofence policy. Unknown space must return false. On success,
/// `terrain_path` contains the checked source-to-target geometry, including
/// endpoints; returning only a chord after validating a terrain polyline is
/// forbidden.
using GridTraverseSegment =
    std::function<bool(const StateVec&, const StateVec&,
                       std::vector<StateVec>& terrain_path)>;

using GridCancelled = std::function<bool()>;

/// Refines blocked segments with bounded deterministic 8-neighbour A*.
///
/// The search is deliberately local and two-dimensional. Each XY cell is
/// projected once from its first deterministic adjacent-parent observation
/// and never silently rebound to another stacked surface. This may reject a
/// feasible multi-level route, but cannot jump floors based on expansion
/// order. Diagonal moves require both cardinal side transitions, preventing
/// corner cutting.
class BoundedGridPlanner final : public GridPlanner {
 public:
  BoundedGridPlanner(StateVec current, GridRefinementLimits limits,
                     GridProjectState project,
                     GridTraverseSegment traversable,
                     GridCancelled cancelled = {});

  FeasiblePath refine(const RouteCorridor& corridor) override;

 private:
  StateVec current_;
  GridRefinementLimits limits_;
  GridProjectState project_;
  GridTraverseSegment traversable_;
  GridCancelled cancelled_;
};

}  // namespace mgg

#endif  // MGG_CORE_GRID_REFINEMENT_H_
