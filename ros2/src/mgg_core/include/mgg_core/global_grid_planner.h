// Full-map 2.5D A* over a TraversabilityRaster: the global stage for
// Navigate and ReturnHome.
//
// The topological graph only knows the lattice vertices and the breadcrumb
// backbone, so a goal off the graph used to be reached by a straight line that
// the bounded grid refinement could not bend around a building (benchbot,
// 2026-09-19). This planner routes over every observed cell of the map at the
// raster's cell size and hands the rolling section machinery a corridor with
// one pose per cell, which that machinery refines, validates and follows
// unchanged.

#ifndef MGG_CORE_GLOBAL_GRID_PLANNER_H_
#define MGG_CORE_GLOBAL_GRID_PLANNER_H_

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "mgg_core/traversability_raster.h"
#include "mgg_core/types.h"

namespace mgg {

struct GlobalGridPlannerLimits {
  /// Largest ground rise between adjacent cells, metres.
  double max_step_height = 0.0;
  /// Largest ground drop between adjacent cells, metres.
  double max_drop_height = 0.0;
  /// Measurement tolerance added to both limits, metres.
  double step_tolerance = 0.01;
  /// Height of an emitted pose above the cell ground, metres.
  double driving_offset = 0.0;
  /// Radius around an unknown start or goal cell in which known free cells
  /// may lend their ground height. The immediate ring of cells always counts.
  double body_radius = 0.0;
  /// Cost added to an edge the blocked-corridor view marks, metres.
  double blocked_penalty = 20.0;
  /// The search ends with kBudgetExceeded once this many cells have been
  /// expanded.
  std::size_t max_expansions = 200000;
  /// The search ends with kDeadlineExceeded once this much time has passed.
  std::chrono::milliseconds timeout{500};
};

/// A vertical cylinder of unbounded height that stands in the world and in no
/// map: another robot of the fleet. Cells whose centre lies within `radius`
/// of `centre` are refused; callers inflate the radius by the body radius.
struct TransientDisc {
  Eigen::Vector2d centre = Eigen::Vector2d::Zero();
  double radius = 0.0;
};

enum class GlobalGridPlanStatus : std::uint8_t {
  kSucceeded = 0,
  kNoRaster,
  kInvalidConfiguration,
  kStartOutside,
  kStartUnknown,
  kStartObstacle,
  kGoalOutside,
  kGoalUnknown,
  kGoalObstacle,
  kGoalInDisc,
  kBudgetExceeded,
  kDeadlineExceeded,
  kUnreachable,
};

const char* toString(GlobalGridPlanStatus status);

struct GlobalGridPlan {
  GlobalGridPlanStatus status = GlobalGridPlanStatus::kNoRaster;
  /// One pose per traversed cell, start to goal: cell centre XY, the cell
  /// ground plus the driving offset, yaw along the path (the last pose keeps
  /// the yaw of the segment leading into it).
  std::vector<StateVec> poses;
  /// Human-readable failure reason; empty on success.
  std::string reason;
  std::size_t expansions = 0;
  std::chrono::milliseconds elapsed{0};
  /// XY length of the emitted path, metres.
  double length_m = 0.0;
};

/// Deterministic 8-connected A* over the raster with Euclidean edge costs.
///
/// A move is admitted when both cells are kFree (or the start or goal cell,
/// which may be kUnknown when its known free neighbours agree on the ground),
/// neither cell lies in a transient disc (the start cell excepted: the robot
/// stands where it stands and may drive away from a neighbour), the ground
/// rises by at most the step limit and drops by at most the drop limit, and,
/// for a diagonal, both cardinal side cells are admissible too, so no corner
/// is cut. Marked edges cost their length plus the blocked penalty: a mark
/// is a routing preference, never a veto.
class GlobalGridPlanner {
 public:
  using BlockedEdge = std::function<bool(const StateVec&, const StateVec&)>;

  GlobalGridPlanner(std::shared_ptr<const TraversabilityRaster> raster,
                    GlobalGridPlannerLimits limits);

  /// Plans from the cell containing `start_xy` to the cell containing
  /// `goal_xy`. `blocked` may be empty; it is only consulted when set.
  GlobalGridPlan plan(const Eigen::Vector2d& start_xy,
                      const Eigen::Vector2d& goal_xy,
                      const std::vector<TransientDisc>& discs = {},
                      const BlockedEdge& blocked = {}) const;

  const GlobalGridPlannerLimits& limits() const { return limits_; }

 private:
  std::shared_ptr<const TraversabilityRaster> raster_;
  GlobalGridPlannerLimits limits_;
};

}  // namespace mgg

#endif  // MGG_CORE_GLOBAL_GRID_PLANNER_H_
