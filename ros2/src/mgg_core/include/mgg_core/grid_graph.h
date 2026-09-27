// The grid local planner: the "grid" in Multi-robot Grid Graph.
//
// Instead of sampling the local space at random, it walks a regular lattice
// around the robot, keeps the cells the robot fits in, and connects each one
// through expandGraph. That is the speed claim the method rests on.
//
// Ported from Rrg::buildGridGraphExapnd. Two changes worth stating:
//
//   * The grid resolution is a named field. In the ROS 1 code it was carried
//     by BoundedSpaceParams::min_extension, a bounds parameter overloaded to
//     mean something else and distinguished only by a "# Resolution" comment
//     in the YAML (rrg.cpp:3216 assigned it to grid_graph_res_val_). That
//     overloading made an otherwise reasonable configuration edit, zeroing the
//     bound extensions, silently disable local exploration.
//   * The function no longer leaks a GraphManager per call. Both ROS 1 grid
//     builders opened with "GraphManager* grid_graph = new GraphManager();"
//     and never used or freed it, once per planning cycle.

#ifndef MGG_CORE_GRID_GRAPH_H_
#define MGG_CORE_GRID_GRAPH_H_

#include <Eigen/Dense>

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "mgg_core/graph_expansion.h"
#include "mgg_core/graph_manager.h"
#include "mgg_core/types.h"

namespace mgg {

/// The lattice swept around the robot.
struct GridGraphParams {
  /// Lower corner relative to the robot, metres. Every component must be <= 0.
  Eigen::Vector3d min_val = Eigen::Vector3d(-10.0, -10.0, -1.0);
  /// Upper corner relative to the robot, metres. Every component must be >= 0.
  Eigen::Vector3d max_val = Eigen::Vector3d(10.0, 10.0, 0.2);
  /// Cell size, metres. No component may be zero.
  Eigen::Vector3d resolution = Eigen::Vector3d(0.5, 0.5, 0.2);
};

enum class GridGraphStatus {
  kOk = 0,
  /// min_val had a positive component, max_val a negative one, or a resolution
  /// component was zero: the lattice would not contain the robot.
  kInvalidBounds,
};

struct GridGraphResult {
  GridGraphStatus status = GridGraphStatus::kOk;
  /// Cells whose footprint was free: candidates offered to expandGraph, and
  /// merged_duplicates.
  int free_cells = 0;
  int vertices_added = 0;
  int edges_added = 0;
  /// True when a size or loop cap stopped the sweep early.
  bool hit_limit = false;
  /// Free cells not offered to expandGraph because their ground lies within
  /// a step of a vertex already at the same lattice column
  /// (LatticeColumnGround): the same place reached from another z level.
  int merged_duplicates = 0;
  /// Cells on another level than the robot's (ground more than a step
  /// above or below) refused an edge in the sweep and joined on a retry
  /// pass once vertices beyond them had been added: ground reached only by
  /// going outward first (a deck over the floor, reached up a ramp further
  /// out).
  int retried_joined = 0;
  /// Why the candidates that were offered got turned away, indexed by
  /// ExpandGraphStatus. A sweep that finds plenty of free cells and produces
  /// no vertices is otherwise indistinguishable from one that found nothing,
  /// and the two have completely different causes.
  int rejected[6] = {0, 0, 0, 0, 0, 0};
  /// Candidates rejected because no ground was found beneath them.
  int no_ground = 0;
  /// Projected candidates rejected by the explicit-objective endpoint box
  /// check, split by the two actionable failure classes.
  int projected_endpoint_occupied = 0;
  int projected_endpoint_unknown = 0;
  /// Edge verdicts summed over every candidate, indexed by
  /// ProjectedEdgeStatus.
  int edge_status[8] = {0, 0, 0, 0, 0, 0, 0, 0};
};

/// The driving heights of the vertices a ground lattice holds at each of its
/// columns. A lattice column has one cell per z level, and on one floor
/// every level projects onto the same ground: run 8's lattices held 4.9
/// vertices per position, so the vertex limit held a fifth of the area.
/// Ground within `step` (max_step_height) of a held height is the same
/// ground; farther apart it is another level, a floor under a walkway, and
/// is held apart.
class LatticeColumnGround {
 public:
  explicit LatticeColumnGround(double step) : step_(step) {}
  /// Whether column (i, j) holds a vertex within the step of height `z`.
  bool holds(int i, int j, double z) const;
  void add(int i, int j, double z);

 private:
  static std::int64_t key(int i, int j) {
    return (static_cast<std::int64_t>(i) << 32) ^ static_cast<std::uint32_t>(j);
  }
  double step_;
  std::unordered_map<std::int64_t, std::vector<double>> heights_;
};

/// Retry passes over the cells the sweep refused an edge (GridGraphResult::
/// retried_joined); each stops the retries when it joins nothing.
inline constexpr int kGridGraphRetryPasses = 4;

/// Sweeps the lattice around `state` and grows `graph` through it.
///
/// `heading` rotates the lattice about z so it follows the robot rather than
/// the world axes. Columns are swept outward from the robot, nearest first,
/// so a size or loop cap leaves out the farthest cells; cells on another
/// level than the robot's refused an edge are offered again after the
/// sweep (kGridGraphRetryPasses). For a
/// ground robot, a cell whose ground a vertex at its column already stands
/// on (LatticeColumnGround) is not offered again.
GridGraphResult buildGridGraph(GraphManager& graph, const StateVec& state,
                               const GridGraphParams& grid,
                               const ExpandContext& ctx, double heading);

}  // namespace mgg

#endif  // MGG_CORE_GRID_GRAPH_H_
