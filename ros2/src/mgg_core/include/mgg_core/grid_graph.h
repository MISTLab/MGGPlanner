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
  /// Cells whose footprint was free, i.e. candidates offered to expandGraph.
  int free_cells = 0;
  int vertices_added = 0;
  int edges_added = 0;
  /// True when a size or loop cap stopped the sweep early.
  bool hit_limit = false;
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
  int edge_status[5] = {0, 0, 0, 0, 0};
};

/// Sweeps the lattice around `state` and grows `graph` through it.
///
/// `heading` rotates the lattice about z so it follows the robot rather than
/// the world axes.
GridGraphResult buildGridGraph(GraphManager& graph, const StateVec& state,
                               const GridGraphParams& grid,
                               const ExpandContext& ctx, double heading);

}  // namespace mgg

#endif  // MGG_CORE_GRID_GRAPH_H_
