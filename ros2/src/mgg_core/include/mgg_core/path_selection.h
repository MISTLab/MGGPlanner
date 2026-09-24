// Choosing where to go: scoring the paths through the local graph and picking
// the best one.
//
// Ported from Rrg::evaluateGraph. Given a graph whose vertices already carry
// volumetric gain, this runs Dijkstra from the root, walks each root-to-leaf
// path, and scores it by accumulated gain discounted for length and for
// deviation from the direction the robot is already exploring in.

#ifndef MGG_CORE_PATH_SELECTION_H_
#define MGG_CORE_PATH_SELECTION_H_

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/gain.h"
#include "mgg_core/graph_manager.h"
#include "mgg_core/map_interface.h"
#include "mgg_core/params.h"
#include "mgg_core/types.h"

namespace mgg {

/// Inclination of each graph edge, keyed by the vertex pair.
///
/// Sparse. The ROS 1 code used a dense num_vertices_max squared matrix of
/// doubles, rebuilt every planning cycle, which at the shipped
/// num_vertices_max of 8000 meant 512 MB of allocation churn per iteration to
/// hold an entry per real edge.
///
/// Entries are stored both ways round. The original wrote only
/// [new][nearest] while the path walk reads [further][closer]; those coincide
/// for the primary link but not for RRG neighbour edges, which then read back
/// as flat and escaped the negative-slope check entirely.
class EdgeInclinations {
 public:
  void set(int a, int b, double value) {
    values_[key(a, b)] = value;
    values_[key(b, a)] = value;
  }
  /// Unknown edges read as flat, matching the zero-filled matrix.
  double get(int a, int b) const {
    auto it = values_.find(key(a, b));
    return it == values_.end() ? 0.0 : it->second;
  }
  void clear() { values_.clear(); }
  size_t size() const { return values_.size(); }

 private:
  static uint64_t key(int a, int b) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
           static_cast<uint32_t>(b);
  }
  std::unordered_map<uint64_t, double> values_;
};

/// Whether the robot, stopped at `viewpoint`, keeps its footprint clear of
/// known obstacles: no occupied voxel within its inscribed radius (half the
/// smaller of RobotParams::size x and y) plus
/// PlanningParams::viewpoint_clearance_margin, in the xy plane, over the
/// height of its collision box. The lattice's body check is a box aligned with
/// the map and only size_extension larger than the robot, so a vertex it
/// admits can still end a path with the robot's flank or corner against a
/// wall: routes on the SubT finals ended 0.08 and 0.11 m from a lethal cell
/// of the controller's costmap (2026-09-23). Unknown space passes, as it does
/// for the lattice's body check, and so does a query the map cannot answer.
bool viewpointClear(const MapInterface& map, const RobotParams& robot,
                    const PlanningParams& planning, const StateVec& viewpoint);

/// Whether an exploration path may end at a vertex, e.g. viewpointClear.
using ViewpointClearFn = std::function<bool(const Vertex&)>;

/// Ends `route` at its last pose that passes `clear`, dropping the poses
/// after it; the first pose, where the robot stands, is never asked. Returns
/// false, leaving `route` untouched, when no pose after the first passes.
bool pullBackToClearViewpoint(
    std::vector<StateVec>& route,
    const std::function<bool(const StateVec&)>& clear);

struct PathSelectionResult {
  /// Vertex ending the best path, or -1 if none scored above zero: a leaf, or
  /// the vertex its path was pulled back to.
  int best_path_id = -1;
  double best_gain = 0.0;
  /// The best path, root first.
  std::vector<Vertex*> best_path;
  /// Any vertex on any evaluated path was a frontier.
  bool frontier_exists = false;
  int leaves_evaluated = 0;
  /// Paths discarded for descending more steeply than max_negative_inclination.
  int paths_rejected_steep = 0;
  /// Paths whose leaf failed `viewpoint_clear`: those pulled back along the
  /// path to the last vertex that passes, and those with none past the root.
  int paths_pulled_back = 0;
  int paths_without_clear_viewpoint = 0;
  /// No admissible path ended clear, so the best path was chosen without
  /// the clearance check.
  bool unclear_viewpoint = false;
};

/// Scores every root-to-leaf path and returns the best.
///
/// `exploring_direction` is the heading the robot has been travelling, used to
/// penalise paths that double back. Gain must already have been computed, for
/// instance by computeExplorationGain. Paths ending within
/// `exclusion_radius` of an `excluded_endpoints` point are skipped; a path
/// pulled back is checked where it now ends.
///
/// With `viewpoint_clear`, a path whose leaf fails it ends at the last vertex
/// along it that passes, and is scored up to there. The best path ending
/// clear wins; only when there is none is the best path chosen without the
/// check, flagged unclear_viewpoint, so that clearance never stops
/// exploration where it would have gone on.
PathSelectionResult selectBestPath(GraphManager& graph,
                                   const PlanningParams& planning,
                                   const RobotParams& robot,
                                   const EdgeInclinations& inclinations,
                                   double map_resolution,
                                   double exploring_direction,
                                   const std::vector<Eigen::Vector3d>&
                                       excluded_endpoints = {},
                                   double exclusion_radius = 0.0,
                                   const ViewpointClearFn& viewpoint_clear =
                                       nullptr);

}  // namespace mgg

#endif  // MGG_CORE_PATH_SELECTION_H_
