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
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/gain.h"
#include "mgg_core/graph_manager.h"
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

struct PathSelectionResult {
  /// Leaf ending the best path, or -1 if none scored above zero.
  int best_path_id = -1;
  double best_gain = 0.0;
  /// The best path, root first.
  std::vector<Vertex*> best_path;
  /// Any vertex on any evaluated path was a frontier.
  bool frontier_exists = false;
  int leaves_evaluated = 0;
  /// Paths discarded for descending more steeply than max_negative_inclination.
  int paths_rejected_steep = 0;
};

/// Scores every root-to-leaf path and returns the best.
///
/// `exploring_direction` is the heading the robot has been travelling, used to
/// penalise paths that double back. Gain must already have been computed, for
/// instance by computeExplorationGain.
PathSelectionResult selectBestPath(GraphManager& graph,
                                   const PlanningParams& planning,
                                   const RobotParams& robot,
                                   const EdgeInclinations& inclinations,
                                   double map_resolution,
                                   double exploring_direction,
                                   const std::vector<Eigen::Vector3d>&
                                       excluded_endpoints = {},
                                   double exclusion_radius = 0.0);

}  // namespace mgg

#endif  // MGG_CORE_PATH_SELECTION_H_
