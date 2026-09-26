// Frontier clusters of the global graph: what the tour orders and the fleet
// auctions (tour-exploration design §2.1).
//
// A cluster is a best-gain frontier vertex, its representative, and every
// other frontier within the merge radius of it. The design took the
// grouping from Vertex::cluster_id, but performShortestPathsClustering sets
// that on the local graph's leaves only: the global graph holds one frontier
// per principal path (addFrontiers) and the expansion's frontiers, and none
// carries it. The global frontiers are grouped here, and cluster_id is set
// to the representative's vertex id.
//
// A cluster's stable ID names a place: the owning robot and its
// representative's cell on a grid of tour.cluster_id_cell_m when first
// seen. ClusterIdRegistry keeps the name while the representative moves
// within the cluster, so fleet messages can refer to it across graph
// revisions.

#ifndef MGG_CORE_FRONTIER_CLUSTERS_H_
#define MGG_CORE_FRONTIER_CLUSTERS_H_

#include <cstdint>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/graph_manager.h"

namespace mgg {

/// Robot ID + 1 in bits 48 to 62, then the x, y and z cells, 16 bits each
/// offset by 2^15 (about 32 km either way at 1 m cells). Never zero.
using ClusterId = std::uint64_t;
inline constexpr ClusterId kNoCluster = 0;

ClusterId makeClusterId(int robot_id, const Eigen::Vector3d& position,
                        double cell_m);

struct FrontierCluster {
  ClusterId id = kNoCluster;
  /// The robot whose frontier the representative is.
  int owner_robot_id = 0;
  /// The best-gain member, a vertex of this robot's global graph.
  int representative_vertex_id = -1;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  /// The representative's gain.
  double gain = 0.0;
  /// The representative first.
  std::vector<int> member_vertex_ids;
};

/// The in-service kFrontier vertices of `graph`, grouped: the best-gain
/// vertex not yet grouped represents a cluster, and every ungrouped frontier
/// within `merge_radius_m` of it joins. Ties in gain go to the lower vertex
/// id. Clusters whose representative gain is below `min_cluster_gain` are
/// dropped. Returned best gain first, with raw IDs (makeClusterId; a second
/// representative in the same cell takes the next free ID).
std::vector<FrontierCluster> extractFrontierClusters(GraphManager& graph,
                                                     double merge_radius_m,
                                                     double min_cluster_gain,
                                                     double cluster_id_cell_m);

/// Keeps each cluster's ID across graph revisions: a cluster whose
/// representative lies within `match_radius_m` of one named last time takes
/// that name (nearest first, each name once); the others keep their raw
/// IDs, bumped past any name already in use. Names of clusters that vanished
/// are forgotten.
class ClusterIdRegistry {
 public:
  void stabilize(std::vector<FrontierCluster>& clusters,
                 double match_radius_m);

 private:
  struct Known {
    ClusterId id = kNoCluster;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
  };
  std::vector<Known> known_;
};

/// Whether this robot's roadmap shows `position` explored: an in-service
/// vertex lies within `radius_m` of it and no frontier does. Far from the
/// roadmap it cannot tell, and answers false.
bool exploredInGraph(GraphManager& graph, const Eigen::Vector3d& position,
                     double radius_m);

}  // namespace mgg

#endif  // MGG_CORE_FRONTIER_CLUSTERS_H_
