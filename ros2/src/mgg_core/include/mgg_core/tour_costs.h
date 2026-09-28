// The tour's costs on the global graph (tour-exploration design §2.2):
// robot-to-cluster and cluster-to-cluster shortest-path lengths between
// representatives, one Dijkstra per source cached per graph revision, and a
// heading-change penalty on the first leg so the tour does not start with a
// U-turn when a cluster lies ahead.

#ifndef MGG_CORE_TOUR_COSTS_H_
#define MGG_CORE_TOUR_COSTS_H_

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "mgg_core/frontier_clusters.h"
#include "mgg_core/graph.h"
#include "mgg_core/graph_manager.h"
#include "mgg_core/tour_solver.h"

namespace mgg {

/// The first leg's heading is the direction from the robot to the first
/// vertex of its route at least this far away (or the route's end).
inline constexpr double kFirstLegLookaheadM = 2.0;

/// Dijkstra reports over one graph, one per source vertex, dropped when the
/// graph instance, its revision or the peer generation changes: the edges a
/// peer body closes change with the peers alone (review r0, I3).
class GraphDistanceCache {
 public:
  /// The report from `source_id`, solved on first use in this `graph` at
  /// this `revision` and `peer_generation`; null when the source is not in
  /// the graph or Dijkstra cannot run (a graph of fewer than two vertices).
  const ShortestPathsReport* from(GraphManager& graph, std::uint64_t revision,
                                  int source_id,
                                  std::uint64_t peer_generation = 0);
  /// Dijkstra runs so far.
  std::size_t solves() const { return solves_; }

 private:
  bool valid_ = false;
  /// Identity only, never dereferenced: which graph the reports came from.
  const GraphManager* graph_ = nullptr;
  std::uint64_t revision_ = 0;
  std::uint64_t peer_generation_ = 0;
  std::unordered_map<int, ShortestPathsReport> reports_;
  std::unordered_set<int> failed_;
  std::size_t solves_ = 0;
};

/// The graph distance `report` found to `target_id`, or kUnreachableCost
/// when Dijkstra did not reach it.
double reachedDistance(const ShortestPathsReport& report, int target_id);

/// Absolute heading change, radians in [0, pi], from `robot_yaw` to the
/// first leg of the shortest route to `target_id` (kFirstLegLookaheadM).
/// Zero when the target is the source or unreachable.
double firstLegHeadingChange(GraphManager& graph,
                             const ShortestPathsReport& from_robot,
                             int target_id, double robot_yaw);

struct TourCostMatrix {
  /// Per cluster: graph distance from the robot's vertex plus
  /// heading_weight times the first leg's heading change.
  std::vector<double> from_robot;
  /// Per cluster: the graph distance alone, the flight a leg takes.
  std::vector<double> distance_from_robot;
  /// Graph distances between representatives; symmetric, zero diagonal:
  /// the longer of the two directions, unreachable if either is.
  std::vector<std::vector<double>> between;
};

/// Costs of `clusters` from `source_vertex_id`, the vertex the robot joins
/// the graph at, cached by `revision` and `peer_generation`. Unreachable
/// legs are kUnreachableCost, which leaves a cluster out of the tour until
/// it connects.
TourCostMatrix computeTourCosts(GraphManager& graph, std::uint64_t revision,
                                GraphDistanceCache& cache,
                                int source_vertex_id, double robot_yaw,
                                const std::vector<FrontierCluster>& clusters,
                                double heading_weight,
                                std::uint64_t peer_generation = 0);

/// A robot with limited flight left (a drone on battery, drone scout design
/// §4.3) does not take clusters it could reach but not return from: those
/// whose distance from the robot (distance_from_robot, without the heading
/// preference, which only orders the tour) plus `cluster_to_home` exceeds
/// `reach_m` get a from_robot cost of kUnreachableCost. An infinite reach
/// changes nothing.
void capTourCostsByReach(TourCostMatrix& costs,
                         const std::vector<double>& cluster_to_home,
                         double reach_m);

/// A cluster's value to the robot: its gain discounted by the graph
/// distance to it, gain * exp(-kGlobalDistancePenalty * distance), as
/// MGG's global planner valued a frontier (rrg.cpp:5798 to 5801,
/// searchGlobalFrontier). Zero when the cluster cannot be reached.
double tourClusterValue(const FrontierCluster& cluster, double distance);

/// Leaves out of the tour every cluster whose tourClusterValue, at its
/// distance_from_robot, is below `min_value` (tour.min_cluster_gain): its
/// from_robot becomes kUnreachableCost. At 0.05 per metre a cluster 60 m
/// away needs about 20 times the gain of one at the robot. A cluster
/// another robot than `robot_id` owns is worth `other_robot_factor` of
/// that (kGlobalOtherRobotPenalty, rrg.cpp:5804: each robot prefers the
/// frontiers it found). The tour orders the clusters left; the others are
/// the greedy search's to weigh. Returns how many it left out. Run 12,
/// robot_3: a tour of one small cluster 48 m back in the start hangar sent
/// it back through the corridor.
int capTourCostsByValue(TourCostMatrix& costs,
                        const std::vector<FrontierCluster>& clusters,
                        double min_value, int robot_id,
                        double other_robot_factor);

}  // namespace mgg

#endif  // MGG_CORE_TOUR_COSTS_H_
