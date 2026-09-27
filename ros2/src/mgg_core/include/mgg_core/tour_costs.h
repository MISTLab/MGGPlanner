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
/// graph instance or its revision changes.
class GraphDistanceCache {
 public:
  /// The report from `source_id`, solved on first use in this `graph` at
  /// this `revision`; null when the source is not in the graph or Dijkstra
  /// cannot run (a graph of fewer than two vertices).
  const ShortestPathsReport* from(GraphManager& graph, std::uint64_t revision,
                                  int source_id);
  /// Dijkstra runs so far.
  std::size_t solves() const { return solves_; }

 private:
  bool valid_ = false;
  /// Identity only, never dereferenced: which graph the reports came from.
  const GraphManager* graph_ = nullptr;
  std::uint64_t revision_ = 0;
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
  /// Graph distances between representatives; symmetric, zero diagonal.
  std::vector<std::vector<double>> between;
};

/// Costs of `clusters` from `source_vertex_id`, the vertex the robot joins
/// the graph at. Unreachable legs are kUnreachableCost, which leaves a
/// cluster out of the tour until it connects.
TourCostMatrix computeTourCosts(GraphManager& graph, std::uint64_t revision,
                                GraphDistanceCache& cache,
                                int source_vertex_id, double robot_yaw,
                                const std::vector<FrontierCluster>& clusters,
                                double heading_weight);

/// A robot with limited flight left (a drone on battery, drone scout design
/// §4.3) does not take clusters it could reach but not return from: those
/// whose cost from the robot plus `cluster_to_home` exceeds `reach_m` become
/// kUnreachableCost. An infinite reach changes nothing.
void capTourCostsByReach(TourCostMatrix& costs,
                         const std::vector<double>& cluster_to_home,
                         double reach_m);

}  // namespace mgg

#endif  // MGG_CORE_TOUR_COSTS_H_
