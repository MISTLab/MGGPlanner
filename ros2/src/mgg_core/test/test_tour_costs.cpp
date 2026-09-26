// Tests for the tour's costs on the global graph (tour-exploration design
// §2.2): shortest-path lengths cached per graph revision, unreachable
// clusters left out, and the first leg's heading penalty.

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/tour_costs.h"

namespace {

using mgg::FrontierCluster;
using mgg::GraphDistanceCache;
using mgg::GraphManager;
using mgg::StateVec;
using mgg::TourCostMatrix;
using mgg::Vertex;
using mgg::VertexType;

Vertex* add(GraphManager& graph, double x, double y,
            Vertex* linked_to = nullptr,
            VertexType type = VertexType::kUnvisited) {
  auto* v = new Vertex(graph.generateVertexID(), StateVec(x, y, 0.0, 0.0));
  v->type = type;
  v->robot_id = 1;
  graph.addVertex(v);
  if (linked_to != nullptr) {
    graph.addEdge(v, linked_to,
                  (v->state.head<3>() - linked_to->state.head<3>()).norm());
  }
  return v;
}

FrontierCluster clusterAt(const Vertex* v) {
  FrontierCluster c;
  c.id = 100 + v->id;
  c.owner_robot_id = 1;
  c.representative_vertex_id = v->id;
  c.position = v->state.head<3>();
  c.member_vertex_ids = {v->id};
  return c;
}

TEST(TourCosts, CostsAreShortestPathLengthsBetweenRepresentatives) {
  GraphManager graph;
  Vertex* root = add(graph, 0.0, 0.0);
  Vertex* corner = add(graph, 5.0, 0.0, root, VertexType::kFrontier);
  Vertex* end = add(graph, 5.0, 5.0, corner, VertexType::kFrontier);
  Vertex* island = add(graph, 20.0, 20.0, nullptr, VertexType::kFrontier);
  GraphDistanceCache cache;
  const TourCostMatrix costs = mgg::computeTourCosts(
      graph, 1, cache, root->id, 0.0,
      {clusterAt(corner), clusterAt(end), clusterAt(island)}, 0.0);
  ASSERT_EQ(costs.from_robot.size(), 3u);
  EXPECT_DOUBLE_EQ(costs.from_robot[0], 5.0);
  EXPECT_DOUBLE_EQ(costs.from_robot[1], 10.0);
  EXPECT_EQ(costs.from_robot[2], mgg::kUnreachableCost);
  EXPECT_DOUBLE_EQ(costs.between[0][1], 5.0);
  EXPECT_DOUBLE_EQ(costs.between[1][0], 5.0);
  EXPECT_EQ(costs.between[0][2], mgg::kUnreachableCost);
  EXPECT_DOUBLE_EQ(costs.between[2][2], 0.0);
  // The island is left out of the tour until it connects.
  const mgg::OpenTour tour =
      mgg::solveOpenTour(costs.from_robot, costs.between);
  EXPECT_EQ(tour.order, (std::vector<int>{0, 1}));
}

TEST(TourCosts, DistancesAreSolvedOncePerGraphRevision) {
  GraphManager graph;
  Vertex* root = add(graph, 0.0, 0.0);
  Vertex* a = add(graph, 5.0, 0.0, root, VertexType::kFrontier);
  Vertex* b = add(graph, 5.0, 5.0, a, VertexType::kFrontier);
  GraphDistanceCache cache;
  const std::vector<FrontierCluster> clusters{clusterAt(a), clusterAt(b)};
  mgg::computeTourCosts(graph, 1, cache, root->id, 0.0, clusters, 0.0);
  EXPECT_EQ(cache.solves(), 3u);  // the robot and each representative
  mgg::computeTourCosts(graph, 1, cache, root->id, 0.0, clusters, 0.0);
  EXPECT_EQ(cache.solves(), 3u);
  mgg::computeTourCosts(graph, 2, cache, root->id, 0.0, clusters, 0.0);
  EXPECT_EQ(cache.solves(), 6u);
}

TEST(TourCosts, AnotherGraphAtTheSameRevisionIsSolvedAfresh) {
  // Two graphs with the same vertex IDs and revision: the cache must not
  // hand the second graph the first one's distances.
  GraphManager near_graph;
  Vertex* near_root = add(near_graph, 0.0, 0.0);
  Vertex* near_goal =
      add(near_graph, 5.0, 0.0, near_root, VertexType::kFrontier);
  GraphManager far_graph;
  Vertex* far_root = add(far_graph, 0.0, 0.0);
  Vertex* far_goal = add(far_graph, 20.0, 0.0, far_root, VertexType::kFrontier);
  ASSERT_EQ(near_goal->id, far_goal->id);
  GraphDistanceCache cache;
  EXPECT_DOUBLE_EQ(mgg::computeTourCosts(near_graph, 1, cache, near_root->id,
                                         0.0, {clusterAt(near_goal)}, 0.0)
                       .from_robot[0],
                   5.0);
  EXPECT_DOUBLE_EQ(mgg::computeTourCosts(far_graph, 1, cache, far_root->id,
                                         0.0, {clusterAt(far_goal)}, 0.0)
                       .from_robot[0],
                   20.0);
}

TEST(TourCosts, AFailedSourceInAnotherGraphAtTheSameRevisionIsRetried) {
  // A lone vertex cannot be solved; a linked graph with the same IDs and
  // revision can.
  GraphManager lone;
  Vertex* only = add(lone, 0.0, 0.0);
  GraphManager linked;
  Vertex* root = add(linked, 0.0, 0.0);
  Vertex* goal = add(linked, 5.0, 0.0, root, VertexType::kFrontier);
  ASSERT_EQ(only->id, root->id);
  GraphDistanceCache cache;
  EXPECT_EQ(cache.from(lone, 1, only->id), nullptr);
  const TourCostMatrix costs = mgg::computeTourCosts(
      linked, 1, cache, root->id, 0.0, {clusterAt(goal)}, 0.0);
  EXPECT_DOUBLE_EQ(costs.from_robot[0], 5.0);
}

TEST(TourCosts, AnUnlinkedSourceLeavesEveryClusterUnreachable) {
  // Review Focus 4: the robot's pose joined no vertex, or the graph is a
  // lone root. No cluster is reachable and the tour is empty.
  GraphManager graph;
  Vertex* root = add(graph, 0.0, 0.0);
  Vertex* a = add(graph, 5.0, 0.0, root, VertexType::kFrontier);
  GraphDistanceCache cache;
  const TourCostMatrix unlinked =
      mgg::computeTourCosts(graph, 1, cache, 999, 0.0, {clusterAt(a)}, 2.0);
  EXPECT_EQ(unlinked.from_robot[0], mgg::kUnreachableCost);
  EXPECT_TRUE(mgg::solveOpenTour(unlinked.from_robot, unlinked.between)
                  .order.empty());

  GraphManager lone;
  Vertex* only = add(lone, 0.0, 0.0, nullptr, VertexType::kFrontier);
  GraphDistanceCache lone_cache;
  const TourCostMatrix alone = mgg::computeTourCosts(
      lone, 1, lone_cache, only->id, 0.0, {clusterAt(only)}, 2.0);
  EXPECT_EQ(alone.from_robot[0], mgg::kUnreachableCost);
  EXPECT_TRUE(mgg::computeTourCosts(lone, 1, lone_cache, only->id, 0.0, {},
                                    2.0)
                  .from_robot.empty());
}

TEST(TourCosts, TheHeadingPenaltyAvoidsAUTurnStart) {
  // A corridor along x: the robot at the origin facing +x, a frontier 4 m
  // behind it and one 5 m ahead.
  GraphManager graph;
  Vertex* root = add(graph, 0.0, 0.0);
  Vertex* behind = root;
  for (int x = -1; x >= -4; --x) behind = add(graph, x, 0.0, behind);
  Vertex* ahead = root;
  for (int x = 1; x <= 5; ++x) ahead = add(graph, x, 0.0, ahead);
  behind->type = VertexType::kFrontier;
  ahead->type = VertexType::kFrontier;
  const std::vector<FrontierCluster> clusters{clusterAt(behind),
                                              clusterAt(ahead)};
  GraphDistanceCache cache;

  // By distance alone, behind first: 4 + 9 < 5 + 9.
  const TourCostMatrix plain =
      mgg::computeTourCosts(graph, 1, cache, root->id, 0.0, clusters, 0.0);
  EXPECT_EQ(mgg::solveOpenTour(plain.from_robot, plain.between).order.front(),
            0);
  // A U-turn costs heading_weight * pi.
  const TourCostMatrix penalised =
      mgg::computeTourCosts(graph, 1, cache, root->id, 0.0, clusters, 2.0);
  EXPECT_NEAR(penalised.from_robot[0], 4.0 + 2.0 * M_PI, 1e-9);
  EXPECT_NEAR(penalised.from_robot[1], 5.0, 1e-9);
  EXPECT_EQ(mgg::solveOpenTour(penalised.from_robot, penalised.between)
                .order.front(),
            1);
  // The penalty applies to the first leg only.
  EXPECT_DOUBLE_EQ(penalised.between[0][1], 9.0);
}

TEST(TourCosts, TheFirstLegHeadingLooksPastTheFirstVertex) {
  // The route leaves along +x for half a metre, then turns to +y: the first
  // leg heads +y, a quarter turn from a robot facing +x.
  GraphManager graph;
  Vertex* root = add(graph, 0.0, 0.0);
  Vertex* step = add(graph, 0.5, 0.0, root);
  Vertex* goal = add(graph, 0.5, 3.0, step);
  mgg::ShortestPathsReport report;
  ASSERT_TRUE(graph.findShortestPaths(root->id, report));
  EXPECT_NEAR(mgg::firstLegHeadingChange(graph, report, goal->id, 0.0),
              std::atan2(3.0, 0.5), 1e-9);
  EXPECT_DOUBLE_EQ(mgg::firstLegHeadingChange(graph, report, root->id, 0.0),
                   0.0);
}

}  // namespace
