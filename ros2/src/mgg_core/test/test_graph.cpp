// Tests for the shared graph structure. The ROS 1 code had none: exercising
// it required launching Gazebo and driving a robot around.

#include <gtest/gtest.h>

#include "mgg_core/graph.h"

namespace {

using mgg::Graph;
using mgg::ShortestPathsReport;

// 0 --1.0-- 1 --1.0-- 2
//  \                  /
//   \------5.0-------/
Graph makeTriangle() {
  Graph g;
  g.addSourceVertex(0);
  g.addVertex(1);
  g.addVertex(2);
  g.addEdge(0, 1, 1.0);
  g.addEdge(1, 2, 1.0);
  g.addEdge(0, 2, 5.0);
  return g;
}

TEST(Graph, CountsVerticesAndEdges) {
  Graph g = makeTriangle();
  EXPECT_EQ(g.getNumVertices(), 3);
  EXPECT_EQ(g.getNumEdges(), 3);
}

TEST(Graph, EdgeSetPreventsDuplicates) {
  // The adjacency_list uses setS for edges specifically so re-adding an edge
  // does not create a parallel one.
  Graph g = makeTriangle();
  const int before = g.getNumEdges();
  g.addEdge(0, 1, 2.0);
  EXPECT_EQ(g.getNumEdges(), before);
}

TEST(Graph, DijkstraPrefersTheTwoHopRoute) {
  Graph g = makeTriangle();
  ShortestPathsReport rep;
  ASSERT_TRUE(g.findDijkstraShortestPaths(0, rep));
  EXPECT_TRUE(rep.status);
  EXPECT_EQ(rep.source_id, 0);
  // 0->1->2 costs 2.0 and beats the direct edge at 5.0.
  EXPECT_DOUBLE_EQ(rep.distance_map[2], 2.0);
  EXPECT_EQ(rep.parent_id_map[2], 1);
  EXPECT_DOUBLE_EQ(rep.distance_map[1], 1.0);
}

TEST(Graph, RemovingAnEdgeChangesTheShortestPath) {
  Graph g = makeTriangle();
  g.removeEdge(1, 2);
  EXPECT_FALSE(g.edgeExists(1, 2));
  ShortestPathsReport rep;
  ASSERT_TRUE(g.findDijkstraShortestPaths(0, rep));
  // Only the direct 5.0 edge is left.
  EXPECT_DOUBLE_EQ(rep.distance_map[2], 5.0);
  EXPECT_EQ(rep.parent_id_map[2], 0);
}

TEST(Graph, UnknownSourceIsRejectedRatherThanCrashing) {
  Graph g = makeTriangle();
  ShortestPathsReport rep;
  EXPECT_FALSE(g.findDijkstraShortestPaths(42, rep));
}

TEST(Graph, ClearEmptiesTheGraph) {
  Graph g = makeTriangle();
  g.clear();
  EXPECT_EQ(g.getNumVertices(), 0);
  EXPECT_EQ(g.getNumEdges(), 0);
}

TEST(Graph, DisconnectedVertexIsReachableOnlyFromItself) {
  Graph g;
  g.addSourceVertex(0);
  g.addVertex(1);
  g.addVertex(2);
  g.addEdge(0, 1, 1.0);  // vertex 2 left isolated
  ShortestPathsReport rep;
  ASSERT_TRUE(g.findDijkstraShortestPaths(0, rep));
  EXPECT_DOUBLE_EQ(rep.distance_map[1], 1.0);
  // Boost leaves unreachable vertices at the numeric_limits max it seeded
  // them with, so an isolated vertex must not look like a cheap neighbour.
  EXPECT_GT(rep.distance_map[2], 1e6);
}

}  // namespace
