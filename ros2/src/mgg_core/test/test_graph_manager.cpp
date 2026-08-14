// Tests for the vertex store: kd-tree lookup, shortest paths over the graph,
// and the per-robot vertex maps the multi-robot merge relies on.

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/graph_manager.h"

namespace {

using mgg::GraphManager;
using mgg::ShortestPathsReport;
using mgg::StateVec;
using mgg::Vertex;
using mgg::VertexType;

/// A chain 0 -- 1 -- 2 -- 3 spaced 1 m apart along +x.
class Chain {
 public:
  Chain() {
    Vertex* prev = nullptr;
    for (int i = 0; i < 4; ++i) {
      // GraphManager takes ownership; deliberately not held in a unique_ptr
      // here, or reset() and the destructor would double-free.
      auto* v = new Vertex(i, StateVec(i * 1.0, 0.0, 0.0, 0.0));
      gm.addVertex(v);
      if (prev != nullptr) gm.addEdge(v, prev, 1.0);
      prev = v;
    }
  }
  GraphManager gm;
};

TEST(GraphManager, TracksVertexAndEdgeCounts) {
  Chain c;
  EXPECT_EQ(c.gm.getNumVertices(), 4);
  EXPECT_EQ(c.gm.getNumEdges(), 3);
}

TEST(GraphManager, NearestVertexUsesTheKdTree) {
  Chain c;
  const StateVec query(2.1, 0.0, 0.0, 0.0);
  Vertex* found = nullptr;
  ASSERT_TRUE(c.gm.getNearestVertex(&query, &found));
  ASSERT_NE(found, nullptr);
  EXPECT_EQ(found->id, 2);
}

TEST(GraphManager, NearestInRangeRespectsTheRadius) {
  Chain c;
  const StateVec far_away(100.0, 0.0, 0.0, 0.0);
  Vertex* found = nullptr;
  EXPECT_FALSE(c.gm.getNearestVertexInRange(&far_away, 1.0, &found));

  const StateVec near(0.1, 0.0, 0.0, 0.0);
  ASSERT_TRUE(c.gm.getNearestVertexInRange(&near, 1.0, &found));
  EXPECT_EQ(found->id, 0);
}

TEST(GraphManager, NearestVerticesReturnsEveryoneInRange) {
  Chain c;
  const StateVec middle(1.5, 0.0, 0.0, 0.0);
  std::vector<Vertex*> found;
  ASSERT_TRUE(c.gm.getNearestVertices(&middle, 1.0, &found));
  // Vertices 1 and 2 sit 0.5 m away; 0 and 3 are 1.5 m away.
  EXPECT_EQ(found.size(), 2u);
}

TEST(GraphManager, ShortestPathWalksTheChain) {
  Chain c;
  ShortestPathsReport rep;
  ASSERT_TRUE(c.gm.findShortestPaths(0, rep));
  EXPECT_DOUBLE_EQ(c.gm.getShortestDistance(3, rep), 3.0);

  std::vector<int> path;
  c.gm.getShortestPath(3, rep, true, path);
  ASSERT_EQ(path.size(), 4u);
  EXPECT_EQ(path.front(), 0);
  EXPECT_EQ(path.back(), 3);
}

TEST(GraphManager, ShortestPathCanBeReturnedTargetFirst) {
  Chain c;
  ShortestPathsReport rep;
  ASSERT_TRUE(c.gm.findShortestPaths(0, rep));
  std::vector<int> path;
  c.gm.getShortestPath(3, rep, false, path);
  ASSERT_EQ(path.size(), 4u);
  EXPECT_EQ(path.front(), 3);
  EXPECT_EQ(path.back(), 0);
}

TEST(GraphManager, ShortestPathAsStatesCarriesTheYaw) {
  Chain c;
  ShortestPathsReport rep;
  ASSERT_TRUE(c.gm.findShortestPaths(0, rep));
  std::vector<StateVec> path;
  c.gm.getShortestPath(2, rep, true, path);
  ASSERT_FALSE(path.empty());
  EXPECT_DOUBLE_EQ(path.back()[0], 2.0);
}

TEST(GraphManager, NeighbourVerticesAreKeyedByRobot) {
  GraphManager gm;
  gm.addVertex(new Vertex(0, StateVec::Zero()));

  // A vertex that robot 2 called id 57 in its own graph.
  auto* theirs = new Vertex(1, StateVec(5.0, 0.0, 0.0, 0.0));
  theirs->robot_id = 2;
  gm.addNeighbourVertex(theirs, 57);

  EXPECT_EQ(gm.getNeighbourVertex(57, 2), theirs);
}

TEST(GraphManager, ResetEmptiesTheStore) {
  Chain c;
  ASSERT_EQ(c.gm.getNumVertices(), 4);
  c.gm.reset();
  EXPECT_EQ(c.gm.getNumVertices(), 0);
  EXPECT_EQ(c.gm.getNumEdges(), 0);
}

TEST(GraphManager, UpdateVertexTypeInRangeMarksVisited) {
  Chain c;
  StateVec at_start(0.0, 0.0, 0.0, 0.0);
  c.gm.updateVertexTypeInRange(at_start, 1.5);
  // Vertices 0 and 1 are within 1.5 m and should now be visited.
  EXPECT_EQ(c.gm.getVertex(0)->type, VertexType::kVisited);
}

}  // namespace
