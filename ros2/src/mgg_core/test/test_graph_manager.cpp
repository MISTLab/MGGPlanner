// Tests for the vertex store: kd-tree lookup, shortest paths over the graph,
// and the per-robot vertex maps the multi-robot merge relies on.

#include <algorithm>
#include <cmath>
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

TEST(GraphManager, CuttingPeerEdgesRetiresOwnershipAcrossAllSenders) {
  GraphManager gm;
  gm.setRobotId(1);
  auto* own = new Vertex(0, StateVec(0, 0, 0, 0));
  gm.addVertex(own);
  auto* a = new Vertex(1, StateVec(1, 0, 0, 0));
  a->robot_id = 2;
  gm.addNeighbourVertex(a, 0);
  auto* b = new Vertex(2, StateVec(2, 0, 0, 0));
  b->robot_id = 3;
  gm.addNeighbourVertex(b, 0);
  auto* b_next = new Vertex(3, StateVec(3, 0, 0, 0));
  b_next->robot_id = 3;
  gm.addNeighbourVertex(b_next, 1);
  gm.addEdge(own, a, 1);
  gm.addEdge(a, b, 1);
  gm.addEdge(b, b_next, 1);
  // A installed its rendezvous with B; B installed its internal edge.
  gm.neighbour_placements_[2].merge_owned_edges = {{0, 1}, {1, 2}};
  gm.neighbour_placements_[3].merge_owned_edges = {{2, 3}};
  EXPECT_EQ(gm.cutNeighbourEdges(3), 2);
  EXPECT_EQ(gm.neighbour_placements_[2].merge_owned_edges.count({1, 2}), 0u);
  EXPECT_EQ(gm.neighbour_placements_[2].merge_owned_edges.count({0, 1}), 1u);
  EXPECT_TRUE(gm.neighbour_placements_[3].merge_owned_edges.empty());
  // B's re-merge installs the same pair. A's next refresh must not cut
  // B's newly owned edge, even if B's unchanged snapshot is cached afterward.
  gm.addEdge(a, b, 1);
  gm.neighbour_placements_[3].merge_owned_edges.insert({1, 2});
  EXPECT_EQ(gm.cutMergeOwnedEdges(2), 1);
  EXPECT_TRUE(gm.graph_->edgeExists(1, 2));
  EXPECT_EQ(gm.getNumEdges(), 1);
  ASSERT_EQ(gm.edge_map_.at(1).size(), 1u);
  ASSERT_EQ(gm.edge_map_.at(2).size(), 1u);
  EXPECT_EQ(gm.edge_map_.at(1).front().first, 2);
  EXPECT_EQ(gm.edge_map_.at(2).front().first, 1);
}

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

TEST(GraphManager, UpdatingVertexStateRebuildsNearestIndex) {
  Chain c;
  ASSERT_TRUE(c.gm.updateVertexState(0, StateVec(20.0, 0.0, 0.0, 0.0)));

  Vertex* found = nullptr;
  const StateVec moved(20.0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(c.gm.getNearestVertexInRange(&moved, 0.1, &found));
  ASSERT_NE(found, nullptr);
  EXPECT_EQ(found->id, 0);

  const StateVec old_position = StateVec::Zero();
  EXPECT_FALSE(c.gm.getNearestVertexInRange(&old_position, 0.1, &found));
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

// No-go zones (run 8): an edge through one stays in the graph but is closed
// to every search, which then detours, or finds nothing; opening the edges
// restores the shortest route.
TEST(GraphManager, BlockedEdgesAreLeftOutOfTheShortestPaths) {
  GraphManager g;
  std::vector<Vertex*> v;
  for (const auto& xy : std::vector<std::pair<double, double>>{
           {0, 0}, {1, 0}, {2, 0}, {1, 1}}) {
    v.push_back(new Vertex(static_cast<int>(v.size()),
                           StateVec(xy.first, xy.second, 0, 0)));
    g.addVertex(v.back());
  }
  g.addEdge(v[0], v[1], 1.0);
  g.addEdge(v[1], v[2], 1.0);
  g.addEdge(v[0], v[3], std::sqrt(2.0));
  g.addEdge(v[3], v[2], std::sqrt(2.0));
  const auto route = [&g]() {
    ShortestPathsReport rep;
    EXPECT_TRUE(g.findShortestPaths(0, rep));
    std::vector<int> path;
    if (rep.parent_id_map.at(2) == 2) return path;  // unreached
    g.getShortestPath(2, rep, true, path);
    return path;
  };
  EXPECT_EQ(route(), (std::vector<int>{0, 1, 2}));
  // A zone at (1.5, 0) closes the straight edge 1-2.
  const Eigen::Vector2d zone(1.5, 0.0);
  g.setEdgeBlocked([&zone](const Vertex& a, const Vertex& b) {
    const Eigen::Vector2d p = a.state.head<2>(), d = b.state.head<2>() - p;
    const double t = std::clamp((zone - p).dot(d) / d.squaredNorm(), 0.0, 1.0);
    return (zone - (p + t * d)).norm() < 0.3;
  });
  EXPECT_EQ(route(), (std::vector<int>{0, 3, 2}));
  EXPECT_EQ(g.getNumEdges(), 4);
  g.setEdgeBlocked([](const Vertex&, const Vertex&) { return true; });
  EXPECT_TRUE(route().empty());
  g.setEdgeBlocked(nullptr);
  EXPECT_EQ(route(), (std::vector<int>{0, 1, 2}));
}

}  // namespace
