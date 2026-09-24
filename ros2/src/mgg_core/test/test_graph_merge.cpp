// Tests for the multi-robot graph merge, the part of MGG that makes it
// multi-robot at all. The ROS 1 version had none: exercising it needed two
// simulated robots driving until their graphs overlapped.

#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/graph_merge.h"

namespace {

using mgg::EdgeAdmissibleFn;
using mgg::GraphExchange;
using mgg::GraphExchangeEdge;
using mgg::GraphExchangeVertex;
using mgg::GraphManager;
using mgg::StateVec;
using mgg::StaticPoseSource;
using mgg::Vertex;

/// Our own graph: a chain along +x at y=0. Ids come from the manager, as the
/// planner's do, so merged vertices never reuse one.
void buildOwnGraph(GraphManager& gm, int n = 4) {
  Vertex* prev = nullptr;
  for (int i = 0; i < n; ++i) {
    const int id = i == 0 ? 0 : gm.generateVertexID();
    auto* v = new Vertex(id, StateVec(i * 1.0, 0.0, 0.0, 0.0));
    v->robot_id = 1;
    gm.addVertex(v);
    if (prev != nullptr) gm.addEdge(v, prev, 1.0);
    prev = v;
  }
}

/// Robot 2's graph, expressed in robot 2's own frame.
GraphExchange neighbourGraph(int n = 3, int robot_id = 2) {
  GraphExchange g;
  for (int i = 0; i < n; ++i) {
    GraphExchangeVertex v;
    v.id = i;
    v.robot_id = robot_id;
    v.state = StateVec(i * 1.0, 0.0, 0.0, 0.0);
    g.vertices.push_back(v);
  }
  for (int i = 1; i < n; ++i) {
    g.edges.push_back(GraphExchangeEdge{i - 1, i, 1.0});
  }
  return g;
}

const EdgeAdmissibleFn kAlwaysAdmissible =
    [](const Eigen::Vector3d&, const Eigen::Vector3d&) { return true; };
const EdgeAdmissibleFn kNeverAdmissible =
    [](const Eigen::Vector3d&, const Eigen::Vector3d&) { return false; };

TEST(GraphMerge, WithoutATransformNothingIsMerged) {
  // A SLAM-backed PoseSource has no transform until the robots have seen the
  // same place, which must be a quiet no-op rather than an error.
  GraphManager gm;
  buildOwnGraph(gm);
  StaticPoseSource poses;  // deliberately empty
  const auto r = mergeNeighbourGraph(gm, neighbourGraph(), poses,
                                     kAlwaysAdmissible);
  EXPECT_TRUE(r.transform_unavailable);
  EXPECT_FALSE(r.merged);
  EXPECT_EQ(r.vertices_added, 0);
}

TEST(GraphMerge, MergesWhenGraphsOverlapAndTheEdgeIsDrivable) {
  GraphManager gm;
  buildOwnGraph(gm);
  StaticPoseSource poses;
  poses.setOffset(2, 0.0, 1.0);  // robot 2 sits 1 m to our left

  const int before = gm.getNumVertices();
  const auto r = mergeNeighbourGraph(gm, neighbourGraph(), poses,
                                     kAlwaysAdmissible);
  EXPECT_TRUE(r.merged);
  EXPECT_EQ(r.vertices_added, 3);
  EXPECT_EQ(r.edges_added, 2);
  EXPECT_EQ(r.edges_unresolved, 0);
  EXPECT_GT(gm.getNumVertices(), before);
}

TEST(GraphMerge, NoMergeWhenNoEdgeIsDrivable) {
  // Graphs can overlap in space while a wall stands between them.
  GraphManager gm;
  buildOwnGraph(gm);
  StaticPoseSource poses;
  poses.setOffset(2, 0.0, 1.0);
  const auto r = mergeNeighbourGraph(gm, neighbourGraph(), poses,
                                     kNeverAdmissible);
  EXPECT_FALSE(r.merged);
  EXPECT_EQ(r.vertices_added, 0);
}

TEST(GraphMerge, NoMergeWhenTheNeighbourIsFarAway) {
  GraphManager gm;
  buildOwnGraph(gm);
  StaticPoseSource poses;
  poses.setOffset(2, 1000.0, 1000.0);
  const auto r = mergeNeighbourGraph(gm, neighbourGraph(), poses,
                                     kAlwaysAdmissible);
  EXPECT_FALSE(r.merged);
}

// REGRESSION: the ROS 1 merge resolved edge endpoints with getNeighbourVertex,
// which goes through unordered_map::operator[] and so returns nullptr for an
// unknown id, then handed that to addEdge, which dereferences it. A graph
// message naming a vertex it did not carry was a null dereference. Graphs
// arrive from other robots over a link we do not control.
TEST(GraphMerge, EdgeNamingAnAbsentVertexIsSkippedNotDereferenced) {
  GraphManager gm;
  buildOwnGraph(gm);
  StaticPoseSource poses;
  poses.setOffset(2, 0.0, 1.0);

  GraphExchange g = neighbourGraph();
  g.edges.push_back(GraphExchangeEdge{0, 999, 1.0});  // 999 is not in g

  const auto r = mergeNeighbourGraph(gm, g, poses, kAlwaysAdmissible);
  EXPECT_TRUE(r.merged);
  EXPECT_EQ(r.edges_unresolved, 1);
  EXPECT_EQ(r.edges_added, 2);
}

TEST(GraphMerge, SecondCallUpdatesRatherThanDuplicates) {
  GraphManager gm;
  buildOwnGraph(gm);
  StaticPoseSource poses;
  poses.setOffset(2, 0.0, 1.0);

  mergeNeighbourGraph(gm, neighbourGraph(), poses, kAlwaysAdmissible);
  const int after_first = gm.getNumVertices();

  GraphExchange again = neighbourGraph();
  again.vertices[1].is_frontier = true;  // the neighbour learned something
  const auto r = mergeNeighbourGraph(gm, again, poses, kAlwaysAdmissible);

  EXPECT_TRUE(r.merged);
  EXPECT_EQ(r.vertices_added, 0);
  EXPECT_EQ(r.vertices_updated, 3);
  EXPECT_EQ(gm.getNumVertices(), after_first);
  EXPECT_TRUE(gm.getNeighbourVertex(1, 2)->vol_gain.is_frontier);
}

// The ROS 1 offsets translated x and y only, so two robots facing different
// directions merged their graphs rotated. A full transform fixes that.
TEST(GraphMerge, RotationBetweenRobotFramesIsApplied) {
  GraphManager gm;
  buildOwnGraph(gm);

  StaticPoseSource poses;
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = Eigen::Matrix3d(
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  t.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  poses.setTransform(2, t);

  const auto r = mergeNeighbourGraph(gm, neighbourGraph(), poses,
                                     kAlwaysAdmissible);
  ASSERT_TRUE(r.merged);
  // Their vertex 2 sat at (2, 0) in their frame; a quarter turn puts it at
  // (0, 2) in ours. The ROS 1 offset-only merge would have left it at (2, 0).
  const Vertex* v = gm.getNeighbourVertex(2, 2);
  ASSERT_NE(v, nullptr);
  EXPECT_NEAR(v->state[0], 0.0, 1e-9);
  EXPECT_NEAR(v->state[1], 2.0, 1e-9);
}

TEST(GraphMerge, ZOffsetIsCarriedThrough) {
  // The ROS 1 merge copied z straight across, ignoring any height difference
  // between the robots' origins.
  GraphManager gm;
  buildOwnGraph(gm);
  StaticPoseSource poses;
  poses.setOffset(2, 0.0, 1.0, 0.5);
  const auto r = mergeNeighbourGraph(gm, neighbourGraph(), poses,
                                     kAlwaysAdmissible);
  ASSERT_TRUE(r.merged);
  EXPECT_NEAR(gm.getNeighbourVertex(0, 2)->state[2], 0.5, 1e-9);
}

TEST(GraphMerge, ExchangedGroundHeightIsPlacedAtTheReceiversDrivingHeight) {
  // A Spot drives 0.5 m above the floor, a Bunker 0.3 m: the neighbour sends
  // ground height and each receiver adds its own driving height.
  GraphManager gm;
  buildOwnGraph(gm);
  StaticPoseSource poses;
  poses.setOffset(2, 0.0, 1.0, 0.2);
  mgg::ReceiverPlatform bunker;
  bunker.driving_height = 0.3;
  const auto r = mergeNeighbourGraph(gm, neighbourGraph(), poses,
                                     kAlwaysAdmissible, 5.0, bunker);
  ASSERT_TRUE(r.merged);
  EXPECT_NEAR(gm.getNeighbourVertex(1, 2)->state[2], 0.5, 1e-9);
}

TEST(GraphMerge, EdgesTooSteepForTheReceiverAreNotTakenOver) {
  // The neighbour climbed a 0.3 m step its platform allows; ours does not.
  GraphManager gm;
  buildOwnGraph(gm);
  StaticPoseSource poses;
  poses.setOffset(2, 0.0, 1.0);
  GraphExchange g = neighbourGraph();
  g.vertices[2].state[2] = 0.3;
  mgg::ReceiverPlatform scout;
  scout.max_step_height = 0.15;
  scout.max_inclination = 0.2;
  const auto r =
      mergeNeighbourGraph(gm, g, poses, kAlwaysAdmissible, 5.0, scout);
  ASSERT_TRUE(r.merged);
  EXPECT_EQ(r.edges_too_steep, 1);
  EXPECT_EQ(r.edges_added, 1);

  // A platform that takes the step keeps the edge.
  GraphManager spot_graph;
  buildOwnGraph(spot_graph);
  mgg::ReceiverPlatform spot;
  spot.max_step_height = 0.35;
  spot.max_inclination = 0.2;
  const auto s = mergeNeighbourGraph(spot_graph, g, poses, kAlwaysAdmissible,
                                     5.0, spot);
  EXPECT_EQ(s.edges_too_steep, 0);
  EXPECT_EQ(s.edges_added, 2);
}

TEST(GraphMerge, MergedVerticesFollowAMovedTransform) {
  GraphManager gm;
  buildOwnGraph(gm);
  StaticPoseSource poses;
  poses.setOffset(2, 0.0, 1.0);
  mergeNeighbourGraph(gm, neighbourGraph(), poses, kAlwaysAdmissible);
  ASSERT_NEAR(gm.getNeighbourVertex(2, 2)->state[1], 1.0, 1e-9);

  // Below the tolerance nothing moves.
  poses.setOffset(2, 0.0, 1.02);
  auto r = mergeNeighbourGraph(gm, neighbourGraph(), poses, kAlwaysAdmissible);
  EXPECT_EQ(r.vertices_replaced, 0);
  EXPECT_NEAR(gm.getNeighbourVertex(2, 2)->state[1], 1.0, 1e-9);

  // C-SLAM corrected the estimate by 0.5 m: every merged vertex moves, and
  // the nearest-neighbour index follows.
  poses.setOffset(2, 0.0, 1.5);
  r = mergeNeighbourGraph(gm, neighbourGraph(), poses, kAlwaysAdmissible);
  EXPECT_EQ(r.vertices_replaced, 3);
  EXPECT_FALSE(r.neighbour_restarted);
  EXPECT_NEAR(gm.getNeighbourVertex(2, 2)->state[1], 1.5, 1e-9);
  const StateVec probe(2.0, 1.5, 0.0, 0.0);
  Vertex* nearest = nullptr;
  ASSERT_TRUE(gm.getNearestVertexInRange(&probe, 0.01, &nearest));
  EXPECT_EQ(nearest, gm.getNeighbourVertex(2, 2));
}

TEST(GraphMerge, LinksAreJudgedAgainWhereTheTransformMovedTheRoadmap) {
  GraphManager gm;
  buildOwnGraph(gm);
  StaticPoseSource poses;
  poses.setOffset(2, 0.0, 1.0);
  mergeNeighbourGraph(gm, neighbourGraph(), poses, kAlwaysAdmissible);
  mgg::ShortestPathsReport rep;
  const auto reaches = [&gm, &rep](const Vertex* v) {
    gm.findShortestPaths(0, rep);
    const auto parent = rep.parent_id_map.find(v->id);
    return parent != rep.parent_id_map.end() && parent->second != v->id;
  };
  ASSERT_TRUE(reaches(gm.getNeighbourVertex(2, 2)));

  // The corrected transform puts robot 2's roadmap behind a wall: the old
  // links would now cross it, so none survives, and none is made.
  poses.setOffset(2, 0.0, 3.0);
  auto r = mergeNeighbourGraph(gm, neighbourGraph(), poses, kNeverAdmissible);
  EXPECT_EQ(r.vertices_replaced, 3);
  EXPECT_FALSE(r.merged);
  EXPECT_FALSE(reaches(gm.getNeighbourVertex(2, 2)));

  // Once a link is drivable again the roadmap rejoins without duplicates.
  const int vertices = gm.getNumVertices();
  r = mergeNeighbourGraph(gm, neighbourGraph(), poses, kAlwaysAdmissible);
  EXPECT_TRUE(r.merged);
  EXPECT_EQ(r.vertices_added, 0);
  EXPECT_EQ(gm.getNumVertices(), vertices);
  EXPECT_TRUE(reaches(gm.getNeighbourVertex(2, 2)));
}

/// Adjacency entries over the whole graph, both directions of every edge.
std::size_t adjacencyEntries(const GraphManager& gm) {
  std::size_t entries = 0;
  for (const auto& entry : gm.edge_map_) entries += entry.second.size();
  return entries;
}

TEST(GraphMerge, RepeatedReplacementKeepsOneAdjacencyEntryPerEdge) {
  GraphManager gm;
  buildOwnGraph(gm);
  StaticPoseSource poses;
  poses.setOffset(2, 0.0, 1.0);
  mergeNeighbourGraph(gm, neighbourGraph(), poses, kAlwaysAdmissible);
  const std::size_t entries = adjacencyEntries(gm);
  ASSERT_EQ(entries, 2u * static_cast<std::size_t>(gm.getNumEdges()));
  for (int i = 0; i < 4; ++i) {
    poses.setOffset(2, 0.0, i % 2 == 0 ? 1.5 : 1.0);
    const auto r =
        mergeNeighbourGraph(gm, neighbourGraph(), poses, kAlwaysAdmissible);
    ASSERT_EQ(r.vertices_replaced, 3);
    EXPECT_EQ(adjacencyEntries(gm),
              2u * static_cast<std::size_t>(gm.getNumEdges()));
  }
  EXPECT_LE(adjacencyEntries(gm), entries + 4u);
}

TEST(GraphMerge, ARestartedNeighbourIsCutOutAndMergedAfresh) {
  GraphManager gm;
  buildOwnGraph(gm);
  StaticPoseSource poses;
  poses.setOffset(2, 0.0, 1.0);
  mergeNeighbourGraph(gm, neighbourGraph(), poses, kAlwaysAdmissible);
  const int edges_before = gm.getNumEdges();
  Vertex* old_two = gm.getNeighbourVertex(2, 2);
  ASSERT_NE(old_two, nullptr);

  // Its planner restarted elsewhere: the same ids now name a line along y.
  GraphExchange restarted = neighbourGraph();
  for (auto& v : restarted.vertices) {
    v.state = StateVec(0.0, v.id * 1.0, 0.0, 0.0);
  }
  const auto r =
      mergeNeighbourGraph(gm, restarted, poses, kAlwaysAdmissible);
  EXPECT_TRUE(r.neighbour_restarted);
  EXPECT_TRUE(r.merged);
  EXPECT_TRUE(gm.isRetired(old_two->id));
  // No edge of the old run survives, and the new vertices stand where the
  // new run put them.
  EXPECT_EQ(gm.edge_map_.count(old_two->id), 0u);
  EXPECT_NEAR(gm.getNeighbourVertex(2, 2)->state[1], 3.0, 1e-9);
  EXPECT_NE(gm.getNeighbourVertex(2, 2), old_two);
  EXPECT_LE(gm.getNumEdges(), edges_before + 1);
  // A retired vertex is never a nearest neighbour again.
  const StateVec probe(2.0, 1.0, 0.0, 0.0);
  Vertex* nearest = nullptr;
  ASSERT_TRUE(gm.getNearestVertex(&probe, &nearest));
  EXPECT_NE(nearest, old_two);
}

TEST(GraphMerge, ARestartedNeighboursRootOnlySnapshotRetiresItsOldGraph) {
  GraphManager gm;
  buildOwnGraph(gm);
  StaticPoseSource poses;
  poses.setOffset(2, 0.0, 1.0);
  mergeNeighbourGraph(gm, neighbourGraph(), poses, kAlwaysAdmissible);
  Vertex* old_two = gm.getNeighbourVertex(2, 2);
  ASSERT_NE(old_two, nullptr);

  // Restarted and parked: its snapshot is its new root alone.
  const auto r = mergeNeighbourGraph(gm, neighbourGraph(1), poses,
                                     kAlwaysAdmissible);
  EXPECT_TRUE(r.neighbour_restarted);
  EXPECT_FALSE(r.merged);
  EXPECT_TRUE(gm.isRetired(old_two->id));
  EXPECT_EQ(gm.edge_map_.count(old_two->id), 0u);
}

TEST(GraphMerge, TinyGraphsAreIgnored) {
  GraphManager gm;
  buildOwnGraph(gm, 1);
  StaticPoseSource poses;
  poses.setOffset(2, 0.0, 1.0);
  const auto r = mergeNeighbourGraph(gm, neighbourGraph(), poses,
                                     kAlwaysAdmissible);
  EXPECT_FALSE(r.merged);
}

}  // namespace
