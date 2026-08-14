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

/// Our own graph: a chain along +x at y=0.
void buildOwnGraph(GraphManager& gm, int n = 4) {
  Vertex* prev = nullptr;
  for (int i = 0; i < n; ++i) {
    auto* v = new Vertex(i, StateVec(i * 1.0, 0.0, 0.0, 0.0));
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
