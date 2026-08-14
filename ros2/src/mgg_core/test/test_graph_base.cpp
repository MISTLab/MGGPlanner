// Tests for the graph value types, focused on the initialisation the ROS 1
// versions left undone.

#include <gtest/gtest.h>

#include "mgg_core/graph_base.h"

namespace {

using mgg::BoundingBoxType;
using mgg::SemanticClass;
using mgg::StateVec;
using mgg::Vertex;
using mgg::VertexType;
using mgg::VolumetricGain;

// REGRESSION: the ROS 1 Vertex constructor assigned twelve fields but not
// robot_id, even though it keys vertex_by_robot_id_ in the multi-robot graph
// merge. A vertex whose robot_id was never set explicitly carried whatever
// was on the heap.
TEST(Vertex, RobotIdIsInitialised) {
  const Vertex v(7, StateVec(1.0, 2.0, 3.0, 0.5));
  EXPECT_EQ(v.robot_id, 0);
  EXPECT_EQ(v.id, 7);
  EXPECT_DOUBLE_EQ(v.state[0], 1.0);
  EXPECT_DOUBLE_EQ(v.state[3], 0.5);
}

TEST(Vertex, DefaultsMatchTheRos1Constructor) {
  const Vertex v(1, StateVec::Zero());
  EXPECT_EQ(v.parent, nullptr);
  EXPECT_DOUBLE_EQ(v.distance, 0.0);
  EXPECT_TRUE(v.is_leaf_vertex);
  EXPECT_FALSE(v.is_hanging);
  EXPECT_EQ(v.type, VertexType::kUnvisited);
  EXPECT_EQ(v.cluster_id, 0);
  EXPECT_EQ(v.pose_id, 0);
  EXPECT_DOUBLE_EQ(v.dm, 0.0);
  EXPECT_EQ(v.semantic_class, SemanticClass::kNone);
}

// REGRESSION: BoundingBoxType's four vectors were undefined until
// setDefault() ran, so reset() before that copied garbage.
TEST(BoundingBoxType, ResetBeforeSetDefaultIsZeroNotGarbage) {
  BoundingBoxType box;
  box.reset();
  EXPECT_TRUE(box.min().isZero());
  EXPECT_TRUE(box.max().isZero());
}

TEST(BoundingBoxType, ResetRestoresTheDefault) {
  BoundingBoxType box;
  box.setDefault(Eigen::Vector3d(-1, -2, -3), Eigen::Vector3d(1, 2, 3));
  box.set(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.5, 0.5, 0.5));
  EXPECT_DOUBLE_EQ(box.max().x(), 0.5);
  box.reset();
  EXPECT_DOUBLE_EQ(box.min().x(), -1.0);
  EXPECT_DOUBLE_EQ(box.max().z(), 3.0);
}

TEST(VolumetricGain, ResetClearsEverythingIncludingTheHashKeys) {
  VolumetricGain g;
  g.gain = 5.0;
  g.num_unknown_voxels = 12;
  g.is_frontier = true;
  g.unseen_voxel_hash_keys.push_back(99);
  g.reset();
  EXPECT_DOUBLE_EQ(g.gain, 0.0);
  EXPECT_EQ(g.num_unknown_voxels, 0);
  EXPECT_FALSE(g.is_frontier);
  EXPECT_TRUE(g.unseen_voxel_hash_keys.empty());
}

TEST(SampleStatistic, TotalTimeSumsTheStages) {
  mgg::SampleStatistic s;
  s.build_graph_time = 1.0;
  s.compute_exp_gain_time = 2.0;
  s.shortest_path_time = 0.5;
  s.evaluate_graph_time = 0.25;
  EXPECT_DOUBLE_EQ(s.totalTime(), 3.75);
  EXPECT_NE(s.formatTimes("local").find("local"), std::string::npos);
}

}  // namespace
