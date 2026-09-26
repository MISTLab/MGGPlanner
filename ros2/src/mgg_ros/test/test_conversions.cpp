// Round-trip tests for the ROS boundary.

#include <cmath>
#include <memory>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "mgg_ros/conversions.h"

namespace {

using mgg::GraphManager;
using mgg::StateVec;
using mgg::Vertex;

TEST(Conversions, PoseRoundTripsIncludingYaw) {
  const StateVec in(1.5, -2.5, 0.75, 1.1);
  const auto msg = mgg_ros::toPoseMsg(in);
  const StateVec out = mgg_ros::fromPoseMsg(msg);
  EXPECT_NEAR(out[0], in[0], 1e-12);
  EXPECT_NEAR(out[1], in[1], 1e-12);
  EXPECT_NEAR(out[2], in[2], 1e-12);
  EXPECT_NEAR(out[3], in[3], 1e-12);
}

TEST(Conversions, YawMatchesTheQuaternionConvention) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0; q.y = 0.0;
  q.z = std::sin(M_PI / 4.0);   // a quarter turn about z
  q.w = std::cos(M_PI / 4.0);
  EXPECT_NEAR(mgg_ros::yawFromQuaternion(q), M_PI / 2.0, 1e-12);
}

TEST(Conversions, TiltIsRollAndPitchWhateverTheYaw) {
  geometry_msgs::msg::Quaternion q;
  // A quarter turn about z: level.
  q.x = 0.0; q.y = 0.0;
  q.z = std::sin(M_PI / 4.0);
  q.w = std::cos(M_PI / 4.0);
  EXPECT_NEAR(mgg_ros::tiltFromQuaternion(q), 0.0, 1e-12);
  // 5 degrees of pitch about y, then a yaw of 60 degrees about z.
  const Eigen::Quaterniond pitched =
      Eigen::AngleAxisd(M_PI / 3.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitY());
  q.x = 2.0 * pitched.x(); q.y = 2.0 * pitched.y();  // not normalised
  q.z = 2.0 * pitched.z(); q.w = 2.0 * pitched.w();
  EXPECT_NEAR(mgg_ros::tiltFromQuaternion(q), 5.0 * M_PI / 180.0, 1e-9);
  q.x = q.y = q.z = q.w = 0.0;
  EXPECT_EQ(mgg_ros::tiltFromQuaternion(q), 0.0);
}

TEST(Conversions, GraphMsgRoundTripsThroughTheExchangeStruct) {
  mgg_msgs::msg::Graph msg;
  for (int i = 0; i < 3; ++i) {
    mgg_msgs::msg::Vertex v;
    v.id = i;
    v.robot_id = 2;
    v.pose = mgg_ros::toPoseMsg(StateVec(i, 0.0, 0.0, 0.0));
    v.num_unknown_voxels = 10 * i;
    v.is_frontier = (i == 2);
    msg.vertices.push_back(v);
  }
  mgg_msgs::msg::Edge e;
  e.source_id = 0; e.target_id = 1; e.weight = 1.0;
  msg.edges.push_back(e);

  const auto ex = mgg_ros::fromGraphMsg(msg);
  ASSERT_EQ(ex.vertices.size(), 3u);
  ASSERT_EQ(ex.edges.size(), 1u);
  EXPECT_EQ(ex.vertices[2].robot_id, 2);
  EXPECT_TRUE(ex.vertices[2].is_frontier);
  EXPECT_EQ(ex.vertices[1].num_unknown_voxels, 10);
  EXPECT_NEAR(ex.vertices[2].state[0], 2.0, 1e-12);
  EXPECT_EQ(ex.edges[0].target_id, 1);
}

TEST(Conversions, OutgoingGraphCarriesOnlyThisRobotsVertices) {
  GraphManager gm;
  gm.setRobotId(1);
  Vertex* prev = nullptr;
  for (int i = 0; i < 3; ++i) {
    auto* v = new Vertex(i, StateVec(i, 0.0, 0.0, 0.0));
    v->robot_id = 1;
    gm.addVertex(v);
    if (prev) gm.addEdge(v, prev, 1.0);
    prev = v;
  }
  // A merged-in vertex from robot 2 must not be broadcast back out.
  auto* theirs = new Vertex(99, StateVec(9.0, 0.0, 0.0, 0.0));
  theirs->robot_id = 2;
  gm.addNeighbourVertex(theirs, 57);

  const auto msg = mgg_ros::toGraphMsg(gm, 1);
  EXPECT_EQ(msg.vertices.size(), 3u);
  for (const auto& v : msg.vertices) EXPECT_EQ(v.robot_id, 1);
  EXPECT_EQ(msg.edges.size(), 2u);
}

// The emitted ids are a subset of the sender's id space once a neighbour
// graph has been merged, so a receiver must not assume they are contiguous.
// This is also what made the ROS 1 reset() loop do O(largest id) work.
TEST(Conversions, EmittedIdsNeedNotBeContiguous) {
  GraphManager gm;
  gm.setRobotId(1);
  auto* a = new Vertex(0, StateVec(0, 0, 0, 0)); a->robot_id = 1;
  gm.addVertex(a);
  auto* theirs = new Vertex(1, StateVec(5, 0, 0, 0)); theirs->robot_id = 2;
  gm.addNeighbourVertex(theirs, 1);
  auto* b = new Vertex(2, StateVec(1, 0, 0, 0)); b->robot_id = 1;
  gm.addVertex(b);

  const auto msg = mgg_ros::toGraphMsg(gm, 1);
  ASSERT_EQ(msg.vertices.size(), 2u);
  std::vector<int> ids;
  for (const auto& v : msg.vertices) ids.push_back(v.id);
  std::sort(ids.begin(), ids.end());
  EXPECT_EQ(ids.front(), 0);
  EXPECT_EQ(ids.back(), 2);  // 1 belongs to robot 2 and is skipped
}

}  // namespace
