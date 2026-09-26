// Tests for the global graph's frontier clusters and their stable IDs
// (tour-exploration design §2.1).

#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/frontier_clusters.h"

namespace {

using mgg::ClusterId;
using mgg::FrontierCluster;
using mgg::GraphManager;
using mgg::StateVec;
using mgg::Vertex;
using mgg::VertexType;

Vertex* addVertex(GraphManager& graph, double x, double y, VertexType type,
                  double gain, int robot_id = 1) {
  auto* v = new Vertex(graph.generateVertexID(), StateVec(x, y, 0.0, 0.0));
  v->type = type;
  v->robot_id = robot_id;
  v->vol_gain.gain = gain;
  v->vol_gain.is_frontier = type == VertexType::kFrontier;
  graph.addVertex(v);
  return v;
}

TEST(FrontierClusters, GroupsFrontiersAroundTheBestGainWithinTheMergeRadius) {
  GraphManager graph;
  addVertex(graph, 0.0, 0.0, VertexType::kVisited, 0.0);
  Vertex* side = addVertex(graph, 10.0, 0.0, VertexType::kFrontier, 900.0);
  Vertex* best = addVertex(graph, 11.0, 0.0, VertexType::kFrontier, 1000.0);
  Vertex* beyond = addVertex(graph, 13.5, 0.0, VertexType::kFrontier, 700.0);
  Vertex* far = addVertex(graph, 30.0, 0.0, VertexType::kFrontier, 800.0);

  const std::vector<FrontierCluster> clusters =
      mgg::extractFrontierClusters(graph, 2.0, 0.0, 1.0);
  ASSERT_EQ(clusters.size(), 3u);
  // Best gain first; the 13.5 m frontier is 2.5 m from the representative
  // and starts its own cluster.
  EXPECT_EQ(clusters[0].representative_vertex_id, best->id);
  EXPECT_EQ(clusters[0].member_vertex_ids,
            (std::vector<int>{best->id, side->id}));
  EXPECT_DOUBLE_EQ(clusters[0].gain, 1000.0);
  EXPECT_TRUE(clusters[0].position.isApprox(Eigen::Vector3d(11.0, 0.0, 0.0)));
  EXPECT_EQ(clusters[1].representative_vertex_id, far->id);
  EXPECT_EQ(clusters[2].representative_vertex_id, beyond->id);
  EXPECT_EQ(side->cluster_id, best->id);
  EXPECT_EQ(best->cluster_id, best->id);
  EXPECT_EQ(clusters[0].owner_robot_id, 1);
}

TEST(FrontierClusters, DropsLowGainClustersAndVerticesOutOfService) {
  GraphManager graph;
  addVertex(graph, 0.0, 0.0, VertexType::kVisited, 0.0);
  addVertex(graph, 5.0, 0.0, VertexType::kFrontier, 100.0);   // below the gain
  addVertex(graph, 9.0, 0.0, VertexType::kUnvisited, 5000.0);  // not a frontier
  // A quarantined neighbour's frontier: out of service.
  auto* theirs = new Vertex(graph.generateVertexID(),
                            StateVec(20.0, 0.0, 0.0, 0.0));
  theirs->robot_id = 2;
  theirs->type = VertexType::kFrontier;
  theirs->vol_gain.gain = 5000.0;
  graph.addNeighbourVertex(theirs, 77);
  graph.disconnectNeighbourGraph(2);
  Vertex* kept = addVertex(graph, -5.0, 0.0, VertexType::kFrontier, 700.0);

  const std::vector<FrontierCluster> clusters =
      mgg::extractFrontierClusters(graph, 2.0, 600.0, 1.0);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].representative_vertex_id, kept->id);
}

TEST(FrontierClusters, IdsDependOnOwnerAndPositionNotOnVertexIds) {
  GraphManager first;
  addVertex(first, 0.0, 0.0, VertexType::kVisited, 0.0);
  addVertex(first, 10.2, 3.7, VertexType::kFrontier, 900.0);
  GraphManager second;
  addVertex(second, 0.0, 0.0, VertexType::kVisited, 0.0);
  addVertex(second, 1.0, 1.0, VertexType::kVisited, 0.0);
  addVertex(second, 2.0, 1.0, VertexType::kVisited, 0.0);
  addVertex(second, 10.2, 3.7, VertexType::kFrontier, 900.0);

  const auto a = mgg::extractFrontierClusters(first, 2.0, 0.0, 1.0);
  const auto b = mgg::extractFrontierClusters(second, 2.0, 0.0, 1.0);
  ASSERT_EQ(a.size(), 1u);
  ASSERT_EQ(b.size(), 1u);
  EXPECT_NE(a[0].representative_vertex_id, b[0].representative_vertex_id);
  EXPECT_EQ(a[0].id, b[0].id);
  EXPECT_EQ(a[0].id, mgg::makeClusterId(1, Eigen::Vector3d(10.2, 3.7, 0.0), 1.0));
  // Another owner at the same place is another name.
  EXPECT_NE(a[0].id, mgg::makeClusterId(2, Eigen::Vector3d(10.2, 3.7, 0.0), 1.0));
}

TEST(FrontierClusters, RegistryKeepsAnIdWhenTheRepresentativeShifts) {
  GraphManager graph;
  addVertex(graph, 0.0, 0.0, VertexType::kVisited, 0.0);
  Vertex* a = addVertex(graph, 10.2, 0.0, VertexType::kFrontier, 1000.0);
  Vertex* b = addVertex(graph, 11.4, 0.0, VertexType::kFrontier, 900.0);
  mgg::ClusterIdRegistry registry;

  auto before = mgg::extractFrontierClusters(graph, 2.0, 0.0, 1.0);
  registry.stabilize(before, 2.0);
  ASSERT_EQ(before.size(), 1u);

  // The map changed: b now sees more, and represents the cluster from
  // another ID cell. The cluster keeps its name.
  a->vol_gain.gain = 500.0;
  b->vol_gain.gain = 1500.0;
  auto after = mgg::extractFrontierClusters(graph, 2.0, 0.0, 1.0);
  ASSERT_EQ(after.size(), 1u);
  EXPECT_EQ(after[0].representative_vertex_id, b->id);
  EXPECT_NE(after[0].id, before[0].id);  // the raw ID moved with it
  registry.stabilize(after, 2.0);
  EXPECT_EQ(after[0].id, before[0].id);
}

TEST(FrontierClusters, RegistryNamesAClusterFarFromEveryKnownOneAfresh) {
  GraphManager graph;
  addVertex(graph, 0.0, 0.0, VertexType::kVisited, 0.0);
  Vertex* a = addVertex(graph, 10.0, 0.0, VertexType::kFrontier, 1000.0);
  mgg::ClusterIdRegistry registry;
  auto first = mgg::extractFrontierClusters(graph, 2.0, 0.0, 1.0);
  registry.stabilize(first, 2.0);

  // The old frontier was explored; a new one appeared 8 m away.
  a->type = VertexType::kUnvisited;
  addVertex(graph, 18.0, 0.0, VertexType::kFrontier, 1000.0);
  auto second = mgg::extractFrontierClusters(graph, 2.0, 0.0, 1.0);
  registry.stabilize(second, 2.0);
  ASSERT_EQ(second.size(), 1u);
  EXPECT_NE(second[0].id, first[0].id);
  EXPECT_EQ(second[0].id,
            mgg::makeClusterId(1, Eigen::Vector3d(18.0, 0.0, 0.0), 1.0));
}

TEST(FrontierClusters, IdsAreDistinctAndNonZeroInEdgeCases) {
  // Review Focus 3: two representatives in one ID cell (a merge radius
  // below the cell), robot ID 0, and cells either side of zero.
  GraphManager graph;
  addVertex(graph, 50.0, 50.0, VertexType::kVisited, 0.0, 0);
  addVertex(graph, 0.1, 0.1, VertexType::kFrontier, 900.0, 0);
  addVertex(graph, 0.6, 0.1, VertexType::kFrontier, 800.0, 0);
  addVertex(graph, -0.4, 0.1, VertexType::kFrontier, 700.0, 0);
  auto clusters = mgg::extractFrontierClusters(graph, 0.3, 0.0, 1.0);
  ASSERT_EQ(clusters.size(), 3u);
  std::set<ClusterId> ids;
  for (const FrontierCluster& c : clusters) {
    EXPECT_NE(c.id, mgg::kNoCluster);
    ids.insert(c.id);
  }
  EXPECT_EQ(ids.size(), 3u);
  mgg::ClusterIdRegistry registry;
  registry.stabilize(clusters, 0.3);
  ids.clear();
  for (const FrontierCluster& c : clusters) ids.insert(c.id);
  EXPECT_EQ(ids.size(), 3u);
  EXPECT_NE(mgg::makeClusterId(0, Eigen::Vector3d(0.5, 0.0, 0.0), 1.0),
            mgg::makeClusterId(0, Eigen::Vector3d(-0.5, 0.0, 0.0), 1.0));
}

TEST(FrontierClusters, ExploredInGraphNeedsRoadmapAndNoFrontierNearby) {
  GraphManager graph;
  addVertex(graph, 0.0, 0.0, VertexType::kVisited, 0.0);
  addVertex(graph, 1.0, 0.0, VertexType::kUnvisited, 0.0);
  addVertex(graph, 10.0, 0.0, VertexType::kVisited, 0.0);
  addVertex(graph, 10.5, 0.0, VertexType::kFrontier, 900.0);
  // Roadmap nearby and no frontier: explored.
  EXPECT_TRUE(mgg::exploredInGraph(graph, Eigen::Vector3d(0.5, 0.0, 0.0), 2.0));
  // A frontier nearby: not explored.
  EXPECT_FALSE(mgg::exploredInGraph(graph, Eigen::Vector3d(10.0, 0.0, 0.0), 2.0));
  // Nothing nearby: this roadmap cannot tell.
  EXPECT_FALSE(mgg::exploredInGraph(graph, Eigen::Vector3d(30.0, 0.0, 0.0), 2.0));
  GraphManager empty;
  EXPECT_FALSE(mgg::exploredInGraph(empty, Eigen::Vector3d::Zero(), 2.0));
}

}  // namespace

TEST(FrontierClusters, IneligibleVerticesNeitherRepresentNorAbsorbClusters) {
  GraphManager graph;
  auto* ignored = addVertex(graph, 3.0, 0.0, VertexType::kFrontier, 1000.0);
  auto* kept = addVertex(graph, 3.5, 0.0, VertexType::kFrontier, 100.0);
  const auto clusters = mgg::extractFrontierClusters(
      graph, 2.0, 0.0, 1.0, [ignored](const Vertex& v) { return v.id != ignored->id; });
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters.front().representative_vertex_id, kept->id);
  EXPECT_EQ(clusters.front().member_vertex_ids, std::vector<int>{kept->id});
}
