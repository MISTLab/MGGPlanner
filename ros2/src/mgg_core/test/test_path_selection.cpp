// Tests for path scoring, the step that turns per-vertex gain into a decision.

#include <cmath>
#include <gtest/gtest.h>

#include "mgg_core/path_selection.h"

namespace {

using mgg::EdgeInclinations;
using mgg::GraphManager;
using mgg::PlanningParams;
using mgg::RobotParams;
using mgg::RobotType;
using mgg::StateVec;
using mgg::Vertex;

PlanningParams makePlanning() {
  PlanningParams p;
  p.path_length_penalty = 0.0;      // isolate gain unless a test says otherwise
  p.path_direction_penalty = 0.0;
  p.hanging_vertex_penalty = 0.0;
  p.max_negative_inclination = 0.37;
  return p;
}

/// Root at the origin with two branches along +x and +y, three vertices each.
struct Fork {
  Fork() {
    auto* root = new Vertex(0, StateVec(0, 0, 0, 0));
    graph.addVertex(root);
    Vertex* prev = root;
    for (int i = 1; i <= 3; ++i) {           // +x branch, ids 1..3
      auto* v = new Vertex(i, StateVec(i * 1.0, 0.0, 0.0, 0.0));
      graph.addVertex(v); graph.addEdge(v, prev, 1.0); prev = v;
      x_branch.push_back(v);
    }
    prev = root;
    for (int i = 4; i <= 6; ++i) {           // +y branch, ids 4..6
      auto* v = new Vertex(i, StateVec(0.0, (i - 3) * 1.0, 0.0, 0.0));
      graph.addVertex(v); graph.addEdge(v, prev, 1.0); prev = v;
      y_branch.push_back(v);
    }
  }
  GraphManager graph;
  std::vector<Vertex*> x_branch, y_branch;
};

TEST(PathSelection, PicksTheBranchWithMoreGain) {
  Fork f;
  for (Vertex* v : f.x_branch) v->vol_gain.gain = 100.0;
  for (Vertex* v : f.y_branch) v->vol_gain.gain = 1.0;

  EdgeInclinations flat;
  const auto r = mgg::selectBestPath(f.graph, makePlanning(), RobotParams(),
                                     flat, 0.2, 0.0);
  EXPECT_GT(r.best_gain, 0.0);
  EXPECT_EQ(r.best_path_id, 3);            // the far end of the +x branch
  ASSERT_FALSE(r.best_path.empty());
  EXPECT_EQ(r.best_path.front()->id, 0);   // root first
}

TEST(PathSelection, LengthPenaltyDiscountsDistantGain) {
  Fork f;
  // Same total gain, but the +y branch concentrates it near the root.
  for (Vertex* v : f.x_branch) v->vol_gain.gain = 0.0;
  f.x_branch.back()->vol_gain.gain = 300.0;      // 3 m away
  for (Vertex* v : f.y_branch) v->vol_gain.gain = 0.0;
  f.y_branch.front()->vol_gain.gain = 300.0;     // 1 m away

  PlanningParams p = makePlanning();
  p.path_length_penalty = 0.5;
  EdgeInclinations flat;
  const auto r = mgg::selectBestPath(f.graph, p, RobotParams(), flat, 0.2, 0.0);
  // The nearer reward wins once distance is discounted.
  EXPECT_EQ(r.best_path_id, 6);
}

TEST(PathSelection, DirectionPenaltyPrefersTheCurrentHeading) {
  Fork f;
  for (Vertex* v : f.x_branch) v->vol_gain.gain = 100.0;
  for (Vertex* v : f.y_branch) v->vol_gain.gain = 100.0;

  PlanningParams p = makePlanning();
  p.path_direction_penalty = 3.0;
  EdgeInclinations flat;

  // Travelling along +x: the +x branch should win.
  auto r_east = mgg::selectBestPath(f.graph, p, RobotParams(), flat, 0.2, 0.0);
  EXPECT_EQ(r_east.best_path_id, 3);

  // Travelling along +y: the +y branch should win instead. This only
  // discriminates because the reference is a ray along the heading; with the
  // ROS 1 collapsed reference the comparison was against a single point.
  Fork g;
  for (Vertex* v : g.x_branch) v->vol_gain.gain = 100.0;
  for (Vertex* v : g.y_branch) v->vol_gain.gain = 100.0;
  auto r_north = mgg::selectBestPath(g.graph, p, RobotParams(), flat, 0.2,
                                     M_PI / 2.0);
  EXPECT_EQ(r_north.best_path_id, 6);
}

TEST(PathSelection, FrontierPresenceIsReported) {
  Fork f;
  for (Vertex* v : f.x_branch) v->vol_gain.gain = 10.0;
  EdgeInclinations flat;
  auto none = mgg::selectBestPath(f.graph, makePlanning(), RobotParams(), flat,
                                  0.2, 0.0);
  EXPECT_FALSE(none.frontier_exists);

  f.x_branch.back()->vol_gain.is_frontier = true;
  auto some = mgg::selectBestPath(f.graph, makePlanning(), RobotParams(), flat,
                                  0.2, 0.0);
  EXPECT_TRUE(some.frontier_exists);
}

// A ground robot must not be sent down a slope steeper than
// max_negative_inclination.
TEST(PathSelection, SteepDescentIsRejectedForGroundRobots) {
  GraphManager graph;
  auto* root = new Vertex(0, StateVec(0, 0, 0, 0));
  graph.addVertex(root);
  auto* down = new Vertex(1, StateVec(1.0, 0.0, -2.0, 0.0));  // steep drop
  down->vol_gain.gain = 1000.0;
  graph.addVertex(down);
  graph.addEdge(down, root, 2.2);

  RobotParams ground;
  ground.type = RobotType::kGroundRobot;
  EdgeInclinations flat;   // geometry alone should reject it

  const auto r = mgg::selectBestPath(graph, makePlanning(), ground, flat, 0.2,
                                     0.0);
  EXPECT_EQ(r.paths_rejected_steep, 1);
  EXPECT_EQ(r.best_path_id, -1);
}

// REGRESSION for the asymmetric store. The path walk reads
// [further][closer]; an edge recorded the other way round used to read back as
// flat, so a steep neighbour edge escaped the check entirely.
TEST(PathSelection, InclinationIsFoundWhicheverWayTheEdgeWasRecorded) {
  GraphManager graph;
  auto* root = new Vertex(0, StateVec(0, 0, 0, 0));
  graph.addVertex(root);
  // Geometry alone is gentle, so only the recorded inclination can reject it.
  auto* far = new Vertex(1, StateVec(5.0, 0.0, -0.3, 0.0));
  far->vol_gain.gain = 1000.0;
  graph.addVertex(far);
  graph.addEdge(far, root, 5.0);

  RobotParams ground;
  ground.type = RobotType::kGroundRobot;

  EdgeInclinations inclinations;
  // Recorded closer-first, the opposite of how the walk reads it.
  inclinations.set(0, 1, 1.2);           // far above max_negative_inclination
  EXPECT_DOUBLE_EQ(inclinations.get(1, 0), 1.2);

  const auto r = mgg::selectBestPath(graph, makePlanning(), ground,
                                     inclinations, 0.2, 0.0);
  EXPECT_EQ(r.paths_rejected_steep, 1);
}

TEST(PathSelection, EmptyGraphIsHandled) {
  GraphManager graph;
  EdgeInclinations flat;
  const auto r = mgg::selectBestPath(graph, makePlanning(), RobotParams(),
                                     flat, 0.2, 0.0);
  EXPECT_EQ(r.best_path_id, -1);
  EXPECT_EQ(r.leaves_evaluated, 0);
}

}  // namespace
