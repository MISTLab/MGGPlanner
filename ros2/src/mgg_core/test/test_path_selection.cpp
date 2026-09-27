// Tests for path scoring, the step that turns per-vertex gain into a decision.

#include <algorithm>
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

TEST(PathSelection, PeerEndpointExclusionChoosesAnotherBranch) {
  Fork f;
  for (Vertex* v : f.x_branch) v->vol_gain.gain = 100.0;
  for (Vertex* v : f.y_branch) v->vol_gain.gain = 10.0;
  EdgeInclinations flat;

  const std::vector<Eigen::Vector3d> exclusions{
      Eigen::Vector3d(3.0, 0.0, 0.0)};
  const auto r = mgg::selectBestPath(f.graph, makePlanning(), RobotParams(),
                                     flat, 0.2, 0.0, exclusions, 0.5);
  EXPECT_EQ(r.best_path_id, 6);
  EXPECT_EQ(r.leaves_evaluated, 1);
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

/// Root at the origin with a +x branch and a -x branch, three vertices each.
struct Line {
  Line() {
    auto* root = new Vertex(0, StateVec(0, 0, 0, 0));
    graph.addVertex(root);
    Vertex* prev = root;
    for (int i = 1; i <= 3; ++i) {           // +x branch, ids 1..3
      auto* v = new Vertex(i, StateVec(i * 1.0, 0.0, 0.0, 0.0));
      graph.addVertex(v); graph.addEdge(v, prev, 1.0); prev = v;
      ahead.push_back(v);
    }
    prev = root;
    for (int i = 4; i <= 6; ++i) {           // -x branch, ids 4..6
      auto* v = new Vertex(i, StateVec(-(i - 3) * 1.0, 0.0, 0.0, 0.0));
      graph.addVertex(v); graph.addEdge(v, prev, 1.0); prev = v;
      behind.push_back(v);
    }
  }
  GraphManager graph;
  std::vector<Vertex*> ahead, behind;
};

// Run 8, robot_3 at its 00:36 restart facing west: the unexplored east had
// 2.5 to 8.8 times the gain of the explored west, but exp(-deviation)
// multiplied the reverse path by 0.007 and the robot drove 30 m back into
// the hangar. The factor is bounded below by path_direction_min_factor.
TEST(PathSelection, DirectionPenaltyNeverDiscountsBelowTheMinimumFactor) {
  PlanningParams p = makePlanning();
  p.path_direction_penalty = 1.0;
  EdgeInclinations flat;

  // 8.8 times the gain behind: the path back wins.
  Line l;
  for (Vertex* v : l.ahead) v->vol_gain.gain = 100.0;
  for (Vertex* v : l.behind) v->vol_gain.gain = 880.0;
  const auto back = mgg::selectBestPath(l.graph, p, RobotParams(), flat, 0.2, 0.0);
  EXPECT_EQ(back.best_path_id, 6);
  EXPECT_NEAR(back.best_gain, 3 * 880.0 * p.path_direction_min_factor, 1e-6);

  // One and a half times the gain behind: not enough, the robot goes on
  // forward.
  Line m;
  for (Vertex* v : m.ahead) v->vol_gain.gain = 100.0;
  for (Vertex* v : m.behind) v->vol_gain.gain = 150.0;
  const auto on = mgg::selectBestPath(m.graph, p, RobotParams(), flat, 0.2, 0.0);
  EXPECT_EQ(on.best_path_id, 3);

  // A minimum factor of 0 is the unbounded upstream penalty.
  p.path_direction_min_factor = 0.0;
  Line n;
  for (Vertex* v : n.ahead) v->vol_gain.gain = 100.0;
  for (Vertex* v : n.behind) v->vol_gain.gain = 880.0;
  const auto unbounded = mgg::selectBestPath(n.graph, p, RobotParams(), flat, 0.2, 0.0);
  EXPECT_EQ(unbounded.best_path_id, 3);
}

TEST(PathSelection, APathEndingBehindTurnsBackAndOneAheadOrAsideDoesNot) {
  PlanningParams p = makePlanning();
  p.path_direction_penalty = 1.0;
  const auto line = [](double angle) {
    std::vector<Eigen::Vector3d> path;
    for (int i = 0; i <= 10; ++i) {
      path.emplace_back(0.3 * i * std::cos(angle), 0.3 * i * std::sin(angle),
                        0.0);
    }
    return path;
  };
  EXPECT_FALSE(mgg::pathTurnsBack(line(0.0), 0.0, p));
  EXPECT_FALSE(mgg::pathTurnsBack(line(M_PI / 4.0), 0.0, p));
  EXPECT_FALSE(mgg::pathTurnsBack(line(-M_PI / 2.0 + 0.01), 0.0, p));
  EXPECT_TRUE(mgg::pathTurnsBack(line(3.0 * M_PI / 4.0), 0.0, p));
  EXPECT_TRUE(mgg::pathTurnsBack(line(M_PI), 0.0, p));
  EXPECT_NEAR(mgg::pathDirectionFactor(line(M_PI), 0.0, p, true),
              p.path_direction_min_factor, 1e-12);
  EXPECT_LT(mgg::pathDirectionFactor(line(M_PI), 0.0, p, false), 0.05);
  // Without a bound nothing is said to turn back.
  p.path_direction_min_factor = 0.0;
  EXPECT_FALSE(mgg::pathTurnsBack(line(M_PI), 0.0, p));
}

// The ping-pong the hysteresis stops (run-8 replays: on an unchanging map a
// robot went back and forth between two viewpoints 2.8 m apart). The robot
// faces +x with 8.8 times the gain behind it and turns back. At the end of
// that path, facing -x, the side it came from still has 2.5 times the gain
// of the side ahead: with the bound alone it would turn back again; right
// after turning back it goes on.
TEST(PathSelection, AfterTurningBackTheRobotDoesNotTurnStraightBackAgain) {
  PlanningParams p = makePlanning();
  p.path_direction_penalty = 1.0;
  EdgeInclinations flat;
  mgg::TurnBackHysteresis hysteresis;
  const auto points = [](const std::vector<Vertex*>& path) {
    std::vector<Eigen::Vector3d> out;
    for (const Vertex* v : path) out.push_back(v->state.head<3>());
    return out;
  };

  Line first;
  for (Vertex* v : first.ahead) v->vol_gain.gain = 100.0;
  for (Vertex* v : first.behind) v->vol_gain.gain = 880.0;
  auto r = mgg::selectBestPath(first.graph, hysteresis.selectionParams(p),
                               RobotParams(), flat, 0.2, 0.0);
  ASSERT_EQ(r.best_path_id, 6);
  hysteresis.record(points(r.best_path), 0.0, p);
  EXPECT_TRUE(hysteresis.lastTurnedBack());

  // Facing -x now: the -x branch is ahead (100), the +x branch behind (250).
  Line second;
  for (Vertex* v : second.ahead) v->vol_gain.gain = 250.0;
  for (Vertex* v : second.behind) v->vol_gain.gain = 100.0;
  const auto bound_only =
      mgg::selectBestPath(second.graph, p, RobotParams(), flat, 0.2, M_PI);
  EXPECT_EQ(bound_only.best_path_id, 3);  // back again, without hysteresis
  Line third;
  for (Vertex* v : third.ahead) v->vol_gain.gain = 250.0;
  for (Vertex* v : third.behind) v->vol_gain.gain = 100.0;
  r = mgg::selectBestPath(third.graph, hysteresis.selectionParams(p),
                          RobotParams(), flat, 0.2, M_PI);
  EXPECT_EQ(r.best_path_id, 6);  // goes on
  hysteresis.record(points(r.best_path), M_PI, p);
  EXPECT_FALSE(hysteresis.lastTurnedBack());
  // Going on, the bound applies again.
  EXPECT_DOUBLE_EQ(hysteresis.selectionParams(p).path_direction_min_factor,
                   p.path_direction_min_factor);
}

// Review r1, R1-1: a lattice path the final check would refuse (one that
// turns back into a no-go zone) was chosen, dropped, and the robot got no
// path at all. selectBestPath's path_admissible leaves such paths out, so
// another is chosen.
TEST(PathSelection, AnInadmissiblePathIsLeftOutAndAnotherChosen) {
  Fork f;
  for (Vertex* v : f.x_branch) v->vol_gain.gain = 100.0;
  for (Vertex* v : f.y_branch) v->vol_gain.gain = 10.0;
  EdgeInclinations flat;
  const mgg::PathTurnsFn not_through_3 = [](const std::vector<Vertex*>& path) {
    return std::none_of(path.begin(), path.end(),
                        [](const Vertex* v) { return v->id == 3; });
  };
  const auto r = mgg::selectBestPath(f.graph, makePlanning(), RobotParams(),
                                     flat, 0.2, 0.0, {}, 0.0, nullptr, nullptr,
                                     nullptr, 0.0, {}, nullptr, not_through_3);
  ASSERT_FALSE(r.best_path.empty());
  for (const Vertex* v : r.best_path) EXPECT_NE(v->id, 3);
}

// Review r1, R1-3: an automatic global repositioning's end, on a slope
// where the robot cannot turn, needs a way back as a lattice path's does:
// room to turn within kDepartureMaxM back along the route. The route is cut
// back to its last end with one, or refused.
TEST(PathSelection, ARouteIsCutBackToItsLastEndWithAWayBack) {
  std::vector<mgg::StateVec> route;
  for (int i = 0; i <= 10; ++i) route.emplace_back(0.5 * i, 0.0, 0.0, 0.0);
  // On the slope from x = 1.5; room to turn up to x = 0.5.
  const auto on_slope = [&route](std::size_t i) { return route[i].x() >= 1.5; };
  const auto room = [&route](std::size_t i) { return route[i].x() <= 0.5; };
  std::vector<mgg::StateVec> cut = route;
  ASSERT_TRUE(mgg::cutBackToWayBack(cut, on_slope, room));
  // The last end within 2 m of x = 0.5 is x = 2.5.
  EXPECT_NEAR(cut.back().x(), 2.5, 1e-9);
  // No room anywhere: an end on the slope has no way back; the last end
  // off it (x = 1.0) is kept.
  cut = route;
  ASSERT_TRUE(mgg::cutBackToWayBack(cut, on_slope,
                                    [](std::size_t) { return false; }));
  EXPECT_NEAR(cut.back().x(), 1.0, 1e-9);
  // Every pose after the first on the slope, and no room: refused, and the
  // route left as it was.
  cut = route;
  EXPECT_FALSE(mgg::cutBackToWayBack(
      cut, [&route](std::size_t i) { return route[i].x() > 0.1; },
      [](std::size_t) { return false; }));
  EXPECT_EQ(cut.size(), route.size());
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

TEST(PathSelection, ViewpointWithoutClearanceIsPulledBackAlongItsPath) {
  Fork f;
  for (Vertex* v : f.x_branch) v->vol_gain.gain = 100.0;
  for (Vertex* v : f.y_branch) v->vol_gain.gain = 1.0;
  EdgeInclinations flat;
  // The +x leaf stands against a wall; the vertex before it has room.
  const mgg::ViewpointClearFn clear = [](const Vertex& v) {
    return v.state.x() < 2.5;
  };
  const auto r = mgg::selectBestPath(f.graph, makePlanning(), RobotParams(),
                                     flat, 0.2, 0.0, {}, 0.0, clear);
  EXPECT_EQ(r.best_path_id, 2);
  ASSERT_EQ(r.best_path.size(), 3u);
  EXPECT_EQ(r.best_path.back()->id, 2);
  // Scored up to where it now ends.
  EXPECT_DOUBLE_EQ(r.best_gain, 200.0);
  EXPECT_EQ(r.paths_pulled_back, 1);
  EXPECT_EQ(r.paths_without_clear_viewpoint, 0);
  EXPECT_FALSE(r.unclear_viewpoint);
}

TEST(PathSelection, ZeroGainClearPrefixWinsOverAnUnclearEndpoint) {
  // With leaf-only gain the vertex a path is pulled back to carries none of
  // its own; the pull-back must still happen (review r0, P1).
  GraphManager graph;
  auto* root = new Vertex(0, StateVec(0, 0, 0, 0));
  graph.addVertex(root);
  auto* inner = new Vertex(1, StateVec(1.0, 0.0, 0.0, 0.0));
  graph.addVertex(inner);
  graph.addEdge(inner, root, 1.0);
  auto* leaf = new Vertex(2, StateVec(2.0, 0.0, 0.0, 0.0));
  leaf->vol_gain.gain = 100.0;
  graph.addVertex(leaf);
  graph.addEdge(leaf, inner, 1.0);
  EdgeInclinations flat;
  const mgg::ViewpointClearFn clear = [](const Vertex& v) {
    return v.id != 2;
  };
  const auto r = mgg::selectBestPath(graph, makePlanning(), RobotParams(),
                                     flat, 0.2, 0.0, {}, 0.0, clear);
  EXPECT_EQ(r.best_path_id, 1);
  ASSERT_EQ(r.best_path.size(), 2u);
  EXPECT_EQ(r.best_path.back(), inner);
  EXPECT_EQ(r.paths_pulled_back, 1);
  EXPECT_FALSE(r.unclear_viewpoint);
}

TEST(PathGoesNowhere, APulledBackPathLeadingToGainIsStillSent) {
  // The zero-gain-prefix fixture: with leaf-only gain the path pulled back
  // to its clear vertex scores nothing itself, but it leads to the leaf's
  // gain, 1 m out. It goes somewhere.
  GraphManager graph;
  auto* root = new Vertex(0, StateVec(0, 0, 0, 0));
  graph.addVertex(root);
  auto* inner = new Vertex(1, StateVec(1.0, 0.0, 0.0, 0.0));
  graph.addVertex(inner);
  graph.addEdge(inner, root, 1.0);
  auto* leaf = new Vertex(2, StateVec(2.0, 0.0, 0.0, 0.0));
  leaf->vol_gain.gain = 100.0;
  graph.addVertex(leaf);
  graph.addEdge(leaf, inner, 1.0);
  EdgeInclinations flat;
  const mgg::ViewpointClearFn clear = [](const Vertex& v) {
    return v.id != 2;
  };
  const auto r = mgg::selectBestPath(graph, makePlanning(), RobotParams(),
                                     flat, 0.2, 0.0, {}, 0.0, clear);
  ASSERT_EQ(r.best_path_id, 1);
  EXPECT_DOUBLE_EQ(r.best_gain, 0.0);
  EXPECT_DOUBLE_EQ(r.best_full_gain, 100.0);
  EXPECT_FALSE(mgg::pathGoesNowhere(r, root->state.head<3>(), 0.3));
  // Within the controller's goal tolerance of the robot, it does not.
  EXPECT_TRUE(mgg::pathGoesNowhere(r, Eigen::Vector3d(0.8, 0.0, 0.0), 0.3));

  // Review r2 (P1): the leaf is clear too, but ends on a slope with no way
  // back (SlopeEndRetreat), and the robot has room to turn nowhere. The
  // clear inner vertex, with no gain of its own, still wins because its
  // path leads to the leaf's gain, as it did before the way back was
  // asked of the fallback.
  mgg::SlopeEndRetreat retreat;
  retreat.admitted_on_slope = [](const Vertex& v) { return v.id == 2; };
  retreat.room_to_turn = [](const Vertex&) { return false; };
  const auto refused = mgg::selectBestPath(
      graph, makePlanning(), RobotParams(), flat, 0.2, 0.0, {}, 0.0,
      [](const Vertex&) { return true; }, nullptr, nullptr, 0.0, retreat);
  ASSERT_EQ(refused.best_path_id, 1);
  EXPECT_FALSE(refused.unclear_viewpoint);
  EXPECT_DOUBLE_EQ(refused.best_gain, 0.0);
  EXPECT_DOUBLE_EQ(refused.best_full_gain, 100.0);
  EXPECT_EQ(refused.slope_ends_without_way_back, 1);
  EXPECT_FALSE(mgg::pathGoesNowhere(refused, root->state.head<3>(), 0.3));
}

TEST(PathSelection, ANarrowTargetWithAWayBackBeatsAClearPrefix) {
  GraphManager graph;
  for (int i = 0; i < 4; ++i) {
    auto* v = new Vertex(i, StateVec(i * 0.5, 0, 0, 0));
    v->vol_gain.gain = i == 3 ? 100 : 0;
    graph.addVertex(v);
    if (i) graph.addEdge(v, graph.getVertex(i - 1), 0.5);
  }
  mgg::SlopeEndRetreat retreat;
  retreat.admitted_on_slope = [](const Vertex&) { return false; };
  retreat.room_to_turn = [](const Vertex& v) { return v.id <= 1; };
  PlanningParams planning = makePlanning();
  planning.departure_reverse_allowed = true;
  const auto select = [&]() {
    return mgg::selectBestPath(graph, planning, RobotParams(),
        EdgeInclinations(), 0.2, 0, {}, 0,
        [](const Vertex& v) { return v.id <= 1; }, {}, {}, 0, retreat);
  };
  EXPECT_EQ(select().best_path_id, 3);
  planning.departure_reverse_allowed = false;
  EXPECT_EQ(select().best_path_id, 1);
  planning.departure_reverse_allowed = true;
  retreat.room_to_turn = [](const Vertex&) { return false; };
  EXPECT_EQ(select().best_path_id, 1);  // never the narrow end without a way back
}

TEST(PathSelection, NoRoomAnywhereMeansNoNarrowFallbackPath) {
  Fork f;
  f.x_branch.back()->vol_gain.gain = 100.0;
  mgg::SlopeEndRetreat retreat;
  retreat.admitted_on_slope = [](const Vertex&) { return false; };
  retreat.room_to_turn = [](const Vertex&) { return false; };
  const auto result = mgg::selectBestPath(
      f.graph, makePlanning(), RobotParams(), EdgeInclinations(), 0.2, 0,
      {}, 0, [](const Vertex&) { return false; }, {}, {}, 0, retreat);
  EXPECT_TRUE(result.best_path.empty());
  EXPECT_EQ(result.best_path_id, -1);
  EXPECT_GT(result.slope_ends_without_way_back, 0);
}

TEST(PathSelection, AFallbackCutBackForItsWayBackNeverEndsInAReservation) {
  // No end is clear. The leaf has no bounded way back, but the inner
  // vertex can reverse to turning space at the root and holds the gain.
  // If reserved, that cut-back end must still be refused.
  GraphManager graph;
  auto* root = new Vertex(0, StateVec(0, 0, 0, 0));
  graph.addVertex(root);
  auto* inner = new Vertex(1, StateVec(1.0, 0.0, 0.0, 0.0));
  inner->vol_gain.gain = 50.0;
  graph.addVertex(inner);
  graph.addEdge(inner, root, 1.0);
  auto* leaf = new Vertex(2, StateVec(2.5, 0.0, 0.0, 0.0));
  graph.addVertex(leaf);
  graph.addEdge(leaf, inner, 1.5);
  EdgeInclinations flat;
  const mgg::ViewpointClearFn nowhere_clear = [](const Vertex&) {
    return false;
  };
  mgg::SlopeEndRetreat retreat;
  retreat.admitted_on_slope = [](const Vertex& v) { return v.id == 2; };
  retreat.room_to_turn = [](const Vertex& v) { return v.id == 0; };
  // Unreserved, the cut path ends at the inner vertex with a way back.
  const auto free = mgg::selectBestPath(
      graph, makePlanning(), RobotParams(), flat, 0.2, 0.0, {}, 0.0,
      nowhere_clear, nullptr, nullptr, 0.0, retreat);
  ASSERT_EQ(free.best_path_id, 1);
  EXPECT_FALSE(free.unclear_viewpoint);
  EXPECT_DOUBLE_EQ(free.best_gain, 50.0);
  // Reserved, there is no path.
  const auto reserved = mgg::selectBestPath(
      graph, makePlanning(), RobotParams(), flat, 0.2, 0.0,
      {Eigen::Vector3d(1.0, 0.0, 0.0)}, 0.3, nowhere_clear, nullptr, nullptr,
      0.0, retreat);
  EXPECT_TRUE(reserved.best_path.empty())
      << "ends at vertex " << reserved.best_path_id;
  EXPECT_EQ(reserved.best_path_id, -1);
  EXPECT_FALSE(reserved.unclear_viewpoint);
}

TEST(PathSelection, AClearEndLeadingToNoGainIsNoClearEnd) {
  // Run 5, robot_3: the +x branch's gain lies beyond ends without
  // clearance, and the +y branch, clear, sees nothing. Clearance used to
  // win the selection with a path to no gain. That clear end goes nowhere
  // and is no clear end (review r0, I-3): no path ends clear, and the +x
  // branch is taken unclear.
  Fork f;
  f.x_branch.back()->vol_gain.gain = 100.0;
  EdgeInclinations flat;
  const mgg::ViewpointClearFn clear = [](const Vertex& v) {
    return v.state.x() < 0.5;
  };
  const auto r = mgg::selectBestPath(f.graph, makePlanning(), RobotParams(),
                                     flat, 0.2, 0.0, {}, 0.0, clear);
  EXPECT_EQ(r.best_path_id, 3);
  EXPECT_TRUE(r.unclear_viewpoint);
  EXPECT_DOUBLE_EQ(r.best_full_gain, 100.0);
  EXPECT_FALSE(mgg::pathGoesNowhere(
      r, f.graph.getVertex(0)->state.head<3>(), 0.3));
}

TEST(PathSelection, AClearEndWithinTheGoalToleranceIsNoClearEnd) {
  // The robot at the mouth of a passage too narrow to end a path in: the
  // passage's path is pulled back to the one clear vertex, 0.25 m from the
  // robot, within the controller's goal tolerance. That end goes nowhere;
  // no path ends clear, and the path into the passage is taken unclear.
  GraphManager graph;
  auto* root = new Vertex(0, StateVec(0, 0, 0, 0));
  graph.addVertex(root);
  auto* mouth = new Vertex(1, StateVec(0.25, 0.0, 0.0, 0.0));
  graph.addVertex(mouth);
  graph.addEdge(mouth, root, 0.25);
  Vertex* prev = mouth;
  for (int i = 2; i <= 5; ++i) {
    auto* v = new Vertex(i, StateVec(0.25 + 0.5 * (i - 1), 0.0, 0.0, 0.0));
    graph.addVertex(v);
    graph.addEdge(v, prev, 0.5);
    prev = v;
  }
  prev->vol_gain.gain = 100.0;
  EdgeInclinations flat;
  const mgg::ViewpointClearFn clear = [](const Vertex& v) {
    return v.id <= 1;
  };
  const auto within = mgg::selectBestPath(
      graph, makePlanning(), RobotParams(), flat, 0.2, 0.0, {}, 0.0, clear,
      nullptr, nullptr, 0.3);
  EXPECT_EQ(within.best_path_id, 5);
  EXPECT_TRUE(within.unclear_viewpoint);
  // Farther from the mouth than the tolerance, the robot is sent to it.
  const auto beyond = mgg::selectBestPath(
      graph, makePlanning(), RobotParams(), flat, 0.2, 0.0, {}, 0.0, clear,
      nullptr, nullptr, 0.2);
  EXPECT_EQ(beyond.best_path_id, 1);
  EXPECT_FALSE(beyond.unclear_viewpoint);
}

TEST(PathGoesNowhere, AShortPathWithGainGoesNowhereWithinTheGoalTolerance) {
  // A 2-pose path 0.25 m long: the controller has reached its end before
  // it starts (PCI reach_distance 0.3 m).
  GraphManager graph;
  auto* root = new Vertex(0, StateVec(0, 0, 0, 0));
  graph.addVertex(root);
  auto* near = new Vertex(1, StateVec(0.25, 0.0, 0.0, 0.0));
  near->vol_gain.gain = 10.0;
  graph.addVertex(near);
  graph.addEdge(near, root, 0.25);
  EdgeInclinations flat;
  const auto r = mgg::selectBestPath(graph, makePlanning(), RobotParams(),
                                     flat, 0.2, 0.0);
  ASSERT_EQ(r.best_path.size(), 2u);
  EXPECT_TRUE(mgg::pathGoesNowhere(r, root->state.head<3>(), 0.3));
  EXPECT_FALSE(mgg::pathGoesNowhere(r, root->state.head<3>(), 0.2));
  EXPECT_FALSE(mgg::pathGoesNowhere(mgg::PathSelectionResult(),
                                    root->state.head<3>(), 0.3));
}

TEST(PathSelection, ReservedGainIsNotPursuedThroughAClearPrefix) {
  // The zero-gain-prefix fixture with its leaf reserved by a peer: the
  // branch's only gain lies in the reservation, so no path at all (controller
  // ruling on review r0, fix round 1), as before the clearance check.
  GraphManager graph;
  auto* root = new Vertex(0, StateVec(0, 0, 0, 0));
  graph.addVertex(root);
  auto* inner = new Vertex(1, StateVec(1.0, 0.0, 0.0, 0.0));
  graph.addVertex(inner);
  graph.addEdge(inner, root, 1.0);
  auto* leaf = new Vertex(2, StateVec(2.0, 0.0, 0.0, 0.0));
  leaf->vol_gain.gain = 100.0;
  graph.addVertex(leaf);
  graph.addEdge(leaf, inner, 1.0);
  EdgeInclinations flat;
  const std::vector<Eigen::Vector3d> exclusions{
      Eigen::Vector3d(2.0, 0.0, 0.0)};
  const mgg::ViewpointClearFn clear = [](const Vertex& v) {
    return v.id != 2;
  };
  const auto r = mgg::selectBestPath(graph, makePlanning(), RobotParams(),
                                     flat, 0.2, 0.0, exclusions, 0.5, clear);
  EXPECT_EQ(r.best_path_id, -1);
  EXPECT_TRUE(r.best_path.empty());
  EXPECT_FALSE(r.unclear_viewpoint);

  // Nor does that prefix win over another branch as a clear alternative:
  // the other branch is taken as it is, flagged unclear.
  Fork f;
  f.x_branch.back()->vol_gain.gain = 100.0;
  for (Vertex* v : f.y_branch) v->vol_gain.gain = 1.0;
  const std::vector<Eigen::Vector3d> reserved{Eigen::Vector3d(3.0, 0.0, 0.0)};
  const mgg::ViewpointClearFn x_prefix_clear = [](const Vertex& v) {
    return v.id == 1 || v.id == 2;
  };
  const auto other = mgg::selectBestPath(f.graph, makePlanning(),
                                         RobotParams(), flat, 0.2, 0.0,
                                         reserved, 0.5, x_prefix_clear);
  EXPECT_EQ(other.best_path_id, 6);
  EXPECT_TRUE(other.unclear_viewpoint);
}

TEST(PathSelection, WithNoGainAnywhereThereIsStillNoPath) {
  Fork f;
  EdgeInclinations flat;
  const mgg::ViewpointClearFn clear = [](const Vertex&) { return true; };
  const auto r = mgg::selectBestPath(f.graph, makePlanning(), RobotParams(),
                                     flat, 0.2, 0.0, {}, 0.0, clear);
  EXPECT_EQ(r.best_path_id, -1);
  EXPECT_TRUE(r.best_path.empty());
}

TEST(PathSelection, PulledBackEndpointOutsideAReservationIsKept) {
  // A peer has reserved the +x leaf. The branch pulled back to vertex 2 ends
  // outside the reservation and is the only clear path, so it must not be
  // dropped with the leaf (review r0, P1).
  Fork f;
  for (Vertex* v : f.x_branch) v->vol_gain.gain = 100.0;
  for (Vertex* v : f.y_branch) v->vol_gain.gain = 1.0;
  EdgeInclinations flat;
  const std::vector<Eigen::Vector3d> exclusions{
      Eigen::Vector3d(3.0, 0.0, 0.0)};
  const mgg::ViewpointClearFn clear = [](const Vertex& v) {
    return v.id == 2;
  };
  const auto r = mgg::selectBestPath(f.graph, makePlanning(), RobotParams(),
                                     flat, 0.2, 0.0, exclusions, 0.5, clear);
  EXPECT_EQ(r.best_path_id, 2);
  EXPECT_FALSE(r.unclear_viewpoint);
  EXPECT_EQ(r.paths_pulled_back, 1);
  // The reserved leaf itself stays out, as without the clearance check.
  EXPECT_EQ(r.leaves_evaluated, 1);
}

TEST(PathSelection, PathEndingClearWinsOverRicherPathsThatDoNot) {
  Fork f;
  for (Vertex* v : f.x_branch) v->vol_gain.gain = 100.0;
  for (Vertex* v : f.y_branch) v->vol_gain.gain = 1.0;
  EdgeInclinations flat;
  // The whole +x branch runs along a wall. The root is not asked: the robot
  // already stands there.
  const mgg::ViewpointClearFn clear = [](const Vertex& v) {
    EXPECT_NE(v.id, 0);
    return v.state.x() < 0.5;
  };
  const auto r = mgg::selectBestPath(f.graph, makePlanning(), RobotParams(),
                                     flat, 0.2, 0.0, {}, 0.0, clear);
  EXPECT_EQ(r.best_path_id, 6);
  EXPECT_DOUBLE_EQ(r.best_gain, 3.0);
  EXPECT_EQ(r.paths_without_clear_viewpoint, 1);
  EXPECT_EQ(r.paths_pulled_back, 0);
  EXPECT_EQ(r.leaves_evaluated, 2);
  EXPECT_FALSE(r.unclear_viewpoint);
}

TEST(PathSelection, WithNoPathEndingClearTheBestPathIsStillChosen) {
  // A passage narrower than the robot plus twice the margin: clearance must
  // not stop exploration where it went on before.
  Fork f;
  for (Vertex* v : f.x_branch) v->vol_gain.gain = 100.0;
  for (Vertex* v : f.y_branch) v->vol_gain.gain = 1.0;
  EdgeInclinations flat;
  const mgg::ViewpointClearFn nowhere = [](const Vertex&) { return false; };
  const auto r = mgg::selectBestPath(f.graph, makePlanning(), RobotParams(),
                                     flat, 0.2, 0.0, {}, 0.0, nowhere);
  EXPECT_EQ(r.best_path_id, 3);
  EXPECT_EQ(r.best_path.size(), 4u);
  EXPECT_DOUBLE_EQ(r.best_gain, 300.0);
  EXPECT_EQ(r.paths_without_clear_viewpoint, 2);
  EXPECT_TRUE(r.unclear_viewpoint);
}

TEST(PathSelection, PathThatTurnsOnlyWhereItMayOutranksRicherOnes) {
  Fork f;
  for (Vertex* v : f.x_branch) v->vol_gain.gain = 100.0;
  for (Vertex* v : f.y_branch) v->vol_gain.gain = 1.0;
  EdgeInclinations flat;
  // The +x branch turns sharply on a slope; the +y branch ends against a
  // wall. Turning outranks clearance.
  const mgg::PathTurnsFn turns = [](const std::vector<Vertex*>& path) {
    return path.back()->state.x() < 0.5;
  };
  const mgg::ViewpointClearFn clear = [](const Vertex& v) {
    return v.state.y() < 0.5;
  };
  const auto r = mgg::selectBestPath(f.graph, makePlanning(), RobotParams(),
                                     flat, 0.2, 0.0, {}, 0.0, clear, turns);
  EXPECT_EQ(r.best_path_id, 6);
  EXPECT_TRUE(r.unclear_viewpoint);
  EXPECT_FALSE(r.sharp_turn_fallback);
  EXPECT_EQ(r.paths_with_sharp_turns, 1);
}

TEST(PathSelection, WithNoPathTurningWhereItMayTheBestPathIsStillChosen) {
  Fork f;
  for (Vertex* v : f.x_branch) v->vol_gain.gain = 100.0;
  for (Vertex* v : f.y_branch) v->vol_gain.gain = 1.0;
  EdgeInclinations flat;
  const mgg::PathTurnsFn nowhere = [](const std::vector<Vertex*>&) {
    return false;
  };
  const auto r = mgg::selectBestPath(f.graph, makePlanning(), RobotParams(),
                                     flat, 0.2, 0.0, {}, 0.0, nullptr,
                                     nowhere);
  EXPECT_EQ(r.best_path_id, 3);
  EXPECT_DOUBLE_EQ(r.best_gain, 300.0);
  EXPECT_TRUE(r.sharp_turn_fallback);
  EXPECT_EQ(r.paths_with_sharp_turns, 2);
  EXPECT_EQ(r.leaves_evaluated, 2);
}

TEST(PullBackToClearViewpoint, RouteEndsAtItsLastClearPose) {
  std::vector<StateVec> route;
  for (int i = 0; i <= 4; ++i) route.emplace_back(0.5 * i, 0.0, 0.0, 0.0);
  // The last metre runs beside a wall.
  const auto clear = [](const StateVec& pose) { return pose.x() < 1.2; };
  ASSERT_TRUE(mgg::pullBackToClearViewpoint(route, clear));
  ASSERT_EQ(route.size(), 3u);
  EXPECT_DOUBLE_EQ(route.back().x(), 1.0);

  // Nothing past the start has room: refused, and the route is kept whole
  // for the caller to report.
  std::vector<StateVec> walled = {StateVec(0.0, 0.0, 0.0, 0.0),
                                  StateVec(0.5, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg::pullBackToClearViewpoint(
      walled, [](const StateVec& pose) { return pose.x() < 0.2; }));
  EXPECT_EQ(walled.size(), 2u);
  std::vector<StateVec> empty;
  EXPECT_FALSE(mgg::pullBackToClearViewpoint(empty, clear));
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
