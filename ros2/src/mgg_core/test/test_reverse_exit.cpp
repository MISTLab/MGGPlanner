#include <gtest/gtest.h>

#include "mgg_core/departure.h"
#include "mgg_core/no_go_zones.h"
#include "mgg_core/path_selection.h"
#include "terrain_fixture.h"

namespace {

using Tops = std::map<std::pair<std::int64_t, std::int64_t>, double>;

Tops floor(double from = -2.0, double height = 0.0) {
  Tops tops;
  for (int x = static_cast<int>(from / 0.2); x < 40; ++x) {
    for (int y = -10; y < 10; ++y) tops[{x, y}] = height;
  }
  return tops;
}

mgg::RobotParams robot() {
  mgg::RobotParams r;
  r.type = mgg::RobotType::kGroundRobot;
  r.size = {1.0, 0.4, 0.2};
  r.size_extension.setZero();
  r.size_extension_min.setZero();
  r.safety_extension.setZero();
  r.bound_mode = mgg::BoundModeType::kExactBound;
  return r;
}

mgg::PlanningParams planning() {
  mgg::PlanningParams p;
  p.max_ground_height = 0.5;
  p.max_step_height = 0.1;
  p.max_inclination = 0.7;
  p.max_negative_inclination = 0.3;
  p.min_observed_ground_fraction = 0.75;
  p.max_footprint_cell_rise = 0.12;
  p.path_direction_penalty = 0.0;
  p.path_length_penalty = 0.0;
  return p;
}

TEST(ReverseExit, LongNarrowCorridorNeedsExplicitReverseValidationAndRespectsBound) {
  Tops tops = floor();
  for (int x = 4; x < 40; ++x) {
    tops[{x, 2}] = 0.5;
    tops[{x, -3}] = 0.5;
  }
  mgg_test::TerrainFixture map(0.2, tops);
  auto p = planning();
  const auto r = robot();
  mgg::GroundProjection ground(map, p, true);
  mgg::GraphManager graph;
  for (int i = 0; i <= 10; ++i) {
    auto* v = new mgg::Vertex(i, mgg::StateVec(i * 0.5, 0, 0.5, 0));
    v->vol_gain.gain = i == 10 ? 100 : 0;
    graph.addVertex(v);
    if (i) graph.addEdge(v, graph.getVertex(i - 1), 0.5);
  }
  mgg::SlopeEndRetreat retreat;
  retreat.admitted_on_slope = [](const mgg::Vertex&) { return false; };
  retreat.room_to_turn = [&](const mgg::Vertex& v) {
    return mgg::roomToTurn(map, r, p, v.state);
  };
  const auto clear = [&](const mgg::Vertex& v) {
    return mgg::viewpointClear(map, r, p, v.state, 0.0);
  };
  ASSERT_TRUE(retreat.room_to_turn(*graph.getVertex(0)));
  ASSERT_FALSE(retreat.room_to_turn(*graph.getVertex(10)));
  const auto select = [&] {
    return mgg::selectBestPath(graph, p, r, {}, 0.2, 0, {}, 0, clear,
                                {}, {}, 0.25, retreat);
  };
  EXPECT_NE(select().best_path_id, 10);  // no callback: still the old bound
  int root_checks = 0;
  retreat.reverse_edge_admissible = [&](const mgg::Vertex& a, const mgg::Vertex& b) {
    if (b.id == 0) ++root_checks;
    return mgg::reverseExitEdgeAdmissible(map, ground, r, p, a.state, b.state);
  };
  EXPECT_EQ(select().best_path_id, 10);
  EXPECT_GT(root_checks, 0);  // the forward-only root edge is checked too
  p.reverse_exit_max_length = 3.0;
  EXPECT_NE(select().best_path_id, 10);
  p.reverse_exit_max_length = 6.0;
  p.departure_reverse_allowed = false;
  EXPECT_NE(select().best_path_id, 10);
}

TEST(ReverseExit, AForwardOnlyRootWithBlindOrDropBackedSupportIsRefused) {
  auto p = planning();
  // Isolate the directional observed-support veto (the cell-rise check
  // would independently reject the mapped drop in either direction).
  p.max_footprint_cell_rise = 0.0;
  const auto r = robot();
  for (const bool drop : {false, true}) {
    SCOPED_TRACE(drop);
    Tops tops = floor(0.0);
    if (drop) {
      for (int x = -10; x < 0; ++x) {
        for (int y = -10; y < 10; ++y) tops[{x, y}] = -2.0;
      }
    }
    mgg_test::TerrainFixture map(0.2, tops);
    mgg::GroundProjection ground(map, p);
    const mgg::StateVec root(0.1, 0.1, 0.5, 0), end(0.7, 0.1, 0.5, 0);
    std::vector<Eigen::Vector3d> projected;
    ASSERT_EQ(ground.getProjectedEdgeStatus(root.head<3>(), end.head<3>(),
        r.getPlanningSize(), false, projected, false, false, nullptr,
        mgg::EdgeTravel::kForward), mgg::ProjectedEdgeStatus::kAdmissible);
    EXPECT_FALSE(mgg::reverseExitEdgeAdmissible(map, ground, r, p, end, root));
    ground.setStandingStart(mgg::StandingStart{{0, 0}, 2.0});
    EXPECT_FALSE(mgg::reverseExitEdgeAdmissible(map, ground, r, p, end, root));
  }
}

TEST(ReverseExit, OneWayAscentCannotBePromisedAsAReverseDescent) {
  auto p = planning();
  p.max_negative_inclination = 0.1;
  const auto r = robot();
  Tops tops = floor();
  for (auto& [cell, top] : tops) top = 0.4 * (cell.first + 0.5) * 0.2;
  mgg_test::TerrainFixture map(0.2, tops);
  mgg::GroundProjection ground(map, p);
  const mgg::StateVec bottom(0.1, 0.1, 0.54, 0), top(1.1, 0.1, 0.94, 0);
  std::vector<Eigen::Vector3d> projected;
  ASSERT_EQ(ground.getProjectedEdgeStatus(bottom.head<3>(), top.head<3>(),
      r.getPlanningSize(), false, projected, false, false, nullptr,
      mgg::EdgeTravel::kForward), mgg::ProjectedEdgeStatus::kAdmissible);
  EXPECT_FALSE(mgg::reverseExitEdgeAdmissible(map, ground, r, p, top, bottom));
}

TEST(ReverseExit, GenuinelyOneWayStepIsNotAReverseExit) {
  auto p = planning();
  // A legged platform can climb this step, but may not back down it.
  p.max_inclination = 1.3;
  p.max_negative_inclination = 0.2;
  p.max_footprint_cell_rise = 0.0;
  const auto r = robot();
  Tops tops = floor();
  for (auto& [cell, top] : tops) if (cell.first >= 2) top = 0.3;
  mgg_test::TerrainFixture map(0.2, tops);
  mgg::GroundProjection ground(map, p);
  const mgg::StateVec bottom(0.1, 0.1, 0.5, 0), top(1.1, 0.1, 0.8, 0);
  std::vector<Eigen::Vector3d> projected;
  ASSERT_EQ(ground.getProjectedEdgeStatus(bottom.head<3>(), top.head<3>(),
      r.getPlanningSize(), false, projected, false, false, nullptr,
      mgg::EdgeTravel::kForward), mgg::ProjectedEdgeStatus::kAdmissible);
  EXPECT_FALSE(mgg::reverseExitEdgeAdmissible(map, ground, r, p, top, bottom));
}

TEST(ReverseExit, StepAndBodyObstacleOnRetreatAreRefused) {
  const auto p = planning();
  const auto r = robot();
  for (const double height : {0.3, 2.0}) {
    Tops tops = floor();
    for (int y = -10; y < 10; ++y) tops[{5, y}] = height;
    mgg_test::TerrainFixture map(0.2, tops);
    mgg::GroundProjection ground(map, p);
    EXPECT_FALSE(mgg::reverseExitEdgeAdmissible(map, ground, r, p,
        mgg::StateVec(2, 0, 0.5, 0), mgg::StateVec(0, 0, 0.5, 0)));
  }
}

TEST(ReverseExit, ANoGoZoneAllowsDepartureButNotReverseReentry) {
  const auto p = planning();
  const auto r = robot();
  mgg_test::TerrainFixture map(0.2, floor());
  mgg::GroundProjection ground(map, p);
  mgg::NoGoZones zones;
  zones.set({{0, 0}}, 0.6);
  ASSERT_TRUE(zones.pathAdmissible({{0, 0, 0.5}, {2, 0, 0.5}}));
  const auto allowed = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return zones.pathAdmissible({a, b});
  };
  EXPECT_FALSE(mgg::reverseExitEdgeAdmissible(map, ground, r, p,
      mgg::StateVec(2, 0, 0.5, 0), mgg::StateVec(0, 0, 0.5, 0), allowed));
}

TEST(ReverseExit, SlopeTurnVetoRemainsIndependentOfTheExit) {
  mgg::GraphManager graph;
  const auto r = robot();
  mgg::PathTurnCheck check(graph, r, [](const mgg::StateVec&) { return true; },
                           [](const Eigen::Vector3d&) { return 16 * M_PI / 180; });
  mgg::Vertex root(0, {0, 0, 0.5, 0});
  mgg::Vertex corner(1, {0, 1, 0.5, 0});
  EXPECT_FALSE(check({&root, &corner}));
  EXPECT_EQ(check.refused_on_slope, 1);
  EXPECT_EQ(check.refused_without_room, 0);
}

}  // namespace
