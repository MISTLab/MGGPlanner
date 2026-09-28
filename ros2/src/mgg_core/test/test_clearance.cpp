#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>

#include <gtest/gtest.h>

#include "mgg_core/grid_graph.h"
#include "mgg_core/global_graph.h"
#include "mgg_core/trajectory.h"
#include "terrain_fixture.h"

namespace {

// Doorway walls with low non-drivable rubble at the wider gate's approach.
mgg_test::TerrainFixture doorway(double width) {
  std::map<std::pair<std::int64_t, std::int64_t>, double> heights;
  for (int x = -40; x <= 40; ++x) {
    for (int y = -35; y <= 35; ++y) {
      const double px = (x + 0.5) * 0.1, py = (y + 0.5) * 0.1;
      double z = std::abs(px) <= 1.0 && std::abs(py) >= width / 2 ? 2.0 : 0.0;
      if (width > 2 && px >= -1.5 && px <= -1 && std::abs(py) >= 0.9) z = 0.12;
      heights[{x, y}] = z;
    }
  }
  return mgg_test::TerrainFixture(0.1, heights);
}

mgg::PlanningParams planning(double margin) {
  mgg::PlanningParams p;
  p.max_ground_height = 0.4475;
  p.max_step_height = 0.15;
  p.max_inclination = 27 * M_PI / 180;
  p.max_cross_slope = 18 * M_PI / 180;
  p.max_footprint_tilt = 25 * M_PI / 180;
  p.max_footprint_step = 0.08;
  p.max_footprint_cell_rise = 0.075;
  p.min_observed_ground_fraction = 0.0;
  p.path_clearance_margin = margin;
  p.rr_mode = mgg::RRModeType::kGraph;
  p.edge_length_min = 0.01;
  p.edge_length_max = 0.5;
  p.edge_overshoot = 0.0;
  p.nearest_range = 0.3;
  p.nearest_range_min = 0.01;
  p.nearest_range_max = 0.31;
  p.num_vertices_max = 3000;
  p.num_edges_max = 30000;
  p.num_loops_max = 50000;
  return p;
}

mgg::PathType route(double width, double margin, int& vertices, int& edges) {
  auto map = doorway(width);
  auto p = planning(margin);
  mgg::RobotParams robot;
  robot.type = mgg::RobotType::kGroundRobot;
  robot.size = {0.612, 0.58, 0.245};
  robot.size_extension = {0.05, 0.05, 0.05};
  const double y = width > 2 ? -0.7 : 0.0;
  const mgg::StateVec start(-2, y, 0.4475, 0);
  mgg::GraphManager graph;
  graph.addVertex(new mgg::Vertex(0, start));
  mgg::GroundProjection ground(map, p, true);
  mgg::ExpandContext ctx;
  ctx.map = &map;
  ctx.planning = &p;
  ctx.robot = &robot;
  ctx.ground = &ground;
  ctx.robot_box_size = {0.662, 0.630, 0.295};
  mgg::GridGraphParams grid;
  grid.min_val = {-0.2, -1.0, 0};
  grid.max_val = {4.2, 2.0, 0};
  grid.resolution = {0.2, 0.2, 0.1};
  const auto before = std::chrono::steady_clock::now();
  mgg::buildGridGraph(graph, start, grid, ctx, 0);
  mgg::ShortestPathsReport report;
  EXPECT_TRUE(graph.findShortestPaths(0, report));
  mgg::StateVec goal(2, y, 0.4475, 0);
  mgg::Vertex* end = nullptr;
  EXPECT_TRUE(graph.getNearestVertex(&goal, &end));
  if (end == nullptr) return {};
  EXPECT_LT((end->state - goal).norm(), 0.01);
  std::vector<mgg::Vertex*> path;
  graph.getShortestPath(end->id, report, true, path);
  mgg::PathType result;
  for (auto* vertex : path) result.push_back(vertex->state.head<3>());
  vertices = graph.getNumVertices();
  edges = graph.getNumEdges();
  const auto ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - before).count();
  std::printf("doorway %.1f margin %.1f: %d vertices %d edges %.3f ms\n",
              width, margin, vertices, edges, ms);
  return result;
}

TEST(Clearance, LatticePrefersCentreWithoutRemovingEdges) {
  int v0 = 0, e0 = 0, v1 = 0, e1 = 0;
  const auto baseline = route(2.5, 0, v0, e0);
  const auto preferred = route(2.5, 0.6, v1, e1);
  EXPECT_EQ(v0, v1);
  EXPECT_EQ(e0, e1);
  ASSERT_GT(preferred.size(), 5u);
  bool crossing = false;
  for (const auto& point : preferred) {
    if (std::abs(point.x()) <= 0.8) {
      crossing = true;
      EXPECT_LT(std::abs(point.y()), 0.35);
    }
  }
  EXPECT_TRUE(crossing);
}

TEST(Clearance, OneMetrePassageRemainsAvailable) {
  int v0 = 0, e0 = 0, v1 = 0, e1 = 0;
  const auto baseline = route(1.0, 0, v0, e0);
  const auto preferred = route(1.0, 0.6, v1, e1);
  EXPECT_EQ(v0, v1);
  EXPECT_EQ(e0, e1);
  ASSERT_GT(preferred.size(), 5u);
  EXPECT_NEAR(preferred.back().x(), 2.0, 1e-6);
}

TEST(Clearance, GlobalShortcutKeepsClearanceAndNarrowPassage) {
  for (double width : {2.5, 1.0}) {
    int vertices = 0, edges = 0;
    const auto path = route(width, 0.6, vertices, edges);
    auto map = doorway(width);
    auto p = planning(0.6);
    mgg::GroundProjection ground(map, p, true);
    const Eigen::Vector3d box(0.662, 0.630, 0.295);
    const auto admissible = [&](const auto& a, const auto& b) {
      std::vector<Eigen::Vector3d> projected;
      return ground.getProjectedEdgeStatus(a, b, box, true, projected, false) ==
             mgg::ProjectedEdgeStatus::kAdmissible;
    };
    const auto cost = [&](const auto& a, const auto& b) {
      return ground.clearanceCost(a, b, box);
    };
    const auto before = std::chrono::steady_clock::now();
    const auto shortcut = mgg::shortcutPath(path, admissible, {}, cost);
    const auto ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - before).count();
    std::printf("doorway %.1f cost-aware shortcut: %.3f ms\n", width, ms);
    ASSERT_GE(shortcut.size(), 2u);
    double original_cost = 0, shortcut_cost = 0;
    for (size_t i = 1; i < path.size(); ++i) {
      original_cost += cost(path[i - 1], path[i]);
    }
    for (size_t i = 1; i < shortcut.size(); ++i) {
      EXPECT_TRUE(admissible(shortcut[i - 1], shortcut[i]));
      shortcut_cost += cost(shortcut[i - 1], shortcut[i]);
      if (width > 2) {
        for (int j = 0; j <= 100; ++j) {
          const Eigen::Vector3d at = shortcut[i - 1] +
              (j / 100.0) * (shortcut[i] - shortcut[i - 1]);
          if (std::abs(at.x()) <= 0.8) EXPECT_LT(std::abs(at.y()), 0.35);
        }
      }
    }
    EXPECT_LE(shortcut_cost, original_cost + 1e-9);
    EXPECT_TRUE(shortcut.back().isApprox(path.back()));
  }
}

TEST(Clearance, RubbleBelowBodyStillAddsSoftCost) {
  auto map = doorway(2.5);
  auto p = planning(0.6);
  const Eigen::Vector3d box(0.662, 0.630, 0.295);
  const Eigen::Vector3d a(-1.9, -0.55, 0.4475), b(-1.6, -0.55, 0.4475);
  mgg::GroundProjection ground(map, p, true);
  const double with_rubble = ground.clearanceCost(a, b, box);
  p.max_footprint_cell_rise = 0.0;
  EXPECT_GT(with_rubble, ground.clearanceCost(a, b, box));
  EXPECT_GE(with_rubble, (b - a).norm());
  EXPECT_LE(with_rubble, 5 * (b - a).norm());
}

TEST(Clearance, CacheDoesNotChangeCostOrRetainAnotherBodyHeight) {
  auto map = doorway(2.5);
  auto p = planning(0.6);
  mgg::GroundProjection cached(map, p, true), plain(map, p);
  for (double height : {0.295, 1.0, 0.295}) {
    for (double y : {-0.7, 0.0, 0.7}) {
      const Eigen::Vector3d a(-2, y, 0.4475), b(2, y, 0.4475);
      const Eigen::Vector3d box(0.662, 0.630, height);
      EXPECT_NEAR(cached.clearanceCost(a, b, box),
                  plain.clearanceCost(a, b, box), 1e-9);
    }
  }
}

class UnenumerableClearance : public mgg_test::TerrainFixture {
 public:
  UnenumerableClearance() : TerrainFixture(0.1, {}) {}
  bool getCircleIntersectingXYCellCenters(
      const Eigen::Vector2d&, double radius, std::size_t limit,
      std::vector<mgg::XYCellCenter>&) const override {
    ++calls;
    EXPECT_EQ(limit, 1024u);
    EXPECT_LT(radius, 2.0);  // margin is capped even if configured enormous
    return false;
  }
  mutable int calls = 0;
};

TEST(Clearance, WorkAndCostStayBoundedWhenMapCannotEnumerate) {
  UnenumerableClearance map;
  auto p = planning(100.0);
  mgg::GroundProjection ground(map, p, true);
  const Eigen::Vector3d a(0, 0, 0.4475), b(100, 0, 0.4475);
  EXPECT_DOUBLE_EQ(ground.clearanceCost(a, b, {0.662, 0.630, 0.295}), 500);
  EXPECT_EQ(map.calls, 32);
  p.path_clearance_margin = 0.0;
  EXPECT_DOUBLE_EQ(ground.clearanceCost(a, b, {0.662, 0.630, 0.295}), 100);
  EXPECT_EQ(map.calls, 32);
}

class GoalClearanceMap : public mgg_test::TerrainFixture {
 public:
  GoalClearanceMap() : TerrainFixture(doorway(2.5)) {}
  bool getCircleIntersectingXYCellCenters(
      const Eigen::Vector2d& center, double radius, std::size_t limit,
      std::vector<mgg::XYCellCenter>& cells) const override {
    if (radius > 0.7) ++clearance_queries;
    return TerrainFixture::getCircleIntersectingXYCellCenters(
        center, radius, limit, cells);
  }
  mutable int clearance_queries = 0;
};

TEST(Clearance, GoalLatticeSkipsCostAndKeepsMetricEdges) {
  GoalClearanceMap map;
  auto p = planning(0.6);
  mgg::GroundProjection ground(map, p);  // same uncached context as the node
  mgg::RobotParams robot;
  robot.type = mgg::RobotType::kGroundRobot;
  robot.size = {0.612, 0.580, 0.245};
  robot.size_extension = {0.05, 0.05, 0.05};
  mgg::ExpandContext ctx;
  ctx.map = &map;
  ctx.planning = &p;
  ctx.robot = &robot;
  ctx.ground = &ground;
  ctx.robot_box_size = {0.662, 0.630, 0.295};
  mgg::GraphManager graph;
  graph.addVertex(new mgg::Vertex(0, mgg::StateVec(-2, 0, 0.4475, 0)));
  mgg::GridGraphParams grid;
  grid.min_val = {-4.2, -1.0, 0};
  grid.max_val = {0.2, 1.0, 0};
  grid.resolution = {0.2, 0.2, 0.1};
  auto* goal = mgg::connectGoalThroughLattice(
      graph, {2, 0, 0.4475, 0}, grid, ctx, 0, {});
  ASSERT_NE(goal, nullptr);
  EXPECT_EQ(map.clearance_queries, 0);
  for (const auto& [from, edges] : graph.edge_map_) {
    for (const auto& [to, weight] : edges) {
      EXPECT_NEAR(weight, (graph.getVertex(from)->state.head<3>() -
                          graph.getVertex(to)->state.head<3>()).norm(), 1e-9);
    }
  }
  EXPECT_DOUBLE_EQ(p.path_clearance_margin, 0.6);
}

}  // namespace
