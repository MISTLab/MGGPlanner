#include "mgg_core/planning_cancellation.h"
// Time budgets of botman's NAVIGATE and exploration plans on its real
// planning product (nav_bench_scenarios.h): NAVIGATE within 0.3 s and an
// exploration lattice within 0.35 s on an x86 desktop. The product is 8 MB
// and is not kept in the repository: the test runs when
// MGG_NAV_BENCH_PRODUCT names its peer root (holding mola/), and is skipped
// otherwise. The same scenarios run in the mgg_nav_bench binary.

#include <cstdlib>
#include <string>

#include <gtest/gtest.h>

#include "nav_bench_scenarios.h"
#include "mgg_map_octomap/native_mola_grid.h"
#include "mgg_core/departure.h"

TEST(NavBench, BotmanPlansMeetTheirTimeBudgets) {
  const char* root = std::getenv("MGG_NAV_BENCH_PRODUCT");
  if (root == nullptr || std::string(root).empty()) {
    GTEST_SKIP() << "MGG_NAV_BENCH_PRODUCT is not set";
  }
  std::string error;
  auto map = mgg::nav_bench::loadProduct(root, error);
  ASSERT_NE(map, nullptr) << error;
  auto lease = map->acquireReadLease();
  for (const auto& scenario : mgg::nav_bench::scenarios()) {
    SCOPED_TRACE(scenario.name);
    for (int r = 0; r < 3; ++r) {
      SCOPED_TRACE(r);
      const auto outcome = mgg::nav_bench::run(*map, scenario);
      std::printf("%s\n", mgg::nav_bench::describe(scenario, outcome).c_str());
      EXPECT_TRUE(outcome.expectation_met)
          << mgg::nav_bench::describe(scenario, outcome);
      EXPECT_LE(outcome.total_ms, scenario.budget_ms);
    }
  }
}

TEST(NavBench, CircumscribedPrefilterMatchesExactSweeps) {
  using Grid = mgg::NativeMolaGrid;
  std::vector<Grid::Cell> occupied{{5, 3, 5}, {0, 9, 10}, {-8, -3, 13}};
  std::vector<Grid::Cell> free;
  for (int x = -20; x <= 20; ++x)
    for (int y = -20; y <= 20; ++y)
      for (int z = 0; z <= 20; ++z) free.push_back({x, y, z});
  Grid map(0.1, occupied, free, {});
  mgg::OrientedBox body;
  body.size = Eigen::Vector3d(1.2, 0.6, 0.8);
  for (bool unknown : {false, true}) {
    for (double heading : {0.0, M_PI / 4, M_PI_2, M_PI}) {
      body.heading = heading;
      for (double z : {0.5, 1.0, 1.6}) {
        for (double rise : {0.0, 0.3, -0.3}) {
          const Eigen::Vector3d a(0.03, 0.07, z);
          const Eigen::Vector3d b = a + Eigen::Vector3d(
              1.8 * std::cos(heading), 1.8 * std::sin(heading), rise);
          EXPECT_EQ(mgg::orientedBoxPathStatus(map, a, b, body, unknown, nullptr, true),
                    mgg::orientedBoxPathStatus(map, a, b, body, unknown, nullptr, false));
        }
      }
    }
  }
  // Front corner: outside the inscribed disc, inside the actual rectangle.
  body.heading = 0;
  const Eigen::Vector3d at(0, 0.1, 0.5);
  EXPECT_EQ(mgg::orientedBoxPathStatus(map, at, at, body, false, nullptr, true),
            mgg::VoxelStatus::kOccupied);
}

TEST(NavBench, ExpiredBudgetInterruptsNativeGainRaycasts) {
  mgg::NativeMolaGrid map(0.1, {}, {}, {});
  mgg::GainCounts gain;
  std::vector<std::pair<Eigen::Vector3d, mgg::VoxelStatus>> log;
  const auto deadline = std::chrono::steady_clock::now();
  mgg::PlanningCancellationScope budget([&] {
    return std::chrono::steady_clock::now() >= deadline;
  });
  EXPECT_THROW(map.getScanStatusIterative(Eigen::Vector3d::Zero(),
      {Eigen::Vector3d(10, 0, 0)}, gain, log, mgg::SensorModel{}),
      mgg::PlanningInterrupted);
}

TEST(NavBench, LazyLatticeClimbsRampOntoDeckAboveVisitedFloor) {
  using Grid = mgg::NativeMolaGrid;
  std::vector<Grid::Cell> occupied, free;
  std::vector<Grid::Surface> surfaces;
  for (int x = -30; x < 90; ++x) for (int y = -30; y < 40; ++y) {
    occupied.push_back({x, y, -1}); surfaces.push_back({{x, y, -1}, 0});
    const double px = (x + .5) * .1, py = (y + .5) * .1;
    double top = -1;
    if (px >= 2 && py >= -.5 && py <= 1.5) top = std::min(2.0, (px - 2) / 3.0);
    if (py > 1.5 && py < 3.5 && px >= -1) top = 2;
    if (top >= 0) {
      const int z = std::floor(top / .1);
      occupied.push_back({x, y, z}); surfaces.push_back({{x, y, z}, top});
    }
    for (int z = 0; z < 35; ++z) free.push_back({x, y, z});
  }
  Grid map(.1, occupied, free, surfaces);
  mgg::RobotParams robot;
  robot.size = Eigen::Vector3d(.2, .2, .2);
  mgg::PlanningParams planning;
  planning.max_ground_height = .4;
  planning.max_step_height = .15;
  planning.max_inclination = .6;
  planning.max_footprint_tilt = .6;
  planning.max_footprint_step = .15;
  planning.min_observed_ground_fraction = 0;
  planning.edge_length_min = 0;
  planning.edge_length_max = .6;
  planning.edge_overshoot = 0;
  planning.num_vertices_max = 5000;
  planning.num_edges_max = 50000;
  planning.num_loops_max = 20000;
  mgg::GroundProjection ground(map, planning, true);
  mgg::ExpandContext ctx;
  ctx.map = &map; ctx.robot = &robot; ctx.planning = &planning;
  ctx.ground = &ground; ctx.robot_box_size = robot.getPlanningSize();
  ctx.allow_unknown_lattice_body = false;
  mgg::GridGraphParams grid;
  grid.min_val = Eigen::Vector3d(-2, -2, 0);
  grid.max_val = Eigen::Vector3d(8.4, 3.2, 3);
  grid.resolution = Eigen::Vector3d(.4, .4, .1);
  mgg::GraphManager graph;
  const auto result = mgg::routeOverLocalLattice(graph, mgg::StateVec(0,0,.4,0),
      mgg::StateVec(0,2.4,2.4,0), grid, ctx);
  EXPECT_TRUE(result.routed) << result.reason;
  bool overlapping_levels = false;
  for (const auto& [id, a] : graph.vertices_map_) for (const auto& [other, b] : graph.vertices_map_)
    if ((a->state.head<2>() - b->state.head<2>()).norm() < 1e-6 &&
        std::abs(a->state.z() - b->state.z()) > 1) overlapping_levels = true;
  EXPECT_TRUE(overlapping_levels);
}
