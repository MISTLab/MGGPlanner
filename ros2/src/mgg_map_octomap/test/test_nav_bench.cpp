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
