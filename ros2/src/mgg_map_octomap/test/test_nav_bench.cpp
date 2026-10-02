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
    mgg::nav_bench::Outcome best;
    for (int r = 0; r < 3; ++r) {
      const auto outcome = mgg::nav_bench::run(*map, scenario);
      if (r == 0 || outcome.total_ms < best.total_ms) best = outcome;
    }
    std::printf("%s\n", mgg::nav_bench::describe(scenario, best).c_str());
    EXPECT_TRUE(best.expectation_met)
        << mgg::nav_bench::describe(scenario, best);
    EXPECT_LE(best.total_ms, scenario.budget_ms);
  }
}
