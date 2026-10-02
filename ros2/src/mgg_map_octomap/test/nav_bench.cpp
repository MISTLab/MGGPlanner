// The navigation benchmark: plans botman's NAVIGATE goals and exploration
// lattices on a real MOLA planning product, as the planner node does, and
// reports the time each takes and where it goes (mgg::PlanProfile).
//
// Usage:
//   mgg_nav_bench <peer_root> [--repeat N] [--json]
//
// <peer_root> holds mola/source.json, mola/index.json and the planner grid
// (the robot's maps/<mission>/<robot>/planning directory). The authority
// transform is the identity, as for a single robot whose component frame is
// its capture frame. The robot, terrain limits and lattice are botman's
// deployed configuration (deploy/mgg/hardware.yaml with bunker.yaml, at the
// 0.10 m planner resolution); see botmanPlanning().
//
// The same scenarios back NavBench tests in test_nav_bench.cpp, which assert
// the x86 time budgets.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "nav_bench_scenarios.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr,
                 "usage: mgg_nav_bench <peer_root> [--repeat N] [--json]\n");
    return 2;
  }
  int repeat = 3;
  bool json = false;
  for (int i = 2; i < argc; ++i) {
    if (std::strcmp(argv[i], "--repeat") == 0 && i + 1 < argc) {
      repeat = std::max(1, std::atoi(argv[++i]));
    } else if (std::strcmp(argv[i], "--json") == 0) {
      json = true;
    }
  }
  std::string error;
  auto map = mgg::nav_bench::loadProduct(argv[1], error);
  if (!map) {
    std::fprintf(stderr, "cannot load %s: %s\n", argv[1], error.c_str());
    return 1;
  }
  auto lease = map->acquireReadLease();
  int failures = 0;
  for (const auto& scenario : mgg::nav_bench::scenarios()) {
    for (int r = 0; r < repeat; ++r) {
      const mgg::nav_bench::Outcome outcome =
          mgg::nav_bench::run(*map, scenario);
      if (!outcome.expectation_met) ++failures;
      // Retain every sample, including the cold first run and slow outliers.
      if (json) {
        std::printf("%s\n", mgg::nav_bench::toJson(scenario, outcome).c_str());
      } else {
        std::printf("repeat %d/%d: %s\n", r + 1, repeat,
                    mgg::nav_bench::describe(scenario, outcome).c_str());
      }
    }
  }
  std::fprintf(stderr, "%d run(s) missed their expectation\n", failures);
  return failures == 0 ? 0 : 1;
}
