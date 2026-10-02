// The navigation benchmark: plans botman's NAVIGATE goals and exploration
// lattices on a real MOLA planning product, as the planner node does, and
// reports the time each takes and where it goes (mgg::PlanProfile).
//
// Usage:
//   mgg_nav_bench <peer_root|--synthetic-floor> [--repeat N] [--budget-ms N] [--request-budget-ms N] [--allow-unknown-body] [--json]
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
#include "mgg_map_octomap/native_mola_grid.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr,
                 "usage: mgg_nav_bench <peer_root|--synthetic-floor> [--repeat N] [--budget-ms N] [--request-budget-ms N] [--allow-unknown-body] [--json]\n");
    return 2;
  }
  int repeat = 3;
  bool json = false;
  double budget_ms = 0.0;
  double request_budget_ms = 500.0;
  bool allow_unknown_body = false;
  for (int i = 2; i < argc; ++i) {
    if (std::strcmp(argv[i], "--repeat") == 0 && i + 1 < argc) {
      repeat = std::max(1, std::atoi(argv[++i]));
    } else if (std::strcmp(argv[i], "--budget-ms") == 0 && i + 1 < argc) {
      char* end = nullptr;
      budget_ms = std::strtod(argv[++i], &end);
      if (*end != '\0' || !std::isfinite(budget_ms) || budget_ms <= 0.0) return 2;
    } else if (std::strcmp(argv[i], "--request-budget-ms") == 0 && i + 1 < argc) {
      char* end = nullptr;
      request_budget_ms = std::strtod(argv[++i], &end);
      if (*end != '\0' || !std::isfinite(request_budget_ms) || request_budget_ms < 0) return 2;
    } else if (std::strcmp(argv[i], "--allow-unknown-body") == 0) {
      allow_unknown_body = true;
    } else if (std::strcmp(argv[i], "--json") == 0) {
      json = true;
    } else {
      return 2;
    }
  }
  std::string error;
  std::unique_ptr<mgg::MolaMap> map;
  std::unique_ptr<mgg::NativeMolaGrid> floor;
  mgg::MolaMap::ReadLease lease;
  const mgg::MapInterface* planning_map = nullptr;
  auto scenarios = mgg::nav_bench::scenarios();
  if (std::strcmp(argv[1], "--synthetic-floor") == 0) {
    std::vector<mgg::NativeMolaGrid::Cell> occupied, free;
    std::vector<mgg::NativeMolaGrid::Surface> surfaces;
    for (int x = -80; x <= 80; ++x) for (int y = -80; y <= 80; ++y) {
      occupied.push_back({x, y, -7});
      surfaces.push_back({{x, y, -7}, -0.61});
      for (int z = -6; z <= 12; ++z) free.push_back({x, y, z});
    }
    floor = std::make_unique<mgg::NativeMolaGrid>(0.1, std::move(occupied),
                                                 std::move(free), std::move(surfaces));
    planning_map = floor.get();
    scenarios.clear();
    for (double length : {2.0, 4.0}) {
      mgg::nav_bench::Scenario scenario;
      scenario.name = "synthetic_floor_" + std::to_string(int(length)) + "m";
      scenario.goal = mgg::StateVec(length, 0, 0, 0);
      scenario.max_length_m = length + 0.1;
      scenario.max_corner_deg = 10.0;
      scenario.budget_ms = 300.0;
      scenarios.push_back(scenario);
    }
    mgg::nav_bench::Scenario explore;
    explore.name = "synthetic_floor_explore_lattice";
    explore.navigate = false;
    explore.budget_ms = 350.0;
    scenarios.push_back(explore);
  } else {
    map = mgg::nav_bench::loadProduct(argv[1], error);
    if (!map) {
      std::fprintf(stderr, "cannot load %s: %s\n", argv[1], error.c_str());
      return 1;
    }
    lease = map->acquireReadLease();
    planning_map = map.get();
  }
  int failures = 0;
  for (auto scenario : scenarios) {
    if (budget_ms > 0.0) scenario.budget_ms = budget_ms;
    for (int r = 0; r < repeat; ++r) {
      const mgg::nav_bench::Outcome outcome =
          mgg::nav_bench::run(*planning_map, scenario, allow_unknown_body, request_budget_ms);
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
  std::fflush(stdout);
  std::fprintf(stderr, "%d run(s) missed their expectation\n", failures);
  return failures == 0 ? 0 : 1;
}
