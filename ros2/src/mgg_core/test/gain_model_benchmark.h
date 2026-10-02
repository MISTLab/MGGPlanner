// Test-only comparison of gain heuristics on a fixed graph/map. Geometry is
// held constant; this reports core best-path ranking, not execution safety.
#ifndef MGG_TEST_GAIN_MODEL_BENCHMARK_H_
#define MGG_TEST_GAIN_MODEL_BENCHMARK_H_

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <tuple>
#include <gtest/gtest.h>

#include "mgg_core/gain.h"
#include "mgg_core/path_selection.h"
#include "mgg_core/tour_params.h"

namespace mgg::test {

inline void benchmarkGainModels(const std::string& scene, GraphManager& graph,
                                GainContext ctx, bool leaves_only,
                                double heading = 0) {
  const int repeat = std::getenv("MGG_GAIN_BENCH_REPEAT")
      ? std::max(1, std::atoi(std::getenv("MGG_GAIN_BENCH_REPEAT"))) : 1;
  PlanningParams planning = *ctx.planning;
  ctx.planning = &planning;
  const RobotParams& robot = *ctx.robot;
  using Clock = std::chrono::steady_clock;
  for (int run = 0; run < repeat; ++run) {
    std::map<int, bool> dense_frontiers;
    std::vector<int> dense_path;
    for (const auto setting : {std::pair<double, double>{0, 0}, {10, 0},
                               {0, 10}, {0, 8}, {10, 10}, {10, 8}, {7.5, 10}}) {
      std::vector<std::tuple<int, int, int, int, bool, double>> unpruned;
      std::vector<int> unpruned_path;
      for (bool pruning : {false, true}) {
        planning.ground_gain_angular_resolution_deg = setting.first;
        planning.ground_gain_max_range = setting.second;
        planning.ground_gain_full_scan = !pruning;
        for (const auto& [id, vertex] : graph.vertices_map_) vertex->vol_gain.reset();
        const auto start = Clock::now();
        const int evaluated = computeExplorationGain(graph, ctx, leaves_only, true);
        const auto scored = Clock::now();
        const auto selected = selectBestPath(graph, planning, robot,
            EdgeInclinations{}, ctx.map->getResolution(), heading);
        const auto done = Clock::now();
        std::vector<int> path;
        for (const auto* vertex : selected.best_path) path.push_back(vertex->id);
        std::vector<std::tuple<int, int, int, int, bool, double>> evidence;
        for (const auto& [id, vertex] : graph.vertices_map_) {
          const auto& g = vertex->vol_gain;
          evidence.emplace_back(id, g.num_unknown_voxels, g.num_free_voxels,
                                g.num_occupied_voxels, g.is_frontier, g.gain);
        }
        if (!pruning) { unpruned = evidence; unpruned_path = path; }
        else {
          EXPECT_EQ(evidence, unpruned) << scene << " step=" << setting.first
                                       << " range=" << setting.second;
          EXPECT_EQ(path, unpruned_path);
        }
        const bool reference = setting.first == 0 && setting.second == 0 && !pruning;
        if (reference) {
          dense_path = path;
          for (const auto& [id, vertex] : graph.vertices_map_)
            if (!leaves_only || vertex->is_leaf_vertex)
              dense_frontiers[id] = vertex->vol_gain.is_frontier;
        }
        int matches = 0, frontiers = 0;
        std::uint64_t rays = 0, visits = 0;
        std::string differences;
        for (const auto& [id, expected] : dense_frontiers) {
          const auto& gain = graph.getVertex(id)->vol_gain;
          if (gain.is_frontier == expected) ++matches;
          else differences += (differences.empty() ? "" : ",") + std::to_string(id);
          frontiers += gain.is_frontier;
          rays += gain.gain_rays_cast;
          visits += gain.gain_voxel_visits;
        }
        const auto found = graph.vertices_map_.find(selected.best_path_id);
        const auto* best = found == graph.vertices_map_.end() ? nullptr : found->second;
        const int best_unknown = best ? best->vol_gain.num_unknown_voxels : 0;
        std::ostringstream row;
        row << "GAIN_MODEL scene=" << scene << " repeat=" << run
                  << " step=" << setting.first << " range=" << setting.second
                  << " pruning=" << pruning << " evaluated=" << evaluated
                  << " gain_ms=" << std::chrono::duration<double, std::milli>(scored-start).count()
                  << " select_ms=" << std::chrono::duration<double, std::milli>(done-scored).count()
                  << " frontier_agree=" << matches << "/" << dense_frontiers.size()
                  << " frontier_diff=" << (differences.empty() ? "none" : differences)
                  << " frontiers=" << frontiers << " path_agree=" << (path == dense_path)
                  << " best=" << selected.best_path_id
                  << " dense_best=" << (dense_path.empty() ? -1 : dense_path.back())
                  << " best_unknown=" << best_unknown << " best_gain=" << selected.best_gain
                  << " interest_pass=" << (best_unknown >= planning.low_gain_voxels &&
                                             selected.best_gain >= TourParams{}.min_cluster_gain)
                  << " rays=" << rays << " visits=" << visits;
        std::cout << row.str() << std::endl;
      }
    }
  }
}

}  // namespace mgg::test
#endif
