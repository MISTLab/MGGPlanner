// Volumetric gain (mgg_core/gain.cpp) evaluated on the native SDMGRID1 grid:
// the gain counts only the unknown space a viewpoint could see.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "mgg_core/gain.h"
#include "../../mgg_core/test/gain_model_benchmark.h"
#include "mgg_core/path_selection.h"
#include "mgg_core/planning_cancellation.h"
#include "mgg_core/tour_params.h"
#include "mgg_map_octomap/native_mola_grid.h"
#include "mgg_map_octomap/scan_walk.h"

namespace {

using Cell = mgg::NativeMolaGrid::Cell;
using mgg::VoxelStatus;

constexpr double kResolution = 0.2;

/// The simulated VLP16 of mgg_argos/config/bistro.yaml: 360 x 45 degrees at
/// 5 degrees, 20 m.
struct GainSetup {
  explicit GainSetup(mgg::MapInterface& map) {
    sensor.type = mgg::SensorType::kLidar;
    sensor.max_range = 20.0;
    sensor.fov = Eigen::Vector2d(2.0 * M_PI, M_PI / 4.0);
    sensor.resolution = Eigen::Vector2d(M_PI / 36.0, M_PI / 36.0);
    sensor.update();
    sensors["VLP16"] = sensor;
    planning.exp_sensor_list = {"VLP16"};
    planning.unknown_voxel_gain = 1.0;
    planning.free_voxel_gain = 0.0;
    planning.occupied_voxel_gain = 0.0;
    space.min_val = Eigen::Vector3d::Constant(-100.0);
    space.max_val = Eigen::Vector3d::Constant(100.0);
    space.setCenter(Eigen::Vector3d(0.0, 0.0, 0.0), false);
    ctx.map = &map;
    ctx.planning = &planning;
    ctx.global_space = &space;
    ctx.sensors = &sensors;
  }
  mgg::SensorParams sensor;
  std::unordered_map<std::string, mgg::SensorParams> sensors;
  mgg::PlanningParams planning;
  mgg::BoundedSpaceParams space;
  mgg::GainContext ctx;
};

// The reference takes the original full frustum through the same backend.
// Compare every counted voxel as well as all production scoring fields.
mgg::VolumetricGain expectFullScanEquivalent(
    const mgg::StateVec& state, GainSetup& setup) {
  mgg::VolumetricGain pruned, full;
  std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> a, b;
  setup.planning.ground_gain_full_scan = false;
  mgg::computeVolumetricGain(state, pruned, setup.ctx, &a);
  setup.planning.ground_gain_full_scan = true;
  mgg::computeVolumetricGain(state, full, setup.ctx, &b);
  setup.planning.ground_gain_full_scan = false;
  EXPECT_EQ(pruned.num_unknown_voxels, full.num_unknown_voxels);
  EXPECT_EQ(pruned.num_free_voxels, full.num_free_voxels);
  EXPECT_EQ(pruned.num_occupied_voxels, full.num_occupied_voxels);
  EXPECT_EQ(pruned.is_frontier, full.is_frontier);
  EXPECT_DOUBLE_EQ(pruned.gain, full.gain);
  std::cout << "SCAN_WORK rays=" << pruned.gain_rays_cast << "/"
            << full.gain_rays_cast << " visits=" << pruned.gain_voxel_visits
            << "/" << full.gain_voxel_visits << '\n';
  auto sorted = [](const auto& entries) {
    std::vector<std::tuple<double, double, double, int>> cells;
    for (const auto& e : entries)
      cells.emplace_back(e.first.x(), e.first.y(), e.first.z(), int(e.second));
    std::sort(cells.begin(), cells.end());
    return cells;
  };
  EXPECT_EQ(sorted(a), sorted(b));
  return full;
}

std::tuple<long, long, long> cellOf(const Eigen::Vector3d& centre) {
  return {std::lround(std::floor(centre.x() / kResolution)),
          std::lround(std::floor(centre.y() / kResolution)),
          std::lround(std::floor(centre.z() / kResolution))};
}

// Diagnosis 2026-09-24 (diag-viewpoint, Q2): the gain walked every ray with
// getScanStatus, which logs a voxel once per ray through it, so the voxels
// next to the viewpoint, crossed by hundreds of rays, were counted hundreds
// of times; the unique count was 0.77 of the raw count. A voxel is revealed
// once, however many rays cross it.
TEST(NativeGain, EachUnknownVoxelCountsOncePerViewpoint) {
  mgg::NativeMolaGrid map(kResolution, {}, {}, {});
  GainSetup setup(map);
  mgg::VolumetricGain gain;
  std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> counted;
  mgg::computeVolumetricGain(mgg::StateVec(0.1, 0.1, 0.1, 0.0), gain,
                             setup.ctx, &counted);

  std::set<std::tuple<long, long, long>> distinct;
  for (const auto& entry : counted) {
    ASSERT_EQ(entry.second, VoxelStatus::kUnknown);
    distinct.insert(cellOf(entry.first));
  }
  ASSERT_GT(distinct.size(), 1000u);
  EXPECT_EQ(counted.size(), distinct.size());
  EXPECT_EQ(gain.num_unknown_voxels, static_cast<int>(distinct.size()));
}

/// A ground robot's gain, the voxels it counted, seen from `viewpoint`: a
/// vertex 0.45 m over its floor at z = 0.05. Its sensor is mounted
/// `mount_height` over that floor (SensorParams::mount_height; 0 casts from
/// the vertex), `center_offset` from the body centre.
std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> groundGain(
    mgg::MapInterface& map, const Eigen::Vector3d& viewpoint,
    double mount_height = 0.0,
    const Eigen::Vector3d& center_offset = Eigen::Vector3d::Zero(),
    mgg::RobotType type = mgg::RobotType::kGroundRobot) {
  GainSetup setup(map);
  mgg::RobotParams robot;
  robot.type = type;
  robot.size = Eigen::Vector3d(0.5, 0.5, 0.3);
  setup.ctx.robot = &robot;
  setup.planning.robot_height = 0.2;
  setup.planning.max_ground_height = 0.45;
  mgg::SensorParams& sensor = setup.sensors["VLP16"];
  sensor.mount_height = mount_height;
  sensor.center_offset = center_offset;
  sensor.update();
  mgg::VolumetricGain gain;
  std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> counted;
  mgg::computeVolumetricGain(
      mgg::StateVec(viewpoint.x(), viewpoint.y(), viewpoint.z(), 0.0), gain,
      setup.ctx, &counted);
  expectFullScanEquivalent(
      mgg::StateVec(viewpoint.x(), viewpoint.y(), viewpoint.z(), 0.0), setup);
  return counted;
}

/// A floor at z = [-0.2, 0) except where `ground_row(x, y)` says otherwise:
/// the row of the column's occupied ground voxel, or no ground at all.
/// Air over the ground is mapped free up to x = 2.0 and z = 2.0; beyond, the
/// map is unknown. 20 m to either side.
mgg::NativeMolaGrid terrain(
    const std::function<std::optional<std::int64_t>(std::int64_t,
                                                    std::int64_t)>& ground) {
  std::vector<Cell> occupied, free;
  for (std::int64_t y = -100; y < 100; ++y)
    for (std::int64_t x = -100; x < 110; ++x) {
      const auto row = ground(x, y);
      if (row) occupied.push_back({x, y, *row});
      if (x < 10)
        for (std::int64_t z = row ? *row + 1 : -1; z < 10; ++z)
          free.push_back({x, y, z});
    }
  return mgg::NativeMolaGrid(kResolution, occupied, free, {});
}

/// Unknown voxels counted between `low` and `high`, beyond x = 2.2.
int beyondBetween(
    const std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& counted,
    double low, double high) {
  return static_cast<int>(std::count_if(
      counted.begin(), counted.end(), [low, high](const auto& entry) {
        return entry.second == VoxelStatus::kUnknown &&
               entry.first.x() > 2.2 && entry.first.z() > low &&
               entry.first.z() < high;
      }));
}

// Diagnosis 2026-09-24 (diag-viewpoint, Q2): the gain counted voxels down to
// 1.0 m below the vertex, about 0.55 m below the floor for the simulated
// robots; under a mapped floor nothing can be seen (0.10 of the count at the
// captured plan ends). Review r0 (P1): cutting the band one voxel under the
// vertex's own floor also cut the space over ground that falls away. The
// cut now follows the ground mapped over each voxel.
TEST(NativeGain, UnderFloorGainDropsOnlyUnderMappedGround) {
  // The vertex's floor is at z = 0.05; one voxel under it is z = -0.15.
  const Eigen::Vector3d viewpoint(0.1, 0.5, 0.5);
  // Flat ground whose floor has one-voxel gaps, as a lidar leaves: nothing
  // under it counts.
  auto flat = terrain([](std::int64_t x, std::int64_t y)
                          -> std::optional<std::int64_t> {
    if (x % 3 == 0 && y % 3 == 0) return std::nullopt;
    return -1;
  });
  const auto over_flat = groundGain(flat, viewpoint);
  const int under_flat = static_cast<int>(std::count_if(
      over_flat.begin(), over_flat.end(), [](const auto& entry) {
        return entry.second == VoxelStatus::kUnknown &&
               entry.first.z() < -0.15;
      }));
  EXPECT_EQ(under_flat, 0);

  // A 16 degree ramp down from x = 1.0: the air over it, below the vertex's
  // floor, counts.
  const double slope = std::tan(16.0 * M_PI / 180.0);
  auto ramp = terrain([slope](std::int64_t x, std::int64_t)
                          -> std::optional<std::int64_t> {
    const double drop = std::max(0.0, ((x + 0.5) * kResolution - 1.0) * slope);
    return -1 - static_cast<std::int64_t>(std::floor(drop / kResolution));
  });
  EXPECT_GT(beyondBetween(groundGain(ramp, viewpoint), -0.5, -0.15), 20);

  // A stairwell opening, 3.4 by 3.0 m, at x = [3.6, 7.0): no ground in it.
  auto stairwell = terrain([](std::int64_t x, std::int64_t y)
                               -> std::optional<std::int64_t> {
    if (x >= 18 && x < 35 && y >= -5 && y < 10) return std::nullopt;
    return -1;
  });
  EXPECT_GT(beyondBetween(groundGain(stairwell, viewpoint), -0.5, -0.15), 30);
}

/// The highest unknown voxel centre in `counted`.
double highestUnknown(
    const std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& counted) {
  double top = -1e9;
  for (const auto& entry : counted) {
    if (entry.second == VoxelStatus::kUnknown) {
      top = std::max(top, entry.first.z());
    }
  }
  return top;
}

// Run 7: rays still originate at the mount, even above the band; unknown
// ground ahead keeps the corridor a frontier without rewarding its ceiling.
TEST(NativeGain, Run7MountedCorridorRetainsGroundBandUnknown) {
  auto floor = terrain([](std::int64_t, std::int64_t)
                           -> std::optional<std::int64_t> { return -1; });
  const Eigen::Vector3d viewpoint(0.1, 0.5, 0.5);
  const auto from_vertex = groundGain(floor, viewpoint);
  EXPECT_GT(beyondBetween(from_vertex, 0.0, 0.85), 20);
  EXPECT_LE(highestUnknown(from_vertex), 0.85);
  const auto mounted =
      groundGain(floor, viewpoint, 2.0, Eigen::Vector3d(-0.2, 0.0, 5.0));
  EXPECT_GT(beyondBetween(mounted, 0.0, 0.85), 20);
  EXPECT_LE(highestUnknown(mounted), 0.85);
  EXPECT_NE(mounted.size(), from_vertex.size());

  // An aerial robot retains full 3D gain and ignores ground mount height.
  const auto aerial = groundGain(floor, viewpoint, 2.0, Eigen::Vector3d::Zero(),
                                  mgg::RobotType::kAerialRobot);
  EXPECT_GT(highestUnknown(aerial), 6.0);
  EXPECT_DOUBLE_EQ(highestUnknown(aerial),
      highestUnknown(groundGain(floor, viewpoint, 0.0, Eigen::Vector3d::Zero(),
                                mgg::RobotType::kAerialRobot)));
}

TEST(NativeGain, WalledCorridorClearsDeployedInterestThresholds) {
  // 1.5 m corridor, rasterized conservatively to 1.4 m at the deployed
  // 0.2 m resolution; also exercise exactly 1.5 m on a 0.1 m grid.
  for (double resolution : {0.1, 0.2}) {
    std::vector<Cell> occupied, free;
    const int half_width = static_cast<int>(0.75 / resolution);
    const int upper = half_width + 1;
    for (int x = -int(22 / resolution); x < int(22 / resolution); ++x) {
      for (int y = -half_width - 1; y <= upper; ++y) {
        occupied.push_back({x, y, -1});
        for (int z = 0; z < int(4 / resolution); ++z) {
          if (y == -half_width - 1 || y == upper)
            occupied.push_back({x, y, z});
          else if (x * resolution < 2.0)
            free.push_back({x, y, z});
        }
      }
    }
    mgg::NativeMolaGrid map(resolution, occupied, free, {});
    for (const auto& platform : std::vector<std::pair<double, double>>{
             {0.45, 0.245}, {0.72, 0.40}, {0.97, 1.0}}) {
      SCOPED_TRACE(::testing::Message() << "resolution=" << resolution
                   << " mount=" << platform.first);
      GainSetup setup(map);
      mgg::RobotParams robot;
      robot.size = Eigen::Vector3d(0.8, 0.5, platform.second);
      setup.ctx.robot = &robot;
      setup.planning.max_ground_height = 0.5;
      setup.planning.unknown_voxel_gain = 60;
      setup.planning.path_length_penalty = 0.25;
      setup.planning.path_direction_penalty = 1.0;
      setup.sensors["VLP16"].mount_height = platform.first;
      setup.sensors["VLP16"].update();
      mgg::GraphManager graph;
      for (int id = 0; id < 4; ++id) {
        auto* v = new mgg::Vertex(id, mgg::StateVec(0.1 + 0.5 * id, 0.05, 0.5, 0));
        v->is_leaf_vertex = id == 3;
        graph.addVertex(v);
        if (id) graph.addEdge(graph.getVertex(id - 1), v, 0.5);
      }
      const auto start = std::chrono::steady_clock::now();
      mgg::computeExplorationGain(graph, setup.ctx, true, true);
      const auto scored = std::chrono::steady_clock::now();
      const auto& gain = graph.getVertex(3)->vol_gain;
      const auto selected = mgg::selectBestPath(graph, setup.planning, robot,
                                                mgg::EdgeInclinations{}, resolution, 0);
      std::cout << "corridor resolution=" << resolution << " mount=" << platform.first
                << " band=" << gain.num_unknown_voxels << " total="
                << gain.num_total_unknown_voxels << " path_gain=" << selected.best_gain
                << " gain_ms=" << std::chrono::duration<double, std::milli>(scored - start).count()
                << " selection_ms=" << std::chrono::duration<double, std::milli>(
                     std::chrono::steady_clock::now() - scored).count() << '\n';
      expectFullScanEquivalent(graph.getVertex(3)->state, setup);
      EXPECT_TRUE(gain.is_frontier);
      EXPECT_GE(gain.num_unknown_voxels, setup.planning.low_gain_voxels);
      EXPECT_GE(selected.best_gain, mgg::TourParams{}.min_cluster_gain);
      EXPECT_EQ(selected.best_path_id, 3);
      for (double vfov : {45.0, 60.0}) {
        setup.sensors["VLP16"].fov.y() = vfov * M_PI / 180;
        setup.sensors["VLP16"].update();
        setup.planning.ground_gain_angular_resolution_deg = 7.5;
        setup.planning.ground_gain_max_range = 10;
        mgg::computeExplorationGain(graph, setup.ctx, true, true);
        const auto sparse_path = mgg::selectBestPath(graph, setup.planning, robot,
            mgg::EdgeInclinations{}, resolution, 0);
        EXPECT_GE(graph.getVertex(3)->vol_gain.num_unknown_voxels,
                  setup.planning.low_gain_voxels);
        EXPECT_GE(sparse_path.best_gain, mgg::TourParams{}.min_cluster_gain);
        EXPECT_EQ(sparse_path.best_path_id, 3);
        setup.planning.ground_gain_angular_resolution_deg = 0;
        setup.planning.ground_gain_max_range = 0;
        mgg::test::benchmarkGainModels("corridor_r" + std::to_string(resolution) +
            "_mount" + std::to_string(platform.first) + "_vfov" + std::to_string(vfov),
            graph, setup.ctx, true);
      }
    }
  }
}

class TallRoomGain : public ::testing::TestWithParam<double> {};
TEST_P(TallRoomGain, UnknownCeilingDoesNotCompeteWithTheDoor) {
  std::vector<Cell> occupied, free;
  // Observed floor, observed air to 1.2 m, upper hangar air unknown.
  // The wall at x=6 has a 2 m doorway, an observed exit at x=6.5,
  // then unknown at x>=7. The selected path must actually cross the door.
  for (int x = -100; x < 100; ++x) {
    for (int y = -100; y < 100; ++y) {
      occupied.push_back({x, y, -1});
      if (x >= 35) continue;
      for (int z = 0; z < 6; ++z) {
        (x == 30 && (y < -5 || y >= 5) ? occupied : free).push_back({x, y, z});
      }
    }
  }
  mgg::NativeMolaGrid map(kResolution, occupied, free, {});
  GainSetup setup(map);
  mgg::RobotParams robot;
  robot.size = Eigen::Vector3d(0.8, 0.5, 0.6);
  setup.ctx.robot = &robot;
  setup.planning.max_ground_height = 0.5;
  auto& sensor = setup.sensors["VLP16"];
  sensor.max_range = GetParam();
  sensor.update();
  mgg::GraphManager graph;
  for (const auto& [id, pos] : std::vector<std::pair<int, Eigen::Vector2d>>{
           {0, {0.1, 0.1}}, {1, {4.1, 0.1}}, {2, {5.1, 0.1}},
           {3, {0.1, 3.1}}, {4, {6.5, 0.1}}}) {
    graph.addVertex(new mgg::Vertex(id, mgg::StateVec(pos.x(), pos.y(), 0.5, 0)));
  }
  graph.addEdge(graph.getVertex(0), graph.getVertex(1), 4);
  graph.addEdge(graph.getVertex(1), graph.getVertex(2), 1);
  graph.addEdge(graph.getVertex(0), graph.getVertex(3), 3);
  graph.addEdge(graph.getVertex(2), graph.getVertex(4), 1.4);
  const auto start = std::chrono::steady_clock::now();
  mgg::computeExplorationGain(graph, setup.ctx, false, true);
  const auto scored = std::chrono::steady_clock::now();
  for (int id : {0, 3}) {
    const auto& gain = graph.getVertex(id)->vol_gain;
    EXPECT_GT(expectFullScanEquivalent(graph.getVertex(id)->state, setup)
                  .num_total_unknown_voxels, 0);
    EXPECT_EQ(gain.num_total_unknown_voxels, -1);
    if (GetParam() == 3.0) {
      EXPECT_EQ(gain.num_unknown_voxels, 0);
      EXPECT_FALSE(gain.is_frontier);
      EXPECT_DOUBLE_EQ(gain.gain, 0);
    }
    EXPECT_LT(gain.gain, graph.getVertex(4)->vol_gain.gain);
  }
  for (int id : {1, 2, 4})
    expectFullScanEquivalent(graph.getVertex(id)->state, setup);
  EXPECT_TRUE(graph.getVertex(2)->vol_gain.is_frontier);
  const auto selection_start = std::chrono::steady_clock::now();
  const auto selected = mgg::selectBestPath(graph, setup.planning, robot,
                                            mgg::EdgeInclinations{}, 0.2, 0);
  std::cout << "room range=" << GetParam()
            << " gain_ms=" << std::chrono::duration<double, std::milli>(scored - start).count()
            << " selection_ms=" << std::chrono::duration<double, std::milli>(
                 std::chrono::steady_clock::now() - selection_start).count() << '\n';
  EXPECT_EQ(selected.best_path_id, 4);
  ASSERT_FALSE(selected.best_path.empty());
  EXPECT_GT(selected.best_path.back()->state.x(), 6.2);  // outside the room
  for (std::size_t i = 1; i < selected.best_path.size(); ++i) {
    EXPECT_EQ(map.getPathStatus(selected.best_path[i - 1]->state.head<3>(),
                                selected.best_path[i]->state.head<3>(),
                                robot.size, true), VoxelStatus::kFree);
  }
  // A separate long-range room/door ranking comparison at hardware vertical FOV.
  setup.sensors["VLP16"].fov.y() = M_PI / 3;
  setup.sensors["VLP16"].update();
  setup.planning.ground_gain_angular_resolution_deg = 7.5;
  setup.planning.ground_gain_max_range = 10;
  mgg::computeExplorationGain(graph, setup.ctx, false, true);
  const auto sparse_path = mgg::selectBestPath(graph, setup.planning, robot,
      mgg::EdgeInclinations{}, kResolution, 0);
  EXPECT_EQ(sparse_path.best_path_id, 4);
  if (GetParam() == 3.0) {
    EXPECT_FALSE(graph.getVertex(0)->vol_gain.is_frontier);
    EXPECT_FALSE(graph.getVertex(3)->vol_gain.is_frontier);
  }
  setup.planning.ground_gain_angular_resolution_deg = 0;
  setup.planning.ground_gain_max_range = 0;
  mgg::test::benchmarkGainModels("room_door_range" + std::to_string(GetParam()),
                                 graph, setup.ctx, false);

}

INSTANTIATE_TEST_SUITE_P(SensorRange, TallRoomGain, ::testing::Values(3.0, 20.0));

// Review r0 (P1): with distinct voxels counted, the frontier test still
// divided by rays x range. At 0.5 degree steps and 1 m range that is 64800,
// and every distinct voxel within 1 m, times 0.2, came to under 0.006 of
// it: no frontier in space that is all unknown. Each voxel is now measured
// against the distinct voxels the rays can reach.
TEST(NativeGain, AnAerialViewOfUnknownSpaceIsAFrontierAtAnyResolution) {
  mgg::NativeMolaGrid unknown_space(kResolution, {}, {}, {});
  std::vector<Cell> free;
  for (std::int64_t x = -8; x < 8; ++x)
    for (std::int64_t y = -8; y < 8; ++y)
      for (std::int64_t z = -8; z < 8; ++z) free.push_back({x, y, z});
  mgg::NativeMolaGrid mapped_space(kResolution, {}, free, {});
  for (const double degrees : {0.5, 1.0, 2.0, 5.0, 10.0}) {
    for (const double range : {1.0, 5.0}) {
      if (degrees < 1.0 && range > 1.0) continue;  // 64800 rays: slow
      GainSetup unknown_setup(unknown_space);
      mgg::SensorParams& sensor = unknown_setup.sensors["VLP16"];
      sensor.max_range = range;
      sensor.resolution = Eigen::Vector2d::Constant(degrees * M_PI / 180.0);
      sensor.frontier_percentage_threshold = 0.05;
      sensor.update();
      mgg::VolumetricGain gain;
      mgg::computeVolumetricGain(mgg::StateVec(0.1, 0.1, 0.1, 0.0), gain,
                                 unknown_setup.ctx);
      EXPECT_TRUE(gain.is_frontier) << degrees << " degrees, " << range << " m";

      if (range > 1.0) continue;  // the mapped block is 3.2 m across
      GainSetup mapped_setup(mapped_space);
      mapped_setup.sensors["VLP16"] = sensor;
      mgg::VolumetricGain mapped;
      mgg::computeVolumetricGain(mgg::StateVec(0.1, 0.1, 0.1, 0.0), mapped,
                                 mapped_setup.ctx);
      EXPECT_FALSE(mapped.is_frontier) << degrees << " degrees";
    }
  }
}

// Review r1 (P2): the frontier denominator walked each ray one axis at a
// time, while the native gain walks every voxel a ray touches, edges and
// corners included; a ray through an edge counted four voxels against five.
// From a voxel centre, a scan of all-unknown space now counts exactly the
// denominator, tied rays included.
TEST(NativeGain, TheFrontierDenominatorIsWhatAnAllUnknownScanCounts) {
  mgg::NativeMolaGrid unknown_space(kResolution, {}, {}, {});
  struct Sensor {
    double vertical_fov, step, range;
  };
  // The simulated VLP16; 45 degree steps, whose rays at (+-45, -45) pass
  // through voxel edges and corners, short and long; and 0.5 degree steps.
  for (const Sensor& s : {Sensor{M_PI / 4.0, M_PI / 36.0, 20.0},
                          Sensor{M_PI / 2.0, M_PI / 4.0, 0.4},
                          Sensor{M_PI / 2.0, M_PI / 4.0, 3.0},
                          Sensor{M_PI / 4.0, M_PI / 360.0, 1.0}}) {
    GainSetup setup(unknown_space);
    mgg::SensorParams& sensor = setup.sensors["VLP16"];
    sensor.fov = Eigen::Vector2d(2.0 * M_PI, s.vertical_fov);
    sensor.resolution = Eigen::Vector2d::Constant(s.step);
    sensor.max_range = s.range;
    sensor.update();
    mgg::VolumetricGain gain;
    mgg::computeVolumetricGain(mgg::StateVec(0.1, 0.1, 0.1, 0.0), gain,
                               setup.ctx);
    EXPECT_EQ(gain.num_unknown_voxels,
              static_cast<int>(sensor.uniqueVoxelsFullFov(kResolution)))
        << s.step * 180.0 / M_PI << " degrees, " << s.range << " m";
  }
}

// Includes boundary-origin and tied rays, a mount above the band, blockers
// before band entry, and tilted authority planes. Out-of-band output is
// permitted, but the complete in-band voxel set must match the reference.
TEST(NativeGain, BandScanPreservesTiesTiltAndOccludingPrefixes) {
  std::vector<Cell> occupied;
  for (int x = -15; x < 15; ++x)
    for (int y = -15; y < 15; ++y) {
      if (x > 0) occupied.push_back({x, y, 8});
      if (y < 0) occupied.push_back({x, y, -3});
    }
  mgg::NativeMolaGrid map(0.2, occupied, {}, {});
  GainSetup setup(map);
  setup.sensor.fov.y() = M_PI;
  setup.sensor.resolution = Eigen::Vector2d::Constant(M_PI / 12);
  setup.sensor.max_range = 8;
  setup.sensor.update();
  for (const Eigen::Vector3d origin : {Eigen::Vector3d(0, 0, 0),
                                      Eigen::Vector3d(0.1, 0.1, 2.1),
                                      Eigen::Vector3d(0.037, -0.181, -2.1)}) {
    for (double tilt : {0.0, 0.03, 0.7}) {
      const Eigen::Vector3d normal = Eigen::AngleAxisd(
          tilt, Eigen::Vector3d(1, 2, 3).normalized()) * Eigen::Vector3d::UnitZ();
      std::vector<Eigen::Vector3d> endpoints;
      setup.sensor.getFrustumEndpoints(
          mgg::StateVec(origin.x(), origin.y(), origin.z(), 0.0), endpoints);
      mgg::GainCounts full, pruned;
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> a, b;
      map.getScanStatusIterative(origin, endpoints, full, a, setup.sensor.model());
      mgg::ScanBounds bounds;
      bounds.bounds_from_map.linear() = Eigen::Quaterniond::FromTwoVectors(
          normal, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      bounds.low.z() = -0.4;
      bounds.high.z() = 0.8;
      map.getScanStatusInBounds(origin, endpoints, pruned, b,
                                setup.sensor.model(), bounds);
      auto band = [&](const auto& entries) {
        std::set<std::tuple<double, double, double, int>> result;
        for (const auto& e : entries) {
          const double height = normal.dot(e.first);
          if (height >= -0.4 && height <= 0.8)
            result.emplace(e.first.x(), e.first.y(), e.first.z(), int(e.second));
        }
        return result;
      };
      EXPECT_EQ(band(a), band(b)) << "origin=" << origin.transpose() << " tilt=" << tilt;
      EXPECT_LT(pruned.voxel_visits, full.voxel_visits);
    }
  }
}

TEST(NativeGain, ScanWalkMatchesReferenceClosedVoxelSetsAndOcclusion) {
  std::mt19937 random(74823);
  for (int trial = 0; trial < 1200; ++trial) {
    auto point = [&]() {
      Eigen::Vector3d p;
      for (int axis = 0; axis < 3; ++axis) {
        p[axis] = (int(random() % 41) - 20) * 0.2;
        if (trial % 3 == 1) p[axis] += 0.1;
        if (trial % 3 == 2) p[axis] = std::nextafter(p[axis], 100.0);
      }
      return p;
    };
    const Eigen::Vector3d a = point();
    const Eigen::Vector3d b = trial % 17 == 0 ? a : point();
    for (bool blockers : {false, true}) {
      std::set<std::tuple<std::int64_t, std::int64_t, std::int64_t>> full, fast;
      auto visitor = [&](auto& cells, const mgg::VoxelIndex& cell) {
        cells.emplace(cell.x, cell.y, cell.z);
        return !blockers || (cell.x * 71 + cell.y * 37 + cell.z * 13) % 23 != 0;
      };
      const bool full_valid = mgg::walkVoxels(a, b, 0.2, 1u << 22,
          [&](const auto& cell) { return visitor(full, cell); });
      const bool fast_valid = mgg::walkScanVoxels(a, b, 0.2, 1u << 22,
          [&](const auto& cell) { return visitor(fast, cell); });
      EXPECT_EQ(full_valid, fast_valid);
      EXPECT_EQ(full, fast) << "trial=" << trial << " blockers=" << blockers;
    }
  }
}

// mgg-integ: the band scan's walker must keep the request deadline's
// throttled per-voxel checkpoint (mgg-astar), as the full scan's walk()
// does: one long ray cannot outrun cancellation between rays.
TEST(NativeGain, BandScanWalkKeepsThrottledCancellationCheckpoints) {
  mgg::NativeMolaGrid map(kResolution, {}, {}, {});
  GainSetup setup(map);
  const Eigen::Vector3d origin(0.1, 0.1, 0.1);  // a cell centre: no DDA ties
  const std::vector<Eigen::Vector3d> ray{origin + Eigen::Vector3d(40, 0, 0)};
  mgg::ScanBounds bounds;
  bounds.low.z() = -0.4;
  bounds.high.z() = 0.8;
  for (bool pruned : {false, true}) {
    SCOPED_TRACE(pruned ? "band scan" : "full scan");
    mgg::GainCounts counts;
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> log;
    auto scan = [&] {
      if (pruned)
        map.getScanStatusInBounds(origin, ray, counts, log, setup.sensor.model(), bounds);
      else
        map.getScanStatusIterative(origin, ray, counts, log, setup.sensor.model());
    };
    int checks = 0;
    {
      mgg::PlanningCancellationScope count([&] { ++checks; return false; });
      scan();
    }
    // 201 cells; the scan walker may revisit its end cell (scan_walk.h).
    EXPECT_GE(counts.voxel_visits, 201u);
    EXPECT_LE(counts.voxel_visits, 202u);
    EXPECT_EQ(checks, 1 + 4);  // the ray, then voxels 0, 64, 128 and 192
    checks = 0;
    log.clear();
    // Expire after the ray's own check: the walk must stop by voxel 64.
    mgg::PlanningCancellationScope expire([&] { return ++checks > 2; });
    EXPECT_THROW(scan(), mgg::PlanningInterrupted);
    EXPECT_EQ(checks, 3);
    EXPECT_LE(log.size(), 65u);
  }
}

TEST(NativeGain, RotatedGainRegionsAndExclusionsRemainExact) {
  auto map = terrain([](std::int64_t, std::int64_t)
                         -> std::optional<std::int64_t> { return -1; });
  GainSetup setup(map);
  mgg::RobotParams robot;
  robot.size.z() = 0.245;
  setup.ctx.robot = &robot;
  setup.planning.max_ground_height = 0.5;
  setup.planning.free_voxel_gain = 3;
  setup.planning.occupied_voxel_gain = 7;
  setup.space.min_val = Eigen::Vector3d(-2, -1, -1);
  setup.space.max_val = Eigen::Vector3d(5, 2, 3);
  setup.space.rotations = Eigen::Vector3d(0.37, 0.03, 0.01);
  setup.space.min_extension = Eigen::Vector3d(-1, -0.4, -0.1);
  setup.space.max_extension = Eigen::Vector3d(0.5, 1, 0.2);
  setup.space.setCenter(Eigen::Vector3d(0.1, 0.3, 0), true);
  std::vector<mgg::BoundedSpaceParams> excluded{setup.space};
  excluded[0].min_val = Eigen::Vector3d(2, -1, 0);
  excluded[0].max_val = Eigen::Vector3d(3, 1, 2);
  excluded[0].setCenter(Eigen::Vector3d::Zero().eval(), false);
  setup.ctx.no_gain_zones = &excluded;
  mgg::BoundedSpaceParams interest = setup.space;
  interest.type = mgg::BoundedSpaceType::kSphere;
  interest.radius = 4;
  interest.setCenter(Eigen::Vector3d(1, 0, 0), false);
  setup.ctx.gain_region = &interest;
  for (double yaw : {0.0, 0.23, M_PI / 4})
    expectFullScanEquivalent(mgg::StateVec(0.1, 0.3, 0.5, yaw), setup);
}

}  // namespace
