// Volumetric gain (mgg_core/gain.cpp) evaluated on the native SDMGRID1 grid:
// the gain counts only the unknown space a viewpoint could see.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "mgg_core/gain.h"
#include "mgg_map_octomap/native_mola_grid.h"

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
/// vertex 0.45 m over its floor at z = 0.05, with a 0.2 m robot, so the wall
/// band runs from z = 0.25 to the top of the gain band at z = 1.7.
std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> groundGain(
    mgg::MapInterface& map, const Eigen::Vector3d& viewpoint) {
  GainSetup setup(map);
  mgg::RobotParams robot;
  robot.type = mgg::RobotType::kGroundRobot;
  robot.size = Eigen::Vector3d(0.5, 0.5, 0.3);
  setup.ctx.robot = &robot;
  setup.planning.robot_height = 0.2;
  setup.planning.max_ground_height = 0.45;
  mgg::VolumetricGain gain;
  std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> counted;
  mgg::computeVolumetricGain(
      mgg::StateVec(viewpoint.x(), viewpoint.y(), viewpoint.z(), 0.0), gain,
      setup.ctx, &counted);
  return counted;
}

/// A floor at z = [-0.2, 0), air mapped free up to x = 2.0 and z = 2.0, and
/// at x = [2.0, 2.2) the occupied rows `rows(y)` of each column: a wall,
/// a sill or a rail. Beyond it the map is unknown. Everything runs 20 m to
/// either side, so that no 20 m ray passes its ends.
mgg::NativeMolaGrid street(
    const std::function<std::vector<std::int64_t>(std::int64_t)>& rows) {
  std::vector<Cell> occupied, free;
  for (std::int64_t y = -100; y < 100; ++y) {
    for (std::int64_t x = -9; x < 110; ++x) occupied.push_back({x, y, -1});
    for (std::int64_t z : rows(y)) occupied.push_back({10, y, z});
    for (std::int64_t x = -9; x < 10; ++x)
      for (std::int64_t z = 0; z < 10; ++z) free.push_back({x, y, z});
  }
  return mgg::NativeMolaGrid(kResolution, occupied, free, {});
}

/// Unknown voxels counted beyond x = 2.2 and above `height`.
int beyondAndAbove(
    const std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& counted,
    double height) {
  return static_cast<int>(std::count_if(
      counted.begin(), counted.end(), [height](const auto& entry) {
        return entry.second == VoxelStatus::kUnknown &&
               entry.first.x() > 2.2 && entry.first.z() > height;
      }));
}

// Diagnosis 2026-09-24 (diag-viewpoint, Q2): only occupied voxels stopped a
// gain ray, and a wall keeps unknown gaps between the voxels its lidar
// returns landed in. Rays went through them and counted the space behind the
// wall, 0.39 of the count at the captured plan ends.
TEST(NativeGain, WallGapsHideWhatIsBehindAndADoorwayDoesNot) {
  // The wall's returns landed at every other row up to 1.8 m, leaving
  // unknown gaps at z = [0.4, 0.6), [0.8, 1.0) and [1.2, 1.4). Columns over
  // y = [0, 1) hold no return: a doorway.
  const Eigen::Vector3d viewpoint(0.1, 0.5, 0.5);
  auto map = street([](std::int64_t y) {
    return y >= 0 && y < 5 ? std::vector<std::int64_t>{}
                           : std::vector<std::int64_t>{0, 1, 3, 5, 7, 8};
  });
  int through_doorway = 0, through_gaps = 0;
  for (const auto& entry : groundGain(map, viewpoint)) {
    const Eigen::Vector3d& v = entry.first;
    if (entry.second != VoxelStatus::kUnknown || v.x() < 2.2) continue;
    // Where the line of sight crosses the wall; a cell of slack either side
    // of the doorway for the rays grazing its edges.
    const double crossing = viewpoint.y() + (v.y() - viewpoint.y()) *
                                                (2.1 - viewpoint.x()) /
                                                (v.x() - viewpoint.x());
    if (crossing >= -0.2 && crossing <= 1.2)
      ++through_doorway;
    else
      ++through_gaps;
  }
  EXPECT_EQ(through_gaps, 0);
  EXPECT_GT(through_doorway, 200);
}

// Review r0 (P1): a single return in a column made the whole column a wall,
// so rays over a window sill, a railing or a rising ramp saw nothing beyond.
// Only a gap between two returns hides what is behind it.
TEST(NativeGain, OpenSpaceOverASillARailingAndARampStaysVisible) {
  const Eigen::Vector3d viewpoint(0.1, 0.5, 0.5);
  // A sill whose top is at 0.6 m.
  auto sill = street([](std::int64_t) {
    return std::vector<std::int64_t>{0, 1, 2};
  });
  EXPECT_GT(beyondAndAbove(groundGain(sill, viewpoint), 0.6), 200);
  // A rail at z = [0.6, 0.8), and nothing under it but the floor.
  auto railing = street([](std::int64_t) {
    return std::vector<std::int64_t>{3};
  });
  const auto past_railing = groundGain(railing, viewpoint);
  EXPECT_GT(beyondAndAbove(past_railing, 0.8), 100);
  EXPECT_GT(beyondAndAbove(past_railing, 0.0) -
                beyondAndAbove(past_railing, 0.6), 100);

  // A 16 degree ramp rising from x = 0.6: one occupied voxel per column at
  // its surface, air mapped free over it up to x = 2.0, unknown beyond.
  const double slope = std::tan(16.0 * M_PI / 180.0);
  const auto surface_row = [slope](std::int64_t x) {
    const double rise = std::max(0.0, ((x + 0.5) * kResolution - 0.6) * slope);
    return static_cast<std::int64_t>(std::floor(rise / kResolution));
  };
  std::vector<Cell> occupied, free;
  for (std::int64_t y = -100; y < 100; ++y)
    for (std::int64_t x = -9; x < 110; ++x) {
      const std::int64_t top = x < 3 ? -1 : surface_row(x);
      occupied.push_back({x, y, top});
      if (x < 10)
        for (std::int64_t z = top + 1; z < 10; ++z) free.push_back({x, y, z});
    }
  mgg::NativeMolaGrid ramp(kResolution, occupied, free, {});
  int over_ramp = 0;
  for (const auto& entry : groundGain(ramp, viewpoint)) {
    const Eigen::Vector3d& v = entry.first;
    const std::int64_t x = std::lround(std::floor(v.x() / kResolution));
    if (entry.second == VoxelStatus::kUnknown && v.x() > 2.2 &&
        v.z() > (surface_row(x) + 1) * kResolution)
      ++over_ramp;
  }
  EXPECT_GT(over_ramp, 100);
}

// Diagnosis 2026-09-24 (diag-viewpoint, Q2): the gain counted voxels down to
// 1.0 m below the vertex, about 0.55 m below the floor for the simulated
// robots, where nothing can be seen (0.10 of the count at the captured plan
// ends). A vertex rides max_ground_height above its ground, and the gain
// stops one voxel below that ground.
TEST(NativeGain, NothingMoreThanAVoxelBelowTheFloorCounts) {
  mgg::NativeMolaGrid map(kResolution, {}, {}, {});
  GainSetup setup(map);
  mgg::RobotParams robot;
  robot.type = mgg::RobotType::kGroundRobot;
  setup.ctx.robot = &robot;
  setup.planning.robot_height = 0.2;
  setup.planning.max_ground_height = 0.45;

  // The floor under this vertex is at z = 0.05.
  const double vertex_z = 0.5;
  const double floor_z = vertex_z - setup.planning.max_ground_height;
  mgg::VolumetricGain gain;
  std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> counted;
  mgg::computeVolumetricGain(mgg::StateVec(0.1, 0.1, vertex_z, 0.0), gain,
                             setup.ctx, &counted);

  ASSERT_FALSE(counted.empty());
  double lowest = std::numeric_limits<double>::infinity();
  for (const auto& entry : counted) lowest = std::min(lowest, entry.first.z());
  // Voxel centres: the one straddling the floor, [0.0, 0.2), counts, and the
  // one below it, [-0.2, 0.0), is within a voxel of the floor.
  EXPECT_GE(lowest, floor_z - kResolution - 1e-9);
  EXPECT_LE(lowest, floor_z);
}

}  // namespace
