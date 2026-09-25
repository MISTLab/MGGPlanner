// Volumetric gain (mgg_core/gain.cpp) evaluated on the native SDMGRID1 grid:
// the gain counts only the unknown space a viewpoint could see.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
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

// Diagnosis 2026-09-24 (diag-viewpoint, Q2): only occupied voxels stopped a
// gain ray, and a wall keeps unknown gaps between the voxels its lidar
// returns landed in. Rays went through them and counted the space behind the
// wall, 0.39 of the count at the captured plan ends.
TEST(NativeGain, WallGapsHideWhatIsBehindAndADoorwayDoesNot) {
  // Free space up to a wall over x = [2.0, 2.2) whose returns landed only at
  // z = [0.4, 0.6); the rest of each wall column is unknown. Columns over
  // y = [0, 1) hold no return: a doorway. Beyond the wall is unknown. The
  // wall runs 20 m to either side, so that no 20 m ray passes its ends.
  std::vector<Cell> occupied, free;
  for (std::int64_t y = -100; y < 100; ++y) {
    if (y < 0 || y > 4) occupied.push_back({10, y, 2});
    for (std::int64_t x = -9; x < 10; ++x)
      for (std::int64_t z = -2; z < 10; ++z) free.push_back({x, y, z});
  }
  mgg::NativeMolaGrid map(kResolution, occupied, free, {});
  GainSetup setup(map);
  mgg::RobotParams robot;
  robot.type = mgg::RobotType::kGroundRobot;
  robot.size = Eigen::Vector3d(0.5, 0.5, 0.3);
  setup.ctx.robot = &robot;
  setup.planning.robot_height = 0.2;
  setup.planning.max_ground_height = 0.45;

  // The body rides at z = [0.35, 0.65], level with the returns.
  const Eigen::Vector3d viewpoint(0.1, 0.5, 0.5);
  mgg::VolumetricGain gain;
  std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> counted;
  mgg::computeVolumetricGain(
      mgg::StateVec(viewpoint.x(), viewpoint.y(), viewpoint.z(), 0.0), gain,
      setup.ctx, &counted);

  int through_doorway = 0, through_gaps = 0;
  for (const auto& entry : counted) {
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

}  // namespace
