// Volumetric gain (mgg_core/gain.cpp) evaluated on the native SDMGRID1 grid:
// the gain counts only the unknown space a viewpoint could see.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
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
/// band runs from z = 0.25 to the top of the gain band at z = 1.7, or to
/// `gain_max_height_above_ground` over the floor.
std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> groundGain(
    mgg::MapInterface& map, const Eigen::Vector3d& viewpoint,
    double gain_max_height_above_ground = 0.0) {
  GainSetup setup(map);
  mgg::RobotParams robot;
  robot.type = mgg::RobotType::kGroundRobot;
  robot.size = Eigen::Vector3d(0.5, 0.5, 0.3);
  setup.ctx.robot = &robot;
  setup.planning.robot_height = 0.2;
  setup.planning.max_ground_height = 0.45;
  setup.planning.gain_max_height_above_ground = gain_max_height_above_ground;
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

// Review r1 (P1), accepted as a minimum opening height: a framed opening of
// at most kMaxWallGapVoxels (0.6 m at 0.2 m) is taken for a gap between
// lidar rings in a wall, and hides what is beyond it; a taller one does not.
TEST(NativeGain, FramedOpeningsUpToTheWallGapLimitHideWhatIsBeyond) {
  static_assert(mgg::kMaxWallGapVoxels == 3);
  const Eigen::Vector3d viewpoint(0.1, 0.5, 0.5);
  // Returns in every row from the wall base to z = 1.8 but the opening's.
  const auto framed = [](std::int64_t low, std::int64_t high) {
    return street([low, high](std::int64_t) {
      std::vector<std::int64_t> rows;
      for (std::int64_t z = 0; z < 9; ++z)
        if (z < low || z > high) rows.push_back(z);
      return rows;
    });
  };
  // 0.4 m, z = [0.4, 0.8), and 0.6 m, z = [0.4, 1.0): wall gaps.
  auto low_window = framed(2, 3);
  EXPECT_EQ(beyondAndAbove(groundGain(low_window, viewpoint), -1.0), 0);
  auto limit_window = framed(2, 4);
  EXPECT_EQ(beyondAndAbove(groundGain(limit_window, viewpoint), -1.0), 0);
  // 0.8 m, z = [0.4, 1.2): a window, seen through.
  auto window = framed(2, 5);
  EXPECT_GT(beyondAndAbove(groundGain(window, viewpoint), -1.0), 200);
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

// Run 6: the planner grid leaves most of the air above 0.8 m unknown in a
// room explored end to end, and a ground robot's gain rays found fresh
// unknown voxels there from every new viewpoint: the explored hangar
// outscored the corridors to unexplored space. A ground robot counts its
// gain only up to gain_max_height_above_ground over its floor; an aerial
// robot counts the whole view.
TEST(NativeGain, AGroundRobotCountsGainOnlyUpToItsBandOverTheFloor) {
  // A floor at z = [-0.2, 0) and unknown air over it; the vertex's floor is
  // at z = 0.05, so 0.8 m over it is z = 0.85.
  auto floor = terrain([](std::int64_t, std::int64_t)
                           -> std::optional<std::int64_t> { return -1; });
  const Eigen::Vector3d viewpoint(0.1, 0.5, 0.5);
  const auto highest = [](const auto& counted) {
    double top = -1e9;
    for (const auto& entry : counted) {
      if (entry.second == VoxelStatus::kUnknown) {
        top = std::max(top, entry.first.z());
      }
    }
    return top;
  };
  const auto capped = groundGain(floor, viewpoint, 0.8);
  ASSERT_FALSE(capped.empty());
  EXPECT_LE(highest(capped), 0.85);
  EXPECT_GT(highest(capped), 0.6);
  // 0: up to the band's top, 1.2 m over the vertex.
  const auto uncapped = groundGain(floor, viewpoint, 0.0);
  EXPECT_GT(highest(uncapped), 1.4);
  EXPECT_LE(highest(uncapped), 1.7);
  EXPECT_GT(uncapped.size(), 2 * capped.size());

  // An aerial robot's view is not capped.
  GainSetup aerial(floor);
  mgg::RobotParams drone;
  drone.type = mgg::RobotType::kAerialRobot;
  aerial.ctx.robot = &drone;
  aerial.planning.gain_max_height_above_ground = 0.8;
  mgg::VolumetricGain gain;
  std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> counted;
  mgg::computeVolumetricGain(
      mgg::StateVec(viewpoint.x(), viewpoint.y(), viewpoint.z(), 0.0), gain,
      aerial.ctx, &counted);
  EXPECT_GT(highest(counted), 2.0);
}

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

}  // namespace
