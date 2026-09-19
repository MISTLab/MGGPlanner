#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_map_octomap/native_mola_grid.h"
#include "mgg_map_octomap/traversability_builder.h"

namespace {

using mgg::NativeMolaGrid;
using mgg::RasterCellState;
using mgg::RasterParams;
using mgg::TraversabilityRaster;

using Cell = NativeMolaGrid::Cell;
using Surface = NativeMolaGrid::Surface;

constexpr double kRes = 0.2;

/// A hand-built scene: a flat road from x = -2 to 6 m and y = -2 to 2 m at
/// 0.05 m, a 0.20 m kerb from x = 3 m on (aligned to the 0.5 m raster), a
/// 1.2 m wall along y in [1.6, 1.8) for x in [0, 2), an unobserved hole at
/// x in [1.0, 1.5), y in [-0.6, 0) (columns 5 and 6 by -3 to -1), and a
/// table top 0.9 m above the road at x in [4.0, 4.5), y in [0, 0.5).
struct Scene {
  std::vector<Cell> occupied;
  std::vector<Cell> free;
  std::vector<Surface> surfaces;

  void column(std::int64_t x, std::int64_t y, std::int64_t z, double max_z) {
    occupied.push_back({x, y, z});
    surfaces.push_back({{x, y, z}, max_z});
  }
  void freeVoxel(std::int64_t x, std::int64_t y, std::int64_t z) {
    free.push_back({x, y, z});
  }
};

Scene roadScene(double kerb_edge_x = 3.0) {
  Scene scene;
  for (std::int64_t x = -10; x < 30; ++x) {
    for (std::int64_t y = -10; y < 10; ++y) {
      const double wx = kRes * (static_cast<double>(x) + 0.5);
      const double wy = kRes * (static_cast<double>(y) + 0.5);
      if (x >= 5 && x <= 6 && y >= -3 && y <= -1) continue;  // hole
      if (wx >= kerb_edge_x) {
        scene.column(x, y, 1, 0.25);  // kerb top in voxel [0.2, 0.4)
      } else {
        scene.column(x, y, 0, 0.05);  // road in voxel [0.0, 0.2)
      }
      if (wy > 1.6 && wy < 1.8 && wx > 0.0 && wx < 2.0) {
        for (std::int64_t z = 0; z < 6; ++z) {
          scene.column(x, y, z, kRes * static_cast<double>(z + 1) - 0.01);
        }
      }
      if (wx > 4.0 && wx < 4.5 && wy > 0.0 && wy < 0.5) {
        scene.column(x, y, 4, 0.90);  // table top in voxel [0.8, 1.0)
      }
    }
  }
  return scene;
}

NativeMolaGrid grid(const Scene& scene) {
  return NativeMolaGrid(kRes, scene.occupied, scene.free, scene.surfaces);
}

RasterParams params() {
  RasterParams value;
  value.cell_size_m = 0.5;
  value.body_radius_m = 0.4;
  value.max_step_height_m = 0.15;
  value.body_height_m = 1.2;
  value.min_clearance_m = 0.0;
  value.step_tolerance_m = 0.01;
  return value;
}

TEST(TraversabilityBuilder, GroundHeightsKerbAndUnknownColumns) {
  const NativeMolaGrid map = grid(roadScene());
  const auto raster = mgg::buildTraversabilityRaster(map, params());
  ASSERT_NE(raster, nullptr);
  EXPECT_TRUE(raster->consistent());
  EXPECT_NEAR(raster->cell_size, 0.5, 1e-12);
  // The raster is aligned to whole cells with a one-cell margin.
  EXPECT_NEAR(raster->origin.x(), -2.5, 1e-9);
  EXPECT_NEAR(raster->origin.y(), -2.5, 1e-9);
  EXPECT_NEAR(std::fmod(raster->origin.x(), 0.5), 0.0, 1e-9);

  // Road cells are free at the road height.
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(0.25, 0.25)), RasterCellState::kFree);
  EXPECT_NEAR(raster->groundAtXY(Eigen::Vector2d(0.25, 0.25)), 0.05, 1e-9);
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(-1.75, -1.75)),
            RasterCellState::kFree);
  // The kerb top is free ground 0.20 m higher; the planner judges the step.
  // (Probed away from the table top, whose inflation reaches y < 0.5.)
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(3.75, -0.75)),
            RasterCellState::kFree);
  EXPECT_NEAR(raster->groundAtXY(Eigen::Vector2d(3.75, -0.75)), 0.25, 1e-9);
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(2.75, -0.75)),
            RasterCellState::kFree);
  EXPECT_NEAR(raster->groundAtXY(Eigen::Vector2d(2.75, -0.75)), 0.05, 1e-9);
  // The table top's inflation does reach the kerb cell beside it.
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(3.75, 0.25)),
            RasterCellState::kInflated);
  // The hole has no surface: unknown with NaN ground.
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(1.25, -0.25)),
            RasterCellState::kUnknown);
  EXPECT_TRUE(std::isnan(raster->groundAtXY(Eigen::Vector2d(1.25, -0.25))));
  // The margin and everything beyond the road are unknown.
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(-2.25, 0.25)),
            RasterCellState::kUnknown);
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(6.25, 0.25)),
            RasterCellState::kUnknown);
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(50.0, 50.0)),
            RasterCellState::kUnknown);
}

TEST(TraversabilityBuilder, WallIsAnObstacleInflatedByTheBodyRadius) {
  const NativeMolaGrid map = grid(roadScene());
  const auto raster = mgg::buildTraversabilityRaster(map, params());
  ASSERT_NE(raster, nullptr);
  // The wall cells themselves.
  for (const double x : {0.25, 0.75, 1.25, 1.75}) {
    EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(x, 1.75)),
              RasterCellState::kObstacle)
        << x;
    // Obstacle cells keep the ground they were classified against.
    EXPECT_NEAR(raster->groundAtXY(Eigen::Vector2d(x, 1.75)), 0.05, 1e-9);
  }
  // The ring within 0.4 m of the wall's squares is inflated: the cells below
  // and beside it, including the diagonal corners. They keep their ground.
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(0.75, 1.25)),
            RasterCellState::kInflated);
  EXPECT_NEAR(raster->groundAtXY(Eigen::Vector2d(0.75, 1.25)), 0.05, 1e-9);
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(-0.25, 1.75)),
            RasterCellState::kInflated);
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(2.25, 1.25)),
            RasterCellState::kInflated);
  // Two cells away the road is free again.
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(0.75, 0.75)),
            RasterCellState::kFree);
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(-0.75, 1.75)),
            RasterCellState::kFree);
  // Inflation never invents ground: the cell above the wall is beyond the
  // road and stays unknown.
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(0.75, 2.25)),
            RasterCellState::kUnknown);

  // Without inflation only the wall cells are obstacles.
  RasterParams bare = params();
  bare.body_radius_m = 0.0;
  const auto tight = mgg::buildTraversabilityRaster(map, bare);
  ASSERT_NE(tight, nullptr);
  EXPECT_EQ(tight->stateAtXY(Eigen::Vector2d(0.75, 1.75)),
            RasterCellState::kObstacle);
  EXPECT_EQ(tight->stateAtXY(Eigen::Vector2d(0.75, 1.25)), RasterCellState::kFree);
  EXPECT_EQ(tight->stateAtXY(Eigen::Vector2d(-0.25, 1.75)),
            RasterCellState::kFree);
}

TEST(TraversabilityBuilder, TableTopOverTheRoadFollowsTheBodyBand) {
  const NativeMolaGrid map = grid(roadScene());
  // A 1.2 m body meets the table top: obstacle, ground still the road.
  RasterParams tall = params();
  tall.body_radius_m = 0.0;
  const auto blocked = mgg::buildTraversabilityRaster(map, tall);
  ASSERT_NE(blocked, nullptr);
  EXPECT_EQ(blocked->stateAtXY(Eigen::Vector2d(4.25, 0.25)),
            RasterCellState::kObstacle);
  EXPECT_NEAR(blocked->groundAtXY(Eigen::Vector2d(4.25, 0.25)), 0.25, 1e-9);
  // A 0.5 m body passes beneath the table: free.
  RasterParams low = tall;
  low.body_height_m = 0.5;
  const auto beneath = mgg::buildTraversabilityRaster(map, low);
  ASSERT_NE(beneath, nullptr);
  EXPECT_EQ(beneath->stateAtXY(Eigen::Vector2d(4.25, 0.25)),
            RasterCellState::kFree);
  EXPECT_NEAR(beneath->groundAtXY(Eigen::Vector2d(4.25, 0.25)), 0.25, 1e-9);
}

TEST(TraversabilityBuilder, KerbInsideOneCellIsAnObstacleAboveTheStepBand) {
  // A kerb edge at x = 3.2 m splits the cell [3.0, 3.5): the gutter columns
  // give the ground and the kerb-top voxel rises 0.20 m above it.
  const NativeMolaGrid map = grid(roadScene(3.2));
  RasterParams scout = params();
  scout.body_radius_m = 0.0;
  const auto raster = mgg::buildTraversabilityRaster(map, scout);
  ASSERT_NE(raster, nullptr);
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(3.25, 0.25)),
            RasterCellState::kObstacle);
  EXPECT_NEAR(raster->groundAtXY(Eigen::Vector2d(3.25, 0.25)), 0.05, 1e-9);
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(3.75, 0.25)), RasterCellState::kFree);
  // A platform that climbs 0.30 m sees terrain, not an obstacle.
  RasterParams spot = scout;
  spot.max_step_height_m = 0.30;
  const auto climbs = mgg::buildTraversabilityRaster(map, spot);
  ASSERT_NE(climbs, nullptr);
  EXPECT_EQ(climbs->stateAtXY(Eigen::Vector2d(3.25, 0.25)), RasterCellState::kFree);
  // A platform that climbs 0.15 m but may drop 0.25 m sees a step: marked
  // like an inflated cell, crossed at a cost, never a wall.
  RasterParams bunker = scout;
  bunker.max_drop_height_m = 0.25;
  const auto steps = mgg::buildTraversabilityRaster(map, bunker);
  ASSERT_NE(steps, nullptr);
  EXPECT_EQ(steps->stateAtXY(Eigen::Vector2d(3.25, 0.25)),
            RasterCellState::kInflated);
  EXPECT_NEAR(steps->groundAtXY(Eigen::Vector2d(3.25, 0.25)), 0.05, 1e-9);
  EXPECT_EQ(steps->stateAtXY(Eigen::Vector2d(3.75, 0.25)), RasterCellState::kFree);
}

TEST(TraversabilityBuilder, TransformPlacesTheRasterInTheNavigationFrame) {
  const NativeMolaGrid map = grid(roadScene());
  Eigen::Isometry3d raster_from_grid = Eigen::Isometry3d::Identity();
  raster_from_grid.linear() =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  raster_from_grid.translation() = Eigen::Vector3d(10.0, 20.0, 0.5);
  const auto raster =
      mgg::buildTraversabilityRaster(map, params(), raster_from_grid);
  ASSERT_NE(raster, nullptr);
  // The road point (0.25, 0.25) in the grid lands at (10 - 0.25, 20 + 0.25).
  const Eigen::Vector3d road =
      raster_from_grid * Eigen::Vector3d(0.25, 0.25, 0.0);
  EXPECT_EQ(raster->stateAtXY(road.head<2>()), RasterCellState::kFree);
  EXPECT_NEAR(raster->groundAtXY(road.head<2>()), 0.55, 1e-9);
  const Eigen::Vector3d kerb =
      raster_from_grid * Eigen::Vector3d(3.75, -0.75, 0.0);
  EXPECT_EQ(raster->stateAtXY(kerb.head<2>()), RasterCellState::kFree);
  EXPECT_NEAR(raster->groundAtXY(kerb.head<2>()), 0.75, 1e-9);
  const Eigen::Vector3d wall = raster_from_grid * Eigen::Vector3d(0.75, 1.75, 0.0);
  EXPECT_EQ(raster->stateAtXY(wall.head<2>()), RasterCellState::kObstacle);
  // The untransformed location is outside the raster now.
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(0.25, 0.25)),
            RasterCellState::kUnknown);

  // A tilt beyond the authority tolerance is refused.
  Eigen::Isometry3d tilted = Eigen::Isometry3d::Identity();
  tilted.linear() =
      Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()).toRotationMatrix();
  EXPECT_EQ(mgg::buildTraversabilityRaster(map, params(), tilted), nullptr);
}

TEST(TraversabilityBuilder, ClearanceRequiresObservedFreeVoxelsAboveTheGround) {
  Scene scene = roadScene();
  // Observed air above the road at x in [0.5, 1.0), y in [0.5, 1.0): three
  // free layers above the ground voxel, 0.6 m of clearance.
  for (std::int64_t x = 2; x < 5; ++x) {
    for (std::int64_t y = 2; y < 5; ++y) {
      for (std::int64_t z = 1; z < 4; ++z) scene.freeVoxel(x, y, z);
    }
  }
  // Only one layer at x in [-1.0, -0.5), y in [0.5, 1.0).
  for (std::int64_t x = -5; x < -2; ++x) {
    for (std::int64_t y = 2; y < 5; ++y) scene.freeVoxel(x, y, 1);
  }
  const NativeMolaGrid map = grid(scene);
  RasterParams strict = params();
  strict.body_radius_m = 0.0;
  strict.min_clearance_m = 0.5;
  const auto raster = mgg::buildTraversabilityRaster(map, strict);
  ASSERT_NE(raster, nullptr);
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(0.75, 0.75)), RasterCellState::kFree);
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(-0.75, 0.75)),
            RasterCellState::kUnknown);
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(0.25, 0.25)),
            RasterCellState::kUnknown);
  // Ground is still known where the air is not.
  EXPECT_NEAR(raster->groundAtXY(Eigen::Vector2d(0.25, 0.25)), 0.05, 1e-9);
  // Obstacles do not depend on clearance evidence.
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(0.75, 1.75)),
            RasterCellState::kObstacle);
  // Without the requirement the same cells are free.
  RasterParams lenient = strict;
  lenient.min_clearance_m = 0.0;
  const auto loose = mgg::buildTraversabilityRaster(map, lenient);
  ASSERT_NE(loose, nullptr);
  EXPECT_EQ(loose->stateAtXY(Eigen::Vector2d(-0.75, 0.75)), RasterCellState::kFree);
}

TEST(TraversabilityBuilder, InvalidInputsGiveNoRaster) {
  const NativeMolaGrid empty(kRes, {}, {}, {});
  EXPECT_EQ(mgg::buildTraversabilityRaster(empty, params()), nullptr);
  const NativeMolaGrid map = grid(roadScene());
  RasterParams bad = params();
  bad.cell_size_m = 0.0;
  EXPECT_EQ(mgg::buildTraversabilityRaster(map, bad), nullptr);
  bad = params();
  bad.body_height_m = 0.05;  // below the step band
  EXPECT_EQ(mgg::buildTraversabilityRaster(map, bad), nullptr);
  bad = params();
  bad.body_radius_m = -1.0;
  EXPECT_EQ(mgg::buildTraversabilityRaster(map, bad), nullptr);
  // A grid whose voxels lie absurdly far apart would exceed the cell bound.
  const NativeMolaGrid vast(kRes, {{0, 0, 0}, {100000000, 100000000, 0}}, {},
                            {{{0, 0, 0}, 0.1}, {{100000000, 100000000, 0}, 0.1}});
  EXPECT_EQ(mgg::buildTraversabilityRaster(vast, params()), nullptr);
}

TEST(TraversabilityBuilder, BuildsALargeProductWellUnderAHundredMilliseconds) {
  // 300 x 300 columns of road (90,000 occupied voxels) with three surface
  // samples per voxel (300,000 surface records) and a few walls.
  std::vector<Cell> occupied;
  std::vector<Surface> surfaces;
  std::vector<Cell> free;
  occupied.reserve(90000 + 3000);
  surfaces.reserve(300000 + 3000);
  for (std::int64_t x = -150; x < 150; ++x) {
    for (std::int64_t y = -150; y < 150; ++y) {
      const double z = 0.01 * static_cast<double>((x + y) % 7);
      occupied.push_back({x, y, 0});
      surfaces.push_back({{x, y, 0}, 0.03 + z});
      surfaces.push_back({{x, y, 0}, 0.05 + z});
      surfaces.push_back({{x, y, 0}, 0.04 + z});
      if (x % 50 == 0 && std::abs(y) < 40) {
        for (std::int64_t k = 1; k < 6; ++k) {
          occupied.push_back({x, y, k});
          surfaces.push_back({{x, y, k}, kRes * static_cast<double>(k + 1) - 0.01});
        }
      }
    }
  }
  const auto construct_started = std::chrono::steady_clock::now();
  const NativeMolaGrid map(kRes, occupied, free, surfaces);
  const auto construct_elapsed =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - construct_started);
  const auto started = std::chrono::steady_clock::now();
  const auto raster = mgg::buildTraversabilityRaster(map, params());
  const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - started);
  ASSERT_NE(raster, nullptr);
  std::cout << "[ raster   ] " << raster->width << " x " << raster->height
            << " cells from " << map.occupiedCells().size()
            << " occupied voxels and " << surfaces.size()
            << " surface records: build " << elapsed.count()
            << " us (grid construction " << construct_elapsed.count() << " us)"
            << std::endl;
  EXPECT_LT(elapsed.count(), 100000) << "raster build took " << elapsed.count()
                                     << " us";
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(5.25, 5.25)), RasterCellState::kFree);
  EXPECT_EQ(raster->stateAtXY(Eigen::Vector2d(0.05, 0.25)),
            RasterCellState::kObstacle);
  std::size_t free_cells = 0;
  std::size_t obstacle_cells = 0;
  for (const RasterCellState state : raster->state) {
    free_cells += state == RasterCellState::kFree;
    obstacle_cells += state == RasterCellState::kObstacle;
  }
  EXPECT_GT(free_cells, 10000u);
  EXPECT_GT(obstacle_cells, 100u);
}

}  // namespace
