#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include "mgg_map_octomap/cell_index.h"
#include "mgg_map_octomap/native_mola_grid.h"

namespace {
using Cell = mgg::NativeMolaGrid::Cell;
using Grid = mgg::NativeMolaGrid;

int reference(const std::vector<Cell>& occupied, const std::vector<Cell>& free,
              const Cell& cell) {
  if (std::binary_search(occupied.begin(), occupied.end(), cell)) return 2;
  if (std::binary_search(free.begin(), free.end(), cell)) return 1;
  return 0;
}

TEST(NativeMolaIndex, MatchesBinarySearchOnRandomAndTunnelCells) {
  std::vector<Cell> occupied, free;
  std::mt19937_64 rng(76543);
  for (int i = 0; i < 12000; ++i) {
    Cell c{static_cast<std::int64_t>(rng() % 4000) - 2000,
           static_cast<std::int64_t>(rng() % 4000) - 2000,
           static_cast<std::int64_t>(rng() % 60) - 30};
    (i % 3 ? free : occupied).push_back(c);
  }
  for (int x = -300; x < 300; ++x)
    for (int y = -3; y <= 3; ++y)
      for (int z = -2; z <= 2; ++z)
        (y == -3 || y == 3 || z == -2 ? occupied : free).push_back({x, y, z});
  // High-bit coordinates, duplicates and occupied/free overlap.
  const auto huge = std::numeric_limits<std::int64_t>::max() / 4 - 4096;
  occupied.push_back({huge, -huge, huge});
  occupied.push_back({-huge, huge, -huge});
  occupied.push_back({0, 0, 0});
  occupied.push_back({0, 0, 0});
  free.push_back({0, 0, 0});
  free.push_back({huge, -huge, huge});
  std::sort(occupied.begin(), occupied.end());
  std::sort(free.begin(), free.end());
  mgg::CellIndex<Cell> index(occupied, free);
  std::vector<Cell> queries = occupied;
  queries.insert(queries.end(), free.begin(), free.end());
  for (int i = 0; i < 100000; ++i) {
    queries.push_back({static_cast<std::int64_t>(rng() % 4200) - 2100,
                       static_cast<std::int64_t>(rng() % 4200) - 2100,
                       static_cast<std::int64_t>(rng() % 70) - 35});
  }
  queries.push_back({huge - 1, -huge, huge});
  queries.push_back({std::numeric_limits<std::int64_t>::min(), 0, 0});
  for (const Cell& c : queries)
    EXPECT_EQ(index.status(c, occupied, free), reference(occupied, free, c))
        << c.x << "," << c.y << "," << c.z;

  Grid grid(1.0, occupied, free, {});
  EXPECT_EQ(grid.getVoxelStatus({0.5, 0.5, 0.5}), mgg::VoxelStatus::kOccupied);
  // Near the key() limit doubles are rounded; direct index queries above
  // exercise exact high-bit keys without rounding them through Eigen.
}

TEST(NativeMolaIndex, EmptyRebuildDiscardsOldSlots) {
  const std::vector<Cell> occupied{{1, 2, 3}};
  const std::vector<Cell> empty;
  mgg::CellIndex<Cell> index(occupied, empty);
  ASSERT_GT(index.bytes(), 0u);
  index.build(empty, empty);
  ASSERT_EQ(index.bytes(), 0u);
  EXPECT_EQ(index.status({1, 2, 3}, empty, empty), 0);
}

TEST(NativeMolaIndex, RaysAndMeasuredSurfacesPreserveClosedCellResults) {
  Grid grid(1.0, {{0, 0, -1}, {3, 0, 0}, {1, 0, 0}},
            {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}},
            {{{0, 0, -1}, -0.6}, {{0, 0, -1}, -0.4},
             {{3, 0, 0}, 0.1}, {{3, 0, 0}, 0.9}});
  Eigen::Vector3d hit;
  EXPECT_EQ(grid.getRayStatus({0.5, 0.5, 0.5}, {3.5, 0.5, 0.5}, false, hit),
            mgg::VoxelStatus::kOccupied);
  EXPECT_TRUE(hit.isApprox(Eigen::Vector3d(1.5, 0.5, 0.5)));
  EXPECT_EQ(grid.getRayStatus({3.5, 0.5, 0.5}, {0.5, 0.5, 0.5}, false, hit),
            mgg::VoxelStatus::kOccupied);
  EXPECT_TRUE(hit.isApprox(Eigen::Vector3d(3.5, 0.5, 0.5)));
  EXPECT_EQ(grid.getRayStatus({0.5, 0.5, 0.5}, {0.5, 2.5, 0.5}, true, hit),
            mgg::VoxelStatus::kUnknown);
  EXPECT_TRUE(hit.isApprox(Eigen::Vector3d(0.5, 1.5, 0.5)));
  EXPECT_EQ(grid.getGroundRayStatus({0.5, 0.5, 0.5}, {0.5, 0.5, -0.5}, false, hit),
            mgg::VoxelStatus::kOccupied);
  EXPECT_DOUBLE_EQ(hit.z(), -0.4);
  EXPECT_EQ(grid.getBoxStatus({0.5, 0.5, 0.5}, {0.4, 0.4, 0.4}, false),
            mgg::VoxelStatus::kFree);
  EXPECT_EQ(grid.getStrictBoxStatus({1.5, 0.5, 0.5}, {0.4, 0.4, 0.4}),
            mgg::VoxelStatus::kOccupied);
}
}  // namespace
