// Dirty regions are how a map update tells every cached result whether it
// must be withdrawn. Reach is inclusive: a change that only touches a
// dependency's face still withdraws it.

#include <gtest/gtest.h>

#include <unordered_set>

#include "mgg_core/dirty_region.h"

namespace {

using mgg::MapChange;
using mgg::VoxelKey;

Eigen::AlignedBox3d cube(const Eigen::Vector3d& center, double edge) {
  const Eigen::Vector3d half = Eigen::Vector3d::Constant(edge / 2);
  return Eigen::AlignedBox3d(center - half, center + half);
}

TEST(DirtyRegion, TouchAndOutside) {
  // The dependency ends at the boundary x = 0.2.
  const Eigen::AlignedBox3d dep(Eigen::Vector3d(0.0, 0.0, 0.0),
                                Eigen::Vector3d(0.2, 0.2, 0.2));
  const double boundary = 0.2;
  MapChange touch;
  touch.boxes.push_back(cube({boundary + 0.1, 0.1, 0.1}, 0.2));
  MapChange outside;
  outside.boxes.push_back(cube({boundary + 0.2, 0.1, 0.1}, 0.2));
  EXPECT_TRUE(mgg::changeReaches(touch, dep));
  EXPECT_FALSE(mgg::changeReaches(outside, dep));

  // An empty change reaches nothing; `everything` reaches everything.
  EXPECT_FALSE(mgg::changeReaches(MapChange{}, dep));
  MapChange everything;
  everything.everything = true;
  EXPECT_TRUE(mgg::changeReaches(everything, dep));

  // The halo grows each box on every side: the outside change, 0.1 m away,
  // reaches the dependency once grown by 0.15 m.
  const auto grown = mgg::dirtyRegions(outside, 0.15);
  ASSERT_EQ(grown.size(), 1u);
  EXPECT_TRUE(grown[0].intersects(dep));
  const auto all = mgg::dirtyRegions(everything, 0.1);
  ASSERT_EQ(all.size(), 1u);
  EXPECT_TRUE(all[0].contains(Eigen::Vector3d(1e6, -1e6, 1e6)));
}

TEST(DirtyRegion, WorldAnchoredKeys) {
  const double res = 0.2;
  const VoxelKey negative = mgg::keyOf({-0.05, 0.05, -0.2}, res);
  EXPECT_EQ(negative, (VoxelKey{-1, 0, -1}));
  EXPECT_TRUE(mgg::centerOf(negative, res).isApprox(
      Eigen::Vector3d(-0.1, 0.1, -0.1)));
  EXPECT_EQ(mgg::keyOf(mgg::centerOf(VoxelKey{7, -3, 2}, res), res),
            (VoxelKey{7, -3, 2}));
  std::unordered_set<VoxelKey, mgg::VoxelKeyHash> keys{negative, negative};
  EXPECT_EQ(keys.size(), 1u);
}

}  // namespace
