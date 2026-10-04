// The rolling voxel map shared by ground and drone local planning: a fixed
// window centred on the robot, voxels forgotten as they scroll out, and a
// MapChange per call naming every region whose ternary state changed.

#include <gtest/gtest.h>

#include <vector>

#include "mgg_core/dirty_region.h"
#include "mgg_map_octomap/rolling_voxel_map.h"
#include "synthetic_scan.h"

namespace {

using mgg::MapChange;
using mgg::RollingVoxelMap;
using mgg::RollingWindowParams;
using mgg::VoxelKey;
using mgg::VoxelStatus;

// Cell centres of the 0.2 m grid, so axis-parallel rays never run along a
// voxel face.
const Eigen::Vector3d kRobot(0.1, 0.1, 1.1);

Eigen::AlignedBox3d probe(const Eigen::Vector3d& center) {
  const Eigen::Vector3d half = Eigen::Vector3d::Constant(0.05);
  return Eigen::AlignedBox3d(center - half, center + half);
}

TEST(RollingVoxelMap, RollAndForget) {
  RollingVoxelMap map(RollingWindowParams{});
  const Eigen::Vector3d wall(-6.9, 0.1, 1.1);
  const Eigen::Vector3d evicted(-5.9, 0.1, 1.1);
  map.insertScan({wall}, kRobot, kRobot);
  ASSERT_EQ(map.getVoxelStatus(evicted), VoxelStatus::kFree);
  ASSERT_EQ(map.getVoxelStatus(wall), VoxelStatus::kOccupied);

  // Window [-8, 8] in x becomes [-4, 12].
  const MapChange change = map.recenter({4.1, 0.1, 1.1});
  EXPECT_EQ(map.getVoxelStatus(evicted), VoxelStatus::kUnknown);
  EXPECT_FALSE(change.boxes.empty());
  EXPECT_TRUE(mgg::changeReaches(change, probe(evicted)));
  EXPECT_EQ(change.revision, map.revision());

  // Forgotten, not hidden: scrolling back does not bring them back.
  map.recenter(kRobot);
  EXPECT_EQ(map.getVoxelStatus(evicted), VoxelStatus::kUnknown);
  EXPECT_EQ(map.getVoxelStatus(wall), VoxelStatus::kUnknown);
}

TEST(RollingVoxelMap, EnteringSlabAndInclusiveBounds) {
  RollingVoxelMap map(RollingWindowParams{});
  map.recenter(kRobot);
  const Eigen::AlignedBox3d before = map.window();
  EXPECT_NEAR(before.min().x(), -8.0, 1e-9);
  EXPECT_NEAR(before.max().x(), 8.0, 1e-9);

  // A scan taken 1 m further along x, with one return beyond the window.
  const Eigen::Vector3d moved = kRobot + Eigen::Vector3d(1.0, 0, 0);
  const MapChange change =
      map.insertScan({Eigen::Vector3d(15.1, 0.1, 1.1)}, moved, moved);
  EXPECT_NEAR(map.window().min().x(), -7.0, 1e-9);
  EXPECT_NEAR(map.window().max().x(), 9.0, 1e-9);

  // Probes away from the ray, so only the slabs can reach them.
  const auto entering_slab = probe({8.5, -5.0, 2.5});
  const auto evicted_slab = probe({-7.5, -5.0, 2.5});
  const auto untouched = probe({3.5, -5.0, 2.5});
  EXPECT_TRUE(mgg::changeReaches(change, entering_slab));
  EXPECT_TRUE(mgg::changeReaches(change, evicted_slab));
  EXPECT_FALSE(mgg::changeReaches(change, untouched));

  // The window is closed: its max face belongs to its last cell.
  const Eigen::Vector3d on_max_bound(map.window().max().x(), 0.1, 1.1);
  EXPECT_EQ(map.getVoxelStatus(on_max_bound), VoxelStatus::kFree);
  EXPECT_EQ(map.getVoxelStatus(on_max_bound + Eigen::Vector3d(1e-6, 0, 0)),
            VoxelStatus::kUnknown);
}

TEST(RollingVoxelMap, HitOnMaxFaceMarksLastVoxel) {
  // A 4 m window of 1 m voxels around (0.5, 0.5, 0.5) spans [-2, 2] on each
  // axis; a return exactly on a max face belongs to the last voxel.
  RollingWindowParams params;
  params.resolution = 1.0;
  params.window_size_m = Eigen::Vector3d(4, 4, 4);
  const Eigen::Vector3d robot(0.5, 0.5, 0.5);
  for (int axis = 0; axis < 3; ++axis) {
    SCOPED_TRACE(axis);
    RollingVoxelMap map(params);
    Eigen::Vector3d beyond = robot;
    beyond[axis] = 3.5;
    map.insertScan({beyond}, robot, robot);
    Eigen::Vector3d last_voxel = robot;
    last_voxel[axis] = 1.5;
    ASSERT_EQ(map.getVoxelStatus(last_voxel), VoxelStatus::kFree);

    Eigen::Vector3d on_max_face = robot;
    on_max_face[axis] = map.window().max()[axis];
    const std::uint64_t before = map.revision();
    const MapChange change = map.insertScan({on_max_face}, robot, robot);
    EXPECT_EQ(map.getVoxelStatus(last_voxel), VoxelStatus::kOccupied);
    EXPECT_EQ(map.getVoxelStatus(on_max_face), VoxelStatus::kOccupied);
    EXPECT_GT(change.revision, before);
    EXPECT_TRUE(mgg::changeReaches(change, probe(last_voxel)));
  }
}

TEST(RollingVoxelMap, StableKeys) {
  RollingVoxelMap map(RollingWindowParams{});
  const Eigen::Vector3d p(-3.33, 2.71, 0.47);
  map.insertScan({p}, kRobot, kRobot);
  const VoxelKey key_before_scroll = map.keyOf(p);
  ASSERT_EQ(map.getVoxelStatus(map.centerOf(key_before_scroll)),
            VoxelStatus::kOccupied);

  // Scroll on two axes, keeping p inside; the ring buffer wraps.
  map.recenter({1.7, -1.3, 1.1});
  map.recenter({1.9, -1.9, 1.1});
  ASSERT_TRUE(map.window().contains(p));
  EXPECT_EQ(map.keyOf(p), key_before_scroll);
  const Eigen::Vector3d keyOf_center = mgg::centerOf(
      mgg::keyOf(p, map.getResolution()), map.getResolution());
  EXPECT_TRUE(map.centerOf(key_before_scroll).isApprox(keyOf_center));
  EXPECT_EQ(map.getVoxelStatus(map.centerOf(key_before_scroll)),
            VoxelStatus::kOccupied);
}

TEST(RollingVoxelMap, VerticalPolicyAndHalo) {
  RollingVoxelMap ground(RollingWindowParams{});
  ground.recenter(kRobot);
  const double old_z = ground.window().min().z();
  ground.recenter({0.1, 0.1, 4.1});
  EXPECT_EQ(ground.window().min().z(), old_z);

  RollingWindowParams drone_params;
  drone_params.follow_vertical = true;
  RollingVoxelMap drone(drone_params);
  drone.recenter(kRobot);
  drone.recenter({0.1, 0.1, 4.0});
  EXPECT_NEAR(drone.window().center().z(), 4, 0.2);

  // One ray one voxel long: the origin voxel turns free, the next occupied.
  const MapChange change = ground.insertScan(
      {Eigen::Vector3d(0.1, 0.1, 1.3)}, kRobot, kRobot);
  ASSERT_EQ(change.boxes.size(), 1u);
  const Eigen::Vector3d neighbour(0.1, 0.1, 1.5);
  EXPECT_FALSE(mgg::changeReaches(change, probe(neighbour)));
  EXPECT_TRUE(mgg::dirtyRegions(change, 0.2)[0].contains(neighbour));
  EXPECT_FALSE(mgg::dirtyRegions(change, 0.2)[0].contains(
      Eigen::Vector3d(0.1, 0.1, 1.7)));
}

TEST(RollingVoxelMap, StableScanAndSingleContraryRay) {
  RollingVoxelMap map(RollingWindowParams{});
  mgg::test::SyntheticScene scene;
  scene.solids.emplace_back(Eigen::Vector3d(3.05, -3.0, -0.5),
                            Eigen::Vector3d(3.45, 3.0, 3.0));
  const auto points = mgg::test::syntheticScan(scene, kRobot);
  ASSERT_FALSE(points.empty());
  for (int i = 0; i < 3; ++i) map.insertScan(points, kRobot, kRobot);
  const Eigen::Vector3d wall_voxel(3.1, 0.1, 1.1);
  ASSERT_EQ(map.getVoxelStatus(wall_voxel), VoxelStatus::kOccupied);

  const std::uint64_t before = map.revision();
  const MapChange repeated = map.insertScan(points, kRobot, kRobot);
  EXPECT_EQ(repeated.revision, before);
  EXPECT_TRUE(repeated.boxes.empty());
  EXPECT_FALSE(repeated.everything);

  // Grazing flicker: one return passes straight through the wall voxel.
  map.insertScan({Eigen::Vector3d(5.1, 0.1, 1.1)}, kRobot, kRobot);
  const VoxelStatus after_one_miss = map.getVoxelStatus(wall_voxel);
  EXPECT_EQ(after_one_miss, VoxelStatus::kOccupied);
}

TEST(RollingVoxelMap, ResetForgetsEverything) {
  RollingVoxelMap map(RollingWindowParams{});
  const Eigen::Vector3d wall(2.1, 0.1, 1.1);
  map.insertScan({wall}, kRobot, kRobot);
  ASSERT_TRUE(map.getStatus());
  const std::uint64_t before = map.revision();
  const MapChange change = map.reset({0.3, 0.1, 3.1});
  EXPECT_TRUE(change.everything);
  EXPECT_GT(change.revision, before);
  EXPECT_EQ(map.getVoxelStatus(wall), VoxelStatus::kUnknown);
  // Reset re-places the ground window vertically as well.
  EXPECT_NEAR(map.window().center().z(), 3.1, 0.2);
}

TEST(RollingVoxelMap, MapInterfaceQueries) {
  RollingVoxelMap map(RollingWindowParams{});
  EXPECT_FALSE(map.getStatus());
  const Eigen::Vector3d wall(2.1, 0.1, 1.1);
  map.insertScan({wall}, kRobot, kRobot);
  EXPECT_TRUE(map.getStatus());
  EXPECT_DOUBLE_EQ(map.getResolution(), 0.2);

  Eigen::Vector3d end;
  EXPECT_EQ(map.getRayStatus(kRobot, Eigen::Vector3d(3.1, 0.1, 1.1), false,
                             end),
            VoxelStatus::kOccupied);
  EXPECT_TRUE(end.isApprox(wall));
  EXPECT_EQ(map.getRayStatus(kRobot, Eigen::Vector3d(1.9, 0.1, 1.1), true),
            VoxelStatus::kFree);
  const Eigen::Vector3d cell = Eigen::Vector3d::Constant(0.1);
  EXPECT_EQ(map.getBoxStatus({1.1, 0.1, 1.1}, cell, true), VoxelStatus::kFree);
  EXPECT_EQ(map.getBoxStatus({2.1, 0.1, 1.1}, cell, true),
            VoxelStatus::kOccupied);
  EXPECT_EQ(map.getBoxStatus({1.1, 0.5, 1.1}, cell, true),
            VoxelStatus::kUnknown);
  EXPECT_EQ(map.getBoxStatus({1.1, 0.5, 1.1}, cell, false),
            VoxelStatus::kFree);
  EXPECT_EQ(map.getPathStatus(kRobot, {1.9, 0.1, 1.1}, cell, true),
            VoxelStatus::kFree);
  EXPECT_EQ(map.getPathStatus(kRobot, {2.5, 0.1, 1.1}, cell, false),
            VoxelStatus::kOccupied);
  // Beyond the window everything is unknown.
  EXPECT_EQ(map.getVoxelStatus({30.1, 0.1, 1.1}), VoxelStatus::kUnknown);
  EXPECT_EQ(map.getBoxStatus({30.1, 0.1, 1.1}, cell, true),
            VoxelStatus::kUnknown);

  Eigen::Vector2d center;
  ASSERT_TRUE(map.getAxisAlignedXYCellCenter({-0.05, 0.31}, center));
  EXPECT_TRUE(center.isApprox(Eigen::Vector2d(-0.1, 0.3)));

  std::vector<Eigen::Vector3d> occupied, free;
  map.extractLocalMap(kRobot, Eigen::Vector3d::Constant(6.0), occupied, free);
  ASSERT_EQ(occupied.size(), 1u);
  EXPECT_TRUE(occupied[0].isApprox(wall));
  EXPECT_EQ(free.size(), 10u);
  // A thin box along the ray holds it all; turned across the ray, only the
  // robot's own voxel.
  const Eigen::Vector3d thin(6.0, 0.3, 0.3);
  map.extractLocalMapAlongAxis(kRobot, Eigen::Vector3d::UnitX(), thin,
                               occupied, free);
  EXPECT_EQ(occupied.size(), 1u);
  EXPECT_EQ(free.size(), 10u);
  map.extractLocalMapAlongAxis(kRobot, Eigen::Vector3d::UnitY(), thin,
                               occupied, free);
  EXPECT_EQ(occupied.size(), 0u);
  EXPECT_EQ(free.size(), 1u);
  map.extractLocalMapAlongAxis(kRobot, -Eigen::Vector3d::UnitX(), thin,
                               occupied, free);
  EXPECT_EQ(occupied.size(), 1u);
  EXPECT_EQ(free.size(), 10u);

  mgg::GainCounts gain;
  std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> log;
  map.getScanStatusIterative(kRobot, {Eigen::Vector3d(3.1, 0.1, 1.1)}, gain,
                             log, mgg::SensorModel{});
  EXPECT_EQ(gain.occupied, 1);
  EXPECT_EQ(gain.free, 10);
  EXPECT_EQ(gain.unknown, 0);

  // Mutations outside a scan would bypass MapChange and stay unsupported.
  EXPECT_FALSE(map.augmentFreeBox(kRobot, Eigen::Vector3d::Constant(1.0)));
}

}  // namespace
