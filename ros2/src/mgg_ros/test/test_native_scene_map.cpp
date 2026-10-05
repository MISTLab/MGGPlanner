#include <gtest/gtest.h>

#include "native_scene_map.h"

using mgg::VoxelStatus;
using mgg_test::NativeSceneMap;

TEST(NativeSceneMap, PersistentObservationsSurviveDistantScansAndReset) {
  NativeSceneMap map({0.1});
  EXPECT_FALSE(map.getStatus());
  map.insertPointCloud({{0.05, 0.05, 0.05}}, {0.05, 0.05, 1.05});
  map.insertPointCloud({{40.05, 0.05, 0.05}}, {40.05, 0.05, 1.05});
  EXPECT_TRUE(map.getStatus());
  EXPECT_EQ(map.getVoxelStatus({0.05, 0.05, 0.05}), VoxelStatus::kOccupied);
  EXPECT_EQ(map.getVoxelStatus({40.05, 0.05, 0.05}), VoxelStatus::kOccupied);
  EXPECT_EQ(map.getVoxelStatus({0.05, 0.05, 0.55}), VoxelStatus::kFree);
  EXPECT_EQ(map.getVoxelStatus({20.05, 0.05, 0.55}), VoxelStatus::kUnknown);
  map.resetMap();
  EXPECT_FALSE(map.getStatus());
  EXPECT_EQ(map.getVoxelStatus({0.05, 0.05, 0.05}), VoxelStatus::kUnknown);
}

TEST(NativeSceneMap, ExplicitFreeObservationReplacesPriorOccupancy) {
  NativeSceneMap map({0.1});
  map.insertPointCloud({{0.05, 0.05, 0.05}}, {0.05, 0.05, 1.05});
  EXPECT_EQ(map.getVoxelStatus({0.05, 0.05, 0.05}), VoxelStatus::kOccupied);
  ASSERT_TRUE(map.augmentFreeBox({0.05, 0.05, 0.05}, {0.4, 0.4, 0.4}));
  EXPECT_EQ(map.getVoxelStatus({0.05, 0.05, 0.05}), VoxelStatus::kFree);
  EXPECT_EQ(map.getVoxelStatus({0.15, 0.15, 0.15}), VoxelStatus::kFree);
  EXPECT_EQ(map.getStrictBoxStatus({0.05, 0.05, 0.05}, {0.1, 0.1, 0.1}),
            VoxelStatus::kFree);
}

TEST(NativeSceneMap, MeasuredGroundAndStrictQueriesUseNativeGridSemantics) {
  NativeSceneMap map({0.1});
  map.setTrackMeasuredSurfaceZ(true);
  map.insertPointCloud({{0.05, 0.05, 0.03}}, {0.05, 0.05, 0.25});
  mgg::NativeMolaGrid reference(0.1, {{0, 0, 0}}, {{0, 0, 1}, {0, 0, 2}},
                               {{{0, 0, 0}, 0.03}});
  Eigen::Vector3d actual, expected;
  EXPECT_EQ(map.getGroundRayStatus({0.05, 0.05, 0.25}, {0.05, 0.05, -0.05},
                                    true, actual),
            reference.getGroundRayStatus({0.05, 0.05, 0.25},
                                           {0.05, 0.05, -0.05}, true, expected));
  EXPECT_TRUE(actual.isApprox(expected));
  EXPECT_DOUBLE_EQ(actual.z(), 0.03);
  for (double x : {0.05, 0.15}) {
    EXPECT_EQ(map.getStrictBoxStatus({x, 0.05, 0.15}, {0.01, 0.01, 0.01}),
              reference.getStrictBoxStatus({x, 0.05, 0.15}, {0.01, 0.01, 0.01}));
  }
}
