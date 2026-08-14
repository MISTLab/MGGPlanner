// Tests for sensor geometry, including a regression guard for the
// uninitialised-rotation trap the ROS 1 version carried.

#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/sensor_params.h"

namespace {

using mgg::SensorParams;
using mgg::SensorType;
using mgg::StateVec;

/// The VLP-16 the shipped MGG configs describe: 360 x 30 degrees, 20 m,
/// 5 degree steps.
SensorParams makeVlp16() {
  SensorParams s;
  s.type = SensorType::kLidar;
  s.max_range = 20.0;
  s.fov = Eigen::Vector2d(2.0 * M_PI, M_PI / 6.0);
  s.resolution = Eigen::Vector2d(5.0 * M_PI / 180.0, 5.0 * M_PI / 180.0);
  s.width = 72;
  s.height = 6;
  s.frontier_percentage_threshold = 0.05;
  s.update();
  return s;
}

// REGRESSION: in the ROS 1 SensorParamsBase the rotation matrices were only
// computed inside loadParams. A hand-built struct therefore multiplied by
// uninitialised Eigen memory, producing garbage ray endpoints; raycasting to
// them tried an astronomical allocation and aborted with std::bad_alloc.
// Here an un-updated sensor must be inert, not dangerous.
TEST(SensorParams, UnupdatedSensorIsInertNotGarbage) {
  SensorParams s;  // deliberately no update()
  EXPECT_FALSE(s.isReady());

  std::vector<Eigen::Vector3d> endpoints;
  s.getFrustumEndpoints(StateVec(0, 0, 0, 0), endpoints);
  EXPECT_TRUE(endpoints.empty());

  // Must not divide by an uninitialised denominator either.
  EXPECT_FALSE(s.isFrontier(1000.0));
}

TEST(SensorParams, RayCountMatchesTheRos1Implementation) {
  const SensorParams s = makeVlp16();
  EXPECT_TRUE(s.isReady());
  std::vector<Eigen::Vector3d> endpoints;
  s.getFrustumEndpoints(StateVec(0, 0, 0, 0), endpoints);

  // 438, not the 432 that 360/5 x 30/5 would suggest. The loop condition is
  // "for (dh = -h_lim; dh < h_lim; dh += h_res)", and accumulating 5 degree
  // steps across a full turn lands just short of +h_lim, so a 73rd azimuth
  // step fits. The ROS 1 build produces the same 438 (see the phase 0 baseline
  // log), so this pins the port to the original rather than to the arithmetic
  // one would expect.
  EXPECT_EQ(endpoints.size(), 73u * 6u);
}

TEST(SensorParams, EveryRayIsExactlyMaxRange) {
  const SensorParams s = makeVlp16();
  const Eigen::Vector3d origin(3.0, -2.0, 1.0);
  std::vector<Eigen::Vector3d> endpoints;
  s.getFrustumEndpoints(StateVec(origin.x(), origin.y(), origin.z(), 0.0),
                        endpoints);
  ASSERT_FALSE(endpoints.empty());

  // The ROS 1 code built rays as max_range * (cos dh, sin dh, sin dv), whose
  // length is max_range * sqrt(1 + sin^2 dv): rays at the vertical extremes
  // overshot by about 3% and the swept volume was barrel-shaped rather than a
  // spherical cap. The spherical form is used instead, so every ray now ends
  // exactly max_range from the sensor.
  for (const auto& e : endpoints) {
    EXPECT_NEAR((e - origin).norm(), 20.0, 1e-9);
  }
}

TEST(SensorParams, RangeScaleShortensTheRays) {
  const SensorParams s = makeVlp16();
  std::vector<Eigen::Vector3d> full, half;
  s.getFrustumEndpoints(StateVec(0, 0, 0, 0), full);
  s.getFrustumEndpoints(StateVec(0, 0, 0, 0), half, 0.5);
  ASSERT_EQ(full.size(), half.size());
  EXPECT_NEAR(half.front().norm(), full.front().norm() / 2.0, 1e-6);
}

TEST(SensorParams, YawRotatesTheRayFan) {
  const SensorParams s = makeVlp16();
  std::vector<Eigen::Vector3d> unrotated, rotated;
  s.getFrustumEndpoints(StateVec(0, 0, 0, 0), unrotated);
  s.getFrustumEndpoints(StateVec(0, 0, 0, M_PI / 2.0), rotated);
  ASSERT_EQ(unrotated.size(), rotated.size());
  // A quarter turn about z maps (x, y) to (-y, x).
  EXPECT_NEAR(rotated.front().x(), -unrotated.front().y(), 1e-6);
  EXPECT_NEAR(rotated.front().y(), unrotated.front().x(), 1e-6);
}

TEST(SensorParams, LidarFovAcceptsNearbyAndRejectsDistant) {
  const SensorParams s = makeVlp16();
  const StateVec at_origin(0, 0, 0, 0);
  // Well inside 20 m and within the 30 degree vertical band.
  EXPECT_TRUE(s.isInsideFOV(at_origin, Eigen::Vector3d(5.0, 0.0, 0.0)));
  // Beyond max_range.
  EXPECT_FALSE(s.isInsideFOV(at_origin, Eigen::Vector3d(50.0, 0.0, 0.0)));
  // Straight up: outside the +/- 15 degree vertical fan.
  EXPECT_FALSE(s.isInsideFOV(at_origin, Eigen::Vector3d(0.1, 0.0, 5.0)));
}

TEST(SensorParams, FrontierTestUsesTheConfiguredThreshold) {
  const SensorParams s = makeVlp16();
  ASSERT_GT(s.numVoxelsFullFov(), 0.0);
  // Threshold is 5% of the full field of view.
  EXPECT_TRUE(s.isFrontier(0.10 * s.numVoxelsFullFov()));
  EXPECT_FALSE(s.isFrontier(0.01 * s.numVoxelsFullFov()));
}

TEST(SensorParams, ModelExposesWhatTheMapLayerNeeds) {
  const SensorParams s = makeVlp16();
  const auto m = s.model();
  EXPECT_EQ(m.width, 72);
  EXPECT_EQ(m.height, 6);
  EXPECT_DOUBLE_EQ(m.resolution[0], 5.0 * M_PI / 180.0);
}

TEST(SensorParams, MountingRotationOffsetsTheRays) {
  SensorParams s;
  s.type = SensorType::kLidar;
  s.max_range = 10.0;
  s.fov = Eigen::Vector2d(M_PI / 2.0, M_PI / 6.0);
  s.resolution = Eigen::Vector2d(M_PI / 12.0, M_PI / 12.0);
  s.center_offset = Eigen::Vector3d(0.5, 0.0, 0.2);
  s.update();
  std::vector<Eigen::Vector3d> endpoints;
  s.getFrustumEndpoints(StateVec(0, 0, 0, 0), endpoints);
  ASSERT_FALSE(endpoints.empty());
  // The mount offset shifts every ray, so none is exactly max_range from the
  // body origin any more.
  EXPECT_NE(endpoints.front().norm(), 10.0);
}

}  // namespace
