#include <cmath>

#include <gtest/gtest.h>

#include "mgg_core/params.h"

namespace {

using mgg::BoundedSpaceParams;
using mgg::BoundedSpaceType;
using mgg::BoundModeType;
using mgg::RobotParams;
using mgg::StateVec;

RobotParams makeSmb() {
  RobotParams r;
  r.size = Eigen::Vector3d(0.8, 0.8, 0.2);
  r.size_extension_min = Eigen::Vector3d::Zero();
  r.size_extension = Eigen::Vector3d(0.2, 0.2, 0.2);
  r.relax_ratio = 0.5;
  return r;
}

TEST(RobotParams, PlanningSizeFollowsTheBoundMode) {
  RobotParams r = makeSmb();

  r.bound_mode = BoundModeType::kExactBound;
  EXPECT_TRUE(r.getPlanningSize().isApprox(Eigen::Vector3d(0.8, 0.8, 0.2)));

  r.bound_mode = BoundModeType::kExtendedBound;
  EXPECT_TRUE(r.getPlanningSize().isApprox(Eigen::Vector3d(1.0, 1.0, 0.4)));

  r.bound_mode = BoundModeType::kMinBound;
  EXPECT_TRUE(r.getPlanningSize().isApprox(Eigen::Vector3d(0.8, 0.8, 0.2)));

  // Halfway between min and full extension.
  r.bound_mode = BoundModeType::kRelaxedBound;
  EXPECT_TRUE(r.getPlanningSize().isApprox(Eigen::Vector3d(0.9, 0.9, 0.3)));

  r.bound_mode = BoundModeType::kNoBound;
  EXPECT_TRUE(r.getPlanningSize().isZero());
}

TEST(BoundedSpace, CuboidContainsAndExcludes) {
  BoundedSpaceParams s;
  s.type = BoundedSpaceType::kCuboid;
  s.min_val = Eigen::Vector3d(-15, -15, -3);
  s.max_val = Eigen::Vector3d(15, 15, 3);
  s.setCenter(StateVec(0, 0, 0, 0), false);

  EXPECT_TRUE(s.isInsideSpace(Eigen::Vector3d(10, 10, 1)));
  EXPECT_FALSE(s.isInsideSpace(Eigen::Vector3d(20, 0, 0)));
  EXPECT_FALSE(s.isInsideSpace(Eigen::Vector3d(0, 0, 5)));
}

TEST(BoundedSpace, CuboidFollowsTheCentre) {
  BoundedSpaceParams s;
  s.min_val = Eigen::Vector3d(-1, -1, -1);
  s.max_val = Eigen::Vector3d(1, 1, 1);
  s.setCenter(Eigen::Vector3d(10, 0, 0), false);
  EXPECT_TRUE(s.isInsideSpace(Eigen::Vector3d(10.5, 0, 0)));
  EXPECT_FALSE(s.isInsideSpace(Eigen::Vector3d(0, 0, 0)));
}

TEST(BoundedSpace, CuboidRotates) {
  BoundedSpaceParams s;
  s.min_val = Eigen::Vector3d(-4, -1, -1);
  s.max_val = Eigen::Vector3d(4, 1, 1);
  s.setRotation(Eigen::Vector3d(M_PI / 2.0, 0, 0));  // quarter turn about z
  s.setCenter(Eigen::Vector3d(0.0, 0.0, 0.0), false);
  // The long axis now runs along y.
  EXPECT_TRUE(s.isInsideSpace(Eigen::Vector3d(0, 3, 0)));
  EXPECT_FALSE(s.isInsideSpace(Eigen::Vector3d(3, 0, 0)));
}

TEST(BoundedSpace, SphereHonoursTheRadiusExtension) {
  BoundedSpaceParams s;
  s.type = BoundedSpaceType::kSphere;
  s.radius = 5.0;
  s.radius_extension = 5.0;

  s.setCenter(Eigen::Vector3d(0.0, 0.0, 0.0), false);
  EXPECT_FALSE(s.isInsideSpace(Eigen::Vector3d(7, 0, 0)));

  // With the extension the same point is inside: the sphere path does use it.
  s.setCenter(Eigen::Vector3d(0.0, 0.0, 0.0), true);
  EXPECT_TRUE(s.isInsideSpace(Eigen::Vector3d(7, 0, 0)));
}

// The extension now applies to cuboids as well as spheres. In ROS 1 it was
// computed and discarded for cuboids, which made min_extension/max_extension
// dead configuration in every shipped file.
TEST(BoundedSpace, CuboidHonoursTheExtension) {
  BoundedSpaceParams s;
  s.type = BoundedSpaceType::kCuboid;
  s.min_val = Eigen::Vector3d(-15, -15, -3);
  s.max_val = Eigen::Vector3d(15, 15, 3);
  s.min_extension = Eigen::Vector3d(-20, -20, -20);
  s.max_extension = Eigen::Vector3d(20, 20, 20);

  // Sampling volume: the extension is not applied.
  s.setCenter(Eigen::Vector3d(0.0, 0.0, 0.0), false);
  EXPECT_TRUE(s.isInsideSpace(Eigen::Vector3d(10, 0, 0)));
  EXPECT_FALSE(s.isInsideSpace(Eigen::Vector3d(30, 0, 0)));

  // Gain volume: it is.
  s.setCenter(Eigen::Vector3d(0.0, 0.0, 0.0), true);
  EXPECT_TRUE(s.maxValTotal().isApprox(Eigen::Vector3d(35, 35, 23)));
  EXPECT_TRUE(s.isInsideSpace(Eigen::Vector3d(30, 0, 0)));
  EXPECT_FALSE(s.isInsideSpace(Eigen::Vector3d(40, 0, 0)));
}

// The shipped configs zero the Local extensions, so use_extension makes no
// difference in practice until someone widens them deliberately. This pins
// that, so enabling the mechanism cannot quietly change behaviour.
TEST(BoundedSpace, ZeroExtensionMakesUseExtensionANoOp) {
  BoundedSpaceParams s;
  s.type = BoundedSpaceType::kCuboid;
  s.min_val = Eigen::Vector3d(-15, -15, -3);
  s.max_val = Eigen::Vector3d(15, 15, 3);
  // min_extension and max_extension default to zero, as the configs now set.

  s.setCenter(Eigen::Vector3d(0.0, 0.0, 0.0), true);
  EXPECT_TRUE(s.minValTotal().isApprox(s.min_val));
  EXPECT_TRUE(s.maxValTotal().isApprox(s.max_val));
  EXPECT_TRUE(s.isInsideSpace(Eigen::Vector3d(14.9, 0, 0)));
  EXPECT_FALSE(s.isInsideSpace(Eigen::Vector3d(15.1, 0, 0)));
}

}  // namespace
