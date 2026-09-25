// Tests for ground projection. The ROS 1 versions had none: exercising them
// meant driving a simulated ground robot over terrain.

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/ground_projection.h"

namespace {

using mgg::GroundProjection;
using mgg::MapInterface;
using mgg::PlanningParams;
using mgg::ProjectedEdgeStatus;
using mgg::VoxelStatus;

/// Ground at z = 0 everywhere except a pit in x in [4,6], where there is no
/// ground at all (unknown below), and a wall at x in [8,9].
class Terrain : public MapInterface {
 public:
  double getResolution() const override { return 0.2; }
  bool getStatus() const override { return true; }

  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    if (p.x() >= 8.0 && p.x() <= 9.0 && p.z() > 0.0 && p.z() < 2.0) {
      return VoxelStatus::kOccupied;
    }
    if (p.x() >= 4.0 && p.x() <= 6.0 && p.z() <= 0.0) {
      return VoxelStatus::kUnknown;  // the pit: nothing mapped below
    }
    if (p.z() <= 0.0) return VoxelStatus::kOccupied;  // ground
    return VoxelStatus::kFree;
  }

  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool stop_at_unknown) const override {
    Eigen::Vector3d ignored;
    return getRayStatus(a, b, stop_at_unknown, ignored);
  }

  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool stop_at_unknown,
                           Eigen::Vector3d& end_voxel) const override {
    const double len = (b - a).norm();
    if (len < 1e-12) { end_voxel = b; return VoxelStatus::kFree; }
    const Eigen::Vector3d dir = (b - a) / len;
    for (double d = 0.0; d <= len; d += 0.2) {
      const Eigen::Vector3d p = a + d * dir;
      const VoxelStatus s = getVoxelStatus(p);
      if (s == VoxelStatus::kOccupied ||
          (s == VoxelStatus::kUnknown && stop_at_unknown)) {
        end_voxel = p;
        return s;
      }
    }
    end_voxel = b;
    return VoxelStatus::kFree;
  }

  VoxelStatus getBoxStatus(const Eigen::Vector3d& c, const Eigen::Vector3d&,
                           bool) const override {
    return getVoxelStatus(c);
  }
  VoxelStatus getPathStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                            const Eigen::Vector3d&,
                            bool stop_at_unknown) const override {
    return getRayStatus(a, b, stop_at_unknown);
  }
  void getScanStatus(const Eigen::Vector3d&,
                     const std::vector<Eigen::Vector3d>&, mgg::GainCounts&,
                     std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>&,
                     const mgg::SensorModel&) override {}
  void getScanStatusIterative(
      const Eigen::Vector3d&, const std::vector<Eigen::Vector3d>&,
      mgg::GainCounts&,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>&,
      const mgg::SensorModel&) override {}
  bool augmentFreeBox(const Eigen::Vector3d&,
                      const Eigen::Vector3d&) override { return true; }
  void augmentFreeFrustum() override {}
  void resetMap() override {}
  void extractLocalMap(const Eigen::Vector3d&, const Eigen::Vector3d&,
                       std::vector<Eigen::Vector3d>&,
                       std::vector<Eigen::Vector3d>&) override {}
  void extractLocalMapAlongAxis(const Eigen::Vector3d&, const Eigen::Vector3d&,
                                const Eigen::Vector3d&,
                                std::vector<Eigen::Vector3d>&,
                                std::vector<Eigen::Vector3d>&) override {}
  void getLocalPointcloud(const Eigen::Vector3d&, double, double,
                          std::vector<Eigen::Vector3d>&, bool) override {}
  void getFreeSpacePointCloud(const std::vector<Eigen::Vector3d>&,
                              const mgg::StateVec&,
                              std::vector<Eigen::Vector3d>&) override {}
  void setRaycastingParams(bool, double) override {}
  void setRobotRadius(double) override {}
};

PlanningParams makeParams() {
  PlanningParams p;
  p.max_inclination = 0.52;       // about 30 degrees
  p.max_ground_height = 0.5;
  return p;
}

TEST(GroundProjection, FindsGroundBelowASample) {
  Terrain map;
  PlanningParams params = makeParams();
  GroundProjection gp(map, params);

  Eigen::Vector3d sample(1.0, 0.0, 2.0);
  VoxelStatus status;
  const double d = gp.projectSample(sample, status);
  EXPECT_EQ(status, VoxelStatus::kOccupied);
  EXPECT_NEAR(d, 2.0, 0.3);  // ground is 2 m below
}

TEST(GroundProjection, ReportsFreeWhenNothingIsWithinReach) {
  Terrain map;
  PlanningParams params = makeParams();
  GroundProjection gp(map, params);
  gp.max_projection_length = 0.5;  // shorter than the drop to the ground

  Eigen::Vector3d sample(1.0, 0.0, 5.0);
  VoxelStatus status;
  const double d = gp.projectSample(sample, status);
  EXPECT_EQ(status, VoxelStatus::kFree);
  EXPECT_LT(d, 0.0);
}

// Endpoints are passed at driving height (max_ground_height above the
// ground), which getProjectedEdgeStatus requires; see the header.
TEST(GroundProjection, FlatEdgeIsAdmissible) {
  Terrain map;
  PlanningParams params = makeParams();
  GroundProjection gp(map, params);

  std::vector<Eigen::Vector3d> path;
  const auto s = gp.getProjectedEdgeStatus({0.0, 0.0, 0.5}, {3.0, 0.0, 0.5},
                                           {0.4, 0.4, 0.4}, true, path, false);
  EXPECT_EQ(s, ProjectedEdgeStatus::kAdmissible);
  EXPECT_FALSE(path.empty());
}

TEST(GroundProjection, SteepEdgeIsRejectedBeforeAnyMapQuery) {
  Terrain map;
  PlanningParams params = makeParams();
  GroundProjection gp(map, params);

  std::vector<Eigen::Vector3d> path;
  // Straight up: far past max_inclination.
  const auto s = gp.getProjectedEdgeStatus({0.0, 0.0, 0.5}, {0.2, 0.0, 5.0},
                                           {0.4, 0.4, 0.4}, true, path, false);
  EXPECT_EQ(s, ProjectedEdgeStatus::kSteep);
}

TEST(GroundProjection, EdgeOverAPitHangs) {
  Terrain map;
  PlanningParams params = makeParams();
  GroundProjection gp(map, params);

  std::vector<Eigen::Vector3d> path;
  // x in [4,6] has no mapped ground beneath it.
  const auto s = gp.getProjectedEdgeStatus({3.0, 0.0, 0.5}, {7.0, 0.0, 0.5},
                                           {0.4, 0.4, 0.4}, true, path, false);
  EXPECT_EQ(s, ProjectedEdgeStatus::kHanging);
}

TEST(GroundProjection, HangingIsToleratedWhenTheCallerAllowsIt) {
  Terrain map;
  PlanningParams params = makeParams();
  GroundProjection gp(map, params);

  std::vector<Eigen::Vector3d> path;
  const auto s = gp.getProjectedEdgeStatus({3.0, 0.0, 0.5}, {7.0, 0.0, 0.5},
                                           {0.4, 0.4, 0.4}, true, path,
                                           /*is_hanging=*/true);
  EXPECT_NE(s, ProjectedEdgeStatus::kHanging);
}

// REGRESSION for the erase(end()) fix: a short edge whose last sample nearly
// coincides with the endpoint used to hit undefined behaviour here.
TEST(GroundProjection, EdgeEndingOnASampleDoesNotCorruptThePath) {
  Terrain map;
  PlanningParams params = makeParams();
  GroundProjection gp(map, params);

  std::vector<Eigen::Vector3d> path;
  // Length is an exact multiple of the 0.4 m step (2 * resolution).
  const auto s = gp.getProjectedEdgeStatus({0.0, 0.0, 0.5}, {2.0, 0.0, 0.5},
                                           {0.4, 0.4, 0.4}, true, path, false);
  EXPECT_EQ(s, ProjectedEdgeStatus::kAdmissible);
  ASSERT_GE(path.size(), 2u);
  // The endpoint must be the last entry, exactly once.
  EXPECT_NEAR(path.back().x(), 2.0, 1e-6);
  EXPECT_GT((path[path.size() - 2] - path.back()).norm(), 1e-6);
}

TEST(GroundProjection, ProjectsEndpointHeightToDrivingHeight) {
  Terrain map;
  PlanningParams params = makeParams();
  GroundProjection gp(map, params);

  std::vector<Eigen::Vector3d> path;
  // End point passed at ground level z = 0.0 (unprojected lattice height)
  const auto s = gp.getProjectedEdgeStatus({0.0, 0.0, 0.5}, {2.0, 0.0, 0.0},
                                           {0.4, 0.4, 0.4}, true, path, false);
  EXPECT_EQ(s, ProjectedEdgeStatus::kAdmissible);
  ASSERT_FALSE(path.empty());
  // The endpoint in the projected edge MUST be at driving height z = 0.5
  EXPECT_NEAR(path.back().z(), 0.5, 1e-3);
}

/// A 16 degree ramp rising along +y from y = 0 to y = 4, level before and
/// after it. Rays are marched finely, so heights read back within a
/// few millimetres.
class Ramp : public Terrain {
 public:
  static double slope() { return std::tan(16.0 * M_PI / 180.0); }
  static double height(double y) {
    return slope() * std::min(std::max(y, 0.0), 4.0);
  }
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    return p.z() <= height(p.y()) ? VoxelStatus::kOccupied
                                   : VoxelStatus::kFree;
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool stop_at_unknown,
                           Eigen::Vector3d& end_voxel) const override {
    (void)stop_at_unknown;
    const double len = (b - a).norm();
    const Eigen::Vector3d dir = len > 1e-12 ? Eigen::Vector3d((b - a) / len)
                                            : Eigen::Vector3d::Zero();
    for (double d = 0.0; d <= len; d += 0.002) {
      const Eigen::Vector3d p = a + d * dir;
      if (getVoxelStatus(p) == VoxelStatus::kOccupied) {
        end_voxel = p;
        return VoxelStatus::kOccupied;
      }
    }
    end_voxel = b;
    return VoxelStatus::kFree;
  }
};

/// A point `max_ground_height` (0.5) above the ramp, where edges start.
Eigen::Vector3d onRamp(double x, double y) {
  return {x, y, Ramp::height(y) + 0.5};
}

const Eigen::Vector3d kBox(0.8, 0.6, 0.4);

TEST(GroundProjection, RampDrivenAlongItsAxisHasNoCrossSlope) {
  Ramp map;
  PlanningParams params = makeParams();
  params.max_cross_slope = 10.0 * M_PI / 180.0;
  GroundProjection gp(map, params);

  std::vector<Eigen::Vector3d> path;
  const auto s = gp.getProjectedEdgeStatus(onRamp(0.0, 0.5), onRamp(0.0, 3.5),
                                           kBox, true, path, false);
  EXPECT_EQ(s, ProjectedEdgeStatus::kAdmissible);
  EXPECT_NEAR(gp.crossSlope(path, kBox), 0.0, 0.5 * M_PI / 180.0);
}

TEST(GroundProjection, EdgeAcrossARampPastTheCrossSlopeLimitIsRefused) {
  Ramp map;
  PlanningParams params = makeParams();
  params.max_cross_slope = 10.0 * M_PI / 180.0;
  GroundProjection gp(map, params);

  std::vector<Eigen::Vector3d> path;
  const auto across = gp.getProjectedEdgeStatus(
      onRamp(-1.5, 2.0), onRamp(1.5, 2.0), kBox, true, path, false);
  EXPECT_EQ(across, ProjectedEdgeStatus::kCrossSlope);
  EXPECT_TRUE(path.empty());  // only an admissible edge is handed back

  // The same edge is fine for a platform allowed 18 degrees, and measures
  // the ramp's 16.
  params.max_cross_slope = 18.0 * M_PI / 180.0;
  ASSERT_EQ(gp.getProjectedEdgeStatus(onRamp(-1.5, 2.0), onRamp(1.5, 2.0),
                                      kBox, true, path, false),
            ProjectedEdgeStatus::kAdmissible);
  EXPECT_NEAR(gp.crossSlope(path, kBox), 16.0 * M_PI / 180.0,
              0.5 * M_PI / 180.0);

  // Diagonally, 45 degrees off the axis, the side slope is
  // atan(tan 16 * sin 45), about 11.4 degrees: past 10, under 18.
  params.max_cross_slope = 10.0 * M_PI / 180.0;
  EXPECT_EQ(gp.getProjectedEdgeStatus(onRamp(-1.0, 1.0), onRamp(1.0, 3.0),
                                      kBox, true, path, false),
            ProjectedEdgeStatus::kCrossSlope);

  // pi/2 disables the check.
  params.max_cross_slope = M_PI / 2.0;
  EXPECT_EQ(gp.getProjectedEdgeStatus(onRamp(-1.5, 2.0), onRamp(1.5, 2.0),
                                      kBox, true, path, false),
            ProjectedEdgeStatus::kAdmissible);
}

/// The ramp as a 0.2 m voxel map reports it: a ray stops in a cell and
/// returns the cell's centre, at the height of the highest ground in it.
class VoxelRamp : public Ramp {
 public:
  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool stop_at_unknown,
                           Eigen::Vector3d& end_voxel) const override {
    const VoxelStatus s = Ramp::getRayStatus(a, b, stop_at_unknown, end_voxel);
    if (s == VoxelStatus::kOccupied) {
      const double cx = (std::floor(end_voxel.x() / 0.2) + 0.5) * 0.2;
      const double cy = (std::floor(end_voxel.y() / 0.2) + 0.5) * 0.2;
      end_voxel = {cx, cy, height(cy + 0.1)};
    }
    return s;
  }
};

TEST(GroundProjection, CrossSlopeIsMeasuredBetweenTheCellsTheMapReturns) {
  // Probes 0.7 m apart, at y = 0.75 and 1.45, stop in cells whose centres
  // are 0.8 m apart: the rise over the probes' spacing would read 18.1
  // degrees on the 16 degree ramp, over the cells' it reads 16.
  VoxelRamp map;
  PlanningParams params = makeParams();
  params.max_cross_slope = 17.0 * M_PI / 180.0;
  GroundProjection gp(map, params);
  const Eigen::Vector3d box(0.8, 0.7, 0.4);
  std::vector<Eigen::Vector3d> path;
  ASSERT_EQ(gp.getProjectedEdgeStatus(onRamp(-1.5, 1.1), onRamp(1.5, 1.1), box,
                                      true, path, false),
            ProjectedEdgeStatus::kAdmissible);
  EXPECT_NEAR(gp.crossSlope(path, box), 16.0 * M_PI / 180.0,
              0.1 * M_PI / 180.0);
}

/// Level ground with a 0.2 m kerb, 0.1 m long, under the left side only.
class Kerb : public Terrain {
 public:
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    const bool kerb = p.x() >= 0.75 && p.x() <= 0.85 && p.y() > 0.0;
    return p.z() <= (kerb ? 0.2 : 0.0) ? VoxelStatus::kOccupied
                                       : VoxelStatus::kFree;
  }
};

TEST(GroundProjection, CrossSlopeIsAveragedOverTheBodyLength) {
  // One point of the edge has its left side on the kerb: 0.2 m over the
  // 0.6 m track is 18 degrees there, but the robot's 0.8 m body spans three
  // points and rolls by about a third of that.
  Kerb map;
  PlanningParams params = makeParams();
  GroundProjection gp(map, params);
  std::vector<Eigen::Vector3d> edge;
  for (int i = 0; i <= 5; ++i) edge.emplace_back(0.4 * i, 0.0, 0.5);
  const double roll = gp.crossSlope(edge, kBox);
  EXPECT_NEAR(roll, std::atan2(0.2 / 3.0, 0.6), 1e-6);

  // Along a level floor nothing is measured at all.
  Terrain flat;
  GroundProjection level(flat, params);
  std::vector<Eigen::Vector3d> path;
  ASSERT_EQ(level.getProjectedEdgeStatus({0.0, 0.0, 0.5}, {3.0, 0.0, 0.5},
                                         kBox, true, path, false),
            ProjectedEdgeStatus::kAdmissible);
  EXPECT_DOUBLE_EQ(level.crossSlope(path, kBox), 0.0);
}

}  // namespace
