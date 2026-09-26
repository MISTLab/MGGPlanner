// Tests for ground projection. The ROS 1 versions had none: exercising them
// meant driving a simulated ground robot over terrain.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/ground_projection.h"
#include "mgg_core/path_turns.h"
#include "terrain_fixture.h"

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
  bool getAxisAlignedXYCellCenter(const Eigen::Vector2d& p,
                                  Eigen::Vector2d& center) const override {
    center = 0.2 * Eigen::Vector2d(std::floor(p.x() / 0.2) + 0.5,
                                   std::floor(p.y() / 0.2) + 0.5);
    return true;
  }
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

/// Level floors, each the solid z in [bottom, top] everywhere. Vertical rays
/// are answered exactly, so heights read back without a voxel's error.
class Floors : public Terrain {
 public:
  explicit Floors(std::vector<std::pair<double, double>> slabs)
      : slabs_(std::move(slabs)) {}
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    for (const auto& [bottom, top] : slabs_) {
      if (p.z() >= bottom && p.z() <= top) return VoxelStatus::kOccupied;
    }
    return VoxelStatus::kFree;
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool, Eigen::Vector3d& end_voxel) const override {
    // Downward only, as ground rays are: the highest solid the ray meets.
    double hit = -std::numeric_limits<double>::infinity();
    for (const auto& [bottom, top] : slabs_) {
      if (top >= b.z() && bottom <= a.z()) hit = std::max(hit, std::min(top, a.z()));
    }
    if (!std::isfinite(hit)) {
      end_voxel = b;
      return VoxelStatus::kFree;
    }
    end_voxel = Eigen::Vector3d(a.x(), a.y(), hit);
    return VoxelStatus::kOccupied;
  }

 private:
  std::vector<std::pair<double, double>> slabs_;
};

TEST(GroundProjection, AGoalFindsTheFloorAboveItsSeedHeight) {
  // A 2-D goal is seeded at the robot's altitude, here 5.5 m below the only
  // floor at the goal (robot_0 on the level below home, SubT 2026-09-23).
  Floors map({{5.3, 5.5}});
  PlanningParams params = makeParams();
  GroundProjection gp(map, params);
  Eigen::Vector3d below(1.0, 2.0, 0.0);
  VoxelStatus status;
  gp.projectSample(below, status);
  EXPECT_NE(status, VoxelStatus::kOccupied);  // downward only

  Eigen::Vector3d goal(1.0, 2.0, 0.0);
  EXPECT_DOUBLE_EQ(gp.projectGoal(goal, status), -5.5);
  EXPECT_EQ(status, VoxelStatus::kOccupied);
  EXPECT_DOUBLE_EQ(goal.x(), 1.0);
  EXPECT_DOUBLE_EQ(goal.y(), 2.0);
}

TEST(GroundProjection, AGoalLooksUpOnlyWithinTheBoundedRise) {
  Floors map({{6.8, 7.0}});
  PlanningParams params = makeParams();
  params.max_goal_ground_rise = 100.0;  // bounded to kMaxGoalGroundRise
  GroundProjection gp(map, params);
  Eigen::Vector3d goal(0.0, 0.0, 0.0);
  VoxelStatus status;
  gp.projectGoal(goal, status);
  EXPECT_NE(status, VoxelStatus::kOccupied);

  // Zero turns the search upward off.
  Floors near_above({{0.8, 1.0}});
  params.max_goal_ground_rise = 0.0;
  GroundProjection off(near_above, params);
  off.projectGoal(goal, status);
  EXPECT_NE(status, VoxelStatus::kOccupied);
}

TEST(GroundProjection, AGoalFindsASupportExactlyAtTheBoundedRise) {
  // The bound is inclusive: a floor whose top is exactly max_goal_ground_rise
  // above the goal supports it (review r0), wherever the search starts.
  PlanningParams params = makeParams();
  params.max_goal_ground_rise = 6.0;
  VoxelStatus status;
  Floors at_bound({{5.8, 6.0}});
  GroundProjection gp(at_bound, params);
  Eigen::Vector3d goal(0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(gp.projectGoal(goal, status), -6.0);
  EXPECT_EQ(status, VoxelStatus::kOccupied);

  // A solid reaching past the bound has its top out of reach; the floor
  // under it is the support.
  Floors past_bound({{5.8, 6.1}, {2.8, 3.0}});
  GroundProjection past(past_bound, params);
  goal = Eigen::Vector3d(0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(past.projectGoal(goal, status), -3.0);
  EXPECT_EQ(status, VoxelStatus::kOccupied);
  Floors only_past({{5.8, 6.1}});
  GroundProjection none(only_past, params);
  goal = Eigen::Vector3d(0.0, 0.0, 0.0);
  none.projectGoal(goal, status);
  EXPECT_NE(status, VoxelStatus::kOccupied);
}

TEST(GroundProjection, AGoalTakesTheFloorNearestItsHeight) {
  Floors map({{-0.2, 0.0}, {1.8, 2.0}, {3.8, 4.0}});
  PlanningParams params = makeParams();
  GroundProjection gp(map, params);
  VoxelStatus status;
  // 0.8 m below the floor at 2 m and 1.2 m above the one at 0: the upper.
  Eigen::Vector3d goal(0.0, 0.0, 1.2);
  EXPECT_DOUBLE_EQ(gp.projectGoal(goal, status), -0.8);
  EXPECT_EQ(status, VoxelStatus::kOccupied);
  // The other way round: the lower.
  goal = Eigen::Vector3d(0.0, 0.0, 0.8);
  EXPECT_DOUBLE_EQ(gp.projectGoal(goal, status), 0.8);
  EXPECT_EQ(status, VoxelStatus::kOccupied);
  // Halfway: the floor below.
  goal = Eigen::Vector3d(0.0, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(gp.projectGoal(goal, status), 1.0);
  EXPECT_EQ(status, VoxelStatus::kOccupied);
}

TEST(GroundProjection, FootprintPlaneMeasuresTheRampUnderTheBody) {
  // The combined roll and pitch: 16 degrees facing up the ramp, across it,
  // or diagonally, with every cell on the plane.
  Ramp map;
  PlanningParams params = makeParams();
  GroundProjection gp(map, params);
  for (const Eigen::Vector2d& heading :
       {Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(1.0, 0.0),
        Eigen::Vector2d(1.0, 1.0)}) {
    const mgg::FootprintPlane plane =
        gp.footprintPlane(onRamp(0.0, 2.0), heading, kBox);
    ASSERT_TRUE(plane.measured);
    EXPECT_GE(plane.cells, 12);  // of 0.2 m cells under the 0.8 x 0.6 m body
    EXPECT_NEAR(plane.tilt, 16.0 * M_PI / 180.0, 0.2 * M_PI / 180.0);
    EXPECT_LT(plane.max_residual, 0.01);
  }
}

TEST(GroundProjection, EdgeUpARampPastTheFootprintTiltIsRefused) {
  // Along the ramp's axis the cross slope is zero and each segment climbs
  // 16 degrees, under max_inclination; only the footprint sees the tilt.
  Ramp map;
  PlanningParams params = makeParams();
  params.max_footprint_tilt = 15.0 * M_PI / 180.0;
  GroundProjection gp(map, params);
  std::vector<Eigen::Vector3d> path;
  EXPECT_EQ(gp.getProjectedEdgeStatus(onRamp(0.0, 0.5), onRamp(0.0, 3.5),
                                      kBox, true, path, false),
            ProjectedEdgeStatus::kFootprintPlane);
  EXPECT_TRUE(path.empty());

  params.max_footprint_tilt = 17.0 * M_PI / 180.0;
  EXPECT_EQ(gp.getProjectedEdgeStatus(onRamp(0.0, 0.5), onRamp(0.0, 3.5),
                                      kBox, true, path, false),
            ProjectedEdgeStatus::kAdmissible);

  // Zero, the default, disables it.
  params.max_footprint_tilt = 0.0;
  EXPECT_EQ(gp.getProjectedEdgeStatus(onRamp(0.0, 0.5), onRamp(0.0, 3.5),
                                      kBox, true, path, false),
            ProjectedEdgeStatus::kAdmissible);
}

/// Level ground with a 0.3 m rock under the left of the body, between the
/// centre line and the side line crossSlope probes.
class Rock : public Ramp {
 public:
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    const bool rock = p.x() >= 1.25 && p.x() <= 1.55 && p.y() >= 0.1 &&
                      p.y() <= 0.25;
    return p.z() <= (rock ? 0.3 : 0.0) ? VoxelStatus::kOccupied
                                       : VoxelStatus::kFree;
  }
};

TEST(GroundProjection, EdgeOverARockPastTheFootprintStepIsRefused) {
  Rock map;
  PlanningParams params = makeParams();
  GroundProjection gp(map, params);
  const Eigen::Vector3d start(0.0, 0.0, 0.5);
  const Eigen::Vector3d end(3.0, 0.0, 0.5);
  std::vector<Eigen::Vector3d> path;
  // Every current check passes it: the rock is below the swept box and
  // outside the side lines.
  ASSERT_EQ(gp.getProjectedEdgeStatus(start, end, kBox, true, path, false),
            ProjectedEdgeStatus::kAdmissible);
  EXPECT_DOUBLE_EQ(gp.crossSlope(path, kBox), 0.0);

  const mgg::FootprintPlane plane =
      gp.footprintPlane({1.2, 0.0, 0.5}, {1.0, 0.0}, kBox);
  ASSERT_TRUE(plane.measured);
  // Two of the sixteen cells are on the rock; the plane leans towards it
  // and leaves it 0.24 m proud.
  EXPECT_EQ(plane.cells, 16);
  EXPECT_NEAR(plane.max_residual, 0.24, 0.005);

  params.max_footprint_step = 0.1;
  EXPECT_EQ(gp.getProjectedEdgeStatus(start, end, kBox, true, path, false),
            ProjectedEdgeStatus::kFootprintPlane);
  params.max_footprint_step = 0.3;
  EXPECT_EQ(gp.getProjectedEdgeStatus(start, end, kBox, true, path, false),
            ProjectedEdgeStatus::kAdmissible);
}

TEST(GroundProjection, FootprintPlaneNeedsGroundUnderHalfTheCells) {
  // The pit at x in [4, 6] has no ground: a body three quarters over it is
  // not measured, one half over it still is.
  Terrain map;
  PlanningParams params = makeParams();
  GroundProjection gp(map, params);
  EXPECT_FALSE(gp.footprintPlane({4.15, 0.0, 0.5}, {1.0, 0.0}, kBox).measured);
  const mgg::FootprintPlane half =
      gp.footprintPlane({4.0, 0.0, 0.5}, {1.0, 0.0}, kBox);
  ASSERT_TRUE(half.measured);
  EXPECT_NEAR(half.tilt, 0.0, 1e-9);
}

TEST(GroundProjection, FootprintPlaneFindsARockInEveryCellUnderAnAngledBody) {
  // Review r0: a Scout facing (1, 1) on 0.2 m cells, with a 0.25 m rock in
  // cell (0, -1), whose centre (0.1, -0.1) is well inside the footprint. A
  // lattice of 16 probes rotated with the body stepped over that cell, saw
  // only level ground, and the rock passed under the chassis.
  std::map<std::pair<std::int64_t, std::int64_t>, double> tops;
  for (std::int64_t x = -10; x < 10; ++x) {
    for (std::int64_t y = -10; y < 10; ++y) tops[{x, y}] = 0.0;
  }
  tops[{0, -1}] = 0.25;
  const mgg_test::TerrainFixture map(0.2, tops);
  PlanningParams params = makeParams();
  params.max_step_height = 0.15;
  params.max_ground_height = 0.4475;
  GroundProjection gp(map, params);
  const Eigen::Vector3d box(0.662, 0.630, 0.295);
  const Eigen::Vector3d start(0.0, 0.0, 0.4475);
  const Eigen::Vector3d end(0.4, 0.4, 0.4475);

  const mgg::FootprintPlane plane = gp.footprintPlane(start, {1.0, 1.0}, box);
  ASSERT_TRUE(plane.measured);
  EXPECT_EQ(plane.cells, 12);
  EXPECT_NEAR(plane.max_residual, 0.218, 0.001);

  // Below the box's underside and between the side lines: every other
  // check passes it.
  std::vector<Eigen::Vector3d> path;
  ASSERT_EQ(gp.getProjectedEdgeStatus(start, end, box, false, path, false),
            ProjectedEdgeStatus::kAdmissible);
  params.max_footprint_tilt = 22.0 * M_PI / 180.0;
  params.max_footprint_step = 0.12;
  EXPECT_EQ(gp.getProjectedEdgeStatus(start, end, box, false, path, false),
            ProjectedEdgeStatus::kFootprintPlane);
}

TEST(GroundProjection, AShortEdgeIsMeasuredBetweenItsEnds) {
  // Run 5, robot_2: a lattice edge 0.4 to 0.57 m long was measured only at
  // its two vertices, and whether a rock's worst footprint was seen hung
  // on where the lattice lay. A Scout's diagonal edge from (0, 0) to
  // (0.4, 0.4) with a 0.15 m rock in cell (1, 0): the plane under either
  // end leaves it 0.103 m off, within a 0.12 m step, but halfway along it
  // is 0.131 m off.
  std::map<std::pair<std::int64_t, std::int64_t>, double> tops;
  for (std::int64_t x = -10; x < 10; ++x) {
    for (std::int64_t y = -10; y < 10; ++y) tops[{x, y}] = 0.0;
  }
  tops[{1, 0}] = 0.15;
  const mgg_test::TerrainFixture map(0.2, tops);
  PlanningParams params = makeParams();
  params.max_step_height = 0.15;
  params.max_ground_height = 0.4475;
  params.max_footprint_tilt = 22.0 * M_PI / 180.0;
  params.max_footprint_step = 0.12;
  GroundProjection gp(map, params);
  const Eigen::Vector3d box(0.662, 0.630, 0.295);
  const Eigen::Vector3d start(0.0, 0.0, 0.4475);
  const Eigen::Vector3d end(0.4, 0.4, 0.4475);
  const Eigen::Vector2d heading(1.0, 1.0);
  EXPECT_NEAR(gp.footprintPlane(start, heading, box).max_residual, 0.103,
              0.001);
  EXPECT_NEAR(gp.footprintPlane(end, heading, box).max_residual, 0.103,
              0.001);
  EXPECT_NEAR(gp.footprintPlane((start + end) / 2, heading, box).max_residual,
              0.131, 0.001);
  std::vector<Eigen::Vector3d> path;
  EXPECT_EQ(gp.getProjectedEdgeStatus(start, end, box, false, path, false),
            ProjectedEdgeStatus::kFootprintPlane);
  // Every other check passes it.
  params.max_footprint_tilt = 0.0;
  params.max_footprint_step = 0.0;
  EXPECT_EQ(gp.getProjectedEdgeStatus(start, end, box, false, path, false),
            ProjectedEdgeStatus::kAdmissible);
}

TEST(GroundProjection, AnEdgeOntoUnobservedGroundIsRefused) {
  // Item 7: level ground mapped up to x = 0.6, nothing beyond (a ledge over
  // an unobserved drop), and a Bunker's planning box heading +x. At x = 0.4
  // the leading half of its footprint reaches 0.54 m ahead: of its cells
  // (centres 0.5, 0.7, 0.9 along it) only the first has ground.
  std::map<std::pair<std::int64_t, std::int64_t>, double> tops;
  for (std::int64_t x = -10; x < 3; ++x) {
    for (std::int64_t y = -10; y < 10; ++y) tops[{x, y}] = 0.0;
  }
  const mgg_test::TerrainFixture map(0.2, tops);
  PlanningParams params = makeParams();
  params.max_step_height = 0.15;
  params.max_ground_height = 0.525;
  GroundProjection gp(map, params);
  const Eigen::Vector3d box(1.073, 0.828, 0.45);
  const Eigen::Vector3d back(-0.4, 0.1, 0.525);
  const Eigen::Vector3d here(0.0, 0.1, 0.525);
  const Eigen::Vector3d edge(0.4, 0.1, 0.525);
  EXPECT_DOUBLE_EQ(gp.observedGroundAhead(here, {1.0, 0.0}, box), 1.0);
  EXPECT_NEAR(gp.observedGroundAhead(edge, {1.0, 0.0}, box), 1.0 / 3.0,
              1e-9);
  std::vector<Eigen::Vector3d> path;
  EXPECT_EQ(gp.getProjectedEdgeStatus(back, here, box, false, path, false),
            ProjectedEdgeStatus::kAdmissible);
  EXPECT_EQ(gp.getProjectedEdgeStatus(here, edge, box, false, path, false),
            ProjectedEdgeStatus::kGroundUnobserved);
  // Driven only away from the ledge (a departure, a shortcut, an edge out
  // of the root), the leading half is over ground.
  EXPECT_EQ(gp.getProjectedEdgeStatus(edge, here, box, false, path, false,
                                      false, nullptr,
                                      mgg::EdgeTravel::kForward),
            ProjectedEdgeStatus::kAdmissible);
  // A graph edge from the ledge vertex away from the ledge may be driven
  // back onto it: checked either way, it is refused (review r2, I-1).
  EXPECT_EQ(gp.getProjectedEdgeStatus(edge, here, box, false, path, false),
            ProjectedEdgeStatus::kGroundUnobserved);
  // Off, as before.
  params.min_observed_ground_fraction = 0.0;
  EXPECT_EQ(gp.getProjectedEdgeStatus(here, edge, box, false, path, false),
            ProjectedEdgeStatus::kAdmissible);

  // A pit that has been observed, its floor 4 m down, is no ground either.
  params.min_observed_ground_fraction = 0.75;
  for (std::int64_t x = 3; x < 10; ++x) {
    for (std::int64_t y = -10; y < 10; ++y) tops[{x, y}] = -4.0;
  }
  const mgg_test::TerrainFixture pit(0.2, tops);
  GroundProjection over_pit(pit, params);
  EXPECT_NEAR(over_pit.observedGroundAhead(edge, {1.0, 0.0}, box),
              1.0 / 3.0, 1e-9);
}

TEST(TurnSpaceObserved, NeedsObservedGroundUnderTheTurningCircle) {
  // Item 7, a turn in place: the turning circle of a Bunker, 0.643 m, needs
  // observed ground under 3/4 of its cells. Level ground up to x = 0.6.
  std::map<std::pair<std::int64_t, std::int64_t>, double> tops;
  for (std::int64_t x = -10; x < 3; ++x) {
    for (std::int64_t y = -10; y < 10; ++y) tops[{x, y}] = 0.0;
  }
  const mgg_test::TerrainFixture map(0.2, tops);
  mgg::RobotParams bunker;
  bunker.type = mgg::RobotType::kGroundRobot;
  bunker.size = Eigen::Vector3d(1.023, 0.778, 0.4);
  PlanningParams params = makeParams();
  params.max_ground_height = 0.525;
  EXPECT_TRUE(mgg::turnSpaceObserved(map, bunker, params,
                                     mgg::StateVec(-0.4, 0.1, 0.525, 0.0)));
  EXPECT_FALSE(mgg::turnSpaceObserved(map, bunker, params,
                                      mgg::StateVec(0.4, 0.1, 0.525, 0.0)));
  params.min_observed_ground_fraction = 0.0;
  EXPECT_TRUE(mgg::turnSpaceObserved(map, bunker, params,
                                     mgg::StateVec(0.4, 0.1, 0.525, 0.0)));
}

TEST(GroundProjection, AStandingStartCountsItsDiskAsObservedGround) {
  // Run 6: a robot placed on level ground has never seen the ground within
  // 1.2 m of itself, its lidar's blind disk. A ledge 2.6 m ahead drops into
  // unobserved space, and a pit 4 m deep, observed, opens 0.4 m ahead.
  std::map<std::pair<std::int64_t, std::int64_t>, double> tops;
  for (std::int64_t x = -12; x < 13; ++x) {
    for (std::int64_t y = -12; y < 12; ++y) {
      const Eigen::Vector2d centre((x + 0.5) * 0.2, (y + 0.5) * 0.2);
      if (centre.norm() < 1.2) continue;  // the blind disk
      tops[{x, y}] = 0.0;
    }
  }
  const mgg_test::TerrainFixture blind(0.2, tops);
  mgg::RobotParams bunker;
  bunker.type = mgg::RobotType::kGroundRobot;
  bunker.size = Eigen::Vector3d(1.023, 0.778, 0.4);
  PlanningParams params = makeParams();
  params.max_ground_height = 0.525;
  const Eigen::Vector3d box(1.073, 0.828, 0.45);
  const Eigen::Vector3d here(0.0, 0.1, 0.525);
  const Eigen::Vector3d before_ledge(2.4, 0.1, 0.525);
  const mgg::StateVec standing_pose(0.0, 0.1, 0.525, 0.0);

  GroundProjection gp(blind, params);
  EXPECT_DOUBLE_EQ(gp.observedGroundAhead(here, {1.0, 0.0}, box), 0.0);
  const double ledge_ahead =
      gp.observedGroundAhead(before_ledge, {1.0, 0.0}, box);
  EXPECT_LT(ledge_ahead, params.min_observed_ground_fraction);
  EXPECT_FALSE(mgg::turnSpaceObserved(blind, bunker, params, standing_pose));

  // Standing at its start, the disk of its initial ground reach counts.
  const mgg::StandingStart standing{Eigen::Vector2d(0.0, 0.1), 2.0};
  gp.setStandingStart(standing);
  ASSERT_NE(gp.standingStart(), nullptr);
  EXPECT_DOUBLE_EQ(gp.observedGroundAhead(here, {1.0, 0.0}, box), 1.0);
  EXPECT_TRUE(mgg::turnSpaceObserved(blind, bunker, params, standing_pose,
                                     &standing));
  EXPECT_TRUE(mgg::roomToTurn(blind, bunker, params, standing_pose,
                              &standing));
  // Beyond the disk unknown stays unknown: the ledge is refused as before.
  EXPECT_DOUBLE_EQ(gp.observedGroundAhead(before_ledge, {1.0, 0.0}, box),
                   ledge_ahead);

  // Ground seen to fall away is a drop, in the disk too.
  for (std::int64_t x = 2; x < 6; ++x) {
    for (std::int64_t y = -4; y < 4; ++y) tops[{x, y}] = -4.0;
  }
  const mgg_test::TerrainFixture pit(0.2, tops);
  GroundProjection over_pit(pit, params);
  over_pit.setStandingStart(standing);
  EXPECT_NEAR(over_pit.observedGroundAhead(here, {1.0, 0.0}, box), 2.0 / 3.0,
              1e-9);
  EXPECT_FALSE(mgg::turnSpaceObserved(
      pit, bunker, params, mgg::StateVec(0.6, 0.1, 0.525, 0.0), &standing));

  // Once the robot has moved, nothing is exempt.
  gp.setStandingStart(std::nullopt);
  EXPECT_EQ(gp.standingStart(), nullptr);
  EXPECT_DOUBLE_EQ(gp.observedGroundAhead(here, {1.0, 0.0}, box), 0.0);
}

TEST(GroundProjection, ARockEdgeRisesMoreBetweenCellsThanARamp) {
  // Item 8: a Scout's edge up a 16 degree ramp rises 0.057 m from one
  // 0.2 m cell to the next; one onto a 0.2 m rock 0.2 m in one step.
  // max_footprint_cell_rise 0.12 keeps the ramp and refuses the rock.
  std::map<std::pair<std::int64_t, std::int64_t>, double> ramp, rock;
  const double slope = std::tan(16.0 * M_PI / 180.0);
  for (std::int64_t x = -10; x < 10; ++x) {
    for (std::int64_t y = -10; y < 10; ++y) {
      ramp[{x, y}] = (x + 0.5) * 0.2 * slope;
      rock[{x, y}] = (x >= 2 && x <= 3 && y >= -1 && y <= 0) ? 0.2 : 0.0;
    }
  }
  PlanningParams params = makeParams();
  params.max_step_height = 0.15;
  params.max_ground_height = 0.4475;
  params.max_footprint_cell_rise = 0.12;
  const Eigen::Vector3d box(0.662, 0.630, 0.295);
  const mgg_test::TerrainFixture on_ramp(0.2, ramp);
  const GroundProjection up(on_ramp, params);
  const Eigen::Vector3d ramp_start(0.0, 0.0, 0.4475);
  const Eigen::Vector3d ramp_end(0.8, 0.0, 0.8 * slope + 0.4475);
  EXPECT_NEAR(up.footprintCellRise(ramp_start, {1.0, 0.0}, box),
              0.2 * slope, 1e-6);
  std::vector<Eigen::Vector3d> path;
  EXPECT_EQ(up.getProjectedEdgeStatus(ramp_start, ramp_end, box, false, path,
                                      false),
            ProjectedEdgeStatus::kAdmissible);

  const mgg_test::TerrainFixture on_rock(0.2, rock);
  const GroundProjection over(on_rock, params);
  const Eigen::Vector3d rock_start(0.0, 0.0, 0.4475);
  const Eigen::Vector3d rock_end(0.4, 0.0, 0.4475);
  EXPECT_NEAR(over.footprintCellRise(rock_end, {1.0, 0.0}, box),
              0.2, 1e-6);
  EXPECT_EQ(over.getProjectedEdgeStatus(rock_start, rock_end, box, false,
                                        path, false),
            ProjectedEdgeStatus::kFootprintPlane);
  params.max_footprint_cell_rise = 0.0;
  EXPECT_EQ(over.getProjectedEdgeStatus(rock_start, rock_end, box, false,
                                        path, false),
            ProjectedEdgeStatus::kAdmissible);
}

/// Level 0.2 m cells that count the ground rays cast into them.
class CountingSurface : public mgg_test::TerrainFixture {
 public:
  CountingSurface() : TerrainFixture(0.2, level()) {}
  using TerrainFixture::getRayStatus;
  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool stop_at_unknown,
                           Eigen::Vector3d& end_voxel) const override {
    ++rays;
    return TerrainFixture::getRayStatus(a, b, stop_at_unknown, end_voxel);
  }
  mutable int rays = 0;

 private:
  static std::map<std::pair<std::int64_t, std::int64_t>, double> level() {
    std::map<std::pair<std::int64_t, std::int64_t>, double> tops;
    for (std::int64_t x = -10; x < 10; ++x) {
      for (std::int64_t y = -10; y < 10; ++y) tops[{x, y}] = 0.0;
    }
    return tops;
  }
};

TEST(GroundProjection, APlanCacheReusesFootprintLookupsAcrossEdges) {
  CountingSurface map;
  PlanningParams params = makeParams();
  const Eigen::Vector3d box(0.662, 0.630, 0.295);
  const Eigen::Vector3d here(0.0, 0.0, 0.5);
  const Eigen::Vector3d next(0.4, 0.0, 0.5);

  const GroundProjection plain(map, params);
  const mgg::FootprintPlane first = plain.footprintPlane(here, {1, 0}, box);
  const int per_plane = map.rays;
  ASSERT_TRUE(first.measured);
  EXPECT_EQ(per_plane, first.cells);  // one ray per cell
  plain.footprintPlane(here, {-1, 0}, box);
  EXPECT_EQ(map.rays, 2 * per_plane);  // no cache: every ray again

  map.rays = 0;
  const GroundProjection cached(map, params, true);
  const mgg::FootprintPlane again = cached.footprintPlane(here, {1, 0}, box);
  EXPECT_EQ(map.rays, per_plane);
  EXPECT_EQ(again.cells, first.cells);
  EXPECT_DOUBLE_EQ(again.tilt, first.tilt);
  // The same body the other way round, from an edge coming back: no ray.
  cached.footprintPlane(here, {-1, 0}, box);
  EXPECT_EQ(map.rays, per_plane);
  // A neighbour's footprint shares all but its new cells.
  cached.footprintPlane(next, {1, 0}, box);
  EXPECT_LT(map.rays, 2 * per_plane);
  EXPECT_GT(map.rays, per_plane);

  // From lower down, still above the ground: those rays would cross only
  // cells the first ones found empty, so none is cast.
  const int before_lower = map.rays;
  const mgg::FootprintPlane lower =
      cached.footprintPlane({0.0, 0.0, 0.35}, {1, 0}, box);
  EXPECT_EQ(map.rays, before_lower);
  EXPECT_EQ(lower.cells, first.cells);
  // From higher up, the rays cross cells nobody has looked at: all cast.
  cached.footprintPlane({0.0, 0.0, 0.8}, {1, 0}, box);
  EXPECT_EQ(map.rays, before_lower + per_plane);
}

/// Level ground at z = 0 for x < 0.2; for x >= 0.2 a solid wall up to
/// z = 0.5 whose voxels are offset from the caller's: [0.1, 0.3],
/// [0.3, 0.5], as in a MOLA snapshot whose frame is raised 0.1 m. A ray
/// that starts inside the wall stops in the voxel it starts in.
class OffsetVoxelWall : public mgg_test::TerrainFixture {
 public:
  OffsetVoxelWall() : TerrainFixture(0.2, level()) {}
  using TerrainFixture::getRayStatus;
  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool stop_at_unknown,
                           Eigen::Vector3d& end_voxel) const override {
    if (a.x() < 0.2) {
      return TerrainFixture::getRayStatus(a, b, stop_at_unknown, end_voxel);
    }
    const double from = std::min(a.z(), 0.5 - 1e-9);
    if (b.z() > from) {
      end_voxel = b;
      return VoxelStatus::kFree;
    }
    const double bottom = 0.1 + 0.2 * std::floor((from - 0.1) / 0.2);
    end_voxel = Eigen::Vector3d(a.x(), a.y(), bottom + 0.1);
    return VoxelStatus::kOccupied;
  }

 private:
  static std::map<std::pair<std::int64_t, std::int64_t>, double> level() {
    std::map<std::pair<std::int64_t, std::int64_t>, double> tops;
    for (std::int64_t x = -10; x < 10; ++x) {
      for (std::int64_t y = -10; y < 10; ++y) tops[{x, y}] = 0.0;
    }
    return tops;
  }
};

TEST(GroundProjection, APlanCacheCastsAgainFromInsideAWall) {
  // A body half against the wall, from two heights in the same 0.2 m layer
  // of the caller's frame but in different voxels of the map's. The rays
  // into the wall stop at 0.4 m from the higher start and at 0.2 m from
  // the lower one, so the two planes differ; the cache must not hand the
  // first one's ground to the second.
  const OffsetVoxelWall map;
  PlanningParams params = makeParams();
  const Eigen::Vector3d box(0.8, 0.6, 0.3);
  const Eigen::Vector3d high(0.2, 0.0, 0.35);
  const Eigen::Vector3d low(0.2, 0.0, 0.25);

  const GroundProjection plain(map, params);
  const mgg::FootprintPlane plain_high = plain.footprintPlane(high, {1, 0}, box);
  const mgg::FootprintPlane plain_low = plain.footprintPlane(low, {1, 0}, box);
  ASSERT_TRUE(plain_high.measured);
  ASSERT_TRUE(plain_low.measured);
  ASSERT_GT(plain_high.max_residual, plain_low.max_residual + 0.01);

  const GroundProjection cached(map, params, true);
  cached.footprintPlane(high, {1, 0}, box);
  const mgg::FootprintPlane cached_low =
      cached.footprintPlane(low, {1, 0}, box);
  EXPECT_DOUBLE_EQ(cached_low.max_residual, plain_low.max_residual);
  EXPECT_DOUBLE_EQ(cached_low.tilt, plain_low.tilt);
}

}  // namespace
