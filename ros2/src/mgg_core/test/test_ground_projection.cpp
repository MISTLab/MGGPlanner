// Tests for ground projection. The ROS 1 versions had none: exercising them
// meant driving a simulated ground robot over terrain.

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

TEST(GroundProjection, DoesNotMirrorAnObstacleAboveTheSampleIntoGround) {
  Terrain map;
  PlanningParams params = makeParams();
  GroundProjection gp(map, params);

  // Terrain's wall occupies 0 < z < 2. The centre ray starts 0.4 m above
  // this sample and immediately hits it. A magnitude would turn that hit
  // into fictitious ground below the robot; signed clearance rejects it.
  Eigen::Vector3d sample(8.5, 0.0, 0.5);
  VoxelStatus status;
  const double d = gp.projectSample(sample, status);
  EXPECT_EQ(status, VoxelStatus::kOccupied);
  EXPECT_LT(d, 0.0);
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
  EXPECT_EQ(s, ProjectedEdgeStatus::kAdmissible);
  ASSERT_FALSE(path.empty());
  for (const auto& point : path) {
    EXPECT_LT(point.z(), 0.8) << "Missing ground must not become a metre-high step";
  }
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

}  // namespace

