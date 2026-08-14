// Proves the map contract is implementable and exercisable with no ROS, no
// simulator and no robot present. That property is the point of mgg_core: the
// ROS 1 planner could only be tested by launching Gazebo.
//
// AnalyticMap below is a room with one box obstacle, described in closed form
// rather than built from sensor data. It is the same scene the ROS 1 baseline
// harness (tools/map_baseline) integrates into voxblox, so the expectations
// here and the recorded baseline describe the same geometry.

#include <cmath>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/map_interface.h"
#include "mgg_core/types.h"

namespace {

using mgg::GainCounts;
using mgg::MapInterface;
using mgg::SensorModel;
using mgg::StateVec;
using mgg::VoxelStatus;

bool inside(const Eigen::Vector3d& p, const Eigen::Vector3d& lo,
            const Eigen::Vector3d& hi) {
  return (p.array() >= lo.array()).all() && (p.array() <= hi.array()).all();
}

/// Room interior is free, the obstacle and anything outside the room is
/// occupied/unknown respectively.
class AnalyticMap : public MapInterface {
 public:
  static constexpr double kResolution = 0.2;

  double getResolution() const override { return kResolution; }
  bool getStatus() const override { return true; }

  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    if (inside(p, obstacle_lo_, obstacle_hi_)) return VoxelStatus::kOccupied;
    if (!inside(p, room_lo_, room_hi_)) return VoxelStatus::kUnknown;
    return VoxelStatus::kFree;
  }

  VoxelStatus getRayStatus(const Eigen::Vector3d& from,
                           const Eigen::Vector3d& to,
                           bool stop_at_unknown) const override {
    Eigen::Vector3d ignored;
    return getRayStatus(from, to, stop_at_unknown, ignored);
  }

  VoxelStatus getRayStatus(const Eigen::Vector3d& from,
                           const Eigen::Vector3d& to, bool stop_at_unknown,
                           Eigen::Vector3d& end_voxel) const override {
    const double len = (to - from).norm();
    const Eigen::Vector3d dir = (to - from).normalized();
    for (double d = 0.0; d <= len; d += kResolution) {
      const Eigen::Vector3d p = from + d * dir;
      const VoxelStatus s = getVoxelStatus(p);
      if (s == VoxelStatus::kOccupied ||
          (s == VoxelStatus::kUnknown && stop_at_unknown)) {
        end_voxel = p;
        return s;
      }
    }
    end_voxel = to;
    return VoxelStatus::kFree;
  }

  VoxelStatus getBoxStatus(const Eigen::Vector3d& c, const Eigen::Vector3d& s,
                           bool stop_at_unknown) const override {
    bool saw_unknown = false;
    for (double dx = -s.x() / 2; dx <= s.x() / 2; dx += kResolution)
      for (double dy = -s.y() / 2; dy <= s.y() / 2; dy += kResolution)
        for (double dz = -s.z() / 2; dz <= s.z() / 2; dz += kResolution) {
          const VoxelStatus v = getVoxelStatus(c + Eigen::Vector3d(dx, dy, dz));
          if (v == VoxelStatus::kOccupied) return VoxelStatus::kOccupied;
          if (v == VoxelStatus::kUnknown) saw_unknown = true;
        }
    if (saw_unknown && stop_at_unknown) return VoxelStatus::kUnknown;
    return VoxelStatus::kFree;
  }

  VoxelStatus getPathStatus(const Eigen::Vector3d& start,
                            const Eigen::Vector3d& end,
                            const Eigen::Vector3d& box,
                            bool stop_at_unknown) const override {
    const double len = (end - start).norm();
    const Eigen::Vector3d dir = (end - start).normalized();
    bool saw_unknown = false;
    for (double d = 0.0; d <= len; d += kResolution) {
      const VoxelStatus v = getBoxStatus(start + d * dir, box, stop_at_unknown);
      if (v == VoxelStatus::kOccupied) return VoxelStatus::kOccupied;
      if (v == VoxelStatus::kUnknown) saw_unknown = true;
    }
    return saw_unknown ? VoxelStatus::kUnknown : VoxelStatus::kFree;
  }

  void getScanStatus(const Eigen::Vector3d& pos,
                     const std::vector<Eigen::Vector3d>& endpoints,
                     GainCounts& gain,
                     std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& log,
                     const SensorModel&) override {
    gain = GainCounts{};
    for (const auto& e : endpoints) {
      const double len = (e - pos).norm();
      const Eigen::Vector3d dir = (e - pos).normalized();
      for (double d = 0.0; d <= len; d += kResolution) {
        const Eigen::Vector3d p = pos + d * dir;
        const VoxelStatus s = getVoxelStatus(p);
        log.emplace_back(p, s);
        if (s == VoxelStatus::kOccupied) { ++gain.occupied; break; }
        if (s == VoxelStatus::kUnknown) ++gain.unknown; else ++gain.free;
      }
    }
  }

  void getScanStatusIterative(
      const Eigen::Vector3d& pos,
      const std::vector<Eigen::Vector3d>& endpoints, GainCounts& gain,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& log,
      const SensorModel& sensor) override {
    getScanStatus(pos, endpoints, gain, log, sensor);
  }

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
                              const StateVec&,
                              std::vector<Eigen::Vector3d>&) override {}
  void setRaycastingParams(bool, double) override {}
  void setRobotRadius(double) override {}

 private:
  const Eigen::Vector3d room_lo_{-10.0, -10.0, 0.0};
  const Eigen::Vector3d room_hi_{10.0, 10.0, 4.0};
  const Eigen::Vector3d obstacle_lo_{2.0, -1.5, 0.0};
  const Eigen::Vector3d obstacle_hi_{4.0, 1.5, 2.0};
};

TEST(MapInterface, HoldsImplementationThroughBasePointer) {
  // The ROS 1 MapManagerVoxblox inherited privately, so this line was
  // impossible: "'MapManager' is an inaccessible base". Guard against the
  // regression.
  AnalyticMap concrete;
  MapInterface* map = &concrete;
  EXPECT_DOUBLE_EQ(map->getResolution(), AnalyticMap::kResolution);
  EXPECT_TRUE(map->getStatus());
}

TEST(MapInterface, ClassifiesTheThreeStates) {
  AnalyticMap map;
  EXPECT_EQ(map.getVoxelStatus({0.0, 0.0, 1.0}), VoxelStatus::kFree);
  EXPECT_EQ(map.getVoxelStatus({3.0, 0.0, 1.0}), VoxelStatus::kOccupied);
  EXPECT_EQ(map.getVoxelStatus({50.0, 0.0, 1.0}), VoxelStatus::kUnknown);
}

TEST(MapInterface, RayStopsAtTheObstacleAndReportsWhere) {
  AnalyticMap map;
  Eigen::Vector3d end_voxel;
  const auto s = map.getRayStatus({-8.0, 0.0, 1.0}, {8.0, 0.0, 1.0}, true,
                                  end_voxel);
  EXPECT_EQ(s, VoxelStatus::kOccupied);
  // The obstacle's near face is at x = 2.0.
  EXPECT_NEAR(end_voxel.x(), 2.0, AnalyticMap::kResolution);
}

TEST(MapInterface, PathThroughObstacleBlockedAndClearPathFree) {
  AnalyticMap map;
  const Eigen::Vector3d box(0.8, 0.8, 0.8);
  EXPECT_EQ(map.getPathStatus({-8.0, 0.0, 1.0}, {8.0, 0.0, 1.0}, box, true),
            VoxelStatus::kOccupied);
  EXPECT_EQ(map.getPathStatus({0.0, -8.0, 1.0}, {0.0, 8.0, 1.0}, box, true),
            VoxelStatus::kFree);
}

TEST(MapInterface, GainIsHigherFacingUnexploredSpace) {
  AnalyticMap map;
  const SensorModel sensor{8, 1, {M_PI / 4.0, M_PI / 4.0}};

  auto gain_at = [&](const Eigen::Vector3d& pos) {
    std::vector<Eigen::Vector3d> endpoints;
    for (int i = 0; i < 8; ++i) {
      const double a = 2.0 * M_PI * i / 8.0;
      endpoints.push_back(pos + 20.0 * Eigen::Vector3d(std::cos(a),
                                                       std::sin(a), 0.0));
    }
    GainCounts g;
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> log;
    map.getScanStatus(pos, endpoints, g, log, sensor);
    return g;
  };

  // Near the wall more rays leave the room, so more unknown is visible than
  // from the middle. This is the quantity the exploration planner maximises.
  EXPECT_GT(gain_at({9.0, 9.0, 1.0}).unknown, gain_at({0.0, 0.0, 1.0}).unknown);
}

}  // namespace
