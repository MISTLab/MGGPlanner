#ifndef MGG_CORE_TEST_LOCAL_PLANNER_FIXTURE_H_
#define MGG_CORE_TEST_LOCAL_PLANNER_FIXTURE_H_
#include <thread>

#include "mgg_core/local_planner.h"
#include "terrain_fixture.h"
namespace mgg_test {
inline auto localFloor() {
  std::map<std::pair<std::int64_t, std::int64_t>, double> tops;
  for (int x = -50; x <= 50; ++x)
    for (int y = -50; y <= 50; ++y) tops[{x, y}] = 0;
  return tops;
}
class LocalMap : public TerrainFixture {
 public:
  LocalMap() : TerrainFixture(0.2, localFloor()) {}
  std::optional<Eigen::AlignedBox3d> windowBounds() const override {
    return Eigen::AlignedBox3d(Eigen::Vector3d(-8, -8, -2),
                               Eigen::Vector3d(8, 8, 4));
  }
  mgg::VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    if (delay_voxel_queries && (++voxel_queries % 1024 == 0))
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (unknown_beside_corridor && std::abs(p.y()) > .8 && p.z() > 0 &&
        p.z() < .8)
      return mgg::VoxelStatus::kUnknown;
    if (!windowBounds()->contains(p)) return mgg::VoxelStatus::kUnknown;
    if (narrow && p.x() > 0.7 && std::abs(p.y()) >= 0.4 && p.z() > 0.1 &&
        p.z() < 0.9)
      return mgg::VoxelStatus::kOccupied;
    if (interior_unknown && p.x() > 6 && p.x() < 7 && p.y() > 1.5 &&
        p.y() < 2.5 && p.z() > 0.2 && p.z() < 0.8)
      return mgg::VoxelStatus::kUnknown;
    if (boundary_unknown && std::abs(p.y()) > 7.5)
      return mgg::VoxelStatus::kUnknown;
    return TerrainFixture::getVoxelStatus(p);
  }
  mgg::VoxelStatus getBoxStatus(const Eigen::Vector3d& p,
                                const Eigen::Vector3d& size,
                                bool stop) const override {
    if (unknown_beside_corridor && std::abs(p.y()) + size.y() / 2 > .8 &&
        p.z() + size.z() / 2 > .2 && p.z() - size.z() / 2 < .8)
      return mgg::VoxelStatus::kUnknown;
    if (interior_unknown && p.x() + size.x() / 2 > 6 &&
        p.x() - size.x() / 2 < 7 && p.y() + size.y() / 2 > 1.5 &&
        p.y() - size.y() / 2 < 2.5 && p.z() + size.z() / 2 > 0.2 &&
        p.z() - size.z() / 2 < 0.8)
      return mgg::VoxelStatus::kUnknown;
    if (wall && p.x() + size.x() / 2 >= wall_x &&
        p.x() - size.x() / 2 <= wall_x + 0.4)
      return mgg::VoxelStatus::kOccupied;
    if (narrow && p.x() + size.x() / 2 > 0.7 &&
        std::abs(p.y()) + size.y() / 2 >= 0.4 && p.z() + size.z() / 2 > 0.1)
      return mgg::VoxelStatus::kOccupied;
    return TerrainFixture::getBoxStatus(p, size, stop);
  }
  using TerrainFixture::getRayStatus;
  mgg::VoxelStatus getRayStatus(const Eigen::Vector3d& a,
                                const Eigen::Vector3d& b, bool stop,
                                Eigen::Vector3d& end) const override {
    // Unobserved ground: downward rays end in unknown space.
    if (unknown_ground && a.x() > 5 && a.x() < 6 && a.y() > 2 &&
        a.y() < 3) {
      end = b;
      return mgg::VoxelStatus::kUnknown;
    }
    return TerrainFixture::getRayStatus(a, b, stop, end);
  }
  bool unknown_beside_corridor = false, delay_voxel_queries = false;
  bool unknown_ground = false;
  mutable size_t voxel_queries = 0;
  double wall_x = 2;
  bool wall = false, narrow = false, boundary_unknown = false,
       interior_unknown = false;
};
inline mgg::RobotParams localRobot() {
  mgg::RobotParams r;
  r.type = mgg::RobotType::kGroundRobot;
  r.size = {1, 0.4, 0.2};
  r.size_extension.setZero();
  r.size_extension_min.setZero();
  r.safety_extension.setZero();
  r.bound_mode = mgg::BoundModeType::kExactBound;
  return r;
}
inline mgg::PlanningParams localPlanning() {
  mgg::PlanningParams p;
  p.max_ground_height = 0.5;
  p.max_step_height = 0.15;
  p.max_inclination = 0.7;
  p.max_negative_inclination = 0.5;
  p.min_observed_ground_fraction = 0.75;
  return p;
}
struct LocalScene {
  LocalMap map;
  mgg::PlanningParams planning = localPlanning();
  mgg::RobotParams robot = localRobot();
  mgg::SensorParams sensor;
  mgg::GroundLayer layer{map, planning, robot};
  mgg::CertificationCache cache{mgg::dependencyHalos(robot, planning, 0.2, 5)};
  LocalScene() {
    sensor.fov = {6.28, 0.2};
    sensor.resolution = {0.4, 0.2};
    sensor.update();
    layer.reset({0, 0, 0.5}, 0);
    layer.recheck(std::chrono::steady_clock::now() + std::chrono::seconds(20));
  }
  mgg::LocalPlanInputs inputs() const {
    mgg::LocalPlanInputs in;
    in.pose = {0, 0, 0.5, 0};
    in.speed_mps = 0.6;
    in.session_id = "session";
    in.request_id = "request";
    in.target = Eigen::Vector3d(7, 0, 0.5);
    return in;
  }
};
}  // namespace mgg_test
#endif
