#ifndef MGG_CORE_TEST_LOCAL_PLANNER_FIXTURE_H_
#define MGG_CORE_TEST_LOCAL_PLANNER_FIXTURE_H_
#include <algorithm>
#include <cmath>
#include <optional>
#include <thread>
#include <vector>

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
    if (solid(p, Eigen::Vector3d::Zero())) return mgg::VoxelStatus::kOccupied;
    if (inPit(p)) return p.z() <= kPitFloor && p.z() > kPitFloor - 0.2
                             ? mgg::VoxelStatus::kOccupied
                             : mgg::VoxelStatus::kFree;
    if (blind(p) && p.z() <= 0.6) return mgg::VoxelStatus::kUnknown;
    if (unseen(p, Eigen::Vector3d::Zero())) return mgg::VoxelStatus::kUnknown;
    if (band_unknown_beyond_y > 0 && std::abs(p.y()) > band_unknown_beyond_y &&
        p.z() > 0 && p.z() < 0.6)
      return mgg::VoxelStatus::kUnknown;
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
    if (solid(p, size)) return mgg::VoxelStatus::kOccupied;
    if (stop && unseen(p, size)) return mgg::VoxelStatus::kUnknown;
    // The body band a standing lidar cannot see near itself: within
    // 0.5 m less than its blind floor.
    if (stop && blind_radius > 0 && p.z() - size.z() / 2 < 0.6 &&
        p.head<2>().norm() <
            blind_radius - 0.5 + size.head<2>().norm() / 2)
      return mgg::VoxelStatus::kUnknown;
    if (stop && band_unknown_beyond_y > 0 &&
        std::abs(p.y()) + size.y() / 2 > band_unknown_beyond_y &&
        p.z() - size.z() / 2 < 0.6)
      return mgg::VoxelStatus::kUnknown;
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
    // Downward rays only, as ground projection casts.
    if (inPit(a) && b.z() <= kPitFloor) {
      end = Eigen::Vector3d((std::floor(a.x() / 0.2) + 0.5) * 0.2,
                            (std::floor(a.y() / 0.2) + 0.5) * 0.2, kPitFloor);
      return mgg::VoxelStatus::kOccupied;
    }
    if (inPit(a) || blind(a)) {
      end = b;
      return inPit(a) ? mgg::VoxelStatus::kFree : mgg::VoxelStatus::kUnknown;
    }
    // Unobserved ground: downward rays end in unknown space.
    if (unknown_ground && a.x() > 5 && a.x() < 6 && a.y() > 2 &&
        a.y() < 3) {
      end = b;
      return mgg::VoxelStatus::kUnknown;
    }
    return TerrainFixture::getRayStatus(a, b, stop, end);
  }
  /// The floor within blind_radius of the origin is unobserved, and so is
  /// the body band within 0.5 m less, as a lidar standing there leaves them.
  double blind_radius = 0;
  /// Floor and body band beyond |y| of it are unknown: unknown confined to
  /// the floor band, away from the robot.
  double band_unknown_beyond_y = 0;
  /// Occupied boxes.
  std::vector<Eigen::AlignedBox3d> solids;
  /// Unknown boxes of body volume; downward ground rays still see the floor.
  std::vector<Eigen::AlignedBox3d> unseen_volumes;
  /// An observed drop: the floor under it lies at kPitFloor, seen.
  std::optional<Eigen::AlignedBox2d> pit;
  static constexpr double kPitFloor = -1.0;
  bool unknown_beside_corridor = false, delay_voxel_queries = false;
  bool unknown_ground = false;
  mutable size_t voxel_queries = 0;
  double wall_x = 2;
  bool wall = false, narrow = false, boundary_unknown = false,
       interior_unknown = false;

 private:
  bool blind(const Eigen::Vector3d& p) const {
    return blind_radius > 0 && p.head<2>().norm() < blind_radius;
  }
  bool inPit(const Eigen::Vector3d& p) const {
    return pit && pit->contains(p.head<2>());
  }
  bool solid(const Eigen::Vector3d& p, const Eigen::Vector3d& size) const {
    const Eigen::AlignedBox3d box(p - size / 2, p + size / 2);
    return std::any_of(solids.begin(), solids.end(), [&](const auto& s) {
      return s.intersects(box);
    });
  }
  bool unseen(const Eigen::Vector3d& p, const Eigen::Vector3d& size) const {
    const Eigen::AlignedBox3d box(p - size / 2, p + size / 2);
    return std::any_of(unseen_volumes.begin(), unseen_volumes.end(),
                       [&](const auto& u) { return u.intersects(box); });
  }
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
