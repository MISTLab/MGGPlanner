// Test scene for the ground local planning core and node: a corridor along
// +x seen by a synthetic lidar, with an optional obstacle. Surfaces sit in
// the middle of 0.2 m voxels so returns never land on a voxel face.

#ifndef MGG_ROS_TEST_LOCAL_PLANNER_SCENE_H_
#define MGG_ROS_TEST_LOCAL_PLANNER_SCENE_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "../../mgg_map_octomap/test/synthetic_scan.h"
#include "mgg_ros/local_planning_core.h"

namespace mgg_test {

constexpr double kFloorTop = -0.1;
constexpr double kBodyHeight = 0.4;
/// Odometry base height: half the body over the floor.
constexpr double kBaseZ = kFloorTop + kBodyHeight / 2;
constexpr double kMaxGroundHeight = 0.5;
/// The planner's driving height over the floor (max_ground_height).
constexpr double kDrivingZ = kFloorTop + kMaxGroundHeight;
constexpr double kLidarAboveBase = 0.4;
constexpr double kWallY = 2.5;

inline mgg::StateVec basePose(double x, double y, double yaw = 0) {
  return mgg::StateVec(x, y, kBaseZ, yaw);
}
inline Eigen::Vector3d lidarOrigin(const mgg::StateVec& base) {
  return base.head<3>() + Eigen::Vector3d(0, 0, kLidarAboveBase);
}
/// A target at driving height.
inline Eigen::Vector3d drivingPoint(double x, double y) {
  return Eigen::Vector3d(x, y, kDrivingZ);
}

struct Corridor {
  // Floor, side walls, end walls and a ceiling: every ray of the window
  // returns, so free space is observed as well as surfaces.
  std::vector<Eigen::AlignedBox3d> solids{
      {Eigen::Vector3d(-30, -10, kFloorTop - 0.2),
       Eigen::Vector3d(40, 10, kFloorTop)},
      {Eigen::Vector3d(-30, kWallY, kFloorTop),
       Eigen::Vector3d(40, kWallY + 0.4, 2.1)},
      {Eigen::Vector3d(-30, -kWallY - 0.4, kFloorTop),
       Eigen::Vector3d(40, -kWallY, 2.1)},
      {Eigen::Vector3d(-14.5, -10, kFloorTop), Eigen::Vector3d(-14.1, 10, 2.1)},
      {Eigen::Vector3d(17.1, -10, kFloorTop), Eigen::Vector3d(17.5, 10, 2.1)},
      {Eigen::Vector3d(-30, -10, 2.1), Eigen::Vector3d(40, 10, 2.3)}};
  std::optional<Eigen::AlignedBox3d> obstacle;

  static mgg::test::SyntheticLidar lidar() {
    mgg::test::SyntheticLidar l;
    l.azimuth_rays = 180;
    l.elevation_rays = 32;
    l.min_elevation_rad = -1.4;
    l.max_elevation_rad = 0.3;
    l.max_range_m = 20;
    return l;
  }
  /// The lidar's returns, plus rays through each column's floor and body
  /// voxels across the window: a ring lidar alone leaves gaps between its
  /// rings that a standing robot never fills, and certified ground needs
  /// each column's own observed support and body band.
  std::vector<Eigen::Vector3d> scan(const Eigen::Vector3d& origin) const {
    mgg::test::SyntheticScene scene{solids};
    if (obstacle) scene.solids.push_back(*obstacle);
    auto points = mgg::test::syntheticScan(scene, origin, lidar());
    for (double dx = -8.3; dx <= 8.3; dx += 0.2) {
      for (double y = -kWallY + 0.1; y < kWallY; y += 0.2) {
        const double x = std::floor((origin.x() + dx) / 0.2) * 0.2 + 0.1;
        for (double z = kFloorTop; z < kFloorTop + 0.9; z += 0.2) {
          const Eigen::Vector3d direction =
              (Eigen::Vector3d(x, y, z) - origin).normalized();
          double nearest = std::numeric_limits<double>::infinity();
          for (const auto& solid : scene.solids) {
            const auto d = mgg::test::rayBoxDistance(origin, direction, solid);
            if (d && *d < nearest) nearest = *d;
          }
          if (nearest <= 20) points.push_back(origin + nearest * direction);
        }
      }
    }
    return points;
  }
  mgg::OdomScan odomScan(const mgg::StateVec& base) const {
    mgg::OdomScan s;
    s.origin = lidarOrigin(base);
    s.points = scan(s.origin);
    return s;
  }
};

inline mgg::LocalPlanningParams sceneParams() {
  mgg::LocalPlanningParams p;
  p.robot.type = mgg::RobotType::kGroundRobot;
  p.robot.size = {1.0, 0.6, kBodyHeight};
  p.robot.size_extension.setZero();
  p.robot.size_extension_min.setZero();
  p.robot.safety_extension.setZero();
  p.robot.bound_mode = mgg::BoundModeType::kExactBound;
  p.planning.max_ground_height = kMaxGroundHeight;
  p.planning.max_inclination = 0.7;
  p.planning.max_negative_inclination = 0.5;
  p.planning.min_observed_ground_fraction = 0.75;
  p.planning.v_max = 0.6;
  p.sensor.fov = {6.28, 0.2};
  p.sensor.resolution = {0.4, 0.2};
  p.sensor.max_range = 8;
  p.sensor.update();
  p.epoch = 77;
  return p;
}

inline double pathLength(const std::vector<mgg::StateVec>& poses) {
  double length = 0;
  for (size_t i = 1; i < poses.size(); ++i)
    length += (poses[i] - poses[i - 1]).head<2>().norm();
  return length;
}

/// Planar distance from `c` to segment ab.
inline double segmentDistance(const Eigen::Vector2d& a,
                              const Eigen::Vector2d& b,
                              const Eigen::Vector2d& c) {
  const Eigen::Vector2d ab = b - a;
  const double t =
      ab.squaredNorm() > 0
          ? std::clamp((c - a).dot(ab) / ab.squaredNorm(), 0.0, 1.0)
          : 0.0;
  return (a + t * ab - c).norm();
}

/// Whether the path's centre line comes within `reach` of `centre`.
inline bool pathCrosses(const std::vector<mgg::StateVec>& poses,
                        const Eigen::Vector2d& centre, double reach) {
  if (poses.size() == 1) return (poses[0].head<2>() - centre).norm() < reach;
  for (size_t i = 1; i < poses.size(); ++i)
    if (segmentDistance(poses[i - 1].head<2>(), poses[i].head<2>(), centre) <
        reach)
      return true;
  return false;
}

/// The arc length along `poses` of the point nearest `xy`.
inline double progressAlong(const std::vector<mgg::StateVec>& poses,
                            const Eigen::Vector2d& xy) {
  double best = std::numeric_limits<double>::infinity(), at = 0, along = 0;
  for (size_t i = 1; i < poses.size(); ++i) {
    const Eigen::Vector2d a = poses[i - 1].head<2>(), b = poses[i].head<2>();
    const double length = (b - a).norm();
    const double t =
        length > 0
            ? std::clamp((xy - a).dot(b - a) / (length * length), 0.0, 1.0)
            : 0.0;
    const double d = (a + t * (b - a) - xy).norm();
    if (d < best) {
      best = d;
      at = along + t * length;
    }
    along += length;
  }
  return at;
}

/// The pose `progress` metres along `poses`.
inline mgg::StateVec poseAt(const std::vector<mgg::StateVec>& poses,
                            double progress) {
  double along = 0;
  for (size_t i = 1; i < poses.size(); ++i) {
    const double length = (poses[i] - poses[i - 1]).head<2>().norm();
    if (along + length >= progress && length > 0) {
      const double t = (progress - along) / length;
      mgg::StateVec p = poses[i - 1] + t * (poses[i] - poses[i - 1]);
      p[3] = poses[i][3];
      return p;
    }
    along += length;
  }
  return poses.back();
}

/// Whether a `size` footprint at planar pose (x, y, yaw) overlaps box's XY.
inline bool footprintOverlaps(const mgg::StateVec& pose,
                              const Eigen::Vector2d& size,
                              const Eigen::AlignedBox3d& box) {
  const Eigen::Vector2d u(std::cos(pose[3]), std::sin(pose[3]));
  const Eigen::Vector2d v(-u.y(), u.x());
  const Eigen::Vector2d c = pose.head<2>();
  const Eigen::Vector2d lo = box.min().head<2>(), hi = box.max().head<2>();
  const Eigen::Vector2d bc = (lo + hi) / 2, bh = (hi - lo) / 2;
  const Eigen::Vector2d half = size / 2;
  for (const Eigen::Vector2d& axis :
       {Eigen::Vector2d(1, 0), Eigen::Vector2d(0, 1), u, v}) {
    const double r_robot =
        half.x() * std::abs(u.dot(axis)) + half.y() * std::abs(v.dot(axis));
    const double r_box =
        bh.x() * std::abs(axis.x()) + bh.y() * std::abs(axis.y());
    if (std::abs((c - bc).dot(axis)) > r_robot + r_box) return false;
  }
  return true;
}

}  // namespace mgg_test

#endif  // MGG_ROS_TEST_LOCAL_PLANNER_SCENE_H_
