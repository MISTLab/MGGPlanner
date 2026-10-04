// Test-only ray-cast scene builder: a lidar of evenly spaced rays against a
// scene of solid axis-aligned boxes. A ray returns the nearest box surface
// it meets within range; rays that meet nothing return no point.

#ifndef MGG_MAP_OCTOMAP_TEST_SYNTHETIC_SCAN_H_
#define MGG_MAP_OCTOMAP_TEST_SYNTHETIC_SCAN_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Geometry>

namespace mgg::test {

struct SyntheticScene {
  std::vector<Eigen::AlignedBox3d> solids;
};

struct SyntheticLidar {
  int azimuth_rays = 360;
  int elevation_rays = 16;
  double min_elevation_rad = -0.26;
  double max_elevation_rad = 0.26;
  double max_range_m = 20.0;
};

/// Distance along the unit ray `direction` from `origin` to `box`, if met.
inline std::optional<double> rayBoxDistance(const Eigen::Vector3d& origin,
                                            const Eigen::Vector3d& direction,
                                            const Eigen::AlignedBox3d& box) {
  double first = 0.0;
  double last = std::numeric_limits<double>::infinity();
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(direction[axis]) < 1e-12) {
      if (origin[axis] < box.min()[axis] || origin[axis] > box.max()[axis])
        return std::nullopt;
      continue;
    }
    double a = (box.min()[axis] - origin[axis]) / direction[axis];
    double b = (box.max()[axis] - origin[axis]) / direction[axis];
    if (a > b) std::swap(a, b);
    first = std::max(first, a);
    last = std::min(last, b);
    if (first > last) return std::nullopt;
  }
  return first;
}

/// The returns one sweep of `lidar` at `origin` gets from `scene`.
inline std::vector<Eigen::Vector3d> syntheticScan(
    const SyntheticScene& scene, const Eigen::Vector3d& origin,
    const SyntheticLidar& lidar = SyntheticLidar()) {
  std::vector<Eigen::Vector3d> points;
  for (int e = 0; e < lidar.elevation_rays; ++e) {
    const double elevation =
        lidar.elevation_rays == 1
            ? lidar.min_elevation_rad
            : lidar.min_elevation_rad +
                  (lidar.max_elevation_rad - lidar.min_elevation_rad) * e /
                      (lidar.elevation_rays - 1);
    for (int a = 0; a < lidar.azimuth_rays; ++a) {
      const double azimuth = 2.0 * M_PI * (a + 0.5) / lidar.azimuth_rays;
      const Eigen::Vector3d direction(std::cos(elevation) * std::cos(azimuth),
                                      std::cos(elevation) * std::sin(azimuth),
                                      std::sin(elevation));
      double nearest = std::numeric_limits<double>::infinity();
      for (const auto& solid : scene.solids) {
        const auto distance = rayBoxDistance(origin, direction, solid);
        if (distance && *distance < nearest) nearest = *distance;
      }
      if (nearest <= lidar.max_range_m)
        points.push_back(origin + nearest * direction);
    }
  }
  return points;
}

}  // namespace mgg::test

#endif  // MGG_MAP_OCTOMAP_TEST_SYNTHETIC_SCAN_H_
