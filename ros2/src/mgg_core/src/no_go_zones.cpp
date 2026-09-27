#include "mgg_core/no_go_zones.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mgg {
namespace {

constexpr double kEps = 1e-9;

/// Where along a -> b (0 to 1) the segment comes nearest `c`.
double nearestFraction(const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                       const Eigen::Vector2d& c) {
  const Eigen::Vector2d d = b - a;
  const double length_squared = d.squaredNorm();
  if (length_squared < 1e-18) return 0.0;
  return std::clamp((c - a).dot(d) / length_squared, 0.0, 1.0);
}

double nearestDistance(const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                       const Eigen::Vector2d& c) {
  return (c - (a + nearestFraction(a, b, c) * (b - a))).norm();
}

}  // namespace

void NoGoZones::set(std::vector<Eigen::Vector2d> centres, double reach) {
  centres_.clear();
  if (!(std::isfinite(reach) && reach > 0.0)) return;
  for (const Eigen::Vector2d& c : centres) {
    if (c.allFinite()) centres_.push_back(c);
  }
  reach_ = reach;
}

bool NoGoZones::inside(const Eigen::Vector3d& p) const {
  return std::any_of(centres_.begin(), centres_.end(),
                     [this, &p](const Eigen::Vector2d& c) {
                       return (p.head<2>() - c).norm() < reach_;
                     });
}

bool NoGoZones::pathAdmissible(const std::vector<Eigen::Vector3d>& path) const {
  if (centres_.empty() || path.empty()) return true;
  for (const Eigen::Vector2d& c : centres_) {
    // Departing: the path started inside this zone and has not left it.
    bool departing = (path.front().head<2>() - c).norm() < reach_;
    for (std::size_t i = 1; i < path.size(); ++i) {
      const Eigen::Vector2d a = path[i - 1].head<2>();
      const Eigen::Vector2d b = path[i].head<2>();
      if (departing) {
        // Monotonic outward: the segment is nearest the centre at its start.
        if (nearestFraction(a, b, c) > kEps &&
            nearestDistance(a, b, c) < (a - c).norm() - kEps) {
          return false;
        }
        if ((b - c).norm() >= reach_) departing = false;
        continue;
      }
      if (nearestDistance(a, b, c) < reach_) return false;
    }
    if ((path.back().head<2>() - c).norm() < reach_) return false;
  }
  return true;
}

bool NoGoZones::blocksEdge(const Eigen::Vector3d& a3, const Eigen::Vector3d& b3,
                           const Eigen::Vector3d& robot) const {
  const Eigen::Vector2d a = a3.head<2>();
  const Eigen::Vector2d b = b3.head<2>();
  for (const Eigen::Vector2d& c : centres_) {
    const double t = nearestFraction(a, b, c);
    const double nearest = (c - (a + t * (b - a))).norm();
    if (nearest >= reach_) continue;
    const double robot_distance = (robot.head<2>() - c).norm();
    const bool at_an_end = t <= kEps || t >= 1.0 - kEps;
    if (robot_distance < reach_ && at_an_end &&
        nearest >= robot_distance - 1e-6) {
      continue;  // only ever driven outward, on the robot's way out
    }
    return true;
  }
  return false;
}

}  // namespace mgg
