#include "mgg_core/dirty_region.h"

#include <cmath>
#include <limits>

namespace mgg {
namespace {
std::int32_t floorToKey(double scaled) {
  constexpr double lo = std::numeric_limits<std::int32_t>::min();
  constexpr double hi = std::numeric_limits<std::int32_t>::max();
  if (!std::isfinite(scaled)) return std::numeric_limits<std::int32_t>::min();
  const double f = std::floor(scaled);
  if (f <= lo) return std::numeric_limits<std::int32_t>::min();
  if (f >= hi) return std::numeric_limits<std::int32_t>::max();
  return static_cast<std::int32_t>(f);
}

// Floating-point slack for inclusive reach: box faces computed from voxel
// keys and from dependency arithmetic may differ by rounding. Erring towards
// reach withdraws more, never less.
constexpr double kReachTolerance = 1e-9;
}  // namespace

VoxelKey keyOf(const Eigen::Vector3d& p, double resolution) {
  return {floorToKey(p.x() / resolution), floorToKey(p.y() / resolution),
          floorToKey(p.z() / resolution)};
}

Eigen::Vector3d centerOf(const VoxelKey& key, double resolution) {
  return resolution * (Eigen::Vector3d(double(key.x), double(key.y),
                                       double(key.z)) +
                       Eigen::Vector3d::Constant(0.5));
}

std::vector<Eigen::AlignedBox3d> dirtyRegions(const MapChange& change,
                                              double halo_m) {
  if (change.everything) {
    const double inf = std::numeric_limits<double>::infinity();
    return {Eigen::AlignedBox3d(Eigen::Vector3d::Constant(-inf),
                                Eigen::Vector3d::Constant(inf))};
  }
  const Eigen::Vector3d halo =
      Eigen::Vector3d::Constant(std::isfinite(halo_m) && halo_m > 0 ? halo_m
                                                                    : 0.0);
  std::vector<Eigen::AlignedBox3d> regions;
  regions.reserve(change.boxes.size());
  for (const auto& box : change.boxes)
    regions.emplace_back(box.min() - halo, box.max() + halo);
  return regions;
}

bool changeReaches(const MapChange& change,
                   const Eigen::AlignedBox3d& dependency) {
  if (change.everything) return true;
  if (dependency.isEmpty()) return false;
  const Eigen::Vector3d slack = Eigen::Vector3d::Constant(kReachTolerance);
  for (const auto& box : change.boxes) {
    if (box.isEmpty()) continue;
    if (((box.min() - slack).array() <= dependency.max().array()).all() &&
        ((dependency.min() - slack).array() <= box.max().array()).all())
      return true;
  }
  return false;
}

}  // namespace mgg
