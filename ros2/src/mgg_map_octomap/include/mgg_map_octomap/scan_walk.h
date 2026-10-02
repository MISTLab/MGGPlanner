#ifndef MGG_MAP_OCTOMAP_SCAN_WALK_H_
#define MGG_MAP_OCTOMAP_SCAN_WALK_H_

#include "mgg_core/voxel_walk.h"

namespace mgg {

/// Scan-only version of walkVoxels. The scan's cell table already deduplicates
/// visits, so it needs neither the collision walk's initial-cell history nor
/// its endpoint-cell history on every step. Duplicate visits are intentional:
/// an occupied duplicate would already have stopped the ray at its first visit.
/// Keep the original endpoint, crossing arithmetic, ties and closed boundaries.
/// Collision/ground/body queries continue to use walkVoxels, never this helper.
template <class F>
bool walkScanVoxels(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                    double resolution, std::uint64_t max_work, F visit) {
  VoxelIndex current, target;
  if (!voxelIndexOf(a, resolution, current) ||
      !voxelIndexOf(b, resolution, target)) return false;
  const long double crossings =
      std::abs(static_cast<long double>(target.x) - current.x) +
      std::abs(static_cast<long double>(target.y) - current.y) +
      std::abs(static_cast<long double>(target.z) - current.z);
  if (15.0L + 7.0L * crossings > max_work) return false;
  const Eigen::Vector3d direction = b - a;
  std::array<int, 3> step{};
  Eigen::Vector3d next = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d delta = next;
  unsigned plane_mask = 0, initial_mask = 0;
  std::array<std::int64_t, 3> end_low{target.x, target.y, target.z};
  auto end_high = end_low;
  for (int axis = 0; axis < 3; ++axis) {
    if (direction[axis] != 0.0) {
      step[axis] = direction[axis] > 0.0 ? 1 : -1;
      const auto index = axis == 0 ? current.x : axis == 1 ? current.y : current.z;
      const double boundary = resolution * double(index + (step[axis] > 0));
      next[axis] = (boundary - a[axis]) / direction[axis];
      delta[axis] = resolution / std::abs(direction[axis]);
    }
    const double start_scaled = a[axis] / resolution;
    if (std::abs(start_scaled - std::round(start_scaled)) <=
        16.0 * std::numeric_limits<double>::epsilon() *
            std::max(1.0, std::abs(start_scaled))) {
      initial_mask |= 1u << axis;
      if (direction[axis] == 0.0) plane_mask |= 1u << axis;
    }
    const double end_scaled = b[axis] / resolution;
    if (std::abs(end_scaled - std::round(end_scaled)) <=
        16.0 * std::numeric_limits<double>::epsilon() *
            std::max(1.0, std::abs(end_scaled))) {
      end_high[axis] = std::llround(end_scaled);
      end_low[axis] = end_high[axis] - 1;
    }
  }
  auto call = [&](const VoxelIndex& cell, unsigned mask) {
    for (unsigned subset = mask;; subset = (subset - 1) & mask) {
      VoxelIndex touched = cell;
      if (subset & 1u) --touched.x;
      if (subset & 2u) --touched.y;
      if (subset & 4u) --touched.z;
      if (!visit(touched)) return false;
      if (subset == 0) return true;
    }
  };
  if (!call(current, initial_mask)) return true;
  while (current.x != target.x || current.y != target.y || current.z != target.z) {
    const unsigned open = (current.x != target.x ? 1u : 0u) |
                          (current.y != target.y ? 2u : 0u) |
                          (current.z != target.z ? 4u : 0u);
    double crossing = std::numeric_limits<double>::infinity();
    for (int axis = 0; axis < 3; ++axis)
      if (open & (1u << axis)) crossing = std::min(crossing, next[axis]);
    if (!std::isfinite(crossing)) return false;
    const double tolerance = 16.0 * std::numeric_limits<double>::epsilon() *
                             std::max(1.0, std::abs(crossing));
    unsigned mask = 0;
    for (int axis = 0; axis < 3; ++axis)
      if ((open & (1u << axis)) && std::abs(next[axis] - crossing) <= tolerance)
        mask |= 1u << axis;
    for (unsigned subset = mask; subset; subset = (subset - 1) & mask) {
      VoxelIndex touched = current;
      if (subset & 1u) touched.x += step[0];
      if (subset & 2u) touched.y += step[1];
      if (subset & 4u) touched.z += step[2];
      if (!call(touched, plane_mask)) return true;
    }
    if (mask & 1u) { current.x += step[0]; next.x() += delta.x(); }
    if (mask & 2u) { current.y += step[1]; next.y() += delta.y(); }
    if (mask & 4u) { current.z += step[2]; next.z() += delta.z(); }
  }
  for (auto x = end_low[0]; x <= end_high[0]; ++x)
    for (auto y = end_low[1]; y <= end_high[1]; ++y)
      for (auto z = end_low[2]; z <= end_high[2]; ++z)
        if (!visit(VoxelIndex{x, y, z})) return true;
  return true;
}

}  // namespace mgg
#endif  // MGG_MAP_OCTOMAP_SCAN_WALK_H_
