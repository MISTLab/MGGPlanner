// The voxels a segment touches, as the native planner grid walks them.
//
// Shared by NativeMolaGrid, whose ray queries and volumetric gain walk
// every cell a segment touches, and SensorParams, whose frontier test
// counts the distinct cells an unobstructed scan reaches: the two must
// agree cell for cell.

#ifndef MGG_CORE_VOXEL_WALK_H_
#define MGG_CORE_VOXEL_WALK_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <Eigen/Dense>

namespace mgg {

/// Integer coordinates of a voxel of a uniform grid anchored at the origin.
struct VoxelIndex {
  std::int64_t x = 0, y = 0, z = 0;
};

/// The voxel of edge `resolution` containing `p`. False for a point or a
/// resolution that is not finite, or a point too far out to index.
inline bool voxelIndexOf(const Eigen::Vector3d& p, double resolution,
                         VoxelIndex& k) {
  if (!p.allFinite() || !std::isfinite(resolution) || resolution <= 0)
    return false;
  constexpr double lim = double(std::numeric_limits<std::int64_t>::max()) / 4;
  const Eigen::Vector3d q = p / resolution;
  if ((q.array().abs() > lim).any()) return false;
  k = {std::int64_t(std::floor(q.x())), std::int64_t(std::floor(q.y())),
       std::int64_t(std::floor(q.z()))};
  return true;
}

/// Calls `visit` on every voxel the closed segment from `a` to `b` touches,
/// in order: where it passes through an edge or a corner, every voxel
/// meeting there, and where it lies on a grid plane, the voxels on both
/// sides. `visit` returns false to stop the walk. Returns false, before any
/// visit, for an invalid segment or one that would visit more than
/// `max_work` voxels; true otherwise.
template <class F>
bool walkVoxels(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                double resolution, std::uint64_t max_work, F visit) {
  VoxelIndex current, target;
  if (!voxelIndexOf(a, resolution, current) ||
      !voxelIndexOf(b, resolution, target))
    return false;
  const long double crossings =
      std::abs(static_cast<long double>(target.x) - current.x) +
      std::abs(static_cast<long double>(target.y) - current.y) +
      std::abs(static_cast<long double>(target.z) - current.z);
  // A 3-D corner may conservatively visit seven adjacent cells. Reject the
  // entire query before producing a partial occupancy result.
  // Include the at-most seven extra closed cells at each endpoint so a query
  // is rejected before its visitor observes any partial result.
  if (15.0L + 7.0L * crossings > max_work) return false;
  const Eigen::Vector3d direction = b - a;
  std::array<int, 3> step{};
  Eigen::Vector3d next =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d delta = next;
  for (int axis = 0; axis < 3; ++axis) {
    if (direction[axis] == 0.0) continue;
    step[axis] = direction[axis] > 0.0 ? 1 : -1;
    const std::int64_t index = axis == 0   ? current.x
                               : axis == 1 ? current.y
                                           : current.z;
    const double boundary = resolution * double(index + (step[axis] > 0));
    next[axis] = (boundary - a[axis]) / direction[axis];
    delta[axis] = resolution / std::abs(direction[axis]);
  }
  // The voxels touching the end point: on each axis, the voxel holding it,
  // or both voxels meeting at a boundary it lies on. The walk stops in one
  // of them; those it did not enter, where the end is an edge or a corner,
  // are visited after it, once each.
  std::array<std::int64_t, 3> end_low{target.x, target.y, target.z};
  std::array<std::int64_t, 3> end_high = end_low;
  for (int axis = 0; axis < 3; ++axis) {
    const double scaled = b[axis] / resolution;
    if (std::abs(scaled - std::round(scaled)) <=
        16.0 * std::numeric_limits<double>::epsilon() *
            std::max(1.0, std::abs(scaled))) {
      end_high[axis] = std::llround(scaled);
      end_low[axis] = end_high[axis] - 1;
    }
  }
  std::array<VoxelIndex, 8> end_cells_visited{};
  std::size_t end_cells_visited_count = 0;
  const auto isEndCell = [&](const VoxelIndex& c) {
    return c.x >= end_low[0] && c.x <= end_high[0] && c.y >= end_low[1] &&
           c.y <= end_high[1] && c.z >= end_low[2] && c.z <= end_high[2];
  };
  const auto endCellVisited = [&](const VoxelIndex& c) {
    return std::any_of(
        end_cells_visited.begin(),
        end_cells_visited.begin() + end_cells_visited_count,
        [&](const VoxelIndex& v) {
          return v.x == c.x && v.y == c.y && v.z == c.z;
        });
  };
  std::uint64_t work = 0;
  unsigned plane_mask = 0;
  unsigned initial_boundary_mask = 0;
  for (int axis = 0; axis < 3; ++axis) {
    const double scaled = a[axis] / resolution;
    if (std::abs(scaled - std::round(scaled)) <=
        16.0 * std::numeric_limits<double>::epsilon() *
            std::max(1.0, std::abs(scaled))) {
      initial_boundary_mask |= 1u << axis;
      if (direction[axis] == 0.0) plane_mask |= 1u << axis;
    }
  }
  std::array<VoxelIndex, 8> initial_cells{};
  std::size_t initial_cell_count = 0;
  bool recording_initial_cells = true;
  auto call = [&](const VoxelIndex& cell) {
    // A ray lying on a grid plane touches cells on both sides for its whole
    // length. Expand those stationary axes before applying the visitor.
    for (unsigned subset = plane_mask;; subset = (subset - 1) & plane_mask) {
      VoxelIndex touched = cell;
      if (subset & 1u) --touched.x;
      if (subset & 2u) --touched.y;
      if (subset & 4u) --touched.z;
      if (recording_initial_cells) {
        initial_cells[initial_cell_count++] = touched;
      } else {
        const bool already_visited = std::any_of(
            initial_cells.begin(), initial_cells.begin() + initial_cell_count,
            [&](const VoxelIndex& initial) {
              return initial.x == touched.x && initial.y == touched.y &&
                     initial.z == touched.z;
            });
        if (already_visited) {
          if (subset == 0) break;
          continue;
        }
      }
      if (++work > max_work) return -1;
      if (isEndCell(touched) && !endCellVisited(touched) &&
          end_cells_visited_count < end_cells_visited.size()) {
        end_cells_visited[end_cells_visited_count++] = touched;
      }
      if (!visit(touched)) return 0;
      if (subset == 0) break;
    }
    return 1;
  };
  const unsigned stationary_plane_mask = plane_mask;
  plane_mask = initial_boundary_mask;
  int action = call(current);
  if (action <= 0) return action == 0;
  recording_initial_cells = false;
  plane_mask = stationary_plane_mask;
  while (current.x != target.x || current.y != target.y ||
         current.z != target.z) {
    // Only axes short of the target cell take part. Where the segment ends
    // on a voxel boundary, rounding can put that boundary's crossing within
    // the tie tolerance of another axis's; stepping such an axis past its
    // target cell left the walk unable to finish until it ran out of work.
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
    // A boundary edge or corner belongs to every adjacent closed cell for
    // conservative ray semantics. Visit all non-empty tied-axis subsets.
    for (unsigned subset = mask; subset != 0; subset = (subset - 1) & mask) {
      VoxelIndex touched = current;
      if (subset & 1u) touched.x += step[0];
      if (subset & 2u) touched.y += step[1];
      if (subset & 4u) touched.z += step[2];
      action = call(touched);
      if (action <= 0) return action == 0;
    }
    if (mask & 1u) {
      current.x += step[0];
      next.x() += delta.x();
    }
    if (mask & 2u) {
      current.y += step[1];
      next.y() += delta.y();
    }
    if (mask & 4u) {
      current.z += step[2];
      next.z() += delta.z();
    }
  }
  // The walk does not step across a boundary the end lies on, and an axis
  // that reached its target does not step with another whose crossing ties
  // at the end: end voxels in any mix of directions may be left unentered.
  for (auto x = end_low[0]; x <= end_high[0]; ++x)
    for (auto y = end_low[1]; y <= end_high[1]; ++y)
      for (auto z = end_low[2]; z <= end_high[2]; ++z) {
        const VoxelIndex touched{x, y, z};
        if (endCellVisited(touched)) continue;
        if (++work > max_work) return false;
        if (!visit(touched)) return true;
      }
  return true;
}

}  // namespace mgg

#endif  // MGG_CORE_VOXEL_WALK_H_
