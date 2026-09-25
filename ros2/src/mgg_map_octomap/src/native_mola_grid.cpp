#include "mgg_map_octomap/native_mola_grid.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace mgg {
namespace {
struct CellHash {
  std::size_t operator()(const NativeMolaGrid::Cell& k) const {
    std::uint64_t h = std::uint64_t(k.x) * 0x9e3779b97f4a7c15ULL;
    h ^= std::uint64_t(k.y) * 0xc2b2ae3d27d4eb4fULL + (h << 6) + (h >> 2);
    h ^= std::uint64_t(k.z) * 0x165667b19e3779f9ULL + (h << 6) + (h >> 2);
    return std::size_t(h);
  }
};
struct CellEqual {
  bool operator()(const NativeMolaGrid::Cell& a,
                  const NativeMolaGrid::Cell& b) const {
    return a.x == b.x && a.y == b.y && a.z == b.z;
  }
};
/// What a scan has made of each cell it visited: every visit of every ray
/// goes through it, about 65k cells for the 648-ray, 20 m simulated VLP16,
/// so it is open-addressed, and it is kept per thread and emptied by
/// advancing a generation rather than by clearing a few megabytes per scan.
/// A cell's state is 0 until the scan assigns it one.
class ScanCells {
 public:
  void reset(std::size_t expected) {
    count_ = 0;
    if (++generation_ == 0) {
      std::fill(generations_.begin(), generations_.end(), 0);
      generation_ = 1;
    }
    if (cells_.size() < 2 * expected) rehash(2 * expected);
  }
  std::uint8_t& operator[](const NativeMolaGrid::Cell& k) {
    if (2 * (count_ + 1) > cells_.size()) rehash(2 * cells_.size());
    const std::size_t mask = cells_.size() - 1;
    std::size_t pos = CellHash()(k) & mask;
    while (generations_[pos] == generation_) {
      if (CellEqual()(cells_[pos], k)) return states_[pos];
      pos = (pos + 1) & mask;
    }
    generations_[pos] = generation_;
    cells_[pos] = k;
    states_[pos] = 0;
    ++count_;
    return states_[pos];
  }

 private:
  void rehash(std::size_t wanted) {
    std::size_t capacity = 64;
    while (capacity < wanted) capacity *= 2;
    std::vector<NativeMolaGrid::Cell> cells(capacity);
    std::vector<std::uint32_t> generations(capacity, 0);
    std::vector<std::uint8_t> states(capacity, 0);
    for (std::size_t i = 0; i < cells_.size(); ++i) {
      if (generations_[i] != generation_) continue;
      std::size_t pos = CellHash()(cells_[i]) & (capacity - 1);
      while (generations[pos] == generation_) pos = (pos + 1) & (capacity - 1);
      generations[pos] = generation_;
      cells[pos] = cells_[i];
      states[pos] = states_[i];
    }
    cells_.swap(cells);
    generations_.swap(generations);
    states_.swap(states);
  }
  std::vector<NativeMolaGrid::Cell> cells_;
  std::vector<std::uint32_t> generations_;
  std::vector<std::uint8_t> states_;
  std::uint32_t generation_ = 0;
  std::size_t count_ = 0;
};

double pointSegmentDistance2(const Eigen::Vector2d& p, const Eigen::Vector2d& a,
                             const Eigen::Vector2d& b) {
  const auto d = b - a;
  const double n = d.squaredNorm();
  const double t = n <= 1e-24 ? 0.0 : std::clamp((p - a).dot(d) / n, 0.0, 1.0);
  return (p - (a + t * d)).squaredNorm();
}
double segmentCellDistance2(const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                            const Eigen::Vector2d& c, double h) {
  const Eigen::Vector2d lo = c - Eigen::Vector2d::Constant(h),
                        hi = c + Eigen::Vector2d::Constant(h);
  double first = 0, last = 1;
  const auto d = b - a;
  bool intersects = true;
  for (int q = 0; q < 2; ++q) {
    if (std::abs(d[q]) <= 1e-15) {
      if (a[q] < lo[q] || a[q] > hi[q]) intersects = false;
    } else {
      double x = (lo[q] - a[q]) / d[q], y = (hi[q] - a[q]) / d[q];
      if (x > y) std::swap(x, y);
      first = std::max(first, x);
      last = std::min(last, y);
      if (first > last) intersects = false;
    }
  }
  if (intersects) return 0;
  double best = std::numeric_limits<double>::infinity();
  for (double x : {lo.x(), hi.x()})
    for (double y : {lo.y(), hi.y()})
      best = std::min(best, pointSegmentDistance2({x, y}, a, b));
  auto pd = [&](const Eigen::Vector2d& p) {
    auto o = (lo - p).cwiseMax(0.0) + (p - hi).cwiseMax(0.0);
    return o.squaredNorm();
  };
  return std::min({best, pd(a), pd(b)});
}
}  // namespace

NativeMolaGrid::NativeMolaGrid(double r, std::vector<Cell> o,
                               std::vector<Cell> f, std::vector<Surface> s)
    : resolution_(r), occupied_(std::move(o)), free_(std::move(f)) {
  std::sort(occupied_.begin(), occupied_.end());
  std::sort(free_.begin(), free_.end());
  cell_index_.build(occupied_, free_);
  for (std::size_t first = 0; first < occupied_.size();) {
    std::size_t last = first + 1;
    while (last < occupied_.size() && occupied_[last].x == occupied_[first].x &&
           occupied_[last].y == occupied_[first].y)
      ++last;
    occupied_columns_.emplace(
        std::make_pair(occupied_[first].x, occupied_[first].y),
        std::make_pair(first, last));
    for (std::size_t i = first + 1; i < last; ++i) {
      const Cell& low = occupied_[i - 1];
      const Cell& high = occupied_[i];
      if (high.z - low.z > kMaxWallGapVoxels + 1) continue;
      for (auto z = low.z + 1; z < high.z; ++z)
        gap_candidates_.push_back({low.x, low.y, z});
    }
    first = last;
  }
  gap_candidate_index_.build(gap_candidates_, no_cells_);
  for (const auto& v : s)
    if (std::isfinite(v.max_z)) {
      auto it = surface_max_z_.find(v.cell);
      if (it == surface_max_z_.end())
        surface_max_z_.emplace(v.cell, v.max_z);
      else
        it->second = std::max(it->second, v.max_z);
    }
}
std::size_t NativeMolaGrid::ColumnHash::operator()(
    const std::pair<std::int64_t, std::int64_t>& column) const {
  return CellHash()({column.first, column.second, 0});
}
bool NativeMolaGrid::key(const Eigen::Vector3d& p, Cell& k) const {
  if (!p.allFinite() || !std::isfinite(resolution_) || resolution_ <= 0)
    return false;
  constexpr double lim = double(std::numeric_limits<std::int64_t>::max()) / 4;
  const auto q = p / resolution_;
  if ((q.array().abs() > lim).any()) return false;
  k = {std::int64_t(std::floor(q.x())), std::int64_t(std::floor(q.y())),
       std::int64_t(std::floor(q.z()))};
  return true;
}
Eigen::Vector3d NativeMolaGrid::center(const Cell& k) const {
  return resolution_ * (Eigen::Vector3d(double(k.x), double(k.y), double(k.z)) +
                        Eigen::Vector3d::Constant(.5));
}
VoxelStatus NativeMolaGrid::status(const Cell& k) const {
  switch (cell_index_.status(k, occupied_, free_)) {
    case 2: return VoxelStatus::kOccupied;
    case 1: return VoxelStatus::kFree;
    default: return VoxelStatus::kUnknown;
  }
}
bool NativeMolaGrid::getAxisAlignedXYCellCenter(const Eigen::Vector2d& p,
                                                Eigen::Vector2d& c) const {
  Cell k;
  if (!key({p.x(), p.y(), 0}, k)) return false;
  c = resolution_ * (Eigen::Vector2d(double(k.x), double(k.y)) +
                     Eigen::Vector2d::Constant(.5));
  return c.allFinite();
}
VoxelStatus NativeMolaGrid::getVoxelStatus(const Eigen::Vector3d& p) const {
  Cell k;
  return key(p, k) ? status(k) : VoxelStatus::kUnknown;
}

template <class F>
bool NativeMolaGrid::walk(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                          F visit) const {
  Cell current, target;
  if (!key(a, current) || !key(b, target)) return false;
  const long double crossings =
      std::abs(static_cast<long double>(target.x) - current.x) +
      std::abs(static_cast<long double>(target.y) - current.y) +
      std::abs(static_cast<long double>(target.z) - current.z);
  // A 3-D corner may conservatively visit seven adjacent cells. Reject the
  // entire query before producing a partial occupancy result.
  // Include the at-most seven extra closed cells at each endpoint so a query
  // is rejected before its visitor observes any partial result.
  if (15.0L + 7.0L * crossings > kMaxWork) return false;
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
    const double boundary = resolution_ * double(index + (step[axis] > 0));
    next[axis] = (boundary - a[axis]) / direction[axis];
    delta[axis] = resolution_ / std::abs(direction[axis]);
  }
  std::uint64_t work = 0;
  unsigned plane_mask = 0;
  unsigned initial_boundary_mask = 0;
  for (int axis = 0; axis < 3; ++axis) {
    const double scaled = a[axis] / resolution_;
    if (std::abs(scaled - std::round(scaled)) <=
        16.0 * std::numeric_limits<double>::epsilon() *
            std::max(1.0, std::abs(scaled))) {
      initial_boundary_mask |= 1u << axis;
      if (direction[axis] == 0.0) plane_mask |= 1u << axis;
    }
  }
  std::array<Cell, 8> initial_cells{};
  std::size_t initial_cell_count = 0;
  bool recording_initial_cells = true;
  auto call = [&](const Cell& cell) {
    // A ray lying on a grid plane touches cells on both sides for its whole
    // length. Expand those stationary axes before applying the visitor.
    for (unsigned subset = plane_mask;; subset = (subset - 1) & plane_mask) {
      Cell touched = cell;
      if (subset & 1u) --touched.x;
      if (subset & 2u) --touched.y;
      if (subset & 4u) --touched.z;
      if (recording_initial_cells) {
        initial_cells[initial_cell_count++] = touched;
      } else {
        const bool already_visited = std::any_of(
            initial_cells.begin(), initial_cells.begin() + initial_cell_count,
            [&](const Cell& initial) {
              return initial.x == touched.x && initial.y == touched.y &&
                     initial.z == touched.z;
            });
        if (already_visited) {
          if (subset == 0) break;
          continue;
        }
      }
      if (++work > kMaxWork) return -1;
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
    const double crossing = next.minCoeff();
    if (!std::isfinite(crossing)) return false;
    const double tolerance = 16.0 * std::numeric_limits<double>::epsilon() *
                             std::max(1.0, std::abs(crossing));
    unsigned mask = 0;
    for (int axis = 0; axis < 3; ++axis)
      if (std::abs(next[axis] - crossing) <= tolerance) mask |= 1u << axis;
    // A boundary edge or corner belongs to every adjacent closed cell for
    // conservative ray semantics. Visit all non-empty tied-axis subsets.
    for (unsigned subset = mask; subset != 0; subset = (subset - 1) & mask) {
      Cell touched = current;
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
  unsigned negative_endpoint_mask = 0;
  for (int axis = 0; axis < 3; ++axis) {
    const double scaled = b[axis] / resolution_;
    if (direction[axis] < 0.0 &&
        std::abs(scaled - std::round(scaled)) <=
            16.0 * std::numeric_limits<double>::epsilon() *
                std::max(1.0, std::abs(scaled))) {
      negative_endpoint_mask |= 1u << axis;
    }
  }
  const unsigned endpoint_mask = negative_endpoint_mask | plane_mask;
  for (unsigned subset = endpoint_mask; subset != 0;
       subset = (subset - 1) & endpoint_mask) {
    if ((subset & negative_endpoint_mask) == 0) continue;
    Cell touched = target;
    if (subset & 1u) --touched.x;
    if (subset & 2u) --touched.y;
    if (subset & 4u) --touched.z;
    if (++work > kMaxWork) return false;
    if (!visit(touched)) return true;
  }
  return true;
}
VoxelStatus NativeMolaGrid::getRayStatus(const Eigen::Vector3d& a,
                                         const Eigen::Vector3d& b,
                                         bool u) const {
  Eigen::Vector3d e;
  return getRayStatus(a, b, u, e);
}
VoxelStatus NativeMolaGrid::getRayStatus(const Eigen::Vector3d& a,
                                         const Eigen::Vector3d& b, bool u,
                                         Eigen::Vector3d& e) const {
  VoxelStatus out = VoxelStatus::kFree;
  e = b;
  if (!walk(a, b, [&](const Cell& k) {
        auto s = status(k);
        if (s == VoxelStatus::kOccupied || (u && s == VoxelStatus::kUnknown)) {
          out = s;
          e = center(k);
          return false;
        }
        return true;
      })) {
    e = a;
    return VoxelStatus::kUnknown;
  }
  return out;
}
VoxelStatus NativeMolaGrid::getGroundRayStatus(const Eigen::Vector3d& a,
                                               const Eigen::Vector3d& b, bool u,
                                               Eigen::Vector3d& e) const {
  auto s = getRayStatus(a, b, u, e);
  if (s == VoxelStatus::kOccupied) {
    Cell k;
    if (key(e, k)) {
      auto it = surface_max_z_.find(k);
      if (it != surface_max_z_.end()) e.z() = it->second;
    }
  }
  return s;
}

VoxelStatus NativeMolaGrid::box(const Eigen::Vector3d& c,
                                const Eigen::Vector3d& s, bool unknown,
                                bool measured) const {
  if (!c.allFinite() || !s.allFinite() || (s.array() < 0).any())
    return VoxelStatus::kUnknown;
  const Eigen::Vector3d lo = c - s * .5;
  const Eigen::Vector3d hi = c + s * .5;
  Cell first, last;
  if (!key(lo - Eigen::Vector3d::Constant(1e-12), first) ||
      !key(hi + Eigen::Vector3d::Constant(1e-12), last))
    return VoxelStatus::kUnknown;
  const long double work = static_cast<long double>(last.x - first.x + 1) *
                           static_cast<long double>(last.y - first.y + 1) *
                           static_cast<long double>(last.z - first.z + 1);
  if (work > kMaxWork) return VoxelStatus::kUnknown;
  bool saw_unknown = false;
  for (auto x = first.x; x <= last.x; ++x)
    for (auto y = first.y; y <= last.y; ++y)
      for (auto z = first.z; z <= last.z; ++z) {
        Cell k{x, y, z};
        auto st = status(k);
        if (st == VoxelStatus::kOccupied) {
          if (measured) {
            auto it = surface_max_z_.find(k);
            if (it != surface_max_z_.end() && it->second < lo.z() - 1e-12)
              continue;
          }
          return st;
        }
        if (st == VoxelStatus::kUnknown) saw_unknown = true;
      }
  return unknown && saw_unknown ? VoxelStatus::kUnknown : VoxelStatus::kFree;
}
VoxelStatus NativeMolaGrid::getBoxStatus(const Eigen::Vector3d& c,
                                         const Eigen::Vector3d& s,
                                         bool u) const {
  return box(c, s, u, !u);
}
VoxelStatus NativeMolaGrid::getStrictBoxStatus(const Eigen::Vector3d& c,
                                               const Eigen::Vector3d& s) const {
  return box(c, s, true, false);
}
VoxelStatus NativeMolaGrid::path(const Eigen::Vector3d& a,
                                 const Eigen::Vector3d& b,
                                 const Eigen::Vector3d& s, bool u,
                                 bool measured) const {
  if (!a.allFinite() || !b.allFinite() || !s.allFinite() ||
      (s.array() < 0).any())
    return VoxelStatus::kUnknown;
  double n_d = std::max(1.0, std::ceil((b - a).norm() / resolution_));
  if (!std::isfinite(n_d) || n_d > double(kMaxWork))
    return VoxelStatus::kUnknown;
  auto step = (b - a) / n_d;
  auto swept = s + step.cwiseAbs();
  long double cells = 1;
  for (int axis = 0; axis < 3; ++axis)
    cells *= std::ceil(swept[axis] / resolution_) + 3.0;
  if (cells * n_d > kMaxWork) return VoxelStatus::kUnknown;
  for (std::uint64_t i = 0; i < std::uint64_t(n_d); ++i) {
    auto st = box(a + (double(i) + .5) * step, swept, u, measured);
    if (st != VoxelStatus::kFree) return st;
  }
  return VoxelStatus::kFree;
}
VoxelStatus NativeMolaGrid::getPathStatus(const Eigen::Vector3d& a,
                                          const Eigen::Vector3d& b,
                                          const Eigen::Vector3d& s,
                                          bool u) const {
  return path(a, b, s, u, !u);
}
VoxelStatus NativeMolaGrid::getStrictPathStatus(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b,
    const Eigen::Vector3d& s) const {
  return path(a, b, s, true, false);
}

VoxelStatus NativeMolaGrid::getOccupiedOnlyCylinderPathStatus(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b, double radius,
    double height) const {
  if (!a.allFinite() || !b.allFinite() || !std::isfinite(radius) ||
      radius < 0 || !std::isfinite(height) || height < 0)
    return VoxelStatus::kUnknown;
  double n_d = std::max(1.0, std::ceil((b - a).norm() / resolution_));
  if (!std::isfinite(n_d) || n_d > double(kMaxWork))
    return VoxelStatus::kUnknown;
  auto step = (b - a) / n_d;
  std::uint64_t work = 0;
  const double half = resolution_ * .5;
  for (std::uint64_t i = 0; i < std::uint64_t(n_d); ++i) {
    const Eigen::Vector3d p = a + double(i) * step;
    const Eigen::Vector3d q = p + step;
    Eigen::Vector3d lo(std::min(p.x(), q.x()) - radius,
                       std::min(p.y(), q.y()) - radius,
                       std::min(p.z(), q.z()) - height * .5),
        hi(std::max(p.x(), q.x()) + radius, std::max(p.y(), q.y()) + radius,
           std::max(p.z(), q.z()) + height * .5);
    Cell first, last;
    if (!key(lo - Eigen::Vector3d::Constant(resolution_), first) ||
        !key(hi + Eigen::Vector3d::Constant(resolution_), last))
      return VoxelStatus::kUnknown;
    const long double count = static_cast<long double>(last.x - first.x + 1) *
                              static_cast<long double>(last.y - first.y + 1) *
                              static_cast<long double>(last.z - first.z + 1);
    if (count > kMaxWork - work) return VoxelStatus::kUnknown;
    work += std::uint64_t(count);
    for (auto x = first.x; x <= last.x; ++x)
      for (auto y = first.y; y <= last.y; ++y)
        for (auto z = first.z; z <= last.z; ++z) {
          Cell k{x, y, z};
          if (status(k) != VoxelStatus::kOccupied) continue;
          auto cc = center(k);
          if (segmentCellDistance2(p.head<2>(), q.head<2>(), cc.head<2>(),
                                   half) > (radius + 1e-12) * (radius + 1e-12))
            continue;
          double top = cc.z() + half;
          auto it = surface_max_z_.find(k);
          if (it != surface_max_z_.end()) top = std::min(top, it->second);
          if (top < lo.z() - 1e-12 || cc.z() - half > hi.z() + 1e-12) continue;
          return VoxelStatus::kOccupied;
        }
  }
  return VoxelStatus::kFree;
}

void NativeMolaGrid::getScanStatus(
    const Eigen::Vector3d& p, const std::vector<Eigen::Vector3d>& ends,
    GainCounts& g, std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& log,
    const SensorModel&) {
  g = {};
  for (const auto& e : ends) {
    const bool valid = walk(p, e, [&](const Cell& k) {
      const auto s = status(k);
      log.emplace_back(center(k), s);
      if (s == VoxelStatus::kOccupied) {
        ++g.occupied;
        return false;
      }
      if (s == VoxelStatus::kUnknown)
        ++g.unknown;
      else
        ++g.free;
      return true;
    });
    if (!valid) {
      ++g.unknown;
      log.emplace_back(p, VoxelStatus::kUnknown);
    }
  }
}

void NativeMolaGrid::getScanStatusIterative(
    const Eigen::Vector3d& p, const std::vector<Eigen::Vector3d>& ends,
    GainCounts& g, std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& log,
    const SensorModel&) {
  scanUnique(p, ends, nullptr, g, log);
}
void NativeMolaGrid::getVisibleScanStatus(
    const Eigen::Vector3d& p, const std::vector<Eigen::Vector3d>& ends,
    const WallBand& wall, GainCounts& g,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& log,
    const SensorModel&) {
  scanUnique(p, ends, &wall, g, log);
}
void NativeMolaGrid::scanUnique(
    const Eigen::Vector3d& p, const std::vector<Eigen::Vector3d>& ends,
    const WallBand* wall, GainCounts& g,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& log) const {
  g = {};
  // The wall band, a band of heights along wall->up: the caller's vertical
  // in this grid's frame, which a tilted frame makes lean. A band that is
  // not finite or inverted, or along a vertical tilted more than 60
  // degrees, turns the wall rule off rather than making every gap a wall's
  // or none.
  const bool walls =
      wall != nullptr && std::isfinite(wall->min_z) &&
      std::isfinite(wall->max_z) && wall->min_z <= wall->max_z &&
      wall->up.allFinite() && wall->up.z() > 0.5 * wall->up.norm();
  // A wall return is an occupied voxel whose centre's height is in the band.
  const auto inBand = [&](const Cell& c) {
    const double height = wall->up.dot(center(c));
    return height >= wall->min_z && height <= wall->max_z;
  };
  // The nearest wall return below, then one above close enough to it, from
  // the column's occupied voxels. Most unknown voxels are no gap candidate.
  const auto isWallGap = [&](const Cell& k) {
    if (gap_candidate_index_.status(k, gap_candidates_, no_cells_) == 0)
      return false;
    const auto column = occupied_columns_.find({k.x, k.y});
    if (column == occupied_columns_.end()) return false;
    const auto first = occupied_.begin() + column->second.first;
    const auto last = occupied_.begin() + column->second.second;
    const auto above = std::upper_bound(
        first, last, k.z,
        [](std::int64_t z, const Cell& cell) { return z < cell.z; });
    std::int64_t below = 0;
    for (auto it = above; it != first;) {
      --it;
      if (k.z - it->z > kMaxWallGapVoxels + 1) break;
      if (it->z < k.z && inBand(*it)) {
        below = k.z - it->z;
        break;
      }
    }
    if (below == 0) return false;
    for (auto it = above; it != last; ++it) {
      if (below + (it->z - k.z) - 1 > kMaxWallGapVoxels) break;
      if (inBand(*it)) return true;
    }
    return false;
  };
  // A cell's status and the wall rule are decided on its first visit; a
  // later ray only needs the verdict.
  enum : std::uint8_t { kNew = 0, kCounted, kCountedOccupied, kWallGap };
  thread_local ScanCells cells;
  cells.reset(ends.size() * 64);
  for (const auto& e : ends) {
    const bool valid = walk(p, e, [&](const Cell& k) {
      std::uint8_t& state = cells[k];
      if (state != kNew) return state == kCounted;
      const auto s = status(k);
      // A gap in a wall hides what is behind it, for every ray.
      if (walls && s == VoxelStatus::kUnknown && isWallGap(k)) {
        state = kWallGap;
        return false;
      }
      state = s == VoxelStatus::kOccupied ? kCountedOccupied : kCounted;
      log.emplace_back(center(k), s);
      if (s == VoxelStatus::kOccupied)
        ++g.occupied;
      else if (s == VoxelStatus::kUnknown)
        ++g.unknown;
      else
        ++g.free;
      return s != VoxelStatus::kOccupied;
    });
    if (!valid) {
      ++g.unknown;
      log.emplace_back(p, VoxelStatus::kUnknown);
    }
  }
}
void NativeMolaGrid::extractLocalMap(const Eigen::Vector3d& c,
                                     const Eigen::Vector3d& s,
                                     std::vector<Eigen::Vector3d>& o,
                                     std::vector<Eigen::Vector3d>& f) {
  o.clear();
  f.clear();
  auto add = [&](const std::vector<Cell>& in, auto& out) {
    for (const auto& k : in) {
      auto p = center(k);
      if (((p - c).array().abs() <= s.array() * .5).all()) out.push_back(p);
    }
  };
  add(occupied_, o);
  add(free_, f);
}
void NativeMolaGrid::extractLocalMapAlongAxis(const Eigen::Vector3d& c,
                                              const Eigen::Vector3d& axis,
                                              const Eigen::Vector3d& s,
                                              std::vector<Eigen::Vector3d>& o,
                                              std::vector<Eigen::Vector3d>& f) {
  const double radius = s.norm();
  extractLocalMap(c, Eigen::Vector3d::Constant(radius), o, f);
  const Eigen::Vector3d direction =
      axis.norm() > 1e-9 ? axis.normalized() : Eigen::Vector3d::UnitX();
  const Eigen::Matrix3d frame =
      Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), direction)
          .toRotationMatrix();
  auto outside = [&](const Eigen::Vector3d& p) {
    return ((frame.transpose() * (p - c)).array().abs() > s.array() * .5).any();
  };
  o.erase(std::remove_if(o.begin(), o.end(), outside), o.end());
  f.erase(std::remove_if(f.begin(), f.end(), outside), f.end());
}
void NativeMolaGrid::getLocalPointcloud(const Eigen::Vector3d& c, double range,
                                        double,
                                        std::vector<Eigen::Vector3d>& points,
                                        bool include_free) {
  points.clear();
  if (!c.allFinite() || !std::isfinite(range) || range < 0) return;
  const auto append = [&](const std::vector<Cell>& cells) {
    for (const auto& k : cells) {
      const auto p = center(k);
      if ((p - c).norm() <= range) points.push_back(p);
    }
  };
  append(occupied_);
  if (include_free) append(free_);
}
void NativeMolaGrid::getFreeSpacePointCloud(
    const std::vector<Eigen::Vector3d>& ends, const StateVec& s,
    std::vector<Eigen::Vector3d>& p) {
  p.clear();
  for (const auto& e : ends)
    walk(s.head<3>(), e, [&](const Cell& k) {
      auto st = status(k);
      if (st == VoxelStatus::kFree) p.push_back(center(k));
      return st != VoxelStatus::kOccupied;
    });
}
}  // namespace mgg
