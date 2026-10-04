#include "mgg_map_octomap/rolling_voxel_map.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <unordered_map>

#include "mgg_core/planning_cancellation.h"
#include "mgg_core/voxel_walk.h"

namespace mgg {
namespace {

double logit(double p) { return std::log(p / (1.0 - p)); }

// OctomapConfig's defaults.
const float kLogHit = float(logit(0.7));
const float kLogMiss = float(logit(0.4));
const float kLogClampMin = float(logit(0.12));
const float kLogClampMax = float(logit(0.97));
const float kLogOccupied = float(logit(0.5));
constexpr double kMaxRange = 20.0;

/// Bounds every query's voxel enumeration, as NativeMolaGrid does.
constexpr std::uint64_t kMaxWork = 1u << 22;
/// Largest dense window, in voxels (64 Mi: 320 MB of state).
constexpr long double kMaxWindowVoxels = 1u << 26;
/// Changed voxels are reported as the bounding box of the changed voxels of
/// each 4x4x4 block: at most 0.8 m boxes at 0.2 m, and a bounded box count.
constexpr std::int64_t kChangeBlock = 4;

std::int64_t floorDiv(std::int64_t a, std::int64_t b) {
  return a >= 0 ? a / b : -((-a + b - 1) / b);
}

std::int64_t positiveMod(std::int64_t a, std::int64_t n) {
  const std::int64_t m = a % n;
  return m < 0 ? m + n : m;
}
}  // namespace

RollingVoxelMap::RollingVoxelMap(const RollingWindowParams& params)
    : resolution_(params.resolution), follow_vertical_(params.follow_vertical) {
  if (!std::isfinite(resolution_) || resolution_ <= 0.0)
    throw std::invalid_argument("RollingVoxelMap: resolution must be positive");
  long double voxels = 1;
  for (int axis = 0; axis < 3; ++axis) {
    const double size = params.window_size_m[axis];
    if (!std::isfinite(size) || size <= 0.0)
      throw std::invalid_argument(
          "RollingVoxelMap: window size must be positive");
    const double cells = std::max(1.0, std::round(size / resolution_));
    if (!std::isfinite(cells) || cells > double(kMaxWindowVoxels))
      throw std::invalid_argument("RollingVoxelMap: window too large");
    dims_[axis] = std::int64_t(cells);
    voxels *= cells;
  }
  if (voxels > kMaxWindowVoxels)
    throw std::invalid_argument("RollingVoxelMap: window too large");
  const auto count = std::size_t(voxels);
  log_odds_.assign(count, 0.0f);
  status_.assign(count, VoxelStatus::kUnknown);
  scan_mark_.assign(count, 0);
}

// ------------------------------------------------------------ geometry

bool RollingVoxelMap::inWindow(std::int64_t x, std::int64_t y,
                               std::int64_t z) const {
  return placed_ && x >= origin_.x && x < origin_.x + dims_[0] &&
         y >= origin_.y && y < origin_.y + dims_[1] && z >= origin_.z &&
         z < origin_.z + dims_[2];
}

std::size_t RollingVoxelMap::slot(std::int64_t x, std::int64_t y,
                                  std::int64_t z) const {
  return std::size_t((positiveMod(x, dims_[0]) * dims_[1] +
                      positiveMod(y, dims_[1])) *
                         dims_[2] +
                     positiveMod(z, dims_[2]));
}

VoxelStatus RollingVoxelMap::status(std::int64_t x, std::int64_t y,
                                    std::int64_t z) const {
  return inWindow(x, y, z) ? status_[slot(x, y, z)] : VoxelStatus::kUnknown;
}

Eigen::AlignedBox3d RollingVoxelMap::windowAt(const VoxelKey& origin) const {
  const Eigen::Vector3d lo{double(origin.x), double(origin.y),
                           double(origin.z)};
  const Eigen::Vector3d n{double(dims_[0]), double(dims_[1]), double(dims_[2])};
  return Eigen::AlignedBox3d(resolution_ * lo, resolution_ * (lo + n));
}

Eigen::AlignedBox3d RollingVoxelMap::window() const {
  return placed_ ? windowAt(origin_) : Eigen::AlignedBox3d();
}

bool RollingVoxelMap::cellOf(const Eigen::Vector3d& p, VoxelKey& key) const {
  if (!placed_ || !p.allFinite() || !window().contains(p)) return false;
  key = mgg::keyOf(p, resolution_);
  // The max faces belong to the last voxel; rounding may also land a point
  // on a face just outside. Both stay inside the closed window.
  key.x = std::int32_t(std::clamp<std::int64_t>(key.x, origin_.x,
                                                origin_.x + dims_[0] - 1));
  key.y = std::int32_t(std::clamp<std::int64_t>(key.y, origin_.y,
                                                origin_.y + dims_[1] - 1));
  key.z = std::int32_t(std::clamp<std::int64_t>(key.z, origin_.z,
                                                origin_.z + dims_[2] - 1));
  return true;
}

VoxelKey RollingVoxelMap::originFor(const Eigen::Vector3d& robot) const {
  const VoxelKey k = mgg::keyOf(robot, resolution_);
  auto start = [](std::int32_t center, std::int64_t n) {
    constexpr std::int64_t lo = std::numeric_limits<std::int32_t>::min();
    const std::int64_t hi = std::numeric_limits<std::int32_t>::max() - n;
    return std::int32_t(std::clamp<std::int64_t>(center - n / 2, lo, hi));
  };
  return {start(k.x, dims_[0]), start(k.y, dims_[1]), start(k.z, dims_[2])};
}

VoxelKey RollingVoxelMap::keyOf(const Eigen::Vector3d& p) const {
  return mgg::keyOf(p, resolution_);
}

Eigen::Vector3d RollingVoxelMap::centerOf(const VoxelKey& key) const {
  return mgg::centerOf(key, resolution_);
}

void RollingVoxelMap::clearCell(std::size_t s) {
  log_odds_[s] = 0.0f;
  status_[s] = VoxelStatus::kUnknown;
}

void RollingVoxelMap::clearAll() {
  std::fill(log_odds_.begin(), log_odds_.end(), 0.0f);
  std::fill(status_.begin(), status_.end(), VoxelStatus::kUnknown);
}

// ------------------------------------------------------------ mutation

MapChange RollingVoxelMap::recenter(const Eigen::Vector3d& robot) {
  MapChange change;
  change.revision = revision_;
  if (!robot.allFinite()) return change;
  VoxelKey target = originFor(robot);
  if (!placed_) {
    origin_ = target;
    placed_ = true;
    clearAll();
    ++revision_;
    change.boxes.push_back(window());
    change.revision = revision_;
    return change;
  }
  if (!follow_vertical_) target.z = origin_.z;
  if (target == origin_) return change;

  const VoxelKey old = origin_;
  const Eigen::AlignedBox3d old_box = window();
  origin_ = target;
  const Eigen::AlignedBox3d new_box = window();
  const std::array<std::int64_t, 3> old_lo{old.x, old.y, old.z};
  const std::array<std::int64_t, 3> new_lo{target.x, target.y, target.z};
  bool disjoint = false;
  for (int axis = 0; axis < 3; ++axis)
    if (std::abs(new_lo[axis] - old_lo[axis]) >= dims_[axis]) disjoint = true;
  if (disjoint) {
    clearAll();
    change.boxes = {old_box, new_box};
  } else {
    for (int axis = 0; axis < 3; ++axis) {
      const std::int64_t shift = new_lo[axis] - old_lo[axis];
      if (shift == 0) continue;
      // Along `axis`, the old window's part outside the new one is evicted
      // and the new window's part outside the old one enters.
      const std::int64_t evict_lo =
          shift > 0 ? old_lo[axis] : new_lo[axis] + dims_[axis];
      const std::int64_t evict_hi =
          shift > 0 ? new_lo[axis] : old_lo[axis] + dims_[axis];
      const std::int64_t enter_lo =
          shift > 0 ? old_lo[axis] + dims_[axis] : new_lo[axis];
      const std::int64_t enter_hi =
          shift > 0 ? new_lo[axis] + dims_[axis] : old_lo[axis];
      Eigen::AlignedBox3d evicted = old_box;
      evicted.min()[axis] = resolution_ * double(evict_lo);
      evicted.max()[axis] = resolution_ * double(evict_hi);
      Eigen::AlignedBox3d entering = new_box;
      entering.min()[axis] = resolution_ * double(enter_lo);
      entering.max()[axis] = resolution_ * double(enter_hi);
      change.boxes.push_back(evicted);
      change.boxes.push_back(entering);
      // The entering voxels reuse the evicted voxels' slots.
      std::array<std::int64_t, 3> lo = new_lo;
      std::array<std::int64_t, 3> hi{new_lo[0] + dims_[0], new_lo[1] + dims_[1],
                                     new_lo[2] + dims_[2]};
      lo[axis] = enter_lo;
      hi[axis] = enter_hi;
      for (auto x = lo[0]; x < hi[0]; ++x)
        for (auto y = lo[1]; y < hi[1]; ++y)
          for (auto z = lo[2]; z < hi[2]; ++z) clearCell(slot(x, y, z));
    }
  }
  ++revision_;
  change.revision = revision_;
  return change;
}

MapChange RollingVoxelMap::reset(const Eigen::Vector3d& robot) {
  clearAll();
  has_data_ = false;
  placed_ = robot.allFinite();
  if (placed_) origin_ = originFor(robot);
  ++revision_;
  MapChange change;
  change.everything = true;
  change.revision = revision_;
  return change;
}

void RollingVoxelMap::resetMap() {
  clearAll();
  has_data_ = false;
  ++revision_;
}

MapChange RollingVoxelMap::insertScan(const std::vector<Eigen::Vector3d>& points,
                                      const Eigen::Vector3d& origin,
                                      const Eigen::Vector3d& robot) {
  MapChange change = recenter(robot);
  if (!placed_ || !origin.allFinite()) return change;
  enum : std::uint8_t { kUntouched = 0, kMissed = 1, kHit = 2 };
  const auto mark = [&](std::int64_t x, std::int64_t y, std::int64_t z,
                        std::uint8_t what) {
    const std::size_t s = slot(x, y, z);
    if (scan_mark_[s] == kUntouched)
      scan_touched_.emplace_back(
          s, VoxelKey{std::int32_t(x), std::int32_t(y), std::int32_t(z)});
    scan_mark_[s] = std::max(scan_mark_[s], what);
  };
  for (const Eigen::Vector3d& point : points) {
    if (!point.allFinite()) continue;
    const Eigen::Vector3d ray = point - origin;
    const double length = ray.norm();
    if (!std::isfinite(length) || length <= 0.0) continue;
    const bool hit = length <= kMaxRange;
    const Eigen::Vector3d end = hit ? point : origin + ray * (kMaxRange / length);
    has_data_ = true;
    walkVoxels(origin, end, resolution_, kMaxWork, [&](const VoxelIndex& v) {
      if (!inWindow(v.x, v.y, v.z)) return true;
      // Voxels whose closed box holds the end belong to the return, not to
      // the free space the ray crossed.
      const Eigen::Vector3d lo =
          resolution_ * Eigen::Vector3d(double(v.x), double(v.y), double(v.z));
      if ((end.array() >= lo.array()).all() &&
          (end.array() <= (lo.array() + resolution_)).all())
        return true;
      mark(v.x, v.y, v.z, kMissed);
      return true;
    });
    if (hit) {
      VoxelIndex v;
      if (voxelIndexOf(end, resolution_, v) && inWindow(v.x, v.y, v.z))
        mark(v.x, v.y, v.z, kHit);
    }
  }

  // Each touched voxel is updated once; a hit anywhere in the scan wins.
  using Block = std::array<std::int64_t, 3>;
  std::map<Block, Eigen::AlignedBox3d> changed;
  for (const auto& [s, key] : scan_touched_) {
    const std::uint8_t what = scan_mark_[s];
    scan_mark_[s] = kUntouched;
    float value = status_[s] == VoxelStatus::kUnknown ? 0.0f : log_odds_[s];
    value = std::clamp(value + (what == kHit ? kLogHit : kLogMiss),
                       kLogClampMin, kLogClampMax);
    log_odds_[s] = value;
    const VoxelStatus now =
        value >= kLogOccupied ? VoxelStatus::kOccupied : VoxelStatus::kFree;
    if (now == status_[s]) continue;
    status_[s] = now;
    const Eigen::Vector3d lo =
        resolution_ * Eigen::Vector3d(double(key.x), double(key.y), double(key.z));
    const Eigen::AlignedBox3d cell(lo, lo + Eigen::Vector3d::Constant(resolution_));
    const Block block{floorDiv(key.x, kChangeBlock), floorDiv(key.y, kChangeBlock),
                      floorDiv(key.z, kChangeBlock)};
    auto [it, inserted] = changed.emplace(block, cell);
    if (!inserted) it->second.extend(cell);
  }
  scan_touched_.clear();
  if (!changed.empty()) {
    ++revision_;
    for (const auto& entry : changed) change.boxes.push_back(entry.second);
  }
  change.revision = revision_;
  return change;
}

// ------------------------------------------------------------ queries

template <class F>
bool RollingVoxelMap::walk(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           F visit) const {
  std::size_t visited = 0;
  return walkVoxels(a, b, resolution_, kMaxWork, [&](const VoxelIndex& v) {
    if ((visited++ & 63u) == 0) planningCheckpoint();
    return visit(v);
  });
}

bool RollingVoxelMap::getAxisAlignedXYCellCenter(const Eigen::Vector2d& p,
                                                 Eigen::Vector2d& c) const {
  VoxelIndex v;
  if (!voxelIndexOf({p.x(), p.y(), 0.0}, resolution_, v)) return false;
  c = resolution_ *
      (Eigen::Vector2d(double(v.x), double(v.y)) + Eigen::Vector2d::Constant(.5));
  return c.allFinite();
}

VoxelStatus RollingVoxelMap::getVoxelStatus(const Eigen::Vector3d& p) const {
  VoxelKey k;
  return cellOf(p, k) ? status_[slot(k.x, k.y, k.z)] : VoxelStatus::kUnknown;
}

VoxelStatus RollingVoxelMap::getRayStatus(const Eigen::Vector3d& a,
                                          const Eigen::Vector3d& b,
                                          bool stop_at_unknown) const {
  Eigen::Vector3d end;
  return getRayStatus(a, b, stop_at_unknown, end);
}

VoxelStatus RollingVoxelMap::getRayStatus(const Eigen::Vector3d& a,
                                          const Eigen::Vector3d& b,
                                          bool stop_at_unknown,
                                          Eigen::Vector3d& end) const {
  VoxelStatus out = VoxelStatus::kFree;
  end = b;
  if (!walk(a, b, [&](const VoxelIndex& v) {
        const VoxelStatus s = status(v.x, v.y, v.z);
        if (s == VoxelStatus::kOccupied ||
            (stop_at_unknown && s == VoxelStatus::kUnknown)) {
          out = s;
          end = resolution_ * (Eigen::Vector3d(double(v.x), double(v.y),
                                               double(v.z)) +
                               Eigen::Vector3d::Constant(.5));
          return false;
        }
        return true;
      })) {
    end = a;
    return VoxelStatus::kUnknown;
  }
  return out;
}

VoxelStatus RollingVoxelMap::box(const Eigen::Vector3d& c,
                                 const Eigen::Vector3d& s,
                                 bool stop_at_unknown) const {
  if (!c.allFinite() || !s.allFinite() || (s.array() < 0).any())
    return VoxelStatus::kUnknown;
  VoxelIndex first, last;
  if (!voxelIndexOf(c - s * .5 - Eigen::Vector3d::Constant(1e-12), resolution_,
                    first) ||
      !voxelIndexOf(c + s * .5 + Eigen::Vector3d::Constant(1e-12), resolution_,
                    last))
    return VoxelStatus::kUnknown;
  const long double work = static_cast<long double>(last.x - first.x + 1) *
                           static_cast<long double>(last.y - first.y + 1) *
                           static_cast<long double>(last.z - first.z + 1);
  if (work > kMaxWork) return VoxelStatus::kUnknown;
  bool saw_unknown = false;
  for (auto x = first.x; x <= last.x; ++x)
    for (auto y = first.y; y <= last.y; ++y) {
      planningCheckpoint();
      for (auto z = first.z; z <= last.z; ++z) {
        const VoxelStatus st = status(x, y, z);
        if (st == VoxelStatus::kOccupied) return st;
        if (st == VoxelStatus::kUnknown) saw_unknown = true;
      }
    }
  return stop_at_unknown && saw_unknown ? VoxelStatus::kUnknown
                                        : VoxelStatus::kFree;
}

VoxelStatus RollingVoxelMap::getBoxStatus(const Eigen::Vector3d& c,
                                          const Eigen::Vector3d& s,
                                          bool stop_at_unknown) const {
  return box(c, s, stop_at_unknown);
}

VoxelStatus RollingVoxelMap::getPathStatus(const Eigen::Vector3d& a,
                                           const Eigen::Vector3d& b,
                                           const Eigen::Vector3d& s,
                                           bool stop_at_unknown) const {
  if (!a.allFinite() || !b.allFinite() || !s.allFinite() ||
      (s.array() < 0).any())
    return VoxelStatus::kUnknown;
  const double n_d = std::max(1.0, std::ceil((b - a).norm() / resolution_));
  if (!std::isfinite(n_d) || n_d > double(kMaxWork))
    return VoxelStatus::kUnknown;
  const Eigen::Vector3d step = (b - a) / n_d;
  const Eigen::Vector3d swept = s + step.cwiseAbs();
  long double cells = 1;
  for (int axis = 0; axis < 3; ++axis)
    cells *= std::ceil(swept[axis] / resolution_) + 3.0;
  if (cells * n_d > kMaxWork) return VoxelStatus::kUnknown;
  for (std::uint64_t i = 0; i < std::uint64_t(n_d); ++i) {
    const VoxelStatus st =
        box(a + (double(i) + .5) * step, swept, stop_at_unknown);
    if (st != VoxelStatus::kFree) return st;
  }
  return VoxelStatus::kFree;
}

void RollingVoxelMap::getScanStatus(
    const Eigen::Vector3d& p, const std::vector<Eigen::Vector3d>& ends,
    GainCounts& g, std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& log,
    const SensorModel&) {
  g = {};
  for (const auto& e : ends) {
    planningCheckpoint();
    ++g.rays_cast;
    const bool valid = walk(p, e, [&](const VoxelIndex& v) {
      ++g.voxel_visits;
      const VoxelStatus s = status(v.x, v.y, v.z);
      log.emplace_back(resolution_ * (Eigen::Vector3d(double(v.x), double(v.y),
                                                      double(v.z)) +
                                      Eigen::Vector3d::Constant(.5)),
                       s);
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

void RollingVoxelMap::getScanStatusIterative(
    const Eigen::Vector3d& p, const std::vector<Eigen::Vector3d>& ends,
    GainCounts& g, std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& log,
    const SensorModel&) {
  g = {};
  // A voxel's status is counted on its first visit; a later ray only needs
  // to know whether it passes.
  std::unordered_map<VoxelKey, bool, VoxelKeyHash> passes;
  for (const auto& e : ends) {
    planningCheckpoint();
    ++g.rays_cast;
    const bool valid = walk(p, e, [&](const VoxelIndex& v) {
      ++g.voxel_visits;
      const VoxelKey key{std::int32_t(v.x), std::int32_t(v.y),
                         std::int32_t(v.z)};
      const auto found = passes.find(key);
      if (found != passes.end()) return found->second;
      const VoxelStatus s = status(v.x, v.y, v.z);
      passes.emplace(key, s != VoxelStatus::kOccupied);
      log.emplace_back(centerOf(key), s);
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

void RollingVoxelMap::extractLocalMap(const Eigen::Vector3d& c,
                                      const Eigen::Vector3d& s,
                                      std::vector<Eigen::Vector3d>& occupied,
                                      std::vector<Eigen::Vector3d>& free) {
  occupied.clear();
  free.clear();
  if (!placed_ || !c.allFinite() || !s.allFinite() || (s.array() < 0).any())
    return;
  VoxelIndex first, last;
  if (!voxelIndexOf(c - s * .5, resolution_, first) ||
      !voxelIndexOf(c + s * .5, resolution_, last))
    return;
  const auto x0 = std::max<std::int64_t>(first.x, origin_.x);
  const auto x1 = std::min<std::int64_t>(last.x, origin_.x + dims_[0] - 1);
  const auto y0 = std::max<std::int64_t>(first.y, origin_.y);
  const auto y1 = std::min<std::int64_t>(last.y, origin_.y + dims_[1] - 1);
  const auto z0 = std::max<std::int64_t>(first.z, origin_.z);
  const auto z1 = std::min<std::int64_t>(last.z, origin_.z + dims_[2] - 1);
  for (auto x = x0; x <= x1; ++x)
    for (auto y = y0; y <= y1; ++y)
      for (auto z = z0; z <= z1; ++z) {
        const VoxelStatus st = status_[slot(x, y, z)];
        if (st == VoxelStatus::kUnknown) continue;
        const Eigen::Vector3d p =
            resolution_ * (Eigen::Vector3d(double(x), double(y), double(z)) +
                           Eigen::Vector3d::Constant(.5));
        if (((p - c).array().abs() > s.array() * .5).any()) continue;
        (st == VoxelStatus::kOccupied ? occupied : free).push_back(p);
      }
}

void RollingVoxelMap::extractLocalMapAlongAxis(
    const Eigen::Vector3d& c, const Eigen::Vector3d& axis,
    const Eigen::Vector3d& s, std::vector<Eigen::Vector3d>& occupied,
    std::vector<Eigen::Vector3d>& free) {
  const double radius = s.norm();
  extractLocalMap(c, Eigen::Vector3d::Constant(radius), occupied, free);
  const Eigen::Vector3d direction =
      axis.norm() > 1e-9 ? axis.normalized() : Eigen::Vector3d::UnitX();
  const Eigen::Matrix3d frame =
      Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), direction)
          .toRotationMatrix();
  const auto outside = [&](const Eigen::Vector3d& p) {
    return ((frame.transpose() * (p - c)).array().abs() > s.array() * .5).any();
  };
  occupied.erase(std::remove_if(occupied.begin(), occupied.end(), outside),
                 occupied.end());
  free.erase(std::remove_if(free.begin(), free.end(), outside), free.end());
}

void RollingVoxelMap::getLocalPointcloud(const Eigen::Vector3d& c, double range,
                                         double,
                                         std::vector<Eigen::Vector3d>& points,
                                         bool include_unknown_voxels) {
  points.clear();
  if (!c.allFinite() || !std::isfinite(range) || range < 0) return;
  std::vector<Eigen::Vector3d> occupied, free;
  extractLocalMap(c, Eigen::Vector3d::Constant(2.0 * range), occupied, free);
  for (const auto& p : occupied)
    if ((p - c).norm() <= range) points.push_back(p);
  if (!include_unknown_voxels) return;
  for (const auto& p : free)
    if ((p - c).norm() <= range) points.push_back(p);
}

void RollingVoxelMap::getFreeSpacePointCloud(
    const std::vector<Eigen::Vector3d>& ends, const StateVec& state,
    std::vector<Eigen::Vector3d>& points) {
  points.clear();
  const Eigen::Vector3d origin = state.head<3>();
  for (const auto& e : ends) {
    planningCheckpoint();
    walk(origin, e, [&](const VoxelIndex& v) {
      const VoxelStatus st = status(v.x, v.y, v.z);
      if (st == VoxelStatus::kFree)
        points.push_back(resolution_ * (Eigen::Vector3d(double(v.x), double(v.y),
                                                        double(v.z)) +
                                        Eigen::Vector3d::Constant(.5)));
      return st != VoxelStatus::kOccupied;
    });
  }
}

}  // namespace mgg
