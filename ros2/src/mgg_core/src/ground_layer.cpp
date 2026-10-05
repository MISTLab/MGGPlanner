#include "mgg_core/ground_layer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace mgg {
namespace {
bool refused(GroundVerdict verdict) {
  return verdict == GroundVerdict::kRefusedStepGrade ||
         verdict == GroundVerdict::kRefusedOverhang;
}
constexpr int kDirections[8][2] = {
    {1, 0}, {1, 1}, {0, 1}, {-1, 1},
    {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
}  // namespace

GroundLayer::GroundLayer(const MapInterface& map, const PlanningParams& planning,
                         const RobotParams& robot, const GroundLayerParams& params)
    : map_(map), planning_(planning),
      projection_length_(GroundProjection(map_, planning_).max_projection_length),
      robot_params_(robot), params_(params),
      resolution_(map.getResolution()) {
  if (!std::isfinite(resolution_) || resolution_ <= 0 ||
      !params_.window_size_m.allFinite() ||
      (params_.window_size_m.array() <= 0).any())
    throw std::invalid_argument("GroundLayer: invalid grid dimensions");
}

int GroundLayer::index(const Eigen::Vector2d& p) const {
  if (!p.allFinite() || width_ == 0 || height_ == 0) return -1;
  const Eigen::Vector2d ij = (p - origin_) / resolution_;
  if ((ij.array() < 0).any() || ij.x() > width_ || ij.y() > height_) return -1;
  const int x = std::min(width_ - 1, static_cast<int>(std::floor(ij.x())));
  const int y = std::min(height_ - 1, static_cast<int>(std::floor(ij.y())));
  return y * width_ + x;
}

Eigen::Vector2d GroundLayer::center(int i) const {
  return origin_ + resolution_ * Eigen::Vector2d(i % width_ + 0.5, i / width_ + 0.5);
}

Eigen::AlignedBox3d GroundLayer::dependency(int i, double z) const {
  const double probe = std::max(0.2, 2 * resolution_);
  const Eigen::Vector3d size = robot_params_.getPlanningSize();
  const double radius = std::max(2 * resolution_ + probe + resolution_,
                                 size.head<2>().norm() / 2 + resolution_);
  const Eigen::Vector2d p = center(i);
  // Includes all eight native step/grade rays, their four offset probes,
  // and the body band. The parent's rays are added before projection.
  return Eigen::AlignedBox3d(
      Eigen::Vector3d(p.x() - radius, p.y() - radius,
                      z - projection_length_ - resolution_),
      Eigen::Vector3d(p.x() + radius, p.y() + radius,
       z + std::max(probe, planning_.max_ground_height +
                    std::abs(robot_params_.center_offset.z()) + size.z()) + resolution_));
}

void GroundLayer::reset(const Eigen::Vector3d& robot, double ground_z) {
  place(robot, ground_z, true);
}
void GroundLayer::recenter(const Eigen::Vector3d& robot, double ground_z) {
  place(robot, ground_z, false);
}
void GroundLayer::place(const Eigen::Vector3d& robot, double ground_z, bool clear) {
  if (!robot.allFinite() || !std::isfinite(ground_z))
    throw std::invalid_argument("GroundLayer: invalid anchor");
  robot_ = robot;
  Eigen::Vector2d origin, extent;
  const auto bounds = map_.windowBounds();
  if (bounds) {
    if (bounds->isEmpty()) {
      columns_.clear();
      width_ = height_ = 0;
      seed_ = -1;
      return;
    }
    origin = bounds->min().head<2>();
    extent = bounds->sizes().head<2>();
  } else {
    extent = params_.window_size_m;
    const Eigen::Vector2d cells = (extent / resolution_).array().round();
    origin = (robot.head<2>() / resolution_).array().floor().matrix() * resolution_ -
             (cells.array() / 2).floor().matrix() * resolution_;
  }
  const Eigen::Vector2d counts = (extent / resolution_).array().round();
  if (!origin.allFinite() || !counts.allFinite() || (counts.array() < 1).any() ||
      counts.x() * counts.y() > 1000000)
    throw std::invalid_argument("GroundLayer: invalid window bounds");
  const int width = static_cast<int>(counts.x());
  const int height = static_cast<int>(counts.y());
  if (!clear && origin.isApprox(origin_, 1e-12) && width == width_ && height == height_) {
    refreshUnsupportedSeed();
    return;
  }

  const auto old_origin = origin_;
  const int old_width = width_, old_height = height_, old_seed = seed_;
  auto old = std::move(columns_);
  origin_ = origin;
  width_ = width;
  height_ = height;
  columns_.assign(width_ * height_, Column{});
  std::vector<int> remap(old.size(), -1);
  if (!clear) {
    for (int i = 0; i < old_width * old_height; ++i) {
      const Eigen::Vector2d p = old_origin + resolution_ *
          Eigen::Vector2d(i % old_width + 0.5, i / old_width + 0.5);
      const int j = index(p);
      if (j >= 0 && center(j).isApprox(p, 1e-9)) {
        columns_[j] = old[i];
        remap[i] = j;
      }
    }
  }
  seed_ = !clear && old_seed >= 0 ? remap[old_seed] : -1;
  for (int i = 0; i < static_cast<int>(columns_.size()); ++i) {
    auto& column = columns_[i];
    if (column.dependency.isEmpty()) column.dependency = dependency(i, ground_z);
    if (column.parent >= 0) {
      column.parent = remap[column.parent];
      if (column.parent < 0) markPending(i);
    }
  }
  if (seed_ < 0) {
    seed_ = index(robot.head<2>());
    seed_ground_hint_ = robot.z() - planning_.max_ground_height;
    // Losing the flood anchor invalidates the entire retained tree.
    for (int i = 0; i < static_cast<int>(columns_.size()); ++i) markPending(i);
    if (seed_ >= 0) columns_[seed_].parent = -1;
  }
  if (!clear && old_width > 0 && old_height > 0) {
    // Recenter is itself a withdrawal boundary, even before the driver's
    // MapChange arrives: retained columns can depend on evicted rays.
    const Eigen::Vector2d old_max = old_origin +
        resolution_ * Eigen::Vector2d(old_width, old_height);
    const Eigen::Vector2d new_max = origin_ +
        resolution_ * Eigen::Vector2d(width_, height_);
    const Eigen::Vector2d low = old_origin.cwiseMin(origin_);
    const Eigen::Vector2d high = old_max.cwiseMax(new_max);
    const double infinity = std::numeric_limits<double>::infinity();
    MapChange scroll;
    for (int axis = 0; axis < 2; ++axis) {
      for (int face = 0; face < 2; ++face) {
        const double before = face == 0 ? old_origin[axis] : old_max[axis];
        const double after = face == 0 ? origin_[axis] : new_max[axis];
        if (std::abs(before - after) < 1e-9) continue;
        Eigen::Vector3d slab_low(low.x(), low.y(), -infinity);
        Eigen::Vector3d slab_high(high.x(), high.y(), infinity);
        slab_low[axis] = std::min(before, after);
        slab_high[axis] = std::max(before, after);
        scroll.boxes.emplace_back(slab_low, slab_high);
      }
    }
    withdraw(scroll);
  } else {
    withdrawDescendants();
  }
  refreshUnsupportedSeed();
}

void GroundLayer::refreshUnsupportedSeed() {
  if (seed_ >= 0 && columns_[seed_].observed) return;
  const int seed = index(robot_.head<2>());
  const double hint = robot_.z() - planning_.max_ground_height;
  if (seed < 0 || (seed == seed_ && std::abs(hint - seed_ground_hint_) < 1e-9))
    return;
  seed_ = seed;
  seed_ground_hint_ = hint;
  columns_[seed_].parent = -1;
  markPending(seed_);
  withdrawDescendants();
}

void GroundLayer::markPending(int i) {
  auto& column = columns_[i];
  column.dirty = true;
  if (column.verdict == GroundVerdict::kAdmitted)
    column.verdict = GroundVerdict::kPending;
}
void GroundLayer::withdrawDescendants() {
  std::vector<std::vector<int>> children(columns_.size());
  std::queue<int> queue;
  for (int i = 0; i < static_cast<int>(columns_.size()); ++i) {
    if (columns_[i].parent >= 0) children[columns_[i].parent].push_back(i);
    if (columns_[i].dirty) queue.push(i);
  }
  while (!queue.empty()) {
    const int i = queue.front();
    queue.pop();
    for (const int child : children[i]) {
      if (!columns_[child].dirty) {
        markPending(child);
        queue.push(child);
      }
    }
  }
}
void GroundLayer::setStandingStart(const std::optional<StandingStart>& standing) {
  const bool same =
      standing_.has_value() == standing.has_value() &&
      (!standing || (standing_->center == standing->center &&
                     standing_->radius == standing->radius));
  if (same) return;
  standing_ = standing;
  // Admissions may rest on the old disk anywhere the flood went through it.
  for (int i = 0; i < static_cast<int>(columns_.size()); ++i) markPending(i);
}
void GroundLayer::withdraw(const MapChange& change) {
  for (int i = 0; i < static_cast<int>(columns_.size()); ++i)
    if (changeReaches(change, columns_[i].dependency)) markPending(i);
  withdrawDescendants();
}

void GroundLayer::recheck(std::chrono::steady_clock::time_point deadline) {
  using Clock = std::chrono::steady_clock;
  if (seed_ < 0 || Clock::now() >= deadline) return;
  const auto* parent_cancelled = planning_cancelled;
  PlanningCancellationScope budget([&] {
    return Clock::now() >= deadline || (parent_cancelled && (*parent_cancelled)());
  });
  GroundProjection projection(map_, planning_);
  projection.max_projection_length = projection_length_;
  using Work = std::pair<double, int>;
  std::priority_queue<Work, std::vector<Work>, std::greater<Work>> queue;
  std::vector<bool> queued(columns_.size(), false);
  std::vector<int> parents(columns_.size(), -1);
  const auto enqueue = [&](int i, int parent) {
    if (i < 0 || queued[i] || !columns_[i].dirty) return;
    queued[i] = true;
    parents[i] = parent;
    queue.emplace((center(i) - robot_.head<2>()).squaredNorm(), i);
  };
  const auto neighbours = [&](int i, const auto& visit) {
    const int x = i % width_, y = i / width_;
    for (const auto& direction : kDirections) {
      const int nx = x + direction[0], ny = y + direction[1];
      if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_)
        visit(ny * width_ + nx);
    }
  };
  try {
    enqueue(seed_, -1);
    for (int i = 0; i < static_cast<int>(columns_.size()); ++i) {
      planningCheckpoint();
      if (!columns_[i].dirty && columns_[i].observed)
        neighbours(i, [&](int child) { enqueue(child, i); });
    }
    while (!queue.empty()) {
      planningCheckpoint();
      const int i = queue.top().second;
      queue.pop();
      Column checked = columns_[i];
      checked.evaluated = true;
      checked.standing = false;
      checked.parent = parents[i];
      const double reference = checked.parent < 0
          ? seed_ground_hint_
          : columns_[checked.parent].ground_z;
      checked.dependency = dependency(i, reference);
      if (checked.parent >= 0)
        checked.dependency.extend(dependency(checked.parent, reference));
      const Eigen::Vector2d xy = center(i);
      // In a standing start's disk, a column with no ground observed under
      // the seed floor hint, and not looked into, rests on the robot's floor.
      const auto standingGround = [&] {
        if (!standing_ || !standing_->covers(xy)) return false;
        Eigen::Vector3d deeper;
        if (projection.groundBelow({xy.x(), xy.y(), seed_ground_hint_ + 1e-6},
                                   deeper))
          return false;  // observed ground, even a drop, is used as observed
        return map_.getVoxelStatus({xy.x(), xy.y(),
                                    seed_ground_hint_ - kGroundBridgeHoleDepth}) !=
               VoxelStatus::kFree;
      };
      Eigen::Vector3d sample(xy.x(), xy.y(), reference);
      VoxelStatus status;
      const double below = projection.projectSample(sample, status);
      if (status != VoxelStatus::kOccupied && !standingGround()) {
        checked.observed = false;
        checked.dirty = false;
        if (!refused(checked.verdict)) checked.verdict = GroundVerdict::kUnknown;
        planningCheckpoint();
        columns_[i] = checked;  // complete unknown; retry only after a change
        continue;
      }
      checked.standing = status != VoxelStatus::kOccupied;
      checked.ground_z = checked.standing ? seed_ground_hint_ : sample.z() - below;
      if (!checked.standing && checked.ground_z > reference + 1e-6) {
        // projectSample starts above the floor hint. Prefer nearer observed
        // support underneath a ceiling. If the hint is slightly below the
        // floor, also look from below the hit voxel, not only from the hint.
        Eigen::Vector3d lower;
        const auto nearer_support = [&](double from_z) {
          return projection.groundBelow({xy.x(), xy.y(), from_z}, lower) &&
                 std::abs(lower.z() - reference) < checked.ground_z - reference &&
                 // A ray starting inside a thick floor can hit a buried
                 // voxel. Only a surface top can be the lower support.
                 map_.getVoxelStatus(lower + Eigen::Vector3d(0, 0, resolution_)) !=
                     VoxelStatus::kOccupied;
        };
        if (nearer_support(reference + 1e-6) ||
            nearer_support(checked.ground_z - resolution_ - 1e-6))
          checked.ground_z = lower.z();
      }
      checked.dependency.extend(dependency(i, checked.ground_z));
      // Offset projection rays may find a rim beside an unseen hole. Only
      // the column's own observed support can certify it as traversable.
      Eigen::Vector3d support;
      if (!checked.standing &&
          (!projection.groundBelow({xy.x(), xy.y(), checked.ground_z + 1e-6}, support) ||
           std::abs(support.z() - checked.ground_z) > 1e-6)) {
        checked.standing = standingGround();
        if (!checked.standing) {
          checked.observed = false;
          checked.dirty = false;
          if (!refused(checked.verdict)) checked.verdict = GroundVerdict::kUnknown;
          planningCheckpoint();
          columns_[i] = checked;
          continue;
        }
        checked.ground_z = seed_ground_hint_;
        checked.dependency.extend(dependency(i, checked.ground_z));
      }
      checked.observed = true;
      const Eigen::Vector3d ground(xy.x(), xy.y(), checked.ground_z);
      // Standing-start support is ground evidence only at its own column:
      // a root at driving height over it (groundStepsAdmissible).
      const auto standingRoot = [&](const Eigen::Vector2d& at, double z) {
        return Eigen::Vector3d(at.x(), at.y(), z + planning_.max_ground_height);
      };
      GroundVerdict verdict = GroundVerdict::kAdmitted;
      if (checked.parent >= 0) {
        const Eigen::Vector2d parent_xy = center(checked.parent);
        const double offset = std::max(0.2, 2 * resolution_) - 1e-6;
        // Anchor both endpoint probes on the actual observed supports. A
        // ceiling must not replace the parent's lower floor in this check.
        const Eigen::Vector3d start(parent_xy.x(), parent_xy.y(), reference - offset);
        const Eigen::Vector3d end(xy.x(), xy.y(), checked.ground_z - offset);
        const bool parent_standing = columns_[checked.parent].standing;
        bool admissible = true;
        if (parent_standing && !checked.standing) {
          const auto root = standingRoot(parent_xy, reference);
          admissible = projection.groundStepsAdmissible(start, end, &root);
        } else if (checked.standing && !parent_standing) {
          const auto root = standingRoot(xy, checked.ground_z);
          admissible = projection.groundStepsAdmissible(end, start, &root);
        } else if (!checked.standing) {
          admissible = projection.groundStepsAdmissible(start, end);
        }  // two standing columns share the seed floor hint
        if (!admissible) verdict = GroundVerdict::kRefusedStepGrade;
      }
      const auto own_root = standingRoot(xy, checked.ground_z);
      for (const auto& direction : kDirections) {
        // Two native cells retain the certified step-AND-grade denominator;
        // serialization and diagonal pose spacing never substitute for it.
        Eigen::Vector3d end = ground;
        end.head<2>() += 2 * resolution_ *
            Eigen::Vector2d(direction[0], direction[1]).normalized();
        if (!projection.groundStepsAdmissible(
                ground, end, checked.standing ? &own_root : nullptr)) {
          verdict = GroundVerdict::kRefusedStepGrade;
          break;
        }
      }
      if (verdict == GroundVerdict::kAdmitted) {
        const Eigen::Vector3d size = robot_params_.getPlanningSize();
        const double low = checked.ground_z + 0.5 * resolution_ + 1e-6;
        const double high = checked.ground_z + std::max(size.z(),
            planning_.max_ground_height + robot_params_.center_offset.z() + size.z() / 2);
        // The costmap carries terrain per column; Nav2 applies its own
        // footprint. A whole horizontal chassis box here would mislabel
        // an ordinary ramp under its uphill end as an overhang.
        const double column_size = resolution_ - 1e-6;
        // In the standing start's disk the lidar cannot see the body band
        // near itself: unknown volume passes in a column wholly in the
        // disk, occupied still refuses.
        const bool blind_band =
            standing_ && standing_->coversCell(xy, resolution_);
        const VoxelStatus body = map_.getBoxStatus(
            {xy.x(), xy.y(), (low + high) / 2},
            {column_size, column_size, std::max(0.0, high - low)}, !blind_band);
        if (body == VoxelStatus::kOccupied) verdict = GroundVerdict::kRefusedOverhang;
        else if (body == VoxelStatus::kUnknown) verdict = GroundVerdict::kUnknown;
      }
      planningCheckpoint();
      if (verdict != GroundVerdict::kUnknown || !refused(checked.verdict))
        checked.verdict = verdict;
      checked.dirty = false;
      columns_[i] = checked;
      neighbours(i, [&](int child) {
        // A completed flood can stop before these columns. Newly observed
        // support now makes them reachable without re-trying already
        // evaluated unknown cells on every budget cycle.
        if (!columns_[child].evaluated) markPending(child);
        enqueue(child, i);
      });
    }
    // No interrupted work remains: columns unreachable from observed
    // support are unknown, not a permanent backlog. Keep lethal refusals.
    for (auto& column : columns_) {
      planningCheckpoint();
      if (!column.dirty) continue;
      column.dirty = false;
      column.observed = false;
      column.standing = false;
      column.evaluated = false;
      if (!refused(column.verdict)) column.verdict = GroundVerdict::kUnknown;
    }
  } catch (const PlanningInterrupted&) {
    if (parent_cancelled && (*parent_cancelled)()) throw;
    // The unfinished column has never been published; its old refusal or
    // withdrawn admission remains until a complete certification fits.
  }
}

int GroundLayer::pendingCount() const {
  return std::count_if(columns_.begin(), columns_.end(),
                       [](const Column& c) { return c.dirty; });
}
bool GroundLayer::pending(const Eigen::AlignedBox3d& region) const {
  MapChange change;
  change.boxes.push_back(region);
  return std::any_of(columns_.begin(), columns_.end(), [&](const Column& c) {
    return c.dirty && changeReaches(change, c.dependency);
  });
}
GroundVerdict GroundLayer::verdict(const Eigen::Vector2d& p) const {
  const int i = index(p);
  return i < 0 ? GroundVerdict::kUnknown : columns_[i].verdict;
}
std::vector<int8_t> GroundLayer::occupancy() const {
  std::vector<int8_t> result;
  result.reserve(columns_.size());
  for (const auto& column : columns_)
    result.push_back(refused(column.verdict) ? 100 :
                     column.verdict == GroundVerdict::kAdmitted ? 0 : -1);
  return result;
}
}  // namespace mgg
