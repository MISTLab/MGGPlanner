#include "mgg_map_octomap/octomap_map.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace mgg {
namespace {

inline octomap::point3d toOct(const Eigen::Vector3d& v) {
  return octomap::point3d(static_cast<float>(v.x()), static_cast<float>(v.y()),
                          static_cast<float>(v.z()));
}

inline Eigen::Vector3d toEigen(const octomap::point3d& p) {
  return Eigen::Vector3d(p.x(), p.y(), p.z());
}

double pointSegmentDistanceSquared(const Eigen::Vector2d& point,
                                   const Eigen::Vector2d& a,
                                   const Eigen::Vector2d& b) {
  const Eigen::Vector2d segment = b - a;
  const double length_squared = segment.squaredNorm();
  if (length_squared <= 1e-24) return (point - a).squaredNorm();
  const double t = std::clamp((point - a).dot(segment) / length_squared,
                              0.0, 1.0);
  return (point - (a + t * segment)).squaredNorm();
}

double pointBoxDistanceSquared(const Eigen::Vector2d& point,
                               const Eigen::Vector2d& lower,
                               const Eigen::Vector2d& upper) {
  const Eigen::Vector2d outside =
      (lower - point).cwiseMax(Eigen::Vector2d::Zero()) +
      (point - upper).cwiseMax(Eigen::Vector2d::Zero());
  return outside.squaredNorm();
}

bool segmentIntersectsBox(const Eigen::Vector2d& a,
                          const Eigen::Vector2d& b,
                          const Eigen::Vector2d& lower,
                          const Eigen::Vector2d& upper) {
  double first = 0.0;
  double last = 1.0;
  const Eigen::Vector2d direction = b - a;
  for (int axis = 0; axis < 2; ++axis) {
    if (std::abs(direction[axis]) <= 1e-15) {
      if (a[axis] < lower[axis] || a[axis] > upper[axis]) return false;
      continue;
    }
    double enter = (lower[axis] - a[axis]) / direction[axis];
    double leave = (upper[axis] - a[axis]) / direction[axis];
    if (enter > leave) std::swap(enter, leave);
    first = std::max(first, enter);
    last = std::min(last, leave);
    if (first > last) return false;
  }
  return true;
}

double segmentBoxDistanceSquared(const Eigen::Vector2d& a,
                                 const Eigen::Vector2d& b,
                                 const Eigen::Vector2d& center,
                                 double half_cell) {
  const Eigen::Vector2d lower =
      center - Eigen::Vector2d::Constant(half_cell);
  const Eigen::Vector2d upper =
      center + Eigen::Vector2d::Constant(half_cell);
  if (segmentIntersectsBox(a, b, lower, upper)) return 0.0;
  double distance = std::min(pointBoxDistanceSquared(a, lower, upper),
                             pointBoxDistanceSquared(b, lower, upper));
  for (double x : {lower.x(), upper.x()}) {
    for (double y : {lower.y(), upper.y()}) {
      distance = std::min(
          distance,
          pointSegmentDistanceSquared(Eigen::Vector2d(x, y), a, b));
    }
  }
  return distance;
}

}  // namespace

OctomapMap::OctomapMap(const OctomapConfig& config)
    : tree_(std::make_unique<UpdateAwareOcTree>(config.resolution)),
      config_(config) {
  tree_->setProbHit(config_.probability_hit);
  tree_->setProbMiss(config_.probability_miss);
  tree_->setClampingThresMin(config_.clamping_min);
  tree_->setClampingThresMax(config_.clamping_max);
  tree_->setOccupancyThres(config_.occupancy_threshold);
}

void OctomapMap::insertPointCloud(const std::vector<Eigen::Vector3d>& points,
                                  const Eigen::Vector3d& sensor_origin) {
  octomap::Pointcloud cloud;
  cloud.reserve(points.size());
  for (const auto& p : points) cloud.push_back(toOct(p));
  if (!track_measured_surface_z_) {
    tree_->insertPointCloud(cloud, toOct(sensor_origin), config_.max_range);
    has_data_ = has_data_ || !points.empty();
    return;
  }

  // This is OctoMap's non-discretized insertPointCloud update sequence, kept
  // in an opt-in branch so surface bookkeeping can use the exact same free
  // and occupied key sets without computing every sensor ray twice.
  octomap::KeySet free_cells;
  octomap::KeySet occupied_cells;
  tree_->computeUpdate(cloud, toOct(sensor_origin), free_cells, occupied_cells,
                       config_.max_range);
  for (const octomap::OcTreeKey& key : free_cells) {
    tree_->updateNode(key, false);
    const octomap::OcTreeNode* node = tree_->search(key);
    if (node == nullptr || !tree_->isNodeOccupied(node)) {
      measured_surface_max_z_.erase({key[0], key[1], key[2]});
    }
  }
  for (const octomap::OcTreeKey& key : occupied_cells) {
    const octomap::OcTreeNode* before = tree_->search(key);
    const bool was_occupied =
        before != nullptr && tree_->isNodeOccupied(before);
    tree_->updateNode(key, true);
    if (!was_occupied) {
      measured_surface_max_z_.erase({key[0], key[1], key[2]});
    }
  }
  for (const Eigen::Vector3d& point : points) {
    if (!point.allFinite() || !sensor_origin.allFinite()) continue;
    // Match computeUpdate's float-coordinate range decision exactly at the
    // configured boundary.
    const octomap::point3d oct_point = toOct(point);
    const double range = (oct_point - toOct(sensor_origin)).norm();
    if (!std::isfinite(range) ||
        (config_.max_range >= 0.0 && range > config_.max_range)) {
      continue;
    }
    octomap::OcTreeKey key;
    if (!tree_->coordToKeyChecked(oct_point, key) ||
        occupied_cells.find(key) == occupied_cells.end()) {
      continue;
    }
    const octomap::OcTreeNode* node = tree_->search(key);
    if (node == nullptr || !tree_->isNodeOccupied(node)) continue;
    const SurfaceKey surface_key{key[0], key[1], key[2]};
    const auto found = measured_surface_max_z_.find(surface_key);
    if (found == measured_surface_max_z_.end()) {
      measured_surface_max_z_.emplace(surface_key, point.z());
    } else {
      found->second = std::max(found->second, point.z());
    }
  }
  has_data_ = has_data_ || !points.empty();
}

void OctomapMap::setTrackMeasuredSurfaceZ(bool enabled) {
  track_measured_surface_z_ = enabled;
  if (!enabled) measured_surface_max_z_.clear();
}

bool OctomapMap::setMeasuredSurfaceZ(
    const Eigen::Vector3d& occupied_position, const double surface_z) {
  if (!occupied_position.allFinite() || !std::isfinite(surface_z)) return false;
  octomap::OcTreeKey key;
  if (!tree_->coordToKeyChecked(toOct(occupied_position), key)) return false;
  const octomap::OcTreeNode* node = tree_->search(key);
  if (node == nullptr || !tree_->isNodeOccupied(node)) return false;
  track_measured_surface_z_ = true;
  const SurfaceKey surface_key{key[0], key[1], key[2]};
  const auto found = measured_surface_max_z_.find(surface_key);
  if (found == measured_surface_max_z_.end()) {
    measured_surface_max_z_.emplace(surface_key, surface_z);
  } else {
    found->second = std::max(found->second, surface_z);
  }
  return true;
}

double OctomapMap::getResolution() const { return tree_->getResolution(); }

bool OctomapMap::getAxisAlignedXYCellCenter(
    const Eigen::Vector2d& position, Eigen::Vector2d& center) const {
  if (!position.allFinite()) return false;
  const double extent = tree_->getResolution() * 32768.0;
  if (!std::isfinite(extent) || extent <= 0.0 ||
      (position.array() < -extent).any() ||
      (position.array() >= extent).any()) {
    return false;
  }
  octomap::OcTreeKey key;
  if (!tree_->coordToKeyChecked(
          octomap::point3d(float(position.x()), float(position.y()), 0.0f),
          key)) {
    return false;
  }
  center = Eigen::Vector2d(tree_->keyToCoord(key[0]),
                           tree_->keyToCoord(key[1]));
  return center.allFinite();
}

bool OctomapMap::getStatus() const { return has_data_; }

VoxelStatus OctomapMap::statusAt(const octomap::point3d& p) const {
  const octomap::OcTreeNode* node = tree_->search(p);
  if (node == nullptr) return VoxelStatus::kUnknown;
  if (tree_->isNodeOccupied(node)) return VoxelStatus::kOccupied;

  // Optional obstacle inflation, to approximate voxblox's truncation band.
  // Off by default; costs nothing when occupied_dilation_voxels == 0.
  if (config_.occupied_dilation_voxels > 0) {
    const double r = tree_->getResolution();
    const int n = config_.occupied_dilation_voxels;
    for (int dx = -n; dx <= n; ++dx)
      for (int dy = -n; dy <= n; ++dy)
        for (int dz = -n; dz <= n; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) continue;
          const octomap::OcTreeNode* nb = tree_->search(
              p.x() + dx * r, p.y() + dy * r, p.z() + dz * r);
          if (nb != nullptr && tree_->isNodeOccupied(nb)) {
            return VoxelStatus::kOccupied;
          }
        }
  }
  return VoxelStatus::kFree;
}

VoxelStatus OctomapMap::getVoxelStatus(const Eigen::Vector3d& position) const {
  return statusAt(toOct(position));
}

template <typename Visitor>
void OctomapMap::walkRay(const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                         Visitor visit) const {
  octomap::KeyRay ray;
  // computeRayKeys gives the exact voxel traversal OctoMap itself uses, so
  // queries and integration agree about which voxels a ray touches.
  if (!tree_->computeRayKeys(toOct(from), toOct(to), ray)) return;
  for (const octomap::OcTreeKey& key : ray) {
    const octomap::point3d centre = tree_->keyToCoord(key);
    if (!visit(centre, statusAt(centre))) return;
  }
}

VoxelStatus OctomapMap::getRayStatus(const Eigen::Vector3d& view_point,
                                     const Eigen::Vector3d& voxel_to_test,
                                     bool stop_at_unknown_voxel) const {
  Eigen::Vector3d ignored;
  return getRayStatus(view_point, voxel_to_test, stop_at_unknown_voxel,
                      ignored);
}

VoxelStatus OctomapMap::getRayStatus(const Eigen::Vector3d& view_point,
                                     const Eigen::Vector3d& voxel_to_test,
                                     bool stop_at_unknown_voxel,
                                     Eigen::Vector3d& end_voxel) const {
  VoxelStatus result = VoxelStatus::kFree;
  end_voxel = voxel_to_test;
  walkRay(view_point, voxel_to_test,
          [&](const octomap::point3d& centre, VoxelStatus s) {
            if (s == VoxelStatus::kOccupied) {
              result = s;
              end_voxel = toEigen(centre);
              return false;
            }
            if (s == VoxelStatus::kUnknown && stop_at_unknown_voxel) {
              result = s;
              end_voxel = toEigen(centre);
              return false;
            }
            return true;
          });
  return result;
}

VoxelStatus OctomapMap::getGroundRayStatus(
    const Eigen::Vector3d& view_point,
    const Eigen::Vector3d& voxel_to_test, bool stop_at_unknown_voxel,
    Eigen::Vector3d& end_voxel) const {
  const VoxelStatus status = getRayStatus(
      view_point, voxel_to_test, stop_at_unknown_voxel, end_voxel);
  if (status != VoxelStatus::kOccupied || !track_measured_surface_z_) {
    return status;
  }
  octomap::OcTreeKey key;
  if (!tree_->coordToKeyChecked(toOct(end_voxel), key)) return status;
  const auto found =
      measured_surface_max_z_.find({key[0], key[1], key[2]});
  if (found != measured_surface_max_z_.end() &&
      std::isfinite(found->second)) {
    end_voxel.z() = found->second;
  }
  return status;
}

VoxelStatus OctomapMap::queryBox(const Eigen::Vector3d& center,
                                  const Eigen::Vector3d& size,
                                  double unknown_fraction) const {
  // Bound a single map callback even for invalid/oversized robot geometry.
  constexpr std::uint64_t kMaxBoxCells = 1u << 20;
  if (!center.allFinite() || !size.allFinite() || (size.array() < 0).any()) {
    return VoxelStatus::kUnknown;
  }
  const Eigen::Vector3d lower = center - size / 2.0;
  const Eigen::Vector3d upper = center + size / 2.0;
  const double extent = tree_->getResolution() * 32768.0;
  if (!std::isfinite(extent) || extent <= 0.0 || !lower.allFinite() ||
      !upper.allFinite() ||
      (lower.array() < -extent).any() || (upper.array() >= extent).any()) {
    return VoxelStatus::kUnknown;
  }
  octomap::OcTreeKey first, last;
  if (!tree_->coordToKeyChecked(toOct(lower), first) ||
      !tree_->coordToKeyChecked(toOct(upper), last)) {
    return VoxelStatus::kUnknown;
  }
  std::uint64_t total = 1;
  for (int axis = 0; axis < 3; ++axis) {
    const std::uint64_t count = std::uint64_t(last[axis]) - first[axis] + 1;
    if (count > kMaxBoxCells / total) return VoxelStatus::kUnknown;
    total *= count;
  }
  std::uint64_t unknown = 0;
  // Floating coordinate stepping can skip a positive-face key. Iterate every
  // touched key exactly once, as augmentFreeBox does, including both faces.
  for (std::uint32_t x = first[0]; x <= last[0]; ++x)
    for (std::uint32_t y = first[1]; y <= last[1]; ++y)
      for (std::uint32_t z = first[2]; z <= last[2]; ++z) {
        const octomap::OcTreeKey key(static_cast<octomap::key_type>(x),
                                    static_cast<octomap::key_type>(y),
                                    static_cast<octomap::key_type>(z));
        const VoxelStatus status = statusAt(tree_->keyToCoord(key));
        if (status == VoxelStatus::kOccupied) return status;
        if (status == VoxelStatus::kUnknown) {
          if (unknown_fraction == 0.0) return status;
          ++unknown;
        }
      }
  return double(unknown) > unknown_fraction * double(total)
             ? VoxelStatus::kUnknown : VoxelStatus::kFree;
}

VoxelStatus OctomapMap::getBoxStatus(const Eigen::Vector3d& center,
                                     const Eigen::Vector3d& size,
                                     bool stop_at_unknown_voxel) const {
  // Preserve the existing exploration policy while fixing voxel enumeration.
  return queryBox(center, size, stop_at_unknown_voxel ? 0.25 : 1.0);
}

VoxelStatus OctomapMap::getStrictBoxStatus(const Eigen::Vector3d& center,
                                           const Eigen::Vector3d& size) const {
  return queryBox(center, size, 0.0);
}

VoxelStatus OctomapMap::getPathStatus(const Eigen::Vector3d& start,
                                      const Eigen::Vector3d& end,
                                      const Eigen::Vector3d& box_size,
                                      bool stop_at_unknown_voxel) const {
  const double r = tree_->getResolution();
  const double len = (end - start).norm();
  if (len < 1e-9) return getBoxStatus(start, box_size, stop_at_unknown_voxel);
  // Point samples a resolution apart cut corners a zero-size segment crosses.
  if (box_size.allFinite() && (box_size.array() == 0.0).all()) {
    return centreLinePathStatus(start, end, stop_at_unknown_voxel);
  }
  const int steps = std::max(1, static_cast<int>(std::ceil(len / r)));

  bool saw_unknown = false;
  // Sample the same endpoint-inclusive boxes in either direction.  Advancing
  // by `r` omitted the final endpoint whenever len was not an exact multiple
  // and made an undirected graph edge pass outward but fail on the Home route.
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const VoxelStatus s =
        getBoxStatus(start + t * (end - start), box_size,
                     stop_at_unknown_voxel);
    if (s == VoxelStatus::kOccupied) return VoxelStatus::kOccupied;
    if (s == VoxelStatus::kUnknown) saw_unknown = true;
  }
  return saw_unknown ? VoxelStatus::kUnknown : VoxelStatus::kFree;
}

VoxelStatus OctomapMap::centreLinePathStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const bool stop_at_unknown_voxel) const {
  if (!start.allFinite() || !end.allFinite()) return VoxelStatus::kUnknown;
  const double resolution = tree_->getResolution();
  std::array<octomap::key_type, 3> current{};
  std::array<octomap::key_type, 3> target{};
  for (int axis = 0; axis < 3; ++axis) {
    if (!tree_->coordToKeyChecked(start[axis], current[axis]) ||
        !tree_->coordToKeyChecked(end[axis], target[axis])) {
      return VoxelStatus::kUnknown;
    }
  }
  // Bounded work in a non-preemptible callback: at most seven voxels meet
  // the segment at each crossing.
  constexpr std::uint64_t kMaxCrossings = 1u << 20;
  std::uint64_t crossings = 0;
  for (int axis = 0; axis < 3; ++axis) {
    crossings += static_cast<std::uint64_t>(
        std::abs(int(target[axis]) - int(current[axis])));
  }
  if (crossings > kMaxCrossings) return VoxelStatus::kUnknown;

  bool saw_unknown = false;
  // True when the voxel is occupied, which ends the walk.
  const auto occupied = [&](const std::array<int, 3>& key) {
    for (int axis = 0; axis < 3; ++axis) {
      if (key[axis] < 0 || key[axis] > 0xFFFF) {
        saw_unknown = true;
        return false;
      }
    }
    const octomap::OcTreeKey voxel(static_cast<octomap::key_type>(key[0]),
                                   static_cast<octomap::key_type>(key[1]),
                                   static_cast<octomap::key_type>(key[2]));
    const VoxelStatus status = statusAt(tree_->keyToCoord(voxel));
    if (status == VoxelStatus::kUnknown) saw_unknown = true;
    return status == VoxelStatus::kOccupied;
  };

  std::array<int, 3> key{current[0], current[1], current[2]};
  if (occupied(key)) return VoxelStatus::kOccupied;
  const Eigen::Vector3d direction = end - start;
  std::array<int, 3> step{};
  Eigen::Vector3d next =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d delta = next;
  for (int axis = 0; axis < 3; ++axis) {
    if (direction[axis] == 0.0) continue;
    step[axis] = direction[axis] > 0.0 ? 1 : -1;
    const double boundary = tree_->keyToCoord(current[axis]) +
                            0.5 * resolution * double(step[axis]);
    next[axis] = (boundary - start[axis]) / direction[axis];
    delta[axis] = resolution / std::abs(direction[axis]);
  }
  // Each pass crosses at least one boundary; the target voxel is checked
  // after the loop whether or not rounding lets the walk reach it.
  for (std::uint64_t pass = 0;
       pass <= crossings &&
       (key[0] != target[0] || key[1] != target[1] || key[2] != target[2]);
       ++pass) {
    const double crossing = next.minCoeff();
    if (!(crossing <= 1.0)) break;
    const double tolerance = 16.0 * std::numeric_limits<double>::epsilon() *
                             std::max(1.0, std::abs(crossing));
    unsigned mask = 0;
    for (int axis = 0; axis < 3; ++axis) {
      if (std::abs(next[axis] - crossing) <= tolerance) mask |= 1u << axis;
    }
    // Through an edge or a corner: every voxel meeting there.
    for (unsigned subset = mask; subset != 0; subset = (subset - 1) & mask) {
      std::array<int, 3> touched = key;
      for (int axis = 0; axis < 3; ++axis) {
        if (subset & (1u << axis)) touched[axis] += step[axis];
      }
      if (occupied(touched)) return VoxelStatus::kOccupied;
    }
    for (int axis = 0; axis < 3; ++axis) {
      if (mask & (1u << axis)) {
        key[axis] += step[axis];
        next[axis] += delta[axis];
      }
    }
  }
  if (occupied({target[0], target[1], target[2]})) {
    return VoxelStatus::kOccupied;
  }
  return stop_at_unknown_voxel && saw_unknown ? VoxelStatus::kUnknown
                                              : VoxelStatus::kFree;
}

VoxelStatus OctomapMap::getOccupiedOnlyCylinderPathStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const double radius, const double height) const {
  if (!start.allFinite() || !end.allFinite() || !std::isfinite(radius) ||
      radius < 0.0 || !std::isfinite(height) || height < 0.0) {
    return VoxelStatus::kUnknown;
  }
  const double resolution = tree_->getResolution();
  const double length = (end - start).norm();
  if (!std::isfinite(resolution) || resolution <= 0.0 ||
      !std::isfinite(length)) {
    return VoxelStatus::kUnknown;
  }
  constexpr std::uint64_t kMaxWork = 1u << 22;
  const double steps_d = std::max(1.0, std::ceil(length / resolution));
  if (!std::isfinite(steps_d) || steps_d > double(kMaxWork)) {
    return VoxelStatus::kUnknown;
  }
  const auto steps = static_cast<std::uint64_t>(steps_d);
  const Eigen::Vector3d step = (end - start) / double(steps);
  const double extent = resolution * 32768.0;
  const double half_cell = 0.5 * resolution;
  std::uint64_t work = 0;
  for (std::uint64_t i = 0; i < steps; ++i) {
    const Eigen::Vector3d a = start + double(i) * step;
    const Eigen::Vector3d b = a + step;
    const double coordinate_scale = std::max(
        {1.0, resolution, radius, height, a.cwiseAbs().maxCoeff(),
         b.cwiseAbs().maxCoeff()});
    // OctoMap stores query coordinates in float point3d values even though
    // its resolution and key conversion API use double. Treat closed voxel
    // faces within that representation error as touching, rather than losing
    // negative/radial/Z tangencies after float conversion.
    const double face_epsilon =
        16.0 * double(std::numeric_limits<float>::epsilon()) *
        coordinate_scale;
    const Eigen::Vector3d lower(
        std::min(a.x(), b.x()) - radius,
        std::min(a.y(), b.y()) - radius,
        std::min(a.z(), b.z()) - 0.5 * height);
    const Eigen::Vector3d upper(
        std::max(a.x(), b.x()) + radius,
        std::max(a.y(), b.y()) + radius,
        std::max(a.z(), b.z()) + 0.5 * height);
    const Eigen::Vector3d padded_lower =
        lower - Eigen::Vector3d::Constant(resolution);
    const Eigen::Vector3d padded_upper =
        upper + Eigen::Vector3d::Constant(resolution);
    if (!std::isfinite(extent) || extent <= 0.0 || !lower.allFinite() ||
        !upper.allFinite() || !padded_lower.allFinite() ||
        !padded_upper.allFinite() ||
        (padded_lower.array() < -extent).any() ||
        (padded_upper.array() >= extent).any()) {
      return VoxelStatus::kUnknown;
    }
    octomap::OcTreeKey first, last;
    if (!tree_->coordToKeyChecked(toOct(padded_lower), first) ||
        !tree_->coordToKeyChecked(toOct(padded_upper), last)) {
      return VoxelStatus::kUnknown;
    }
    std::uint64_t sample_work = 1;
    for (int axis = 0; axis < 3; ++axis) {
      const std::uint64_t count =
          std::uint64_t(last[axis]) - first[axis] + 1;
      if (count > (kMaxWork - work) / sample_work) {
        return VoxelStatus::kUnknown;
      }
      sample_work *= count;
    }
    work += sample_work;
    const Eigen::Vector2d a_xy = a.head<2>();
    const Eigen::Vector2d b_xy = b.head<2>();
    const double radius_squared =
        (radius + face_epsilon) * (radius + face_epsilon);
    for (std::uint32_t x = first[0]; x <= last[0]; ++x) {
      for (std::uint32_t y = first[1]; y <= last[1]; ++y) {
        const double cell_x =
            tree_->keyToCoord(static_cast<octomap::key_type>(x));
        const double cell_y =
            tree_->keyToCoord(static_cast<octomap::key_type>(y));
        if (segmentBoxDistanceSquared(
                a_xy, b_xy, Eigen::Vector2d(cell_x, cell_y),
                half_cell) > radius_squared) {
          continue;
        }
        for (std::uint32_t z = first[2]; z <= last[2]; ++z) {
          const octomap::OcTreeKey key(
              static_cast<octomap::key_type>(x),
              static_cast<octomap::key_type>(y),
              static_cast<octomap::key_type>(z));
          const double cell_z =
              tree_->keyToCoord(static_cast<octomap::key_type>(z));
          if (cell_z + half_cell < lower.z() - face_epsilon ||
              cell_z - half_cell > upper.z() + face_epsilon) {
            continue;
          }
          const octomap::point3d cell_center = tree_->keyToCoord(key);
          if (statusAt(cell_center) == VoxelStatus::kOccupied) {
            return VoxelStatus::kOccupied;
          }
        }
      }
    }
  }
  return VoxelStatus::kFree;
}

VoxelStatus OctomapMap::getStrictPathStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size) const {
  if (!start.allFinite() || !end.allFinite() || !box_size.allFinite() ||
      (box_size.array() < 0).any()) return VoxelStatus::kUnknown;
  const double resolution = tree_->getResolution();
  const double length = (end - start).norm();
  if (!std::isfinite(resolution) || resolution <= 0.0 ||
      !std::isfinite(length)) {
    return VoxelStatus::kUnknown;
  }
  if (length < 1e-9) return getStrictBoxStatus(start, box_size);
  // Conservative upper bound on total voxel visits in this non-preemptible
  // callback. The grid's cooperative deadline is checked around it.
  constexpr std::uint64_t kMaxSweepCells = 1u << 22;
  const double steps_d = std::max(1.0, std::ceil(length / resolution));
  if (!std::isfinite(steps_d) || steps_d > double(kMaxSweepCells)) {
    return VoxelStatus::kUnknown;
  }
  const std::uint64_t steps = static_cast<std::uint64_t>(steps_d);
  const Eigen::Vector3d step = (end - start) / double(steps);
  // Each query is the axis-aligned envelope of the continuously swept body
  // over one <=resolution segment. It is conservative on diagonals and also
  // covers every crossed voxel when box_size is zero.
  const Eigen::Vector3d swept_size = box_size + step.cwiseAbs();
  std::uint64_t box_cells = 1;
  for (int axis = 0; axis < 3; ++axis) {
    const double count = std::ceil(swept_size[axis] / resolution) + 2.0;
    if (!std::isfinite(count) || count > double(kMaxSweepCells / box_cells)) {
      return VoxelStatus::kUnknown;
    }
    box_cells *= static_cast<std::uint64_t>(count);
  }
  if (steps > kMaxSweepCells / box_cells) {
    return VoxelStatus::kUnknown;
  }
  for (std::uint64_t i = 0; i < steps; ++i) {
    const double fraction = (double(i) + 0.5) / double(steps);
    const VoxelStatus status =
        getStrictBoxStatus(start + fraction * (end - start), swept_size);
    if (status != VoxelStatus::kFree) return status;
  }
  return VoxelStatus::kFree;
}

void OctomapMap::getScanStatus(
    const Eigen::Vector3d& pos,
    const std::vector<Eigen::Vector3d>& multiray_endpoints, GainCounts& gain,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
    const SensorModel& /*sensor*/) {
  gain = GainCounts{};
  for (const Eigen::Vector3d& endpoint : multiray_endpoints) {
    walkRay(pos, endpoint, [&](const octomap::point3d& centre, VoxelStatus s) {
      voxel_log.emplace_back(toEigen(centre), s);
      switch (s) {
        case VoxelStatus::kOccupied:
          ++gain.occupied;
          return false;  // the ray stops at the first surface
        case VoxelStatus::kUnknown:
          ++gain.unknown;
          return true;
        case VoxelStatus::kFree:
          ++gain.free;
          return true;
      }
      return true;
    });
  }
}

void OctomapMap::getScanStatusIterative(
    const Eigen::Vector3d& pos,
    const std::vector<Eigen::Vector3d>& multiray_endpoints, GainCounts& gain,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
    const SensorModel& /*sensor*/) {
  scanUnique(pos, multiray_endpoints, nullptr, gain, voxel_log);
}

void OctomapMap::getVisibleScanStatus(
    const Eigen::Vector3d& pos,
    const std::vector<Eigen::Vector3d>& multiray_endpoints,
    const WallBand& wall, GainCounts& gain,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
    const SensorModel& /*sensor*/) {
  scanUnique(pos, multiray_endpoints, &wall, gain, voxel_log);
}

void OctomapMap::scanUnique(
    const Eigen::Vector3d& pos,
    const std::vector<Eigen::Vector3d>& multiray_endpoints,
    const WallBand* wall, GainCounts& gain,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log) const {
  // The voxblox "iterative" variant skipped rays whose voxels a neighbouring
  // ray had already covered, using the sensor's angular resolution. OctoMap's
  // key-based traversal makes the equivalent saving available far more
  // cheaply: dedupe on voxel keys instead of reasoning about ray geometry.
  gain = GainCounts{};
  // The wall band as key rows; see NativeMolaGrid::scanUnique.
  constexpr int kMaxWallBandCells = 64;
  const double resolution = tree_->getResolution();
  octomap::key_type band_low = 0, band_high = 0;
  const bool walls =
      wall != nullptr && wall->min_z <= wall->max_z &&
      tree_->coordToKeyChecked(wall->min_z - 0.5 * resolution, band_low) &&
      tree_->coordToKeyChecked(wall->max_z - 0.5 * resolution, band_high) &&
      int(band_high) - int(band_low) < kMaxWallBandCells;
  int first_row = band_low, last_row = band_high;
  if (walls && tree_->keyToCoord(band_low) < wall->min_z) ++first_row;
  // Occupied rows of the band, per column, in ascending order.
  std::unordered_map<std::uint32_t, std::vector<int>> wall_rows;
  const auto isWallGap = [&](const octomap::OcTreeKey& key) {
    const std::uint32_t column = (std::uint32_t(key[0]) << 16) | key[1];
    auto rows = wall_rows.find(column);
    if (rows == wall_rows.end()) {
      std::vector<int> occupied;
      for (int z = first_row; z <= last_row; ++z) {
        const octomap::OcTreeKey cell(key[0], key[1],
                                      static_cast<octomap::key_type>(z));
        if (statusAt(tree_->keyToCoord(cell)) == VoxelStatus::kOccupied)
          occupied.push_back(z);
      }
      rows = wall_rows.emplace(column, std::move(occupied)).first;
    }
    const auto& occupied = rows->second;
    const auto above =
        std::upper_bound(occupied.begin(), occupied.end(), int(key[2]));
    if (above == occupied.begin() || above == occupied.end()) return false;
    return *above - *(above - 1) - 1 <= kMaxWallGapVoxels;
  };
  octomap::KeySet seen;
  for (const Eigen::Vector3d& endpoint : multiray_endpoints) {
    walkRay(pos, endpoint, [&](const octomap::point3d& centre, VoxelStatus s) {
      octomap::OcTreeKey key;
      const bool keyed = tree_->coordToKeyChecked(centre, key);
      // A gap in a wall hides what is behind it, for every ray.
      if (walls && keyed && s == VoxelStatus::kUnknown && isWallGap(key)) {
        return false;
      }
      if (keyed && !seen.insert(key).second) {
        // Already counted through another ray; keep walking but do not
        // double-count.
        return s != VoxelStatus::kOccupied;
      }
      voxel_log.emplace_back(toEigen(centre), s);
      switch (s) {
        case VoxelStatus::kOccupied: ++gain.occupied; return false;
        case VoxelStatus::kUnknown: ++gain.unknown; return true;
        case VoxelStatus::kFree: ++gain.free; return true;
      }
      return true;
    });
  }
}

bool OctomapMap::augmentFreeBox(const Eigen::Vector3d& position,
                                const Eigen::Vector3d& box_size) {
  octomap::OcTreeKey min_key;
  octomap::OcTreeKey max_key;
  const Eigen::Vector3d half = box_size / 2.0;
  if (!tree_->coordToKeyChecked(toOct(position - half), min_key) ||
      !tree_->coordToKeyChecked(toOct(position + half), max_key)) {
    return false;
  }
  // Iterate discrete keys rather than adding resolution-sized doubles. At
  // coordinate boundaries, accumulated floating-point error could skip one
  // key and leave a thin unknown slice through an otherwise cleared robot
  // footprint.
  for (std::uint32_t x = min_key[0]; x <= max_key[0]; ++x)
    for (std::uint32_t y = min_key[1]; y <= max_key[1]; ++y)
      for (std::uint32_t z = min_key[2]; z <= max_key[2]; ++z) {
        const octomap::OcTreeKey key(
            static_cast<octomap::key_type>(x),
            static_cast<octomap::key_type>(y),
            static_cast<octomap::key_type>(z));
        // setNodeValue, not updateNode: this must assert free rather than
        // accumulate evidence, or clearing the robot's own footprint would
        // take several calls to take effect.
        tree_->setNodeValue(key, tree_->getClampingThresMinLog());
        measured_surface_max_z_.erase({key[0], key[1], key[2]});
      }
  has_data_ = true;
  return true;
}

void OctomapMap::augmentFreeFrustum() {
  // The voxblox implementation was a no-op too: the parameters that would
  // have driven it are commented out as "NOT IMPLEMENTED FOR VOXBLOX" in the
  // shipped configs.
}

void OctomapMap::resetMap() {
  tree_->clear();
  measured_surface_max_z_.clear();
  has_data_ = false;
}

void OctomapMap::extractLocalMap(const Eigen::Vector3d& center,
                                 const Eigen::Vector3d& bounding_box_size,
                                 std::vector<Eigen::Vector3d>& occupied_voxels,
                                 std::vector<Eigen::Vector3d>& free_voxels) {
  occupied_voxels.clear();
  free_voxels.clear();
  const octomap::point3d lo = toOct(center - bounding_box_size / 2.0);
  const octomap::point3d hi = toOct(center + bounding_box_size / 2.0);
  for (auto it = tree_->begin_leafs_bbx(lo, hi), end = tree_->end_leafs_bbx();
       it != end; ++it) {
    const Eigen::Vector3d c(it.getX(), it.getY(), it.getZ());
    if (tree_->isNodeOccupied(*it)) {
      occupied_voxels.push_back(c);
    } else {
      free_voxels.push_back(c);
    }
  }
}

void OctomapMap::extractLocalMapAlongAxis(
    const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
    const Eigen::Vector3d& bounding_box_size,
    std::vector<Eigen::Vector3d>& occupied_voxels,
    std::vector<Eigen::Vector3d>& free_voxels) {
  // Extract the enclosing axis-aligned box, then reject anything outside the
  // rotated box. Cheaper than walking the tree in the rotated frame.
  occupied_voxels.clear();
  free_voxels.clear();
  const Eigen::Vector3d a = axis.norm() > 1e-9 ? axis.normalized()
                                               : Eigen::Vector3d::UnitX();
  Eigen::Quaterniond q =
      Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), a);
  const Eigen::Matrix3d r_wb = q.toRotationMatrix();
  const double radius = bounding_box_size.norm() / 2.0;

  const octomap::point3d lo =
      toOct(center - Eigen::Vector3d::Constant(radius));
  const octomap::point3d hi =
      toOct(center + Eigen::Vector3d::Constant(radius));
  for (auto it = tree_->begin_leafs_bbx(lo, hi), end = tree_->end_leafs_bbx();
       it != end; ++it) {
    const Eigen::Vector3d c(it.getX(), it.getY(), it.getZ());
    const Eigen::Vector3d local = r_wb.transpose() * (c - center);
    if ((local.array().abs() > (bounding_box_size.array() / 2.0)).any()) {
      continue;
    }
    if (tree_->isNodeOccupied(*it)) {
      occupied_voxels.push_back(c);
    } else {
      free_voxels.push_back(c);
    }
  }
}

void OctomapMap::getLocalPointcloud(const Eigen::Vector3d& center, double range,
                                    double /*yaw*/,
                                    std::vector<Eigen::Vector3d>& points,
                                    bool include_unknown_voxels) {
  points.clear();
  const octomap::point3d lo = toOct(center - Eigen::Vector3d::Constant(range));
  const octomap::point3d hi = toOct(center + Eigen::Vector3d::Constant(range));
  for (auto it = tree_->begin_leafs_bbx(lo, hi), end = tree_->end_leafs_bbx();
       it != end; ++it) {
    const Eigen::Vector3d c(it.getX(), it.getY(), it.getZ());
    if ((c - center).norm() > range) continue;
    if (tree_->isNodeOccupied(*it) || include_unknown_voxels) {
      points.push_back(c);
    }
  }
}

void OctomapMap::getFreeSpacePointCloud(
    const std::vector<Eigen::Vector3d>& multiray_endpoints,
    const StateVec& state, std::vector<Eigen::Vector3d>& points) {
  points.clear();
  const Eigen::Vector3d origin(state[0], state[1], state[2]);
  for (const Eigen::Vector3d& endpoint : multiray_endpoints) {
    walkRay(origin, endpoint, [&](const octomap::point3d& centre,
                                  VoxelStatus s) {
      if (s == VoxelStatus::kOccupied) return false;
      if (s == VoxelStatus::kFree) points.push_back(toEigen(centre));
      return true;
    });
  }
}

void OctomapMap::setRaycastingParams(bool nonuniform_ray_cast,
                                     double ray_cast_step_size_multiplier) {
  nonuniform_ray_cast_ = nonuniform_ray_cast;
  ray_cast_step_size_multiplier_ = ray_cast_step_size_multiplier;
}

void OctomapMap::setRobotRadius(double robot_radius) {
  robot_radius_ = robot_radius;
}

}  // namespace mgg
