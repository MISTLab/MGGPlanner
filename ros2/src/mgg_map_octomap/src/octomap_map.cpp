#include "mgg_map_octomap/octomap_map.h"

#include <algorithm>
#include <cmath>

namespace mgg {
namespace {

inline octomap::point3d toOct(const Eigen::Vector3d& v) {
  return octomap::point3d(static_cast<float>(v.x()), static_cast<float>(v.y()),
                          static_cast<float>(v.z()));
}

inline Eigen::Vector3d toEigen(const octomap::point3d& p) {
  return Eigen::Vector3d(p.x(), p.y(), p.z());
}

}  // namespace

OctomapMap::OctomapMap(const OctomapConfig& config)
    : tree_(std::make_unique<octomap::OcTree>(config.resolution)),
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
  tree_->insertPointCloud(cloud, toOct(sensor_origin), config_.max_range);
  has_data_ = has_data_ || !points.empty();
}

double OctomapMap::getResolution() const { return tree_->getResolution(); }

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

VoxelStatus OctomapMap::getBoxStatus(const Eigen::Vector3d& center,
                                     const Eigen::Vector3d& size,
                                     bool stop_at_unknown_voxel) const {
  const double r = tree_->getResolution();
  bool saw_unknown = false;
  for (double dx = -size.x() / 2; dx <= size.x() / 2; dx += r)
    for (double dy = -size.y() / 2; dy <= size.y() / 2; dy += r)
      for (double dz = -size.z() / 2; dz <= size.z() / 2; dz += r) {
        const VoxelStatus s =
            statusAt(toOct(center + Eigen::Vector3d(dx, dy, dz)));
        // Occupied always wins: a box straddling an obstacle is in collision
        // regardless of how much unknown space it also covers.
        if (s == VoxelStatus::kOccupied) return VoxelStatus::kOccupied;
        if (s == VoxelStatus::kUnknown) saw_unknown = true;
      }
  if (saw_unknown && stop_at_unknown_voxel) return VoxelStatus::kUnknown;
  return VoxelStatus::kFree;
}

VoxelStatus OctomapMap::getPathStatus(const Eigen::Vector3d& start,
                                      const Eigen::Vector3d& end,
                                      const Eigen::Vector3d& box_size,
                                      bool stop_at_unknown_voxel) const {
  const double r = tree_->getResolution();
  const double len = (end - start).norm();
  if (len < 1e-9) return getBoxStatus(start, box_size, stop_at_unknown_voxel);
  const Eigen::Vector3d dir = (end - start) / len;

  bool saw_unknown = false;
  for (double d = 0.0; d <= len; d += r) {
    const VoxelStatus s =
        getBoxStatus(start + d * dir, box_size, stop_at_unknown_voxel);
    if (s == VoxelStatus::kOccupied) return VoxelStatus::kOccupied;
    if (s == VoxelStatus::kUnknown) saw_unknown = true;
  }
  return saw_unknown ? VoxelStatus::kUnknown : VoxelStatus::kFree;
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
    const SensorModel& sensor) {
  // The voxblox "iterative" variant skipped rays whose voxels a neighbouring
  // ray had already covered, using the sensor's angular resolution. OctoMap's
  // key-based traversal makes the equivalent saving available far more
  // cheaply: dedupe on voxel keys instead of reasoning about ray geometry.
  gain = GainCounts{};
  octomap::KeySet seen;
  for (const Eigen::Vector3d& endpoint : multiray_endpoints) {
    walkRay(pos, endpoint, [&](const octomap::point3d& centre, VoxelStatus s) {
      octomap::OcTreeKey key;
      if (tree_->coordToKeyChecked(centre, key) && !seen.insert(key).second) {
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
  (void)sensor;
}

bool OctomapMap::augmentFreeBox(const Eigen::Vector3d& position,
                                const Eigen::Vector3d& box_size) {
  const double r = tree_->getResolution();
  for (double dx = -box_size.x() / 2; dx <= box_size.x() / 2; dx += r)
    for (double dy = -box_size.y() / 2; dy <= box_size.y() / 2; dy += r)
      for (double dz = -box_size.z() / 2; dz <= box_size.z() / 2; dz += r) {
        const Eigen::Vector3d p = position + Eigen::Vector3d(dx, dy, dz);
        // setNodeValue, not updateNode: this must assert free rather than
        // accumulate evidence, or clearing the robot's own footprint would
        // take several calls to take effect.
        tree_->setNodeValue(toOct(p), tree_->getClampingThresMinLog());
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
