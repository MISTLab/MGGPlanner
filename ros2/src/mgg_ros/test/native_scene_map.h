// Mutable test scenes queried through the production native persistent grid.
// Scans and explicit free boxes only build the fixture; every geometry query
// delegates to NativeMolaGrid. No production map API or backend is added.
#ifndef MGG_ROS_TEST_NATIVE_SCENE_MAP_H_
#define MGG_ROS_TEST_NATIVE_SCENE_MAP_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "mgg_core/voxel_walk.h"
#include "mgg_map_octomap/native_mola_grid.h"

namespace mgg_test {
struct NativeSceneConfig {
  double resolution = 0.2;
};

class NativeSceneMap : public mgg::MapInterface {
 public:
  explicit NativeSceneMap(NativeSceneConfig config = {})
      : resolution_(config.resolution) {}
  using Cell = mgg::NativeMolaGrid::Cell;
  using VoxelStatus = mgg::VoxelStatus;
  using GainCounts = mgg::GainCounts;
  using SensorModel = mgg::SensorModel;
  using StateVec = mgg::StateVec;

  void insertPointCloud(const std::vector<Eigen::Vector3d>& points,
                        const Eigen::Vector3d& origin) {
    // Carve between cell centres: an endpoint on a grid plane must not
    // clear the neighbouring floor/wall merely by touching that plane.
    std::set<Cell> free, occupied;
    for (const auto& p : points) {
      if (!p.allFinite()) continue;
      const auto delta = p - origin;
      const bool hit = delta.norm() <= 20.0;
      const Eigen::Vector3d end = hit ? p : origin + delta.normalized() * 20.0;
      mgg::walkVoxels(center(key(origin)), center(key(end)), resolution_, 1u << 22,
          [&](const mgg::VoxelIndex& k) {
            free.insert({k.x, k.y, k.z});
            return true;
          });
      if (hit) {
        const Cell k = key(p);
        occupied.insert(k);
        auto [it, inserted] = surfaces_.emplace(k, p.z());
        if (!inserted) it->second = std::max(it->second, p.z());
      }
    }
    for (const auto& k : free)
      if (!occupied.count(k)) update(k, -0.4054651081081643);
    for (const auto& k : occupied) update(k, 0.8472978603872037);
    has_data_ = has_data_ || !points.empty();
    grid_.reset();
  }
  bool augmentFreeBox(const Eigen::Vector3d& center,
                      const Eigen::Vector3d& size) override {
    const Cell lo = key(center - size / 2), hi = key(center + size / 2);
    for (auto x = lo.x; x <= hi.x; ++x)
      for (auto y = lo.y; y <= hi.y; ++y)
        for (auto z = lo.z; z <= hi.z; ++z) {
          const Cell k{x, y, z};
          // Explicit fixture observation replaces prior occupancy.
          odds_[k] = -1.9924301646902063;
          surfaces_.erase(k);
        }
    has_data_ = true;
    grid_.reset();
    return true;
  }
  void setTrackMeasuredSurfaceZ(bool enabled) {
    measured_ = enabled;
    grid_.reset();
  }
  double getResolution() const override { return resolution_; }
  bool getStatus() const override { return has_data_; }
  void resetMap() override {
    odds_.clear();
    surfaces_.clear();
    grid_.reset();
    has_data_ = false;
  }
  void augmentFreeFrustum() override {}
  void setRaycastingParams(bool, double) override {}
  void setRobotRadius(double) override {}
  bool getAxisAlignedXYCellCenter(
      const Eigen::Vector2d& p,
      Eigen::Vector2d& c) const override {
    return grid().getAxisAlignedXYCellCenter(p, c);
  }
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    return grid().getVoxelStatus(p);
  }
  VoxelStatus getRayStatus(
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      bool u) const override {
    return grid().getRayStatus(a, b, u);
  }
  VoxelStatus getRayStatus(
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      bool u,
      Eigen::Vector3d& e) const override {
    return grid().getRayStatus(a, b, u, e);
  }
  VoxelStatus getGroundRayStatus(
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      bool u,
      Eigen::Vector3d& e) const override {
    return grid().getGroundRayStatus(a, b, u, e);
  }
  VoxelStatus getBoxStatus(
      const Eigen::Vector3d& c,
      const Eigen::Vector3d& s,
      bool u) const override {
    return grid().getBoxStatus(c, s, u);
  }
  VoxelStatus getPathStatus(
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      const Eigen::Vector3d& s,
      bool u) const override {
    return grid().getPathStatus(a, b, s, u);
  }
  VoxelStatus getStrictBoxStatus(
      const Eigen::Vector3d& c,
      const Eigen::Vector3d& s) const override {
    return grid().getStrictBoxStatus(c, s);
  }
  VoxelStatus getStrictPathStatus(
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      const Eigen::Vector3d& s) const override {
    return grid().getStrictPathStatus(a, b, s);
  }
  VoxelStatus getOccupiedOnlyPathStatus(
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      const Eigen::Vector3d& s) const override {
    return grid().getOccupiedOnlyPathStatus(a, b, s);
  }
  VoxelStatus getOccupiedOnlyCylinderPathStatus(
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      double r,
      double h) const override {
    return grid().getOccupiedOnlyCylinderPathStatus(a, b, r, h);
  }
  bool aerialRootRecoveryTraversable(
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b,
      const Eigen::Vector3d& s) const override {
    return grid().aerialRootRecoveryTraversable(a, b, s);
  }
  void getScanStatus(
      const Eigen::Vector3d& p,
      const std::vector<Eigen::Vector3d>& ends,
      GainCounts& gain,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& log,
      const SensorModel& sensor) override {
    return grid().getScanStatus(p, ends, gain, log, sensor);
  }
  void getScanStatusIterative(
      const Eigen::Vector3d& p,
      const std::vector<Eigen::Vector3d>& ends,
      GainCounts& gain,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& log,
      const SensorModel& sensor) override {
    return grid().getScanStatusIterative(p, ends, gain, log, sensor);
  }
  void extractLocalMap(
      const Eigen::Vector3d& p,
      const Eigen::Vector3d& size,
      std::vector<Eigen::Vector3d>& occupied,
      std::vector<Eigen::Vector3d>& free) override {
    return grid().extractLocalMap(p, size, occupied, free);
  }
  void extractLocalMapAlongAxis(
      const Eigen::Vector3d& p,
      const Eigen::Vector3d& axis,
      const Eigen::Vector3d& size,
      std::vector<Eigen::Vector3d>& occupied,
      std::vector<Eigen::Vector3d>& free) override {
    return grid().extractLocalMapAlongAxis(p, axis, size, occupied, free);
  }
  void getLocalPointcloud(
      const Eigen::Vector3d& p,
      double range,
      double yaw,
      std::vector<Eigen::Vector3d>& points,
      bool u = false) override {
    return grid().getLocalPointcloud(p, range, yaw, points, u);
  }
  void getFreeSpacePointCloud(
      const std::vector<Eigen::Vector3d>& ends,
      const StateVec& state,
      std::vector<Eigen::Vector3d>& points) override {
    return grid().getFreeSpacePointCloud(ends, state, points);
  }

 private:
  Cell key(const Eigen::Vector3d& p) const {
    // Scene loops describe exact decimal lattice boundaries. Remove only
    // their floating-point stepping noise before constructing cell data.
    const Eigen::Vector3d q = p / resolution_;
    return {std::int64_t(std::floor(q.x() + 1e-9)),
            std::int64_t(std::floor(q.y() + 1e-9)),
            std::int64_t(std::floor(q.z() + 1e-9))};
  }
  Eigen::Vector3d center(const Cell& k) const {
    return resolution_ * Eigen::Vector3d(k.x + .5, k.y + .5, k.z + .5);
  }
  void update(const Cell& k, double increment) {
    auto& odds = odds_[k];
    odds = std::clamp(odds + increment, -1.9924301646902063, 3.4760986898352724);
  }
  mgg::NativeMolaGrid& grid() const {
    if (!grid_) {
      std::vector<Cell> occupied, free;
      std::vector<mgg::NativeMolaGrid::Surface> surfaces;
      for (const auto& [k, odds] : odds_) {
        (odds >= 0 ? occupied : free).push_back(k);
        const auto top = surfaces_.find(k);
        if (measured_ && odds >= 0 && top != surfaces_.end())
          surfaces.push_back({k, top->second});
      }
      grid_ = std::make_unique<mgg::NativeMolaGrid>(
          resolution_, std::move(occupied), std::move(free), std::move(surfaces));
    }
    return *grid_;
  }
  double resolution_;
  bool has_data_ = false, measured_ = false;
  std::map<Cell, double> odds_, surfaces_;
  mutable std::unique_ptr<mgg::NativeMolaGrid> grid_;
};

// Adversarial ordinary query for the shortcut test only. The planner must
// certify with strict queries even when a provider tolerates unknown space.
class PermissiveSceneMap : public NativeSceneMap {
 public:
  using NativeSceneMap::NativeSceneMap;
  VoxelStatus getPathStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                            const Eigen::Vector3d& size, bool) const override {
    return NativeSceneMap::getPathStatus(a, b, size, false);
  }
};
}  // namespace mgg_test
#endif
