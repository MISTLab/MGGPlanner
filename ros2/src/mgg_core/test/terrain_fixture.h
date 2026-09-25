// A mapped ground surface read from test/data, as a MapInterface.
//
// The fixtures hold the top of the ground in each 0.2 m column under and
// around a path a robot drove, taken from the maps of the SubT run of
// 2026-09-24 (test/data/README.md). Only the surface is recorded, not free
// space, so the map answers as NativeMolaGrid does for its measured
// surface and treats every other cell as free:
//   * the XY cell grid is axis-aligned, as NativeMolaGrid's is;
//   * a downward ray stops in the column's top cell and returns the cell's
//     centre at the surface's height;
//   * a box is occupied when a column under it rises above its underside.

#ifndef MGG_CORE_TEST_TERRAIN_FIXTURE_H_
#define MGG_CORE_TEST_TERRAIN_FIXTURE_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mgg_core/map_interface.h"

namespace mgg_test {

class TerrainFixture : public mgg::MapInterface {
 public:
  explicit TerrainFixture(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open " + path);
    std::string line;
    while (std::getline(in, line)) {
      std::istringstream fields(line);
      std::string kind;
      if (!(fields >> kind) || kind[0] == '#') continue;
      if (kind == "resolution") {
        fields >> resolution_;
      } else if (kind == "pose") {
        Eigen::Vector3d pose;
        fields >> pose.x() >> pose.y() >> pose.z();
        poses_.push_back(pose);
      } else if (kind == "cell") {
        std::int64_t x = 0, y = 0;
        double top = 0.0;
        fields >> x >> y >> top;
        top_[{x, y}] = top;
      }
    }
    if (!(resolution_ > 0.0) || poses_.size() < 2 || top_.empty()) {
      throw std::runtime_error("malformed terrain fixture " + path);
    }
  }

  /// A surface given column by column: `tops` maps a cell's (x, y) index to
  /// the height of its top.
  TerrainFixture(double resolution,
                 std::map<std::pair<std::int64_t, std::int64_t>, double> tops)
      : resolution_(resolution), top_(std::move(tops)) {}

  /// Where the robot's base was at each keyframe, in drive order.
  const std::vector<Eigen::Vector3d>& poses() const { return poses_; }

  double getResolution() const override { return resolution_; }
  bool getAxisAlignedXYCellCenter(const Eigen::Vector2d& p,
                                  Eigen::Vector2d& center) const override {
    center = Eigen::Vector2d((std::floor(p.x() / resolution_) + 0.5),
                             (std::floor(p.y() / resolution_) + 0.5)) *
             resolution_;
    return center.allFinite();
  }
  bool getStatus() const override { return true; }

  mgg::VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    double top = 0.0;
    if (!columnTop(p, top)) return mgg::VoxelStatus::kFree;
    return p.z() <= top && p.z() >= cellBottom(top)
               ? mgg::VoxelStatus::kOccupied
               : mgg::VoxelStatus::kFree;
  }

  mgg::VoxelStatus getRayStatus(const Eigen::Vector3d& a,
                                const Eigen::Vector3d& b,
                                bool stop_at_unknown) const override {
    Eigen::Vector3d ignored;
    return getRayStatus(a, b, stop_at_unknown, ignored);
  }

  /// Vertical rays only, which is all ground projection casts.
  mgg::VoxelStatus getRayStatus(const Eigen::Vector3d& a,
                                const Eigen::Vector3d& b, bool,
                                Eigen::Vector3d& end_voxel) const override {
    end_voxel = b;
    double top = 0.0;
    if (!columnTop(a, top) || a.z() < cellBottom(top) || b.z() > top) {
      return mgg::VoxelStatus::kFree;
    }
    end_voxel = Eigen::Vector3d(
        (std::floor(a.x() / resolution_) + 0.5) * resolution_,
        (std::floor(a.y() / resolution_) + 0.5) * resolution_, top);
    return mgg::VoxelStatus::kOccupied;
  }

  mgg::VoxelStatus getBoxStatus(const Eigen::Vector3d& center,
                                const Eigen::Vector3d& size,
                                bool) const override {
    const Eigen::Vector3d lo = center - 0.5 * size;
    const Eigen::Vector3d hi = center + 0.5 * size;
    for (auto x = key(lo.x()); x <= key(hi.x()); ++x) {
      for (auto y = key(lo.y()); y <= key(hi.y()); ++y) {
        const auto it = top_.find({x, y});
        if (it != top_.end() && it->second >= lo.z() &&
            cellBottom(it->second) <= hi.z()) {
          return mgg::VoxelStatus::kOccupied;
        }
      }
    }
    return mgg::VoxelStatus::kFree;
  }

  /// NativeMolaGrid's sweep: one box per map cell of travel, grown by the
  /// step.
  mgg::VoxelStatus getPathStatus(const Eigen::Vector3d& a,
                                 const Eigen::Vector3d& b,
                                 const Eigen::Vector3d& size,
                                 bool stop_at_unknown) const override {
    const double steps =
        std::max(1.0, std::ceil((b - a).norm() / resolution_));
    const Eigen::Vector3d step = (b - a) / steps;
    const Eigen::Vector3d swept = size + step.cwiseAbs();
    for (int i = 0; i < static_cast<int>(steps); ++i) {
      const mgg::VoxelStatus s =
          getBoxStatus(a + (i + 0.5) * step, swept, stop_at_unknown);
      if (s != mgg::VoxelStatus::kFree) return s;
    }
    return mgg::VoxelStatus::kFree;
  }

  void getScanStatus(
      const Eigen::Vector3d&, const std::vector<Eigen::Vector3d>&,
      mgg::GainCounts&,
      std::vector<std::pair<Eigen::Vector3d, mgg::VoxelStatus>>&,
      const mgg::SensorModel&) override {}
  void getScanStatusIterative(
      const Eigen::Vector3d&, const std::vector<Eigen::Vector3d>&,
      mgg::GainCounts&,
      std::vector<std::pair<Eigen::Vector3d, mgg::VoxelStatus>>&,
      const mgg::SensorModel&) override {}
  bool augmentFreeBox(const Eigen::Vector3d&,
                      const Eigen::Vector3d&) override {
    return true;
  }
  void augmentFreeFrustum() override {}
  void resetMap() override {}
  void extractLocalMap(const Eigen::Vector3d&, const Eigen::Vector3d&,
                       std::vector<Eigen::Vector3d>&,
                       std::vector<Eigen::Vector3d>&) override {}
  void extractLocalMapAlongAxis(const Eigen::Vector3d&,
                                const Eigen::Vector3d&,
                                const Eigen::Vector3d&,
                                std::vector<Eigen::Vector3d>&,
                                std::vector<Eigen::Vector3d>&) override {}
  void getLocalPointcloud(const Eigen::Vector3d&, double, double,
                          std::vector<Eigen::Vector3d>&, bool) override {}
  void getFreeSpacePointCloud(const std::vector<Eigen::Vector3d>&,
                              const mgg::StateVec&,
                              std::vector<Eigen::Vector3d>&) override {}
  void setRaycastingParams(bool, double) override {}
  void setRobotRadius(double) override {}

 private:
  std::int64_t key(double v) const {
    return static_cast<std::int64_t>(std::floor(v / resolution_));
  }
  double cellBottom(double top) const {
    return std::floor(top / resolution_) * resolution_;
  }
  bool columnTop(const Eigen::Vector3d& p, double& top) const {
    const auto it = top_.find({key(p.x()), key(p.y())});
    if (it == top_.end()) return false;
    top = it->second;
    return true;
  }

  double resolution_ = 0.0;
  std::vector<Eigen::Vector3d> poses_;
  std::map<std::pair<std::int64_t, std::int64_t>, double> top_;
};

}  // namespace mgg_test

#endif  // MGG_CORE_TEST_TERRAIN_FIXTURE_H_
