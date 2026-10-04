// The rolling local voxel map shared by ground and drone local planning.
//
// A dense ring buffer of ternary voxels over a fixed window centred on the
// robot, in its odometry frame. Voxels that scroll out of the window are
// forgotten; outside the window every status is kUnknown. Keys are those of
// the world-anchored grid (mgg::keyOf), so they survive scrolling.
//
// Every mutating call returns the MapChange it made: the regions whose
// ternary state changed, plus the slabs evicted from and entering the window
// on a scroll. The revision changes only with such a change. There are no
// per-consumer cursors: the driver hands each consumer the MapChange of the
// call it made.
//
// Occupancy follows OctomapMap's defaults: hit/miss probability .7/.4,
// clamping .12/.97, occupied threshold .5, maximum range 20 m. Each scan
// updates a voxel at most once, and a hit in the scan outranks a miss, so a
// single grazing ray cannot flip a voxel that is solidly observed occupied.
//
// Built without OctoMap: this class must not depend on MGG_WITH_OCTOMAP.

#ifndef MGG_MAP_OCTOMAP_ROLLING_VOXEL_MAP_H_
#define MGG_MAP_OCTOMAP_ROLLING_VOXEL_MAP_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "mgg_core/dirty_region.h"
#include "mgg_core/map_interface.h"
#include "mgg_core/types.h"

namespace mgg {

struct RollingWindowParams {
  Eigen::Vector3d window_size_m{16, 16, 6};
  /// Drones set this: the window then recentres in z on the vehicle. Ground
  /// robots leave it unset, and the window's z stays put until reset().
  bool follow_vertical = false;
  double resolution = 0.2;
};

class RollingVoxelMap : public MapInterface {
 public:
  /// Throws std::invalid_argument for a non-positive or non-finite size or
  /// resolution, or a window too large to allocate densely.
  explicit RollingVoxelMap(const RollingWindowParams& params);

  /// Recentres on `robot`, then integrates rays from `origin` to each point.
  MapChange insertScan(const std::vector<Eigen::Vector3d>& points,
                       const Eigen::Vector3d& origin,
                       const Eigen::Vector3d& robot);
  /// Scrolls the window to centre on `robot` (in z only with
  /// follow_vertical). The first call places the window.
  MapChange recenter(const Eigen::Vector3d& robot);
  /// Forgets every voxel and places the window on `robot`, z included.
  MapChange reset(const Eigen::Vector3d& robot);

  /// The closed window: a point on its max face belongs to its last voxel.
  /// Empty before the window is placed.
  Eigen::AlignedBox3d window() const;
  std::optional<Eigen::AlignedBox3d> windowBounds() const override {
    return window();
  }
  std::uint64_t revision() const { return revision_; }
  VoxelKey keyOf(const Eigen::Vector3d& p) const;
  Eigen::Vector3d centerOf(const VoxelKey& key) const;

  // ------------------------------------------------------ MapInterface

  double getResolution() const override { return resolution_; }
  bool getAxisAlignedXYCellCenter(const Eigen::Vector2d& position,
                                  Eigen::Vector2d& center) const override;
  /// True once a scan has been integrated since construction or reset.
  bool getStatus() const override { return placed_ && has_data_; }
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& position) const override;
  VoxelStatus getRayStatus(const Eigen::Vector3d& view_point,
                           const Eigen::Vector3d& voxel_to_test,
                           bool stop_at_unknown_voxel) const override;
  VoxelStatus getRayStatus(const Eigen::Vector3d& view_point,
                           const Eigen::Vector3d& voxel_to_test,
                           bool stop_at_unknown_voxel,
                           Eigen::Vector3d& end_voxel) const override;
  /// Any occupied voxel the closed box touches: kOccupied; otherwise, with
  /// `stop_at_unknown_voxel`, any unknown one: kUnknown.
  VoxelStatus getBoxStatus(const Eigen::Vector3d& center,
                           const Eigen::Vector3d& size,
                           bool stop_at_unknown_voxel) const override;
  VoxelStatus getPathStatus(const Eigen::Vector3d& start,
                            const Eigen::Vector3d& end,
                            const Eigen::Vector3d& box_size,
                            bool stop_at_unknown_voxel) const override;
  void getScanStatus(
      const Eigen::Vector3d& pos,
      const std::vector<Eigen::Vector3d>& multiray_endpoints, GainCounts& gain,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
      const SensorModel& sensor) override;
  void getScanStatusIterative(
      const Eigen::Vector3d& pos,
      const std::vector<Eigen::Vector3d>& multiray_endpoints, GainCounts& gain,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
      const SensorModel& sensor) override;
  /// Unsupported: a mutation outside insertScan/recenter/reset would bypass
  /// the MapChange consumers rely on.
  bool augmentFreeBox(const Eigen::Vector3d&,
                      const Eigen::Vector3d&) override {
    return false;
  }
  void augmentFreeFrustum() override {}
  /// Forgets every voxel and bumps the revision, keeping the window. Drivers
  /// that need the MapChange call reset() instead.
  void resetMap() override;
  void extractLocalMap(const Eigen::Vector3d& center,
                       const Eigen::Vector3d& bounding_box_size,
                       std::vector<Eigen::Vector3d>& occupied_voxels,
                       std::vector<Eigen::Vector3d>& free_voxels) override;
  void extractLocalMapAlongAxis(const Eigen::Vector3d& center,
                                const Eigen::Vector3d& axis,
                                const Eigen::Vector3d& bounding_box_size,
                                std::vector<Eigen::Vector3d>& occupied_voxels,
                                std::vector<Eigen::Vector3d>& free_voxels) override;
  /// Occupied voxel centres within `range`; known free ones as well with
  /// `include_unknown_voxels`, as OctomapMap does.
  void getLocalPointcloud(const Eigen::Vector3d& center, double range,
                          double yaw, std::vector<Eigen::Vector3d>& points,
                          bool include_unknown_voxels = false) override;
  void getFreeSpacePointCloud(
      const std::vector<Eigen::Vector3d>& multiray_endpoints,
      const StateVec& state, std::vector<Eigen::Vector3d>& points) override;
  void setRaycastingParams(bool, double) override {}
  void setRobotRadius(double) override {}

 private:
  bool inWindow(std::int64_t x, std::int64_t y, std::int64_t z) const;
  std::size_t slot(std::int64_t x, std::int64_t y, std::int64_t z) const;
  VoxelStatus status(std::int64_t x, std::int64_t y, std::int64_t z) const;
  /// The window voxel holding `p`, max faces included.
  bool cellOf(const Eigen::Vector3d& p, VoxelKey& key) const;
  VoxelKey originFor(const Eigen::Vector3d& robot) const;
  void clearAll();
  void clearCell(std::size_t slot);
  Eigen::AlignedBox3d windowAt(const VoxelKey& origin) const;
  VoxelStatus box(const Eigen::Vector3d& center, const Eigen::Vector3d& size,
                  bool stop_at_unknown) const;
  template <class F>
  bool walk(const Eigen::Vector3d& a, const Eigen::Vector3d& b, F visit) const;

  double resolution_;
  bool follow_vertical_;
  std::array<std::int64_t, 3> dims_{};
  bool placed_ = false;
  bool has_data_ = false;
  /// Key of the window's min voxel.
  VoxelKey origin_;
  std::uint64_t revision_ = 0;
  std::vector<float> log_odds_;
  std::vector<VoxelStatus> status_;
  /// Per-scan scratch: 0 untouched, 1 miss, 2 hit; reset after each scan.
  std::vector<std::uint8_t> scan_mark_;
  std::vector<std::pair<std::size_t, VoxelKey>> scan_touched_;
};

}  // namespace mgg

#endif  // MGG_MAP_OCTOMAP_ROLLING_VOXEL_MAP_H_
