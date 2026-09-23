#ifndef MGG_MAP_OCTOMAP_NATIVE_MOLA_GRID_H_
#define MGG_MAP_OCTOMAP_NATIVE_MOLA_GRID_H_

#include <cstdint>
#include <map>
#include <vector>

#include "mgg_core/map_interface.h"
#include "mgg_map_octomap/cell_index.h"

namespace mgg {

/// Immutable, bounded adapter for the native SDMGRID1 ternary grid.
class NativeMolaGrid final : public MapInterface {
 public:
  struct Cell {
    std::int64_t x = 0, y = 0, z = 0;
    friend bool operator<(const Cell& a, const Cell& b) {
      if (a.x != b.x) return a.x < b.x;
      if (a.y != b.y) return a.y < b.y;
      return a.z < b.z;
    }
  };
  struct Surface {
    Cell cell;
    double max_z = 0.0;
  };

  NativeMolaGrid(double resolution, std::vector<Cell> occupied,
                 std::vector<Cell> free, std::vector<Surface> surfaces);
  double getResolution() const override { return resolution_; }
  bool getAxisAlignedXYCellCenter(const Eigen::Vector2d&,
                                  Eigen::Vector2d&) const override;
  bool getStatus() const override { return true; }
  VoxelStatus getVoxelStatus(const Eigen::Vector3d&) const override;
  VoxelStatus getRayStatus(const Eigen::Vector3d&, const Eigen::Vector3d&,
                           bool) const override;
  VoxelStatus getRayStatus(const Eigen::Vector3d&, const Eigen::Vector3d&, bool,
                           Eigen::Vector3d&) const override;
  VoxelStatus getGroundRayStatus(const Eigen::Vector3d&, const Eigen::Vector3d&,
                                 bool, Eigen::Vector3d&) const override;
  VoxelStatus getBoxStatus(const Eigen::Vector3d&, const Eigen::Vector3d&,
                           bool) const override;
  VoxelStatus getPathStatus(const Eigen::Vector3d&, const Eigen::Vector3d&,
                            const Eigen::Vector3d&, bool) const override;
  VoxelStatus getOccupiedOnlyCylinderPathStatus(const Eigen::Vector3d&,
                                                const Eigen::Vector3d&, double,
                                                double) const override;
  VoxelStatus getStrictBoxStatus(const Eigen::Vector3d&,
                                 const Eigen::Vector3d&) const override;
  VoxelStatus getStrictPathStatus(const Eigen::Vector3d&,
                                  const Eigen::Vector3d&,
                                  const Eigen::Vector3d&) const override;
  void getScanStatus(const Eigen::Vector3d&,
                     const std::vector<Eigen::Vector3d>&, GainCounts&,
                     std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>&,
                     const SensorModel&) override;
  void getScanStatusIterative(
      const Eigen::Vector3d&, const std::vector<Eigen::Vector3d>&, GainCounts&,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>&,
      const SensorModel&) override;
  bool augmentFreeBox(const Eigen::Vector3d&, const Eigen::Vector3d&) override {
    return false;
  }
  void augmentFreeFrustum() override {}
  void resetMap() override {}
  void extractLocalMap(const Eigen::Vector3d&, const Eigen::Vector3d&,
                       std::vector<Eigen::Vector3d>&,
                       std::vector<Eigen::Vector3d>&) override;
  void extractLocalMapAlongAxis(const Eigen::Vector3d&, const Eigen::Vector3d&,
                                const Eigen::Vector3d&,
                                std::vector<Eigen::Vector3d>&,
                                std::vector<Eigen::Vector3d>&) override;
  void getLocalPointcloud(const Eigen::Vector3d&, double, double,
                          std::vector<Eigen::Vector3d>&, bool = false) override;
  void getFreeSpacePointCloud(const std::vector<Eigen::Vector3d>&,
                              const StateVec&,
                              std::vector<Eigen::Vector3d>&) override;
  void setRaycastingParams(bool, double) override {}
  void setRobotRadius(double) override {}

 private:
  static constexpr std::uint64_t kMaxWork = 1u << 22;
  bool key(const Eigen::Vector3d&, Cell&) const;
  Eigen::Vector3d center(const Cell&) const;
  VoxelStatus status(const Cell&) const;
  VoxelStatus box(const Eigen::Vector3d&, const Eigen::Vector3d&, bool,
                  bool) const;
  VoxelStatus path(const Eigen::Vector3d&, const Eigen::Vector3d&,
                   const Eigen::Vector3d&, bool, bool) const;
  template <class F>
  bool walk(const Eigen::Vector3d&, const Eigen::Vector3d&, F) const;
  double resolution_;
  std::vector<Cell> occupied_, free_;
  CellIndex<Cell> cell_index_;
  std::map<Cell, double> surface_max_z_;
};
}  // namespace mgg
#endif
