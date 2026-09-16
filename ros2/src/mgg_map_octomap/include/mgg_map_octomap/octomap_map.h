// A MapInterface backed by OctoMap.
//
// This is the replacement for MapManagerVoxblox. The planner only ever asked
// the map for ternary occupancy (see section 4 of ROS2_PORT_PLAN.md), and
// OctoMap's semantics map onto that directly:
//
//     voxblox                              octomap
//     -------                              -------
//     weight < 1e-6            unknown     search() == nullptr
//     distance <= voxel_size   occupied    isNodeOccupied(node)
//     otherwise                free        !isNodeOccupied(node)
//
// Deliberately free of ROS: octomap is a plain library, so this class can be
// unit tested and reused off-robot. The PointCloud2/TF front end lives in
// octomap_map_node.
//
// One behavioural difference is expected and is NOT hidden. Voxblox marks
// every voxel within the truncation band of a surface as occupied, so
// obstacles are effectively dilated by roughly one voxel; OctoMap marks only
// the voxels rays actually terminate in. `occupied_dilation_voxels` exists to
// compensate if the baseline comparison shows it matters. It defaults to 0,
// i.e. no fudging until the numbers justify it.

#ifndef MGG_MAP_OCTOMAP_OCTOMAP_MAP_H_
#define MGG_MAP_OCTOMAP_OCTOMAP_MAP_H_

#include <array>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include <octomap/OcTree.h>
#include <octomap/octomap_types.h>

#include "mgg_core/map_interface.h"
#include "mgg_core/types.h"

namespace mgg {

struct OctomapConfig {
  double resolution = 0.2;        ///< matches the voxblox tsdf_voxel_size
  double probability_hit = 0.7;
  double probability_miss = 0.4;
  double clamping_min = 0.12;
  double clamping_max = 0.97;
  double occupancy_threshold = 0.5;
  double max_range = 20.0;        ///< -1 disables range clipping
  /// Extra voxels of obstacle inflation; see the note at the top.
  int occupied_dilation_voxels = 0;
};

class OctomapMap : public MapInterface {
 public:
  explicit OctomapMap(const OctomapConfig& config = OctomapConfig());

  /// Integrate a scan taken from `sensor_origin`; points are in world frame.
  void insertPointCloud(const std::vector<Eigen::Vector3d>& points,
                        const Eigen::Vector3d& sensor_origin);
  void setTrackMeasuredSurfaceZ(bool enabled);
  std::size_t measuredSurfaceCount() const {
    return measured_surface_max_z_.size();
  }

  octomap::OcTree* tree() { return tree_.get(); }
  const octomap::OcTree* tree() const { return tree_.get(); }

  // ------------------------------------------------------ MapInterface

  double getResolution() const override;
  bool getStatus() const override;

  VoxelStatus getVoxelStatus(const Eigen::Vector3d& position) const override;

  VoxelStatus getRayStatus(const Eigen::Vector3d& view_point,
                           const Eigen::Vector3d& voxel_to_test,
                           bool stop_at_unknown_voxel) const override;
  VoxelStatus getRayStatus(const Eigen::Vector3d& view_point,
                           const Eigen::Vector3d& voxel_to_test,
                           bool stop_at_unknown_voxel,
                           Eigen::Vector3d& end_voxel) const override;
  VoxelStatus getGroundRayStatus(
      const Eigen::Vector3d& view_point,
      const Eigen::Vector3d& voxel_to_test, bool stop_at_unknown_voxel,
      Eigen::Vector3d& end_voxel) const override;

  VoxelStatus getBoxStatus(const Eigen::Vector3d& center,
                           const Eigen::Vector3d& size,
                           bool stop_at_unknown_voxel) const override;
  VoxelStatus getPathStatus(const Eigen::Vector3d& start,
                            const Eigen::Vector3d& end,
                            const Eigen::Vector3d& box_size,
                            bool stop_at_unknown_voxel) const override;

  /// Explicit objectives require every touched voxel to be observed free.
  /// Legacy MapInterface queries retain their configured historical tolerance.
  VoxelStatus getStrictBoxStatus(const Eigen::Vector3d& center,
                                  const Eigen::Vector3d& size) const override;
  VoxelStatus getStrictPathStatus(const Eigen::Vector3d& start,
                                   const Eigen::Vector3d& end,
                                   const Eigen::Vector3d& box_size) const override;

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

  bool augmentFreeBox(const Eigen::Vector3d& position,
                      const Eigen::Vector3d& box_size) override;
  void augmentFreeFrustum() override;
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

  void getLocalPointcloud(const Eigen::Vector3d& center, double range,
                          double yaw, std::vector<Eigen::Vector3d>& points,
                          bool include_unknown_voxels = false) override;
  void getFreeSpacePointCloud(
      const std::vector<Eigen::Vector3d>& multiray_endpoints,
      const StateVec& state, std::vector<Eigen::Vector3d>& points) override;

  void setRaycastingParams(bool nonuniform_ray_cast,
                           double ray_cast_step_size_multiplier) override;
  void setRobotRadius(double robot_radius) override;

 private:
  class UpdateAwareOcTree : public octomap::OcTree {
   public:
    using octomap::OcTree::OcTree;
    using octomap::OccupancyOcTreeBase<octomap::OcTreeNode>::computeUpdate;
  };

  /// Walks a segment voxel by voxel, applying `visit` to each status until it
  /// returns false. Shared by every ray-shaped query so they cannot drift
  /// apart.
  template <typename Visitor>
  void walkRay(const Eigen::Vector3d& from, const Eigen::Vector3d& to,
               Visitor visit) const;

  VoxelStatus statusAt(const octomap::point3d& p) const;
  VoxelStatus queryBox(const Eigen::Vector3d& center,
                       const Eigen::Vector3d& size,
                       double unknown_fraction) const;

  std::unique_ptr<UpdateAwareOcTree> tree_;
  OctomapConfig config_;
  bool nonuniform_ray_cast_ = true;
  double ray_cast_step_size_multiplier_ = 1.0;
  double robot_radius_ = 0.0;
  bool has_data_ = false;
  bool track_measured_surface_z_ = false;
  using SurfaceKey = std::array<octomap::key_type, 3>;
  std::map<SurfaceKey, double> measured_surface_max_z_;
};

}  // namespace mgg

#endif  // MGG_MAP_OCTOMAP_OCTOMAP_MAP_H_
