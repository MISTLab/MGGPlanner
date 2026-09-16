// Immutable MOLA planner snapshots exposed through MapInterface.

#ifndef MGG_MAP_OCTOMAP_MOLA_MAP_H_
#define MGG_MAP_OCTOMAP_MOLA_MAP_H_

#include <chrono>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/map_interface.h"
#include "mgg_map_octomap/octomap_map.h"

namespace mgg {

struct MolaMapConfig {
  std::string peer_root;
  double resolution = 0.2;
  double snapshot_ttl_sec = 3.0;
  std::size_t max_snapshot_bytes = 4u * 1024u * 1024u;
  std::size_t max_index_bytes = 4u * 1024u * 1024u;
  std::size_t max_grid_bytes = 256u * 1024u * 1024u;
  std::size_t max_voxels = 2000000u;
  std::chrono::milliseconds max_load_time{2000};
};

/// Exact authority identity associated with one MappingSnapshot message.
struct MolaSnapshotRequest {
  std::string component_id;
  std::uint64_t epoch = 0;
  std::uint64_t graph_revision = 0;
  std::string geometry_revision;
  std::uint64_t source_stamp_ns = 0;
  /// Transform from navigation coordinates into the component map frame.
  Eigen::Isometry3d component_from_navigation = Eigen::Isometry3d::Identity();
};

/// A correction-aware MOLA provider. Requests enqueue bounded filesystem
/// validation and tree construction on one worker; planner callbacks never
/// decode a grid. The active immutable tree is replaced atomically.
class MolaMap : public MapInterface {
 public:
  /// Keeps one immutable active snapshot installed for a bounded group of
  /// MapInterface calls. Publication can continue decoding off-thread, but
  /// commits wait until the lease is released.
  class ReadLease {
   public:
    ReadLease() = default;
    ReadLease(ReadLease&&) = default;
    ReadLease& operator=(ReadLease&&) = default;

    ReadLease(const ReadLease&) = delete;
    ReadLease& operator=(const ReadLease&) = delete;

   private:
    friend class MolaMap;
    explicit ReadLease(std::recursive_mutex& mutex) : lock_(mutex) {}
    std::unique_lock<std::recursive_mutex> lock_;
  };

  explicit MolaMap(MolaMapConfig config);
  ~MolaMap() override;

  MolaMap(const MolaMap&) = delete;
  MolaMap& operator=(const MolaMap&) = delete;

  /// Queue the latest authority heartbeat. Even an unchanged key is
  /// revalidated, so disappearance or retraction cannot refresh freshness.
  void requestSnapshot(const MolaSnapshotRequest& request);
  ReadLease acquireReadLease() const;
  std::string lastError() const;
  std::uint64_t activeGeneration() const;

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
  VoxelStatus getBoxStatus(const Eigen::Vector3d& center,
                           const Eigen::Vector3d& size,
                           bool stop_at_unknown_voxel) const override;
  VoxelStatus getPathStatus(const Eigen::Vector3d& start,
                            const Eigen::Vector3d& end,
                            const Eigen::Vector3d& box_size,
                            bool stop_at_unknown_voxel) const override;
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
  void extractLocalMapAlongAxis(
      const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
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
  struct Snapshot;
  struct PendingRequest;

  std::shared_ptr<const Snapshot> current() const;
  void workerLoop();
  std::shared_ptr<const Snapshot> load(const PendingRequest& pending) const;
  void failIfLatest(std::uint64_t generation, const std::string& error);

  MolaMapConfig config_;
  mutable std::mutex request_mutex_;
  std::condition_variable request_ready_;
  std::unique_ptr<PendingRequest> pending_;
  std::uint64_t generation_ = 0;
  bool stopping_ = false;
  std::thread worker_;

  mutable std::mutex error_mutex_;
  std::string last_error_;
  mutable std::recursive_mutex publication_mutex_;
  mutable std::shared_ptr<const Snapshot> active_;
  mutable std::atomic<std::uint64_t> active_generation_{0};
};

}  // namespace mgg

#endif  // MGG_MAP_OCTOMAP_MOLA_MAP_H_
