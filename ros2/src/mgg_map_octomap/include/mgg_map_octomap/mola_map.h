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
#include "mgg_map_octomap/native_mola_grid.h"

namespace mgg {

struct MolaMapConfig {
  std::string peer_root;
  double resolution = 0.2;
  /// An active snapshot expires this long after its last on-disk
  /// confirmation: the load that built it, a same-key reload, or a compatible
  /// successor heartbeat that found no product for its revision yet.
  double snapshot_ttl_sec = 3.0;
  /// Bound on `<peer_root>/mola/source.json`.
  std::size_t max_snapshot_bytes = 64u * 1024u * 1024u;
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
/// validation and native-grid construction on one worker; planner callbacks never
/// decode a grid. The active immutable tree is replaced atomically.
///
/// The product is self-described: the mapping worker writes
/// `<peer_root>/mola/source.json` (the exact mapping snapshot bytes it built
/// from) and then `<peer_root>/mola/index.json`, whose `source_sha256` and
/// `source_snapshot_id` name those bytes. `<peer_root>/snapshot.json` may run
/// ahead of the product and is never read here.
///
/// Two requests are compatible when they name the same component and epoch
/// and carry the same authority transform. A compatible successor request
/// (a newer graph revision, geometry revision or source stamp) keeps the
/// active snapshot in service while its product is decoded; if the product on
/// disk still describes another revision when the load budget runs out, the
/// predecessor is kept and its validity clock refreshed, because a
/// product-gated authority key only names revisions the worker has published.
/// An incompatible or invalid request retracts the active snapshot at once.
class MolaMap : public MapInterface {
 private:
  struct Snapshot;

 public:
  /// Keeps one exact immutable snapshot for a bounded group of MapInterface
  /// calls. Publication is excluded by default; allowPublication() lets a
  /// worker commit while this transaction retains its admitted snapshot.
  class ReadLease {
   public:
    ReadLease() = default;
    ReadLease(ReadLease&& other) noexcept;
    ReadLease& operator=(ReadLease&& other) noexcept;
    ~ReadLease();

    ReadLease(const ReadLease&) = delete;
    ReadLease& operator=(const ReadLease&) = delete;

    /// Permit snapshot publication while retaining this transaction's exact
    /// immutable snapshot. The lease remains thread-affine.
    void allowPublication();
    /// Re-establish publication exclusion before completing the transaction.
    void reacquirePublication();

   private:
    friend class MolaMap;
    explicit ReadLease(const MolaMap& owner);
    void release() noexcept;

    const MolaMap* owner_ = nullptr;
    std::shared_ptr<const Snapshot> snapshot_;
    std::thread::id owner_thread_;
    std::unique_lock<std::recursive_mutex> lock_;
  };

  explicit MolaMap(MolaMapConfig config);
  ~MolaMap() override;

  MolaMap(const MolaMap&) = delete;
  MolaMap& operator=(const MolaMap&) = delete;

  /// Queue the latest authority heartbeat. Even an unchanged key is
  /// revalidated, so disappearance or retraction cannot refresh freshness:
  /// a heartbeat that repeats the active identity is confirmed by stat of
  /// the product files it was read from (statRevalidationCount()), any
  /// other is loaded. A request compatible with the active snapshot leaves
  /// it in service; an incompatible or invalid one retracts it before this
  /// call returns.
  void requestSnapshot(const MolaSnapshotRequest& request);
  ReadLease acquireReadLease() const;
  /// The error that removed or withheld the active snapshot; empty while a
  /// snapshot is served, including one retained across a pending successor.
  std::string lastError() const;
  std::uint64_t activeGeneration() const;
  /// Number of successor loads that ended in a coherence race after the load
  /// budget while a compatible predecessor stayed in service.
  std::uint64_t retainedPredecessorCount() const;
  /// Number of heartbeats confirmed against the active snapshot's product
  /// files by stat alone, without a load.
  std::uint64_t statRevalidationCount() const;

  double getResolution() const override;
  bool getCircleIntersectingXYCellCenters(
      const Eigen::Vector2d& circle_center, double radius,
      std::size_t maximum_cells,
      std::vector<XYCellCenter>& centers) const override;
  bool getStatus() const override;

  /// Bodies that stand in the world and in no map: other robots of the fleet.
  /// A capture-time mask keeps them out of the persistent product, so without
  /// this the planner routes straight through a parked neighbour. Each disc is
  /// a vertical cylinder of unbounded height in the navigation frame; the box,
  /// path and cylinder queries report it occupied. The list expires after
  /// `ttl_s`, so a silent publisher cannot freeze an obstacle in place. The
  /// cost per query is one distance test per disc.
  void setTransientDiscs(std::vector<Eigen::Vector2d> centres, double radius_m,
                         double ttl_s);
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
  VoxelStatus getOccupiedOnlyCylinderPathStatus(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end, double radius,
      double height) const override;
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
  /// The wall band moves with the viewpoint into the component frame. The
  /// frames differ by at most kMaxAuthorityTiltRad of tilt, which shifts the
  /// band by the tilt times the distance from the viewpoint: 0.1 m at 5 m
  /// for a 0.02 rad (1.1 degree) merged frame.
  void getVisibleScanStatus(
      const Eigen::Vector3d& pos,
      const std::vector<Eigen::Vector3d>& multiray_endpoints,
      const WallBand& wall, GainCounts& gain,
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
  struct PendingRequest;
  struct ThreadPin {
    const MolaMap* owner = nullptr;
    std::shared_ptr<const Snapshot> snapshot;
    std::size_t depth = 0;
  };

  std::shared_ptr<const Snapshot> current() const;
  std::shared_ptr<const Snapshot> beginReadLease() const;
  void endReadLease(const std::shared_ptr<const Snapshot>& snapshot) const;
  void workerLoop();
  std::shared_ptr<const Snapshot> load(const PendingRequest& pending) const;
  /// One read of the peer directory against a fixed deadline.
  std::shared_ptr<const Snapshot> loadOnce(const PendingRequest& pending,
                                           std::chrono::steady_clock::time_point deadline) const;
  /// Structural failure: drop the active snapshot and record the error unless
  /// a newer request has already superseded this one.
  void failIfLatest(std::uint64_t generation, const std::string& error);
  /// Coherence race after the retry budget: keep a compatible predecessor in
  /// service and refresh its validity clock, otherwise fail as failIfLatest.
  void retainPredecessorOrFail(const PendingRequest& pending,
                               const std::string& error);
  /// The product files are the ones this snapshot's geometry was read from.
  bool publicationUnchanged(const Snapshot& snapshot) const;

  MolaMapConfig config_;
  mutable std::mutex request_mutex_;
  std::condition_variable request_ready_;
  std::unique_ptr<PendingRequest> pending_;
  std::uint64_t generation_ = 0;
  bool stopping_ = false;
  std::thread worker_;

  struct TransientDiscs {
    std::vector<Eigen::Vector2d> centres;
    double radius_m = 0.0;
    std::chrono::steady_clock::time_point expires;
  };
  std::shared_ptr<const TransientDiscs> transient_discs_;
  bool discsBlockBox(const Eigen::Vector3d& center,
                     const Eigen::Vector3d& size) const;
  bool discsBlockSweep(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                       double half_width) const;
  mutable std::mutex error_mutex_;
  std::string last_error_;
  mutable std::recursive_mutex publication_mutex_;
  mutable std::shared_ptr<const Snapshot> active_;
  mutable std::atomic<std::uint64_t> active_generation_{0};
  std::atomic<std::uint64_t> retained_predecessor_count_{0};
  std::atomic<std::uint64_t> stat_revalidation_count_{0};
  static thread_local std::vector<ThreadPin> thread_pins_;
};

}  // namespace mgg

#endif  // MGG_MAP_OCTOMAP_MOLA_MAP_H_
