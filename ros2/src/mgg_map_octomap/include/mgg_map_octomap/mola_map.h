// Immutable MOLA planner snapshots exposed through MapInterface.

#ifndef MGG_MAP_OCTOMAP_MOLA_MAP_H_
#define MGG_MAP_OCTOMAP_MOLA_MAP_H_

#include "mgg_core/no_go_zones.h"
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/map_interface.h"
#include "mgg_map_octomap/native_mola_grid.h"

namespace mgg {

/// One new grid put in service (MolaMap installs it on its worker thread).
struct MolaInstallation {
  /// Steady-clock seconds from receiving the authority heartbeat whose
  /// product this grid is (requestSnapshot) to its installation.
  double authority_to_install_s = 0.0;
  /// The product's newest keyframe stamp (the manifest's newest submap
  /// observed_at_ns, which the grid's source_stamp_ns must equal), in the
  /// mapping clock; 0 when the product has none.
  std::uint64_t source_stamp_ns = 0;
  std::string component_id;
  std::uint64_t graph_revision = 0;
};

struct MolaMapConfig {
  std::string peer_root;
  double resolution = 0.2;
  /// Authority expires this long after its last compatible receipt. Waiting
  /// for a geometry load or publication lease never extends that receipt.
  /// Compatible refinements renew the verified predecessor while loading.
  double snapshot_ttl_sec = 3.0;
  /// Bound on `<peer_root>/mola/source.json`.
  std::size_t max_snapshot_bytes = 64u * 1024u * 1024u;
  std::size_t max_index_bytes = 4u * 1024u * 1024u;
  std::size_t max_grid_bytes = 256u * 1024u * 1024u;
  std::size_t max_voxels = 2000000u;
  std::chrono::milliseconds max_load_time{2000};
  /// Called once per new grid installed, on the worker thread, outside
  /// every lock; not for a same-grid reload or a retained predecessor.
  std::function<void(const MolaInstallation&)> on_install{};
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
  mutable std::mutex aerial_recovery_mutex_;
  mutable std::uint64_t aerial_recovery_uses_ = 0;
  mutable std::size_t aerial_recovery_cells_ = 0;
  mutable Eigen::Vector3d aerial_recovery_direction_ = Eigen::Vector3d::Zero();

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

    /// Whether immutable geometry was admitted. This remains true after
    /// authority revocation, which authorityValid() checks independently.
    bool hasSnapshot() const { return snapshot_ != nullptr; }

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
  /// Authority validity, unlike getStatus(), is never prolonged by a pin.
  bool authorityValid() const;
  std::uint64_t expiryCount() const { return expiry_count_.load(); }
  /// Ground evidence only at the physical, oriented footprint. Does not
  /// certify free body volume, neighbouring ground, or overwrite a return.
  void setFootprintGroundSupport(const Eigen::Vector3d& floor_center,
                                 const Eigen::Vector2d& size, double yaw);
  std::optional<MolaSnapshotRequest> activeRequest() const;
  /// Number of successor loads that ended in a coherence race after the load
  /// budget while a compatible predecessor stayed in service.
  std::uint64_t retainedPredecessorCount() const;
  /// Number of heartbeats confirmed against the active snapshot's product
  /// files by stat alone, without a load.
  std::uint64_t statRevalidationCount() const;

  struct AerialRootRecoveryStats {
    std::uint64_t uses = 0;
    std::size_t exempted_cells = 0;
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
  };
  /// Successful fallback query count and latest accepted direction in nav.
  /// Counts admission queries, not executed motions (a route may be rechecked).
  AerialRootRecoveryStats aerialRootRecoveryStats() const;
  /// An aerial robot's body (lane drone-door): every box and path query
  /// then holds the box upright and aligned with the map's own grid, as
  /// given, and sweeps it exactly (NativeMolaGrid::getSweptBoxStatus),
  /// never its axis-aligned enclosure of the navigation-frame box. A drone's
  /// box is the square its round footprint circumscribes, so any one
  /// orientation is its body; the grid's makes voxels no wider than they
  /// are. Peer/no-go disc sweep radii follow half the configured box width;
  /// root recovery departure semantics are unchanged.
  /// Off for ground robots, whose box turns with them.
  void setGridAlignedBody(bool aligned);
  bool gridAlignedBody() const;
  /// The heading, in navigation coordinates, of the current snapshot's
  /// grid x axis: an aerial lattice laid out along it runs along the
  /// voxels. False without a snapshot.
  bool gridHeading(double& heading) const;
  bool aerialRootRecoveryTraversable(const Eigen::Vector3d&,
                                      const Eigen::Vector3d&,
                                      const Eigen::Vector3d&) const override;

  double getResolution() const override;
  bool getCircleIntersectingXYCellCenters(
      const Eigen::Vector2d& circle_center, double radius,
      std::size_t maximum_cells,
      std::vector<XYCellCenter>& centers) const override;
  bool getStatus() const override;

  /// Persistent no-go discs, such as tilt trip zones. Each message replaces
  /// the set; the publisher owns their lifetime.
  void setNoGoDiscs(std::vector<Eigen::Vector2d> centres, double radius_m);
  /// Already body-inflated centre-line reaches, independent of terrain discs.
  void setNoGoCentreLineDiscs(std::vector<Eigen::Vector2d> centres,
                              std::vector<double> reaches);
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
  bool dynamicBoxBlocked(const Eigen::Vector3d& center,
                         const Eigen::Vector3d& size) const override;
  bool dynamicSweepBlocked(const Eigen::Vector3d& start,
                           const Eigen::Vector3d& end,
                           double half_width) const override;
  VoxelStatus getStaticBoxStatus(const Eigen::Vector3d& center,
                                const Eigen::Vector3d& size,
                                bool stop_at_unknown) const override;
  VoxelStatus getBoxStatus(const Eigen::Vector3d& center,
                           const Eigen::Vector3d& size,
                           bool stop_at_unknown_voxel) const override;
  VoxelStatus getPathStatus(const Eigen::Vector3d& start,
                            const Eigen::Vector3d& end,
                            const Eigen::Vector3d& box_size,
                            bool stop_at_unknown_voxel) const override;
  VoxelStatus getOccupiedOnlyPathStatus(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end,
      const Eigen::Vector3d& box_size) const override;
  VoxelStatus getOccupiedOnlyCylinderPathStatus(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end, double radius,
      double height) const override;
  VoxelStatus getStrictBoxStatus(const Eigen::Vector3d& center,
                                 const Eigen::Vector3d& size) const override;
  VoxelStatus getStaticStrictBoxStatus(const Eigen::Vector3d& center,
                                       const Eigen::Vector3d& size) const override;
  VoxelStatus getStrictPathStatus(const Eigen::Vector3d& start,
                                  const Eigen::Vector3d& end,
                                  const Eigen::Vector3d& box_size) const override;
  VoxelStatus getStaticStrictPathStatus(const Eigen::Vector3d& start,
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
  void getScanStatusInBounds(
      const Eigen::Vector3d&, const std::vector<Eigen::Vector3d>&,
      GainCounts&, std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>&,
      const SensorModel&, const ScanBounds&) override;

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

  struct NoGoDiscs {
    std::vector<Eigen::Vector2d> centres;
    double radius_m = 0.0;
    std::chrono::steady_clock::time_point expires;
  };
  std::shared_ptr<const NoGoDiscs> no_go_discs_;
  std::shared_ptr<const NoGoZones> no_go_centre_line_discs_;
  bool discsBlockBox(const Eigen::Vector3d& center,
                     const Eigen::Vector3d& size) const;
  static bool discSetBlocksBox(
      const std::shared_ptr<const NoGoDiscs>& discs,
      const Eigen::Vector3d& center, const Eigen::Vector3d& size);
  static bool discSetBlocksSweep(
      const std::shared_ptr<const NoGoDiscs>& discs,
      const Eigen::Vector3d& start, const Eigen::Vector3d& end,
      double half_width);
  bool discsBlockSweep(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                       double half_width) const;
  mutable std::mutex error_mutex_;
  mutable std::string last_error_;
  mutable std::recursive_mutex publication_mutex_;
  mutable std::shared_ptr<const Snapshot> active_;
  mutable std::atomic<std::uint64_t> active_generation_{0};
  mutable std::atomic<std::uint64_t> expiry_count_{0};
  struct FootprintGroundSupport {
    MolaSnapshotRequest authority;
    Eigen::Vector3d center;
    Eigen::Vector2d size;
    double yaw;
  };
  std::shared_ptr<const FootprintGroundSupport> footprint_ground_;
  std::atomic<bool> grid_aligned_body_{false};
  /// One snapshot's grid-aligned strict box verdicts (aerial lattice cells
  /// and their nudges, asked again by every plan while the drone holds
  /// still), keyed by the box's component-frame centre and size in
  /// micrometres; emptied when the snapshot changes or it grows past a cap.
  struct BoxKey {
    std::int64_t c[3], s[3];
    bool operator==(const BoxKey& o) const {
      return c[0] == o.c[0] && c[1] == o.c[1] && c[2] == o.c[2] &&
             s[0] == o.s[0] && s[1] == o.s[1] && s[2] == o.s[2];
    }
  };
  struct BoxKeyHash {
    std::size_t operator()(const BoxKey& k) const {
      std::uint64_t h = 1469598103934665603ULL;
      for (std::int64_t v : {k.c[0], k.c[1], k.c[2], k.s[0], k.s[1], k.s[2]}) {
        h ^= std::uint64_t(v) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
      }
      return std::size_t(h);
    }
  };
  mutable std::mutex box_cache_mutex_;
  mutable std::weak_ptr<const Snapshot> box_cache_snapshot_;
  mutable std::unordered_map<BoxKey, VoxelStatus, BoxKeyHash> box_cache_;
  VoxelStatus gridStrictBoxStatus(const std::shared_ptr<const Snapshot>& snapshot,
                                  const Eigen::Vector3d& center,
                                  const Eigen::Vector3d& size) const;
  std::atomic<std::uint64_t> retained_predecessor_count_{0};
  std::atomic<std::uint64_t> stat_revalidation_count_{0};
  static thread_local std::vector<ThreadPin> thread_pins_;
};

}  // namespace mgg

#endif  // MGG_MAP_OCTOMAP_MOLA_MAP_H_
