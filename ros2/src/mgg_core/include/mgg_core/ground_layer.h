// Incrementally certified 2.5D terrain over a rolling MapInterface window.
#ifndef MGG_CORE_GROUND_LAYER_H_
#define MGG_CORE_GROUND_LAYER_H_

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

#include "mgg_core/dirty_region.h"
#include "mgg_core/ground_projection.h"

namespace mgg {

enum class GroundVerdict {
  kUnknown,
  kPending,
  kAdmitted,
  kRefusedStepGrade,
  kRefusedOverhang
};

struct GroundLayerParams {
  /// Used only by maps without windowBounds(); resolution still follows map.
  Eigen::Vector2d window_size_m{16, 16};
};

class GroundLayer {
 public:
  GroundLayer(const MapInterface& map, const PlanningParams& planning,
              const RobotParams& robot,
              const GroundLayerParams& params = GroundLayerParams{});
  /// robot is the driving anchor: the seed floor hint is its z minus
  /// max_ground_height. ground_z only initializes unseen dependency boxes;
  /// it does not substitute for observed support or override that hint.
  void reset(const Eigen::Vector3d& robot, double ground_z);
  /// Refreshes an unsupported seed from corrected odometry as well as
  /// scrolling the grid. ground_z has the same meaning as in reset().
  void recenter(const Eigen::Vector3d& robot, double ground_z);
  void withdraw(const MapChange& change);
  /// The robot stands in the disk where it started (StandingStart): its
  /// lidar cannot see the ground near itself. A disk column with no ground
  /// found under it then counts as supported at the seed floor hint; ground
  /// observed in the disk is used as observed, so a drop stays refused, and
  /// so does a column the lidar looked into (free kGroundBridgeHoleDepth
  /// below the hint). In a column wholly in the disk an unknown body band
  /// passes; occupied still refuses. Elsewhere unknown stays unknown, in
  /// the body band of a column reaching past the disk's edge too. A change
  /// withdraws the columns the two disks read differently, and every
  /// column whose admission rests on one of them.
  void setStandingStart(const std::optional<StandingStart>& standing);
  /// Rechecks dirty columns, nearest the robot first, except that columns
  /// whose dependency reaches one of `first` (an executing commitment's
  /// dependencies) come before all others.
  void recheck(std::chrono::steady_clock::time_point deadline,
               const std::vector<Eigen::AlignedBox3d>& first = {});
  /// Pending means work since the last change, not completed unknown space.
  int pendingCount() const;
  bool pending(const Eigen::AlignedBox3d& region) const;
  GroundVerdict verdict(const Eigen::Vector2d& position) const;
  /// Whether a footprint of `size` centred at `at`, facing `heading`,
  /// overlaps a column exported as refused (occupancy 100): Nav2's costmap
  /// would put that footprint on lethal terrain.
  bool refusedUnder(const Eigen::Vector2d& at, double heading,
                    const Eigen::Vector2d& size) const;
  /// Row-major, x fastest: admitted 0, refused 100, unknown/pending -1.
  std::vector<int8_t> occupancy() const;
  Eigen::Vector2d origin() const { return origin_; }

 private:
  struct Column {
    GroundVerdict verdict = GroundVerdict::kUnknown;
    bool dirty = true;
    bool observed = false;
    bool evaluated = false;  // clean but unreached columns may join a later flood
    bool standing = false;   // supported only by the standing start
    double ground_z = 0;
    int parent = -1;
    Eigen::AlignedBox3d dependency;
  };
  int index(const Eigen::Vector2d& position) const;
  Eigen::Vector2d center(int index) const;
  Eigen::AlignedBox3d dependency(int index, double ground_z) const;
  void place(const Eigen::Vector3d& robot, double ground_z, bool clear);
  void refreshUnsupportedSeed();
  void markPending(int index);
  void withdrawDescendants();
  const MapInterface& map_;
  const PlanningParams planning_;
  const double projection_length_;  // one depth for rays and their dependencies
  const RobotParams robot_params_;
  const GroundLayerParams params_;
  const double resolution_;
  Eigen::Vector2d origin_ = Eigen::Vector2d::Zero();
  Eigen::Vector3d robot_ = Eigen::Vector3d::Zero();
  double seed_ground_hint_ = 0;
  std::optional<StandingStart> standing_;
  int width_ = 0, height_ = 0, seed_ = -1;
  std::vector<Column> columns_;
};

}  // namespace mgg
#endif  // MGG_CORE_GROUND_LAYER_H_
