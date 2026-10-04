// Incrementally certified 2.5D terrain over a rolling MapInterface window.
#ifndef MGG_CORE_GROUND_LAYER_H_
#define MGG_CORE_GROUND_LAYER_H_

#include <chrono>
#include <cstdint>
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
  void recheck(std::chrono::steady_clock::time_point deadline);
  /// Pending means work since the last change, not completed unknown space.
  int pendingCount() const;
  bool pending(const Eigen::AlignedBox3d& region) const;
  GroundVerdict verdict(const Eigen::Vector2d& position) const;
  /// Row-major, x fastest: admitted 0, refused 100, unknown/pending -1.
  std::vector<int8_t> occupancy() const;
  Eigen::Vector2d origin() const { return origin_; }

 private:
  struct Column {
    GroundVerdict verdict = GroundVerdict::kUnknown;
    bool dirty = true;
    bool observed = false;
    bool evaluated = false;  // clean but unreached columns may join a later flood
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
  int width_ = 0, height_ = 0, seed_ = -1;
  std::vector<Column> columns_;
};

}  // namespace mgg
#endif  // MGG_CORE_GROUND_LAYER_H_
