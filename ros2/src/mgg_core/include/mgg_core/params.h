// Robot geometry and bounded sampling/gain volumes.
//
// Ported from planner_common/params.h. Parameter *loading* is deliberately
// absent; it belongs at the ROS boundary. These are plain structs the host
// fills in.

#ifndef MGG_CORE_PARAMS_H_
#define MGG_CORE_PARAMS_H_

#include <Eigen/Dense>

#include "mgg_core/types.h"

namespace mgg {

enum class RobotType { kAerialRobot = 0, kGroundRobot };

/// How much clearance the planner demands around the robot's actual size.
enum class BoundModeType {
  kExtendedBound = 0,  ///< size + full extension
  kRelaxedBound,       ///< size + a blend of min and full extension
  kMinBound,           ///< size + minimum extension
  kExactBound,         ///< the robot's actual size
  kNoBound,            ///< no bound at all
};

struct RobotParams {
  RobotType type = RobotType::kGroundRobot;
  /// Actual size, length(x) by width(y) by height(z), metres.
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  /// Least extension the planner will accept.
  Eigen::Vector3d size_extension_min = Eigen::Vector3d::Zero();
  /// Recommended extension; must be at least size_extension_min.
  Eigen::Vector3d size_extension = Eigen::Vector3d::Zero();
  /// Cuboid centre = state + center_offset.
  Eigen::Vector3d center_offset = Eigen::Vector3d::Zero();
  /// Blend factor in [0,1] for kRelaxedBound.
  double relax_ratio = 0.5;
  BoundModeType bound_mode = BoundModeType::kExtendedBound;
  Eigen::Vector3d safety_extension = Eigen::Vector3d::Zero();

  /// Planning footprint implied by bound_mode.
  Eigen::Vector3d getPlanningSize() const;
};

enum class BoundedSpaceType { kCuboid = 0, kSphere };

/// A volume the planner samples in, optionally enlarged for gain evaluation.
class BoundedSpaceParams {
 public:
  BoundedSpaceType type = BoundedSpaceType::kCuboid;
  /// Vertex sampling space, metres.
  Eigen::Vector3d min_val = Eigen::Vector3d::Zero();
  Eigen::Vector3d max_val = Eigen::Vector3d::Zero();
  /// Enlargement applied for exploration gain. See the caveat on
  /// isInsideSpace: for kCuboid these are currently inert.
  Eigen::Vector3d min_extension = Eigen::Vector3d::Zero();
  Eigen::Vector3d max_extension = Eigen::Vector3d::Zero();
  /// [yaw, pitch, roll] of the volume relative to world.
  Eigen::Vector3d rotations = Eigen::Vector3d::Zero();
  /// Sphere only.
  double radius = 0.0;
  double radius_extension = 0.0;

  void setCenter(const StateVec& state, bool use_extension);
  void setCenter(const Eigen::Vector3d& root, bool use_extension);
  void setBound(const Eigen::Vector3d& min_in, const Eigen::Vector3d& max_in);
  void setRotation(const Eigen::Vector3d& rotations_in);

  Eigen::Vector3d getCenter() const { return root_pos_; }
  Eigen::Matrix3d getRotationMatrix() const { return rot_b2w_; }

  /// Is `pos` (world frame) inside the volume?
  ///
  /// KNOWN LIMITATION, carried over deliberately. For kCuboid this compares
  /// against min_val/max_val, NOT against the extended bounds, so
  /// min_extension and max_extension have no effect. In the ROS 1 code
  /// min_val_total and max_val_total were assigned in three places and never
  /// read; only radius_total, on the sphere path, was ever used.
  ///
  /// This is live in the shipped configuration: BoundedSpaceParams/Local sets
  /// min_extension [-20,-20,-20] and max_extension [20,20,20], intending gain
  /// to be evaluated over +/-35 m while sampling stays within +/-15 m. Those
  /// values are silently discarded, and every shipped space is kCuboid.
  ///
  /// Honouring them would enlarge the gain volume substantially and change
  /// exploration behaviour, so it is not done unilaterally. See
  /// ROS2_PORT_PLAN.md section 4.3.
  bool isInsideSpace(const Eigen::Vector3d& pos) const;

  /// The extended bounds, exposed so the fix above can be evaluated without
  /// resurrecting dead members.
  Eigen::Vector3d minValTotal() const { return min_val_total_; }
  Eigen::Vector3d maxValTotal() const { return max_val_total_; }
  double radiusTotal() const { return radius_total_; }

 private:
  void updateRotation();

  Eigen::Vector3d root_pos_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rot_b2w_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d min_val_total_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d max_val_total_ = Eigen::Vector3d::Zero();
  double radius_total_ = 0.0;
};

}  // namespace mgg

#endif  // MGG_CORE_PARAMS_H_
