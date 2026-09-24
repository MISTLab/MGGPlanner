// Robot geometry and bounded sampling/gain volumes.
//
// Ported from planner_common/params.h. Parameter *loading* is deliberately
// absent; it belongs at the ROS boundary. These are plain structs the host
// fills in.

#ifndef MGG_CORE_PARAMS_H_
#define MGG_CORE_PARAMS_H_

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

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
  /// Enlargement applied for exploration gain, active when setCenter is
  /// called with use_extension.
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
  /// Honours min_extension/max_extension (via setCenter's use_extension flag)
  /// for both kCuboid and kSphere. The ROS 1 version did so only for
  /// kSphere: for kCuboid it compared against min_val/max_val, and
  /// min_val_total/max_val_total were assigned in three places and read in
  /// none. Since every shipped space is kCuboid, the extensions were dead
  /// configuration.
  ///
  /// The shipped configs set the Local extensions to zero, so enabling this
  /// changes nothing until someone deliberately widens them. Note that
  /// GridGraphLocal's min_extension is NOT a bound extension: it carries the
  /// grid resolution (rrg.cpp assigns it to grid_graph_res_val_), which is
  /// why it is left non-zero.
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


enum class PlanningModeType {
  kBasicExploration = 0,      ///< bare-bones
  kNarrowEnvExploration = 1,  ///< tuned for narrow environments
  kAdaptiveExploration = 2,   ///< adapts the sampling volume to the geometry
};

enum class RRModeType {
  kGraph = 0,  ///< graph based search (default)
  kTree,       ///< tree based search
};

/// Everything the planner reads from configuration.
///
/// The ROS 1 struct had no constructor, so every field was indeterminate until
/// loadParams ran. The defaults below are the fallbacks loadParams itself
/// used, recovered from planner_common/src/params.cpp, so a default-constructed
/// PlanningParams behaves like a config file that specified nothing.
struct PlanningParams {
  uint32_t robot_id = 1;
  uint32_t sim = 0;

  std::string global_frame_id = "world";
  bool freespace_cloud_enable = false;

  // Robot dynamics.
  double v_max = 0.2;
  double v_homing_max = 0.2;
  double yaw_rate_max = 0.4;
  bool yaw_tangent_correction = false;

  // Graph building.
  PlanningModeType type = PlanningModeType::kBasicExploration;
  RRModeType rr_mode = RRModeType::kGraph;
  double edge_length_min = 0.2;
  double edge_length_max = 0.2;
  double edge_overshoot = 0.2;
  double num_vertices_max = 500;
  double num_edges_max = 5000;
  double num_loops_cutoff = 1000;
  double num_loops_max = 10000;
  double nearest_range = 1.0;
  double nearest_range_z = 1.0;
  double nearest_range_min = 0.5;
  double nearest_range_max = 2.0;
  bool build_grid_local_graph = false;
  bool use_current_state = true;
  bool geofence_checking_enable = false;

  // Ground robots.
  double max_ground_height = 1.2;
  double robot_height = 1.0;
  double max_inclination = 0.52;
  double max_step_height = 0.0;

  // Free-space augmentation (deprecated upstream).
  double augment_free_voxels_time = 5.0;
  bool augment_free_frustum_en = false;
  bool free_frustum_before_planning = false;

  // Exploration gain.
  std::vector<std::string> exp_sensor_list;
  std::vector<std::string> no_gain_zones_list;
  double exp_gain_voxel_size = 0.4;
  bool use_ray_model_for_volumetric_gain = false;
  double free_voxel_gain = 1.0;
  double occupied_voxel_gain = 1.0;
  double unknown_voxel_gain = 10.0;
  double path_length_penalty = 0.0;
  double path_direction_penalty = 0.0;
  double hanging_vertex_penalty = 0.0;
  bool leafs_only_for_volumetric_gain = false;
  bool cluster_vertices_for_gain = false;
  double clustering_radius = 2.0;
  double ray_cast_step_size_multiplier = 1.0;
  bool nonuniform_ray_cast = true;

  // Path generation and safety.
  double traverse_length_max = 0.2;
  double traverse_time_max = 1.0;
  bool planning_backward = false;
  bool path_safety_enhance_enable = false;
  double path_interpolation_distance = 0.5;
  /// Clearance beyond the robot's inscribed radius that an exploration
  /// path's final pose keeps from known obstacles (viewpointClear), metres.
  /// Not an upstream parameter.
  double viewpoint_clearance_margin = 0.1;

  // Global planner.
  double relaxed_corridor_multiplier = 1.0;
  bool auto_global_planner_enable = true;
  bool go_home_if_fully_explored = false;
  bool auto_homing_enable = false;
  bool homing_backward = false;
  double time_budget_limit = std::numeric_limits<double>::max();
  bool auto_landing_enable = false;
  double time_budget_before_landing = std::numeric_limits<double>::max();
  double max_negative_inclination = 0.37;
};

}  // namespace mgg

#endif  // MGG_CORE_PARAMS_H_
