#include "mgg_ros/param_loader.h"

#include <algorithm>

namespace mgg_ros {
namespace {

/// Maps the ROS 1 config spellings onto the mgg_core enums. Unknown values are
/// reported by the caller rather than silently defaulting, which is what the
/// ROS 1 ROSPARAM_ERROR path did.
bool parseRobotType(const std::string& s, mgg::RobotType& out) {
  if (s == "kAerialRobot") { out = mgg::RobotType::kAerialRobot; return true; }
  if (s == "kGroundRobot") { out = mgg::RobotType::kGroundRobot; return true; }
  return false;
}

bool parseBoundMode(const std::string& s, mgg::BoundModeType& out) {
  if (s == "kExtendedBound") { out = mgg::BoundModeType::kExtendedBound; return true; }
  if (s == "kRelaxedBound") { out = mgg::BoundModeType::kRelaxedBound; return true; }
  if (s == "kMinBound") { out = mgg::BoundModeType::kMinBound; return true; }
  if (s == "kExactBound") { out = mgg::BoundModeType::kExactBound; return true; }
  if (s == "kNoBound") { out = mgg::BoundModeType::kNoBound; return true; }
  return false;
}

bool parsePlanningMode(const std::string& s, mgg::PlanningModeType& out) {
  if (s == "kBasicExploration") { out = mgg::PlanningModeType::kBasicExploration; return true; }
  if (s == "kNarrowEnvExploration") { out = mgg::PlanningModeType::kNarrowEnvExploration; return true; }
  if (s == "kAdaptiveExploration") { out = mgg::PlanningModeType::kAdaptiveExploration; return true; }
  return false;
}

bool parseBoundedSpaceType(const std::string& s, mgg::BoundedSpaceType& out) {
  if (s == "kCuboid") { out = mgg::BoundedSpaceType::kCuboid; return true; }
  if (s == "kSphere") { out = mgg::BoundedSpaceType::kSphere; return true; }
  return false;
}

bool parseSensorType(const std::string& s, mgg::SensorType& out) {
  if (s == "kCamera") { out = mgg::SensorType::kCamera; return true; }
  if (s == "kLidar") { out = mgg::SensorType::kLidar; return true; }
  return false;
}

}  // namespace

std::string ParamLoader::toRos2(const std::string& ros1_name) {
  std::string out = ros1_name;
  std::replace(out.begin(), out.end(), '/', '.');
  // Tolerate the leading separator the ROS 1 namespaces sometimes carried.
  if (!out.empty() && out.front() == '.') out.erase(out.begin());
  return out;
}

bool ParamLoader::fetch(const std::string& ros1_name,
                        rclcpp::Parameter& out) const {
  const std::string name = toRos2(ros1_name);
  if (!node_->get_parameter(name, out) ||
      out.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    missing_.push_back(name);
    return false;
  }
  return true;
}

bool ParamLoader::get(const std::string& n, double& value) const {
  rclcpp::Parameter p;
  if (!fetch(n, p)) return false;
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    value = static_cast<double>(p.as_int());
    return true;
  }
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    // Most likely an unevaluated rad()/deg() expression: ROS 1's rosparam
    // evaluated those while loading, ROS 2 does not.
    RCLCPP_ERROR(node_->get_logger(),
                 "parameter '%s' is not a number (run the config through "
                 "tools/convert_config.py?)", toRos2(n).c_str());
    return false;
  }
  value = p.as_double();
  return true;
}

bool ParamLoader::get(const std::string& n, int& value) const {
  rclcpp::Parameter p;
  if (!fetch(n, p)) return false;
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) return false;
  value = static_cast<int>(p.as_int());
  return true;
}

bool ParamLoader::get(const std::string& n, bool& value) const {
  rclcpp::Parameter p;
  if (!fetch(n, p)) return false;
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) return false;
  value = p.as_bool();
  return true;
}

bool ParamLoader::get(const std::string& n, std::string& value) const {
  rclcpp::Parameter p;
  if (!fetch(n, p)) return false;
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) return false;
  value = p.as_string();
  return true;
}

bool ParamLoader::get(const std::string& n, std::vector<double>& value) const {
  rclcpp::Parameter p;
  if (!fetch(n, p)) return false;
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    value = p.as_double_array();
    return true;
  }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    // A YAML list of whole numbers arrives as integers.
    value.clear();
    for (int64_t v : p.as_integer_array()) value.push_back(static_cast<double>(v));
    return true;
  }
  RCLCPP_ERROR(node_->get_logger(),
               "parameter '%s' is not a numeric array (run the config through "
               "tools/convert_config.py?)", toRos2(n).c_str());
  return false;
}

bool ParamLoader::get(const std::string& n,
                      std::vector<std::string>& value) const {
  rclcpp::Parameter p;
  if (!fetch(n, p)) return false;
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY) return false;
  value = p.as_string_array();
  return true;
}

bool ParamLoader::get(const std::string& n, Eigen::Vector2d& value) const {
  std::vector<double> v;
  if (!get(n, v) || v.size() != 2) return false;
  value << v[0], v[1];
  return true;
}

bool ParamLoader::get(const std::string& n, Eigen::Vector3d& value) const {
  std::vector<double> v;
  if (!get(n, v) || v.size() != 3) return false;
  value << v[0], v[1], v[2];
  return true;
}

bool loadRobotParams(const ParamLoader& p, const std::string& ns,
                     mgg::RobotParams& out) {
  std::string s;
  if (p.get(ns + "/type", s) && !parseRobotType(s, out.type)) return false;
  p.get(ns + "/size", out.size);
  p.get(ns + "/size_extension_min", out.size_extension_min);
  p.get(ns + "/size_extension", out.size_extension);
  p.get(ns + "/center_offset", out.center_offset);
  p.get(ns + "/relax_ratio", out.relax_ratio);
  p.get(ns + "/safety_extension", out.safety_extension);
  if (p.get(ns + "/bound_mode", s) && !parseBoundMode(s, out.bound_mode)) {
    return false;
  }
  return true;
}

bool loadBoundedSpace(const ParamLoader& p, const std::string& ns,
                      mgg::BoundedSpaceParams& out) {
  std::string s;
  if (p.get(ns + "/type", s) && !parseBoundedSpaceType(s, out.type)) {
    return false;
  }
  p.get(ns + "/min_val", out.min_val);
  p.get(ns + "/max_val", out.max_val);
  p.get(ns + "/min_extension", out.min_extension);
  p.get(ns + "/max_extension", out.max_extension);
  p.get(ns + "/rotations", out.rotations);
  p.get(ns + "/radius", out.radius);
  p.get(ns + "/radius_extension", out.radius_extension);
  out.setCenter(Eigen::Vector3d(0.0, 0.0, 0.0), false);
  return true;
}

bool loadGridGraphParams(const ParamLoader& p, const std::string& ns,
                         mgg::GridGraphParams& out) {
  p.get(ns + "/min_val", out.min_val);
  p.get(ns + "/max_val", out.max_val);
  // Named `resolution` here. The ROS 1 config carried it in `min_extension`,
  // a bounds field overloaded to mean something else;
  // tools/convert_config.py renames it.
  return p.get(ns + "/resolution", out.resolution);
}

bool loadSensorParams(const ParamLoader& p, const std::string& ns,
                      mgg::SensorParams& out) {
  std::string s;
  if (!p.get(ns + "/type", s) || !parseSensorType(s, out.type)) return false;
  p.get(ns + "/min_range", out.min_range);
  p.get(ns + "/max_range", out.max_range);
  p.get(ns + "/center_offset", out.center_offset);
  p.get(ns + "/rotations", out.rotations);
  p.get(ns + "/fov", out.fov);
  p.get(ns + "/resolution", out.resolution);
  p.get(ns + "/width", out.width);
  p.get(ns + "/height", out.height);
  p.get(ns + "/frontier_percentage_threshold",
        out.frontier_percentage_threshold);
  // Must run: it builds the ray table and the rotations. Skipping it is what
  // left the ROS 1 struct with an uninitialised rot_B2S.
  out.update();
  return true;
}

bool loadSensorSet(const ParamLoader& p, const std::string& ns,
                   std::unordered_map<std::string, mgg::SensorParams>& out) {
  std::vector<std::string> names;
  if (!p.get(ns + "/sensor_list", names)) return false;
  for (const std::string& name : names) {
    mgg::SensorParams sensor;
    if (!loadSensorParams(p, ns + "/" + name, sensor)) return false;
    out[name] = sensor;
  }
  return true;
}

bool loadPlanningParams(const ParamLoader& p, const std::string& ns,
                        mgg::PlanningParams& out) {
  std::string s;
  if (p.get(ns + "/type", s) && !parsePlanningMode(s, out.type)) return false;

  int robot_id = static_cast<int>(out.robot_id);
  if (p.get(ns + "/robot_id", robot_id)) out.robot_id = robot_id;
  int sim = static_cast<int>(out.sim);
  if (p.get(ns + "/sim", sim)) out.sim = sim;

  p.get(ns + "/global_frame_id", out.global_frame_id);
  p.get(ns + "/freespace_cloud_enable", out.freespace_cloud_enable);
  p.get(ns + "/v_max", out.v_max);
  p.get(ns + "/v_homing_max", out.v_homing_max);
  p.get(ns + "/yaw_rate_max", out.yaw_rate_max);
  p.get(ns + "/yaw_tangent_correction", out.yaw_tangent_correction);
  p.get(ns + "/edge_length_min", out.edge_length_min);
  p.get(ns + "/edge_length_max", out.edge_length_max);
  p.get(ns + "/edge_overshoot", out.edge_overshoot);
  p.get(ns + "/num_vertices_max", out.num_vertices_max);
  p.get(ns + "/num_edges_max", out.num_edges_max);
  p.get(ns + "/num_loops_cutoff", out.num_loops_cutoff);
  p.get(ns + "/num_loops_max", out.num_loops_max);
  p.get(ns + "/nearest_range", out.nearest_range);
  p.get(ns + "/nearest_range_z", out.nearest_range_z);
  p.get(ns + "/nearest_range_min", out.nearest_range_min);
  p.get(ns + "/nearest_range_max", out.nearest_range_max);
  p.get(ns + "/build_grid_local_graph", out.build_grid_local_graph);
  p.get(ns + "/use_current_state", out.use_current_state);
  p.get(ns + "/geofence_checking_enable", out.geofence_checking_enable);
  p.get(ns + "/max_ground_height", out.max_ground_height);
  p.get(ns + "/robot_height", out.robot_height);
  p.get(ns + "/max_inclination", out.max_inclination);
  p.get(ns + "/max_cross_slope", out.max_cross_slope);
  p.get(ns + "/max_footprint_tilt", out.max_footprint_tilt);
  p.get(ns + "/max_footprint_step", out.max_footprint_step);
  p.get(ns + "/max_goal_ground_rise", out.max_goal_ground_rise);
  p.get(ns + "/exp_sensor_list", out.exp_sensor_list);
  p.get(ns + "/no_gain_zones_list", out.no_gain_zones_list);
  p.get(ns + "/exp_gain_voxel_size", out.exp_gain_voxel_size);
  p.get(ns + "/use_ray_model_for_volumetric_gain",
        out.use_ray_model_for_volumetric_gain);
  p.get(ns + "/free_voxel_gain", out.free_voxel_gain);
  p.get(ns + "/occupied_voxel_gain", out.occupied_voxel_gain);
  p.get(ns + "/unknown_voxel_gain", out.unknown_voxel_gain);
  p.get(ns + "/path_length_penalty", out.path_length_penalty);
  p.get(ns + "/path_direction_penalty", out.path_direction_penalty);
  p.get(ns + "/hanging_vertex_penalty", out.hanging_vertex_penalty);
  p.get(ns + "/leafs_only_for_volumetric_gain",
        out.leafs_only_for_volumetric_gain);
  p.get(ns + "/cluster_vertices_for_gain", out.cluster_vertices_for_gain);
  p.get(ns + "/clustering_radius", out.clustering_radius);
  p.get(ns + "/ray_cast_step_size_multiplier",
        out.ray_cast_step_size_multiplier);
  p.get(ns + "/nonuniform_ray_cast", out.nonuniform_ray_cast);
  p.get(ns + "/traverse_length_max", out.traverse_length_max);
  p.get(ns + "/traverse_time_max", out.traverse_time_max);
  p.get(ns + "/planning_backward", out.planning_backward);
  p.get(ns + "/path_safety_enhance_enable", out.path_safety_enhance_enable);
  p.get(ns + "/path_interpolation_distance", out.path_interpolation_distance);
  p.get(ns + "/viewpoint_clearance_margin", out.viewpoint_clearance_margin);
  p.get(ns + "/departure_reverse_allowed", out.departure_reverse_allowed);
  p.get(ns + "/relaxed_corridor_multiplier", out.relaxed_corridor_multiplier);
  p.get(ns + "/auto_global_planner_enable", out.auto_global_planner_enable);
  p.get(ns + "/go_home_if_fully_explored", out.go_home_if_fully_explored);
  p.get(ns + "/auto_homing_enable", out.auto_homing_enable);
  p.get(ns + "/homing_backward", out.homing_backward);
  p.get(ns + "/time_budget_limit", out.time_budget_limit);
  p.get(ns + "/auto_landing_enable", out.auto_landing_enable);
  p.get(ns + "/time_budget_before_landing", out.time_budget_before_landing);
  p.get(ns + "/max_negative_inclination", out.max_negative_inclination);
  return true;
}

}  // namespace mgg_ros
