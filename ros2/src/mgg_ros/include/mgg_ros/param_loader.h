// Filling mgg_core's plain parameter structs from ROS 2 parameters.
//
// This is the whole of what ROS 1's *Params::loadParams methods did, moved to
// the ROS boundary so mgg_core stays free of ROS.
//
// The differences that matter:
//
//   * ROS 1 used '/' to separate nested parameter names and read them from a
//     global parameter server with ros::param::get. ROS 2 uses '.' and reads
//     from the node's own parameters, so every name is translated.
//   * ROS 2 requires parameters to be declared before they can be read. The
//     node is constructed with automatically_declare_parameters_from_overrides,
//     so anything present in the YAML is declared for us; anything absent
//     falls back to the struct's default, which is the behaviour the ROS 1
//     ROSPARAM_WARN path had.
//   * ROS 1's rosparam evaluated rad()/deg() expressions while loading YAML.
//     ROS 2 does not, so configs must be passed through
//     tools/convert_config.py first. A string where a double is expected is
//     reported rather than silently treated as zero.

#ifndef MGG_ROS_PARAM_LOADER_H_
#define MGG_ROS_PARAM_LOADER_H_

#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include "mgg_core/geofence_manager.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/params.h"
#include "mgg_core/sensor_params.h"

namespace mgg_ros {

/// Declares `name` only if it is not declared already, then returns its value.
///
/// Necessary because the node uses
/// automatically_declare_parameters_from_overrides: any parameter the user
/// actually supplies is already declared by the time the constructor runs, and
/// calling declare_parameter on it again throws
/// ParameterAlreadyDeclaredException. A plain declare_parameter therefore works
/// while a setting is left at its default and crashes as soon as somebody sets
/// it, which is exactly backwards.
template <typename T>
T declareOrGet(rclcpp::Node* node, const std::string& name,
               const T& default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

/// Reads nested parameters using the ROS 1 '/' spelling, so ported call sites
/// keep their original names.
class ParamLoader {
 public:
  explicit ParamLoader(rclcpp::Node* node) : node_(node) {}

  /// True when the parameter existed. `value` is untouched otherwise, leaving
  /// the caller's default in place.
  bool get(const std::string& ros1_name, double& value) const;
  bool get(const std::string& ros1_name, int& value) const;
  bool get(const std::string& ros1_name, bool& value) const;
  bool get(const std::string& ros1_name, std::string& value) const;
  bool get(const std::string& ros1_name, std::vector<double>& value) const;
  bool get(const std::string& ros1_name,
           std::vector<std::string>& value) const;
  bool get(const std::string& ros1_name, Eigen::Vector2d& value) const;
  bool get(const std::string& ros1_name, Eigen::Vector3d& value) const;

  /// Names that were requested but not found. Useful in bring-up, where a
  /// mistyped section silently leaves every field at its default.
  const std::vector<std::string>& missing() const { return missing_; }

 private:
  static std::string toRos2(const std::string& ros1_name);
  bool fetch(const std::string& ros1_name, rclcpp::Parameter& out) const;

  rclcpp::Node* node_;
  mutable std::vector<std::string> missing_;
};

bool loadRobotParams(const ParamLoader& p, const std::string& ns,
                     mgg::RobotParams& out);
bool loadPlanningParams(const ParamLoader& p, const std::string& ns,
                        mgg::PlanningParams& out);
bool loadBoundedSpace(const ParamLoader& p, const std::string& ns,
                      mgg::BoundedSpaceParams& out);
bool loadGridGraphParams(const ParamLoader& p, const std::string& ns,
                         mgg::GridGraphParams& out);
bool loadSensorParams(const ParamLoader& p, const std::string& ns,
                      mgg::SensorParams& out);
/// Loads every sensor named in `<ns>/sensor_list`.
bool loadSensorSet(const ParamLoader& p, const std::string& ns,
                   std::unordered_map<std::string, mgg::SensorParams>& out);

}  // namespace mgg_ros

#endif  // MGG_ROS_PARAM_LOADER_H_
