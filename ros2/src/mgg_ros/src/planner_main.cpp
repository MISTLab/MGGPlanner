#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mgg_ros/planner_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  // The parameter names are nested (PlanningParams.edge_length_max and so on)
  // and there are well over a hundred. Declaring each by hand would be pure
  // boilerplate; this makes everything the YAML supplies readable, and the
  // loader falls back to the struct defaults for anything absent.
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<mgg_ros::PlannerNode>(options);

  // MultiThreadedExecutor, deliberately.
  //
  // In ROS 1 the control interface called the planner's service synchronously
  // from inside its own callback. That pattern deadlocks on a
  // SingleThreadedExecutor: the one executor thread is already inside the
  // calling callback and can never deliver the response. Section 6 of
  // ROS2_PORT_PLAN.md calls for deciding this up front rather than
  // discovering it as a hang, so the node's callbacks live in a reentrant
  // group and the executor has threads to run them.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
