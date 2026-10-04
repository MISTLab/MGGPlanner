#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mgg_ros/local_planner_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  // The planner's parameter file is nested and long; read what it supplies,
  // as mggplanner_node does, and fall back to struct defaults.
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<mgg::LocalPlannerNode>(options);
  // One thread: the node's callbacks are serialized anyway (one core).
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
