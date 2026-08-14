#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mgg_pci/pci_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mgg_pci::PciNode>(rclcpp::NodeOptions());

  // MultiThreadedExecutor is required, not preferred. PciNode calls the
  // planner's service from inside its own service callback and blocks on the
  // response. With one executor thread that thread is already inside the
  // calling callback, so the response is never delivered and the call times
  // out. See test_pci_deadlock.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
