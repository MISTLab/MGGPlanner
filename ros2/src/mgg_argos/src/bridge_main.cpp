#include "mgg_argos/bridge_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // The socket loop runs on its own thread and publishes from there while
  // path subscriptions fire on the executor, so the executor has to be able
  // to run more than one callback at a time.
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<mgg_argos::BridgeNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
