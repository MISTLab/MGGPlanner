// Which map backends the planner node accepts. mola_snapshot is always built;
// cloud_octomap only when mgg_map_octomap has OctoMap (MGG_WITH_OCTOMAP), and
// asking for it otherwise fails at construction with a message saying why.

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "mgg_ros/planner_node.h"

namespace mgg_ros {
namespace {

std::shared_ptr<PlannerNode> makeNode(const std::string& name,
                                      const std::string& backend) {
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=" + name});
  // MolaMap only needs its product directory to be absolute until a
  // snapshot arrives.
  options.parameter_overrides(
      {rclcpp::Parameter("map.backend", backend),
       rclcpp::Parameter("map.mola.peer_root", "/nonexistent/mgg_peer_root")});
  options.automatically_declare_parameters_from_overrides(true);
  return std::make_shared<PlannerNode>(options);
}

class MapBackendTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(MapBackendTest, MolaSnapshotIsAlwaysAvailable) {
  EXPECT_NO_THROW(makeNode("mola_backend", "mola_snapshot"));
}

TEST_F(MapBackendTest, AnUnknownBackendIsRefused) {
  EXPECT_THROW(makeNode("unknown_backend", "voxblox"), std::invalid_argument);
}

#ifdef MGG_WITH_OCTOMAP
TEST_F(MapBackendTest, CloudOctomapIsAvailableWithOctomap) {
  EXPECT_NO_THROW(makeNode("octomap_backend", "cloud_octomap"));
}
#else
TEST_F(MapBackendTest, CloudOctomapIsRefusedWithoutOctomap) {
  try {
    makeNode("octomap_backend", "cloud_octomap");
    FAIL() << "cloud_octomap was accepted without OctoMap";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("MGG_WITH_OCTOMAP=OFF"),
              std::string::npos)
        << e.what();
  }
}
#endif

}  // namespace
}  // namespace mgg_ros
