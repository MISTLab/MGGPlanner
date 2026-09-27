// Which map backends the planner node accepts. mola_snapshot is always built;
// cloud_octomap only when mgg_map_octomap has OctoMap (MGG_WITH_OCTOMAP), and
// asking for it otherwise fails at construction with a message saying why.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "mgg_ros/planner_node.h"

namespace mgg_ros {

class PlannerNodeTestPeer {
 public:
  /// The graph solution file rebuilds read, or null when they are off.
  static GraphSolutionFile* keyframeSource(PlannerNode& node) {
    return dynamic_cast<GraphSolutionFile*>(node.keyframe_source_.get());
  }
};

namespace {

std::shared_ptr<PlannerNode> makeNode(
    const std::string& name, const std::string& backend,
    const std::string& peer_root = "/nonexistent/mgg_peer_root",
    std::vector<rclcpp::Parameter> extra = {}) {
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=" + name});
  // MolaMap only needs its product directory to be absolute until a
  // snapshot arrives.
  std::vector<rclcpp::Parameter> parameters{
      rclcpp::Parameter("map.backend", backend),
      rclcpp::Parameter("map.mola.peer_root", peer_root)};
  parameters.insert(parameters.end(), extra.begin(), extra.end());
  options.parameter_overrides(parameters);
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

TEST_F(MapBackendTest, RebuildsReadTheRobotsGraphSolutionInItsPeerRoot) {
  // Review r0, M-2: a trailing separator on the peer root still names the
  // robot.
  for (const std::string root :
       {"/nonexistent/mission/robot_1", "/nonexistent/mission/robot_1/"}) {
    SCOPED_TRACE(root);
    auto node = makeNode("mola_rebuild_source", "mola_snapshot", root);
    const GraphSolutionFile* source =
        PlannerNodeTestPeer::keyframeSource(*node);
    ASSERT_NE(source, nullptr);
    EXPECT_EQ(source->robotId(), "robot_1");
    EXPECT_EQ(source->path(),
              "/nonexistent/mission/robot_1/graph_solution.json");
  }
}

TEST_F(MapBackendTest, RebuildsReadTheGraphSolutionAndRobotTheyAreGiven) {
  // Runs 9 and 10: SwarmDeck pointed map.mola.peer_root at the robot's fast
  // planning product, <peer>/planning, while the bridge writes the graph
  // solution to <peer>/graph_solution.json. Derived from the peer root, the
  // planner read <peer>/planning/graph_solution.json as robot "planning",
  // never found its keyframes, and had no standing start: every robot was
  // boxed in where it was placed. Given explicitly, both are used as given.
  const std::filesystem::path peer =
      std::filesystem::temp_directory_path() / "mgg_explicit_graph_solution" /
      "robot_1";
  std::filesystem::create_directories(peer / "planning");
  std::ofstream(peer / "graph_solution.json")
      << R"({"schema": "swarmdeck.pose-snapshot.v1", "solution": {
        "revision": {"component_id": "component:c", "epoch": 1,
                     "revision": 4},
        "poses": [{"keyframe_id": {"robot_id": "robot_1",
                                   "session_id": "s", "seq": 0},
                   "T_component_keyframe": [[1, 0, 0, 0], [0, 1, 0, -2],
                                            [0, 0, 1, 0], [0, 0, 0, 1]]}]}})";
  auto node = makeNode(
      "mola_explicit_rebuild_source", "mola_snapshot",
      (peer / "planning").string(),
      {rclcpp::Parameter("roadmap_rebuild.graph_solution",
                         (peer / "graph_solution.json").string()),
       rclcpp::Parameter("roadmap_rebuild.robot_id", "robot_1")});
  GraphSolutionFile* source = PlannerNodeTestPeer::keyframeSource(*node);
  ASSERT_NE(source, nullptr);
  EXPECT_EQ(source->robotId(), "robot_1");
  EXPECT_EQ(source->path(), (peer / "graph_solution.json").string());
  KeyframeTrajectory trajectory;
  std::string error;
  EXPECT_TRUE(source->read(trajectory, error)) << error;
  ASSERT_EQ(trajectory.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(trajectory.poses.front().translation().y(), -2.0);
  std::filesystem::remove_all(peer.parent_path());
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
