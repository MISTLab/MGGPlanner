// The graph solution file the planner rebuilds its roadmap from: only the
// robot's own keyframes, home first and in the order they were taken.

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "mgg_ros/keyframe_trajectory.h"

namespace {

using mgg_ros::GraphSolutionFile;
using mgg_ros::KeyframeTrajectory;
using mgg_ros::parseGraphSolution;

std::string pose(const std::string& robot, const std::string& session,
                 int seq, double x, double y) {
  return R"({"keyframe_id": {"robot_id": ")" + robot +
         R"(", "session_id": ")" + session + R"(", "seq": )" +
         std::to_string(seq) + R"(}, "T_component_keyframe": [[0, -1, 0, )" +
         std::to_string(x) + "], [1, 0, 0, " + std::to_string(y) +
         "], [0, 0, 1, 0.1], [0, 0, 0, 1]]}";
}

std::string solution(const std::string& poses) {
  return R"({"schema": "swarmdeck.pose-snapshot.v1", "solution": {)"
         R"("schema": "swarmdeck.autonomy.v1", "revision": {)"
         R"("component_id": "component:abc", "epoch": 2, "revision": 731},)"
         R"("anchor": {}, "membership": [], "poses": [)" +
         poses + R"(], "retracted_constraints": [], "retracted_keyframes": []}})";
}

TEST(GraphSolution, TheRobotsOwnKeyframesHomeFirstInSeqOrder) {
  // Membership is sorted by robot, session and seq as text, and a merged
  // component holds other robots' keyframes too.
  const std::string text = solution(
      pose("robot_0", "s0", 0, 9.0, 9.0) + ", " +
      pose("robot_1", "s1", 10, 3.0, 0.0) + ", " +
      pose("robot_1", "s1", 2, 2.0, 0.0) + ", " +
      pose("robot_1", "s1", 0, 0.0, 0.0) + ", " +
      pose("robot_1", "s1", 1, 1.0, 0.0));
  KeyframeTrajectory trajectory;
  std::string error;
  ASSERT_TRUE(parseGraphSolution(text, "robot_1", trajectory, error)) << error;
  EXPECT_EQ(trajectory.component_id, "component:abc");
  EXPECT_EQ(trajectory.epoch, 2u);
  EXPECT_EQ(trajectory.revision, 731u);
  ASSERT_EQ(trajectory.poses.size(), 4u);
  EXPECT_DOUBLE_EQ(trajectory.poses[0].translation().x(), 0.0);
  EXPECT_DOUBLE_EQ(trajectory.poses[1].translation().x(), 1.0);
  EXPECT_DOUBLE_EQ(trajectory.poses[2].translation().x(), 2.0);
  EXPECT_DOUBLE_EQ(trajectory.poses[3].translation().x(), 3.0);
  EXPECT_DOUBLE_EQ(trajectory.poses[0].translation().z(), 0.1);
  // Rotated a quarter turn about z.
  EXPECT_NEAR(trajectory.poses[0].linear()(1, 0), 1.0, 1e-12);
}

TEST(GraphSolution, WithoutTheHomeKeyframeThereIsNoTrajectory) {
  KeyframeTrajectory trajectory;
  std::string error;
  EXPECT_FALSE(parseGraphSolution(
      solution(pose("robot_1", "s1", 1, 1.0, 0.0)), "robot_1", trajectory,
      error));
  EXPECT_NE(error.find("seq 0"), std::string::npos) << error;
  EXPECT_FALSE(parseGraphSolution(solution(pose("robot_0", "s0", 0, 0.0, 0.0)),
                                  "robot_1", trajectory, error));
}

TEST(GraphSolution, TwoSessionsOfTheRobotAreRefused) {
  KeyframeTrajectory trajectory;
  std::string error;
  EXPECT_FALSE(parseGraphSolution(
      solution(pose("robot_1", "s1", 0, 0.0, 0.0) + ", " +
               pose("robot_1", "s2", 0, 5.0, 0.0)),
      "robot_1", trajectory, error));
  EXPECT_NE(error.find("sessions"), std::string::npos) << error;
}

TEST(GraphSolution, MalformedDocumentsAreRefused) {
  KeyframeTrajectory trajectory;
  std::string error;
  EXPECT_FALSE(parseGraphSolution("{", "robot_1", trajectory, error));
  EXPECT_FALSE(parseGraphSolution(R"({"schema": "other"})", "robot_1",
                                  trajectory, error));
  EXPECT_FALSE(parseGraphSolution(
      R"({"schema": "swarmdeck.pose-snapshot.v1", "solution": {}})",
      "robot_1", trajectory, error));
  const std::string not_rigid = solution(
      R"({"keyframe_id": {"robot_id": "robot_1", "session_id": "s1", "seq": 0},)"
      R"( "T_component_keyframe": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0],)"
      R"( [0, 0, 1, 1]]})");
  EXPECT_FALSE(parseGraphSolution(not_rigid, "robot_1", trajectory, error));
  // Review r0, M-6: a rotation block that is not a rotation.
  const std::string sheared = solution(
      R"({"keyframe_id": {"robot_id": "robot_1", "session_id": "s1", "seq": 0},)"
      R"( "T_component_keyframe": [[1, 0.3, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0],)"
      R"( [0, 0, 0, 1]]})");
  EXPECT_FALSE(parseGraphSolution(sheared, "robot_1", trajectory, error));
  EXPECT_NE(error.find("rigid"), std::string::npos) << error;
}

TEST(GraphSolution, AKeyframeTwiceIsRefused) {
  // Review r0, M-6: a duplicate seq is not silently overwritten.
  KeyframeTrajectory trajectory;
  std::string error;
  EXPECT_FALSE(parseGraphSolution(
      solution(pose("robot_1", "s1", 0, 0.0, 0.0) + ", " +
               pose("robot_1", "s1", 1, 1.0, 0.0) + ", " +
               pose("robot_1", "s1", 1, 5.0, 0.0)),
      "robot_1", trajectory, error));
  EXPECT_NE(error.find("twice"), std::string::npos) << error;
}

TEST(GraphSolution, TheFileIsReadWholeAndBounded) {
  const std::string path =
      ::testing::TempDir() + "/graph_solution_test.json";
  {
    std::ofstream out(path);
    out << solution(pose("robot_1", "s1", 0, 0.0, 0.0));
  }
  KeyframeTrajectory trajectory;
  std::string error;
  GraphSolutionFile file(path, "robot_1", 1 << 20);
  ASSERT_TRUE(file.read(trajectory, error)) << error;
  EXPECT_EQ(trajectory.poses.size(), 1u);
  GraphSolutionFile small(path, "robot_1", 16);
  EXPECT_FALSE(small.read(trajectory, error));
  GraphSolutionFile missing(path + ".missing", "robot_1", 1 << 20);
  EXPECT_FALSE(missing.read(trajectory, error));
  std::remove(path.c_str());
}

}  // namespace
