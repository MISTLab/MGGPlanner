// C4a payloads of the local planner node's native events: p0's required
// fields are present (path_length_m null without a path), outcomes map
// from the cycle's status, and the extras sit beside them.

#include <initializer_list>
#include <string>

#include <gtest/gtest.h>

#include "mgg_ros/local_events.h"

using namespace mgg;

namespace {

// p0's per-kind required fields (adapters/exploration_telemetry.py), plus
// the session every planner event carries.
void expectFields(const nlohmann::json& j,
                  std::initializer_list<const char*> keys) {
  EXPECT_TRUE(j.contains("session_id")) << j.dump();
  for (const char* key : keys) EXPECT_TRUE(j.contains(key)) << key;
}

LocalPlanResult withStatus(LocalStatus status) {
  LocalPlanResult result;
  result.status = status;
  result.reason = "why";
  result.cycle_ms = 3.5;
  return result;
}

}  // namespace

TEST(LocalEvents, RequiredFields) {
  const auto j =
      localPlanPayload(withStatus(LocalStatus::kWaitingForMap), "s", "r", 7);
  expectFields(j, {"request_id", "duration_ms", "outcome", "path_length_m",
                   "map_revision"});
  EXPECT_TRUE(j.contains("path_length_m") && j["path_length_m"].is_null());
  EXPECT_EQ(j["outcome"], "waiting_for_map");
  EXPECT_EQ(j["session_id"], "s");
  EXPECT_EQ(j["request_id"], "r");
  EXPECT_EQ(j["map_revision"], 7u);
  EXPECT_DOUBLE_EQ(j["duration_ms"].get<double>(), 3.5);
  EXPECT_FALSE(j.contains("frontier_count"));
  EXPECT_EQ(
      localPlanPayload(withStatus(LocalStatus::kBlocked), "s", "r", 7)["outcome"],
      "blocked");
  EXPECT_EQ(localPlanPayload(withStatus(LocalStatus::kNoLocalTarget), "s", "r",
                             7)["outcome"],
            "no_path");
  EXPECT_EQ(
      localPlanPayload(withStatus(LocalStatus::kMoving), "s", "r", 7)["outcome"],
      "no_path");

  // A splice keeping two poses, then a 4 m reverse leg.
  auto moving = withStatus(LocalStatus::kMoving);
  LocalPathPlan path;
  path.epoch = 9;
  path.sequence_id = 4;
  path.kind = LocalPathKind::kExtend;
  path.prefix_length = 2;
  path.poses = {StateVec(0, 0, 0, 0), StateVec(3, 0, 0, 0),
                StateVec(3, 4, 0, 0)};
  path.reverse = {false, true, true};
  moving.path = path;
  moving.checks_complete = true;
  moving.candidate_count = 12;
  const auto p = localPlanPayload(moving, "s", "r", 8);
  EXPECT_EQ(p["outcome"], "path");
  EXPECT_DOUBLE_EQ(p["path_length_m"].get<double>(), 7.0);
  EXPECT_DOUBLE_EQ(p["extension_m"].get<double>(), 4.0);
  EXPECT_DOUBLE_EQ(p["reverse_m"].get<double>(), 4.0);
  EXPECT_EQ(p["epoch"], 9u);
  EXPECT_EQ(p["sequence_id"], 4u);
  EXPECT_EQ(p["frontier_count"], 12u);
  EXPECT_TRUE(p["checks_complete"].get<bool>());

  mgg_msgs::msg::LocalPlannerStatus status;
  status.status = "blocked";
  status.reason = "no_transform:map";
  status.map_revision = 3;
  status.session_id = "s";
  status.request_id = "r";
  status.stamp.sec = 12;
  status.stamp.nanosec = 500000000;
  status.guidance_sequence_id = 2;
  const auto s = localStatusPayload(status);
  expectFields(s, {"status", "reason", "map_revision", "request_id", "stamp"});
  EXPECT_DOUBLE_EQ(s["stamp"].get<double>(), 12.5);
  EXPECT_EQ(s["guidance_sequence_id"], 2u);

  LocalModeRequest request;
  request.session_id = "s";
  request.request_id = "r";
  request.mode = LocalMode::kFollowRoute;
  request.frame_id = "map";
  request.goal = Eigen::Vector3d(12, 0, 0.5);
  request.tolerance_m = 0.3;
  request.route = {Eigen::Vector3d(0, 0, 0.5), Eigen::Vector3d(12, 0, 0.5)};
  const auto q =
      localRequestPayload(request, true, StateVec(1, 2, 3, 0.5), 42.0);
  expectFields(q, {"request_id", "continuation", "pose", "stamp"});
  EXPECT_TRUE(q["continuation"].get<bool>());
  EXPECT_DOUBLE_EQ(q["pose"]["yaw"].get<double>(), 0.5);
  EXPECT_EQ(q["mode"], "follow_route");
  EXPECT_EQ(q["frame_id"], "map");
  EXPECT_EQ(q["route"].size(), 2u);
  EXPECT_DOUBLE_EQ(q["goal"]["x"].get<double>(), 12.0);
  const auto unplaced = localRequestPayload(request, false, std::nullopt, 1.0);
  EXPECT_TRUE(unplaced["pose"].contains("x") && unplaced["pose"]["x"].is_null());

  EXPECT_EQ(robotIdFromNamespace("/robot_3/mgg/local_planner", "n"), "robot_3");
  EXPECT_EQ(robotIdFromNamespace("/", "n"), "n");
}
