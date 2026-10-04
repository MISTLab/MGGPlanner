#include <gtest/gtest.h>
#include <limits>
#include "mgg_ros/exploration_event.h"

TEST(ExplorationEvent, LineParsesAsEnvelope) {
  const nlohmann::json payload = {{"goal_id", 7}, {"phase", "started"}, {"reason", "accepted"}};
  auto line = mgg_ros::explorationEventLine("goal", "r", payload, 1.0);
  ASSERT_EQ(line.substr(0, 7), "SDEVT1 ");
  const auto parsed = nlohmann::json::parse(line.substr(7));
  EXPECT_EQ(parsed["v"], 1);
  EXPECT_EQ(parsed["payload"], payload);
  EXPECT_EQ(parsed["stamp"], 1.0);
  EXPECT_EQ(parsed["robot_id"], "r");
  EXPECT_EQ(parsed["boot"].get<std::string>().size(), 32u);
  EXPECT_EQ(parsed["boot"].get<std::string>()[12], '4');
  const auto next = nlohmann::json::parse(mgg_ros::explorationEventLine("goal", "r", payload, std::nullopt).substr(7));
  EXPECT_EQ(next["boot"], parsed["boot"]);
  EXPECT_EQ(next["seq"].get<uint64_t>(), parsed["seq"].get<uint64_t>() + 1);
  EXPECT_TRUE(next["stamp"].is_null());
}

TEST(ExplorationEvent, RejectsOversizeAndNonfinite) {
  EXPECT_THROW(mgg_ros::explorationEventLine("goal", "r", {{"reason", std::string(16385, 'x')}}, 1.0), std::invalid_argument);
  EXPECT_THROW(mgg_ros::explorationEventLine("goal", "r", {{"duration_ms", std::numeric_limits<double>::infinity()}}, 1.0), std::invalid_argument);
  EXPECT_THROW(mgg_ros::explorationEventLine("goal", "r", nlohmann::json::object(), std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST(ExplorationEvent, ReservesLogTransportHeadroom) {
  EXPECT_THROW(mgg_ros::explorationEventLine("goal", "r", {{"reason", std::string(15400, 'x')}}, 1.0), std::invalid_argument);
  const nlohmann::json payload = {{"reason", std::string(15000, 'x')}};
  const auto line = mgg_ros::explorationEventLine("goal", "r", payload, 1.0);
  EXPECT_LE(line.size() + 1024, 16384u);
  EXPECT_EQ(nlohmann::json::parse(line.substr(7))["payload"], payload);
}
