// C4a payloads of the local planner node's native events. The node logs
// each through mgg_ros::explorationEventLine and RCLCPP_INFO:
//   local_plan    one per planning cycle in EXPLORE or FOLLOW_ROUTE;
//   local_status  when status, reason, session or request changes;
//   local_request one per accepted set_mode (replay re-issues them).
// Every payload carries session_id. Required fields are p0's canonical
// names (adapters/exploration_telemetry.py); the rest are optional extras
// that consumers must not require.

#ifndef MGG_ROS_LOCAL_EVENTS_H_
#define MGG_ROS_LOCAL_EVENTS_H_

#include <cstdint>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>

#include <mgg_msgs/msg/local_planner_status.hpp>

#include "mgg_core/local_planner.h"
#include "mgg_ros/local_planning_core.h"

namespace mgg {

/// local_plan. `outcome` is "path" when the cycle produced a path, else
/// waiting_for_map or blocked from its status, else no_path;
/// path_length_m is null without a path. Extras: epoch, sequence_id,
/// extension_m (beyond the retained prefix), reverse_m, checks_complete,
/// reason, and frontier_count when candidate scoring ran.
nlohmann::json localPlanPayload(const LocalPlanResult& result,
                                const std::string& session_id,
                                const std::string& request_id,
                                std::uint64_t map_revision);

/// local_status: the C2a fields (stamp in seconds); extra
/// guidance_sequence_id.
nlohmann::json localStatusPayload(
    const mgg_msgs::msg::LocalPlannerStatus& status);

/// local_request. `pose` is the odometry base pose {x, y, z, yaw}, each
/// null before the first odometry. Extras: mode, frame_id, goal,
/// tolerance_m, route.
nlohmann::json localRequestPayload(const LocalModeRequest& request,
                                   bool continuation,
                                   const std::optional<StateVec>& pose,
                                   double stamp_s);

/// The robot of a node namespace: its first segment ("/robot_1/mgg/..."),
/// else `fallback`.
std::string robotIdFromNamespace(const std::string& ns,
                                 const std::string& fallback);

}  // namespace mgg

#endif  // MGG_ROS_LOCAL_EVENTS_H_
