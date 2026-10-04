// C4a payloads of the local planner node's native events. The node logs
// each through mgg_ros::explorationEventLine and RCLCPP_INFO:
//   local_plan    one per planning cycle in EXPLORE or FOLLOW_ROUTE;
//   local_status  when status, reason, session or request changes;
//   local_request one per accepted set_mode (replay re-issues them).
// Every payload carries session_id. Required fields are p0's canonical
// names (adapters/exploration_telemetry.py); the rest are optional extras
// that consumers must not require.
//
// A local_request is recorded losslessly: when its route would overflow one
// log line, it is split into parts (localRequestParts), and replay
// reassembles them (recordedLocalRequests), failing on a missing part.

#ifndef MGG_ROS_LOCAL_EVENTS_H_
#define MGG_ROS_LOCAL_EVENTS_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

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

/// Payload bytes per local_request part: p0's 15360-byte line budget less
/// the envelope (kind, robot, stamps, boot, seq) with a wide margin.
inline constexpr std::size_t kLocalRequestPartBytes = 12 * 1024;

/// `payload` (from localRequestPayload) in parts whose dumps fit
/// `max_bytes`. Every part repeats request_id, session_id, continuation,
/// pose and stamp and carries route_part (from 0) and route_parts; part 0
/// also carries the other extras. The parts' route slices concatenate to
/// the route, unchanged. One part when everything fits.
std::vector<nlohmann::json> localRequestParts(const nlohmann::json& payload,
                                              std::size_t max_bytes);

/// One accepted set_mode, as recorded.
struct RecordedLocalRequest {
  LocalModeRequest request;
  bool continuation = false;
  /// Odometry base pose at the request; nullopt before odometry.
  std::optional<StateVec> pose;
  double stamp = 0;
};

/// The requests of local_request payloads in exported order, each in the
/// order of its first part, with route parts reassembled. Throws
/// std::invalid_argument for a missing, repeated or inconsistent part, an
/// unknown mode or a malformed field: an incomplete recording never
/// replays.
std::vector<RecordedLocalRequest> recordedLocalRequests(
    const std::vector<nlohmann::json>& payloads);

/// The robot of a node namespace: its first segment ("/robot_1/mgg/..."),
/// else `fallback`.
std::string robotIdFromNamespace(const std::string& ns,
                                 const std::string& fallback);

}  // namespace mgg

#endif  // MGG_ROS_LOCAL_EVENTS_H_
