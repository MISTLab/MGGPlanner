// Translating the ground local planner's wire (mgg_msgs LocalPath,
// LocalPathFeedback, GlobalGuidance, LocalPlannerStatus, SetLocalPlannerMode)
// to and from the ROS-free types of LocalPlanningCore and mgg_core.

#ifndef MGG_ROS_LOCAL_PATH_CONVERSIONS_H_
#define MGG_ROS_LOCAL_PATH_CONVERSIONS_H_

#include <string>

#include <mgg_msgs/msg/global_guidance.hpp>
#include <mgg_msgs/msg/local_path.hpp>
#include <mgg_msgs/msg/local_path_feedback.hpp>
#include <mgg_msgs/srv/set_local_planner_mode.hpp>

#include "mgg_core/local_path.h"
#include "mgg_core/local_planner_status.h"
#include "mgg_ros/local_planning_core.h"

namespace mgg {

/// Header frame and stamp come from the plan's frame_id and stamp_ns;
/// dependency boxes stay local.
mgg_msgs::msg::LocalPath toLocalPathMsg(const LocalPathPlan& plan);
/// Throws std::invalid_argument for an unknown kind.
LocalPathPlan fromLocalPathMsg(const mgg_msgs::msg::LocalPath& msg);
/// The LocalPlannerStatus constant for `status`.
std::string statusName(LocalStatus status);

LocalFeedback fromFeedbackMsg(const mgg_msgs::msg::LocalPathFeedback& msg);
/// Throws std::invalid_argument for an unknown kind.
LocalGuidance fromGuidanceMsg(const mgg_msgs::msg::GlobalGuidance& msg);
/// Throws std::invalid_argument for an unknown mode.
LocalModeRequest fromModeRequest(
    const mgg_msgs::srv::SetLocalPlannerMode::Request& request);

}  // namespace mgg

#endif  // MGG_ROS_LOCAL_PATH_CONVERSIONS_H_
