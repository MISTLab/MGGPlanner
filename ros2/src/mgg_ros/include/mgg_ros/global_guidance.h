#ifndef MGG_ROS_GLOBAL_GUIDANCE_H_
#define MGG_ROS_GLOBAL_GUIDANCE_H_

#include <string>
#include <vector>
#include <mgg_msgs/msg/global_guidance.hpp>
#include <mgg_msgs/msg/local_planner_status.hpp>
#include <mgg_msgs/srv/set_local_planner_mode.hpp>

namespace mgg {

// Session state and wire construction for the temporary global graph guide.
// This is not an execution path; only the local planner certifies motion.
class GlobalGuidance {
 public:
  bool setMode(const mgg_msgs::srv::SetLocalPlannerMode::Request& request);
  const mgg_msgs::srv::SetLocalPlannerMode::Request& mode() const { return mode_; }
  mgg_msgs::msg::GlobalGuidance message(
      const std::vector<geometry_msgs::msg::Point>& route, bool complete,
      const std::string& frame, const std::string& reason);
  bool setsTargetAside(const mgg_msgs::msg::LocalPlannerStatus& status) const;

 private:
  mgg_msgs::srv::SetLocalPlannerMode::Request mode_;
  mgg_msgs::msg::GlobalGuidance last_;
  uint64_t sequence_ = 0;
};

}  // namespace mgg
#endif
