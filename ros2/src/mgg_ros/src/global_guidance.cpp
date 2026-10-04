#include "mgg_ros/global_guidance.h"
#include <cmath>

namespace mgg {
bool GlobalGuidance::setMode(const mgg_msgs::srv::SetLocalPlannerMode::Request& request) {
  using Mode = mgg_msgs::srv::SetLocalPlannerMode::Request;
  const auto finite = [](const geometry_msgs::msg::Point& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  };
  if (request.mode > Mode::FOLLOW_ROUTE || request.session_id.empty() ||
      request.request_id.empty() || !std::isfinite(request.tolerance_m) ||
      request.tolerance_m < 0 || !finite(request.goal)) return false;
  for (const auto& point : request.route) if (!finite(point)) return false;
  mode_ = request;
  last_ = {};
  return true;
}

mgg_msgs::msg::GlobalGuidance GlobalGuidance::message(
    const std::vector<geometry_msgs::msg::Point>& route, bool complete,
    const std::string& frame, const std::string& reason,
    const builtin_interfaces::msg::Time& stamp,
    const std::optional<StandingStart>& standing, std::uint64_t boot) {
  mgg_msgs::msg::GlobalGuidance out;
  out.header.frame_id = frame;
  out.header.stamp = stamp;
  out.session_id = mode_.session_id;
  out.sequence_id = ++sequence_;
  out.reason = reason;
  out.kind = out.NONE;
  if (!route.empty()) {
    out.kind = out.TARGET;
    out.route = route;
    out.target = route.back();
  } else if (complete) {
    out.kind = out.COMPLETE;
  }
  out.standing_start.boot = boot;
  if (standing) {
    out.standing_start.valid = true;
    out.standing_start.center.x = standing->center.x();
    out.standing_start.center.y = standing->center.y();
    out.standing_start.radius = standing->radius;
  }
  last_ = out;
  return out;
}

bool GlobalGuidance::setsTargetAside(const mgg_msgs::msg::LocalPlannerStatus& status) const {
  return mode_.mode == mode_.EXPLORE && last_.kind == last_.TARGET &&
      status.session_id == mode_.session_id && status.request_id == mode_.request_id &&
      (status.stamp.sec > last_.header.stamp.sec ||
       (status.stamp.sec == last_.header.stamp.sec &&
        status.stamp.nanosec >= last_.header.stamp.nanosec)) &&
      (status.guidance_sequence_id == 0 || status.guidance_sequence_id == last_.sequence_id) &&
      (status.status == status.BLOCKED || status.status == status.NO_LOCAL_TARGET);
}
}  // namespace mgg
