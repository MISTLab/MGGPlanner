#include "mgg_ros/local_events.h"

#include <algorithm>
#include <cstddef>

namespace mgg {
namespace {

nlohmann::json point(const Eigen::Vector3d& p) {
  return {{"x", p.x()}, {"y", p.y()}, {"z", p.z()}};
}

double segment(const LocalPathPlan& path, std::size_t i) {
  return (path.poses[i + 1] - path.poses[i]).head<3>().norm();
}

std::string modeName(LocalMode mode) {
  switch (mode) {
    case LocalMode::kExplore:
      return "explore";
    case LocalMode::kFollowRoute:
      return "follow_route";
    case LocalMode::kIdle:
      break;
  }
  return "idle";
}

}  // namespace

nlohmann::json localPlanPayload(const LocalPlanResult& result,
                                const std::string& session_id,
                                const std::string& request_id,
                                std::uint64_t map_revision) {
  nlohmann::json j = {{"session_id", session_id},
                      {"request_id", request_id},
                      {"duration_ms", std::max(0.0, result.cycle_ms)},
                      {"map_revision", map_revision},
                      {"checks_complete", result.checks_complete},
                      {"reason", result.reason}};
  if (result.candidate_count) j["frontier_count"] = *result.candidate_count;
  if (!result.path) {
    j["outcome"] = result.status == LocalStatus::kWaitingForMap ? "waiting_for_map"
                   : result.status == LocalStatus::kBlocked     ? "blocked"
                                                                : "no_path";
    j["path_length_m"] = nullptr;
    return j;
  }
  const auto& path = *result.path;
  // Travel beyond the retained prefix starts at its last pose.
  const std::size_t splice =
      path.prefix_length > 0 ? path.prefix_length - 1 : 0;
  double extension = 0, reverse = 0;
  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
    if (i >= splice) extension += segment(path, i);
    if (i < path.reverse.size() && path.reverse[i]) reverse += segment(path, i);
  }
  j["outcome"] = "path";
  j["path_length_m"] = pathLength(path);
  j["epoch"] = path.epoch;
  j["sequence_id"] = path.sequence_id;
  j["extension_m"] = extension;
  j["reverse_m"] = reverse;
  return j;
}

nlohmann::json localStatusPayload(
    const mgg_msgs::msg::LocalPlannerStatus& status) {
  return {{"status", status.status},
          {"reason", status.reason},
          {"map_revision", status.map_revision},
          {"session_id", status.session_id},
          {"request_id", status.request_id},
          {"stamp", status.stamp.sec + status.stamp.nanosec * 1e-9},
          {"guidance_sequence_id", status.guidance_sequence_id}};
}

nlohmann::json localRequestPayload(const LocalModeRequest& request,
                                   bool continuation,
                                   const std::optional<StateVec>& pose,
                                   double stamp_s) {
  nlohmann::json where = {
      {"x", nullptr}, {"y", nullptr}, {"z", nullptr}, {"yaw", nullptr}};
  if (pose)
    where = {{"x", pose->x()},
             {"y", pose->y()},
             {"z", pose->z()},
             {"yaw", pose->w()}};
  nlohmann::json route = nlohmann::json::array();
  for (const auto& p : request.route) route.push_back(point(p));
  return {{"request_id", request.request_id},
          {"session_id", request.session_id},
          {"continuation", continuation},
          {"pose", where},
          {"stamp", stamp_s},
          {"mode", modeName(request.mode)},
          {"frame_id", request.frame_id},
          {"goal", point(request.goal)},
          {"tolerance_m", request.tolerance_m},
          {"route", route}};
}

std::string robotIdFromNamespace(const std::string& ns,
                                 const std::string& fallback) {
  const std::size_t first = ns.find_first_not_of('/');
  if (first == std::string::npos) return fallback;
  const std::string robot = ns.substr(first, ns.find('/', first) - first);
  return robot.empty() ? fallback : robot;
}

}  // namespace mgg
