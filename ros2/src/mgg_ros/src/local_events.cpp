#include "mgg_ros/local_events.h"

#include <algorithm>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <utility>

namespace mgg {
namespace {

nlohmann::json point(const Eigen::Vector3d& p) {
  return {{"x", p.x()}, {"y", p.y()}, {"z", p.z()}};
}

double segment(const LocalPathPlan& path, std::size_t i) {
  return (path.poses[i + 1] - path.poses[i]).head<3>().norm();
}

// The identity every local_request part repeats.
constexpr const char* kRequestIdentity[] = {"request_id", "session_id",
                                            "continuation", "pose", "stamp"};

Eigen::Vector3d pointOf(const nlohmann::json& j) {
  return {j.at("x").get<double>(), j.at("y").get<double>(),
          j.at("z").get<double>()};
}

LocalMode modeOf(const std::string& name) {
  if (name == "idle") return LocalMode::kIdle;
  if (name == "explore") return LocalMode::kExplore;
  if (name == "follow_route") return LocalMode::kFollowRoute;
  throw std::invalid_argument("local_request: unknown mode '" + name + "'");
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

std::vector<nlohmann::json> localRequestParts(const nlohmann::json& payload,
                                              std::size_t max_bytes) {
  const nlohmann::json route = payload.at("route");
  nlohmann::json first = payload;
  first["route"] = nlohmann::json::array();
  nlohmann::json other = {{"route", nlohmann::json::array()}};
  for (const char* key : kRequestIdentity) other[key] = payload.at(key);
  // Sized with the widest part numbers the request can have.
  const auto emptySize = [&](const nlohmann::json& part) {
    nlohmann::json sized = part;
    sized["route_part"] = route.size();
    sized["route_parts"] = route.size();
    return sized.dump().size();
  };
  std::vector<nlohmann::json> slices{nlohmann::json::array()};
  std::size_t size = emptySize(first);
  for (const auto& point : route) {
    const std::size_t bytes = point.dump().size();
    if (!slices.back().empty() && size + 1 + bytes > max_bytes) {
      slices.push_back(nlohmann::json::array());
      size = emptySize(other);
    }
    size += bytes + (slices.back().empty() ? 0 : 1);
    slices.back().push_back(point);
  }
  std::vector<nlohmann::json> parts;
  for (std::size_t k = 0; k < slices.size(); ++k) {
    nlohmann::json part = k == 0 ? first : other;
    part["route"] = std::move(slices[k]);
    part["route_part"] = k;
    part["route_parts"] = slices.size();
    parts.push_back(std::move(part));
  }
  return parts;
}

namespace {

std::vector<RecordedLocalRequest> assembleRequests(
    const std::vector<nlohmann::json>& payloads) {
  struct Parts {
    std::vector<std::optional<nlohmann::json>> parts;
  };
  using Key = std::pair<std::string, std::string>;
  std::vector<Key> order;
  std::map<Key, Parts> requests;
  for (const auto& payload : payloads) {
    const Key key{payload.at("session_id").get<std::string>(),
                  payload.at("request_id").get<std::string>()};
    const auto count = payload.value("route_parts", std::size_t{1});
    const auto index = payload.value("route_part", std::size_t{0});
    if (count == 0 || index >= count)
      throw std::invalid_argument("local_request " + key.second +
                                  ": route_part out of range");
    auto [it, added] = requests.try_emplace(key);
    auto& parts = it->second.parts;
    if (added) {
      order.push_back(key);
      parts.resize(count);
    } else if (parts.size() != count) {
      throw std::invalid_argument("local_request " + key.second +
                                  ": inconsistent route_parts");
    }
    for (const auto& known : parts) {
      if (!known) continue;
      for (const char* field : kRequestIdentity)
        if (known->at(field) != payload.at(field))
          throw std::invalid_argument("local_request " + key.second +
                                      ": parts disagree on " + field);
      break;
    }
    if (parts[index])
      throw std::invalid_argument("local_request " + key.second +
                                  ": repeated route_part");
    parts[index] = payload;
  }
  std::vector<RecordedLocalRequest> result;
  for (const auto& key : order) {
    const auto& parts = requests.at(key).parts;
    for (std::size_t k = 0; k < parts.size(); ++k)
      if (!parts[k])
        throw std::invalid_argument("local_request " + key.second +
                                    ": missing route_part " +
                                    std::to_string(k) + " of " +
                                    std::to_string(parts.size()));
    const auto& head = *parts.front();
    RecordedLocalRequest recorded;
    auto& request = recorded.request;
    request.session_id = key.first;
    request.request_id = key.second;
    request.mode = modeOf(head.at("mode").get<std::string>());
    request.frame_id = head.at("frame_id").get<std::string>();
    request.goal = pointOf(head.at("goal"));
    request.tolerance_m = head.at("tolerance_m").get<double>();
    for (const auto& part : parts)
      for (const auto& point : part->at("route"))
        request.route.push_back(pointOf(point));
    recorded.continuation = head.at("continuation").get<bool>();
    const auto& pose = head.at("pose");
    if (!pose.at("x").is_null())
      recorded.pose = StateVec(pose.at("x").get<double>(),
                               pose.at("y").get<double>(),
                               pose.at("z").get<double>(),
                               pose.at("yaw").get<double>());
    recorded.stamp = head.at("stamp").get<double>();
    result.push_back(std::move(recorded));
  }
  return result;
}

}  // namespace

std::vector<RecordedLocalRequest> recordedLocalRequests(
    const std::vector<nlohmann::json>& payloads) {
  try {
    return assembleRequests(payloads);
  } catch (const nlohmann::json::exception& error) {
    throw std::invalid_argument(std::string("local_request: ") + error.what());
  }
}

std::string robotIdFromNamespace(const std::string& ns,
                                 const std::string& fallback) {
  const std::size_t first = ns.find_first_not_of('/');
  if (first == std::string::npos) return fallback;
  const std::string robot = ns.substr(first, ns.find('/', first) - first);
  return robot.empty() ? fallback : robot;
}

}  // namespace mgg
