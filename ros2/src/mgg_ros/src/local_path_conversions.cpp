#include "mgg_ros/local_path_conversions.h"

#include <stdexcept>

#include <mgg_msgs/msg/local_planner_status.hpp>

#include "mgg_ros/conversions.h"

namespace mgg {
namespace {

Eigen::Vector3d fromPointMsg(const geometry_msgs::msg::Point& p) {
  return Eigen::Vector3d(p.x, p.y, p.z);
}

std::vector<Eigen::Vector3d> fromPointMsgs(
    const std::vector<geometry_msgs::msg::Point>& points) {
  std::vector<Eigen::Vector3d> out;
  out.reserve(points.size());
  for (const auto& p : points) out.push_back(fromPointMsg(p));
  return out;
}

}  // namespace

mgg_msgs::msg::LocalPath toLocalPathMsg(const LocalPathPlan& plan) {
  mgg_msgs::msg::LocalPath msg;
  msg.header.frame_id = plan.frame_id;
  msg.header.stamp.sec = static_cast<int32_t>(plan.stamp_ns / 1000000000ULL);
  msg.header.stamp.nanosec =
      static_cast<uint32_t>(plan.stamp_ns % 1000000000ULL);
  msg.session_id = plan.session_id;
  msg.request_id = plan.request_id;
  msg.epoch = plan.epoch;
  msg.sequence_id = plan.sequence_id;
  msg.extends_sequence_id = plan.extends_sequence_id;
  msg.map_revision = plan.map_revision;
  msg.guidance_sequence_id = plan.guidance_sequence_id;
  switch (plan.kind) {
    case LocalPathKind::kStart:
      msg.kind = mgg_msgs::msg::LocalPath::START;
      break;
    case LocalPathKind::kExtend:
      msg.kind = mgg_msgs::msg::LocalPath::EXTEND;
      break;
    case LocalPathKind::kBreak:
      msg.kind = mgg_msgs::msg::LocalPath::BREAK;
      break;
  }
  msg.prefix_length = plan.prefix_length;
  msg.commit_length_m = plan.commit_length_m;
  msg.poses.reserve(plan.poses.size());
  for (const auto& pose : plan.poses)
    msg.poses.push_back(mgg_ros::toPoseMsg(pose));
  msg.reverse.assign(plan.reverse.begin(), plan.reverse.end());
  msg.reaches_goal = plan.reaches_goal;
  return msg;
}

LocalPathPlan fromLocalPathMsg(const mgg_msgs::msg::LocalPath& msg) {
  LocalPathPlan plan;
  switch (msg.kind) {
    case mgg_msgs::msg::LocalPath::START:
      plan.kind = LocalPathKind::kStart;
      break;
    case mgg_msgs::msg::LocalPath::EXTEND:
      plan.kind = LocalPathKind::kExtend;
      break;
    case mgg_msgs::msg::LocalPath::BREAK:
      plan.kind = LocalPathKind::kBreak;
      break;
    default:
      throw std::invalid_argument("LocalPath: unknown kind " +
                                  std::to_string(msg.kind));
  }
  plan.frame_id = msg.header.frame_id;
  plan.stamp_ns = static_cast<uint64_t>(msg.header.stamp.sec) * 1000000000ULL +
                  msg.header.stamp.nanosec;
  plan.session_id = msg.session_id;
  plan.request_id = msg.request_id;
  plan.epoch = msg.epoch;
  plan.sequence_id = msg.sequence_id;
  plan.extends_sequence_id = msg.extends_sequence_id;
  plan.map_revision = msg.map_revision;
  plan.guidance_sequence_id = msg.guidance_sequence_id;
  plan.prefix_length = msg.prefix_length;
  plan.commit_length_m = msg.commit_length_m;
  plan.poses.reserve(msg.poses.size());
  for (const auto& pose : msg.poses)
    plan.poses.push_back(mgg_ros::fromPoseMsg(pose));
  plan.reverse.assign(msg.reverse.begin(), msg.reverse.end());
  plan.reaches_goal = msg.reaches_goal;
  return plan;
}

std::string statusName(LocalStatus status) {
  using mgg_msgs::msg::LocalPlannerStatus;
  switch (status) {
    case LocalStatus::kMoving:
      return LocalPlannerStatus::MOVING;
    case LocalStatus::kWaitingForMap:
      return LocalPlannerStatus::WAITING_FOR_MAP;
    case LocalStatus::kNoLocalTarget:
      return LocalPlannerStatus::NO_LOCAL_TARGET;
    case LocalStatus::kBlocked:
      return LocalPlannerStatus::BLOCKED;
  }
  return LocalPlannerStatus::BLOCKED;
}

LocalFeedback fromFeedbackMsg(const mgg_msgs::msg::LocalPathFeedback& msg) {
  LocalFeedback feedback;
  feedback.session_id = msg.session_id;
  feedback.epoch = msg.epoch;
  feedback.sequence_id = msg.sequence_id;
  feedback.executing = msg.state == mgg_msgs::msg::LocalPathFeedback::EXECUTING;
  feedback.progress_m = msg.progress_m;
  feedback.speed_mps = msg.speed_mps;
  feedback.deceleration_mps2 = msg.deceleration_mps2;
  feedback.latency_s = msg.latency_s;
  feedback.speed_cap_mps = msg.speed_cap_mps;
  return feedback;
}

LocalGuidance fromGuidanceMsg(const mgg_msgs::msg::GlobalGuidance& msg) {
  LocalGuidance guidance;
  switch (msg.kind) {
    case mgg_msgs::msg::GlobalGuidance::NONE:
      guidance.kind = GuidanceKind::kNone;
      break;
    case mgg_msgs::msg::GlobalGuidance::TARGET:
      guidance.kind = GuidanceKind::kTarget;
      break;
    case mgg_msgs::msg::GlobalGuidance::COMPLETE:
      guidance.kind = GuidanceKind::kComplete;
      break;
    default:
      throw std::invalid_argument("GlobalGuidance: unknown kind " +
                                  std::to_string(msg.kind));
  }
  guidance.session_id = msg.session_id;
  guidance.sequence_id = msg.sequence_id;
  guidance.frame_id = msg.header.frame_id;
  guidance.target = fromPointMsg(msg.target);
  guidance.route = fromPointMsgs(msg.route);
  guidance.reason = msg.reason;
  return guidance;
}

LocalModeRequest fromModeRequest(
    const mgg_msgs::srv::SetLocalPlannerMode::Request& request) {
  using Request = mgg_msgs::srv::SetLocalPlannerMode::Request;
  LocalModeRequest mode;
  switch (request.mode) {
    case Request::IDLE:
      mode.mode = LocalMode::kIdle;
      break;
    case Request::EXPLORE:
      mode.mode = LocalMode::kExplore;
      break;
    case Request::FOLLOW_ROUTE:
      mode.mode = LocalMode::kFollowRoute;
      break;
    default:
      throw std::invalid_argument("SetLocalPlannerMode: unknown mode " +
                                  std::to_string(request.mode));
  }
  mode.session_id = request.session_id;
  mode.request_id = request.request_id;
  mode.frame_id = request.frame_id;
  mode.goal = fromPointMsg(request.goal);
  mode.tolerance_m = request.tolerance_m;
  mode.route = fromPointMsgs(request.route);
  return mode;
}

}  // namespace mgg
