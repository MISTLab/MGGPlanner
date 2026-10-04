#include "mgg_ros/planner_node.h"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "mgg_core/planning_cancellation.h"

namespace mgg_ros {

bool PlannerNode::objectiveRouteIsGuidance() const {
  return exploration_architecture_ == "v2" &&
         robot_params_.type == mgg::RobotType::kGroundRobot;
}

void PlannerNode::setupGuidance() {
  guidance_pub_ = create_publisher<mgg_msgs::msg::GlobalGuidance>(
      "global_guidance", rclcpp::QoS(1).reliable().transient_local());
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  local_status_sub_ = create_subscription<mgg_msgs::msg::LocalPlannerStatus>(
      "local_planner/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](mgg_msgs::msg::LocalPlannerStatus::ConstSharedPtr msg) {
        onLocalPlannerStatus(msg);
      }, options);
  guidance_mode_service_ = create_service<mgg_msgs::srv::SetLocalPlannerMode>(
      "guidance/set_mode",
      [this](std::shared_ptr<mgg_msgs::srv::SetLocalPlannerMode::Request> request,
             std::shared_ptr<mgg_msgs::srv::SetLocalPlannerMode::Response> response) {
        const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
        response->epoch = planner_config_state_.incarnation;
        response->accepted = guidance_.setMode(*request);
        response->reason = response->accepted ? "accepted" : "invalid mode or identity";
        if (!response->accepted) return;
        guidance_target_.reset();
        // Maintenance previously starts only after a legacy plan request.
        if (request->mode != request->IDLE) ++planner_trigger_count_;
        guidanceTick();
      }, rclcpp::ServicesQoS(), callback_group_);
  guidance_timer_ = create_timer(std::chrono::seconds(1),
      [this] { guidanceTick(); }, callback_group_);
}

void PlannerNode::onLocalPlannerStatus(
    mgg_msgs::msg::LocalPlannerStatus::ConstSharedPtr msg) {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (!objectiveRouteIsGuidance() || !guidance_target_ ||
      !guidance_.setsTargetAside(*msg)) return;
  // A local search failure is not physical unreachability. Reuse the tour's
  // expiring retry entry; never demote the graph's frontier or mark it covered.
  setTourClusterAside(*guidance_target_, tour_params_.route_retry_s);
  guidance_target_.reset();
  guidanceTick();
}

void PlannerNode::guidanceTick() {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  const auto& mode = guidance_.mode();
  if (mode.session_id.empty()) return;
  std::vector<geometry_msgs::msg::Point> route;
  std::string reason = mode.mode == mode.IDLE ? "idle" : "objective route";
  bool complete = false;
  std::optional<mgg::StandingStart> standing;
  guidance_target_.reset();
  if (mode.mode != mode.IDLE) {
    try {
      applyLatestOdometry();
      standing = guidanceStandingStart();
      auto map_read = mapReadLease();
      refreshMapRevision();
      refreshNoGoZones();
      withdrawUnplacedNeighbours();
      const bool fresh = have_odometry_ &&
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        last_odometry_received_).count() <= odometry_stale_s_;
      if (!fresh || !map_->getStatus()) {
        reason = "waiting for current odometry and map";
      } else {
        auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
        // A finite RETURN_HOME goal uses the existing certified global route
        // search, without invoking the legacy local exploration/departure loop.
        request->objective = request->RETURN_HOME;
        request->component_id = mapping_snapshot_.component_id;
        request->map_epoch = mapping_snapshot_.epoch;
        request->goal.orientation.w = 1;
        bool have_target = false;
        if (mode.mode == mode.EXPLORE) {
          // Phase 1a retains the current frontier mechanism. buildLocalGraph
          // owns lattice_time_budget_s and returns before path selection in v2;
          // only its frontier vertices/routes enter the global graph.
          guidance_discovery_complete_ = false;
          reason = buildLocalGraph();
          std::string tour_reason;
          guidance_target_ = refreshTour(tour_reason);
          reason += tour_reason;
          if (guidance_target_) {
            const auto& p = guidance_target_->position;
            request->goal.position.x = p.x();
            request->goal.position.y = p.y();
            request->goal.position.z = p.z();
            have_target = true;
          } else {
            // Set-aside, disconnected or unplaced frontiers are not exhausted.
            complete = guidance_discovery_complete_ && tour_set_aside_.empty() &&
                globalFrontierClusters().empty() && completionWithheld().empty();
          }
        } else {
          geometry_msgs::msg::PointStamped goal;
          goal.point = mode.goal;
          goal.header.frame_id = mode.frame_id;
          if (goal.header.frame_id.empty() && applied_odometry_)
            goal.header.frame_id = applied_odometry_->header.frame_id;
          if (goal.header.frame_id.empty()) goal.header.frame_id = world_frame_;
          if (goal.header.frame_id != world_frame_) {
            const auto transform = tf_buffer_->lookupTransform(
                world_frame_, goal.header.frame_id, tf2::TimePointZero);
            tf2::doTransform(goal, goal, transform);
          }
          request->goal.position = goal.point;
          have_target = true;
        }
        if (have_target) {
          auto response = std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
          onObjectiveRequest(request, response);
          if (!response->reason.empty()) reason += "; " + response->reason;
          if (response->status == response->SUCCEEDED) {
            for (const auto& pose : response->path) route.push_back(pose.position);
          } else if (mode.mode == mode.EXPLORE && guidance_target_ &&
                     response->status != response->STALE_REVISION) {
            // A failed route must not monopolize the tour. This is an
            // expiring retry, not physical unreachability; stale map authority
            // is not evidence against the target at all.
            setTourClusterAside(*guidance_target_, tour_params_.route_retry_s);
            guidance_target_.reset();
          }
          // A failed bounded search remains NONE, not COMPLETE/unreachable.
        }
      }
    } catch (const tf2::TransformException& ex) {
      reason = std::string("no_transform:") + ex.what();
    } catch (const mgg::PlanningInterrupted&) {
      reason = "guidance search interrupted";
    }
  }
  const auto message = guidance_.message(route, complete, world_frame_, reason,
                                         now(), standing,
                                         planner_config_state_.incarnation);
  guidance_pub_->publish(message);
}

}  // namespace mgg_ros
