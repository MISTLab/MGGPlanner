// The MGG planner as a ROS 2 node.
//
// Owns mgg_core and wires it to ROS: parameters in, odometry in, neighbour
// graphs in and out, markers out. The algorithm itself lives in mgg_core and
// links no ROS libraries.
//
// SCOPE. This node currently covers graph construction and the multi-robot
// exchange, which is what has been ported. Volumetric gain evaluation and
// best-path selection (Rrg::computeExplorationGain, evaluateGraph and
// getBestPath) are still in the ROS 1 rrg.cpp; the planning service therefore
// reports the graph it built and returns an empty path rather than inventing
// one. See ROS2_PORT_PLAN.md.

#ifndef MGG_ROS_PLANNER_NODE_H_
#define MGG_ROS_PLANNER_NODE_H_

#include <memory>
#include <string>
#include <unordered_map>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mgg_msgs/msg/graph.hpp>

#include "mgg_core/geofence_manager.h"
#include "mgg_core/graph_expansion.h"
#include "mgg_core/graph_manager.h"
#include "mgg_core/graph_merge.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/ground_projection.h"
#include "mgg_core/params.h"
#include "mgg_core/sensor_params.h"
#include "mgg_map_octomap/octomap_map.h"

namespace mgg_ros {

class PlannerNode : public rclcpp::Node {
 public:
  explicit PlannerNode(const rclcpp::NodeOptions& options);

 private:
  void loadParameters();
  void onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void onNeighbourGraph(mgg_msgs::msg::Graph::ConstSharedPtr msg);
  void onBuildRequest(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void publishOwnGraph();
  void publishMarkers();

  /// Builds the local grid graph around the current state. Returns a summary
  /// suitable for a service response.
  std::string buildLocalGraph();

  mgg::ExpandContext makeContext();

  // Core state. None of these know about ROS.
  std::unique_ptr<mgg::OctomapMap> map_;
  std::unique_ptr<mgg::GroundProjection> ground_;
  std::unique_ptr<mgg::GeofenceManager> geofence_;
  std::shared_ptr<mgg::GraphManager> local_graph_;
  std::shared_ptr<mgg::GraphManager> global_graph_;
  std::unique_ptr<mgg::StaticPoseSource> poses_;

  mgg::RobotParams robot_params_;
  mgg::PlanningParams planning_params_;
  mgg::GridGraphParams grid_params_;
  std::unordered_map<std::string, mgg::SensorParams> sensors_;

  mgg::StateVec current_state_ = mgg::StateVec::Zero();
  bool have_odometry_ = false;
  std::string world_frame_ = "world";

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<mgg_msgs::msg::Graph>::SharedPtr neighbour_sub_;
  rclcpp::Publisher<mgg_msgs::msg::Graph>::SharedPtr graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr build_srv_;
  rclcpp::TimerBase::SharedPtr graph_timer_;

  /// Reentrant, so the planning service and the subscriptions can run
  /// concurrently under a MultiThreadedExecutor. See the note in main().
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

}  // namespace mgg_ros

#endif  // MGG_ROS_PLANNER_NODE_H_
