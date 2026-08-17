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
#include <mutex>
#include <string>
#include <unordered_map>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mgg_msgs/msg/graph.hpp>
#include <mgg_msgs/srv/planner_srv.hpp>

#include "mgg_core/geofence_manager.h"
#include "mgg_core/gain.h"
#include "mgg_core/graph_expansion.h"
#include "mgg_core/graph_manager.h"
#include "mgg_core/graph_merge.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/path_selection.h"
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
  /// The planning service proper: run a cycle and hand back the chosen path.
  void onPlanRequest(
      const std::shared_ptr<mgg_msgs::srv::PlannerSrv::Request> request,
      std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response);
  void publishOwnGraph();
  void publishPath();
  void publishMarkers();

  /// Builds the local grid graph around the current state. Returns a summary
  /// suitable for a service response.
  std::string buildLocalGraph();

  /// Extends the global topological graph with the robot's current pose.
  ///
  /// The global graph is the sparse, persistent one that robots exchange: a
  /// root, the trajectory, and (once gain evaluation is ported) frontiers.
  /// Without the trajectory backbone there is nothing to broadcast and no
  /// geometry for a neighbour's graph to rendezvous with.
  void updateGlobalGraph();

  mgg::ExpandContext makeContext();
  mgg::GainContext makeGainContext();

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
  mgg::BoundedSpaceParams global_space_;
  std::vector<mgg::BoundedSpaceParams> no_gain_zones_;
  std::unordered_map<std::string, mgg::SensorParams> sensors_;

  /// Serialises everything that touches the map or the graphs.
  ///
  /// Every callback here shares one reentrant group under a
  /// MultiThreadedExecutor - which the PCI needs, or a service call made from
  /// inside a callback deadlocks - and reentrant means genuinely concurrent.
  /// OctoMap is not thread-safe, so two point clouds arriving faster than one
  /// can be inserted will corrupt the octree and segfault inside
  /// insertPointCloud. Slow, occasional callbacks hide this: it takes a real
  /// sensor rate to make the callbacks overlap.
  ///
  /// One mutex rather than one per structure, because planning reads the map
  /// and writes the graphs as a single unit and would need both anyway.
  std::recursive_mutex planner_mutex_;


  mgg::StateVec current_state_ = mgg::StateVec::Zero();
  bool have_odometry_ = false;
  /// Point clouds arrive in the sensor frame and have to be placed in the
  /// world frame before they go into the map. TF, rather than the odometry
  /// pose, because it is the only thing that knows where the sensor is
  /// mounted: rays have to be carved from the sensor, not from the robot's
  /// origin.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  /// How long to wait for the transform matching a cloud's stamp.
  double cloud_tf_timeout_sec_ = 0.1;
  double global_vertex_spacing_ = 1.0;
  /// Where the last global-graph vertex was dropped, so odometry can decide
  /// cheaply whether the backbone needs extending without touching the map.
  Eigen::Vector3d last_global_anchor_ = Eigen::Vector3d::Zero();
  bool have_global_anchor_ = false;
  /// Heading the robot has been travelling, for the direction penalty.
  double exploring_direction_ = 0.0;
  mgg::EdgeInclinations edge_inclinations_;
  /// Last chosen path, in world coordinates.
  std::vector<mgg::StateVec> best_path_;
  /// Points before and after shortcutting, reported so it is visible whether
  /// the smoothing did anything.
  int path_shortcut_from_ = 0;
  /// Corners left after shortcutting, before resampling puts points back.
  /// This is the number that says whether the smoothing did anything: the
  /// final count goes up again, because interpolation adds evenly spaced
  /// points along the straightened route.
  int path_shortcut_corners_ = 0;
  int path_shortcut_to_ = 0;
  std::string world_frame_ = "world";

  struct MergeEvent {
    rclcpp::Time stamp;
    int sender_id = 0;
    Eigen::Vector3d our_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d their_pos = Eigen::Vector3d::Zero();
  };
  std::vector<MergeEvent> recent_merges_;


  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<mgg_msgs::msg::Graph>::SharedPtr neighbour_sub_;
  rclcpp::Publisher<mgg_msgs::msg::Graph>::SharedPtr graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr build_srv_;
  rclcpp::Service<mgg_msgs::srv::PlannerSrv>::SharedPtr plan_srv_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
  /// One-shot guard against use_sim_time with no /clock.
  rclcpp::TimerBase::SharedPtr sim_time_check_;

  /// Reentrant, so the planning service and the subscriptions can run
  /// concurrently under a MultiThreadedExecutor. See the note in main().
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

}  // namespace mgg_ros

#endif  // MGG_ROS_PLANNER_NODE_H_
