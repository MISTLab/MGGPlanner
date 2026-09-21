// The MGG planner as a ROS 2 node.
//
// Owns mgg_core and wires it to ROS: parameters in, odometry and the map in,
// neighbour graphs in and out, paths and markers out. The algorithm itself
// lives in mgg_core and links no ROS libraries.
//
// One exploration cycle (Mggplanner::plannerServiceCallback, rrg.cpp):
//   * the local grid graph is built around the robot and its leaves are
//     scored by volumetric gain; the best leaf's shortest path through the
//     lattice is the plan, shortcut and resampled, and returned whole;
//   * without a frontier for `auto_global_planner_low_gain_rounds` cycles the
//     global planner runs Dijkstra over the global graph to the best frontier
//     and that route is the plan, returned whole. The frontier stays the
//     target until the robot is within `global_frontier_reach_m` of it;
//   * the accepted path and the local graph's frontier clusters join the
//     global graph; odometry keeps wiring the robot's track into it; the
//     expansion timer grows it around unvisited clusters between cycles.
// Explicit objectives (Navigate, Return Home) are routes over the global
// graph, returned whole.

#ifndef MGG_ROS_PLANNER_NODE_H_
#define MGG_ROS_PLANNER_NODE_H_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mgg_msgs/msg/graph.hpp>
#include <mgg_msgs/msg/mapping_snapshot.hpp>
#include <mgg_msgs/srv/plan_objective.hpp>
#include <mgg_msgs/srv/planner_srv.hpp>

#include "mgg_core/geofence_manager.h"
#include "mgg_core/gain.h"
#include "mgg_core/global_graph.h"
#include "mgg_core/graph_expansion.h"
#include "mgg_core/graph_manager.h"
#include "mgg_core/graph_merge.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/path_selection.h"
#include "mgg_core/ground_projection.h"
#include "mgg_core/params.h"
#include "mgg_core/random_sampler.h"
#include "mgg_core/sensor_params.h"
#include "mgg_map_octomap/mola_map.h"
#include "mgg_map_octomap/octomap_map.h"

namespace mgg_ros {

class PlannerNode : public rclcpp::Node {
  friend class PlannerNodeTestPeer;

 public:
  explicit PlannerNode(const rclcpp::NodeOptions& options);

  /// PlannerSrv status values beyond the FORWARD path. Negative so they
  /// cannot collide with the upstream constants.
  static constexpr int kStatusNotReady = -1;      // no odometry or no map
  static constexpr int kStatusNoPath = -2;        // this cycle found none
  static constexpr int kStatusComplete = -3;      // nothing left to explore

 private:
  void loadParameters();
  void onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void onMappingSnapshot(mgg_msgs::msg::MappingSnapshot::ConstSharedPtr msg);
  void onNeighbourGraph(mgg_msgs::msg::Graph::ConstSharedPtr msg);
  void onCoordinationExclusions(
      geometry_msgs::msg::PoseArray::ConstSharedPtr msg);
  void onPeerBodies(geometry_msgs::msg::PoseArray::ConstSharedPtr msg);
  void onBuildRequest(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  /// The exploration service: one cycle, the chosen path whole.
  void onPlanRequest(
      const std::shared_ptr<mgg_msgs::srv::PlannerSrv::Request> request,
      std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response);
  /// Navigate and Return Home: a route over the global graph, whole.
  void onObjectiveRequest(
      const std::shared_ptr<mgg_msgs::srv::PlanObjective::Request> request,
      std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> response);
  void publishOwnGraph();
  void publishPath();
  void publishMarkers();

  /// Builds the local grid graph around the current state, scores it and
  /// selects the best path into best_path_. Returns a summary for the log.
  std::string buildLocalGraph();
  /// Dijkstra over the global graph to the best frontier (rrg.cpp:5559
  /// Rrg::runGlobalPlanner), or to `target_id` when the current global
  /// repositioning is resumed. Fills best_path_; returns false with a reason
  /// when no route exists.
  bool runGlobalPlanner(int target_id, std::string& reason);
  /// Dijkstra over the global graph from the robot to `goal`, linking both
  /// ends into the graph first: the goal stands for the vertex within
  /// `goal_tolerance` of it, or (tolerance zero, or none there) gets its own
  /// checked vertex at the exact goal. Returns the route in `path`.
  bool routeOverGlobalGraph(const mgg::StateVec& goal, double goal_tolerance,
                            std::vector<mgg::StateVec>& path,
                            std::string& reason);
  /// Straightens a route where the map vouches for the straight segment and
  /// resamples it at path_interpolation_distance (rrg.cpp:4160 and 4176).
  void shortcutAndResample(std::vector<mgg::StateVec>& path);

  /// The roadmap side of the cycle: the accepted exploration path and the
  /// frontier clusters of the local graph join the global graph.
  void addRefPathToGraph(const std::vector<mgg::StateVec>& path);
  void addFrontiers();
  /// rrg.cpp:5247 timerCallback: every kOdoUpdateMinLength of travel the
  /// robot's state joins the global graph wired to every reachable
  /// neighbour; every kMinLength it is recorded and event E1 marks the
  /// roadmap around it visited.
  void ingestOdometryIntoGlobalGraph();
  /// The root of the global graph is home: the first odometry, dropped onto
  /// the terrain once the map shows ground under it.
  void seedGlobalGraph();
  /// rrg.cpp:2535 expandGlobalGraphTimerCallback, idle while its inputs
  /// (graph, map, robot position) are unchanged.
  void expandGlobalGraphTimerCallback();

  /// A ground robot's state at driving height above mapped ground. False
  /// when the map shows no ground under it.
  bool projectToDrivingHeight(mgg::StateVec& state) const;
  /// Peer reservations and refused leaves, as points the selectors skip.
  std::vector<Eigen::Vector3d> selectionExclusions();
  mgg::RecomputeGainFn globalFrontierGain();
  void refreshMapRevision();
  mgg::MolaMap::ReadLease mapReadLease() const;

  mgg::ExpandContext makeContext();
  mgg::GainContext makeGainContext();
  /// makeContext for the roadmap: no lattice inclinations, unknown space
  /// blocks an edge.
  mgg::ExpandContext makeGlobalContext();
  mgg::Vertex* findGlobalVertex(int id) const;

  // Core state. None of these know about ROS.
  std::unique_ptr<mgg::MapInterface> map_;
  /// The map backends behind map_: exactly one is non-null.
  mgg::OctomapMap* cloud_map_ = nullptr;
  mgg::MolaMap* mola_map_ = nullptr;
  std::unique_ptr<mgg::GroundProjection> ground_;
  std::unique_ptr<mgg::GeofenceManager> geofence_;
  std::shared_ptr<mgg::GraphManager> local_graph_;
  std::shared_ptr<mgg::GraphManager> global_graph_;
  std::unique_ptr<mgg::StaticPoseSource> poses_;
  mgg::RandomSampler random_sampler_;
  mgg::RobotStateHistory robot_state_hist_;

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

  std::string map_backend_ = "cloud_octomap";
  mgg::StateVec current_state_ = mgg::StateVec::Zero();
  bool have_odometry_ = false;
  std::int64_t last_odometry_stamp_ns_ = 0;
  std::chrono::steady_clock::time_point last_odometry_received_{};
  /// A plan from where the robot was is completed at once by the controller
  /// where the robot is; refuse to plan on odometry older than this.
  double odometry_stale_s_ = 5.0;

  /// Point clouds arrive in the sensor frame and have to be placed in the
  /// world frame before they go into the map. TF, rather than the odometry
  /// pose, because it is the only thing that knows where the sensor is
  /// mounted: rays have to be carved from the sensor, not from the robot's
  /// origin.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  /// How long to wait for the transform matching a cloud's stamp.
  double cloud_tf_timeout_sec_ = 0.1;

  /// The MOLA product the planner reads and the transform placing it in the
  /// planning frame, from the latest MappingSnapshot heartbeat.
  bool have_mapping_snapshot_ = false;
  mgg_msgs::msg::MappingSnapshot mapping_snapshot_;
  std::uint64_t observed_map_generation_ = 0;
  /// Counts map changes; the idle gates compare it.
  std::uint64_t map_revision_ = 0;
  /// Counts global graph changes; the idle gates compare it.
  std::uint64_t graph_revision_ = 0;

  double global_vertex_spacing_ = 1.0;
  /// Where odometry last joined the global graph and was last recorded
  /// (rrg.h last_state_marker_ and last_state_marker_global_).
  mgg::StateVec last_state_marker_ = mgg::StateVec::Zero();
  mgg::StateVec last_state_marker_global_ = mgg::StateVec::Zero();
  bool global_root_supported_ = false;

  /// Below this displacement the robot is standing still for the expansion
  /// sampler: its other inputs are the graph and map revisions.
  static constexpr double kOdometryStillM = 1e-3;
  std::uint64_t expansion_graph_revision_ = 0;
  std::uint64_t expansion_map_revision_ = 0;
  mgg::StateVec expansion_state_ = mgg::StateVec::Zero();
  /// rrg.cpp:2548: nothing to grow before the first plan.
  int planner_trigger_count_ = 0;

  /// rrg.cpp:2098 to 2120: rounds without a frontier among the local
  /// leaves; at the configured count the global planner runs.
  int low_gain_rounds_ = 0;
  int auto_global_planner_low_gain_rounds_ = 15;
  /// rrg.cpp:5838 to 5843 and 1229 to 1240: the global frontier being
  /// driven to, kept until the robot is within global_frontier_reach_m.
  bool global_exploration_ongoing_ = false;
  int current_global_vertex_id_ = -1;
  double global_frontier_reach_m_ = 5.0;
  /// See the parameter's comment in the constructor.
  bool allow_unknown_lattice_body_ = false;
  /// The last cycle's frontier paths join the global graph before that
  /// graph is rebuilt (rrg.cpp:121 Rrg::reset).
  bool add_frontiers_to_global_graph_ = false;

  /// Frontiers reserved by peers, in the planning frame, and how long a
  /// message stays in force.
  std::vector<Eigen::Vector3d> coordination_exclusions_;
  std::chrono::steady_clock::time_point coordination_exclusions_received_{};
  bool have_coordination_exclusions_ = false;
  double reservation_exclusion_radius_m_ = 4.0;
  double reservation_exclusion_ttl_s_ = 3.0;
  double peer_body_radius_m_ = 0.6;
  double peer_body_ttl_s_ = 3.0;

  /// Heading the robot has been travelling, for the direction penalty.
  double exploring_direction_ = 0.0;
  mgg::EdgeInclinations edge_inclinations_;
  /// Last chosen path, in world coordinates.
  std::vector<mgg::StateVec> best_path_;
  /// Whether best_path_ is a global-graph route (already in the roadmap) or
  /// a lattice path (joins it once accepted).
  bool best_path_from_global_graph_ = false;
  /// Points before and after shortcutting, reported so it is visible whether
  /// the smoothing did anything.
  int path_shortcut_from_ = 0;
  /// Corners left after shortcutting, before resampling puts points back.
  int path_shortcut_corners_ = 0;
  int path_shortcut_to_ = 0;
  std::string world_frame_ = "world";
  double communication_range_ = 10.0;

  struct MergeEvent {
    rclcpp::Time stamp;
    int sender_id = 0;
    Eigen::Vector3d our_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d their_pos = Eigen::Vector3d::Zero();
  };
  std::vector<MergeEvent> recent_merges_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<mgg_msgs::msg::MappingSnapshot>::SharedPtr
      mapping_snapshot_sub_;
  rclcpp::Subscription<mgg_msgs::msg::Graph>::SharedPtr neighbour_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
      coordination_exclusions_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
      peer_bodies_sub_;
  rclcpp::Publisher<mgg_msgs::msg::Graph>::SharedPtr graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr build_srv_;
  rclcpp::Service<mgg_msgs::srv::PlannerSrv>::SharedPtr plan_srv_;
  rclcpp::Service<mgg_msgs::srv::PlanObjective>::SharedPtr objective_srv_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
  /// rrg.h:367 global_graph_update_timer_.
  rclcpp::TimerBase::SharedPtr global_graph_update_timer_;
  /// One-shot guard against use_sim_time with no /clock.
  rclcpp::TimerBase::SharedPtr sim_time_check_;

  /// Reentrant, so the planning service and the subscriptions can run
  /// concurrently under a MultiThreadedExecutor. See the note in main().
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

}  // namespace mgg_ros

#endif  // MGG_ROS_PLANNER_NODE_H_
