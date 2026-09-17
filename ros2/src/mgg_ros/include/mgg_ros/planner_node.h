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

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mgg_msgs/msg/graph.hpp>
#include <mgg_msgs/msg/mapping_snapshot.hpp>
#include <mgg_msgs/srv/plan_objective.hpp>
#include <mgg_msgs/srv/query_map_batch.hpp>
#include <mgg_msgs/srv/refine_objective_route.hpp>
#include <mgg_msgs/srv/validate_objective_route.hpp>
#include <mgg_msgs/srv/planner_srv.hpp>

#include "mgg_core/geofence_manager.h"
#include "mgg_core/gain.h"
#include "mgg_core/graph_expansion.h"
#include "mgg_core/graph_manager.h"
#include "mgg_core/graph_merge.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/grid_refinement.h"
#include "mgg_core/path_selection.h"
#include "mgg_core/planning_stages.h"
#include "mgg_core/ground_projection.h"
#include "mgg_core/params.h"
#include "mgg_core/sensor_params.h"
#include "mgg_map_octomap/mola_map.h"
#include "mgg_map_octomap/octomap_map.h"

namespace mgg_ros {

class PlannerNodeTestPeer;

class PlannerNode : public rclcpp::Node {
 public:
  explicit PlannerNode(const rclcpp::NodeOptions& options);

 private:
  friend class PlannerNodeTestPeer;
  void loadParameters();
  void onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void onNeighbourGraph(mgg_msgs::msg::Graph::ConstSharedPtr msg);
  void onCoordinationExclusions(geometry_msgs::msg::PoseArray::ConstSharedPtr msg);
  void onMappingSnapshot(mgg_msgs::msg::MappingSnapshot::ConstSharedPtr msg);
  void onBuildRequest(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  /// The planning service proper: run a cycle and hand back the chosen path.
  void onPlanRequest(
      const std::shared_ptr<mgg_msgs::srv::PlannerSrv::Request> request,
      std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response);
  // Planning is synchronous and serialized with map updates. The ROS service
  // cannot cancel an in-progress graph build; callers must enforce a bounded
  // wait and may retry against the returned revisions.
  void onObjectiveRequest(
      const std::shared_ptr<mgg_msgs::srv::PlanObjective::Request> request,
      std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> response);
  void onRefineObjectiveRoute(
      const std::shared_ptr<mgg_msgs::srv::RefineObjectiveRoute::Request> request,
      std::shared_ptr<mgg_msgs::srv::RefineObjectiveRoute::Response> response);
  struct CachedObjectiveRoute;
  mgg::FeasiblePath refineCorridor(
      const mgg::RouteCorridor& corridor,
      const mgg::GridRefinementLimits* limits = nullptr);
  /// Everything one objective needs to ask the topological stage again after
  /// a corridor has been marked blocked. Shared by Explore, Navigate and Home.
  struct TopologicalRetry {
    bool valid = false;
    mgg::GraphManager* graph = nullptr;
    mgg::StateVec current = mgg::StateVec::Zero();
    mgg::PlanningRequest request;
    double goal_tolerance = 1.0;
    double minimum_partial_progress = 0.0;
  };
  /// Slices a complete topological route into the next bounded local section
  /// and, when the route continues past it, prepares the continuation token.
  /// `corridor` carries the full route in and the local section out.
  bool sliceObjectiveRouteWindow(
      const mgg::PlanningRequest& core, mgg::RouteCorridor& corridor,
      std::vector<mgg::StateVec>& global_objective_path,
      std::unique_ptr<CachedObjectiveRoute>& pending_objective_route);
  /// Which indexed-authority relaxations an objective may use.
  struct ObjectiveIndexedFlags {
    bool height_refinement = false;
    bool prefix_truncation = false;
    bool bounded_unknown_tail = false;
  };
  ObjectiveIndexedFlags objectiveIndexedQueryFlags(
      mgg::ObjectiveKind objective) const;
  /// Monotonic seconds for the blocked-corridor memory. Deliberately not the
  /// ROS clock: these marks must expire even when /clock stops.
  static double steadyNowSeconds();
  mgg::BlockedCorridorView blockedCorridorView() const;
  /// Marks one topological segment blocked against the live map revision.
  void blockCorridorSegment(const mgg::StateVec& from, const mgg::StateVec& to);
  /// Marks the cached route's segment closest to a hazard the controller or
  /// the route validator reported. Bounded by the cached route length.
  void blockCachedRouteNear(const Eigen::Vector3d& hazard);
  /// Marks the segment of `route` closest to `hazard`.
  void blockRouteNear(const std::vector<mgg::StateVec>& route,
                      const Eigen::Vector3d& hazard);
  void convertPathToNavigationBase(mgg::FeasiblePath& path) const;
  struct IndexedQueryContext {
    mgg::StateVec route_start = mgg::StateVec::Zero();
    Eigen::Isometry3d component_from_navigation = Eigen::Isometry3d::Identity();
    Eigen::Vector3d body = Eigen::Vector3d::Zero();
    Eigen::Vector3d physical_size = Eigen::Vector3d::Zero();
    Eigen::Vector3d center_offset = Eigen::Vector3d::Zero();
    mgg::RobotType robot_type = mgg::RobotType::kGroundRobot;
    double max_step_height = 0.0;
    double max_inclination = 0.0;
    double graph_to_base = 0.0;
    double max_provisional_ground_prefix = 0.0;
    bool observed_ground_body_evidence = false;
    bool provisional_unknown_ground = false;
    bool preserve_physical_start_height = false;
    bool have_mapping_snapshot = false;
    bool mapping_snapshot_fresh_at_capture = false;
    mgg_msgs::msg::MappingSnapshot mapping_snapshot;
  };
  IndexedQueryContext indexedQueryContext() const;
  /// Validates a refined route against the indexed terrain authority.
  ///
  /// \param allow_prefix_truncation qualified Explore may shorten the route to
  ///   its last fully checked measured sample after any rejection.
  /// \param allow_bounded_unknown_tail qualified Navigate/Home may retain a
  ///   bounded unmeasured tail behind physical provenance.
  /// \param allow_continuable_prefix Navigate/Home may shorten the route to its
  ///   last fully checked measured sample when the only rejection is that the
  ///   terrain ahead is unmeasured or lies beyond the provisional connector
  ///   bound. A hazard farther along the route shortens the section to the
  ///   last checked sample at least hazard_prefix_standoff_m before it; a
  ///   hazard nearer than that still fails closed.
  /// \param truncated_to_validated_prefix set when the emitted route is a
  ///   strict prefix of the requested one, so the caller resumes its committed
  ///   objective from that exact endpoint instead of the requested horizon.
  bool queryIndexedMap(mgg::FeasiblePath& path,
                       const IndexedQueryContext& context,
                       bool allow_height_refinement = false,
                       bool allow_prefix_truncation = false,
                       bool allow_bounded_unknown_tail = false,
                       bool allow_continuable_prefix = false,
                       bool* truncated_to_validated_prefix = nullptr,
                       Eigen::Vector3d* hazard_ahead = nullptr);
  void publishOwnGraph();
  void publishPath();
  void publishMarkers();

  /// Builds the local grid graph around the current state. Returns a summary
  /// suitable for a service response.
  std::string buildLocalGraph(bool strict_projected_endpoints = false);

  /// Extends the global topological graph with the robot's current pose.
  ///
  /// The global graph is the sparse, persistent one that robots exchange: a
  /// root, the trajectory, and (once gain evaluation is ported) frontiers.
  /// Without the trajectory backbone there is nothing to broadcast and no
  /// geometry for a neighbour's graph to rendezvous with.
  void updateGlobalGraph();
  void stageGlobalBreadcrumbs(const mgg::StateVec& state);
  bool projectStateToDrivingHeight(mgg::StateVec& state,
                                   bool preserve_xy = false,
                                   bool accept_ground_above_sample = false) const;
  bool resolveNavigateGoalDrivingHeight(mgg::StateVec& state) const;
  mgg::StateVec physicalAnchorAtDrivingHeight(
      const mgg::StateVec& base_pose) const;
  bool validateObjectiveStartSupport(
      const mgg::StateVec& anchor, const mgg::StateVec& supported,
      std::vector<mgg::StateVec>& checked,
      bool tolerate_unknown_body = false) const;
  /// Body-volume verdicts for the shared refinement stage.
  ///
  /// `tolerate_unknown` reproduces graph_expansion's lattice policy, which
  /// admits an edge whose body volume is not fully observed. Exploration must
  /// keep it: a frontier is by definition adjacent to unknown space, so a
  /// strict body volume would stop a robot from ever approaching one. Occupied
  /// space is still a veto, and unknown is never recorded as free.
  mgg::VoxelStatus objectiveBodyStatus(const Eigen::Vector3d& center,
                                       const Eigen::Vector3d& body,
                                       bool tolerate_unknown = false) const;
  mgg::VoxelStatus objectiveSweptBodyStatus(
      const Eigen::Vector3d& from, const Eigen::Vector3d& to,
      const Eigen::Vector3d& body, bool tolerate_unknown = false) const;
  bool objectiveFootprintTerrainSupported(
      const Eigen::Vector3d& driving_pose,
      const Eigen::Vector3d& body) const;
  mgg::GridProjectionStatus objectiveFootprintTerrainStatus(
      const Eigen::Vector3d& driving_pose,
      const Eigen::Vector3d& body) const;
  bool objectiveTerrainPathSupported(
      const std::vector<Eigen::Vector3d>& driving_path) const;
  static bool retainTerrainSafeExplorationPath(
      const std::vector<mgg::StateVec>& selected_lattice_path,
      const std::function<bool(const std::vector<Eigen::Vector3d>&)>&
          terrain_supported,
      std::vector<mgg::StateVec>& candidate);
  void refreshMolaRevision();
  void onValidateObjectiveRoute(
      const std::shared_ptr<mgg_msgs::srv::ValidateObjectiveRoute::Request>& request,
      std::shared_ptr<mgg_msgs::srv::ValidateObjectiveRoute::Response> response);

  mgg::ExpandContext makeContext();
  mgg::GainContext makeGainContext();

  // Core state. None of these know about ROS.
  std::unique_ptr<mgg::MapInterface> map_;
  mgg::OctomapMap* cloud_map_ = nullptr;
  mgg::MolaMap* mola_map_ = nullptr;
  std::string map_backend_ = "cloud_octomap";
  bool observed_ground_body_evidence_ = false;
  bool provisional_unknown_ground_ = false;
  std::string mission_id_;
  std::uint64_t observed_mola_generation_ = 0;
  std::unique_ptr<mgg::GroundProjection> ground_;
  std::unique_ptr<mgg::GeofenceManager> geofence_;
  std::shared_ptr<mgg::GraphManager> local_graph_;
  std::shared_ptr<mgg::GraphManager> global_graph_;
  std::unique_ptr<mgg::StaticPoseSource> poses_;

  mgg::RobotParams robot_params_;
  mgg::PlanningParams planning_params_;
  mgg::GridGraphParams grid_params_;
  mgg::GridRefinementLimits grid_refinement_limits_;
  mgg::GridRefinementLimits objective_grid_limits_;
  double objective_grid_max_margin_m_ = 4.0;
  double partial_route_min_progress_m_ = 1.0;
  /// Distance a committed section keeps from a hazard found farther along its
  /// route. The hazard was seen from the section start, often tens of metres
  /// away; the checked samples before it are as valid as any other route, and
  /// from their end the next section sees the hazard up close and routes
  /// around it. Zero restores refusing the whole section.
  double hazard_prefix_standoff_m_ = 1.5;
  /// Measurement tolerance added to the platform step limit when footprint
  /// ground heights are compared. Map points are quantised at centimetre
  /// scale, so a kerb exactly at the limit otherwise flips between admitted
  /// and refused on noise (measured on Bistro: refusals at 0.151 m and
  /// 0.152 m against a 0.150 m limit). It is a statement about measurement
  /// resolution, not about what the platform can climb, and stays well below
  /// the margin between the limit and a physically refused step.
  double footprint_step_tolerance_m_ = 0.01;
  double objective_start_support_max_distance_m_ = 3.0;
  double objective_route_horizon_m_ = 8.0;
  double objective_route_progress_tolerance_m_ = 1.0;
  std::size_t objective_route_max_poses_ = 4096;
  /// Corridors whose bounded refinement or execution has just been rejected.
  /// Bounded and expiring; it only steers corridor choice and never relaxes a
  /// terrain, body, step, drop or geofence veto.
  mgg::BlockedCorridorRegistry blocked_corridors_;
  /// Exploration selection handed to the shared topological stage. The
  /// utility/gain selector still chooses the target; the stage plans the
  /// corridor to it.
  mgg::StateVec explore_root_ = mgg::StateVec::Zero();
  mgg::StateVec explore_target_ = mgg::StateVec::Zero();
  bool have_explore_selection_ = false;
  mutable std::string objective_start_support_failure_;
  mutable std::string objective_footprint_failure_;
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
  struct CachedObjectiveRoute {
    std::string id;
    std::string mission_id;
    std::string component_id;
    mgg::ObjectiveKind objective = mgg::ObjectiveKind::kNavigate;
    std::uint64_t graph_revision = 0;
    mgg::PlanningGoal exact_goal;
    std::vector<mgg::StateVec> global_poses;
    std::size_t next_index = 0;
    mgg::StateVec expected_endpoint = mgg::StateVec::Zero();
  };
  std::unique_ptr<CachedObjectiveRoute> cached_objective_route_;
  std::string route_instance_id_;
  std::uint64_t route_sequence_ = 0;
  std::uint64_t objective_request_generation_ = 0;
  /// First finite navigation pose.  This is the mission home landmark and is
  /// latched before mapping or commanded motion can move the current pose.
  mgg::StateVec initial_state_ = mgg::StateVec::Zero();
  bool have_initial_state_ = false;
  /// The root exists immediately, but no edge may attach to it until mapped
  /// ground support has corrected it to the graph's driving-height convention.
  bool initial_anchor_supported_ = false;
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
  struct PendingGlobalBreadcrumb {
    mgg::StateVec state = mgg::StateVec::Zero();
    double path_length = 0.0;
  };
  /// Raw navigation-frame trajectory samples waiting for mapped support.
  /// Values, rather than graph pointers, keep ownership with GraphManager.
  std::deque<PendingGlobalBreadcrumb> pending_global_breadcrumbs_;
  std::size_t pending_global_max_samples_ = 128;
  double pending_global_max_length_m_ = 128.0;
  std::size_t pending_global_drain_max_samples_ = 32;
  double pending_global_length_m_ = 0.0;
  mgg::StateVec global_sampling_anchor_ = mgg::StateVec::Zero();
  bool have_global_sampling_anchor_ = false;
  mgg::StateVec last_global_odometry_ = mgg::StateVec::Zero();
  Eigen::Vector3d last_global_motion_ = Eigen::Vector3d::Zero();
  int last_own_global_vertex_id_ = 0;
  bool global_backbone_blocked_on_map_ = false;
  std::uint64_t global_backbone_blocked_map_revision_ = 0;
  bool global_backbone_blockage_reported_ = false;
  bool global_backbone_history_lost_ = false;
  std::string global_backbone_history_lost_reason_;
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
  std::string component_id_ = "local";
  std::uint64_t graph_revision_ = 0;
  std::uint64_t local_graph_revision_ = 0;
  std::uint64_t local_graph_map_revision_ = 0;
  std::uint64_t map_revision_ = 0;
  double reservation_exclusion_radius_m_ = 4.0;
  double reservation_exclusion_ttl_s_ = 3.0;
  /// Exploration leaves whose final route was refused, with the time of the
  /// refusal. The gain selector is deterministic: without this it picks the
  /// same leaf again, the same sample is refused again, and the robot sits at
  /// "planner returned no path" for good (robot_2, 2026-09-17, thirteen
  /// identical refusals in three minutes). Refused leaves are excluded from
  /// selection, like a peer's reservation, until they expire.
  std::deque<std::pair<Eigen::Vector3d, double>> rejected_explore_leaves_;
  double rejected_explore_leaf_ttl_s_ = 60.0;
  std::optional<Eigen::Vector3d> selected_explore_leaf_;
  std::vector<Eigen::Vector3d> coordination_exclusions_;
  std::chrono::steady_clock::time_point coordination_exclusions_received_;
  bool have_coordination_exclusions_ = false;
  mgg_msgs::msg::MappingSnapshot mapping_snapshot_;
  std::chrono::steady_clock::time_point mapping_snapshot_received_;
  bool have_mapping_snapshot_ = false;
  std::string indexed_map_query_service_;
  double indexed_map_query_timeout_s_ = 1.0;
  double indexed_map_snapshot_ttl_s_ = 3.0;
  double indexed_map_sample_spacing_m_ = 0.20;
  double indexed_map_max_roughness_m_ = 0.10;
  // Half of the currently pinned 0.20 m indexed-grid resolution. QueryMapBatch
  // does not transport resolution, so deployments must change this parameter
  // together with the provider resolution.
  double indexed_map_ground_tolerance_m_ = 0.10;
  // Zero disables ROS-clock source-age expiry. A keyframe timestamp binds the
  // snapshot but does not advance while a healthy robot is stationary.
  double indexed_map_max_source_age_s_ = 0.0;
  Eigen::Isometry3d component_from_navigation_ = Eigen::Isometry3d::Identity();
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
  rclcpp::Subscription<mgg_msgs::msg::Graph>::SharedPtr neighbour_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
      coordination_exclusions_sub_;
  rclcpp::Subscription<mgg_msgs::msg::MappingSnapshot>::SharedPtr
      mapping_snapshot_sub_;
  rclcpp::Publisher<mgg_msgs::msg::Graph>::SharedPtr graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr build_srv_;
  rclcpp::Service<mgg_msgs::srv::PlannerSrv>::SharedPtr plan_srv_;
  rclcpp::Service<mgg_msgs::srv::PlanObjective>::SharedPtr objective_srv_;
  rclcpp::Service<mgg_msgs::srv::RefineObjectiveRoute>::SharedPtr
      refine_objective_route_srv_;
  rclcpp::Service<mgg_msgs::srv::ValidateObjectiveRoute>::SharedPtr
      validate_objective_route_srv_;
  rclcpp::Client<mgg_msgs::srv::QueryMapBatch>::SharedPtr indexed_map_client_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
  /// One-shot guard against use_sim_time with no /clock.
  rclcpp::TimerBase::SharedPtr sim_time_check_;

  /// Reentrant, so the planning service and the subscriptions can run
  /// concurrently under a MultiThreadedExecutor. See the note in main().
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  /// Serializes service-side graph/refinement work without occupying executor
  /// threads on planner_mutex_ while an indexed query is in flight.
  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
};

}  // namespace mgg_ros

#endif  // MGG_ROS_PLANNER_NODE_H_
