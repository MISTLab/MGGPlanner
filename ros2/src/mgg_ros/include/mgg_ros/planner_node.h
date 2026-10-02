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

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mgg_msgs/msg/graph.hpp>
#include <mgg_msgs/msg/mapping_snapshot.hpp>
#include <mgg_msgs/msg/planner_config_state.hpp>
#include <mgg_msgs/srv/plan_objective.hpp>
#include <mgg_msgs/srv/planner_set_exploration_target.hpp>
#include <mgg_msgs/srv/planner_set_exploration_region.hpp>
#include <mgg_msgs/srv/planner_srv.hpp>
#include <mgg_msgs/msg/tour_award.hpp>
#include <mgg_msgs/msg/tour_bid.hpp>
#include <mgg_msgs/srv/release_claims.hpp>

#include "mgg_core/departure.h"
#include "mgg_core/geofence_manager.h"
#include "mgg_core/gain.h"
#include "mgg_core/global_graph.h"
#include "mgg_core/graph_expansion.h"
#include "mgg_core/graph_manager.h"
#include "mgg_core/graph_merge.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/no_go_zones.h"
#include "mgg_core/path_selection.h"
#include "mgg_core/ground_projection.h"
#include "mgg_core/params.h"
#include "mgg_core/random_sampler.h"
#include "mgg_core/sensor_params.h"
#include "mgg_core/trajectory.h"
#include "mgg_core/frontier_clusters.h"
#include "mgg_core/tour_costs.h"
#include "mgg_core/tour_params.h"
#include "mgg_core/tour_planner.h"
#include "mgg_core/fleet_coordinator.h"
#include "mgg_map_octomap/mola_map.h"
#include "mgg_ros/keyframe_trajectory.h"

namespace mgg {
// Declared only: the OctoMap backend is compiled in only when mgg_map_octomap
// was built with MGG_WITH_OCTOMAP.
class OctomapMap;
}  // namespace mgg

namespace mgg_ros {

class PlannerNode : public rclcpp::Node {
  friend class PlannerNodeTestPeer;

 public:
  explicit PlannerNode(const rclcpp::NodeOptions& options);
  /// Attach latency-critical inputs to an executor that has its own thread.
  /// These groups deliberately do not follow add_node() onto the planner pool.
  void addInputCallbackGroupsTo(rclcpp::Executor& executor);

  /// PlannerSrv status values beyond the FORWARD path. Negative so they
  /// cannot collide with the upstream constants.
  static constexpr int kStatusNotReady = -1;      // waiting for inputs
  static constexpr int kStatusNoPath = -2;        // this cycle found none
  static constexpr int kStatusComplete = -3;      // nothing left to explore


  /// What set a rebuild of the global graph off; each has its own rate
  /// limit, so one kind failing does not hold another back.
  enum class RoadmapRebuildTrigger {
    kSeedOnly = 0,
    kPoseUnlinkable = 1,
    kPathUnlinkable = 2,
  };
  static constexpr int kRoadmapRebuildTriggers = 3;

 private:
  void loadParameters();
  void applyLatestOdometry();
  void applyLatestOdometryImpl();
  void publishPlanningStatus();
  void setAcquiringObservations(bool acquiring);
  void onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void onMappingSnapshot(mgg_msgs::msg::MappingSnapshot::ConstSharedPtr msg);
  void onNeighbourGraph(mgg_msgs::msg::Graph::ConstSharedPtr msg);
  void onNeighbourTransforms(tf2_msgs::msg::TFMessage::ConstSharedPtr msg);
  /// Sets the merge's transform to `sender` from the latest one received for
  /// its planning frame, or clears it when there is none within the TTL.
  /// Always true for the static source.
  bool refreshNeighbourTransform(int sender, const std::string& sender_frame);
  /// Disconnects every merged roadmap whose transform is no longer current,
  /// before a plan uses the graph and before a merge: a withdrawn or expired
  /// placement must not keep an old roadmap routable.
  void withdrawUnplacedNeighbours();
  /// Merges a neighbour's roadmap with its current transform, as a graph
  /// message does; bumps the graph revision on any change.
  mgg::MergeResult mergeNeighbourRoadmap(const mgg::GraphExchange& incoming);
  /// A quarantined roadmap whose neighbour's transform is current again is
  /// merged again from the last roadmap received from it
  /// (neighbour_roadmaps_). In run 8 robot_1's peers were quarantined when
  /// a long global search outlasted the transform TTL; the transforms came
  /// back at once, but the peers were beyond communication_range, no
  /// roadmap of theirs was merged again, and robot_1 declared exploration
  /// complete with none of their frontiers.
  void readmitQuarantinedNeighbours();
  /// Updates the turn-back hysteresis from the path a plan request sends.
  void recordSentPath();
  void enforceSafeCompletion(bool& complete);
  /// The last nets on best_path_: the standing-start arrival band, no-go
  /// zones and peer bodies. Clears a path one refuses; returns why.
  std::string clearInadmissibleBestPath();
  std::string rememberReverseExit();
  std::string retainReverseExit(const std::vector<mgg::StateVec>& path, bool departure);
  // True means attempted; an empty output path is a refusal. Does not alter
  // exploration state or consume the stored corridor. A refusal that no
  // peer's moving on can lift (heading, swept clearance, or a reverse edge
  // that fails without peers too) is named in unusable_here.
  bool validateStoredReverseExit(const mgg::StateVec& start,
      std::vector<mgg::StateVec>& path, std::string& note,
      std::string* unusable_here = nullptr);
  void keepReverseDeparture(const std::vector<mgg::StateVec>& path);
  void retainEntryPastRefuge(const std::vector<mgg::StateVec>& reverse);
  bool reverseExitEndpointExcluded(const mgg::StateVec& pose) const;
  /// Within the radius of a given-up exclusion: unreachable from here.
  bool reverseExitEndpointGivenUp(const mgg::StateVec& pose) const;
  /// Records one retention refusal of each end (a path's end, a global
  /// target): a new exclusion, or a refused one again with a fresh TTL and
  /// one more refusal counted. Returns what was given up, for the summary.
  std::string excludeReverseExitEndpoints(const std::vector<Eigen::Vector3d>& ends);
  void excludeReverseExitEndpoint(const Eigen::Vector3d& endpoint) {
    excludeReverseExitEndpoints({endpoint});
  }
  /// Forgets the exclusions once the robot is more than
  /// kReverseExitExclusionMoveM from where the first was made, or the map
  /// in service is another component or epoch.
  void expireReverseExitExclusions();
  bool tryStoredReverseExit(const mgg::StateVec& start, std::string& note);
  bool storedReverseExitApplies(const mgg::StateVec& pose) const;
  bool currentPoseNeedsStoredExit() const;
  bool reverseExitShortcutAdmissible(const mgg::PathType& points);
  bool endpointNeedsReverseExit(const mgg::StateVec& pose) const;
  // Travel order, current/candidate pose first and refuge last. Objective
  // findDeparture keeps its slope semantics but still needs the room band.
  bool refugeArrivalBandAdmissible(const std::vector<mgg::StateVec>& path,
                                    bool require_level = true) const;
  bool reverseExitEdge(const mgg::GroundProjection& ground,
                       const mgg::StateVec& from, const mgg::StateVec& to) const;
  /// reverseExitEdge with no peer bodies: whether only a peer blocks it.
  bool reverseExitEdgeWithoutPeers(const mgg::StateVec& from, const mgg::StateVec& to) const;
  void forgetReverseExitIfOffRoute();
  /// Whether a route, the robot's pose first, starts with a sharp turn
  /// (kSharpTurnRad, measured over the robot's length from its heading)
  /// where the robot has no room to turn: the route a boxed-in robot is
  /// not sent.
  bool startPathAfterChassisSpin(std::vector<mgg::StateVec>& path,
                                 bool lattice_route);
  bool routeStartsWithTurnWithoutRoom(
      const std::vector<Eigen::Vector3d>& points);
  /// Why exploration may not be declared complete although no frontier is
  /// left to go to, or empty: a quarantined neighbour roadmap, whose
  /// frontiers are out of every search until its transform returns, a
  /// global search cut short by its time budget, or one that found a
  /// frontier it could not route to.
  std::string completionWithheld() const;
  /// This robot's platform as a neighbour's roadmap is re-read for it.
  mgg::ReceiverPlatform receiverPlatform() const;
  /// Attaches a goal with no mapped ground under it to the nearest vertex of
  /// another robot's roadmap within kLinkRadius in xy: that robot drove
  /// there, which is the evidence of traversable ground. The goal keeps its
  /// x and y and takes the vertex's height; the edge is refused only through
  /// space this robot's map knows to be occupied. Only `reachable` vertices
  /// (from where the robot joins the graph) are candidates, so a nearer
  /// disconnected part of a roadmap cannot hide a reachable one.
  mgg::Vertex* attachGoalToNeighbourRoadmap(const mgg::StateVec& goal,
                                            const mgg::UsableVertexFn& reachable);
  void onCoordinationExclusions(
      geometry_msgs::msg::PoseArray::ConstSharedPtr msg);
  void onPeerBodies(geometry_msgs::msg::PoseArray::ConstSharedPtr msg);
  /// Places the robot must not drive into (no_go_zones: SwarmDeck marks
  /// where a robot tripped its tilt guard), in the planning frame. Each
  /// message replaces the set; an empty one clears it. Each position is a
  /// disc of PlanningParams::no_go_radius_m, of unbounded height, kept
  /// apart from the peer bodies: with the mola_snapshot backend the map
  /// reports it occupied to the lattice, path ends and every sweep (a sweep
  /// leaving one it starts in excepted), and on every backend the global
  /// graph's searches leave out the edges through it (noGoBlocksEdge).
  void onNoGoZones(geometry_msgs::msg::PoseArray::ConstSharedPtr msg);
  /// no_go_discs: PoseArray, position.x/y = centre, position.z = already
  /// body-inflated centre-line reach in metres (NOT altitude). Orientation is
  /// ignored. Same frame, lifetime and replacement rules as no_go_zones,
  /// but independent: neither topic clears the other, no extra inflation.
  void onNoGoDiscs(geometry_msgs::msg::PoseArray::ConstSharedPtr msg);
  /// scouting_exclusions: soft scouting preferences, not obstacles, encoded
  /// as no_go_discs (x/y centre, z centre-line reach, orientation ignored),
  /// in the planning frame. Each message replaces the set; an empty one
  /// clears it; a wrong frame or any malformed entry refuses the whole
  /// message (the set in force is kept). The set lapses
  /// scouting_exclusion_ttl_s after its receipt. Exploration only: tour,
  /// cluster, lifted and greedy targets, the lattice's viewpoints and
  /// exploration paths; never objectives (Navigate, Return Home).
  void onScoutingExclusions(geometry_msgs::msg::PoseArray::ConstSharedPtr msg);
  /// Drops the set once its TTL has passed since receipt.
  void refreshScoutingExclusions();
  /// Whether an exploration path keeps to the exclusions' rule (XY only,
  /// mgg::NoGoZones::pathAdmissible): from inside one it leaves with the
  /// distance to its centre never decreasing until out, then never enters
  /// one, and it does not end inside one. Counts a refusal.
  bool scoutingPathAdmissible(const std::vector<mgg::StateVec>& path);
  /// Whether an exploration target at `p` lies inside an exclusion.
  bool scoutingExcludes(const Eigen::Vector3d& p) const;
  /// Sets no_go_ from no_go_zones_, its reach the zone radius plus half
  /// the robot's planning box.
  void refreshNoGoZones();
  /// Whether a path, driven from its first pose, keeps out of the no-go
  /// zones (mgg::NoGoZones::pathAdmissible): the last check on every path
  /// and route sent, on every backend.
  bool noGoAdmissible(const std::vector<mgg::StateVec>& path);
  /// Whether a no-go zone closes a global graph edge
  /// (mgg::NoGoZones::blocksEdge): only an outward departure of the robot
  /// from a zone it stands in stays open.
  bool noGoBlocksEdge(const mgg::Vertex& a, const mgg::Vertex& b) const;
  /// The same for a straight segment, such as a shortcut.
  bool noGoBlocksSegment(const Eigen::Vector3d& from,
                         const Eigen::Vector3d& to) const;
  /// Advances peer_generation_ when the peer bodies in force (a request's
  /// pinned set, else those published and not expired) differ from those
  /// it last saw, centres compared on a kPeerGenerationCellM grid: a peer
  /// appearing, leaving, expiring or moving at all. Run when the tour's
  /// costs are read, which are cached by it with the graph revision.
  void refreshPeerGeneration();
  /// Whether a peer body closes the straight segment from `from` to `to`,
  /// driven that way: the sweep the lattice's edges take
  /// (mgg::MolaMap::transientDiscsBlockSweep, the outward departure from a
  /// peer's reach left open) of the robot's planning box. Aerial spec-aware
  /// cylinders work on every backend. Never while peer_edges_open_.
  struct PeerBodyPin;
  bool peerBlocksSegment(const Eigen::Vector3d& from,
                         const Eigen::Vector3d& to) const;
  /// What closes a segment for peerBlocksSegment, counted in
  /// aerial_counters_.
  enum class PeerBlock { kNone, kAerialCylinder, kLegacyDisc };
  PeerBlock peerBlockingSegment(const Eigen::Vector3d& from,
                                const Eigen::Vector3d& to) const;
  /// The aerial features' counters and home status as one JSON object,
  /// logged as "aerial_status {...}" (throttled) after an aerial plan.
  std::string aerialStatusJson() const;
  void onAerialPeerBodies(geometry_msgs::msg::PoseArray::ConstSharedPtr msg);
  /// Pins the peer bodies in force now in `pin`, on this thread, for the
  /// rest of a plan or objective request (PeerBodyPin):
  /// every map query, roadmap search and route check of the request sees
  /// this one set, and onPeerBodies, serialised with planning, publishes
  /// the next only after it. Ground discs still require mola_snapshot.
  void pinPeerBodies(std::optional<PeerBodyPin>& pin);
  /// Whether `path`, driven from its first pose, keeps clear of the peer
  /// bodies (peerBlocksSegment): the last check on every path and route
  /// sent, as noGoAdmissible is for the zones.
  bool peerAdmissible(const std::vector<mgg::StateVec>& path) const;
  /// Whether any peer body is in force (a request's pinned set, or those
  /// published and not expired): without, no peer diagnosis runs.
  bool peersInForce() const;
  /// A peer diagnosis: the roadmap search from `source_id` with the peers'
  /// edges open, stopped at peer_diagnosis_deadline_, or else
  /// global_search_time_budget_s from now (review r0, I4). False when cut
  /// short, which sets peer_diagnosis_cut_short_.
  bool diagnosePeerSearch(int source_id, mgg::ShortestPathsReport& rep);
  /// The global graph's edge test (GraphManager::setEdgeBlocked): a no-go
  /// zone or a peer body closes the edge from `a` to `b` for this search,
  /// and the roadmap keeps it. Records each edge a peer closes in
  /// peer_blocked_edges_.
  bool globalEdgeBlocked(const mgg::Vertex& a, const mgg::Vertex& b);
  void onBuildRequest(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  /// The exploration service: one cycle, the chosen path whole.
  void cancelPlanning();
  void applyPendingCancel();
  void cancelExplorationPlanning();
  bool onPlanRequestImpl(
      const std::shared_ptr<mgg_msgs::srv::PlannerSrv::Request> request,
      std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response);
  void onObjectiveRequestImpl(
      const std::shared_ptr<mgg_msgs::srv::PlanObjective::Request> request,
      std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> response);
  void onPlanRequest(
      const std::shared_ptr<mgg_msgs::srv::PlannerSrv::Request> request,
      std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response);
  /// Navigate and Return Home: a route over the global graph, whole, ending
  /// at the requested goal. Return Home routes to vertex 0 only when its
  /// goal is not finite.
  void onObjectiveRequest(
      const std::shared_ptr<mgg_msgs::srv::PlanObjective::Request> request,
      std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> response);
  /// Sets or clears exploration_target_.
  void onExplorationTargetRequest(
      const std::shared_ptr<mgg_msgs::srv::PlannerSetExplorationTarget::Request>
          request,
      std::shared_ptr<mgg_msgs::srv::PlannerSetExplorationTarget::Response>
          response);
  /// "Explore here" (drone scout §3.5): gain and tour only inside a box.
  void onExplorationRegionRequest(
      const std::shared_ptr<mgg_msgs::srv::PlannerSetExplorationRegion::Request>
          request,
      std::shared_ptr<mgg_msgs::srv::PlannerSetExplorationRegion::Response>
          response);
  /// Leave the auction on landing (true), rejoin after take-off (false),
  /// drone scout design §4.4.
  void onLeaveFleet(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  /// Metres of flight left for exploring and coming home (drone scout
  /// §4.3), from the drone's adapter; +inf until one arrives.
  void onFlightReach(const std_msgs::msg::Float64::SharedPtr msg);
  /// A drone's flight state from SwarmDeck's adapter (its supervisor's
  /// state: "landed", "flying", ...), kept as latest_flight_state_.
  void onFlightState(const std_msgs::msg::String::SharedPtr msg);
  /// `clusters` without those outside the exploration region, if one is set.
  std::vector<mgg::FrontierCluster> insideExplorationRegion(
      std::vector<mgg::FrontierCluster> clusters) const;
  /// This robot's global graph as broadcast on neighbour_graph_out.
  mgg_msgs::msg::Graph ownGraphMessage();
  void publishOwnGraph();
  void publishPath();
  void publishPathUnderCancellationFence();
  void publishMarkers();
  /// Snapshot the applied configuration and publish it with planner_mutex_
  /// held by the caller. No-op setters retain the last change's version/time.
  void publishPlannerConfigState();

  /// Builds the local grid graph around the current state, scores it and
  /// selects the best path into best_path_. Returns a summary for the log.
  std::string buildLocalGraph();
  /// A departure for a ground robot boxed in at `start`, at driving height:
  /// mgg::findDeparture on this robot's map. Returns false, with `path`
  /// empty, when there is no way out.
  bool straightDeparture(const mgg::StateVec& start, mgg::Departure& departure,
                         bool arrival_band = false);
  /// The robot is boxed in at `root_state`, its pose at driving height as
  /// the lattice root takes it: it has no room to turn where it stands, and
  /// `why`, for the log, says why the path it would otherwise be sent
  /// starts with a turn it cannot make. best_path_ becomes its departure
  /// (straightDeparture), or empty when it has none, which sets
  /// boxed_in_without_departure_now_. Counts and logs the outcome; returns
  /// the note for the plan summary.
  std::string departBoxedIn(const mgg::StateVec& root_state, const char* why, bool consult_stored = true);
  /// Dijkstra over the global graph to the best frontier (rrg.cpp:5559
  /// Rrg::runGlobalPlanner), or to `target_id` when the current global
  /// repositioning is resumed. Fills best_path_; returns false with a reason
  /// when no route exists, or when the best frontier's discounted gain is
  /// under `min_gain` (a low-gain lattice path is handed over only for a
  /// frontier worth low_gain_handoff_min_voxels).
  bool runGlobalPlanner(int target_id, std::string& reason,
                        double min_gain = 0.0);
  /// Dijkstra over the global graph from the robot to `goal`, linking both
  /// ends into the graph first: the goal stands for the vertex within
  /// `goal_tolerance` of it, or (tolerance zero, or none there) gets its own
  /// checked vertex at the exact goal. Returns the route in `path`, under
  /// the turn rule (applyRouteTurnRule), and in `turns_ok` the check its
  /// shortcut must keep passing. `goal` is taken by value: linking the
  /// robot may rebuild the graph, which would free a goal read from a
  /// vertex.
  bool routeOverGlobalGraph(mgg::StateVec goal, double goal_tolerance,
                            std::vector<mgg::StateVec>& path,
                            mgg::PathOkFn& turns_ok, std::string& reason);
  /// A ground robot's route to a goal turns sharply only where it may, as an
  /// exploration path does (mgg::chooseTurnCompliantRoute): `route` runs
  /// through `graph` from where the robot joins it to the goal, after
  /// `lead_in`, the robot's pose when it is off the first vertex. The slope
  /// is measured from the map's ground with `slope_from_map` (the global
  /// graph, whose vertices a metre apart seldom span a plane within the
  /// robot's length), and from `graph`'s vertices otherwise (a lattice).
  /// `route_name` names the route in the log. Returns the check the
  /// route's shortcut must keep passing; empty for other robots.
  mgg::PathOkFn applyRouteTurnRule(mgg::GraphManager& graph,
                                   bool slope_from_map,
                                   const std::vector<mgg::StateVec>& lead_in,
                                   std::vector<mgg::Vertex*>& route,
                                   const char* route_name);
  /// Straightens a route where the map vouches for the straight segment and
  /// resamples it at path_interpolation_distance (rrg.cpp:4160 and 4176).
  /// With `turns_ok`, a route that passes it still passes afterwards: the
  /// shortcut takes only leaps that keep it, and a resampled route that
  /// fails it is replaced by the route as it came. `corridor_ok` is mandatory
  /// even when the original path is a sharp-turn fallback. A ground
  /// robot's leap is checked as a lattice edge driven that way
  /// (mgg::groundShortcutSegmentAdmissible) under the unknown-space policy
  /// of the graph the route came from: `lattice_route` admits the unknown
  /// body volume the local lattice admits, otherwise space must be observed
  /// free as on the roadmap. A leap may not pass closer to a hazard than the
  /// edges it replaces (mgg::shortcutPathKeepingClearance).
  void shortcutAndResample(std::vector<mgg::StateVec>& path,
                           const mgg::PathOkFn& turns_ok = nullptr,
                           const mgg::PathOkFn& corridor_ok = nullptr,
                           bool lattice_route = false);

  /// The roadmap side of the cycle: the accepted exploration path and the
  /// frontier clusters of the local graph join the global graph.
  void addRefPathToGraph(const std::vector<mgg::StateVec>& path);
  void addFrontiers();
  /// rrg.cpp:5247 timerCallback: every kOdoUpdateMinLength of travel the
  /// robot's state joins the global graph wired to every reachable
  /// neighbour; every kMinLength it is recorded and event E1 marks the
  /// roadmap around it visited.
  void ingestOdometryIntoGlobalGraph();
  /// The root is home: the current odometry at seeding, dropped onto the
  /// terrain once mapped. An aerial anchor waits for flight_state or its
  /// timeout; only "landed" lifts it (home_seeded_landed_).
  void seedGlobalGraph();
  /// A drone's first flight_state arriving after home was seeded on the
  /// wait's timeout: "landed" re-roots home aerial_home_height_m over the
  /// pose, as a timely one would have seeded it, when the drone has not
  /// moved since (kStandingStartMoveM). Home's old edges are cut; later
  /// keyframe rebuilds lift their first keyframe (home_seeded_landed_).
  void rerootHomeOnLateLandedState();
  /// Replaces the global graph with one rebuilt from the robot's keyframe
  /// trajectory (mgg::rebuildRoadmapFromTrajectory), vertex 0 at its home
  /// keyframe, when the trajectory is for the map in service and home has
  /// mapped ground. The old graph's frontiers are not carried over: the
  /// rebuilt graph's come from exploration, and merged neighbours' with
  /// their next broadcast; how many of its own it dropped is kept
  /// (frontiers_dropped_in_rebuild_). With `link_what_failed`, the rebuilt
  /// graph replaces the old one only when it links what the old one could
  /// not (the robot's pose, an exploration path), which it may add to it;
  /// the function returns the vertex that links it, or null. For an aerial
  /// robot, nor does it replace an old graph whose home reaches its other
  /// vertices when it would cut home off from that vertex (or, without
  /// one, from every other vertex) or split places the old graph connects
  /// to home (rebuildLosesHome; roadmap_rebuilds_refused_). Otherwise the old graph
  /// is kept. `why`, for the log, says what triggered it. At most once per
  /// roadmap_rebuild_min_interval_s_ for each trigger, and not again on the
  /// same trajectory revision and map. Returns true when the graph was
  /// replaced.
  bool rebuildGlobalGraphFromKeyframes(
      RoadmapRebuildTrigger trigger, const char* why,
      const std::function<mgg::Vertex*(mgg::GraphManager&)>& link_what_failed =
          nullptr);
  /// Whether an own in-service vertex of the global graph lies within
  /// edge_length_max of `state`: the graph reaches there, and a link
  /// refused there (a wedged robot's box) is no reason to rebuild it.
  bool globalGraphReaches(const mgg::StateVec& state) const;
  /// This robot's own vertices in the global graph.
  std::size_t ownGlobalVertices() const;
  /// rrg.cpp:2535 expandGlobalGraphTimerCallback, idle while its inputs
  /// (graph, map, peer bodies, robot position) are unchanged.
  void expandGlobalGraphTimerCallback();
  void expandGlobalGraphTimerCallbackImpl();

  /// A ground robot's state at driving height above mapped ground. False
  /// when the map shows no ground under it.
  bool projectToDrivingHeight(mgg::StateVec& state) const;
  /// As projectToDrivingHeight for a goal, whose height is only a hint: the
  /// ground nearest it above or below (GroundProjection::projectGoal).
  bool projectGoalToDrivingHeight(mgg::StateVec& state) const;
  /// A ground robot's state at driving height above the floor its base
  /// stands on, for where the map shows no ground yet.
  mgg::StateVec physicalAnchorAtDrivingHeight(const mgg::StateVec& base_pose) const;
  /// A ground robot stands at its start when it has not moved
  /// kStandingStartMoveM since its first odometry and its keyframe
  /// trajectory, which outlives the planner process, shows it never left:
  /// the trajectory is for the map in service, none of its keyframes lies
  /// kStandingStartMoveM or more from its home keyframe, and the robot
  /// stands within that of home. A planner restarted mid-run, whose first
  /// odometry is wherever the robot then is, so does not count it as
  /// standing at its start (review r0, I-1). Then the disk of
  /// hanging_root_edge_length_max round where it stood counts as observed
  /// ground (mgg::StandingStart) for its lattice, turns and departures.
  /// nullopt otherwise: without a keyframe trajectory source (the rebuild
  /// turned off, or a backend without keyframes), without a trajectory to
  /// read, and without a hanging_root_edge_length_max. Once the trajectory
  /// shows the robot left, it never stands at its start again.
  // Nested entry points share one lazy snapshot; the outer plan owns its
  // lifetime, including cached absence and the unreadable-source diagnostic.
  struct StandingStartScope {
    explicit StandingStartScope(PlannerNode& node);
    ~StandingStartScope();
    PlannerNode& node;
  };
  std::optional<mgg::StandingStart> standingStart();
  std::optional<mgg::StandingStart> readStandingStart();
  std::shared_ptr<const mgg::KnownFreeBodyVolumes> ownBodyKnownFree();
  std::shared_ptr<const mgg::KnownFreeBodyVolumes> plan_own_body_known_free_;
  int standing_start_scope_depth_ = 0;
  std::optional<std::optional<mgg::StandingStart>> plan_standing_start_;
  // Directed geometry, shared by selection and every shortcut trial. Never
  // reused across requests or map generations; the request pins peers.
  mutable std::map<std::array<double, 6>, bool> plan_reverse_edges_;
  mutable std::uint64_t plan_reverse_generation_ = 0;
  // Ends (path ends and global targets) whose reverse-exit retention was
  // refused. They accumulate while the robot stays put, so selection cannot
  // alternate between two refused candidates (run 14). Each lasts
  // kReverseExitExclusionPlans plan requests, then is judged again by normal
  // selection and retention; refused kReverseExitExclusionGiveUpRefusals
  // times from here it is given up until the robot moves or the map epoch
  // changes, and no longer withholds completion.
  static constexpr double kReverseExitExclusionMoveM = 0.5;
  static constexpr std::uint64_t kReverseExitExclusionPlans = 10;
  static constexpr int kReverseExitExclusionGiveUpRefusals = 3;
  struct ReverseExitExclusion {
    Eigen::Vector3d position;
    std::uint64_t excluded_through_request = 0;  // plan_requests_ value
    int refusals = 1;
    bool given_up = false;
  };
  bool reverseExitExclusionActive(const ReverseExitExclusion& exclusion) const {
    return exclusion.given_up || plan_requests_ <= exclusion.excluded_through_request;
  }
  // Plan requests that planned (past the readiness checks).
  std::uint64_t plan_requests_ = 0;
  std::vector<ReverseExitExclusion> reverse_exit_exclusions_;
  Eigen::Vector2d reverse_exit_exclusions_anchor_ = Eigen::Vector2d::Zero();
  std::pair<std::string, std::uint64_t> reverse_exit_exclusions_map_;
  bool standingStartGoalAdmissible(const mgg::StateVec& goal);
  /// keyframe_source_->read. A failure is an ERROR, logged at once and then
  /// at most every kKeyframeReadErrorPeriodS until a read succeeds: without
  /// its keyframes a robot has neither a standing start nor a roadmap
  /// rebuild (runs 9 and 10).
  bool readOwnKeyframes(KeyframeTrajectory& trajectory, std::string& error);
  /// T_navigation_component of the map in service (mapping_snapshot_).
  Eigen::Isometry3d navigationFromComponent() const;
  /// Dijkstra through a fresh local lattice from the robot to a goal inside
  /// the lattice box, the goal linked in with checked edges; under the turn
  /// rule as routeOverGlobalGraph.
  bool routeOverLocalLattice(const mgg::StateVec& goal,
                             std::vector<mgg::StateVec>& path,
                             mgg::PathOkFn& turns_ok, std::string& reason);
  /// Peer reservations and refused leaves, as points the selectors skip.
  std::vector<Eigen::Vector3d> selectionExclusions();
  mgg::RecomputeGainFn globalFrontierGain();
  void refreshMapRevision();
  mgg::MolaMap::ReadLease mapReadLease() const;

  mgg::ExpandContext makeContext(bool include_own_body = true);
  mgg::GainContext makeGainContext();
  /// makeContext for the roadmap: no lattice inclinations, unknown space
  /// blocks an edge.
  mgg::ExpandContext makeGlobalContext();
  mgg::Vertex* findGlobalVertex(int id) const;
  /// Links where the robot stands into the global graph, as a route out of
  /// here does (mgg::linkDeparture); null when nothing links.
  mgg::Vertex* linkRobotToGlobalGraph();
  /// Bumps graph_revision_ when the global graph's edges changed since the
  /// tour last looked: most writers bump it only for new vertices, and an
  /// edge alone can make a cluster reachable (GraphDistanceCache).
  void noteGlobalGraphEdges();
  /// Leaves the tour's `cluster` out of its solves, and releases it as the
  /// target, until the robot moves more than kTourSetAsideMoveM from here,
  /// its gain rises by more than kTourSetAsideGainRise of its scored gain
  /// now and by more than tour.min_cluster_gain, or `retry_s` passes: the
  /// robot
  /// could not be routed to it (tour.route_retry_s; less when only peer
  /// bodies were in the way), reached it with little local gain, or stands
  /// on its representative (the route's source, which the tour costs zero
  /// and no route leaves).
  /// At-target failures do not lapse on movement; consecutive failures
  /// double the retry up to 4x, until new gain or a progressing decision.
  void setTourClusterAside(const mgg::FrontierCluster& cluster,
                           double retry_s, bool at_target = false);
  /// Demotes this robot's frontiers a peer's visited vertex covers
  /// (mgg::demoteFleetCoveredFrontiers, fleet_coverage_radius_m). Runs
  /// before every choice of a frontier: the tour's clusters, the fleet
  /// bid's, and the greedy search's.
  void demoteFleetCoveredFrontiers();
  /// Every frontier cluster of the global graph, other robots' included,
  /// under its stable name (tour-exploration design §2.1).
  std::vector<mgg::FrontierCluster> globalFrontierClusters();
  /// For valuation only: a lifted target serves the aerial fleet scout,
  /// without changing the actual cluster's owner or auction identity.
  mgg::FrontierCluster tourValueCluster(const mgg::FrontierCluster& cluster) const;
  int capTourValues(mgg::TourCostMatrix& costs,
                    const std::vector<mgg::FrontierCluster>& clusters) const;
  /// Of `clusters`, those this robot's tour may visit: without fleet
  /// assignment, its own, or every robot's once none of its own is left
  /// (as kGlobalOtherRobotPenalty preferred them), less those a peer's
  /// reservation excludes.
  std::vector<mgg::FrontierCluster> tourCandidates(
      std::vector<mgg::FrontierCluster> clusters);
  /// §2.3: solves the tour again when due and returns its current target,
  /// or nothing when it has none. `note` is for the plan summary.
  std::optional<mgg::FrontierCluster> refreshTour(std::string& note);
  /// Graph distance from each cluster's representative to home (vertex 0),
  /// the way back, from the tour's distance cache under the same key as the
  /// tour's costs: graph revision and peer generation.
  std::vector<double> homeDistances(
      const std::vector<mgg::FrontierCluster>& clusters);
  /// An aerial robot's home (vertex 0) is the endpoint its return is costed
  /// to. When the roadmap from `robot_vertex_id` does not reach it, home is
  /// wired to its reachable neighbours (mgg::expandGraphEdges) once its body
  /// is observed free, as Return Home links its goal: a home linked before
  /// its air was seen would otherwise stay cut off, and every cluster fail
  /// the battery reach cap (drone-r3). At most once per graph, revision, map
  /// revision and peer generation; records home_link_status_. No-op for
  /// ground robots.
  void relinkAerialHome(int robot_vertex_id);
  /// Whether a repositioning to global vertex `vertex_id` still heads for
  /// the tour's target (always, when the tour is off or has no target). A
  /// resumed route skips refreshTour, so the target is checked against the
  /// graph and the peers' reservations as they are now: not when a peer
  /// reserved it or it no longer holds `vertex_id`.
  bool tourKeepsRoute(int vertex_id);
  /// The robot's pose, then its clusters' representatives in tour order.
  void publishTour();
  /// Fleet frontier assignment (tour-exploration design §3, §4). A peer's
  /// bid or award is placed in this robot's frame with the transform its
  /// roadmap would merge with, and dropped without one, when malformed, or
  /// from beyond communication_range, as a roadmap is. Calls and awards
  /// require an in-range bid heard within fleet.peer_timeout_s.
  void onTourBid(mgg_msgs::msg::TourBid::ConstSharedPtr msg);
  void onTourAward(mgg_msgs::msg::TourAward::ConstSharedPtr msg);
  /// §4 release 3: the operator releases a silent robot's claims on the
  /// group's auctioneer, which forwards the release in its next award.
  void onReleaseClaims(
      const std::shared_ptr<mgg_msgs::srv::ReleaseClaims::Request> request,
      std::shared_ptr<mgg_msgs::srv::ReleaseClaims::Response> response);
  /// One fleet step at `now_s`; fleet_timer_ calls it with the node's clock,
  /// read under planner_mutex_.
  void fleetTick(double now_s);
  /// This robot's bid content: every frontier cluster it knows, costed from
  /// where it joins the global graph (no heading penalty), its tour's
  /// target and when it took it, and the awarded clusters its roadmap shows
  /// explored.
  mgg::TourBidData ownTourBid();
  /// The auctioneer's estimate of a cost a bid does not give: the global
  /// graph distance between the vertices nearest the two points, plus the
  /// straight links to them.
  mgg::CostEstimateFn roadmapCostEstimate();
  /// mgg::exploredInGraph on this robot's global graph.
  mgg::ExploredFn exploredByRoadmap();
  /// The transform placing a peer's messages from `frame` in this robot's
  /// frame, as its roadmap is placed; false when there is none.
  bool peerTransform(int robot_id, const std::string& frame,
                     Eigen::Isometry3d& t_ours_theirs);
  /// Clusters other robots hold and clusters peers explored, for the greedy
  /// fallback's frontier search.
  std::vector<Eigen::Vector3d> fleetExclusions();
  /// §3.5 for a robot with no tour target and no local path. In a group it
  /// asks for an auction and gets no path until the award answers; if that
  /// award leaves it nothing, exploration is complete for it, with the
  /// failed global search's exceptions: local gain remains, or a graph
  /// rebuild dropped frontiers since (review r0, I-2). Alone, it
  /// takes over the claim of the robot silent longest. False when it is
  /// alone with no claim to take over: the low-gain rule decides.
  bool settleIdleRobot(std::string& summary, bool& complete);
  /// §2.4: whether the local path ending at `viewpoint` serves the tour's
  /// `target`. The lattice is laid out along the robot's heading
  /// (buildGridGraph), so both offsets from the robot are turned into its
  /// frame before mgg::localPathServesTarget compares them with its bounds;
  /// the target's reach is global_frontier_reach_m.
  bool localPathServesTour(const Eigen::Vector3d& viewpoint,
                           const Eigen::Vector3d& target) const;

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
  std::string neighbour_pose_source_ = "static";
  double neighbour_transform_ttl_s_ = 5.0;
  struct NeighbourTransform {
    Eigen::Isometry3d t_ours_theirs = Eigen::Isometry3d::Identity();
    std::chrono::steady_clock::time_point received{};
  };
  /// Keyed by the neighbour's planning frame.
  std::unordered_map<std::string, NeighbourTransform> neighbour_transforms_;
  /// Each neighbour robot id's planning frame, from its graph messages.
  std::unordered_map<int, std::string> neighbour_frames_;
  /// The last roadmap received within communication range from each
  /// neighbour, to re-admit it from when its transform returns.
  std::unordered_map<int, mgg::GraphExchange> neighbour_roadmaps_;
  std::vector<mgg::FrontierCluster> liftedPeerFrontiers();
  // Revalidate on every query; replace slots only when their accepted set
  // changes. Never roadmap anchors, own frontier evidence, or broadcasts.
  std::weak_ptr<mgg::GraphManager> lifted_target_graph_;
  std::vector<int> lifted_target_vertices_;
  struct LiftedTarget {
    mgg::FrontierCluster cluster;
    int anchor;
    double length;
  };
  std::vector<LiftedTarget> lifted_targets_;
  double aerial_frontier_height_m_ = 1.3;
  double aerial_min_height_m_ = 0.8;
  double aerial_max_height_m_ = 2.5;
  struct AerialMergeCache {
    mgg::GraphExchange snapshot;
    std::weak_ptr<mgg::GraphManager> graph;
    const mgg::MapInterface* map = nullptr;
    std::uint64_t map_revision = 0;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    Eigen::Vector3d body_size = Eigen::Vector3d::Zero();
    Eigen::Vector3d center_offset = Eigen::Vector3d::Zero();
    bool merged = false;
  };
  std::unordered_map<int, AerialMergeCache> aerial_merge_cache_;
  /// Neighbours whose roadmap was merged and is not in the current global
  /// graph: quarantined when its transform expired, or dropped by a roadmap
  /// rebuild. Kept across rebuilds; readmitQuarantinedNeighbours merges
  /// each cached roadmap again once its transform is current, and until
  /// then exploration is not complete.
  std::set<int> roadmaps_to_readmit_;
  /// The last global frontier search was cut short by
  /// global_search_time_budget_s (runGlobalPlanner), found a frontier or
  /// not.
  bool global_search_cut_short_ = false;
  /// The last global search found a frontier, or resumed one, and routing
  /// to it failed (runGlobalPlanner).
  bool global_frontier_not_routed_ = false;
  /// The global target whose route reverse-exit retention refused, kept
  /// apart from routing failures: it withholds completion only while a
  /// live, not-given-up exclusion covers it (completionWithheld).
  std::optional<Eigen::Vector3d> global_target_refused_by_retention_;
  /// The last global route failed its final progress check while the
  /// robot was already within reach_distance of the requested frontier.
  bool global_route_at_target_ = false;
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
  std::mutex input_mutex_;
  std::mutex cancellation_mutex_;
  std::atomic<bool> pending_cancel_clear_{false};
  std::atomic<std::uint64_t> request_generation_{0};
  // PCI stop cannot revoke NAVIGATE/RETURN_HOME, even when delivered late.
  std::atomic<std::uint64_t> exploration_generation_{0};
  std::atomic<std::uint64_t> cancellations_{0};
  std::atomic<bool> acquiring_observations_{true};
  std::atomic<std::int64_t> bootstrap_started_ns_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planning_status_pub_;
  rclcpp::TimerBase::SharedPtr planning_status_timer_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odometry_;
  nav_msgs::msg::Odometry::ConstSharedPtr applied_odometry_;
  std::chrono::steady_clock::time_point latest_odometry_received_;
  mgg_msgs::msg::MappingSnapshot::ConstSharedPtr latest_snapshot_;
  std::optional<mgg::MolaSnapshotRequest> served_map_identity_;
  std::atomic<std::uint64_t> map_identity_changes_{0};
  std::atomic<std::uint64_t> heartbeats_received_{0};
  std::atomic<std::uint64_t> heartbeats_during_planning_{0};
  std::atomic<bool> request_active_{false};
  std::shared_ptr<const mgg::MolaSnapshotRequest> last_planning_snapshot_;
  std::atomic<std::int64_t> odometry_sample_stamp_ns_{0};
  std::atomic<double> odometry_ingest_lag_s_{0.0};

  std::string map_backend_ = "cloud_octomap";
  mgg::StateVec current_state_ = mgg::StateVec::Zero();
  /// How far the robot is tilted from level, radians: its odometry's roll
  /// and pitch (mgg::PathTurnCheck::setRobotTilt).
  double current_tilt_ = 0.0;
  /// Where the first odometry placed the robot, until it moves
  /// kStandingStartMoveM from there (standingStart()).
  std::optional<Eigen::Vector2d> standing_start_xy_;
  bool left_standing_start_ = false;
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

  /// The robot's keyframe trajectory the global graph is rebuilt from, or
  /// null when there is none (a map backend without one, or rebuilding
  /// turned off).
  std::unique_ptr<KeyframeTrajectorySource> keyframe_source_;
  /// ERRORs logged since the node started because keyframe_source_ could
  /// not be read (readOwnKeyframes), and when the last was, while reads
  /// keep failing.
  static constexpr double kKeyframeReadErrorPeriodS = 60.0;
  int keyframe_read_errors_logged_ = 0;
  std::optional<std::chrono::steady_clock::time_point>
      keyframe_read_error_logged_at_;
  /// Where standingStart() last looked for keyframes it could not read,
  /// while the robot may still stand at its start; empty otherwise.
  std::string standing_start_unread_keyframes_;
  mgg::RoadmapRebuildParams roadmap_rebuild_params_;
  double roadmap_rebuild_min_interval_s_ = 10.0;
  /// Per trigger: whether and when it last tried, and the trajectory
  /// revision and map revision it tried with.
  bool roadmap_rebuild_attempted_[kRoadmapRebuildTriggers] = {false, false,
                                                              false};
  std::chrono::steady_clock::time_point
      last_roadmap_rebuild_attempt_[kRoadmapRebuildTriggers] = {};
  std::string last_roadmap_rebuild_inputs_[kRoadmapRebuildTriggers];
  /// Global graphs rebuilt from the trajectory since the node started.
  int roadmap_rebuilds_ = 0;
  /// Rebuilt graphs refused because they would have cut home off from what
  /// the rebuild was for, or split places the current graph connects to
  /// home (rebuildLosesHome; aerial robots only).
  int roadmap_rebuilds_refused_ = 0;
  /// This robot's in-service frontiers the last rebuild that dropped any
  /// replaced, until the next failed global search, which is then no path
  /// rather than exploration complete (review r0, I-2).
  int frontiers_dropped_in_rebuild_ = 0;

  /// Below this displacement the robot is standing still for the expansion
  /// sampler: its other inputs are the graph and map revisions.
  static constexpr double kOdometryStillM = 1e-3;
  std::uint64_t expansion_graph_revision_ = 0;
  /// The peer generation (refreshPeerGeneration) the last pass sampled
  /// against.
  std::uint64_t expansion_peer_generation_ = 0;
  std::uint64_t expansion_map_revision_ = 0;
  mgg::StateVec expansion_state_ = mgg::StateVec::Zero();
  /// rrg.cpp:2548: nothing to grow before the first plan.
  int planner_trigger_count_ = 0;

  /// rrg.cpp:2098 to 2120: rounds without a frontier among the local
  /// leaves, or whose best path scores under low_gain_voxels; at the
  /// configured count the global planner runs.
  int low_gain_rounds_ = 0;
  /// Whether the last lattice path sent turned back: the next selection
  /// then bounds no direction penalty. Reset when the plan sent something
  /// else, and on a new exploration target or an objective.
  mgg::TurnBackHysteresis turn_back_hysteresis_;
  /// This plan's lattice path as buildLocalGraph leaves it to be sent, and
  /// the direction it was scored against (recordSentPath).
  std::vector<mgg::StateVec> lattice_path_;
  double lattice_selection_direction_ = 0.0;
  /// Lattice paths chosen that turned back, since the node started.
  int paths_turning_back_ = 0;
  /// This cycle's lattice path scores under low_gain_voxels
  /// (buildLocalGraph resets it). Once the low-gain rounds are due, the
  /// global planner is consulted first and the path is kept only when it
  /// finds no route.
  bool low_gain_path_now_ = false;
  /// Low-gain lattice paths replaced by a global repositioning, since the
  /// node started.
  int low_gain_handoffs_ = 0;
  /// Exploration paths and global routes sent to end where no pose had
  /// viewpoint clearance (mgg::viewpointClear), since the node started.
  int unclear_viewpoints_selected_ = 0;
  /// Exploration paths sent although they turn sharply on a slope or
  /// without room to turn (mgg::PathTurnCheck), because no path complied,
  /// since the node started.
  int sharp_turn_fallbacks_ = 0;
  /// Exploration paths dropped because they went nowhere
  /// (mgg::pathGoesNowhere), since the node started.
  int paths_going_nowhere_ = 0;
  /// Poses of the last of them, for the log and tests.
  int last_nowhere_poses_ = 0;
  /// Routes to a goal (objectives and global repositioning) sent although
  /// they turn sharply on a slope or without room to turn, because no route
  /// complied (applyRouteTurnRule), since the node started.
  int route_sharp_turn_fallbacks_ = 0;
  /// Times the robot was found boxed in: no exploration path, or the global
  /// route kept in place of one, complied with the turn rule and it had no
  /// room to turn where it stood (departBoxedIn). It was sent a straight
  /// departure, or no path when it had none, since the node started.
  int boxed_in_departures_ = 0;
  int boxed_in_without_departure_ = 0;
  /// This cycle found the robot boxed in with no straight departure
  /// (departBoxedIn; buildLocalGraph resets it): no global repositioning is
  /// tried in its place.
  bool boxed_in_without_departure_now_ = false;
  /// This cycle's lattice still had gain to go to: a frontier, or a path
  /// that went nowhere only because it ended too near or unclear while the
  /// gain lay beyond (buildLocalGraph resets it). A failed global search
  /// is then no path, not exploration complete (review r0, I-2).
  bool local_gain_remains_now_ = false;
  /// The last route to a goal kept although it turns sharply where it may
  /// not, and its first turn, from the robot's heading, is sharp where the
  /// robot has no room to turn (applyRouteTurnRule): a turn it cannot make.
  bool last_route_starts_with_turn_without_room_ = false;
  /// The last route to a goal failed only because peer bodies closed every
  /// way there (routeOverGlobalGraph), or the last global frontier search
  /// found frontiers only behind them (runGlobalPlanner): a retryable
  /// outcome, not a missing route.
  bool last_route_blocked_by_peer_ = false;
  /// Global graph edges a peer body closed in the searches of this plan
  /// request (globalEdgeBlocked), as (lower id, higher id).
  std::set<std::pair<int, int>> peer_blocked_edges_;
  /// Of those, the ones an aerial peer cylinder closed (the rest: legacy
  /// discs).
  std::set<std::pair<int, int>> peer_cylinder_blocked_edges_;
  /// Cheap counters for the aerial features (mgg-home, drone-r3 left them
  /// unobservable). "last_" values are of the latest evaluation; the rest
  /// accumulate since start. Counts per evaluation, not per unique target:
  /// a plan request evaluates the candidates more than once.
  struct AerialCounters {
    /// liftedPeerFrontiers, its latest evaluation: peer frontier vertices
    /// it lifted, those refused (gain floor, the 64-slot cap, within the
    /// merge radius of a better one, no own vertex within 5 m, no
    /// observed-free link from one) and those admitted.
    int lifted_proposed = 0;
    int lifted_rejected_gain = 0;
    int lifted_rejected_cap = 0;
    int lifted_rejected_merged = 0;
    int lifted_rejected_no_anchor = 0;
    int lifted_rejected_link = 0;
    /// Inside a scouting exclusion.
    int lifted_rejected_scouting = 0;
    int lifted_admitted = 0;
    /// Senders whose roadmap had no current transform.
    int lifted_senders_unplaced = 0;
    /// Times the tour took a lifted target as its new target (a solve that
    /// keeps the same one is not counted again).
    std::uint64_t lifted_selected = 0;
    mgg::ClusterId lifted_selected_target = mgg::kNoCluster;
    /// aerial_peer_bodies messages accepted, refused for their frame or a
    /// malformed entry, and the frame of the last accepted one.
    std::uint64_t cylinder_messages_accepted = 0;
    std::uint64_t cylinder_messages_wrong_frame = 0;
    std::uint64_t cylinder_messages_invalid = 0;
    std::string cylinder_frame;
    /// Segment checks (roadmap edges in a search, path segments) a peer
    /// closed, by what closed them.
    mutable std::uint64_t cylinder_segment_blocks = 0;
    mutable std::uint64_t disc_segment_blocks = 0;
    /// Clusters the battery reach cap refused, those with no way home at
    /// all, and the last refusal's out and back distances and budget.
    std::uint64_t reach_cap_rejects = 0;
    std::uint64_t reach_cap_home_unreachable = 0;
    double last_reach_out_m = std::numeric_limits<double>::quiet_NaN();
    double last_reach_back_m = std::numeric_limits<double>::quiet_NaN();
    double last_reach_budget_m = std::numeric_limits<double>::quiet_NaN();
  };
  AerialCounters aerial_counters_;
  /// The reach-cap note for an aerial cluster refused by it, counted.
  std::string noteReachCapReject(double out_m, double back_m);
  /// Peer bodies are left out of peerBlocksSegment: set only to ask whether
  /// a failed search would have succeeded without them.
  bool peer_edges_open_ = false;
  /// While an objective is routed again without peers to tell whether they
  /// alone stopped it, every roadmap search stops here (diagnosePeerSearch).
  std::optional<std::chrono::steady_clock::time_point> peer_diagnosis_deadline_;
  /// A peer diagnosis was cut short since last reset.
  bool peer_diagnosis_cut_short_ = false;
  bool peer_diagnosis_in_progress_ = false;
  /// Peer diagnoses run since the node started.
  int peer_diagnoses_ = 0;
  /// Changes whenever the peer bodies in force change
  /// (refreshPeerGeneration): the tour's route costs are cached by it and
  /// the graph revision (review r0, I3). Its key: the centres, quantized
  /// and sorted, then the radius.
  std::uint64_t peer_generation_ = 0;
  std::vector<std::array<long, 2>> peer_generation_key_;
  /// Exploration paths sent unshortcut because the shortcut, once resampled,
  /// turned where the lattice path did not, since the node started.
  int shortcut_turn_reverts_ = 0;
  int auto_global_planner_low_gain_rounds_ = 15;
  /// rrg.cpp:5838 to 5843 and 1229 to 1240: the global frontier being
  /// driven to, kept until the robot is within global_frontier_reach_m.
  bool global_exploration_ongoing_ = false;
  int current_global_vertex_id_ = -1;
  double global_frontier_reach_m_ = 5.0;
  /// The controller's goal tolerance, metres: PCI's reach_distance. An
  /// exploration path ending this close to the robot goes nowhere.
  double reach_distance_ = 0.3;
  /// See the parameter's comment in the constructor.
  double ground_exploration_lattice_budget_s_ = 0.1;
  bool allow_unknown_lattice_body_ = false;
  std::string unknown_body_policy_ = "strict";
  std::string unknown_body_sensor_ = "VLP16";
  /// How long one request's lattice sweeps may take, seconds
  /// (lattice_time_budget_s); 0 is no bound.
  double lattice_time_budget_s_ = 0.5;
  /// While a request runs, where its lattice sweeps stop (makeContext's
  /// ExpandContext::deadline): the sweep keeps what it built by then.
  std::optional<std::chrono::steady_clock::time_point> lattice_deadline_;
  /// The last local lattice route's work (mgg::PlanProfile::summary), for
  /// the objective's log line.
  std::string local_route_profile_;
  double hanging_root_edge_length_max_ = 0.0;
  /// How far over its pad a drone's home is, metres: the height it takes
  /// off to, where its flight links (drone scout Task 17, fix round 1).
  /// Zero: home is where the first odometry is.
  double aerial_home_height_m_ = 0.0;
  /// With an aerial anchor, wait at most this many node-time seconds after
  /// first odometry for flight_state. While pending, no planner work runs.
  double aerial_home_state_wait_s_ = 5.0;
  std::optional<rclcpp::Time> home_state_wait_started_;
  rclcpp::TimerBase::SharedPtr home_state_wait_timer_;
  /// Home was seeded while flight_state was "landed" (a drone on its pad,
  /// with aerial_home_height_m_ set), so it is that high over the pose. The
  /// keyframe rebuild lifts its first keyframe on this decision alone.
  bool home_seeded_landed_ = false;
  /// Home was seeded on the flight_state wait's timeout, with no state yet,
  /// from home_seed_pose_: a late "landed" may still re-root it
  /// (rerootHomeOnLateLandedState), once.
  bool home_seeded_on_timeout_ = false;
  mgg::StateVec home_seed_pose_ = mgg::StateVec::Zero();
  /// relinkAerialHome's last inputs and outcome, and how often it re-rooted
  /// or relinked home.
  std::tuple<const mgg::GraphManager*, std::uint64_t, std::uint64_t,
             std::uint64_t>
      home_link_key_{nullptr, 0, 0, 0};
  std::string home_link_status_ = "unknown";
  int home_reroots_ = 0;
  int home_relink_attempts_ = 0;
  int home_relinks_ = 0;
  /// The last cycle's frontier paths join the global graph before that
  /// graph is rebuilt (rrg.cpp:121 Rrg::reset).
  bool add_frontiers_to_global_graph_ = false;
  /// Tour-based exploration (tour-exploration design §2): its parameters,
  /// the fleet's (cluster_merge_radius_m groups the clusters), the stable
  /// names of the global frontier clusters, the tour over them, and the
  /// graph distances it was costed with, one Dijkstra per cluster per graph
  /// revision.
  mgg::TourParams tour_params_;
  mgg::FleetParams fleet_params_;
  mgg::ClusterIdRegistry cluster_ids_;
  std::unique_ptr<mgg::TourPlanner> tour_planner_;
  mgg::GraphDistanceCache tour_distances_;
  /// The clusters the last refreshTour offered the tour.
  std::vector<mgg::FrontierCluster> tour_clusters_;
  /// Changes when the clusters this robot may visit change for a reason
  /// other than the graph (fleet assignment or exploration region).
  std::uint64_t tour_assignment_version_ = 0;
  /// Last fleet version incorporated above; never overwrite local changes.
  std::uint64_t tour_fleet_assignment_version_ = 0;
  /// The last tour's costing and solving time, for the plan summary.
  double tour_solve_ms_ = 0.0;
  /// Clusters the last tour solve left out as not worth their distance
  /// (capTourCostsByValue; fleet assignment off only).
  int tour_value_left_out_ = 0;
  /// The last tour solve without fleet assignment: each cluster the reach
  /// cap left in, at its distance from the robot then, and those of them
  /// worth it. Every refresh judges these again on the current gains
  /// (review r0, I-2).
  std::unordered_map<mgg::ClusterId, double> tour_value_distances_;
  std::set<mgg::ClusterId> tour_value_worth_;
  /// Fleet frontier assignment (tour-exploration design §3); null when
  /// fleet.enabled is false.
  std::unique_ptr<mgg::FleetCoordinator> fleet_;
  /// Calls/awards require a recent bid admitted by the radio-range filter.
  std::unordered_map<int, double> fleet_bid_received_s_;
  /// The target refreshTour last released as reached. A reached cluster
  /// that is still a frontier is released once: the free solve may take it
  /// again, and it is then kept until explored, reassigned or unroutable,
  /// rather than released and taken again every cycle, which would solve
  /// every cycle and restamp its claim (targetSince). Cleared once the
  /// target is another cluster.
  mgg::ClusterId tour_reached_cluster_ = mgg::kNoCluster;
  /// The global graph's edge count when the tour last looked
  /// (noteGlobalGraphEdges).
  int tour_graph_edges_ = -1;
  /// Clusters setTourClusterAside left out of the tour: where the robot
  /// stood, the cluster's gain and the time then, and for how long.
  struct TourSetAside {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double gain = 0.0;
    double at_s = 0.0;
    double retry_s = 0.0;
    bool at_target = false;
  };
  std::unordered_map<mgg::ClusterId, TourSetAside> tour_set_aside_;
  struct TourAtTargetFailure {
    double gain = 0.0;
    int retry_multiplier = 0;
  };
  /// Retained across retry expiry, pruned when a cluster disappears, gains
  /// new evidence, or gets a successful progressing tour decision.
  std::unordered_map<mgg::ClusterId, TourAtTargetFailure> tour_at_target_failures_;
  /// Tour targets the robot could not be routed to.
  int tour_routes_failed_ = 0;
  /// Targets already within tour reach, set aside instead of routed back to.
  std::uint64_t tour_in_reach_set_aside_ = 0;

  /// Frontiers reserved by peers, in the planning frame, and how long a
  /// message stays in force.
  std::vector<Eigen::Vector3d> coordination_exclusions_;
  std::chrono::steady_clock::time_point coordination_exclusions_received_{};
  bool have_coordination_exclusions_ = false;
  double reservation_exclusion_radius_m_ = 4.0;
  /// An own frontier within this of a peer's visited vertex, horizontally,
  /// and a walk of 1.5 times this from it (the roadmap, perhaps after one
  /// map-checked link of up to 1.5 m to a peer vertex), is covered by the
  /// fleet (mgg::demoteFleetCoveredFrontiers); zero disables.
  double fleet_coverage_radius_m_ = 3.0;
  /// Swept link checks one coverage pass may make
  /// (fleet_coverage_max_link_checks); the frontiers left over wait for the
  /// next pass.
  int fleet_coverage_max_link_checks_ = mgg::kFleetCoverageMaxLinkChecks;
  /// The nearest peer vertices one frontier's links may run to
  /// (fleet_coverage_max_links_per_frontier); the budget is at least this.
  int fleet_coverage_max_links_per_frontier_ =
      mgg::kFleetCoverageMaxLinksPerFrontier;
  /// fleet_coverage_max_link_checks was below it and was raised, with an
  /// error logged.
  bool fleet_coverage_link_budget_raised_ = false;
  /// Where the next coverage pass starts among this robot's frontiers.
  mgg::FleetCoverageCursor fleet_coverage_cursor_;
  /// The bound mode RobotParams were loaded with: coverage links are swept
  /// with its box whatever a request sets (review r3, P1).
  mgg::BoundModeType nominal_bound_mode_ = mgg::BoundModeType::kExtendedBound;
  double reservation_exclusion_ttl_s_ = 3.0;
  double peer_body_radius_m_ = 0.6;
  double aerial_peer_margin_m_ = 0.35;
  struct AerialPeerBody {
    Eigen::Vector3d top;
    double radius;
  };
  std::vector<AerialPeerBody> aerial_peer_bodies_;
  std::chrono::steady_clock::time_point aerial_peer_bodies_received_;
  bool have_aerial_peer_bodies_ = false;
  std::vector<std::array<double, 4>> aerial_peer_generation_key_;
  std::optional<std::vector<AerialPeerBody>> pinned_aerial_peer_bodies_;
  std::vector<AerialPeerBody> activeAerialPeerBodies() const;
  struct PeerBodyPin {
    explicit PeerBodyPin(PlannerNode& node);
    ~PeerBodyPin();
    PlannerNode& node;
    std::optional<std::vector<AerialPeerBody>> previous;
    std::optional<mgg::MolaMap::TransientDiscPin> ground;
  };
  double peer_body_ttl_s_ = 3.0;

  /// A goal the robot has no known route to yet, in the world frame. While
  /// set, local path selection is biased toward it instead of along the
  /// robot's heading, and global frontiers nearer to it rank higher. Soft: nothing is
  /// excluded, so a way round that first leads away stays open.
  std::optional<Eigen::Vector3d> exploration_target_;
  mgg::EdgeInclinations edge_inclinations_;
  /// Last chosen path, in world coordinates.
  std::vector<mgg::StateVec> best_path_;
  // Endpoint first, refuge last; headings stay those of forward entry.
  // Only associated with the last nonempty path actually sent.
  std::vector<mgg::StateVec> stored_reverse_exit_;
  std::vector<mgg::StateVec> reverse_exit_entry_path_;
  bool departure_sent_now_ = false;
  bool stored_reverse_sent_now_ = false;
  bool stored_reverse_retreating_ = false;
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
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr
      neighbour_transforms_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
      coordination_exclusions_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
      peer_bodies_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr aerial_peer_bodies_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
      no_go_zones_sub_;
  /// The no-go zones' centres, planning frame (onNoGoZones).
  std::vector<Eigen::Vector2d> no_go_zones_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr no_go_discs_sub_;
  std::vector<Eigen::Vector2d> no_go_disc_centres_;
  std::vector<double> no_go_disc_reaches_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
      scouting_exclusions_sub_;
  /// The scouting exclusions in force (onScoutingExclusions), received at
  /// scouting_exclusions_received_ (steady clock, MGG's receipt).
  std::vector<Eigen::Vector2d> scouting_exclusion_centres_;
  std::vector<double> scouting_exclusion_reaches_;
  std::chrono::steady_clock::time_point scouting_exclusions_received_;
  double scouting_exclusion_ttl_s_ = 3.0;
  mgg::NoGoZones scouting_zones_;
  /// Set while runGlobalPlanner routes: routeOverGlobalGraph then keeps to
  /// the scouting exclusions as well as the no-go zones.
  bool exploration_route_ = false;
  struct ScoutingExclusionCounters {
    std::uint64_t messages_accepted = 0;
    std::uint64_t messages_wrong_frame = 0;
    std::uint64_t messages_invalid = 0;
    std::uint64_t lapsed = 0;
    /// Tour clusters, lifted targets and greedy frontiers refused for
    /// lying inside one (per evaluation).
    std::uint64_t targets_refused = 0;
    /// Lattice vertices left out as viewpoints (per evaluation).
    std::uint64_t viewpoints_refused = 0;
    /// Exploration paths refused (lattice candidates, global routes, the
    /// plan's final check).
    std::uint64_t paths_refused = 0;
  };
  ScoutingExclusionCounters scouting_counters_;
  /// The same zones with their reach, as every check applies them.
  mgg::NoGoZones no_go_;
  rclcpp::Publisher<mgg_msgs::msg::Graph>::SharedPtr graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::Publisher<mgg_msgs::msg::PlannerConfigState>::SharedPtr
      planner_config_state_pub_;
  mgg_msgs::msg::PlannerConfigState planner_config_state_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tour_pub_;
  rclcpp::Publisher<mgg_msgs::msg::TourBid>::SharedPtr tour_bid_pub_;
  rclcpp::Publisher<mgg_msgs::msg::TourAward>::SharedPtr tour_award_pub_;
  rclcpp::Subscription<mgg_msgs::msg::TourBid>::SharedPtr tour_bid_sub_;
  rclcpp::Subscription<mgg_msgs::msg::TourAward>::SharedPtr tour_award_sub_;
  rclcpp::Service<mgg_msgs::srv::ReleaseClaims>::SharedPtr
      release_claims_srv_;
  /// Runs fleetTick: bids, calls and awards.
  rclcpp::TimerBase::SharedPtr fleet_timer_;
  static constexpr double kFleetTickPeriodS = 0.1;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr build_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_exploration_srv_;
  rclcpp::Service<mgg_msgs::srv::PlannerSrv>::SharedPtr plan_srv_;
  rclcpp::Service<mgg_msgs::srv::PlanObjective>::SharedPtr objective_srv_;
  rclcpp::Service<mgg_msgs::srv::PlannerSetExplorationTarget>::SharedPtr
      exploration_target_srv_;
  rclcpp::Service<mgg_msgs::srv::PlannerSetExplorationRegion>::SharedPtr
      exploration_region_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr leave_fleet_srv_;
  std::optional<mgg::BoundedSpaceParams> exploration_region_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr flight_reach_sub_;
  double flight_reach_m_ = std::numeric_limits<double>::infinity();
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr flight_state_sub_;
  /// The latest flight_state received; none until one arrives.
  std::optional<std::string> latest_flight_state_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
  /// rrg.h:367 global_graph_update_timer_.
  rclcpp::TimerBase::SharedPtr global_graph_update_timer_;
  /// One-shot guard against use_sim_time with no /clock.
  rclcpp::TimerBase::SharedPtr sim_time_check_;

  /// Reentrant, so the planning service and the subscriptions can run
  /// concurrently under a MultiThreadedExecutor. See the note in main().
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::CallbackGroup::SharedPtr input_callback_group_;
  rclcpp::CallbackGroup::SharedPtr snapshot_callback_group_;
  rclcpp::TimerBase::SharedPtr input_timer_;
  /// no_go_zones alone: its messages replace one another, so they are
  /// handled one at a time.
  rclcpp::CallbackGroup::SharedPtr no_go_zones_group_;
  /// peer_bodies alone, for the same reason (review r1, R3).
  rclcpp::CallbackGroup::SharedPtr peer_bodies_group_;
  /// flight_state alone, for the same reason.
  rclcpp::CallbackGroup::SharedPtr flight_state_group_;
};

}  // namespace mgg_ros

#endif  // MGG_ROS_PLANNER_NODE_H_
