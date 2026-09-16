// Vertex, edge and bookkeeping types for the exploration graphs.
//
// Ported from planner_common/graph_base.h. Three things are deliberately left
// behind at the ROS boundary:
//
//   * BoundingBoxType's geometry_msgs::Point overloads. Message conversion
//     belongs in mgg_ros.
//   * The ros::serialization::Serializer specialisations used by the
//     save-graph and load-graph services. Same reason.
//   * SemanticClass, which was a *message type* used purely for three
//     integer constants. It is a plain enum here.
//
// Every member is initialised. The ROS 1 originals left several undefined:
// Vertex::robot_id was never set by the constructor even though it keys the
// multi-robot graph merge, and BoundingBoxType's four vectors were undefined
// until setDefault() happened to be called.

#ifndef MGG_CORE_GRAPH_BASE_H_
#define MGG_CORE_GRAPH_BASE_H_

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/types.h"

namespace mgg {

/// Was planner_semantic_msgs/SemanticClass, a message carrying one int.
enum class SemanticClass : int {
  kNone = 0,
  kStaircase = 1,
  kDoor = 2,
};

/// Where a vertex stands in the exploration.
enum class VertexType {
  kUnvisited = 0,  ///< collision free and worth visiting, not yet visited
  kVisited = 1,
  kFrontier = 2,   ///< borders unknown space
};

enum class ExpandGraphStatus {
  kSuccess = 0,
  kErrorKdTree,
  kErrorCollisionEdge,
  kErrorShortEdge,
  kErrorGeofenceViolated,
  kNull,
};

enum class ConnectStatus {
  kSuccess = 0,
  kErrorCollisionAtSource,
  kErrorNoFeasiblePath,
};

/// What a viewpoint would reveal. Richer than GainCounts, which is only the
/// map layer's tally; this adds the planner's scoring on top.
struct VolumetricGain {
  double gain = 0.0;
  double accumulative_gain = 0.0;
  int num_unknown_voxels = 0;
  int num_free_voxels = 0;
  int num_occupied_voxels = 0;
  int num_unknown_surf_voxels = 0;
  bool is_frontier = false;
  std::vector<std::size_t> unseen_voxel_hash_keys;

  void reset() { *this = VolumetricGain(); }
};

/// An axis-aligned box that can be reset to a remembered default.
class BoundingBoxType {
 public:
  void setDefault(const Eigen::Vector3d& v_min, const Eigen::Vector3d& v_max) {
    min_val_default_ = v_min;
    max_val_default_ = v_max;
    min_val_ = v_min;
    max_val_ = v_max;
  }

  void reset() {
    min_val_ = min_val_default_;
    max_val_ = max_val_default_;
  }

  void set(const Eigen::Vector3d& v_min, const Eigen::Vector3d& v_max) {
    min_val_ = v_min;
    max_val_ = v_max;
  }

  void get(Eigen::Vector3d& v_min, Eigen::Vector3d& v_max) const {
    v_min = min_val_;
    v_max = max_val_;
  }

  const Eigen::Vector3d& min() const { return min_val_; }
  const Eigen::Vector3d& max() const { return max_val_; }

 private:
  // Initialised, unlike the ROS 1 version where reset() before setDefault()
  // copied undefined memory.
  Eigen::Vector3d min_val_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d max_val_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d min_val_default_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d max_val_default_ = Eigen::Vector3d::Zero();
};

struct Vertex {
  Vertex(int v_id, const StateVec& v_state) : id(v_id), state(v_state) {}

  /// Unique id (root is 0, others positive).
  int id = 0;
  /// x, y, z, yaw.
  StateVec state = StateVec::Zero();
  /// Which robot created this vertex. Keys the multi-robot merge, and was
  /// left uninitialised by the ROS 1 constructor.
  int robot_id = 0;
  VolumetricGain vol_gain;
  /// NBVP legacy, kept in case a tree is wanted for comparison.
  Vertex* parent = nullptr;
  std::vector<Vertex*> children;
  /// Distance to the root.
  double distance = 0.0;
  /// Leaf in the simplified tree derived from the graph.
  bool is_leaf_vertex = true;
  /// No ground plane beneath (legged robots only).
  bool is_hanging = false;
  VertexType type = VertexType::kUnvisited;
  int cluster_id = 0;
  /// Associated pose-graph node, the anchor a SLAM backend can deform.
  int pose_id = 0;
  /// Distance to the nearest obstacle.
  double dm = 0.0;
  SemanticClass semantic_class = SemanticClass::kNone;
};

/// Flattened vertex for saving a graph to disk.
struct SerializeVertex {
  int id = 0;
  StateVec state = StateVec::Zero();
  VolumetricGain vol_gain;
  int parent_id = 0;
  int type = 0;
};

struct ExpandGraphReport {
  ExpandGraphStatus status = ExpandGraphStatus::kNull;
  int num_vertices_added = 0;
  int num_edges_added = 0;
  Vertex* vertex_added = nullptr;
  /// Candidate edges rejected for exceeding max_inclination. The ROS 1 code
  /// counted these in a local that was never read; reporting it makes the
  /// "the robot is boxed in by slopes" case visible to the caller.
  int steep_edges = 0;
  /// Set when the candidate was rejected because no ground was found beneath
  /// it, as opposed to because an edge to it was blocked. Both report
  /// kErrorCollisionEdge, and they mean very different things: the first says
  /// the sensor never saw the floor there, the second that something is in
  /// the way.
  bool no_ground = false;
  /// Strict projected-endpoint verdict for explicit-objective graph builds.
  /// Kept separate from edge_status because this check happens before an
  /// edge is attempted and otherwise collapses occupied and unknown into the
  /// same kErrorCollisionEdge bucket.
  VoxelStatus projected_endpoint_status = VoxelStatus::kFree;
  /// How the first blocked edge was blocked, indexed by ProjectedEdgeStatus.
  /// Ground robots reject a candidate for four quite different reasons and
  /// all four surface as kErrorCollisionEdge; without this, a lattice that
  /// produces no vertices gives no clue whether the terrain is too steep, the
  /// space is unmapped, or something is genuinely in the way.
  int edge_status[5] = {0, 0, 0, 0, 0};
};

struct RandomSamplingParams {
  int num_vertices_max = 500;
  int num_edges_max = 10000;
  double reached_target_radius = 2.0;
  bool check_collision_at_source = true;
  int num_paths_to_target_max = 5;
  double num_loops_cutoff = 2000;
  double num_loops_max = 100000;
};

/// Per-planning-cycle timings. The ROS 1 version logged through ROS_INFO;
/// here it formats a string and leaves logging to the caller.
struct SampleStatistic {
  StateVec current_state = StateVec::Zero();
  int num_vertices_fail = 0;
  int num_edges_fail = 0;
  /// States sampled in free space but rejected because every edge collided.
  std::vector<std::vector<double>> edges_fail;
  double build_graph_time = 0.0;
  double compute_exp_gain_time = 0.0;
  double shortest_path_time = 0.0;
  double evaluate_graph_time = 0.0;
  double total_time = 0.0;

  void init(const StateVec& state) { current_state = state; }

  double totalTime() const {
    return build_graph_time + compute_exp_gain_time + shortest_path_time +
           evaluate_graph_time;
  }

  std::string formatTimes(const std::string& title = "") const;
};

}  // namespace mgg

#endif  // MGG_CORE_GRAPH_BASE_H_
