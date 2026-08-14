// Multi-robot graph merge: folding a neighbour's exploration graph into ours.
//
// This is the "multi-robot" half of MGG. Ported from Rrg::updateNeighbourGraph
// with two substantive changes.
//
// 1. The inter-robot transform is injected rather than hardcoded.
//
//    The ROS 1 code carried a table of literal offsets built in loadParams
//    under the authors' own comment, "Sloppy way of init pose. TODO: got to do
//    it better", and applied it as
//
//        state[0] = v.pose.position.x + init_offsets_[v.robot_id-1][0];
//        state[1] = v.pose.position.y + init_offsets_[v.robot_id-1][1];
//        state[2] = v.pose.position.z;
//
//    Note what that is: a translation in x and y only. No z, and no rotation,
//    so it silently assumes every robot starts at the same heading and
//    altitude, and that those relative poses are known in advance.
//
//    Here the transform comes from a PoseSource and is a full Eigen::Isometry3d.
//    StaticPoseSource reproduces the old behaviour for bring-up; a Swarm-SLAM
//    backed implementation supplies transforms estimated from inter-robot loop
//    closures, which is what lets robots start anywhere and align on
//    rendezvous. See section 3.4 of ROS2_PORT_PLAN.md.
//
// 2. Incoming graphs are validated.
//
//    A graph arrives from another robot over a link we do not control. The ROS
//    1 merge looked edge endpoints up with getNeighbourVertex, which resolves
//    through unordered_map::operator[] and so returns nullptr for an unknown
//    id, then passed that straight to addEdge, which dereferences it. An edge
//    naming a vertex absent from the same message was an immediate null
//    dereference. Endpoints are checked here and unresolvable edges are
//    counted and skipped.
//
// The merge takes plain structs, not ROS messages: section 2.2 of the plan
// keeps the exchange format at the ROS boundary so the transport stays
// swappable.

#ifndef MGG_CORE_GRAPH_MERGE_H_
#define MGG_CORE_GRAPH_MERGE_H_

#include <functional>
#include <map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "mgg_core/graph_manager.h"
#include "mgg_core/types.h"

namespace mgg {

/// One vertex of a neighbour's graph, in that neighbour's own frame.
struct GraphExchangeVertex {
  int id = 0;
  int robot_id = 0;
  /// x, y, z, yaw, in the sending robot's frame.
  StateVec state = StateVec::Zero();
  int num_unknown_voxels = 0;
  int num_free_voxels = 0;
  int num_occupied_voxels = 0;
  bool is_frontier = false;
};

struct GraphExchangeEdge {
  int source_id = 0;
  int target_id = 0;
  double weight = 0.0;
};

/// A neighbour's graph as received. Mirrors mgg_msgs/Graph; conversion happens
/// in mgg_ros.
struct GraphExchange {
  std::vector<GraphExchangeVertex> vertices;
  std::vector<GraphExchangeEdge> edges;
};

/// Supplies the rigid transform taking another robot's frame into ours.
class PoseSource {
 public:
  virtual ~PoseSource() = default;
  /// False when the transform to `robot_id` is not known yet, in which case
  /// that robot's graph cannot be merged and is left alone.
  virtual bool getRobotTransform(int robot_id,
                                 Eigen::Isometry3d& t_ours_theirs) const = 0;
};

/// Fixed, pre-surveyed transforms. Reproduces the ROS 1 behaviour, and is
/// enough for ARGoS bring-up where the spawn poses are known exactly.
class StaticPoseSource : public PoseSource {
 public:
  void setTransform(int robot_id, const Eigen::Isometry3d& t_ours_theirs);
  /// Convenience matching the ROS 1 table, which was a planar translation.
  void setOffset(int robot_id, double dx, double dy, double dz = 0.0);
  bool getRobotTransform(int robot_id,
                         Eigen::Isometry3d& t_ours_theirs) const override;

 private:
  std::map<int, Eigen::Isometry3d> transforms_;
};

/// Is a straight move between two world points traversable? Supplied by the
/// planner, which owns the map and the robot's footprint.
using EdgeAdmissibleFn =
    std::function<bool(const Eigen::Vector3d& from, const Eigen::Vector3d& to)>;

struct MergeResult {
  /// True once this robot's graph and the neighbour's are connected.
  bool merged = false;
  int vertices_added = 0;
  int vertices_updated = 0;
  int edges_added = 0;
  /// Edges naming a vertex that was not in the same message. Non-zero means
  /// the sender and receiver disagree about the graph.
  int edges_unresolved = 0;
  /// Set when the neighbour's transform is not yet known.
  bool transform_unavailable = false;
};

/// Folds `incoming` into `global_graph`.
///
/// Until the two graphs are connected, every incoming vertex is a candidate
/// rendezvous point: the nearest own vertex within `rendezvous_radius` is
/// found and, if `is_admissible` says the robot could actually drive between
/// them, the graphs are joined there. Once joined, subsequent calls simply add
/// what is new and refresh what is not.
MergeResult mergeNeighbourGraph(GraphManager& global_graph,
                                const GraphExchange& incoming,
                                const PoseSource& poses,
                                const EdgeAdmissibleFn& is_admissible,
                                double rendezvous_radius = 5.0);

}  // namespace mgg

#endif  // MGG_CORE_GRAPH_MERGE_H_
