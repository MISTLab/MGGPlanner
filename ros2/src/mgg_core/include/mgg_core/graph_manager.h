// Vertex store for the exploration graphs: owns the vertices, keeps a kd-tree
// for nearest-neighbour lookup, and wraps Graph for shortest paths.
//
// Ported from planner_common/graph_manager.h. Left behind at the ROS
// boundary, where they belong:
//
//   * convertGraphToMsg / convertThisRobotGraphNodesToMsg / convertMsgToGraph
//     and UpdateNeighbourGraph, which all take a planner_msgs::Graph. Section
//     2.2 of ROS2_PORT_PLAN.md keeps the multi-robot exchange as a message and
//     converts at the edge, so the merge stays swappable (DDS today, Zenoh or
//     a radio link on deployment).
//   * saveGraph / loadGraph, which used ros::serialization.
//
// Not carried across at all: existVertexInRange, which the ROS 1 header
// declared but no translation unit defined and nothing called. Reproducing a
// declaration that would fail at link time is not worth it; the same question
// is answered by getNearestVertexInRange.

#ifndef MGG_CORE_GRAPH_MANAGER_H_
#define MGG_CORE_GRAPH_MANAGER_H_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "mgg_kdtree/kdtree.h"

#include "mgg_core/graph.h"
#include "mgg_core/graph_base.h"
#include "mgg_core/types.h"

namespace mgg {

class GraphManager {
 public:
  GraphManager();
  ~GraphManager();

  // Non-copyable: it owns raw Vertex pointers.
  GraphManager(const GraphManager&) = delete;
  GraphManager& operator=(const GraphManager&) = delete;

  /// Drops every vertex and starts a fresh graph.
  void reset();

  void setRobotId(int robot_id);

  int generateSubgraphIndex();
  int generateVertexID();

  /// Takes ownership: the vertex is deleted by reset() or the destructor.
  /// This was implicit in the ROS 1 class and is easy to get wrong.
  void addVertex(Vertex* v);
  /// Adds a vertex belonging to another robot's graph, keyed by that robot's
  /// own vertex id.
  void addNeighbourVertex(Vertex* v, int neighbour_vertex_id);
  void addEdge(Vertex* v, Vertex* u, double weight);
  void addNeighbourEdge(Vertex* v, Vertex* u, double weight);
  void removeEdge(Vertex* v, Vertex* u);

  int getNumVertices() { return graph_->getNumVertices(); }
  int getNumEdges() { return graph_->getNumEdges(); }

  Vertex* getVertex(int id) { return vertices_map_[id]; }
  Vertex* getNeighbourVertex(int neighbour_vertex_id, int robot_id) {
    return vertex_by_robot_id_[robot_id][neighbour_vertex_id];
  }

  void getLeafVertices(std::vector<Vertex*>& leaf_vertices);
  void findLeafVertices(const ShortestPathsReport& rep);

  bool findShortestPaths(ShortestPathsReport& rep);
  bool findShortestPaths(int source_id, ShortestPathsReport& rep);

  void getShortestPath(int target_id, const ShortestPathsReport& rep,
                       bool source_to_target_order, std::vector<int>& path);
  void getShortestPath(int target_id, const ShortestPathsReport& rep,
                       bool source_to_target_order, std::vector<Vertex*>& path);
  void getShortestPath(int target_id, const ShortestPathsReport& rep,
                       bool source_to_target_order,
                       std::vector<Eigen::Vector3d>& path);
  void getShortestPath(int target_id, const ShortestPathsReport& rep,
                       bool source_to_target_order, std::vector<StateVec>& path);
  double getShortestDistance(int target_id, const ShortestPathsReport& rep);
  int getParentIDFromShortestPath(int target_id,
                                  const ShortestPathsReport& rep);

  bool getNearestVertex(const StateVec* state, Vertex** v_res);
  bool getNearestVertexInRange(const StateVec* state, double range,
                               Vertex** v_res);
  bool getNearestVertices(const StateVec* state, double range,
                          std::vector<Vertex*>* v_res);
  bool updatePoseIdToNearestVertices(const StateVec* state, double range,
                                     int pose_id);

  void updateVertexTypeInRange(StateVec& state, double range);

  /// Boost.Graph wrapper holding ids and weights.
  std::shared_ptr<Graph> graph_;
  /// Vertex id to vertex.
  std::unordered_map<int, Vertex*> vertices_map_;
  /// id -> [(neighbour id, edge cost)]
  std::map<int, std::vector<std::pair<int, double>>> edge_map_;
  /// Other robots' graphs: robot id -> (their vertex id -> our vertex).
  std::unordered_map<int, std::unordered_map<int, Vertex*>> vertex_by_robot_id_;
  /// Which neighbours' graphs have been merged already.
  std::unordered_map<int, bool> merged_graphs_;

 private:
  /// Nearest-neighbour index over the vertices.
  kdtree* kd_tree_ = nullptr;
  int subgraph_ind_ = -1;
  int id_count_ = -1;
  int robot_id_ = 0;

  /// Boost-local id to <subgraph id, vertex id>. Debug aid.
  std::unordered_map<int, std::pair<int, int>> local_id_map_;
};

}  // namespace mgg

#endif  // MGG_CORE_GRAPH_MANAGER_H_
