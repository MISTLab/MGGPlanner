#include "mgg_core/graph_manager.h"

#include <algorithm>
#include <cstdio>

#include "mgg_core/log.h"

namespace mgg {
namespace {

/// printf-style formatting into a std::string, so the messages that were
/// ROS_WARN_COND format strings survive the move intact.
template <typename... Args>
std::string fmt(const char* f, Args... args) {
  char buf[512];
  std::snprintf(buf, sizeof(buf), f, args...);
  return std::string(buf);
}
std::string fmt(const char* f) { return std::string(f); }

}  // namespace

GraphManager::GraphManager() { reset(); }

GraphManager::~GraphManager() {
  // The ROS 1 class had no destructor, so the vertices it owns and the final
  // kd-tree were leaked at teardown.
  for (auto& entry : vertices_map_) delete entry.second;
  vertices_map_.clear();
  if (kd_tree_) kd_free(kd_tree_);
}

void GraphManager::reset() {
  // Reset kdtree first.
  if (kd_tree_) kd_free(kd_tree_);
  kd_tree_ = kd_create(3);

  // Reset graph.
  graph_.reset(new Graph());

  // Free the vertices this manager owns.
  //
  // The ROS 1 version looped "for (i = 0; i < vertices_map_.size(); ++i)
  // delete vertices_map_[i];" over an unordered_map. operator[] inserts a null
  // entry for a missing key, so with non-contiguous ids the map grew as the
  // loop ran. It did still terminate and delete every vertex, because the
  // inserts fill the gaps until i passes the largest key, but it cost
  // O(largest id) iterations and that many transient allocations: 5001
  // iterations to free 2 vertices whose ids were 0 and 5000. Non-contiguous
  // ids are reachable, since convertThisRobotGraphNodesToMsg emits only this
  // robot's vertices out of a map that also holds merged neighbours.
  //
  // Iterating the map directly is O(n) and allocates nothing.
  for (auto& entry : vertices_map_) delete entry.second;
  vertices_map_.clear();
  edge_map_.clear();
  vertex_by_robot_id_.clear();

  // Other params.
  subgraph_ind_ = -1;
  id_count_ = -1;
}

// sets the robot of the robot.
void GraphManager::setRobotId(int robot_id) { 
  robot_id_ = robot_id;
  graph_->setRobotId(robot_id_);
}

void GraphManager::addVertex(Vertex* v) {
  kd_insert3(kd_tree_, v->state.x(), v->state.y(), v->state.z(), v);
  if (v->id == 0){
    int root_vertex_id = 0;
    v->id = root_vertex_id;
    id_count_ = root_vertex_id;
    graph_->addSourceVertex(root_vertex_id); // TODO: Add robot id 
  }
  else{
    graph_->addVertex(v->id);
  }
  vertex_by_robot_id_[robot_id_][v->id] = v;
  vertices_map_[v->id] = v;
}

void GraphManager::addNeighbourVertex(Vertex* v, int neighbour_vertex_id) {
  kd_insert3(kd_tree_, v->state.x(), v->state.y(), v->state.z(), v);
  graph_->addVertex(v->id);
  vertex_by_robot_id_[v->robot_id][neighbour_vertex_id] = v;
  vertices_map_[v->id] = v;
}


void GraphManager::addEdge(Vertex* v, Vertex* u, double weight) {
  graph_->addEdge(v->id, u->id, weight);
  edge_map_[v->id].push_back(std::make_pair(u->id, weight));
  edge_map_[u->id].push_back(std::make_pair(v->id, weight));
}

void GraphManager::addNeighbourEdge(Vertex* v, Vertex* u, double weight) {
  if( !(graph_->edgeExists(v->id, u->id)) ){
    graph_->addEdge(v->id, u->id, weight);  
    edge_map_[v->id].push_back(std::make_pair(u->id, weight));
    edge_map_[u->id].push_back(std::make_pair(v->id, weight));
    // printf("[%i]New Neighbour edge %i -> %i\n",robot_id_, v->id,u->id);
  }
}

void GraphManager::removeEdge(Vertex* v, Vertex* u) {
  graph_->removeEdge(v->id, u->id);
}

bool GraphManager::updateVertexState(int id, const StateVec& state) {
  const auto found = vertices_map_.find(id);
  if (found == vertices_map_.end() || found->second == nullptr) return false;
  found->second->state = state;

  if (kd_tree_) kd_free(kd_tree_);
  kd_tree_ = kd_create(3);
  for (const auto& entry : vertices_map_) {
    const Vertex* vertex = entry.second;
    if (vertex == nullptr) continue;
    kd_insert3(kd_tree_, vertex->state.x(), vertex->state.y(),
               vertex->state.z(), entry.second);
  }
  return true;
}

bool GraphManager::getNearestVertex(const StateVec* state, Vertex** v_res) {
  if (getNumVertices() <= 0) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return false;
  }
  *v_res = (Vertex*)kd_res_item_data(nearest);
  kd_res_free(nearest);
  return true;
}

bool GraphManager::getNearestVertexInRange(const StateVec* state, double range,
                                           Vertex** v_res) {
  if (getNumVertices() <= 0) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return false;
  }
  *v_res = (Vertex*)kd_res_item_data(nearest);
  Eigen::Vector3d dist;
  dist << state->x() - (*v_res)->state.x(), state->y() - (*v_res)->state.y(),
      state->z() - (*v_res)->state.z();
  kd_res_free(nearest);
  if (dist.norm() > range) return false;
  return true;
}

bool GraphManager::getNearestVertices(const StateVec* state, double range,
                                      std::vector<Vertex*>* v_res) {
  // Notice that this might include the same vertex in the result.
  // if that vertex is added to the tree before.
  // Use the distance 0 or small threshold to filter out.
  kdres* neighbors =
      kd_nearest_range3(kd_tree_, state->x(), state->y(), state->z(), range);
  int neighbors_size = kd_res_size(neighbors);
  if (neighbors_size <= 0) {
    kd_res_free(neighbors);
    return false;
  }
  v_res->clear();
  for (int i = 0; i < neighbors_size; ++i) {
    Vertex* new_neighbor = (Vertex*)kd_res_item_data(neighbors);
    v_res->push_back(new_neighbor);
    if (kd_res_next(neighbors) <= 0) break;
  }
  kd_res_free(neighbors);
  return true;
}

bool GraphManager::updatePoseIdToNearestVertices(const StateVec* state, 
                                          double range, int pose_id) {
  kdres* neighbors =
      kd_nearest_range3(kd_tree_, state->x(), state->y(), state->z(), range);
  int neighbors_size = kd_res_size(neighbors);
  if (neighbors_size <= 0) return false;
  
  for (int i = 0; i < neighbors_size; ++i) {
    Vertex* new_neighbor = (Vertex*)kd_res_item_data(neighbors);
    int c_pose_graph_id = new_neighbor->pose_id;
    if(c_pose_graph_id == 0){
      new_neighbor->pose_id = pose_id;
    }
    if (kd_res_next(neighbors) <= 0) break;
  }
  return true;
}

bool GraphManager::findShortestPaths(ShortestPathsReport& rep) {
  return graph_->findDijkstraShortestPaths(0, rep);
}

bool GraphManager::findShortestPaths(int source_id, ShortestPathsReport& rep) {
  return graph_->findDijkstraShortestPaths(source_id, rep);
}

void GraphManager::getShortestPath(int target_id,
                                   const ShortestPathsReport& rep,
                                   bool source_to_target_order,
                                   std::vector<Vertex*>& path) {
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p) {
    path.push_back(vertices_map_[*p]);
  }
}

void GraphManager::getShortestPath(int target_id,
                                   const ShortestPathsReport& rep,
                                   bool source_to_target_order,
                                   std::vector<Eigen::Vector3d>& path) {
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p) {
    path.emplace_back(Eigen::Vector3d(vertices_map_[*p]->state.x(),
                                      vertices_map_[*p]->state.y(),
                                      vertices_map_[*p]->state.z()));
  }
}

void GraphManager::getShortestPath(int target_id,
                                   const ShortestPathsReport& rep,
                                   bool source_to_target_order,
                                   std::vector<StateVec>& path) {
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p) {
    path.emplace_back(vertices_map_[*p]->state);
  }
}

void GraphManager::getShortestPath(int target_id,
                                   const ShortestPathsReport& rep,
                                   bool source_to_target_order,
                                   std::vector<int>& path) {
  path.clear();
  if (!rep.status) {
    logWarn(fmt("Shortest paths report is not valid"));
    return;
  }

  if (rep.parent_id_map.size() <= target_id) {
    logWarn(fmt("Vertext with ID [%d] doesn't exist in the graph", target_id));
    return;
  }

  if (target_id == rep.source_id) {
    path.push_back(target_id);
    return;
  }

  int parent_id = rep.parent_id_map.at(target_id);
  if (parent_id == target_id) {
    logWarn(fmt("Vertex with ID [%d] is isolated from the graph", target_id));
    return;
  }

  path.push_back(target_id);  // current vertex id first
  path.push_back(
      parent_id);  // its first parent, the rest is recursively looked up
  while (parent_id != rep.source_id) {
    parent_id = rep.parent_id_map.at(parent_id);
    path.push_back(parent_id);
  }

  // Initially, the path follows target to source order. Reverse if required.
  if (source_to_target_order) {
    std::reverse(path.begin(), path.end());
  }
}

double GraphManager::getShortestDistance(int target_id,
                                         const ShortestPathsReport& rep) {
  double dist = std::numeric_limits<double>::max();

  if (!rep.status) {
    logWarn(fmt("Shortest paths report is not valid"));
    return dist;
  }
  if (rep.parent_id_map.size() <= target_id) {
    logWarn(fmt("Vertex with ID [%d] doesn't exist in the graph", target_id));
    return dist;
  }
  dist = rep.distance_map.at(target_id);
  return dist;
}

int GraphManager::getParentIDFromShortestPath(int target_id,
                                              const ShortestPathsReport& rep) {
  if (!rep.status) {
    logWarn(fmt("Shortest paths report is not valid"));
    return target_id;
  }

  if (rep.parent_id_map.size() <= target_id) {
    logWarn(fmt("Vertex with ID [%d] doesn't exist in the graph", target_id));
    return target_id;
  }

  return rep.parent_id_map.at(target_id);
}

void GraphManager::getLeafVertices(std::vector<Vertex*>& leaf_vertices) {
  leaf_vertices.clear();
  for (int id = 0; id < getNumVertices(); ++id) {
    Vertex* v = getVertex(id);
    if (v->is_leaf_vertex) leaf_vertices.push_back(v);
  }
}

void GraphManager::findLeafVertices(const ShortestPathsReport& rep) {
  int num_vertices = getNumVertices();
  for (int id = 0; id < num_vertices; ++id) {
    int pid = getParentIDFromShortestPath(id, rep);
    getVertex(pid)->is_leaf_vertex = false;
  }
}

int GraphManager::generateSubgraphIndex() { return ++subgraph_ind_; }

int GraphManager::generateVertexID() { return ++id_count_; }

void GraphManager::updateVertexTypeInRange(StateVec& state, double range) {
  std::vector<Vertex*> nearest_vertices;
  getNearestVertices(&state, range, &nearest_vertices);
  for (auto& v : nearest_vertices) {
    v->type = VertexType::kVisited;
  }
}

}  // namespace mgg
