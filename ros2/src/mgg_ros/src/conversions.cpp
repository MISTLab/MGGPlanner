#include "mgg_ros/conversions.h"

#include <cmath>

namespace mgg_ros {

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  // Same formula tf::getYaw used, without pulling in a dependency for it.
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Pose toPoseMsg(const mgg::StateVec& state) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = state[0];
  pose.position.y = state[1];
  pose.position.z = state[2];
  const double half_yaw = state[3] * 0.5;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = std::sin(half_yaw);
  pose.orientation.w = std::cos(half_yaw);
  return pose;
}

mgg::StateVec fromPoseMsg(const geometry_msgs::msg::Pose& pose) {
  return mgg::StateVec(pose.position.x, pose.position.y, pose.position.z,
                       yawFromQuaternion(pose.orientation));
}

mgg::GraphExchange fromGraphMsg(const mgg_msgs::msg::Graph& msg) {
  mgg::GraphExchange out;
  out.vertices.reserve(msg.vertices.size());
  for (const auto& v : msg.vertices) {
    mgg::GraphExchangeVertex ev;
    ev.id = v.id;
    ev.robot_id = v.robot_id;
    ev.state = fromPoseMsg(v.pose);
    ev.num_unknown_voxels = v.num_unknown_voxels;
    ev.num_occupied_voxels = v.num_occupied_voxels;
    ev.num_free_voxels = v.num_free_voxels;
    ev.is_frontier = v.is_frontier;
    out.vertices.push_back(ev);
  }
  out.edges.reserve(msg.edges.size());
  for (const auto& e : msg.edges) {
    out.edges.push_back(mgg::GraphExchangeEdge{e.source_id, e.target_id,
                                               e.weight});
  }
  return out;
}

mgg_msgs::msg::Graph toGraphMsg(mgg::GraphManager& graph, int robot_id,
                                double driving_height) {
  mgg_msgs::msg::Graph msg;

  auto own = graph.vertex_by_robot_id_.find(robot_id);
  if (own == graph.vertex_by_robot_id_.end()) return msg;

  for (const auto& entry : own->second) {
    const mgg::Vertex* v = entry.second;
    if (v == nullptr) continue;
    mgg_msgs::msg::Vertex vertex;
    vertex.id = entry.first;
    vertex.pose = toPoseMsg(v->state);
    vertex.pose.position.z -= driving_height;
    vertex.num_unknown_voxels = v->vol_gain.num_unknown_voxels;
    vertex.num_occupied_voxels = v->vol_gain.num_occupied_voxels;
    vertex.num_free_voxels = v->vol_gain.num_free_voxels;
    vertex.is_frontier = v->vol_gain.is_frontier;
    vertex.robot_id = v->robot_id;
    msg.vertices.push_back(vertex);
  }

  // Edges between this robot's own vertices only.
  //
  // The ROS 1 version filtered with vertices_map_[id]->robot_id, using
  // unordered_map::operator[], which inserts a null for an id the Boost graph
  // knows but the vertex map does not, and then dereferences it. Graph::addEdge
  // auto-creates Boost vertices for unknown ids, so that is reachable. Looked
  // up without insertion and skipped here instead.
  std::pair<mgg::Graph::GraphType::edge_iterator,
            mgg::Graph::GraphType::edge_iterator> ei;
  graph.graph_->getEdgeIterator(ei);
  for (auto it = ei.first; it != ei.second; ++it) {
    int source_id = 0, target_id = 0;
    double weight = 0.0;
    std::tie(source_id, target_id, weight) = graph.graph_->getEdgeProperty(it);

    auto s = graph.vertices_map_.find(source_id);
    auto t = graph.vertices_map_.find(target_id);
    if (s == graph.vertices_map_.end() || t == graph.vertices_map_.end() ||
        s->second == nullptr || t->second == nullptr) {
      continue;
    }
    if (s->second->robot_id != robot_id || t->second->robot_id != robot_id) {
      continue;
    }
    mgg_msgs::msg::Edge edge;
    edge.source_id = source_id;
    edge.target_id = target_id;
    edge.weight = weight;
    msg.edges.push_back(edge);
  }
  return msg;
}

}  // namespace mgg_ros
