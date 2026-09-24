// Translating between mgg_msgs and mgg_core.
//
// Section 2.2 of ROS2_PORT_PLAN.md keeps the multi-robot graph exchange as a
// message and converts at the edge, so the core operates on plain structs and
// the transport stays swappable (DDS today, Zenoh or a radio link on a
// deployment).

#ifndef MGG_ROS_CONVERSIONS_H_
#define MGG_ROS_CONVERSIONS_H_

#include <geometry_msgs/msg/pose.hpp>
#include <mgg_msgs/msg/graph.hpp>

#include "mgg_core/graph_manager.h"
#include "mgg_core/graph_merge.h"
#include "mgg_core/types.h"

namespace mgg_ros {

/// Yaw about z from a quaternion. Replaces tf::getYaw, which does not exist in
/// ROS 2.
double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

geometry_msgs::msg::Pose toPoseMsg(const mgg::StateVec& state);
mgg::StateVec fromPoseMsg(const geometry_msgs::msg::Pose& pose);

/// A received neighbour graph, ready for mgg::mergeNeighbourGraph.
mgg::GraphExchange fromGraphMsg(const mgg_msgs::msg::Graph& msg);

/// This robot's own vertices and the edges between them, for broadcast.
///
/// Only vertices belonging to `robot_id` are emitted, matching the ROS 1
/// convertThisRobotGraphNodesToMsg. Note the consequence: the emitted ids are
/// a subset of the sender's id space once any neighbour graph has been merged,
/// so a receiver must not assume they are contiguous.
/// z is lowered by `driving_height`, to the ground under each vertex
/// (GraphExchangeVertex).
mgg_msgs::msg::Graph toGraphMsg(mgg::GraphManager& graph, int robot_id,
                                double driving_height = 0.0);

}  // namespace mgg_ros

#endif  // MGG_ROS_CONVERSIONS_H_
