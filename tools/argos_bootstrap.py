#!/usr/bin/env python3
"""Drives the robot a short way so the planner can take over.

The planner cannot start from a standstill. A lidar mounted on the robot never
sees the ground directly beneath it - the lowest ray meets the floor some way
out, and closer than that is a blind disc - so the graph's root has no mapped
ground under it and every edge leaving it is rejected as hanging. Driving even
a couple of metres sweeps that disc with the rings that were previously
looking further away, the map closes up, and planning proceeds unaided from
then on.

This publishes a short path directly to the robot, which is also the cleanest
test of the command direction: if the robot moves, then ROS paths, the bridge,
the wire protocol, the ARGoS controller and the wheels are all working.

Usage: argos_bootstrap.py [robot_id] [--distance M]
"""
import argparse
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('robot', nargs='?', default='r0')
    parser.add_argument('--distance', type=float, default=5.0,
                        help='how far to drive, metres')
    parser.add_argument('--timeout', type=float, default=90.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node('argos_bootstrap')
    node.set_parameters([rclpy.parameter.Parameter(
        'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

    latched = QoSProfile(depth=1)
    latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
    publisher = node.create_publisher(Path, '/%s/path' % args.robot, latched)
    poses = []
    node.create_subscription(Odometry, '/%s/ground_truth' % args.robot,
                             poses.append, 10)

    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if poses:
            break
    if not poses:
        print('no ground truth on /%s/ground_truth; is the simulator running '
              'with publish_ground_truth?' % args.robot)
        return 1
    start = poses[-1].pose.pose.position
    print('bootstrap: starting from (%.2f, %.2f)' % (start.x, start.y))

    # Straight ahead along +x in three steps. The follower retires waypoints
    # within tolerance, so intermediate ones keep it on the line rather than
    # letting it arc.
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = node.get_clock().now().to_msg()
    for fraction in (0.33, 0.66, 1.0):
        pose = PoseStamped()
        pose.pose.position.x = start.x + args.distance * fraction
        pose.pose.position.y = start.y
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    publisher.publish(path)
    print('bootstrap: published %d waypoints, driving %.1f m'
          % (len(path.poses), args.distance))

    # Stop as soon as it has gone far enough; no point burning the full budget.
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        now = poses[-1].pose.pose.position
        moved = ((now.x - start.x) ** 2 + (now.y - start.y) ** 2) ** 0.5
        if moved >= args.distance * 0.7:
            break
    now = poses[-1].pose.pose.position
    moved = ((now.x - start.x) ** 2 + (now.y - start.y) ** 2) ** 0.5
    print('bootstrap: now at (%.2f, %.2f), moved %.2f m' % (now.x, now.y, moved))
    rclpy.shutdown()
    if moved < 0.2:
        print('bootstrap: the robot did not move. The planner will have no '
              'ground under its graph root and will return empty paths.')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
