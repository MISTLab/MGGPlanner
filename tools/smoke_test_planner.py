#!/usr/bin/env python3
"""Smoke test for the ROS 2 planner node.

Publishes odometry and a synthetic room, then asks the node to build its local
grid graph. Proves the whole chain: parameters, odometry, PointCloud2 into
OctoMap, ground projection, grid sweep and graph expansion.

    ros2 run mgg_ros mggplanner_node --ros-args \
        --params-file <ws>/src/mgg_ros/config/mgg_simple.yaml &
    python3 tools/smoke_test_planner.py

Exits non-zero if no graph was built.
"""
import math, struct, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger

def room_cloud(half=6.0, res=0.2):
    """A 12x12 m room: a continuous floor plus walls.

    The floor is sampled at the map resolution. Sampling it coarsely leaves
    gaps between occupied voxels, and projectSample casts a ray straight down:
    it slips between the samples, finds no ground, and every candidate vertex
    is rejected.
    """
    pts = []
    n = int(2 * half / res) + 1
    for i in range(n):
        x = -half + i * res
        for j in range(n):
            y = -half + j * res
            pts.append((x, y, -1.0))              # floor
        # walls at +/- half, up to 2 m
        for k in range(int(3.0 / res)):
            z = -1.0 + k * res
            pts.append((x, -half, z))
            pts.append((x, half, z))
            pts.append((-half, x, z))
            pts.append((half, x, z))
    return pts

class Feeder(Node):
    def __init__(self):
        super().__init__('feeder')
        self.odom = self.create_publisher(Odometry, '/odometry', 10)
        qos = QoSProfile(depth=10); qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.cloud = self.create_publisher(PointCloud2, '/pointcloud', qos)
        self.cli = self.create_client(Trigger, '/build_local_graph')

    def send(self):
        o = Odometry()
        o.header.frame_id = 'map'
        o.pose.pose.position.x = 0.0
        o.pose.pose.orientation.w = 1.0
        for _ in range(5):
            self.odom.publish(o); time.sleep(0.05)

        pts = room_cloud()
        msg = PointCloud2()
        msg.header.frame_id = 'map'
        msg.height = 1; msg.width = len(pts)
        msg.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32,
                                 count=1) for i, n in enumerate(('x','y','z'))]
        msg.is_bigendian = False; msg.point_step = 12
        msg.row_step = 12*len(pts); msg.is_dense = True
        msg.data = b''.join(struct.pack('<fff', *p) for p in pts)
        for _ in range(3):
            self.cloud.publish(msg); time.sleep(0.2)
        print(f"published odometry and {len(pts)} points")

    def build(self):
        if not self.cli.wait_for_service(timeout_sec=5.0):
            print("service missing"); return 1
        fut = self.cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
        r = fut.result()
        print(f"success={r.success}  message={r.message}")
        return 0 if r.success else 1

def main():
    rclpy.init()
    n = Feeder()
    n.send()
    time.sleep(1.0)
    rc = n.build()
    rclpy.shutdown()
    sys.exit(rc)

main()
