#!/usr/bin/env python3
"""Two-robot smoke test for the ROS 2 planner.

Drives two planner nodes along parallel tracks, feeding each odometry and a
synthetic room, and lets their publish timers exchange graphs. Expect
"merged robot N" in both logs.

Start the pair with their graph topics crossed:

    ros2 run mgg_ros mggplanner_node --ros-args -r __ns:=/r1 \
        --params-file <cfg> -p PlanningParams.robot_id:=1 \
        -p neighbour_offsets:="[2.0, 0.0, 0.0, 0.0]" \
        -r /r1/neighbour_graph_in:=/r2/neighbour_graph_out &
    (mirror for r2)
    python3 tools/smoke_test_multirobot.py
"""
import math, struct, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger

def room(half=6.0, res=0.2, cx=0.0):
    """Floor plus walls.

    The walls matter: OctoMap only carves free space along rays to observed
    points. With a floor alone every ray points downward, so the space at
    robot height a couple of metres to the side is never observed and stays
    unknown, and the merge correctly refuses to join two graphs across it.
    """
    pts = []
    n = int(2*half/res)+1
    for i in range(n):
        x = -half + i*res
        for j in range(n):
            pts.append((x+cx, -half + j*res, -1.0))          # floor
        for k in range(int(3.0/res)):                         # walls
            z = -1.0 + k*res
            pts.append((x+cx, -half, z))
            pts.append((x+cx,  half, z))
            pts.append((cx-half, x, z))
            pts.append((cx+half, x, z))
    return pts

def cloud_msg(pts):
    m = PointCloud2()
    m.header.frame_id = 'map'
    m.height = 1; m.width = len(pts)
    m.fields = [PointField(name=nm, offset=k*4, datatype=PointField.FLOAT32, count=1)
                for k, nm in enumerate(('x','y','z'))]
    m.is_bigendian = False; m.point_step = 12
    m.row_step = 12*len(pts); m.is_dense = True
    m.data = b''.join(struct.pack('<fff', *p) for p in pts)
    return m

class Driver(Node):
    def __init__(self):
        super().__init__('driver')
        q = QoSProfile(depth=10); q.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pubs = {}
        for r in ('r1', 'r2'):
            self.pubs[r] = (
                self.create_publisher(Odometry, f'/{r}/odometry', 10),
                self.create_publisher(PointCloud2, f'/{r}/pointcloud', q),
                self.create_client(Trigger, f'/{r}/build_local_graph'))

    def step(self, robot, x, y):
        odom, cloud, cli = self.pubs[robot]
        o = Odometry(); o.header.frame_id = 'map'
        o.pose.pose.position.x = float(x); o.pose.pose.position.y = float(y)
        o.pose.pose.orientation.w = 1.0
        for _ in range(4):
            odom.publish(o); time.sleep(0.03)
        cloud.publish(cloud_msg(room(cx=x)))
        time.sleep(0.4)
        if not cli.wait_for_service(timeout_sec=5.0):
            return "no service"
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
        return fut.result().message

def main():
    rclpy.init()
    d = Driver()
    # Both robots drive along +x, r2 trailing 2 m behind in y.
    for i in range(4):
        d.step('r1', i * 1.5, 0.0)
        d.step('r2', i * 1.5, -2.0)
        print(f"step {i}: graphs built")
        time.sleep(1.2)          # let the publish timers exchange graphs
    time.sleep(3.0)
    rclpy.shutdown()

main()
