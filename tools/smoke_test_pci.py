#!/usr/bin/env python3
"""Feeds the planner, then drives it through the control interface."""
import math, struct, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger

def room(half=6.0, res=0.2):
    pts, n = [], int(2*half/res)+1
    for i in range(n):
        x = -half + i*res
        for j in range(n):
            pts.append((x, -half + j*res, -1.0))
        for k in range(int(3.0/res)):
            z = -1.0 + k*res
            pts += [(x,-half,z),(x,half,z),(-half,x,z),(half,x,z)]
    return pts

class Driver(Node):
    def __init__(self):
        super().__init__('pci_driver')
        q = QoSProfile(depth=10); q.reliability = ReliabilityPolicy.BEST_EFFORT
        self.odom = self.create_publisher(Odometry, '/odometry', 10)
        self.cloud = self.create_publisher(PointCloud2, '/pointcloud', q)
        self.trig = self.create_client(Trigger, '/pci_trigger')
        self.stop = self.create_client(Trigger, '/pci_stop')
        self.got = []
        lat = QoSProfile(depth=1)
        from rclpy.qos import DurabilityPolicy
        lat.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Path, '/command_path',
                                 lambda m: self.got.append(len(m.poses)), lat)

    def feed(self):
        o = Odometry(); o.header.frame_id='map'; o.pose.pose.orientation.w=1.0
        for _ in range(5): self.odom.publish(o); time.sleep(0.05)
        pts = room()
        m = PointCloud2(); m.header.frame_id='map'; m.height=1; m.width=len(pts)
        m.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
                  for i,n in enumerate(('x','y','z'))]
        m.is_bigendian=False; m.point_step=12; m.row_step=12*len(pts); m.is_dense=True
        m.data=b''.join(struct.pack('<fff',*p) for p in pts)
        for _ in range(3): self.cloud.publish(m); time.sleep(0.2)
        print(f"fed {len(pts)} points")

    def call(self, cli, name):
        if not cli.wait_for_service(timeout_sec=10.0):
            print(f"{name}: service missing"); return None
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=120.0)
        r = fut.result()
        print(f"{name}: success={r.success} message={r.message}" if r
              else f"{name}: NO RESPONSE (deadlock?)")
        return r

def main():
    rclpy.init(); d = Driver(); d.feed(); time.sleep(1.0)
    r = d.call(d.trig, 'pci_trigger')
    for _ in range(20):
        rclpy.spin_once(d, timeout_sec=0.2)
        if d.got: break
    print(f"command_path received: {d.got}")
    d.call(d.stop, 'pci_stop')
    rclpy.shutdown()
    sys.exit(0 if (r and r.success and d.got) else 1)

main()
