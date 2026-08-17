#!/usr/bin/env python3
"""Speaks the ARGoS side of the bridge protocol without ARGoS.

Checks the wire format and the two directions independently of the simulator,
so a failure here is a bridge bug and a failure in the real run is a
simulation one. Sends a synthetic scan of a known box room and verifies that
what comes out the ROS side is the same room: the ranges have to survive the
round trip through f32 ranges, a hit mask and the implied ray grid.

Run inside the Jazzy container, with the workspace built and sourced:
    python3 tools/smoke_test_argos_bridge.py
"""
import math
import os
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

OBS_MAGIC = b'MGGB'
CMD_MAGIC = b'MGGC'
VERSION = 3
BLOCK_ODOMETRY, BLOCK_LIDAR, BLOCK_IMU, BLOCK_TRUTH = 1, 2, 4, 5
CMD_NONE, CMD_PATH, CMD_STOP = 0, 1, 2
OVERLAY_PATH, OVERLAY_GRAPH_EDGES, OVERLAY_POINTS = 1, 2, 3
OVERLAY_GLOBAL_GRAPH, OVERLAY_MERGE = 4, 5

RINGS, AZIMUTHS = 16, 360
ELEV_MIN, ELEV_MAX = math.radians(-15.0), math.radians(15.0)
MAX_RANGE = 20.0
# The robot sits at the origin of a room with walls at these distances.
WALLS = {'x_max': 2.0, 'x_min': -4.0, 'y_max': 3.0, 'y_min': -5.0}


def true_range(azimuth, elevation):
    """Range to the walls of the box room, ignoring floor and ceiling."""
    dx = math.cos(elevation) * math.cos(azimuth)
    dy = math.cos(elevation) * math.sin(azimuth)
    best = MAX_RANGE
    for value, direction in ((WALLS['x_max'], dx), (WALLS['x_min'], dx)):
        if abs(direction) > 1e-9:
            t = value / direction
            if 0 < t < best:
                best = t
    for value, direction in ((WALLS['y_max'], dy), (WALLS['y_min'], dy)):
        if abs(direction) > 1e-9:
            t = value / direction
            if 0 < t < best:
                best = t
    return best


def build_scan():
    ranges, hits = [], []
    for a in range(AZIMUTHS):
        azimuth = 2.0 * math.pi * a / AZIMUTHS
        for r in range(RINGS):
            elevation = ELEV_MIN + (ELEV_MAX - ELEV_MIN) * r / (RINGS - 1)
            rng = true_range(azimuth, elevation)
            hit = rng < MAX_RANGE
            ranges.append(rng if hit else MAX_RANGE)
            hits.append(1 if hit else 0)
    return ranges, hits


def block(kind, payload):
    return struct.pack('<BI', kind, len(payload)) + payload


def observation(tick, robot_id, ranges, hits, position):
    lidar = struct.pack('<IIfff', RINGS, AZIMUTHS, ELEV_MIN, ELEV_MAX, MAX_RANGE)
    lidar += struct.pack('<%df' % len(ranges), *ranges)
    lidar += bytes(hits)
    odom = struct.pack('<7d', position[0], position[1], position[2],
                       1.0, 0.0, 0.0, 0.0)
    blocks = block(BLOCK_ODOMETRY, odom) + block(BLOCK_LIDAR, lidar)
    body = struct.pack('<B', len(robot_id)) + robot_id.encode()
    body += struct.pack('<B', 2) + blocks
    return (OBS_MAGIC + struct.pack('<HIII', VERSION, tick, 10, 1) + body)


def recv_exactly(sock, n):
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise RuntimeError('bridge closed the connection')
        buf += chunk
    return buf


def read_command(sock):
    magic = recv_exactly(sock, 4)
    assert magic == CMD_MAGIC, 'bad command magic %r' % magic
    version, tick, count = struct.unpack('<HII', recv_exactly(sock, 10))
    assert version == VERSION, 'version %d' % version
    out = {}
    for _ in range(count):
        (length,) = struct.unpack('<B', recv_exactly(sock, 1))
        name = recv_exactly(sock, length).decode()
        (kind,) = struct.unpack('<B', recv_exactly(sock, 1))
        waypoints = []
        if kind == CMD_PATH:
            (n,) = struct.unpack('<I', recv_exactly(sock, 4))
            for _ in range(n):
                waypoints.append(struct.unpack('<4d', recv_exactly(sock, 32)))
        # Overlay blocks. Parsing them by their length prefix rather than by
        # their contents is the point: an unknown type has to be skippable, or
        # the two ends cannot be upgraded independently.
        overlays = {}
        (blocks,) = struct.unpack('<B', recv_exactly(sock, 1))
        for _ in range(blocks):
            btype, blen = struct.unpack('<BI', recv_exactly(sock, 5))
            payload = recv_exactly(sock, blen)
            (bcount,) = struct.unpack('<I', payload[:4])
            floats = struct.unpack('<%df' % ((len(payload) - 4) // 4),
                                   payload[4:])
            overlays[btype] = (bcount, floats)
        out[name] = (kind, waypoints, overlays)
    return tick, out


class Listener(Node):
    def __init__(self):
        super().__init__('bridge_smoke_listener')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        sensor = QoSProfile(depth=10)
        sensor.reliability = ReliabilityPolicy.BEST_EFFORT
        self.clouds, self.odoms, self.clocks, self.tfs = [], [], [], []
        self.create_subscription(PointCloud2, '/r0/pointcloud',
                                 self.clouds.append, sensor)
        self.create_subscription(Odometry, '/r0/odometry',
                                 self.odoms.append, 10)
        # /clock goes out on ClockQoS, which is best effort. That is what
        # rclcpp's own TimeSource subscribes with; asking for reliable here
        # silently receives nothing.
        clock_qos = QoSProfile(depth=1)
        clock_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Clock, '/clock', self.clocks.append, clock_qos)
        self.create_subscription(TFMessage, '/tf', self.tfs.append, 10)
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, '/r0/path', latched)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/r0/graph_markers', QoSProfile(depth=1))


def read_points(msg):
    step = msg.point_step
    return [struct.unpack_from('<fff', msg.data, i * step)
            for i in range(msg.width * msg.height)]


def main():
    tmpdir = tempfile.mkdtemp(prefix='mgg_bridge_')
    sock_path = os.path.join(tmpdir, 'argos.sock')
    bridge = subprocess.Popen(
        ['ros2', 'run', 'mgg_argos', 'mgg_argos_bridge',
         '--ros-args',
         '-p', 'socket_path:=' + sock_path,
         '-p', 'robots:=[r0]',
         '-p', 'use_sim_time:=true',
         '-p', 'real_time_factor:=0.0',
         '-p', 'lidar_translation:=[0.0,0.0,0.4]'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        client = None
        for _ in range(100):
            if os.path.exists(sock_path):
                client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                try:
                    client.connect(sock_path)
                    break
                except OSError:
                    client.close()
                    client = None
            time.sleep(0.1)
        if client is None:
            print('FAIL: the bridge never created %s' % sock_path)
            return 1

        rclpy.init()
        node = Listener()
        spin = threading.Thread(
            target=lambda: rclpy.spin(node), daemon=True)
        spin.start()
        time.sleep(1.0)

        ranges, hits = build_scan()
        failures = []

        # A few plain ticks: everything should come out with no command back.
        for tick in range(1, 6):
            client.sendall(observation(tick, 'r0', ranges, hits, (0.0, 0.0, 0.0)))
            echoed, commands = read_command(client)
            if echoed != tick:
                failures.append('tick %d echoed as %d' % (tick, echoed))
            if commands.get('r0', (None,))[0] != CMD_NONE:
                failures.append('tick %d: unexpected command %r'
                                % (tick, commands.get('r0')))
        time.sleep(1.0)

        if not node.clouds:
            failures.append('no point cloud published')
        else:
            points = read_points(node.clouds[-1])
            if len(points) != RINGS * AZIMUTHS:
                failures.append('cloud has %d points, expected %d'
                                % (len(points), RINGS * AZIMUTHS))
            else:
                # The cloud is in the sensor frame, so the endpoints must
                # reproduce the room the ranges described.
                worst, worst_at = 0.0, None
                for a in range(AZIMUTHS):
                    azimuth = 2.0 * math.pi * a / AZIMUTHS
                    for r in range(RINGS):
                        i = a * RINGS + r
                        elevation = (ELEV_MIN + (ELEV_MAX - ELEV_MIN)
                                     * r / (RINGS - 1))
                        x, y, z = points[i]
                        got = math.sqrt(x * x + y * y + z * z)
                        want = true_range(azimuth, elevation)
                        if want >= MAX_RANGE:
                            # A miss must land beyond the range, or octomap
                            # marks an obstacle where nothing was seen.
                            if got <= MAX_RANGE:
                                failures.append(
                                    'miss at ray %d came back at %.3f, not '
                                    'past %.1f' % (i, got, MAX_RANGE))
                            continue
                        if abs(got - want) > worst:
                            worst, worst_at = abs(got - want), i
                if worst > 0.01:
                    failures.append('worst range error %.4f m at ray %s'
                                    % (worst, worst_at))
                else:
                    print('cloud: %d points, worst range error %.5f m'
                          % (len(points), worst))

        if not node.clocks:
            failures.append('no /clock published')
        else:
            print('clock: %d ticks, last %.1f s'
                  % (len(node.clocks), node.clocks[-1].clock.sec
                     + node.clocks[-1].clock.nanosec * 1e-9))
        if not node.odoms:
            failures.append('no odometry published')
        frames = {t.child_frame_id for m in node.tfs for t in m.transforms}
        if 'r0/base_link' not in frames:
            failures.append('no map -> r0/base_link transform, saw %s' % frames)
        else:
            print('tf: %s' % sorted(frames))

        # Now the return direction: a path published to the robot has to come
        # back as a command on the next tick, exactly once.
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = node.get_clock().now().to_msg()
        for x, y in ((1.0, 0.0), (1.0, 1.0), (0.0, 1.0)):
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        node.path_pub.publish(path)
        time.sleep(1.0)

        client.sendall(observation(6, 'r0', ranges, hits, (0.0, 0.0, 0.0)))
        _, commands = read_command(client)
        kind, waypoints, overlays = commands.get('r0', (None, [], {}))
        if kind != CMD_PATH:
            failures.append('path was not forwarded, got command %r' % kind)
        elif len(waypoints) != 3:
            failures.append('forwarded %d waypoints, expected 3'
                            % len(waypoints))
        else:
            print('path: forwarded %d waypoints, first %s'
                  % (len(waypoints), waypoints[0][:3]))

        # The path must also come back as overlay geometry, so the simulator
        # can draw it. Same points, as f32 triplets.
        if OVERLAY_PATH not in overlays:
            failures.append('no path overlay accompanied the path command')
        else:
            count, floats = overlays[OVERLAY_PATH]
            if count != 3 or len(floats) != 9:
                failures.append('path overlay has %d points / %d floats, '
                                'expected 3 / 9' % (count, len(floats)))
            elif abs(floats[0] - 1.0) > 1e-5 or abs(floats[1] - 0.0) > 1e-5:
                failures.append('path overlay starts at %s, expected (1, 0)'
                                % (floats[:3],))
            else:
                print('overlay: path echoed as %d points' % count)

        # The graph overlays. The bridge sorts the planner's markers by
        # namespace into three separate overlay streams, and getting that
        # mapping wrong is invisible until someone looks at the viewer, so
        # check each one arrives as its own block type.
        markers = MarkerArray()
        for ns, count in (('local_graph_edges', 2),
                          ('global_graph_edges', 3),
                          ('graph_merges', 1)):
            marker = Marker()
            marker.ns = ns
            marker.type = Marker.LINE_LIST
            for i in range(count * 2):
                point = Point()
                point.x, point.y, point.z = float(i), float(ns == 'graph_merges'), 0.0
                marker.points.append(point)
            markers.markers.append(marker)
        node.marker_pub.publish(markers)
        time.sleep(1.5)

        client.sendall(observation(8, 'r0', ranges, hits, (0.0, 0.0, 0.0)))
        _, commands = read_command(client)
        _, _, overlays = commands.get('r0', (None, [], {}))
        for name, block, want in (('local graph', OVERLAY_GRAPH_EDGES, 2),
                                  ('global graph', OVERLAY_GLOBAL_GRAPH, 3),
                                  ('merge beacons', OVERLAY_MERGE, 1)):
            if block not in overlays:
                failures.append('no %s overlay block reached ARGoS' % name)
            elif overlays[block][0] != want:
                failures.append('%s overlay carried %d segments, expected %d'
                                % (name, overlays[block][0], want))
            else:
                print('overlay: %s forwarded as %d segments'
                      % (name, overlays[block][0]))

        # And it must not be sent again: re-forwarding the same path would
        # restart the robot at waypoint zero every tick.
        client.sendall(observation(7, 'r0', ranges, hits, (0.0, 0.0, 0.0)))
        _, commands = read_command(client)
        if commands.get('r0', (None,))[0] != CMD_NONE:
            failures.append('the same path was forwarded twice')
        # ... but the overlay must still be resent, because the far side
        # clears its overlay every tick and anything not resent vanishes.
        _, _, overlays = commands.get('r0', (None, [], {}))
        if OVERLAY_PATH not in overlays:
            failures.append('the path overlay was not resent on a tick with '
                            'no new path; it would flicker off')
        else:
            print('overlay: path redrawn on a tick with no new command')

        rclpy.shutdown()
        if failures:
            print('\nFAIL')
            for f in failures:
                print('  - %s' % f)
            return 1
        print('\nPASS')
        return 0
    finally:
        bridge.terminate()
        bridge.wait(timeout=10)


if __name__ == '__main__':
    sys.exit(main())
