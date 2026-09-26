#!/usr/bin/env python3
"""Cut the departure and observed-space fixtures out of SwarmDeck run-5
planner grids (SDMGRID1).

usage: make_departure_fixtures.py <grid dir> <output dir>

<grid dir> holds r0.sdpg and r1.sdpg from diag-run5/data, and r0n.sdpg and
r1n.sdpg, robot_0's r1490 and robot_1's r1615 grids of the same run; and
r3r15.sdpg, robot_3's r15 grid of run 6 (diag-run6/data/r3).

Each fixture is the part of a robot's planner grid within 3 m (in x and y)
of where it stood boxed in, between 0.6 m below and 1.2 m above the ground
there: every occupied and free cell, and each occupied cell's highest
surface point. See README.md.
"""
import json
import math
import struct
import sys

import numpy as np

CASES = [
    # name, grid, robot position (odom), what it is, cells made unknown
    ("boxed_in_r0.txt", "r0.sdpg", 4.16, -5.28,
     "robot_0 (Bunker), boxed in at 1790345307 in a room's SE corner", []),
    ("boxed_in_r1.txt", "r1.sdpg", 53.76, -40.92,
     "robot_1 (Bunker), boxed in from 1790345621 beside a wall column", []),
    ("ledge_r0.txt", "r0n.sdpg", 137.8, -79.5,
     "robot_0 (Bunker), parked at a ledge over a 4 m pit at 1790349295, "
     "which it fell into; the pit floor lies below the cut", []),
    # The wall foot within the turning circle at robot_1's 1790348990.57
    # root, voxels 1 to 4 over the floor, made unknown as it was before the
    # robot faced it.
    ("turn_r1.txt", "r1n.sdpg", 76.60, -43.76,
     "robot_1 (Bunker), turned 170 degrees at 1790348990.57 beside a wall "
     "foot it had not yet observed",
     [((379, -220), 1, 4), ((379, -219), 1, 4), ((379, -218), 1, 4),
      ((380, -218), 1, 4), ((380, -217), 1, 4), ((381, -216), 1, 4)]),
    # Run 6: robot_3 (Spot) at its start, never moved, and robot_1 (Bunker)
    # 2 m east of it. robot_3's lidar had seen no ground within about 2.4 m
    # (4.8 m east, behind robot_1), so the ground under it is given: the
    # hangar floor, 0.975 m under its driving height.
    ("standing_r3.txt", "r3r15.sdpg", -2.02, -2.00,
     "robot_3 (Spot) at its start in run 6, with robot_1's start 2 m east, "
     "the ground within about 2.4 m of it never observed", []),
]
HALF_WINDOWS = {"ledge_r0.txt": 2.5, "turn_r1.txt": 2.5, "standing_r3.txt": 3.5}
# Where the window is centred, when not on the robot; the ground under the
# robot, when nothing is mapped there; the band above the ground, when a
# taller robot's body must be in it.
WINDOW_CENTRES = {"standing_r3.txt": (-1.0, -2.0)}
GROUND_Z = {"standing_r3.txt": -0.16}
ABOVE = {"standing_r3.txt": 2.0}
HALF_WINDOW_M = 3.0
BELOW_M, ABOVE_M = 0.6, 1.2


def load(path):
    b = open(path, "rb").read()
    assert b[:8] == b"SDMGRID1"
    ms = struct.unpack("<I", b[8:12])[0]
    meta = json.loads(b[12:12 + ms])
    off = 12 + ms
    no, nf, ns = meta["occupied_count"], meta["free_count"], meta["surface_count"]
    occ = np.frombuffer(b, "<i8", no * 3, off).reshape(-1, 3)
    off += no * 24
    free = np.frombuffer(b, "<i8", nf * 3, off).reshape(-1, 3)
    off += nf * 24
    s = np.frombuffer(b, dtype=[("x", "<i8"), ("y", "<i8"), ("z", "<f8")],
                      count=ns, offset=off)
    return meta["resolution_m"], occ, free, s


def main(src, dst):
    for name, grid, x, y, what, unknown in CASES:
        res, occ, free, surf = load(f"{src}/{grid}")
        half = HALF_WINDOWS.get(name, HALF_WINDOW_M)
        cx, cy = WINDOW_CENTRES.get(name, (x, y))
        i0, i1 = math.floor((cx - half) / res), math.floor((cx + half) / res)
        j0, j1 = math.floor((cy - half) / res), math.floor((cy + half) / res)
        inside = lambda a: (a[:, 0] >= i0) & (a[:, 0] <= i1) & (a[:, 1] >= j0) & (a[:, 1] <= j1)
        o = occ[inside(occ)]
        top = {}
        for sx, sy, sz in zip(surf["x"], surf["y"], surf["z"]):
            if i0 <= sx <= i1 and j0 <= sy <= j1:
                k = (int(sx), int(sy), int(math.floor(sz / res)))
                top[k] = max(top.get(k, -1e9), float(sz))
        # The ground under the robot: the highest surface in its column below 1 m.
        ci, cj = math.floor(x / res), math.floor(y / res)
        ground = GROUND_Z.get(name)
        if ground is None:
            ground = max(v for (a, b, c), v in top.items() if a == ci and b == cj and v < 1.0)
        k0 = math.floor((ground - BELOW_M) / res)
        k1 = math.floor((ground + ABOVE.get(name, ABOVE_M)) / res)
        band = lambda a: a[(a[:, 2] >= k0) & (a[:, 2] <= k1)]
        o = band(o)
        f = band(free[inside(free)])
        hidden = lambda c: any((int(c[0]), int(c[1])) == ij and lo <= int(c[2]) <= hi
                               for ij, lo, hi in unknown)
        o = np.array([c for c in o if not hidden(c)]).reshape(-1, 3)
        f = np.array([c for c in f if not hidden(c)]).reshape(-1, 3)
        with open(f"{dst}/{name}", "w") as out:
            out.write(f"# {what}\n# From {grid} by make_departure_fixtures.py; see README.md.\n")
            out.write(f"resolution {res}\nrobot {x} {y} {ground:.3f}\n")
            for c in sorted(map(tuple, o)):
                t = top.get(tuple(int(v) for v in c))
                out.write(f"o {c[0]} {c[1]} {c[2]}" + (f" {t:.3f}" if t is not None else "") + "\n")
            for c in sorted(map(tuple, f)):
                out.write(f"f {c[0]} {c[1]} {c[2]}\n")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
