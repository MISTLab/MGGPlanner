#!/usr/bin/env python3
"""Cut the departure fixtures out of SwarmDeck run-5 planner grids (SDMGRID1).

usage: make_departure_fixtures.py <diag-run5 data dir> <output dir>

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
    # name, grid, robot position (odom), what it is
    ("boxed_in_r0.txt", "r0.sdpg", 4.16, -5.28,
     "robot_0 (Bunker), boxed in at 1790345307 in a room's SE corner"),
    ("boxed_in_r1.txt", "r1.sdpg", 53.76, -40.92,
     "robot_1 (Bunker), boxed in from 1790345621 beside a wall column"),
]
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
    for name, grid, x, y, what in CASES:
        res, occ, free, surf = load(f"{src}/{grid}")
        i0, i1 = math.floor((x - HALF_WINDOW_M) / res), math.floor((x + HALF_WINDOW_M) / res)
        j0, j1 = math.floor((y - HALF_WINDOW_M) / res), math.floor((y + HALF_WINDOW_M) / res)
        inside = lambda a: (a[:, 0] >= i0) & (a[:, 0] <= i1) & (a[:, 1] >= j0) & (a[:, 1] <= j1)
        o = occ[inside(occ)]
        top = {}
        for sx, sy, sz in zip(surf["x"], surf["y"], surf["z"]):
            if i0 <= sx <= i1 and j0 <= sy <= j1:
                k = (int(sx), int(sy), int(math.floor(sz / res)))
                top[k] = max(top.get(k, -1e9), float(sz))
        # The ground under the robot: the highest surface in its column below 1 m.
        ci, cj = math.floor(x / res), math.floor(y / res)
        ground = max(v for (a, b, c), v in top.items() if a == ci and b == cj and v < 1.0)
        k0, k1 = math.floor((ground - BELOW_M) / res), math.floor((ground + ABOVE_M) / res)
        band = lambda a: a[(a[:, 2] >= k0) & (a[:, 2] <= k1)]
        o = band(o)
        f = band(free[inside(free)])
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
