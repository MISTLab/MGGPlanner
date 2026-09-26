# Planner-grid fixtures for test_mola_map

Pieces of the SwarmDeck run-5 planner grids (2026-09-25, `diag-run5`),
the SDMGRID1 grids MGG planned on, cut read-only around robots that stood
boxed in.

| file | grid | where |
|---|---|---|
| `boxed_in_r0.txt` | robot_0 (Bunker), `r0.sdpg` | (4.16, -5.28) odom, the SE corner of a room, boxed in at 1790345307 |
| `boxed_in_r1.txt` | robot_1 (Bunker), `r1.sdpg` | (53.76, -40.92) odom, beside a wall column, boxed in 43 times from 1790345621 |
| `ledge_r0.txt` | robot_0 (Bunker), its r1490 grid | (137.8, -79.5), the ledge over a 4 m pit it was parked at (1790349295) and fell into; the pit floor lies below the cut, as it lay unobserved before the fall |
| `turn_r1.txt` | robot_1 (Bunker), its r1615 grid | (76.60, -43.76), where it was sent a 170 degree turn at 1790348990.57; the wall-foot cells within its turning circle (voxels 1 to 4 over the floor of six columns) are left out, unknown, as they were before it faced them |
| `standing_r3.txt` | robot_3 (Spot), its r15 grid of run 6 (`diag-run6`, 2026-09-25) | (-2.02, -2.00) component frame, where it stood from its start, never moving; robot_1 (Bunker) started 2 m east. No ground was observed within about 2.4 m of robot_3 (4.8 m east, behind robot_1's body), so the ground height is given (-0.16, the hangar floor); the window is centred on (-1.0, -2.0), 3.5 m either way, from 0.6 m below to 2.0 m above the ground |

The r0 and r1 grids were pulled after the operator drove those robots out; the
walls around the stuck poses are unchanged. r0n and r1n (robot_0's r1490 and
robot_1's r1615) were pulled read-only from the mapping volume after the events,
and robot_3's r15 grid of run 6 (r3r15) the same way by the run-6 diagnosis.

Each file lists `resolution` (0.2 m), `robot x y ground_z`, every occupied
cell as `o ix iy iz [surface_max_z]` and every free cell as `f ix iy iz`,
within 3 m (2.5 m for the ledge and turn) in x and y of the robot and between
0.6 m below and 1.2 m above the ground under it, unless the table says
otherwise. Other cells are unknown.

`make_departure_fixtures.py <grid dir> <output dir>` regenerates the files
from r0.sdpg, r1.sdpg (diag-run5/data), r0n.sdpg, r1n.sdpg and r3r15.sdpg.
