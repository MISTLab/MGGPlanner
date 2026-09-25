# Planner-grid fixtures for test_mola_map

Pieces of the SwarmDeck run-5 planner grids (2026-09-25, `diag-run5`),
the SDMGRID1 grids MGG planned on, cut read-only around robots that stood
boxed in.

| file | grid | where |
|---|---|---|
| `boxed_in_r0.txt` | robot_0 (Bunker), `r0.sdpg` | (4.16, -5.28) odom, the SE corner of a room, boxed in at 1790345307 |
| `boxed_in_r1.txt` | robot_1 (Bunker), `r1.sdpg` | (53.76, -40.92) odom, beside a wall column, boxed in 43 times from 1790345621 |

The r0 and r1 grids were pulled after the operator drove those robots out; the
walls around the stuck poses are unchanged.

Each file lists `resolution` (0.2 m), `robot x y ground_z`, every occupied
cell as `o ix iy iz [surface_max_z]` and every free cell as `f ix iy iz`,
within 3 m in x and y of the robot and between 0.6 m below and 1.2 m above
the ground under it. Other cells are unknown.

`make_departure_fixtures.py <diag-run5 data dir> <output dir>` regenerates
the files.
