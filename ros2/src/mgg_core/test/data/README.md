# Terrain fixtures for test_footprint_plane

Ground surfaces from the SwarmDeck SubT run of 2026-09-24 (diag-run4). They
were extracted read-only from the robots' MOLA submaps in their C-SLAM
component frames.

| file | map | driven path |
|---|---|---|
| `rock_field_preplan.txt` | robot_2 (Scout Mini), only the submaps up to keyframe 622, which is what MGG had when it planned the drive onto the rocks | keyframes 612–629, 4.6 m from level floor onto the rock pile's face |
| `rock_field_now.txt` | robot_2, every submap, after the robot stuck on the face at 33 degrees | the same |
| `subt_ramp.txt` | robot_0 (Bunker), the 16 degree SubT ramp it climbed at a steady 15.8 degrees of pitch | keyframes 1285–1328, 21 m |

Each file lists `resolution` (0.2 m, MGG's MOLA grid), the keyframe base
positions as `pose x y z`, and the ground as `cell ix iy top_z`. The ground is
the highest map point in each 0.2 m column within 1.5 m of the path and
between 1.0 m below and 0.3 m above the nearest keyframe's base. That is the
surface a downward ground ray from driving height meets. Free space is not
recorded.

`make_terrain_fixtures.py <diag-run4 data dir> <output dir>` regenerates the
files from the extracted point clouds (`r2_rocks_preplan.npy`,
`r2_rocks.npy`, `r0_ramp.npy`) and the peers' `graph_solution.json`.
