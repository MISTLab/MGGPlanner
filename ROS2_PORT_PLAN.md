# MGGPlanner: ROS 2 Packaging and Port Plan

**Goal:** a packaged MGG planner (the grid + graph exploration core) that runs
under ROS 2, is testable in ARGoS with the Filament photorealism renderer,
deployable on real robots, and able to take its map and its inter-robot frame
alignment from Swarm-SLAM.

The emphasis is *packaging*, not a faithful port. Everything that exists only
to serve the original Gazebo/SMB demo is dropped.

---

## 1. What is actually being kept

### 1.1 The core

MGG = **M**ulti-robot **G**rid **G**raph. Two things matter:

- **The grid local planner**: `buildGridGraph` / `buildGridGraphExapnd`
  (`rrg.cpp:1419-1572`), configured by `BoundedSpaceParams/GridGraphLocal`
  and gated on `planning_params_.build_grid_local_graph`. This is the speed
  claim: a regular grid over the local bound instead of RRG sampling.
- **The global graph and its multi-robot merge**:
  `neighbourGraphCallback` / `updateNeighbourGraph` (`rrg.cpp:231-390`),
  exchanging `planner_msgs/Graph` between robots.

### 1.2 Measured ROS coupling (this is the key enabler)

| File | Lines | ROS-coupled | % |
|---|---:|---:|---:|
| `rrg.cpp` (grid + graph + merge) | 6,325 | 373 | **5%** |
| `graph.cpp` | 155 | 0 | **0%** |
| `trajectory.cpp` | 360 | 0 | **0%** |
| `geofence_manager.cpp` | 327 | 8 | 2% |
| `random_sampler.cpp` | 272 | 10 | 3% |
| `graph_manager.cpp` | 462 | 24 | 5% |
| `params.cpp` | 1,271 | 119 | 9% |
| `mggplanner_rviz.cpp` | 2,632 | 516 | 19% |
| `mggplanner.cpp` (service surface) | 369 | 93 | 25% |

The algorithm is already nearly ROS-free. ROS lives at the edges: parameters,
visualization, and the service surface. **Extracting a ROS-free core is
therefore mostly moving files and abstracting logging, timing, and the
graph-exchange message type**, not restructuring the algorithm.

**But the map abstraction was not real until phase 1 fixed it.**
`map_manager_voxblox_impl.h:125` read `class MapManagerVoxblox : MapManager`,
with no access specifier, so `class` gave it **private inheritance**.
`MapManager` was an inaccessible base: no code could hold a `MapManager*`
pointing at the voxblox implementation, and the compiler rejected the attempt
with "'MapManager' is an inaccessible base of ...". The abstract interface in
`map_manager.h` was decorative. That, rather than any single method, is why
the concrete templated type was threaded through `rrg.h`, `mggplanner.h` and
`adaptive_obb`. Phase 1 makes the inheritance public and promotes the six
concrete-only methods the planner and `adaptive_obb` actually call, which is
what turns `MapInterface` in section 2.1 from a rewrite into a rename.

The residual 5% in `rrg.cpp` is `ROS_INFO`-style logging, `ros::Time` timing,
and `planner_msgs::Graph` in the merge path. Logging and timing are trivially
abstracted. The graph message is a genuine interface question, resolved in
2.2 below.

### 1.3 Dropped

`planner_gazebo_sim`, `local_planner` (2,633 LOC CMU terrain follower),
`smb_teleop_twist_joy`, `visualization_tools`, `mggplanner_ui`, and the whole
`rotors_simulator` / `smb_simulator` / `subt_cave_sim` / `gazebo_ros` tree.
`smb_path_tracker` is retained only as an optional reference path follower.

**voxblox is dropped.** See section 4.

### 1.4 External dependencies (findings from phase 0/1)

- **`adaptive_obb_ros`** (`ntnu-arl/adaptive_obb_ros`). **Not an independent
  library.** It build-depends on `planner_common`, `#include`s
  `planner_common/map_manager_voxblox_impl.h`, and stores
  `MapManagerVoxblox<Server,Voxel>*` as a member, so it cannot survive the
  voxblox removal. It also *blocks the phase 1 abstraction restore*: its
  constructor takes the concrete type, so pointing `Rrg::map_manager_` at
  the abstract interface fails to compile against it. Mitigating facts: it is
  only **264 lines** and calls exactly one map method, the 4-argument
  `getLocalPointcloud`. Recommend vendoring it and switching that member to
  `MapManager*`.
- **`pci_general`** (`ntnu-arl/pci_general`). Provides `pci_general_ros_node`,
  the concrete `PCIManager` that `planner_control_interface` leaves abstract.
  Being replaced by a minimal `mgg_pci` (phase 5) rather than ported.
- **Dissolved into system packages:** `catkin_simple`, `eigen_catkin`,
  `yaml_cpp_catkin` become `libeigen3-dev` / `libyaml-cpp-dev` under ament.
- **Gone with voxblox:** `protobuf_catkin`, `glog_catkin`, `gflags_catkin`,
  `minkindr`, `minkindr_ros`, `eigen_checks`.

Build environments for both sides live in `docker/`: `noetic-verify.Dockerfile`
(ROS 1, for verifying changes to the existing tree and capturing the
behavioural baseline, since there is no ROS on the host) and
`jazzy-dev.Dockerfile` (ROS 2 target).

---

## 2. Target package layout

```
mgg_core/          ROS-free C++17 library.  Eigen only.
  map_interface.h      ternary occupancy queries (was MapManager)
  pose_source.h        current pose + keyframe anchors
  grid_graph.h         local grid exploration graph      <- the "grid"
  global_graph.h       global topological graph          <- the "graph"
  graph_merge.h        multi-robot merge; offsets INJECTED, not hardcoded
  gain.h               volumetric gain
  params.h             plain structs
  test/                unit tests against synthetic maps

mgg_msgs/          planner_msgs, cleaned and made ROS 2 legal
mgg_ros/           rclcpp node: params, services, TF, viz, core<->msg conversion
mgg_map_octomap/   MapInterface from PointCloud2 + TF (standalone / ARGoS / robot)
mgg_cslam/         MapInterface + PoseSource from Swarm-SLAM   <- section 3
mgg_pci/           control interface; path out to whatever tracks it
mgg_argos/         ARGoS bridge, controller, experiments, compose  <- section 5
```

The payoff of this layering is that **the robot deployment and the ARGoS test
run use the same packages**, differing only in which `MapInterface`
implementation and which control back end are loaded.

### 2.1 What `MapInterface` must provide

Twelve methods, roughly 63 call sites: resolution; voxel / box / path / ray
status as ternary unknown-occupied-free; `getScanStatus` (raycast and tally
for volumetric gain); `augmentFreeBox` and `augmentFreeFrustum`; local map
extraction; reset. Nothing else. This is why voxblox is removable (section 4)
and why a Swarm-SLAM-backed map is possible at all.

### 2.2 The multi-robot graph exchange

`planner_msgs/Graph` in the merge path is a **communications** concern, not an
algorithmic one. Keep it as a ROS message, convert at the `mgg_ros` boundary,
and have `mgg_core::graph_merge` operate on plain structs. This keeps the core
ROS-free while leaving the exchange swappable (DDS today, Zenoh or a radio
link on deployment).

---

## 3. Swarm-SLAM integration

### 3.1 There is already vendored intent, unwired

`planner_msgs` contains `PoseGraph.msg`, `PoseGraphValue.msg`,
`PoseGraphEdge.msg`, `MultiRobotKey.msg`, `InterRobotLoopClosure.msg`, and
`IntraRobotLoopClosure.msg`. I diffed these against
`cslam_common_interfaces` inside the local `swarmslam` image: **they are
Swarm-SLAM's definitions**, copied in (`PoseGraph` is a simplified subset
without the logging fields). **Zero lines of code reference any of them.**
Someone set this up and stopped.

### 3.2 Be clear about what Swarm-SLAM does and does not give you

Swarm-SLAM does **not** publish a dense map. Verified from the source in the
container, the relevant outputs are:

| Topic | Type | Use |
|---|---|---|
| `cslam/keyframe_data` | `KeyframePointCloud` (id + PointCloud2) | the geometry |
| `cslam/keyframe_odom` | `KeyframeOdom` (id + Odometry) | keyframe poses |
| `/rN/cslam/optimized_estimates` | `OptimizationResult` (factors + estimates) | **optimized multi-robot keyframe poses** |
| `cslam/viz/pose_graph` | `PoseGraph` | the graph |
| `cslam/reference_frames` | `ReferenceFrames` | **inter-robot frame alignment** |

(`decentralized_pgo.cpp:98-195`.)

So "a map coming from Swarm-SLAM" means: `mgg_cslam` **assembles** the
occupancy map as the sum of keyframe clouds transformed by their optimized
keyframe poses. Swarm-SLAM supplies geometry and corrected poses; MGG supplies
the volumetric representation.

### 3.3 The hard part: loop-closure deformation

When the pose graph re-optimizes, past keyframe poses jump. An incrementally
built occupancy map is then wrong. Recommended design:

**Keyframe-anchored submaps.** Each submap is a small occupancy volume anchored
to one cslam keyframe. On `optimized_estimates`, submaps are **re-placed by
their new keyframe poses**, never re-integrated from raw data. Precedent:
`voxgraph` does exactly this over voxblox.

The elegant part, and the reason this fits MGG specifically:

- **MGG's global graph and cslam's pose graph are the same kind of object.**
  Anchor each global-graph vertex to its nearest keyframe and deformation
  comes for free: vertices move with their anchors on every optimization.
- **The local grid graph is rebuilt every planning cycle anyway**, so it only
  needs a *locally* consistent map. It reads the current submap directly and
  is unaffected by global deformation.

This gives a clean split: local planning uses live local geometry, global
planning uses a deformable anchored graph.

### 3.4 The real prize: replacing the hardcoded offsets

Inter-robot alignment is currently **hardcoded C++ constants**
(`rrg.cpp:3092-3181`), under the authors' own comment:

```
// Sloppy way of init pose. TODO: got to do it better.
```

Robots must physically start at known relative offsets, (0,-2,0), (0,-4,0),
and so on. This is why the merge code reads
`init_offsets_[v.robot_id-1]` at `rrg.cpp:252`, `316`, `361`.

Swarm-SLAM's `optimized_estimates` and `ReferenceFrames` **estimate** those
transforms from inter-robot loop closures. Wiring them in means robots no
longer need known start poses and can align opportunistically on rendezvous.
That is not a bolt-on; it completes the design the TODO is asking for, and it
is the single highest-value item in this plan.

Concretely: `graph_merge` takes the offsets as an **injected dependency**
(`PoseSource`), with two implementations: a static one reading the current
constants from config (preserves today's behaviour, keeps ARGoS bring-up
simple) and a cslam one tracking `optimized_estimates`.

---

## 4. Mapping backend: drop voxblox

Verified against the code:

1. **Only ternary occupancy is ever queried.** `getVoxelStatus`
   (`voxblox_common_impl.cpp:114-128`) collapses the TSDF: `weight < 1e-6`
   -> unknown, `distance <= 1.0 * voxel_size` -> occupied, else free. The
   distance field is thresholded at one voxel and discarded.
2. **All continuous-distance code is compiled out.** `getPointDistance` (the
   only `voxblox::Interpolator` user) has 7 call sites, all inside
   `COL_CHECK_METHOD == 1|2` or `EDGE_CHECK_METHOD == 2|3`. The active build
   is `COL_CHECK_METHOD=0`, `EDGE_CHECK_METHOD=1`
   (`planner_common/CMakeLists.txt:10-20`).
3. **The one live call site is config-disabled.** `rrg.cpp:534` is guarded by
   `interpolate_projection_distance`, which is `false` in all four shipped
   configs and defaults to `false` (`params.cpp:1110`).
4. **The `rrg.h` concrete-type dependency is cosmetic**, present only because
   `getPointDistance` is absent from the abstract interface.

Dropping voxblox also removes `protobuf_catkin`, `glog_catkin`,
`gflags_catkin`, `minkindr`, `minkindr_ros`, and `eigen_checks`.

**Recommendation: OctoMap** for `mgg_map_octomap`. ROS 2 native in Jazzy, and
its semantics map one-to-one onto `MapInterface`: null node is unknown,
`isNodeOccupied`, `computeRayKeys` / `castRay`, `insertPointCloud`. gbplanner1
shipped an OctoMap map manager, so the interface is known to be satisfiable.
For `mgg_cslam`, each keyframe submap is small, so a per-submap octree is a
good fit.

**Fallback** if octree lookup dominates the hot loops (27 `getPathStatus`
sites): a flat voxel-hash occupancy grid behind the same interface, roughly
600 to 900 LOC including an Amanatides-Woo DDA.

**Caveat to validate:** TSDF truncation and log-odds raycasting disagree about
free space near surfaces, so frontier positions and gain tallies will not be
bit-identical to published runs. Capture a ROS 1 baseline before touching
anything and compare.

### 4.1 Result of that comparison (open question)

The baseline was captured (`tools/map_baseline/baseline_ros1.csv`) and
`mgg_map_octomap` was measured against it on the same scene and query battery.

Agrees: voxel classification (free inside, occupied on surfaces, unknown
outside), ray verdicts, path verdicts, resolution.

Differs, understood: OctoMap reports more free and less unknown on the voxel
lattice (1422/2512 against voxblox's 1047/2874), because it carves along every
ray to `max_range` and its obstacles lack the truncation band, making them
roughly a voxel thinner. `occupied_dilation_voxels` can compensate; it
defaults to 0.

Also learned: voxblox's `getScanStatus` deduplicates shared free space across
neighbouring rays via its `starting_points` matrix, so the correct counterpart
is `getScanStatusIterative`, not the plain variant.

**Differs, decided:** the baseline ranks the room centre above the corner for
unknown volume (2361 against 1798); exact traversal ranks them the other way.

The difference traces to `nonuniform_ray_cast_`, on by default in the voxblox
implementation. It grows the ray step with distance and multiplies each sample
up by `ceil(step_size/og_step_size)`, so a ray can step straight over a
one-voxel-thick wall and bank the unknown space behind it. That inflates the
count from viewpoints whose walls are far away, which is why the room centre
scored highest.

**Decision: keep exact traversal.** `mgg_map_octomap` counts what its rays
actually pass through, using OctoMap's own `computeRayKeys` traversal, with no
step-size extrapolation. Reproducing the voxblox numbers would mean
reproducing a sampling artifact deliberately.

Consequence, accepted: **exploration decisions will differ from the published
ROS 1 runs.** Frontier rankings between distant and nearby viewpoints are the
place to expect it. Any quantitative comparison against the paper's results
has to account for this rather than treat the two as interchangeable.

The property chosen is pinned by
`OctomapMap.GainCountsEachTraversedVoxelExactlyOnce` rather than by a
scene-specific ordering, which would be brittle.

### 4.2 Sensor ray geometry corrected

Related, and also a deliberate divergence. The ROS 1 ray table was built as

    max_range * (cos dh, sin dh, sin dv)

which is not a unit-sphere parameterisation: ray length came out as
`max_range * sqrt(1 + sin^2 dv)`, so rays at the vertical extremes overshot the
configured range by about 3% (20.66 m for a nominal 20 m at +/-15 degrees) and
the sensor swept a barrel rather than a spherical cap. `mgg_core` uses the
spherical form, scaling the horizontal components by `cos(dv)`, so every ray
ends exactly `max_range` away.

This shifts gain values slightly against the recorded baseline, in addition to
the traversal change above.

One artifact of the original is **not** corrected: the ray table holds 438
endpoints for the shipped VLP-16 configuration rather than the 432 that
360/5 x 30/5 implies, because the loop runs `dh < h_lim` and accumulated
floating-point steps leave room for a 73rd azimuth step. Changing it would
alter the ray density itself. Flagged here as a separate call if the exact
sensor discretisation ever matters.

---

## 5. ARGoS harness

`../argos3-examples` already provides most of it: `swarm_slam_bridge`
(loop function streaming RGB-D + odom + IMU over a Unix socket, ARGoS never
links ROS 2), `bridge_node.py` (the rclpy side, publishing `/clock` and
per-robot topics), `bistro_footbot_live.argos`, and a
`ros:jazzy-perception` Docker precedent.

**Missing for a planner, as opposed to a SLAM consumer:**

1. **The bridge is one-way.** A planner must drive. Extend the protocol so the
   reply carries per-robot commands instead of a bare `ACK`.
2. **No controller accepting external commands.** `swarm_slam_footbot` drives
   a hardcoded route.
3. **No `PointCloud2`** (project the depth image) and **no TF**.
4. **FOV mismatch.** Config assumes a VLP-16 (360 deg x 30 deg); the foot-bots
   have one 60 deg camera. Since gain is computed from declared FOV,
   recommend a **ring of 6 depth-only cameras** merged into one cloud, which
   preserves the planner's tuning. Alternative is retuning `SensorParams`,
   which changes exploration behaviour and tends to make robots spin to look
   around.
5. **Scale.** Foot-bots are ~0.17 m; configs assume an SMB at 0.8 m. Robot
   size, local bound (15 m), and velocities all need retuning.

Because ARGoS feeds real RGB-D through the same bridge Swarm-SLAM already
consumes, **the ARGoS harness can exercise the full cslam integration**, not
just the planner. That is the natural end-to-end test.

---

## 6. ROS 2 conversion mechanics

These apply to `mgg_ros`, `mgg_msgs`, and the edges of the core.

**Interfaces (empirically verified with `rosidl_adapter` from
`ros:jazzy-perception`: 15 parsed, 28 failed):**

- All 24 `.srv` use `-------` (7 dashes); ROS 2 requires exactly `---`.
- All 24 `.srv` are `snake_case`; ROS 2 derives type names from filenames, so
  `planner_set_vel.srv` yields the invalid `planner_set_vel_Request`. Rename
  to CamelCase.
- `kFoo` constants violate `^[A-Z]([A-Z0-9_]?[A-Z0-9]+)*$` in `BoundMode`,
  `ExecutionPathMode`, `PlanningMode`, `TriggerMode`, and inline in several
  `.srv`. This ripples into C++; ship a compat header mapping old spellings.
- Rename `pathFollowerAction.action` to `PathFollower.action`.
- Drop the vendored cslam messages; `mgg_cslam` depends on
  `cslam_common_interfaces` directly. Only that package takes the dependency.
- Bare `Header header` is fine (`rosidl_adapter` maps it), and `.msg`
  filenames are already CamelCase.

**Parameters:** 285 reads use `ros::param::get("Ns/sub/name", v)` against
nested YAML. ROS 2 uses `.`, requires declaration, has no global parameter
server. Do not hand-rewrite: in `mgg_core` these become plain structs, and
`mgg_ros` populates them through one `ParamClient` using
`automatically_declare_parameters_from_overrides(true)`.

**Executors (will bite):** the PCI calls the planner service synchronously
from inside a callback, which deadlocks on a `SingleThreadedExecutor`. Use
`MultiThreadedExecutor` with a `ReentrantCallbackGroup` for service clients,
by design rather than after debugging a hang.

**Timers:** `ros::Timer` becomes `create_timer` with the node clock, **not**
`create_wall_timer`, or timers ignore `/clock`.

**tf:** 118 `tf::` uses, zero `tf2`. All must become `tf2`.

---

## 7. Phases

**Phase 0 [DONE]: baseline and skeleton (3 to 4 days).** Build the Jazzy dev image.
Capture a ROS 1 behavioural baseline (gain values, frontier positions on a
fixed cloud) as the reference for the OctoMap swap. Stand up the workspace on
a `ros2` branch so the ROS 1 tree stays runnable for comparison.

**Phase 1 [DONE]: prune, on the ROS 1 tree (2 to 3 days).** Delete the dead
distance-field paths (`COL_CHECK_METHOD != 0`, `EDGE_CHECK_METHOD != 1`,
`getPointDistance`, `getVoxelDistance`, the `interpolate_projection_distance`
branch), then revert `rrg.h`'s 3 declarations and `rrg.cpp:15` to
`MapManager*`. Pure deletion; proves section 4 empirically; worth doing to the
original codebase regardless.

**Phase 2 [STARTED]: `mgg_core` extraction (1.5 to 2.5 weeks).** Move the algorithm into
a ROS-free library. Abstract logging, timing, and the graph-exchange type.
Convert `params.cpp` to plain structs. **Add unit tests against synthetic
maps** (this is the packaging deliverable: the core becomes testable without
ROS, ARGoS, or a robot).

Carried over from phase 1, as interface-design changes rather than deletions:

- **Drop the `tsdf_dist` out-parameter from the 5-argument `getRayStatus`.**
  After the phase 1 prune it has exactly one caller (`projectSample`), which
  writes it and never reads it. The sibling `end_voxel` out-parameter *is*
  read and must stay. This is the last TSDF-shaped item in the interface and
  would be meaningless for an occupancy backend.
- **Reconsider `getFreeSpacePointCloud`'s PCL signature.** Phase 1 had to
  promote it (with `getScanStatusIterative`, `setRaycastingParams` and
  `setRobotRadius`) onto the abstract interface to make the planner
  backend-agnostic. It is the only one that drags PCL into what should be a
  dependency-light core; consider returning plain `Eigen::Vector3d` and
  converting at the ROS boundary.

**Phase 3 [DONE]: `mgg_msgs` (2 to 3 days).** The section 6 interface fixes. Verify
by re-running the `rosidl_adapter` check to 43/43.

**Phase 4 [MOSTLY DONE]: `mgg_map_octomap` (4 days to 1.5 weeks).** `MapInterface` over
OctoMap plus a `PointCloud2` + TF front end. Validate against the phase 0
baseline before building on it.

**Phase 5: `mgg_ros` + `mgg_pci` (2 to 3 weeks).** The rclcpp node, service
surface, TF, parameters, executor design, and RViz markers
(`mggplanner_rviz.cpp` is 2,632 lines of mechanical marker code; deprioritize
within the phase). Write a minimal `mgg_pci` rather than porting
`pci_general`: the interface is narrow (trigger, path, execute, home, stop)
and a clean one serves packaging better.

**Phase 6: `mgg_argos` (1.5 to 2 weeks).** Bidirectional bridge protocol v2,
a commandable foot-bot controller, depth-ring to `PointCloud2`, TF, the
experiment file, and compose. Single robot exploring bistro end to end.

**Phase 7: multi-robot with static offsets (1 week).** Three foot-bots, graph
merge via the existing constants moved to config. This validates the merge
independently of Swarm-SLAM.

**Phase 8: `mgg_cslam` (2 to 3 weeks).** Keyframe-anchored submaps, deformation
on `optimized_estimates`, global-graph vertices anchored to keyframes, and
`PoseSource` from cslam replacing the static offsets. End-to-end in ARGoS with
real cslam running.

**Phase 9: robot bring-up (open-ended).** Swap `mgg_map_octomap` or `mgg_cslam`
onto the real sensor topics; swap `mgg_pci`'s back end onto the real
controller. No core changes should be required, which is the test of whether
the packaging worked.

**Total to end of phase 8: roughly 10 to 14 weeks.**

Phases 3, 4, and 6 are largely independent of 2 and 5 and can be parallelized.

---

## 8. Risks

| Risk | Impact | Mitigation |
|---|---|---|
| Core extraction reveals hidden ROS coupling in `rrg.cpp` | Phase 2 slips | The 5% measurement is line-level, not semantic; budget the upper range |
| OctoMap free-space carving differs from TSDF | Frontiers/gain shift vs published runs | Phase 0 baseline, phase 4 comparison gate |
| Loop-closure deformation corrupts the occupancy map | Global planning degrades after closures | Keyframe-anchored submaps (3.3); never re-integrate raw data |
| cslam optimization rate too slow for planning | Planner acts on stale global geometry | Local grid uses live submap; only global graph waits on optimization |
| Service-client deadlock under ROS 2 executors | Hangs resembling planner bugs | Design executors up front (section 6) |
| Narrow-FOV sensor changes exploration behaviour | Not comparable to the paper | Depth camera ring (5.4) |
| OctoMap query cost in hot loops | Planning rate drops | Voxel-hash fallback behind the same interface |
| Filament + Vulkan in Docker | Blocks all-in-Docker | Keep host-ARGoS working as fallback |

## 9. Status

Phases 0, 1 and 3 are complete and verified in Docker. Phase 4 has a working
OctoMap backend measured against the baseline, with the gain-ordering question
in 4.1 outstanding and a PointCloud2/TF front end still to write. Phase 2 has
its map contract and value types; the algorithm itself (grid graph, global
graph, merge) is still in the ROS 1 tree.

Next: finish phase 4's ROS front end, then move the planner core across.

## 10. Recommended first move

Phases 0 and 1 are cheap, are pure improvements to the existing ROS 1 codebase,
and de-risk the largest assumption in the plan. Start there before committing
to the rest.
