# Map authority, input execution and cancellation

`mggplanner_node` attaches heartbeat, odometry-ingestion, planning-status and both
cancel callback groups to a dedicated `SingleThreadedExecutor` thread. They are
**not automatically attached** by `add_node()`. Embedders of `PlannerNode` must
call `addInputCallbackGroupsTo(input_executor)` and spin that executor on a thread
separate from all planner callbacks. Increasing the planner pool size alone is
not sufficient: callbacks waiting for the planner mutex can consume every thread.
No graph work runs on the input thread. Requests and a planner-thread maintenance
timer apply the short-lock latest-odometry slot.

Relative to the shared planner/PCI namespace (normally `/<robot_id>/mgg`):

- `cancel_planning` (`std_srvs/srv/Trigger`) revokes exploration and objectives.
  Its response acknowledges revocation, not completion of unwinding. Retained
  exploration state is cleared at the next request admission or maintenance tick
  once background planner work releases its lock. The target-setting service
  consumes any queued clear before accepting a new target, so a target set after
  the cancel acknowledgement is not erased by a later maintenance tick.
- `cancel_exploration_planning` (same type) revokes exploration requests only.
  `pci_stop` forwards only here, so a delayed PCI stop never cancels NAVIGATE or
  RETURN_HOME. A delayed stop can still cancel *new exploration*: sequence-bearing
  scoped cancellation is a follow-up, not implemented by Trigger.
- Adapter ordering: clear exploration target → `pci_stop` → `cancel_planning`
  **answered** → `plan_objective`. A newer objective also pre-empts implicitly.
  `plan_objective` itself does not clear the exploration target or ongoing global
  repositioning: NAVIGATE is also used for explore-toward-goal route probes.
  Exploration interrupted by a probe retains that state for its next request.
  Operator dispatch must keep the explicit target clear and acknowledged general
  cancel above; probes do neither. Explicit general cancellation still clears
  exploration state through its queued clear.
- Interrupted exploration returns `PlannerSrv.Response.CANCELLED` (`-4`), with no
  path. PCI reports `waiting`, reason `planning cancelled; retrying automatically`.
  Missing initial observations still return NOT_READY (`-1`) and PCI reports
  acquiring observations. PCI stop's generation guard prevents restarting a
  stopped session; a still-running session may retry after cancellation.

The final exploration cancellation check precedes hysteresis/bootstrap-state
commit and path publication, under the same short cancellation fence. Cancellation
linearized after that commit cannot turn a sent result into a cancelled response.
Partially computed graph topology is not transactional; interruption invalidates
its caches. Component, epoch **or transform** changes reset local/global graphs
and lifted vertex slots. Compatible geometry refinement retains the predecessor
identity through its lease, then installs the successor after lease release.

Footprint-only ground support uses `state + RobotParams.center_offset` (the core
cuboid-centre convention), minus half body height, with oriented XY extents. It
never declares surrounding free space or overrides measured occupied returns.

## Legacy blind bootstrap

PCI's `bootstrap_distance` defaults to **0.0** (disabled), including the shipped
bunker configuration. Do not use unchecked PCI departure to acquire observations
in normal MGG deployments. To explicitly opt into the old behavior, set
`bootstrap_distance: 3.0` (metres, or another positive distance) in the PCI node's
ROS parameters. This is blind forward motion on planner failures/empty or
insufficient paths, **not an MGG-validated path**. NOT_READY and CANCELLED never
use this fallback even when explicitly enabled.

## Timing boundary

`mgg_core/planning_cancellation.h` is the mgg-astar deadline hook. OR a deadline
into the existing predicate, preserving map authority and operator cancellation;
see the header for nested-scope composition. This lane provides **no 1 s planning
guarantee**. Benchbot tests bound input ingestion and cooperative cancellation
under executor starvation; these are offline bounds, not robot qualification.
