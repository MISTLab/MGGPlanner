# mgg_msgs: what changed from planner_msgs

`mgg_msgs` merges the ROS 1 `planner_msgs` and `planner_semantic_msgs`
packages. 41 interfaces: **16 msg, 24 srv, 1 action**.

The conversion is scripted, not hand-edited, so it can be re-run against the
ROS 1 tree while both exist. Every change below was driven by an actual
failure from ROS 2's own `rosidl_adapter`, not by convention.

## 1. Service separator: `-------` to `---`

All 24 `.srv` files used seven dashes. ROS 1 tolerated it; `rosidl_adapter`
requires exactly three and otherwise reports
`Could not find separator '---' between request and response`.

## 2. One service had no separator at all

`pci_to_waypoint.srv` had **no** separator. ROS 1 accepted that and treated
the whole file as the request with an empty response. The behaviour is load
bearing: `PlannerControlInterface::goToWaypointCallback` reads `req.waypoint`
(`planner_control_interface.cpp:270`). A trailing `---` therefore preserves
the semantics. Its `# Return best path.` comment is a copy-paste leftover
from another service and does **not** mean the field was a response.

## 3. `snake_case` filenames to CamelCase

ROS 2 derives the interface type name from the filename, so
`planner_set_vel.srv` produced the invalid type name
`planner_set_vel_Request`. All 24 services are renamed, for example:

| ROS 1 | ROS 2 |
|---|---|
| `planner_srv.srv` | `PlannerSrv.srv` |
| `pci_to_waypoint.srv` | `PciToWaypoint.srv` |
| `planner_set_vel.srv` | `PlannerSetVel.srv` |

## 4. `kFooBar` constants to `FOO_BAR`

`rosidl_adapter` enforces `^[A-Z]([A-Z0-9_]?[A-Z0-9]+)*$` for constant names.
20 constants were renamed, for example `kExtendedBound` to `EXTENDED_BOUND`,
`kAdaptiveExploration` to `ADAPTIVE_EXPLORATION`, `kForward` to `FORWARD`.

No compatibility alias header is provided. The ROS 1 spellings only appear in
code that is itself being rewritten (`mgg_ros`, `mgg_pci`), so carrying the
old names forward would preserve churn rather than avoid it.

## 5. Bare `Header` must be qualified

`Header header` becomes `std_msgs/Header header`.

This one is a trap: `rosidl_adapter`'s **parser accepts** the bare name, so a
parse-only check passes. The failure comes later, in
`rosidl_generator_type_description`, which resolves `Header` against the
current package and dies with
`FileNotFoundError: .../mgg_msgs/msg/Header.json`. ROS 1 special-cased the
bare name; ROS 2 does not.

## 6. Action renamed

`pathFollowerAction.action` becomes `PathFollower.action`. ROS 2 action files
are CamelCase and carry no `Action` suffix (the suffix is generated).

## 7. Vendored Swarm-SLAM messages dropped

`planner_msgs` contained six definitions copied from Swarm-SLAM's
`cslam_common_interfaces`: `PoseGraph`, `PoseGraphValue`, `PoseGraphEdge`,
`MultiRobotKey`, `InterRobotLoopClosure`, `IntraRobotLoopClosure`. They match
upstream (`PoseGraph` is a subset without the logging fields) and are
referenced by **zero lines of code** in the repository.

They are not carried over. Duplicating another package's message types means
the two silently diverge and cannot be exchanged with real Swarm-SLAM nodes.
The `mgg_cslam` package (phase 8) depends on `cslam_common_interfaces`
directly instead, and it is the only package that takes that dependency.

## Regenerating

    python3 tools/port_msgs.py <repo-root> ros2/src/mgg_msgs
