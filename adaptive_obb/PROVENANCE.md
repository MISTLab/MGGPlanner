# adaptive_obb: vendored code

This package is **not original to MGGPlanner**. It was copied from:

- Upstream: <https://github.com/ntnu-arl/adaptive_obb_ros>
- Commit: `7dc24c973d233f63d73f2a8548e6f5227bc422bb` (2021-11-02)
- Author: Mihir Dharmadhikari (NTNU Autonomous Robots Lab)
- Path taken: the `adaptive_obb/` subdirectory (`format_code.sh` omitted)

## Why it was vendored

It could not remain an external dependency. Despite living in its own
repository it is structurally part of this project:

- its `package.xml` build-depends on `planner_common`, so it cannot be built
  without MGGPlanner checked out;
- it `#include`d `planner_common/map_manager_voxblox_impl.h` and stored
  `MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>*`
  directly, so it could not survive the removal of the voxblox backend
  (see section 4 of `ROS2_PORT_PLAN.md`);
- that concrete type in its constructor also blocked the planner from
  holding its map through the abstract `MapManager` interface.

## Local modifications

1. `#include "planner_common/map_manager_voxblox_impl.h"` becomes
   `#include "planner_common/map_manager.h"`.
2. The constructor parameter and the `map_manager_` member become
   `MapManager*`.

The only map method this package calls is the four-argument
`getLocalPointcloud`, which was promoted onto the abstract `MapManager`
interface to support the change. No algorithmic code was touched.

## Licensing: unresolved

**Upstream ships no LICENSE file, and its `package.xml` declares
`<license>TODO</license>`.** The licensing terms are therefore unspecified,
and vendoring it here does not resolve that. Before this repository is
redistributed, the terms should be confirmed with the upstream authors and a
proper license recorded both here and in `adaptive_obb/package.xml`.
