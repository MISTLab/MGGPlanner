# Tour-Based Exploration and Fleet Frontier Assignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Each MGG robot orders its frontier clusters into a committed tour (delivery step 1), and each connected group of robots splits its frontier clusters by a one-round sequential auction run by its lowest robot ID (delivery step 2), all inside MGGPlanner's `ros2` tree.

**Architecture:** The new logic lives in new, ROS-free `mgg_core` files: frontier clusters with stable IDs, graph costs, the open-tour solver, the tour planner with its commitment rule, and the fleet side (exchange types, cluster pool and sequential auction, claim registry, and the coordinator state machine that elects the auctioneer and runs the bid/award rounds). `mgg_msgs` gains `TourBid`/`TourAward` (plus the `TourCluster`/`TourBundle` elements they carry and a `ReleaseClaims` service); `mgg_ros` only converts messages at the frame boundary and wires the core into `PlannerNode`.

**Tech Stack:** C++17, Eigen, Boost.Graph (through `mgg::Graph`), ROS 2 Jazzy (`rclcpp`, rosidl), GoogleTest via `ament_cmake_gtest`, colcon, Docker on the build host `tuf`.

**Spec:** `ros2/docs/2026-09-25-tour-exploration-design.md` (the authority; executors read it alongside this plan).

## Global Constraints

- Scope: MGGPlanner `ros2` branch, native code (not a downstream patch). SwarmDeck consumes it.
- Fleet assignment lives entirely inside MGG, independent of SwarmDeck, with no central server: each connected group of robots runs its own.
- MGG does not choose the shared-frame source: it uses whatever neighbour transforms it receives.
- `Graph.msg` is unchanged, so MGG versions without this feature still interoperate.
- Unchanged: local exploration, terrain and turn checks, boxed-in departures, and the run-5 and run-6 fixes (among them the failed global search's no-path exceptions, `local_gain_remains_now_` and `frontiers_dropped_in_rebuild_`, which the fleet's "exploration complete for this robot" honours too: Task 11). The tour only decides where the robot goes next.
- Out of scope: changing local exploration, gain computation, or terrain/turn checks; optimal multi-robot TSP (the sequential auction heuristic is deliberate); selecting the shared-frame source inside MGG.
- Parameter names and defaults are the spec's §5 table verbatim: `tour.enabled` true, `tour.min_cluster_gain` tuned, `tour.cluster_id_cell_m` 1.0, `tour.heading_weight` tuned, `tour.recompute_interval_s` 1.0, `tour.commit_margin` 0.2, `fleet.enabled` true, `fleet.cluster_merge_radius_m` 2.0, `fleet.balance_weight` tuned, `fleet.auction_interval_s` 2.0, `fleet.bid_deadline_s` 1.0, `fleet.peer_timeout_s` 5.0, `fleet.claim_ttl_s` 1800.
- Tour solve target: < 10 ms for 50 clusters. Median plan time stays under about 350 ms.
- `mgg_core` stays ROS-free (Eigen, Boost, `mgg_kdtree` only); every new algorithm is unit-tested there with in-memory roadmaps, bids and awards (no ROS).
- Both builds are gates for every task: `-DMGG_WITH_OCTOMAP=OFF` (the deployed build) and `-DMGG_WITH_OCTOMAP=ON`. `test_planner_node` builds only with ON; pure `mgg_core` tests run in both.
- Build and test only on `tuf` through the script in "Build and test" below (`docker run --rm --cpus 8`, `MAKEFLAGS=-j3`, one colcon worker, scratch dir `/tmp/mgg-tour` on tuf, removed at the end).
- Compiler flags are the packages' own `-Wall -Wextra -Wpedantic`; no new warnings.
- Never stage or commit anything outside the files a task lists.

## Review Focus

These five inputs are implied by the spec but no requirement names them; each has a test in the task that owns the code.

1. **A malformed or hostile peer bid** (cost arrays not sized to its clusters, a NaN or negative cost, a non-finite pose, a zero cluster ID): it is dropped whole and the receiving robot's group, pool and award are unchanged. Tests: `FleetTypes.WellFormedRejectsInconsistentOrInvalidBids` (Task 7) and `PlannerNodeTest.AMalformedBidIsIgnored` (Task 11).
2. **Out-of-range tour/fleet parameters from YAML** (zero or negative cell size or merge radius, `commit_margin` of 1 or more, NaN, a negative interval): the planner clamps them to working values instead of dividing by zero or never committing. Tests: `TourParams.ClampingRepairsOutOfRangeValues`, `FleetParams.ClampingRepairsOutOfRangeValues` (Task 1), `ParamFixture.LoadsTourAndFleetParams` (Task 1).
3. **Stable-ID edge cases**: two representatives in one ID cell (merge radius below the cell), robot ID 0, negative coordinates: every cluster still gets a distinct, non-zero ID. Test: `FrontierClusters.IdsAreDistinctAndNonZeroInEdgeCases` (Task 2).
4. **A robot that cannot join its graph, or a graph of one vertex**: the tour is empty and every cluster unreachable; nothing crashes and no Dijkstra runs on a missing source. Test: `TourCosts.AnUnlinkedSourceLeavesEveryClusterUnreachable` (Task 4).
5. **An auctioneer that restarts and reuses auction IDs**: its awards are still applied by its peers, which otherwise would ignore them as duplicates forever. Test: `FleetCoordinator.AnAuctioneerThatRestartsStillHasItsAwardsApplied` (Task 10).

---

## Before Task 1

- [ ] **Start from `ros2` at 4777a17 or later.** This plan was refreshed against `ros2` 4777a17 ("Centre the standing-start disk where the robot stood"): the run-5 fixes, the roadmap rebuild from the keyframes, and the run-6 simplification. Every task's code was applied to that tree, built and tested on tuf (2026-09-26): the OFF gate `Summary: 429 tests, 0 errors, 0 failures, 0 skipped`, the ON gate `Summary: 510 tests, 0 errors, 0 failures, 0 skipped` with `test_planner_node` at 51 tests and `test_fleet_exploration` passing in about 8 s. `test_fleet_exploration` failed 3 of about 40 runs on a loaded host, when a late bid let one award hand a peer's fresh frontier to the auctioneer; Task 12 Step 2 describes it.

```bash
git fetch origin
git rebase origin/ros2
git merge-base --is-ancestor 4777a17 HEAD && echo ok   # must print ok
```

What the refresh found in 4777a17, and where the tasks account for it:

- **The low-gain branch of `onPlanRequest` gained two no-path exceptions** (2702fae and 7cf98fe, review r0 I-2): a failed global search is no path, not exploration complete, while `local_gain_remains_now_` holds or the first time after a rebuild dropped this robot's frontiers (`frontiers_dropped_in_rebuild_`). Task 6's replacement block keeps both. The tour's target replaces the low-gain trigger only when the tour has a target; with none, the low-gain rule and its exceptions run as before. Task 11's `settleIdleRobot`, which reports "exploration complete for this robot" when an award leaves it nothing, honours the same two exceptions (a guard and a test that the first version of this plan did not have).
- **Roadmap rebuilds** (`rebuildGlobalGraphFromKeyframes`) replace the global graph, bump `graph_revision_` and drop this robot's frontiers (aeec9aa removed the carry-over). The tour costs with a cache keyed by `graph_revision_` (Task 4), re-finds its target by stable ID every cycle (Task 6's `refreshTour` reads the representative vertex from the fresh clusters), and `runGlobalPlanner` gives a route up when routing rebuilt the graph, after which Task 6 releases the target. Nothing to change.
- **Merged frontiers** (89d3f6c, 3452122): `refreshVertex` takes the owner's frontier mark both ways, so a frontier its owner explored is demoted on every peer by the owner's next broadcast, and `addFrontiers` re-checks a peer's frontier only near the new local graph. A frontier explored by a robot other than its owner is still not demoted: the owner's map does not show that ground, the owner keeps marking it, and its broadcasts re-mark every merged copy. Task 10's memory of explored clusters is kept for that case; its comment and its test now describe it (the test's frontier is the bidder's own, explored by its peer).
- **The heading reference is the current heading** (aeec9aa removed `estimateDirectionFromPath` and `exploring_direction_`, with clearance scoring and the visible-gain wall rule). Task 4's first-leg penalty already takes `current_state_[3]`, the robot's current yaw, so the tour and local path selection prefer the same direction.
- `gain_max_height_above_ground` (7dcf1f9) and the standing-start exemption (4f5bf95, 383c884, 4777a17) change what gain a frontier has and whether a standing robot is boxed in; the tour reads gains and the boxed-in flags as they are. Nothing to change.

Tasks 6, 11 and 12 touch `planner_node.cpp`/`.h` and `test_planner_node.cpp`; they name their anchors (functions and member lines), not line numbers. Where a task's snippet quotes surrounding code, re-read the current code first and keep the run-5 and run-6 changes (`goes_nowhere`, `mgg::Departure`, `local_gain_remains_now_`, `frontiers_dropped_in_rebuild_`, the rebuild triggers) intact.

## Build and test

Save this as `/tmp/mgg-tour/tuf-run.sh` on the workstation (it is the lane script of `mgg-routes`, pointed at this lane) and `chmod +x` it. Run it from inside this repository.

```bash
#!/bin/bash
# usage: tuf-run.sh on|off test|quick [PKG BIN FILTER]
# Syncs this lane's MGG sources to tuf:/tmp/mgg-tour/ws-<variant>/src and
# builds/tests there with docker run --rm --cpus 8, MAKEFLAGS=-j3, one colcon worker.
set -e
variant=$1; mode=$2; pkg=$3; bin=$4; filt=$5
src=$(git rev-parse --show-toplevel)/ros2/src
ws=/tmp/mgg-tour/ws-$variant
ssh -o BatchMode=yes tuf "mkdir -p $ws/src"
rsync -rlpD --checksum --delete -e "ssh -o BatchMode=yes" $src/ tuf:$ws/src/
if [ "$variant" = on ]; then opt=-DMGG_WITH_OCTOMAP=ON; img=swarmdeck-mgg:voxel-bb45403; else opt=-DMGG_WITH_OCTOMAP=OFF; img=swarmdeck-mgg:latest; fi
build="MAKEFLAGS=-j3 colcon build --parallel-workers 1 --event-handlers console_direct- --cmake-args -DCMAKE_BUILD_TYPE=Release $opt"
case $mode in
  test) cmd="$build > build.log 2>&1 || { grep -B2 -A8 'error' build.log | head -80; exit 1; }; tail -n 2 build.log; rm -rf build/*/test_results; colcon test --parallel-workers 1 --event-handlers console_direct- > test.log 2>&1; colcon test-result --verbose > results.log 2>&1; tail -n 40 results.log" ;;
  quick) cmd="MAKEFLAGS=-j3 colcon build --packages-up-to $pkg --parallel-workers 1 --event-handlers console_direct- --cmake-args -DCMAKE_BUILD_TYPE=Release $opt > quick.log 2>&1 || { grep -B2 -A8 'error' quick.log | head -80; exit 1; }; source install/setup.bash; cd build/$pkg && ./$bin --gtest_filter='$filt' 2>&1 | grep -E '^\[ *(OK|FAILED|PASSED) *\]|Failure|Expected|Which is|Value of|Actual|error|rror:' | head -80" ;;
esac
ssh -o BatchMode=yes tuf "docker run --rm --cpus 8 -u \$(id -u):\$(id -g) -e HOME=/tmp -v $ws:/work -w /work $img bash -c \"source /opt/ros/jazzy/setup.bash; $cmd\""
```

- One test binary: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_tour_solver '*'`.
- The two gates (every task ends with both): `/tmp/mgg-tour/tuf-run.sh off test` and `/tmp/mgg-tour/tuf-run.sh on test`. Expected: the last lines of `results.log` show `0 errors, 0 failures` (the summary line `Summary: N tests, 0 errors, 0 failures, M skipped`).
- A compile failure in `quick`/`test` mode prints the compiler's `error` lines and exits non-zero; that is the "FAIL" of a red step when the code under test does not exist yet.
- Cleanup after the last task (or when abandoning the lane): `ssh -o BatchMode=yes tuf rm -rf /tmp/mgg-tour` and `rm -rf /tmp/mgg-tour` locally.

## File map

Created in `ros2/src/mgg_core`:

| File | Responsibility |
|---|---|
| `include/mgg_core/tour_params.h` | `TourParams`, `FleetParams`, their clamping |
| `include/mgg_core/frontier_clusters.h`, `src/frontier_clusters.cpp` | `ClusterId`, `FrontierCluster`, `extractFrontierClusters`, `ClusterIdRegistry`, `exploredInGraph` |
| `include/mgg_core/tour_solver.h`, `src/tour_solver.cpp` | open tour: exact / NN + 2-opt + Or-opt, `cheapestInsertion` |
| `include/mgg_core/tour_costs.h`, `src/tour_costs.cpp` | `GraphDistanceCache`, `computeTourCosts`, heading penalty |
| `include/mgg_core/tour_planner.h`, `src/tour_planner.cpp` | `TourPlanner` (recompute trigger, commitment), `localPathServesTarget` |
| `include/mgg_core/fleet_types.h`, `src/fleet_types.cpp` | `FleetCluster`, `TourBidData`, `RobotBundle`, `TourAwardData` |
| `include/mgg_core/fleet_auction.h`, `src/fleet_auction.cpp` | `buildClusterPool`, `bidderCosts`, `runSequentialAuction` |
| `include/mgg_core/fleet_claims.h`, `src/fleet_claims.cpp` | `ClaimRegistry` (TTL, releases) |
| `include/mgg_core/fleet_coordinator.h`, `src/fleet_coordinator.cpp` | groups, auctioneer election, bid/award rounds |
| `test/test_tour_params.cpp` … `test/test_fleet_coordinator.cpp` | one test file per source file |

Created in `ros2/src/mgg_msgs`: `msg/TourCluster.msg`, `msg/TourBundle.msg`, `msg/TourBid.msg`, `msg/TourAward.msg`, `srv/ReleaseClaims.srv`.

Created in `ros2/src/mgg_ros`: `include/mgg_ros/fleet_conversions.h`, `src/fleet_conversions.cpp`, `test/test_fleet_conversions.cpp`.

Modified: `mgg_core/CMakeLists.txt`, `mgg_msgs/CMakeLists.txt`, `mgg_ros/CMakeLists.txt`, `mgg_ros/include/mgg_ros/param_loader.h`, `mgg_ros/src/param_loader.cpp`, `mgg_ros/test/test_param_loader.cpp`, `mgg_ros/include/mgg_ros/planner_node.h`, `mgg_ros/src/planner_node.cpp`, `mgg_ros/test/test_planner_node.cpp`, and (Task 13) the spec.

Tasks 1 to 6 are delivery step 1 (the per-robot tour). Tasks 7 to 12 are delivery step 2 (the frontier auction). Task 13 measures both in the SubT simulation.

---

### Task 1: Tour and fleet parameters

**Files:**
- Create: `ros2/src/mgg_core/include/mgg_core/tour_params.h`
- Create: `ros2/src/mgg_core/test/test_tour_params.cpp`
- Modify: `ros2/src/mgg_core/CMakeLists.txt` (test registration)
- Modify: `ros2/src/mgg_ros/include/mgg_ros/param_loader.h` (two declarations)
- Modify: `ros2/src/mgg_ros/src/param_loader.cpp` (two loaders, appended)
- Test: `ros2/src/mgg_ros/test/test_param_loader.cpp` (fixture overrides and one test)

**Interfaces:**
- Consumes: `mgg_ros::ParamLoader::get` (existing).
- Produces: `struct mgg::TourParams { bool enabled; double min_cluster_gain, cluster_id_cell_m, heading_weight, recompute_interval_s, commit_margin; }`, `struct mgg::FleetParams { bool enabled; double cluster_merge_radius_m, balance_weight, auction_interval_s, bid_deadline_s, peer_timeout_s, claim_ttl_s; }`, `inline constexpr double mgg::kMinClusterCellM = 0.05`, `void mgg::clampTourParams(TourParams&)`, `void mgg::clampFleetParams(FleetParams&)`, `bool mgg_ros::loadTourParams(const ParamLoader&, const std::string& ns, mgg::TourParams&)`, `bool mgg_ros::loadFleetParams(const ParamLoader&, const std::string& ns, mgg::FleetParams&)`. Namespaces are `"tour"` and `"fleet"`, so the ROS names are `tour.enabled`, `fleet.claim_ttl_s`, … exactly as in the spec.

`fleet.cluster_merge_radius_m` is also the tour's clustering radius (Task 2: "clusters closer than this are one"), so both structs land now even though the rest of `FleetParams` is only read from Task 10 on.

- [ ] **Step 1: Write the failing core test**

Create `ros2/src/mgg_core/test/test_tour_params.cpp`:

```cpp
// Tests for the tour and fleet parameters (tour-exploration design §5).

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mgg_core/tour_params.h"

namespace {

TEST(TourParams, DefaultsAreTheDesignTable) {
  const mgg::TourParams p;
  EXPECT_TRUE(p.enabled);
  // Tuned in the SubT simulation (Task 13); these are the starting values.
  EXPECT_DOUBLE_EQ(p.min_cluster_gain, 600.0);
  EXPECT_DOUBLE_EQ(p.heading_weight, 2.0);
  EXPECT_DOUBLE_EQ(p.cluster_id_cell_m, 1.0);
  EXPECT_DOUBLE_EQ(p.recompute_interval_s, 1.0);
  EXPECT_DOUBLE_EQ(p.commit_margin, 0.2);
}

TEST(FleetParams, DefaultsAreTheDesignTable) {
  const mgg::FleetParams p;
  EXPECT_TRUE(p.enabled);
  EXPECT_DOUBLE_EQ(p.cluster_merge_radius_m, 2.0);
  EXPECT_DOUBLE_EQ(p.balance_weight, 0.3);  // tuned (Task 13)
  EXPECT_DOUBLE_EQ(p.auction_interval_s, 2.0);
  EXPECT_DOUBLE_EQ(p.bid_deadline_s, 1.0);
  EXPECT_DOUBLE_EQ(p.peer_timeout_s, 5.0);
  EXPECT_DOUBLE_EQ(p.claim_ttl_s, 1800.0);
}

TEST(TourParams, ClampingRepairsOutOfRangeValues) {
  mgg::TourParams p;
  p.min_cluster_gain = -5.0;
  p.cluster_id_cell_m = 0.0;
  p.heading_weight = std::numeric_limits<double>::quiet_NaN();
  p.recompute_interval_s = -1.0;
  p.commit_margin = 1.5;
  mgg::clampTourParams(p);
  EXPECT_DOUBLE_EQ(p.min_cluster_gain, 0.0);
  EXPECT_DOUBLE_EQ(p.cluster_id_cell_m, mgg::kMinClusterCellM);
  EXPECT_DOUBLE_EQ(p.heading_weight, 2.0);  // NaN takes the default
  EXPECT_DOUBLE_EQ(p.recompute_interval_s, 0.0);
  EXPECT_DOUBLE_EQ(p.commit_margin, 0.95);

  mgg::TourParams in_range;
  in_range.commit_margin = 0.35;
  mgg::clampTourParams(in_range);
  EXPECT_DOUBLE_EQ(in_range.commit_margin, 0.35);
}

TEST(FleetParams, ClampingRepairsOutOfRangeValues) {
  mgg::FleetParams p;
  p.cluster_merge_radius_m = -2.0;
  p.balance_weight = -0.1;
  p.auction_interval_s = std::numeric_limits<double>::infinity();
  p.bid_deadline_s = -1.0;
  p.peer_timeout_s = 0.0;
  p.claim_ttl_s = -600.0;
  mgg::clampFleetParams(p);
  EXPECT_DOUBLE_EQ(p.cluster_merge_radius_m, mgg::kMinClusterCellM);
  EXPECT_DOUBLE_EQ(p.balance_weight, 0.0);
  EXPECT_DOUBLE_EQ(p.auction_interval_s, 2.0);  // not finite: the default
  EXPECT_DOUBLE_EQ(p.bid_deadline_s, 0.0);
  EXPECT_DOUBLE_EQ(p.peer_timeout_s, 0.1);
  EXPECT_DOUBLE_EQ(p.claim_ttl_s, 0.0);
}

}  // namespace
```

Register it in `ros2/src/mgg_core/CMakeLists.txt`, inside `if(BUILD_TESTING)`, after the two `test_global_graph` lines:

```cmake
  ament_add_gtest(test_tour_params test/test_tour_params.cpp)
  target_link_libraries(test_tour_params ${PROJECT_NAME})
```

- [ ] **Step 2: Run it to verify it fails**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_tour_params '*'`
Expected: FAIL at compile time with `mgg_core/tour_params.h: No such file or directory`.

- [ ] **Step 3: Write the parameters header**

Create `ros2/src/mgg_core/include/mgg_core/tour_params.h`:

```cpp
// Parameters of tour-based exploration and fleet frontier assignment
// (ros2/docs/2026-09-25-tour-exploration-design.md §5).
//
// Plain structs the host fills in, as params.h is; mgg_ros loads them from
// the `tour.` and `fleet.` ROS parameters (loadTourParams, loadFleetParams)
// and clamps them here, so a mistyped YAML value cannot divide by zero or
// switch commitment off by accident.

#ifndef MGG_CORE_TOUR_PARAMS_H_
#define MGG_CORE_TOUR_PARAMS_H_

#include <algorithm>
#include <cmath>

namespace mgg {

struct TourParams {
  /// Use the tour instead of low-gain-triggered greedy repositioning.
  bool enabled = true;
  /// Clusters whose representative's gain is below this are dropped. Tuned
  /// in the SubT simulation; the starting value is ten unknown voxels at the
  /// deployed unknown_voxel_gain of 60.
  double min_cluster_gain = 600.0;
  /// Grid a representative's position is quantized on for its stable ID,
  /// metres.
  double cluster_id_cell_m = 1.0;
  /// Cost of the first leg's heading change, metres per radian. Tuned; the
  /// starting value makes a U-turn cost about 6 m of driving.
  double heading_weight = 2.0;
  /// Minimum interval between tour solves, seconds.
  double recompute_interval_s = 1.0;
  /// Fraction of the remaining tour cost a new first cluster must save to
  /// replace the current target.
  double commit_margin = 0.2;
};

struct FleetParams {
  /// Bid, run and follow frontier auctions.
  bool enabled = true;
  /// Clusters closer than this are one: in the tour's clustering, in the
  /// auction pool, and when matching a cluster to a claim, metres.
  double cluster_merge_radius_m = 2.0;
  /// Balance penalty on a bidder's bundle tour cost. Tuned.
  double balance_weight = 0.3;
  /// Minimum interval between auctions, and between periodic bids, seconds.
  double auction_interval_s = 2.0;
  /// How long the auctioneer waits for bids, seconds.
  double bid_deadline_s = 1.0;
  /// A robot not heard for this long leaves the group (its claims stay,
  /// see claim_ttl_s), seconds.
  double peer_timeout_s = 5.0;
  /// Claim lifetime of a silent peer, seconds (SwarmDeck SubT sim: 600).
  double claim_ttl_s = 1800.0;
};

/// Smallest ID cell and merge radius, metres: below this a cluster's ID
/// changes with every centimetre its representative moves.
inline constexpr double kMinClusterCellM = 0.05;

namespace detail {
inline double finiteOr(double value, double fallback) {
  return std::isfinite(value) ? value : fallback;
}
}  // namespace detail

/// Non-finite values take the default; lengths, gains and times are at least
/// their floor; commit_margin lies in [0, 0.95].
inline void clampTourParams(TourParams& p) {
  const TourParams d;
  p.min_cluster_gain =
      std::max(0.0, detail::finiteOr(p.min_cluster_gain, d.min_cluster_gain));
  p.cluster_id_cell_m = std::max(
      kMinClusterCellM, detail::finiteOr(p.cluster_id_cell_m,
                                         d.cluster_id_cell_m));
  p.heading_weight =
      std::max(0.0, detail::finiteOr(p.heading_weight, d.heading_weight));
  p.recompute_interval_s = std::max(
      0.0, detail::finiteOr(p.recompute_interval_s, d.recompute_interval_s));
  p.commit_margin = std::clamp(
      detail::finiteOr(p.commit_margin, d.commit_margin), 0.0, 0.95);
}

inline void clampFleetParams(FleetParams& p) {
  const FleetParams d;
  p.cluster_merge_radius_m = std::max(
      kMinClusterCellM, detail::finiteOr(p.cluster_merge_radius_m,
                                         d.cluster_merge_radius_m));
  p.balance_weight =
      std::max(0.0, detail::finiteOr(p.balance_weight, d.balance_weight));
  p.auction_interval_s = std::max(
      0.0, detail::finiteOr(p.auction_interval_s, d.auction_interval_s));
  p.bid_deadline_s =
      std::max(0.0, detail::finiteOr(p.bid_deadline_s, d.bid_deadline_s));
  p.peer_timeout_s =
      std::max(0.1, detail::finiteOr(p.peer_timeout_s, d.peer_timeout_s));
  p.claim_ttl_s =
      std::max(0.0, detail::finiteOr(p.claim_ttl_s, d.claim_ttl_s));
}

}  // namespace mgg

#endif  // MGG_CORE_TOUR_PARAMS_H_
```

- [ ] **Step 4: Run the core test to verify it passes**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_tour_params '*'`
Expected: `[  PASSED  ] 4 tests.`

- [ ] **Step 5: Write the failing loader test**

In `ros2/src/mgg_ros/test/test_param_loader.cpp`, add these overrides to the `opts.parameter_overrides({...})` list in `ParamFixture::SetUp`, just before the `// A rad() expression that was never evaluated` comment:

```cpp
        // Tour and fleet parameters under their design names. claim_ttl_s
        // is a YAML integer, as SwarmDeck's SubT override (600) is written;
        // the merge radius is out of range and is clamped.
        {"tour.enabled", false},
        {"tour.min_cluster_gain", 250.0},
        {"tour.commit_margin", 0.3},
        {"fleet.claim_ttl_s", 600},
        {"fleet.balance_weight", 0.5},
        {"fleet.cluster_merge_radius_m", 0.0},
```

and add this test after `LoadsSensorsAndBuildsTheRayTable`:

```cpp
TEST_F(ParamFixture, LoadsTourAndFleetParams) {
  ParamLoader p(node_.get());
  mgg::TourParams tour;
  ASSERT_TRUE(mgg_ros::loadTourParams(p, "tour", tour));
  EXPECT_FALSE(tour.enabled);
  EXPECT_DOUBLE_EQ(tour.min_cluster_gain, 250.0);
  EXPECT_DOUBLE_EQ(tour.commit_margin, 0.3);
  // Absent from the overrides: the design default survives.
  EXPECT_DOUBLE_EQ(tour.cluster_id_cell_m, 1.0);

  mgg::FleetParams fleet;
  ASSERT_TRUE(mgg_ros::loadFleetParams(p, "fleet", fleet));
  EXPECT_TRUE(fleet.enabled);
  EXPECT_DOUBLE_EQ(fleet.claim_ttl_s, 600.0);
  EXPECT_DOUBLE_EQ(fleet.balance_weight, 0.5);
  EXPECT_DOUBLE_EQ(fleet.cluster_merge_radius_m, mgg::kMinClusterCellM);
}
```

- [ ] **Step 6: Run it to verify it fails**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_ros test_param_loader '*'`
Expected: FAIL at compile time with `'loadTourParams' is not a member of 'mgg_ros'`.

- [ ] **Step 7: Write the loaders**

In `ros2/src/mgg_ros/include/mgg_ros/param_loader.h`, add `#include "mgg_core/tour_params.h"` after `#include "mgg_core/sensor_params.h"`, and after the `loadSensorSet` declaration add:

```cpp
/// The tour's parameters under `<ns>/` (the design's `tour.` names), clamped
/// with mgg::clampTourParams.
bool loadTourParams(const ParamLoader& p, const std::string& ns,
                    mgg::TourParams& out);
/// The fleet's parameters under `<ns>/` (the design's `fleet.` names),
/// clamped with mgg::clampFleetParams.
bool loadFleetParams(const ParamLoader& p, const std::string& ns,
                     mgg::FleetParams& out);
```

In `ros2/src/mgg_ros/src/param_loader.cpp`, before the closing `}  // namespace mgg_ros`, add:

```cpp
bool loadTourParams(const ParamLoader& p, const std::string& ns,
                    mgg::TourParams& out) {
  p.get(ns + "/enabled", out.enabled);
  p.get(ns + "/min_cluster_gain", out.min_cluster_gain);
  p.get(ns + "/cluster_id_cell_m", out.cluster_id_cell_m);
  p.get(ns + "/heading_weight", out.heading_weight);
  p.get(ns + "/recompute_interval_s", out.recompute_interval_s);
  p.get(ns + "/commit_margin", out.commit_margin);
  mgg::clampTourParams(out);
  return true;
}

bool loadFleetParams(const ParamLoader& p, const std::string& ns,
                     mgg::FleetParams& out) {
  p.get(ns + "/enabled", out.enabled);
  p.get(ns + "/cluster_merge_radius_m", out.cluster_merge_radius_m);
  p.get(ns + "/balance_weight", out.balance_weight);
  p.get(ns + "/auction_interval_s", out.auction_interval_s);
  p.get(ns + "/bid_deadline_s", out.bid_deadline_s);
  p.get(ns + "/peer_timeout_s", out.peer_timeout_s);
  p.get(ns + "/claim_ttl_s", out.claim_ttl_s);
  mgg::clampFleetParams(out);
  return true;
}
```

- [ ] **Step 8: Run the loader test to verify it passes**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_ros test_param_loader '*'`
Expected: `[  PASSED  ] 8 tests.`

- [ ] **Step 9: Run both gates**

Run: `/tmp/mgg-tour/tuf-run.sh off test` then `/tmp/mgg-tour/tuf-run.sh on test`
Expected: both end with `0 errors, 0 failures`.

- [ ] **Step 10: Commit**

```bash
git add ros2/src/mgg_core/include/mgg_core/tour_params.h \
        ros2/src/mgg_core/test/test_tour_params.cpp \
        ros2/src/mgg_core/CMakeLists.txt \
        ros2/src/mgg_ros/include/mgg_ros/param_loader.h \
        ros2/src/mgg_ros/src/param_loader.cpp \
        ros2/src/mgg_ros/test/test_param_loader.cpp
git commit -m "Add the tour and fleet parameters of the tour-exploration design"
```

---

### Task 2: Frontier clusters with stable IDs

**Files:**
- Create: `ros2/src/mgg_core/include/mgg_core/frontier_clusters.h`
- Create: `ros2/src/mgg_core/src/frontier_clusters.cpp`
- Create: `ros2/src/mgg_core/test/test_frontier_clusters.cpp`
- Modify: `ros2/src/mgg_core/CMakeLists.txt` (library source, test)

**Interfaces:**
- Consumes: `mgg::GraphManager` (`vertices_map_`, `inService`, `getNearestVertices`), `mgg::Vertex` (`type`, `vol_gain.gain`, `robot_id`, `cluster_id`, `state`).
- Produces:
  - `using mgg::ClusterId = std::uint64_t; inline constexpr ClusterId mgg::kNoCluster = 0;`
  - `ClusterId mgg::makeClusterId(int robot_id, const Eigen::Vector3d& position, double cell_m);`
  - `struct mgg::FrontierCluster { ClusterId id; int owner_robot_id; int representative_vertex_id; Eigen::Vector3d position; double gain; std::vector<int> member_vertex_ids; };`
  - `std::vector<FrontierCluster> mgg::extractFrontierClusters(GraphManager& graph, double merge_radius_m, double min_cluster_gain, double cluster_id_cell_m);` (best gain first)
  - `class mgg::ClusterIdRegistry { public: void stabilize(std::vector<FrontierCluster>& clusters, double match_radius_m); };`
  - `bool mgg::exploredInGraph(GraphManager& graph, const Eigen::Vector3d& position, double radius_m);`

The spec (§2.1) says MGG "already assigns `cluster_id` to global frontier vertices". It does not: `performShortestPathsClustering` sets it on the local graph's leaves, and `addRefPathToGraph` copies only the type and gain into the new global vertices. This task groups the global frontiers itself (best-gain representative, members within `fleet.cluster_merge_radius_m`) and sets `Vertex::cluster_id` to the representative's vertex id.

- [ ] **Step 1: Write the failing test**

Create `ros2/src/mgg_core/test/test_frontier_clusters.cpp`:

```cpp
// Tests for the global graph's frontier clusters and their stable IDs
// (tour-exploration design §2.1).

#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/frontier_clusters.h"

namespace {

using mgg::ClusterId;
using mgg::FrontierCluster;
using mgg::GraphManager;
using mgg::StateVec;
using mgg::Vertex;
using mgg::VertexType;

Vertex* addVertex(GraphManager& graph, double x, double y, VertexType type,
                  double gain, int robot_id = 1) {
  auto* v = new Vertex(graph.generateVertexID(), StateVec(x, y, 0.0, 0.0));
  v->type = type;
  v->robot_id = robot_id;
  v->vol_gain.gain = gain;
  v->vol_gain.is_frontier = type == VertexType::kFrontier;
  graph.addVertex(v);
  return v;
}

TEST(FrontierClusters, GroupsFrontiersAroundTheBestGainWithinTheMergeRadius) {
  GraphManager graph;
  addVertex(graph, 0.0, 0.0, VertexType::kVisited, 0.0);
  Vertex* side = addVertex(graph, 10.0, 0.0, VertexType::kFrontier, 900.0);
  Vertex* best = addVertex(graph, 11.0, 0.0, VertexType::kFrontier, 1000.0);
  Vertex* beyond = addVertex(graph, 13.5, 0.0, VertexType::kFrontier, 700.0);
  Vertex* far = addVertex(graph, 30.0, 0.0, VertexType::kFrontier, 800.0);

  const std::vector<FrontierCluster> clusters =
      mgg::extractFrontierClusters(graph, 2.0, 0.0, 1.0);
  ASSERT_EQ(clusters.size(), 3u);
  // Best gain first; the 13.5 m frontier is 2.5 m from the representative
  // and starts its own cluster.
  EXPECT_EQ(clusters[0].representative_vertex_id, best->id);
  EXPECT_EQ(clusters[0].member_vertex_ids,
            (std::vector<int>{best->id, side->id}));
  EXPECT_DOUBLE_EQ(clusters[0].gain, 1000.0);
  EXPECT_TRUE(clusters[0].position.isApprox(Eigen::Vector3d(11.0, 0.0, 0.0)));
  EXPECT_EQ(clusters[1].representative_vertex_id, far->id);
  EXPECT_EQ(clusters[2].representative_vertex_id, beyond->id);
  EXPECT_EQ(side->cluster_id, best->id);
  EXPECT_EQ(best->cluster_id, best->id);
  EXPECT_EQ(clusters[0].owner_robot_id, 1);
}

TEST(FrontierClusters, DropsLowGainClustersAndVerticesOutOfService) {
  GraphManager graph;
  addVertex(graph, 0.0, 0.0, VertexType::kVisited, 0.0);
  addVertex(graph, 5.0, 0.0, VertexType::kFrontier, 100.0);   // below the gain
  addVertex(graph, 9.0, 0.0, VertexType::kUnvisited, 5000.0);  // not a frontier
  // A quarantined neighbour's frontier: out of service.
  auto* theirs = new Vertex(graph.generateVertexID(),
                            StateVec(20.0, 0.0, 0.0, 0.0));
  theirs->robot_id = 2;
  theirs->type = VertexType::kFrontier;
  theirs->vol_gain.gain = 5000.0;
  graph.addNeighbourVertex(theirs, 77);
  graph.disconnectNeighbourGraph(2);
  Vertex* kept = addVertex(graph, -5.0, 0.0, VertexType::kFrontier, 700.0);

  const std::vector<FrontierCluster> clusters =
      mgg::extractFrontierClusters(graph, 2.0, 600.0, 1.0);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].representative_vertex_id, kept->id);
}

TEST(FrontierClusters, IdsDependOnOwnerAndPositionNotOnVertexIds) {
  GraphManager first;
  addVertex(first, 0.0, 0.0, VertexType::kVisited, 0.0);
  addVertex(first, 10.2, 3.7, VertexType::kFrontier, 900.0);
  GraphManager second;
  addVertex(second, 0.0, 0.0, VertexType::kVisited, 0.0);
  addVertex(second, 1.0, 1.0, VertexType::kVisited, 0.0);
  addVertex(second, 2.0, 1.0, VertexType::kVisited, 0.0);
  addVertex(second, 10.2, 3.7, VertexType::kFrontier, 900.0);

  const auto a = mgg::extractFrontierClusters(first, 2.0, 0.0, 1.0);
  const auto b = mgg::extractFrontierClusters(second, 2.0, 0.0, 1.0);
  ASSERT_EQ(a.size(), 1u);
  ASSERT_EQ(b.size(), 1u);
  EXPECT_NE(a[0].representative_vertex_id, b[0].representative_vertex_id);
  EXPECT_EQ(a[0].id, b[0].id);
  EXPECT_EQ(a[0].id, mgg::makeClusterId(1, Eigen::Vector3d(10.2, 3.7, 0.0), 1.0));
  // Another owner at the same place is another name.
  EXPECT_NE(a[0].id, mgg::makeClusterId(2, Eigen::Vector3d(10.2, 3.7, 0.0), 1.0));
}

TEST(FrontierClusters, RegistryKeepsAnIdWhenTheRepresentativeShifts) {
  GraphManager graph;
  addVertex(graph, 0.0, 0.0, VertexType::kVisited, 0.0);
  Vertex* a = addVertex(graph, 10.2, 0.0, VertexType::kFrontier, 1000.0);
  Vertex* b = addVertex(graph, 11.4, 0.0, VertexType::kFrontier, 900.0);
  mgg::ClusterIdRegistry registry;

  auto before = mgg::extractFrontierClusters(graph, 2.0, 0.0, 1.0);
  registry.stabilize(before, 2.0);
  ASSERT_EQ(before.size(), 1u);

  // The map changed: b now sees more, and represents the cluster from
  // another ID cell. The cluster keeps its name.
  a->vol_gain.gain = 500.0;
  b->vol_gain.gain = 1500.0;
  auto after = mgg::extractFrontierClusters(graph, 2.0, 0.0, 1.0);
  ASSERT_EQ(after.size(), 1u);
  EXPECT_EQ(after[0].representative_vertex_id, b->id);
  EXPECT_NE(after[0].id, before[0].id);  // the raw ID moved with it
  registry.stabilize(after, 2.0);
  EXPECT_EQ(after[0].id, before[0].id);
}

TEST(FrontierClusters, RegistryNamesAClusterFarFromEveryKnownOneAfresh) {
  GraphManager graph;
  addVertex(graph, 0.0, 0.0, VertexType::kVisited, 0.0);
  Vertex* a = addVertex(graph, 10.0, 0.0, VertexType::kFrontier, 1000.0);
  mgg::ClusterIdRegistry registry;
  auto first = mgg::extractFrontierClusters(graph, 2.0, 0.0, 1.0);
  registry.stabilize(first, 2.0);

  // The old frontier was explored; a new one appeared 8 m away.
  a->type = VertexType::kUnvisited;
  addVertex(graph, 18.0, 0.0, VertexType::kFrontier, 1000.0);
  auto second = mgg::extractFrontierClusters(graph, 2.0, 0.0, 1.0);
  registry.stabilize(second, 2.0);
  ASSERT_EQ(second.size(), 1u);
  EXPECT_NE(second[0].id, first[0].id);
  EXPECT_EQ(second[0].id,
            mgg::makeClusterId(1, Eigen::Vector3d(18.0, 0.0, 0.0), 1.0));
}

TEST(FrontierClusters, IdsAreDistinctAndNonZeroInEdgeCases) {
  // Review Focus 3: two representatives in one ID cell (a merge radius
  // below the cell), robot ID 0, and cells either side of zero.
  GraphManager graph;
  addVertex(graph, 50.0, 50.0, VertexType::kVisited, 0.0, 0);
  addVertex(graph, 0.1, 0.1, VertexType::kFrontier, 900.0, 0);
  addVertex(graph, 0.6, 0.1, VertexType::kFrontier, 800.0, 0);
  addVertex(graph, -0.4, 0.1, VertexType::kFrontier, 700.0, 0);
  auto clusters = mgg::extractFrontierClusters(graph, 0.3, 0.0, 1.0);
  ASSERT_EQ(clusters.size(), 3u);
  std::set<ClusterId> ids;
  for (const FrontierCluster& c : clusters) {
    EXPECT_NE(c.id, mgg::kNoCluster);
    ids.insert(c.id);
  }
  EXPECT_EQ(ids.size(), 3u);
  mgg::ClusterIdRegistry registry;
  registry.stabilize(clusters, 0.3);
  ids.clear();
  for (const FrontierCluster& c : clusters) ids.insert(c.id);
  EXPECT_EQ(ids.size(), 3u);
  EXPECT_NE(mgg::makeClusterId(0, Eigen::Vector3d(0.5, 0.0, 0.0), 1.0),
            mgg::makeClusterId(0, Eigen::Vector3d(-0.5, 0.0, 0.0), 1.0));
}

TEST(FrontierClusters, ExploredInGraphNeedsRoadmapAndNoFrontierNearby) {
  GraphManager graph;
  addVertex(graph, 0.0, 0.0, VertexType::kVisited, 0.0);
  addVertex(graph, 1.0, 0.0, VertexType::kUnvisited, 0.0);
  addVertex(graph, 10.0, 0.0, VertexType::kVisited, 0.0);
  addVertex(graph, 10.5, 0.0, VertexType::kFrontier, 900.0);
  // Roadmap nearby and no frontier: explored.
  EXPECT_TRUE(mgg::exploredInGraph(graph, Eigen::Vector3d(0.5, 0.0, 0.0), 2.0));
  // A frontier nearby: not explored.
  EXPECT_FALSE(mgg::exploredInGraph(graph, Eigen::Vector3d(10.0, 0.0, 0.0), 2.0));
  // Nothing nearby: this roadmap cannot tell.
  EXPECT_FALSE(mgg::exploredInGraph(graph, Eigen::Vector3d(30.0, 0.0, 0.0), 2.0));
  GraphManager empty;
  EXPECT_FALSE(mgg::exploredInGraph(empty, Eigen::Vector3d::Zero(), 2.0));
}

}  // namespace
```

Register it in `ros2/src/mgg_core/CMakeLists.txt` after the `test_tour_params` lines:

```cmake
  ament_add_gtest(test_frontier_clusters test/test_frontier_clusters.cpp)
  target_link_libraries(test_frontier_clusters ${PROJECT_NAME})
```

- [ ] **Step 2: Run it to verify it fails**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_frontier_clusters '*'`
Expected: FAIL at compile time with `mgg_core/frontier_clusters.h: No such file or directory`.

- [ ] **Step 3: Write the header**

Create `ros2/src/mgg_core/include/mgg_core/frontier_clusters.h`:

```cpp
// Frontier clusters of the global graph: what the tour orders and the fleet
// auctions (tour-exploration design §2.1).
//
// A cluster is a best-gain frontier vertex, its representative, and every
// other frontier within the merge radius of it. The design took the
// grouping from Vertex::cluster_id, but performShortestPathsClustering sets
// that on the local graph's leaves only: the global graph holds one frontier
// per principal path (addFrontiers) and the expansion's frontiers, and none
// carries it. The global frontiers are grouped here, and cluster_id is set
// to the representative's vertex id.
//
// A cluster's stable ID names a place: the owning robot and its
// representative's cell on a grid of tour.cluster_id_cell_m when first
// seen. ClusterIdRegistry keeps the name while the representative moves
// within the cluster, so fleet messages can refer to it across graph
// revisions.

#ifndef MGG_CORE_FRONTIER_CLUSTERS_H_
#define MGG_CORE_FRONTIER_CLUSTERS_H_

#include <cstdint>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/graph_manager.h"

namespace mgg {

/// Robot ID + 1 in bits 48 to 62, then the x, y and z cells, 16 bits each
/// offset by 2^15 (about 32 km either way at 1 m cells). Never zero.
using ClusterId = std::uint64_t;
inline constexpr ClusterId kNoCluster = 0;

ClusterId makeClusterId(int robot_id, const Eigen::Vector3d& position,
                        double cell_m);

struct FrontierCluster {
  ClusterId id = kNoCluster;
  /// The robot whose frontier the representative is.
  int owner_robot_id = 0;
  /// The best-gain member, a vertex of this robot's global graph.
  int representative_vertex_id = -1;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  /// The representative's gain.
  double gain = 0.0;
  /// The representative first.
  std::vector<int> member_vertex_ids;
};

/// The in-service kFrontier vertices of `graph`, grouped: the best-gain
/// vertex not yet grouped represents a cluster, and every ungrouped frontier
/// within `merge_radius_m` of it joins. Ties in gain go to the lower vertex
/// id. Clusters whose representative gain is below `min_cluster_gain` are
/// dropped. Returned best gain first, with raw IDs (makeClusterId; a second
/// representative in the same cell takes the next free ID).
std::vector<FrontierCluster> extractFrontierClusters(GraphManager& graph,
                                                     double merge_radius_m,
                                                     double min_cluster_gain,
                                                     double cluster_id_cell_m);

/// Keeps each cluster's ID across graph revisions: a cluster whose
/// representative lies within `match_radius_m` of one named last time takes
/// that name (nearest first, each name once); the others keep their raw
/// IDs, bumped past any name already in use. Names of clusters that vanished
/// are forgotten.
class ClusterIdRegistry {
 public:
  void stabilize(std::vector<FrontierCluster>& clusters,
                 double match_radius_m);

 private:
  struct Known {
    ClusterId id = kNoCluster;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
  };
  std::vector<Known> known_;
};

/// Whether this robot's roadmap shows `position` explored: an in-service
/// vertex lies within `radius_m` of it and no frontier does. Far from the
/// roadmap it cannot tell, and answers false.
bool exploredInGraph(GraphManager& graph, const Eigen::Vector3d& position,
                     double radius_m);

}  // namespace mgg

#endif  // MGG_CORE_FRONTIER_CLUSTERS_H_
```

- [ ] **Step 4: Write the implementation**

Create `ros2/src/mgg_core/src/frontier_clusters.cpp`:

```cpp
#include "mgg_core/frontier_clusters.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace mgg {
namespace {

std::uint64_t cellBits(double coordinate, double cell_m) {
  double cell = std::floor(coordinate / cell_m);
  if (!std::isfinite(cell)) cell = 0.0;
  cell = std::clamp(cell, -32768.0, 32767.0);
  return static_cast<std::uint64_t>(static_cast<std::int64_t>(cell) + 32768) &
         0xFFFFu;
}

}  // namespace

ClusterId makeClusterId(int robot_id, const Eigen::Vector3d& position,
                        double cell_m) {
  const std::uint64_t robot =
      (static_cast<std::uint64_t>(robot_id) & 0x7FFFu) + 1u;
  return (robot << 48) | (cellBits(position.x(), cell_m) << 32) |
         (cellBits(position.y(), cell_m) << 16) |
         cellBits(position.z(), cell_m);
}

std::vector<FrontierCluster> extractFrontierClusters(GraphManager& graph,
                                                     double merge_radius_m,
                                                     double min_cluster_gain,
                                                     double cluster_id_cell_m) {
  std::vector<Vertex*> frontiers;
  for (auto& entry : graph.vertices_map_) {
    Vertex* vertex = entry.second;
    if (vertex == nullptr || vertex->type != VertexType::kFrontier ||
        !graph.inService(*vertex)) {
      continue;
    }
    frontiers.push_back(vertex);
  }
  // Best gain first; ties by id, so the grouping does not follow hash order.
  std::sort(frontiers.begin(), frontiers.end(),
            [](const Vertex* a, const Vertex* b) {
              if (a->vol_gain.gain != b->vol_gain.gain) {
                return a->vol_gain.gain > b->vol_gain.gain;
              }
              return a->id < b->id;
            });

  const double radius_sq = merge_radius_m * merge_radius_m;
  std::vector<bool> grouped(frontiers.size(), false);
  std::vector<FrontierCluster> clusters;
  for (std::size_t i = 0; i < frontiers.size(); ++i) {
    if (grouped[i]) continue;
    const Vertex* representative = frontiers[i];
    FrontierCluster cluster;
    cluster.owner_robot_id = representative->robot_id;
    cluster.representative_vertex_id = representative->id;
    cluster.position = representative->state.head<3>();
    cluster.gain = representative->vol_gain.gain;
    for (std::size_t j = i; j < frontiers.size(); ++j) {
      if (grouped[j] ||
          (frontiers[j]->state.head<3>() - cluster.position).squaredNorm() >
              radius_sq) {
        continue;
      }
      grouped[j] = true;
      frontiers[j]->cluster_id = representative->id;
      cluster.member_vertex_ids.push_back(frontiers[j]->id);
    }
    if (!(cluster.gain >= min_cluster_gain)) continue;  // also drops NaN
    cluster.id = makeClusterId(cluster.owner_robot_id, cluster.position,
                               cluster_id_cell_m);
    clusters.push_back(std::move(cluster));
  }
  std::unordered_set<ClusterId> used;
  for (FrontierCluster& cluster : clusters) {
    while (!used.insert(cluster.id).second) ++cluster.id;
  }
  return clusters;
}

void ClusterIdRegistry::stabilize(std::vector<FrontierCluster>& clusters,
                                  double match_radius_m) {
  std::vector<bool> reused(known_.size(), false);
  std::vector<bool> renamed(clusters.size(), false);
  std::unordered_set<ClusterId> used;
  // Old names first, so a newcomer cannot take the name of a cluster that
  // stayed.
  for (std::size_t c = 0; c < clusters.size(); ++c) {
    int best = -1;
    double best_distance = match_radius_m;
    for (std::size_t k = 0; k < known_.size(); ++k) {
      if (reused[k]) continue;
      const double distance =
          (known_[k].position - clusters[c].position).norm();
      if (distance <= best_distance) {
        best_distance = distance;
        best = static_cast<int>(k);
      }
    }
    if (best < 0) continue;
    reused[best] = true;
    renamed[c] = true;
    clusters[c].id = known_[best].id;
    used.insert(clusters[c].id);
  }
  for (std::size_t c = 0; c < clusters.size(); ++c) {
    if (renamed[c]) continue;
    while (!used.insert(clusters[c].id).second) ++clusters[c].id;
  }
  known_.clear();
  for (const FrontierCluster& cluster : clusters) {
    known_.push_back({cluster.id, cluster.position});
  }
}

bool exploredInGraph(GraphManager& graph, const Eigen::Vector3d& position,
                     double radius_m) {
  const StateVec state(position.x(), position.y(), position.z(), 0.0);
  std::vector<Vertex*> nearby;
  if (!graph.getNearestVertices(&state, radius_m, &nearby)) return false;
  bool covered = false;
  for (const Vertex* vertex : nearby) {
    if (vertex == nullptr || !graph.inService(*vertex)) continue;
    if (vertex->type == VertexType::kFrontier) return false;
    covered = true;
  }
  return covered;
}

}  // namespace mgg
```

Add the source to the `add_library(${PROJECT_NAME} …)` list in `ros2/src/mgg_core/CMakeLists.txt`, on a new line after `  src/global_graph.cpp`:

```cmake
  src/frontier_clusters.cpp
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_frontier_clusters '*'`
Expected: `[  PASSED  ] 7 tests.`

- [ ] **Step 6: Run both gates**

Run: `/tmp/mgg-tour/tuf-run.sh off test` then `/tmp/mgg-tour/tuf-run.sh on test`
Expected: both end with `0 errors, 0 failures`.

- [ ] **Step 7: Commit**

```bash
git add ros2/src/mgg_core/include/mgg_core/frontier_clusters.h \
        ros2/src/mgg_core/src/frontier_clusters.cpp \
        ros2/src/mgg_core/test/test_frontier_clusters.cpp \
        ros2/src/mgg_core/CMakeLists.txt
git commit -m "Group global frontiers into clusters with stable IDs"
```

---
### Task 3: The open tour solver

**Files:**
- Create: `ros2/src/mgg_core/include/mgg_core/tour_solver.h`
- Create: `ros2/src/mgg_core/src/tour_solver.cpp`
- Create: `ros2/src/mgg_core/test/test_tour_solver.cpp`
- Modify: `ros2/src/mgg_core/CMakeLists.txt` (library source, test)

**Interfaces:**
- Consumes: nothing from earlier tasks (pure cost matrices).
- Produces:
  - `inline constexpr double mgg::kUnreachableCost` (= +infinity), `inline constexpr std::size_t mgg::kExactTourMaxClusters = 7`
  - `struct mgg::OpenTour { std::vector<int> order; double cost; };`
  - `double mgg::openTourCost(const std::vector<int>& order, const std::vector<double>& from_start, const std::vector<std::vector<double>>& between);`
  - `OpenTour mgg::improveOpenTour(std::vector<int> order, const std::vector<double>& from_start, const std::vector<std::vector<double>>& between, bool fixed_first = false);`
  - `OpenTour mgg::solveOpenTour(const std::vector<double>& from_start, const std::vector<std::vector<double>>& between, int first = -1);`
  - `struct mgg::Insertion { std::size_t position; double added_cost; };`
  - `Insertion mgg::cheapestInsertion(const std::vector<int>& order, int index, const std::vector<double>& from_start, const std::vector<std::vector<double>>& between, bool keep_first);`

Small tours are solved exactly (every order of up to seven clusters after a fixed first), so the spec's "optimal on small synthetic graphs (brute force)" holds by construction; larger tours use the spec's greedy nearest-neighbour start, improved with 2-opt and Or-opt until no move improves.

- [ ] **Step 1: Write the failing test**

Create `ros2/src/mgg_core/test/test_tour_solver.cpp`:

```cpp
// Tests for the open tour over frontier clusters (tour-exploration design
// §2.3): optimal on small scenes, within 5 % of the best known on 50
// clusters, and fast.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "mgg_core/tour_solver.h"

namespace {

using mgg::OpenTour;

struct Scene {
  std::vector<double> from_start;
  std::vector<std::vector<double>> between;
};

/// Clusters at `points`, the robot at the origin, straight-line costs.
Scene euclidean(const std::vector<Eigen::Vector2d>& points) {
  Scene scene;
  for (const Eigen::Vector2d& p : points) {
    scene.from_start.push_back(p.norm());
    scene.between.emplace_back();
    for (const Eigen::Vector2d& q : points) {
      scene.between.back().push_back((p - q).norm());
    }
  }
  return scene;
}

Scene randomScene(std::size_t n, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> coordinate(-40.0, 40.0);
  std::vector<Eigen::Vector2d> points;
  for (std::size_t i = 0; i < n; ++i) {
    points.emplace_back(coordinate(rng), coordinate(rng));
  }
  return euclidean(points);
}

double bruteForce(const Scene& scene) {
  std::vector<int> order(scene.from_start.size());
  std::iota(order.begin(), order.end(), 0);
  double best = mgg::kUnreachableCost;
  do {
    best = std::min(best,
                    mgg::openTourCost(order, scene.from_start, scene.between));
  } while (std::next_permutation(order.begin(), order.end()));
  return best;
}

TEST(OpenTour, EmptyAndSingleClusterTours) {
  const OpenTour none = mgg::solveOpenTour({}, {});
  EXPECT_TRUE(none.order.empty());
  EXPECT_DOUBLE_EQ(none.cost, 0.0);
  const Scene one = euclidean({{3.0, 4.0}});
  const OpenTour single = mgg::solveOpenTour(one.from_start, one.between);
  EXPECT_EQ(single.order, std::vector<int>{0});
  EXPECT_DOUBLE_EQ(single.cost, 5.0);
}

TEST(OpenTour, SmallToursAreOptimal) {
  for (std::size_t n = 1; n <= 7; ++n) {
    for (unsigned seed = 1; seed <= 20; ++seed) {
      const Scene scene = randomScene(n, seed);
      const OpenTour tour = mgg::solveOpenTour(scene.from_start, scene.between);
      ASSERT_EQ(tour.order.size(), n);
      EXPECT_NEAR(tour.cost, bruteForce(scene), 1e-9)
          << n << " clusters, seed " << seed;
    }
  }
}

TEST(OpenTour, LocalImprovementUntanglesACrossedTour) {
  // Four clusters on a line ahead of the robot, visited out of order.
  const Scene line = euclidean({{1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0}});
  const OpenTour free = mgg::improveOpenTour({2, 0, 3, 1}, line.from_start,
                                             line.between);
  EXPECT_EQ(free.order, (std::vector<int>{0, 1, 2, 3}));
  EXPECT_DOUBLE_EQ(free.cost, 4.0);
  // With the first cluster fixed, the rest are ordered back from it.
  const OpenTour fixed = mgg::improveOpenTour({3, 0, 1, 2}, line.from_start,
                                              line.between, true);
  EXPECT_EQ(fixed.order, (std::vector<int>{3, 2, 1, 0}));
  EXPECT_DOUBLE_EQ(fixed.cost, 7.0);
}

TEST(OpenTour, FiftyClustersWithinFivePercentOfTheBestKnown) {
  for (unsigned seed = 1; seed <= 5; ++seed) {
    const Scene scene = randomScene(50, seed);
    const OpenTour tour = mgg::solveOpenTour(scene.from_start, scene.between);
    ASSERT_EQ(tour.order.size(), 50u);
    // Best known: the solver's own tour and local improvement from thirty
    // random orders.
    double best = tour.cost;
    std::mt19937 rng(seed + 100);
    std::vector<int> order(50);
    std::iota(order.begin(), order.end(), 0);
    for (int restart = 0; restart < 30; ++restart) {
      std::shuffle(order.begin(), order.end(), rng);
      best = std::min(best, mgg::improveOpenTour(order, scene.from_start,
                                                 scene.between)
                                .cost);
    }
    EXPECT_LE(tour.cost, 1.05 * best) << "seed " << seed;
  }
}

TEST(OpenTour, FiftyClustersSolveWithinTenMilliseconds) {
  const Scene scene = randomScene(50, 7);
  constexpr int kRuns = 20;
  const auto start = std::chrono::steady_clock::now();
  for (int run = 0; run < kRuns; ++run) {
    const OpenTour tour = mgg::solveOpenTour(scene.from_start, scene.between);
    ASSERT_EQ(tour.order.size(), 50u);
  }
  const double ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - start)
                        .count() /
                    kRuns;
  EXPECT_LT(ms, 10.0);
}

TEST(OpenTour, AForcedFirstClusterStartsTheTourOrRefusesIt) {
  Scene scene = euclidean({{1.0, 0.0}, {2.0, 0.0}, {-5.0, 0.0}});
  const OpenTour forced =
      mgg::solveOpenTour(scene.from_start, scene.between, 2);
  EXPECT_EQ(forced.order, (std::vector<int>{2, 0, 1}));
  EXPECT_DOUBLE_EQ(forced.cost, 5.0 + 6.0 + 1.0);
  scene.from_start[2] = mgg::kUnreachableCost;
  const OpenTour refused =
      mgg::solveOpenTour(scene.from_start, scene.between, 2);
  EXPECT_TRUE(refused.order.empty());
  EXPECT_EQ(refused.cost, mgg::kUnreachableCost);
}

TEST(OpenTour, UnreachableClustersAreLeftOut) {
  Scene scene = euclidean({{1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}});
  scene.from_start[1] = mgg::kUnreachableCost;
  const OpenTour tour = mgg::solveOpenTour(scene.from_start, scene.between);
  EXPECT_EQ(tour.order, (std::vector<int>{0, 2}));
}

TEST(OpenTour, DisconnectedLegsStillGiveAWholeTour) {
  // Ten clusters in two groups, each reachable from the robot but not from
  // the other group: the heuristic still orders all of them.
  Scene scene = randomScene(10, 3);
  for (int i = 0; i < 10; ++i) {
    for (int j = 0; j < 10; ++j) {
      if ((i < 5) != (j < 5)) scene.between[i][j] = mgg::kUnreachableCost;
    }
  }
  const OpenTour tour = mgg::solveOpenTour(scene.from_start, scene.between);
  EXPECT_EQ(tour.order.size(), 10u);
  EXPECT_FALSE(std::isfinite(tour.cost));
}

TEST(OpenTour, CheapestInsertionFindsTheGapAndKeepsAKeptFirst) {
  const Scene line = euclidean({{1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}});
  const mgg::Insertion gap =
      mgg::cheapestInsertion({0, 2}, 1, line.from_start, line.between, false);
  EXPECT_EQ(gap.position, 1u);
  EXPECT_NEAR(gap.added_cost, 0.0, 1e-12);
  const mgg::Insertion front =
      mgg::cheapestInsertion({1, 2}, 0, line.from_start, line.between, false);
  EXPECT_EQ(front.position, 0u);
  EXPECT_NEAR(front.added_cost, 0.0, 1e-12);
  const mgg::Insertion kept =
      mgg::cheapestInsertion({1, 2}, 0, line.from_start, line.between, true);
  EXPECT_EQ(kept.position, 1u);
  EXPECT_NEAR(kept.added_cost, 2.0, 1e-12);
  // Nothing to insert into: the start leg alone.
  const mgg::Insertion empty =
      mgg::cheapestInsertion({}, 2, line.from_start, line.between, true);
  EXPECT_EQ(empty.position, 0u);
  EXPECT_DOUBLE_EQ(empty.added_cost, 3.0);
}

}  // namespace
```

Register it in `ros2/src/mgg_core/CMakeLists.txt` after the `test_frontier_clusters` lines:

```cmake
  ament_add_gtest(test_tour_solver test/test_tour_solver.cpp)
  target_link_libraries(test_tour_solver ${PROJECT_NAME})
```

- [ ] **Step 2: Run it to verify it fails**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_tour_solver '*'`
Expected: FAIL at compile time with `mgg_core/tour_solver.h: No such file or directory`.

- [ ] **Step 3: Write the header**

Create `ros2/src/mgg_core/include/mgg_core/tour_solver.h`:

```cpp
// The open tour over frontier clusters (tour-exploration design §2.3): the
// robot visits every reachable cluster once, starting where it stands, and
// does not return.
//
// Pure: it works on a cost matrix, so the tour (§2) and the fleet auction
// (§3.4) share it and it is tested without a graph. Up to
// kExactTourMaxClusters clusters every order is tried; beyond that a greedy
// nearest-neighbour start is improved with 2-opt and Or-opt until no move
// improves it.

#ifndef MGG_CORE_TOUR_SOLVER_H_
#define MGG_CORE_TOUR_SOLVER_H_

#include <cstddef>
#include <limits>
#include <vector>

namespace mgg {

/// A leg no route covers.
inline constexpr double kUnreachableCost =
    std::numeric_limits<double>::infinity();
/// Up to this many clusters after a fixed first one, solveOpenTour tries
/// every order (7! = 5040).
inline constexpr std::size_t kExactTourMaxClusters = 7;

struct OpenTour {
  /// Cluster indices in visiting order.
  std::vector<int> order;
  /// from_start of the first plus every leg between; kUnreachableCost when
  /// a leg is, or when a required first cluster cannot be reached.
  double cost = 0.0;
};

/// Cost of visiting `order`: from_start[order[0]] plus `between` along it,
/// no return leg. Zero for an empty order.
double openTourCost(const std::vector<int>& order,
                    const std::vector<double>& from_start,
                    const std::vector<std::vector<double>>& between);

/// 2-opt and Or-opt (segments of one to three clusters, either way round)
/// from `order` until no move lowers the cost. `fixed_first` keeps order[0]
/// first. `between` must be symmetric.
OpenTour improveOpenTour(std::vector<int> order,
                         const std::vector<double>& from_start,
                         const std::vector<std::vector<double>>& between,
                         bool fixed_first = false);

/// The open tour over every index whose from_start is finite; the others
/// are left out until they can be reached. With `first` >= 0 the tour
/// starts there; when that index cannot be reached the tour is empty with
/// cost kUnreachableCost. `between` must be symmetric.
OpenTour solveOpenTour(const std::vector<double>& from_start,
                       const std::vector<std::vector<double>>& between,
                       int first = -1);

/// Where `index` joins a tour order for the least added cost.
struct Insertion {
  /// `index` goes before order[position]; order.size() appends it.
  std::size_t position = 0;
  double added_cost = kUnreachableCost;
};

/// The cheapest insertion of `index` into `order`, never before order[0]
/// when `keep_first`. added_cost is kUnreachableCost when no position has
/// finite legs.
Insertion cheapestInsertion(const std::vector<int>& order, int index,
                            const std::vector<double>& from_start,
                            const std::vector<std::vector<double>>& between,
                            bool keep_first);

}  // namespace mgg

#endif  // MGG_CORE_TOUR_SOLVER_H_
```

- [ ] **Step 4: Write the implementation**

Create `ros2/src/mgg_core/src/tour_solver.cpp`:

```cpp
#include "mgg_core/tour_solver.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mgg {
namespace {

/// Stands in for an unreachable leg inside the search, so every cost there
/// stays finite and comparable; openTourCost reports the true sum.
constexpr double kDisconnectedLegCost = 1e9;
/// A move counts only when it lowers the cost by more than this.
constexpr double kImproveEps = 1e-9;
/// Every move lowers the cost, so the search ends long before this; it only
/// bounds a pathological input.
constexpr int kMaxImprovingMoves = 100000;
/// Pseudo-indices: where the robot stands, and the open end of the tour.
constexpr int kStart = -1;
constexpr int kEnd = -2;

class LegCost {
 public:
  LegCost(const std::vector<double>& from_start,
          const std::vector<std::vector<double>>& between)
      : from_start_(from_start), between_(between) {}

  double operator()(int from, int to) const {
    if (to == kEnd) return 0.0;
    const double cost = from == kStart ? from_start_[to] : between_[from][to];
    return std::isfinite(cost) ? cost : kDisconnectedLegCost;
  }

 private:
  const std::vector<double>& from_start_;
  const std::vector<std::vector<double>>& between_;
};

/// First improving 2-opt move: reverse order[i..j].
bool twoOptMove(std::vector<int>& order, const LegCost& leg, std::size_t lo) {
  const std::size_t n = order.size();
  for (std::size_t i = lo; i + 1 < n; ++i) {
    const int before = i == 0 ? kStart : order[i - 1];
    for (std::size_t j = i + 1; j < n; ++j) {
      const int after = j + 1 < n ? order[j + 1] : kEnd;
      const double delta = leg(before, order[j]) + leg(order[i], after) -
                           leg(before, order[i]) - leg(order[j], after);
      if (delta < -kImproveEps) {
        std::reverse(order.begin() + i, order.begin() + j + 1);
        return true;
      }
    }
  }
  return false;
}

/// First improving Or-opt move: a segment of one to three clusters moved
/// elsewhere, either way round.
bool orOptMove(std::vector<int>& order, const LegCost& leg, std::size_t lo) {
  const std::size_t n = order.size();
  for (std::size_t length = 1; length <= 3; ++length) {
    for (std::size_t i = lo; i + length <= n; ++i) {
      const std::size_t k = i + length - 1;
      const int first = order[i];
      const int last = order[k];
      const int a = i == 0 ? kStart : order[i - 1];
      const int b = k + 1 < n ? order[k + 1] : kEnd;
      const double removed = leg(a, first) + leg(last, b) - leg(a, b);
      std::vector<int> rest;
      rest.reserve(n - length);
      rest.insert(rest.end(), order.begin(), order.begin() + i);
      rest.insert(rest.end(), order.begin() + k + 1, order.end());
      for (std::size_t p = lo; p <= rest.size(); ++p) {
        if (p == i) continue;  // where the segment came from
        const int u = p == 0 ? kStart : rest[p - 1];
        const int v = p < rest.size() ? rest[p] : kEnd;
        const double base = leg(u, v);
        const double forward = leg(u, first) + leg(last, v) - base;
        const double backward = leg(u, last) + leg(first, v) - base;
        const bool reverse = backward < forward;
        if ((reverse ? backward : forward) - removed < -kImproveEps) {
          std::vector<int> segment(order.begin() + i, order.begin() + k + 1);
          if (reverse) std::reverse(segment.begin(), segment.end());
          rest.insert(rest.begin() + p, segment.begin(), segment.end());
          order.swap(rest);
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace

double openTourCost(const std::vector<int>& order,
                    const std::vector<double>& from_start,
                    const std::vector<std::vector<double>>& between) {
  if (order.empty()) return 0.0;
  double cost = from_start[order.front()];
  for (std::size_t i = 1; i < order.size(); ++i) {
    cost += between[order[i - 1]][order[i]];
  }
  return cost;
}

OpenTour improveOpenTour(std::vector<int> order,
                         const std::vector<double>& from_start,
                         const std::vector<std::vector<double>>& between,
                         bool fixed_first) {
  const LegCost leg(from_start, between);
  const std::size_t lo = fixed_first ? 1 : 0;
  for (int move = 0; move < kMaxImprovingMoves; ++move) {
    if (twoOptMove(order, leg, lo)) continue;
    if (orOptMove(order, leg, lo)) continue;
    break;
  }
  OpenTour tour;
  tour.cost = openTourCost(order, from_start, between);
  tour.order = std::move(order);
  return tour;
}

OpenTour solveOpenTour(const std::vector<double>& from_start,
                       const std::vector<std::vector<double>>& between,
                       int first) {
  OpenTour tour;
  const int n = static_cast<int>(from_start.size());
  if (first >= 0 && (first >= n || !std::isfinite(from_start[first]))) {
    tour.cost = kUnreachableCost;
    return tour;
  }
  std::vector<int> rest;
  for (int i = 0; i < n; ++i) {
    if (i != first && std::isfinite(from_start[i])) rest.push_back(i);
  }
  if (rest.empty() && first < 0) return tour;
  const LegCost leg(from_start, between);
  std::vector<int> order;
  if (first >= 0) order.push_back(first);

  if (rest.size() <= kExactTourMaxClusters) {
    // Every order of the rest; `rest` is ascending, as next_permutation
    // needs to visit them all.
    std::vector<int> best = rest;
    double best_cost = kUnreachableCost;
    do {
      int previous = first >= 0 ? first : kStart;
      double cost = first >= 0 ? leg(kStart, first) : 0.0;
      for (const int index : rest) {
        cost += leg(previous, index);
        previous = index;
      }
      if (cost < best_cost - kImproveEps) {
        best_cost = cost;
        best = rest;
      }
    } while (std::next_permutation(rest.begin(), rest.end()));
    order.insert(order.end(), best.begin(), best.end());
    tour.cost = openTourCost(order, from_start, between);
    tour.order = std::move(order);
    return tour;
  }

  // Greedy nearest neighbour from the start (lower index on a tie), then
  // local improvement.
  std::vector<bool> placed(n, false);
  int current = first >= 0 ? first : kStart;
  for (std::size_t step = 0; step < rest.size(); ++step) {
    int next = -1;
    double next_cost = kUnreachableCost;
    for (const int index : rest) {
      if (placed[index]) continue;
      const double cost = leg(current, index);
      if (next < 0 || cost < next_cost) {
        next_cost = cost;
        next = index;
      }
    }
    placed[next] = true;
    order.push_back(next);
    current = next;
  }
  return improveOpenTour(std::move(order), from_start, between, first >= 0);
}

Insertion cheapestInsertion(const std::vector<int>& order, int index,
                            const std::vector<double>& from_start,
                            const std::vector<std::vector<double>>& between,
                            bool keep_first) {
  const auto cost = [&](int from, int to) {
    return from == kStart ? from_start[to] : between[from][to];
  };
  Insertion best;
  const std::size_t lo = keep_first && !order.empty() ? 1 : 0;
  for (std::size_t p = lo; p <= order.size(); ++p) {
    const int u = p == 0 ? kStart : order[p - 1];
    const double in = cost(u, index);
    const double out = p < order.size() ? between[index][order[p]] : 0.0;
    const double base = p < order.size() ? cost(u, order[p]) : 0.0;
    if (!std::isfinite(in) || !std::isfinite(out) || !std::isfinite(base)) {
      continue;
    }
    const double added = in + out - base;
    if (added < best.added_cost) {
      best.added_cost = added;
      best.position = p;
    }
  }
  return best;
}

}  // namespace mgg
```

Add the source to `add_library` in `ros2/src/mgg_core/CMakeLists.txt`, after `  src/frontier_clusters.cpp`:

```cmake
  src/tour_solver.cpp
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_tour_solver '*'`
Expected: `[  PASSED  ] 9 tests.`

- [ ] **Step 6: Run both gates**

Run: `/tmp/mgg-tour/tuf-run.sh off test` then `/tmp/mgg-tour/tuf-run.sh on test`
Expected: both end with `0 errors, 0 failures`.

- [ ] **Step 7: Commit**

```bash
git add ros2/src/mgg_core/include/mgg_core/tour_solver.h \
        ros2/src/mgg_core/src/tour_solver.cpp \
        ros2/src/mgg_core/test/test_tour_solver.cpp \
        ros2/src/mgg_core/CMakeLists.txt
git commit -m "Solve the open tour over frontier clusters"
```

---

### Task 4: Tour costs on the global graph

**Files:**
- Create: `ros2/src/mgg_core/include/mgg_core/tour_costs.h`
- Create: `ros2/src/mgg_core/src/tour_costs.cpp`
- Create: `ros2/src/mgg_core/test/test_tour_costs.cpp`
- Modify: `ros2/src/mgg_core/CMakeLists.txt` (library source, test)

**Interfaces:**
- Consumes: `mgg::FrontierCluster` (Task 2), `mgg::kUnreachableCost`, `mgg::solveOpenTour` (Task 3, in the test), `mgg::GraphManager::findShortestPaths(int, ShortestPathsReport&)`, `GraphManager::getShortestPath(int, const ShortestPathsReport&, bool, std::vector<int>&)`.
- Produces:
  - `inline constexpr double mgg::kFirstLegLookaheadM = 2.0;`
  - `class mgg::GraphDistanceCache { public: const ShortestPathsReport* from(GraphManager& graph, std::uint64_t revision, int source_id); std::size_t solves() const; };`
  - `double mgg::reachedDistance(const ShortestPathsReport& report, int target_id);`
  - `double mgg::firstLegHeadingChange(GraphManager& graph, const ShortestPathsReport& from_robot, int target_id, double robot_yaw);`
  - `struct mgg::TourCostMatrix { std::vector<double> from_robot; std::vector<std::vector<double>> between; };`
  - `TourCostMatrix mgg::computeTourCosts(GraphManager& graph, std::uint64_t revision, GraphDistanceCache& cache, int source_vertex_id, double robot_yaw, const std::vector<FrontierCluster>& clusters, double heading_weight);`

- [ ] **Step 1: Write the failing test**

Create `ros2/src/mgg_core/test/test_tour_costs.cpp`:

```cpp
// Tests for the tour's costs on the global graph (tour-exploration design
// §2.2): shortest-path lengths cached per graph revision, unreachable
// clusters left out, and the first leg's heading penalty.

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/tour_costs.h"

namespace {

using mgg::FrontierCluster;
using mgg::GraphDistanceCache;
using mgg::GraphManager;
using mgg::StateVec;
using mgg::TourCostMatrix;
using mgg::Vertex;
using mgg::VertexType;

Vertex* add(GraphManager& graph, double x, double y,
            Vertex* linked_to = nullptr,
            VertexType type = VertexType::kUnvisited) {
  auto* v = new Vertex(graph.generateVertexID(), StateVec(x, y, 0.0, 0.0));
  v->type = type;
  v->robot_id = 1;
  graph.addVertex(v);
  if (linked_to != nullptr) {
    graph.addEdge(v, linked_to,
                  (v->state.head<3>() - linked_to->state.head<3>()).norm());
  }
  return v;
}

FrontierCluster clusterAt(const Vertex* v) {
  FrontierCluster c;
  c.id = 100 + v->id;
  c.owner_robot_id = 1;
  c.representative_vertex_id = v->id;
  c.position = v->state.head<3>();
  c.member_vertex_ids = {v->id};
  return c;
}

TEST(TourCosts, CostsAreShortestPathLengthsBetweenRepresentatives) {
  GraphManager graph;
  Vertex* root = add(graph, 0.0, 0.0);
  Vertex* corner = add(graph, 5.0, 0.0, root, VertexType::kFrontier);
  Vertex* end = add(graph, 5.0, 5.0, corner, VertexType::kFrontier);
  Vertex* island = add(graph, 20.0, 20.0, nullptr, VertexType::kFrontier);
  GraphDistanceCache cache;
  const TourCostMatrix costs = mgg::computeTourCosts(
      graph, 1, cache, root->id, 0.0,
      {clusterAt(corner), clusterAt(end), clusterAt(island)}, 0.0);
  ASSERT_EQ(costs.from_robot.size(), 3u);
  EXPECT_DOUBLE_EQ(costs.from_robot[0], 5.0);
  EXPECT_DOUBLE_EQ(costs.from_robot[1], 10.0);
  EXPECT_EQ(costs.from_robot[2], mgg::kUnreachableCost);
  EXPECT_DOUBLE_EQ(costs.between[0][1], 5.0);
  EXPECT_DOUBLE_EQ(costs.between[1][0], 5.0);
  EXPECT_EQ(costs.between[0][2], mgg::kUnreachableCost);
  EXPECT_DOUBLE_EQ(costs.between[2][2], 0.0);
  // The island is left out of the tour until it connects.
  const mgg::OpenTour tour =
      mgg::solveOpenTour(costs.from_robot, costs.between);
  EXPECT_EQ(tour.order, (std::vector<int>{0, 1}));
}

TEST(TourCosts, DistancesAreSolvedOncePerGraphRevision) {
  GraphManager graph;
  Vertex* root = add(graph, 0.0, 0.0);
  Vertex* a = add(graph, 5.0, 0.0, root, VertexType::kFrontier);
  Vertex* b = add(graph, 5.0, 5.0, a, VertexType::kFrontier);
  GraphDistanceCache cache;
  const std::vector<FrontierCluster> clusters{clusterAt(a), clusterAt(b)};
  mgg::computeTourCosts(graph, 1, cache, root->id, 0.0, clusters, 0.0);
  EXPECT_EQ(cache.solves(), 3u);  // the robot and each representative
  mgg::computeTourCosts(graph, 1, cache, root->id, 0.0, clusters, 0.0);
  EXPECT_EQ(cache.solves(), 3u);
  mgg::computeTourCosts(graph, 2, cache, root->id, 0.0, clusters, 0.0);
  EXPECT_EQ(cache.solves(), 6u);
}

TEST(TourCosts, AnUnlinkedSourceLeavesEveryClusterUnreachable) {
  // Review Focus 4: the robot's pose joined no vertex, or the graph is a
  // lone root. No cluster is reachable and the tour is empty.
  GraphManager graph;
  Vertex* root = add(graph, 0.0, 0.0);
  Vertex* a = add(graph, 5.0, 0.0, root, VertexType::kFrontier);
  GraphDistanceCache cache;
  const TourCostMatrix unlinked =
      mgg::computeTourCosts(graph, 1, cache, 999, 0.0, {clusterAt(a)}, 2.0);
  EXPECT_EQ(unlinked.from_robot[0], mgg::kUnreachableCost);
  EXPECT_TRUE(mgg::solveOpenTour(unlinked.from_robot, unlinked.between)
                  .order.empty());

  GraphManager lone;
  Vertex* only = add(lone, 0.0, 0.0, nullptr, VertexType::kFrontier);
  GraphDistanceCache lone_cache;
  const TourCostMatrix alone = mgg::computeTourCosts(
      lone, 1, lone_cache, only->id, 0.0, {clusterAt(only)}, 2.0);
  EXPECT_EQ(alone.from_robot[0], mgg::kUnreachableCost);
  EXPECT_TRUE(mgg::computeTourCosts(lone, 1, lone_cache, only->id, 0.0, {},
                                    2.0)
                  .from_robot.empty());
}

TEST(TourCosts, TheHeadingPenaltyAvoidsAUTurnStart) {
  // A corridor along x: the robot at the origin facing +x, a frontier 4 m
  // behind it and one 5 m ahead.
  GraphManager graph;
  Vertex* root = add(graph, 0.0, 0.0);
  Vertex* behind = root;
  for (int x = -1; x >= -4; --x) behind = add(graph, x, 0.0, behind);
  Vertex* ahead = root;
  for (int x = 1; x <= 5; ++x) ahead = add(graph, x, 0.0, ahead);
  behind->type = VertexType::kFrontier;
  ahead->type = VertexType::kFrontier;
  const std::vector<FrontierCluster> clusters{clusterAt(behind),
                                              clusterAt(ahead)};
  GraphDistanceCache cache;

  // By distance alone, behind first: 4 + 9 < 5 + 9.
  const TourCostMatrix plain =
      mgg::computeTourCosts(graph, 1, cache, root->id, 0.0, clusters, 0.0);
  EXPECT_EQ(mgg::solveOpenTour(plain.from_robot, plain.between).order.front(),
            0);
  // A U-turn costs heading_weight * pi.
  const TourCostMatrix penalised =
      mgg::computeTourCosts(graph, 1, cache, root->id, 0.0, clusters, 2.0);
  EXPECT_NEAR(penalised.from_robot[0], 4.0 + 2.0 * M_PI, 1e-9);
  EXPECT_NEAR(penalised.from_robot[1], 5.0, 1e-9);
  EXPECT_EQ(mgg::solveOpenTour(penalised.from_robot, penalised.between)
                .order.front(),
            1);
  // The penalty applies to the first leg only.
  EXPECT_DOUBLE_EQ(penalised.between[0][1], 9.0);
}

TEST(TourCosts, TheFirstLegHeadingLooksPastTheFirstVertex) {
  // The route leaves along +x for half a metre, then turns to +y: the first
  // leg heads +y, a quarter turn from a robot facing +x.
  GraphManager graph;
  Vertex* root = add(graph, 0.0, 0.0);
  Vertex* step = add(graph, 0.5, 0.0, root);
  Vertex* goal = add(graph, 0.5, 3.0, step);
  mgg::ShortestPathsReport report;
  ASSERT_TRUE(graph.findShortestPaths(root->id, report));
  EXPECT_NEAR(mgg::firstLegHeadingChange(graph, report, goal->id, 0.0),
              std::atan2(3.0, 0.5), 1e-9);
  EXPECT_DOUBLE_EQ(mgg::firstLegHeadingChange(graph, report, root->id, 0.0),
                   0.0);
}

}  // namespace
```

Register it in `ros2/src/mgg_core/CMakeLists.txt` after the `test_tour_solver` lines:

```cmake
  ament_add_gtest(test_tour_costs test/test_tour_costs.cpp)
  target_link_libraries(test_tour_costs ${PROJECT_NAME})
```

- [ ] **Step 2: Run it to verify it fails**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_tour_costs '*'`
Expected: FAIL at compile time with `mgg_core/tour_costs.h: No such file or directory`.

- [ ] **Step 3: Write the header**

Create `ros2/src/mgg_core/include/mgg_core/tour_costs.h`:

```cpp
// The tour's costs on the global graph (tour-exploration design §2.2):
// robot-to-cluster and cluster-to-cluster shortest-path lengths between
// representatives, one Dijkstra per source cached per graph revision, and a
// heading-change penalty on the first leg so the tour does not start with a
// U-turn when a cluster lies ahead.

#ifndef MGG_CORE_TOUR_COSTS_H_
#define MGG_CORE_TOUR_COSTS_H_

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "mgg_core/frontier_clusters.h"
#include "mgg_core/graph.h"
#include "mgg_core/graph_manager.h"
#include "mgg_core/tour_solver.h"

namespace mgg {

/// The first leg's heading is the direction from the robot to the first
/// vertex of its route at least this far away (or the route's end).
inline constexpr double kFirstLegLookaheadM = 2.0;

/// Dijkstra reports over one graph, one per source vertex, dropped when the
/// graph revision changes.
class GraphDistanceCache {
 public:
  /// The report from `source_id`, solved on first use in this `revision`;
  /// null when the source is not in the graph or Dijkstra cannot run (a
  /// graph of fewer than two vertices).
  const ShortestPathsReport* from(GraphManager& graph, std::uint64_t revision,
                                  int source_id);
  /// Dijkstra runs so far.
  std::size_t solves() const { return solves_; }

 private:
  bool valid_ = false;
  std::uint64_t revision_ = 0;
  std::unordered_map<int, ShortestPathsReport> reports_;
  std::unordered_set<int> failed_;
  std::size_t solves_ = 0;
};

/// The graph distance `report` found to `target_id`, or kUnreachableCost
/// when Dijkstra did not reach it.
double reachedDistance(const ShortestPathsReport& report, int target_id);

/// Absolute heading change, radians in [0, pi], from `robot_yaw` to the
/// first leg of the shortest route to `target_id` (kFirstLegLookaheadM).
/// Zero when the target is the source or unreachable.
double firstLegHeadingChange(GraphManager& graph,
                             const ShortestPathsReport& from_robot,
                             int target_id, double robot_yaw);

struct TourCostMatrix {
  /// Per cluster: graph distance from the robot's vertex plus
  /// heading_weight times the first leg's heading change.
  std::vector<double> from_robot;
  /// Graph distances between representatives; symmetric, zero diagonal.
  std::vector<std::vector<double>> between;
};

/// Costs of `clusters` from `source_vertex_id`, the vertex the robot joins
/// the graph at. Unreachable legs are kUnreachableCost, which leaves a
/// cluster out of the tour until it connects.
TourCostMatrix computeTourCosts(GraphManager& graph, std::uint64_t revision,
                                GraphDistanceCache& cache,
                                int source_vertex_id, double robot_yaw,
                                const std::vector<FrontierCluster>& clusters,
                                double heading_weight);

}  // namespace mgg

#endif  // MGG_CORE_TOUR_COSTS_H_
```

- [ ] **Step 4: Write the implementation**

Create `ros2/src/mgg_core/src/tour_costs.cpp`:

```cpp
#include "mgg_core/tour_costs.h"

#include <cmath>
#include <limits>
#include <utility>

namespace mgg {

const ShortestPathsReport* GraphDistanceCache::from(GraphManager& graph,
                                                    std::uint64_t revision,
                                                    int source_id) {
  if (!valid_ || revision != revision_) {
    reports_.clear();
    failed_.clear();
    revision_ = revision;
    valid_ = true;
  }
  const auto found = reports_.find(source_id);
  if (found != reports_.end()) return &found->second;
  if (failed_.count(source_id) > 0) return nullptr;
  // Graph::findDijkstraShortestPaths prints to stdout for an unknown
  // source; ask the vertex map first.
  if (graph.vertices_map_.find(source_id) == graph.vertices_map_.end()) {
    failed_.insert(source_id);
    return nullptr;
  }
  ShortestPathsReport report;
  ++solves_;
  if (!graph.findShortestPaths(source_id, report) || !report.status) {
    failed_.insert(source_id);
    return nullptr;
  }
  return &reports_.emplace(source_id, std::move(report)).first->second;
}

double reachedDistance(const ShortestPathsReport& report, int target_id) {
  if (!report.status) return kUnreachableCost;
  if (target_id == report.source_id) return 0.0;
  const auto parent = report.parent_id_map.find(target_id);
  const auto distance = report.distance_map.find(target_id);
  // Dijkstra leaves an unreached vertex as its own parent at the largest
  // double (global_graph.cpp searchGlobalFrontier reads it the same way).
  if (parent == report.parent_id_map.end() ||
      distance == report.distance_map.end() || parent->second == target_id ||
      !std::isfinite(distance->second) ||
      distance->second >= std::numeric_limits<double>::max() / 2.0) {
    return kUnreachableCost;
  }
  return distance->second;
}

double firstLegHeadingChange(GraphManager& graph,
                             const ShortestPathsReport& from_robot,
                             int target_id, double robot_yaw) {
  if (target_id == from_robot.source_id ||
      !std::isfinite(reachedDistance(from_robot, target_id))) {
    return 0.0;
  }
  std::vector<int> path;
  graph.getShortestPath(target_id, from_robot,
                        /*source_to_target_order=*/true, path);
  if (path.size() < 2) return 0.0;
  const auto source = graph.vertices_map_.find(path.front());
  if (source == graph.vertices_map_.end() || source->second == nullptr) {
    return 0.0;
  }
  const Eigen::Vector2d origin = source->second->state.head<2>();
  Eigen::Vector2d toward = Eigen::Vector2d::Zero();
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto vertex = graph.vertices_map_.find(path[i]);
    if (vertex == graph.vertices_map_.end() || vertex->second == nullptr) {
      continue;
    }
    toward = vertex->second->state.head<2>() - origin;
    if (toward.norm() >= kFirstLegLookaheadM) break;
  }
  if (toward.norm() < 1e-6) return 0.0;
  const double bearing = std::atan2(toward.y(), toward.x());
  return std::abs(std::remainder(bearing - robot_yaw, 2.0 * M_PI));
}

TourCostMatrix computeTourCosts(GraphManager& graph, std::uint64_t revision,
                                GraphDistanceCache& cache,
                                int source_vertex_id, double robot_yaw,
                                const std::vector<FrontierCluster>& clusters,
                                double heading_weight) {
  const std::size_t n = clusters.size();
  TourCostMatrix costs;
  costs.from_robot.assign(n, kUnreachableCost);
  costs.between.assign(n, std::vector<double>(n, kUnreachableCost));
  for (std::size_t i = 0; i < n; ++i) costs.between[i][i] = 0.0;

  const ShortestPathsReport* robot =
      cache.from(graph, revision, source_vertex_id);
  if (robot != nullptr) {
    for (std::size_t i = 0; i < n; ++i) {
      const int target = clusters[i].representative_vertex_id;
      const double distance = reachedDistance(*robot, target);
      if (!std::isfinite(distance)) continue;
      costs.from_robot[i] =
          distance + heading_weight * firstLegHeadingChange(graph, *robot,
                                                            target, robot_yaw);
    }
  }
  for (std::size_t i = 0; i < n; ++i) {
    const ShortestPathsReport* from_cluster =
        cache.from(graph, revision, clusters[i].representative_vertex_id);
    if (from_cluster == nullptr) continue;
    for (std::size_t j = i + 1; j < n; ++j) {
      const double distance = reachedDistance(
          *from_cluster, clusters[j].representative_vertex_id);
      costs.between[i][j] = distance;
      costs.between[j][i] = distance;
    }
  }
  return costs;
}

}  // namespace mgg
```

Add the source to `add_library` in `ros2/src/mgg_core/CMakeLists.txt`, after `  src/tour_solver.cpp`:

```cmake
  src/tour_costs.cpp
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_tour_costs '*'`
Expected: `[  PASSED  ] 5 tests.`

- [ ] **Step 6: Run both gates**

Run: `/tmp/mgg-tour/tuf-run.sh off test` then `/tmp/mgg-tour/tuf-run.sh on test`
Expected: both end with `0 errors, 0 failures`.

- [ ] **Step 7: Commit**

```bash
git add ros2/src/mgg_core/include/mgg_core/tour_costs.h \
        ros2/src/mgg_core/src/tour_costs.cpp \
        ros2/src/mgg_core/test/test_tour_costs.cpp \
        ros2/src/mgg_core/CMakeLists.txt
git commit -m "Cost the tour on the global graph with a first-leg heading penalty"
```

---

### Task 5: The tour planner: when to solve, and commitment

**Files:**
- Create: `ros2/src/mgg_core/include/mgg_core/tour_planner.h`
- Create: `ros2/src/mgg_core/src/tour_planner.cpp`
- Create: `ros2/src/mgg_core/test/test_tour_planner.cpp`
- Modify: `ros2/src/mgg_core/CMakeLists.txt` (library source, test)

**Interfaces:**
- Consumes: `mgg::TourParams` (Task 1), `mgg::FrontierCluster`, `mgg::ClusterId`, `mgg::kNoCluster` (Task 2), `mgg::solveOpenTour` (Task 3), `mgg::TourCostMatrix` (Task 4).
- Produces:
  - `struct mgg::TourPlan { std::vector<FrontierCluster> clusters; double cost; bool kept_target; };` (tour order; `clusters.front()` is the target)
  - `class mgg::TourPlanner { public: explicit TourPlanner(const TourParams&); bool needsSolve(const std::vector<FrontierCluster>& clusters, std::uint64_t graph_revision, std::uint64_t assignment_version, double now_s) const; const TourPlan& solve(const std::vector<FrontierCluster>& clusters, const TourCostMatrix& costs, std::uint64_t graph_revision, std::uint64_t assignment_version, double now_s); void releaseTarget(); ClusterId target() const; double targetSince() const; const TourPlan& plan() const; };`
  - `inline constexpr double mgg::kTowardTargetMaxAngleRad` (= pi/3)
  - `bool mgg::localPathServesTarget(const Eigen::Vector3d& robot, const Eigen::Vector3d& viewpoint, const Eigen::Vector3d& target, const Eigen::Vector3d& lattice_min, const Eigen::Vector3d& lattice_max);`

The spec (§2.4) has the robot explore locally "if the local lattice has gain in the direction of the target", and replaces `low_gain_rounds` with "the current target is outside the local lattice's reach or the lattice has no gain toward it". Read together: the local path is kept when the target lies inside the lattice box (local exploration reaches it) or the path's viewpoint lies within `kTowardTargetMaxAngleRad` of the target's bearing; otherwise the robot takes the global route. `localPathServesTarget` is that rule.

- [ ] **Step 1: Write the failing test**

Create `ros2/src/mgg_core/test/test_tour_planner.cpp`:

```cpp
// Tests for the tour planner (tour-exploration design §2.3, §2.4): when the
// tour is solved again, the commitment rule, and whether a local path
// serves the target.

#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/tour_planner.h"

namespace {

using mgg::ClusterId;
using mgg::FrontierCluster;
using mgg::TourCostMatrix;
using mgg::TourPlan;
using mgg::TourPlanner;

FrontierCluster named(ClusterId id, double x) {
  FrontierCluster c;
  c.id = id;
  c.owner_robot_id = 1;
  c.representative_vertex_id = static_cast<int>(id);
  c.position = Eigen::Vector3d(x, 0.0, 0.0);
  c.member_vertex_ids = {static_cast<int>(id)};
  return c;
}

/// Two clusters, `a` and `b` from the robot, `apart` between them.
TourCostMatrix twoClusters(double a, double b, double apart) {
  TourCostMatrix costs;
  costs.from_robot = {a, b};
  costs.between = {{0.0, apart}, {apart, 0.0}};
  return costs;
}

TEST(TourPlanner, TheFirstSolveTargetsTheTourFirstCluster) {
  TourPlanner planner{mgg::TourParams{}};
  const std::vector<FrontierCluster> clusters{named(1, 10.0), named(2, -11.0)};
  EXPECT_TRUE(planner.needsSolve(clusters, 1, 0, 0.0));
  EXPECT_EQ(planner.target(), mgg::kNoCluster);
  const TourPlan& plan =
      planner.solve(clusters, twoClusters(10.0, 11.0, 30.0), 1, 0, 0.0);
  ASSERT_EQ(plan.clusters.size(), 2u);
  EXPECT_EQ(plan.clusters.front().id, 1u);
  EXPECT_EQ(planner.target(), 1u);
  EXPECT_DOUBLE_EQ(plan.cost, 40.0);
  EXPECT_FALSE(plan.kept_target);
  EXPECT_DOUBLE_EQ(planner.targetSince(), 0.0);
}

TEST(TourPlanner, TheCommitmentMarginPreventsFlipFlop) {
  TourPlanner planner{mgg::TourParams{}};  // commit_margin 0.2
  const std::vector<FrontierCluster> clusters{named(1, 10.0), named(2, -11.0)};
  planner.solve(clusters, twoClusters(10.0, 11.0, 30.0), 1, 0, 0.0);
  ASSERT_EQ(planner.target(), 1u);

  // Starting at b is now 2.5 % cheaper (39 against 40): not enough.
  const TourPlan& kept =
      planner.solve(clusters, twoClusters(10.0, 9.0, 30.0), 2, 0, 1.0);
  EXPECT_EQ(planner.target(), 1u);
  EXPECT_TRUE(kept.kept_target);
  EXPECT_DOUBLE_EQ(kept.cost, 40.0);
  EXPECT_DOUBLE_EQ(planner.targetSince(), 0.0);

  // Starting at b saves 44 % (39 against 70): the target switches.
  const TourPlan& switched =
      planner.solve(clusters, twoClusters(40.0, 9.0, 30.0), 3, 0, 2.0);
  EXPECT_EQ(planner.target(), 2u);
  EXPECT_FALSE(switched.kept_target);
  EXPECT_DOUBLE_EQ(planner.targetSince(), 2.0);
}

TEST(TourPlanner, SolvesOnChangeAtMostEveryIntervalAndAtOnceWhenTheTargetGoes) {
  mgg::TourParams params;
  params.recompute_interval_s = 1.0;
  TourPlanner planner(params);
  const std::vector<FrontierCluster> clusters{named(1, 10.0), named(2, -11.0)};
  planner.solve(clusters, twoClusters(10.0, 11.0, 30.0), 1, 0, 0.0);

  EXPECT_FALSE(planner.needsSolve(clusters, 1, 0, 5.0));  // nothing changed
  EXPECT_FALSE(planner.needsSolve(clusters, 2, 0, 0.5));  // too soon
  EXPECT_TRUE(planner.needsSolve(clusters, 2, 0, 1.0));   // graph changed
  EXPECT_TRUE(planner.needsSolve(clusters, 1, 1, 1.0));   // assignment changed
  EXPECT_TRUE(planner.needsSolve({named(1, 10.0)}, 1, 0, 1.0));  // set changed
  // The target's cluster is gone: at once.
  EXPECT_TRUE(planner.needsSolve({named(2, -11.0)}, 1, 0, 0.1));
  // The target was reached and released: at once.
  planner.releaseTarget();
  EXPECT_EQ(planner.target(), mgg::kNoCluster);
  EXPECT_TRUE(planner.needsSolve(clusters, 1, 0, 0.1));
}

TEST(TourPlanner, AnUnreachableClusterIsLeftOut) {
  TourPlanner planner{mgg::TourParams{}};
  const std::vector<FrontierCluster> clusters{named(1, 10.0), named(2, -11.0)};
  const TourPlan& plan = planner.solve(
      clusters, twoClusters(10.0, mgg::kUnreachableCost, 30.0), 1, 0, 0.0);
  ASSERT_EQ(plan.clusters.size(), 1u);
  EXPECT_EQ(planner.target(), 1u);
  const TourPlan& none = planner.solve(
      clusters,
      twoClusters(mgg::kUnreachableCost, mgg::kUnreachableCost, 30.0), 2, 0,
      1.0);
  EXPECT_TRUE(none.clusters.empty());
  EXPECT_EQ(planner.target(), mgg::kNoCluster);
}

TEST(LocalPathServesTarget, AheadOrInsideTheLatticeOnly) {
  const Eigen::Vector3d lo(-1.0, -1.0, 0.0);
  const Eigen::Vector3d hi(3.0, 1.0, 0.0);
  const Eigen::Vector3d robot = Eigen::Vector3d::Zero();
  const Eigen::Vector3d far(20.0, 0.0, 0.0);
  EXPECT_TRUE(mgg::localPathServesTarget(robot, {2.0, 0.5, 0.0}, far, lo, hi));
  EXPECT_FALSE(mgg::localPathServesTarget(robot, {-2.0, 0.0, 0.0}, far, lo, hi));
  EXPECT_FALSE(mgg::localPathServesTarget(robot, {0.0, 2.0, 0.0}, far, lo, hi));
  EXPECT_FALSE(mgg::localPathServesTarget(robot, robot, far, lo, hi));
  // A target inside the lattice box is served by whatever local path.
  EXPECT_TRUE(mgg::localPathServesTarget(robot, {-1.0, 0.0, 0.0},
                                         {2.0, 0.5, 0.0}, lo, hi));
}

}  // namespace
```

Register it in `ros2/src/mgg_core/CMakeLists.txt` after the `test_tour_costs` lines:

```cmake
  ament_add_gtest(test_tour_planner test/test_tour_planner.cpp)
  target_link_libraries(test_tour_planner ${PROJECT_NAME})
```

- [ ] **Step 2: Run it to verify it fails**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_tour_planner '*'`
Expected: FAIL at compile time with `mgg_core/tour_planner.h: No such file or directory`.

- [ ] **Step 3: Write the header**

Create `ros2/src/mgg_core/include/mgg_core/tour_planner.h`:

```cpp
// The per-robot tour (tour-exploration design §2.3, §2.4): when it is solved
// again, and the commitment that keeps the robot on its target.
//
// The robot's current target is the first cluster of its tour. It is kept
// until reached, explored (its cluster vanishes) or reassigned (the
// assignment version changes and the cluster leaves the robot's set); a new
// first cluster replaces it only when it lowers the remaining tour cost by
// more than tour.commit_margin.

#ifndef MGG_CORE_TOUR_PLANNER_H_
#define MGG_CORE_TOUR_PLANNER_H_

#include <cstdint>
#include <set>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/frontier_clusters.h"
#include "mgg_core/tour_costs.h"
#include "mgg_core/tour_params.h"

namespace mgg {

struct TourPlan {
  /// In tour order; the first is the current target.
  std::vector<FrontierCluster> clusters;
  double cost = 0.0;
  /// The commitment kept a target the solver would have replaced.
  bool kept_target = false;
};

class TourPlanner {
 public:
  explicit TourPlanner(const TourParams& params) : params_(params) {}

  /// §2.3: at once before the first solve, when the target's cluster is
  /// gone, or after releaseTarget; otherwise when the graph revision, the
  /// assignment version or the set of cluster IDs changed, and at least
  /// tour.recompute_interval_s after the last solve.
  bool needsSolve(const std::vector<FrontierCluster>& clusters,
                  std::uint64_t graph_revision,
                  std::uint64_t assignment_version, double now_s) const;
  /// Solves the tour over `clusters` with `costs` (indexed alike) and applies
  /// the commitment rule.
  const TourPlan& solve(const std::vector<FrontierCluster>& clusters,
                        const TourCostMatrix& costs,
                        std::uint64_t graph_revision,
                        std::uint64_t assignment_version, double now_s);
  /// The target was reached, or no route to it exists: the next solve
  /// chooses freely.
  void releaseTarget();

  ClusterId target() const { return target_; }
  /// When the current target was taken (the bid's claim stamp).
  double targetSince() const { return target_since_s_; }
  const TourPlan& plan() const { return plan_; }

 private:
  TourParams params_;
  TourPlan plan_;
  ClusterId target_ = kNoCluster;
  double target_since_s_ = 0.0;
  bool solved_ = false;
  bool released_ = false;
  std::uint64_t graph_revision_ = 0;
  std::uint64_t assignment_version_ = 0;
  std::set<ClusterId> cluster_ids_;
  double solved_at_s_ = 0.0;
};

/// A local path's viewpoint serves the target when it heads within this of
/// the target's bearing from the robot.
inline constexpr double kTowardTargetMaxAngleRad = 1.0471975511965976;  // pi/3

/// §2.4: whether the local exploration path ending at `viewpoint` serves the
/// tour's `target`: the target lies within the lattice box around the robot
/// (`lattice_min`/`lattice_max`, offsets in x and y, as the grid graph lays
/// it out), or the viewpoint lies within kTowardTargetMaxAngleRad of the
/// target's bearing.
bool localPathServesTarget(const Eigen::Vector3d& robot,
                           const Eigen::Vector3d& viewpoint,
                           const Eigen::Vector3d& target,
                           const Eigen::Vector3d& lattice_min,
                           const Eigen::Vector3d& lattice_max);

}  // namespace mgg

#endif  // MGG_CORE_TOUR_PLANNER_H_
```

- [ ] **Step 4: Write the implementation**

Create `ros2/src/mgg_core/src/tour_planner.cpp`:

```cpp
#include "mgg_core/tour_planner.h"

#include <cmath>

#include "mgg_core/tour_solver.h"

namespace mgg {

bool TourPlanner::needsSolve(const std::vector<FrontierCluster>& clusters,
                             std::uint64_t graph_revision,
                             std::uint64_t assignment_version,
                             double now_s) const {
  if (!solved_ || released_) return true;
  std::set<ClusterId> ids;
  for (const FrontierCluster& cluster : clusters) ids.insert(cluster.id);
  if (target_ != kNoCluster && ids.count(target_) == 0) return true;
  const bool changed = graph_revision != graph_revision_ ||
                       assignment_version != assignment_version_ ||
                       ids != cluster_ids_;
  return changed && now_s - solved_at_s_ >= params_.recompute_interval_s;
}

const TourPlan& TourPlanner::solve(const std::vector<FrontierCluster>& clusters,
                                   const TourCostMatrix& costs,
                                   std::uint64_t graph_revision,
                                   std::uint64_t assignment_version,
                                   double now_s) {
  solved_ = true;
  released_ = false;
  graph_revision_ = graph_revision;
  assignment_version_ = assignment_version;
  solved_at_s_ = now_s;
  cluster_ids_.clear();
  for (const FrontierCluster& cluster : clusters) cluster_ids_.insert(cluster.id);

  const OpenTour best = solveOpenTour(costs.from_robot, costs.between);
  OpenTour chosen = best;
  bool kept = false;
  int current = -1;
  for (std::size_t i = 0; i < clusters.size(); ++i) {
    if (clusters[i].id == target_) current = static_cast<int>(i);
  }
  if (current >= 0 && !best.order.empty() && best.order.front() != current) {
    const OpenTour committed =
        solveOpenTour(costs.from_robot, costs.between, current);
    if (!committed.order.empty() && std::isfinite(committed.cost) &&
        !(best.cost < (1.0 - params_.commit_margin) * committed.cost)) {
      chosen = committed;
      kept = true;
    }
  }

  plan_ = TourPlan{};
  plan_.cost = chosen.cost;
  plan_.kept_target = kept;
  for (const int index : chosen.order) plan_.clusters.push_back(clusters[index]);
  const ClusterId next =
      plan_.clusters.empty() ? kNoCluster : plan_.clusters.front().id;
  if (next != target_) target_since_s_ = now_s;
  target_ = next;
  return plan_;
}

void TourPlanner::releaseTarget() {
  target_ = kNoCluster;
  released_ = true;
}

bool localPathServesTarget(const Eigen::Vector3d& robot,
                           const Eigen::Vector3d& viewpoint,
                           const Eigen::Vector3d& target,
                           const Eigen::Vector3d& lattice_min,
                           const Eigen::Vector3d& lattice_max) {
  const Eigen::Vector2d offset = (target - robot).head<2>();
  if (offset.x() >= lattice_min.x() && offset.x() <= lattice_max.x() &&
      offset.y() >= lattice_min.y() && offset.y() <= lattice_max.y()) {
    return true;
  }
  const Eigen::Vector2d step = (viewpoint - robot).head<2>();
  if (step.norm() < 1e-6 || offset.norm() < 1e-6) return false;
  const double angle =
      std::abs(std::remainder(std::atan2(step.y(), step.x()) -
                                  std::atan2(offset.y(), offset.x()),
                              2.0 * M_PI));
  return angle <= kTowardTargetMaxAngleRad;
}

}  // namespace mgg
```

Add the source to `add_library` in `ros2/src/mgg_core/CMakeLists.txt`, after `  src/tour_costs.cpp`:

```cmake
  src/tour_planner.cpp
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_tour_planner '*'`
Expected: `[  PASSED  ] 5 tests.`

- [ ] **Step 6: Run both gates**

Run: `/tmp/mgg-tour/tuf-run.sh off test` then `/tmp/mgg-tour/tuf-run.sh on test`
Expected: both end with `0 errors, 0 failures`.

- [ ] **Step 7: Commit**

```bash
git add ros2/src/mgg_core/include/mgg_core/tour_planner.h \
        ros2/src/mgg_core/src/tour_planner.cpp \
        ros2/src/mgg_core/test/test_tour_planner.cpp \
        ros2/src/mgg_core/CMakeLists.txt
git commit -m "Keep the tour's target unless a new start saves the commit margin"
```

---
### Task 6: Wire the tour into the planner node (delivery step 1)

> **Apply by anchor.** This task edits `PlannerNode::onPlanRequest` (the resume check and the head of the low-gain block) and adds `PlannerNodeTestPeer` helpers. Apply each edit by its anchor after re-reading the current code; keep the run-5 and run-6 changes (`goes_nowhere`, `mgg::Departure`, `paths_going_nowhere_`, `local_gain_remains_now_`, `frontiers_dropped_in_rebuild_`) as they are. `buildLocalGraph`, `departBoxedIn`, `straightDeparture`, `runGlobalPlanner` and `path_selection.cpp` are not touched. Checked against 4777a17: every anchor below exists once, and the tour's first-leg heading penalty takes `current_state_[3]`, the current heading that local path selection also prefers since aeec9aa.

**Files:**
- Modify: `ros2/src/mgg_ros/include/mgg_ros/planner_node.h` (includes, six member functions, eight members, one publisher)
- Modify: `ros2/src/mgg_ros/src/planner_node.cpp` (`loadParameters`, constructor, six new functions after `findGlobalVertex`, `onPlanRequest`)
- Test: `ros2/src/mgg_ros/test/test_planner_node.cpp` (three helpers, one helper changed, two tests)

**Interfaces:**
- Consumes: `mgg_ros::loadTourParams`, `mgg_ros::loadFleetParams`, `mgg::TourParams`, `mgg::FleetParams` (Task 1); `mgg::extractFrontierClusters`, `mgg::ClusterIdRegistry`, `mgg::FrontierCluster`, `mgg::kNoCluster` (Task 2); `mgg::computeTourCosts`, `mgg::GraphDistanceCache` (Task 4); `mgg::TourPlanner`, `mgg::TourPlan`, `mgg::localPathServesTarget` (Task 5).
- Produces (used by Tasks 11 and 12):
  - `mgg::Vertex* PlannerNode::linkRobotToGlobalGraph();`
  - `std::vector<mgg::FrontierCluster> PlannerNode::globalFrontierClusters();`
  - `std::vector<mgg::FrontierCluster> PlannerNode::tourCandidates(std::vector<mgg::FrontierCluster> clusters);` (Task 11 replaces its body)
  - `std::optional<mgg::FrontierCluster> PlannerNode::refreshTour(std::string& note);`
  - `bool PlannerNode::tourKeepsRoute(int vertex_id) const;`, `void PlannerNode::publishTour();`
  - members `tour_params_`, `fleet_params_`, `cluster_ids_`, `tour_planner_`, `tour_distances_`, `tour_clusters_`, `tour_assignment_version_`, `tour_solve_ms_`, `tour_pub_`
  - topic `tour` (`nav_msgs/Path`, latched): the robot's pose, then its clusters' representatives in tour order. SwarmDeck's delivery step 3 shows it.
  - test helpers `PlannerNodeTestPeer::setTour(PlannerNode&, bool enabled, double min_cluster_gain)`, `tourTarget(PlannerNode&) -> mgg::ClusterId`, `bestPathFromGlobalGraph(PlannerNode&) -> bool`

Behaviour (spec §2.4): after `buildLocalGraph`, the tour is refreshed. When it has a target, the local path is kept if it serves the target (`localPathServesTarget`); otherwise the robot routes over the global graph to the target's representative (`runGlobalPlanner(target_id)`), with no `low_gain_rounds` wait. A boxed-in robot keeps its departure. With no target, or `tour.enabled` false, the low-gain rule runs exactly as before. Without fleet assignment (Task 11), the tour prefers this robot's own clusters and takes other robots' only when none of its own is left, as `kGlobalOtherRobotPenalty` did, and a peer reservation (SwarmDeck lease) still excludes its cluster.

- [ ] **Step 1: Write the failing tests**

In `ros2/src/mgg_ros/test/test_planner_node.cpp`, class `PlannerNodeTestPeer`, replace `consultGlobalPlannerAtOnce` with:

```cpp
  /// The global planner runs as soon as the lattice has no frontier. That
  /// is the low-gain rule, which the tour replaces (tour-exploration design
  /// §2.4), so the tour is off.
  static void consultGlobalPlannerAtOnce(PlannerNode& node) {
    node.auto_global_planner_low_gain_rounds_ = 0;
    node.tour_params_.enabled = false;
  }
```

and add after it:

```cpp
  /// The tour on or off, and the least cluster gain it takes: a frontier
  /// added by a test has no gain until the first re-check scores it.
  static void setTour(PlannerNode& node, bool enabled,
                      double min_cluster_gain) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.tour_params_.enabled = enabled;
    node.tour_params_.min_cluster_gain = min_cluster_gain;
    node.tour_planner_ = std::make_unique<mgg::TourPlanner>(node.tour_params_);
  }
  static mgg::ClusterId tourTarget(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.tour_planner_->target();
  }
  static bool bestPathFromGlobalGraph(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.best_path_from_global_graph_;
  }
```

Add these two tests at the end of the file, before the closing `}  // namespace mgg_ros`:

```cpp
TEST_F(PlannerNodeTest, TheTourHeadsForItsTargetWithoutWaitingForLowGainRounds) {
  // exploredDeadEnd: the lattice has no gain, and the tour's only cluster
  // is the frontier 2.5 m behind the robot. The low-gain rule waits
  // auto_global_planner_low_gain_rounds (15) cycles and sends nothing; the
  // tour routes to the frontier at once (tour-exploration design §2.4).
  // That route starts with a turn the robot has no room for, so it backs
  // out toward the frontier first, as any withheld route does.
  for (const bool tour : {false, true}) {
    SCOPED_TRACE(tour ? "tour on" : "tour off");
    int frontier = -1;
    auto node =
        exploredDeadEnd(tour ? "tour_at_once" : "tour_off_waits", frontier);
    PlannerNodeTestPeer::setTour(*node, tour, 0.0);

    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    if (!tour) {
      EXPECT_EQ(response->status, PlannerNode::kStatusNoPath);
      EXPECT_EQ(PlannerNodeTestPeer::routeSharpTurnFallbacks(*node), 0);
      EXPECT_EQ(PlannerNodeTestPeer::tourTarget(*node), mgg::kNoCluster);
      continue;
    }
    EXPECT_NE(PlannerNodeTestPeer::tourTarget(*node), mgg::kNoCluster);
    EXPECT_EQ(PlannerNodeTestPeer::routeSharpTurnFallbacks(*node), 1);
    expectReverseDepartureFromDeadEnd(*node, *response);
  }
}

TEST_F(PlannerNodeTest, LocalExplorationTowardTheTourTargetIsKept) {
  // Floor mapped to x = 4 and unknown beyond: the lattice has gain ahead,
  // and so does the tour: a global frontier 3.5 m ahead, and the frontier
  // the accepted lattice path leaves in the global graph. The robot
  // explores locally toward its target rather than taking the global
  // route.
  auto node = makeNode("tour_local_toward");
  PlannerNodeTestPeer::observeFloor(*node, -1.5, 4.0, -1.5, 1.5);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(
      *node, {{0.5, 0.0}, {1.0, 0.0}, {1.5, 0.0}, {2.0, 0.0}, {2.5, 0.0},
              {3.0, 0.0}, {3.5, 0.0}});
  PlannerNodeTestPeer::setTour(*node, true, 0.0);

  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  PlannerNodeTestPeer::plan(*node, response);
  ASSERT_EQ(response->status, mgg_msgs::srv::PlannerSrv::Response::FORWARD);
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_FALSE(PlannerNodeTestPeer::bestPathFromGlobalGraph(*node));
  EXPECT_NE(PlannerNodeTestPeer::tourTarget(*node), mgg::kNoCluster);
  EXPECT_GT(response->path.back().position.x, 0.0);
}
```

- [ ] **Step 2: Run them to verify they fail**

Run: `/tmp/mgg-tour/tuf-run.sh on quick mgg_ros test_planner_node '*Tour*'`
Expected: FAIL at compile time with `'class mgg_ros::PlannerNode' has no member named 'tour_params_'`.

- [ ] **Step 3: Declare the tour in the node's header**

In `ros2/src/mgg_ros/include/mgg_ros/planner_node.h`:

After `#include "mgg_core/trajectory.h"` add:

```cpp
#include "mgg_core/frontier_clusters.h"
#include "mgg_core/tour_costs.h"
#include "mgg_core/tour_params.h"
#include "mgg_core/tour_planner.h"
```

After the declaration `mgg::Vertex* findGlobalVertex(int id) const;` add:

```cpp
  /// Links where the robot stands into the global graph, as a route out of
  /// here does (mgg::linkDeparture); null when nothing links.
  mgg::Vertex* linkRobotToGlobalGraph();
  /// Every frontier cluster of the global graph, other robots' included,
  /// under its stable name (tour-exploration design §2.1).
  std::vector<mgg::FrontierCluster> globalFrontierClusters();
  /// Of `clusters`, those this robot's tour may visit: without fleet
  /// assignment, its own, or every robot's once none of its own is left
  /// (as kGlobalOtherRobotPenalty preferred them), less those a peer's
  /// reservation excludes.
  std::vector<mgg::FrontierCluster> tourCandidates(
      std::vector<mgg::FrontierCluster> clusters);
  /// §2.3: solves the tour again when due and returns its current target,
  /// or nothing when it has none. `note` is for the plan summary.
  std::optional<mgg::FrontierCluster> refreshTour(std::string& note);
  /// Whether a repositioning to global vertex `vertex_id` still heads for
  /// the tour's target (always, when the tour is off or has no target).
  bool tourKeepsRoute(int vertex_id) const;
  /// The robot's pose, then its clusters' representatives in tour order.
  void publishTour();
```

After the member `bool add_frontiers_to_global_graph_ = false;` add:

```cpp
  /// Tour-based exploration (tour-exploration design §2): its parameters,
  /// the fleet's (cluster_merge_radius_m groups the clusters), the stable
  /// names of the global frontier clusters, the tour over them, and the
  /// graph distances it was costed with, one Dijkstra per cluster per graph
  /// revision.
  mgg::TourParams tour_params_;
  mgg::FleetParams fleet_params_;
  mgg::ClusterIdRegistry cluster_ids_;
  std::unique_ptr<mgg::TourPlanner> tour_planner_;
  mgg::GraphDistanceCache tour_distances_;
  /// The clusters the last refreshTour offered the tour.
  std::vector<mgg::FrontierCluster> tour_clusters_;
  /// Changes when the clusters this robot may visit change for a reason
  /// other than the graph (the fleet's assignment); the tour solves again.
  std::uint64_t tour_assignment_version_ = 0;
  /// The last tour's costing and solving time, for the plan summary.
  double tour_solve_ms_ = 0.0;
```

After the member `rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;` add:

```cpp
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tour_pub_;
```

- [ ] **Step 4: Load, create and publish the tour**

In `ros2/src/mgg_ros/src/planner_node.cpp`:

In `PlannerNode::loadParameters`, after the `loadSensorSet` block (the one warning `no sensors loaded from SensorParams`), add:

```cpp
  loadTourParams(p, "tour", tour_params_);
  loadFleetParams(p, "fleet", fleet_params_);
```

In the constructor, after the `path_pub_ = create_publisher<nav_msgs::msg::Path>(...)` statement, add:

```cpp
  // The tour (tour-exploration design §2), latched as best_path is.
  tour_planner_ = std::make_unique<mgg::TourPlanner>(tour_params_);
  tour_pub_ = create_publisher<nav_msgs::msg::Path>(
      "tour", rclcpp::QoS(1).transient_local());
```

After the definition of `PlannerNode::findGlobalVertex`, add:

```cpp
mgg::Vertex* PlannerNode::linkRobotToGlobalGraph() {
  mgg::StateVec current = current_state_;
  if (!projectToDrivingHeight(current)) {
    current = physicalAnchorAtDrivingHeight(current_state_);
  }
  const int before = global_graph_->getNumVertices();
  mgg::Vertex* link = mgg::linkDeparture(*global_graph_, current,
                                         makeGlobalContext(), kLinkRadius)
                          .vertex;
  if (global_graph_->getNumVertices() != before) ++graph_revision_;
  return link;
}

std::vector<mgg::FrontierCluster> PlannerNode::globalFrontierClusters() {
  std::vector<mgg::FrontierCluster> clusters = mgg::extractFrontierClusters(
      *global_graph_, fleet_params_.cluster_merge_radius_m,
      tour_params_.min_cluster_gain, tour_params_.cluster_id_cell_m);
  cluster_ids_.stabilize(clusters, fleet_params_.cluster_merge_radius_m);
  return clusters;
}

std::vector<mgg::FrontierCluster> PlannerNode::tourCandidates(
    std::vector<mgg::FrontierCluster> clusters) {
  const std::vector<Eigen::Vector3d> reserved = selectionExclusions();
  clusters.erase(
      std::remove_if(clusters.begin(), clusters.end(),
                     [&](const mgg::FrontierCluster& cluster) {
                       return std::any_of(
                           reserved.begin(), reserved.end(),
                           [&](const Eigen::Vector3d& point) {
                             return (cluster.position - point).norm() <=
                                    reservation_exclusion_radius_m_;
                           });
                     }),
      clusters.end());
  const int own_id = static_cast<int>(planning_params_.robot_id);
  const auto others = [own_id](const mgg::FrontierCluster& cluster) {
    return cluster.owner_robot_id != own_id;
  };
  if (!std::all_of(clusters.begin(), clusters.end(), others)) {
    clusters.erase(std::remove_if(clusters.begin(), clusters.end(), others),
                   clusters.end());
  }
  return clusters;
}

std::optional<mgg::FrontierCluster> PlannerNode::refreshTour(
    std::string& note) {
  note.clear();
  if (!tour_params_.enabled || global_graph_->getNumVertices() <= 1) {
    tour_clusters_.clear();
    return std::nullopt;
  }
  std::vector<mgg::FrontierCluster> clusters =
      tourCandidates(globalFrontierClusters());
  // Reached: within global_frontier_reach_m of it, as a repositioning's
  // frontier is (onPlanRequest). The next solve chooses freely.
  for (const mgg::FrontierCluster& cluster : clusters) {
    if (cluster.id == tour_planner_->target() &&
        (cluster.position - current_state_.head<3>()).norm() <=
            global_frontier_reach_m_) {
      tour_planner_->releaseTarget();
      break;
    }
  }
  const double now_s = now().seconds();
  if (tour_planner_->needsSolve(clusters, graph_revision_,
                                tour_assignment_version_, now_s)) {
    mgg::Vertex* link = linkRobotToGlobalGraph();
    if (link == nullptr) {
      tour_clusters_.clear();
      note = "; tour: the robot's pose cannot be linked to the global graph";
      return std::nullopt;
    }
    const auto started = std::chrono::steady_clock::now();
    const mgg::TourCostMatrix costs = mgg::computeTourCosts(
        *global_graph_, graph_revision_, tour_distances_, link->id,
        current_state_[3], clusters, tour_params_.heading_weight);
    tour_planner_->solve(clusters, costs, graph_revision_,
                         tour_assignment_version_, now_s);
    tour_solve_ms_ = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - started)
                         .count();
    publishTour();
  }
  tour_clusters_ = clusters;
  const mgg::TourPlan& plan = tour_planner_->plan();
  for (const mgg::FrontierCluster& cluster : tour_clusters_) {
    if (cluster.id != tour_planner_->target()) continue;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "; tour: %zu of %zu cluster(s), %.1f m%s, target %016llx "
                  "at (%.2f, %.2f, %.2f), costed and solved in %.1f ms",
                  plan.clusters.size(), tour_clusters_.size(), plan.cost,
                  plan.kept_target ? " (target kept)" : "",
                  static_cast<unsigned long long>(cluster.id),
                  cluster.position.x(), cluster.position.y(),
                  cluster.position.z(), tour_solve_ms_);
    note = buf;
    return cluster;
  }
  note = tour_clusters_.empty() ? "; tour: no cluster"
                                : "; tour: no reachable cluster";
  return std::nullopt;
}

bool PlannerNode::tourKeepsRoute(int vertex_id) const {
  if (!tour_params_.enabled) return true;
  const mgg::ClusterId target = tour_planner_->target();
  if (target == mgg::kNoCluster) return true;
  for (const mgg::FrontierCluster& cluster : tour_clusters_) {
    if (cluster.id != target) continue;
    return std::find(cluster.member_vertex_ids.begin(),
                     cluster.member_vertex_ids.end(),
                     vertex_id) != cluster.member_vertex_ids.end();
  }
  return true;
}

void PlannerNode::publishTour() {
  nav_msgs::msg::Path msg;
  msg.header.stamp = now();
  msg.header.frame_id = world_frame_;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg.header;
  pose.pose = toPoseMsg(current_state_);
  msg.poses.push_back(pose);
  for (const mgg::FrontierCluster& cluster : tour_planner_->plan().clusters) {
    pose.pose = toPoseMsg(mgg::StateVec(cluster.position.x(),
                                        cluster.position.y(),
                                        cluster.position.z(), 0.0));
    msg.poses.push_back(pose);
  }
  tour_pub_->publish(msg);
}
```

- [ ] **Step 5: Let the tour decide in `onPlanRequest`**

In `PlannerNode::onPlanRequest`, the resume check keeps a repositioning only while it still heads for the tour's target. Replace:

```cpp
    if (target != nullptr &&
        (current_state_.head<3>() - target->state.head<3>()).norm() >
            global_frontier_reach_m_) {
      resume_global = true;
```

with:

```cpp
    if (target != nullptr &&
        (current_state_.head<3>() - target->state.head<3>()).norm() >
            global_frontier_reach_m_ &&
        tourKeepsRoute(current_global_vertex_id_)) {
      resume_global = true;
```

Then, in the `if (best_path_.empty()) {` block that follows the resume block, replace its head, from the block's first line through the condition of its boxed-in branch:

```cpp
  if (best_path_.empty()) {
    summary = buildLocalGraph() + resumed;
    // An empty lattice is a map that does not yet show the robot's
    // surroundings, not an explored one: PCI retries as the map grows. The
    // global planner is consulted once the lattice existed and saw nothing
    // new for long enough.
    const bool low_gain =
        best_path_.empty() && local_graph_->getNumVertices() > 1 &&
        low_gain_rounds_ >= auto_global_planner_low_gain_rounds_;
    if (low_gain &&
        (boxed_in_without_departure_now_ || withheld_without_departure)) {
```

with:

```cpp
  if (best_path_.empty()) {
    const int departures_before = boxed_in_departures_;
    summary = buildLocalGraph() + resumed;
    // A boxed-in robot sent a straight departure (buildLocalGraph) keeps it.
    const bool departed = boxed_in_departures_ != departures_before;
    // Tour-exploration design §2.4: the tour's target, not low_gain_rounds,
    // decides when the robot leaves local exploration for the global graph:
    // as soon as the lattice has no path toward the target.
    std::optional<mgg::FrontierCluster> tour_target;
    if (tour_params_.enabled && local_graph_->getNumVertices() > 1) {
      auto map_read = mapReadLease();
      std::string tour_note;
      tour_target = refreshTour(tour_note);
      summary += tour_note;
    }
    bool tour_decided = false;
    if (tour_target.has_value() && !departed &&
        !boxed_in_without_departure_now_ && !withheld_without_departure) {
      if (!best_path_.empty() &&
          mgg::localPathServesTarget(
              current_state_.head<3>(), best_path_.back().head<3>(),
              tour_target->position, grid_params_.min_val,
              grid_params_.max_val)) {
        tour_decided = true;
        summary += "; exploring locally toward the tour's target";
      } else {
        auto map_read = mapReadLease();
        const std::vector<mgg::StateVec> local_path = best_path_;
        std::string departure;
        if (runGlobalPlanner(tour_target->representative_vertex_id, reason)) {
          tour_decided = true;
          low_gain_rounds_ = 0;
          if (depart_instead_of_turning_route(departure)) {
            summary +=
                "; the route to the tour's target starts with a turn the "
                "robot has no room for" +
                departure;
          } else {
            summary += "; routing to the tour's target over the global graph";
          }
        } else {
          // No route after all: the next solve chooses again, and this
          // cycle keeps what local exploration found.
          best_path_ = local_path;
          best_path_from_global_graph_ = false;
          tour_planner_->releaseTarget();
          summary += "; no route to the tour's target: " + reason;
        }
      }
    }
    // An empty lattice is a map that does not yet show the robot's
    // surroundings, not an explored one: PCI retries as the map grows. The
    // global planner is consulted once the lattice existed and saw nothing
    // new for long enough.
    const bool low_gain =
        best_path_.empty() && local_graph_->getNumVertices() > 1 &&
        low_gain_rounds_ >= auto_global_planner_low_gain_rounds_;
    if (tour_decided) {
      // The tour decided this cycle.
    } else if (low_gain &&
               (boxed_in_without_departure_now_ ||
                withheld_without_departure)) {
```

The rest of the block stays as it is: the boxed-in branch's body, and the `else if (low_gain) { ... }` branch with its two no-path exceptions (`local_gain_remains_now_`, `frontiers_dropped_in_rebuild_`) before `exploration complete`. The first version of this plan replaced the whole block with a copy of the run-5 low-gain branch, which drops those exceptions: `AFailedGlobalSearchWithLocalGainLeftIsNotExplorationComplete` and `TheFirstFailedSearchAfterARebuildDroppedFrontiersIsNotComplete` (tour off through `consultGlobalPlannerAtOnce`) would then get exploration complete where they expect no path.

- [ ] **Step 6: Run the new tests to verify they pass**

Run: `/tmp/mgg-tour/tuf-run.sh on quick mgg_ros test_planner_node '*Tour*'`
Expected: `[       OK ] PlannerNodeTest.TheTourHeadsForItsTargetWithoutWaitingForLowGainRounds` and `[       OK ] PlannerNodeTest.LocalExplorationTowardTheTourTargetIsKept`.

Then the whole binary: `/tmp/mgg-tour/tuf-run.sh on quick mgg_ros test_planner_node '*'`
Expected: every test `OK`. If an existing test changed outcome, it is one that relies on the low-gain rule without calling `consultGlobalPlannerAtOnce`: read its plan summary (`plan request: ... tour: ...`) and report it to the supervisor before changing the test.

- [ ] **Step 7: Run both gates**

Run: `/tmp/mgg-tour/tuf-run.sh off test` then `/tmp/mgg-tour/tuf-run.sh on test`
Expected: both end with `0 errors, 0 failures`.

- [ ] **Step 8: Commit**

```bash
git add ros2/src/mgg_ros/include/mgg_ros/planner_node.h \
        ros2/src/mgg_ros/src/planner_node.cpp \
        ros2/src/mgg_ros/test/test_planner_node.cpp
git commit -m "Let the tour's target decide when the robot repositions"
```

Delivery step 1 is complete here: the per-robot tour can be deployed and measured alone (Task 13, phase A).

---
### Task 7: Fleet messages, exchange types and their conversions

**Files:**
- Create: `ros2/src/mgg_msgs/msg/TourCluster.msg`, `ros2/src/mgg_msgs/msg/TourBundle.msg`, `ros2/src/mgg_msgs/msg/TourBid.msg`, `ros2/src/mgg_msgs/msg/TourAward.msg`
- Modify: `ros2/src/mgg_msgs/CMakeLists.txt` (four messages)
- Create: `ros2/src/mgg_core/include/mgg_core/fleet_types.h`, `ros2/src/mgg_core/src/fleet_types.cpp`, `ros2/src/mgg_core/test/test_fleet_types.cpp`
- Modify: `ros2/src/mgg_core/CMakeLists.txt` (library source, test)
- Create: `ros2/src/mgg_ros/include/mgg_ros/fleet_conversions.h`, `ros2/src/mgg_ros/src/fleet_conversions.cpp`, `ros2/src/mgg_ros/test/test_fleet_conversions.cpp`
- Modify: `ros2/src/mgg_ros/CMakeLists.txt` (library source, test)

**Interfaces:**
- Consumes: `mgg::ClusterId`, `mgg::kNoCluster` (Task 2); `mgg_ros::toPoseMsg`, `mgg_ros::fromPoseMsg` (existing `conversions.h`).
- Produces:
  - messages `mgg_msgs/TourCluster`, `mgg_msgs/TourBundle`, `mgg_msgs/TourBid`, `mgg_msgs/TourAward`
  - `inline constexpr std::size_t mgg::kMaxBidClusters = 1024;`
  - `struct mgg::FleetCluster { ClusterId id; int owner_robot_id; Eigen::Vector3d position; double gain; };`
  - `struct mgg::TourBidData { int robot_id; std::uint64_t seq; double stamp_s; std::uint64_t auction_id; StateVec pose; std::vector<FleetCluster> clusters; std::vector<double> costs_from_pose; std::vector<double> costs_between; ClusterId current_target; double claim_stamp_s; std::vector<ClusterId> bundle; std::vector<ClusterId> explored; bool request_auction; double costBetween(std::size_t i, std::size_t j) const; bool wellFormed() const; };`
  - `struct mgg::RobotBundle { int robot_id; std::vector<ClusterId> clusters; double silent_s; };`
  - `struct mgg::TourAwardData { std::uint64_t auction_id; int auctioneer_id; double stamp_s; bool call; std::vector<FleetCluster> clusters; std::vector<RobotBundle> bundles; std::vector<FleetCluster> explored; std::vector<int> released_robot_ids; const RobotBundle* bundleOf(int robot_id) const; const FleetCluster* cluster(ClusterId id) const; };`
  - `using mgg::ExploredFn = std::function<bool(const Eigen::Vector3d&)>;`, `using mgg::CostEstimateFn = std::function<double(const Eigen::Vector3d& from, const Eigen::Vector3d& to)>;`
  - `double mgg_ros::stampSeconds(const builtin_interfaces::msg::Time&)`, `builtin_interfaces::msg::Time mgg_ros::stampFromSeconds(double)`
  - `mgg::TourBidData mgg_ros::fromTourBidMsg(const mgg_msgs::msg::TourBid&, const Eigen::Isometry3d& t_ours_theirs)`, `mgg_msgs::msg::TourBid mgg_ros::toTourBidMsg(const mgg::TourBidData&, const std::string& frame_id)`
  - `mgg::TourAwardData mgg_ros::fromTourAwardMsg(const mgg_msgs::msg::TourAward&, const Eigen::Isometry3d& t_ours_theirs)`, `mgg_msgs::msg::TourAward mgg_ros::toTourAwardMsg(const mgg::TourAwardData&, const std::string& frame_id)`

The message fields are the spec's (§3.3, §3.4) plus what the protocol needs to work, each named in the message comments: `TourBid` carries `explored` (awarded clusters the bidder's roadmap shows explored, how "a cluster lying in explored space of any robot in the group is dropped" reaches the auctioneer) and `request_auction` (§3.2's "a robot can request one through its bid"); `TourAward` carries `call` (§3.3), the clusters' positions (so a receiver can place the IDs it is given), per-robot `silent_s` (so a silent robot's claim ages the same everywhere: §4's TTL counts from when the robot was last heard, not from when the award arrived), `explored`, and `released_robot_ids` (§4's operator release, "forwarded in the next award"). The spec's `stamp` is `header.stamp`; `header.frame_id` names the sender's planning frame, which picks the neighbour transform, as `Graph.msg` does.

- [ ] **Step 1: Write the messages**

Create `ros2/src/mgg_msgs/msg/TourCluster.msg`:

```
# A frontier cluster as the fleet exchanges it (tour-exploration design
# §2.1, §3.3).
uint64 id                      # stable cluster ID (mgg::makeClusterId); never 0
int32 owner_robot_id           # the robot whose frontier it is
geometry_msgs/Point position   # its representative, in the message's frame
float64 gain                   # the representative's volumetric gain
```

Create `ros2/src/mgg_msgs/msg/TourBundle.msg`:

```
# One robot's awarded frontier clusters, in tour order (design §3.4).
int32 robot_id
uint64[] clusters
# How long the auctioneer has not heard this robot, seconds; 0 when it bid on
# time. A silent robot's claim expires fleet.claim_ttl_s after it was last
# heard (§4), wherever the award is received.
float64 silent_s
```

Create `ros2/src/mgg_msgs/msg/TourBid.msg`:

```
# One robot's bid in its group's frontier auction (tour-exploration design
# §3.3). Every robot also bids every fleet.auction_interval_s unasked, which
# is how its peers hear it (§3.1).
std_msgs/Header header             # stamp; frame_id: the sender's planning frame
int32 robot_id
uint64 seq
uint64 auction_id                  # the auction called; 0 for a periodic bid
geometry_msgs/Pose pose            # the sender's pose, in its planning frame
TourCluster[] clusters             # every frontier cluster it knows
float64[] costs_from_pose          # one per cluster; .inf when unreachable
float64[] costs_between            # clusters x clusters, row-major; .inf when unreachable
uint64 current_target              # 0: none
builtin_interfaces/Time claim_stamp  # when it took its current target
uint64[] bundle                    # its current bundle
uint64[] explored                  # awarded clusters its roadmap shows explored
bool request_auction               # its bundle is done: it asks for an auction
```

Create `ros2/src/mgg_msgs/msg/TourAward.msg`:

```
# The auctioneer's call for bids (call = true, nothing else set) or its award
# (tour-exploration design §3.4).
std_msgs/Header header             # stamp; frame_id: the auctioneer's planning frame
uint64 auction_id
int32 auctioneer_id
bool call
TourCluster[] clusters             # every cluster the bundles name
TourBundle[] bundles               # per robot, in tour order
TourCluster[] explored             # clusters dropped as explored by a robot of the group
int32[] released_robot_ids         # silent robots whose claims were released (§4)
```

In `ros2/src/mgg_msgs/CMakeLists.txt`, inside `rosidl_generate_interfaces`, after `  "msg/SemanticPolygon.msg"` add:

```cmake
  "msg/TourAward.msg"
  "msg/TourBid.msg"
  "msg/TourBundle.msg"
  "msg/TourCluster.msg"
```

- [ ] **Step 2: Write the failing core test**

Create `ros2/src/mgg_core/test/test_fleet_types.cpp`:

```cpp
// Tests for the fleet's exchange types (tour-exploration design §3.3, §3.4).

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mgg_core/fleet_types.h"

namespace {

using mgg::FleetCluster;
using mgg::TourAwardData;
using mgg::TourBidData;

FleetCluster cluster(mgg::ClusterId id, double x) {
  FleetCluster c;
  c.id = id;
  c.owner_robot_id = 1;
  c.position = Eigen::Vector3d(x, 0.0, 0.0);
  c.gain = 1000.0;
  return c;
}

TourBidData twoClusterBid() {
  TourBidData bid;
  bid.robot_id = 1;
  bid.pose = mgg::StateVec(0.0, 0.0, 0.0, 0.0);
  bid.clusters = {cluster(11, 3.0), cluster(12, 7.0)};
  bid.costs_from_pose = {3.0, 7.0};
  bid.costs_between = {0.0, 4.0, 4.0, 0.0};
  return bid;
}

TEST(FleetTypes, WellFormedRejectsInconsistentOrInvalidBids) {
  // Review Focus 1: a peer's bid arrives over a link nobody controls.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const TourBidData bid = twoClusterBid();
  EXPECT_TRUE(bid.wellFormed());
  TourBidData b = bid;
  b.costs_from_pose.push_back(1.0);
  EXPECT_FALSE(b.wellFormed());
  b = bid;
  b.costs_between.pop_back();
  EXPECT_FALSE(b.wellFormed());
  b = bid;
  b.costs_between[1] = nan;
  EXPECT_FALSE(b.wellFormed());
  b = bid;
  b.costs_from_pose[0] = -1.0;
  EXPECT_FALSE(b.wellFormed());
  b = bid;
  b.pose[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(b.wellFormed());
  b = bid;
  b.clusters[0].id = mgg::kNoCluster;
  EXPECT_FALSE(b.wellFormed());
  b = bid;
  b.clusters[1].position.x() = nan;
  EXPECT_FALSE(b.wellFormed());
  // Unreachable is a cost like any other.
  b = bid;
  b.costs_between[1] = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(b.wellFormed());
  // A robot with nothing to report still bids.
  EXPECT_TRUE(TourBidData{}.wellFormed());
}

TEST(FleetTypes, CostBetweenReadsTheRowMajorMatrix) {
  TourBidData bid = twoClusterBid();
  bid.costs_between = {0.0, 4.0, 5.0, 0.0};
  EXPECT_DOUBLE_EQ(bid.costBetween(0, 1), 4.0);
  EXPECT_DOUBLE_EQ(bid.costBetween(1, 0), 5.0);
}

TEST(FleetTypes, AnAwardFindsBundlesAndClustersById) {
  TourAwardData award;
  award.clusters = {cluster(11, 3.0), cluster(21, 9.0)};
  award.bundles = {{1, {11}, 0.0}, {2, {21}, 4.5}};
  ASSERT_NE(award.bundleOf(2), nullptr);
  EXPECT_DOUBLE_EQ(award.bundleOf(2)->silent_s, 4.5);
  EXPECT_EQ(award.bundleOf(3), nullptr);
  ASSERT_NE(award.cluster(21), nullptr);
  EXPECT_DOUBLE_EQ(award.cluster(21)->position.x(), 9.0);
  EXPECT_EQ(award.cluster(99), nullptr);
}

}  // namespace
```

Register it in `ros2/src/mgg_core/CMakeLists.txt` after the `test_tour_planner` lines:

```cmake
  ament_add_gtest(test_fleet_types test/test_fleet_types.cpp)
  target_link_libraries(test_fleet_types ${PROJECT_NAME})
```

- [ ] **Step 3: Run it to verify it fails**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_fleet_types '*'`
Expected: FAIL at compile time with `mgg_core/fleet_types.h: No such file or directory`.

- [ ] **Step 4: Write the exchange types**

Create `ros2/src/mgg_core/include/mgg_core/fleet_types.h`:

```cpp
// What robots exchange for fleet frontier assignment (tour-exploration
// design §3): bids, awards and the clusters they name, as plain structs.
// mgg_ros converts them to and from mgg_msgs/TourBid and TourAward at the
// frame boundary, as it does the roadmap exchange (graph_merge.h), so the
// auction runs in unit tests without ROS.
//
// Positions are in the holder's planning frame: a received bid or award has
// been placed with the neighbour transform before it gets here. Costs are
// frame-free.

#ifndef MGG_CORE_FLEET_TYPES_H_
#define MGG_CORE_FLEET_TYPES_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/frontier_clusters.h"
#include "mgg_core/types.h"

namespace mgg {

/// A bid listing more clusters than this is refused (wellFormed): its cost
/// matrix alone would be eight megabytes.
inline constexpr std::size_t kMaxBidClusters = 1024;

/// A frontier cluster as bids and awards carry it.
struct FleetCluster {
  ClusterId id = kNoCluster;
  int owner_robot_id = 0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double gain = 0.0;
};

/// One robot's bid (§3.3).
struct TourBidData {
  int robot_id = 0;
  std::uint64_t seq = 0;
  double stamp_s = 0.0;
  /// The auction it answers; 0 for a periodic bid.
  std::uint64_t auction_id = 0;
  StateVec pose = StateVec::Zero();
  /// Every frontier cluster the bidder knows.
  std::vector<FleetCluster> clusters;
  /// Per cluster, from the bidder's pose; kUnreachableCost when unreachable.
  std::vector<double> costs_from_pose;
  /// clusters x clusters, row-major.
  std::vector<double> costs_between;
  ClusterId current_target = kNoCluster;
  double claim_stamp_s = 0.0;
  std::vector<ClusterId> bundle;
  /// Awarded clusters the bidder's roadmap shows explored.
  std::vector<ClusterId> explored;
  /// The bidder's bundle is done and it asks for an auction (§3.2, §3.5).
  bool request_auction = false;

  double costBetween(std::size_t i, std::size_t j) const {
    return costs_between[i * clusters.size() + j];
  }
  /// At most kMaxBidClusters clusters, cost arrays sized to them, a finite
  /// pose, finite positions and non-zero IDs; costs may be +inf, never NaN
  /// or negative.
  bool wellFormed() const;
};

/// One robot's awarded clusters, in tour order (§3.4).
struct RobotBundle {
  int robot_id = 0;
  std::vector<ClusterId> clusters;
  /// How long the auctioneer has not heard this robot; 0 when it bid on
  /// time.
  double silent_s = 0.0;
};

/// An auction call (call = true, nothing else set) or an award (§3.4).
struct TourAwardData {
  std::uint64_t auction_id = 0;
  int auctioneer_id = 0;
  double stamp_s = 0.0;
  bool call = false;
  /// Every cluster the bundles name.
  std::vector<FleetCluster> clusters;
  std::vector<RobotBundle> bundles;
  /// Clusters dropped as explored by a robot of the group.
  std::vector<FleetCluster> explored;
  /// Silent robots whose claims were released (§4).
  std::vector<int> released_robot_ids;

  const RobotBundle* bundleOf(int robot_id) const;
  const FleetCluster* cluster(ClusterId id) const;
};

/// Whether the holder's own map or roadmap shows a position explored.
using ExploredFn = std::function<bool(const Eigen::Vector3d& position)>;
/// The auctioneer's estimate of a cost a bid does not give, on its merged
/// roadmap (§3.3); kUnreachableCost when it has none.
using CostEstimateFn =
    std::function<double(const Eigen::Vector3d& from, const Eigen::Vector3d& to)>;

}  // namespace mgg

#endif  // MGG_CORE_FLEET_TYPES_H_
```

Create `ros2/src/mgg_core/src/fleet_types.cpp`:

```cpp
#include "mgg_core/fleet_types.h"

#include <algorithm>
#include <cmath>

namespace mgg {

bool TourBidData::wellFormed() const {
  const std::size_t n = clusters.size();
  if (n > kMaxBidClusters) return false;
  if (costs_from_pose.size() != n || costs_between.size() != n * n) {
    return false;
  }
  if (!pose.allFinite()) return false;
  for (const FleetCluster& cluster : clusters) {
    if (cluster.id == kNoCluster || !cluster.position.allFinite()) {
      return false;
    }
  }
  const auto valid = [](double cost) {
    return !std::isnan(cost) && cost >= 0.0;
  };
  return std::all_of(costs_from_pose.begin(), costs_from_pose.end(), valid) &&
         std::all_of(costs_between.begin(), costs_between.end(), valid);
}

const RobotBundle* TourAwardData::bundleOf(int robot_id) const {
  for (const RobotBundle& bundle : bundles) {
    if (bundle.robot_id == robot_id) return &bundle;
  }
  return nullptr;
}

const FleetCluster* TourAwardData::cluster(ClusterId id) const {
  for (const FleetCluster& c : clusters) {
    if (c.id == id) return &c;
  }
  return nullptr;
}

}  // namespace mgg
```

Add the source to `add_library` in `ros2/src/mgg_core/CMakeLists.txt`, after `  src/tour_planner.cpp`:

```cmake
  src/fleet_types.cpp
```

- [ ] **Step 5: Run the core test to verify it passes**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_fleet_types '*'`
Expected: `[  PASSED  ] 3 tests.`

- [ ] **Step 6: Write the failing conversion test**

Create `ros2/src/mgg_ros/test/test_fleet_conversions.cpp`:

```cpp
// Round-trip tests for the fleet messages at the frame boundary
// (tour-exploration design §3.3, §3.4): positions move into the receiver's
// frame, costs do not.

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mgg_ros/fleet_conversions.h"

namespace {

using mgg::FleetCluster;
using mgg::TourAwardData;
using mgg::TourBidData;

FleetCluster cluster(mgg::ClusterId id, int owner, double x, double y) {
  FleetCluster c;
  c.id = id;
  c.owner_robot_id = owner;
  c.position = Eigen::Vector3d(x, y, 0.5);
  c.gain = 750.0;
  return c;
}

TourBidData sampleBid() {
  TourBidData bid;
  bid.robot_id = 2;
  bid.seq = 17;
  bid.stamp_s = 123.25;
  bid.auction_id = 9;
  bid.pose = mgg::StateVec(1.0, 0.0, 0.3, 0.0);
  bid.clusters = {cluster(21, 2, 2.0, 0.0), cluster(22, 3, 4.0, 1.0)};
  bid.costs_from_pose = {1.0, std::numeric_limits<double>::infinity()};
  bid.costs_between = {0.0, 2.5, 2.5, 0.0};
  bid.current_target = 21;
  bid.claim_stamp_s = 100.5;
  bid.bundle = {21};
  bid.explored = {31};
  bid.request_auction = true;
  return bid;
}

TEST(FleetConversions, ABidRoundTripsInItsOwnFrame) {
  const TourBidData in = sampleBid();
  const auto msg = mgg_ros::toTourBidMsg(in, "robot_1/odom");
  EXPECT_EQ(msg.header.frame_id, "robot_1/odom");
  const TourBidData out =
      mgg_ros::fromTourBidMsg(msg, Eigen::Isometry3d::Identity());
  EXPECT_EQ(out.robot_id, 2);
  EXPECT_EQ(out.seq, 17u);
  EXPECT_NEAR(out.stamp_s, 123.25, 1e-9);
  EXPECT_EQ(out.auction_id, 9u);
  EXPECT_TRUE(out.pose.isApprox(in.pose, 1e-12));
  ASSERT_EQ(out.clusters.size(), 2u);
  EXPECT_EQ(out.clusters[1].id, 22u);
  EXPECT_EQ(out.clusters[1].owner_robot_id, 3);
  EXPECT_DOUBLE_EQ(out.clusters[1].gain, 750.0);
  EXPECT_EQ(out.costs_from_pose[1], std::numeric_limits<double>::infinity());
  EXPECT_EQ(out.costs_between, in.costs_between);
  EXPECT_EQ(out.current_target, 21u);
  EXPECT_NEAR(out.claim_stamp_s, 100.5, 1e-9);
  EXPECT_EQ(out.bundle, in.bundle);
  EXPECT_EQ(out.explored, in.explored);
  EXPECT_TRUE(out.request_auction);
  EXPECT_TRUE(out.wellFormed());
}

TEST(FleetConversions, ABidIsPlacedInTheReceiversFrame) {
  // The sender's frame is 5 m east of ours and turned a quarter left.
  Eigen::Isometry3d t_ours_theirs = Eigen::Isometry3d::Identity();
  t_ours_theirs.linear() =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  t_ours_theirs.translation() = Eigen::Vector3d(5.0, 0.0, 0.0);
  const TourBidData out = mgg_ros::fromTourBidMsg(
      mgg_ros::toTourBidMsg(sampleBid(), "robot_1/odom"), t_ours_theirs);
  EXPECT_TRUE(out.pose.head<3>().isApprox(Eigen::Vector3d(5.0, 1.0, 0.3)));
  EXPECT_NEAR(out.pose[3], M_PI / 2.0, 1e-9);
  EXPECT_TRUE(out.clusters[0].position.isApprox(Eigen::Vector3d(5.0, 2.0, 0.5)));
  EXPECT_TRUE(out.clusters[1].position.isApprox(Eigen::Vector3d(4.0, 4.0, 0.5)));
  // Costs are lengths, the same in every frame.
  EXPECT_EQ(out.costs_between, sampleBid().costs_between);
}

TEST(FleetConversions, AnAwardRoundTripsWithBundlesExploredAndReleases) {
  TourAwardData in;
  in.auction_id = (std::uint64_t{1} << 40) | 3;
  in.auctioneer_id = 1;
  in.stamp_s = 42.5;
  in.clusters = {cluster(11, 1, 2.0, 0.0), cluster(21, 2, 9.0, 0.0)};
  in.bundles = {{1, {11}, 0.0}, {2, {21}, 7.5}};
  in.explored = {cluster(31, 3, -4.0, 0.0)};
  in.released_robot_ids = {4};
  Eigen::Isometry3d t_ours_theirs = Eigen::Isometry3d::Identity();
  t_ours_theirs.translation() = Eigen::Vector3d(0.0, 10.0, 0.0);
  const TourAwardData out = mgg_ros::fromTourAwardMsg(
      mgg_ros::toTourAwardMsg(in, "robot_0/odom"), t_ours_theirs);
  EXPECT_EQ(out.auction_id, in.auction_id);
  EXPECT_EQ(out.auctioneer_id, 1);
  EXPECT_NEAR(out.stamp_s, 42.5, 1e-9);
  EXPECT_FALSE(out.call);
  ASSERT_EQ(out.bundles.size(), 2u);
  EXPECT_EQ(out.bundles[1].robot_id, 2);
  EXPECT_EQ(out.bundles[1].clusters, std::vector<mgg::ClusterId>{21});
  EXPECT_DOUBLE_EQ(out.bundles[1].silent_s, 7.5);
  ASSERT_NE(out.cluster(21), nullptr);
  EXPECT_TRUE(out.cluster(21)->position.isApprox(Eigen::Vector3d(9.0, 10.0, 0.5)));
  ASSERT_EQ(out.explored.size(), 1u);
  EXPECT_TRUE(out.explored[0].position.isApprox(Eigen::Vector3d(-4.0, 10.0, 0.5)));
  EXPECT_EQ(out.released_robot_ids, std::vector<int>{4});

  TourAwardData call;
  call.auction_id = 5;
  call.auctioneer_id = 1;
  call.call = true;
  EXPECT_TRUE(mgg_ros::fromTourAwardMsg(mgg_ros::toTourAwardMsg(call, "f"),
                                        Eigen::Isometry3d::Identity())
                  .call);
}

TEST(FleetConversions, StampsSurviveTheRoundTrip) {
  const auto stamp = mgg_ros::stampFromSeconds(12.25);
  EXPECT_EQ(stamp.sec, 12);
  EXPECT_EQ(stamp.nanosec, 250000000u);
  EXPECT_NEAR(mgg_ros::stampSeconds(stamp), 12.25, 1e-9);
  // Not a time: the zero stamp.
  EXPECT_EQ(mgg_ros::stampFromSeconds(-1.0).sec, 0);
  EXPECT_EQ(mgg_ros::stampFromSeconds(std::nan("")).nanosec, 0u);
}

}  // namespace
```

Register it in `ros2/src/mgg_ros/CMakeLists.txt`, inside `if(BUILD_TESTING)`, after the three `test_conversions` lines:

```cmake
  ament_add_gtest(test_fleet_conversions test/test_fleet_conversions.cpp)
  target_link_libraries(test_fleet_conversions ${PROJECT_NAME})
  ament_target_dependencies(test_fleet_conversions rclcpp mgg_msgs)
```

- [ ] **Step 7: Run it to verify it fails**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_ros test_fleet_conversions '*'`
Expected: FAIL at compile time with `mgg_ros/fleet_conversions.h: No such file or directory`.

- [ ] **Step 8: Write the conversions**

Create `ros2/src/mgg_ros/include/mgg_ros/fleet_conversions.h`:

```cpp
// Translating the fleet messages (mgg_msgs/TourBid, TourAward) to and from
// mgg_core's exchange types (fleet_types.h). A received message is placed in
// this robot's planning frame with the transform its roadmap would merge
// with; positions move, costs do not.

#ifndef MGG_ROS_FLEET_CONVERSIONS_H_
#define MGG_ROS_FLEET_CONVERSIONS_H_

#include <string>

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <mgg_msgs/msg/tour_award.hpp>
#include <mgg_msgs/msg/tour_bid.hpp>

#include "mgg_core/fleet_types.h"

namespace mgg_ros {

double stampSeconds(const builtin_interfaces::msg::Time& stamp);
/// The zero stamp for a time that is not finite and positive.
builtin_interfaces::msg::Time stampFromSeconds(double seconds);

mgg::TourBidData fromTourBidMsg(const mgg_msgs::msg::TourBid& msg,
                                const Eigen::Isometry3d& t_ours_theirs);
mgg_msgs::msg::TourBid toTourBidMsg(const mgg::TourBidData& bid,
                                    const std::string& frame_id);
mgg::TourAwardData fromTourAwardMsg(const mgg_msgs::msg::TourAward& msg,
                                    const Eigen::Isometry3d& t_ours_theirs);
mgg_msgs::msg::TourAward toTourAwardMsg(const mgg::TourAwardData& award,
                                        const std::string& frame_id);

}  // namespace mgg_ros

#endif  // MGG_ROS_FLEET_CONVERSIONS_H_
```

Create `ros2/src/mgg_ros/src/fleet_conversions.cpp`:

```cpp
#include "mgg_ros/fleet_conversions.h"

#include <algorithm>
#include <cmath>

#include "mgg_ros/conversions.h"

namespace mgg_ros {
namespace {

mgg::FleetCluster fromClusterMsg(const mgg_msgs::msg::TourCluster& msg,
                                 const Eigen::Isometry3d& t_ours_theirs) {
  mgg::FleetCluster cluster;
  cluster.id = msg.id;
  cluster.owner_robot_id = msg.owner_robot_id;
  cluster.position = t_ours_theirs * Eigen::Vector3d(
                                         msg.position.x, msg.position.y,
                                         msg.position.z);
  cluster.gain = msg.gain;
  return cluster;
}

mgg_msgs::msg::TourCluster toClusterMsg(const mgg::FleetCluster& cluster) {
  mgg_msgs::msg::TourCluster msg;
  msg.id = cluster.id;
  msg.owner_robot_id = cluster.owner_robot_id;
  msg.position.x = cluster.position.x();
  msg.position.y = cluster.position.y();
  msg.position.z = cluster.position.z();
  msg.gain = cluster.gain;
  return msg;
}

std::vector<mgg::FleetCluster> fromClusterMsgs(
    const std::vector<mgg_msgs::msg::TourCluster>& msgs,
    const Eigen::Isometry3d& t_ours_theirs) {
  std::vector<mgg::FleetCluster> clusters;
  clusters.reserve(msgs.size());
  for (const auto& msg : msgs) {
    clusters.push_back(fromClusterMsg(msg, t_ours_theirs));
  }
  return clusters;
}

std::vector<mgg_msgs::msg::TourCluster> toClusterMsgs(
    const std::vector<mgg::FleetCluster>& clusters) {
  std::vector<mgg_msgs::msg::TourCluster> msgs;
  msgs.reserve(clusters.size());
  for (const auto& cluster : clusters) msgs.push_back(toClusterMsg(cluster));
  return msgs;
}

}  // namespace

double stampSeconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         1e-9 * static_cast<double>(stamp.nanosec);
}

builtin_interfaces::msg::Time stampFromSeconds(double seconds) {
  builtin_interfaces::msg::Time stamp;
  if (!std::isfinite(seconds) || seconds <= 0.0) return stamp;
  const double whole = std::floor(seconds);
  stamp.sec = static_cast<std::int32_t>(whole);
  stamp.nanosec = static_cast<std::uint32_t>(
      std::min(999999999.0, std::round((seconds - whole) * 1e9)));
  return stamp;
}

mgg::TourBidData fromTourBidMsg(const mgg_msgs::msg::TourBid& msg,
                                const Eigen::Isometry3d& t_ours_theirs) {
  mgg::TourBidData bid;
  bid.robot_id = msg.robot_id;
  bid.seq = msg.seq;
  bid.stamp_s = stampSeconds(msg.header.stamp);
  bid.auction_id = msg.auction_id;
  const mgg::StateVec theirs = fromPoseMsg(msg.pose);
  const Eigen::Vector3d position =
      t_ours_theirs * Eigen::Vector3d(theirs.head<3>());
  const Eigen::Matrix3d rotation = t_ours_theirs.linear();
  const double yaw = std::remainder(
      theirs[3] + std::atan2(rotation(1, 0), rotation(0, 0)), 2.0 * M_PI);
  bid.pose = mgg::StateVec(position.x(), position.y(), position.z(), yaw);
  bid.clusters = fromClusterMsgs(msg.clusters, t_ours_theirs);
  bid.costs_from_pose.assign(msg.costs_from_pose.begin(),
                             msg.costs_from_pose.end());
  bid.costs_between.assign(msg.costs_between.begin(), msg.costs_between.end());
  bid.current_target = msg.current_target;
  bid.claim_stamp_s = stampSeconds(msg.claim_stamp);
  bid.bundle.assign(msg.bundle.begin(), msg.bundle.end());
  bid.explored.assign(msg.explored.begin(), msg.explored.end());
  bid.request_auction = msg.request_auction;
  return bid;
}

mgg_msgs::msg::TourBid toTourBidMsg(const mgg::TourBidData& bid,
                                    const std::string& frame_id) {
  mgg_msgs::msg::TourBid msg;
  msg.header.stamp = stampFromSeconds(bid.stamp_s);
  msg.header.frame_id = frame_id;
  msg.robot_id = bid.robot_id;
  msg.seq = bid.seq;
  msg.auction_id = bid.auction_id;
  msg.pose = toPoseMsg(bid.pose);
  msg.clusters = toClusterMsgs(bid.clusters);
  msg.costs_from_pose.assign(bid.costs_from_pose.begin(),
                             bid.costs_from_pose.end());
  msg.costs_between.assign(bid.costs_between.begin(), bid.costs_between.end());
  msg.current_target = bid.current_target;
  msg.claim_stamp = stampFromSeconds(bid.claim_stamp_s);
  msg.bundle.assign(bid.bundle.begin(), bid.bundle.end());
  msg.explored.assign(bid.explored.begin(), bid.explored.end());
  msg.request_auction = bid.request_auction;
  return msg;
}

mgg::TourAwardData fromTourAwardMsg(const mgg_msgs::msg::TourAward& msg,
                                    const Eigen::Isometry3d& t_ours_theirs) {
  mgg::TourAwardData award;
  award.auction_id = msg.auction_id;
  award.auctioneer_id = msg.auctioneer_id;
  award.stamp_s = stampSeconds(msg.header.stamp);
  award.call = msg.call;
  award.clusters = fromClusterMsgs(msg.clusters, t_ours_theirs);
  for (const auto& bundle : msg.bundles) {
    award.bundles.push_back(
        {bundle.robot_id,
         std::vector<mgg::ClusterId>(bundle.clusters.begin(),
                                     bundle.clusters.end()),
         bundle.silent_s});
  }
  award.explored = fromClusterMsgs(msg.explored, t_ours_theirs);
  award.released_robot_ids.assign(msg.released_robot_ids.begin(),
                                  msg.released_robot_ids.end());
  return award;
}

mgg_msgs::msg::TourAward toTourAwardMsg(const mgg::TourAwardData& award,
                                        const std::string& frame_id) {
  mgg_msgs::msg::TourAward msg;
  msg.header.stamp = stampFromSeconds(award.stamp_s);
  msg.header.frame_id = frame_id;
  msg.auction_id = award.auction_id;
  msg.auctioneer_id = award.auctioneer_id;
  msg.call = award.call;
  msg.clusters = toClusterMsgs(award.clusters);
  for (const mgg::RobotBundle& bundle : award.bundles) {
    mgg_msgs::msg::TourBundle out;
    out.robot_id = bundle.robot_id;
    out.clusters.assign(bundle.clusters.begin(), bundle.clusters.end());
    out.silent_s = bundle.silent_s;
    msg.bundles.push_back(out);
  }
  msg.explored = toClusterMsgs(award.explored);
  msg.released_robot_ids.assign(award.released_robot_ids.begin(),
                                award.released_robot_ids.end());
  return msg;
}

}  // namespace mgg_ros
```

In `ros2/src/mgg_ros/CMakeLists.txt`, add the source to `add_library(${PROJECT_NAME} ...)` on a new line after `  src/conversions.cpp`:

```cmake
  src/fleet_conversions.cpp
```

- [ ] **Step 9: Run the conversion test to verify it passes**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_ros test_fleet_conversions '*'`
Expected: `[  PASSED  ] 4 tests.`

- [ ] **Step 10: Run both gates**

Run: `/tmp/mgg-tour/tuf-run.sh off test` then `/tmp/mgg-tour/tuf-run.sh on test`
Expected: both end with `0 errors, 0 failures`.

- [ ] **Step 11: Commit**

```bash
git add ros2/src/mgg_msgs/msg/TourCluster.msg ros2/src/mgg_msgs/msg/TourBundle.msg \
        ros2/src/mgg_msgs/msg/TourBid.msg ros2/src/mgg_msgs/msg/TourAward.msg \
        ros2/src/mgg_msgs/CMakeLists.txt \
        ros2/src/mgg_core/include/mgg_core/fleet_types.h \
        ros2/src/mgg_core/src/fleet_types.cpp \
        ros2/src/mgg_core/test/test_fleet_types.cpp \
        ros2/src/mgg_core/CMakeLists.txt \
        ros2/src/mgg_ros/include/mgg_ros/fleet_conversions.h \
        ros2/src/mgg_ros/src/fleet_conversions.cpp \
        ros2/src/mgg_ros/test/test_fleet_conversions.cpp \
        ros2/src/mgg_ros/CMakeLists.txt
git commit -m "Add the fleet's bid and award messages and their conversions"
```

---
### Task 8: The cluster pool and the sequential auction

**Files:**
- Create: `ros2/src/mgg_core/include/mgg_core/fleet_auction.h`
- Create: `ros2/src/mgg_core/src/fleet_auction.cpp`
- Create: `ros2/src/mgg_core/test/test_fleet_auction.cpp`
- Modify: `ros2/src/mgg_core/CMakeLists.txt` (library source, test)

**Interfaces:**
- Consumes: `mgg::TourBidData`, `mgg::FleetCluster`, `mgg::ExploredFn`, `mgg::CostEstimateFn` (Task 7); `mgg::solveOpenTour`, `mgg::openTourCost`, `mgg::cheapestInsertion`, `mgg::kUnreachableCost` (Task 3).
- Produces:
  - `struct mgg::ClusterPool { std::vector<FleetCluster> clusters; std::vector<std::vector<ClusterId>> member_ids; std::vector<std::vector<int>> bid_to_pool; std::vector<FleetCluster> dropped_explored; int indexOf(ClusterId) const; int indexNear(const Eigen::Vector3d&, double radius_m) const; };`
  - `ClusterPool mgg::buildClusterPool(const std::vector<TourBidData>& bids, const std::vector<FleetCluster>& held, double merge_radius_m, const ExploredFn& explored);`
  - `struct mgg::AuctionBidder { int robot_id; std::vector<double> from_pose; std::vector<std::vector<double>> between; int current_target; };`
  - `AuctionBidder mgg::bidderCosts(const TourBidData& bid, const std::vector<int>& bid_to_pool, const ClusterPool& pool, const CostEstimateFn& estimate);`
  - `struct mgg::AuctionResult { std::map<int, std::vector<int>> bundles; std::vector<int> unassigned; };` (pool indices, each bundle in tour order; every bidder has an entry)
  - `AuctionResult mgg::runSequentialAuction(const std::vector<AuctionBidder>& bidders, const std::vector<bool>& fixed, double commit_margin, double balance_weight);`

The sequential auction is a heuristic (spec §8: "the sequential auction heuristic is deliberate"). The spec's test "matches a brute-force optimum on small cases" is pinned on scenes where the greedy order is optimal, and brute force (all assignments, each bundle's exact open tour) confirms it.

- [ ] **Step 1: Write the failing test**

Create `ros2/src/mgg_core/test/test_fleet_auction.cpp`:

```cpp
// Tests for the auctioneer's cluster pool and sequential single-item
// auction (tour-exploration design §3.3, §3.4).

#include <cmath>
#include <limits>
#include <map>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/fleet_auction.h"
#include "mgg_core/tour_solver.h"

namespace {

using mgg::AuctionBidder;
using mgg::AuctionResult;
using mgg::ClusterPool;
using mgg::FleetCluster;
using mgg::TourBidData;

FleetCluster cluster(mgg::ClusterId id, int owner, double x, double y = 0.0) {
  FleetCluster c;
  c.id = id;
  c.owner_robot_id = owner;
  c.position = Eigen::Vector3d(x, y, 0.0);
  c.gain = 1000.0;
  return c;
}

double euclid(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return (a - b).norm();
}

/// A bid from `robot` at (x, 0) listing `clusters` at straight-line costs.
TourBidData bidAt(int robot, double x, const std::vector<FleetCluster>& clusters) {
  TourBidData bid;
  bid.robot_id = robot;
  bid.pose = mgg::StateVec(x, 0.0, 0.0, 0.0);
  bid.clusters = clusters;
  for (const FleetCluster& c : clusters) {
    bid.costs_from_pose.push_back(euclid(bid.pose.head<3>(), c.position));
  }
  for (const FleetCluster& a : clusters) {
    for (const FleetCluster& b : clusters) {
      bid.costs_between.push_back(euclid(a.position, b.position));
    }
  }
  return bid;
}

/// A bidder at `at` on a line of clusters at `xs`.
AuctionBidder onLine(int robot, double at, const std::vector<double>& xs) {
  AuctionBidder b;
  b.robot_id = robot;
  for (const double x : xs) b.from_pose.push_back(std::abs(x - at));
  for (const double x : xs) {
    b.between.emplace_back();
    for (const double y : xs) b.between.back().push_back(std::abs(x - y));
  }
  return b;
}

/// A place along one of two corridors that meet at the origin.
struct CorridorPoint {
  char corridor;
  double along;
};

double corridorDistance(const CorridorPoint& p, const CorridorPoint& q) {
  return p.corridor == q.corridor ? std::abs(p.along - q.along)
                                  : p.along + q.along;
}

/// A bidder standing where the corridors meet.
AuctionBidder atJunction(int robot, const std::vector<CorridorPoint>& points) {
  AuctionBidder b;
  b.robot_id = robot;
  for (const CorridorPoint& p : points) b.from_pose.push_back(p.along);
  for (const CorridorPoint& p : points) {
    b.between.emplace_back();
    for (const CorridorPoint& q : points) {
      b.between.back().push_back(corridorDistance(p, q));
    }
  }
  return b;
}

/// The best open tour over `indices` for `b`.
double bundleCost(const AuctionBidder& b, const std::vector<int>& indices) {
  std::vector<double> from;
  std::vector<std::vector<double>> between;
  for (const int i : indices) {
    from.push_back(b.from_pose[i]);
    between.emplace_back();
    for (const int j : indices) between.back().push_back(b.between[i][j]);
  }
  return mgg::solveOpenTour(from, between).cost;
}

/// Every assignment of `n` clusters to the bidders, each bundle toured
/// optimally: the least total.
double bruteForceMinSum(const std::vector<AuctionBidder>& bidders,
                        std::size_t n) {
  const std::size_t m = bidders.size();
  std::size_t assignments = 1;
  for (std::size_t c = 0; c < n; ++c) assignments *= m;
  double best = mgg::kUnreachableCost;
  for (std::size_t code = 0; code < assignments; ++code) {
    std::vector<std::vector<int>> parts(m);
    std::size_t rest = code;
    for (std::size_t c = 0; c < n; ++c) {
      parts[rest % m].push_back(static_cast<int>(c));
      rest /= m;
    }
    double total = 0.0;
    for (std::size_t r = 0; r < m; ++r) total += bundleCost(bidders[r], parts[r]);
    best = std::min(best, total);
  }
  return best;
}

double auctionCost(const std::vector<AuctionBidder>& bidders,
                   const AuctionResult& result) {
  double total = 0.0;
  for (const AuctionBidder& b : bidders) {
    total += mgg::openTourCost(result.bundles.at(b.robot_id), b.from_pose,
                               b.between);
  }
  return total;
}

TEST(ClusterPool, MergesClustersWithinTheRadiusUnderTheOwnersName) {
  // Robot 2 lists robot 1's frontier, merged into its roadmap, under its own
  // quantization: one place, two names.
  const TourBidData a =
      bidAt(1, 0.0, {cluster(100, 1, 10.0), cluster(101, 1, 30.0)});
  const TourBidData b =
      bidAt(2, 20.0, {cluster(200, 1, 10.5), cluster(201, 2, 25.0)});
  const ClusterPool pool = mgg::buildClusterPool({a, b}, {}, 2.0, nullptr);
  ASSERT_EQ(pool.clusters.size(), 3u);
  const int p = pool.indexOf(200);
  ASSERT_GE(p, 0);
  EXPECT_EQ(pool.clusters[p].id, 100u);
  EXPECT_EQ(pool.indexOf(100), p);
  EXPECT_EQ(pool.bid_to_pool[0][0], p);
  EXPECT_EQ(pool.bid_to_pool[1][0], p);
  EXPECT_NE(pool.bid_to_pool[1][1], p);
  EXPECT_EQ(pool.indexNear(Eigen::Vector3d(29.0, 0.0, 0.0), 2.0),
            pool.indexOf(101));
  EXPECT_EQ(pool.indexNear(Eigen::Vector3d(50.0, 0.0, 0.0), 2.0), -1);
}

TEST(ClusterPool, DropsClustersExploredByAnyRobot) {
  TourBidData a =
      bidAt(1, 0.0, {cluster(100, 1, 10.0), cluster(300, 3, -10.0)});
  TourBidData b = bidAt(2, 20.0, {cluster(201, 2, 25.0)});
  b.explored = {100};  // robot 2's roadmap shows robot 1's cluster explored
  // The auctioneer's own roadmap shows everything behind it explored.
  const auto explored_here = [](const Eigen::Vector3d& p) {
    return p.x() < 0.0;
  };
  const ClusterPool pool =
      mgg::buildClusterPool({a, b}, {}, 2.0, explored_here);
  ASSERT_EQ(pool.clusters.size(), 1u);
  EXPECT_EQ(pool.clusters[0].id, 201u);
  EXPECT_EQ(pool.dropped_explored.size(), 2u);
  EXPECT_EQ(pool.bid_to_pool[0], (std::vector<int>{-1, -1}));
  EXPECT_EQ(pool.bid_to_pool[1], (std::vector<int>{0}));
}

TEST(ClusterPool, HoldsClaimedClustersNoBidNames) {
  const ClusterPool pool = mgg::buildClusterPool(
      {bidAt(1, 0.0, {cluster(100, 1, 10.0)})}, {cluster(400, 4, 40.0)}, 2.0,
      nullptr);
  EXPECT_EQ(pool.clusters.size(), 2u);
  EXPECT_GE(pool.indexOf(400), 0);
  EXPECT_EQ(pool.bid_to_pool.size(), 1u);
}

TEST(ClusterPool, BidderCostsComeFromTheBidOrTheEstimate) {
  const TourBidData a = bidAt(1, 0.0, {cluster(100, 1, 10.0)});
  const TourBidData b = bidAt(2, 20.0, {cluster(201, 2, 25.0)});
  const ClusterPool pool = mgg::buildClusterPool({a, b}, {}, 2.0, nullptr);
  const int p100 = pool.indexOf(100);
  const int p201 = pool.indexOf(201);
  const auto doubled = [](const Eigen::Vector3d& from,
                          const Eigen::Vector3d& to) {
    return 2.0 * (from - to).norm();
  };
  const AuctionBidder bidder =
      mgg::bidderCosts(a, pool.bid_to_pool[0], pool, doubled);
  EXPECT_DOUBLE_EQ(bidder.from_pose[p100], 10.0);  // from its bid
  EXPECT_DOUBLE_EQ(bidder.from_pose[p201], 50.0);  // estimated
  EXPECT_DOUBLE_EQ(bidder.between[p100][p201], 30.0);
  EXPECT_DOUBLE_EQ(bidder.between[p201][p100], 30.0);
  EXPECT_DOUBLE_EQ(bidder.between[p100][p100], 0.0);
  EXPECT_EQ(bidder.current_target, -1);
  // Without an estimate, what the bid does not give is unreachable.
  EXPECT_EQ(mgg::bidderCosts(a, pool.bid_to_pool[0], pool, nullptr)
                .from_pose[p201],
            mgg::kUnreachableCost);
  TourBidData targeting = a;
  targeting.current_target = 100;
  EXPECT_EQ(mgg::bidderCosts(targeting, pool.bid_to_pool[0], pool, doubled)
                .current_target,
            p100);
}

TEST(SequentialAuction, MatchesTheBruteForceOptimumOnLines) {
  {
    const std::vector<double> xs{1.0, 2.0, 3.0, 7.0, 8.0, 9.0};
    const std::vector<AuctionBidder> bidders{onLine(1, 0.0, xs),
                                             onLine(2, 10.0, xs)};
    const AuctionResult result = mgg::runSequentialAuction(
        bidders, std::vector<bool>(xs.size(), false), 0.2, 0.0);
    EXPECT_NEAR(auctionCost(bidders, result),
                bruteForceMinSum(bidders, xs.size()), 1e-9);
    EXPECT_NEAR(auctionCost(bidders, result), 6.0, 1e-9);
    EXPECT_TRUE(result.unassigned.empty());
  }
  {
    const std::vector<double> xs{2.0, 4.0, 12.0, 14.0, 18.0};
    const std::vector<AuctionBidder> bidders{
        onLine(1, 0.0, xs), onLine(2, 10.0, xs), onLine(3, 20.0, xs)};
    const AuctionResult result = mgg::runSequentialAuction(
        bidders, std::vector<bool>(xs.size(), false), 0.2, 0.0);
    EXPECT_NEAR(auctionCost(bidders, result),
                bruteForceMinSum(bidders, xs.size()), 1e-9);
    EXPECT_NEAR(auctionCost(bidders, result), 10.0, 1e-9);
  }
}

TEST(SequentialAuction, TwoCorridorsGoOnePerRobot) {
  // Both robots where two corridors meet, five clusters down each.
  std::vector<CorridorPoint> points;
  for (const double along : {2.0, 4.0, 6.0, 8.0, 10.0}) points.push_back({'A', along});
  for (const double along : {2.0, 4.0, 6.0, 8.0, 10.0}) points.push_back({'B', along});
  const std::vector<AuctionBidder> bidders{atJunction(1, points),
                                           atJunction(2, points)};
  const AuctionResult result = mgg::runSequentialAuction(
      bidders, std::vector<bool>(points.size(), false), 0.2, 0.3);
  const std::vector<int>& one = result.bundles.at(1);
  const std::vector<int>& two = result.bundles.at(2);
  ASSERT_EQ(one.size(), 5u);
  ASSERT_EQ(two.size(), 5u);
  for (const int i : one) EXPECT_EQ(points[i].corridor, points[one.front()].corridor);
  for (const int i : two) EXPECT_EQ(points[i].corridor, points[two.front()].corridor);
  EXPECT_NE(points[one.front()].corridor, points[two.front()].corridor);
  // Each tours its corridor outward.
  EXPECT_DOUBLE_EQ(points[one.front()].along, 2.0);
  EXPECT_DOUBLE_EQ(points[one.back()].along, 10.0);
}

TEST(SequentialAuction, TheBalancePenaltyHandsAnEqualClusterToTheLighterBundle) {
  // Robot 1 keeps a target 10 m away; cluster 1 lies 3 m past it. Robot 2
  // is 3 m from cluster 1. Equal marginal costs: the tie goes to robot 1
  // without the balance penalty, to robot 2 with it.
  AuctionBidder heavy;
  heavy.robot_id = 1;
  heavy.from_pose = {10.0, 13.0};
  heavy.between = {{0.0, 3.0}, {3.0, 0.0}};
  heavy.current_target = 0;
  AuctionBidder light;
  light.robot_id = 2;
  light.from_pose = {30.0, 3.0};
  light.between = {{0.0, 3.0}, {3.0, 0.0}};
  const AuctionResult none =
      mgg::runSequentialAuction({heavy, light}, {false, false}, 0.2, 0.0);
  EXPECT_EQ(none.bundles.at(1), (std::vector<int>{0, 1}));
  EXPECT_TRUE(none.bundles.at(2).empty());
  const AuctionResult balanced =
      mgg::runSequentialAuction({heavy, light}, {false, false}, 0.2, 0.3);
  EXPECT_EQ(balanced.bundles.at(1), std::vector<int>{0});
  EXPECT_EQ(balanced.bundles.at(2), std::vector<int>{1});
}

TEST(SequentialAuction, CommitmentKeepsATargetUnlessAnotherRobotIsMuchCloser) {
  AuctionBidder holder;
  holder.robot_id = 1;
  holder.from_pose = {5.0};
  holder.between = {{0.0}};
  holder.current_target = 0;
  AuctionBidder rival;
  rival.robot_id = 2;
  rival.from_pose = {4.5};  // 10 % closer: within the 20 % margin
  rival.between = {{0.0}};
  const AuctionResult kept =
      mgg::runSequentialAuction({holder, rival}, {false}, 0.2, 0.0);
  EXPECT_EQ(kept.bundles.at(1), std::vector<int>{0});
  EXPECT_TRUE(kept.bundles.at(2).empty());
  rival.from_pose = {3.9};  // 22 % closer
  const AuctionResult lost =
      mgg::runSequentialAuction({holder, rival}, {false}, 0.2, 0.0);
  EXPECT_TRUE(lost.bundles.at(1).empty());
  EXPECT_EQ(lost.bundles.at(2), std::vector<int>{0});
}

TEST(SequentialAuction, AContestedTargetStaysWithTheCloserRobot) {
  // After a reconnection both robots were heading for the same cluster.
  AuctionBidder a;
  a.robot_id = 1;
  a.from_pose = {6.0};
  a.between = {{0.0}};
  a.current_target = 0;
  AuctionBidder b = a;
  b.robot_id = 2;
  b.from_pose = {5.0};
  const AuctionResult result =
      mgg::runSequentialAuction({a, b}, {false}, 0.2, 0.0);
  EXPECT_TRUE(result.bundles.at(1).empty());
  EXPECT_EQ(result.bundles.at(2), std::vector<int>{0});
}

TEST(SequentialAuction, FixedAndUnreachableClustersAreNotAwarded) {
  AuctionBidder b = onLine(1, 0.0, {1.0, 2.0, 3.0});
  b.from_pose[2] = mgg::kUnreachableCost;
  for (int i = 0; i < 3; ++i) {
    if (i != 2) {
      b.between[i][2] = mgg::kUnreachableCost;
      b.between[2][i] = mgg::kUnreachableCost;
    }
  }
  const AuctionResult result =
      mgg::runSequentialAuction({b}, {true, false, false}, 0.2, 0.3);
  EXPECT_EQ(result.bundles.at(1), std::vector<int>{1});
  EXPECT_EQ(result.unassigned, std::vector<int>{2});
}

}  // namespace
```

Register it in `ros2/src/mgg_core/CMakeLists.txt` after the `test_fleet_types` lines:

```cmake
  ament_add_gtest(test_fleet_auction test/test_fleet_auction.cpp)
  target_link_libraries(test_fleet_auction ${PROJECT_NAME})
```

- [ ] **Step 2: Run it to verify it fails**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_fleet_auction '*'`
Expected: FAIL at compile time with `mgg_core/fleet_auction.h: No such file or directory`.

- [ ] **Step 3: Write the header**

Create `ros2/src/mgg_core/include/mgg_core/fleet_auction.h`:

```cpp
// The frontier auction's pool and award (tour-exploration design §3.3,
// §3.4), computed by the group's auctioneer from one round of bids.
//
// Pool: every bid's clusters and the clusters held by robots that did not
// bid, merged when closer than fleet.cluster_merge_radius_m (robots name one
// place differently: a robot names a peer's merged frontier by its own
// quantization), under the owner's own name when the owner bid it. A cluster
// any bidder reports explored, or at which the auctioneer's own roadmap
// shows explored space, is dropped.
//
// Award: each bidder keeps its current target unless another bidder's cost
// to it is lower by more than tour.commit_margin; the rest go by sequential
// single-item auction. Each round every bidder bids for every open cluster
// its marginal tour cost (cheapest insertion into its bundle's tour) plus
// fleet.balance_weight times its bundle's tour cost; the lowest bid wins,
// ties to the lower robot ID. Held clusters are fixed: nobody gets them.

#ifndef MGG_CORE_FLEET_AUCTION_H_
#define MGG_CORE_FLEET_AUCTION_H_

#include <map>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/fleet_types.h"

namespace mgg {

struct ClusterPool {
  std::vector<FleetCluster> clusters;
  /// Every ID merged into each pool cluster.
  std::vector<std::vector<ClusterId>> member_ids;
  /// Per bid, in the order given: the pool index of each of its clusters, or
  /// -1 when it was dropped as explored.
  std::vector<std::vector<int>> bid_to_pool;
  /// The clusters dropped as explored.
  std::vector<FleetCluster> dropped_explored;

  /// The pool cluster `id` was merged into, or -1.
  int indexOf(ClusterId id) const;
  /// The pool cluster nearest `position` within `radius_m`, or -1.
  int indexNear(const Eigen::Vector3d& position, double radius_m) const;
};

/// §3.3 pool construction from `bids` and the `held` clusters of robots that
/// did not bid. `explored` is the auctioneer's own check; null checks
/// nothing.
ClusterPool buildClusterPool(const std::vector<TourBidData>& bids,
                             const std::vector<FleetCluster>& held,
                             double merge_radius_m,
                             const ExploredFn& explored);

/// One bidder's costs over the pool.
struct AuctionBidder {
  int robot_id = 0;
  /// Per pool cluster, from the bidder's pose.
  std::vector<double> from_pose;
  /// Pool x pool, symmetric.
  std::vector<std::vector<double>> between;
  /// The pool index of its current target, or -1.
  int current_target = -1;
};

/// The bid's costs over the pool: its own where it lists the cluster (the
/// least of its entries merged into one), otherwise `estimate` on the
/// auctioneer's roadmap (kUnreachableCost without one).
AuctionBidder bidderCosts(const TourBidData& bid,
                          const std::vector<int>& bid_to_pool,
                          const ClusterPool& pool,
                          const CostEstimateFn& estimate);

struct AuctionResult {
  /// Pool indices each bidder won, in its tour order; every bidder has an
  /// entry, and a kept target stays first.
  std::map<int, std::vector<int>> bundles;
  /// Pool indices neither fixed nor reachable by any bidder.
  std::vector<int> unassigned;
};

/// §3.4 steps 1 and 2 over the pool clusters not `fixed`.
AuctionResult runSequentialAuction(const std::vector<AuctionBidder>& bidders,
                                   const std::vector<bool>& fixed,
                                   double commit_margin,
                                   double balance_weight);

}  // namespace mgg

#endif  // MGG_CORE_FLEET_AUCTION_H_
```

- [ ] **Step 4: Write the implementation**

Create `ros2/src/mgg_core/src/fleet_auction.cpp`:

```cpp
#include "mgg_core/fleet_auction.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "mgg_core/tour_solver.h"

namespace mgg {

int ClusterPool::indexOf(ClusterId id) const {
  for (std::size_t p = 0; p < member_ids.size(); ++p) {
    if (std::find(member_ids[p].begin(), member_ids[p].end(), id) !=
        member_ids[p].end()) {
      return static_cast<int>(p);
    }
  }
  return -1;
}

int ClusterPool::indexNear(const Eigen::Vector3d& position,
                           double radius_m) const {
  int best = -1;
  double best_distance = radius_m;
  for (std::size_t p = 0; p < clusters.size(); ++p) {
    const double distance = (clusters[p].position - position).norm();
    if (distance <= best_distance) {
      best_distance = distance;
      best = static_cast<int>(p);
    }
  }
  return best;
}

ClusterPool buildClusterPool(const std::vector<TourBidData>& bids,
                             const std::vector<FleetCluster>& held,
                             double merge_radius_m,
                             const ExploredFn& explored) {
  struct Entry {
    FleetCluster cluster;
    int bid = -1;
    int index = -1;
    bool owner_report = false;
  };
  std::vector<Entry> entries;
  for (std::size_t b = 0; b < bids.size(); ++b) {
    for (std::size_t i = 0; i < bids[b].clusters.size(); ++i) {
      const FleetCluster& c = bids[b].clusters[i];
      entries.push_back({c, static_cast<int>(b), static_cast<int>(i),
                         c.owner_robot_id == bids[b].robot_id});
    }
  }
  for (const FleetCluster& c : held) entries.push_back({c, -1, -1, false});
  // Owners' own reports first, then by ID, so a place's name does not depend
  // on the order the bids arrived in.
  std::stable_sort(entries.begin(), entries.end(),
                   [](const Entry& a, const Entry& b) {
                     if (a.owner_report != b.owner_report) {
                       return a.owner_report;
                     }
                     return a.cluster.id < b.cluster.id;
                   });

  ClusterPool merged;
  std::vector<int> entry_pool(entries.size(), -1);
  for (std::size_t k = 0; k < entries.size(); ++k) {
    const FleetCluster& c = entries[k].cluster;
    int p = merged.indexNear(c.position, merge_radius_m);
    if (p < 0) {
      p = static_cast<int>(merged.clusters.size());
      merged.clusters.push_back(c);
      merged.member_ids.emplace_back();
    } else {
      merged.clusters[p].gain = std::max(merged.clusters[p].gain, c.gain);
    }
    merged.member_ids[p].push_back(c.id);
    entry_pool[k] = p;
  }

  std::unordered_set<ClusterId> explored_ids;
  for (const TourBidData& bid : bids) {
    explored_ids.insert(bid.explored.begin(), bid.explored.end());
  }
  ClusterPool pool;
  std::vector<int> reindex(merged.clusters.size(), -1);
  for (std::size_t p = 0; p < merged.clusters.size(); ++p) {
    const bool named_explored = std::any_of(
        merged.member_ids[p].begin(), merged.member_ids[p].end(),
        [&explored_ids](ClusterId id) { return explored_ids.count(id) > 0; });
    if (named_explored ||
        (explored && explored(merged.clusters[p].position))) {
      pool.dropped_explored.push_back(merged.clusters[p]);
      continue;
    }
    reindex[p] = static_cast<int>(pool.clusters.size());
    pool.clusters.push_back(merged.clusters[p]);
    pool.member_ids.push_back(merged.member_ids[p]);
  }
  pool.bid_to_pool.resize(bids.size());
  for (std::size_t b = 0; b < bids.size(); ++b) {
    pool.bid_to_pool[b].assign(bids[b].clusters.size(), -1);
  }
  for (std::size_t k = 0; k < entries.size(); ++k) {
    if (entries[k].bid < 0) continue;
    pool.bid_to_pool[entries[k].bid][entries[k].index] =
        reindex[entry_pool[k]];
  }
  return pool;
}

AuctionBidder bidderCosts(const TourBidData& bid,
                          const std::vector<int>& bid_to_pool,
                          const ClusterPool& pool,
                          const CostEstimateFn& estimate) {
  const std::size_t n = pool.clusters.size();
  AuctionBidder bidder;
  bidder.robot_id = bid.robot_id;
  bidder.from_pose.assign(n, kUnreachableCost);
  bidder.between.assign(n, std::vector<double>(n, kUnreachableCost));
  // The bid's own clusters at each pool cluster.
  std::vector<std::vector<std::size_t>> mine(n);
  for (std::size_t i = 0; i < bid.clusters.size() && i < bid_to_pool.size();
       ++i) {
    if (bid_to_pool[i] >= 0) mine[bid_to_pool[i]].push_back(i);
  }
  const auto estimated = [&estimate](const Eigen::Vector3d& from,
                                     const Eigen::Vector3d& to) {
    if (!estimate) return kUnreachableCost;
    const double cost = estimate(from, to);
    return std::isnan(cost) || cost < 0.0 ? kUnreachableCost : cost;
  };
  const Eigen::Vector3d at = bid.pose.head<3>();
  for (std::size_t p = 0; p < n; ++p) {
    if (mine[p].empty()) {
      bidder.from_pose[p] = estimated(at, pool.clusters[p].position);
      continue;
    }
    for (const std::size_t i : mine[p]) {
      bidder.from_pose[p] = std::min(bidder.from_pose[p],
                                     bid.costs_from_pose[i]);
    }
  }
  for (std::size_t p = 0; p < n; ++p) {
    bidder.between[p][p] = 0.0;
    for (std::size_t q = p + 1; q < n; ++q) {
      double cost = kUnreachableCost;
      if (!mine[p].empty() && !mine[q].empty()) {
        for (const std::size_t i : mine[p]) {
          for (const std::size_t j : mine[q]) {
            cost = std::min({cost, bid.costBetween(i, j),
                             bid.costBetween(j, i)});
          }
        }
      } else {
        cost = estimated(pool.clusters[p].position, pool.clusters[q].position);
      }
      bidder.between[p][q] = cost;
      bidder.between[q][p] = cost;
    }
  }
  if (bid.current_target != kNoCluster) {
    bidder.current_target = pool.indexOf(bid.current_target);
  }
  return bidder;
}

AuctionResult runSequentialAuction(const std::vector<AuctionBidder>& input,
                                   const std::vector<bool>& fixed,
                                   double commit_margin,
                                   double balance_weight) {
  std::vector<const AuctionBidder*> bidders;
  for (const AuctionBidder& b : input) bidders.push_back(&b);
  std::sort(bidders.begin(), bidders.end(),
            [](const AuctionBidder* a, const AuctionBidder* b) {
              return a->robot_id < b->robot_id;
            });
  const std::size_t n = fixed.size();
  std::vector<bool> taken = fixed;
  std::map<int, std::vector<int>> order;
  std::map<int, bool> keeps_first;
  std::map<int, double> tour_cost;
  for (const AuctionBidder* b : bidders) {
    order[b->robot_id];
    keeps_first[b->robot_id] = false;
    tour_cost[b->robot_id] = 0.0;
  }

  // 1. Each bidder keeps its current target unless another's cost to it is
  // lower by more than the commit margin; of two heading for one cluster,
  // the closer keeps it (the lower ID on a tie).
  for (std::size_t t = 0; t < n; ++t) {
    if (taken[t]) continue;
    const AuctionBidder* keeper = nullptr;
    for (const AuctionBidder* b : bidders) {
      if (b->current_target == static_cast<int>(t) &&
          std::isfinite(b->from_pose[t]) &&
          (keeper == nullptr || b->from_pose[t] < keeper->from_pose[t])) {
        keeper = b;
      }
    }
    if (keeper == nullptr) continue;
    double best_other = kUnreachableCost;
    for (const AuctionBidder* b : bidders) {
      if (b != keeper) best_other = std::min(best_other, b->from_pose[t]);
    }
    if (best_other < (1.0 - commit_margin) * keeper->from_pose[t]) continue;
    order[keeper->robot_id].push_back(static_cast<int>(t));
    keeps_first[keeper->robot_id] = true;
    tour_cost[keeper->robot_id] = keeper->from_pose[t];
    taken[t] = true;
  }

  // 2. Sequential single-item auction over the rest.
  for (;;) {
    const AuctionBidder* winner = nullptr;
    int item = -1;
    Insertion where;
    double best_bid = kUnreachableCost;
    for (const AuctionBidder* b : bidders) {
      for (std::size_t c = 0; c < n; ++c) {
        if (taken[c]) continue;
        const Insertion insertion = cheapestInsertion(
            order[b->robot_id], static_cast<int>(c), b->from_pose, b->between,
            keeps_first[b->robot_id]);
        if (!std::isfinite(insertion.added_cost)) continue;
        const double bid =
            insertion.added_cost + balance_weight * tour_cost[b->robot_id];
        if (bid < best_bid) {
          best_bid = bid;
          winner = b;
          item = static_cast<int>(c);
          where = insertion;
        }
      }
    }
    if (winner == nullptr) break;
    std::vector<int>& bundle = order[winner->robot_id];
    bundle.insert(bundle.begin() + where.position, item);
    taken[item] = true;
    tour_cost[winner->robot_id] =
        openTourCost(bundle, winner->from_pose, winner->between);
  }

  // Each bundle in the order its robot will tour it (§2.3), a kept target
  // first.
  AuctionResult result;
  for (const AuctionBidder* b : bidders) {
    std::vector<int> bundle = order[b->robot_id];
    if (bundle.size() > 1) {
      std::vector<double> from;
      std::vector<std::vector<double>> between;
      for (const int i : bundle) {
        from.push_back(b->from_pose[i]);
        between.emplace_back();
        for (const int j : bundle) between.back().push_back(b->between[i][j]);
      }
      const OpenTour tour =
          solveOpenTour(from, between, keeps_first[b->robot_id] ? 0 : -1);
      if (tour.order.size() == bundle.size()) {
        std::vector<int> ordered;
        for (const int k : tour.order) ordered.push_back(bundle[k]);
        bundle = std::move(ordered);
      }
    }
    result.bundles[b->robot_id] = std::move(bundle);
  }
  for (std::size_t c = 0; c < n; ++c) {
    if (!taken[c]) result.unassigned.push_back(static_cast<int>(c));
  }
  return result;
}

}  // namespace mgg
```

Add the source to `add_library` in `ros2/src/mgg_core/CMakeLists.txt`, after `  src/fleet_types.cpp`:

```cmake
  src/fleet_auction.cpp
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_fleet_auction '*'`
Expected: `[  PASSED  ] 10 tests.`

- [ ] **Step 6: Run both gates**

Run: `/tmp/mgg-tour/tuf-run.sh off test` then `/tmp/mgg-tour/tuf-run.sh on test`
Expected: both end with `0 errors, 0 failures`.

- [ ] **Step 7: Commit**

```bash
git add ros2/src/mgg_core/include/mgg_core/fleet_auction.h \
        ros2/src/mgg_core/src/fleet_auction.cpp \
        ros2/src/mgg_core/test/test_fleet_auction.cpp \
        ros2/src/mgg_core/CMakeLists.txt
git commit -m "Pool the group's frontier clusters and award them by sequential auction"
```

---

### Task 9: The claim registry

**Files:**
- Create: `ros2/src/mgg_core/include/mgg_core/fleet_claims.h`
- Create: `ros2/src/mgg_core/src/fleet_claims.cpp`
- Create: `ros2/src/mgg_core/test/test_fleet_claims.cpp`
- Modify: `ros2/src/mgg_core/CMakeLists.txt` (library source, test)

**Interfaces:**
- Consumes: `mgg::FleetCluster`, `mgg::ExploredFn` (Task 7).
- Produces:
  - `struct mgg::Claim { int robot_id; std::vector<FleetCluster> clusters; double last_heard_s; };`
  - `class mgg::ClaimRegistry { public: void record(int robot_id, std::vector<FleetCluster> clusters, double heard_s); void heard(int robot_id, double heard_s); std::vector<int> expire(double now_s, double ttl_s); int dropExplored(const ExploredFn& explored); bool release(int robot_id); int releaseOldest(const std::set<int>& candidates); const Claim* find(int robot_id) const; std::vector<int> robots() const; std::vector<FleetCluster> clustersOf(const std::set<int>& robots) const; std::vector<FleetCluster> clustersExcept(int robot_id) const; };`

- [ ] **Step 1: Write the failing test**

Create `ros2/src/mgg_core/test/test_fleet_claims.cpp`:

```cpp
// Tests for the claims robots hold on frontier clusters (tour-exploration
// design §4): they outlive silence until the TTL, and are released early
// when explored, by an idle robot's take-over, or by the operator.

#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/fleet_claims.h"

namespace {

using mgg::ClaimRegistry;
using mgg::FleetCluster;

FleetCluster cluster(mgg::ClusterId id, int owner, double x) {
  FleetCluster c;
  c.id = id;
  c.owner_robot_id = owner;
  c.position = Eigen::Vector3d(x, 0.0, 0.0);
  return c;
}

TEST(ClaimRegistry, AClaimLastsUntilItsRobotHasBeenSilentForTheTtl) {
  ClaimRegistry claims;
  claims.record(2, {cluster(21, 2, 10.0)}, 100.0);
  EXPECT_TRUE(claims.expire(100.0 + 599.0, 600.0).empty());
  ASSERT_NE(claims.find(2), nullptr);
  // Heard again: the TTL counts from here.
  claims.heard(2, 300.0);
  EXPECT_TRUE(claims.expire(850.0, 600.0).empty());
  EXPECT_EQ(claims.expire(901.0, 600.0), std::vector<int>{2});
  EXPECT_EQ(claims.find(2), nullptr);
}

TEST(ClaimRegistry, RecordingKeepsTheLatestHeardTimeAndDropsAnEmptyBundle) {
  ClaimRegistry claims;
  claims.record(2, {cluster(21, 2, 10.0)}, 200.0);
  claims.record(2, {cluster(21, 2, 10.0), cluster(22, 2, 12.0)}, 150.0);
  ASSERT_NE(claims.find(2), nullptr);
  EXPECT_EQ(claims.find(2)->clusters.size(), 2u);
  EXPECT_DOUBLE_EQ(claims.find(2)->last_heard_s, 200.0);
  claims.record(2, {}, 250.0);
  EXPECT_EQ(claims.find(2), nullptr);
  // Hearing a robot that holds nothing records nothing.
  claims.heard(5, 10.0);
  EXPECT_TRUE(claims.robots().empty());
}

TEST(ClaimRegistry, ExploredClustersLeaveEveryClaim) {
  ClaimRegistry claims;
  claims.record(2, {cluster(21, 2, 10.0), cluster(22, 2, 20.0)}, 0.0);
  claims.record(3, {cluster(31, 3, -10.0)}, 0.0);
  const int dropped = claims.dropExplored(
      [](const Eigen::Vector3d& p) { return p.x() < 0.0 || p.x() > 15.0; });
  EXPECT_EQ(dropped, 2);
  ASSERT_NE(claims.find(2), nullptr);
  ASSERT_EQ(claims.find(2)->clusters.size(), 1u);
  EXPECT_EQ(claims.find(2)->clusters[0].id, 21u);
  EXPECT_EQ(claims.find(3), nullptr);
  EXPECT_EQ(claims.dropExplored(nullptr), 0);
}

TEST(ClaimRegistry, TheOperatorReleasesAClaim) {
  ClaimRegistry claims;
  claims.record(2, {cluster(21, 2, 10.0)}, 0.0);
  EXPECT_TRUE(claims.release(2));
  EXPECT_FALSE(claims.release(2));
  EXPECT_EQ(claims.find(2), nullptr);
}

TEST(ClaimRegistry, TheLongestSilentCandidateIsReleasedFirst) {
  ClaimRegistry claims;
  claims.record(2, {cluster(21, 2, 10.0)}, 50.0);
  claims.record(3, {cluster(31, 3, 20.0)}, 20.0);
  claims.record(4, {cluster(41, 4, 30.0)}, 10.0);
  // Robot 4 has been silent longest but is not a candidate (in the group).
  EXPECT_EQ(claims.releaseOldest({2, 3}), 3);
  EXPECT_EQ(claims.releaseOldest({2, 3}), 2);
  EXPECT_EQ(claims.releaseOldest({2, 3}), -1);
  EXPECT_NE(claims.find(4), nullptr);
}

TEST(ClaimRegistry, ClustersByHolder) {
  ClaimRegistry claims;
  claims.record(1, {cluster(11, 1, 1.0)}, 0.0);
  claims.record(2, {cluster(21, 2, 2.0), cluster(22, 2, 3.0)}, 0.0);
  claims.record(3, {cluster(31, 3, 4.0)}, 0.0);
  EXPECT_EQ(claims.robots(), (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(claims.clustersOf({2, 3}).size(), 3u);
  const std::vector<FleetCluster> others = claims.clustersExcept(1);
  ASSERT_EQ(others.size(), 3u);
  for (const FleetCluster& c : others) EXPECT_NE(c.owner_robot_id, 1);
}

}  // namespace
```

Register it in `ros2/src/mgg_core/CMakeLists.txt` after the `test_fleet_auction` lines:

```cmake
  ament_add_gtest(test_fleet_claims test/test_fleet_claims.cpp)
  target_link_libraries(test_fleet_claims ${PROJECT_NAME})
```

- [ ] **Step 2: Run it to verify it fails**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_fleet_claims '*'`
Expected: FAIL at compile time with `mgg_core/fleet_claims.h: No such file or directory`.

- [ ] **Step 3: Write the header**

Create `ros2/src/mgg_core/include/mgg_core/fleet_claims.h`:

```cpp
// The clusters other robots hold (tour-exploration design §4). A robot's
// claim is its last awarded bundle; nobody else gets those clusters. Silence
// is not failure: a claim stays after its robot falls silent, until
// fleet.claim_ttl_s after it was last heard, and is released earlier when a
// robot of the group finds a cluster explored (it simply stops being a
// frontier), when idle robots take the oldest silent claims over, or by the
// operator.

#ifndef MGG_CORE_FLEET_CLAIMS_H_
#define MGG_CORE_FLEET_CLAIMS_H_

#include <map>
#include <set>
#include <vector>

#include "mgg_core/fleet_types.h"

namespace mgg {

struct Claim {
  int robot_id = 0;
  std::vector<FleetCluster> clusters;
  /// When this robot, or the auctioneer it learned the claim from, last
  /// heard the holder.
  double last_heard_s = 0.0;
};

class ClaimRegistry {
 public:
  /// `robot_id` holds `clusters` as of an award, heard at `heard_s` (the
  /// later of this and any earlier time is kept). An empty bundle is no
  /// claim.
  void record(int robot_id, std::vector<FleetCluster> clusters,
              double heard_s);
  /// `robot_id` was heard: its claim, if any, is fresh again.
  void heard(int robot_id, double heard_s);
  /// Drops the claims of robots silent for more than `ttl_s`; returns their
  /// IDs.
  std::vector<int> expire(double now_s, double ttl_s);
  /// Removes every claimed cluster at which `explored` holds; returns how
  /// many.
  int dropExplored(const ExploredFn& explored);
  /// The operator's release. False when there was no claim.
  bool release(int robot_id);
  /// Releases the claim of the candidate silent longest (the lower ID on a
  /// tie) and returns its robot ID, or -1 when no candidate holds one.
  int releaseOldest(const std::set<int>& candidates);

  const Claim* find(int robot_id) const;
  std::vector<int> robots() const;
  std::vector<FleetCluster> clustersOf(const std::set<int>& robots) const;
  std::vector<FleetCluster> clustersExcept(int robot_id) const;

 private:
  std::map<int, Claim> claims_;
};

}  // namespace mgg

#endif  // MGG_CORE_FLEET_CLAIMS_H_
```

- [ ] **Step 4: Write the implementation**

Create `ros2/src/mgg_core/src/fleet_claims.cpp`:

```cpp
#include "mgg_core/fleet_claims.h"

#include <algorithm>
#include <utility>

namespace mgg {

void ClaimRegistry::record(int robot_id, std::vector<FleetCluster> clusters,
                           double heard_s) {
  const auto found = claims_.find(robot_id);
  const double heard = found == claims_.end()
                           ? heard_s
                           : std::max(found->second.last_heard_s, heard_s);
  if (clusters.empty()) {
    if (found != claims_.end()) claims_.erase(found);
    return;
  }
  claims_[robot_id] = Claim{robot_id, std::move(clusters), heard};
}

void ClaimRegistry::heard(int robot_id, double heard_s) {
  const auto found = claims_.find(robot_id);
  if (found == claims_.end()) return;
  found->second.last_heard_s = std::max(found->second.last_heard_s, heard_s);
}

std::vector<int> ClaimRegistry::expire(double now_s, double ttl_s) {
  std::vector<int> expired;
  for (auto it = claims_.begin(); it != claims_.end();) {
    if (now_s - it->second.last_heard_s > ttl_s) {
      expired.push_back(it->first);
      it = claims_.erase(it);
    } else {
      ++it;
    }
  }
  return expired;
}

int ClaimRegistry::dropExplored(const ExploredFn& explored) {
  if (!explored) return 0;
  int dropped = 0;
  for (auto it = claims_.begin(); it != claims_.end();) {
    std::vector<FleetCluster>& clusters = it->second.clusters;
    const std::size_t before = clusters.size();
    clusters.erase(std::remove_if(clusters.begin(), clusters.end(),
                                  [&explored](const FleetCluster& c) {
                                    return explored(c.position);
                                  }),
                   clusters.end());
    dropped += static_cast<int>(before - clusters.size());
    if (clusters.empty()) {
      it = claims_.erase(it);
    } else {
      ++it;
    }
  }
  return dropped;
}

bool ClaimRegistry::release(int robot_id) {
  return claims_.erase(robot_id) > 0;
}

int ClaimRegistry::releaseOldest(const std::set<int>& candidates) {
  auto oldest = claims_.end();
  for (auto it = claims_.begin(); it != claims_.end(); ++it) {
    if (candidates.count(it->first) == 0) continue;
    if (oldest == claims_.end() ||
        it->second.last_heard_s < oldest->second.last_heard_s) {
      oldest = it;
    }
  }
  if (oldest == claims_.end()) return -1;
  const int robot_id = oldest->first;
  claims_.erase(oldest);
  return robot_id;
}

const Claim* ClaimRegistry::find(int robot_id) const {
  const auto found = claims_.find(robot_id);
  return found == claims_.end() ? nullptr : &found->second;
}

std::vector<int> ClaimRegistry::robots() const {
  std::vector<int> ids;
  for (const auto& entry : claims_) ids.push_back(entry.first);
  return ids;
}

std::vector<FleetCluster> ClaimRegistry::clustersOf(
    const std::set<int>& robots) const {
  std::vector<FleetCluster> clusters;
  for (const auto& [robot_id, claim] : claims_) {
    if (robots.count(robot_id) == 0) continue;
    clusters.insert(clusters.end(), claim.clusters.begin(),
                    claim.clusters.end());
  }
  return clusters;
}

std::vector<FleetCluster> ClaimRegistry::clustersExcept(int robot_id) const {
  std::vector<FleetCluster> clusters;
  for (const auto& [holder, claim] : claims_) {
    if (holder == robot_id) continue;
    clusters.insert(clusters.end(), claim.clusters.begin(),
                    claim.clusters.end());
  }
  return clusters;
}

}  // namespace mgg
```

Add the source to `add_library` in `ros2/src/mgg_core/CMakeLists.txt`, after `  src/fleet_auction.cpp`:

```cmake
  src/fleet_claims.cpp
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_fleet_claims '*'`
Expected: `[  PASSED  ] 6 tests.`

- [ ] **Step 6: Run both gates**

Run: `/tmp/mgg-tour/tuf-run.sh off test` then `/tmp/mgg-tour/tuf-run.sh on test`
Expected: both end with `0 errors, 0 failures`.

- [ ] **Step 7: Commit**

```bash
git add ros2/src/mgg_core/include/mgg_core/fleet_claims.h \
        ros2/src/mgg_core/src/fleet_claims.cpp \
        ros2/src/mgg_core/test/test_fleet_claims.cpp \
        ros2/src/mgg_core/CMakeLists.txt
git commit -m "Keep silent robots' claims until their TTL or an early release"
```

---
### Task 10: The fleet coordinator: groups, the auctioneer, bids and awards

**Files:**
- Create: `ros2/src/mgg_core/include/mgg_core/fleet_coordinator.h`
- Create: `ros2/src/mgg_core/src/fleet_coordinator.cpp`
- Create: `ros2/src/mgg_core/test/test_fleet_coordinator.cpp`
- Modify: `ros2/src/mgg_core/CMakeLists.txt` (library source, test)

**Interfaces:**
- Consumes: `mgg::FleetParams` (Task 1); `mgg::TourBidData`, `mgg::TourAwardData`, `mgg::RobotBundle`, `mgg::FleetCluster`, `mgg::ExploredFn`, `mgg::CostEstimateFn` (Task 7); `mgg::buildClusterPool`, `mgg::bidderCosts`, `mgg::runSequentialAuction` (Task 8); `mgg::ClaimRegistry` (Task 9).
- Produces:
  - `using mgg::OwnBidFn = std::function<TourBidData()>;`
  - `struct mgg::FleetTickOutput { std::optional<TourBidData> bid; std::optional<TourAwardData> award; };`
  - `inline constexpr std::size_t mgg::kMaxExploredElsewhere = 4096;`
  - `class mgg::FleetCoordinator` with `FleetCoordinator(int robot_id, const FleetParams& params, double commit_margin)`, `void onBid(const TourBidData&, double now_s)`, `void onAward(const TourAwardData&, double now_s)`, `FleetTickOutput tick(double now_s, const OwnBidFn& own_bid, const CostEstimateFn& estimate, const ExploredFn& explored)`, `void requestAuction()`, `bool awaitingAuction() const`, `bool requestAnswered() const`, `bool releaseClaims(int robot_id, double now_s)`, `int takeOverOldestClaim(double now_s)`, `std::vector<int> group(double now_s) const`, `int auctioneer(double now_s) const`, `bool inGroup(double now_s) const`, `bool hasAward() const`, `const std::vector<FleetCluster>& bundle() const`, `std::vector<FleetCluster> claimedByOthers(double now_s)`, `const std::vector<FleetCluster>& exploredElsewhere() const`, `const std::vector<FleetCluster>& lastAwardClusters() const`, `std::size_t lastUnassigned() const`, `std::uint64_t assignmentVersion() const`.

Protocol decisions the spec leaves open, made here and pinned by tests:
- Every robot bids every `fleet.auction_interval_s` unasked (spec §3.3's "its own periodic bid"); that is how peers hear it, so `fleet.peer_timeout_s` (5 s) spans two missed bids.
- Auction IDs are `robot_id << 40 | counter`. An award is applied once, identified by (auctioneer, auction ID, stamp): an auctioneer that restarts and counts from 1 again is still followed (Review Focus 5).
- A robot follows only its current auctioneer's calls and awards: after a group splits or merges, a stale auctioneer's award is ignored.
- One silent claim is released per auction for idle bidders (§3.5, §4 release 2), the longest silent first; its clusters join that auction's pool.
- A cluster once reported explored stays out of every later pool at the auctioneer (its `exploredElsewhere` list). Since 89d3f6c a frontier its owner explored is demoted on every peer by the owner's next broadcast (`refreshVertex` takes the owner's mark both ways), so that case needs no memory. A frontier explored by a robot other than its owner still does: the owner's map does not show that robot's ground, so the owner keeps marking it, bidding it and broadcasting it, which re-marks every merged copy. Without this memory the owner keeps bidding it, and the award flip-flops between it and the report that it is explored. The test `AClusterReportedExploredStaysOutOfLaterAuctions` pins that case: robot 1 bids its own frontier after robot 2 reported it explored.
- A robot that requested an auction (`requestAuction`) is "answered" once the award of an auction it bid in with the request flag set is applied; if that award leaves it empty, exploration is complete for it (§3.5). An award that gives it clusters clears the request.

- [ ] **Step 1: Write the failing test**

Create `ros2/src/mgg_core/test/test_fleet_coordinator.cpp`:

```cpp
// Tests for the fleet coordinator (tour-exploration design §3, §4): the
// auctioneer election, one-round auctions over an in-memory radio, late
// bids and missed awards, explored clusters, merging groups, silent claims
// and their release. Time is simulated.

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/fleet_coordinator.h"

namespace {

using mgg::ClusterId;
using mgg::FleetCluster;
using mgg::FleetCoordinator;
using mgg::FleetParams;
using mgg::TourAwardData;
using mgg::TourBidData;

FleetCluster cluster(ClusterId id, int owner, double x, double y = 0.0) {
  FleetCluster c;
  c.id = id;
  c.owner_robot_id = owner;
  c.position = Eigen::Vector3d(x, y, 0.0);
  c.gain = 1000.0;
  return c;
}

double euclid(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return (a - b).norm();
}

std::vector<ClusterId> idsOf(const std::vector<FleetCluster>& clusters) {
  std::vector<ClusterId> ids;
  for (const FleetCluster& c : clusters) ids.push_back(c.id);
  std::sort(ids.begin(), ids.end());
  return ids;
}

/// A robot standing still at (x, y) that knows `known`, with straight-line
/// costs, and bids its bundle's first cluster as its current target.
struct SimRobot {
  SimRobot(int robot_id, double x, double y, const FleetParams& params)
      : id(robot_id),
        position(x, y, 0.0),
        coordinator(std::make_unique<FleetCoordinator>(robot_id, params, 0.2)) {}

  TourBidData ownBid() const {
    TourBidData bid;
    bid.pose = mgg::StateVec(position.x(), position.y(), 0.0, 0.0);
    bid.clusters = known;
    for (const FleetCluster& c : known) {
      bid.costs_from_pose.push_back(euclid(position, c.position));
    }
    for (const FleetCluster& a : known) {
      for (const FleetCluster& b : known) {
        bid.costs_between.push_back(euclid(a.position, b.position));
      }
    }
    bid.current_target = front();
    bid.explored = explored;
    return bid;
  }
  ClusterId front() const {
    const auto& bundle = coordinator->bundle();
    return bundle.empty() ? mgg::kNoCluster : bundle.front().id;
  }

  int id;
  Eigen::Vector3d position;
  std::vector<FleetCluster> known;
  std::vector<ClusterId> explored;
  std::unique_ptr<FleetCoordinator> coordinator;
};

/// Ticks every robot and delivers what each sends to every robot that hears
/// it, all in one frame. `deaf` holds (from, to) pairs that do not get
/// through; results (not calls) are logged with the time they were sent.
struct Radio {
  explicit Radio(std::vector<SimRobot*> members) : robots(std::move(members)) {}

  std::vector<SimRobot*> robots;
  std::set<std::pair<int, int>> deaf;
  std::vector<std::pair<double, TourAwardData>> awards;

  void cut(int a, int b) {
    deaf.insert({a, b});
    deaf.insert({b, a});
  }
  void restore(int a, int b) {
    deaf.erase({a, b});
    deaf.erase({b, a});
  }
  void step(double now) {
    std::vector<std::pair<int, mgg::FleetTickOutput>> sent;
    for (SimRobot* r : robots) {
      sent.emplace_back(r->id,
                        r->coordinator->tick(
                            now, [r] { return r->ownBid(); }, euclid, nullptr));
    }
    for (const auto& [from, out] : sent) {
      if (out.award && !out.award->call) awards.emplace_back(now, *out.award);
      for (SimRobot* r : robots) {
        if (r->id == from || deaf.count({from, r->id}) > 0) continue;
        if (out.bid) r->coordinator->onBid(*out.bid, now);
        if (out.award) r->coordinator->onAward(*out.award, now);
      }
    }
  }
  void runUntil(double& now, double until, double dt = 0.1) {
    for (; now < until - 1e-9; now += dt) step(now);
  }
};

TEST(FleetCoordinator, TheLowestIdIsTheAuctioneerAndHandsOverWhenItFallsSilent) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 10.0, 0.0, params),
      r3(3, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 12.0)};
  r3.known = {cluster(31, 3, 22.0)};
  Radio radio{{&r1, &r2, &r3}};
  double now = 0.0;
  radio.runUntil(now, 8.0);
  for (const SimRobot* r : radio.robots) {
    EXPECT_EQ(r->coordinator->auctioneer(now), 1);
  }
  ASSERT_FALSE(radio.awards.empty());
  for (const auto& [sent, award] : radio.awards) EXPECT_EQ(award.auctioneer_id, 1);

  // Robot 1 falls silent: after fleet.peer_timeout_s the next lowest takes
  // over at the next auction.
  radio.cut(1, 2);
  radio.cut(1, 3);
  const std::size_t before = radio.awards.size();
  radio.runUntil(now, 20.0);
  EXPECT_EQ(r2.coordinator->auctioneer(now), 2);
  EXPECT_EQ(r3.coordinator->auctioneer(now), 2);
  EXPECT_EQ(r1.coordinator->group(now), std::vector<int>{1});
  bool from_two = false;
  for (std::size_t i = before; i < radio.awards.size(); ++i) {
    EXPECT_NE(radio.awards[i].second.auctioneer_id, 3);
    from_two |= radio.awards[i].second.auctioneer_id == 2;
  }
  EXPECT_TRUE(from_two);
}

TEST(FleetCoordinator, AnAwardSplitsTheClustersAndEveryRobotAppliesItsBundle) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  // More than fleet.cluster_merge_radius_m apart: four clusters.
  r1.known = {cluster(11, 1, 2.0), cluster(12, 1, 6.0)};
  r2.known = {cluster(21, 2, 18.0), cluster(22, 2, 14.0)};
  Radio radio{{&r1, &r2}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  EXPECT_EQ(idsOf(r1.coordinator->bundle()), (std::vector<ClusterId>{11, 12}));
  EXPECT_EQ(idsOf(r2.coordinator->bundle()), (std::vector<ClusterId>{21, 22}));
  ASSERT_FALSE(radio.awards.empty());
  std::set<ClusterId> named;
  for (const mgg::RobotBundle& bundle : radio.awards.back().second.bundles) {
    for (const ClusterId id : bundle.clusters) EXPECT_TRUE(named.insert(id).second);
  }
  // Each knows what the other holds.
  EXPECT_EQ(idsOf(r1.coordinator->claimedByOthers(now)),
            (std::vector<ClusterId>{21, 22}));
}

TEST(FleetCoordinator, ALateBidKeepsItsPreviousBundle) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 18.0)};
  Radio radio{{&r1, &r2}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  ASSERT_EQ(r2.front(), 21u);

  // Robot 2's bids stop reaching robot 1 for three seconds (it stays in the
  // group: fleet.peer_timeout_s is 5 s), while a new cluster appears.
  radio.deaf.insert({2, 1});
  r1.known.push_back(cluster(13, 1, 6.0));
  const std::size_t first = radio.awards.size();
  radio.runUntil(now, 8.0);
  ASSERT_GT(radio.awards.size(), first);
  const TourAwardData& award = radio.awards.back().second;
  const mgg::RobotBundle* held = award.bundleOf(2);
  ASSERT_NE(held, nullptr);
  EXPECT_GT(held->silent_s, 0.0);
  EXPECT_EQ(held->clusters, std::vector<ClusterId>{21});
  EXPECT_EQ(r2.front(), 21u);
  EXPECT_EQ(idsOf(r1.coordinator->bundle()), (std::vector<ClusterId>{11, 13}));
}

TEST(FleetCoordinator, AMissedAwardKeepsThePreviousBundle) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 18.0)};
  Radio radio{{&r1, &r2}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  const std::vector<ClusterId> before = idsOf(r2.coordinator->bundle());
  ASSERT_EQ(before, std::vector<ClusterId>{21});

  // Robot 2 hears neither calls nor awards for three seconds, while its own
  // new cluster makes robot 1 auction.
  radio.deaf.insert({1, 2});
  r2.known.push_back(cluster(22, 2, 14.0));
  const std::size_t first = radio.awards.size();
  radio.runUntil(now, 8.0);
  ASSERT_GT(radio.awards.size(), first);
  EXPECT_EQ(idsOf(r2.coordinator->bundle()), before);

  // Hearing again, it follows the next award.
  radio.deaf.erase({1, 2});
  r1.known.push_back(cluster(14, 1, 5.0));
  const std::size_t second = radio.awards.size();
  radio.runUntil(now, 14.0);
  ASSERT_GT(radio.awards.size(), second);
  const mgg::RobotBundle* latest = radio.awards.back().second.bundleOf(2);
  ASSERT_NE(latest, nullptr);
  std::vector<ClusterId> expected = latest->clusters;
  std::sort(expected.begin(), expected.end());
  EXPECT_EQ(idsOf(r2.coordinator->bundle()), expected);
}

TEST(FleetCoordinator, ClustersInAPeersExploredSpaceAreDropped) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0), cluster(12, 1, 10.0)};
  r2.known = {cluster(21, 2, 18.0)};
  r2.explored = {12};  // robot 2's roadmap shows it explored
  Radio radio{{&r1, &r2}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  ASSERT_FALSE(radio.awards.empty());
  const TourAwardData& award = radio.awards.back().second;
  for (const mgg::RobotBundle& bundle : award.bundles) {
    for (const ClusterId id : bundle.clusters) EXPECT_NE(id, 12u);
  }
  EXPECT_EQ(idsOf(award.explored), std::vector<ClusterId>{12});
  EXPECT_EQ(idsOf(r1.coordinator->exploredElsewhere()),
            std::vector<ClusterId>{12});
  EXPECT_EQ(idsOf(r1.coordinator->bundle()), std::vector<ClusterId>{11});
}

TEST(FleetCoordinator, AClusterReportedExploredStaysOutOfLaterAuctions) {
  // Robot 2 explored ground robot 1 marked a frontier and reported it once.
  // Robot 1's own map does not show robot 2's ground, so robot 1 keeps
  // marking the frontier and bidding it.
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0), cluster(22, 1, 14.0)};
  r2.known = {cluster(21, 2, 18.0)};
  r2.explored = {22};
  Radio radio{{&r1, &r2}};
  double now = 0.0;
  radio.runUntil(now, 3.0);
  r2.explored.clear();
  r1.known.push_back(cluster(13, 1, 6.0));  // another auction
  const std::size_t first = radio.awards.size();
  radio.runUntil(now, 8.0);
  ASSERT_GT(radio.awards.size(), first);
  for (std::size_t i = first; i < radio.awards.size(); ++i) {
    for (const mgg::RobotBundle& bundle : radio.awards[i].second.bundles) {
      for (const ClusterId id : bundle.clusters) EXPECT_NE(id, 22u);
    }
  }
}

TEST(FleetCoordinator, TwoGroupsMergingSettleWithoutNeedlessTargetChanges) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 5.0, 0.0, params),
      r3(3, 30.0, 0.0, params), r4(4, 35.0, 0.0, params);
  r1.known = {cluster(11, 1, -2.0)};
  r2.known = {cluster(21, 2, 7.0)};
  r3.known = {cluster(31, 3, 28.0)};
  r4.known = {cluster(41, 4, 37.0)};
  Radio radio{{&r1, &r2, &r3, &r4}};
  for (const int a : {1, 2}) {
    for (const int b : {3, 4}) radio.cut(a, b);
  }
  double now = 0.0;
  radio.runUntil(now, 8.0);
  EXPECT_EQ(r3.coordinator->auctioneer(now), 3);
  EXPECT_EQ(r1.coordinator->auctioneer(now), 1);
  std::map<int, ClusterId> targets;
  for (const SimRobot* r : radio.robots) {
    targets[r->id] = r->front();
    ASSERT_NE(r->front(), mgg::kNoCluster) << "robot " << r->id;
  }

  // The groups come into contact: robot 1, the lowest ID of the merged
  // group, runs the auctions; robot 3 stops as soon as it hears robot 1.
  for (const int a : {1, 2}) {
    for (const int b : {3, 4}) radio.restore(a, b);
  }
  const double merged_at = now;
  const std::size_t first = radio.awards.size();
  radio.runUntil(now, 16.0);
  bool everyone = false;
  for (std::size_t i = first; i < radio.awards.size(); ++i) {
    const auto& [sent, award] = radio.awards[i];
    if (sent >= merged_at + params.auction_interval_s) {
      EXPECT_EQ(award.auctioneer_id, 1);
    }
    everyone |= award.auctioneer_id == 1 && award.bundleOf(3) != nullptr &&
                award.bundleOf(4) != nullptr && award.bundleOf(2) != nullptr;
  }
  EXPECT_TRUE(everyone);
  for (const SimRobot* r : radio.robots) {
    EXPECT_EQ(r->coordinator->auctioneer(now), 1);
    EXPECT_EQ(r->front(), targets[r->id]) << "robot " << r->id;
  }
}

TEST(FleetCoordinator, ASilentPeersClaimsPersistUntilTheTtl) {
  FleetParams params;
  params.claim_ttl_s = 600.0;  // SwarmDeck's SubT simulation value
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 18.0)};
  Radio radio{{&r1, &r2}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  ASSERT_EQ(r2.front(), 21u);

  radio.cut(1, 2);
  const double silent_from = now;
  radio.runUntil(now, 300.0, 1.0);
  // Long out of the group, robot 2 still holds its cluster.
  EXPECT_EQ(r1.coordinator->group(now), std::vector<int>{1});
  EXPECT_EQ(idsOf(r1.coordinator->claimedByOthers(now)),
            std::vector<ClusterId>{21});
  radio.runUntil(now, silent_from + params.claim_ttl_s + 5.0, 1.0);
  EXPECT_TRUE(r1.coordinator->claimedByOthers(now).empty());
}

TEST(FleetCoordinator, IdleRobotsTakeOverTheLongestSilentClaimFirst) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 10.0, 0.0, params),
      r3(3, 20.0, 0.0, params), r4(4, 30.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 12.0)};
  r3.known = {cluster(31, 3, 22.0)};
  r4.known = {cluster(41, 4, 32.0)};
  Radio radio{{&r1, &r2, &r3, &r4}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  ASSERT_EQ(r4.front(), 41u);
  // Robot 4 falls silent first, robot 3 two seconds later.
  for (const int other : {1, 2, 3}) radio.cut(4, other);
  radio.runUntil(now, 7.0);
  for (const int other : {1, 2}) radio.cut(3, other);
  radio.runUntil(now, 15.0);

  // Robots 1 and 2 finish their bundles with nothing left to explore.
  r1.known.clear();
  r2.known.clear();
  r1.coordinator->requestAuction();
  r2.coordinator->requestAuction();
  const std::size_t first = radio.awards.size();
  radio.runUntil(now, 22.0);
  ASSERT_GT(radio.awards.size(), first);
  bool released_four = false;
  for (std::size_t i = first; i < radio.awards.size(); ++i) {
    const auto& ids = radio.awards[i].second.released_robot_ids;
    released_four |= std::find(ids.begin(), ids.end(), 4) != ids.end();
  }
  EXPECT_TRUE(released_four);
  EXPECT_TRUE(r1.front() == 41u || r2.front() == 41u);
  // Robot 3's claim, younger, stays.
  const std::vector<ClusterId> held = idsOf(r1.coordinator->claimedByOthers(now));
  EXPECT_TRUE(std::find(held.begin(), held.end(), 31u) != held.end());
}

TEST(FleetCoordinator, ARobotAloneTakesOverTheLongestSilentClaimFirst) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 10.0, 0.0, params),
      r3(3, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 12.0)};
  r3.known = {cluster(31, 3, 22.0)};
  Radio radio{{&r1, &r2, &r3}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  radio.cut(3, 1);
  radio.cut(3, 2);
  radio.runUntil(now, 7.0);
  radio.cut(2, 1);
  radio.runUntil(now, 20.0);
  ASSERT_EQ(r1.coordinator->group(now), std::vector<int>{1});
  EXPECT_EQ(r1.coordinator->takeOverOldestClaim(now), 3);
  EXPECT_EQ(r1.coordinator->takeOverOldestClaim(now), 2);
  EXPECT_EQ(r1.coordinator->takeOverOldestClaim(now), -1);
  EXPECT_TRUE(r1.coordinator->claimedByOthers(now).empty());
}

TEST(FleetCoordinator, WithoutATransformARobotIsAloneAndJoinsWhenOneAppears) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 18.0)};
  Radio radio{{&r1, &r2}};
  // The ROS layer drops messages from a robot it holds no transform to.
  radio.cut(1, 2);
  double now = 0.0;
  radio.runUntil(now, 5.0);
  EXPECT_EQ(r1.coordinator->group(now), std::vector<int>{1});
  EXPECT_FALSE(r1.coordinator->hasAward());
  EXPECT_TRUE(r1.coordinator->bundle().empty());
  EXPECT_TRUE(radio.awards.empty());

  radio.restore(1, 2);  // a transform appears
  radio.runUntil(now, 10.0);
  EXPECT_EQ(r1.coordinator->group(now), (std::vector<int>{1, 2}));
  EXPECT_TRUE(r1.coordinator->hasAward());
  ASSERT_FALSE(radio.awards.empty());
  const mgg::RobotBundle* theirs = radio.awards.back().second.bundleOf(2);
  ASSERT_NE(theirs, nullptr);
  EXPECT_EQ(theirs->clusters, std::vector<ClusterId>{21});
}

TEST(FleetCoordinator, AnAuctioneerThatRestartsStillHasItsAwardsApplied) {
  // Review Focus 5: a restarted planner numbers its auctions from 1 again.
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 18.0)};
  Radio radio{{&r1, &r2}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  ASSERT_FALSE(radio.awards.empty());

  r1.coordinator = std::make_unique<FleetCoordinator>(1, params, 0.2);
  r1.known.push_back(cluster(13, 1, 6.0));
  const std::size_t first = radio.awards.size();
  radio.runUntil(now, 10.0);
  ASSERT_GT(radio.awards.size(), first);
  EXPECT_EQ(radio.awards[first].second.auction_id,
            radio.awards.front().second.auction_id);
  const std::vector<ClusterId> held = idsOf(r2.coordinator->claimedByOthers(now));
  EXPECT_TRUE(std::find(held.begin(), held.end(), 13u) != held.end());
}

TEST(FleetCoordinator, AnOperatorReleaseIsForwardedInTheNextAward) {
  const FleetParams params;
  SimRobot r1(1, 0.0, 0.0, params), r2(2, 10.0, 0.0, params),
      r3(3, 20.0, 0.0, params);
  r1.known = {cluster(11, 1, 2.0)};
  r2.known = {cluster(21, 2, 12.0)};
  r3.known = {cluster(31, 3, 22.0)};
  Radio radio{{&r1, &r2, &r3}};
  double now = 0.0;
  radio.runUntil(now, 5.0);
  radio.cut(3, 1);
  radio.cut(3, 2);
  radio.runUntil(now, 15.0);
  ASSERT_FALSE(r2.coordinator->claimedByOthers(now).empty());

  // Only the auctioneer answers the operator.
  EXPECT_FALSE(r2.coordinator->releaseClaims(3, now));
  EXPECT_TRUE(r1.coordinator->releaseClaims(3, now));
  const std::size_t first = radio.awards.size();
  radio.runUntil(now, 20.0);
  bool forwarded = false;
  for (std::size_t i = first; i < radio.awards.size(); ++i) {
    const auto& ids = radio.awards[i].second.released_robot_ids;
    forwarded |= std::find(ids.begin(), ids.end(), 3) != ids.end();
  }
  EXPECT_TRUE(forwarded);
  for (const FleetCluster& c : r2.coordinator->claimedByOthers(now)) {
    EXPECT_NE(c.id, 31u);
  }
  for (const FleetCluster& c : r1.coordinator->claimedByOthers(now)) {
    EXPECT_NE(c.id, 31u);
  }
}

}  // namespace
```

Register it in `ros2/src/mgg_core/CMakeLists.txt` after the `test_fleet_claims` lines:

```cmake
  ament_add_gtest(test_fleet_coordinator test/test_fleet_coordinator.cpp)
  target_link_libraries(test_fleet_coordinator ${PROJECT_NAME})
```

- [ ] **Step 2: Run it to verify it fails**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_fleet_coordinator '*'`
Expected: FAIL at compile time with `mgg_core/fleet_coordinator.h: No such file or directory`.

- [ ] **Step 3: Write the header**

Create `ros2/src/mgg_core/include/mgg_core/fleet_coordinator.h`:

```cpp
// Fleet frontier assignment inside MGG (tour-exploration design §3, §4): who
// is in this robot's group, who runs the auction, and the one-round call,
// bid and award exchange. No central server: each connected group runs its
// own.
//
// A robot's group is itself and every robot it heard (a bid or an award)
// within fleet.peer_timeout_s. The ROS layer delivers only messages from
// robots it holds a neighbour transform to, placed in this robot's frame, so
// a robot without a shared frame is alone and tours alone. The auctioneer is
// the group's lowest robot ID. It calls an auction when the pool's clusters,
// the membership, a robot's request or an operator release changed, at most
// every fleet.auction_interval_s; takes one bid per member for
// fleet.bid_deadline_s; and awards bundles to the bidders, with the claims
// of members that did not bid and of silent robots fixed.
//
// Silence is not failure (§4): a silent robot's claims stay excluded for
// others until fleet.claim_ttl_s after it was last heard, a robot finds them
// explored, an idle robot takes the oldest over, or the operator releases
// them.

#ifndef MGG_CORE_FLEET_COORDINATOR_H_
#define MGG_CORE_FLEET_COORDINATOR_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include "mgg_core/fleet_auction.h"
#include "mgg_core/fleet_claims.h"
#include "mgg_core/fleet_types.h"
#include "mgg_core/tour_params.h"

namespace mgg {

/// This robot's bid content now: pose, clusters and costs, current target
/// and claim stamp, explored clusters. The coordinator fills in the rest.
using OwnBidFn = std::function<TourBidData()>;

struct FleetTickOutput {
  /// To broadcast to peers.
  std::optional<TourBidData> bid;
  /// A call or an award, to broadcast to peers.
  std::optional<TourAwardData> award;
};

/// Clusters peers explored, remembered at most this many (the oldest go).
inline constexpr std::size_t kMaxExploredElsewhere = 4096;

class FleetCoordinator {
 public:
  FleetCoordinator(int robot_id, const FleetParams& params,
                   double commit_margin);

  /// A peer's bid, in this robot's frame. Malformed bids are dropped.
  void onBid(const TourBidData& bid, double now_s);
  /// A peer's call or award, in this robot's frame. Only this robot's
  /// current auctioneer is followed, and each award is applied once.
  void onAward(const TourAwardData& award, double now_s);
  /// One step. Returns this robot's bid when an auction was called, every
  /// fleet.auction_interval_s, and at once when it requests an auction; as
  /// auctioneer, a call when one is due and the award once the bid deadline
  /// has passed (applied here too). `own_bid` is called only for a bid.
  FleetTickOutput tick(double now_s, const OwnBidFn& own_bid,
                       const CostEstimateFn& estimate,
                       const ExploredFn& explored);

  /// §3.5: this robot's bundle is done; its bids ask for an auction.
  void requestAuction();
  /// A request is out and the auction answering it has not been awarded.
  bool awaitingAuction() const { return requested_ && !answered_; }
  /// The auction this robot asked for was awarded and left it nothing.
  bool requestAnswered() const { return requested_ && answered_; }
  /// §4 release 3, on the auctioneer only: `robot_id`'s claims are released
  /// here and forwarded in the next award. False when this robot is not the
  /// group's auctioneer.
  bool releaseClaims(int robot_id, double now_s);
  /// §3.5 and §4 release 2 for a robot alone: the claim of the robot silent
  /// longest is released so this robot may take its clusters over. Returns
  /// that robot's ID, or -1 when no silent robot holds one.
  int takeOverOldestClaim(double now_s);

  /// This robot and every robot heard within fleet.peer_timeout_s, sorted.
  std::vector<int> group(double now_s) const;
  int auctioneer(double now_s) const { return group(now_s).front(); }
  bool inGroup(double now_s) const { return group(now_s).size() > 1; }
  bool hasAward() const { return has_award_; }
  /// This robot's bundle from the last award it applied.
  const std::vector<FleetCluster>& bundle() const { return bundle_; }
  /// Every cluster other robots hold, stale claims expired first.
  std::vector<FleetCluster> claimedByOthers(double now_s);
  /// Clusters awards reported explored by some robot.
  const std::vector<FleetCluster>& exploredElsewhere() const {
    return explored_elsewhere_;
  }
  /// The clusters the last applied award named.
  const std::vector<FleetCluster>& lastAwardClusters() const {
    return award_clusters_;
  }
  /// Pool clusters no bidder could reach in this robot's last auction.
  std::size_t lastUnassigned() const { return last_unassigned_; }
  /// Changes whenever this robot's bundle or the claims it respects may
  /// have changed; the tour solves again.
  std::uint64_t assignmentVersion() const { return assignment_version_; }

 private:
  struct Collection {
    std::uint64_t auction_id = 0;
    double started_s = 0.0;
    std::vector<int> members;
    std::set<ClusterId> signature;
    std::map<int, TourBidData> bids;
  };

  void noteHeard(int robot_id, double now_s);
  void noteExplored(const std::vector<FleetCluster>& clusters);
  void applyAward(const TourAwardData& award, double now_s);
  TourAwardData computeAward(double now_s, const CostEstimateFn& estimate,
                             const ExploredFn& explored);
  /// The cluster IDs the members' latest bids name: the pool's makeup.
  std::set<ClusterId> poolSignature(const std::vector<int>& members) const;

  static constexpr double kNever = -std::numeric_limits<double>::infinity();

  int robot_id_;
  FleetParams params_;
  double commit_margin_;

  std::map<int, double> last_heard_s_;
  std::map<int, TourBidData> last_bids_;
  ClaimRegistry claims_;
  std::vector<FleetCluster> bundle_;
  std::vector<FleetCluster> award_clusters_;
  std::vector<FleetCluster> explored_elsewhere_;
  bool has_award_ = false;
  std::uint64_t assignment_version_ = 0;
  std::size_t last_unassigned_ = 0;

  // The last award applied, by (auctioneer, auction ID, stamp).
  int applied_auctioneer_ = -1;
  std::uint64_t applied_auction_id_ = 0;
  double applied_stamp_s_ = kNever;

  // Bidding.
  std::uint64_t seq_ = 0;
  double last_bid_s_ = kNever;
  std::uint64_t called_auction_ = 0;
  std::uint64_t bid_for_auction_ = 0;
  std::set<ClusterId> own_cluster_ids_;
  bool requested_ = false;
  bool request_sent_ = false;
  bool answered_ = false;
  std::uint64_t answer_auction_ = 0;
  int answer_auctioneer_ = -1;

  // Auctioneering.
  std::optional<Collection> collecting_;
  std::uint64_t auction_counter_ = 0;
  double last_auction_s_ = kNever;
  std::vector<int> auctioned_members_;
  std::set<ClusterId> auctioned_signature_;
  bool peer_requested_ = false;
  std::vector<int> pending_releases_;
};

}  // namespace mgg

#endif  // MGG_CORE_FLEET_COORDINATOR_H_
```

- [ ] **Step 4: Write the implementation**

Create `ros2/src/mgg_core/src/fleet_coordinator.cpp`:

```cpp
#include "mgg_core/fleet_coordinator.h"

#include <algorithm>
#include <utility>

namespace mgg {
namespace {

std::vector<ClusterId> idsOf(const std::vector<FleetCluster>& clusters) {
  std::vector<ClusterId> ids;
  ids.reserve(clusters.size());
  for (const FleetCluster& c : clusters) ids.push_back(c.id);
  return ids;
}

bool isMember(const std::vector<int>& members, int robot_id) {
  return std::find(members.begin(), members.end(), robot_id) != members.end();
}

}  // namespace

FleetCoordinator::FleetCoordinator(int robot_id, const FleetParams& params,
                                   double commit_margin)
    : robot_id_(robot_id), params_(params), commit_margin_(commit_margin) {}

std::vector<int> FleetCoordinator::group(double now_s) const {
  std::vector<int> members{robot_id_};
  for (const auto& [robot_id, heard_s] : last_heard_s_) {
    if (robot_id != robot_id_ && now_s - heard_s <= params_.peer_timeout_s) {
      members.push_back(robot_id);
    }
  }
  std::sort(members.begin(), members.end());
  return members;
}

void FleetCoordinator::noteHeard(int robot_id, double now_s) {
  const auto found = last_heard_s_.find(robot_id);
  if (found == last_heard_s_.end()) {
    last_heard_s_[robot_id] = now_s;
  } else {
    found->second = std::max(found->second, now_s);
  }
  claims_.heard(robot_id, now_s);
}

void FleetCoordinator::onBid(const TourBidData& bid, double now_s) {
  if (bid.robot_id == robot_id_ || !bid.wellFormed()) return;
  noteHeard(bid.robot_id, now_s);
  last_bids_[bid.robot_id] = bid;
  if (bid.request_auction) peer_requested_ = true;
  if (collecting_ && bid.auction_id == collecting_->auction_id) {
    collecting_->bids[bid.robot_id] = bid;
  }
}

void FleetCoordinator::onAward(const TourAwardData& award, double now_s) {
  if (award.auctioneer_id == robot_id_) return;
  noteHeard(award.auctioneer_id, now_s);
  // A robot that left the sender's group, or whose group just merged with a
  // lower ID, ignores it.
  if (award.auctioneer_id != auctioneer(now_s)) return;
  if (award.call) {
    called_auction_ = award.auction_id;
    return;
  }
  // Applied once. A restarted auctioneer numbers its auctions afresh, so an
  // award is known by its stamp as well as its number.
  if (award.auctioneer_id == applied_auctioneer_ &&
      award.auction_id == applied_auction_id_ &&
      award.stamp_s == applied_stamp_s_) {
    return;
  }
  applyAward(award, now_s);
}

void FleetCoordinator::requestAuction() {
  requested_ = true;
  request_sent_ = false;
  answered_ = false;
  answer_auction_ = 0;
  answer_auctioneer_ = -1;
}

FleetTickOutput FleetCoordinator::tick(double now_s, const OwnBidFn& own_bid,
                                       const CostEstimateFn& estimate,
                                       const ExploredFn& explored) {
  FleetTickOutput out;
  bool claims_changed = !claims_.expire(now_s, params_.claim_ttl_s).empty();
  if (explored && claims_.dropExplored(explored) > 0) claims_changed = true;
  if (claims_changed) ++assignment_version_;

  const std::vector<int> members = group(now_s);
  const bool leader = members.size() > 1 && members.front() == robot_id_;
  const bool requesting = requested_ && !answered_;

  const auto makeBid = [&](std::uint64_t auction_id) {
    TourBidData bid = own_bid ? own_bid() : TourBidData{};
    bid.robot_id = robot_id_;
    bid.seq = ++seq_;
    bid.stamp_s = now_s;
    bid.auction_id = auction_id;
    bid.bundle = idsOf(bundle_);
    bid.request_auction = bid.request_auction || requesting;
    own_cluster_ids_.clear();
    for (const FleetCluster& c : bid.clusters) own_cluster_ids_.insert(c.id);
    if (requesting && auction_id != 0 && answer_auction_ == 0) {
      answer_auction_ = auction_id;
      answer_auctioneer_ = members.front();
    }
    return bid;
  };

  // §3.3: bid on a call, every auction interval so peers hear this robot,
  // and at once when its bundle is done.
  const bool called = called_auction_ != 0 && called_auction_ != bid_for_auction_;
  const bool periodic = now_s - last_bid_s_ >= params_.auction_interval_s;
  const bool request_now = requesting && !request_sent_;
  if (called || periodic || request_now) {
    out.bid = makeBid(called ? called_auction_ : 0);
    last_bid_s_ = now_s;
    if (called) bid_for_auction_ = called_auction_;
    if (requesting) request_sent_ = true;
  }

  if (!leader) {
    collecting_.reset();
    return out;
  }
  if (collecting_) {
    if (now_s - collecting_->started_s < params_.bid_deadline_s) return out;
    TourAwardData award = computeAward(now_s, estimate, explored);
    auctioned_members_ = collecting_->members;
    auctioned_signature_ = collecting_->signature;
    collecting_.reset();
    last_auction_s_ = now_s;
    peer_requested_ = false;
    applyAward(award, now_s);
    out.award = std::move(award);
    return out;
  }
  // §3.2: a cluster appeared or disappeared in the pool, a robot joined or
  // left, a robot asked, or the operator released a claim.
  const std::set<ClusterId> signature = poolSignature(members);
  const bool due = !has_award_ || members != auctioned_members_ ||
                   signature != auctioned_signature_ || peer_requested_ ||
                   requesting || !pending_releases_.empty();
  if (!due || now_s - last_auction_s_ < params_.auction_interval_s) return out;
  Collection collection;
  collection.auction_id =
      (static_cast<std::uint64_t>(robot_id_ & 0xFFFF) << 40) |
      ++auction_counter_;
  collection.started_s = now_s;
  collection.members = members;
  collection.signature = signature;
  collection.bids[robot_id_] = makeBid(collection.auction_id);
  TourAwardData call;
  call.auction_id = collection.auction_id;
  call.auctioneer_id = robot_id_;
  call.stamp_s = now_s;
  call.call = true;
  collecting_ = std::move(collection);
  out.award = std::move(call);
  return out;
}

TourAwardData FleetCoordinator::computeAward(double now_s,
                                             const CostEstimateFn& estimate,
                                             const ExploredFn& explored) {
  const Collection& collection = *collecting_;
  std::vector<TourBidData> bids;
  std::set<int> bidders;
  for (const auto& [robot_id, bid] : collection.bids) {
    if (!isMember(collection.members, robot_id)) continue;
    bids.push_back(bid);
    bidders.insert(robot_id);
  }
  // Robots that did not bid keep what they hold (§3.4 step 3): members whose
  // bid missed the deadline, and silent robots.
  std::set<int> holders;
  std::set<int> silent;
  for (const int robot_id : claims_.robots()) {
    if (robot_id == robot_id_ || bidders.count(robot_id) > 0) continue;
    holders.insert(robot_id);
    if (!isMember(collection.members, robot_id)) silent.insert(robot_id);
  }
  std::vector<int> released = pending_releases_;
  pending_releases_.clear();

  // A cluster once reported explored stays out of the pool. A frontier its
  // owner explored is demoted everywhere by the owner's next broadcast
  // (graph_merge takes the owner's mark both ways), but one another robot
  // explored is not: the owner's map does not show that robot's ground, so
  // the owner keeps marking it, bidding it and broadcasting it as a
  // frontier, which re-marks every merged copy of it.
  const ExploredFn explored_anywhere = [this,
                                        &explored](const Eigen::Vector3d& p) {
    if (explored && explored(p)) return true;
    return std::any_of(explored_elsewhere_.begin(), explored_elsewhere_.end(),
                       [&](const FleetCluster& e) {
                         return (e.position - p).norm() <=
                                params_.cluster_merge_radius_m;
                       });
  };

  ClusterPool pool;
  AuctionResult result;
  std::vector<FleetCluster> freed;
  for (int attempt = 0; attempt < 2; ++attempt) {
    const std::vector<FleetCluster> held = claims_.clustersOf(holders);
    std::vector<FleetCluster> listed = held;
    listed.insert(listed.end(), freed.begin(), freed.end());
    pool = buildClusterPool(bids, listed, params_.cluster_merge_radius_m,
                            explored_anywhere);
    std::vector<bool> fixed(pool.clusters.size(), false);
    for (const FleetCluster& c : held) {
      int p = pool.indexOf(c.id);
      if (p < 0) p = pool.indexNear(c.position, params_.cluster_merge_radius_m);
      if (p >= 0) fixed[p] = true;
    }
    std::vector<AuctionBidder> auction_bidders;
    for (std::size_t k = 0; k < bids.size(); ++k) {
      auction_bidders.push_back(
          bidderCosts(bids[k], pool.bid_to_pool[k], pool, estimate));
    }
    result = runSequentialAuction(auction_bidders, fixed, commit_margin_,
                                  params_.balance_weight);
    // §3.5 and §4 release 2: a bidder left with nothing takes over the
    // claim of the robot silent longest, one per auction.
    const bool idle = std::any_of(
        bidders.begin(), bidders.end(), [&result](int robot_id) {
          const auto found = result.bundles.find(robot_id);
          return found == result.bundles.end() || found->second.empty();
        });
    if (attempt > 0 || !idle || silent.empty()) break;
    std::set<int> candidates;
    for (const int robot_id : silent) {
      if (holders.count(robot_id) > 0) candidates.insert(robot_id);
    }
    const Claim* oldest = nullptr;
    for (const int robot_id : candidates) {
      const Claim* claim = claims_.find(robot_id);
      if (claim != nullptr &&
          (oldest == nullptr || claim->last_heard_s < oldest->last_heard_s)) {
        oldest = claim;
      }
    }
    if (oldest == nullptr) break;
    freed = oldest->clusters;
    const int taken = claims_.releaseOldest(candidates);
    holders.erase(taken);
    released.push_back(taken);
    ++assignment_version_;
  }
  last_unassigned_ = result.unassigned.size();

  TourAwardData award;
  award.auction_id = collection.auction_id;
  award.auctioneer_id = robot_id_;
  award.stamp_s = now_s;
  std::set<ClusterId> named;
  const auto name = [&](int p) {
    const FleetCluster& c = pool.clusters[p];
    if (named.insert(c.id).second) award.clusters.push_back(c);
    return c.id;
  };
  for (const auto& [robot_id, indices] : result.bundles) {
    RobotBundle bundle;
    bundle.robot_id = robot_id;
    for (const int p : indices) bundle.clusters.push_back(name(p));
    award.bundles.push_back(std::move(bundle));
  }
  for (const int holder : holders) {
    const Claim* claim = claims_.find(holder);
    if (claim == nullptr) continue;
    RobotBundle bundle;
    bundle.robot_id = holder;
    bundle.silent_s = std::max(0.0, now_s - claim->last_heard_s);
    for (const FleetCluster& c : claim->clusters) {
      int p = pool.indexOf(c.id);
      if (p < 0) p = pool.indexNear(c.position, params_.cluster_merge_radius_m);
      if (p < 0) continue;  // explored meanwhile
      const ClusterId id = pool.clusters[p].id;
      if (std::find(bundle.clusters.begin(), bundle.clusters.end(), id) ==
          bundle.clusters.end()) {
        bundle.clusters.push_back(name(p));
      }
    }
    award.bundles.push_back(std::move(bundle));
  }
  award.explored = pool.dropped_explored;
  award.released_robot_ids = std::move(released);
  return award;
}

void FleetCoordinator::applyAward(const TourAwardData& award, double now_s) {
  applied_auctioneer_ = award.auctioneer_id;
  applied_auction_id_ = award.auction_id;
  applied_stamp_s_ = award.stamp_s;
  has_award_ = true;
  award_clusters_ = award.clusters;
  noteExplored(award.explored);
  for (const int robot_id : award.released_robot_ids) claims_.release(robot_id);
  for (const RobotBundle& bundle : award.bundles) {
    std::vector<FleetCluster> clusters;
    for (const ClusterId id : bundle.clusters) {
      if (const FleetCluster* c = award.cluster(id)) clusters.push_back(*c);
    }
    if (bundle.robot_id == robot_id_) {
      bundle_ = std::move(clusters);
      continue;
    }
    // Last heard as the auctioneer knows it, so a silent robot's claim ages
    // here as it does there.
    double heard_s = now_s - std::max(0.0, bundle.silent_s);
    const auto known = last_heard_s_.find(bundle.robot_id);
    if (known != last_heard_s_.end()) {
      heard_s = std::max(heard_s, known->second);
    }
    claims_.record(bundle.robot_id, std::move(clusters), heard_s);
  }
  // A robot the award does not name (its bid missed the deadline) keeps its
  // previous bundle.
  if (requested_ && award.auction_id == answer_auction_ &&
      award.auctioneer_id == answer_auctioneer_) {
    answered_ = true;
  }
  if (!bundle_.empty()) {
    requested_ = false;
    answered_ = false;
    request_sent_ = false;
    answer_auction_ = 0;
    answer_auctioneer_ = -1;
  }
  ++assignment_version_;
}

bool FleetCoordinator::releaseClaims(int robot_id, double now_s) {
  if (auctioneer(now_s) != robot_id_) return false;
  claims_.release(robot_id);
  pending_releases_.push_back(robot_id);
  ++assignment_version_;
  return true;
}

int FleetCoordinator::takeOverOldestClaim(double now_s) {
  const std::vector<int> members = group(now_s);
  std::set<int> silent;
  for (const int robot_id : claims_.robots()) {
    if (!isMember(members, robot_id)) silent.insert(robot_id);
  }
  const int taken = claims_.releaseOldest(silent);
  if (taken >= 0) ++assignment_version_;
  return taken;
}

std::vector<FleetCluster> FleetCoordinator::claimedByOthers(double now_s) {
  if (!claims_.expire(now_s, params_.claim_ttl_s).empty()) {
    ++assignment_version_;
  }
  return claims_.clustersExcept(robot_id_);
}

void FleetCoordinator::noteExplored(const std::vector<FleetCluster>& clusters) {
  for (const FleetCluster& c : clusters) {
    const bool known = std::any_of(
        explored_elsewhere_.begin(), explored_elsewhere_.end(),
        [&](const FleetCluster& e) {
          return (e.position - c.position).norm() <=
                 params_.cluster_merge_radius_m;
        });
    if (!known) explored_elsewhere_.push_back(c);
  }
  if (explored_elsewhere_.size() > kMaxExploredElsewhere) {
    explored_elsewhere_.erase(
        explored_elsewhere_.begin(),
        explored_elsewhere_.begin() +
            static_cast<std::ptrdiff_t>(explored_elsewhere_.size() -
                                        kMaxExploredElsewhere));
  }
}

std::set<ClusterId> FleetCoordinator::poolSignature(
    const std::vector<int>& members) const {
  std::set<ClusterId> ids = own_cluster_ids_;
  for (const int robot_id : members) {
    const auto bid = last_bids_.find(robot_id);
    if (bid == last_bids_.end()) continue;
    for (const FleetCluster& c : bid->second.clusters) ids.insert(c.id);
  }
  return ids;
}

}  // namespace mgg
```

Add the source to `add_library` in `ros2/src/mgg_core/CMakeLists.txt`, after `  src/fleet_claims.cpp`:

```cmake
  src/fleet_coordinator.cpp
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `/tmp/mgg-tour/tuf-run.sh off quick mgg_core test_fleet_coordinator '*'`
Expected: `[  PASSED  ] 13 tests.`

- [ ] **Step 6: Run both gates**

Run: `/tmp/mgg-tour/tuf-run.sh off test` then `/tmp/mgg-tour/tuf-run.sh on test`
Expected: both end with `0 errors, 0 failures`.

- [ ] **Step 7: Commit**

```bash
git add ros2/src/mgg_core/include/mgg_core/fleet_coordinator.h \
        ros2/src/mgg_core/src/fleet_coordinator.cpp \
        ros2/src/mgg_core/test/test_fleet_coordinator.cpp \
        ros2/src/mgg_core/CMakeLists.txt
git commit -m "Elect the lowest ID as auctioneer and run one-round frontier auctions"
```

---
### Task 11: Wire fleet assignment into the planner node

> **Apply by anchor.** This task edits `PlannerNode::onPlanRequest` (the branch after the tour, from Task 6), `PlannerNode::runGlobalPlanner` (the exclusions its frontier search takes; checked against 4777a17, the `searchGlobalFrontier` call quoted in Step 6 is verbatim) and `PlannerNode::tourCandidates` (Task 6), and adds `PlannerNodeTestPeer` helpers. Re-read the current code before each edit; `buildLocalGraph`, `straightDeparture` and `path_selection.cpp` are not touched. The `else if (low_gain)` branch after it keeps its no-path exceptions; `settleIdleRobot` applies the same two before it reports exploration complete.

**Files:**
- Create: `ros2/src/mgg_msgs/srv/ReleaseClaims.srv`
- Modify: `ros2/src/mgg_msgs/CMakeLists.txt` (one service)
- Modify: `ros2/src/mgg_ros/include/mgg_ros/planner_node.h` (includes, ten member functions, the coordinator, topics, service, timer)
- Modify: `ros2/src/mgg_ros/src/planner_node.cpp` (constructor, new functions after `publishTour`, `tourCandidates`, `runGlobalPlanner`, `onPlanRequest`)
- Test: `ros2/src/mgg_ros/test/test_planner_node.cpp` (seven helpers, five tests)

**Interfaces:**
- Consumes: `mgg::FleetCoordinator`, `mgg::FleetTickOutput` (Task 10); `mgg_ros::fromTourBidMsg`, `toTourBidMsg`, `fromTourAwardMsg`, `toTourAwardMsg` (Task 7); `mgg::exploredInGraph` (Task 2); `mgg::computeTourCosts`, `mgg::GraphDistanceCache::from`, `mgg::reachedDistance` (Task 4); `PlannerNode::globalFrontierClusters`, `linkRobotToGlobalGraph`, `tourCandidates`, `tour_planner_`, `tour_distances_`, `tour_assignment_version_` (Task 6); existing `refreshNeighbourTransform`, `poses_`, `communication_range_`, `selectionExclusions`.
- Produces:
  - service `mgg_msgs/srv/ReleaseClaims` (`int32 robot_id` → `bool success`, `string message`), served as `release_claims`
  - topics `tour_bid_out`/`tour_bid_in` (`mgg_msgs/TourBid`) and `tour_award_out`/`tour_award_in` (`mgg_msgs/TourAward`); a deployment remaps each pair to one shared topic, as it does `neighbour_graph_out`/`neighbour_graph_in`
  - `void PlannerNode::fleetTick(double now_s)`, `mgg::TourBidData PlannerNode::ownTourBid()`, `bool PlannerNode::settleIdleRobot(std::string& summary, bool& complete)`, `void PlannerNode::onTourBid(...)`, `void PlannerNode::onTourAward(...)`, `void PlannerNode::onReleaseClaims(...)`, `mgg::CostEstimateFn PlannerNode::roadmapCostEstimate()`, `mgg::ExploredFn PlannerNode::exploredByRoadmap()`, `bool PlannerNode::peerTransform(int, const std::string&, Eigen::Isometry3d&)`, `std::vector<Eigen::Vector3d> PlannerNode::fleetExclusions()`
  - test helpers `PlannerNodeTestPeer::ownTourBidMsg`, `receiveTourBid`, `fleetGroup`, `fleetTick`, `fleetHasAward`, `hearPeer`, `releaseClaims`

Behaviour: with `fleet.enabled` (the default) the node bids, auctions and follows awards on a 0.1 s timer. A peer's messages count only with a neighbour transform to it and (bids) within `communication_range`, the conditions under which its roadmap merges today. The tour's candidates become: its awarded bundle, plus, in a group with an award, frontiers it found itself that no award named yet; never clusters other robots hold, and never clusters peers reported explored, even when an older award put them in its bundle. A robot with nothing on its tour and no local path asks for an auction and gets no path until the award answers; if that award leaves it nothing, exploration is complete for it, except as for a failed global search (Global Constraints): while the lattice still sees gain (`local_gain_remains_now_`), and the first time after a rebuild dropped this robot's frontiers (`frontiers_dropped_in_rebuild_`, reset there as the low-gain branch resets it), it is no path. Alone, it takes over the longest-silent claim first. The greedy fallback (`runGlobalPlanner(-1)`) also skips clusters other robots hold or peers explored. SwarmDeck's reservation leases are still honoured when sent (spec §3.6: SwarmDeck stops sending them in delivery step 3).

- [ ] **Step 1: Write the service**

Create `ros2/src/mgg_msgs/srv/ReleaseClaims.srv`:

```
# The operator releases a silent robot's claimed frontier clusters
# (tour-exploration design §4). Answered by the group's auctioneer, which
# forwards the release in its next award.
int32 robot_id
---
bool success
string message
```

In `ros2/src/mgg_msgs/CMakeLists.txt`, after `  "srv/PlannerStringTrigger.srv"` add:

```cmake
  "srv/ReleaseClaims.srv"
```

- [ ] **Step 2: Write the failing tests**

In `ros2/src/mgg_ros/test/test_planner_node.cpp`, add `#include "mgg_ros/fleet_conversions.h"` after `#include "mgg_ros/planner_node.h"`, and add these helpers to `PlannerNodeTestPeer` after `bestPathFromGlobalGraph`:

```cpp
  /// This robot's bid as it would broadcast it.
  static mgg_msgs::msg::TourBid ownTourBidMsg(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    auto map_read = node.mapReadLease();
    mgg::TourBidData bid = node.ownTourBid();
    bid.robot_id = static_cast<int>(node.planning_params_.robot_id);
    bid.stamp_s = node.now().seconds();
    return toTourBidMsg(bid, node.world_frame_);
  }
  static void receiveTourBid(PlannerNode& node,
                             const mgg_msgs::msg::TourBid& msg) {
    node.onTourBid(std::make_shared<mgg_msgs::msg::TourBid>(msg));
  }
  static std::vector<int> fleetGroup(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.fleet_->group(node.now().seconds());
  }
  static void fleetTick(PlannerNode& node, double now_s) {
    node.fleetTick(now_s);
  }
  static bool fleetHasAward(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.fleet_->hasAward();
  }
  /// Robot `robot_id` heard now, bidding nothing: this robot is in a group,
  /// whose auctioneer is the lower ID of the two.
  static void hearPeer(PlannerNode& node, int robot_id) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    mgg::TourBidData bid;
    bid.robot_id = robot_id;
    node.fleet_->onBid(bid, node.now().seconds());
  }
  static std::shared_ptr<mgg_msgs::srv::ReleaseClaims::Response>
  releaseClaims(PlannerNode& node, int robot_id) {
    auto request = std::make_shared<mgg_msgs::srv::ReleaseClaims::Request>();
    request->robot_id = robot_id;
    auto response = std::make_shared<mgg_msgs::srv::ReleaseClaims::Response>();
    node.onReleaseClaims(request, response);
    return response;
  }
```

Add these tests at the end of the file, before the closing `}  // namespace mgg_ros`:

```cpp
TEST_F(PlannerNodeTest, APeersBidJoinsTheGroupOnlyWithATransformToIt) {
  TwoPlanners fleet("fleet_transform");
  const mgg_msgs::msg::TourBid bid = PlannerNodeTestPeer::ownTourBidMsg(*fleet.b);
  // No shared frame yet: robot 1 tours alone (tour-exploration design §4).
  PlannerNodeTestPeer::receiveTourBid(*fleet.a, bid);
  EXPECT_EQ(PlannerNodeTestPeer::fleetGroup(*fleet.a), std::vector<int>{1});
  // A transform appears: robot 2 joins robot 1's group.
  PlannerNodeTestPeer::receiveTransform(*fleet.a, "robot_0/odom",
                                        "robot_1/odom", 5.0, 0.0);
  PlannerNodeTestPeer::receiveTourBid(*fleet.a, bid);
  EXPECT_EQ(PlannerNodeTestPeer::fleetGroup(*fleet.a),
            (std::vector<int>{1, 2}));
}

TEST_F(PlannerNodeTest, AMalformedBidIsIgnored) {
  // Review Focus 1: one cost more than the bid has clusters.
  TwoPlanners fleet("fleet_malformed");
  PlannerNodeTestPeer::receiveTransform(*fleet.a, "robot_0/odom",
                                        "robot_1/odom", 5.0, 0.0);
  mgg_msgs::msg::TourBid bid = PlannerNodeTestPeer::ownTourBidMsg(*fleet.b);
  bid.costs_from_pose.push_back(1.0);
  PlannerNodeTestPeer::receiveTourBid(*fleet.a, bid);
  EXPECT_EQ(PlannerNodeTestPeer::fleetGroup(*fleet.a), std::vector<int>{1});
}

TEST_F(PlannerNodeTest, OnlyTheAuctioneerReleasesClaims) {
  TwoPlanners fleet("fleet_release");
  PlannerNodeTestPeer::receiveTransform(*fleet.a, "robot_0/odom",
                                        "robot_1/odom", 5.0, 0.0);
  PlannerNodeTestPeer::receiveTourBid(
      *fleet.a, PlannerNodeTestPeer::ownTourBidMsg(*fleet.b));
  PlannerNodeTestPeer::receiveTransform(*fleet.b, "robot_1/odom",
                                        "robot_0/odom", -5.0, 0.0);
  PlannerNodeTestPeer::receiveTourBid(
      *fleet.b, PlannerNodeTestPeer::ownTourBidMsg(*fleet.a));
  EXPECT_TRUE(PlannerNodeTestPeer::releaseClaims(*fleet.a, 3)->success);
  const auto refused = PlannerNodeTestPeer::releaseClaims(*fleet.b, 3);
  EXPECT_FALSE(refused->success);
  EXPECT_NE(refused->message.find("robot 1"), std::string::npos)
      << refused->message;
}

TEST_F(PlannerNodeTest, TheAuctioneerCallsAndAwardsOnItsFleetTimer) {
  TwoPlanners fleet("fleet_award");
  PlannerNodeTestPeer::receiveTransform(*fleet.a, "robot_0/odom",
                                        "robot_1/odom", 5.0, 0.0);
  PlannerNodeTestPeer::receiveTourBid(
      *fleet.a, PlannerNodeTestPeer::ownTourBidMsg(*fleet.b));
  const double t0 = fleet.a->now().seconds();
  PlannerNodeTestPeer::fleetTick(*fleet.a, t0);
  EXPECT_FALSE(PlannerNodeTestPeer::fleetHasAward(*fleet.a));  // call out
  PlannerNodeTestPeer::fleetTick(*fleet.a, t0 + 1.1);  // past the deadline
  EXPECT_TRUE(PlannerNodeTestPeer::fleetHasAward(*fleet.a));
}

TEST_F(PlannerNodeTest, AnEmptyAwardAfterARebuildDroppedFrontiersIsNotYetComplete) {
  // TheFirstFailedSearchAfterARebuildDroppedFrontiersIsNotComplete, in a
  // group: the award answering this robot's request is its failed search.
  // The first one after the rebuild is no path; the next is exploration
  // complete.
  auto node = makeNode("fleet_rebuild_dropped");
  PlannerNodeTestPeer::gainFromUnknownVoxelsOnly(*node);
  PlannerNodeTestPeer::observeFloor(*node, -3.55, 5.55, -2.55, 2.55);
  PlannerNodeTestPeer::acceptOdometry(*node, 0.0, 0.0, 1.0);
  PlannerNodeTestPeer::addGlobalChainToFrontier(*node, {{-0.5, 0.0}, {-1.0, 0.0}});
  PlannerNodeTestPeer::serveMap(*node, "component:test", 0);
  auto source = std::make_unique<TrajectoryInMemory>();
  source->trajectory = keyframesAlongX(0.0, 1.0);
  PlannerNodeTestPeer::setKeyframeSource(*node, std::move(source));
  ASSERT_TRUE(PlannerNodeTestPeer::rebuildRoadmap(
      *node, PlannerNode::RoadmapRebuildTrigger::kPoseUnlinkable));
  PlannerNodeTestPeer::setTour(*node, true, 0.0);
  PlannerNodeTestPeer::hearPeer(*node, 2);

  const auto plan = [&node]() {
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    PlannerNodeTestPeer::plan(*node, response);
    EXPECT_TRUE(response->path.empty())
        << "path of " << response->path.size() << " poses";
    return response->status;
  };
  // Nothing on its tour and no local path: it asks for an auction.
  EXPECT_EQ(plan(), PlannerNode::kStatusNoPath);
  const double t0 = node->now().seconds();
  PlannerNodeTestPeer::fleetTick(*node, t0);        // the call
  PlannerNodeTestPeer::fleetTick(*node, t0 + 1.1);  // the award: nothing
  ASSERT_TRUE(PlannerNodeTestPeer::fleetHasAward(*node));
  EXPECT_EQ(plan(), PlannerNode::kStatusNoPath);
  EXPECT_EQ(plan(), PlannerNode::kStatusComplete);
}
```

- [ ] **Step 3: Run them to verify they fail**

Run: `/tmp/mgg-tour/tuf-run.sh on quick mgg_ros test_planner_node '*Fleet*:*Bid*:*Claims*:*EmptyAward*'`
Expected: FAIL at compile time with `'class mgg_ros::PlannerNode' has no member named 'ownTourBid'`.

- [ ] **Step 4: Declare the fleet in the node's header**

In `ros2/src/mgg_ros/include/mgg_ros/planner_node.h`:

After `#include <mgg_msgs/srv/planner_srv.hpp>` add:

```cpp
#include <mgg_msgs/msg/tour_award.hpp>
#include <mgg_msgs/msg/tour_bid.hpp>
#include <mgg_msgs/srv/release_claims.hpp>
```

After `#include "mgg_core/tour_planner.h"` add:

```cpp
#include "mgg_core/fleet_coordinator.h"
```

After the declaration `void publishTour();` add:

```cpp
  /// Fleet frontier assignment (tour-exploration design §3, §4). A peer's
  /// bid or award is placed in this robot's frame with the transform its
  /// roadmap would merge with, and dropped without one, when malformed, or
  /// (a bid) from beyond communication_range, as a roadmap is.
  void onTourBid(mgg_msgs::msg::TourBid::ConstSharedPtr msg);
  void onTourAward(mgg_msgs::msg::TourAward::ConstSharedPtr msg);
  /// §4 release 3: the operator releases a silent robot's claims on the
  /// group's auctioneer, which forwards the release in its next award.
  void onReleaseClaims(
      const std::shared_ptr<mgg_msgs::srv::ReleaseClaims::Request> request,
      std::shared_ptr<mgg_msgs::srv::ReleaseClaims::Response> response);
  /// One fleet step at `now_s`; fleet_timer_ calls it with the node's clock.
  void fleetTick(double now_s);
  /// This robot's bid content: every frontier cluster it knows, costed from
  /// where it joins the global graph (no heading penalty), its tour's
  /// target and when it took it, and the awarded clusters its roadmap shows
  /// explored.
  mgg::TourBidData ownTourBid();
  /// The auctioneer's estimate of a cost a bid does not give: the global
  /// graph distance between the vertices nearest the two points, plus the
  /// straight links to them.
  mgg::CostEstimateFn roadmapCostEstimate();
  /// mgg::exploredInGraph on this robot's global graph.
  mgg::ExploredFn exploredByRoadmap();
  /// The transform placing a peer's messages from `frame` in this robot's
  /// frame, as its roadmap is placed; false when there is none.
  bool peerTransform(int robot_id, const std::string& frame,
                     Eigen::Isometry3d& t_ours_theirs);
  /// Clusters other robots hold and clusters peers explored, for the greedy
  /// fallback's frontier search.
  std::vector<Eigen::Vector3d> fleetExclusions();
  /// §3.5 for a robot with no tour target and no local path. In a group it
  /// asks for an auction and gets no path until the award answers; if that
  /// award leaves it nothing, exploration is complete for it, with the
  /// failed global search's exceptions: local gain remains, or a graph
  /// rebuild dropped frontiers since (review r0, I-2). Alone, it
  /// takes over the claim of the robot silent longest. False when it is
  /// alone with no claim to take over: the low-gain rule decides.
  bool settleIdleRobot(std::string& summary, bool& complete);
```

After the member `double tour_solve_ms_ = 0.0;` add:

```cpp
  /// Fleet frontier assignment (tour-exploration design §3); null when
  /// fleet.enabled is false.
  std::unique_ptr<mgg::FleetCoordinator> fleet_;
```

After the member `rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tour_pub_;` add:

```cpp
  rclcpp::Publisher<mgg_msgs::msg::TourBid>::SharedPtr tour_bid_pub_;
  rclcpp::Publisher<mgg_msgs::msg::TourAward>::SharedPtr tour_award_pub_;
  rclcpp::Subscription<mgg_msgs::msg::TourBid>::SharedPtr tour_bid_sub_;
  rclcpp::Subscription<mgg_msgs::msg::TourAward>::SharedPtr tour_award_sub_;
  rclcpp::Service<mgg_msgs::srv::ReleaseClaims>::SharedPtr
      release_claims_srv_;
  /// Runs fleetTick: bids, calls and awards.
  rclcpp::TimerBase::SharedPtr fleet_timer_;
  static constexpr double kFleetTickPeriodS = 0.1;
```

- [ ] **Step 5: Create the fleet and its handlers**

In `ros2/src/mgg_ros/src/planner_node.cpp`, add `#include "mgg_ros/fleet_conversions.h"` after `#include "mgg_ros/conversions.h"`.

In the constructor, after the two statements that create `tour_planner_` and `tour_pub_` (Task 6), add:

```cpp
  // Fleet frontier assignment (tour-exploration design §3). Each robot
  // publishes on tour_bid_out/tour_award_out and reads its peers' on
  // tour_bid_in/tour_award_in; a deployment remaps each pair to one shared
  // topic, as it does neighbour_graph_out/in.
  if (fleet_params_.enabled) {
    fleet_ = std::make_unique<mgg::FleetCoordinator>(
        static_cast<int>(planning_params_.robot_id), fleet_params_,
        tour_params_.commit_margin);
    tour_bid_pub_ = create_publisher<mgg_msgs::msg::TourBid>(
        "tour_bid_out", rclcpp::QoS(10));
    tour_award_pub_ = create_publisher<mgg_msgs::msg::TourAward>(
        "tour_award_out", rclcpp::QoS(10));
    tour_bid_sub_ = create_subscription<mgg_msgs::msg::TourBid>(
        "tour_bid_in", rclcpp::QoS(10),
        [this](mgg_msgs::msg::TourBid::ConstSharedPtr m) { onTourBid(m); },
        sub_opts);
    tour_award_sub_ = create_subscription<mgg_msgs::msg::TourAward>(
        "tour_award_in", rclcpp::QoS(10),
        [this](mgg_msgs::msg::TourAward::ConstSharedPtr m) {
          onTourAward(m);
        },
        sub_opts);
    release_claims_srv_ = create_service<mgg_msgs::srv::ReleaseClaims>(
        "release_claims",
        [this](const std::shared_ptr<mgg_msgs::srv::ReleaseClaims::Request> req,
               std::shared_ptr<mgg_msgs::srv::ReleaseClaims::Response> res) {
          onReleaseClaims(req, res);
        },
        rclcpp::ServicesQoS(), callback_group_);
    fleet_timer_ = create_timer(
        std::chrono::duration<double>(kFleetTickPeriodS),
        [this]() { fleetTick(now().seconds()); }, callback_group_);
  }
```

After the definition of `PlannerNode::publishTour` (Task 6), add:

```cpp
bool PlannerNode::peerTransform(int robot_id, const std::string& frame,
                                Eigen::Isometry3d& t_ours_theirs) {
  if (!refreshNeighbourTransform(robot_id, frame)) return false;
  return poses_->getRobotTransform(robot_id, t_ours_theirs);
}

void PlannerNode::onTourBid(mgg_msgs::msg::TourBid::ConstSharedPtr msg) {
  if (!fleet_ || msg->robot_id == static_cast<int>(planning_params_.robot_id)) {
    return;
  }
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  Eigen::Isometry3d t_ours_theirs = Eigen::Isometry3d::Identity();
  if (!peerTransform(msg->robot_id, msg->header.frame_id, t_ours_theirs)) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
                          "bid from robot %d ignored: no transform to '%s'",
                          msg->robot_id, msg->header.frame_id.c_str());
    return;
  }
  const mgg::TourBidData bid = fromTourBidMsg(*msg, t_ours_theirs);
  if (!bid.wellFormed()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "ignoring a malformed bid from robot %d",
                         msg->robot_id);
    return;
  }
  if (communication_range_ > 0.0 &&
      (bid.pose.head<3>() - current_state_.head<3>()).norm() >
          communication_range_) {
    return;
  }
  fleet_->onBid(bid, now().seconds());
}

void PlannerNode::onTourAward(mgg_msgs::msg::TourAward::ConstSharedPtr msg) {
  if (!fleet_ ||
      msg->auctioneer_id == static_cast<int>(planning_params_.robot_id)) {
    return;
  }
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  Eigen::Isometry3d t_ours_theirs = Eigen::Isometry3d::Identity();
  if (!peerTransform(msg->auctioneer_id, msg->header.frame_id,
                     t_ours_theirs)) {
    return;
  }
  const std::uint64_t before = fleet_->assignmentVersion();
  fleet_->onAward(fromTourAwardMsg(*msg, t_ours_theirs), now().seconds());
  if (!msg->call && fleet_->assignmentVersion() != before) {
    RCLCPP_INFO(get_logger(),
                "fleet award %llu from robot %d: %zu cluster(s) for this "
                "robot",
                static_cast<unsigned long long>(msg->auction_id),
                msg->auctioneer_id, fleet_->bundle().size());
  }
}

void PlannerNode::onReleaseClaims(
    const std::shared_ptr<mgg_msgs::srv::ReleaseClaims::Request> request,
    std::shared_ptr<mgg_msgs::srv::ReleaseClaims::Response> response) {
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (!fleet_) {
    response->success = false;
    response->message = "fleet assignment is off (fleet.enabled)";
    return;
  }
  const double now_s = now().seconds();
  if (!fleet_->releaseClaims(request->robot_id, now_s)) {
    response->success = false;
    response->message = "not the auctioneer: robot " +
                        std::to_string(fleet_->auctioneer(now_s)) + " is";
    return;
  }
  response->success = true;
  response->message = "robot " + std::to_string(request->robot_id) +
                      "'s claims released; forwarded in the next award";
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void PlannerNode::fleetTick(double now_s) {
  if (!fleet_) return;
  const std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
  if (!have_odometry_) return;
  auto map_read = mapReadLease();
  const mgg::FleetTickOutput out =
      fleet_->tick(now_s, [this]() { return ownTourBid(); },
                   roadmapCostEstimate(), exploredByRoadmap());
  if (out.bid) tour_bid_pub_->publish(toTourBidMsg(*out.bid, world_frame_));
  if (!out.award) return;
  tour_award_pub_->publish(toTourAwardMsg(*out.award, world_frame_));
  if (out.award->call) return;
  std::size_t assigned = 0;
  for (const mgg::RobotBundle& bundle : out.award->bundles) {
    assigned += bundle.clusters.size();
  }
  RCLCPP_INFO(get_logger(),
              "fleet award %llu: %zu cluster(s) to %zu robot(s), %zu "
              "unassigned, %zu explored, %zu claim(s) released",
              static_cast<unsigned long long>(out.award->auction_id),
              assigned, out.award->bundles.size(), fleet_->lastUnassigned(),
              out.award->explored.size(),
              out.award->released_robot_ids.size());
}

mgg::TourBidData PlannerNode::ownTourBid() {
  mgg::TourBidData bid;
  bid.pose = current_state_;
  bid.current_target = tour_planner_->target();
  bid.claim_stamp_s = tour_planner_->targetSince();
  // §3.3 pool construction: the awarded clusters this robot's roadmap
  // shows explored.
  for (const mgg::FleetCluster& cluster : fleet_->lastAwardClusters()) {
    if (mgg::exploredInGraph(*global_graph_, cluster.position,
                             fleet_params_.cluster_merge_radius_m)) {
      bid.explored.push_back(cluster.id);
    }
  }
  if (global_graph_->getNumVertices() <= 1) return bid;
  std::vector<mgg::FrontierCluster> clusters = globalFrontierClusters();
  // Best gain first: what a peer would refuse as too large is cut from the
  // low-gain end.
  if (clusters.size() > mgg::kMaxBidClusters) {
    clusters.resize(mgg::kMaxBidClusters);
  }
  if (clusters.empty()) return bid;
  mgg::TourCostMatrix costs;
  if (mgg::Vertex* link = linkRobotToGlobalGraph()) {
    costs = mgg::computeTourCosts(*global_graph_, graph_revision_,
                                  tour_distances_, link->id,
                                  current_state_[3], clusters,
                                  /*heading_weight=*/0.0);
  } else {
    costs.from_robot.assign(clusters.size(), mgg::kUnreachableCost);
    costs.between.assign(clusters.size(),
                         std::vector<double>(clusters.size(),
                                             mgg::kUnreachableCost));
  }
  for (std::size_t i = 0; i < clusters.size(); ++i) {
    const mgg::FrontierCluster& c = clusters[i];
    bid.clusters.push_back({c.id, c.owner_robot_id, c.position, c.gain});
    bid.costs_from_pose.push_back(costs.from_robot[i]);
    for (std::size_t j = 0; j < clusters.size(); ++j) {
      bid.costs_between.push_back(i == j ? 0.0 : costs.between[i][j]);
    }
  }
  return bid;
}

mgg::CostEstimateFn PlannerNode::roadmapCostEstimate() {
  return [this](const Eigen::Vector3d& from, const Eigen::Vector3d& to) {
    const mgg::StateVec from_state(from.x(), from.y(), from.z(), 0.0);
    const mgg::StateVec to_state(to.x(), to.y(), to.z(), 0.0);
    mgg::Vertex* a = nullptr;
    mgg::Vertex* b = nullptr;
    if (!global_graph_->getNearestVertex(&from_state, &a) ||
        !global_graph_->getNearestVertex(&to_state, &b) || a == nullptr ||
        b == nullptr) {
      return mgg::kUnreachableCost;
    }
    const mgg::ShortestPathsReport* report =
        tour_distances_.from(*global_graph_, graph_revision_, a->id);
    if (report == nullptr) return mgg::kUnreachableCost;
    return mgg::reachedDistance(*report, b->id) +
           (from - a->state.head<3>()).norm() +
           (to - b->state.head<3>()).norm();
  };
}

mgg::ExploredFn PlannerNode::exploredByRoadmap() {
  return [this](const Eigen::Vector3d& position) {
    return mgg::exploredInGraph(*global_graph_, position,
                                fleet_params_.cluster_merge_radius_m);
  };
}

std::vector<Eigen::Vector3d> PlannerNode::fleetExclusions() {
  std::vector<Eigen::Vector3d> points;
  if (!fleet_) return points;
  for (const mgg::FleetCluster& c : fleet_->claimedByOthers(now().seconds())) {
    points.push_back(c.position);
  }
  for (const mgg::FleetCluster& c : fleet_->exploredElsewhere()) {
    points.push_back(c.position);
  }
  return points;
}

bool PlannerNode::settleIdleRobot(std::string& summary, bool& complete) {
  const double now_s = now().seconds();
  if (fleet_->inGroup(now_s)) {
    if (fleet_->requestAnswered()) {
      // As a failed global search is (review r0, I-2): not exploration
      // complete while the lattice still sees gain it cannot send a path
      // to, nor the first time after a graph rebuild dropped frontiers.
      if (local_gain_remains_now_) {
        summary +=
            "; the fleet's award leaves it nothing, but local gain remains: "
            "no path";
      } else if (frontiers_dropped_in_rebuild_ > 0) {
        summary += "; the fleet's award leaves it nothing, but a graph "
                   "rebuild dropped " +
                   std::to_string(frontiers_dropped_in_rebuild_) +
                   " frontier(s): no path";
        frontiers_dropped_in_rebuild_ = 0;
      } else {
        complete = true;
        summary +=
            "; exploration complete for this robot: the fleet's award "
            "leaves it nothing";
      }
    } else {
      if (!fleet_->awaitingAuction()) fleet_->requestAuction();
      summary += "; its bundle is done: waiting for the auction it asked for";
    }
    return true;
  }
  const int taken = fleet_->takeOverOldestClaim(now_s);
  if (taken < 0) return false;
  summary += "; took over silent robot " + std::to_string(taken) +
             "'s claims";
  return true;
}
```

- [ ] **Step 6: Let the fleet shape the tour, the fallback and the idle robot**

Replace the whole body of `PlannerNode::tourCandidates` (Task 6) with:

```cpp
std::vector<mgg::FrontierCluster> PlannerNode::tourCandidates(
    std::vector<mgg::FrontierCluster> clusters) {
  const std::vector<Eigen::Vector3d> reserved = selectionExclusions();
  clusters.erase(
      std::remove_if(clusters.begin(), clusters.end(),
                     [&](const mgg::FrontierCluster& cluster) {
                       return std::any_of(
                           reserved.begin(), reserved.end(),
                           [&](const Eigen::Vector3d& point) {
                             return (cluster.position - point).norm() <=
                                    reservation_exclusion_radius_m_;
                           });
                     }),
      clusters.end());
  const int own_id = static_cast<int>(planning_params_.robot_id);
  if (!fleet_) {
    const auto others = [own_id](const mgg::FrontierCluster& cluster) {
      return cluster.owner_robot_id != own_id;
    };
    if (!std::all_of(clusters.begin(), clusters.end(), others)) {
      clusters.erase(std::remove_if(clusters.begin(), clusters.end(), others),
                     clusters.end());
    }
    return clusters;
  }
  // Tour-exploration design §3.5: never a cluster another robot holds or a
  // peer explored; in a group with an award, the awarded bundle and the
  // frontiers this robot found itself that no award has named yet.
  const double now_s = now().seconds();
  const double radius = fleet_params_.cluster_merge_radius_m;
  const auto near = [radius](const mgg::FrontierCluster& cluster,
                             const std::vector<mgg::FleetCluster>& others) {
    return std::any_of(others.begin(), others.end(),
                       [&](const mgg::FleetCluster& other) {
                         return (other.position - cluster.position).norm() <=
                                radius;
                       });
  };
  const std::vector<mgg::FleetCluster> held = fleet_->claimedByOthers(now_s);
  const std::vector<mgg::FleetCluster>& bundle = fleet_->bundle();
  const std::vector<mgg::FleetCluster>& explored = fleet_->exploredElsewhere();
  const std::vector<mgg::FleetCluster>& awarded = fleet_->lastAwardClusters();
  const bool assigned = fleet_->inGroup(now_s) && fleet_->hasAward();
  clusters.erase(
      std::remove_if(clusters.begin(), clusters.end(),
                     [&](const mgg::FrontierCluster& cluster) {
                       if (near(cluster, explored)) return true;
                       if (near(cluster, bundle)) return false;
                       if (near(cluster, held)) return true;
                       return assigned && (cluster.owner_robot_id != own_id ||
                                           near(cluster, awarded));
                     }),
      clusters.end());
  tour_assignment_version_ = fleet_->assignmentVersion();
  return clusters;
}
```

In `PlannerNode::runGlobalPlanner`, replace the `searchGlobalFrontier` call:

```cpp
    const mgg::GlobalFrontierReport report = mgg::searchGlobalFrontier(
        *global_graph_, link_vertex->id,
        static_cast<int>(planning_params_.robot_id), globalFrontierGain(),
        selectionExclusions(), reservation_exclusion_radius_m_,
        exploration_target_.has_value() ? &*exploration_target_ : nullptr);
```

with:

```cpp
    // A peer's reservation, and with fleet assignment the clusters other
    // robots hold or peers explored (tour-exploration design §3.5): the
    // greedy fallback takes none of them either.
    std::vector<Eigen::Vector3d> excluded = selectionExclusions();
    const std::vector<Eigen::Vector3d> fleet_excluded = fleetExclusions();
    excluded.insert(excluded.end(), fleet_excluded.begin(),
                    fleet_excluded.end());
    const double exclusion_radius =
        fleet_ ? std::max(reservation_exclusion_radius_m_,
                          fleet_params_.cluster_merge_radius_m)
               : reservation_exclusion_radius_m_;
    const mgg::GlobalFrontierReport report = mgg::searchGlobalFrontier(
        *global_graph_, link_vertex->id,
        static_cast<int>(planning_params_.robot_id), globalFrontierGain(),
        excluded, exclusion_radius,
        exploration_target_.has_value() ? &*exploration_target_ : nullptr);
```

In `PlannerNode::onPlanRequest`, in the block Task 6 wrote, replace:

```cpp
    if (tour_decided) {
      // The tour decided this cycle.
    } else if (low_gain &&
```

with:

```cpp
    // Design §3.5: with fleet assignment, a robot with nothing on its tour
    // and no local path asks for an auction, or takes a silent robot's
    // claims over, before the greedy fallback is consulted.
    const bool fleet_decided =
        !tour_decided && fleet_ && tour_params_.enabled &&
        !tour_target.has_value() && best_path_.empty() &&
        local_graph_->getNumVertices() > 1 && !departed &&
        !boxed_in_without_departure_now_ && !withheld_without_departure &&
        settleIdleRobot(summary, complete);
    if (tour_decided || fleet_decided) {
      // The tour or the fleet decided this cycle.
    } else if (low_gain &&
```

- [ ] **Step 7: Run the tests to verify they pass**

Run: `/tmp/mgg-tour/tuf-run.sh on quick mgg_ros test_planner_node '*'`
Expected: every test `OK` (51 on 4777a17), including `APeersBidJoinsTheGroupOnlyWithATransformToIt`, `AMalformedBidIsIgnored`, `OnlyTheAuctioneerReleasesClaims`, `TheAuctioneerCallsAndAwardsOnItsFleetTimer` and `AnEmptyAwardAfterARebuildDroppedFrontiersIsNotYetComplete`. Without the two exceptions in `settleIdleRobot`, the last fails at its second plan with `-3` (complete) for `-2` (no path).

- [ ] **Step 8: Run both gates**

Run: `/tmp/mgg-tour/tuf-run.sh off test` then `/tmp/mgg-tour/tuf-run.sh on test`
Expected: both end with `0 errors, 0 failures`.

- [ ] **Step 9: Commit**

```bash
git add ros2/src/mgg_msgs/srv/ReleaseClaims.srv ros2/src/mgg_msgs/CMakeLists.txt \
        ros2/src/mgg_ros/include/mgg_ros/planner_node.h \
        ros2/src/mgg_ros/src/planner_node.cpp \
        ros2/src/mgg_ros/test/test_planner_node.cpp
git commit -m "Bid, auction and follow awards in the planner node"
```

---
### Task 12: Two planners explore one corridor together (integration test)

> **Apply by anchor.** This task adds a test binary and a CMake entry inside `mgg_ros`'s `if(mgg_map_octomap_WITH_OCTOMAP)` block, after `ament_target_dependencies(test_planner_node rclcpp mgg_msgs)` (present once on 4777a17). It changes no production code.

**Files:**
- Create: `ros2/src/mgg_ros/test/test_fleet_exploration.cpp`
- Modify: `ros2/src/mgg_ros/CMakeLists.txt` (one test, ON build only)

**Interfaces:**
- Consumes: everything from Tasks 1 to 11 through the real node: `PlannerNode` with `tour.*`/`fleet.*` parameters, the `tour_bid_*`/`tour_award_*`/`neighbour_graph_*` topics, `mgg_msgs/TourAward`.
- Produces: `test_fleet_exploration` (spec §6: "two or three planner nodes on a synthetic map with a shared frame, through the real message path; they split the frontiers and the map is fully explored").

The scene is built around how MGG ingests frontiers. `addFrontiers` keeps no frontier within 1.5 m of a visited vertex, and event E1 marks every vertex within 3 m of the track visited, so a robot's first frontiers lie ahead of it. The two robots therefore start back to back in the middle of a 20 m corridor, each half longer than E1's 3 m. The test world is scaled to the test robot (0.2 m body, 0.5 m lattice), so `fleet.cluster_merge_radius_m` is 0.5 there: at the default 2 m, clusters 1.7 m apart on either side of the start merge into one. The discriminating check is that each robot finishes its own half without crossing into the other's. With `fleet.enabled` false each robot, once none of its own clusters is left, tours the peer's (Task 6's `tourCandidates` takes other robots' clusters then) and drives into the other's half: its own map still shows that half unknown, and nothing tells it the peer holds or explored them. The owner's broadcasts demote a merged frontier only once the owner itself no longer marks it (89d3f6c), which does not keep the other robot out. Checked on 4777a17: fleet on, the test passes in about 8 s; fleet off (Step 3), it fails as described below.

- [ ] **Step 1: Write the test**

Create `ros2/src/mgg_ros/test/test_fleet_exploration.cpp`:

```cpp
// Two planners explore one corridor together through the real message path
// (tour-exploration design §6, the mgg_ros integration test): one shared
// frame, roadmaps, bids and awards over ROS topics, and a simulated lidar
// that maps what each robot drives past. They split the frontiers, and the
// corridor is fully explored.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "mgg_map_octomap/octomap_map.h"
#include "mgg_ros/planner_node.h"

namespace mgg_ros {

/// This binary's access to the node (test_planner_node.cpp has its own, with
/// more): the small ground robot and map helpers that test uses.
class PlannerNodeTestPeer {
 public:
  static void configureGroundRobot(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.robot_params_.type = mgg::RobotType::kGroundRobot;
    node.robot_params_.size = Eigen::Vector3d(0.20, 0.20, 0.15);
    node.robot_params_.size_extension.setZero();
    node.robot_params_.size_extension_min.setZero();
    node.robot_params_.safety_extension.setZero();
    node.robot_params_.bound_mode = mgg::BoundModeType::kExactBound;
    node.planning_params_.max_ground_height = 0.30;
    node.planning_params_.max_step_height = 0.10;
    node.planning_params_.max_inclination = 0.52;
    node.planning_params_.edge_length_min = 0.05;
    node.planning_params_.edge_length_max = 0.55;
    node.planning_params_.edge_overshoot = 0.0;
    node.planning_params_.num_vertices_max = 200;
    node.planning_params_.num_edges_max = 800;
    node.planning_params_.num_loops_max = 200;
    node.planning_params_.path_interpolation_distance = 0.10;
    node.planning_params_.traverse_length_max = 20.0;
    node.planning_params_.free_voxel_gain = 0.0;
    node.planning_params_.occupied_voxel_gain = 0.0;
    node.global_vertex_spacing_ = 0.50;
    node.grid_params_.min_val = Eigen::Vector3d(-1.0, -1.0, 0.0);
    node.grid_params_.max_val = Eigen::Vector3d(3.0, 1.0, 0.0);
    node.grid_params_.resolution = Eigen::Vector3d(0.50, 0.50, 0.20);
    node.global_space_.setBound(Eigen::Vector3d(-12.0, -4.0, -1.0),
                                Eigen::Vector3d(12.0, 4.0, 2.0));
    node.global_space_.min_extension.setZero();
    node.global_space_.max_extension.setZero();
    // A lidar seeing all round: frontiers on every side.
    mgg::SensorParams sensor;
    sensor.type = mgg::SensorType::kLidar;
    sensor.max_range = 2.0;
    sensor.fov = Eigen::Vector2d(2.0 * M_PI, 0.20);
    sensor.resolution = Eigen::Vector2d(M_PI / 8.0, 0.20);
    sensor.frontier_percentage_threshold = 0.01;
    sensor.update();
    node.sensors_["test_lidar"] = sensor;
    node.planning_params_.exp_sensor_list = {"test_lidar"};
    node.odometry_stale_s_ = 3600.0;
  }

  /// Mapped floor at z = 0 over [xmin, xmax] x [ymin, ymax], seen from
  /// above, with a free body volume over it.
  static void observeFloor(PlannerNode& node, double xmin, double xmax,
                           double ymin, double ymax) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    std::vector<Eigen::Vector3d> floor;
    for (double x = xmin; x <= xmax + 1e-9; x += 0.10) {
      for (double y = ymin; y <= ymax + 1e-9; y += 0.10) {
        floor.emplace_back(x, y, 0.0);
      }
    }
    for (int repeat = 0; repeat < 6; ++repeat) {
      for (const Eigen::Vector3d& p : floor) {
        node.cloud_map_->insertPointCloud({p},
                                          Eigen::Vector3d(p.x(), p.y(), 1.5));
      }
    }
    node.cloud_map_->augmentFreeBox(
        Eigen::Vector3d(0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.40),
        Eigen::Vector3d(xmax - xmin, ymax - ymin, 0.60));
    ++node.map_revision_;
  }

  /// Wall points, each seen from `origin_of(point)`.
  template <typename OriginFn>
  static void observeWall(PlannerNode& node,
                          const std::vector<Eigen::Vector3d>& points,
                          const OriginFn& origin_of) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (int repeat = 0; repeat < 6; ++repeat) {
      for (const Eigen::Vector3d& p : points) {
        node.cloud_map_->insertPointCloud({p}, origin_of(p));
      }
    }
    ++node.map_revision_;
  }

  static void acceptOdometryPose(PlannerNode& node,
                                 const geometry_msgs::msg::Pose& pose,
                                 double stamp_s) {
    auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    msg->header.stamp.sec = static_cast<std::int32_t>(stamp_s);
    msg->header.stamp.nanosec = static_cast<std::uint32_t>(
        (stamp_s - std::floor(stamp_s)) * 1e9);
    msg->pose.pose = pose;
    msg->pose.pose.position.z = 0.075;
    node.onOdometry(msg);
  }

  static void plan(PlannerNode& node,
                   std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response) {
    node.onPlanRequest(std::make_shared<mgg_msgs::srv::PlannerSrv::Request>(),
                       response);
  }
};

namespace {

// The corridor: floor at voxel centres from -9.95 to 9.95 along x and -0.65
// to 0.65 across, side walls at y = +-0.75, end walls at x = +-10.05. Each
// half is longer than the 3 m around its robot's track that odometry marks
// visited (event E1), so frontiers outlive a cycle and are auctioned.
constexpr double kFloorHalfLength = 9.95;
constexpr double kFloorHalfWidth = 0.65;
constexpr double kWallY = 0.75;
constexpr double kEndWallX = 10.05;
constexpr double kSensingRangeM = 2.0;
constexpr int kFloorColumns = 200;
constexpr int kMaxCycles = 80;

double voxelCentre(double v) { return std::floor(v * 10.0) / 10.0 + 0.05; }

/// What a robot at x sees: the floor and walls within kSensingRangeM along
/// the corridor. Records the floor columns seen.
void senseFrom(PlannerNode& node, double x, std::set<long>& seen) {
  const double x0 =
      std::max(-kFloorHalfLength, voxelCentre(x - kSensingRangeM));
  const double x1 = std::min(kFloorHalfLength, voxelCentre(x + kSensingRangeM));
  PlannerNodeTestPeer::observeFloor(node, x0, x1, -kFloorHalfWidth,
                                    kFloorHalfWidth);
  std::vector<Eigen::Vector3d> side;
  for (double wx = x0; wx <= x1 + 1e-9; wx += 0.1) {
    for (double z = 0.1; z <= 0.6 + 1e-9; z += 0.1) {
      side.emplace_back(wx, kWallY, z);
      side.emplace_back(wx, -kWallY, z);
    }
  }
  PlannerNodeTestPeer::observeWall(node, side, [](const Eigen::Vector3d& p) {
    return Eigen::Vector3d(p.x(), 0.0, p.z());
  });
  for (const double end : {-kEndWallX, kEndWallX}) {
    if (std::abs(end - x) > kSensingRangeM + 0.2) continue;
    std::vector<Eigen::Vector3d> wall;
    for (double wy = -kWallY; wy <= kWallY + 1e-9; wy += 0.1) {
      for (double z = 0.1; z <= 0.6 + 1e-9; z += 0.1) {
        wall.emplace_back(end, wy, z);
      }
    }
    PlannerNodeTestPeer::observeWall(node, wall,
                                     [end](const Eigen::Vector3d& p) {
                                       return Eigen::Vector3d(
                                           end - std::copysign(1.0, end),
                                           p.y(), p.z());
                                     });
  }
  for (double c = x0; c <= x1 + 1e-9; c += 0.1) {
    seen.insert(std::lround((c - 0.05) * 10.0));
  }
}

struct FleetRobot {
  std::shared_ptr<PlannerNode> node;
  double stamp = 1.0;
  bool complete = false;
  /// How far west and east it drove.
  double min_x = 0.0;
  double max_x = 0.0;
};

std::shared_ptr<PlannerNode> makeFleetNode(int robot_id) {
  rclcpp::NodeOptions options;
  options.arguments(
      {"--ros-args", "-r", "__node:=fleet_itest_" + std::to_string(robot_id),
       "-r", "tour_bid_out:=/fleet_itest/bids", "-r",
       "tour_bid_in:=/fleet_itest/bids", "-r",
       "tour_award_out:=/fleet_itest/awards", "-r",
       "tour_award_in:=/fleet_itest/awards", "-r",
       "neighbour_graph_out:=/fleet_itest/graphs", "-r",
       "neighbour_graph_in:=/fleet_itest/graphs"});
  options.parameter_overrides({
      rclcpp::Parameter("map.backend", "cloud_octomap"),
      rclcpp::Parameter("map.resolution", 0.10),
      rclcpp::Parameter("PlanningParams.global_frame_id", "world"),
      rclcpp::Parameter("PlanningParams.robot_id", robot_id),
      // One shared frame: every robot at a zero offset.
      rclcpp::Parameter("neighbour_offsets",
                        std::vector<double>{1, 0, 0, 0, 2, 0, 0, 0}),
      rclcpp::Parameter("graph_publish_period_sec", 0.5),
      // In radio range from end to end of the corridor.
      rclcpp::Parameter("communication_range", 50.0),
      // This world is scaled to a 0.2 m robot on a 0.5 m lattice: clusters
      // a lattice cell apart are distinct places.
      rclcpp::Parameter("fleet.cluster_merge_radius_m", 0.5),
      rclcpp::Parameter("tour.min_cluster_gain", 0.0),
      rclcpp::Parameter("tour.recompute_interval_s", 0.0),
      rclcpp::Parameter("fleet.auction_interval_s", 0.3),
      rclcpp::Parameter("fleet.bid_deadline_s", 0.1),
  });
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<PlannerNode>(options);
  PlannerNodeTestPeer::configureGroundRobot(*node);
  return node;
}

}  // namespace

TEST(FleetExploration, TwoPlannersSplitTheFrontiersAndExploreTheCorridor) {
  rclcpp::init(0, nullptr);
  std::array<FleetRobot, 2> robots;
  robots[0].node = makeFleetNode(1);
  robots[1].node = makeFleetNode(2);
  auto listener = std::make_shared<rclcpp::Node>("fleet_itest_listener");
  std::mutex awards_mutex;
  std::vector<mgg_msgs::msg::TourAward> awards;
  auto award_sub = listener->create_subscription<mgg_msgs::msg::TourAward>(
      "/fleet_itest/awards", rclcpp::QoS(100),
      [&](mgg_msgs::msg::TourAward::ConstSharedPtr msg) {
        const std::lock_guard<std::mutex> lock(awards_mutex);
        if (!msg->call) awards.push_back(*msg);
      });
  rclcpp::executors::MultiThreadedExecutor executor;
  for (const FleetRobot& robot : robots) executor.add_node(robot.node);
  executor.add_node(listener);
  std::thread spinner([&executor] { executor.spin(); });

  // Back to back in the middle of the corridor: robot 1 facing east, robot
  // 2 west. MGG adds no frontier within 1.5 m of a robot's track to its
  // roadmap, so each first finds frontiers ahead of it.
  std::set<long> seen;
  const std::array<double, 2> start_x{0.35, -0.35};
  for (std::size_t k = 0; k < robots.size(); ++k) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = start_x[k];
    pose.orientation.z = k == 0 ? 0.0 : 1.0;  // yaw 0 or pi
    pose.orientation.w = k == 0 ? 1.0 : 0.0;
    robots[k].min_x = robots[k].max_x = start_x[k];
    senseFrom(*robots[k].node, start_x[k], seen);
    PlannerNodeTestPeer::acceptOdometryPose(*robots[k].node, pose,
                                            robots[k].stamp);
    robots[k].stamp += 1.0;
  }
  // The planners hear each other and share their roadmaps.
  std::this_thread::sleep_for(std::chrono::seconds(2));

  int cycles = 0;
  for (; cycles < kMaxCycles && !(robots[0].complete && robots[1].complete);
       ++cycles) {
    for (FleetRobot& robot : robots) {
      if (robot.complete) continue;
      auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
      PlannerNodeTestPeer::plan(*robot.node, response);
      if (response->status == PlannerNode::kStatusComplete) {
        robot.complete = true;
        continue;
      }
      if (response->status != mgg_msgs::srv::PlannerSrv::Response::FORWARD ||
          response->path.empty()) {
        continue;
      }
      // Drive the path, mapping as it goes.
      double sensed_at = response->path.front().position.x;
      for (const geometry_msgs::msg::Pose& pose : response->path) {
        PlannerNodeTestPeer::acceptOdometryPose(*robot.node, pose,
                                                robot.stamp);
        robot.stamp += 0.2;
        robot.min_x = std::min(robot.min_x, pose.position.x);
        robot.max_x = std::max(robot.max_x, pose.position.x);
        if (std::abs(pose.position.x - sensed_at) >= 0.5) {
          senseFrom(*robot.node, pose.position.x, seen);
          sensed_at = pose.position.x;
        }
      }
      senseFrom(*robot.node, response->path.back().position.x, seen);
    }
    // Robots drive between plans, and the fleet assigns between drives:
    // wait for the next award (or two seconds) before planning again.
    std::size_t awards_before = 0;
    {
      const std::lock_guard<std::mutex> lock(awards_mutex);
      awards_before = awards.size();
    }
    const auto waited_from = std::chrono::steady_clock::now();
    for (;;) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      const std::lock_guard<std::mutex> lock(awards_mutex);
      if (awards.size() > awards_before ||
          std::chrono::steady_clock::now() - waited_from >
              std::chrono::seconds(2)) {
        break;
      }
    }
  }
  executor.cancel();
  spinner.join();

  EXPECT_TRUE(robots[0].complete) << "after " << cycles << " cycles";
  EXPECT_TRUE(robots[1].complete) << "after " << cycles << " cycles";
  EXPECT_EQ(seen.size(), static_cast<std::size_t>(kFloorColumns));
  // Each explored its own half: neither toured the frontiers the other held
  // or had explored, though its own map still shows them unknown.
  EXPECT_GT(robots[0].min_x, -1.5) << "robot 1 went west to " << robots[0].min_x;
  EXPECT_LT(robots[1].max_x, 1.5) << "robot 2 went east to " << robots[1].max_x;
  const std::lock_guard<std::mutex> lock(awards_mutex);
  ASSERT_FALSE(awards.empty());
  // The auction split them: no cluster in two bundles, and every cluster a
  // bidder won lies in its own half.
  std::size_t won = 0;
  for (const mgg_msgs::msg::TourAward& award : awards) {
    std::set<std::uint64_t> named;
    for (const mgg_msgs::msg::TourBundle& bundle : award.bundles) {
      for (const std::uint64_t id : bundle.clusters) {
        EXPECT_TRUE(named.insert(id).second)
            << "cluster " << id << " in two bundles of award "
            << award.auction_id;
        if (bundle.silent_s > 0.0) continue;
        for (const mgg_msgs::msg::TourCluster& cluster : award.clusters) {
          if (cluster.id != id) continue;
          ++won;
          if (bundle.robot_id == 1) {
            EXPECT_GT(cluster.position.x, -1.5);
          }
          if (bundle.robot_id == 2) {
            EXPECT_LT(cluster.position.x, 1.5);
          }
        }
      }
    }
  }
  EXPECT_GT(won, 0u);
  robots[0].node.reset();
  robots[1].node.reset();
  rclcpp::shutdown();
}

}  // namespace mgg_ros
```

Register it in `ros2/src/mgg_ros/CMakeLists.txt`, inside `if(mgg_map_octomap_WITH_OCTOMAP)`, after `ament_target_dependencies(test_planner_node rclcpp mgg_msgs)`:

```cmake
    # Two planners exploring one corridor over real topics: seconds, not
    # milliseconds.
    ament_add_gtest(test_fleet_exploration test/test_fleet_exploration.cpp
      TIMEOUT 600)
    target_link_libraries(test_fleet_exploration ${PROJECT_NAME})
    ament_target_dependencies(test_fleet_exploration rclcpp mgg_msgs)
```

- [ ] **Step 2: Run it**

Run: `/tmp/mgg-tour/tuf-run.sh on quick mgg_ros test_fleet_exploration '*'`
Expected: `[       OK ] FleetExploration.TwoPlannersSplitTheFrontiersAndExploreTheCorridor` in about 10 s. Run it three times; it must pass each time.

This test passes on correct code as written. If it fails, read the `plan request:` summaries (`tour: ...`, `its bundle is done ...`) and the auctioneer's `fleet award N: ...` lines in its output to find the fault in Tasks 6 to 11. Do not loosen its assertions without the supervisor's approval.

Known on 4777a17 (refresh, 2026-09-26): about 40 runs on tuf while the SwarmDeck stack was running there, with `--cpus 4`. Three failed, all with one signature: one or two awards in mid-run (the fifth and sixth) gave robot 1 robot 2's frontiers at x = -4.85 or -6.35. The next award corrected it, and robot 1 never drove there, so only the award check `cluster.position.x > -1.5` failed. An instrumented run showed the cause. Those awards name one robot (`1 cluster(s) to 1 robot(s)`): robot 2's bid missed the deadline, and its previous bundle was empty. The only other listing of its fresh frontiers was robot 1's merged copy of robot 2's roadmap, so robot 1 won them. That is the spec's rule (§3.4: a late bidder keeps only its previous bundle). In this test, `fleet.bid_deadline_s` (0.1 s) equals the node's `kFleetTickPeriodS` (0.1 s), so a peer that ticks just after the call answers at the deadline. A deadline of a few tick periods (for example 0.3 s) would likely remove the flake, but that changes the test. If the test fails with this signature, report it to the supervisor; do not change the deadline or the assertion on your own.

- [ ] **Step 3: Check that it tests the fleet**

Temporarily add `rclcpp::Parameter("fleet.enabled", false),` to `makeFleetNode`'s overrides, and change `ASSERT_FALSE(awards.empty());` to `EXPECT_TRUE(awards.empty());`, then run it again.
Expected: FAIL with `robot 1 went west to ...` and `robot 2 went east to ...` (each robot crosses into the other's half; on 4777a17 to -8.35 and 8.35), `won` is 0, and neither robot completes within `kMaxCycles` (it loops over the peer's frontiers, so this run takes about 3 minutes). Revert both edits (`git diff ros2/src/mgg_ros/test/test_fleet_exploration.cpp` shows only the new file as created in Step 1).

- [ ] **Step 4: Run both gates**

Run: `/tmp/mgg-tour/tuf-run.sh off test` then `/tmp/mgg-tour/tuf-run.sh on test`
Expected: both end with `0 errors, 0 failures` (the OFF build does not build this test).

- [ ] **Step 5: Commit**

```bash
git add ros2/src/mgg_ros/test/test_fleet_exploration.cpp ros2/src/mgg_ros/CMakeLists.txt
git commit -m "Test two planners splitting one corridor over the real message path"
```

Delivery step 2 is complete here.

---
### Task 13: Measure in the SubT simulation and record the tuned values

**Files:**
- Modify: `ros2/docs/2026-09-25-tour-exploration-design.md` (§5 table: the three "tuned" entries; a new §5.1 tuning record)
- Modify: `ros2/src/mgg_core/include/mgg_core/tour_params.h` (the three defaults)
- Modify: `ros2/src/mgg_core/test/test_tour_params.cpp` (`DefaultsAreTheDesignTable`)
- Outside this repository, not committed: a scratch SwarmDeck worktree for the runs

**Interfaces:**
- Consumes: the whole feature (Tasks 1 to 12), SwarmDeck's `scripts/sim-up` (`--scenario subt_finals`, `--robot-poses`, `--explore`), its MGG image (`deploy/docker/Dockerfile.mgg`) and its MGG launch (`deploy/mgg/robot.launch.py`, mounted into the MGG container from the SwarmDeck checkout).
- Produces: measured values for `tour.min_cluster_gain`, `tour.heading_weight` and `fleet.balance_weight`, written into the spec and the struct defaults; the spec's §1 success metrics for the tour alone (delivery step 1) and with fleet assignment (delivery step 2).

Starting values (Task 1): `tour.min_cluster_gain` 600, `tour.heading_weight` 2.0, `fleet.balance_weight` 0.3. Candidates: gain {300, 600, 1200}, heading {1.0, 2.0, 4.0}, balance {0.1, 0.3, 0.6}. Decision rule, fixed before the runs: the candidate with the highest fraction of reachable space explored at 20 minutes wins; within 2 percentage points, the one with fewer metres driven over already-explored space wins. A candidate whose median plan time exceeds 350 ms is out.

- [ ] **Step 1: Build SwarmDeck's MGG image from this branch**

The simulation's MGG image clones MGGPlanner from GitHub at `MGG_REV` (`deploy/docker/Dockerfile.mgg`); this branch is not pushed. Find the image name `sim-up` uses for the `mgg` service, then build an image of that name from this branch's sources on top of the existing one:

```bash
cd ~/Projects/swarmdeck
./scripts/sim-up --scenario subt_finals --robot-poses ground_truth --dry-run | grep -i mgg
```

Save as `/tmp/mgg-tour/Dockerfile.mgg-tour`, with `SIM_MGG_IMAGE` the image name found above:

```dockerfile
ARG SIM_MGG_IMAGE
FROM ${SIM_MGG_IMAGE}
SHELL ["/bin/bash", "-c"]
# This branch's planner in place of the pinned revision, built as
# Dockerfile.mgg builds it.
RUN rm -rf /opt/mgg/ros2
COPY ros2 /opt/mgg/ros2
WORKDIR /opt/mgg/ros2
RUN source /opt/ros/jazzy/setup.bash && rm -rf build install log && \
    CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-up-to mgg_ros mgg_pci \
    --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DMGG_WITH_OCTOMAP=OFF
```

```bash
cd "$(git rev-parse --show-toplevel)"
docker tag "$SIM_MGG_IMAGE" "${SIM_MGG_IMAGE%:*}:before-tour"
docker build -f /tmp/mgg-tour/Dockerfile.mgg-tour --build-arg SIM_MGG_IMAGE="$SIM_MGG_IMAGE" -t "$SIM_MGG_IMAGE" .
```

Run this on the host that runs the simulation. If you do not know that host, ask the supervisor through `contact_supervisor` before building, and do not push the branch without the supervisor's approval.

- [ ] **Step 2: Prepare a scratch SwarmDeck for the runs**

```bash
cd ~/Projects/swarmdeck
git worktree add /tmp/mgg-tour/swarmdeck-runs HEAD
```

In `/tmp/mgg-tour/swarmdeck-runs/deploy/mgg/robot.launch.py`, route the fleet topics as the roadmaps are routed, and read the candidate values from the environment. After `ROADMAP_REMAPS = [...]` add:

```python
# Fleet frontier assignment (MGG tour-exploration design §3): every planner
# publishes and reads bids and awards on one shared topic each.
FLEET_REMAPS = [
    ("tour_bid_out", "/mgg/tour_bids"),
    ("tour_bid_in", "/mgg/tour_bids"),
    ("tour_award_out", "/mgg/tour_awards"),
    ("tour_award_in", "/mgg/tour_awards"),
]
TOUR_RUN_PARAMETERS = {
    "tour.enabled": os.environ.get("MGG_TOUR_ENABLED", "true") == "true",
    "fleet.enabled": os.environ.get("MGG_FLEET_ENABLED", "true") == "true",
    "tour.min_cluster_gain": float(os.environ.get("MGG_TOUR_MIN_CLUSTER_GAIN", "600")),
    "tour.heading_weight": float(os.environ.get("MGG_TOUR_HEADING_WEIGHT", "2.0")),
    "fleet.balance_weight": float(os.environ.get("MGG_FLEET_BALANCE_WEIGHT", "0.3")),
    "fleet.claim_ttl_s": 600.0,
}
```

After `overrides.update(planner_overrides or {})` add `overrides.update(TOUR_RUN_PARAMETERS)`, and change the planner node's remappings to `tf_remaps + [("odometry", odom)] + ROADMAP_REMAPS + FLEET_REMAPS`. Pass the `MGG_*` variables to the `mgg` service by adding them to its `environment:` in `/tmp/mgg-tour/swarmdeck-runs/deploy/compose/docker-compose.yml`, for example `MGG_TOUR_MIN_CLUSTER_GAIN: "${MGG_TOUR_MIN_CLUSTER_GAIN:-600}"`, and the same for the other four.

- [ ] **Step 3: Run the baseline and phase A (the tour alone, delivery step 1)**

Each run starts fresh, explores for 30 minutes, and is kept (mission ID and logs) for the metrics:

```bash
cd /tmp/mgg-tour/swarmdeck-runs
# Baseline: today's behaviour, for the comparison with run 5.
MGG_TOUR_ENABLED=false MGG_FLEET_ENABLED=false \
  ./scripts/sim-up --scenario subt_finals --robot-poses ground_truth --no-build --explore 1800
# Phase A, one run per candidate, fleet off:
for gain in 300 600 1200; do
  MGG_FLEET_ENABLED=false MGG_TOUR_MIN_CLUSTER_GAIN=$gain \
    ./scripts/sim-up --scenario subt_finals --robot-poses ground_truth --no-build --explore 1800
done
# Then, with the winning gain G:
for heading in 1.0 4.0; do
  MGG_FLEET_ENABLED=false MGG_TOUR_MIN_CLUSTER_GAIN=G MGG_TOUR_HEADING_WEIGHT=$heading \
    ./scripts/sim-up --scenario subt_finals --robot-poses ground_truth --no-build --explore 1800
done
```

Between runs, `./scripts/sim-up --down`. `--robot-poses ground_truth` gives the planners a shared frame from the start: inter-robot C-SLAM closures are off in the simulation until SwarmDeck's delivery step 3, so the spec's "with C-SLAM merges on" measurement moves to that step.

- [ ] **Step 4: Run phase B (fleet assignment, delivery step 2)**

```bash
for balance in 0.1 0.3 0.6; do
  MGG_TOUR_MIN_CLUSTER_GAIN=G MGG_TOUR_HEADING_WEIGHT=H MGG_FLEET_BALANCE_WEIGHT=$balance \
    ./scripts/sim-up --scenario subt_finals --robot-poses ground_truth --no-build --explore 1800
done
```

with G and H the phase A winners. SwarmDeck still passes its reservation leases in these runs; note that in the record.

- [ ] **Step 5: Compute the metrics**

From each run's MGG log (`docker compose logs mgg` in the scratch worktree, or the saved session logs):

```bash
# Median plan time (ms), from every "plan request:" summary.
grep -o '; [0-9]* ms (global' mgg.log | grep -o '[0-9]*' | sort -n | awk '{a[NR]=$1} END {print a[int((NR+1)/2)]}'
# Longest tour costing and solve (ms).
grep -o 'costed and solved in [0-9.]* ms' mgg.log | grep -o '[0-9.]*' | sort -n | tail -1
# Auctions leaving clusters unassigned, and robots waiting with nothing.
grep -E 'fleet award [0-9]+: .* [1-9][0-9]* unassigned' mgg.log
grep -E 'its bundle is done|exploration complete for this robot' mgg.log
```

"No frontier cluster stays unassigned for more than about 2 minutes while a robot idles": from the timestamps of those two greps, the longest span in which awards reported unassigned clusters while some robot logged that its bundle was done. The fraction of reachable space explored over time (and the times to 90 % and 99 %) and the metres driven over already-explored space (own or peer) are computed the way the run-5 figures were. If that procedure is not written down in SwarmDeck's `docs/operations`, ask the supervisor for it through `contact_supervisor` before computing them.

- [ ] **Step 6: Record the values**

In `ros2/docs/2026-09-25-tour-exploration-design.md` §5, replace `tuned` in the three rows with the chosen values, and after the table add:

```markdown
### 5.1 Tuning record

Measured on the SwarmDeck SubT finals simulation, 4 robots,
`--robot-poses ground_truth`, 30 min per run, fleet.claim_ttl_s 600,
SwarmDeck reservation leases still on; <date>, MGG <commit>.

| Run | Parameters | Explored at 20 min | 90 % at | 99 % at | Re-driven m | Median plan ms | Longest unassigned while idle |
|---|---|---|---|---|---|---|---|
| baseline | tour off, fleet off | | | | | | |
| A1 | min_cluster_gain 300 | | | | | | |
| ... | | | | | | | |

Chosen: tour.min_cluster_gain <G>, tour.heading_weight <H>, fleet.balance_weight <B>.
```

Fill every cell from Step 5 (a metric that could not be computed is written `not measured`, with the reason under the table). Set the same three values as the defaults in `ros2/src/mgg_core/include/mgg_core/tour_params.h` (and their comments), and in `TourParams.DefaultsAreTheDesignTable` / `FleetParams.DefaultsAreTheDesignTable` in `ros2/src/mgg_core/test/test_tour_params.cpp`.

- [ ] **Step 7: Run both gates**

Run: `/tmp/mgg-tour/tuf-run.sh off test` then `/tmp/mgg-tour/tuf-run.sh on test`
Expected: both end with `0 errors, 0 failures`.

- [ ] **Step 8: Commit, and clean up**

```bash
git add ros2/docs/2026-09-25-tour-exploration-design.md \
        ros2/src/mgg_core/include/mgg_core/tour_params.h \
        ros2/src/mgg_core/test/test_tour_params.cpp
git commit -m "Record the tour and fleet values tuned in the SubT simulation"
```

Restore the simulation's MGG image (`docker tag "${SIM_MGG_IMAGE%:*}:before-tour" "$SIM_MGG_IMAGE"`), remove the scratch worktree (`git -C ~/Projects/swarmdeck worktree remove /tmp/mgg-tour/swarmdeck-runs`), and the build scratch (`ssh -o BatchMode=yes tuf rm -rf /tmp/mgg-tour`, `rm -rf /tmp/mgg-tour`).

---

## Out of scope: delivery step 3 (SwarmDeck)

Delivery step 3 is SwarmDeck work, planned separately. It covers: inter-robot C-SLAM closures in the simulation; a `deployment` neighbour-transform source in `deploy/mgg/robot_poses.py`; `fleet.claim_ttl_s: 600` for the SubT simulation; no reservation leases for MGG exploration targets (leases stay for operator goals and Return Home); and showing each robot's tour (MGG's latched `tour` topic, Task 6) and assigned clusters (`TourAward` bundles). The spec's list also omits two items this plan relies on:
- remapping `tour_bid_out`/`tour_bid_in` and `tour_award_out`/`tour_award_in` to one shared topic each, as Task 13's scratch launch does and as `ROADMAP_REMAPS` does for roadmaps;
- exposing MGG's `release_claims` service to the operator.
