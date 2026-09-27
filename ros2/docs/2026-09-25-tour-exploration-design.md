# Tour-based exploration and fleet frontier assignment

Status: design, awaiting operator review. Date: 2026-09-25.
Scope: MGGPlanner `ros2` branch (native code, not a downstream patch). SwarmDeck consumes it.

## 1. Problem and goal

### 1.1 The problem today (MGG b977bfc, before this design)

Observed in SwarmDeck's simulated SubT runs 4 and 5:

- Clear frontiers remain in the map while robots linger in explored areas.
- Robots work off leftover local gain and only reposition globally after
  `low_gain_rounds` (3) of low local gain; the global planner then picks one
  frontier greedily (`searchGlobalFrontier`: best gain over distance). There is
  no ordering of frontiers and no commitment to a plan.
- Today, nothing splits the frontiers among robots. Each robot chooses its next
  frontier as if it were alone; the only coordination is SwarmDeck's reservation
  lease, which keeps other robots away from the one spot a robot is heading to
  (within a radius). Several robots can head into the same region while another
  region has nobody.

### 1.2 The goal

Each robot orders its frontiers into a tour and commits to it (§2), and each
connected group of robots splits its frontiers among its members by auction
(§3), so that the robots explore the whole reachable environment, spread
across it, and stop revisiting ground that is already explored, by themselves
or by a peer.

Success means, on the 4-robot SubT simulation with C-SLAM merges on:

- the fraction of reachable space explored rises faster, and reaches 90 % / 99 %;
- metres driven over already-explored space (own or peer) drop clearly versus run 5;
- no frontier cluster stays unassigned for more than about 2 minutes while a robot idles;
- median plan time stays under about 350 ms.

Decisions taken with the operator:

- Both layers: a per-robot tour, and fleet assignment on top.
- Fleet assignment lives entirely inside MGG, independent of SwarmDeck, with no
  central server: each connected group of robots runs its own.
- Shared frame: the C-SLAM component when robots share one; otherwise the
  surveyed deployment frame; otherwise the robot tours alone. MGG does not choose
  the source: it uses whatever neighbour transforms it receives.
- Approach: a frontier auction per group, run by the group's lowest robot ID
  (operator decision: with a handful of robots it is quick, and a single
  auctioneer makes one decision nobody can disagree with). Bids are sent in one
  round; the auctioneer runs the sequential auction locally. Considered and
  dropped: replicated deterministic assignment with a claim rule (robots can
  disagree when their pictures differ), multi-round auctions or consensus
  bidding (one message exchange per cluster, fragile over cave radio), and static
  region partition (ignores where frontiers are).
- Disconnected robots keep their work for a long time: claims survive silence.

## 2. Per-robot tour

### 2.1 Clusters
- The tour is over **frontier clusters** of the global graph (frontier vertices
  with gain). MGG already assigns `cluster_id` to global frontier vertices
  (`global_graph.cpp`, path-similarity clustering); a cluster's representative is
  its best-gain member.
- Clusters below `tour.min_cluster_gain` are dropped.
- Each cluster gets a **stable ID** derived from the quantized position of its
  representative (grid of `tour.cluster_id_cell_m`) plus the owning robot's ID,
  so it survives graph revisions and can be named in fleet messages.

### 2.2 Costs
- Robot-to-cluster and cluster-to-cluster costs are shortest-path lengths on the
  global graph (one Dijkstra per cluster, cached per graph revision).
- A heading-change penalty `tour.heading_weight` (per radian) applies to the
  first leg, so the tour does not start with a U-turn when a cluster lies ahead.
- Clusters unreachable in the graph are left out until they connect.

### 2.3 Solving
- Open tour from the robot (no return). Greedy nearest-neighbour start, improved
  with 2-opt and Or-opt until no improving move. Target: < 10 ms for 50 clusters.
- Recompute when the graph revision changes, a cluster appears or disappears, or
  the assignment (§3) changes; at most every `tour.recompute_interval_s`.

### 2.4 Use
- The robot's **current target** is the first cluster on its tour.
- If the local lattice has gain in the direction of the target, the local
  planner picks viewpoints along the way (today's local exploration, unchanged).
  Otherwise the robot takes the global route to the target.
- This replaces the `low_gain_rounds` trigger with "the current target is outside
  the local lattice's reach or the lattice has no gain toward it".
- Commitment: the target is kept until reached, explored (no longer a frontier),
  or reassigned. A new first cluster replaces it only if it lowers the remaining
  cost by more than `tour.commit_margin` (fraction).
- Unchanged: local exploration, terrain and turn checks, boxed-in departures,
  and the run-5 fixes. The tour only decides where the robot goes next.

## 3. Fleet assignment: frontier auction (inside MGG)

### 3.1 Groups and the auctioneer
- A robot's **group** is the set of robots it currently hears (a `TourBid` or
  `TourAward` within `fleet.peer_timeout_s`) and holds a neighbour transform to
  (the same condition under which roadmaps merge today).
- The **auctioneer** is the robot with the lowest ID in the group. If it falls
  silent, the next-lowest takes over at the next auction. Two groups out of
  contact each have their own auctioneer; on reconnection the lowest ID of the
  merged group runs one auction that settles everything.

### 3.2 When an auction runs
The auctioneer starts one when a frontier cluster appears or disappears in the
pool, a robot joins or leaves the group, or a robot completes its bundle; at most
every `fleet.auction_interval_s`. A robot can request one (e.g. on finishing its
bundle) through its bid.

### 3.3 Bids: one round
On an auction call (`TourAward` with `call=true`, or its own periodic bid) each
robot sends one `mgg_msgs/TourBid`:
- `robot_id`, `seq`, `stamp`, `auction_id`;
- `pose` in its own frame (the auctioneer transforms with the neighbour transform);
- `clusters[]`: the clusters it knows (stable ID, representative point in its own
  frame, gain);
- `costs`: its cost from its pose to each pool cluster and between pool clusters
  (shortest paths on its own global graph; unreachable = infinite; clusters not in
  its graph are estimated on the merged roadmap);
- `current_target`, `claim_stamp`, `bundle[]` (its current bundle).

Pool construction at the auctioneer: clusters within `fleet.cluster_merge_radius_m`
are one; a cluster lying in explored space of any robot in the group is dropped.

### 3.4 Award
The auctioneer computes, locally:
1. Each robot keeps its current target unless another robot's cost to it is lower
   by more than `tour.commit_margin`.
2. Sequential single-item auction over the remaining clusters: in each round every
   robot's bid for each unassigned cluster is its marginal tour cost (insertion
   cost into its current bundle's tour) plus `fleet.balance_weight ×` its bundle's
   tour cost; the lowest bid wins; bundles update; repeat until all are assigned.
   Ties: lower robot ID.
3. Silent robots' kept claims (§4) are included as fixed bundles: nobody else
   gets them.

It broadcasts `mgg_msgs/TourAward` (`auction_id`, `auctioneer_id`, `stamp`, and per
robot its bundle in tour order). Robots that bid on time get bundles; a robot
whose bid missed `fleet.bid_deadline_s` keeps its previous bundle, which stays
excluded for others.

### 3.5 Robots between auctions
Each robot tours its awarded bundle (§2). Frontiers it discovers itself are added
to its own bundle and reported in its next bid. A robot that misses an award keeps
its previous bundle until the next one. A robot with an empty bundle bids and
thereby requests an auction; if the award still leaves it empty, it takes over
silent peers' claims as in §4, else reports exploration complete for itself.

### 3.6 SwarmDeck leases
MGG no longer needs SwarmDeck's reservation leases for its own exploration
targets. SwarmDeck stops passing them as exclusions when MGG fleet assignment is
on, and keeps leases for operator goals and Return Home.

`Graph.msg` is unchanged, so MGG versions without this feature still interoperate
(they simply bid nothing and are treated as solo).

## 4. Disconnection, reconnection, no shared frame

- **Silence is not failure.** A silent peer's last bid and award stay valid: its
  assigned clusters and current target stay excluded for others. The silent
  robot keeps touring its last assignment plus frontiers it discovers itself.
- Claims expire after `fleet.claim_ttl_s`: default **1800 s** in MGG; SwarmDeck's
  simulated SubT configuration sets **600 s**. Other missions set their own.
- Earlier release of a silent robot's claims:
  1. a robot in the group observes the cluster explored (it simply stops being
     a frontier);
  2. robots are idle with no unclaimed clusters left: the oldest silent claims
     are released first, so idle robots can take them over;
  3. operator release (a service `release_claims(robot_id)` on the auctioneer,
     forwarded in the next award).
- **Reconnection:** roadmaps merge as usual; the merged group's lowest ID runs an
  auction with everyone's bids; clusters explored by both sides vanish;
  overlapping bundles are settled by the award (the commitment rule of §3.4
  applies to current targets).
- **No shared frame:** the robot tours alone (§2). When a transform to a peer
  appears, its clusters and claims join the pool.
- Two groups out of contact auction independently; brief overlap at the
  boundary is accepted and resolved by the first joint auction.

## 5. Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `tour.enabled` | true | Use the tour instead of low-gain-triggered greedy repositioning |
| `tour.min_cluster_gain` | 1200 | Drop clusters below this gain |
| `tour.cluster_id_cell_m` | 1.0 | Quantization for stable cluster IDs |
| `tour.heading_weight` | 2.0 | Cost per radian of first-leg heading change |
| `tour.recompute_interval_s` | 1.0 | Minimum interval between tour solves |
| `tour.commit_margin` | 0.2 | Fractional cost gain needed to switch target |
| `fleet.enabled` | true | Bid, run and follow frontier auctions |
| `fleet.cluster_merge_radius_m` | 2.0 | Clusters closer than this are one |
| `fleet.balance_weight` | 0.6 | Balance penalty on bundle tour cost |
| `fleet.auction_interval_s` | 2.0 | Minimum interval between auctions |
| `fleet.bid_deadline_s` | 1.0 | How long the auctioneer waits for bids |
| `fleet.peer_timeout_s` | 5.0 | A robot not heard for this long leaves the group (its claims stay, see TTL) |
| `fleet.claim_ttl_s` | 1800 | Claim lifetime of a silent peer (SwarmDeck SubT sim: 600) |

### 5.1 Tuning record

**Chosen on exploration metrics; plan-time gate deferred, not passed.**
Measured on 2026-09-27 in SwarmDeck's SubT finals simulation, four robots,
30 simulated minutes per run, `--render gpu --odometry drift
--robot-poses ground_truth`, `fleet.claim_ttl_s=600`, reservation leases
still on. MGG `81e33447db01c729491a8277963b5861873e8ff3`; SwarmDeck
`48c0c9ef304e8d94cc98084e4b1097bb76638c1d`. Inter-robot C-SLAM merges
remain deferred to delivery step 3.

The original rule excluded median plan times above 350 ms. The baseline
and all three gain candidates exceeded it. The supervisor deferred that
exclusion for this tuning, attributing it to the base planner's duplicated,
size-limited lattice (the separate run-8 fixes lane). **Re-check the timing
gate after those fixes merge.** A4 subsequently measured 341 ms, but three
of its robots spent most of the run stationary; that is not evidence that
the feature generally meets the timing goal.

The remaining rule was fixed before comparison: highest coverage at
20 minutes; within two percentage points, fewer re-driven metres.
Gain candidates were 300/600/1200 at heading 2, then headings 1/2/4 at
gain 1200, then balance weights 0.1/0.3/0.6 at gain 1200, heading 2.

| Run | Parameters | Explored at 20 min | 90 % at | 99 % at | Re-driven m | Median plan ms | Longest unassigned while idle |
|---|---|---:|---|---|---:|---:|---|
| baseline | tour off, fleet off | 62.03% | not reached | not reached | 1345.9 | 697 (contended) | not applicable: fleet off |
| A1 | gain 300, heading 2, fleet off | 49.45% | not reached | not reached | 640.8 | 581 | not applicable: fleet off |
| A2 | gain 600, heading 2, fleet off | 42.24% | not reached | not reached | 1687.9 | 723 | not applicable: fleet off |
| A3 | gain 1200, heading 2, fleet off | 65.18% | not reached | not reached | 798.1 | 740 | not applicable: fleet off |
| A4 | gain 1200, heading 1, fleet off | 49.99% | not reached | not reached | 143.9 | 341 | not applicable: fleet off |
| A5 | gain 1200, heading 4, fleet off | 52.78% | not reached | not reached | 2003.8 | 795 | not applicable: fleet off |
| B1 | gain 1200, heading 2, balance 0.1 | 40.24% | not reached | not reached | 965.5 | 625 | 0 s logged overlap* |
| B2 | gain 1200, heading 2, balance 0.3 | 46.19% | not reached | not reached | 1608.6 | 783 | 0 s logged overlap* |
| B3 | gain 1200, heading 2, balance 0.6 | 48.27% | not reached | not reached | 582.3 | 574 | 0 s logged overlap* |
| baseline repeat | tour off, fleet off; uncontended | 33.28% | not reached | not reached | 1396.3 | 613 | not applicable: fleet off |

Use **613 ms** as the uncontended baseline timing reference. Its exploration
outcome differs materially from the original baseline, so both observations
are retained rather than combining one run's coverage with another's timing.
The original baseline had intermittent colcon/test contention; candidate
runs and the repeat had none observed. Every run's early real-time factor
was at least 0.97. Whole-run factors were 0.890 for B2, 0.990 for A5, and
approximately 1.000 otherwise. All windows used `/clock`, not wall time.

| Run | Coverage at 5 / 10 / 30 min | Longest tour costing + solve ms | Re-driven m, robots 0 / 1 / 2 / 3 |
|---|---|---:|---|
| baseline | 26.02 / 41.07 / 64.31% | not applicable: tour off | 156.1 / 401.8 / 384.2 / 403.9 |
| A1 | 20.82 / 45.83 / 49.45% | 92.6 | 60.0 / 351.0 / 134.5 / 95.3 |
| A2 | 18.79 / 33.80 / 55.37% | 168.2 | 90.7 / 554.8 / 665.7 / 376.7 |
| A3 | 23.97 / 45.50 / 65.18% | 81.1 | 51.7 / 108.6 / 297.5 / 340.4 |
| A4 | 24.67 / 33.98 / 61.59% | 36.3 | 0.0 / 0.0 / 142.6 / 1.3 |
| A5 | 24.01 / 39.16 / 67.21% | 749.8 | 491.9 / 652.8 / 653.3 / 205.7 |
| B1 | 20.28 / 25.04 / 65.24% | 121.0 | 212.9 / 0.0 / 176.6 / 576.1 |
| B2 | 23.03 / 35.16 / 56.60% | 110.9 | 627.7 / 411.2 / 321.3 / 248.4 |
| B3 | 19.55 / 40.09 / 61.65% | 88.6 | 429.3 / 20.9 / 132.0 / 0.0 |
| baseline repeat | 10.02 / 20.82 / 33.73% | not applicable: tour off | 39.7 / 572.0 / 564.1 / 220.5 |

**Measurement procedure and limitations.** The earlier run-5/run-6 reports
contain plan-time diagnostics, but no coverage/re-driving procedure or
reference figures. The supervisor approved these replacements before runs:

- Read-only, coherent SDMGRID1 grid/source/index snapshots every 60 simulated
  seconds. A floor-proxy column has a measured surface endpoint with two
  contiguous free 0.2-m voxels immediately above its occupied voxel. Union
  the 0.2-m XY columns across robots, placed using their configured ground-truth
  starts. This is projected observed floor, not a traversability oracle;
  different levels collapse in XY and odometry drift affects placement.
  The earlier fixed-height corridor floor proxies do not cover this multi-level
  course. On the baseline, 4023/4104 trajectory-overlapping counted columns
  (98.03%) had an endpoint within ±0.5 m of ground-truth base height, using
  the configured +0.15-m vertical frame placement.
- Denominator: union of final coverage over all ten runs, **3396.08 m²**
  (84,902 columns). Numerators are intersections with that denominator;
  90%/99% times are first observed 60-s samples, with no interpolation.
  Final maps were captured after Stop (8–17 simulated seconds later, usually 11);
  the 30-min column reports that post-stop snapshot. All 29 intermediate
  sampling operations in every run began at their scheduled simulated second.
  Robots were copied sequentially from their latest published products, not
  an atomic fleet-wide map; source timestamps are retained in metadata.
- Ground-truth positions at 1 Hz, 1801 samples per robot per run, maximum gap
  1 s. Credit a full 3D segment as re-driven if its endpoint is within 2 m
  of any own/peer trajectory point strictly older than 60 simulated seconds.
- Lower median of logged `plan request: ...; N ms (global ...)` summaries;
  maximum logged `costed and solved in N ms`. Logs were restricted to the
  exploration window. Docker timestamps were mapped to recorded simulation
  time for award/idle intervals.
- *B1/B2/B3 had 86/45/5 positive-unassigned award lines, respectively, but
  **no** `its bundle is done` or `exploration complete for this robot` lines.
  Thus there was no logged overlap. Actual starvation while a controller is
  blocked is **not measured** by this log proxy; this is not a pass of the
  two-minute no-starvation goal.
- At this SwarmDeck revision `--explore 1800` was inert. The scratch runner
  waited for four ready adapters, then sent the UI's WebSocket commands
  `{"type":"start_explore","robot_id":"robot_N"}` for N=0..3 at `/ws`,
  and corresponding `stop_explore` commands after 1800 `/clock` seconds.
  Each mission was fresh and ended with `sim-up --down`. A first baseline
  at real-time factor 0.2 was discarded before adopting GPU/drift for all
  measured runs.

Stage decision denominators were 2905.28 m² (gain), 3221.60 m² (heading),
and 3383.88 m² (balance): winners A3, A3, B3, respectively, with no two-point
contender. Recomputing against the final union did not change any winner.
**Chosen: `tour.min_cluster_gain=1200`, `tour.heading_weight=2.0`,
`fleet.balance_weight=0.6`.** These are exploration-metric choices, not
validated evidence of the §1 success goals.

Known problems were retained, not fixed. Counts below are full traversals
of the entrance corridor's x=-2..2 m segment with |y|≤3 m (multiple passages
show backtracking), and the longest continuous period within 0.5 m of a
stationary anchor; all tuples are robots 0/1/2/3, in simulated seconds.

| Run | Corridor passages | Longest stationary span s | Notable recorded event |
|---|---|---|---|
| baseline | 1/1/3/1 | 620/23/454/209 | r0 tilt stop; r2/r3 progress failures |
| A1 | 1/1/1/1 | 995/758/1037/1193 | r2 slope-turn/no-departure trap at planner (78.57,-37.33,0.43) |
| A2 | 1/3/5/3 | 947/28/39/268 | corridor backtracking; r0 tilt stop |
| A3 | 1/1/1/5 | 763/963/589/794 | r2 terminal 31° tilt; r3 backtracking/progress failure |
| A4 | 1/0/1/0 | 1487/1745/43/1745 | r0 slope-turn/no-departure trap at planner (22.71,-62.79,-0.33) |
| A5 | 3/3/5/1 | 25/11/53/24 | repeated corridor backtracking; no long final stall |
| B1 | 1/0/1/3 | 186/1759/877/32 | r1 near-start tilt stop; r2 tilt escalation |
| B2 | 9/2/1/3 | 32/290/47/1140 | r0 repeated backtracking; r1 terminal tilt; r3 controller failure |
| B3 | 5/1/1/0 | 29/1268/982/1771 | r1 unsupported rising path; r3 replacement-path timeout |
| baseline repeat | 1/3/7/3 | 1560/40/22/1019 | r0 controller failure; r2 repeated backtracking |

The chosen tour alone (A3) exceeded the original baseline's 20-min proxy
coverage by 3.15 points and reduced re-driving by 40.7%. Adding fleet
assignment (B3) reduced coverage relative to A3 and the original baseline;
its lower re-driving is confounded by stopped robots. No run reached 90% or
99%. Baseline repeat variability is large. These single trials do not
establish robust improvement, the historical run-5 comparison is not
measured, and timing/starvation acceptance remains open.

Reproduction evidence: tuf `/tmp/mgg-tour/runs/<run>/` (mission IDs,
parameters, clocks, trajectory CSVs, snapshots, logs, metrics); scripts
`/tmp/mgg-tour/metrics/`, also copied with the external Task-13 report.
The report records the full commit IDs, gates, decision snapshots and
cleanup. No SwarmDeck changes were committed.

## 6. Testing

MGG unit tests (`colcon test`, both OctoMap OFF and ON builds), with in-memory
roadmaps, bids and awards (no ROS):

- Tour: stable cluster IDs across graph revisions; optimal on small synthetic
  graphs (brute force); within 5 % of the best known on 50 random clusters;
  heading penalty avoids a U-turn start; commitment margin prevents flip-flop.
- Auction: the lowest ID is auctioneer and hands over when it falls silent; the
  sequential auction matches a brute-force optimum on small cases and spreads
  clusters in a two-corridor scene (balance penalty); commitment keeps current
  targets; a late bid keeps its previous bundle; a missed award keeps the previous
  bundle; clusters in a peer's explored space are dropped; two groups merging
  settle in one auction.
- Disconnection: claims of a silent peer persist until the TTL (simulated
  clock); idle robots take over the oldest stale claims; reconnection merges
  without needless target changes.
- Frames: no transform means solo touring; a transform appearing mid-run joins
  the pool.

`mgg_ros` integration test: two or three planner nodes on a synthetic map with a
shared frame, through the real message path; they split the frontiers and the
map is fully explored.

SwarmDeck live check (4 robots, SubT, C-SLAM merges on): the §1 metrics, plus a
5-minute link cut of one robot mid-run (it keeps exploring its clusters, peers do
not take them, it rejoins without duplicated work).

## 7. Delivery

1. Per-robot tour in MGG; deployed and measured alone.
2. Frontier auction in MGG (`mgg_msgs/TourBid` / `TourAward`, auctioneer
   election, award, claims, TTL, release service); measured with C-SLAM merges on.
3. SwarmDeck (outside this repo): enable inter-robot C-SLAM closures in the
   simulation, add a `deployment` neighbour-transform source to
   `deploy/mgg/robot_poses.py`, set `fleet.claim_ttl_s: 600` for the SubT
   simulation, stop passing leases for MGG targets, and show each robot's tour
   and assigned clusters.

Each step is reviewed and lands on MGG's `ros2` branch.

## 8. Out of scope

- Changing local exploration, gain computation, or terrain/turn checks.
- Optimal multi-robot TSP; the sequential auction heuristic is deliberate.
- Selecting the shared-frame source inside MGG.
