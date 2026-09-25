# Tour-based exploration and fleet frontier assignment

Status: design, awaiting operator review. Date: 2026-09-25.
Scope: MGGPlanner `ros2` branch (native code, not a downstream patch). SwarmDeck consumes it.

## 1. Goal

Robots must explore the whole reachable environment, spread across it, and stop
revisiting ground that is already explored — by themselves or by a peer.

Observed in SwarmDeck's simulated SubT runs (runs 4 and 5, MGG b977bfc):

- Clear frontiers remain in the map while robots linger in explored areas.
- Robots work off leftover local gain and only reposition globally after
  `low_gain_rounds` (3) of low local gain; the global planner then picks one
  frontier greedily (`searchGlobalFrontier`: best gain over distance). There is
  no ordering of frontiers and no commitment to a plan.
- Robots do not divide the work: fleet coordination today is only SwarmDeck's
  reservation leases (a claimed target is excluded for others within a radius).

Success means, on the 4-robot SubT simulation with C-SLAM merges on:

- the fraction of reachable space explored rises faster, and reaches 90 % / 99 %;
- metres driven over already-explored space (own or peer) drop clearly versus run 5;
- no frontier cluster stays unassigned for more than about 2 minutes while a robot idles;
- median plan time stays under about 350 ms.

Decisions taken with the operator:

- Both layers: a per-robot tour, and fleet assignment on top.
- Fleet assignment is decentralized among peers and lives entirely inside MGG,
  independent of SwarmDeck.
- Shared frame: the C-SLAM component when robots share one; otherwise the
  surveyed deployment frame; otherwise the robot tours alone. MGG does not choose
  the source: it uses whatever neighbour transforms it receives.
- Approach: replicated deterministic assignment (every robot computes the same
  assignment from the same shared data) with an MGG-native claim rule for
  conflicts. Rejected: auction/consensus bidding (multi-round, loss-sensitive,
  much more code) and static region partition (ignores where frontiers are).
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

## 3. Fleet assignment (inside MGG)

### 3.1 Shared data
A new message `mgg_msgs/Tour`, exchanged on the same peer channel and QoS as the
roadmap (`neighbour_graph_out` / the neighbour graph input), published with every
roadmap and on change:

- `robot_id`, `seq`, `stamp`;
- `pose` in the robot's own frame (peers transform it with the neighbour transform);
- `clusters[]`: stable ID, representative point (own frame), gain;
- `current_target` (cluster ID or none), `claim_stamp` (when it was claimed),
  `tour[]` (ordered cluster IDs) and `tour_cost`.

`Graph.msg` is unchanged, so peers without this feature still interoperate.

### 3.2 Who is in the group
A peer is in the robot's group when MGG holds a neighbour transform to it (the
same condition under which roadmaps merge today). Peers without a transform are
invisible to assignment.

### 3.3 Assignment (identical on every robot)
1. Pool the clusters of the robot and its group, in the shared frame. Clusters
   within `fleet.cluster_merge_radius_m` of each other are one cluster. A cluster
   that lies in explored space of any robot in the group is dropped — this is
   what stops re-exploring a peer's area.
2. Every robot keeps its current target unless another robot's claim on it wins
   (§3.4).
3. Assign the remaining clusters one at a time, cheapest insertion first: each
   goes to the robot whose tour it lengthens least, plus a balance penalty
   `fleet.balance_weight × that robot's current tour cost`. Ties: lower robot ID.
   Costs use each robot's own global graph from its pose; for a peer's cluster
   not yet in the robot's own graph, the cost is estimated on the merged roadmap.
4. Each robot tours only its own clusters (§2).

Recompute when the shared data changes, at most every `fleet.assign_interval_s`;
the commitment margin of §2.4 applies to reassignments.

A robot with no clusters of its own helps with the nearest cluster that is
claimed but not yet reached; if there is none, it reports exploration complete
for itself.

### 3.4 Conflicting claims
Two robots claiming the same target (different pictures of the shared data):
the lower remaining cost wins; if within 10 %, the older `claim_stamp` wins;
then the lower robot ID. The loser drops the cluster and continues its tour.
Every robot evaluates the same rule on the same data, so both sides agree.

### 3.5 SwarmDeck leases
MGG no longer needs SwarmDeck's reservation leases for its own exploration
targets. SwarmDeck stops passing them as exclusions when MGG fleet assignment is
on, and keeps leases for operator goals and Return Home.

## 4. Disconnection, reconnection, no shared frame

- **Silence is not failure.** A silent peer's last `Tour` stays valid: its
  assigned clusters and current target stay excluded for others. The silent
  robot keeps touring its last assignment plus frontiers it discovers itself.
- Claims expire after `fleet.claim_ttl_s`: default **1800 s** in MGG; SwarmDeck's
  simulated SubT configuration sets **600 s**. Other missions set their own.
- Earlier release of a silent robot's claims:
  1. a robot in the group observes the cluster explored (it simply stops being
     a frontier);
  2. robots are idle with no unclaimed clusters left: the oldest silent claims
     are released first, so idle robots can take them over;
  3. operator release (a service `release_claims(robot_id)`).
- **Reconnection:** roadmaps and `Tour`s merge as usual, new frontiers join the
  pool, the assignment re-runs with the commitment rule, clusters explored by
  both sides vanish, overlapping claims resolve by §3.4.
- **No shared frame:** the robot tours alone (§2). When a transform to a peer
  appears, its clusters and claims join the pool.
- Two groups out of contact assign independently; brief overlap at the boundary
  is accepted and resolved on contact.

## 5. Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `tour.enabled` | true | Use the tour instead of low-gain-triggered greedy repositioning |
| `tour.min_cluster_gain` | tuned | Drop clusters below this gain |
| `tour.cluster_id_cell_m` | 1.0 | Quantization for stable cluster IDs |
| `tour.heading_weight` | tuned | Cost per radian of first-leg heading change |
| `tour.recompute_interval_s` | 1.0 | Minimum interval between tour solves |
| `tour.commit_margin` | 0.2 | Fractional cost gain needed to switch target |
| `fleet.enabled` | true | Exchange `Tour`s and assign clusters |
| `fleet.cluster_merge_radius_m` | 2.0 | Clusters closer than this are one |
| `fleet.balance_weight` | tuned | Balance penalty on tour cost |
| `fleet.assign_interval_s` | 2.0 | Minimum interval between assignments |
| `fleet.claim_ttl_s` | 1800 | Claim lifetime of a silent peer (SwarmDeck SubT sim: 600) |

"Tuned" values are set from the SubT simulation during implementation and
recorded here.

## 6. Testing

MGG unit tests (`colcon test`, both OctoMap OFF and ON builds), with in-memory
roadmaps and `Tour`s (no ROS):

- Tour: stable cluster IDs across graph revisions; optimal on small synthetic
  graphs (brute force); within 5 % of the best known on 50 random clusters;
  heading penalty avoids a U-turn start; commitment margin prevents flip-flop.
- Assignment: identical inputs give identical outputs on every robot; the balance
  penalty spreads clusters in a two-corridor scene; conflict resolution agrees
  from both sides; clusters in a peer's explored space are dropped.
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
2. Fleet assignment in MGG (`mgg_msgs/Tour`, assignment, claims, TTL, release
   service); measured with C-SLAM merges on.
3. SwarmDeck (outside this repo): enable inter-robot C-SLAM closures in the
   simulation, add a `deployment` neighbour-transform source to
   `deploy/mgg/robot_poses.py`, set `fleet.claim_ttl_s: 600` for the SubT
   simulation, stop passing leases for MGG targets, and show each robot's tour
   and assigned clusters.

Each step is reviewed and lands on MGG's `ros2` branch.

## 8. Out of scope

- Changing local exploration, gain computation, or terrain/turn checks.
- Optimal multi-robot TSP; the heuristic assignment is deliberate.
- Selecting the shared-frame source inside MGG.
