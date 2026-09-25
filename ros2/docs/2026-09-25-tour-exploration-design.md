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
- Nothing splits the frontiers among robots. Each robot chooses its next
  frontier as if it were alone; the only coordination is SwarmDeck's reservation
  lease, which keeps other robots away from the one spot a robot is heading to
  (within a radius). Several robots can still head into the same region while
  another region has nobody.

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
| `tour.min_cluster_gain` | tuned | Drop clusters below this gain |
| `tour.cluster_id_cell_m` | 1.0 | Quantization for stable cluster IDs |
| `tour.heading_weight` | tuned | Cost per radian of first-leg heading change |
| `tour.recompute_interval_s` | 1.0 | Minimum interval between tour solves |
| `tour.commit_margin` | 0.2 | Fractional cost gain needed to switch target |
| `fleet.enabled` | true | Bid, run and follow frontier auctions |
| `fleet.cluster_merge_radius_m` | 2.0 | Clusters closer than this are one |
| `fleet.balance_weight` | tuned | Balance penalty on bundle tour cost |
| `fleet.auction_interval_s` | 2.0 | Minimum interval between auctions |
| `fleet.bid_deadline_s` | 1.0 | How long the auctioneer waits for bids |
| `fleet.peer_timeout_s` | 5.0 | A robot not heard for this long leaves the group (its claims stay, see TTL) |
| `fleet.claim_ttl_s` | 1800 | Claim lifetime of a silent peer (SwarmDeck SubT sim: 600) |

"Tuned" values are set from the SubT simulation during implementation and
recorded here.

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
