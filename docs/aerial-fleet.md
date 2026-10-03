# Aerial fleet targets

Ground roadmap messages carry ground heights, not aerial body centres. A rendezvous at that height fails the aerial observed-free body sweep. An aerial receiver therefore keeps peer snapshots as frontier evidence, rather than importing their edges.

`aerial_frontier_height_m` (default 1.3 m) lifts a peer frontier above its transformed ground height, clamped to `aerial_min_height_m` (0.8) and `aerial_max_height_m` (2.5). Set these to the drone's configured height band. These are candidate targets only: a strict static aerial sweep must join each to the receiver's own roadmap within 5 m. Unknown space refuses the target. At most 64 highest-gain, spatially separated admitted targets have query endpoint slots. The slots are revalidated against the current snapshot, transform and map on each query; explored/withdrawn evidence disappears. Only a change to the accepted targets (identity, owner, position, gain or attachment) replaces the slot links and advances the graph revision. Unchanged revalidation leaves caches and ongoing routes intact. They are never nearest-neighbour rendezvous, never scored as own frontiers, and never broadcast. Their edges are receiver-validated, never ground edges.

Without the fleet auction, aerial reachability, battery-return reach and distance-discounted value filtering all precede the preference for own clusters. The drone is the fleet scout: lifted peer targets do not take the ground other-robot gain penalty. Their owner/auction identity is unchanged, and the default `tour.min_cluster_gain` (9000) and return-reach cap still apply. For example, 300 unknown voxels at the deployed gain of 60 are worth 18000 before distance discount, rather than 18. Ground preference and valuation are unchanged. Logs distinguish an own representative missing from the graph, a disconnected/search-blocked route, the battery return cap and the value floor. Route admission still checks the entire aerial path with strict observed-free queries.

## Robot collision obstacles

Robots are ordinary occupied voxels from live sensor observations. There are no
peer-body disc/cylinder topics, collision overlays or peer-only BLOCKED diagnosis.
Routing and sweeps apply the same clearance and refusal rules as for other mapped
obstacles. Exploration reservations and scouting exclusions still control frontier
ownership, and ground receivers retain the `aerial_peer_robot_ids` frontier filter.

SwarmDeck leaves peer-body masking off. A moved robot can remain in the planning
map until its old occupied cells receive three later qualified lidar
through-traversals; time alone does not clear them. Live local collision layers
remain responsible for current observations.

## Deployment boundary: one-drone fleet

Frontier sharing is for a **one-drone fleet** with ground peers. Graph messages do not identify the sender's platform: an aerial receiver refuses every peer roadmap's edges, including another aerial robot's, and lifts every peer frontier as if its Z were ground height. Feeding another drone's frontiers would incorrectly add a second flight-height offset. Multi-drone frontier sharing needs a separate typed protocol; it is not supported here.

## Drone home and reach costing

Return costs for the battery reach cap go to home, vertex 0. A drone whose
`flight_state` arrives after `aerial_home_state_wait_s` is seeded at its pose,
unlifted. The first state after that seed decides once: `landed` with the
drone less than 0.5 m from the seed pose (and still at its standing start)
re-roots home `aerial_home_height_m` over the current pose, cuts home's old
edges, and makes later keyframe rebuilds lift their first keyframe. Any other
first state, or a drone that has moved, keeps home where it is (warned).

When tour or bid costing finds home unreachable from the robot, and home's
body is observed free (the evidence Return Home asks of its goal), home is
wired to its reachable neighbours with the ordinary strict roadmap edge check
(`expandGraphEdges`), at most once per graph, revision and map revision. Occupied or unknown home bodies are never linked.

## Occupied-root departure

Only an aerial physical-root/departure query may recover from occupied root
volume on the native MOLA grid. Ordinary edges and ground checks are unchanged.
The fallback uses the navigation planning box transformed as an OBB (full SE(3),
not its enlarged component AABB), and an exact continuous translated-OBB/voxel
SAT sweep. Up to 16 occupied voxels with positive-volume root OBB intersection
may be left; for every such voxel the initial derivative of squared
centre-to-voxel distance must be positive. Convexity makes distance strictly
increasing for the whole straight departure. Tangential/inward motion and a
centre inside an occupied voxel fail closed. No other occupied or unknown
swept voxel is allowed, including unknown root air on this recovery branch.
The endpoint must pass the unchanged strict enclosing-AABB clearance and all
dynamic sweep/endpoint checks remain in force. Work is capped at 65,536 cells;
unsupported backends reject recovery. No map cells are cleared.

## Observability

After an aerial plan request, at most every 10 s, MGG logs one line
`aerial_status {json}` with: `home` (status, position, lifted, edges,
re-roots, relink attempts/successes), `lifted` (latest evaluation: proposed,
admitted, rejected by gain/cap/merged/no_anchor/link/scouting, senders without
a transform; cumulative tour selections of a new lifted target),
`reach_cap` (rejects, those with no way home, last out/back distances and
budget, current reach) and `scouting_exclusions` (in force, accepted,
rejected frame/malformed, lapsed, targets, lattice viewpoints and paths
refused). Counts are per evaluation, not per unique target. Reach-cap
refusals of own clusters log `out X m + back Y m > reach Z m` (or no way back
with home's status). The plan summary identifies a lifted tour target.
`root_recovery` reports cumulative successful fallback query `uses`,
`last_exempted_cells`, and the last admitted unit `last_direction` in navigation
coordinates. It is included in the same throttled log. Revalidation/search
queries count separately; this is not an executed-flight counter.

## Scouting exclusions input

`scouting_exclusions` (relative: `/<robot>/mgg/scouting_exclusions`),
`geometry_msgs/PoseArray`, transient-local, at most one message kept.
Encoding as `no_go_discs`: `position.x/y` centre, `position.z` centre-line
reach in metres (> 0, finite), orientation ignored. `header.frame_id` must be
the planning frame (`PlanningParams.global_frame_id`); a wrong frame or any
malformed entry refuses the whole message with a throttled warning, keeping
the set in force. Each message replaces the set; an empty array clears it.
The set lapses `scouting_exclusion_ttl_s` (default 3.0 s, finite, > 0) after
MGG received it; `header.stamp` is not read. The producer re-publishes at
least every second.

Exploration only, XY only: tour clusters, lifted peer targets and greedy
global frontiers inside one are not chosen; lattice viewpoints inside one are
left out; lattice candidates, global exploration routes (rerouted round them
where the roadmap allows), their shortcuts, and the plan's final check obey
the egress rule: a path starting inside one may leave it while its horizontal
distance to the centre never decreases, must then enter none, and must not
end inside one. Objectives (Navigate, Return Home) ignore them. Without a
message, behaviour is unchanged.
