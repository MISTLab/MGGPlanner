# Aerial fleet targets

Ground roadmap messages carry ground heights, not aerial body centres. A rendezvous at that height fails the aerial observed-free body sweep. An aerial receiver therefore keeps peer snapshots as frontier evidence, rather than importing their edges.

`aerial_frontier_height_m` (default 1.3 m) lifts a peer frontier above its transformed ground height, clamped to `aerial_min_height_m` (0.8) and `aerial_max_height_m` (2.5). Set these to the drone's configured height band. These are candidate targets only: a strict static aerial sweep must join each to the receiver's own roadmap within 5 m. Unknown space refuses the target. At most 64 highest-gain, spatially separated admitted targets have query endpoint slots. The slots are detached and revalidated against the current snapshot, transform and map on each query; explored/withdrawn evidence disappears. They are never nearest-neighbour rendezvous, never scored as own frontiers, and never broadcast. Their edges are receiver-validated, never ground edges.

Without the fleet auction, aerial reachability and battery-return reach filtering precede the preference for own clusters. Ground preference is unchanged. Logs distinguish an own representative missing from the graph, a disconnected/search-blocked route, and the battery return cap. Route admission still checks the entire aerial path with strict observed-free queries.

## Aerial peer clearance input

Publish `geometry_msgs/PoseArray` on the drone planner's `aerial_peer_bodies` topic, in its planning frame (`header.frame_id`). Each entry is explicitly **not a quaternion pose**:

- `position.x/y`: peer centre in the planning frame;
- `position.z`: peer TOP height in that frame, from its pose and spec;
- `orientation.x`: positive horizontal radius (spec footprint half-diagonal);
- `orientation.y/z/w`: exactly zero (the non-quaternion marker).

Each describes a yaw-invariant vertical cylinder from minus infinity to the top: the drone may fly over, never under, a ground peer. `aerial_peer_margin_m` defaults to **0.35 m** (EGO 0.25 inflation + 0.1 execution tolerance), added radially and above the top. The drone's own planning half-diagonal/half-height and centre offset are also included. Static obstacle admission is unchanged.

A message atomically replaces the set. Wrong frames or any malformed/non-finite entry reject the whole replacement, preserving the previous set. Empty messages clear it. `peer_body_ttl_s` (default 3 s) bounds freshness. Requests pin the active set through the entire search and route check, even across TTL expiry. Receipt and expiry invalidate tour costs. Ground robots ignore this input.

Until the first valid cylinder message, existing aerial `peer_bodies` XY-disc behaviour remains. That first message clears the aerial receiver's legacy discs; subsequent legacy messages cannot override the spec-aware input. The SwarmDeck launch should set the margin and publish this topic at its normal peer-body cadence (0.5 s). No SwarmDeck files are changed here.

Cylinders block local/global searches and final path admission, not stored edges. Moving/expired peers reopen those same edges without rebuilding the roadmap.
