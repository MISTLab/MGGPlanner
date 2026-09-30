# Aerial fleet targets

Ground roadmap messages carry ground heights, not aerial body centres. A rendezvous at that height fails the aerial observed-free body sweep. An aerial receiver therefore keeps peer snapshots as frontier evidence, rather than importing their edges.

`aerial_frontier_height_m` (default 1.3 m) lifts a peer frontier above its transformed ground height, clamped to `aerial_min_height_m` (0.8) and `aerial_max_height_m` (2.5). Set these to the drone's configured height band. These are candidate targets only: a strict static aerial sweep must join each to the receiver's own roadmap within 5 m. Unknown space refuses the target. At most 64 highest-gain, spatially separated admitted targets have query endpoint slots. The slots are revalidated against the current snapshot, transform and map on each query; explored/withdrawn evidence disappears. Only a change to the accepted targets (identity, owner, position, gain or attachment) replaces the slot links and advances the graph revision. Unchanged revalidation leaves caches and ongoing routes intact. They are never nearest-neighbour rendezvous, never scored as own frontiers, and never broadcast. Their edges are receiver-validated, never ground edges.

Without the fleet auction, aerial reachability, battery-return reach and distance-discounted value filtering all precede the preference for own clusters. The drone is the fleet scout: lifted peer targets do not take the ground other-robot gain penalty. Their owner/auction identity is unchanged, and the default `tour.min_cluster_gain` (9000) and return-reach cap still apply. For example, 300 unknown voxels at the deployed gain of 60 are worth 18000 before distance discount, rather than 18. Ground preference and valuation are unchanged. Logs distinguish an own representative missing from the graph, a disconnected/search-blocked route, the battery return cap and the value floor. Route admission still checks the entire aerial path with strict observed-free queries.

## Aerial peer clearance input

Publish `geometry_msgs/PoseArray` on the drone planner's `aerial_peer_bodies` topic, in its planning frame (`header.frame_id`). Each entry is explicitly **not a quaternion pose**:

- `position.x/y`: peer centre in the planning frame;
- `position.z`: peer TOP height in that frame, from its pose and spec;
- `orientation.x`: positive horizontal radius (spec footprint half-diagonal);
- `orientation.y/z/w`: exactly zero (the non-quaternion marker).

Each describes a yaw-invariant vertical cylinder from minus infinity to the top: the drone may fly over, never under, a ground peer. `aerial_peer_margin_m` defaults to **0.35 m** (EGO 0.25 inflation + 0.1 execution tolerance), added radially and above the top. The drone's own planning half-diagonal/half-height and centre offset are also included. Static obstacle admission is unchanged.

A message atomically replaces the set. Wrong frames or any malformed/non-finite entry reject the whole replacement, preserving the previous set. Empty messages clear it. `peer_body_ttl_s` (default 3 s) bounds freshness. Requests pin the active set through the entire search and route check, even across TTL expiry. Tour costs and expansion are invalidated only when the active cylinder-set key changes: XY, top and radius are quantised to 0.05 m, sorted and deduplicated. Repeated receipts refresh freshness but not the generation; expiry changes the key to empty. Ground robots ignore this input.

Until the first valid cylinder message, existing aerial `peer_bodies` XY-disc behaviour remains. That first message clears the aerial receiver's legacy discs; subsequent legacy messages cannot override the spec-aware input. The SwarmDeck launch should set the margin and publish this topic at its normal peer-body cadence (0.5 s). No SwarmDeck files are changed here.

Cylinders block local/global searches, shortcutting and final path admission, not stored edges. The cylinder's core is the peer's own spec volume, which may be masked out of lidar; static observed-free checks cannot replace this protection. A robot already inside the inflated cylinder may leave only with horizontal distance never decreasing and without descending, including a vertical climb. Entering deeper or descending (with any horizontal component) remains refused. A path may start inside at the robot's current pose or at a roadmap vertex within the shared `kDeltaLimit` snap distance (0.1 m). Any nonzero hop from the current pose to that front must also pass the cylinder sweep: snapping cannot hide inward or descending motion. The strict static observed-free checks also remain in force. Moving/expired peers reopen the same edges without rebuilding the roadmap.

## Deployment boundary: one-drone fleet

This interface is for a **one-drone fleet** with ground peers. Graph messages do not identify the sender's platform: an aerial receiver refuses every peer roadmap's edges, including another aerial robot's, and lifts every peer frontier as if its Z were ground height. Feeding another drone's frontiers would incorrectly add a second flight-height offset. Multi-drone frontier sharing needs a separate typed protocol; it is not supported here.

The cylinder producer must publish **only ground peers**, never the receiving drone itself. Publish every 0.5 s, including empty arrays, and use the exact planning-frame string (normally `robot_4/odom`, without a leading slash).

The first valid cylinder message, including an empty one, permanently selects the spec-aware input for that planner instance. There is **no automatic fallback** to legacy XY discs if the producer stops. After `peer_body_ttl_s` expires, there is no peer-cylinder avoidance until a fresh valid message arrives (static obstacle checks remain). This is the existing bounded-freshness policy, not a producer-failure safety stop. Deployment must keep the producer alive and monitor its publication; restarting legacy publication alone cannot restore avoidance.
