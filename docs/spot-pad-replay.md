# Spot beside the drone pad: replay, not a departure-policy change

The drone-r2 replay does **not** confirm that the pad disc removed an otherwise usable reverse departure. The supervisor approved report-only G1: ground admission and departure policy are unchanged.

## Evidence

Read-only input: tuf `~/swarmdeck-ws/drone-scout/sessions/sim-trials/drone-r2/peer_maps-44c15021-f197-4832-b1dd-6727ebdc1710.tgz`, robot_3's `planning/mola` product. Only the needed source/index, native grids and fleet snapshot were copied to tmpfs. The replay source, command and output are in `/tmp/swarmdeck-lanes/mgg-aerial-fleet/replay/`.

- Replayed native grid revision 18, SHA-256 `b5674cf64d653a162a5408ec6adc02921dea556b43704ac5a21d28ac18e3ac1b`, source stamp sim 196.4 s, resolution 0.2 m.
- The saved 0180, 0240 and 0300 source snapshots have identical SHA-256 `98c5999d9a1c4312c2ee7c087a59a33e59a0e9bbacee912c60b6164de95e0402` and name this same product. This is a replay of the stationary failure within the requested sim 170–300 interval, not a reconstruction of every historical map. The archive also retains revision 17 (source stamp 186 s); its matching historical source/authority is not available in these snapshots. No exact sim-170 map replay is claimed.
- Used the saved `T_component_navigation`, navigation XY `(0.80398999, 0.66125134)`, yaw `1.10699225`, and logged planner driving Z `0.82`.
- Spot body `1.1 x 0.5 x 1.0 m`, extension `0.05 m`, driving height `0.975 m`, max step `0.30 m`, climb 30°, cross-slope 12°, footprint tilt 25°, plane residual/cell rise `0.25 m`, observation fraction `0.75`, interpolation `0.25 m`, reverse allowed. No standing-start exemption; no transient peers. Pad in navigation coordinates `(-1.5, -2.0)`, centre-line exclusion radius `2.392377439199098 m`.

`findDeparture` offered **zero endpoints** and returned no departure both with and without the pad. `roomToTurn` refused the start in both cases. At the first 0.25 m step, both forward and reverse ground projections found no occupied support (`drop = -1`, terminal ray status FREE). The occupied ground required by `toDrivingHeight` is absent. Direct strict body sweeps returned UNKNOWN already at 0.1, 0.2, 0.5 and 1.0 m reverse.

| Reverse distance | Pad on: body sweep | Pad off: body sweep | End distance from pad |
|---|---|---|---|
| 0.5 m | UNKNOWN | UNKNOWN | 3.038 m |
| 1.0 m | UNKNOWN | UNKNOWN | 2.563 m |
| 1.2 m | OCCUPIED | UNKNOWN | 2.376 m |
| 2.0 m | OCCUPIED | UNKNOWN | 1.658 m |

Thus the disc adds a restriction only after the missing-observation failure already prevents departure. A full 2 m reverse also **ends inside the disc**, so the proposed ends-outside/never-deeper exception would not admit it. Removing the disc does not restore any departure.

The actual blocker is the stationary Spot's unobserved ground/body space under and behind it (its lidar blind disk), after its standing-start exemption expired. The archived product has **15 qualified ray keyframes**, not one: the diagnostic's `keyframes = [seq 0]` describes one submap rather than the complete source. More keyframes at nearly the same place do not establish the missing swept space.

## Separate ground-lane follow-up

Investigate retaining narrowly scoped standing-start evidence, or a known driven-track retreat, when a robot has not meaningfully moved since its start/peer stop. That needs a separate ground policy decision and evidence tests. This lane does not relax observed-ground/body checks or the rule that a departure must never end inside a pad disc.
