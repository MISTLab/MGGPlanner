# No-go inputs

Both inputs use reliable, transient-local `geometry_msgs/msg/PoseArray` (depth 1), in the planner's exact `world_frame`. Each replaces its own complete set; an empty array clears only that set. The publishers own lifetime. They are serialized in the same mutually-exclusive callback group and applied under the planner mutex between requests.

- `no_go_zones`: unchanged terrain contract. Pose position x/y is a centre, z/orientation ignored. Every disc uses `PlanningParams.no_go_radius_m` (default 1.5 m), with the existing planning-body inflation.
- `no_go_discs`: explicit **centre-line** exclusions. Pose position x/y is the centre in metres; **position.z is the already body-inflated reach in metres, NOT altitude**. Orientation is ignored. No further robot-size inflation is added. Nonfinite coordinates/reaches or nonpositive reaches reject the whole replacement. A frame mismatch is ignored. A change to radius alone invalidates route costs.

SwarmDeck uses one explicit disc per drone pad per ground robot: take-off clearance plus that ground platform's planning half-diagonal. Neither topic replaces the other. Both affect graph routing, path validation and MOLA dynamic box/sweep queries. A path starting in a disc may depart monotonically outward; it cannot re-enter and cannot end inside. Static obstacles are never exempted. Multiple overlapping discs still require outward motion from every containing disc; callers must not approximate a larger disc by a ring of smaller centres (which can trap an interior start).

Tests: `NoGoZones.PerDiscReachPreservesTerrainAndAllowsPadDeparture`, `MolaMap.CentreLineDiscsUseTheirOwnReachAndAllowOutwardSweeps`, `PlannerNodeTest.PadDiscsHaveIndependentRadiiAndDoNotReplaceTerrain`, and `PlannerNodeTest.PadDiscsStartingInsidePlansOutOnBothBackends`.

Wrong-frame or malformed discs produce throttled warnings (5 seconds) and keep
the previous set. Planner and MOLA tests exercise a deployed Bunker planning
footprint at the exact centre of a 2.43 m disc, both on open floor and with a
wall 0.8 m ahead: the latter backs out without entering the wall. The bounded
straight fallback clears the reach within its existing 3.0 m cap in both
scenes; no departure distance or collision constraint was relaxed.
