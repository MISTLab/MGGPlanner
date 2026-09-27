uint64 incarnation             # system_clock nanoseconds since epoch at planner construction
uint64 generation              # starts at 1; +1 on applied region/target/fleet changes
bool region_active
geometry_msgs/Point region_low
geometry_msgs/Point region_high
bool target_active
geometry_msgs/Point target
bool leaving_fleet             # FleetCoordinator::leaving(); false when fleet assignment is off
bool fleet_enabled             # whether a fleet coordinator exists (leave_fleet is refused when false)
builtin_interfaces/Time stamp  # node time of the configuration change
# Generation used by the most recent computed PlannerSrv answer; 0 before any.
# NOT_READY refusals do not update it; it does not bump generation or stamp.
uint64 last_plan_generation
