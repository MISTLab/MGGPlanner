uint64 incarnation             # random, non-zero, drawn once per planner process start
uint64 generation              # starts at 1; +1 on every applied change to any field below
bool region_active
geometry_msgs/Point region_low
geometry_msgs/Point region_high
bool target_active
geometry_msgs/Point target
bool leaving_fleet             # FleetCoordinator::leaving(); false when fleet assignment is off
bool fleet_enabled             # whether a fleet coordinator exists (leave_fleet is refused when false)
builtin_interfaces/Time stamp  # node time of the change
