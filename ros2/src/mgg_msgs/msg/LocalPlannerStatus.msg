# The ground local planner's status (cross-plan contract C2a), published on
# <robot>/mgg/local_planner/status. The drone's viewpoint selector reuses
# this type. status is one of the four constants, matching mgg::LocalStatus.
string MOVING=moving
string WAITING_FOR_MAP=waiting_for_map
string NO_LOCAL_TARGET=no_local_target
string BLOCKED=blocked
string status
string reason
uint64 map_revision
# Echo the session and the latest accepted request (SetLocalPlannerMode).
string session_id
string request_id
builtin_interfaces/Time stamp
# Optional extra: the latest guidance applied. Consumers must not require it.
uint64 guidance_sequence_id
