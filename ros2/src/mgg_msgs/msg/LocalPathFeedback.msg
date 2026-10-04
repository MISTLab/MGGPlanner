# The executor's report on the local path it drives, sent to the local
# planner. A feedback from another session or epoch is ignored.
string EXECUTING=executing
string IDLE=idle
std_msgs/Header header
string session_id
uint64 epoch
# The executed path; 0 when none.
uint64 sequence_id
# EXECUTING or IDLE.
string state
# Distance travelled along path sequence_id, metres.
float64 progress_m
float64 speed_mps
# Braking bounds of the executor's motion fence.
float64 deceleration_mps2
float64 latency_s
float64 speed_cap_mps
