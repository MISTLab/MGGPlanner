# The executor's report on the local path it drives, sent to the local
# planner. A feedback from another session or epoch is ignored.
string EXECUTING=executing
string IDLE=idle
# The executor's refusal of path sequence_id, sent at once with its reason
# logged executor-side: the planner's explicit evidence that a publication
# will not be driven (an acceptance is the first EXECUTING report naming it).
string REFUSED=refused
std_msgs/Header header
string session_id
uint64 epoch
# The executed path; 0 when none. REFUSED: the refused path.
uint64 sequence_id
# EXECUTING, IDLE or REFUSED.
string state
# Distance travelled along path sequence_id, metres.
float64 progress_m
float64 speed_mps
# Braking bounds of the executor's motion fence.
float64 deceleration_mps2
float64 latency_s
float64 speed_cap_mps
