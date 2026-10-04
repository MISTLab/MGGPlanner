# A certified local path in the local planner's odometry frame
# (header.frame_id). Poses sit at MGG's driving height, like the planner's
# graph vertices. The first prefix_length poses repeat exactly the committed
# prefix of path extends_sequence_id (EXTEND); BREAK replaces that path from
# the current pose; START begins a session or a new process epoch.
uint8 START=0
uint8 EXTEND=1
uint8 BREAK=2
std_msgs/Header header
string session_id
string request_id
# Process epoch of the planner; sequence_id starts at 1 in each epoch.
uint64 epoch
uint64 sequence_id
uint64 extends_sequence_id
uint64 map_revision
uint64 guidance_sequence_id
uint8 kind
uint32 prefix_length
float64 commit_length_m
geometry_msgs/Pose[] poses
# One flag per pose: travel leaving that pose is in reverse; the last repeats.
bool[] reverse
bool reaches_goal
