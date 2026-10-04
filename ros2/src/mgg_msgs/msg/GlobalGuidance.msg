# Temporary global guidance for the ground local planner: a target and a
# coarse route in header.frame_id, at MGG's driving height. COMPLETE means the
# global planner has nothing left to guide to; NONE carries no target.
# standing_start rides on every kind.
uint8 NONE=0
uint8 TARGET=1
uint8 COMPLETE=2
std_msgs/Header header
string session_id
uint64 sequence_id
uint8 kind
geometry_msgs/Point target
geometry_msgs/Point[] route
string reason
StandingStart standing_start
