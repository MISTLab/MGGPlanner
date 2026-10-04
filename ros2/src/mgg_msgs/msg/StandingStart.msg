# The guidance planner's proof that a ground robot still stands in the disk
# where it started (mgg::StandingStart), carried in every GlobalGuidance.
# Its lidar cannot see the ground near itself, so within the disk unobserved
# ground counts as ground; observed drops stay drops. The local planner may
# then seed its ground layer there and depart from a hanging root.
#
# valid is true only while the planner's keyframe trajectory for the map in
# service shows the robot never left the disk round its home keyframe, and
# the robot stands in it now; false once it has left (never again true for
# that map in that planner process), or without that proof (no keyframes,
# no hanging_root_edge_length_max, an aerial robot).
bool valid
# The disk's centre (x, y; z unused), the home keyframe, in the
# GlobalGuidance header frame.
geometry_msgs/Point center
# The disk's radius: the planner's hanging_root_edge_length_max, metres.
float64 radius
# Identity: the guidance planner's incarnation (PlannerConfigState), with
# the GlobalGuidance sequence_id that carries this block.
uint64 boot
