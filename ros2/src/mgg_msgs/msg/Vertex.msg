int32 id
geometry_msgs/Pose pose

# For volumetric gain
int32 num_unknown_voxels
int32 num_occupied_voxels
int32 num_free_voxels
bool is_frontier
int32 robot_id

# The sender marked this vertex visited: within its update radius of where
# it drove (event E1). Receivers treat its own frontiers near one as covered
# by the fleet. A sender built before this field leaves it false.
bool visited
