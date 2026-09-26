# A frontier cluster as the fleet exchanges it (tour-exploration design
# §2.1, §3.3).
uint64 id                      # stable cluster ID (mgg::makeClusterId); never 0
int32 owner_robot_id           # the robot whose frontier it is
geometry_msgs/Point position   # its representative, in the message's frame
float64 gain                   # the representative's volumetric gain
