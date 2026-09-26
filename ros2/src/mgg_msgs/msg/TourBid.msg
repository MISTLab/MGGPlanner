# One robot's bid in its group's frontier auction (tour-exploration design
# §3.3). Every robot also bids every fleet.auction_interval_s unasked, which
# is how its peers hear it (§3.1).
std_msgs/Header header             # stamp; frame_id: the sender's planning frame
int32 robot_id
uint64 seq
uint64 auction_id                  # the auction called; 0 for a periodic bid
int32 auctioneer_id                # the auctioneer it follows; robot_id when it leads
geometry_msgs/Pose pose            # the sender's pose, in its planning frame
TourCluster[] clusters             # every frontier cluster it knows
float64[] costs_from_pose          # one per cluster; .inf when unreachable
float64[] costs_between            # clusters x clusters, row-major; .inf when unreachable
uint64 current_target              # 0: none
builtin_interfaces/Time claim_stamp  # when it took its current target
uint64[] bundle                    # its current bundle
uint64[] explored                  # awarded clusters its roadmap shows explored
bool request_auction               # its bundle is done: it asks for an auction
