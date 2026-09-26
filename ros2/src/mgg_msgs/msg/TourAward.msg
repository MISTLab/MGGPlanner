# The auctioneer's call for bids (call = true, nothing else set) or its award
# (tour-exploration design §3.4).
std_msgs/Header header             # stamp; frame_id: the auctioneer's planning frame
uint64 auction_id
int32 auctioneer_id
bool call
TourCluster[] clusters             # every cluster the bundles name
TourBundle[] bundles               # per robot, in tour order
TourCluster[] explored             # clusters dropped as explored by a robot of the group
int32[] released_robot_ids         # silent robots whose claims were released (§4)
