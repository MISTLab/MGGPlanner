# One robot's awarded frontier clusters, in tour order (design §3.4).
int32 robot_id
uint64[] clusters
# How long the auctioneer has not heard this robot, seconds; 0 when it bid on
# time. A silent robot's claim expires fleet.claim_ttl_s after it was last
# heard (§4), wherever the award is received.
float64 silent_s
# The seq of this robot's bid the auctioneer collected in this round; 0 when
# it collected none (a robot that only holds its claim).
uint64 bid_seq
