// Translating the fleet messages (mgg_msgs/TourBid, TourAward) to and from
// mgg_core's exchange types (fleet_types.h). A received message is placed in
// this robot's planning frame with the transform its roadmap would merge
// with; positions move, costs do not.

#ifndef MGG_ROS_FLEET_CONVERSIONS_H_
#define MGG_ROS_FLEET_CONVERSIONS_H_

#include <string>

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <mgg_msgs/msg/tour_award.hpp>
#include <mgg_msgs/msg/tour_bid.hpp>

#include "mgg_core/fleet_types.h"

namespace mgg_ros {

double stampSeconds(const builtin_interfaces::msg::Time& stamp);
/// The zero stamp for a time that is not finite and positive.
builtin_interfaces::msg::Time stampFromSeconds(double seconds);

mgg::TourBidData fromTourBidMsg(const mgg_msgs::msg::TourBid& msg,
                                const Eigen::Isometry3d& t_ours_theirs);
mgg_msgs::msg::TourBid toTourBidMsg(const mgg::TourBidData& bid,
                                    const std::string& frame_id);
mgg::TourAwardData fromTourAwardMsg(const mgg_msgs::msg::TourAward& msg,
                                    const Eigen::Isometry3d& t_ours_theirs);
mgg_msgs::msg::TourAward toTourAwardMsg(const mgg::TourAwardData& award,
                                        const std::string& frame_id);

}  // namespace mgg_ros

#endif  // MGG_ROS_FLEET_CONVERSIONS_H_
