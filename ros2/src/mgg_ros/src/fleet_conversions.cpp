#include "mgg_ros/fleet_conversions.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "mgg_ros/conversions.h"

namespace mgg_ros {
namespace {

mgg::FleetCluster fromClusterMsg(const mgg_msgs::msg::TourCluster& msg,
                                 const Eigen::Isometry3d& t_ours_theirs) {
  mgg::FleetCluster cluster;
  cluster.id = msg.id;
  cluster.owner_robot_id = msg.owner_robot_id;
  cluster.position = t_ours_theirs * Eigen::Vector3d(
                                         msg.position.x, msg.position.y,
                                         msg.position.z);
  cluster.gain = msg.gain;
  return cluster;
}

mgg_msgs::msg::TourCluster toClusterMsg(const mgg::FleetCluster& cluster) {
  mgg_msgs::msg::TourCluster msg;
  msg.id = cluster.id;
  msg.owner_robot_id = cluster.owner_robot_id;
  msg.position.x = cluster.position.x();
  msg.position.y = cluster.position.y();
  msg.position.z = cluster.position.z();
  msg.gain = cluster.gain;
  return msg;
}

std::vector<mgg::FleetCluster> fromClusterMsgs(
    const std::vector<mgg_msgs::msg::TourCluster>& msgs,
    const Eigen::Isometry3d& t_ours_theirs) {
  std::vector<mgg::FleetCluster> clusters;
  clusters.reserve(msgs.size());
  for (const auto& msg : msgs) {
    clusters.push_back(fromClusterMsg(msg, t_ours_theirs));
  }
  return clusters;
}

std::vector<mgg_msgs::msg::TourCluster> toClusterMsgs(
    const std::vector<mgg::FleetCluster>& clusters) {
  std::vector<mgg_msgs::msg::TourCluster> msgs;
  msgs.reserve(clusters.size());
  for (const auto& cluster : clusters) msgs.push_back(toClusterMsg(cluster));
  return msgs;
}

}  // namespace

double stampSeconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         1e-9 * static_cast<double>(stamp.nanosec);
}

builtin_interfaces::msg::Time stampFromSeconds(double seconds) {
  builtin_interfaces::msg::Time stamp;
  const double seconds_limit =
      static_cast<double>(std::numeric_limits<std::int32_t>::max()) + 1.0;
  if (!std::isfinite(seconds) || seconds <= 0.0 || seconds >= seconds_limit) {
    return stamp;
  }
  const double whole = std::floor(seconds);
  stamp.sec = static_cast<std::int32_t>(whole);
  stamp.nanosec = static_cast<std::uint32_t>(
      std::min(999999999.0, std::round((seconds - whole) * 1e9)));
  return stamp;
}

mgg::TourBidData fromTourBidMsg(const mgg_msgs::msg::TourBid& msg,
                                const Eigen::Isometry3d& t_ours_theirs) {
  mgg::TourBidData bid;
  bid.robot_id = msg.robot_id;
  bid.seq = msg.seq;
  bid.stamp_s = stampSeconds(msg.header.stamp);
  bid.auction_id = msg.auction_id;
  bid.auctioneer_id = msg.auctioneer_id;
  const mgg::StateVec theirs = fromPoseMsg(msg.pose);
  const Eigen::Vector3d position =
      t_ours_theirs * Eigen::Vector3d(theirs.head<3>());
  // The heading of the composed rotation, as the roadmap merge places a
  // neighbour's vertex (graph_merge.cpp).
  const Eigen::Matrix3d rotation =
      t_ours_theirs.linear() *
      Eigen::Matrix3d(Eigen::AngleAxisd(theirs[3], Eigen::Vector3d::UnitZ()));
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  bid.pose = mgg::StateVec(position.x(), position.y(), position.z(), yaw);
  bid.clusters = fromClusterMsgs(msg.clusters, t_ours_theirs);
  bid.costs_from_pose.assign(msg.costs_from_pose.begin(),
                             msg.costs_from_pose.end());
  bid.costs_between.assign(msg.costs_between.begin(), msg.costs_between.end());
  bid.current_target = msg.current_target;
  bid.claim_stamp_s = stampSeconds(msg.claim_stamp);
  bid.bundle.assign(msg.bundle.begin(), msg.bundle.end());
  bid.explored.assign(msg.explored.begin(), msg.explored.end());
  bid.request_auction = msg.request_auction;
  bid.speed_mps = msg.speed_mps;
  bid.reach_m = msg.reach_m;
  bid.home = t_ours_theirs *
             Eigen::Vector3d(msg.home.x, msg.home.y, msg.home.z);
  return bid;
}

mgg_msgs::msg::TourBid toTourBidMsg(const mgg::TourBidData& bid,
                                    const std::string& frame_id) {
  mgg_msgs::msg::TourBid msg;
  msg.header.stamp = stampFromSeconds(bid.stamp_s);
  msg.header.frame_id = frame_id;
  msg.robot_id = bid.robot_id;
  msg.seq = bid.seq;
  msg.auction_id = bid.auction_id;
  msg.auctioneer_id = bid.auctioneer_id;
  msg.pose = toPoseMsg(bid.pose);
  msg.clusters = toClusterMsgs(bid.clusters);
  msg.costs_from_pose.assign(bid.costs_from_pose.begin(),
                             bid.costs_from_pose.end());
  msg.costs_between.assign(bid.costs_between.begin(), bid.costs_between.end());
  msg.current_target = bid.current_target;
  msg.claim_stamp = stampFromSeconds(bid.claim_stamp_s);
  msg.bundle.assign(bid.bundle.begin(), bid.bundle.end());
  msg.explored.assign(bid.explored.begin(), bid.explored.end());
  msg.request_auction = bid.request_auction;
  msg.speed_mps = bid.speed_mps;
  msg.reach_m = bid.reach_m;
  msg.home.x = bid.home.x();
  msg.home.y = bid.home.y();
  msg.home.z = bid.home.z();
  return msg;
}

mgg::TourAwardData fromTourAwardMsg(const mgg_msgs::msg::TourAward& msg,
                                    const Eigen::Isometry3d& t_ours_theirs) {
  mgg::TourAwardData award;
  award.auction_id = msg.auction_id;
  award.auctioneer_id = msg.auctioneer_id;
  award.stamp_s = stampSeconds(msg.header.stamp);
  award.call = msg.call;
  award.clusters = fromClusterMsgs(msg.clusters, t_ours_theirs);
  for (const auto& bundle : msg.bundles) {
    award.bundles.push_back(
        {bundle.robot_id,
         std::vector<mgg::ClusterId>(bundle.clusters.begin(),
                                     bundle.clusters.end()),
         bundle.silent_s, bundle.bid_seq});
  }
  award.explored = fromClusterMsgs(msg.explored, t_ours_theirs);
  award.released_robot_ids.assign(msg.released_robot_ids.begin(),
                                  msg.released_robot_ids.end());
  return award;
}

mgg_msgs::msg::TourAward toTourAwardMsg(const mgg::TourAwardData& award,
                                        const std::string& frame_id) {
  mgg_msgs::msg::TourAward msg;
  msg.header.stamp = stampFromSeconds(award.stamp_s);
  msg.header.frame_id = frame_id;
  msg.auction_id = award.auction_id;
  msg.auctioneer_id = award.auctioneer_id;
  msg.call = award.call;
  msg.clusters = toClusterMsgs(award.clusters);
  for (const mgg::RobotBundle& bundle : award.bundles) {
    mgg_msgs::msg::TourBundle out;
    out.robot_id = bundle.robot_id;
    out.clusters.assign(bundle.clusters.begin(), bundle.clusters.end());
    out.silent_s = bundle.silent_s;
    out.bid_seq = bundle.bid_seq;
    msg.bundles.push_back(out);
  }
  msg.explored = toClusterMsgs(award.explored);
  msg.released_robot_ids.assign(award.released_robot_ids.begin(),
                                award.released_robot_ids.end());
  return msg;
}

}  // namespace mgg_ros
