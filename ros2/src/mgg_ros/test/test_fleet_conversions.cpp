// Round-trip tests for the fleet messages at the frame boundary
// (tour-exploration design §3.3, §3.4): positions move into the receiver's
// frame, costs do not.

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mgg_ros/fleet_conversions.h"

namespace {

using mgg::FleetCluster;
using mgg::TourAwardData;
using mgg::TourBidData;

FleetCluster cluster(mgg::ClusterId id, int owner, double x, double y) {
  FleetCluster c;
  c.id = id;
  c.owner_robot_id = owner;
  c.position = Eigen::Vector3d(x, y, 0.5);
  c.gain = 750.0;
  return c;
}

TourBidData sampleBid() {
  TourBidData bid;
  bid.robot_id = 2;
  bid.seq = 17;
  bid.stamp_s = 123.25;
  bid.auction_id = 9;
  bid.pose = mgg::StateVec(1.0, 0.0, 0.3, 0.0);
  bid.clusters = {cluster(21, 2, 2.0, 0.0), cluster(22, 3, 4.0, 1.0)};
  bid.costs_from_pose = {1.0, std::numeric_limits<double>::infinity()};
  bid.costs_between = {0.0, 2.5, 2.5, 0.0};
  bid.current_target = 21;
  bid.claim_stamp_s = 100.5;
  bid.bundle = {21};
  bid.explored = {31};
  bid.request_auction = true;
  return bid;
}

TEST(FleetConversions, ABidRoundTripsInItsOwnFrame) {
  const TourBidData in = sampleBid();
  const auto msg = mgg_ros::toTourBidMsg(in, "robot_1/odom");
  EXPECT_EQ(msg.header.frame_id, "robot_1/odom");
  const TourBidData out =
      mgg_ros::fromTourBidMsg(msg, Eigen::Isometry3d::Identity());
  EXPECT_EQ(out.robot_id, 2);
  EXPECT_EQ(out.seq, 17u);
  EXPECT_NEAR(out.stamp_s, 123.25, 1e-9);
  EXPECT_EQ(out.auction_id, 9u);
  EXPECT_TRUE(out.pose.isApprox(in.pose, 1e-12));
  ASSERT_EQ(out.clusters.size(), 2u);
  EXPECT_EQ(out.clusters[1].id, 22u);
  EXPECT_EQ(out.clusters[1].owner_robot_id, 3);
  EXPECT_DOUBLE_EQ(out.clusters[1].gain, 750.0);
  EXPECT_EQ(out.costs_from_pose[1], std::numeric_limits<double>::infinity());
  EXPECT_EQ(out.costs_between, in.costs_between);
  EXPECT_EQ(out.current_target, 21u);
  EXPECT_NEAR(out.claim_stamp_s, 100.5, 1e-9);
  EXPECT_EQ(out.bundle, in.bundle);
  EXPECT_EQ(out.explored, in.explored);
  EXPECT_TRUE(out.request_auction);
  EXPECT_TRUE(out.wellFormed());
}

TEST(FleetConversions, ABidIsPlacedInTheReceiversFrame) {
  // The sender's frame is 5 m east of ours and turned a quarter left.
  Eigen::Isometry3d t_ours_theirs = Eigen::Isometry3d::Identity();
  t_ours_theirs.linear() =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  t_ours_theirs.translation() = Eigen::Vector3d(5.0, 0.0, 0.0);
  const TourBidData out = mgg_ros::fromTourBidMsg(
      mgg_ros::toTourBidMsg(sampleBid(), "robot_1/odom"), t_ours_theirs);
  EXPECT_TRUE(out.pose.head<3>().isApprox(Eigen::Vector3d(5.0, 1.0, 0.3)));
  EXPECT_NEAR(out.pose[3], M_PI / 2.0, 1e-9);
  EXPECT_TRUE(out.clusters[0].position.isApprox(Eigen::Vector3d(5.0, 2.0, 0.5)));
  EXPECT_TRUE(out.clusters[1].position.isApprox(Eigen::Vector3d(4.0, 4.0, 0.5)));
  // Costs are lengths, the same in every frame.
  EXPECT_EQ(out.costs_between, sampleBid().costs_between);
}

TEST(FleetConversions, ABidsYawTurnsWithTheFrameAndWraps) {
  TourBidData in = sampleBid();
  in.pose[3] = 3.0;
  Eigen::Isometry3d t_ours_theirs = Eigen::Isometry3d::Identity();
  t_ours_theirs.linear() =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const TourBidData turned = mgg_ros::fromTourBidMsg(
      mgg_ros::toTourBidMsg(in, "robot_1/odom"), t_ours_theirs);
  EXPECT_NEAR(turned.pose[3], 3.0 + M_PI / 2.0 - 2.0 * M_PI, 1e-9);
  // A pitched frame: the heading is that of the composed rotation, as the
  // roadmap merge places a neighbour's vertex (graph_merge.cpp).
  in.pose[3] = M_PI / 4.0;
  t_ours_theirs.linear() =
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const TourBidData pitched = mgg_ros::fromTourBidMsg(
      mgg_ros::toTourBidMsg(in, "robot_1/odom"), t_ours_theirs);
  EXPECT_NEAR(pitched.pose[3], std::atan2(1.0, std::cos(0.5)), 1e-9);
}

TEST(FleetConversions, AnAwardRoundTripsWithBundlesExploredAndReleases) {
  TourAwardData in;
  in.auction_id = (std::uint64_t{1} << 40) | 3;
  in.auctioneer_id = 1;
  in.stamp_s = 42.5;
  in.clusters = {cluster(11, 1, 2.0, 0.0), cluster(21, 2, 9.0, 0.0)};
  in.bundles = {{1, {11}, 0.0}, {2, {21}, 7.5}};
  in.explored = {cluster(31, 3, -4.0, 0.0)};
  in.released_robot_ids = {4};
  Eigen::Isometry3d t_ours_theirs = Eigen::Isometry3d::Identity();
  t_ours_theirs.translation() = Eigen::Vector3d(0.0, 10.0, 0.0);
  const TourAwardData out = mgg_ros::fromTourAwardMsg(
      mgg_ros::toTourAwardMsg(in, "robot_0/odom"), t_ours_theirs);
  EXPECT_EQ(out.auction_id, in.auction_id);
  EXPECT_EQ(out.auctioneer_id, 1);
  EXPECT_NEAR(out.stamp_s, 42.5, 1e-9);
  EXPECT_FALSE(out.call);
  ASSERT_EQ(out.clusters.size(), 2u);
  for (std::size_t i = 0; i < in.clusters.size(); ++i) {
    EXPECT_EQ(out.clusters[i].id, in.clusters[i].id);
    EXPECT_EQ(out.clusters[i].owner_robot_id, in.clusters[i].owner_robot_id);
    EXPECT_DOUBLE_EQ(out.clusters[i].gain, in.clusters[i].gain);
  }
  ASSERT_EQ(out.bundles.size(), 2u);
  EXPECT_EQ(out.bundles[0].robot_id, 1);
  EXPECT_EQ(out.bundles[0].clusters, std::vector<mgg::ClusterId>{11});
  EXPECT_DOUBLE_EQ(out.bundles[0].silent_s, 0.0);
  EXPECT_EQ(out.bundles[1].robot_id, 2);
  EXPECT_EQ(out.bundles[1].clusters, std::vector<mgg::ClusterId>{21});
  EXPECT_DOUBLE_EQ(out.bundles[1].silent_s, 7.5);
  ASSERT_NE(out.cluster(21), nullptr);
  EXPECT_TRUE(out.cluster(21)->position.isApprox(Eigen::Vector3d(9.0, 10.0, 0.5)));
  ASSERT_EQ(out.explored.size(), 1u);
  EXPECT_EQ(out.explored[0].id, 31u);
  EXPECT_EQ(out.explored[0].owner_robot_id, 3);
  EXPECT_DOUBLE_EQ(out.explored[0].gain, 750.0);
  EXPECT_TRUE(out.explored[0].position.isApprox(Eigen::Vector3d(-4.0, 10.0, 0.5)));
  EXPECT_EQ(out.released_robot_ids, std::vector<int>{4});

  TourAwardData call;
  call.auction_id = 5;
  call.auctioneer_id = 1;
  call.call = true;
  EXPECT_TRUE(mgg_ros::fromTourAwardMsg(mgg_ros::toTourAwardMsg(call, "f"),
                                        Eigen::Isometry3d::Identity())
                  .call);
}

TEST(FleetConversions, StampsSurviveTheRoundTrip) {
  const auto stamp = mgg_ros::stampFromSeconds(12.25);
  EXPECT_EQ(stamp.sec, 12);
  EXPECT_EQ(stamp.nanosec, 250000000u);
  EXPECT_NEAR(mgg_ros::stampSeconds(stamp), 12.25, 1e-9);
  // Just short of a second: the nanoseconds stop at 999999999 instead of
  // rounding up to a whole second.
  const auto last_nanosecond = mgg_ros::stampFromSeconds(12.9999999999);
  EXPECT_EQ(last_nanosecond.sec, 12);
  EXPECT_EQ(last_nanosecond.nanosec, 999999999u);
  // Not a time: the zero stamp.
  const double inf = std::numeric_limits<double>::infinity();
  for (const double seconds : {-1.0, 0.0, std::nan(""), inf, -inf}) {
    const auto zero = mgg_ros::stampFromSeconds(seconds);
    EXPECT_EQ(zero.sec, 0) << seconds;
    EXPECT_EQ(zero.nanosec, 0u) << seconds;
  }
}

TEST(FleetConversions, UnrepresentableStampsBecomeZero) {
  // ROS Time stores whole seconds in a signed 32-bit field. Reject before
  // narrowing; an out-of-range floating-point to integer cast is undefined.
  for (const double seconds : {2147483648.0, 1e300}) {
    const auto stamp = mgg_ros::stampFromSeconds(seconds);
    EXPECT_EQ(stamp.sec, 0);
    EXPECT_EQ(stamp.nanosec, 0u);
  }
  const auto last_second = mgg_ros::stampFromSeconds(2147483647.0);
  EXPECT_EQ(last_second.sec, 2147483647);
  EXPECT_EQ(last_second.nanosec, 0u);
  EXPECT_DOUBLE_EQ(mgg_ros::stampSeconds(last_second), 2147483647.0);
}

}  // namespace
