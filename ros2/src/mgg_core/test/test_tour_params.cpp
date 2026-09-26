// Tests for the tour and fleet parameters (tour-exploration design §5).

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mgg_core/tour_params.h"

namespace {

TEST(TourParams, DefaultsAreTheDesignTable) {
  const mgg::TourParams p;
  EXPECT_TRUE(p.enabled);
  // Tuned in the SubT simulation (Task 13); these are the starting values.
  EXPECT_DOUBLE_EQ(p.min_cluster_gain, 600.0);
  EXPECT_DOUBLE_EQ(p.heading_weight, 2.0);
  EXPECT_DOUBLE_EQ(p.cluster_id_cell_m, 1.0);
  EXPECT_DOUBLE_EQ(p.recompute_interval_s, 1.0);
  EXPECT_DOUBLE_EQ(p.commit_margin, 0.2);
  EXPECT_DOUBLE_EQ(p.route_retry_s, 30.0);
}

TEST(FleetParams, DefaultsAreTheDesignTable) {
  const mgg::FleetParams p;
  EXPECT_TRUE(p.enabled);
  EXPECT_DOUBLE_EQ(p.cluster_merge_radius_m, 2.0);
  EXPECT_DOUBLE_EQ(p.balance_weight, 0.3);  // tuned (Task 13)
  EXPECT_DOUBLE_EQ(p.auction_interval_s, 2.0);
  EXPECT_DOUBLE_EQ(p.bid_deadline_s, 1.0);
  EXPECT_DOUBLE_EQ(p.peer_timeout_s, 5.0);
  EXPECT_DOUBLE_EQ(p.claim_ttl_s, 1800.0);
}

TEST(TourParams, ClampingRepairsOutOfRangeValues) {
  mgg::TourParams p;
  p.min_cluster_gain = -5.0;
  p.cluster_id_cell_m = 0.0;
  p.heading_weight = std::numeric_limits<double>::quiet_NaN();
  p.recompute_interval_s = -1.0;
  p.commit_margin = 1.5;
  p.route_retry_s = -3.0;
  mgg::clampTourParams(p);
  EXPECT_DOUBLE_EQ(p.min_cluster_gain, 0.0);
  EXPECT_DOUBLE_EQ(p.cluster_id_cell_m, mgg::kMinClusterCellM);
  EXPECT_DOUBLE_EQ(p.heading_weight, 2.0);  // NaN takes the default
  EXPECT_DOUBLE_EQ(p.recompute_interval_s, 0.0);
  EXPECT_DOUBLE_EQ(p.commit_margin, 0.95);
  EXPECT_DOUBLE_EQ(p.route_retry_s, 0.0);

  mgg::TourParams in_range;
  in_range.commit_margin = 0.35;
  mgg::clampTourParams(in_range);
  EXPECT_DOUBLE_EQ(in_range.commit_margin, 0.35);
}

TEST(FleetParams, ClampingRepairsOutOfRangeValues) {
  mgg::FleetParams p;
  p.cluster_merge_radius_m = -2.0;
  p.balance_weight = -0.1;
  p.auction_interval_s = std::numeric_limits<double>::infinity();
  p.bid_deadline_s = -1.0;
  p.peer_timeout_s = 0.0;
  p.claim_ttl_s = -600.0;
  mgg::clampFleetParams(p);
  EXPECT_DOUBLE_EQ(p.cluster_merge_radius_m, mgg::kMinClusterCellM);
  EXPECT_DOUBLE_EQ(p.balance_weight, 0.0);
  EXPECT_DOUBLE_EQ(p.auction_interval_s, 2.0);  // not finite: the default
  EXPECT_DOUBLE_EQ(p.bid_deadline_s, 0.0);
  EXPECT_DOUBLE_EQ(p.peer_timeout_s, 0.1);
  EXPECT_DOUBLE_EQ(p.claim_ttl_s, 0.0);
}

}  // namespace
