// Tests for no-go zones: places a robot must not drive into, such as where
// it tripped its tilt guard (run 8). Review r0, I-3: standing in a zone
// disabled it for the whole route; only a monotonic outward departure is
// allowed, and the zone is enforced after it.

#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/no_go_zones.h"

namespace {

using mgg::NoGoZones;

std::vector<Eigen::Vector3d> path(
    const std::vector<std::pair<double, double>>& xy) {
  std::vector<Eigen::Vector3d> out;
  for (const auto& [x, y] : xy) out.emplace_back(x, y, 0.0);
  return out;
}

NoGoZones zoneAtOrigin() {
  NoGoZones zones;
  zones.set({Eigen::Vector2d(0.0, 0.0)}, 1.0);
  return zones;
}

TEST(NoGoZones, AnEmptySetBlocksNothing) {
  NoGoZones zones;
  EXPECT_TRUE(zones.empty());
  EXPECT_TRUE(zones.pathAdmissible(path({{-5, 0}, {5, 0}})));
  EXPECT_FALSE(zones.blocksEdge({-5, 0, 0}, {5, 0, 0}, {-5, 0, 0}));
}

TEST(NoGoZones, APathFromOutsideMayNotCrossOrEndInAZone) {
  const NoGoZones zones = zoneAtOrigin();
  EXPECT_TRUE(zones.pathAdmissible(path({{-5, 2}, {5, 2}})));
  EXPECT_FALSE(zones.pathAdmissible(path({{-5, 0}, {5, 0}})));
  EXPECT_FALSE(zones.pathAdmissible(path({{-5, 0.5}, {5, 0.5}})));
  // An end inside the zone.
  EXPECT_FALSE(zones.pathAdmissible(path({{-5, 0}, {-0.5, 0}})));
  EXPECT_TRUE(zones.inside({0.5, 0.5, 0.0}));
  EXPECT_FALSE(zones.inside({1.5, 0.0, 0.0}));
}

TEST(NoGoZones, FromInsideOnlyAMonotonicOutwardDeparture) {
  const NoGoZones zones = zoneAtOrigin();
  // Straight out, from near the edge or from the centre.
  EXPECT_TRUE(zones.pathAdmissible(path({{0.6, 0}, {3, 0}})));
  EXPECT_TRUE(zones.pathAdmissible(path({{0, 0}, {0, 0.5}, {0.3, 2}})));
  // Inward first: across the centre to the far side.
  EXPECT_FALSE(zones.pathAdmissible(path({{0.6, 0}, {-3, 0}})));
  // Out, then along the edge closer again.
  EXPECT_FALSE(zones.pathAdmissible(path({{0.6, 0}, {0.8, 0}, {0.3, 0.5}, {3, 3}})));
  // Out, then back in: re-entry.
  EXPECT_FALSE(zones.pathAdmissible(path({{0.6, 0}, {2, 0}, {2, 2}, {0, 2}, {0, 0.5}, {-3, 0.5}})));
  // Out and round it, clear of it.
  EXPECT_TRUE(zones.pathAdmissible(path({{0.6, 0}, {2, 0}, {2, 2}, {-2, 2}, {-2, 0}})));
  // Outward but never out: it ends inside.
  EXPECT_FALSE(zones.pathAdmissible(path({{0.2, 0}, {0.6, 0}})));
}

TEST(NoGoZones, AnUndirectedEdgeIsOpenOnlyAsAnOutwardDepartureOfTheRobot) {
  const NoGoZones zones = zoneAtOrigin();
  const Eigen::Vector3d robot_in(0.6, 0.0, 0.0);
  const Eigen::Vector3d robot_out(-5.0, 0.0, 0.0);
  // Clear of the zone: open.
  EXPECT_FALSE(zones.blocksEdge({-5, 2, 0}, {5, 2, 0}, robot_out));
  // Through it, the robot outside: closed.
  EXPECT_FALSE(zones.blocksEdge({-5, 2, 0}, {5, 2, 0}, robot_in));
  EXPECT_TRUE(zones.blocksEdge({-3, 0, 0}, {3, 0, 0}, robot_out));
  // The robot inside: an edge whose nearest point to the centre is an end
  // no nearer than the robot is open (it can only be left along it) ...
  EXPECT_FALSE(zones.blocksEdge({0.6, 0, 0}, {3, 0, 0}, robot_in));
  EXPECT_FALSE(zones.blocksEdge({3, 0, 0}, {0.8, 0, 0}, robot_in));
  // ... one nearer than the robot, or passing nearer, is not.
  EXPECT_TRUE(zones.blocksEdge({0.3, 0, 0}, {3, 0, 0}, robot_in));
  EXPECT_TRUE(zones.blocksEdge({0.6, 0, 0}, {-3, 0, 0}, robot_in));
  EXPECT_TRUE(zones.blocksEdge({0.8, -1, 0}, {0.8, 1, 0}, robot_in));
}

}  // namespace
