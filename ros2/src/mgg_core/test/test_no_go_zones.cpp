// Tests for no-go zones: places a robot must not drive into, such as where
// it tripped its tilt guard (run 8). Review r0, I-3: standing in a zone
// disabled it for the whole route; only a monotonic outward departure is
// allowed, and the zone is enforced after it.

#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/graph_manager.h"
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

// Review r1, R1-1: the search filter (blocksEdge) opens departure edges
// either way, so Dijkstra took S-A-B-T, which leaves the zone at A and
// re-enters it at B; the final check refused it and the admissible
// S-A-C-T was never tried. The zone-respecting search expands directed
// states, departing outward only and never re-entering, and finds S-A-C-T.
TEST(NoGoZones, TheZoneRespectingSearchFindsTheRouteThatDoesNotReEnter) {
  mgg::GraphManager graph;
  const auto add = [&graph](int id, double x, double y) {
    auto* v = new mgg::Vertex(id, mgg::StateVec(x, y, 0.0, 0.0));
    graph.addVertex(v);
    return v;
  };
  mgg::Vertex* s = add(0, 0.5, 0.0);
  mgg::Vertex* a = add(1, 2.0, 0.0);
  mgg::Vertex* b = add(2, 0.5, 0.5);
  mgg::Vertex* t = add(3, 0.0, 2.0);
  mgg::Vertex* c = add(4, 2.0, 2.0);
  const auto link = [&graph](mgg::Vertex* u, mgg::Vertex* v) {
    graph.addEdge(u, v, (u->state - v->state).head<3>().norm());
  };
  link(s, a);
  link(a, b);
  link(b, t);
  link(a, c);
  link(c, t);
  const NoGoZones zones = zoneAtOrigin();
  const Eigen::Vector3d robot = s->state.head<3>();
  graph.setEdgeBlocked([&zones, &robot](const mgg::Vertex& u, const mgg::Vertex& v) {
    return zones.blocksEdge(u.state.head<3>(), v.state.head<3>(), robot);
  });
  // Dijkstra over the open edges: S-A-B-T, which re-enters.
  mgg::ShortestPathsReport rep;
  ASSERT_TRUE(graph.findShortestPaths(0, rep));
  std::vector<int> dijkstra;
  graph.getShortestPath(3, rep, true, dijkstra);
  EXPECT_EQ(dijkstra, (std::vector<int>{0, 1, 2, 3}));
  std::vector<Eigen::Vector3d> points;
  for (int id : dijkstra) points.push_back(graph.getVertex(id)->state.head<3>());
  EXPECT_FALSE(zones.pathAdmissible(points));
  // The zone-respecting search: S-A-C-T.
  const std::vector<mgg::Vertex*> route =
      mgg::zoneRespectingRoute(graph, 0, 3, robot, zones);
  ASSERT_EQ(route.size(), 4u);
  EXPECT_EQ(route[0], s);
  EXPECT_EQ(route[1], a);
  EXPECT_EQ(route[2], c);
  EXPECT_EQ(route[3], t);
  points.clear();
  for (const mgg::Vertex* v : route) points.push_back(v->state.head<3>());
  EXPECT_TRUE(zones.pathAdmissible(points));
  // With A-C gone, nothing is admissible: no route.
  graph.removeEdge(a, c);
  EXPECT_TRUE(mgg::zoneRespectingRoute(graph, 0, 3, robot, zones).empty());
}

// Review r2, R2-1: departures were tracked for the first 64 zones only, so
// a start inside the 65th could not depart it, not even straight out.
TEST(NoGoZones, ADepartureFromTheSixtyFifthZoneIsAdmissible) {
  std::vector<Eigen::Vector2d> centres;
  for (int i = 0; i < 64; ++i) centres.emplace_back(100.0 + 5.0 * i, 100.0);
  centres.emplace_back(0.0, 0.0);
  NoGoZones zones;
  zones.set(centres, 1.0);
  EXPECT_TRUE(zones.pathAdmissible(path({{0.5, 0}, {2, 0}})));
  EXPECT_FALSE(zones.pathAdmissible(path({{0.5, 0}, {-2, 0}})));
  // The search departs it too.
  mgg::GraphManager graph;
  graph.addVertex(new mgg::Vertex(0, mgg::StateVec(0.5, 0.0, 0.0, 0.0)));
  graph.addVertex(new mgg::Vertex(1, mgg::StateVec(2.0, 0.0, 0.0, 0.0)));
  graph.addEdge(graph.getVertex(0), graph.getVertex(1), 1.5);
  EXPECT_EQ(mgg::zoneRespectingRoute(graph, 0, 1, {0.5, 0.0, 0.0}, zones).size(),
            2u);
}

// Review r2, R2-1: what a path may do does not depend on the order the
// zones were given in.
TEST(NoGoZones, AdmissibilityDoesNotDependOnTheZonesOrder) {
  std::vector<Eigen::Vector2d> centres;
  for (int i = 0; i < 70; ++i) centres.emplace_back(10.0 + 3.0 * i, 3.0 * (i % 5));
  centres.emplace_back(0.0, 0.0);
  centres.emplace_back(0.6, 0.0);
  const std::vector<std::vector<Eigen::Vector3d>> paths = {
      path({{0.3, 0}, {0.3, 3}}),
      path({{0.3, 0}, {-3, 0}}),
      path({{0.3, 0}, {3, 0}}),
      path({{0.3, 0}, {0.3, 3}, {3, 3}, {3, 0.5}, {1.4, 0.5}}),
      path({{-5, 5}, {5, 5}}),
      path({{-5, 0}, {5, 0}}),
      path({{12, 3}, {12, 6}}),
  };
  std::vector<bool> expected;
  NoGoZones zones;
  zones.set(centres, 1.0);
  for (const auto& p : paths) expected.push_back(zones.pathAdmissible(p));
  for (int order = 0; order < 3; ++order) {
    std::vector<Eigen::Vector2d> shuffled = centres;
    if (order == 0) std::reverse(shuffled.begin(), shuffled.end());
    if (order == 1) std::rotate(shuffled.begin(), shuffled.begin() + 35, shuffled.end());
    if (order == 2) std::rotate(shuffled.begin(), shuffled.begin() + 71, shuffled.end());
    NoGoZones other;
    other.set(shuffled, 1.0);
    for (std::size_t k = 0; k < paths.size(); ++k) {
      EXPECT_EQ(other.pathAdmissible(paths[k]), expected[k])
          << "order " << order << ", path " << k;
    }
  }
}

// Review r2, R2-1: a start inside two overlapping zones departs both,
// moving away from each centre; moving toward either is refused, and so is
// coming back into either once out.
TEST(NoGoZones, AStartInsideTwoOverlappingZonesDepartsBoth) {
  NoGoZones zones;
  zones.set({Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.6, 0.0)}, 1.0);
  EXPECT_TRUE(zones.pathAdmissible(path({{0.3, 0}, {0.3, 3}})));
  // West: away from (0.6, 0), but across (0, 0)'s centre.
  EXPECT_FALSE(zones.pathAdmissible(path({{0.3, 0}, {-3, 0}})));
  // East: across (0.6, 0)'s centre.
  EXPECT_FALSE(zones.pathAdmissible(path({{0.3, 0}, {3, 0}})));
  EXPECT_FALSE(zones.pathAdmissible(
      path({{0.3, 0}, {0.3, 3}, {3, 3}, {3, 0.5}, {1.4, 0.5}})));
  EXPECT_TRUE(zones.pathAdmissible(path({{0.3, 0}, {0.3, 3}, {3, 3}, {3, 2}})));
}

}  // namespace

TEST(NoGoZones, PerDiscReachPreservesTerrainAndAllowsPadDeparture) {
  mgg::NoGoZones zones;
  zones.set({{0.0, 0.0}, {10.0, 0.0}}, std::vector<double>{2.4, 1.6});
  EXPECT_TRUE(zones.inside({2.3, 0, 0}));
  EXPECT_FALSE(zones.inside({8.3, 0, 0}));
  EXPECT_TRUE(zones.pathAdmissible({{0, 0, 0}, {3, 0, 0}}));
  EXPECT_TRUE(zones.pathAdmissible({{1.1, 0, 0}, {3, 0, 0}}));
  EXPECT_FALSE(zones.pathAdmissible({{3, 0, 0}, {2, 0, 0}}));
  EXPECT_FALSE(zones.pathAdmissible({{1.1, 0, 0}, {-3, 0, 0}}));
  EXPECT_TRUE(zones.blocksEdge({3, 0, 0}, {2, 0, 0}, {3, 0, 0}));
  zones.set({}, std::vector<double>{});
  EXPECT_TRUE(zones.empty());
}
