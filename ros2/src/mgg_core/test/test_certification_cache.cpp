// Tests for the dependency-complete certification cache: a map change
// withdraws every cached verdict whose dependency it reaches, not only the
// verdicts whose own cells it touched, and a warm cache answers exactly as
// a cold build would.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <utility>

#include "mgg_core/certification_cache.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/ground_projection.h"
#include "mgg_core/path_turns.h"
#include "terrain_fixture.h"

namespace mgg {
namespace {

constexpr double kResolution = 0.2;
using Tops = std::map<std::pair<std::int64_t, std::int64_t>, double>;

Tops flatFloor() {
  Tops tops;
  for (std::int64_t x = -15; x < 15; ++x)
    for (std::int64_t y = -15; y < 15; ++y) tops[{x, y}] = 0.0;
  return tops;
}

double cellBottom(double top) {
  return std::floor(top / kResolution) * kResolution;
}

/// The voxels a column's surface occupied before and after an edit.
MapChange columnEdit(Tops& tops, std::int64_t x, std::int64_t y, double top,
                     std::uint64_t revision) {
  const double before = tops.at({x, y});
  tops[{x, y}] = top;
  MapChange change;
  change.revision = revision;
  change.boxes.emplace_back(
      Eigen::Vector3d(x * kResolution, y * kResolution,
                      std::min(cellBottom(before), cellBottom(top))),
      Eigen::Vector3d((x + 1) * kResolution, (y + 1) * kResolution,
                      std::max(before, top)));
  return change;
}

/// The ground robot and lattice policy of GridGraph's flat-floor tests.
struct Scene {
  Scene() {
    robot.type = RobotType::kGroundRobot;
    robot.size = Eigen::Vector3d(0.4, 0.4, 0.4);
    planning.max_ground_height = 0.5;
    planning.max_step_height = 0.15;
    planning.max_inclination = 0.6;
    planning.edge_length_min = 0.05;
    planning.edge_length_max = 2.0;
    planning.edge_overshoot = 0.0;
    planning.nearest_range = 0.6;
    planning.nearest_range_min = 0.05;
    planning.nearest_range_max = 1.0;
    planning.nearest_range_z = 0.15;
  }

  ExpandContext context(const MapInterface& map,
                        const GroundProjection& ground) const {
    ExpandContext ctx;
    ctx.map = &map;
    ctx.planning = &planning;
    ctx.robot = &robot;
    ctx.ground = &ground;
    ctx.robot_box_size = robot.getPlanningSize();
    ctx.root_is_robot = true;
    return ctx;
  }

  DependencyHalos halos(const MapInterface& map) const {
    return dependencyHalos(
        robot, planning, kResolution,
        GroundProjection(map, planning).max_projection_length);
  }

  RobotParams robot;
  PlanningParams planning;
};

using EdgeSet = std::set<std::array<double, 6>>;

/// Every edge of `graph` by the positions of its two ends, lower end first.
EdgeSet edgesOf(const GraphManager& graph) {
  EdgeSet edges;
  for (const auto& [id, neighbours] : graph.edge_map_) {
    const Eigen::Vector3d a = graph.vertices_map_.at(id)->state.head<3>();
    for (const auto& [other, cost] : neighbours) {
      (void)cost;
      const Eigen::Vector3d b = graph.vertices_map_.at(other)->state.head<3>();
      std::array<double, 6> edge{a.x(), a.y(), a.z(), b.x(), b.y(), b.z()};
      if (std::lexicographical_compare(edge.begin() + 3, edge.end(),
                                       edge.begin(), edge.begin() + 3)) {
        std::rotate(edge.begin(), edge.begin() + 3, edge.end());
      }
      edges.insert(edge);
    }
  }
  return edges;
}

/// One lattice build from a fresh graph rooted at `root`.
EdgeSet build(const Scene& scene, const MapInterface& map,
              const StateVec& root, const GridGraphParams& grid,
              EdgeVerdictCache* verdicts) {
  const GroundProjection ground(map, scene.planning);
  ExpandContext ctx = scene.context(map, ground);
  ctx.edge_verdicts = verdicts;
  GraphManager graph;
  graph.addVertex(new Vertex(0, root));
  buildGridGraph(graph, root, grid, ctx, 0.0);
  return edgesOf(graph);
}

TEST(CertificationCache, FloorBelowPoseWithdrawsSlope) {
  const Scene scene;
  Tops tops = flatFloor();
  auto map = std::make_unique<mgg_test::TerrainFixture>(kResolution, tops);
  auto ground = std::make_unique<GroundProjection>(*map, scene.planning);
  CertificationCache cache(scene.halos(*map));

  int calls = 0;
  const double radius = std::max(scene.robot.size.x(), scene.robot.size.y());
  const SlopeFn slope = cache.slope([&](const Eigen::Vector3d& at) {
    ++calls;
    return groundSlope(*ground, at, radius, nullptr);
  });
  bool turn_room = true;  // what the map would say now
  const TurnRoomFn room =
      cache.turnRoom([&](const StateVec&) { return turn_room; });

  const Eigen::Vector3d pose(0.1, 0.1, 0.5);
  const StateVec state(pose.x(), pose.y(), pose.z(), 0.0);
  EXPECT_NEAR(slope(pose), 0.0, 1e-9);
  EXPECT_NEAR(slope(pose), 0.0, 1e-9);
  ASSERT_TRUE(room(state));
  ASSERT_EQ(calls, 1);

  // A change far from the pose keeps both.
  Tops far_tops = tops;
  cache.withdraw(columnEdit(far_tops, 12, 12, 0.4, 1));
  slope(pose);
  ASSERT_EQ(calls, 1);

  // The floor under the pose drops, half a metre below its own voxel.
  const MapChange floor = columnEdit(tops, 0, 0, -0.2, 2);
  ASSERT_FALSE(floor.boxes[0].contains(pose));
  ground.reset();
  map = std::make_unique<mgg_test::TerrainFixture>(kResolution, tops);
  ground = std::make_unique<GroundProjection>(*map, scene.planning);
  cache.withdraw(floor);
  turn_room = false;

  slope(pose);
  slope(pose);
  const int calls_after_floor_change = calls;
  const bool old_admission_available = room(state);
  EXPECT_EQ(calls_after_floor_change, 2);
  EXPECT_FALSE(old_admission_available);
}

TEST(CertificationCache, HaloNotOnlyCells) {
  const Scene scene;
  const Tops tops = flatFloor();
  const mgg_test::TerrainFixture map(kResolution, tops);
  CertificationCache cache(scene.halos(map));
  GridGraphParams grid;
  grid.min_val = Eigen::Vector3d(0.0, 0.0, 0.0);
  grid.max_val = Eigen::Vector3d(1.2, 0.0, 0.0);
  grid.resolution = Eigen::Vector3d(0.4, 0.4, 0.1);
  const StateVec root(0.1, 0.1, 0.5, 0.0);
  const EdgeSet edges = build(scene, map, root, grid, &cache.edges());
  ASSERT_GE(edges.size(), 3u);
  const std::size_t cached = cache.edges().size();
  ASSERT_GT(cached, 0u);

  // Beside the sweep at floor height: no cell any edge or its body box
  // covers (the body ends at y = 0.3), but within the bridged ground.
  const Eigen::AlignedBox3d beside(Eigen::Vector3d(0.6, 0.6, 0.0),
                                   Eigen::Vector3d(0.8, 0.8, 0.2));
  MapChange far;
  far.revision = 1;
  far.boxes.push_back(beside.translated(Eigen::Vector3d(0.0, 2.0, 0.0)));
  cache.withdraw(far);
  ASSERT_EQ(cache.edges().size(), cached);

  MapChange change;
  change.revision = 2;
  change.boxes.push_back(beside);
  cache.withdraw(change);
  EXPECT_EQ(cache.edges().size(), 0u);
}

TEST(CertificationCache, WarmEqualsCold) {
  const Scene scene;
  Tops tops = flatFloor();
  CertificationCache cache(
      scene.halos(mgg_test::TerrainFixture(kResolution, tops)));
  GridGraphParams grid;
  grid.min_val = Eigen::Vector3d(-1.6, -1.6, -0.2);
  grid.max_val = Eigen::Vector3d(1.6, 1.6, 0.3);
  grid.resolution = Eigen::Vector3d(0.4, 0.4, 0.1);
  grid.world_aligned = true;
  const std::array<StateVec, 2> roots{StateVec(0.1, 0.1, 0.5, 0.0),
                                      StateVec(0.23, 0.1, 0.5, 0.0)};
  std::mt19937 random(20261003);
  std::uniform_int_distribution<std::int64_t> column(-9, 9);
  const std::array<double, 7> heights{-0.4, -0.2, 0.0, 0.1, 0.2, 0.4, 0.6};
  std::uniform_int_distribution<std::size_t> height(0, heights.size() - 1);

  {
    const mgg_test::TerrainFixture map(kResolution, tops);
    build(scene, map, roots[0], grid, &cache.edges());
  }
  int warm_reused = 0;
  int withdrawn = 0;
  for (int edit = 0; edit < 20; ++edit) {
    std::int64_t x = 0, y = 0;
    do {
      x = column(random);
      y = column(random);
    } while (std::abs(x) <= 2 && std::abs(y) <= 2);  // keep the roots' floor
    const std::size_t before = cache.edges().size();
    cache.withdraw(columnEdit(tops, x, y, heights[height(random)],
                              static_cast<std::uint64_t>(edit + 1)));
    if (cache.edges().size() < before) ++withdrawn;
    if (cache.edges().size() > 0) ++warm_reused;
    const mgg_test::TerrainFixture map(kResolution, tops);
    const StateVec& root = roots[edit % 2];
    const EdgeSet warm_edges = build(scene, map, root, grid, &cache.edges());
    const EdgeSet cold_edges = build(scene, map, root, grid, nullptr);
    EXPECT_EQ(warm_edges, cold_edges) << "edit " << edit;
  }
  EXPECT_GT(warm_reused, 0);
  EXPECT_GT(withdrawn, 0);
}

}  // namespace
}  // namespace mgg
