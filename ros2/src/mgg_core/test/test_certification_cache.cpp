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

/// Voxels with unknown space: observed free air from z = 0 up, an observed
/// floor layer of voxels from -0.2 to 0 except at `unseen` columns, and
/// unknown everywhere under the floor; `voxels` overrides single voxels.
class VoxelTerrain : public MapInterface {
 public:
  using Key = std::array<std::int64_t, 3>;

  std::set<std::pair<std::int64_t, std::int64_t>> unseen;
  std::map<Key, VoxelStatus> voxels;

  static Key keyAt(const Eigen::Vector3d& p) {
    return {static_cast<std::int64_t>(std::floor(p.x() / kResolution)),
            static_cast<std::int64_t>(std::floor(p.y() / kResolution)),
            static_cast<std::int64_t>(std::floor(p.z() / kResolution))};
  }
  static Eigen::AlignedBox3d boxOf(const Key& k) {
    const Eigen::Vector3d lo(k[0] * kResolution, k[1] * kResolution,
                             k[2] * kResolution);
    return Eigen::AlignedBox3d(lo, lo + Eigen::Vector3d::Constant(kResolution));
  }
  VoxelStatus at(const Key& k) const {
    const auto set = voxels.find(k);
    if (set != voxels.end()) return set->second;
    if (k[2] >= 0) return VoxelStatus::kFree;
    if (k[2] == -1 && unseen.count({k[0], k[1]}) == 0) {
      return VoxelStatus::kOccupied;
    }
    return VoxelStatus::kUnknown;
  }

  double getResolution() const override { return kResolution; }
  bool getAxisAlignedXYCellCenter(const Eigen::Vector2d& p,
                                  Eigen::Vector2d& center) const override {
    center = ((p.array() / kResolution).floor() + 0.5).matrix() * kResolution;
    return center.allFinite();
  }
  bool getStatus() const override { return true; }
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    return at(keyAt(p));
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool stop_at_unknown) const override {
    Eigen::Vector3d ignored;
    return getRayStatus(a, b, stop_at_unknown, ignored);
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool stop_at_unknown,
                           Eigen::Vector3d& end_voxel) const override {
    const double length = (b - a).norm();
    const int steps =
        std::max(1, static_cast<int>(std::ceil(length / (0.25 * kResolution))));
    for (int i = 0; i <= steps; ++i) {
      const Key k = keyAt(a + (b - a) * (double(i) / steps));
      const VoxelStatus status = at(k);
      if (status == VoxelStatus::kOccupied ||
          (stop_at_unknown && status == VoxelStatus::kUnknown)) {
        end_voxel = boxOf(k).center();
        return status;
      }
    }
    end_voxel = b;
    return VoxelStatus::kFree;
  }
  VoxelStatus getBoxStatus(const Eigen::Vector3d& center,
                           const Eigen::Vector3d& size,
                           bool stop_at_unknown) const override {
    const Key lo = keyAt(center - 0.5 * size);
    const Key hi = keyAt(center + 0.5 * size);
    bool unknown = false;
    for (auto x = lo[0]; x <= hi[0]; ++x)
      for (auto y = lo[1]; y <= hi[1]; ++y)
        for (auto z = lo[2]; z <= hi[2]; ++z) {
          const VoxelStatus status = at({x, y, z});
          if (status == VoxelStatus::kOccupied) return status;
          unknown = unknown || status == VoxelStatus::kUnknown;
        }
    return unknown && stop_at_unknown ? VoxelStatus::kUnknown
                                      : VoxelStatus::kFree;
  }
  VoxelStatus getPathStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                            const Eigen::Vector3d& size,
                            bool stop_at_unknown) const override {
    const double steps =
        std::max(1.0, std::ceil((b - a).norm() / kResolution));
    const Eigen::Vector3d step = (b - a) / steps;
    for (int i = 0; i <= static_cast<int>(steps); ++i) {
      const VoxelStatus status =
          getBoxStatus(a + i * step, size, stop_at_unknown);
      if (status != VoxelStatus::kFree) return status;
    }
    return VoxelStatus::kFree;
  }
  void getScanStatus(const Eigen::Vector3d&,
                     const std::vector<Eigen::Vector3d>&, GainCounts&,
                     std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>&,
                     const SensorModel&) override {}
  void getScanStatusIterative(
      const Eigen::Vector3d&, const std::vector<Eigen::Vector3d>&,
      GainCounts&, std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>&,
      const SensorModel&) override {}
  bool augmentFreeBox(const Eigen::Vector3d&,
                      const Eigen::Vector3d&) override {
    return true;
  }
  void augmentFreeFrustum() override {}
  void resetMap() override {}
  void extractLocalMap(const Eigen::Vector3d&, const Eigen::Vector3d&,
                       std::vector<Eigen::Vector3d>&,
                       std::vector<Eigen::Vector3d>&) override {}
  void extractLocalMapAlongAxis(const Eigen::Vector3d&,
                                const Eigen::Vector3d&,
                                const Eigen::Vector3d&,
                                std::vector<Eigen::Vector3d>&,
                                std::vector<Eigen::Vector3d>&) override {}
  void getLocalPointcloud(const Eigen::Vector3d&, double, double,
                          std::vector<Eigen::Vector3d>&, bool) override {}
  void getFreeSpacePointCloud(const std::vector<Eigen::Vector3d>&,
                              const StateVec&,
                              std::vector<Eigen::Vector3d>&) override {}
  void setRaycastingParams(bool, double) override {}
  void setRobotRadius(double) override {}
};

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

// review r0, P1: a ground bridge over an unseen floor cell looks for a hole
// from the bridged ground down a whole projection length, deeper than the
// edge's own ground rays reach. A voxel seen free only down there disproves
// the bridge, and the cached admission with it.
TEST(CertificationCache, BridgeHoleBelowProjectionWithdrawsEdge) {
  Scene scene;
  scene.planning.min_observed_ground_fraction = 1.0;  // every cell ahead
  VoxelTerrain map;
  map.unseen.insert({2, 1});  // the cell centred (0.5, 0.3), under the body
  const double projection =
      GroundProjection(map, scene.planning).max_projection_length;
  CertificationCache cache(
      dependencyHalos(scene.robot, scene.planning, kResolution, projection));
  // The edge from a lattice vertex (not the root) at driving height 0.4,
  // 0.5 m over the floor's top voxel centre, along y = 0.1.
  const auto edge = [&](EdgeVerdictCache* verdicts) {
    const GroundProjection ground(map, scene.planning);
    ExpandContext ctx = scene.context(map, ground);
    ctx.edge_verdicts = verdicts;
    GraphManager graph;
    graph.addVertex(new Vertex(0, StateVec(-2.0, 0.1, 0.4, 0.0)));
    auto* parent = new Vertex(1, StateVec(0.1, 0.1, 0.4, 0.0));
    graph.addVertex(parent);
    Vertex candidate(2, StateVec(0.9, 0.1, 0.4, 0.0));
    ExpandGraphReport report;
    expandGraphFrom(graph, candidate, parent, report, ctx);
    return report.status;
  };
  ASSERT_EQ(edge(nullptr), ExpandGraphStatus::kSuccess);
  ASSERT_EQ(edge(&cache.edges()), ExpandGraphStatus::kSuccess);
  ASSERT_GT(cache.edges().size(), 0u);

  // Seen free 4.9 m down the unseen column: below every ray from the edge
  // (0.4 - P), within the hole check under the bridged floor (-0.1 - P).
  const VoxelTerrain::Key below{2, 1, -25};
  MapChange change;
  change.revision = 1;
  change.boxes.push_back(VoxelTerrain::boxOf(below));
  ASSERT_LT(change.boxes[0].max().z(), 0.4 - projection);
  ASSERT_GT(change.boxes[0].center().z(), -0.1 - projection);
  map.voxels[below] = VoxelStatus::kFree;
  cache.withdraw(change);
  EXPECT_EQ(cache.edges().size(), 0u);

  const ExpandGraphStatus cold = edge(nullptr);
  const ExpandGraphStatus warm = edge(&cache.edges());
  EXPECT_NE(cold, ExpandGraphStatus::kSuccess);
  EXPECT_EQ(warm, cold);
}

}  // namespace
}  // namespace mgg
