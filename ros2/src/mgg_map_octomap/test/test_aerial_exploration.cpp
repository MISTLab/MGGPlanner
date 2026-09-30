// Aerial exploration on the actual ternary MOLA grid, without ground support.
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <unordered_map>
#include <vector>

#include "mgg_core/grid_graph.h"
#include "mgg_core/path_selection.h"
#include "mgg_map_octomap/native_mola_grid.h"

namespace {
using mgg::NativeMolaGrid;
using mgg::StateVec;
using mgg::VoxelStatus;
using Cell = NativeMolaGrid::Cell;

// Observed room: x [-2,2], y [-2,2], z [0,3]. Its east wall has a
// one-metre gap with observed air up to x=3; everything else outside is
// unknown. This deliberately has only one graph/keyframe anchor.
std::unique_ptr<NativeMolaGrid> room(bool blind_root = false,
                                     bool unknown_outside_root = false) {
  std::vector<Cell> occupied, free;
  for (int x = -11; x <= 16; ++x) {
    for (int y = -11; y <= 10; ++y) {
      for (int z = -1; z <= 15; ++z) {
        const bool gap = y >= -2 && y <= 2 && z >= 4 && z <= 10;
        const bool wall = x == -11 || y == -11 || y == 10 || z == -1 ||
                          z == 15 || (x == 10 && !gap);
        if (x <= 10 && wall) occupied.push_back({x, y, z});
        else if ((x < 10 || (x <= 14 && gap)) && !wall) {
          if (blind_root && x == 0 && y == 0 && z == 7) continue;
          if (unknown_outside_root && x == 2 && y == 0 && z == 7) continue;
          free.push_back({x, y, z});
        }
      }
    }
  }
  return std::make_unique<NativeMolaGrid>(0.2, occupied, free,
                                         std::vector<NativeMolaGrid::Surface>{});
}

struct Aerial {
  explicit Aerial(mgg::MapInterface& map) {
    robot.type = mgg::RobotType::kAerialRobot;
    robot.size = {0.5, 0.5, 0.3};
    planning.edge_length_min = 0.1;
    planning.edge_length_max = 0.9;
    planning.edge_overshoot = 0.0;
    planning.nearest_range = 1.0;
    planning.nearest_range_min = 0.1;
    planning.nearest_range_max = 100.0;
    planning.nearest_range_z = 100.0;
    planning.num_vertices_max = 1500;
    planning.num_edges_max = 50000;
    planning.num_loops_max = 100000;
    ctx.map = &map;
    ctx.robot = &robot;
    ctx.planning = &planning;
    ctx.robot_box_size = robot.size;
    // SwarmDeck's simulation setting, also inherited by the drone.
    ctx.allow_unknown_lattice_body = true;
    ctx.root_footprint_exempt = true;
    ctx.root_is_robot = true;
    graph.addVertex(new mgg::Vertex(0, start));
  }
  void build() {
    mgg::GridGraphParams grid;
    grid.min_val = {-3.0, -3.0, -0.2};
    grid.max_val = {4.0, 3.0, 0.2};
    grid.resolution = {0.4, 0.4, 0.2};
    mgg::buildGridGraph(graph, start, grid, ctx, 0.0);
  }
  mgg::PathSelectionResult select() {
    mgg::SensorParams sensor;
    sensor.type = mgg::SensorType::kLidar;
    sensor.max_range = 4.0;
    sensor.fov = {2 * M_PI, M_PI / 2};
    sensor.resolution = {M_PI / 12, M_PI / 12};
    sensor.update();
    std::unordered_map<std::string, mgg::SensorParams> sensors{{"lidar", sensor}};
    planning.exp_sensor_list = {"lidar"};
    planning.unknown_voxel_gain = 1.0;
    mgg::BoundedSpaceParams region;
    region.min_val = {-5, -5, 0.8};
    region.max_val = {5, 5, 2.2};
    region.setCenter(Eigen::Vector3d(0, 0, 0), false);
    mgg::GainContext gain;
    gain.map = const_cast<mgg::MapInterface*>(ctx.map);
    gain.robot = &robot;
    gain.planning = &planning;
    gain.global_space = &region;
    gain.gain_region = &region;
    gain.sensors = &sensors;
    mgg::computeExplorationGain(graph, gain, false, false);
    return mgg::selectBestPath(graph, planning, robot, inclinations, 0.2, 0.0);
  }
  StateVec start{0.1, 0.1, 1.5, 0};
  mgg::RobotParams robot;
  mgg::PlanningParams planning;
  mgg::ExpandContext ctx;
  mgg::GraphManager graph;
  mgg::EdgeInclinations inclinations;
};

TEST(AerialExploration, WalledRoomKeepsVerticesAndSelectedPathObservedFree) {
  auto map = room();
  Aerial drone(*map);
  drone.build();
  bool reached_gap = false;
  for (const auto& [id, v] : drone.graph.vertices_map_) {
    EXPECT_EQ(map->getStrictBoxStatus(v->state.head<3>(), drone.robot.size),
              VoxelStatus::kFree) << id << ": " << v->state.transpose();
    if (v->state.x() > 2.4) reached_gap = true;
    EXPECT_GT(v->state.y(), -2.0);
    EXPECT_LT(v->state.y(), 2.0);
    EXPECT_LT(v->state.x(), 3.0);
  }
  EXPECT_TRUE(reached_gap);  // not a vacuous no-motion safety test
  const auto chosen = drone.select();
  ASSERT_GE(chosen.best_path.size(), 2u);
  for (std::size_t i = 1; i < chosen.best_path.size(); ++i) {
    EXPECT_EQ(map->getStrictPathStatus(chosen.best_path[i - 1]->state.head<3>(),
                                      chosen.best_path[i]->state.head<3>(),
                                      drone.robot.size), VoxelStatus::kFree);
  }
  // Check every graph edge, not merely the chosen tree path.
  drone.graph.setEdgeBlocked([&](const mgg::Vertex& a, const mgg::Vertex& b) {
    EXPECT_EQ(map->getStrictPathStatus(a.state.head<3>(), b.state.head<3>(),
                                      drone.robot.size), VoxelStatus::kFree);
    return false;
  });
  mgg::ShortestPathsReport paths;
  drone.graph.findShortestPaths(paths);
}

TEST(AerialExploration, UnknownHoverBodyCanDepartButNotUnknownJustOutsideIt) {
  for (bool outside : {false, true}) {
    auto map = room(true, outside);
    Aerial drone(*map);
    mgg::Vertex target(1, StateVec(0.9, 0.1, 1.5, 0));
    mgg::ExpandGraphReport report;
    mgg::expandGraph(drone.graph, target, report, drone.ctx);
    EXPECT_EQ(report.num_vertices_added, outside ? 0 : 1);
  }
  auto map = room(true);
  Aerial drone(*map);
  drone.build();
  ASSERT_GE(drone.select().best_path.size(), 2u);
}

TEST(AerialExploration, FreeEndpointCannotBridgeUnknownOrAnOccupiedWall) {
  for (bool wall : {false, true}) {
    std::vector<Cell> free, occupied;
    for (int x = -2; x < 12; ++x)
      for (int y = -2; y < 3; ++y)
        for (int z = 5; z < 10; ++z) {
          if (x == 4) { if (wall) occupied.push_back({x, y, z}); }
          else free.push_back({x, y, z});
        }
    NativeMolaGrid map(0.2, occupied, free, {});
    Aerial drone(map);
    drone.planning.edge_length_max = 3;
    mgg::Vertex target(1, StateVec(1.7, 0.1, 1.5, 0));
    mgg::ExpandGraphReport report;
    mgg::expandGraph(drone.graph, target, report, drone.ctx);
    EXPECT_EQ(report.num_vertices_added, 0) << "wall=" << wall;
  }
}

TEST(AerialExploration, ClippedEndpointInsideBlindRootIsNotAViewpoint) {
  auto map = room(true);
  Aerial drone(*map);
  drone.planning.edge_length_max = 0.05;
  mgg::Vertex target(1, StateVec(0.9, 0.1, 1.5, 0));
  mgg::ExpandGraphReport report;
  mgg::expandGraph(drone.graph, target, report, drone.ctx);
  EXPECT_EQ(report.num_vertices_added, 0);
}

TEST(AerialExploration, RootExemptionNeverIgnoresOccupiedMeasuredSurface) {
  std::vector<Cell> free;
  for (int x = -2; x < 8; ++x)
    for (int y = -2; y < 3; ++y)
      for (int z = 5; z < 10; ++z) free.push_back({x, y, z});
  // The root body touches voxel z=6, but sits above the measured return.
  NativeMolaGrid map(0.2, {{0, 0, 6}}, free, {{{0, 0, 6}, 1.21}});
  Aerial drone(map);
  mgg::Vertex target(1, StateVec(0.9, 0.1, 1.5, 0));
  mgg::ExpandGraphReport report;
  mgg::expandGraph(drone.graph, target, report, drone.ctx);
  EXPECT_EQ(report.num_vertices_added, 0);
}
}  // namespace
