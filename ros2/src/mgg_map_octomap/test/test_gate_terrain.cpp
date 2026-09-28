// Saved Scout revision-50 gate grid, with occupied/free/surface evidence intact.
#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <sstream>
#include "mgg_core/ground_projection.h"
#include "mgg_core/grid_graph.h"
#include "mgg_map_octomap/native_mola_grid.h"

namespace {
mgg::NativeMolaGrid gate() {
  std::ifstream input(std::string(MGG_MAP_TEST_DATA_DIR) + "/gate_scout_grid.txt");
  std::vector<mgg::NativeMolaGrid::Cell> occupied, free;
  std::vector<mgg::NativeMolaGrid::Surface> surfaces;
  std::string line;
  while (std::getline(input, line)) {
    std::istringstream fields(line);
    std::string kind; std::int64_t x, y; double z;
    if (!(fields >> kind >> x >> y >> z)) continue;
    if (kind == "surface") surfaces.push_back({{x,y,static_cast<std::int64_t>(std::floor(z/.2))},z});
    else (kind == "occupied" ? occupied : free).push_back({x,y,static_cast<std::int64_t>(z)});
  }
  return mgg::NativeMolaGrid(.2, occupied, free, surfaces);
}
mgg::PlanningParams scout() {
  mgg::PlanningParams p;
  p.max_step_height=.15; p.max_ground_height=.4475;
  p.max_inclination=27*M_PI/180; p.max_cross_slope=18*M_PI/180;
  p.max_footprint_tilt=25*M_PI/180; p.max_footprint_step=.08;
  p.min_observed_ground_fraction=.75;
  return p;
}
const Eigen::Vector3d box(.662,.630,.295);
mgg::ProjectedEdgeStatus edge(mgg::GroundProjection& ground, double x, double y,
                              double yaw=0., double driving_height=.4475,
                              const Eigen::Vector3d& body=box) {
  Eigen::Vector3d a(x+16.5,y-1.,.4);
  Eigen::Vector3d b=a+Eigen::Vector3d(.25*std::cos(yaw),.25*std::sin(yaw),0);
  for (auto* point : {&a,&b}) {
    mgg::VoxelStatus status;
    const double drop=ground.projectSample(*point,status);
    EXPECT_EQ(status,mgg::VoxelStatus::kOccupied);
    point->z()-=drop-driving_height;
  }
  std::vector<Eigen::Vector3d> path;
  return ground.getProjectedEdgeStatus(a,b,body,false,path,false);
}
TEST(GateTerrain, EnabledScoutRiseRejectsPreviouslyDrivableRubbleToe) {
  auto map=gate(); auto p=scout(); mgg::GroundProjection ground(map,p,true);
  const double heading=62.26938*M_PI/180;
  EXPECT_EQ(edge(ground,-11.4,-1.5,heading),mgg::ProjectedEdgeStatus::kAdmissible);
  p.max_footprint_cell_rise=.075;
  EXPECT_EQ(edge(ground,-11.4,-1.5,heading),mgg::ProjectedEdgeStatus::kFootprintPlane);
}
TEST(GateTerrain, RiseAddsNoRejectionInBaselineAdmissibleCentralStrip) {
  auto map=gate(); auto p=scout(); mgg::GroundProjection ground(map,p,true);
  int admissible=0;
  for(int i=-8;i<=8;++i) {
    p.max_footprint_cell_rise=0.;
    const auto baseline=edge(ground,-10.5,i*.1);
    p.max_footprint_cell_rise=.075;
    EXPECT_EQ(edge(ground,-10.5,i*.1),baseline) << "world y=" << i*.1;
    if(baseline==mgg::ProjectedEdgeStatus::kAdmissible) ++admissible;
  }
  EXPECT_EQ(admissible,15); // +0.7/+0.8 already fail body collision in this map.
}
TEST(GateTerrain, BunkerRiseKeepsObservedAdmissibleHangarFloor) {
  auto map=gate(); auto p=scout();
  p.max_ground_height=.525; p.max_footprint_tilt=20*M_PI/180;
  p.max_footprint_step=.10;
  mgg::GroundProjection ground(map,p,true);
  const Eigen::Vector3d body(1.073,.828,.45);
  int admissible=0;
  // Cross the mapped approach and doorway, including any observed floor
  // seams. The saved grid does not identify cable ducts semantically.
  for(int ix=0;ix<=12;++ix) for(int iy=-4;iy<=4;++iy) {
    const double x=-13.+ix*.2, y=iy*.1;
    p.max_footprint_cell_rise=0.;
    const auto baseline=edge(ground,x,y,0.,.525,body);
    p.max_footprint_cell_rise=.12;
    if(baseline==mgg::ProjectedEdgeStatus::kAdmissible) {
      ++admissible;
      EXPECT_EQ(edge(ground,x,y,0.,.525,body),baseline) << x << "," << y;
    }
  }
  EXPECT_GT(admissible,50);
}
TEST(GateTerrain, DisabledRiseClearanceRouteAvoidsSouthToe) {
  auto map = gate();
  auto p = scout();
  p.max_footprint_cell_rise = 0.0;
  p.path_clearance_margin = 0.6;
  p.rr_mode = mgg::RRModeType::kGraph;
  p.edge_length_min = 0.01;
  p.edge_length_max = 0.6;
  p.edge_overshoot = 0.0;
  p.nearest_range = 0.3;
  p.nearest_range_min = 0.01;
  p.nearest_range_max = 0.31;
  p.num_vertices_max = 3000;
  p.num_edges_max = 30000;
  p.num_loops_max = 50000;
  mgg::GroundProjection ground(map, p, true);
  mgg::RobotParams robot;
  robot.type = mgg::RobotType::kGroundRobot;
  robot.size = {0.612, 0.580, 0.245};
  robot.size_extension = {0.05, 0.05, 0.05};
  const auto driving = [&](double world_x, double world_y) {
    Eigen::Vector3d point(world_x + 16.5, world_y - 1, 0.4);
    mgg::VoxelStatus status;
    const double drop = ground.projectSample(point, status);
    EXPECT_EQ(status, mgg::VoxelStatus::kOccupied);
    point.z() -= drop - p.max_ground_height;
    return mgg::StateVec(point.x(), point.y(), point.z(), 0);
  };
  const auto start = driving(-12.5, -1.4);
  const auto goal = driving(-10.1, 0.0);
  mgg::GraphManager graph;
  graph.addVertex(new mgg::Vertex(0, start));
  mgg::ExpandContext ctx;
  ctx.map = &map;
  ctx.planning = &p;
  ctx.robot = &robot;
  ctx.ground = &ground;
  ctx.robot_box_size = box;
  mgg::GridGraphParams grid;
  grid.min_val = {-0.8, -1.0, 0};
  grid.max_val = {3.6, 3.0, 0};
  grid.resolution = {0.2, 0.2, 0.1};
  mgg::buildGridGraph(graph, start, grid, ctx, 0);
  mgg::Vertex* end = nullptr;
  ASSERT_TRUE(graph.getNearestVertex(&goal, &end));
  ASSERT_NE(end, nullptr);
  ASSERT_LT((end->state.head<2>() - goal.head<2>()).norm(), 0.01);
  mgg::ShortestPathsReport report;
  ASSERT_TRUE(graph.findShortestPaths(0, report));
  std::vector<mgg::Vertex*> path;
  graph.getShortestPath(end->id, report, true, path);
  ASSERT_GT(path.size(), 2u);
  bool crossed = false;
  double lowest_y = 100;
  for (size_t i = 1; i < path.size(); ++i) {
    for (int k = 0; k <= 20; ++k) {
      const Eigen::Vector3d point = path[i - 1]->state.head<3>() +
          (k / 20.0) * (path[i]->state - path[i - 1]->state).head<3>();
      const double x = point.x() - 16.5, y = point.y() + 1;
      if (x >= -11.8 && x <= -10.8) {
        crossed = true;
        lowest_y = std::min(lowest_y, y);
        EXPECT_GT(y, -1.0) << "world x=" << x;
      }
    }
  }
  EXPECT_TRUE(crossed);
  std::printf("native gate rise=0 margin=0.6: %zu poses, minimum y %.3f\n",
              path.size(), lowest_y);
}
} // namespace
