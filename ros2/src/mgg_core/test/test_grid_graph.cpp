// Tests for the grid local planner, the "grid" in Multi-robot Grid Graph.
// The ROS 1 version had none: it ran only inside a full planning cycle.

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/grid_graph.h"

namespace {

using mgg::ExpandContext;
using mgg::GainCounts;
using mgg::GraphManager;
using mgg::GridGraphParams;
using mgg::GridGraphStatus;
using mgg::MapInterface;
using mgg::PlanningParams;
using mgg::RobotParams;
using mgg::RobotType;
using mgg::SensorModel;
using mgg::StateVec;
using mgg::Vertex;
using mgg::VoxelStatus;

/// Open space, with an optional wall at x >= wall_x.
class OpenSpace : public MapInterface {
 public:
  explicit OpenSpace(double wall_x = 1e9) : wall_x_(wall_x) {}

  double getResolution() const override { return 0.2; }
  bool getStatus() const override { return true; }
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    return p.x() >= wall_x_ ? VoxelStatus::kOccupied : VoxelStatus::kFree;
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool s) const override {
    Eigen::Vector3d ignored;
    return getRayStatus(a, b, s, ignored);
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool, Eigen::Vector3d& end_voxel) const override {
    const double len = (b - a).norm();
    if (len < 1e-12) { end_voxel = b; return getVoxelStatus(b); }
    const Eigen::Vector3d dir = (b - a) / len;
    for (double d = 0.0; d <= len; d += 0.2) {
      const Eigen::Vector3d p = a + d * dir;
      if (getVoxelStatus(p) == VoxelStatus::kOccupied) {
        end_voxel = p;
        return VoxelStatus::kOccupied;
      }
    }
    end_voxel = b;
    return VoxelStatus::kFree;
  }
  VoxelStatus getBoxStatus(const Eigen::Vector3d& c, const Eigen::Vector3d& s,
                           bool) const override {
    // Occupied if any corner in x reaches the wall.
    return (c.x() + s.x() / 2.0 >= wall_x_) ? VoxelStatus::kOccupied
                                            : VoxelStatus::kFree;
  }
  VoxelStatus getPathStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                            const Eigen::Vector3d&, bool s) const override {
    return getRayStatus(a, b, s);
  }
  void getScanStatus(const Eigen::Vector3d&,
                     const std::vector<Eigen::Vector3d>&, GainCounts&,
                     std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>&,
                     const SensorModel&) override {}
  void getScanStatusIterative(
      const Eigen::Vector3d&, const std::vector<Eigen::Vector3d>&, GainCounts&,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>&,
      const SensorModel&) override {}
  bool augmentFreeBox(const Eigen::Vector3d&,
                      const Eigen::Vector3d&) override { return true; }
  void augmentFreeFrustum() override {}
  void resetMap() override {}
  void extractLocalMap(const Eigen::Vector3d&, const Eigen::Vector3d&,
                       std::vector<Eigen::Vector3d>&,
                       std::vector<Eigen::Vector3d>&) override {}
  void extractLocalMapAlongAxis(const Eigen::Vector3d&, const Eigen::Vector3d&,
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

 private:
  double wall_x_;
};

/// Ground projection moves raw samples from z=1.0 to driving height z=0.5.
/// The raw lattice box is free, while one projected endpoint is unknown.
class ProjectionMismatchSpace : public OpenSpace {
 public:
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    return p.z() <= 0.0 ? VoxelStatus::kOccupied : VoxelStatus::kFree;
  }
  VoxelStatus getStrictBoxStatus(const Eigen::Vector3d& center,
                                 const Eigen::Vector3d&) const override {
    if (std::abs(center.x() - 0.5) < 1e-9 &&
        std::abs(center.z() - 0.5) < 1e-9) {
      return VoxelStatus::kUnknown;
    }
    return VoxelStatus::kFree;
  }
};

struct Fixture {
  Fixture() {
    robot.type = RobotType::kAerialRobot;  // no ground projection needed
    robot.size = Eigen::Vector3d(0.4, 0.4, 0.4);
    planning.edge_length_min = 0.1;
    planning.edge_length_max = 2.0;
    planning.edge_overshoot = 0.0;
    planning.nearest_range = 1.0;
    planning.nearest_range_min = 0.1;
    planning.nearest_range_max = 100.0;
    planning.nearest_range_z = 100.0;
    planning.num_vertices_max = 5000;
    planning.num_edges_max = 50000;
    planning.num_loops_max = 100000;

    ctx.map = &map;
    ctx.planning = &planning;
    ctx.robot = &robot;
    ctx.robot_box_size = Eigen::Vector3d(0.4, 0.4, 0.4);
    ctx.robot_id = 1;

    // A root the lattice can attach to.
    auto* root = new Vertex(0, StateVec(0, 0, 0, 0));
    root->robot_id = 1;
    graph.addVertex(root);
  }
  OpenSpace map;
  RobotParams robot;
  PlanningParams planning;
  ExpandContext ctx;
  GraphManager graph;
};

GridGraphParams smallGrid() {
  GridGraphParams g;
  g.min_val = Eigen::Vector3d(-1.0, -1.0, 0.0);
  g.max_val = Eigen::Vector3d(1.0, 1.0, 0.0);
  g.resolution = Eigen::Vector3d(0.5, 0.5, 0.5);
  return g;
}

TEST(GridGraph, SweepsTheLatticeAndGrowsTheGraph) {
  Fixture f;
  const auto r = buildGridGraph(f.graph, StateVec(0, 0, 0, 0), smallGrid(),
                                f.ctx, 0.0);
  EXPECT_EQ(r.status, GridGraphStatus::kOk);
  // 5 x 5 x 1 lattice, all free.
  EXPECT_EQ(r.free_cells, 25);
  EXPECT_GT(r.vertices_added, 0);
  EXPECT_GT(f.graph.getNumVertices(), 1);
}

TEST(GridGraph, RejectsBoundsThatDoNotStraddleTheRobot) {
  Fixture f;
  GridGraphParams g = smallGrid();
  g.min_val = Eigen::Vector3d(1.0, 1.0, 0.0);  // entirely ahead of the robot
  const auto r = buildGridGraph(f.graph, StateVec(0, 0, 0, 0), g, f.ctx, 0.0);
  EXPECT_EQ(r.status, GridGraphStatus::kInvalidBounds);
  EXPECT_EQ(r.free_cells, 0);
}

// The ROS 1 code took the resolution from BoundedSpaceParams::min_extension.
// Zeroing the bound extensions, an otherwise sensible configuration change,
// therefore set the resolution to zero. A named field makes the failure
// explicit instead of silent.
TEST(GridGraph, ZeroResolutionIsRejectedExplicitly) {
  Fixture f;
  GridGraphParams g = smallGrid();
  g.resolution = Eigen::Vector3d(0.0, 0.0, 0.0);
  const auto r = buildGridGraph(f.graph, StateVec(0, 0, 0, 0), g, f.ctx, 0.0);
  EXPECT_EQ(r.status, GridGraphStatus::kInvalidBounds);
}

TEST(GridGraph, ObstacleReducesTheFreeCellCount) {
  Fixture f;
  f.map = OpenSpace(0.6);  // wall at x >= 0.6
  const auto r = buildGridGraph(f.graph, StateVec(0, 0, 0, 0), smallGrid(),
                                f.ctx, 0.0);
  EXPECT_EQ(r.status, GridGraphStatus::kOk);
  EXPECT_GT(r.free_cells, 0);
  EXPECT_LT(r.free_cells, 25);
}

TEST(GridGraph, HeadingRotatesTheLattice) {
  Fixture f;
  // A wall to the robot's +x. With no heading the lattice runs into it; turned
  // a quarter turn the same lattice extends along y instead, so more cells are
  // free.
  f.map = OpenSpace(0.6);
  GridGraphParams g;
  g.min_val = Eigen::Vector3d(0.0, -0.5, 0.0);
  g.max_val = Eigen::Vector3d(2.0, 0.5, 0.0);
  g.resolution = Eigen::Vector3d(0.5, 0.5, 0.5);

  GraphManager graph_a;
  auto* ra = new Vertex(0, StateVec(0, 0, 0, 0));
  graph_a.addVertex(ra);
  const auto straight = buildGridGraph(graph_a, StateVec(0, 0, 0, 0), g,
                                       f.ctx, 0.0);

  GraphManager graph_b;
  auto* rb = new Vertex(0, StateVec(0, 0, 0, 0));
  graph_b.addVertex(rb);
  const auto turned = buildGridGraph(graph_b, StateVec(0, 0, 0, 0), g, f.ctx,
                                     M_PI / 2.0);

  EXPECT_GT(turned.free_cells, straight.free_cells);
}

TEST(GridGraph, RespectsTheVertexLimit) {
  Fixture f;
  f.planning.num_vertices_max = 3;
  const auto r = buildGridGraph(f.graph, StateVec(0, 0, 0, 0), smallGrid(),
                                f.ctx, 0.0);
  EXPECT_TRUE(r.hit_limit);
}

TEST(GridGraph, LatticeFollowsTheRobotPosition) {
  Fixture f;
  const auto r = buildGridGraph(f.graph, StateVec(50.0, 50.0, 0.0, 0.0),
                                smallGrid(), f.ctx, 0.0);
  EXPECT_EQ(r.status, GridGraphStatus::kOk);
  EXPECT_EQ(r.free_cells, 25);
}

TEST(GridGraph, ExplicitBuildRejectsUnknownProjectedEndpointAndKeepsAlternative) {
  ProjectionMismatchSpace map;
  RobotParams robot;
  robot.type = RobotType::kGroundRobot;
  robot.size = Eigen::Vector3d(0.2, 0.2, 0.2);
  PlanningParams planning;
  planning.max_ground_height = 0.5;
  planning.max_step_height = 0.2;
  planning.max_inclination = 0.6;
  planning.edge_length_min = 0.1;
  planning.edge_length_max = 2.0;
  planning.edge_overshoot = 0.0;
  planning.nearest_range = 1.1;
  planning.nearest_range_min = 0.1;
  planning.nearest_range_max = 100.0;
  planning.nearest_range_z = 100.0;
  planning.num_vertices_max = 20;
  planning.num_edges_max = 40;
  planning.num_loops_max = 20;
  mgg::GroundProjection ground(map, planning);
  ExpandContext ctx;
  ctx.map = &map;
  ctx.planning = &planning;
  ctx.robot = &robot;
  ctx.ground = &ground;
  ctx.robot_box_size = robot.getPlanningSize();

  GridGraphParams grid;
  grid.min_val = Eigen::Vector3d(0.0, 0.0, 0.0);
  grid.max_val = Eigen::Vector3d(1.0, 0.0, 0.0);
  grid.resolution = Eigen::Vector3d(0.5, 0.5, 0.5);
  const StateVec sample_origin(0.0, 0.0, 1.0, 0.0);

  const auto build = [&](bool strict) {
    auto graph = std::make_unique<GraphManager>();
    graph->addVertex(new Vertex(0, StateVec(0.0, 0.0, 0.5, 0.0)));
    ctx.strict_projected_endpoint = strict;
    buildGridGraph(*graph, sample_origin, grid, ctx, 0.0);
    return graph;
  };
  const auto legacy = build(false);
  const auto explicit_graph = build(true);

  StateVec unknown_endpoint(0.5, 0.0, 0.5, 0.0);
  Vertex* found = nullptr;
  EXPECT_TRUE(legacy->getNearestVertexInRange(&unknown_endpoint, 1e-6, &found));
  found = nullptr;
  EXPECT_FALSE(
      explicit_graph->getNearestVertexInRange(&unknown_endpoint, 1e-6, &found));
  StateVec observed_alternative(1.0, 0.0, 0.5, 0.0);
  found = nullptr;
  EXPECT_TRUE(explicit_graph->getNearestVertexInRange(
      &observed_alternative, 1e-6, &found));
}

}  // namespace
