#include "mgg_core/local_route.h"
// Tests for the grid local planner, the "grid" in Multi-robot Grid Graph.
// The ROS 1 version had none: it ran only inside a full planning cycle.

#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/graph_expansion.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/planning_cancellation.h"
#include "terrain_fixture.h"

namespace {

using mgg::ExpandContext;
using mgg::GainCounts;
using mgg::GraphManager;
using mgg::GridGraphParams;
using mgg::GridGraphResult;
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
  bool getAxisAlignedXYCellCenter(const Eigen::Vector2d& p,
                                 Eigen::Vector2d& center) const override {
    center = (p.array() / 0.2).floor().matrix() * 0.2 +
             Eigen::Vector2d::Constant(0.1);
    return true;
  }
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

class OccupiedProjectionMismatchSpace : public ProjectionMismatchSpace {
 public:
  VoxelStatus getStrictBoxStatus(const Eigen::Vector3d& center,
                                 const Eigen::Vector3d& size) const override {
    const VoxelStatus base =
        ProjectionMismatchSpace::getStrictBoxStatus(center, size);
    return base == VoxelStatus::kUnknown ? VoxelStatus::kOccupied : base;
  }
};

/// A sparse sensor map has known ground at the candidate but does not prove
/// the whole body volume free. The relaxed query still preserves a known
/// obstacle, matching OctomapMap's stop-at-unknown contract.
class SparseBodySpace : public OpenSpace {
 public:
  SparseBodySpace(bool has_ground, bool occupied_candidate = false,
                  bool invalid_candidate = false)
      : has_ground_(has_ground),
        occupied_candidate_(occupied_candidate),
        invalid_candidate_(invalid_candidate) {}

  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    return has_ground_ && p.z() <= 0.0 ? VoxelStatus::kOccupied
                                       : VoxelStatus::kFree;
  }

  VoxelStatus getBoxStatus(const Eigen::Vector3d& center,
                           const Eigen::Vector3d&,
                           bool stop_at_unknown) const override {
    if (center.x() < 0.25) return VoxelStatus::kFree;
    if (occupied_candidate_) return VoxelStatus::kOccupied;
    if (invalid_candidate_) return VoxelStatus::kUnknown;
    return stop_at_unknown ? VoxelStatus::kUnknown : VoxelStatus::kFree;
  }

 private:
  bool has_ground_;
  bool occupied_candidate_;
  bool invalid_candidate_;
};

struct SparseGroundFixture {
  explicit SparseGroundFixture(bool has_ground, bool occupied_candidate = false,
                               bool invalid_candidate = false)
      : map(has_ground, occupied_candidate, invalid_candidate),
        ground(map, planning) {
    robot.type = RobotType::kGroundRobot;
    robot.size = Eigen::Vector3d(0.4, 0.4, 0.4);
    planning.max_ground_height = 0.5;
    // Ground under only part of the body, which the unobserved-ground check
    // (min_observed_ground_fraction, tested on its own) would refuse.
    planning.min_observed_ground_fraction = 0.0;
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
    ctx.map = &map;
    ctx.planning = &planning;
    ctx.robot = &robot;
    ctx.ground = &ground;
    ctx.robot_box_size = robot.getPlanningSize();
    graph.addVertex(new Vertex(0, StateVec(0.0, 0.0, 0.5, 0.0)));
  }

  GridGraphParams grid() const {
    GridGraphParams value;
    value.min_val = Eigen::Vector3d(0.0, 0.0, 0.0);
    value.max_val = Eigen::Vector3d(0.5, 0.0, 0.0);
    value.resolution = Eigen::Vector3d(0.5, 0.5, 0.5);
    return value;
  }

  SparseBodySpace map;
  RobotParams robot;
  PlanningParams planning;
  mgg::GroundProjection ground;
  ExpandContext ctx;
  GraphManager graph;
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

TEST(GridGraph, CancelledRequestStopsAtLatticeCheckpoint) {
  Fixture f;
  mgg::PlanningCancellationScope token([] { return true; });
  EXPECT_THROW(buildGridGraph(f.graph, StateVec(0, 0, 0, 0), smallGrid(),
                             f.ctx, 0.0), mgg::PlanningInterrupted);
  EXPECT_EQ(f.graph.getNumVertices(), 1);
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

TEST(GridGraph, StrictDefaultRejectsUnknownLatticeBody) {
  SparseGroundFixture fixture(/*has_ground=*/true);
  const GridGraphResult result = buildGridGraph(
      fixture.graph, StateVec(0.0, 0.0, 0.5, 0.0), fixture.grid(),
      fixture.ctx, 0.0);

  EXPECT_EQ(result.free_cells, 1);
  EXPECT_EQ(result.vertices_added, 0);
  EXPECT_EQ(fixture.graph.getNumVertices(), 1);
}

TEST(GridGraph, QualifiedPolicyAdmitsUnknownBodyWithMeasuredGround) {
  SparseGroundFixture fixture(/*has_ground=*/true);
  fixture.ctx.allow_unknown_lattice_body = true;
  const GridGraphResult result = buildGridGraph(
      fixture.graph, StateVec(0.0, 0.0, 0.5, 0.0), fixture.grid(),
      fixture.ctx, 0.0);

  EXPECT_EQ(result.free_cells, 2);
  EXPECT_EQ(result.vertices_added, 1);
  StateVec candidate(0.5, 0.0, 0.5, 0.0);
  Vertex* found = nullptr;
  EXPECT_TRUE(fixture.graph.getNearestVertexInRange(&candidate, 0.11, &found));
}

TEST(GridGraph, QualifiedPolicyStillRequiresMeasuredGround) {
  SparseGroundFixture fixture(/*has_ground=*/false);
  fixture.ctx.allow_unknown_lattice_body = true;
  const GridGraphResult result = buildGridGraph(
      fixture.graph, StateVec(0.0, 0.0, 0.5, 0.0), fixture.grid(),
      fixture.ctx, 0.0);

  EXPECT_EQ(result.free_cells, 2);
  EXPECT_EQ(result.no_ground, 1);
  EXPECT_EQ(result.vertices_added, 0);
}

TEST(GridGraph, QualifiedPolicyStillRejectsKnownOccupiedBody) {
  SparseGroundFixture fixture(/*has_ground=*/true,
                              /*occupied_candidate=*/true);
  fixture.ctx.allow_unknown_lattice_body = true;
  const GridGraphResult result = buildGridGraph(
      fixture.graph, StateVec(0.0, 0.0, 0.5, 0.0), fixture.grid(),
      fixture.ctx, 0.0);

  EXPECT_EQ(result.free_cells, 1);
  EXPECT_EQ(result.vertices_added, 0);
}

TEST(GridGraph, QualifiedPolicyRejectsInvalidUnknownBodyQuery) {
  SparseGroundFixture fixture(/*has_ground=*/true,
                              /*occupied_candidate=*/false,
                              /*invalid_candidate=*/true);
  fixture.ctx.allow_unknown_lattice_body = true;
  const GridGraphResult result = buildGridGraph(
      fixture.graph, StateVec(0.0, 0.0, 0.5, 0.0), fixture.grid(),
      fixture.ctx, 0.0);

  EXPECT_EQ(result.free_cells, 1);
  EXPECT_EQ(result.vertices_added, 0);
}

TEST(GridGraph, HangingRootLimitOverridesLongOrdinaryEdgeLimit) {
  Fixture f;
  f.planning.edge_length_max = 10.0;
  f.ctx.hanging_root_edge_length_max = 1.0;
  f.graph.vertices_map_.at(0)->is_hanging = true;
  Vertex candidate(1, StateVec(5.0, 0.0, 0.0, 0.0));
  mgg::ExpandGraphReport report;

  mgg::expandGraph(f.graph, candidate, report, f.ctx);

  ASSERT_EQ(report.num_vertices_added, 1);
  StateVec bounded(1.0, 0.0, 0.0, 0.0);
  Vertex* found = nullptr;
  EXPECT_TRUE(f.graph.getNearestVertexInRange(&bounded, 1e-9, &found));
  StateVec unbounded(5.0, 0.0, 0.0, 0.0);
  found = nullptr;
  EXPECT_FALSE(f.graph.getNearestVertexInRange(&unbounded, 1e-9, &found));
}

TEST(GridGraph, ProjectedEdgePolicyRejectsCandidateBeforeAdmission) {
  Fixture f;
  int checks = 0;
  f.ctx.projected_edge_admissible =
      [&checks](const std::vector<Eigen::Vector3d>&) {
        ++checks;
        return false;
      };
  const auto r = buildGridGraph(f.graph, StateVec(0, 0, 0, 0), smallGrid(),
                                f.ctx, 0.0);
  EXPECT_EQ(r.status, GridGraphStatus::kOk);
  EXPECT_GT(checks, 0);
  EXPECT_EQ(r.vertices_added, 0);
  EXPECT_EQ(f.graph.getNumVertices(), 1);
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

// Run 8: the lattice was swept from its rear row to its front, so the
// 1500-vertex limit cut the rows ahead of the robot (robot_3 at a corridor
// junction reached +2.8 m of the 6 m ahead). The sweep now grows outward
// from the robot, and a limit drops the farthest cells.
TEST(GridGraph, VertexLimitDropsTheFarthestCellsFirst) {
  Fixture f;
  f.planning.num_vertices_max = 9;
  GridGraphParams g;
  g.min_val = Eigen::Vector3d(-2.0, -2.0, 0.0);
  g.max_val = Eigen::Vector3d(2.0, 2.0, 0.0);
  g.resolution = Eigen::Vector3d(0.5, 0.5, 0.5);
  const auto r = buildGridGraph(f.graph, StateVec(0, 0, 0, 0), g, f.ctx, 0.0);
  EXPECT_TRUE(r.hit_limit);
  ASSERT_EQ(f.graph.getNumVertices(), 9);
  // The robot and the eight cells around it, none farther.
  for (const auto& [id, v] : f.graph.vertices_map_) {
    EXPECT_LE(v->state.head<2>().norm(), std::sqrt(0.5) + 1e-9) << id;
  }
}

// Run 8: every lattice column's six z levels projected to the same ground,
// and each was kept: 4.9 vertices per position, so the 1500-vertex limit
// held about 300 positions of the 961 the lattice has.
TEST(GridGraph, GroundLatticeKeepsOneVertexPerColumnOfOneFloor) {
  std::map<std::pair<std::int64_t, std::int64_t>, double> tops;
  for (std::int64_t x = -10; x < 10; ++x) {
    for (std::int64_t y = -10; y < 10; ++y) tops[{x, y}] = 0.0;
  }
  const mgg_test::TerrainFixture map(0.2, tops);
  RobotParams robot;
  robot.type = RobotType::kGroundRobot;
  robot.size = Eigen::Vector3d(0.4, 0.4, 0.4);
  PlanningParams planning;
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
  const mgg::GroundProjection ground(map, planning);
  ExpandContext ctx;
  ctx.map = &map;
  ctx.planning = &planning;
  ctx.robot = &robot;
  ctx.ground = &ground;
  ctx.robot_box_size = robot.getPlanningSize();
  ctx.root_is_robot = true;
  GraphManager graph;
  graph.addVertex(new Vertex(0, StateVec(0.1, 0.1, 0.5, 0.0)));
  GridGraphParams g;
  g.min_val = Eigen::Vector3d(-0.8, -0.8, -0.2);
  g.max_val = Eigen::Vector3d(0.8, 0.8, 0.3);
  g.resolution = Eigen::Vector3d(0.4, 0.4, 0.1);
  const auto r = buildGridGraph(graph, StateVec(0.1, 0.1, 0.5, 0.0), g, ctx,
                                0.0);
  EXPECT_EQ(r.status, GridGraphStatus::kOk);
  // All 25 columns, the robot's included, once each.
  EXPECT_EQ(graph.getNumVertices(), 25);
  EXPECT_GT(r.merged_duplicates, 0);
  std::set<std::pair<long, long>> columns;
  for (const auto& [id, v] : graph.vertices_map_) {
    columns.insert({std::lround(v->state.x() * 10), std::lround(v->state.y() * 10)});
  }
  EXPECT_EQ(columns.size(), graph.getNumVertices());
}

// The merge is by ground height: within a step it is the same ground, more
// than a step apart another level (a floor under a walkway), kept apart.
TEST(GridGraph, LatticeColumnGroundMergesWithinAStepAndKeepsOtherLevels) {
  mgg::LatticeColumnGround columns(0.15);
  EXPECT_FALSE(columns.holds(2, 3, 0.5));
  columns.add(2, 3, 0.5);
  EXPECT_TRUE(columns.holds(2, 3, 0.5));
  EXPECT_TRUE(columns.holds(2, 3, 0.64));
  EXPECT_TRUE(columns.holds(2, 3, 0.36));
  EXPECT_FALSE(columns.holds(2, 3, 0.66));
  EXPECT_FALSE(columns.holds(2, 3, 2.5));
  EXPECT_FALSE(columns.holds(3, 3, 0.5));
  columns.add(2, 3, 2.5);
  EXPECT_TRUE(columns.holds(2, 3, 2.5));
  EXPECT_TRUE(columns.holds(2, 3, 0.5));
}

/// Occupied everywhere, counting its box probes.
class OccupiedEverywhere : public OpenSpace {
 public:
  OccupiedEverywhere() : OpenSpace(-1e9) {}
  VoxelStatus getBoxStatus(const Eigen::Vector3d& c, const Eigen::Vector3d& s,
                           bool stop) const override {
    ++probes;
    return OpenSpace::getBoxStatus(c, s, stop);
  }
  mutable int probes = 0;
};

// Review r1, R1-4: the retry refactoring charged the loop budget only for
// cells whose body box was free, so a lattice wholly occupied or unknown
// probed every cell and never reported hit_limit. Every swept cell is
// charged before it is probed again.
TEST(GridGraph, TheLoopCapBoundsAnOccupiedLattice) {
  Fixture f;
  OccupiedEverywhere occupied;
  f.ctx.map = &occupied;
  f.planning.num_loops_max = 10;
  GridGraphParams g;
  g.min_val = Eigen::Vector3d(-2.0, -2.0, 0.0);
  g.max_val = Eigen::Vector3d(2.0, 2.0, 0.0);
  g.resolution = Eigen::Vector3d(0.5, 0.5, 0.5);  // 81 cells
  const auto r = buildGridGraph(f.graph, StateVec(0, 0, 0, 0), g, f.ctx, 0.0);
  EXPECT_TRUE(r.hit_limit);
  EXPECT_EQ(r.free_cells, 0);
  EXPECT_LE(occupied.probes, 11);
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
  // Ground under only part of the body, which the unobserved-ground check
  // (min_observed_ground_fraction, tested on its own) would refuse.
  planning.min_observed_ground_fraction = 0.0;
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

  const auto build = [&](bool strict, GridGraphResult* result = nullptr) {
    auto graph = std::make_unique<GraphManager>();
    graph->addVertex(new Vertex(0, StateVec(0.0, 0.0, 0.5, 0.0)));
    ctx.strict_projected_endpoint = strict;
    const GridGraphResult built =
        buildGridGraph(*graph, sample_origin, grid, ctx, 0.0);
    if (result != nullptr) *result = built;
    return graph;
  };
  const auto legacy = build(false);
  GridGraphResult explicit_result;
  const auto explicit_graph = build(true, &explicit_result);

  EXPECT_EQ(explicit_result.projected_endpoint_unknown, 1);
  EXPECT_EQ(explicit_result.projected_endpoint_occupied, 0);

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

TEST(GridGraph, CountsOccupiedProjectedEndpointSeparatelyFromUnknown) {
  OccupiedProjectionMismatchSpace map;
  RobotParams robot;
  robot.type = RobotType::kGroundRobot;
  robot.size = Eigen::Vector3d(0.2, 0.2, 0.2);
  PlanningParams planning;
  planning.max_ground_height = 0.5;
  planning.max_step_height = 0.2;
  planning.max_inclination = 0.6;
  planning.edge_length_min = 0.1;
  planning.edge_length_max = 2.0;
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
  ctx.strict_projected_endpoint = true;
  GridGraphParams grid;
  grid.min_val = Eigen::Vector3d(0.0, 0.0, 0.0);
  grid.max_val = Eigen::Vector3d(0.5, 0.0, 0.0);
  grid.resolution = Eigen::Vector3d(0.5, 0.5, 0.5);
  GraphManager graph;
  graph.addVertex(new Vertex(0, StateVec(0.0, 0.0, 0.5, 0.0)));

  const GridGraphResult result = buildGridGraph(
      graph, StateVec(0.0, 0.0, 1.0, 0.0), grid, ctx, 0.0);
  EXPECT_EQ(result.projected_endpoint_occupied, 1);
  EXPECT_EQ(result.projected_endpoint_unknown, 0);
}

TEST(GridGraph, HomeOnALedgeIsNotEnteredWithItsFrontOverTheDrop) {
  // Review r3, I-1b: in the global roadmap vertex zero is home, not the
  // robot, and the robot drives into it. Home stands on level ground that
  // ends at x = 0.6 (a ledge over unobserved space), 0.2 m from its edge; a
  // new vertex lies 0.4 m back from it, away from the drop. The edge from
  // home to it is driven the other way, into home, arriving with only 1/3
  // of the leading half of a Bunker's footprint on observed ground: it is
  // refused. Only where vertex zero is the robot itself (the local
  // lattice) is an edge out of it checked outwards alone.
  std::map<std::pair<std::int64_t, std::int64_t>, double> tops;
  for (std::int64_t x = -10; x < 3; ++x) {
    for (std::int64_t y = -10; y < 10; ++y) tops[{x, y}] = 0.0;
  }
  for (const bool root_is_robot : {false, true}) {
    SCOPED_TRACE(root_is_robot ? "vertex zero is the robot"
                               : "vertex zero is home");
    const mgg_test::TerrainFixture map(0.2, tops);
    RobotParams robot;
    robot.type = RobotType::kGroundRobot;
    robot.size = Eigen::Vector3d(1.023, 0.778, 0.4);
    PlanningParams planning;
    planning.max_ground_height = 0.525;
    planning.max_step_height = 0.15;
    planning.max_inclination = 0.6;
    planning.edge_length_min = 0.1;
    planning.edge_length_max = 2.0;
    planning.edge_overshoot = 0.0;
    planning.nearest_range = 1.1;
    planning.nearest_range_min = 0.1;
    planning.nearest_range_max = 100.0;
    planning.nearest_range_z = 100.0;
    const mgg::GroundProjection ground(map, planning);
    mgg::ExpandContext ctx;
    ctx.map = &map;
    ctx.planning = &planning;
    ctx.robot = &robot;
    ctx.ground = &ground;
    ctx.robot_box_size = robot.getPlanningSize();
    ctx.stop_at_unknown = !root_is_robot;
    ctx.root_is_robot = root_is_robot;
    GraphManager graph;
    graph.addVertex(new Vertex(0, StateVec(0.4, 0.1, 0.525, 0.0)));
    Vertex candidate(1, StateVec(0.0, 0.1, 0.525, 0.0));
    mgg::ExpandGraphReport report;
    mgg::expandGraph(graph, candidate, report, ctx);
    if (root_is_robot) {
      EXPECT_EQ(report.num_vertices_added, 1);
    } else {
      EXPECT_EQ(report.num_vertices_added, 0);
      EXPECT_EQ(report.edge_status[static_cast<int>(
                    mgg::ProjectedEdgeStatus::kGroundUnobserved)],
                1);
    }
  }
}

/// Ground at several levels: each 0.2 m column holds the tops of the solids
/// in it, each solid 0.2 m thick, and every other cell is free. A vertical
/// ray stops on the first top below where it starts.
class StackedFloors : public OpenSpace {
 public:
  void add(double x0, double x1, double y0, double y1,
           const std::function<double(double, double)>& top) {
    for (auto x = key(x0); x < key(x1); ++x) {
      for (auto y = key(y0); y < key(y1); ++y) {
        tops_[{x, y}].push_back(top((x + 0.5) * 0.2, (y + 0.5) * 0.2));
      }
    }
  }
  bool getAxisAlignedXYCellCenter(const Eigen::Vector2d& p,
                                  Eigen::Vector2d& center) const override {
    center = Eigen::Vector2d(key(p.x()) + 0.5, key(p.y()) + 0.5) * 0.2;
    return true;
  }
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    for (const double top : column(p)) {
      if (p.z() <= top && p.z() >= top - 0.2) return VoxelStatus::kOccupied;
    }
    return VoxelStatus::kFree;
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool, Eigen::Vector3d& end_voxel) const override {
    end_voxel = b;
    double best = -1e9;
    for (const double top : column(a)) {
      if (top <= a.z() + 1e-9 && top >= b.z() && top > best) best = top;
    }
    if (best < -1e8) return VoxelStatus::kFree;
    end_voxel = Eigen::Vector3d((key(a.x()) + 0.5) * 0.2,
                                (key(a.y()) + 0.5) * 0.2, best);
    return VoxelStatus::kOccupied;
  }
  VoxelStatus getBoxStatus(const Eigen::Vector3d& c, const Eigen::Vector3d& size,
                           bool) const override {
    const Eigen::Vector3d lo = c - 0.5 * size, hi = c + 0.5 * size;
    for (auto x = key(lo.x()); x <= key(hi.x()); ++x) {
      for (auto y = key(lo.y()); y <= key(hi.y()); ++y) {
        const auto it = tops_.find({x, y});
        if (it == tops_.end()) continue;
        for (const double top : it->second) {
          if (top >= lo.z() && top - 0.2 <= hi.z()) return VoxelStatus::kOccupied;
        }
      }
    }
    return VoxelStatus::kFree;
  }
  VoxelStatus getPathStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                            const Eigen::Vector3d& size, bool s) const override {
    const double steps = std::max(1.0, std::ceil((b - a).norm() / 0.2));
    const Eigen::Vector3d step = (b - a) / steps;
    for (int i = 0; i < static_cast<int>(steps); ++i) {
      if (getBoxStatus(a + (i + 0.5) * step, size + step.cwiseAbs(), s) !=
          VoxelStatus::kFree) {
        return VoxelStatus::kOccupied;
      }
    }
    return VoxelStatus::kFree;
  }

 private:
  static std::int64_t key(double v) {
    return static_cast<std::int64_t>(std::floor(v / 0.2));
  }
  std::vector<double> column(const Eigen::Vector3d& p) const {
    const auto it = tops_.find({key(p.x()), key(p.y())});
    return it == tops_.end() ? std::vector<double>{} : it->second;
  }
  std::map<std::pair<std::int64_t, std::int64_t>, std::vector<double>> tops_;
};

// Review r0 (mgg-run8), multi-level connectivity: a deck 1.2 m over the
// robot's floor, near it, is reached only by driving outward first: along
// the floor to a ramp 4.6 m out, up it, back along an upper walkway and
// onto the deck. The lattice sweeps outward, so the deck's columns come
// before the ramp that leads to them; the lattice must still join the
// deck, and keep the floor under it as a level of its own.
TEST(GridGraph, ADeckOverTheFloorReachedOnlyByGoingOutwardFirstIsJoined) {
  StackedFloors map;
  const auto flat = [](double level) {
    return [level](double, double) { return level; };
  };
  map.add(-1.0, 6.0, -0.6, 0.6, flat(0.0));             // floor
  map.add(4.6, 6.0, 0.6, 3.0, [](double, double y) {    // ramp, 26.6 deg
    return 0.5 * (y - 0.6);
  });
  map.add(0.4, 6.0, 3.0, 3.8, flat(1.2));               // upper walkway
  map.add(0.4, 2.0, 0.6, 3.0, flat(1.2));               // bridge
  map.add(0.4, 2.0, -0.6, 0.6, flat(1.2));              // deck over the floor
  RobotParams robot;
  robot.type = RobotType::kGroundRobot;
  robot.size = Eigen::Vector3d(0.4, 0.4, 0.3);
  PlanningParams planning;
  planning.max_ground_height = 0.3;
  planning.max_step_height = 0.15;
  planning.max_inclination = 0.52;
  planning.edge_length_min = 0.05;
  planning.edge_length_max = 0.8;
  planning.edge_overshoot = 0.0;
  planning.nearest_range = 0.6;
  planning.nearest_range_min = 0.05;
  planning.nearest_range_max = 1.0;
  planning.nearest_range_z = 0.3;
  planning.num_vertices_max = 5000;
  planning.num_edges_max = 50000;
  planning.num_loops_max = 100000;
  planning.min_observed_ground_fraction = 0.0;
  const mgg::GroundProjection ground(map, planning);
  ExpandContext ctx;
  ctx.map = &map;
  ctx.planning = &planning;
  ctx.robot = &robot;
  ctx.ground = &ground;
  ctx.robot_box_size = robot.getPlanningSize();
  ctx.root_is_robot = true;
  GraphManager graph;
  const StateVec root(0.1, 0.1, 0.3, 0.0);
  graph.addVertex(new Vertex(0, root));
  GridGraphParams g;
  g.min_val = Eigen::Vector3d(-0.8, -0.8, -0.2);
  g.max_val = Eigen::Vector3d(6.0, 4.0, 1.4);
  g.resolution = Eigen::Vector3d(0.4, 0.4, 0.2);
  const auto r = buildGridGraph(graph, root, g, ctx, 0.0);
  ASSERT_EQ(r.status, GridGraphStatus::kOk);
  mgg::ShortestPathsReport rep;
  ASSERT_TRUE(graph.findShortestPaths(0, rep));
  int deck = 0, floor_under_deck = 0, walkway = 0;
  for (const auto& [id, v] : graph.vertices_map_) {
    const bool reached = id == 0 || rep.parent_id_map.at(id) != id;
    if (!reached) continue;
    const bool over_floor = v->state.x() > 0.5 && v->state.x() < 1.9 &&
                            std::abs(v->state.y()) < 0.5;
    if (over_floor && v->state.z() > 1.3) ++deck;
    if (over_floor && v->state.z() < 0.5) ++floor_under_deck;
    if (v->state.y() > 3.0 && v->state.z() > 1.3) ++walkway;
  }
  EXPECT_GT(walkway, 0);
  EXPECT_GT(floor_under_deck, 0);
  EXPECT_GT(deck, 0);
}

// Ground support stays flat; one overhanging occupied body column is
// deliberately outside the unrotated offset footprint.
class OffsetObstacle : public mgg_test::TerrainFixture {
 public:
  explicit OffsetObstacle(double obstacle_y = -0.75, double slope = 0)
      : TerrainFixture(0.1, floor(slope)), obstacle_y_(obstacle_y) {}
  double obstacle_y_;
  static std::map<std::pair<std::int64_t, std::int64_t>, double> floor(double slope = 0) {
    std::map<std::pair<std::int64_t, std::int64_t>, double> cells;
    for (int x = -40; x <= 40; ++x)
      for (int y = -40; y <= 40; ++y) cells[{x, y}] = slope * (y + .5) * .1;
    return cells;
  }
  VoxelStatus getBoxStatus(const Eigen::Vector3d& c,
                           const Eigen::Vector3d& size, bool unknown) const override {
    if (std::abs(c.x() - 0.05) <= size.x() / 2 + 0.05 &&
        std::abs(c.y() - obstacle_y_) <= size.y() / 2 + 0.05 &&
        c.z() + size.z() / 2 >= 0.4) return VoxelStatus::kOccupied;
    return TerrainFixture::getBoxStatus(c, size, unknown);
  }
};

TEST(GridGraph, ShortcutRotatesBodyOffsetWithHeading) {
  OffsetObstacle map;
  PlanningParams planning;
  planning.max_ground_height = 0.5;
  planning.min_observed_ground_fraction = 0.0;
  RobotParams robot;
  robot.size = Eigen::Vector3d(0.8, 0.4, 0.4);
  robot.center_offset = Eigen::Vector3d(-0.8, 0, 0);
  mgg::GroundProjection ground(map, planning);
  ExpandContext ctx;
  ctx.map = &map; ctx.planning = &planning; ctx.robot = &robot;
  ctx.ground = &ground; ctx.robot_box_size = robot.getPlanningSize();
  EXPECT_FALSE(mgg::groundShortcutSegmentAdmissible(
      ctx, Eigen::Vector3d(0, 0, 0.5), Eigen::Vector3d(0, 0.4, 0.5), false));
}

TEST(GridGraph, BidirectionalEdgeChecksTheOppositeOffsetFootprint) {
  OffsetObstacle map(0.75);
  PlanningParams planning;
  planning.max_ground_height = 0.5;
  planning.min_observed_ground_fraction = 0.0;
  planning.edge_length_max = 1.0;
  planning.edge_length_min = 0.0;
  planning.edge_overshoot = 0.0;
  RobotParams robot;
  robot.size = Eigen::Vector3d(0.8, 0.4, 0.4);
  robot.center_offset = Eigen::Vector3d(-0.8, 0, 0);
  mgg::GroundProjection ground(map, planning);
  ExpandContext ctx;
  ctx.map = &map; ctx.planning = &planning; ctx.robot = &robot;
  ctx.ground = &ground; ctx.robot_box_size = robot.getPlanningSize();
  for (bool forward_only : {true, false}) {
    GraphManager graph;
    graph.addVertex(new Vertex(0, StateVec(0, 0, 0.5, 0)));
    ctx.root_is_robot = forward_only;
    Vertex candidate(-1, StateVec(0, 0.4, 0.5, M_PI_2));
    mgg::ExpandGraphReport report;
    mgg::expandGraph(graph, candidate, report, ctx, true);
    EXPECT_EQ(report.status == mgg::ExpandGraphStatus::kSuccess, forward_only);
  }
}

TEST(GridGraph, LazyGroundDijkstraMatchesFullyEvaluatedEightNeighbourGraph) {
  for (double slope : {0.0, 0.15}) {
  SCOPED_TRACE(slope);
  OffsetObstacle map(0.75, slope);
  PlanningParams planning;
  planning.max_step_height = .1;
  planning.max_ground_height = 0.5;
  planning.min_observed_ground_fraction = 0.0;
  planning.edge_length_max = 1.0;
  planning.edge_length_min = 0.0;
  planning.edge_overshoot = 0.0;
  planning.path_clearance_margin = 0.0;
  planning.num_vertices_max = 1000;
  planning.num_edges_max = 10000;
  planning.num_loops_max = 10000;
  RobotParams robot;
  robot.size = Eigen::Vector3d(0.4, 0.4, 0.4);
  mgg::GroundProjection ground(map, planning, true);
  ExpandContext ctx;
  ctx.map = &map; ctx.planning = &planning; ctx.robot = &robot;
  ctx.ground = &ground; ctx.robot_box_size = robot.getPlanningSize();
  ctx.allow_unknown_lattice_body = true;
  GridGraphParams grid;
  grid.min_val = Eigen::Vector3d(-2, -2, -0.2);
  grid.max_val = Eigen::Vector3d(2, 2, 0.3);
  grid.resolution = Eigen::Vector3d(0.4, 0.4, 0.1);
  GraphManager lazy;
  const auto route = mgg::routeOverLocalLattice(lazy, StateVec(0, 0, 0.2, 0),
                                               StateVec(0, 2, .5 + slope * 2, 0), grid, ctx);
  ASSERT_TRUE(route.routed) << route.reason;
  ASSERT_GT(route.route.size(), 2u);  // the direct route intersects the post
  mgg::ShortestPathsReport lazy_paths;
  ASSERT_TRUE(lazy.findShortestPaths(0, lazy_paths));
  const double actual = lazy_paths.distance_map.at(route.route.back()->id);
  GraphManager full;
  std::map<std::pair<int, int>, Vertex*> cells;
  full.addVertex(new Vertex(0, StateVec(0, 0, 0.5 + slope * .05, 0)));
  cells[{0, 0}] = full.getVertex(0);
  for (int x = -5; x <= 5; ++x) for (int y = -5; y <= 5; ++y) {
    if (x == 0 && y == 0) continue;
    auto* v = new Vertex(full.generateVertexID(), StateVec(0.4*x, 0.4*y, 0.5 + slope * (0.4*y + .05), 0));
    full.addVertex(v); cells[{x, y}] = v;
  }
  Vertex* goal = cells.at({0, 5});
  for (const auto& [a, from] : cells) for (const auto& [b, to] : cells) {
    if (from->id >= to->id) continue;
    const bool stencil = std::abs(a.first-b.first) <= 1 && std::abs(a.second-b.second) <= 1;
    const double length = (from->state.head<3>()-to->state.head<3>()).norm();
    const bool endpoint_link = from == goal || to == goal || from->id == 0 || to->id == 0;
    if (!stencil && !(endpoint_link && length <= 1.0)) continue;
    mgg::ExpandGraphReport rep;
    if (mgg::latticeEdgeTraversable(ctx, *from, *to, rep)) full.addEdge(from, to, length);
  }
  mgg::ShortestPathsReport reference;
  ASSERT_TRUE(full.findShortestPaths(0, reference));
  EXPECT_NEAR(actual, reference.distance_map.at(goal->id), 1e-9);
  }
}

}  // namespace

TEST(GridGraph, HeadingAlignedBodyAndCrossNudgeEnterANarrowNorthPassage) {
  // A long body fits northwards, not map-aligned. With the second mouth
  // offset, only the +0.1 m centre-line retry fits the same full-size body.
  for (double offset : {0.0, 0.15}) {
    SCOPED_TRACE(offset);
    std::map<std::pair<std::int64_t, std::int64_t>, double> tops;
    for (int x = -30; x < 30; ++x) {
      for (int y = -30; y < 80; ++y) {
        const double px = (x + 0.5) * 0.05;
        const double py = (y + 0.5) * 0.05;
        tops[{x, y}] = py > 0.8 && std::abs(px - offset) > 0.45 ? 0.4 : 0.0;
      }
    }
    mgg_test::TerrainFixture map(0.05, tops);
    RobotParams robot;
    robot.type = RobotType::kGroundRobot;
    robot.size = Eigen::Vector3d(1.2, 0.6, 0.3);
    PlanningParams planning;
    planning.max_ground_height = 0.4;
    planning.max_step_height = 0.15;
    planning.edge_length_min = 0.05;
    planning.edge_length_max = 0.6;
    planning.edge_overshoot = 0;
    planning.min_observed_ground_fraction = 0;
    mgg::GroundProjection ground(map, planning);
    ExpandContext ctx;
    ctx.map = &map; ctx.robot = &robot; ctx.planning = &planning;
    ctx.ground = &ground; ctx.robot_box_size = robot.getPlanningSize();
    GraphManager graph;
    const StateVec root(0, 0, 0.4, M_PI / 2);
    graph.addVertex(new Vertex(0, root));
    GridGraphParams grid;
    grid.min_val = Eigen::Vector3d::Zero();
    grid.max_val = Eigen::Vector3d(2.8, 0, 0);
    grid.resolution = Eigen::Vector3d::Constant(0.4);
    const auto result = buildGridGraph(graph, root, grid, ctx, M_PI / 2);
    // Diagnostics count the eight nominal cells, not the retry offers.
    EXPECT_LE(result.free_cells, 8);
    int offers = result.merged_duplicates;
    for (int count : result.rejected) offers += count;
    EXPECT_LE(offers, 8);
    double reach = 0;
    for (const auto& entry : graph.vertices_map_)
      reach = std::max(reach, entry.second->state.y());
    EXPECT_GT(reach, 2.0);
  }
}

TEST(GridGraph, CrossNudgesCannotEnterAGapNarrowerThanTheOrientedBody) {
  // A 0.4 m gap cannot fit the full 0.6 m body width at any cross-offset,
  // including the shifted mouth that needs nudges in the wider-gap test.
  for (double offset : {0.0, 0.15}) {
    SCOPED_TRACE(offset);
    std::map<std::pair<std::int64_t, std::int64_t>, double> tops;
    for (int x = -30; x < 30; ++x) {
      for (int y = -30; y < 80; ++y) {
        const double px = (x + 0.5) * 0.05;
        const double py = (y + 0.5) * 0.05;
        tops[{x, y}] = py > 0.8 && std::abs(px - offset) > 0.2 ? 0.4 : 0.0;
      }
    }
    mgg_test::TerrainFixture map(0.05, tops);
    RobotParams robot;
    robot.type = RobotType::kGroundRobot;
    robot.size = Eigen::Vector3d(1.2, 0.6, 0.3);
    PlanningParams planning;
    planning.max_ground_height = 0.4;
    planning.max_step_height = 0.15;
    planning.edge_length_min = 0.05;
    planning.edge_length_max = 0.6;
    planning.edge_overshoot = 0;
    planning.min_observed_ground_fraction = 0;
    mgg::GroundProjection ground(map, planning);
    ExpandContext ctx;
    ctx.map = &map; ctx.robot = &robot; ctx.planning = &planning;
    ctx.ground = &ground; ctx.robot_box_size = robot.getPlanningSize();
    GraphManager graph;
    const StateVec root(0, 0, 0.4, M_PI / 2);
    graph.addVertex(new Vertex(0, root));
    GridGraphParams grid;
    grid.min_val = Eigen::Vector3d::Zero();
    grid.max_val = Eigen::Vector3d(2.8, 0, 0);
    grid.resolution = Eigen::Vector3d::Constant(0.4);
    const auto result = buildGridGraph(graph, root, grid, ctx, M_PI / 2);
    // Diagnostics count the eight nominal cells, not the retry offers.
    EXPECT_LE(result.free_cells, 8);
    int offers = result.merged_duplicates;
    for (int count : result.rejected) offers += count;
    EXPECT_LE(offers, 8);
    double reach = 0;
    for (const auto& entry : graph.vertices_map_)
      reach = std::max(reach, entry.second->state.y());
    EXPECT_LE(reach, 0.8);
  }
}

TEST(GridGraph, HardwareUnknownPolicyMatchesEagerInRotatedDoorways) {
  for (bool sensor_policy : {false, true}) for (bool narrow : {false, true}) for (bool upper_unknown : {false, true})
  for (double heading : {M_PI / 2, M_PI / 3}) {
    SCOPED_TRACE(heading);
    std::map<std::pair<std::int64_t, std::int64_t>, double> tops;
    for (int x = -90; x < 90; ++x) for (int y = -90; y < 90; ++y) {
      const double px = (x + .5) * .05, py = (y + .5) * .05;
      const double along = px * std::cos(heading) + py * std::sin(heading);
      const double across = -px * std::sin(heading) + py * std::cos(heading);
      tops[{x, y}] = along > .8 && std::abs(across) > (narrow ? .20 : .48) ? .4 : 0;
    }
    class UpperUnknown : public mgg_test::TerrainFixture {
     public:
      using mgg_test::TerrainFixture::TerrainFixture;
      bool upper = false;
      VoxelStatus getBoxStatus(const Eigen::Vector3d& c, const Eigen::Vector3d& size,
                               bool stop) const override {
        const auto status = mgg_test::TerrainFixture::getBoxStatus(c, size, stop);
        if (status == VoxelStatus::kOccupied) return status;
        return upper && stop && c.z() + size.z()/2 > .45
            ? VoxelStatus::kUnknown : status;
      }
      VoxelStatus getStaticStrictBoxStatus(const Eigen::Vector3d& c,
                                           const Eigen::Vector3d& size) const override {
        return getBoxStatus(c, size, true);
      }
    } map(.05, tops);
    map.upper = upper_unknown;
    RobotParams robot;
    robot.type = RobotType::kGroundRobot;
    robot.size = Eigen::Vector3d(1.2, .6, .3);
    PlanningParams planning;
    planning.max_ground_height = .4;
    planning.max_step_height = .15;
    planning.edge_length_min = .05;
    planning.edge_length_max = .6;
    planning.edge_overshoot = 0;
    planning.path_clearance_margin = .6;  // direct route cannot skip the doorway
    planning.min_observed_ground_fraction = 0;
    mgg::GroundProjection ground(map, planning, true);
    ExpandContext ctx;
    ctx.map = &map; ctx.robot = &robot; ctx.planning = &planning;
    ctx.ground = &ground; ctx.robot_box_size = robot.getPlanningSize();
    ctx.allow_unknown_lattice_body = false;
    if (sensor_policy) ctx.unknown_body_above_center = .025;
    const StateVec root(0, 0, .4, heading);
    GridGraphParams grid;
    grid.min_val = Eigen::Vector3d::Zero();
    grid.max_val = Eigen::Vector3d(2.8, 0, 0);
    grid.resolution = Eigen::Vector3d::Constant(.4);
    GraphManager eager, lazy;
    eager.addVertex(new Vertex(0, root));
    buildGridGraph(eager, root, grid, ctx, heading);
    const StateVec goal(2.4 * std::cos(heading), 2.4 * std::sin(heading), .4, heading);
    Vertex* reached = nullptr;
    const bool eager_reaches = eager.getNearestVertexInRange(&goal, .01, &reached);
    EXPECT_EQ(eager_reaches, !narrow && (!upper_unknown || sensor_policy));
    EXPECT_EQ(mgg::routeOverLocalLattice(lazy, root, goal, grid, ctx).routed,
              eager_reaches);
  }
}

TEST(GridGraph, DirectShortcutFallsBackWhenTheStartTurnHasNoRoom) {
  class NoTurnRoom : public OffsetObstacle {
   public:
    VoxelStatus getOccupiedOnlyCylinderPathStatus(const Eigen::Vector3d&,
        const Eigen::Vector3d&, double, double) const override {
      return VoxelStatus::kOccupied;
    }
  } map;
  PlanningParams planning;
  planning.max_ground_height = .5;
  planning.max_step_height = .1;
  planning.min_observed_ground_fraction = 0;
  planning.path_clearance_margin = 0;
  planning.edge_length_max = .6;
  planning.edge_length_min = 0;
  planning.edge_overshoot = 0;
  RobotParams robot;
  robot.size = Eigen::Vector3d(.2,.2,.2);
  mgg::GroundProjection ground(map, planning, true);
  ExpandContext ctx;
  ctx.map=&map; ctx.ground=&ground; ctx.robot=&robot; ctx.planning=&planning;
  ctx.robot_box_size=robot.getPlanningSize(); ctx.allow_unknown_lattice_body=true;
  GridGraphParams grid;
  grid.min_val=Eigen::Vector3d(-2,-2,0); grid.max_val=Eigen::Vector3d(2,2,0);
  grid.resolution=Eigen::Vector3d::Constant(.4);
  GraphManager graph;
  const auto result = mgg::routeOverLocalLattice(graph, StateVec(0,0,.5,M_PI/2),
      StateVec(2,0,.5,0), grid, ctx);
  ASSERT_TRUE(result.routed);
  EXPECT_GT(graph.getNumVertices(), 2);
}

TEST(GridGraph, DirectShortcutFallsBackForASharpStartTurnOnASlope) {
  OffsetObstacle map(-3.0, .25);
  PlanningParams planning;
  planning.max_ground_height = .5;
  planning.max_step_height = .1;
  planning.min_observed_ground_fraction = 0;
  planning.path_clearance_margin = 0;
  planning.edge_length_max = .6;
  planning.edge_length_min = 0;
  planning.edge_overshoot = 0;
  RobotParams robot;
  robot.size = Eigen::Vector3d(.2,.2,.2);
  mgg::GroundProjection ground(map, planning, true);
  ExpandContext ctx;
  ctx.map=&map; ctx.ground=&ground; ctx.robot=&robot; ctx.planning=&planning;
  ctx.robot_box_size=robot.getPlanningSize(); ctx.allow_unknown_lattice_body=true;
  GridGraphParams grid;
  grid.min_val=Eigen::Vector3d(-2,-2,0); grid.max_val=Eigen::Vector3d(2,2,0);
  grid.resolution=Eigen::Vector3d::Constant(.4);
  GraphManager graph;
  const auto result = mgg::routeOverLocalLattice(graph, StateVec(0,0,.5,0),
      StateVec(0,2,1.0,0), grid, ctx);
  ASSERT_TRUE(result.routed);
  EXPECT_GT(graph.getNumVertices(), 2);
}
