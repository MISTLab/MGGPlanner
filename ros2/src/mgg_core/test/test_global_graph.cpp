// Tests for the global roadmap: path ingestion (Rrg::addRefPathToGraph),
// frontier ingestion (Rrg::addFrontiers), the global frontier search
// (Rrg::runGlobalPlanner's ranking), the timed expansion into observed space
// (Rrg::expandGlobalGraphTimerCallback) and the odometry ingestion of
// Rrg::timerCallback. The ROS 1 versions ran only inside a full planning
// cycle or a live timer.

#include <cmath>
#include <map>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/global_graph.h"

namespace {

using mgg::ExpandContext;
using mgg::GainCounts;
using mgg::GraphManager;
using mgg::MapInterface;
using mgg::PlanningParams;
using mgg::RobotParams;
using mgg::RobotType;
using mgg::SensorModel;
using mgg::ShortestPathsReport;
using mgg::StateVec;
using mgg::Vertex;
using mgg::VertexType;
using mgg::VoxelStatus;

/// Open space, with an optional half-space y >= blocked_y that is occupied,
/// or unknown when `unknown_instead` is set.
class OpenSpace : public MapInterface {
 public:
  explicit OpenSpace(double blocked_y = 1e9, bool unknown_instead = false)
      : blocked_y_(blocked_y), unknown_instead_(unknown_instead) {}

  double getResolution() const override { return 0.2; }
  bool getStatus() const override { return true; }
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    if (p.y() < blocked_y_) return VoxelStatus::kFree;
    return unknown_instead_ ? VoxelStatus::kUnknown : VoxelStatus::kOccupied;
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool s) const override {
    Eigen::Vector3d ignored;
    return getRayStatus(a, b, s, ignored);
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool stop_at_unknown,
                           Eigen::Vector3d& end_voxel) const override {
    const double len = (b - a).norm();
    Eigen::Vector3d dir = Eigen::Vector3d::Zero();
    if (len > 1e-12) dir = (b - a) / len;
    for (double d = 0.0; d <= len + 1e-9; d += 0.1) {
      const Eigen::Vector3d p = a + std::min(d, len) * dir;
      const VoxelStatus status = getVoxelStatus(p);
      if (status == VoxelStatus::kOccupied ||
          (status == VoxelStatus::kUnknown && stop_at_unknown)) {
        end_voxel = p;
        return status;
      }
    }
    end_voxel = b;
    return VoxelStatus::kFree;
  }
  VoxelStatus getBoxStatus(const Eigen::Vector3d& c, const Eigen::Vector3d&,
                           bool s) const override {
    const VoxelStatus status = getVoxelStatus(c);
    return status == VoxelStatus::kUnknown && !s ? VoxelStatus::kFree : status;
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
  double blocked_y_;
  bool unknown_instead_;
};

/// A thin wall: the slab y in [wall_y0, wall_y1] is occupied.
class SlabSpace : public OpenSpace {
 public:
  SlabSpace(double wall_y0, double wall_y1) : y0_(wall_y0), y1_(wall_y1) {}
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    return p.y() >= y0_ && p.y() <= y1_ ? VoxelStatus::kOccupied
                                        : VoxelStatus::kFree;
  }

 private:
  double y0_;
  double y1_;
};

/// A column: the square |x - x0|, |y - y0| < half_width is occupied.
class ColumnSpace : public OpenSpace {
 public:
  ColumnSpace(double x0, double y0, double half_width)
      : x0_(x0), y0_(y0), half_width_(half_width) {}
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    return std::abs(p.x() - x0_) < half_width_ &&
                   std::abs(p.y() - y0_) < half_width_
               ? VoxelStatus::kOccupied
               : VoxelStatus::kFree;
  }

 private:
  double x0_;
  double y0_;
  double half_width_;
};

/// An aerial robot keeps ground projection out of the picture; the roadmap
/// logic under test is the same for both robot types.
struct Roadmap {
  explicit Roadmap(double blocked_y = 1e9, bool unknown_instead = false)
      : map(blocked_y, unknown_instead) {
    robot.type = RobotType::kAerialRobot;
    robot.size = Eigen::Vector3d(0.4, 0.4, 0.4);
    robot.size_extension.setZero();
    robot.size_extension_min.setZero();
    robot.safety_extension.setZero();
    planning.rr_mode = mgg::RRModeType::kGraph;
    planning.edge_length_min = 0.1;
    planning.edge_length_max = 2.0;
    planning.edge_overshoot = 0.0;
    planning.nearest_range = 1.1;
    planning.nearest_range_min = 0.1;
    planning.nearest_range_max = 100.0;
    planning.nearest_range_z = 100.0;
    ctx.map = &map;
    ctx.planning = &planning;
    ctx.robot = &robot;
    ctx.robot_id = 0;
    ctx.robot_box_size = robot.getPlanningSize();
    global.addVertex(new Vertex(0, StateVec(0.0, 0.0, 0.0, 0.0)));
  }

  Vertex* add(GraphManager& graph, const StateVec& state, Vertex* parent,
              VertexType type = VertexType::kUnvisited) {
    auto* vertex = new Vertex(graph.generateVertexID(), state);
    vertex->type = type;
    vertex->vol_gain.is_frontier = type == VertexType::kFrontier;
    graph.addVertex(vertex);
    if (parent != nullptr) {
      graph.addEdge(vertex, parent,
                    (vertex->state.head<3>() - parent->state.head<3>()).norm());
    }
    return vertex;
  }

  static std::vector<StateVec> line(double x0, double x1, double step) {
    std::vector<StateVec> path;
    for (double x = x0; x <= x1 + 1e-9; x += step) {
      path.emplace_back(x, 0.0, 0.0, 0.0);
    }
    return path;
  }

  double distanceFromRoot(int id) {
    ShortestPathsReport rep;
    if (!global.findShortestPaths(rep)) return -1.0;
    return global.getShortestDistance(id, rep);
  }

  Vertex* nearest(const Eigen::Vector3d& p) {
    const StateVec state(p.x(), p.y(), p.z(), 0.0);
    Vertex* vertex = nullptr;
    return global.getNearestVertexInRange(&state, 1e-6, &vertex) ? vertex
                                                                 : nullptr;
  }

  OpenSpace map;
  RobotParams robot;
  PlanningParams planning;
  ExpandContext ctx;
  GraphManager global;
};

TEST(AddRefPathToGraph, PathVerticesJoinTheGlobalGraphWithEdgesAlongIt) {
  Roadmap fixture;
  std::vector<Vertex*> path_vertices;
  ASSERT_TRUE(mgg::addRefPathToGraph(fixture.global,
                                     Roadmap::line(0.0, 3.0, 0.5), fixture.ctx,
                                     1.0, &path_vertices));
  // The first pose sits on the root, so the root stands for it; the six
  // remaining poses at half-metre steps are resampled to one per metre.
  ASSERT_EQ(path_vertices.size(), 4u);
  EXPECT_EQ(path_vertices.front()->id, 0);
  EXPECT_EQ(fixture.global.getNumVertices(), 4);
  EXPECT_EQ(fixture.global.getNumEdges(), 3);
  for (int i = 1; i <= 3; ++i) {
    ASSERT_NE(fixture.nearest(Eigen::Vector3d(i, 0.0, 0.0)), nullptr);
  }
  // Routable end to end over the new chain.
  EXPECT_NEAR(fixture.distanceFromRoot(path_vertices.back()->id), 3.0, 1e-9);
  EXPECT_EQ(path_vertices.back()->parent, path_vertices[2]);
}

TEST(AddRefPathToGraph, LongSegmentsAreDensifiedAtTheSpacing) {
  Roadmap fixture;
  ASSERT_TRUE(mgg::addRefPathToGraph(
      fixture.global,
      {StateVec(0.0, 0.0, 0.0, 0.0), StateVec(3.0, 0.0, 0.0, 0.0)},
      fixture.ctx, 1.0));
  // One chain vertex at 3 m, plus two intermediate vertices at 1 m and 2 m.
  EXPECT_EQ(fixture.global.getNumVertices(), 4);
  EXPECT_NE(fixture.nearest(Eigen::Vector3d(1.0, 0.0, 0.0)), nullptr);
  EXPECT_NE(fixture.nearest(Eigen::Vector3d(2.0, 0.0, 0.0)), nullptr);
  const int edges = fixture.global.getNumEdges();
  // The intermediate vertices are reused on a repeat as well.
  ASSERT_TRUE(mgg::addRefPathToGraph(
      fixture.global,
      {StateVec(0.0, 0.0, 0.0, 0.0), StateVec(3.0, 0.0, 0.0, 0.0)},
      fixture.ctx, 1.0));
  EXPECT_EQ(fixture.global.getNumVertices(), 4);
  EXPECT_EQ(fixture.global.getNumEdges(), edges);
}

TEST(AddRefPathToGraph, NeighbourEdgesOnlyThroughKnownFreeSpace) {
  const auto run = [](Roadmap& fixture) {
    Vertex* root = fixture.global.getVertex(0);
    fixture.add(fixture.global, StateVec(1.0, 0.6, 0.0, 0.0), root);
    EXPECT_TRUE(mgg::addRefPathToGraph(fixture.global,
                                       Roadmap::line(0.0, 2.0, 1.0),
                                       fixture.ctx, 1.0));
    return fixture.global.getNumEdges();
  };
  // root-side, root-(1,0), (1,0)-(2,0), and the extra edge (1,0)-side.
  Roadmap open;
  EXPECT_EQ(run(open), 4);
  // The side vertex is behind an obstacle: no extra edge.
  Roadmap walled(0.3);
  EXPECT_EQ(run(walled), 3);
  // Unobserved space blocks a roadmap edge, unlike a lattice edge.
  Roadmap unmapped(0.3, /*unknown_instead=*/true);
  EXPECT_EQ(run(unmapped), 3);
}

TEST(AddRefPathToGraph, FirstPoseAwayFromTheGraphIsLinkedOrRefused) {
  // Within the blind radius: a plain edge to the nearest vertex.
  Roadmap near;
  std::vector<Vertex*> vertices;
  ASSERT_TRUE(mgg::addRefPathToGraph(
      near.global,
      {StateVec(0.4, 0.0, 0.0, 0.0), StateVec(1.4, 0.0, 0.0, 0.0)}, near.ctx,
      1.0, &vertices));
  ASSERT_EQ(vertices.size(), 2u);
  EXPECT_EQ(vertices.front()->parent, near.global.getVertex(0));
  EXPECT_NEAR(vertices.front()->state.x(), 0.4, 1e-9);

  // Beyond it: a checked expandGraph link, which the map can refuse.
  Roadmap far;
  ASSERT_TRUE(mgg::addRefPathToGraph(
      far.global,
      {StateVec(0.0, 1.5, 0.0, 0.0), StateVec(0.0, 2.5, 0.0, 0.0)}, far.ctx,
      1.0));
  EXPECT_EQ(far.global.getNumVertices(), 3);
  Roadmap blocked(1.0);
  EXPECT_FALSE(mgg::addRefPathToGraph(
      blocked.global,
      {StateVec(0.0, 1.5, 0.0, 0.0), StateVec(0.0, 2.5, 0.0, 0.0)},
      blocked.ctx, 1.0));
  EXPECT_EQ(blocked.global.getNumVertices(), 1);

  Roadmap empty;
  EXPECT_FALSE(mgg::addRefPathToGraph(empty.global, std::vector<StateVec>{},
                                      empty.ctx, 1.0));
}

TEST(AddRefPathToGraph, RepeatedPathReusesItsOwnVertices) {
  Roadmap fixture;
  const std::vector<StateVec> path = Roadmap::line(0.0, 3.0, 1.0);
  ASSERT_TRUE(mgg::addRefPathToGraph(fixture.global, path, fixture.ctx, 1.0));
  const int vertices = fixture.global.getNumVertices();
  const int edges = fixture.global.getNumEdges();
  // A replan from the same place repeats the same lattice; the roadmap must
  // not grow by one copy of it per cycle.
  std::vector<Vertex*> again;
  ASSERT_TRUE(mgg::addRefPathToGraph(fixture.global, path, fixture.ctx, 1.0,
                                     &again));
  EXPECT_EQ(fixture.global.getNumVertices(), vertices);
  EXPECT_EQ(fixture.global.getNumEdges(), edges);
  ASSERT_EQ(again.size(), 4u);
  EXPECT_EQ(again.back(), fixture.nearest(Eigen::Vector3d(3.0, 0.0, 0.0)));
  // Another robot's vertex on the same spot is not reused: its placement
  // carries the inter-robot transform error.
  fixture.nearest(Eigen::Vector3d(2.0, 0.0, 0.0))->robot_id = 3;
  ASSERT_TRUE(mgg::addRefPathToGraph(fixture.global, path, fixture.ctx, 1.0));
  EXPECT_EQ(fixture.global.getNumVertices(), vertices + 1);
}

TEST(AddRefPathToGraph, LinkSkipsANearerVertexBehindAThinWall) {
  Roadmap fixture;
  SlabSpace wall(0.25, 0.35);
  fixture.ctx.map = &wall;
  Vertex* root = fixture.global.getVertex(0);
  root->state = StateVec(0.0, -0.2, 0.0, 0.0);
  fixture.global.updateVertexState(0, root->state);
  // Nearer to the path start than the root, but across the wall from it.
  Vertex* behind = fixture.add(fixture.global, StateVec(0.0, 0.45, 0.0, 0.0),
                               nullptr);
  std::vector<Vertex*> vertices;
  ASSERT_TRUE(mgg::addRefPathToGraph(
      fixture.global,
      {StateVec(0.0, 0.2, 0.0, 0.0), StateVec(1.0, 0.2, 0.0, 0.0)},
      fixture.ctx, 1.0, &vertices));
  ASSERT_EQ(vertices.size(), 2u);
  EXPECT_EQ(vertices.front()->parent, root);
  EXPECT_FALSE(fixture.global.graph_->edgeExists(vertices.front()->id,
                                                  behind->id));
}

TEST(AddRefPathToGraph, StartThatCannotBeLinkedLinksAtTheFirstReachablePose) {
  // The planner's start is where the robot stood: pinned against a wall or
  // tilted on a ramp crest, its body box overlaps mapped geometry and no link
  // to the graph's tip is free (robot_2 and robot_1 on the SubT finals,
  // 2026-09-23). The path's later poses are free, so the path joins there.
  Roadmap fixture;
  ColumnSpace column(1.3, 0.0, 0.1);
  fixture.ctx.map = &column;
  Vertex* tip = fixture.add(fixture.global, StateVec(1.0, 0.0, 0.0, 0.0),
                            fixture.global.getVertex(0));
  const std::vector<StateVec> path = {StateVec(1.3, 0.0, 0.0, 0.0),
                                      StateVec(1.3, 0.8, 0.0, 0.0),
                                      StateVec(1.3, 1.6, 0.0, 0.0)};
  std::vector<Vertex*> vertices;
  ASSERT_TRUE(mgg::addRefPathToGraph(fixture.global, path, fixture.ctx, 0.5,
                                     &vertices));
  // The unlinked start stays out; the rest is a chain off the tip.
  EXPECT_EQ(fixture.nearest(Eigen::Vector3d(1.3, 0.0, 0.0)), nullptr);
  ASSERT_EQ(vertices.size(), 2u);
  EXPECT_EQ(vertices.front(), fixture.nearest(Eigen::Vector3d(1.3, 0.8, 0.0)));
  EXPECT_EQ(vertices.front()->parent, tip);
  EXPECT_EQ(vertices.back(), fixture.nearest(Eigen::Vector3d(1.3, 1.6, 0.0)));
  EXPECT_EQ(fixture.global.getNumVertices(), 4);
  EXPECT_GT(fixture.distanceFromRoot(vertices.back()->id), 0.0);

  // The next start, free and 1.5 m from the tip, links as well.
  ASSERT_TRUE(mgg::addRefPathToGraph(
      fixture.global,
      {StateVec(1.0, -1.5, 0.0, 0.0), StateVec(1.0, -2.5, 0.0, 0.0)},
      fixture.ctx, 1.0));
  EXPECT_EQ(fixture.global.getNumVertices(), 6);

  // Joined at a lattice vertex, that vertex keeps its frontier mark.
  GraphManager local;
  local.addVertex(new Vertex(0, StateVec(1.3, 0.0, 0.0, 0.0)));
  Vertex* leaf = fixture.add(local, StateVec(2.0, 0.8, 0.0, 0.0),
                             local.getVertex(0), VertexType::kFrontier);
  leaf->vol_gain.gain = 7.0;
  std::vector<Vertex*> added;
  ASSERT_TRUE(mgg::addRefPathToGraph(
      fixture.global, std::vector<Vertex*>{local.getVertex(0), leaf},
      fixture.ctx, 1.0, &added));
  ASSERT_EQ(added.size(), 1u);
  EXPECT_EQ(added.front()->type, VertexType::kFrontier);
  EXPECT_DOUBLE_EQ(added.front()->vol_gain.gain, 7.0);
}

TEST(AddRefPathToGraph, StartRaisedOnARampCrestLinksWhereThePathLevelsOut) {
  // bistro.yaml's bounds on a link's height change: away from home (graph
  // distance over nearest_range_max) a link may not climb more than
  // nearest_range_z. A start raised on a ramp crest, 0.3 m above the graph's
  // tip, does not link (robot_2 at 16:14:03 on the SubT finals); the path's
  // next pose, level with the tip again, does.
  Roadmap fixture;
  fixture.planning.nearest_range_z = 0.15;
  fixture.planning.nearest_range_max = 1.0;
  Vertex* tip = fixture.add(fixture.global, StateVec(1.0, 0.0, 0.0, 0.0),
                            fixture.global.getVertex(0));
  tip->distance = 5.0;
  const std::vector<StateVec> path = {StateVec(1.8, 0.0, 0.3, 0.0),
                                      StateVec(1.8, 0.7, 0.1, 0.0),
                                      StateVec(1.8, 1.5, 0.1, 0.0)};
  std::vector<Vertex*> vertices;
  ASSERT_TRUE(mgg::addRefPathToGraph(fixture.global, path, fixture.ctx, 0.5,
                                     &vertices));
  EXPECT_EQ(fixture.nearest(Eigen::Vector3d(1.8, 0.0, 0.3)), nullptr);
  ASSERT_EQ(vertices.size(), 2u);
  EXPECT_EQ(vertices.front()->parent, tip);
  EXPECT_EQ(fixture.global.getNumVertices(), 4);
}

TEST(AddRefPathToGraph, PathOutOfReachIsRefusedUntilItComesBackWithinReach) {
  Roadmap fixture;
  fixture.add(fixture.global, StateVec(1.0, 0.0, 0.0, 0.0),
              fixture.global.getVertex(0));
  // Every pose is farther from the graph than one edge. A link clipped short
  // of the start would leave the chain's first edge, from the clipped vertex
  // to the next pose, unchecked; nothing is added instead.
  EXPECT_FALSE(mgg::addRefPathToGraph(fixture.global,
                                      Roadmap::line(10.0, 12.0, 1.0),
                                      fixture.ctx, 1.0));
  EXPECT_EQ(fixture.global.getNumVertices(), 2);

  // A path that returns within reach joins where it arrives.
  std::vector<Vertex*> vertices;
  ASSERT_TRUE(mgg::addRefPathToGraph(
      fixture.global,
      {StateVec(10.0, 0.0, 0.0, 0.0), StateVec(6.0, 0.0, 0.0, 0.0),
       StateVec(2.5, 0.0, 0.0, 0.0)},
      fixture.ctx, 1.0, &vertices));
  ASSERT_EQ(vertices.size(), 1u);
  EXPECT_EQ(vertices.front(), fixture.nearest(Eigen::Vector3d(2.5, 0.0, 0.0)));
  EXPECT_EQ(fixture.nearest(Eigen::Vector3d(6.0, 0.0, 0.0)), nullptr);
  EXPECT_EQ(fixture.global.getNumVertices(), 3);
}

TEST(AddRefPathToGraph, VertexOverloadCarriesTypeAndStopsAtHangingVertices) {
  Roadmap fixture;
  GraphManager local;
  local.addVertex(new Vertex(0, StateVec(0.0, 0.0, 0.0, 0.0)));
  Vertex* middle = fixture.add(local, StateVec(1.0, 0.0, 0.0, 0.0),
                               local.getVertex(0));
  Vertex* leaf = fixture.add(local, StateVec(2.0, 0.0, 0.0, 0.0), middle,
                             VertexType::kFrontier);
  leaf->vol_gain.gain = 42.0;
  Vertex* hanging = fixture.add(local, StateVec(3.0, 0.0, 0.0, 0.0), leaf);
  hanging->is_hanging = true;
  std::vector<Vertex*> added;
  ASSERT_TRUE(mgg::addRefPathToGraph(
      fixture.global, {local.getVertex(0), middle, leaf, hanging},
      fixture.ctx, 1.0, &added));
  ASSERT_EQ(added.size(), 3u);
  EXPECT_EQ(added[1]->type, VertexType::kUnvisited);
  EXPECT_EQ(added[2]->type, VertexType::kFrontier);
  EXPECT_DOUBLE_EQ(added[2]->vol_gain.gain, 42.0);
  EXPECT_EQ(fixture.nearest(Eigen::Vector3d(3.0, 0.0, 0.0)), nullptr);
}

TEST(ConnectStateToGraph, LinksTheCurrentStateAndWiresItsNeighbours) {
  Roadmap fixture;
  fixture.planning.nearest_range = 1.5;
  Vertex* root = fixture.global.getVertex(0);
  // Farther from the state than the root, so the root is the link, but
  // within nearest_range, so it gets the extra edge.
  Vertex* side = fixture.add(fixture.global, StateVec(1.2, 1.3, 0.0, 0.0), root);
  Vertex* linked = mgg::connectStateToGraph(
      fixture.global, StateVec(1.2, 0.0, 0.0, 0.0), fixture.ctx, 1.5, false);
  ASSERT_NE(linked, nullptr);
  EXPECT_EQ(linked->parent, root);
  // root-side, root-linked and the extra linked-side edge.
  EXPECT_EQ(fixture.global.getNumEdges(), 3);
  EXPECT_TRUE(fixture.global.graph_->edgeExists(linked->id, side->id));
  // On a vertex already: that vertex.
  EXPECT_EQ(mgg::connectStateToGraph(fixture.global,
                                     StateVec(1.2, 0.02, 0.0, 0.0),
                                     fixture.ctx, 1.5, false),
            linked);
}

TEST(ConnectStateToGraph, ExactAttachmentRoutesToGoalBesideExistingVertex) {
  Roadmap fixture;
  Vertex* root = fixture.global.getVertex(0);
  fixture.add(fixture.global, StateVec(1.2, 0.0, 0.0, 0.0), root);
  const StateVec goal(1.2, 0.02, 0.0, 0.0);
  Vertex* linked = mgg::connectStateToGraph(
      fixture.global, goal, fixture.ctx, 1.5, true);
  ASSERT_NE(linked, nullptr);
  ShortestPathsReport report;
  ASSERT_TRUE(fixture.global.findShortestPaths(0, report));
  std::vector<StateVec> path;
  fixture.global.getShortestPath(linked->id, report, true, path);
  ASSERT_GE(path.size(), 2u);
  EXPECT_DOUBLE_EQ(path.back().x(), goal.x());
  EXPECT_DOUBLE_EQ(path.back().y(), goal.y());
  EXPECT_DOUBLE_EQ(path.back().z(), goal.z());
}

TEST(ConnectStateToGraph, ExactAttachmentRejectsOccupiedShortLink) {
  Roadmap fixture(/*blocked_y=*/0.0);
  // A stale roadmap vertex is not proof that its vicinity remains clear.
  EXPECT_EQ(mgg::connectStateToGraph(
                fixture.global, StateVec(0.02, 0.0, 0.0, 0.0),
                fixture.ctx, 1.5, true),
            nullptr);
}

TEST(ConnectStateToGraph, ExactAttachmentRejectsClippedGoalExtension) {
  Roadmap fixture;
  EXPECT_EQ(mgg::connectStateToGraph(
                fixture.global, StateVec(4.0, 0.0, 0.0, 0.0),
                fixture.ctx, 1.5, true),
            nullptr);
}

/// A wall y in [1.0, 1.4] everywhere except a gap at x >= 4.5.
class WallWithGap : public OpenSpace {
 public:
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    return p.y() >= 1.0 && p.y() <= 1.4 && p.x() < 4.5 ? VoxelStatus::kOccupied
                                                       : VoxelStatus::kFree;
  }
};

/// A roadmap along y = 0 from home to x = 4, as an odometry trail leaves it,
/// checked with a roadmap context (unobserved space blocks), and the paper's
/// lattice scaled to the fixture: +/-6 m in one layer at 0.5 m.
struct GoalLatticeScene {
  explicit GoalLatticeScene(const MapInterface& map_in) {
    fixture.ctx.map = &map_in;
    fixture.ctx.stop_at_unknown = true;
    fixture.planning.num_vertices_max = 5000;
    fixture.planning.num_edges_max = 50000;
    Vertex* parent = fixture.global.getVertex(0);
    for (double x = 1.0; x <= 4.0 + 1e-9; x += 1.0) {
      parent = fixture.add(fixture.global, StateVec(x, 0.0, 0.0, 0.0), parent);
    }
    grid.min_val = Eigen::Vector3d(-6.0, -6.0, 0.0);
    grid.max_val = Eigen::Vector3d(6.0, 6.0, 0.0);
    grid.resolution = Eigen::Vector3d(0.5, 0.5, 0.5);
  }

  std::vector<StateVec> routeFromHome(const Vertex* goal) {
    ShortestPathsReport rep;
    std::vector<StateVec> path;
    if (!fixture.global.findShortestPaths(0, rep)) return path;
    fixture.global.getShortestPath(goal->id, rep, true, path);
    return path;
  }

  Roadmap fixture;
  mgg::GridGraphParams grid;
};

TEST(ConnectGoalThroughLattice, RoutesAroundAWallNoSingleEdgeCrosses) {
  WallWithGap map;
  GoalLatticeScene scene(map);
  // Behind the wall and more than one edge from the trail: no single edge
  // attaches the goal.
  const StateVec goal(3.0, 2.5, 0.0, 0.0);
  ASSERT_EQ(mgg::connectStateToGraph(scene.fixture.global, goal,
                                     scene.fixture.ctx, 0.1, true),
            nullptr);
  const int before = scene.fixture.global.getNumVertices();

  mgg::GoalLatticeReport report;
  Vertex* linked = mgg::connectGoalThroughLattice(
      scene.fixture.global, goal, scene.grid, scene.fixture.ctx, 0.0, nullptr,
      &report);
  ASSERT_NE(linked, nullptr);
  EXPECT_DOUBLE_EQ(linked->state.x(), goal.x());
  EXPECT_DOUBLE_EQ(linked->state.y(), goal.y());
  EXPECT_DOUBLE_EQ(linked->state.z(), goal.z());
  EXPECT_GE(report.passes, 1);
  EXPECT_GT(report.lattice_vertices, 1);
  EXPECT_GT(report.chain_vertices, 2);
  EXPECT_GT(scene.fixture.global.getNumVertices(), before);

  // The route from home ends exactly on the goal, turns through the gap,
  // and no segment of it crosses the wall.
  const std::vector<StateVec> path = scene.routeFromHome(linked);
  ASSERT_GE(path.size(), 2u);
  EXPECT_DOUBLE_EQ(path.back().x(), goal.x());
  EXPECT_DOUBLE_EQ(path.back().y(), goal.y());
  int crossings = 0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const StateVec& a = path[i];
    const StateVec& b = path[i + 1];
    EXPECT_EQ(map.getRayStatus(a.head<3>(), b.head<3>(), true),
              VoxelStatus::kFree)
        << "segment " << i << " crosses the wall";
    // Where the route crosses the wall's centre line, it is in the gap.
    if ((a.y() - 1.2) * (b.y() - 1.2) < 0.0) {
      const double t = (1.2 - a.y()) / (b.y() - a.y());
      EXPECT_GE(a.x() + t * (b.x() - a.x()), 4.5);
      ++crossings;
    }
  }
  EXPECT_EQ(crossings, 1);

  // The lattice path stays in the roadmap: a goal beside it now attaches
  // with a single edge.
  EXPECT_NE(mgg::connectStateToGraph(scene.fixture.global,
                                     StateVec(3.4, 2.5, 0.0, 0.0),
                                     scene.fixture.ctx, 0.1, true),
            nullptr);
}

TEST(ConnectGoalThroughLattice, UnobservedSpaceIsNotCrossed) {
  // Everything from y = 1 on is unobserved: known space does not reach it.
  OpenSpace map(/*blocked_y=*/1.0, /*unknown_instead=*/true);
  GoalLatticeScene scene(map);
  const int before = scene.fixture.global.getNumVertices();
  EXPECT_EQ(mgg::connectGoalThroughLattice(
                scene.fixture.global, StateVec(2.0, 2.5, 0.0, 0.0),
                scene.grid, scene.fixture.ctx, 0.0, nullptr),
            nullptr);
  EXPECT_EQ(scene.fixture.global.getNumVertices(), before);
}

TEST(ConnectGoalThroughLattice, BridgesOnlyToUsableRoadmapVertices) {
  WallWithGap map;
  GoalLatticeScene scene(map);
  const StateVec goal(3.0, 2.5, 0.0, 0.0);
  const int before = scene.fixture.global.getNumVertices();
  // Nothing the robot can reach: the goal stays unattached and the roadmap
  // unchanged.
  EXPECT_EQ(mgg::connectGoalThroughLattice(
                scene.fixture.global, goal, scene.grid, scene.fixture.ctx, 0.0,
                [](const Vertex&) { return false; }),
            nullptr);
  EXPECT_EQ(scene.fixture.global.getNumVertices(), before);
  // Only home usable, 4.5 m west of the gap: a single sweep never reaches
  // round the wall's end to it; the repeated sweeps do.
  mgg::GoalLatticeReport report;
  Vertex* linked = mgg::connectGoalThroughLattice(
      scene.fixture.global, goal, scene.grid, scene.fixture.ctx, 0.0,
      [](const Vertex& vertex) { return vertex.id == 0; }, &report);
  ASSERT_NE(linked, nullptr);
  EXPECT_GT(report.passes, 1);
  EXPECT_GE(scene.routeFromHome(linked).size(), 2u);
}

/// A local grid graph with three frontier arms, plus a global graph whose
/// existing vertices surround two of them.
struct FrontierScene {
  FrontierScene() {
    local.addVertex(new Vertex(0, StateVec(0.0, 0.0, 0.0, 0.0)));
    Vertex* root = local.getVertex(0);
    // Arm A: open, should be added.
    arm_a = fixture.add(local, StateVec(2.0, 0.0, 0.0, 0.0),
                        fixture.add(local, StateVec(1.0, 0.0, 0.0, 0.0), root),
                        VertexType::kFrontier);
    arm_a->vol_gain.gain = 50.0;
    // Arm B: a global vertex already stands within 1 m of its leaf.
    arm_b = fixture.add(local, StateVec(0.0, 2.0, 0.0, 0.0),
                        fixture.add(local, StateVec(0.0, 1.0, 0.0, 0.0), root),
                        VertexType::kFrontier);
    // Arm C: the robot has passed within 3 m of its leaf.
    arm_c = fixture.add(local, StateVec(-2.0, 0.0, 0.0, 0.0),
                        fixture.add(local, StateVec(-1.0, 0.0, 0.0, 0.0), root),
                        VertexType::kFrontier);

    Vertex* global_root = fixture.global.getVertex(0);
    fixture.add(fixture.global, StateVec(0.0, 2.2, 0.0, 0.0), global_root);
    visited = fixture.add(fixture.global, StateVec(-2.0, 1.5, 0.0, 0.0),
                          global_root, VertexType::kVisited);
    stale = fixture.add(fixture.global, StateVec(5.0, 5.0, 0.0, 0.0),
                        global_root, VertexType::kFrontier);
  }

  Roadmap fixture;
  GraphManager local;
  Vertex* arm_a = nullptr;
  Vertex* arm_b = nullptr;
  Vertex* arm_c = nullptr;
  Vertex* visited = nullptr;
  Vertex* stale = nullptr;
};

TEST(AddFrontiers, PrincipalFrontierPathIsAddedAndSurroundedOnesAreNot) {
  FrontierScene scene;
  int rechecked = 0;
  const auto recompute = [&rechecked](Vertex& vertex) {
    ++rechecked;
    vertex.vol_gain.is_frontier = false;  // the stale frontier has been seen
  };
  const int before = scene.fixture.global.getNumVertices();
  const mgg::FrontierAdditionReport report = mgg::addFrontiers(
      scene.fixture.global, scene.local, scene.fixture.ctx, recompute, 1.0);

  EXPECT_EQ(report.global_frontiers_rechecked, 1);
  EXPECT_EQ(report.global_frontiers_demoted, 1);
  EXPECT_EQ(rechecked, 1);
  EXPECT_EQ(scene.stale->type, VertexType::kUnvisited);
  EXPECT_EQ(report.local_frontiers, 3);
  EXPECT_EQ(report.clusters, 3);
  EXPECT_EQ(report.paths_added, 1);

  // Arm A joined as a chain whose leaf is the only frontier on it.
  EXPECT_EQ(scene.fixture.global.getNumVertices(), before + 2);
  Vertex* middle = scene.fixture.nearest(Eigen::Vector3d(1.0, 0.0, 0.0));
  Vertex* leaf = scene.fixture.nearest(Eigen::Vector3d(2.0, 0.0, 0.0));
  ASSERT_NE(middle, nullptr);
  ASSERT_NE(leaf, nullptr);
  EXPECT_EQ(middle->type, VertexType::kUnvisited);
  EXPECT_EQ(leaf->type, VertexType::kFrontier);
  EXPECT_DOUBLE_EQ(leaf->vol_gain.gain, 50.0);
  EXPECT_NEAR(scene.fixture.distanceFromRoot(leaf->id), 2.0, 1e-9);
  // The surrounded arms stayed out.
  EXPECT_EQ(scene.fixture.nearest(Eigen::Vector3d(0.0, 2.0, 0.0)), nullptr);
  EXPECT_EQ(scene.fixture.nearest(Eigen::Vector3d(-2.0, 0.0, 0.0)), nullptr);
  // Running again adds nothing: arm A's leaf now has a global vertex on it.
  const mgg::FrontierAdditionReport again = mgg::addFrontiers(
      scene.fixture.global, scene.local, scene.fixture.ctx, nullptr, 1.0);
  EXPECT_EQ(again.paths_added, 0);
  EXPECT_EQ(scene.fixture.global.getNumVertices(), before + 2);
}

TEST(PerformShortestPathsClustering, SimilarPathsShareAClusterAndShortOnesFold) {
  Roadmap fixture;
  GraphManager local;
  local.addVertex(new Vertex(0, StateVec(0.0, 0.0, 0.0, 0.0)));
  Vertex* root = local.getVertex(0);
  Vertex* stem = fixture.add(local, StateVec(1.0, 0.0, 0.0, 0.0), root);
  Vertex* east = fixture.add(local, StateVec(2.0, 0.0, 0.0, 0.0), stem);
  Vertex* east_twin = fixture.add(local, StateVec(2.0, 0.1, 0.0, 0.0), stem);
  Vertex* north = fixture.add(local, StateVec(0.0, 2.0, 0.0, 0.0),
                              fixture.add(local, StateVec(0.0, 1.0, 0.0, 0.0),
                                          root));
  Vertex* stub = fixture.add(local, StateVec(0.5, -0.5, 0.0, 0.0), root);
  ShortestPathsReport rep;
  ASSERT_TRUE(local.findShortestPaths(rep));
  std::vector<Vertex*> leaves{east, east_twin, north, stub};
  const std::vector<int> principal =
      mgg::performShortestPathsClustering(local, rep, leaves);
  // Two clusters: the twin eastern paths (the longer twin leads) and north.
  // The stub's own path is under a metre, so it folds into the nearest.
  ASSERT_EQ(principal.size(), 2u);
  EXPECT_EQ(principal[0], east_twin->id);
  EXPECT_EQ(principal[1], north->id);
  EXPECT_EQ(east->cluster_id, east_twin->id);
  EXPECT_EQ(east_twin->cluster_id, east_twin->id);
  EXPECT_EQ(north->cluster_id, north->id);
  EXPECT_TRUE(stub->cluster_id == east_twin->id || stub->cluster_id == north->id);
}

/// A global graph with frontiers of every kind the search must tell apart.
struct FrontierGraph {
  FrontierGraph() {
    Vertex* root = fixture.global.getVertex(0);
    near_ = fixture.add(fixture.global, StateVec(5.0, 0.0, 0.0, 0.0), root,
                        VertexType::kFrontier);
    far_ = fixture.add(fixture.global, StateVec(10.0, 0.0, 0.0, 0.0), near_,
                       VertexType::kFrontier);
    isolated_ = fixture.add(fixture.global, StateVec(0.0, 5.0, 0.0, 0.0),
                            nullptr, VertexType::kFrontier);
    theirs_ = fixture.add(fixture.global, StateVec(0.0, -5.0, 0.0, 0.0), root,
                          VertexType::kFrontier);
    theirs_->robot_id = 7;
    excluded_ = fixture.add(fixture.global, StateVec(5.0, 5.0, 0.0, 0.0), root,
                            VertexType::kFrontier);
    seen_ = fixture.add(fixture.global, StateVec(-5.0, 0.0, 0.0, 0.0), root,
                        VertexType::kFrontier);
    gains_ = {{near_->id, 100.0}, {far_->id, 150.0}, {isolated_->id, 1e6},
              {theirs_->id, 100.0}, {excluded_->id, 1e6}, {seen_->id, 1e6}};
  }

  mgg::RecomputeGainFn recompute() {
    return [this](Vertex& vertex) {
      vertex.vol_gain.gain = gains_.at(vertex.id);
      vertex.vol_gain.is_frontier = vertex.id != seen_->id;
    };
  }

  Roadmap fixture;
  Vertex* near_ = nullptr;
  Vertex* far_ = nullptr;
  Vertex* isolated_ = nullptr;
  Vertex* theirs_ = nullptr;
  Vertex* excluded_ = nullptr;
  Vertex* seen_ = nullptr;
  std::map<int, double> gains_;
};

TEST(SearchGlobalFrontier, PicksTheReachableFrontierWithTheBestDiscountedGain) {
  FrontierGraph graph;
  const mgg::GlobalFrontierReport report = mgg::searchGlobalFrontier(
      graph.fixture.global, 0, 0, graph.recompute(),
      {Eigen::Vector3d(5.0, 5.0, 0.0)}, 1.0);
  // The re-check demoted the frontier that has since been seen.
  EXPECT_EQ(report.demoted, 1);
  EXPECT_EQ(graph.seen_->type, VertexType::kUnvisited);
  EXPECT_EQ(report.frontiers, 5);
  // Reachable and not excluded: near, far and the other robot's.
  EXPECT_EQ(report.feasible, 3);
  // 150 * exp(-0.05 * 10) beats 100 * exp(-0.05 * 5); the isolated frontier
  // is unreachable, the excluded one is a peer's reservation, and the other
  // robot's frontier is discounted a thousandfold.
  ASSERT_NE(report.best_frontier, nullptr);
  EXPECT_EQ(report.best_frontier, graph.far_);
  EXPECT_NEAR(report.best_gain, 150.0 * std::exp(-0.5), 1e-9);
  EXPECT_NEAR(report.best_distance, 10.0, 1e-9);
}

TEST(SearchGlobalFrontier, AnExplorationTargetPullsTowardTheNearerFrontier) {
  FrontierGraph graph;
  // Without a target the far frontier wins (the test above). Toward a goal
  // beside the near one: 100 * exp(-0.05 * 5) * exp(-0.05 * 1) beats
  // 150 * exp(-0.05 * 10) * exp(-0.05 * sqrt(26)).
  const Eigen::Vector3d target(5.0, -1.0, 0.0);
  const mgg::GlobalFrontierReport report = mgg::searchGlobalFrontier(
      graph.fixture.global, 0, 0, graph.recompute(),
      {Eigen::Vector3d(5.0, 5.0, 0.0)}, 1.0, &target);
  ASSERT_NE(report.best_frontier, nullptr);
  EXPECT_EQ(report.best_frontier, graph.near_);
  EXPECT_NEAR(report.best_gain, 100.0 * std::exp(-0.25) * std::exp(-0.05),
              1e-9);
  // Soft: an unreachable frontier stays out however close it is to the
  // target, and the ranking still covers every reachable one.
  const Eigen::Vector3d beside_isolated(0.0, 5.0, 0.0);
  const mgg::GlobalFrontierReport soft = mgg::searchGlobalFrontier(
      graph.fixture.global, 0, 0, graph.recompute(),
      {Eigen::Vector3d(5.0, 5.0, 0.0)}, 1.0, &beside_isolated);
  EXPECT_EQ(soft.feasible, 3);
  ASSERT_NE(soft.best_frontier, nullptr);
  EXPECT_NE(soft.best_frontier, graph.isolated_);
}

TEST(SearchGlobalFrontier, OtherRobotsFrontierIsTakenOnlyWhenNothingElseIs) {
  FrontierGraph graph;
  graph.gains_[graph.near_->id] = 0.0;
  graph.gains_[graph.far_->id] = 0.0;
  const mgg::GlobalFrontierReport report = mgg::searchGlobalFrontier(
      graph.fixture.global, 0, 0, graph.recompute(),
      {Eigen::Vector3d(5.0, 5.0, 0.0)}, 1.0);
  ASSERT_NE(report.best_frontier, nullptr);
  EXPECT_EQ(report.best_frontier, graph.theirs_);
  EXPECT_NEAR(report.best_gain, 100.0 * std::exp(-0.25) * 0.001, 1e-12);
}

TEST(SearchGlobalFrontier, NoReachableFrontierReportsNone) {
  Roadmap fixture;
  // Nothing typed as a frontier at all.
  EXPECT_EQ(mgg::searchGlobalFrontier(fixture.global, 0, 0, nullptr)
                .best_frontier,
            nullptr);
  // A frontier the graph cannot reach from the source.
  Vertex* island = fixture.add(fixture.global, StateVec(0.0, 5.0, 0.0, 0.0),
                               nullptr, VertexType::kFrontier);
  island->vol_gain.gain = 1e6;
  const mgg::GlobalFrontierReport report =
      mgg::searchGlobalFrontier(fixture.global, 0, 0, nullptr);
  EXPECT_EQ(report.frontiers, 1);
  EXPECT_EQ(report.feasible, 0);
  EXPECT_EQ(report.best_frontier, nullptr);
  // An unknown source vertex cannot be routed from.
  EXPECT_EQ(mgg::searchGlobalFrontier(fixture.global, 99, 0, nullptr)
                .best_frontier,
            nullptr);
}

/// A sampler over a square local box in the plane, seeded so a run repeats.
mgg::RandomSampler boxSampler(double half_width, unsigned seed = 7) {
  return mgg::RandomSampler(Eigen::Vector3d(-half_width, -half_width, 0.0),
                            Eigen::Vector3d(half_width, half_width, 0.0),
                            seed);
}

/// A visited root at the origin and one unvisited vertex 30 m east: one
/// cluster, well clear of everything else.
struct ExpansionScene {
  ExpansionScene() {
    fixture.global.getVertex(0)->type = VertexType::kVisited;
    unvisited = fixture.add(fixture.global, StateVec(30.0, 0.0, 0.0, 0.0),
                            nullptr);
  }

  int degree(int id) const {
    const auto edges = fixture.global.edge_map_.find(id);
    return edges == fixture.global.edge_map_.end()
               ? 0
               : static_cast<int>(edges->second.size());
  }

  Roadmap fixture;
  Vertex* unvisited = nullptr;
  mgg::RobotStateHistory history;
};

TEST(ExpandGlobalGraph, GrowsAroundUnvisitedClustersOnly) {
  ExpansionScene scene;
  mgg::RandomSampler sampler = boxSampler(10.0);
  int scored = 0;
  const auto gain = [&scored](Vertex& vertex) {
    ++scored;
    vertex.vol_gain.is_frontier = false;
  };
  const mgg::GlobalGraphExpansionReport report = mgg::expandGlobalGraph(
      scene.fixture.global, scene.fixture.ctx, sampler, scene.history, gain,
      0.02);
  EXPECT_EQ(report.unvisited_vertices, 1);
  EXPECT_EQ(report.clusters, 1);
  EXPECT_GE(report.passes, 2);
  EXPECT_GT(report.samples, 0);
  EXPECT_GT(report.vertices_added, 0);
  EXPECT_EQ(report.edges_added, report.vertices_added);
  EXPECT_EQ(report.frontiers_added, 0);
  EXPECT_EQ(scored, report.vertices_added);
  EXPECT_EQ(scene.fixture.global.getNumVertices(), 2 + report.vertices_added);
  // Every new vertex grew out of the cluster's box, wired into the graph and
  // typed unvisited; nothing was sampled around the visited root.
  for (const auto& entry : scene.fixture.global.vertices_map_) {
    if (entry.first <= 1) continue;
    EXPECT_GT(entry.second->state.x(), 15.0);
    EXPECT_EQ(entry.second->type, VertexType::kUnvisited);
    EXPECT_GE(scene.degree(entry.first), 1);
  }

  // With nothing unvisited there is no cluster and no pass: a frontier or a
  // visited vertex is not sampled around.
  Roadmap quiet;
  quiet.global.getVertex(0)->type = VertexType::kVisited;
  quiet.add(quiet.global, StateVec(30.0, 0.0, 0.0, 0.0), nullptr,
            VertexType::kFrontier);
  const mgg::GlobalGraphExpansionReport none = mgg::expandGlobalGraph(
      quiet.global, quiet.ctx, sampler, scene.history, gain, 0.02);
  EXPECT_EQ(none.unvisited_vertices, 0);
  EXPECT_EQ(none.clusters, 0);
  EXPECT_EQ(none.passes, 0);
  EXPECT_EQ(quiet.global.getNumVertices(), 2);
}

TEST(ExpandGlobalGraph, ClustersUnvisitedVerticesByTheLocalBoxRadius) {
  ExpansionScene scene;
  // Within kLocalBoxRadius of the first unvisited vertex: the same cluster.
  scene.fixture.add(scene.fixture.global, StateVec(36.0, 0.0, 0.0, 0.0),
                    nullptr);
  // Farther than that from both: a cluster of its own.
  scene.fixture.add(scene.fixture.global, StateVec(30.0, 40.0, 0.0, 0.0),
                    nullptr);
  mgg::RandomSampler sampler = boxSampler(10.0);
  const mgg::GlobalGraphExpansionReport report = mgg::expandGlobalGraph(
      scene.fixture.global, scene.fixture.ctx, sampler, scene.history,
      nullptr, 0.0);
  EXPECT_EQ(report.unvisited_vertices, 3);
  EXPECT_EQ(report.clusters, 2);
}

TEST(ExpandGlobalGraph, TypesNewVerticesAsFrontiersWhenTheirGainSaysSo) {
  ExpansionScene scene;
  mgg::RandomSampler sampler = boxSampler(10.0);
  const auto gain = [](Vertex& vertex) {
    vertex.vol_gain.gain = 12.0;
    vertex.vol_gain.is_frontier = true;
  };
  const mgg::GlobalGraphExpansionReport report = mgg::expandGlobalGraph(
      scene.fixture.global, scene.fixture.ctx, sampler, scene.history, gain,
      0.02);
  ASSERT_GT(report.vertices_added, 0);
  EXPECT_EQ(report.frontiers_added, report.vertices_added);
  for (const auto& entry : scene.fixture.global.vertices_map_) {
    if (entry.first <= 1) continue;
    EXPECT_EQ(entry.second->type, VertexType::kFrontier);
    EXPECT_DOUBLE_EQ(entry.second->vol_gain.gain, 12.0);
  }
}

TEST(ExpandGlobalGraph, SkipsSamplesNearVerticesRecordedStatesOrFrontiers) {
  // A box small enough that every sample lies within kSparseRadius of the
  // cluster's own vertex: nothing is offered to expandGraph.
  {
    ExpansionScene scene;
    mgg::RandomSampler sampler = boxSampler(2.0);
    const mgg::GlobalGraphExpansionReport report = mgg::expandGlobalGraph(
        scene.fixture.global, scene.fixture.ctx, sampler, scene.history,
        nullptr, 0.005);
    EXPECT_GE(report.passes, 1);
    EXPECT_EQ(report.samples, 0);
    EXPECT_EQ(report.vertices_added, 0);
    EXPECT_EQ(scene.fixture.global.getNumVertices(), 2);
  }
  // The robot has already driven through the box: states recorded every
  // metre or so leave no sample farther than kSparseRadius from one.
  {
    ExpansionScene scene;
    for (double x = 15.0; x <= 45.0 + 1e-9; x += 5.0) {
      for (double y = -15.0; y <= 15.0 + 1e-9; y += 5.0) {
        scene.history.addState(StateVec(x, y, 0.0, 0.0));
      }
    }
    mgg::RandomSampler sampler = boxSampler(10.0);
    const mgg::GlobalGraphExpansionReport report = mgg::expandGlobalGraph(
        scene.fixture.global, scene.fixture.ctx, sampler, scene.history,
        nullptr, 0.005);
    EXPECT_GE(report.passes, 1);
    EXPECT_EQ(report.samples, 0);
    EXPECT_EQ(scene.fixture.global.getNumVertices(), 2);
  }
  // Frontiers around the box: no sample lies farther than
  // kOverlappedFrontierRadius from one, so none is expanded and the
  // frontiers keep the space to themselves.
  {
    ExpansionScene scene;
    for (double x = 15.0; x <= 45.0 + 1e-9; x += 5.0) {
      for (double y = -15.0; y <= 15.0 + 1e-9; y += 5.0) {
        if (std::abs(x - 30.0) < 1e-9 && std::abs(y) < 1e-9) continue;
        scene.fixture.add(scene.fixture.global, StateVec(x, y, 0.0, 0.0),
                          nullptr, VertexType::kFrontier);
      }
    }
    const int before = scene.fixture.global.getNumVertices();
    mgg::RandomSampler sampler = boxSampler(10.0);
    const mgg::GlobalGraphExpansionReport report = mgg::expandGlobalGraph(
        scene.fixture.global, scene.fixture.ctx, sampler, scene.history,
        nullptr, 0.005);
    EXPECT_EQ(report.unvisited_vertices, 1);
    EXPECT_GE(report.passes, 1);
    EXPECT_EQ(report.samples, 0);
    EXPECT_EQ(scene.fixture.global.getNumVertices(), before);
  }
}

TEST(ExpandGlobalGraph, TimeBudgetBoundsTheWork) {
  // Zero budget: the clusters are formed but no pass is made (rrg.cpp:2613).
  {
    ExpansionScene scene;
    mgg::RandomSampler sampler = boxSampler(10.0);
    const mgg::GlobalGraphExpansionReport report = mgg::expandGlobalGraph(
        scene.fixture.global, scene.fixture.ctx, sampler, scene.history,
        nullptr, 0.0);
    EXPECT_EQ(report.clusters, 1);
    EXPECT_EQ(report.passes, 0);
    EXPECT_EQ(report.samples, 0);
    EXPECT_EQ(scene.fixture.global.getNumVertices(), 2);
  }
  // A budget: passes are made while it lasts, and the run ends soon after.
  // The bound is loose because a pass over one cluster in open space takes
  // microseconds, so the run cannot be timed to the millisecond.
  {
    ExpansionScene scene;
    mgg::RandomSampler sampler = boxSampler(10.0);
    const double budget = 0.02;
    const mgg::GlobalGraphExpansionReport report = mgg::expandGlobalGraph(
        scene.fixture.global, scene.fixture.ctx, sampler, scene.history,
        nullptr, budget);
    EXPECT_GE(report.passes, 2);
    EXPECT_GE(report.elapsed_s, budget);
    EXPECT_LT(report.elapsed_s, 1.0);
  }
}

TEST(ExpandGlobalGraph, HistoryAnswersRangeQueries) {
  mgg::RobotStateHistory history;
  std::vector<const StateVec*> found;
  EXPECT_FALSE(history.getNearestStates(StateVec::Zero(), 100.0, &found));
  for (int i = 0; i < 300; ++i) {
    history.addState(StateVec(i * 1.0, 0.0, 0.0, 0.0));
  }
  EXPECT_EQ(history.size(), 300u);
  ASSERT_TRUE(history.getNearestStates(StateVec(10.0, 0.0, 0.0, 0.0), 2.5,
                                       &found));
  EXPECT_EQ(found.size(), 5u);
  for (const StateVec* state : found) {
    EXPECT_LE(std::abs(state->x() - 10.0), 2.5);
  }
  EXPECT_FALSE(history.getNearestStates(StateVec(500.0, 0.0, 0.0, 0.0), 2.5,
                                        &found));
  history.reset();
  EXPECT_EQ(history.size(), 0u);
  EXPECT_FALSE(history.getNearestStates(StateVec(10.0, 0.0, 0.0, 0.0), 2.5,
                                        &found));
}

TEST(SampleVertex, DrawsAFreeStateInTheBoxAroundTheRoot) {
  Roadmap open;
  mgg::RandomSampler sampler = boxSampler(3.0);
  const StateVec root(30.0, 40.0, 0.0, 0.0);
  for (int i = 0; i < 50; ++i) {
    Vertex vertex(-1, StateVec::Zero());
    ASSERT_TRUE(mgg::sampleVertex(sampler, root, open.ctx, vertex));
    EXPECT_LE(std::abs(vertex.state.x() - root.x()), 3.0);
    EXPECT_LE(std::abs(vertex.state.y() - root.y()), 3.0);
    EXPECT_DOUBLE_EQ(vertex.state.z(), 0.0);
    EXPECT_FALSE(vertex.is_hanging);
  }
  // Every draw of the box lands in occupied space: no vertex.
  Roadmap blocked(0.0);
  Vertex vertex(-1, StateVec::Zero());
  EXPECT_FALSE(mgg::sampleVertex(sampler, StateVec(0.0, 10.0, 0.0, 0.0),
                                 blocked.ctx, vertex));
}

TEST(OdometryIngestion, EventE1MarksVerticesWithinThreeMetresVisited) {
  // rrg.cpp:5279 and 5285: every kMinLength of travel, the vertices within
  // kUpdateRadius of the robot are visited, frontiers included.
  Roadmap fixture;
  Vertex* previous = fixture.global.getVertex(0);
  for (int x = 1; x <= 6; ++x) {
    previous = fixture.add(fixture.global, StateVec(x, 0.0, 0.0, 0.0),
                           previous,
                           x == 2 ? VertexType::kFrontier
                                  : VertexType::kUnvisited);
  }
  StateVec robot(0.0, 0.0, 0.0, 0.0);
  fixture.global.updateVertexTypeInRange(robot, 3.0);
  for (int x = 0; x <= 6; ++x) {
    const Vertex* vertex = fixture.nearest(Eigen::Vector3d(x, 0.0, 0.0));
    ASSERT_NE(vertex, nullptr);
    EXPECT_EQ(vertex->type, x <= 3 ? VertexType::kVisited
                                   : VertexType::kUnvisited)
        << "x = " << x;
  }
}

TEST(OdometryIngestion, ExpandGraphWiresTheOdometryStateToEveryReachableNeighbour) {
  // rrg.cpp:5263: the robot's state joins the global graph through
  // expandGraph, which in graph mode adds an edge to every vertex within
  // nearest_range it can reach, not only to the nearest one.
  struct Wiring {
    bool to_root = false;
    bool to_side = false;
  };
  const auto run = [](Roadmap& fixture) {
    fixture.planning.nearest_range = 1.5;
    Vertex* root = fixture.global.getVertex(0);
    Vertex* side = fixture.add(fixture.global, StateVec(0.0, 1.0, 0.0, 0.0),
                               root);
    mgg::ExpandGraphReport rep;
    Vertex state(-1, StateVec(1.0, 0.4, 0.0, 0.0));
    mgg::expandGraph(fixture.global, state, rep, fixture.ctx);
    EXPECT_EQ(rep.status, mgg::ExpandGraphStatus::kSuccess);
    EXPECT_EQ(rep.num_vertices_added, 1);
    Wiring wiring;
    if (rep.vertex_added != nullptr) {
      EXPECT_EQ(rep.vertex_added->parent, root);
      wiring.to_root =
          fixture.global.graph_->edgeExists(rep.vertex_added->id, root->id);
      wiring.to_side =
          fixture.global.graph_->edgeExists(rep.vertex_added->id, side->id);
    }
    return wiring;
  };
  // Nearest is the root (1.08 m); the side vertex (1.17 m) is in range too.
  Roadmap open;
  const Wiring meshed = run(open);
  EXPECT_TRUE(meshed.to_root);
  EXPECT_TRUE(meshed.to_side);
  EXPECT_EQ(open.global.getNumEdges(), 3);
  // A thin wall between the state and the side vertex: only the root edge.
  Roadmap walled;
  SlabSpace wall(0.7, 0.8);
  walled.ctx.map = &wall;
  const Wiring chained = run(walled);
  EXPECT_TRUE(chained.to_root);
  EXPECT_FALSE(chained.to_side);
  EXPECT_EQ(walled.global.getNumEdges(), 2);
}

}  // namespace
