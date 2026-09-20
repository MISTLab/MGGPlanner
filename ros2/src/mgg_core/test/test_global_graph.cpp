// Tests for the global roadmap: path ingestion (Rrg::addRefPathToGraph),
// frontier ingestion (Rrg::addFrontiers) and the global frontier search
// (Rrg::runGlobalPlanner's ranking). The ROS 1 versions ran only inside a
// full planning cycle.

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
      fixture.global, StateVec(1.2, 0.0, 0.0, 0.0), fixture.ctx, 1.5);
  ASSERT_NE(linked, nullptr);
  EXPECT_EQ(linked->parent, root);
  // root-side, root-linked and the extra linked-side edge.
  EXPECT_EQ(fixture.global.getNumEdges(), 3);
  EXPECT_TRUE(fixture.global.graph_->edgeExists(linked->id, side->id));
  // On a vertex already: that vertex.
  EXPECT_EQ(mgg::connectStateToGraph(fixture.global,
                                     StateVec(1.2, 0.02, 0.0, 0.0),
                                     fixture.ctx, 1.5),
            linked);
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

}  // namespace
