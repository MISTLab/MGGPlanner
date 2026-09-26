// Tests for volumetric gain, the quantity the exploration planner maximises.

#include <cmath>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/gain.h"

namespace {

using mgg::BoundedSpaceParams;
using mgg::BoundedSpaceType;
using mgg::GainContext;
using mgg::GainCounts;
using mgg::MapInterface;
using mgg::PlanningParams;
using mgg::SensorModel;
using mgg::SensorParams;
using mgg::SensorType;
using mgg::StateVec;
using mgg::Vertex;
using mgg::VolumetricGain;
using mgg::VoxelStatus;

/// Everything with x < frontier_x has been mapped free; beyond it is unknown.
/// So a viewpoint further along +x sees more unknown.
class HalfMapped : public MapInterface {
 public:
  explicit HalfMapped(double frontier_x = 5.0) : frontier_x_(frontier_x) {}
  double getResolution() const override { return 0.2; }
  bool getStatus() const override { return true; }
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    return p.x() < frontier_x_ ? VoxelStatus::kFree : VoxelStatus::kUnknown;
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool s) const override {
    Eigen::Vector3d ig; return getRayStatus(a, b, s, ig);
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d&, const Eigen::Vector3d& b,
                           bool, Eigen::Vector3d& end) const override {
    end = b; return VoxelStatus::kFree;
  }
  VoxelStatus getBoxStatus(const Eigen::Vector3d& c, const Eigen::Vector3d&,
                           bool) const override { return getVoxelStatus(c); }
  VoxelStatus getPathStatus(const Eigen::Vector3d&, const Eigen::Vector3d& b,
                            const Eigen::Vector3d&, bool) const override {
    return getVoxelStatus(b);
  }
  void getScanStatus(const Eigen::Vector3d& pos,
                     const std::vector<Eigen::Vector3d>& endpoints,
                     GainCounts& gain,
                     std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& log,
                     const SensorModel&) override {
    gain = GainCounts{};
    for (const auto& e : endpoints) {
      const double len = (e - pos).norm();
      if (len < 1e-9) continue;
      const Eigen::Vector3d dir = (e - pos) / len;
      for (double d = 0.0; d <= len; d += 1.0) {
        const Eigen::Vector3d p = pos + d * dir;
        const VoxelStatus s = getVoxelStatus(p);
        log.emplace_back(p, s);
        if (s == VoxelStatus::kUnknown) ++gain.unknown; else ++gain.free;
      }
    }
  }
  void getScanStatusIterative(
      const Eigen::Vector3d& pos, const std::vector<Eigen::Vector3d>& e,
      GainCounts& g, std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& l,
      const SensorModel& m) override { getScanStatus(pos, e, g, l, m); }
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
  double frontier_x_;
};

/// Reports `count` distinct unknown voxels along +x from the viewpoint and
/// nothing else, whatever the rays.
class CountedUnknown : public HalfMapped {
 public:
  explicit CountedUnknown(int count) : count_(count) {}
  void getScanStatusIterative(
      const Eigen::Vector3d& pos, const std::vector<Eigen::Vector3d>&,
      GainCounts& gain,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& log,
      const SensorModel&) override {
    gain = GainCounts{};
    for (int i = 0; i < count_; ++i) {
      log.emplace_back(pos + Eigen::Vector3d(0.2 * (i + 1), 0.0, 0.0),
                       VoxelStatus::kUnknown);
      ++gain.unknown;
    }
  }
 private:
  int count_;
};

struct Fixture {
  Fixture() {
    sensor.type = SensorType::kLidar;
    sensor.max_range = 10.0;
    sensor.fov = Eigen::Vector2d(2.0 * M_PI, M_PI / 6.0);
    sensor.resolution = Eigen::Vector2d(M_PI / 9.0, M_PI / 9.0);
    sensor.width = 18; sensor.height = 3;
    sensor.frontier_percentage_threshold = 0.01;
    sensor.update();
    sensors["VLP16"] = sensor;

    planning.exp_sensor_list = {"VLP16"};
    planning.unknown_voxel_gain = 10.0;
    planning.free_voxel_gain = 1.0;
    planning.occupied_voxel_gain = 1.0;
    planning.clustering_radius = 2.0;

    space.type = BoundedSpaceType::kCuboid;
    space.min_val = Eigen::Vector3d(-100, -100, -100);
    space.max_val = Eigen::Vector3d(100, 100, 100);
    space.setCenter(Eigen::Vector3d(0.0, 0.0, 0.0), false);

    ctx.map = &map;
    ctx.planning = &planning;
    ctx.global_space = &space;
    ctx.sensors = &sensors;
  }
  HalfMapped map;
  SensorParams sensor;
  std::unordered_map<std::string, SensorParams> sensors;
  PlanningParams planning;
  BoundedSpaceParams space;
  GainContext ctx;
};

TEST(Gain, ViewpointNearerTheFrontierScoresHigher) {
  Fixture f;
  VolumetricGain deep, shallow;
  computeVolumetricGain(StateVec(-20.0, 0, 0, 0), deep, f.ctx);
  computeVolumetricGain(StateVec(4.0, 0, 0, 0), shallow, f.ctx);
  // From x = 4 most rays cross into unknown; from x = -20 far fewer do.
  EXPECT_GT(shallow.num_unknown_voxels, deep.num_unknown_voxels);
  EXPECT_GT(shallow.gain, deep.gain);
}

TEST(Gain, UnknownIsWeightedAboveFreeAndOccupied) {
  Fixture f;
  VolumetricGain g;
  computeVolumetricGain(StateVec(4.0, 0, 0, 0), g, f.ctx);
  const double expected = g.num_unknown_voxels * 10.0 +
                          g.num_free_voxels * 1.0 +
                          g.num_occupied_voxels * 1.0;
  EXPECT_NEAR(g.gain, expected, 1e-9);
}

TEST(Gain, VoxelsOutsideTheGlobalBoundsDoNotCount) {
  Fixture f;
  VolumetricGain wide, narrow;
  computeVolumetricGain(StateVec(4.0, 0, 0, 0), wide, f.ctx);

  // Squeeze the allowed region down to a small box around the robot.
  f.space.min_val = Eigen::Vector3d(3.0, -1.0, -1.0);
  f.space.max_val = Eigen::Vector3d(5.0, 1.0, 1.0);
  f.space.setCenter(Eigen::Vector3d(0.0, 0.0, 0.0), false);
  computeVolumetricGain(StateVec(4.0, 0, 0, 0), narrow, f.ctx);

  EXPECT_LT(narrow.num_unknown_voxels, wide.num_unknown_voxels);
}

TEST(Gain, NoGainZonesSuppressTheirContents) {
  Fixture f;
  VolumetricGain without;
  computeVolumetricGain(StateVec(4.0, 0, 0, 0), without, f.ctx);

  BoundedSpaceParams zone;
  zone.type = BoundedSpaceType::kCuboid;
  zone.min_val = Eigen::Vector3d(-50, -50, -50);
  zone.max_val = Eigen::Vector3d(50, 50, 50);
  zone.setCenter(Eigen::Vector3d(0.0, 0.0, 0.0), false);
  std::vector<BoundedSpaceParams> zones{zone};
  f.ctx.no_gain_zones = &zones;

  VolumetricGain with;
  computeVolumetricGain(StateVec(4.0, 0, 0, 0), with, f.ctx);
  EXPECT_EQ(with.num_unknown_voxels, 0);
  EXPECT_LT(with.gain, without.gain);
}

TEST(Gain, FrontierIsFlaggedWhenEnoughUnknownIsVisible) {
  Fixture f;
  VolumetricGain g;
  computeVolumetricGain(StateVec(4.0, 0, 0, 0), g, f.ctx);
  EXPECT_TRUE(g.is_frontier);

  // Deep inside mapped space there is nothing new to see.
  VolumetricGain deep;
  computeVolumetricGain(StateVec(-50.0, 0, 0, 0), deep, f.ctx);
  EXPECT_FALSE(deep.is_frontier);
}

// A ground robot's vertex is a frontier with 0.5 m of unknown: three voxels
// of 0.2 m, as before the frontier test counted distinct voxels.
TEST(Gain, AGroundRobotsFrontierNeedsHalfAMetreOfUnknown) {
  for (const int count : {2, 3}) {
    Fixture f;
    CountedUnknown map(count);
    f.ctx.map = &map;
    mgg::RobotParams robot;
    robot.type = mgg::RobotType::kGroundRobot;
    f.ctx.robot = &robot;
    // The voxels lie level with the vertex, 0.5 m over its floor: within
    // gain_max_height_above_ground.
    f.planning.max_ground_height = 0.5;
    VolumetricGain g;
    computeVolumetricGain(StateVec(0, 0, 0, 0), g, f.ctx);
    EXPECT_EQ(g.num_unknown_voxels, count);
    EXPECT_EQ(g.is_frontier, count >= 3) << count << " voxels";
  }
}

TEST(Gain, MissingSensorIsReportedNotCrashed) {
  Fixture f;
  f.planning.exp_sensor_list = {"NoSuchSensor"};
  VolumetricGain g;
  computeVolumetricGain(StateVec(0, 0, 0, 0), g, f.ctx);
  EXPECT_EQ(g.num_unknown_voxels, 0);
  EXPECT_DOUBLE_EQ(g.gain, 0.0);
}

TEST(Gain, ExplorationGainScoresEveryVertexAndMarksFrontiers) {
  Fixture f;
  mgg::GraphManager graph;
  for (int i = 0; i < 4; ++i) {
    auto* v = new Vertex(i, StateVec(i * 2.0, 0.0, 0.0, 0.0));
    graph.addVertex(v);
  }
  const int n = computeExplorationGain(graph, f.ctx, false, false);
  EXPECT_EQ(n, 4);
  // The vertex closest to the frontier at x = 5 should score highest.
  EXPECT_GT(graph.getVertex(3)->vol_gain.gain,
            graph.getVertex(0)->vol_gain.gain);
  EXPECT_EQ(graph.getVertex(3)->type, mgg::VertexType::kFrontier);
}

TEST(Gain, ClusteringEvaluatesFewerViewpoints) {
  Fixture f;
  auto build = [](mgg::GraphManager& g) {
    for (int i = 0; i < 8; ++i) {
      g.addVertex(new Vertex(i, StateVec(i * 0.5, 0.0, 0.0, 0.0)));
    }
  };
  mgg::GraphManager plain, clustered;
  build(plain); build(clustered);

  const int n_plain = computeExplorationGain(plain, f.ctx, false, false);
  const int n_clustered = computeExplorationGain(clustered, f.ctx, false, true);
  EXPECT_EQ(n_plain, 8);
  // Vertices 0.5 m apart with a 2 m clustering radius share gains.
  EXPECT_LT(n_clustered, n_plain);
}

// REGRESSION: the ROS 1 loop read vertices_map_[i] for i in [0,
// getNumVertices()) with operator[], which inserts a null for a missing key
// and then dereferences it. A graph whose ids are not 0..n-1 was a segfault.
TEST(Gain, NonContiguousVertexIdsAreHandled) {
  Fixture f;
  mgg::GraphManager graph;
  graph.addVertex(new Vertex(0, StateVec(0, 0, 0, 0)));
  graph.addVertex(new Vertex(7, StateVec(2, 0, 0, 0)));
  graph.addVertex(new Vertex(99, StateVec(4, 0, 0, 0)));
  const int n = computeExplorationGain(graph, f.ctx, false, false);
  EXPECT_EQ(n, 3);
}

}  // namespace
