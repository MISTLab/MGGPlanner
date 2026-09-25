// Tests for where a path turns and whether it may turn there.

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/path_selection.h"
#include "mgg_core/path_turns.h"
#include "mgg_core/trajectory.h"

namespace {

using mgg::GraphManager;
using mgg::PathTurnCheck;
using mgg::RobotParams;
using mgg::StateVec;
using mgg::Vertex;
using mgg::VoxelStatus;

constexpr double kDeg = M_PI / 180.0;

double maxOf(const std::vector<double>& v) {
  return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end());
}

TEST(PathTurns, StraightPathHasNoTurns) {
  const std::vector<Eigen::Vector3d> line = {
      {0, 0, 0}, {0.4, 0, 0}, {0.8, 0, 0.1}, {1.2, 0, 0.2}};
  EXPECT_DOUBLE_EQ(maxOf(mgg::pathTurns(line, 0.0, 0.0)), 0.0);
  EXPECT_DOUBLE_EQ(maxOf(mgg::pathTurns(line, 0.0, 1.0)), 0.0);
}

TEST(PathTurns, FirstPointTurnsFromTheRobotsHeading) {
  const std::vector<Eigen::Vector3d> line = {{0, 0, 0}, {0.4, 0, 0}};
  const auto turns = mgg::pathTurns(line, M_PI, 1.0);
  ASSERT_EQ(turns.size(), 2u);
  EXPECT_NEAR(turns[0], M_PI, 1e-9);
  EXPECT_DOUBLE_EQ(turns[1], 0.0);  // the end: no way out
}

TEST(PathTurns, LatticeStaircaseReadsAsTheLineItApproximates) {
  // Steps of 0 and 45 degrees: a line at 22.5 degrees on the lattice.
  std::vector<Eigen::Vector3d> stairs = {{0, 0, 0}};
  for (int i = 0; i < 6; ++i) {
    const Eigen::Vector3d last = stairs.back();
    stairs.push_back(last + (i % 2 == 0 ? Eigen::Vector3d(0.4, 0, 0)
                                        : Eigen::Vector3d(0.4, 0.4, 0)));
  }
  const double heading = std::atan2(1.2, 2.4);
  EXPECT_NEAR(maxOf(mgg::pathTurns(stairs, 0.0, 0.0)), 45.0 * kDeg, 1e-9);
  // Over a metre each way no turn is sharp; near the ends, where the window
  // is cut short, the reading is coarser.
  const auto over_a_metre = mgg::pathTurns(stairs, heading, 1.0);
  EXPECT_LT(maxOf(over_a_metre), 35.0 * kDeg);
}

TEST(PathTurns, CornerSplitOverTwoVerticesIsOneSharpTurn) {
  // Two 45 degree steps 0.57 m apart: a right-angle corner.
  const std::vector<Eigen::Vector3d> corner = {
      {0, 0, 0},     {0.4, 0, 0},   {0.8, 0, 0},  {1.2, 0.4, 0},
      {1.2, 0.8, 0}, {1.2, 1.2, 0}, {1.2, 1.6, 0}};
  EXPECT_NEAR(maxOf(mgg::pathTurns(corner, 0.0, 0.0)), 45.0 * kDeg, 1e-9);
  EXPECT_GT(maxOf(mgg::pathTurns(corner, 0.0, 0.8)), 60.0 * kDeg);
}

/// Ground rising at 16 degrees along +x from x = 0 to x = 3, level before
/// and after.
double rampGround(double x) {
  return std::tan(16.0 * kDeg) * std::min(std::max(x, 0.0), 3.0);
}

/// The lattice a planner would lay over the ramp: 0.4 m cells, each vertex
/// 0.5 m above the ground.
void addRampLattice(GraphManager& graph) {
  for (int i = -5; i <= 15; ++i) {
    for (int j = -3; j <= 8; ++j) {
      const double x = 0.4 * i;
      const double y = 0.4 * j;
      graph.addVertex(new Vertex(graph.generateVertexID(),
                                 StateVec(x, y, rampGround(x) + 0.5, 0.0)));
    }
  }
}

TEST(TerrainSlope, FitsTheGroundUnderTheLattice) {
  GraphManager ground;
  addRampLattice(ground);
  const Vertex on_ramp(-1, StateVec(1.6, 0.8, rampGround(1.6) + 0.5, 0.0));
  EXPECT_NEAR(mgg::terrainSlope(ground, on_ramp, 0.8), 16.0 * kDeg,
              0.1 * kDeg);
  const Vertex level(-1, StateVec(5.2, 0.8, rampGround(5.2) + 0.5, 0.0));
  EXPECT_NEAR(mgg::terrainSlope(ground, level, 0.8), 0.0, 1e-9);

  // Two vertices, or only vertices without ground under them, fit nothing.
  GraphManager sparse;
  sparse.addVertex(new Vertex(0, StateVec(0, 0, 0, 0)));
  sparse.addVertex(new Vertex(1, StateVec(0.4, 0, 0.3, 0)));
  EXPECT_DOUBLE_EQ(mgg::terrainSlope(sparse, *sparse.getVertex(0), 1.0), 0.0);
  auto* hanging = new Vertex(2, StateVec(0, 0.4, 0.3, 0));
  hanging->is_hanging = true;
  sparse.addVertex(hanging);
  EXPECT_DOUBLE_EQ(mgg::terrainSlope(sparse, *sparse.getVertex(0), 1.0), 0.0);
}

RobotParams robot() {
  RobotParams r;
  r.type = mgg::RobotType::kGroundRobot;
  r.size = Eigen::Vector3d(0.8, 0.6, 0.4);
  return r;
}

/// Vertices at the given (x, y) lattice points over the ramp, chained.
std::vector<Vertex*> rampPath(const std::vector<Eigen::Vector2d>& at,
                              double heading, int first_id = 0) {
  std::vector<Vertex*> path;
  for (const Eigen::Vector2d& p : at) {
    path.push_back(new Vertex(first_id++, StateVec(p.x(), p.y(),
                                                   rampGround(p.x()) + 0.5,
                                                   heading)));
  }
  return path;
}

std::vector<Eigen::Vector2d> line(Eigen::Vector2d from, Eigen::Vector2d step,
                                  int count) {
  std::vector<Eigen::Vector2d> out;
  for (int i = 0; i < count; ++i) out.push_back(from + i * step);
  return out;
}

TEST(PathTurnCheck, RampDrivenAlongItsAxisIsAllowed) {
  GraphManager ground;
  addRampLattice(ground);
  PathTurnCheck check(ground, robot());
  // From the level ground below, straight up the ramp and onto the top.
  auto path = rampPath(line({-1.2, 0.8}, {0.4, 0.0}, 14), 0.0);
  EXPECT_TRUE(check(path));
  // And down it, facing downhill.
  std::reverse(path.begin(), path.end());
  for (Vertex* v : path) v->state[3] = M_PI;
  EXPECT_TRUE(check(path));
  EXPECT_EQ(check.refused_on_slope, 0);
  for (Vertex* v : path) delete v;
}

TEST(PathTurnCheck, SharpTurnOnTheRampIsRefused) {
  GraphManager ground;
  addRampLattice(ground);
  PathTurnCheck check(ground, robot());
  // Up the ramp to its middle, then a right angle across it.
  auto on_ramp = line({-0.4, 0.0}, {0.4, 0.0}, 5);
  const auto across = line({1.6, 0.4}, {0.0, 0.4}, 4);
  on_ramp.insert(on_ramp.end(), across.begin(), across.end());
  auto path = rampPath(on_ramp, 0.0);
  EXPECT_FALSE(check(path));
  EXPECT_EQ(check.refused_on_slope, 1);
  for (Vertex* v : path) delete v;

  // Standing on the ramp facing uphill, a path leaving across it turns
  // where the robot stands.
  auto from_ramp = rampPath(line({1.2, 0.0}, {0.0, 0.4}, 4), 0.0);
  EXPECT_FALSE(check(from_ramp));
  for (Vertex* v : from_ramp) delete v;
}

TEST(PathTurnCheck, SelectionTurnsOnLevelGroundRatherThanOnTheRamp) {
  GraphManager ground;
  addRampLattice(ground);
  // A trunk from the level ground up the ramp's axis. Branch A leaves it
  // across the ramp from x = 1.6; branch B carries on to the level top and
  // turns there.
  GraphManager graph;
  const auto trunk = rampPath(line({-1.2, 0.0}, {0.4, 0.0}, 8), 0.0);
  const auto a = rampPath(line({1.6, 0.4}, {0.0, 0.4}, 5), 0.0, 8);
  const auto b_axis = rampPath(line({2.0, 0.0}, {0.4, 0.0}, 5), 0.0, 13);
  const auto b_top = rampPath(line({3.6, 0.4}, {0.0, 0.4}, 5), 0.0, 18);
  const auto chain = [&graph](Vertex* from, const std::vector<Vertex*>& to) {
    for (Vertex* v : to) {
      graph.addVertex(v);
      graph.addEdge(v, from, (v->state - from->state).head<3>().norm());
      from = v;
    }
  };
  graph.addVertex(trunk.front());
  chain(trunk.front(), std::vector<Vertex*>(trunk.begin() + 1, trunk.end()));
  chain(trunk.back(), a);
  chain(trunk.back(), b_axis);
  chain(b_axis.back(), b_top);
  a.back()->vol_gain.gain = 100.0;
  b_top.back()->vol_gain.gain = 50.0;

  mgg::PlanningParams planning;
  planning.path_length_penalty = 0.0;
  planning.path_direction_penalty = 0.0;
  mgg::EdgeInclinations flat;
  const auto unchecked =
      mgg::selectBestPath(graph, planning, robot(), flat, 0.2, 0.0);
  EXPECT_EQ(unchecked.best_path_id, a.back()->id);

  PathTurnCheck check(ground, robot());
  const auto r = mgg::selectBestPath(graph, planning, robot(), flat, 0.2, 0.0,
                                     {}, 0.0, nullptr, std::ref(check));
  EXPECT_EQ(r.best_path_id, b_top.back()->id);
  EXPECT_EQ(r.paths_with_sharp_turns, 1);
  EXPECT_FALSE(r.sharp_turn_fallback);
  EXPECT_EQ(check.refused_on_slope, 1);

  // With nothing to gain on level ground, the ramp turn is still taken
  // rather than no path at all, and flagged.
  b_top.back()->vol_gain.gain = 0.0;
  const auto fallback = mgg::selectBestPath(
      graph, planning, robot(), flat, 0.2, 0.0, {}, 0.0, nullptr,
      std::ref(check));
  EXPECT_EQ(fallback.best_path_id, a.back()->id);
  EXPECT_TRUE(fallback.sharp_turn_fallback);
  EXPECT_EQ(fallback.paths_with_sharp_turns, 1);
}

/// Flat ground with walls, 0 to 2 m tall, standing on the map's 0.2 m cells.
/// Only what turnClear asks is answered: cell centres, and whether a column
/// of a cell is occupied.
class Walls : public mgg::MapInterface {
 public:
  explicit Walls(std::function<bool(double, double)> wall)
      : wall_(std::move(wall)) {}
  double getResolution() const override { return 0.2; }
  bool getStatus() const override { return true; }
  bool getAxisAlignedXYCellCenter(const Eigen::Vector2d& p,
                                  Eigen::Vector2d& center) const override {
    center = ((p / 0.2).array().floor() + 0.5) * 0.2;
    return true;
  }
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    Eigen::Vector2d c;
    getAxisAlignedXYCellCenter(p.head<2>(), c);
    return wall_(c.x(), c.y()) && p.z() >= 0.0 && p.z() <= 2.0
               ? VoxelStatus::kOccupied
               : VoxelStatus::kFree;
  }
  VoxelStatus getBoxStatus(const Eigen::Vector3d& c, const Eigen::Vector3d& s,
                           bool) const override {
    // A column: occupied when its cell is a wall and its span meets 0 to 2.
    return wall_(c.x(), c.y()) && c.z() - s.z() / 2 <= 2.0 &&
                   c.z() + s.z() / 2 >= 0.0
               ? VoxelStatus::kOccupied
               : VoxelStatus::kFree;
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool u) const override {
    Eigen::Vector3d e;
    return getRayStatus(a, b, u, e);
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d&, const Eigen::Vector3d& b,
                           bool, Eigen::Vector3d& e) const override {
    e = b;
    return VoxelStatus::kFree;
  }
  VoxelStatus getPathStatus(const Eigen::Vector3d&, const Eigen::Vector3d&,
                            const Eigen::Vector3d&, bool) const override {
    return VoxelStatus::kFree;
  }
  void getScanStatus(const Eigen::Vector3d&,
                     const std::vector<Eigen::Vector3d>&, mgg::GainCounts&,
                     std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>&,
                     const mgg::SensorModel&) override {}
  void getScanStatusIterative(
      const Eigen::Vector3d&, const std::vector<Eigen::Vector3d>&,
      mgg::GainCounts&, std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>&,
      const mgg::SensorModel&) override {}
  bool augmentFreeBox(const Eigen::Vector3d&,
                      const Eigen::Vector3d&) override {
    return true;
  }
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
  std::function<bool(double, double)> wall_;
};

TEST(TurnClear, NeedsTheCircleThroughTheRobotsCorners) {
  // The robot is 0.8 by 0.6: its corners reach 0.5 m from its centre.
  // Walls from |y| = 0.4 leave room to drive (0.3 m each side) but not to
  // turn; walls from |y| = 0.6 leave both.
  const Walls narrow([](double, double y) { return std::abs(y) > 0.4; });
  const Walls wide([](double, double y) { return std::abs(y) > 0.6; });
  const StateVec centre(0.1, 0.0, 0.5, 0.0);
  EXPECT_FALSE(mgg::turnClear(narrow, robot(), centre));
  EXPECT_TRUE(mgg::turnClear(wide, robot(), centre));
  // Only the body's height counts: riding above the walls clears them.
  EXPECT_TRUE(mgg::turnClear(narrow, robot(), StateVec(0.1, 0.0, 2.5, 0.0)));
}

/// A corridor 0.8 m wide along +x, x in [-1, 4.4], opening into a room
/// beyond; halfway along, at x in [1.6, 2.4], a passage leaves it to +y.
bool corridorWall(double x, double y) {
  if (x < -1.0 || x > 4.4 || std::abs(y) < 0.4 || std::abs(y) > 1.2) {
    return false;
  }
  return !(y > 0.0 && x > 1.6 && x < 2.4);
}

TEST(PathTurnCheck, TurnWithoutRoomIsAvoidedWhenThereIsAnother) {
  // On level ground: branch A turns into the side passage, where the
  // corridor's far wall is 0.4 m away; branch B turns in the room.
  GraphManager graph;
  const auto trunk = rampPath(line({-2.4, 0.0}, {0.4, 0.0}, 12), 0.0);
  for (Vertex* v : trunk) v->state.z() = 0.5;
  const auto a = rampPath(line({2.0, 0.4}, {0.0, 0.4}, 5), 0.0, 12);
  const auto b_axis = rampPath(line({2.4, 0.0}, {0.4, 0.0}, 8), 0.0, 17);
  const auto b_room = rampPath(line({5.2, 0.4}, {0.0, 0.4}, 5), 0.0, 25);
  for (const auto* part : {&a, &b_axis, &b_room}) {
    for (Vertex* v : *part) v->state.z() = 0.5;
  }
  const auto chain = [&graph](Vertex* from, const std::vector<Vertex*>& to) {
    for (Vertex* v : to) {
      graph.addVertex(v);
      graph.addEdge(v, from, (v->state - from->state).head<3>().norm());
      from = v;
    }
  };
  graph.addVertex(trunk.front());
  chain(trunk.front(), std::vector<Vertex*>(trunk.begin() + 1, trunk.end()));
  chain(trunk.back(), a);
  chain(trunk.back(), b_axis);
  chain(b_axis.back(), b_room);
  a.back()->vol_gain.gain = 100.0;
  b_room.back()->vol_gain.gain = 50.0;

  const Walls map(corridorWall);
  const RobotParams bot = robot();
  const auto room = [&map, &bot](const StateVec& pose) {
    return mgg::turnClear(map, bot, pose);
  };
  mgg::PlanningParams planning;
  planning.path_length_penalty = 0.0;
  planning.path_direction_penalty = 0.0;
  mgg::EdgeInclinations flat;

  // Level ground: without room to check, the richer branch A wins.
  PathTurnCheck slope_only(graph, bot);
  EXPECT_EQ(mgg::selectBestPath(graph, planning, bot, flat, 0.2, 0.0, {}, 0.0,
                                nullptr, std::ref(slope_only))
                .best_path_id,
            a.back()->id);

  PathTurnCheck check(graph, bot, room);
  const auto r = mgg::selectBestPath(graph, planning, bot, flat, 0.2, 0.0, {},
                                     0.0, nullptr, std::ref(check));
  EXPECT_EQ(r.best_path_id, b_room.back()->id);
  EXPECT_EQ(r.paths_with_sharp_turns, 1);
  EXPECT_FALSE(r.sharp_turn_fallback);
  EXPECT_EQ(check.refused_without_room, 1);
  EXPECT_EQ(check.refused_on_slope, 0);

  // With nothing to gain in the room, the corridor turn is still taken.
  b_room.back()->vol_gain.gain = 0.0;
  const auto fallback = mgg::selectBestPath(
      graph, planning, bot, flat, 0.2, 0.0, {}, 0.0, nullptr, std::ref(check));
  EXPECT_EQ(fallback.best_path_id, a.back()->id);
  EXPECT_TRUE(fallback.sharp_turn_fallback);
}

TEST(PathTurnCheck, ShortcutMakesNoTurnTheChosenPathDidNotHave) {
  // Review r0, P1 1: segment headings of 0, 30, 60 and 90 degrees, one
  // metre each, over the 16 degree ramp. Each turn is 30 degrees, so the
  // path is chosen; joining the segments in pairs gives headings of 15 and
  // 75 degrees, a 60 degree turn on the ramp.
  GraphManager ground;
  addRampLattice(ground);
  std::vector<Eigen::Vector2d> at = {{0.2, 0.0}};
  for (double heading : {0.0, 30.0, 60.0, 90.0}) {
    at.push_back(at.back() + Eigen::Vector2d(std::cos(heading * kDeg),
                                             std::sin(heading * kDeg)));
  }
  const auto branch = rampPath(at, 0.0);
  GraphManager graph;
  graph.addVertex(branch.front());
  for (std::size_t i = 1; i < branch.size(); ++i) {
    graph.addVertex(branch[i]);
    graph.addEdge(branch[i], branch[i - 1], 1.0);
  }
  branch.back()->vol_gain.gain = 100.0;
  PathTurnCheck check(ground, robot());
  mgg::PlanningParams planning;
  mgg::EdgeInclinations flat;
  const auto sel = mgg::selectBestPath(graph, planning, robot(), flat, 0.2,
                                       0.0, {}, 0.0, nullptr,
                                       std::ref(check));
  ASSERT_EQ(sel.best_path_id, branch.back()->id);
  ASSERT_FALSE(sel.sharp_turn_fallback);

  // The planner's shortcut, with only leaps over one point free.
  mgg::PathType points;
  for (const Vertex* v : sel.best_path) points.push_back(v->state.head<3>());
  const auto index = [&points](const Eigen::Vector3d& p) {
    return std::find(points.begin(), points.end(), p) - points.begin();
  };
  const mgg::SegmentFreeFn skip_one = [&index](const Eigen::Vector3d& a,
                                               const Eigen::Vector3d& b) {
    return index(b) - index(a) <= 2;
  };
  const mgg::PathOkFn turns_ok = [&check](const mgg::PathType& p) {
    return check.admissible(p, 0.0);
  };
  ASSERT_TRUE(turns_ok(points));
  const mgg::PathType plain = mgg::shortcutPath(points, skip_one);
  ASSERT_EQ(plain.size(), 3u);
  EXPECT_FALSE(turns_ok(plain));  // what the planner used to send

  const mgg::PathType kept = mgg::shortcutPath(points, skip_one, turns_ok);
  EXPECT_TRUE(turns_ok(kept));
  EXPECT_LT(kept.size(), points.size());  // it still shortcuts where it can
  EXPECT_EQ(kept.front(), points.front());
  EXPECT_EQ(kept.back(), points.back());
}

TEST(PathTurnCheck, LongerRouteToTheSameFrontierWinsOverATurnOnTheRamp) {
  // Review r0, P1 2. The frontier F lies up the ramp to the left. The short
  // way turns left across the ramp from its axis; the long way carries on
  // to the level top, turns there, and comes back down onto F. The detour's
  // vertices carry no gain, so every leaf's shortest path to gain turns on
  // the ramp.
  GraphManager ground;
  addRampLattice(ground);
  GraphManager graph;
  const auto trunk = rampPath(line({-1.2, 0.0}, {0.4, 0.0}, 7), 0.0);
  const auto left = rampPath(line({1.2, 0.4}, {0.0, 0.4}, 4), 0.0, 7);
  const auto axis = rampPath(line({1.6, 0.0}, {0.4, 0.0}, 6), 0.0, 11);
  const auto top = rampPath(line({3.6, 0.4}, {0.0, 0.4}, 4), 0.0, 17);
  const auto back = rampPath(line({3.2, 1.6}, {-0.4, 0.0}, 5), 0.0, 21);
  const auto chain = [&graph](Vertex* from, const std::vector<Vertex*>& to) {
    for (Vertex* v : to) {
      graph.addVertex(v);
      graph.addEdge(v, from, (v->state - from->state).head<3>().norm());
      from = v;
    }
  };
  graph.addVertex(trunk.front());
  chain(trunk.front(), std::vector<Vertex*>(trunk.begin() + 1, trunk.end()));
  chain(trunk.back(), left);
  chain(trunk.back(), axis);
  chain(axis.back(), top);
  chain(top.back(), back);
  Vertex* frontier = left.back();  // (1.2, 1.6)
  graph.addEdge(frontier, back.back(),
                (frontier->state - back.back()->state).head<3>().norm());
  frontier->vol_gain.gain = 100.0;

  mgg::PlanningParams planning;
  planning.path_length_penalty = 0.0;
  planning.path_direction_penalty = 0.0;
  mgg::EdgeInclinations flat;
  PathTurnCheck check(ground, robot());
  const mgg::SharpTurnAllowedFn allowed = [&check](const Vertex& v) {
    return check.sharpTurnAllowedAt(v.state.head<3>());
  };

  // Without the search, the only paths to the frontier's gain turn on the
  // ramp, and one of them is taken as the fallback.
  const auto before = mgg::selectBestPath(graph, planning, robot(), flat, 0.2,
                                          0.0, {}, 0.0, nullptr,
                                          std::ref(check));
  EXPECT_TRUE(before.sharp_turn_fallback);

  const auto r = mgg::selectBestPath(graph, planning, robot(), flat, 0.2, 0.0,
                                     {}, 0.0, nullptr, std::ref(check),
                                     allowed);
  EXPECT_FALSE(r.sharp_turn_fallback);
  EXPECT_TRUE(r.sharp_turn_detour);
  EXPECT_FALSE(r.detour_search_capped);
  EXPECT_GT(r.detour_states_expanded, 0);
  EXPECT_EQ(r.best_path_id, frontier->id);
  ASSERT_GE(r.best_path.size(), 2u);
  // The long way round: over the level top, and into F from (1.6, 1.6).
  EXPECT_NE(std::find(r.best_path.begin(), r.best_path.end(), top.back()),
            r.best_path.end());
  EXPECT_EQ(r.best_path[r.best_path.size() - 2], back.back());
  EXPECT_TRUE(check(r.best_path));
  // Re-parented along the route it takes.
  EXPECT_EQ(frontier->parent, back.back());
}

TEST(FindTurnCompliantRoutes, StopsAtItsBound) {
  GraphManager graph;
  const auto path = rampPath(line({-1.2, 0.0}, {0.4, 0.0}, 6), 0.0);
  graph.addVertex(path.front());
  for (std::size_t i = 1; i < path.size(); ++i) {
    graph.addVertex(path[i]);
    graph.addEdge(path[i], path[i - 1], 0.4);
  }
  const mgg::SharpTurnAllowedFn anywhere = [](const Vertex&) { return true; };
  const auto all = mgg::findTurnCompliantRoutes(graph, 0.0, 0.8, {5}, anywhere,
                                                100);
  ASSERT_EQ(all.to.count(5), 1u);
  EXPECT_EQ(all.to.at(5).path.size(), 6u);
  EXPECT_NEAR(all.to.at(5).along.back(), 2.0, 1e-9);
  EXPECT_FALSE(all.capped);
  const auto bounded =
      mgg::findTurnCompliantRoutes(graph, 0.0, 0.8, {5}, anywhere, 3);
  EXPECT_TRUE(bounded.capped);
  EXPECT_EQ(bounded.states_expanded, 3);
  EXPECT_EQ(bounded.to.count(5), 0u);
}

}  // namespace
