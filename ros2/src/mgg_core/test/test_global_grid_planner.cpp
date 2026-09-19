#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/global_grid_planner.h"

namespace {

using mgg::GlobalGridPlan;
using mgg::GlobalGridPlanner;
using mgg::GlobalGridPlannerLimits;
using mgg::GlobalGridPlanStatus;
using mgg::RasterCellState;
using mgg::StateVec;
using mgg::TransientDisc;
using mgg::TraversabilityRaster;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

std::shared_ptr<TraversabilityRaster> flatRaster(std::size_t width,
                                                 std::size_t height,
                                                 double cell = 0.5,
                                                 double ground = 0.0) {
  auto raster = std::make_shared<TraversabilityRaster>();
  raster->cell_size = cell;
  raster->origin = Eigen::Vector2d::Zero();
  raster->width = width;
  raster->height = height;
  raster->ground_z.assign(width * height, ground);
  raster->state.assign(width * height, RasterCellState::kFree);
  return raster;
}

void setCell(TraversabilityRaster& raster, std::size_t x, std::size_t y,
             RasterCellState state, double ground = 0.0) {
  raster.state[raster.index(x, y)] = state;
  raster.ground_z[raster.index(x, y)] =
      state == RasterCellState::kUnknown ? kNaN : ground;
}

GlobalGridPlannerLimits limits() {
  GlobalGridPlannerLimits value;
  value.max_step_height = 0.15;
  value.max_drop_height = 0.25;
  value.step_tolerance = 0.01;
  value.driving_offset = 0.4;
  value.body_radius = 0.3;
  value.blocked_penalty = 20.0;
  value.max_expansions = 100000;
  value.timeout = std::chrono::milliseconds(2000);
  return value;
}

bool onCellCentres(const TraversabilityRaster& raster,
                   const std::vector<StateVec>& poses) {
  for (const StateVec& pose : poses) {
    std::size_t x = 0;
    std::size_t y = 0;
    if (!raster.cellOf(pose.head<2>(), x, y)) return false;
    if ((raster.centre(x, y) - pose.head<2>()).norm() > 1e-9) return false;
  }
  return true;
}

bool avoidsState(const TraversabilityRaster& raster,
                 const std::vector<StateVec>& poses, RasterCellState state) {
  for (const StateVec& pose : poses) {
    if (raster.stateAtXY(pose.head<2>()) == state) return false;
  }
  return true;
}

double spacing(const StateVec& a, const StateVec& b) {
  return (b.head<2>() - a.head<2>()).norm();
}

TEST(GlobalGridPlanner, RoutesAroundAWallOnCellCentres) {
  auto raster = flatRaster(12, 10);
  // A wall from the bottom edge up to y = 7 leaves a gap along the top.
  for (std::size_t y = 0; y < 8; ++y) {
    setCell(*raster, 6, y, RasterCellState::kObstacle);
  }
  GlobalGridPlanner planner(raster, limits());
  const GlobalGridPlan plan =
      planner.plan(Eigen::Vector2d(1.3, 0.6), Eigen::Vector2d(5.2, 0.7));
  ASSERT_EQ(plan.status, GlobalGridPlanStatus::kSucceeded) << plan.reason;
  ASSERT_GE(plan.poses.size(), 2u);
  EXPECT_TRUE(onCellCentres(*raster, plan.poses));
  EXPECT_TRUE(avoidsState(*raster, plan.poses, RasterCellState::kObstacle));
  // Starts in the start cell and ends in the goal cell.
  EXPECT_NEAR(plan.poses.front().x(), 1.25, 1e-9);
  EXPECT_NEAR(plan.poses.front().y(), 0.75, 1e-9);
  EXPECT_NEAR(plan.poses.back().x(), 5.25, 1e-9);
  EXPECT_NEAR(plan.poses.back().y(), 0.75, 1e-9);
  // The route must climb above the wall and come back down.
  double top = 0.0;
  for (const StateVec& pose : plan.poses) top = std::max(top, pose.y());
  EXPECT_GE(top, 4.0);
  EXPECT_GT(plan.length_m, 4.0);
  // Each step is one cell along an axis or one diagonal, at driving height,
  // with the yaw pointing along the path.
  for (std::size_t i = 1; i < plan.poses.size(); ++i) {
    const double step = spacing(plan.poses[i - 1], plan.poses[i]);
    EXPECT_TRUE(std::abs(step - 0.5) < 1e-9 ||
                std::abs(step - 0.5 * std::sqrt(2.0)) < 1e-9)
        << step;
    EXPECT_NEAR(plan.poses[i].z(), 0.4, 1e-9);
    const double yaw = std::atan2(plan.poses[i].y() - plan.poses[i - 1].y(),
                                  plan.poses[i].x() - plan.poses[i - 1].x());
    EXPECT_NEAR(plan.poses[i - 1][3], yaw, 1e-9);
  }
  EXPECT_NEAR(plan.poses.back()[3], plan.poses[plan.poses.size() - 2u][3],
              1e-9);
  EXPECT_GT(plan.expansions, 0u);
}

TEST(GlobalGridPlanner, StraightRouteIsSpacedOneCellApart) {
  auto raster = flatRaster(20, 5, 0.5, 1.0);
  GlobalGridPlanner planner(raster, limits());
  const GlobalGridPlan plan =
      planner.plan(Eigen::Vector2d(0.3, 1.2), Eigen::Vector2d(9.7, 1.2));
  ASSERT_EQ(plan.status, GlobalGridPlanStatus::kSucceeded) << plan.reason;
  // Cells 0 to 19 along one row: twenty poses, half a metre apart.
  ASSERT_EQ(plan.poses.size(), 20u);
  for (std::size_t i = 1; i < plan.poses.size(); ++i) {
    EXPECT_NEAR(spacing(plan.poses[i - 1], plan.poses[i]), 0.5, 1e-9);
    EXPECT_NEAR(plan.poses[i][3], 0.0, 1e-9);
  }
  EXPECT_NEAR(plan.length_m, 9.5, 1e-9);
  for (const StateVec& pose : plan.poses) EXPECT_NEAR(pose.z(), 1.4, 1e-9);
}

TEST(GlobalGridPlanner, SameCellStartAndGoalIsOnePose) {
  auto raster = flatRaster(4, 4);
  GlobalGridPlanner planner(raster, limits());
  const GlobalGridPlan plan =
      planner.plan(Eigen::Vector2d(0.6, 0.6), Eigen::Vector2d(0.9, 0.8));
  ASSERT_EQ(plan.status, GlobalGridPlanStatus::kSucceeded) << plan.reason;
  ASSERT_EQ(plan.poses.size(), 1u);
  EXPECT_NEAR(plan.poses.front().x(), 0.75, 1e-9);
  EXPECT_NEAR(plan.poses.front().y(), 0.75, 1e-9);
  EXPECT_NEAR(plan.length_m, 0.0, 1e-12);
}

TEST(GlobalGridPlanner, KerbIsRefusedUpwardButAllowedDownwardWithinTheDrop) {
  // Left half at 0.0 m, right half a 0.20 m kerb higher: above the 0.15 m
  // step limit, within the 0.25 m drop limit.
  auto raster = flatRaster(8, 3);
  for (std::size_t y = 0; y < 3; ++y) {
    for (std::size_t x = 4; x < 8; ++x) {
      setCell(*raster, x, y, RasterCellState::kFree, 0.20);
    }
  }
  GlobalGridPlanner planner(raster, limits());
  const GlobalGridPlan up =
      planner.plan(Eigen::Vector2d(0.25, 0.75), Eigen::Vector2d(3.75, 0.75));
  EXPECT_EQ(up.status, GlobalGridPlanStatus::kUnreachable) << up.reason;
  EXPECT_TRUE(up.poses.empty());
  const GlobalGridPlan down =
      planner.plan(Eigen::Vector2d(3.75, 0.75), Eigen::Vector2d(0.25, 0.75));
  ASSERT_EQ(down.status, GlobalGridPlanStatus::kSucceeded) << down.reason;
  ASSERT_EQ(down.poses.size(), 8u);
  EXPECT_NEAR(down.poses.front().z(), 0.60, 1e-9);
  EXPECT_NEAR(down.poses.back().z(), 0.40, 1e-9);

  // A drop beyond the limit is refused in both directions.
  for (std::size_t y = 0; y < 3; ++y) {
    for (std::size_t x = 4; x < 8; ++x) {
      setCell(*raster, x, y, RasterCellState::kFree, 0.30);
    }
  }
  GlobalGridPlanner steep(raster, limits());
  EXPECT_EQ(steep.plan(Eigen::Vector2d(3.75, 0.75), Eigen::Vector2d(0.25, 0.75))
                .status,
            GlobalGridPlanStatus::kUnreachable);
  // A rise at the limit passes with the measurement tolerance.
  for (std::size_t y = 0; y < 3; ++y) {
    for (std::size_t x = 4; x < 8; ++x) {
      setCell(*raster, x, y, RasterCellState::kFree, 0.155);
    }
  }
  GlobalGridPlanner tolerant(raster, limits());
  EXPECT_EQ(tolerant.plan(Eigen::Vector2d(0.25, 0.75),
                          Eigen::Vector2d(3.75, 0.75))
                .status,
            GlobalGridPlanStatus::kSucceeded);
}

TEST(GlobalGridPlanner, UnknownCellsAreRefusedExceptAgreeingEndpoints) {
  auto raster = flatRaster(8, 3);
  for (std::size_t y = 0; y < 3; ++y) {
    setCell(*raster, 4, y, RasterCellState::kUnknown);
  }
  GlobalGridPlanner planner(raster, limits());
  const GlobalGridPlan across =
      planner.plan(Eigen::Vector2d(0.25, 0.75), Eigen::Vector2d(3.75, 0.75));
  EXPECT_EQ(across.status, GlobalGridPlanStatus::kUnreachable) << across.reason;

  // The robot's own footprint is unobserved; its free neighbours at 0.0 m
  // lend it their ground, and the emitted start pose sits at driving height.
  auto own = flatRaster(8, 3);
  setCell(*own, 1, 1, RasterCellState::kUnknown);
  GlobalGridPlanner own_planner(own, limits());
  const GlobalGridPlan from_unknown =
      own_planner.plan(Eigen::Vector2d(0.75, 0.75), Eigen::Vector2d(3.75, 0.75));
  ASSERT_EQ(from_unknown.status, GlobalGridPlanStatus::kSucceeded)
      << from_unknown.reason;
  EXPECT_NEAR(from_unknown.poses.front().x(), 0.75, 1e-9);
  EXPECT_NEAR(from_unknown.poses.front().z(), 0.40, 1e-9);

  // A goal on a departed neighbour's masked footprint takes the median of
  // its known neighbours.
  auto masked = flatRaster(8, 3, 0.5, 0.10);
  setCell(*masked, 6, 1, RasterCellState::kUnknown);
  GlobalGridPlanner masked_planner(masked, limits());
  const GlobalGridPlan to_unknown = masked_planner.plan(
      Eigen::Vector2d(0.75, 0.75), Eigen::Vector2d(3.25, 0.75));
  ASSERT_EQ(to_unknown.status, GlobalGridPlanStatus::kSucceeded)
      << to_unknown.reason;
  EXPECT_NEAR(to_unknown.poses.back().x(), 3.25, 1e-9);
  EXPECT_NEAR(to_unknown.poses.back().z(), 0.50, 1e-9);

  // No known free cell within the body radius: refused with the reason.
  auto island = flatRaster(8, 3);
  for (std::size_t y = 0; y < 3; ++y) {
    for (std::size_t x = 0; x < 3; ++x) {
      setCell(*island, x, y, RasterCellState::kUnknown);
    }
  }
  GlobalGridPlanner island_planner(island, limits());
  const GlobalGridPlan no_neighbours = island_planner.plan(
      Eigen::Vector2d(0.75, 0.75), Eigen::Vector2d(3.75, 0.75));
  EXPECT_EQ(no_neighbours.status, GlobalGridPlanStatus::kStartUnknown);
  EXPECT_NE(no_neighbours.reason.find("unobserved"), std::string::npos);

  // Neighbours that disagree by more than one step cannot lend a height.
  auto split = flatRaster(8, 3);
  setCell(*split, 1, 1, RasterCellState::kUnknown);
  for (std::size_t y = 0; y < 3; ++y) {
    setCell(*split, 0, y, RasterCellState::kFree, 0.0);
    setCell(*split, 2, y, RasterCellState::kFree, 0.5);
  }
  setCell(*split, 1, 0, RasterCellState::kFree, 0.5);
  setCell(*split, 1, 2, RasterCellState::kFree, 0.0);
  GlobalGridPlanner split_planner(split, limits());
  const GlobalGridPlan disagreeing = split_planner.plan(
      Eigen::Vector2d(0.75, 0.75), Eigen::Vector2d(3.75, 0.75));
  EXPECT_EQ(disagreeing.status, GlobalGridPlanStatus::kStartUnknown);
  EXPECT_NE(disagreeing.reason.find("disagree"), std::string::npos);

  // The goal side reports its own class.
  auto goal_island = flatRaster(8, 3);
  for (std::size_t y = 0; y < 3; ++y) {
    for (std::size_t x = 5; x < 8; ++x) {
      setCell(*goal_island, x, y, RasterCellState::kUnknown);
    }
  }
  GlobalGridPlanner goal_planner(goal_island, limits());
  EXPECT_EQ(goal_planner.plan(Eigen::Vector2d(0.75, 0.75),
                              Eigen::Vector2d(3.75, 0.75))
                .status,
            GlobalGridPlanStatus::kGoalUnknown);
}

TEST(GlobalGridPlanner, ObstacleEndpointsAreRefused) {
  auto raster = flatRaster(6, 3);
  setCell(*raster, 0, 1, RasterCellState::kObstacle);
  setCell(*raster, 5, 1, RasterCellState::kObstacle);
  GlobalGridPlanner planner(raster, limits());
  EXPECT_EQ(planner.plan(Eigen::Vector2d(0.25, 0.75), Eigen::Vector2d(2.25, 0.75))
                .status,
            GlobalGridPlanStatus::kStartObstacle);
  EXPECT_EQ(planner.plan(Eigen::Vector2d(2.25, 0.75), Eigen::Vector2d(2.75, 0.75))
                .status,
            GlobalGridPlanStatus::kGoalObstacle);
  EXPECT_EQ(planner.plan(Eigen::Vector2d(-1.0, 0.75), Eigen::Vector2d(2.75, 0.75))
                .status,
            GlobalGridPlanStatus::kStartOutside);
  EXPECT_EQ(planner.plan(Eigen::Vector2d(2.25, 0.75), Eigen::Vector2d(2.75, 9.0))
                .status,
            GlobalGridPlanStatus::kGoalOutside);
  GlobalGridPlanner none(nullptr, limits());
  EXPECT_EQ(none.plan(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 0.0)).status,
            GlobalGridPlanStatus::kNoRaster);
}

TEST(GlobalGridPlanner, TransientDiscIsAvoidedAndMayBeLeft) {
  auto raster = flatRaster(12, 7);
  GlobalGridPlanner planner(raster, limits());
  // A neighbour parked on the straight line between start and goal.
  const std::vector<TransientDisc> discs{{Eigen::Vector2d(3.0, 1.75), 0.9}};
  const GlobalGridPlan plan = planner.plan(
      Eigen::Vector2d(0.75, 1.75), Eigen::Vector2d(5.25, 1.75), discs);
  ASSERT_EQ(plan.status, GlobalGridPlanStatus::kSucceeded) << plan.reason;
  for (const StateVec& pose : plan.poses) {
    EXPECT_GE((pose.head<2>() - discs.front().centre).norm(), 0.9 - 1e-9)
        << pose.transpose();
  }
  EXPECT_GT(plan.length_m, 4.5);

  // A goal inside the disc is refused: the neighbour has not left.
  const GlobalGridPlan into = planner.plan(
      Eigen::Vector2d(0.75, 1.75), Eigen::Vector2d(3.25, 1.75), discs);
  EXPECT_EQ(into.status, GlobalGridPlanStatus::kGoalInDisc) << into.reason;

  // A start inside the disc (two robots parked side by side) may drive away.
  const GlobalGridPlan away = planner.plan(
      Eigen::Vector2d(2.25, 1.75), Eigen::Vector2d(0.25, 1.75), discs);
  ASSERT_EQ(away.status, GlobalGridPlanStatus::kSucceeded) << away.reason;
  for (std::size_t i = 1; i < away.poses.size(); ++i) {
    EXPECT_GE((away.poses[i].head<2>() - discs.front().centre).norm(),
              0.9 - 1e-9);
  }
}

TEST(GlobalGridPlanner, DiagonalsDoNotCutCorners) {
  // Two obstacles touching at a corner: the diagonal between them is refused,
  // so the route has to go around.
  auto raster = flatRaster(4, 4);
  setCell(*raster, 1, 2, RasterCellState::kObstacle);
  setCell(*raster, 2, 1, RasterCellState::kObstacle);
  GlobalGridPlanner planner(raster, limits());
  const GlobalGridPlan plan =
      planner.plan(Eigen::Vector2d(0.75, 0.75), Eigen::Vector2d(1.25, 1.25));
  ASSERT_EQ(plan.status, GlobalGridPlanStatus::kSucceeded) << plan.reason;
  EXPECT_GT(plan.poses.size(), 2u);
  EXPECT_TRUE(avoidsState(*raster, plan.poses, RasterCellState::kObstacle));
}

TEST(GlobalGridPlanner, BlockedEdgesArePenalisedNotVetoed) {
  // A corridor two cells wide with a single-cell gap in a wall at x = 3; the
  // long way round costs about six metres more.
  auto raster = flatRaster(12, 8);
  for (std::size_t y = 0; y < 8; ++y) {
    if (y == 1) continue;
    setCell(*raster, 6, y, RasterCellState::kObstacle);
  }
  // Open a second gap at the far end.
  setCell(*raster, 6, 7, RasterCellState::kFree);
  GlobalGridPlanner planner(raster, limits());
  const Eigen::Vector2d start(1.25, 0.75);
  const Eigen::Vector2d goal(5.25, 0.75);
  const auto through_gap = [](const StateVec& a, const StateVec& b) {
    const bool crosses = (a.x() < 3.0) != (b.x() < 3.0);
    return crosses && a.y() < 1.5 && b.y() < 1.5;
  };
  const GlobalGridPlan direct = planner.plan(start, goal);
  ASSERT_EQ(direct.status, GlobalGridPlanStatus::kSucceeded) << direct.reason;
  EXPECT_LT(direct.length_m, 5.0);

  const GlobalGridPlan detour = planner.plan(start, goal, {}, through_gap);
  ASSERT_EQ(detour.status, GlobalGridPlanStatus::kSucceeded) << detour.reason;
  EXPECT_GT(detour.length_m, 8.0);
  for (std::size_t i = 1; i < detour.poses.size(); ++i) {
    EXPECT_FALSE(through_gap(detour.poses[i - 1], detour.poses[i]));
  }

  // With a penalty smaller than the detour the marked edge is still taken.
  GlobalGridPlannerLimits cheap = limits();
  cheap.blocked_penalty = 1.0;
  GlobalGridPlanner lenient(raster, cheap);
  const GlobalGridPlan taken = lenient.plan(start, goal, {}, through_gap);
  ASSERT_EQ(taken.status, GlobalGridPlanStatus::kSucceeded) << taken.reason;
  EXPECT_LT(taken.length_m, 5.0);

  // Sealing the second gap leaves the marked edge as the only route.
  setCell(*raster, 6, 7, RasterCellState::kObstacle);
  GlobalGridPlanner sealed(raster, limits());
  const GlobalGridPlan only = sealed.plan(start, goal, {}, through_gap);
  ASSERT_EQ(only.status, GlobalGridPlanStatus::kSucceeded) << only.reason;
  EXPECT_LT(only.length_m, 5.0);
}

TEST(GlobalGridPlanner, BudgetAndDeadlineEndTheSearchWithAReason) {
  auto raster = flatRaster(40, 40);
  GlobalGridPlannerLimits tight = limits();
  tight.max_expansions = 5;
  GlobalGridPlanner budgeted(raster, tight);
  const GlobalGridPlan over = budgeted.plan(Eigen::Vector2d(0.25, 0.25),
                                            Eigen::Vector2d(19.75, 19.75));
  EXPECT_EQ(over.status, GlobalGridPlanStatus::kBudgetExceeded) << over.reason;
  EXPECT_NE(over.reason.find("expansion budget"), std::string::npos);
  EXPECT_TRUE(over.poses.empty());
  EXPECT_EQ(over.expansions, 5u);

  // An enclosed goal on a large raster forces the search to visit everything;
  // a one millisecond deadline ends it first.
  auto large = flatRaster(1000, 1000);
  for (std::size_t x = 990; x < 1000; ++x) {
    setCell(*large, x, 989, RasterCellState::kObstacle);
  }
  for (std::size_t y = 989; y < 1000; ++y) {
    setCell(*large, 989, y, RasterCellState::kObstacle);
  }
  GlobalGridPlannerLimits brief = limits();
  brief.max_expansions = 10000000;
  brief.timeout = std::chrono::milliseconds(1);
  GlobalGridPlanner timed(large, brief);
  const GlobalGridPlan late = timed.plan(Eigen::Vector2d(0.25, 0.25),
                                         Eigen::Vector2d(499.75, 499.75));
  EXPECT_EQ(late.status, GlobalGridPlanStatus::kDeadlineExceeded) << late.reason;
  EXPECT_NE(late.reason.find("deadline"), std::string::npos);
  EXPECT_TRUE(late.poses.empty());
}

TEST(GlobalGridPlanner, InvalidLimitsAreRefused) {
  auto raster = flatRaster(4, 4);
  GlobalGridPlannerLimits bad = limits();
  bad.max_expansions = 0;
  EXPECT_EQ(GlobalGridPlanner(raster, bad)
                .plan(Eigen::Vector2d(0.25, 0.25), Eigen::Vector2d(1.75, 1.75))
                .status,
            GlobalGridPlanStatus::kInvalidConfiguration);
  bad = limits();
  bad.max_step_height = -1.0;
  EXPECT_EQ(GlobalGridPlanner(raster, bad)
                .plan(Eigen::Vector2d(0.25, 0.25), Eigen::Vector2d(1.75, 1.75))
                .status,
            GlobalGridPlanStatus::kInvalidConfiguration);
  auto inconsistent = flatRaster(4, 4);
  inconsistent->state.pop_back();
  EXPECT_EQ(GlobalGridPlanner(inconsistent, limits())
                .plan(Eigen::Vector2d(0.25, 0.25), Eigen::Vector2d(1.75, 1.75))
                .status,
            GlobalGridPlanStatus::kNoRaster);
  bad = limits();
  bad.unknown_cost_factor = 0.5;
  EXPECT_EQ(GlobalGridPlanner(raster, bad)
                .plan(Eigen::Vector2d(0.25, 0.25), Eigen::Vector2d(1.75, 1.75))
                .status,
            GlobalGridPlanStatus::kInvalidConfiguration);
}

TEST(GlobalGridPlanner, UnknownCellsAreCrossedAtTheirCostWhenAdmitted) {
  // An 8 by 5 raster with an unobserved column at x = 4 in rows 0 to 3; row
  // 4 is observed all along, so a known detour exists over the top.
  auto raster = flatRaster(8, 5);
  for (std::size_t y = 0; y < 4; ++y) {
    setCell(*raster, 4, y, RasterCellState::kUnknown);
  }
  const Eigen::Vector2d from(0.75, 0.75);
  const Eigen::Vector2d to(3.75, 0.75);

  // The straight route crosses one unknown cell (2.5 m known + 0.5 m
  // unknown); the known detour over row 4 is six diagonals, 4.24 m. At a
  // factor of 3 the crossing costs 4.0 m and wins; at 8 it costs 6.5 m and
  // the detour wins. The crossed cell carries the ground of the cell before
  // it, so every pose stays at driving height.
  GlobalGridPlannerLimits cheap = limits();
  cheap.unknown_cost_factor = 3.0;
  const GlobalGridPlan crossing = GlobalGridPlanner(raster, cheap).plan(from, to);
  ASSERT_EQ(crossing.status, GlobalGridPlanStatus::kSucceeded) << crossing.reason;
  EXPECT_FALSE(avoidsState(*raster, crossing.poses, RasterCellState::kUnknown));
  EXPECT_NEAR(crossing.length_m, 3.0, 1e-9);
  EXPECT_TRUE(onCellCentres(*raster, crossing.poses));
  for (const StateVec& pose : crossing.poses) EXPECT_NEAR(pose.z(), 0.40, 1e-9);

  GlobalGridPlannerLimits dear = limits();
  dear.unknown_cost_factor = 8.0;
  const GlobalGridPlan detour = GlobalGridPlanner(raster, dear).plan(from, to);
  ASSERT_EQ(detour.status, GlobalGridPlanStatus::kSucceeded) << detour.reason;
  EXPECT_TRUE(avoidsState(*raster, detour.poses, RasterCellState::kUnknown));
  EXPECT_GT(detour.length_m, 4.0);

  // A wall on the known side leaves only the crossing; the search takes it.
  auto walled = flatRaster(8, 5);
  for (std::size_t y = 0; y < 4; ++y) {
    setCell(*walled, 4, y, RasterCellState::kUnknown);
  }
  for (std::size_t x = 0; x < 8; ++x) {
    setCell(*walled, x, 4, RasterCellState::kObstacle);
  }
  const GlobalGridPlan forced = GlobalGridPlanner(walled, dear).plan(from, to);
  ASSERT_EQ(forced.status, GlobalGridPlanStatus::kSucceeded) << forced.reason;
  EXPECT_FALSE(avoidsState(*walled, forced.poses, RasterCellState::kUnknown));
  // Without the factor the same raster is unreachable, as before.
  EXPECT_EQ(GlobalGridPlanner(walled, limits()).plan(from, to).status,
            GlobalGridPlanStatus::kUnreachable);
}

TEST(GlobalGridPlanner, InflatedCellsAreObstaclesUntilAdmittedAtACost) {
  // A wall along y = 1.25 across x in [1.0, 3.0) with its inflation ring
  // (the cells around it), on an 8 by 4 raster. The goal stands in the ring
  // beside the wall, as an operator's goal next to a building does.
  auto raster = flatRaster(8, 4);
  for (std::size_t x = 2; x < 6; ++x) {
    setCell(*raster, x, 2, RasterCellState::kObstacle);
  }
  for (std::size_t x = 1; x < 7; ++x) {
    for (const std::size_t y : {1u, 3u}) {
      setCell(*raster, x, y, RasterCellState::kInflated);
    }
  }
  setCell(*raster, 1, 2, RasterCellState::kInflated);
  setCell(*raster, 6, 2, RasterCellState::kInflated);
  const Eigen::Vector2d from(0.25, 0.25);
  const Eigen::Vector2d beside(2.25, 0.75);

  // Refused as an obstacle endpoint by default, with a reason that says why.
  const GlobalGridPlan refused = GlobalGridPlanner(raster, limits()).plan(from, beside);
  EXPECT_EQ(refused.status, GlobalGridPlanStatus::kGoalObstacle);
  EXPECT_NE(refused.reason.find("body radius"), std::string::npos) << refused.reason;

  // Admitted at a cost: the route reaches the goal and enters the ring only
  // for the goal cell itself, staying on free cells before that.
  GlobalGridPlannerLimits admit = limits();
  admit.inflated_cost_factor = 3.0;
  const GlobalGridPlan reached = GlobalGridPlanner(raster, admit).plan(from, beside);
  ASSERT_EQ(reached.status, GlobalGridPlanStatus::kSucceeded) << reached.reason;
  EXPECT_NEAR(reached.poses.back().x(), 2.25, 1e-9);
  EXPECT_NEAR(reached.poses.back().y(), 0.75, 1e-9);
  std::size_t inflated_poses = 0;
  for (const StateVec& pose : reached.poses) {
    inflated_poses += raster->stateAtXY(pose.head<2>()) == RasterCellState::kInflated;
  }
  EXPECT_EQ(inflated_poses, 1u);
  EXPECT_TRUE(avoidsState(*raster, reached.poses, RasterCellState::kObstacle));

  // A start in the ring (a robot parked beside the wall) leaves it.
  const GlobalGridPlan parked =
      GlobalGridPlanner(raster, admit).plan(beside, Eigen::Vector2d(3.75, 0.25));
  ASSERT_EQ(parked.status, GlobalGridPlanStatus::kSucceeded) << parked.reason;
  EXPECT_TRUE(avoidsState(*raster, parked.poses, RasterCellState::kObstacle));

  // Obstacle cells themselves stay refused whatever the factor.
  EXPECT_EQ(GlobalGridPlanner(raster, admit)
                .plan(from, Eigen::Vector2d(2.25, 1.25))
                .status,
            GlobalGridPlanStatus::kGoalObstacle);
  GlobalGridPlannerLimits bad = limits();
  bad.inflated_cost_factor = 0.5;
  EXPECT_EQ(GlobalGridPlanner(raster, bad).plan(from, beside).status,
            GlobalGridPlanStatus::kInvalidConfiguration);
}

TEST(GlobalGridPlanner, CarriedGroundJudgesTheStepBackOntoKnownGround) {
  // The start sits in a 3 by 3 unobserved block with no known neighbour in
  // reach; the caller's hint is its ground. Known ground beyond is at 0.0 m.
  auto island = flatRaster(8, 3);
  for (std::size_t y = 0; y < 3; ++y) {
    for (std::size_t x = 0; x < 3; ++x) {
      setCell(*island, x, y, RasterCellState::kUnknown);
    }
  }
  GlobalGridPlannerLimits admit = limits();
  admit.unknown_cost_factor = 3.0;
  const GlobalGridPlanner planner(island, admit);
  const Eigen::Vector2d from(0.75, 0.75);
  const Eigen::Vector2d to(3.75, 0.75);

  // No hint: refused as before, with the reason.
  const GlobalGridPlan unhinted = planner.plan(from, to);
  EXPECT_EQ(unhinted.status, GlobalGridPlanStatus::kStartUnknown);

  // A 0.20 m hint is a 0.20 m drop onto the known road, within the 0.25 m
  // drop limit: the route crosses the block on the hinted ground and steps
  // down where the observed cells begin.
  const GlobalGridPlan stepped = planner.plan(from, to, {}, {}, 0.20);
  ASSERT_EQ(stepped.status, GlobalGridPlanStatus::kSucceeded) << stepped.reason;
  EXPECT_NEAR(stepped.poses.front().z(), 0.60, 1e-9);
  EXPECT_NEAR(stepped.poses.back().z(), 0.40, 1e-9);
  bool stepped_down = false;
  for (std::size_t i = 1; i < stepped.poses.size(); ++i) {
    const double delta = stepped.poses[i].z() - stepped.poses[i - 1].z();
    EXPECT_LE(delta, 1e-9);
    if (delta < -0.19) stepped_down = true;
  }
  EXPECT_TRUE(stepped_down);

  // A 0.50 m hint would be a 0.50 m drop everywhere the road begins: no
  // route may step down that far, so the goal is unreachable.
  const GlobalGridPlan cliff = planner.plan(from, to, {}, {}, 0.50);
  EXPECT_EQ(cliff.status, GlobalGridPlanStatus::kUnreachable) << cliff.reason;

  // A goal in an unobserved region beyond the map carries the last known
  // ground of its path and costs its multiple.
  auto beyond = flatRaster(8, 3, 0.5, 0.10);
  for (std::size_t y = 0; y < 3; ++y) {
    for (std::size_t x = 5; x < 8; ++x) {
      setCell(*beyond, x, y, RasterCellState::kUnknown);
    }
  }
  const GlobalGridPlan onward =
      GlobalGridPlanner(beyond, admit).plan(from, Eigen::Vector2d(3.75, 0.75));
  ASSERT_EQ(onward.status, GlobalGridPlanStatus::kSucceeded) << onward.reason;
  EXPECT_NEAR(onward.poses.back().z(), 0.50, 1e-9);
  EXPECT_NEAR(onward.poses.back().x(), 3.75, 1e-9);
  EXPECT_EQ(GlobalGridPlanner(beyond, limits())
                .plan(from, Eigen::Vector2d(3.75, 0.75))
                .status,
            GlobalGridPlanStatus::kGoalUnknown);
}

}  // namespace
