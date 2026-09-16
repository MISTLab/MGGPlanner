#include <atomic>
#include <chrono>
#include <cmath>
#include <set>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/grid_refinement.h"

namespace {

using mgg::BoundedGridPlanner;
using mgg::FeasiblePath;
using mgg::GridRefinementLimits;
using mgg::PlanningStatus;
using mgg::RouteCorridor;
using mgg::StateVec;

RouteCorridor routeTo(double x, double y) {
  RouteCorridor route;
  route.status = PlanningStatus::kSucceeded;
  route.request.objective = mgg::ObjectiveKind::kReturnHome;
  route.request.goal.pose = StateVec(x, y, 0.0, 0.7);
  route.poses.push_back(route.request.goal.pose);
  return route;
}

GridRefinementLimits limits() {
  GridRefinementLimits value;
  value.resolution_m = 1.0;
  value.detour_margin_m = 1.0;
  value.max_cells = 64;
  value.max_expansions = 64;
  value.timeout = std::chrono::milliseconds(100);
  return value;
}

auto flatProjection() {
  return [](StateVec& state) {
    state.z() = 0.0;
    return state.allFinite();
  };
}

auto sampledTraversal(std::vector<Eigen::Vector2d> obstacles) {
  return [obstacles = std::move(obstacles)](
             const StateVec& from, const StateVec& to,
             std::vector<StateVec>& checked) {
    const double length = (to.head<2>() - from.head<2>()).norm();
    const int steps = std::max(1, static_cast<int>(std::ceil(length / 0.1)));
    checked.clear();
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      StateVec sample = from + t * (to - from);
      for (const auto& obstacle : obstacles) {
        if ((sample.head<2>() - obstacle).norm() < 0.35) return false;
      }
      checked.push_back(sample);
    }
    return true;
  };
}

TEST(GridRefinement, ReturnsTheCheckedTerrainPolyline) {
  auto traverse = [](const StateVec& from, const StateVec& to,
                     std::vector<StateVec>& checked) {
    StateVec middle = 0.5 * (from + to);
    middle.z() = 0.2;
    checked = {from, middle, to};
    return true;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), flatProjection(),
                             traverse);
  const FeasiblePath path = planner.refine(routeTo(2.0, 0.0));
  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  ASSERT_EQ(path.poses.size(), 3u);
  EXPECT_NEAR(path.poses[1].z(), 0.2, 1e-9);
  EXPECT_NEAR(path.poses.back()[3], 0.7, 1e-9);
}

TEST(GridRefinement, RejectsMalformedSuccessfulTraversalCallback) {
  auto malformed = [](const StateVec&, const StateVec&,
                      std::vector<StateVec>& checked) {
    checked.clear();
    return true;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), flatProjection(),
                             malformed);
  const FeasiblePath path = planner.refine(routeTo(2.0, 0.0));
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_TRUE(path.poses.empty());
}

TEST(GridRefinement, ExactGoalYawOverridesMatchingGraphVertex) {
  RouteCorridor route = routeTo(2.0, 0.0);
  route.poses.back()[3] = -0.4;
  BoundedGridPlanner planner(StateVec::Zero(), limits(), flatProjection(),
                             sampledTraversal({}));
  const FeasiblePath path = planner.refine(route);
  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back()[3], 0.7, 1e-9);
}

TEST(GridRefinement, ProjectorCannotReplaceRequestOwnedGoalYaw) {
  auto yaw_mutating_projector = [](StateVec& state) {
    state.z() = 0.0;
    state[3] = -2.4;
    return true;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(),
                             yaw_mutating_projector, sampledTraversal({}));
  const FeasiblePath path = planner.refine(routeTo(2.0, 0.0));
  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back()[3], 0.7, 1e-9);
}

TEST(GridRefinement, RejectsExploreBeforeCallingMapPredicates) {
  RouteCorridor route = routeTo(1.0, 0.0);
  route.request.objective = mgg::ObjectiveKind::kExplore;
  int projections = 0;
  auto project = [&projections](StateVec&) {
    ++projections;
    return true;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), project,
                             sampledTraversal({}));
  const FeasiblePath path = planner.refine(route);
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_EQ(projections, 0);
}

TEST(GridRefinement, FindsObservedDetourAroundBlockedChord) {
  BoundedGridPlanner planner(
      StateVec::Zero(), limits(), flatProjection(),
      sampledTraversal({Eigen::Vector2d(1.0, 0.0)}));
  const FeasiblePath path = planner.refine(routeTo(2.0, 0.0));
  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  ASSERT_GE(path.poses.size(), 2u);
  bool left_chord = false;
  for (const StateVec& pose : path.poses) {
    left_chord = left_chord || std::abs(pose.y()) > 0.5;
  }
  EXPECT_TRUE(left_chord);
}

TEST(GridRefinement, UnknownCellsCannotFormADetour) {
  const std::set<std::pair<int, int>> known{{0, 0}, {1, 0}, {2, 0}};
  auto project = [known](StateVec& state) {
    return known.count({static_cast<int>(std::lround(state.x())),
                        static_cast<int>(std::lround(state.y()))}) != 0;
  };
  BoundedGridPlanner planner(
      StateVec::Zero(), limits(), project,
      sampledTraversal({Eigen::Vector2d(1.0, 0.0)}));
  const FeasiblePath path = planner.refine(routeTo(2.0, 0.0));
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_TRUE(path.poses.empty());
  EXPECT_NE(path.reason.find("no observed traversable"), std::string::npos);
}

TEST(GridRefinement, DoesNotCutDiagonallyBetweenBlockedCardinalCells) {
  const std::set<std::pair<int, int>> known{{0, 0}, {1, 1}, {2, 2}};
  auto project = [known](StateVec& state) {
    return known.count({static_cast<int>(std::lround(state.x())),
                        static_cast<int>(std::lround(state.y()))}) != 0;
  };
  auto traverse = [](const StateVec& from, const StateVec& to,
                     std::vector<StateVec>& checked) {
    checked = {from, to};
    return (to.head<2>() - from.head<2>()).norm() < 1.5;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), project, traverse);
  const FeasiblePath path = planner.refine(routeTo(2.0, 2.0));
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_TRUE(path.poses.empty());
}

TEST(GridRefinement, RejectsCellCountBeforeAllocation) {
  GridRefinementLimits bounded = limits();
  bounded.max_cells = 8;
  BoundedGridPlanner planner(StateVec::Zero(), bounded, flatProjection(),
                             sampledTraversal({{1.0, 0.0}}));
  const FeasiblePath path = planner.refine(routeTo(10.0, 0.0));
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_NE(path.reason.find("cell limit"), std::string::npos);
}

TEST(GridRefinement, ExpansionLimitIsGlobalAndDeterministic) {
  GridRefinementLimits bounded = limits();
  bounded.max_expansions = 1;
  BoundedGridPlanner planner(StateVec::Zero(), bounded, flatProjection(),
                             sampledTraversal({{1.0, 0.0}}));
  const FeasiblePath path = planner.refine(routeTo(2.0, 0.0));
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_NE(path.reason.find("expansion limit"), std::string::npos);
}

TEST(GridRefinement, CancellationStopsInsideTheSearch) {
  std::atomic<bool> cancelled{false};
  int traversals = 0;
  auto cancel_from_search = [&cancelled, &traversals](
                                const StateVec&, const StateVec&,
                                std::vector<StateVec>&) {
    ++traversals;
    // The first call is the blocked direct chord. Raise cancellation only
    // when the search evaluates its first candidate transition.
    if (traversals >= 2) cancelled = true;
    return false;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), flatProjection(),
                             cancel_from_search,
                             [&cancelled]() { return cancelled.load(); });
  const FeasiblePath path = planner.refine(routeTo(2.0, 0.0));
  EXPECT_GE(traversals, 2);
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_NE(path.reason.find("cancelled"), std::string::npos);
}

TEST(GridRefinement, CooperativeDeadlineIsCheckedBetweenMapCalls) {
  GridRefinementLimits bounded = limits();
  bounded.timeout = std::chrono::milliseconds(1);
  auto slow_blocked = [](const StateVec&, const StateVec&,
                         std::vector<StateVec>&) {
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    return false;
  };
  BoundedGridPlanner planner(StateVec::Zero(), bounded, flatProjection(),
                             slow_blocked);
  const FeasiblePath path = planner.refine(routeTo(2.0, 0.0));
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_NE(path.reason.find("cooperative deadline"), std::string::npos);
}

TEST(GridRefinement, SlowSuccessfulDirectPathCannotBypassDeadline) {
  GridRefinementLimits bounded = limits();
  bounded.timeout = std::chrono::milliseconds(1);
  auto slow_success = [](const StateVec& from, const StateVec& to,
                         std::vector<StateVec>& checked) {
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    checked = {from, to};
    return true;
  };
  BoundedGridPlanner planner(StateVec::Zero(), bounded, flatProjection(),
                             slow_success);
  const FeasiblePath path = planner.refine(routeTo(2.0, 0.0));
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_NE(path.reason.find("cooperative deadline"), std::string::npos);
}

TEST(GridRefinement, CancellationRaisedByEndpointProjectionIsObserved) {
  std::atomic<bool> cancelled{false};
  int projections = 0;
  auto project = [&cancelled, &projections](StateVec& state) {
    state.z() = 0.0;
    if (++projections == 2) cancelled = true;
    return true;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), project,
                             sampledTraversal({}),
                             [&cancelled]() { return cancelled.load(); });
  const FeasiblePath path = planner.refine(routeTo(2.0, 0.0));
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_NE(path.reason.find("cancelled"), std::string::npos);
}

TEST(GridRefinement, ZeroTimeoutIsInvalidInsteadOfDisablingDeadline) {
  GridRefinementLimits unbounded = limits();
  unbounded.timeout = std::chrono::milliseconds(0);
  BoundedGridPlanner planner(StateVec::Zero(), unbounded, flatProjection(),
                             sampledTraversal({}));
  const FeasiblePath path = planner.refine(routeTo(1.0, 0.0));
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_NE(path.reason.find("configuration"), std::string::npos);
}

}  // namespace
