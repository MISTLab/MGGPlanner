#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/grid_refinement.h"

namespace {

using mgg::BoundedGridPlanner;
using mgg::FeasiblePath;
using mgg::GridProjectionStatus;
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
    return state.allFinite() ? GridProjectionStatus::kSupported
                             : GridProjectionStatus::kNoGround;
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

TEST(GridRefinement, BoundedStartConnectorReachesFirstObservedRoadCell) {
  GridRefinementLimits bounded = limits();
  bounded.resolution_m = 0.25;
  bounded.detour_margin_m = 1.5;
  bounded.start_connector_max_distance_m = 1.5;
  auto project = [](StateVec& state) {
    if (std::abs(state.x()) < 1e-9 || state.x() >= 1.0) {
      state.z() = 0.0;
      return GridProjectionStatus::kSupported;
    }
    return GridProjectionStatus::kNoGround;
  };
  BoundedGridPlanner planner(StateVec::Zero(), bounded, project,
                             sampledTraversal({}));
  const FeasiblePath result = planner.refine(routeTo(3.0, 0.0));
  EXPECT_EQ(result.status, PlanningStatus::kSucceeded) << result.reason;
  ASSERT_GE(result.poses.size(), 2u);
  EXPECT_NEAR(result.poses.back().x(), 3.0, 1e-9);
}

TEST(GridRefinement, BoundedStartConnectorDoesNotCrossBlockedSweep) {
  GridRefinementLimits bounded = limits();
  bounded.resolution_m = 0.25;
  bounded.detour_margin_m = 1.5;
  bounded.start_connector_max_distance_m = 1.5;
  auto project = [](StateVec& state) {
    return std::abs(state.x()) < 1e-9 || state.x() >= 1.0
               ? GridProjectionStatus::kSupported
               : GridProjectionStatus::kNoGround;
  };
  auto blocked_from_start = [](const StateVec& from, const StateVec& to,
                               std::vector<StateVec>& checked) {
    if (from.head<2>().norm() < 1e-9) return false;
    checked = {from, to};
    return true;
  };
  BoundedGridPlanner planner(StateVec::Zero(), bounded, project,
                             blocked_from_start);
  const FeasiblePath result = planner.refine(routeTo(3.0, 0.0));
  EXPECT_EQ(result.status, PlanningStatus::kBlocked);
}

TEST(GridRefinement, BoundedStartConnectorRequiresKnownSupportedEndpoint) {
  GridRefinementLimits bounded = limits();
  bounded.resolution_m = 0.25;
  bounded.detour_margin_m = 1.5;
  bounded.start_connector_max_distance_m = 1.5;
  auto project = [](StateVec& state) {
    return std::abs(state.x()) < 1e-9 || state.x() >= 3.0
               ? GridProjectionStatus::kSupported
               : GridProjectionStatus::kBodyUnknown;
  };
  auto short_only = [](const StateVec& from, const StateVec& to,
                       std::vector<StateVec>& checked) {
    if ((to.head<2>() - from.head<2>()).norm() > 0.3) return false;
    checked = {from, to};
    return true;
  };
  BoundedGridPlanner planner(StateVec::Zero(), bounded, project, short_only);
  const FeasiblePath result = planner.refine(routeTo(3.0, 0.0));
  EXPECT_EQ(result.status, PlanningStatus::kBlocked);
  EXPECT_NE(result.reason.find("unknown="), std::string::npos);
}

TEST(GridRefinement, RejectsUnrepresentableStartConnectorRadius) {
  GridRefinementLimits bounded = limits();
  bounded.start_connector_max_distance_m = 1e300;
  auto project = [](StateVec& state) {
    if (state.head<2>().norm() < 1e-9 ||
        (state.head<2>() - Eigen::Vector2d(3.0, 0.0)).norm() < 1e-9) {
      state.z() = 0.0;
      return GridProjectionStatus::kSupported;
    }
    return GridProjectionStatus::kNoGround;
  };
  auto blocked = [](const StateVec&, const StateVec&,
                    std::vector<StateVec>&) { return false; };
  BoundedGridPlanner planner(StateVec::Zero(), bounded, project, blocked);

  const FeasiblePath result = planner.refine(routeTo(3.0, 0.0));
  EXPECT_EQ(result.status, PlanningStatus::kBlocked);
  EXPECT_NE(result.reason.find("start connector bounds are invalid"),
            std::string::npos);
}

TEST(GridRefinement, StartConnectorCandidateWorkUsesSharedCellBudget) {
  GridRefinementLimits bounded = limits();
  bounded.detour_margin_m = 100.0;
  bounded.start_connector_max_distance_m = 50.0;
  bounded.max_cells = 32;
  bounded.max_expansions = 32;
  std::size_t projections = 0;
  auto project = [&projections](StateVec& state) {
    ++projections;
    state.z() = 0.0;
    return GridProjectionStatus::kSupported;
  };
  auto blocked = [](const StateVec&, const StateVec&,
                    std::vector<StateVec>&) { return false; };
  BoundedGridPlanner planner(StateVec::Zero(), bounded, project, blocked);

  const FeasiblePath result = planner.refine(routeTo(3.0, 0.0));
  EXPECT_EQ(result.status, PlanningStatus::kBlocked);
  EXPECT_EQ(result.reason.find("deadline"), std::string::npos);
  // Current, corridor and exact-goal projections are outside the cell store.
  EXPECT_LE(projections, bounded.max_cells + 3u);
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

TEST(GridRefinement, PartialRouteStopsAtProxyAndRetainsExactGoal) {
  RouteCorridor route = routeTo(30.0, 4.0);
  route.request.objective = mgg::ObjectiveKind::kNavigate;
  route.poses = {StateVec(0.0, 0.0, 0.0, 0.0),
                 StateVec(2.0, 0.0, 0.0, 0.25)};
  route.partial = true;
  BoundedGridPlanner planner(StateVec::Zero(), limits(), flatProjection(),
                             sampledTraversal({}));
  const FeasiblePath path = planner.refine(route);
  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  EXPECT_TRUE(path.partial);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().x(), 2.0, 1e-9);
  EXPECT_NEAR(path.poses.back().y(), 0.0, 1e-9);
  EXPECT_NEAR(path.poses.back()[3], 0.25, 1e-9);
  EXPECT_DOUBLE_EQ(route.request.goal.pose.x(), 30.0);
  EXPECT_DOUBLE_EQ(route.request.goal.pose.y(), 4.0);
}

TEST(GridRefinement, ProvisionalCellsRetainDistinctParentHeights) {
  RouteCorridor route = routeTo(2.0, 0.0);
  route.request.objective = mgg::ObjectiveKind::kNavigate;
  route.poses.clear();
  auto project = [](StateVec& state) {
    if (state.x() < 0.5 && state.y() > 0.5) {
      state.z() = 1.0;
      return GridProjectionStatus::kSupported;
    }
    if (state.x() >= 0.5) {
      state[3] = -1.2;
      return GridProjectionStatus::kProvisionalUnknown;
    }
    state.z() = 0.0;
    return GridProjectionStatus::kSupported;
  };
  auto traverse = [](const StateVec& from, const StateVec& to,
                     std::vector<StateVec>& checked) {
    const double xy = (to.head<2>() - from.head<2>()).norm();
    if (xy > std::sqrt(2.0) + 1e-9) return false;
    // The lower approach reaches (1, 0) first but cannot connect the raised
    // provisional destination. The upper approach must retain a distinct
    // height-qualified state for that same XY cell.
    if (to.x() > 1.5 && to.z() < 0.5) return false;
    const bool initial_known_ramp = from.head<2>().norm() < 1e-9 &&
                                    to.x() < 0.5 && to.y() > 0.5;
    if (!initial_known_ramp && std::abs(to.z() - from.z()) > 1e-9) {
      return false;
    }
    checked = {from, to};
    return true;
  };
  GridRefinementLimits bounded = limits();
  bounded.detour_margin_m = 1.0;
  BoundedGridPlanner planner(StateVec::Zero(), bounded, project, traverse);

  const FeasiblePath path = planner.refine(route);
  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  ASSERT_FALSE(path.poses.empty());
  EXPECT_TRUE(std::any_of(path.poses.begin(), path.poses.end(),
                          [](const StateVec& pose) {
                            return pose.y() > 0.5 && pose.z() > 0.5;
                          }));
  EXPECT_NEAR(path.poses.back().x(), 2.0, 1e-9);
  EXPECT_NEAR(path.poses.back().y(), 0.0, 1e-9);
  EXPECT_NEAR(path.poses.back().z(), 1.0, 1e-9);
  EXPECT_NEAR(path.poses.back()[3], 0.7, 1e-9);
}

TEST(GridRefinement, ProvisionalConnectorCannotMoveExactGoal) {
  RouteCorridor route = routeTo(2.0, 0.0);
  route.request.objective = mgg::ObjectiveKind::kNavigate;
  route.poses.clear();
  auto project = [](StateVec& state) {
    if (state.x() < 0.5 && state.y() > 0.5) {
      state.z() = 1.0;
      return GridProjectionStatus::kSupported;
    }
    if (state.x() >= 0.5) {
      // Only the high-parent retry attempts to move the requested XY.
      // Such a projection must not be accepted as the exact endpoint.
      if (state.x() > 1.5 && state.z() > 0.5) state.x() += 0.25;
      return GridProjectionStatus::kProvisionalUnknown;
    }
    state.z() = 0.0;
    return GridProjectionStatus::kSupported;
  };
  auto traverse = [](const StateVec& from, const StateVec& to,
                     std::vector<StateVec>& checked) {
    if ((to.head<2>() - from.head<2>()).norm() > std::sqrt(2.0) + 1e-9) {
      return false;
    }
    if (to.x() > 1.5 && to.z() < 0.5) return false;
    const bool initial_known_ramp = from.head<2>().norm() < 1e-9 &&
                                    to.x() < 0.5 && to.y() > 0.5;
    if (!initial_known_ramp && std::abs(to.z() - from.z()) > 1e-9) {
      return false;
    }
    checked = {from, to};
    return true;
  };
  GridRefinementLimits bounded = limits();
  bounded.detour_margin_m = 1.0;
  BoundedGridPlanner planner(StateVec::Zero(), bounded, project, traverse);

  const FeasiblePath path = planner.refine(route);
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_TRUE(path.poses.empty());
}

TEST(GridRefinement, ProvisionalEndpointRemainsInvalidForPartialNavigate) {
  RouteCorridor route = routeTo(2.0, 0.0);
  route.request.objective = mgg::ObjectiveKind::kNavigate;
  route.partial = true;
  auto project = [](StateVec& state) {
    return state.head<2>().norm() < 1e-9
               ? GridProjectionStatus::kSupported
               : GridProjectionStatus::kProvisionalUnknown;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), project,
                             sampledTraversal({}));

  const FeasiblePath path = planner.refine(route);
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_FALSE(path.partial);
  EXPECT_TRUE(path.poses.empty());
}

TEST(GridRefinement, PartialNavigateEndsAtTheLastSupportedCorridorPose) {
  // The horizon of a partial section ends on provisional terrain. The section
  // stops at the last corridor pose on measured ground instead of refusing.
  RouteCorridor route = routeTo(3.0, 0.0);
  route.request.objective = mgg::ObjectiveKind::kNavigate;
  route.partial = true;
  StateVec middle = StateVec::Zero();
  middle.x() = 1.0;
  StateVec horizon = StateVec::Zero();
  horizon.x() = 2.0;
  route.poses = {middle, horizon};
  auto project = [](StateVec& state) {
    return state.x() < 1.5 ? GridProjectionStatus::kSupported
                           : GridProjectionStatus::kProvisionalUnknown;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), project,
                             sampledTraversal({}));

  const FeasiblePath path = planner.refine(route);
  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  EXPECT_TRUE(path.partial);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().x(), 1.0, 1e-9);
}

TEST(GridRefinement, ProvisionalEndpointRemainsInvalidForReturnHome) {
  auto project = [](StateVec& state) {
    return state.head<2>().norm() < 1e-9
               ? GridProjectionStatus::kSupported
               : GridProjectionStatus::kProvisionalUnknown;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), project,
                             sampledTraversal({}));

  const FeasiblePath path = planner.refine(routeTo(2.0, 0.0));
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_FALSE(path.partial);
  EXPECT_TRUE(path.poses.empty());
}

TEST(GridRefinement, FailedPartialRouteCannotAdvertiseContinuation) {
  RouteCorridor route = routeTo(30.0, 0.0);
  route.request.objective = mgg::ObjectiveKind::kNavigate;
  route.poses.clear();
  route.partial = true;
  BoundedGridPlanner planner(StateVec::Zero(), limits(), flatProjection(),
                             sampledTraversal({}));
  const FeasiblePath path = planner.refine(route);
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_FALSE(path.partial);
  EXPECT_TRUE(path.poses.empty());
}

TEST(GridRefinement, ProjectorCannotReplaceRequestOwnedGoalYaw) {
  auto yaw_mutating_projector = [](StateVec& state) {
    state.z() = 0.0;
    state[3] = -2.4;
    return GridProjectionStatus::kSupported;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(),
                             yaw_mutating_projector, sampledTraversal({}));
  const FeasiblePath path = planner.refine(routeTo(2.0, 0.0));
  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back()[3], 0.7, 1e-9);
}

TEST(GridRefinement, RefinesExploreThroughTheSharedTerrainStage) {
  // Explore now shares one grid stage with Navigate and ReturnHome, so its
  // selected target is projected and its corridor swept like any other.
  RouteCorridor route = routeTo(1.0, 0.0);
  route.request.objective = mgg::ObjectiveKind::kExplore;
  int projections = 0;
  auto project = [&projections](StateVec& state) {
    ++projections;
    state.z() = 0.0;
    return GridProjectionStatus::kSupported;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), project,
                             sampledTraversal({}));
  const FeasiblePath path = planner.refine(route);
  EXPECT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  EXPECT_GT(projections, 0);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().x(), 1.0, 1e-9);
  EXPECT_FALSE(path.blocked_segment_identified);
}

TEST(GridRefinement, ExploreDetoursAroundAnObstacleLikeExplicitObjectives) {
  RouteCorridor route = routeTo(3.0, 0.0);
  route.request.objective = mgg::ObjectiveKind::kExplore;
  BoundedGridPlanner planner(StateVec::Zero(), limits(), flatProjection(),
                             sampledTraversal({{2.0, 0.0}}));
  const FeasiblePath path = planner.refine(route);
  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().x(), 3.0, 1e-9);
  for (const StateVec& pose : path.poses) {
    EXPECT_GT((pose.head<2>() - Eigen::Vector2d(2.0, 0.0)).norm(), 0.34);
  }
}

TEST(GridRefinement, RejectsObjectivesWithNoGridStageBeforeMapPredicates) {
  RouteCorridor route = routeTo(1.0, 0.0);
  route.request.objective = mgg::ObjectiveKind::kInspect;
  int projections = 0;
  auto project = [&projections](StateVec&) {
    ++projections;
    return GridProjectionStatus::kSupported;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), project,
                             sampledTraversal({}));
  const FeasiblePath path = planner.refine(route);
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_EQ(projections, 0);
  EXPECT_FALSE(path.blocked_segment_identified);
}

TEST(GridRefinement, NamesTheRejectedCorridorWaypointForBlockedFeedback) {
  RouteCorridor route;
  route.status = PlanningStatus::kSucceeded;
  route.request.objective = mgg::ObjectiveKind::kReturnHome;
  route.poses.push_back(StateVec(1.0, 0.0, 0.0, 0.0));
  route.poses.push_back(StateVec(2.0, 0.0, 0.0, 0.0));
  route.request.goal.pose = route.poses.back();
  auto project = [](StateVec& state) {
    state.z() = 0.0;
    return std::abs(state.x() - 2.0) < 1e-9 ? GridProjectionStatus::kNoGround
                                            : GridProjectionStatus::kSupported;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), project,
                             sampledTraversal({}));
  const FeasiblePath path = planner.refine(route);
  ASSERT_EQ(path.status, PlanningStatus::kBlocked);
  ASSERT_TRUE(path.blocked_segment_identified);
  EXPECT_EQ(path.blocked_from_index, 0u);
  EXPECT_EQ(path.blocked_to_index, 1u);
}

TEST(GridRefinement, NamesTheCorridorSegmentWithNoTraversableDetour) {
  RouteCorridor route;
  route.status = PlanningStatus::kSucceeded;
  route.request.objective = mgg::ObjectiveKind::kExplore;
  route.poses.push_back(StateVec(1.0, 0.0, 0.0, 0.0));
  route.poses.push_back(StateVec(4.0, 0.0, 0.0, 0.0));
  route.request.goal.pose = route.poses.back();
  // A wall across the whole detour window between the two corridor poses.
  auto walled = [](const StateVec& from, const StateVec& to,
                   std::vector<StateVec>& checked) {
    const double length = (to.head<2>() - from.head<2>()).norm();
    const int steps = std::max(1, static_cast<int>(std::ceil(length / 0.1)));
    checked.clear();
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      StateVec sample = from + t * (to - from);
      if (sample.x() > 1.9 && sample.x() < 2.6) return false;
      checked.push_back(sample);
    }
    return true;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), flatProjection(),
                             walled);
  const FeasiblePath path = planner.refine(route);
  ASSERT_EQ(path.status, PlanningStatus::kBlocked) << path.reason;
  ASSERT_TRUE(path.blocked_segment_identified);
  EXPECT_EQ(path.blocked_from_index, 0u);
  EXPECT_EQ(path.blocked_to_index, 1u);
}

TEST(GridRefinement, ADeadlineNeverBlamesTheCorridorItDidNotFinishChecking) {
  GridRefinementLimits tight = limits();
  tight.timeout = std::chrono::milliseconds(1);
  RouteCorridor route;
  route.status = PlanningStatus::kSucceeded;
  route.request.objective = mgg::ObjectiveKind::kExplore;
  route.poses.push_back(StateVec(1.0, 0.0, 0.0, 0.0));
  route.poses.push_back(StateVec(2.0, 0.0, 0.0, 0.0));
  route.request.goal.pose = route.poses.back();
  auto slow = [](StateVec& state) {
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    state.z() = 0.0;
    return GridProjectionStatus::kSupported;
  };
  BoundedGridPlanner planner(StateVec::Zero(), tight, slow,
                             sampledTraversal({}));
  const FeasiblePath path = planner.refine(route);
  ASSERT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_NE(path.reason.find("cooperative deadline"), std::string::npos)
      << path.reason;
  EXPECT_FALSE(path.blocked_segment_identified);
}

TEST(GridRefinement, PartialCorridorNamesTheSegmentEndingAtItsProxy) {
  RouteCorridor route;
  route.status = PlanningStatus::kSucceeded;
  route.partial = true;
  route.request.objective = mgg::ObjectiveKind::kNavigate;
  route.poses.push_back(StateVec(1.0, 0.0, 0.0, 0.0));
  route.poses.push_back(StateVec(2.0, 0.0, 0.0, 0.0));
  route.request.goal.pose = StateVec(30.0, 0.0, 0.0, 0.0);
  auto blocked_after_first = [](const StateVec& from, const StateVec& to,
                               std::vector<StateVec>& checked) {
    if (from.x() >= 1.0 - 1e-9 || to.x() >= 1.5) return false;
    checked = {from, to};
    return true;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), flatProjection(),
                             blocked_after_first);
  const FeasiblePath path = planner.refine(route);
  ASSERT_EQ(path.status, PlanningStatus::kBlocked);
  ASSERT_TRUE(path.blocked_segment_identified);
  EXPECT_EQ(path.blocked_to_index, 1u);
}

TEST(GridRefinement, ReportsWaypointIndexCoordinatesAndProjectionClass) {
  const std::vector<std::pair<GridProjectionStatus, std::string>> failures = {
      {GridProjectionStatus::kNoGround, "no mapped ground support"},
      {GridProjectionStatus::kBodyOccupied,
       "body intersects occupied space"},
      {GridProjectionStatus::kBodyUnknown, "body includes unknown space"},
      {GridProjectionStatus::kGeofenceViolation, "geofence violation"},
  };
  for (const auto& [status, expected_class] : failures) {
    SCOPED_TRACE(expected_class);
    // A single-waypoint corridor: once its only pose is rejected nothing of
    // the corridor survives, so the request still fails and names that pose.
    RouteCorridor route = routeTo(2.0, 0.0);
    route.poses = {StateVec(1.0, -0.5, 0.25, 0.0)};
    auto project = [status](StateVec& state) {
      return state.x() == 1.0 ? status : GridProjectionStatus::kSupported;
    };
    BoundedGridPlanner planner(StateVec::Zero(), limits(), project,
                               sampledTraversal({}));
    const FeasiblePath path = planner.refine(route);
    EXPECT_EQ(path.status, PlanningStatus::kBlocked);
    EXPECT_NE(path.reason.find("route corridor waypoint[0] rejected"),
              std::string::npos)
        << path.reason;
    EXPECT_NE(path.reason.find(expected_class), std::string::npos);
    EXPECT_NE(path.reason.find("(1.00, -0.50, 0.25)"), std::string::npos);
    EXPECT_TRUE(path.blocked_segment_identified);
    EXPECT_EQ(path.blocked_from_index, mgg::kNoCorridorIndex);
    EXPECT_EQ(path.blocked_to_index, 0u);
  }
}

TEST(GridRefinement, DropsARejectedMiddleWaypointAndRefinesAroundIt) {
  // The middle graph waypoint stands on a post beside the robot's earlier
  // track. It is a refinement hint, not a requirement: the bounded search
  // connects its neighbours around the post instead of refusing the route.
  RouteCorridor route = routeTo(3.0, 0.0);
  route.poses = {StateVec(1.0, 0.0, 0.0, 0.0), StateVec(2.0, 0.0, 0.0, 0.0),
                 StateVec(3.0, 0.0, 0.0, 0.0)};
  auto project = [](StateVec& state) {
    state.z() = 0.0;
    return (state.head<2>() - Eigen::Vector2d(2.0, 0.0)).norm() < 1e-9
               ? GridProjectionStatus::kBodyOccupied
               : GridProjectionStatus::kSupported;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), project,
                             sampledTraversal({{2.0, 0.0}}));
  const FeasiblePath path = planner.refine(route);
  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  EXPECT_TRUE(path.reason.empty()) << path.reason;
  EXPECT_FALSE(path.blocked_segment_identified);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().x(), 3.0, 1e-9);
  EXPECT_NEAR(path.poses.back().y(), 0.0, 1e-9);
  EXPECT_NEAR(path.poses.back()[3], 0.7, 1e-9);
  for (const StateVec& pose : path.poses) {
    EXPECT_GT((pose.head<2>() - Eigen::Vector2d(2.0, 0.0)).norm(), 0.34);
  }
}

TEST(GridRefinement, DropsARejectedLastWaypointWhenTheExactGoalIsSupported) {
  // A full corridor's endpoint is the request-owned exact goal, which is
  // projected on its own. The last graph pose is only a hint like the rest.
  RouteCorridor route = routeTo(3.0, 0.0);
  route.poses = {StateVec(1.0, 0.0, 0.0, 0.0), StateVec(2.0, 0.5, 0.0, 0.0)};
  auto project = [](StateVec& state) {
    state.z() = 0.0;
    return (state.head<2>() - Eigen::Vector2d(2.0, 0.5)).norm() < 1e-9
               ? GridProjectionStatus::kNoGround
               : GridProjectionStatus::kSupported;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), project,
                             sampledTraversal({}));
  const FeasiblePath path = planner.refine(route);
  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  EXPECT_TRUE(path.reason.empty()) << path.reason;
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().x(), 3.0, 1e-9);
  EXPECT_NEAR(path.poses.back().y(), 0.0, 1e-9);
  for (const StateVec& pose : path.poses) {
    EXPECT_GT((pose.head<2>() - Eigen::Vector2d(2.0, 0.5)).norm(), 1e-3);
  }
}

TEST(GridRefinement, PartialCorridorWithOnlyRejectedWaypointsFails) {
  RouteCorridor route = routeTo(30.0, 0.0);
  route.request.objective = mgg::ObjectiveKind::kNavigate;
  route.partial = true;
  route.poses = {StateVec(1.0, 0.0, 0.0, 0.0), StateVec(2.0, 0.0, 0.0, 0.0)};
  auto project = [](StateVec& state) {
    state.z() = 0.0;
    return state.x() >= 1.0 ? GridProjectionStatus::kNoGround
                            : GridProjectionStatus::kSupported;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), project,
                             sampledTraversal({}));
  const FeasiblePath path = planner.refine(route);
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_FALSE(path.partial);
  EXPECT_TRUE(path.poses.empty());
  // The proxy is the pose the continuation would have resumed from and is
  // named itself; the earlier hint it could not fall back on is listed too.
  EXPECT_NE(path.reason.find("route corridor waypoint[1] rejected"),
            std::string::npos)
      << path.reason;
  EXPECT_NE(path.reason.find("rejected waypoints: 0"),
            std::string::npos)
      << path.reason;

  // A partial corridor reduced to its proxy alone fails the same way.
  route.poses = {StateVec(2.0, 0.0, 0.0, 0.0)};
  const FeasiblePath proxy_only = planner.refine(route);
  EXPECT_EQ(proxy_only.status, PlanningStatus::kBlocked);
  EXPECT_NE(proxy_only.reason.find("route corridor waypoint[0] rejected"),
            std::string::npos)
      << proxy_only.reason;
}

TEST(GridRefinement, RejectedProxyOfAPartialCorridorEndsAtTheLastSupportedPose) {
  // The continuation resumes from wherever the section ends, so a proxy on
  // an obstacle shortens the section to the surviving hint before it.
  RouteCorridor route = routeTo(30.0, 0.0);
  route.request.objective = mgg::ObjectiveKind::kNavigate;
  route.partial = true;
  route.poses = {StateVec(1.0, 0.0, 0.0, 0.0), StateVec(2.0, 0.0, 0.0, 0.0)};
  auto project = [](StateVec& state) {
    state.z() = 0.0;
    return std::abs(state.x() - 2.0) < 1e-9 ? GridProjectionStatus::kNoGround
                                            : GridProjectionStatus::kSupported;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), project,
                             sampledTraversal({}));
  const FeasiblePath path = planner.refine(route);
  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  EXPECT_TRUE(path.partial);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().x(), 1.0, 1e-9);
  EXPECT_TRUE(path.reason.empty());

  // With no supported pose left before it, the rejected proxy is named.
  RouteCorridor only_proxy = route;
  only_proxy.poses = {StateVec(2.0, 0.0, 0.0, 0.0)};
  const FeasiblePath refused = planner.refine(only_proxy);
  EXPECT_EQ(refused.status, PlanningStatus::kBlocked);
  EXPECT_NE(refused.reason.find("route corridor waypoint[0] rejected"),
            std::string::npos)
      << refused.reason;
  ASSERT_TRUE(refused.blocked_segment_identified);
  EXPECT_EQ(refused.blocked_to_index, 0u);
}

TEST(GridRefinement, NamesTheDroppedWaypointWhenItsSpanCannotBeRefined) {
  // Waypoint 1 is dropped, and the span 0->2 that replaces it meets a wall
  // across the whole window. Blocked feedback keys on adjacent corridor
  // poses, so the segment named is the one leading into the dropped pose.
  RouteCorridor route;
  route.status = PlanningStatus::kSucceeded;
  route.request.objective = mgg::ObjectiveKind::kReturnHome;
  route.poses = {StateVec(1.0, 0.0, 0.0, 0.0), StateVec(2.0, 0.0, 0.0, 0.0),
                 StateVec(4.0, 0.0, 0.0, 0.0)};
  route.request.goal.pose = route.poses.back();
  auto project = [](StateVec& state) {
    state.z() = 0.0;
    return (state.head<2>() - Eigen::Vector2d(2.0, 0.0)).norm() < 1e-9
               ? GridProjectionStatus::kBodyOccupied
               : GridProjectionStatus::kSupported;
  };
  auto walled = [](const StateVec& from, const StateVec& to,
                   std::vector<StateVec>& checked) {
    const double length = (to.head<2>() - from.head<2>()).norm();
    const int steps = std::max(1, static_cast<int>(std::ceil(length / 0.1)));
    checked.clear();
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      StateVec sample = from + t * (to - from);
      if (sample.x() > 2.9 && sample.x() < 3.6) return false;
      checked.push_back(sample);
    }
    return true;
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), project, walled);
  const FeasiblePath path = planner.refine(route);
  ASSERT_EQ(path.status, PlanningStatus::kBlocked) << path.reason;
  EXPECT_NE(path.reason.find("rejected waypoints: 1"),
            std::string::npos)
      << path.reason;
  ASSERT_TRUE(path.blocked_segment_identified);
  EXPECT_EQ(path.blocked_from_index, 0u);
  EXPECT_EQ(path.blocked_to_index, 1u);
}

TEST(GridRefinement, IdentifiesCurrentAndExactGoalProjectionFailures) {
  auto current_failure = [](StateVec& state) {
    return state.x() == 0.0 ? GridProjectionStatus::kBodyUnknown
                            : GridProjectionStatus::kSupported;
  };
  BoundedGridPlanner current_planner(StateVec::Zero(), limits(),
                                     current_failure, sampledTraversal({}));
  const FeasiblePath current = current_planner.refine(routeTo(2.0, 0.0));
  EXPECT_NE(current.reason.find("current pose rejected: body includes unknown "
                                "space at (0.00, 0.00, 0.00)"),
            std::string::npos);

  RouteCorridor route = routeTo(2.25, -0.25);
  route.poses = {StateVec(1.0, 0.0, 0.0, 0.0)};
  auto goal_failure = [](StateVec& state) {
    return state.x() == 2.25 ? GridProjectionStatus::kGeofenceViolation
                             : GridProjectionStatus::kSupported;
  };
  BoundedGridPlanner goal_planner(StateVec::Zero(), limits(), goal_failure,
                                  sampledTraversal({}));
  const FeasiblePath goal = goal_planner.refine(route);
  EXPECT_NE(goal.reason.find("exact goal rejected: geofence violation at "
                             "(2.25, -0.25, 0.00)"),
            std::string::npos);
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

TEST(GridRefinement, MemoizesRepeatedDirectedTerrainChecksDuringDetour) {
  std::map<std::array<double, 8>, std::size_t> calls;
  auto checked = sampledTraversal({Eigen::Vector2d(1.0, 0.0)});
  auto counted = [&calls, checked](const StateVec& from, const StateVec& to,
                                  std::vector<StateVec>& path) mutable {
    std::array<double, 8> key{};
    for (Eigen::Index i = 0; i < 4; ++i) {
      key[static_cast<std::size_t>(i)] = from[i];
      key[static_cast<std::size_t>(i + 4)] = to[i];
    }
    ++calls[key];
    return checked(from, to, path);
  };
  BoundedGridPlanner planner(StateVec::Zero(), limits(), flatProjection(),
                             counted);

  const FeasiblePath path = planner.refine(routeTo(2.0, 0.0));

  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  EXPECT_NEAR(path.poses.back().x(), 2.0, 1e-9);
  ASSERT_FALSE(calls.empty());
  EXPECT_TRUE(std::all_of(calls.begin(), calls.end(), [](const auto& call) {
    return call.second == 1u;
  }));
}

TEST(GridRefinement, TraversalMemoizationPreservesDirection) {
  GridRefinementLimits bounded = limits();
  bounded.max_expansions = 1;
  std::size_t exact_forward_calls = 0;
  std::size_t exact_reverse_calls = 0;
  auto directional = [&exact_forward_calls, &exact_reverse_calls](
                         const StateVec& from, const StateVec& to,
                         std::vector<StateVec>& checked) {
    if (std::abs(from.x()) < 1e-9 && std::abs(to.x() - 1.0) < 1e-9 &&
        std::abs(from.y()) < 1e-9 && std::abs(to.y()) < 1e-9) {
      ++exact_forward_calls;
    }
    if (std::abs(from.x() - 1.0) < 1e-9 && std::abs(to.x()) < 1e-9 &&
        std::abs(from.y()) < 1e-9 && std::abs(to.y()) < 1e-9) {
      ++exact_reverse_calls;
    }
    if (from.x() < to.x()) {
      checked = {from, to};
      return true;
    }
    checked.clear();
    return false;
  };
  RouteCorridor route = routeTo(0.0, 0.0);
  route.poses = {StateVec(1.0, 0.0, 0.0, 0.0),
                 StateVec(0.0, 0.0, 0.0, 0.0)};
  BoundedGridPlanner planner(StateVec::Zero(), bounded, flatProjection(),
                             directional);

  const FeasiblePath path = planner.refine(route);

  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_EQ(exact_forward_calls, 1u);
  EXPECT_GE(exact_reverse_calls, 1u);
}

TEST(GridRefinement, GoalBiasedSearchBoundsMapQueriesOnWideDetourGrid) {
  GridRefinementLimits bounded = limits();
  bounded.resolution_m = 0.25;
  bounded.detour_margin_m = 4.0;
  bounded.max_cells = 8192;
  bounded.max_expansions = 2048;
  bounded.timeout = std::chrono::milliseconds(500);
  std::size_t traversals = 0;
  auto checked = sampledTraversal({Eigen::Vector2d(10.0, 0.0)});
  auto counted = [&traversals, checked](const StateVec& from,
                                       const StateVec& to,
                                       std::vector<StateVec>& path) mutable {
    ++traversals;
    return checked(from, to, path);
  };
  BoundedGridPlanner planner(StateVec::Zero(), bounded, flatProjection(),
                             counted);
  const FeasiblePath path = planner.refine(routeTo(20.0, 0.0));
  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  EXPECT_LT(traversals, 1500u);
  EXPECT_NEAR(path.poses.back().x(), 20.0, 1e-9);
}

TEST(GridRefinement, MemoizesRepeatedKnownIncomingHeightKeys) {
  GridRefinementLimits bounded = limits();
  bounded.resolution_m = 0.25;
  bounded.detour_margin_m = 4.0;
  bounded.max_cells = 8192;
  bounded.max_expansions = 2048;
  bounded.timeout = std::chrono::milliseconds(500);
  std::map<std::array<double, 3>, std::size_t> projection_calls;
  auto ramp = [&projection_calls](StateVec& state) {
    // Endpoint qualification happens outside the cell store. Count only
    // interior lattice projections so repeated calls identify ensureCell.
    if (state.x() > 0.5 && state.x() < 19.5) {
      ++projection_calls[{state.x(), state.y(), state.z()}];
    }
    state.z() = 0.025 * state.x() + 0.010 * state.y();
    return GridProjectionStatus::kSupported;
  };
  BoundedGridPlanner planner(
      StateVec::Zero(), bounded, ramp,
      sampledTraversal({Eigen::Vector2d(10.0, -1.0)}));

  const FeasiblePath path = planner.refine(routeTo(20.0, -2.0));

  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  ASSERT_FALSE(projection_calls.empty());
  EXPECT_TRUE(std::all_of(
      projection_calls.begin(), projection_calls.end(), [](const auto& call) {
        return call.second == 1u;
      }));
}

TEST(GridRefinement, FullKnownIncomingMemoFallsBackToProjection) {
  GridRefinementLimits bounded = limits();
  bounded.resolution_m = 0.25;
  bounded.detour_margin_m = 4.0;
  bounded.max_cells = 300;
  bounded.max_expansions = 2048;
  bounded.timeout = std::chrono::milliseconds(500);
  std::size_t projections = 0;
  std::map<std::array<double, 3>, std::size_t> projection_calls;
  auto ramp = [&projections, &projection_calls](StateVec& state) {
    ++projections;
    if (state.x() > 0.5 && state.x() < 19.5) {
      ++projection_calls[{state.x(), state.y(), state.z()}];
    }
    state.z() = 0.025 * state.x() + 0.010 * state.y();
    return GridProjectionStatus::kSupported;
  };
  BoundedGridPlanner planner(
      StateVec::Zero(), bounded, ramp,
      sampledTraversal({Eigen::Vector2d(10.0, -1.0)}));

  const FeasiblePath path = planner.refine(routeTo(20.0, -2.0));

  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  EXPECT_NEAR(path.poses.back().x(), 20.0, 1e-9);
  EXPECT_NEAR(path.poses.back().y(), -2.0, 1e-9);
  EXPECT_GT(projections, bounded.max_cells);
  EXPECT_TRUE(std::any_of(
      projection_calls.begin(), projection_calls.end(), [](const auto& call) {
        return call.second > 1u;
      }));
}

TEST(GridRefinement, SparseSearchReachesFarDiagonalBeyondDenseAreaLimit) {
  GridRefinementLimits bounded = limits();
  bounded.resolution_m = 0.5;
  bounded.detour_margin_m = 2.0;
  bounded.max_cells = 1024;
  bounded.max_expansions = 1024;
  bounded.timeout = std::chrono::milliseconds(500);
  BoundedGridPlanner planner(
      StateVec::Zero(), bounded, flatProjection(),
      sampledTraversal({Eigen::Vector2d(25.0, 25.0)}));

  const FeasiblePath path = planner.refine(routeTo(50.0, 50.0));

  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  EXPECT_NEAR(path.poses.back().x(), 50.0, 1e-9);
  EXPECT_NEAR(path.poses.back().y(), 50.0, 1e-9);
  EXPECT_TRUE(std::any_of(path.poses.begin(), path.poses.end(),
                          [](const StateVec& pose) {
                            return std::abs(pose.x() - pose.y()) > 0.5;
                          }));
}

TEST(GridRefinement, UnknownCellsCannotFormADetour) {
  const std::set<std::pair<int, int>> known{{0, 0}, {1, 0}, {2, 0}};
  auto project = [known](StateVec& state) {
    return known.count({static_cast<int>(std::lround(state.x())),
                        static_cast<int>(std::lround(state.y()))}) != 0
               ? GridProjectionStatus::kSupported
               : GridProjectionStatus::kBodyUnknown;
  };
  BoundedGridPlanner planner(
      StateVec::Zero(), limits(), project,
      sampledTraversal({Eigen::Vector2d(1.0, 0.0)}));
  const FeasiblePath path = planner.refine(routeTo(2.0, 0.0));
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_TRUE(path.poses.empty());
  EXPECT_NE(path.reason.find("no observed traversable"), std::string::npos);
  EXPECT_NE(path.reason.find("grid evidence: projections="), std::string::npos);
  EXPECT_NE(path.reason.find("unknown="), std::string::npos);
  EXPECT_NE(path.reason.find("expansions="), std::string::npos);
}

TEST(GridRefinement, DoesNotCutDiagonallyBetweenBlockedCardinalCells) {
  const std::set<std::pair<int, int>> known{{0, 0}, {1, 1}, {2, 2}};
  auto project = [known](StateVec& state) {
    return known.count({static_cast<int>(std::lround(state.x())),
                        static_cast<int>(std::lround(state.y()))}) != 0
               ? GridProjectionStatus::kSupported
               : GridProjectionStatus::kBodyUnknown;
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
    return GridProjectionStatus::kSupported;
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

TEST(GridRefinement, CompletesLongKnownRoadBeyondLocalGraphExtent) {
  GridRefinementLimits value = limits();
  value.detour_margin_m = 2.0;
  value.max_cells = 4096;
  value.max_expansions = 4096;
  RouteCorridor route = routeTo(30.0, 0.0);
  route.request.objective = mgg::ObjectiveKind::kNavigate;
  route.poses = {StateVec(3.0, 0.0, 0.0, 0.0)};
  BoundedGridPlanner planner(StateVec::Zero(), value, flatProjection(),
                             sampledTraversal({}));
  const FeasiblePath path = planner.refine(route);
  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  ASSERT_FALSE(path.partial);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().x(), 30.0, 1e-9);
}

TEST(GridRefinement, LongKnownRoadDetoursAndStillReachesExactGoal) {
  GridRefinementLimits value = limits();
  value.detour_margin_m = 2.0;
  value.max_cells = 4096;
  value.max_expansions = 4096;
  RouteCorridor route = routeTo(30.0, 0.0);
  route.request.objective = mgg::ObjectiveKind::kNavigate;
  BoundedGridPlanner planner(
      StateVec::Zero(), value, flatProjection(),
      sampledTraversal({Eigen::Vector2d(15.0, 0.0)}));
  const FeasiblePath path = planner.refine(route);
  ASSERT_EQ(path.status, PlanningStatus::kSucceeded) << path.reason;
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().x(), 30.0, 1e-9);
  EXPECT_TRUE(std::any_of(path.poses.begin(), path.poses.end(),
                          [](const StateVec& pose) {
                            return std::abs(pose.y()) > 0.5;
                          }));
}

TEST(GridRefinement, LongUnknownGapFailsInsteadOfReturningPrefix) {
  GridRefinementLimits value = limits();
  value.detour_margin_m = 2.0;
  value.max_cells = 4096;
  value.max_expansions = 4096;
  RouteCorridor route = routeTo(30.0, 0.0);
  route.request.objective = mgg::ObjectiveKind::kNavigate;
  route.poses = {StateVec(3.0, 0.0, 0.0, 0.0)};
  auto known_ends = [](StateVec& state) {
    state.z() = 0.0;
    return (state.x() <= 4.0 || state.x() >= 29.0)
               ? GridProjectionStatus::kSupported
               : GridProjectionStatus::kBodyUnknown;
  };
  BoundedGridPlanner planner(StateVec::Zero(), value, known_ends,
                             sampledTraversal({Eigen::Vector2d(15.0, 0.0)}));
  const FeasiblePath path = planner.refine(route);
  EXPECT_EQ(path.status, PlanningStatus::kBlocked);
  EXPECT_FALSE(path.partial);
  EXPECT_TRUE(path.poses.empty());
}
