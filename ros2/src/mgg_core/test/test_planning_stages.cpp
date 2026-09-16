#include <gtest/gtest.h>

#include <limits>

#include "mgg_core/planning_stages.h"

namespace {

using namespace mgg;

class Chain {
 public:
  Chain() {
    Vertex* previous = nullptr;
    for (int i = 0; i < 4; ++i) {
      auto* vertex = new Vertex(i, StateVec(i, 0.0, 0.0, 0.0));
      graph.addVertex(vertex);
      if (previous != nullptr) graph.addEdge(previous, vertex, 1.0);
      previous = vertex;
    }
  }
  GraphManager graph;
};

PlanningRequest request(ObjectiveKind objective) {
  PlanningRequest value;
  value.mission_id = "mission-7";
  value.objective = objective;
  value.goal.pose = StateVec(3.1, 0.0, 0.0, 0.0);
  value.component_id = "component-a";
  value.graph_revision = 12;
  value.map_revision = 34;
  return value;
}

TEST(PlanningStages, NavigateAndReturnHomeShareTheGraphStage) {
  for (const auto objective :
       {ObjectiveKind::kNavigate, ObjectiveKind::kReturnHome}) {
    Chain chain;
    TopologicalGoalPlanner planner("component-a", 12, 34, 0.25);
    const auto route =
        planner.plan(chain.graph, StateVec(0.1, 0.0, 0.0, 0.0),
                     request(objective));
    ASSERT_EQ(route.status, PlanningStatus::kSucceeded);
    ASSERT_EQ(route.poses.size(), 4u);
    EXPECT_DOUBLE_EQ(route.poses.front().x(), 0.0);
    EXPECT_DOUBLE_EQ(route.poses.back().x(), 3.0);
    // The graph corridor does not silently replace the requested exact goal.
    EXPECT_DOUBLE_EQ(route.request.goal.pose.x(), 3.1);
    EXPECT_FALSE(route.partial);
  }
}

TEST(PlanningStages, FarNavigateUsesPersistentPrefixForExactCompletion) {
  Chain chain;
  TopologicalGoalPlanner planner("component-a", 12, 34, 0.25, 1.0);
  auto far = request(ObjectiveKind::kNavigate);
  far.goal.pose = StateVec(30.0, 4.0, 0.0, 1.2);
  const auto route = planner.plan(
      chain.graph, StateVec(0.1, 0.0, 0.0, 0.0), far);
  ASSERT_EQ(route.status, PlanningStatus::kSucceeded) << route.reason;
  ASSERT_FALSE(route.partial);
  ASSERT_EQ(route.poses.size(), 4u);
  EXPECT_DOUBLE_EQ(route.poses[2].x(), 2.0);
  EXPECT_DOUBLE_EQ(route.poses.back().x(), 30.0);
  EXPECT_DOUBLE_EQ(route.poses.back().y(), 4.0);
  // The final edge is an optimistic global connector. Rolling grid refinement
  // validates it in bounded local windows without changing the destination.
  EXPECT_DOUBLE_EQ(route.request.goal.pose.x(), 30.0);
  EXPECT_DOUBLE_EQ(route.request.goal.pose.y(), 4.0);
  EXPECT_DOUBLE_EQ(route.request.goal.pose[3], 1.2);
}

TEST(PlanningStages, FarNavigateWithoutHelpfulPrefixKeepsTentativeGraphRoute) {
  Chain chain;
  TopologicalGoalPlanner planner("component-a", 12, 34, 0.25, 1.0);
  auto far = request(ObjectiveKind::kNavigate);
  far.goal.pose = StateVec(-30.0, 0.0, 0.0, 0.0);
  const auto route = planner.plan(chain.graph, StateVec::Zero(), far);
  EXPECT_EQ(route.status, PlanningStatus::kSucceeded) << route.reason;
  EXPECT_FALSE(route.partial);
  ASSERT_EQ(route.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(route.poses.front().x(), 0.0);
  EXPECT_DOUBLE_EQ(route.poses.back().x(), -30.0);
}

TEST(PlanningStages, PartialProgressMustMeetTheConfiguredMinimum) {
  Chain chain;
  TopologicalGoalPlanner planner("component-a", 12, 34, 0.25, 3.5);
  auto far = request(ObjectiveKind::kNavigate);
  far.goal.pose = StateVec(30.0, 0.0, 0.0, 0.0);
  const auto route = planner.plan(chain.graph, StateVec::Zero(), far);
  EXPECT_EQ(route.status, PlanningStatus::kSucceeded) << route.reason;
  EXPECT_FALSE(route.partial);
  ASSERT_EQ(route.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(route.poses.front().x(), 0.0);
  EXPECT_DOUBLE_EQ(route.poses.back().x(), 30.0);
}

TEST(PlanningStages, DisconnectedCloserProxyIsNotSelected) {
  Chain chain;
  auto* disconnected = new Vertex(4, StateVec(29.0, 0.0, 0.0, 0.0));
  chain.graph.addVertex(disconnected);
  TopologicalGoalPlanner planner("component-a", 12, 34, 0.25, 1.0);
  auto far = request(ObjectiveKind::kNavigate);
  far.goal.pose = StateVec(30.0, 0.0, 0.0, 0.0);
  const auto route = planner.plan(chain.graph, StateVec::Zero(), far);
  ASSERT_EQ(route.status, PlanningStatus::kSucceeded) << route.reason;
  ASSERT_FALSE(route.partial);
  ASSERT_FALSE(route.poses.empty());
  ASSERT_EQ(route.poses.size(), 5u);
  EXPECT_DOUBLE_EQ(route.poses[3].x(), 3.0);
  EXPECT_DOUBLE_EQ(route.poses.back().x(), 30.0);
}

TEST(PlanningStages, TentativeConnectorMinimizesGraphPlusUnknownCost) {
  GraphManager graph;
  auto* source = new Vertex(0, StateVec(0.0, 0.0, 0.0, 0.0));
  auto* cheap = new Vertex(1, StateVec(4.0, 0.0, 0.0, 0.0));
  auto* expensive = new Vertex(2, StateVec(9.0, 0.0, 0.0, 0.0));
  graph.addVertex(source);
  graph.addVertex(cheap);
  graph.addVertex(expensive);
  graph.addEdge(source, cheap, 4.0);
  graph.addEdge(source, expensive, 20.0);
  TopologicalGoalPlanner planner("component-a", 12, 34, 0.25, 1.0);
  auto far = request(ObjectiveKind::kNavigate);
  far.goal.pose = StateVec(10.0, 0.0, 0.0, 0.0);

  const auto route = planner.plan(graph, StateVec::Zero(), far);
  ASSERT_EQ(route.status, PlanningStatus::kSucceeded) << route.reason;
  ASSERT_EQ(route.poses.size(), 3u);
  EXPECT_DOUBLE_EQ(route.poses[1].x(), 4.0);
  EXPECT_DOUBLE_EQ(route.poses.back().x(), 10.0);
}

TEST(PlanningStages, StaleMapOrGraphSnapshotIsRejected) {
  Chain chain;
  TopologicalGoalPlanner planner("component-a", 12, 34, 0.25);
  auto stale = request(ObjectiveKind::kNavigate);
  stale.map_revision = 33;
  EXPECT_EQ(planner.plan(chain.graph, StateVec::Zero(), stale).status,
            PlanningStatus::kStaleRevision);
  stale = request(ObjectiveKind::kNavigate);
  stale.graph_revision = 11;
  EXPECT_EQ(planner.plan(chain.graph, StateVec::Zero(), stale).status,
            PlanningStatus::kStaleRevision);
}

TEST(PlanningStages, GoalMustResolveToTheActiveGraph) {
  Chain chain;
  TopologicalGoalPlanner planner("component-a", 12, 34, 0.25);
  auto far = request(ObjectiveKind::kReturnHome);
  far.goal.pose.x() = 30.0;
  const auto route = planner.plan(chain.graph, StateVec::Zero(), far);
  EXPECT_EQ(route.status, PlanningStatus::kUnreachable);
  EXPECT_EQ(route.reason,
            "goal is outside the graph tolerance 0.25 m at "
            "(30.00, 0.00, 0.00)");
}

TEST(PlanningStages, CurrentPoseMustResolveToTheActiveGraph) {
  Chain chain;
  TopologicalGoalPlanner planner("component-a", 12, 34, 0.25);
  auto home = request(ObjectiveKind::kReturnHome);
  home.goal.pose = StateVec::Zero();
  const auto route =
      planner.plan(chain.graph, StateVec(30.0, 0.0, 0.0, 0.0), home);
  EXPECT_EQ(route.status, PlanningStatus::kUnreachable);
  EXPECT_EQ(route.reason,
            "current pose is outside the graph tolerance 0.25 m at "
            "(30.00, 0.00, 0.00)");
}

TEST(PlanningStages, NonfiniteGoalIsRejectedBeforeGraphLookup) {
  Chain chain;
  TopologicalGoalPlanner planner("component-a", 12, 34, 0.25);
  auto invalid = request(ObjectiveKind::kNavigate);
  invalid.goal.pose.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(planner.plan(chain.graph, StateVec::Zero(), invalid).status,
            PlanningStatus::kUnsupportedObjective);
}

TEST(PlanningStages, ExploreIsKeptBehindItsExistingSelector) {
  Chain chain;
  TopologicalGoalPlanner planner("component-a", 12, 34, 0.25);
  EXPECT_EQ(
      planner.plan(chain.graph, StateVec::Zero(), request(ObjectiveKind::kExplore))
          .status,
      PlanningStatus::kUnsupportedObjective);
}

}  // namespace
