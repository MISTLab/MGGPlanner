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
  }
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
  EXPECT_EQ(planner.plan(chain.graph, StateVec::Zero(), far).status,
            PlanningStatus::kUnreachable);
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
