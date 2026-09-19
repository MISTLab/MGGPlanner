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

TEST(PlanningStages, ReturnHomeConnectsFromInsideTheConnectorRadius) {
  // The robot stands 2 m beside the chain (past the 0.25 m tolerance, inside
  // the 10 m connector radius): the corridor starts at the live pose, joins
  // the nearest vertex and follows the graph to the home landmark.
  Chain chain;
  TopologicalGoalPlanner planner("component-a", 12, 34, 0.25);
  auto home = request(ObjectiveKind::kReturnHome);
  home.goal.pose = StateVec::Zero();
  const StateVec beside(2.0, 2.0, 0.0, 0.0);
  const auto route = planner.plan(chain.graph, beside, home);
  ASSERT_EQ(route.status, PlanningStatus::kSucceeded);
  ASSERT_EQ(route.poses.size(), 4u);
  EXPECT_TRUE(route.poses.front().isApprox(beside));
  EXPECT_DOUBLE_EQ(route.poses[1].x(), 2.0);
  EXPECT_DOUBLE_EQ(route.poses[1].y(), 0.0);
  EXPECT_DOUBLE_EQ(route.poses.back().x(), 0.0);
  EXPECT_FALSE(route.partial);

  // A zero connector radius restores the strict binding.
  TopologicalGoalPlanner strict("component-a", 12, 34, 0.25, 0.0, 0.0);
  const auto refused = strict.plan(chain.graph, beside, home);
  EXPECT_EQ(refused.status, PlanningStatus::kUnreachable);
  EXPECT_EQ(refused.reason,
            "current pose is outside the graph tolerance 0.25 m at "
            "(2.00, 2.00, 0.00)");

  // A far Navigate from beside the chain also joins the graph first: the
  // corridor is live pose, nearest vertex, the graph towards the goal, and
  // the exact goal, not a straight line handed to the grid stage.
  auto far = request(ObjectiveKind::kNavigate);
  far.goal.pose = StateVec(30.0, 0.0, 0.0, 0.0);
  const auto joined = planner.plan(chain.graph, beside, far);
  ASSERT_EQ(joined.status, PlanningStatus::kSucceeded);
  ASSERT_GE(joined.poses.size(), 3u);
  EXPECT_TRUE(joined.poses.front().isApprox(beside));
  EXPECT_DOUBLE_EQ(joined.poses[1].x(), 2.0);
  EXPECT_DOUBLE_EQ(joined.poses[1].y(), 0.0);
  EXPECT_DOUBLE_EQ(joined.poses.back().x(), 30.0);

  // Explore keeps binding both ends to the graph.
  auto explore = request(ObjectiveKind::kExplore);
  explore.goal.pose = StateVec(3.0, 0.0, 0.0, 0.0);
  EXPECT_EQ(planner.plan(chain.graph, beside, explore).status,
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

TEST(PlanningStages, ExploreSharesTheGraphStageWithItsSelectedTarget) {
  // The utility/gain selector chooses the leaf; the shared stage plans the
  // corridor to it exactly as it does for Navigate and ReturnHome.
  Chain chain;
  TopologicalGoalPlanner planner("component-a", 12, 34, 0.25);
  auto explore = request(ObjectiveKind::kExplore);
  explore.goal.pose = StateVec(3.0, 0.0, 0.0, 0.4);
  const auto route =
      planner.plan(chain.graph, StateVec(0.1, 0.0, 0.0, 0.0), explore);
  ASSERT_EQ(route.status, PlanningStatus::kSucceeded) << route.reason;
  ASSERT_EQ(route.poses.size(), 4u);
  EXPECT_DOUBLE_EQ(route.poses.front().x(), 0.0);
  EXPECT_DOUBLE_EQ(route.poses.back().x(), 3.0);
  EXPECT_FALSE(route.partial);
  EXPECT_DOUBLE_EQ(route.request.goal.pose[3], 0.4);
}

TEST(PlanningStages, ExploreNeverBootstrapsAConnectorBeyondTheGraph) {
  // Only Navigate may follow an optimistic connector towards a goal the graph
  // cannot reach. An exploration candidate must end on measured support, so
  // its target has to bind to an admitted vertex.
  Chain chain;
  TopologicalGoalPlanner planner("component-a", 12, 34, 0.25, 1.0);
  auto explore = request(ObjectiveKind::kExplore);
  explore.goal.pose = StateVec(30.0, 0.0, 0.0, 0.0);
  const auto route = planner.plan(chain.graph, StateVec::Zero(), explore);
  EXPECT_EQ(route.status, PlanningStatus::kUnreachable);
  EXPECT_TRUE(route.poses.empty());
}

TEST(PlanningStages, ObjectivesWithoutAGraphStageStayUnsupported) {
  Chain chain;
  TopologicalGoalPlanner planner("component-a", 12, 34, 0.25);
  EXPECT_EQ(planner
                .plan(chain.graph, StateVec::Zero(),
                      request(ObjectiveKind::kInspect))
                .status,
            PlanningStatus::kUnsupportedObjective);
}

// A diamond whose upper leg is cheaper, so the unmarked corridor is
// deterministic and a mark has a visible alternative.
class Diamond {
 public:
  Diamond() {
    auto* source = new Vertex(0, StateVec(0.0, 0.0, 0.0, 0.0));
    auto* upper = new Vertex(1, StateVec(1.0, 1.0, 0.0, 0.0));
    auto* lower = new Vertex(2, StateVec(1.0, -1.0, 0.0, 0.0));
    auto* target = new Vertex(3, StateVec(2.0, 0.0, 0.0, 0.0));
    graph.addVertex(source);
    graph.addVertex(upper);
    graph.addVertex(lower);
    graph.addVertex(target);
    graph.addEdge(source, upper, 1.0);
    graph.addEdge(upper, target, 1.0);
    graph.addEdge(source, lower, 4.0);
    graph.addEdge(lower, target, 4.0);
  }
  GraphManager graph;
};

PlanningRequest diamondRequest(ObjectiveKind objective) {
  PlanningRequest value = request(objective);
  value.goal.pose = StateVec(2.0, 0.0, 0.0, 0.0);
  return value;
}

TEST(BlockedCorridor, MarkForcesAnAlternativeTopologicalRoute) {
  for (const auto objective : {ObjectiveKind::kExplore,
                               ObjectiveKind::kNavigate,
                               ObjectiveKind::kReturnHome}) {
    Diamond diamond;
    TopologicalGoalPlanner planner("component-a", 12, 34, 0.25);
    const auto unmarked = planner.plan(diamond.graph, StateVec::Zero(),
                                       diamondRequest(objective));
    ASSERT_EQ(unmarked.status, PlanningStatus::kSucceeded) << unmarked.reason;
    ASSERT_EQ(unmarked.poses.size(), 3u);
    EXPECT_DOUBLE_EQ(unmarked.poses[1].y(), 1.0);

    BlockedCorridorRegistry registry;
    registry.block(StateVec(0.0, 0.0, 0.0, 0.0), StateVec(1.0, 1.0, 0.0, 0.0),
                   7, 100.0);
    BlockedCorridorView view;
    view.registry = &registry;
    view.map_revision = 7;
    view.now_s = 100.0;
    ASSERT_TRUE(view.active());
    const auto rerouted = planner.plan(diamond.graph, StateVec::Zero(),
                                       diamondRequest(objective), view);
    ASSERT_EQ(rerouted.status, PlanningStatus::kSucceeded) << rerouted.reason;
    ASSERT_EQ(rerouted.poses.size(), 3u);
    EXPECT_DOUBLE_EQ(rerouted.poses[1].y(), -1.0);
  }
}

TEST(BlockedCorridor, MarkIsUndirectedAndLeavesOtherCorridorsAlone) {
  BlockedCorridorRegistry registry;
  registry.block(StateVec(0.0, 0.0, 0.0, 0.0), StateVec(1.0, 1.0, 0.0, 0.0), 7,
                 100.0);
  EXPECT_TRUE(registry.isBlocked(StateVec(1.0, 1.0, 0.0, 0.0),
                                 StateVec(0.0, 0.0, 0.0, 0.0), 7, 100.0));
  EXPECT_FALSE(registry.isBlocked(StateVec(0.0, 0.0, 0.0, 0.0),
                                  StateVec(1.0, -1.0, 0.0, 0.0), 7, 100.0));
  // A segment shorter than one cell would mark the pose rather than a
  // corridor, so it is refused.
  registry.block(StateVec(5.0, 5.0, 0.0, 0.0), StateVec(5.01, 5.0, 0.0, 0.0), 7,
                 100.0);
  EXPECT_FALSE(registry.isBlocked(StateVec(5.0, 5.0, 0.0, 0.0),
                                  StateVec(5.01, 5.0, 0.0, 0.0), 7, 100.0));
}

TEST(BlockedCorridor, MarksExpireOnTimeAndOnAMaterialMapRevision) {
  BlockedCorridorLimits limits;
  limits.ttl_s = 5.0;
  limits.revision_window = 3;
  BlockedCorridorRegistry registry(limits);
  const StateVec from(0.0, 0.0, 0.0, 0.0);
  const StateVec to(1.0, 1.0, 0.0, 0.0);
  registry.block(from, to, 10, 100.0);
  EXPECT_TRUE(registry.isBlocked(from, to, 10, 104.9));
  EXPECT_FALSE(registry.isBlocked(from, to, 10, 105.1));
  EXPECT_TRUE(registry.isBlocked(from, to, 12, 100.0));
  EXPECT_FALSE(registry.isBlocked(from, to, 13, 100.0));
  // A snapshot replacement that moves the revision backwards invalidates the
  // map the mark was taken against.
  EXPECT_FALSE(registry.isBlocked(from, to, 9, 100.0));
  EXPECT_EQ(registry.activeCount(13, 100.0), 0u);
}

TEST(BlockedCorridor, MemoryStaysBounded) {
  BlockedCorridorLimits limits;
  limits.max_entries = 4;
  limits.ttl_s = 1000.0;
  limits.revision_window = 10000;
  BlockedCorridorRegistry registry(limits);
  for (int i = 0; i < 50; ++i) {
    registry.block(StateVec(10.0 * i, 0.0, 0.0, 0.0),
                   StateVec(10.0 * i + 5.0, 0.0, 0.0, 0.0), 1, 10.0);
  }
  EXPECT_EQ(registry.size(), 4u);
  // The newest marks survive; the oldest were evicted.
  EXPECT_TRUE(registry.isBlocked(StateVec(490.0, 0.0, 0.0, 0.0),
                                 StateVec(495.0, 0.0, 0.0, 0.0), 1, 10.0));
  EXPECT_FALSE(registry.isBlocked(StateVec(0.0, 0.0, 0.0, 0.0),
                                  StateVec(5.0, 0.0, 0.0, 0.0), 1, 10.0));
}

TEST(BlockedCorridor, NoLiveMarkLeavesTheCorridorUntouched) {
  Diamond diamond;
  TopologicalGoalPlanner planner("component-a", 12, 34, 0.25);
  BlockedCorridorRegistry registry;
  registry.block(StateVec(0.0, 0.0, 0.0, 0.0), StateVec(1.0, 1.0, 0.0, 0.0), 7,
                 100.0);
  BlockedCorridorView expired;
  expired.registry = &registry;
  expired.map_revision = 7;
  expired.now_s = 1000.0;  // past the default ten-second lifetime
  EXPECT_FALSE(expired.active());
  const auto route = planner.plan(diamond.graph, StateVec::Zero(),
                                  diamondRequest(ObjectiveKind::kNavigate),
                                  expired);
  ASSERT_EQ(route.status, PlanningStatus::kSucceeded) << route.reason;
  ASSERT_EQ(route.poses.size(), 3u);
  EXPECT_DOUBLE_EQ(route.poses[1].y(), 1.0);
}

TEST(BlockedCorridor, EveryCorridorBlockedLeavesTheGoalUnreachable) {
  Diamond diamond;
  TopologicalGoalPlanner planner("component-a", 12, 34, 0.25);
  BlockedCorridorRegistry registry;
  registry.block(StateVec(0.0, 0.0, 0.0, 0.0), StateVec(1.0, 1.0, 0.0, 0.0), 7,
                 100.0);
  registry.block(StateVec(0.0, 0.0, 0.0, 0.0), StateVec(1.0, -1.0, 0.0, 0.0), 7,
                 100.0);
  BlockedCorridorView view;
  view.registry = &registry;
  view.map_revision = 7;
  view.now_s = 100.0;
  const auto route = planner.plan(diamond.graph, StateVec::Zero(),
                                  diamondRequest(ObjectiveKind::kExplore),
                                  view);
  EXPECT_EQ(route.status, PlanningStatus::kUnreachable);
  EXPECT_EQ(route.reason, "goal has no unblocked corridor in the active graph");
}

}  // namespace
