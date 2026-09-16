// Transport-free contracts between mission objectives, the persistent graph
// planner, and a terrain-aware grid planner.

#ifndef MGG_CORE_PLANNING_STAGES_H_
#define MGG_CORE_PLANNING_STAGES_H_

#include <cstdint>
#include <string>
#include <vector>

#include "mgg_core/graph_manager.h"
#include "mgg_core/types.h"

namespace mgg {

enum class ObjectiveKind : std::uint8_t {
  kExplore = 0,
  kNavigate,
  kReturnHome,
  kInspect,
  kRendezvous,
};

struct PlanningGoal {
  StateVec pose = StateVec::Zero();
  // A mapper/SLAM adapter may resolve this stable reference again after a
  // graph correction. Empty means pose is the only available reference.
  std::string landmark_id;
};

struct PlanningRequest {
  std::string mission_id;
  ObjectiveKind objective = ObjectiveKind::kExplore;
  PlanningGoal goal;
  std::string component_id;
  std::uint64_t graph_revision = 0;
  std::uint64_t map_revision = 0;
  // Exact correction-aware mapping snapshot used for feasibility checks.
  // These revisions are distinct from MGG's route graph/map revisions.
  std::uint64_t map_epoch = 0;
  std::uint64_t mapping_graph_revision = 0;
  std::string geometry_revision;
  std::int32_t map_source_stamp_sec = 0;
  std::uint32_t map_source_stamp_nanosec = 0;
};

enum class PlanningStatus : std::uint8_t {
  kSucceeded = 0,
  kUnreachable,
  kStaleRevision,
  kUnsupportedObjective,
  kBlocked,
};

struct RouteCorridor {
  PlanningStatus status = PlanningStatus::kUnreachable;
  PlanningRequest request;
  // Topological route in source-to-target order. The exact requested goal is
  // retained separately: a grid planner must decide how to connect it safely.
  std::vector<StateVec> poses;
  std::string reason;
};

struct FeasiblePath {
  PlanningStatus status = PlanningStatus::kBlocked;
  std::string mission_id;
  std::string component_id;
  std::uint64_t graph_revision = 0;
  std::uint64_t map_revision = 0;
  std::uint64_t map_epoch = 0;
  std::uint64_t mapping_graph_revision = 0;
  std::string geometry_revision;
  std::int32_t map_source_stamp_sec = 0;
  std::uint32_t map_source_stamp_nanosec = 0;
  std::vector<StateVec> poses;
  std::vector<double> speed_limits;
  std::string reason;
};

class GridPlanner {
 public:
  virtual ~GridPlanner() = default;
  virtual FeasiblePath refine(const RouteCorridor& corridor) = 0;
};

// Implements the graph stage for explicit navigate and return-home goals.
// Explore remains in the existing gain/path-selection pipeline until it can
// implement the same boundary without changing its tested scoring behaviour.
class TopologicalGoalPlanner {
 public:
  TopologicalGoalPlanner(std::string component_id,
                         std::uint64_t graph_revision,
                         std::uint64_t map_revision,
                         double goal_vertex_tolerance);

  RouteCorridor plan(GraphManager& graph, const StateVec& current,
                     const PlanningRequest& request) const;

 private:
  std::string component_id_;
  std::uint64_t graph_revision_;
  std::uint64_t map_revision_;
  double goal_vertex_tolerance_;
};

}  // namespace mgg

#endif  // MGG_CORE_PLANNING_STAGES_H_
