// Transport-free contracts between mission objectives, the persistent graph
// planner, and a terrain-aware grid planner.

#ifndef MGG_CORE_PLANNING_STAGES_H_
#define MGG_CORE_PLANNING_STAGES_H_

#include <cstddef>
#include <cstdint>
#include <map>
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

/// Configuration of the blocked-corridor memory.
struct BlockedCorridorLimits {
  /// Hard bound on retained marks. Marking beyond this evicts the oldest.
  std::size_t max_entries = 64;
  /// Quantisation of a corridor endpoint, in metres. Two segments share one
  /// mark when both of their endpoints fall in the same cell.
  double cell_size_m = 0.5;
  /// Wall-clock lifetime of a mark, in seconds.
  double ttl_s = 10.0;
  /// A mark also expires once the planning map revision has advanced by this
  /// many revisions: new measurements are what can change the verdict, so a
  /// materially different map reopens the corridor.
  std::uint64_t revision_window = 8;
};

/// Bounded, expiring record of topological corridors whose bounded terrain
/// refinement, or whose execution, has just been rejected.
///
/// This is a routing preference and never a safety authority. A mark can only
/// make the topological stage prefer another corridor; every terrain, body,
/// step, drop and geofence veto still runs on whichever route is chosen. That
/// is why marks may expire eagerly: an expired mark costs one bounded
/// refinement attempt, never an unchecked path.
class BlockedCorridorRegistry {
 public:
  BlockedCorridorRegistry() = default;
  explicit BlockedCorridorRegistry(const BlockedCorridorLimits& limits);

  void setLimits(const BlockedCorridorLimits& limits);
  const BlockedCorridorLimits& limits() const { return limits_; }

  /// Marks the undirected segment from->to blocked. Non-finite endpoints, a
  /// zero-length segment and a zero capacity are ignored.
  void block(const StateVec& from, const StateVec& to,
             std::uint64_t map_revision, double now_s);
  bool isBlocked(const StateVec& from, const StateVec& to,
                 std::uint64_t map_revision, double now_s) const;
  /// Marks still active at this revision and time.
  std::size_t activeCount(std::uint64_t map_revision, double now_s) const;
  /// Retained marks, expired or not. Never exceeds limits().max_entries.
  std::size_t size() const { return entries_.size(); }
  /// Drops every expired mark. Work is bounded by size().
  void expire(std::uint64_t map_revision, double now_s);
  void clear();

 private:
  struct Key {
    std::int64_t cells[6] = {0, 0, 0, 0, 0, 0};
    bool operator<(const Key& other) const {
      for (std::size_t i = 0; i < 6; ++i) {
        if (cells[i] != other.cells[i]) return cells[i] < other.cells[i];
      }
      return false;
    }
  };
  struct Entry {
    std::uint64_t map_revision = 0;
    double deadline_s = 0.0;
    std::uint64_t sequence = 0;
  };
  bool makeKey(const StateVec& from, const StateVec& to, Key& key) const;
  bool active(const Entry& entry, std::uint64_t map_revision,
              double now_s) const;

  BlockedCorridorLimits limits_;
  std::map<Key, Entry> entries_;
  std::uint64_t sequence_ = 0;
};

/// Read-only handle passed to the topological stage so it can route around
/// marks without owning their lifetime.
struct BlockedCorridorView {
  const BlockedCorridorRegistry* registry = nullptr;
  std::uint64_t map_revision = 0;
  double now_s = 0.0;

  bool blocked(const StateVec& a, const StateVec& b) const {
    return registry != nullptr &&
           registry->isBlocked(a, b, map_revision, now_s);
  }
  /// True when at least one mark is live, so the avoiding search is worth
  /// running. With no live mark the stage keeps its original search exactly.
  bool active() const {
    return registry != nullptr &&
           registry->activeCount(map_revision, now_s) > 0;
  }
};

struct RouteCorridor {
  PlanningStatus status = PlanningStatus::kUnreachable;
  PlanningRequest request;
  // Topological route in source-to-target order. The exact requested goal is
  // retained separately: a grid planner must decide how to connect it safely.
  std::vector<StateVec> poses;
  // True when poses end at a local progress proxy rather than the exact goal
  // retained in request.goal. A controller success for this corridor must
  // trigger continuation, never completion of the operator objective.
  bool partial = false;
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
  bool partial = false;
  // True only after the complete emitted path passed QueryMapBatch against
  // the exact mapping snapshot carried above.
  bool indexed_map_validated = false;
  std::string reason;
  // The corridor segment whose bounded refinement failed, as indices into
  // RouteCorridor::poses. kNoCorridorIndex stands for the implicit start (the
  // live pose) and poses.size() for the exact request-owned goal. Only
  // meaningful when blocked_segment_identified is true; callers may use it to
  // mark that corridor so the topological stage picks an alternative.
  bool blocked_segment_identified = false;
  std::size_t blocked_from_index = 0;
  std::size_t blocked_to_index = 0;
};

/// Stands for "not a corridor pose" in FeasiblePath's blocked-segment indices.
inline constexpr std::size_t kNoCorridorIndex =
    static_cast<std::size_t>(-1);

class GridPlanner {
 public:
  virtual ~GridPlanner() = default;
  virtual FeasiblePath refine(const RouteCorridor& corridor) = 0;
};

// Implements the graph stage for every mission objective.
//
// Navigate and ReturnHome supply their operator goal directly. Explore keeps
// its own utility/gain selection: that selector chooses the target frontier
// vertex, and the resulting pose arrives here as the goal, so all three
// objectives share one corridor stage, one terrain decision and one route
// continuation. Only Navigate may bootstrap an optimistic connector towards a
// goal that the graph cannot yet reach; Explore must bind both ends to the
// active graph. ReturnHome binds its goal to the graph, and its start either
// to a vertex within the tolerance or, within `return_home_connector_radius`,
// to the nearest vertex through an unvalidated first segment that the rolling
// grid stage checks like any other: the global backbone stops admitting
// breadcrumbs whenever the map under one of them is unknown or occupied, so a
// robot is routinely a few metres past its last admitted breadcrumb when it
// is told to come home (every Return Home refused on benchbot, 2026-09-18).
class TopologicalGoalPlanner {
 public:
  TopologicalGoalPlanner(std::string component_id,
                         std::uint64_t graph_revision,
                         std::uint64_t map_revision,
                         double goal_vertex_tolerance,
                         double minimum_partial_progress = 0.0,
                         double return_home_connector_radius = 10.0);

  RouteCorridor plan(GraphManager& graph, const StateVec& current,
                     const PlanningRequest& request,
                     const BlockedCorridorView& blocked = {}) const;

 private:
  std::string component_id_;
  std::uint64_t graph_revision_;
  std::uint64_t map_revision_;
  double goal_vertex_tolerance_;
  double minimum_partial_progress_;
  double return_home_connector_radius_;
};

}  // namespace mgg

#endif  // MGG_CORE_PLANNING_STAGES_H_
