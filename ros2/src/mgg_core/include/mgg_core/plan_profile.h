// Where one ground plan spends its time: counters and accumulated wall time
// per kind of map work. A GroundProjection, buildGridGraph and the local
// route fill one in when given it; nothing reads it but logs and the
// navigation benchmark (mgg_map_octomap/test/nav_bench.cpp).

#ifndef MGG_CORE_PLAN_PROFILE_H_
#define MGG_CORE_PLAN_PROFILE_H_

#include <chrono>
#include <cstdint>
#include <string>

namespace mgg {

/// Calls and nanoseconds of one kind of work. Nested kinds overlap: an edge
/// check's time includes the projections, sweeps and footprint checks it
/// made.
struct ProfileCounter {
  std::uint64_t calls = 0;
  std::uint64_t ns = 0;
  double ms() const { return static_cast<double>(ns) * 1e-6; }
};

struct PlanProfile {
  /// GroundProjection::projectSample, and those answered from the per-plan
  /// column cache.
  ProfileCounter projection;
  std::uint64_t projection_cache_hits = 0;
  /// Ground rays cast down a column (projection probes and footprint
  /// cells), whatever answered them.
  ProfileCounter ground_rays;
  /// GroundProjection::getProjectedEdgeStatus, whole.
  ProfileCounter edge_checks;
  /// Body sweeps inside edge checks, one per projected segment.
  ProfileCounter body_sweeps;
  /// crossSlope inside edge checks.
  ProfileCounter cross_slope;
  /// footprintPlane and footprintCellRise inside edge checks.
  ProfileCounter footprint;
  /// footprintPlane answered from the per-plan cache.
  std::uint64_t footprint_cache_hits = 0;
  /// observedGroundAhead inside edge checks.
  ProfileCounter ground_ahead;
  /// clearanceCost (soft lattice weights and shortcut costs).
  ProfileCounter clearance;
  /// Lattice cell body prechecks (box, then oriented box).
  ProfileCounter cell_prechecks;
  /// Lattice cell prechecks answered from the build's own memo.
  std::uint64_t precheck_cache_hits = 0;
  /// Lattice cells refused by the conservative inscribed-disc pre-filter
  /// before any exact check.
  std::uint64_t prefilter_rejects = 0;
  /// Edge verdicts answered from the per-plan edge cache.
  std::uint64_t edge_cache_hits = 0;
  /// Other-level retry offers and lateral nudge offers made.
  std::uint64_t retries = 0;
  std::uint64_t nudges = 0;
  /// Lattice cells tried from another vertex than their nearest.
  std::uint64_t alternate_parents = 0;
  /// Lattice build, goal link, graph search and shortcut, whole.
  ProfileCounter lattice;
  ProfileCounter goal_link;
  ProfileCounter search;
  ProfileCounter shortcut;
  /// Set when a time budget stopped work early.
  bool budget_exhausted = false;

  /// One line, "name calls/ms ...", for logs.
  std::string summary() const;
};

/// Adds the time from construction to destruction to a counter; does
/// nothing without one.
class ProfileScope {
 public:
  explicit ProfileScope(ProfileCounter* counter)
      : counter_(counter),
        start_(counter ? std::chrono::steady_clock::now()
                       : std::chrono::steady_clock::time_point()) {}
  ~ProfileScope() {
    if (counter_ == nullptr) return;
    ++counter_->calls;
    counter_->ns += static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - start_)
            .count());
  }
  ProfileScope(const ProfileScope&) = delete;
  ProfileScope& operator=(const ProfileScope&) = delete;

 private:
  ProfileCounter* counter_;
  std::chrono::steady_clock::time_point start_;
};

}  // namespace mgg

#endif  // MGG_CORE_PLAN_PROFILE_H_
