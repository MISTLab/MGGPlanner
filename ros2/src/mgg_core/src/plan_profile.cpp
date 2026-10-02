#include "mgg_core/plan_profile.h"

#include <cinttypes>
#include <cstdio>

namespace mgg {

std::string PlanProfile::summary() const {
  char buf[768];
  std::snprintf(
      buf, sizeof(buf),
      "lattice %.0f ms, goal link %.0f ms, search %.0f ms, shortcut %.0f ms; "
      "edge checks %" PRIu64 "/%.0f ms (cached %" PRIu64 "), projections %" PRIu64
      "/%.0f ms, sweeps %" PRIu64 "/%.0f ms, cross slope %" PRIu64
      "/%.0f ms, footprint %" PRIu64 "/%.0f ms (cached %" PRIu64
      "), ground ahead %" PRIu64 "/%.0f ms, clearance %" PRIu64
      "/%.0f ms, cell prechecks %" PRIu64 "/%.0f ms (pre-filtered %" PRIu64
      "), retries %" PRIu64 ", nudges %" PRIu64 "%s",
      lattice.ms(), goal_link.ms(), search.ms(), shortcut.ms(),
      edge_checks.calls, edge_checks.ms(), edge_cache_hits, projection.calls,
      projection.ms(), body_sweeps.calls, body_sweeps.ms(), cross_slope.calls,
      cross_slope.ms(), footprint.calls, footprint.ms(), footprint_cache_hits,
      ground_ahead.calls, ground_ahead.ms(), clearance.calls, clearance.ms(),
      cell_prechecks.calls, cell_prechecks.ms(), prefilter_rejects, retries,
      nudges, budget_exhausted ? "; time budget exhausted" : "");
  return buf;
}

}  // namespace mgg
