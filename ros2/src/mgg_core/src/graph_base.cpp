#include "mgg_core/graph_base.h"

#include <cstdio>

namespace mgg {

std::string SampleStatistic::formatTimes(const std::string& title) const {
  char buf[512];
  std::snprintf(buf, sizeof(buf),
                "Time statistics%s%s:\n"
                "  Build graph    : %3.3f s\n"
                "  Compute gain   : %3.3f s\n"
                "  Dijkstra       : %3.3f s\n"
                "  Evaluate graph : %3.3f s\n"
                "  Total          : %3.3f s",
                title.empty() ? "" : " ", title.c_str(), build_graph_time,
                compute_exp_gain_time, shortest_path_time,
                evaluate_graph_time, totalTime());
  return std::string(buf);
}

}  // namespace mgg
