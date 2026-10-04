#ifndef MGG_CORE_RECENT_TRACK_H_
#define MGG_CORE_RECENT_TRACK_H_
#include <vector>

#include "mgg_core/types.h"
namespace mgg {
// Chronological odometry only: never populate from planned geometry.
class RecentTrack {
 public:
  void add(const StateVec&);
  std::vector<StateVec> backFrom(const StateVec&, double max_m) const;
  void clear() { poses_.clear(); }

 private:
  std::vector<StateVec> poses_;
};
}  // namespace mgg
#endif
