#include "mgg_core/recent_track.h"

#include <algorithm>
#include <cmath>
namespace mgg {
void RecentTrack::add(const StateVec& pose) {
  if (!pose.allFinite()) return;
  if (!poses_.empty() && (pose - poses_.back()).head<3>().norm() < 0.01) {
    poses_.back() = pose;
    return;
  }
  poses_.push_back(pose);
  double length = 0;
  for (size_t i = poses_.size() - 1; i > 0; --i) {
    length += (poses_[i] - poses_[i - 1]).head<3>().norm();
    if (length > 10) {
      poses_.erase(poses_.begin(), poses_.begin() + i);
      break;
    }
  }
}
std::vector<StateVec> RecentTrack::backFrom(const StateVec& pose,
                                            double max_m) const {
  if (poses_.empty() || !(max_m > 0) || !pose.allFinite()) return {};
  // Prefer the most recent visit; never bridge from a different track.
  size_t at = poses_.size();
  for (size_t i = poses_.size(); i > 0; --i)
    if ((pose - poses_[i - 1]).head<3>().norm() <= 0.05) {
      at = i - 1;
      break;
    }
  if (at == poses_.size()) return {};
  std::vector<StateVec> out{pose};
  double length = 0;
  for (size_t i = at + 1; i > 0; --i) {
    const auto& next = poses_[i - 1];
    const double segment = (next - out.back()).head<3>().norm();
    if (segment < 1e-9) continue;
    const StateVec start = out.back();
    const double take = std::min(segment, max_m - length);
    const int steps = std::max(1, static_cast<int>(std::ceil(take / 0.25)));
    const double yaw_delta =
        std::atan2(std::sin(next[3] - start[3]), std::cos(next[3] - start[3]));
    for (int step = 1; step <= steps; ++step) {
      const double fraction = take * step / (steps * segment);
      StateVec sample = start + (next - start) * fraction;
      sample[3] = start[3] + yaw_delta * fraction;
      out.push_back(sample);
    }
    length += take;
    if (length >= max_m) break;
  }
  return out;
}
}  // namespace mgg
