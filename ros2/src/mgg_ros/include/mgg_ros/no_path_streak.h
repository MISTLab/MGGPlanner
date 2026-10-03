#ifndef MGG_ROS_NO_PATH_STREAK_H_
#define MGG_ROS_NO_PATH_STREAK_H_

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
#include <Eigen/Core>

namespace mgg_ros {

/// The planning inputs besides map geometry and pose that change a no-path
/// answer, for the streak's identity (ground12 review P2-1): the scouting
/// exclusions' revision and the operator no-go discs in force, so a zone
/// set, cleared or expired starts a new streak. Peer bodies no longer exist
/// (mgg-nopeer).
inline std::string noPathPlanningInputs(std::uint64_t scouting_revision,
                                        const std::vector<Eigen::Vector2d>& no_go_centres,
                                        const std::vector<double>& no_go_reaches) {
  std::string key = "scouting " + std::to_string(scouting_revision) + "; no-go";
  char disc[96];
  for (std::size_t i = 0; i < no_go_centres.size(); ++i) {
    std::snprintf(disc, sizeof(disc), " %.3f,%.3f,%.3f", no_go_centres[i].x(),
                  no_go_centres[i].y(), i < no_go_reaches.size() ? no_go_reaches[i] : 0.0);
    key += disc;
  }
  return key;
}

// Request-scoped geometry identity and physical pose, never two asynchronous
// topic samples. Compare against the first answer: slow drift is not a stall.
class NoPathStreak {
 public:
  bool note(const std::string& identity, const Eigen::Vector4d& pose,
            bool no_path) {
    if (!no_path || identity.empty() || !pose.allFinite()) {
      count_ = 0;
      return false;
    }
    if (count_ == 0 || identity != identity_ ||
        (pose.head<3>() - anchor_.head<3>()).norm() > 0.1 ||
        std::abs(std::remainder(pose[3] - anchor_[3], 2 * M_PI)) > 0.1) {
      identity_ = identity;
      anchor_ = pose;
      count_ = 0;
    }
    if (count_ < 6) ++count_;
    return count_ == 6;
  }

 private:
  std::string identity_;
  Eigen::Vector4d anchor_ = Eigen::Vector4d::Zero();
  int count_ = 0;
};

}  // namespace mgg_ros
#endif
