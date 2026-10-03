#ifndef MGG_ROS_NO_PATH_STREAK_H_
#define MGG_ROS_NO_PATH_STREAK_H_

#include <cmath>
#include <string>
#include <Eigen/Core>

namespace mgg_ros {

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
