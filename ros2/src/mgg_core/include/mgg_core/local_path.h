#ifndef MGG_CORE_LOCAL_PATH_H_
#define MGG_CORE_LOCAL_PATH_H_
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "mgg_core/types.h"
namespace mgg {
enum class LocalPathKind { kStart, kExtend, kBreak };
// ROS-free counterpart of LocalPath. The driver supplies frame/stamp and
// process epoch/sequence; dependency boxes are local, not transmitted.
struct LocalPathPlan {
  std::string frame_id, session_id, request_id;
  uint64_t stamp_ns = 0, epoch = 0, sequence_id = 0, extends_sequence_id = 0;
  uint64_t map_revision = 0, guidance_sequence_id = 0;
  LocalPathKind kind = LocalPathKind::kStart;
  uint32_t prefix_length = 0;
  double commit_length_m = 0;
  std::vector<StateVec> poses;
  // One flag per pose, describing travel leaving that pose; last repeats.
  std::vector<bool> reverse;
  bool reaches_goal = false;
  std::vector<Eigen::AlignedBox3d> edge_dependencies;
};
struct BrakingBounds {
  double deceleration_mps2 = 0.25;
  double latency_s = 0.2;
  double planning_latency_s = 0.3;
  double margin_m = 0.05;
};
double pathLength(const LocalPathPlan& path);
double commitmentLength(double speed_mps, const BrakingBounds& bounds);
double commitmentSpeedCap(double available_m, const BrakingBounds& bounds);
// Keep original vertices (including the vertex just before progress), rounding
// the end outward. No interpolation may alter an already certified prefix.
LocalPathPlan committedPrefix(const LocalPathPlan&, double progress_m,
                              double length_m);
// What the executor may still drive of `path` without a newer one: from
// progress_m to its commit_length_m (at least the segment it is on).
LocalPathPlan remainingCommitment(const LocalPathPlan& path, double progress_m);
LocalPathPlan splicePath(const LocalPathPlan& prefix,
                         const LocalPathPlan& extension);
}  // namespace mgg
#endif
