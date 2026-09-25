// Where the robot has been: its own keyframe trajectory, from which the
// planner rebuilds its global graph (mgg::rebuildRoadmapFromTrajectory)
// when that graph is lost or no longer reaches the robot.
//
// The planner reads it through KeyframeTrajectorySource, so it does not
// depend on how a mapping system stores its solution. GraphSolutionFile is
// the one implementation, for the graph solution SwarmDeck's C-SLAM bridge
// writes next to the MOLA products the planner already reads.

#ifndef MGG_ROS_KEYFRAME_TRAJECTORY_H_
#define MGG_ROS_KEYFRAME_TRAJECTORY_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace mgg_ros {

/// A robot's own keyframes in its map's component frame.
struct KeyframeTrajectory {
  /// The map solution the poses belong to: its component, map epoch and
  /// revision.
  std::string component_id;
  std::uint64_t epoch = 0;
  std::uint64_t revision = 0;
  /// T_component_keyframe of each of the robot's keyframes: its home
  /// keyframe first, then in the order they were taken.
  std::vector<Eigen::Isometry3d> poses;
};

class KeyframeTrajectorySource {
 public:
  virtual ~KeyframeTrajectorySource() = default;
  /// The latest trajectory; false, with `error` saying why, when there is
  /// none to read.
  virtual bool read(KeyframeTrajectory& trajectory, std::string& error) = 0;
};

/// The graph solution file of SwarmDeck's C-SLAM bridge (graph_solution.json
/// in the robot's peer root, replaced whole by a rename on each new
/// solution):
///
///   {"schema": "swarmdeck.pose-snapshot.v1",
///    "solution": {
///      "revision": {"component_id": "<id>", "epoch": <n>, "revision": <n>},
///      "poses": [{"keyframe_id": {"robot_id": "<robot>",
///                                 "session_id": "<run id>", "seq": <n>},
///                 "T_component_keyframe": [[4 x 4, row-major]]}, ...]}}
///
/// Other fields are ignored. The component may hold other robots'
/// keyframes; only `robot_id`'s are taken, which must all be of one session
/// and include its first keyframe, seq 0: the robot's home, each seq once,
/// each pose a rigid transform. They are ordered by seq, which is the order
/// they were taken in.
class GraphSolutionFile : public KeyframeTrajectorySource {
 public:
  /// Files larger than `max_bytes` are refused.
  GraphSolutionFile(std::string path, std::string robot_id,
                    std::size_t max_bytes);
  bool read(KeyframeTrajectory& trajectory, std::string& error) override;

  const std::string& path() const { return path_; }
  const std::string& robotId() const { return robot_id_; }

 private:
  std::string path_;
  std::string robot_id_;
  std::size_t max_bytes_;
};

/// Parses a graph solution document as GraphSolutionFile reads it.
bool parseGraphSolution(const std::string& text, const std::string& robot_id,
                        KeyframeTrajectory& trajectory, std::string& error);

}  // namespace mgg_ros

#endif  // MGG_ROS_KEYFRAME_TRAJECTORY_H_
