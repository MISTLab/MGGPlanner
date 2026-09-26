// Path helpers used by the exploration-path scoring.
//
// Ported from planner_common/trajectory.h, restricted to the PathType
// (std::vector<Eigen::Vector3d>) overloads. The geometry_msgs::Pose based
// overloads stay at the ROS boundary; the algorithm only needs these.

#ifndef MGG_CORE_TRAJECTORY_H_
#define MGG_CORE_TRAJECTORY_H_

#include <functional>
#include <vector>

#include <Eigen/Dense>

namespace mgg {

using VectorType = Eigen::Vector3d;
using PathType = std::vector<VectorType>;

/// Wraps an angle into (-pi, pi].
void truncateYaw(double& x);

double getPathLength(const PathType& path);

/// Resamples `path` at `discrete_length` spacing.
bool interpolatePath(const PathType& path, double discrete_length,
                     PathType& out);

/// Is a straight move between two points traversable? Supplied by the caller,
/// which owns the map and the robot's footprint.
using SegmentFreeFn =
    std::function<bool(const VectorType& from, const VectorType& to)>;

/// Removes intermediate points a straight line can skip.
///
/// A path taken straight off the grid graph is a walk along lattice edges, so
/// it steps between cell centres and reads as a staircase even across open
/// floor. Shortcutting is what turns it back into the route a person would
/// have drawn.
///
/// Greedy: from each kept point, take the farthest later point the segment
/// test accepts, and drop everything in between. That is stronger than the ROS
/// 1 version, which only collapsed a node when the segment leading to it was
/// under half a metre (rrg.cpp:4589, kSegmentLenMin). On a 0.25 m lattice that
/// rule leaves every longer zig-zag in place, which is most of what makes the
/// published path look erratic.
///
/// The endpoints are always kept: the first is where the robot is, and the
/// last is the viewpoint the gain was computed for, so moving it would score
/// one path and drive another.
PathType shortcutPath(const PathType& path, const SegmentFreeFn& segment_free);

/// Is a whole path acceptable, e.g. PathTurnCheck::admissible? Supplied by
/// the caller.
using PathOkFn = std::function<bool(const PathType& path)>;

/// shortcutPath that takes a leap only when the path it makes, the points
/// kept so far, the leap and the rest of `path` as it is, passes `path_ok`.
/// Joining two segments can make a turn neither had: segment headings of 0,
/// 30, 60 and 90 degrees become 15 and 75, a 60 degree turn. Leaping one
/// point ahead keeps the path as it is, so a `path` that passes `path_ok`
/// gives a shortcut that passes too. A `path` that fails it, or no
/// `path_ok`, is shortcut as by shortcutPath.
PathType shortcutPath(const PathType& path, const SegmentFreeFn& segment_free,
                      const PathOkFn& path_ok);

/// Dynamic time warping distance between two paths.
double computeDTWDistance(const PathType& a, const PathType& b);

/// Cuts `path` after the first segment that takes its length past `max_len`,
/// keeping at least one segment.
void shortenPath(PathType& path, double max_len);

/// DTW distance between two paths after resampling both at `discrete_length`
/// and, with `shorten_to_same_length`, cutting the longer one down to the
/// shorter one's length; `scale_with_length` divides by that length squared.
/// The similarity the frontier path clustering groups by
/// (rrg.cpp:5393, Trajectory::compareTwoTrajectories).
double computeDistanceBetweenTwoTrajectories(const PathType& path_1,
                                             const PathType& path_2,
                                             double discrete_length = 0.2,
                                             bool shorten_to_same_length = true,
                                             bool scale_with_length = true);

/// How far `path` deviates from travelling along `heading`.
///
/// Compares the path against a straight reference ray of the same length
/// running along `heading`, using a dynamic time warping distance, optionally
/// normalised by the squared path length.
///
/// The ROS 1 version built that reference with `p0 + uvector * n` inside a
/// loop over `i`, so every reference point coincided and the DTW measured
/// distance to a single point rather than deviation from a direction. It was
/// live, since path_direction_penalty is non-zero in every shipped config.
/// Corrected here; see ROS2_PORT_PLAN.md section 4.4.
double computeDistanceBetweenTrajectoryAndDirection(const PathType& path,
                                                    double heading,
                                                    double discrete_length,
                                                    bool scale_with_length);

}  // namespace mgg

#endif  // MGG_CORE_TRAJECTORY_H_
