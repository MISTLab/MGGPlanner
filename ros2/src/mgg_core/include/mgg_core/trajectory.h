// Path helpers used by the exploration-path scoring.
//
// Ported from planner_common/trajectory.h, restricted to the PathType
// (std::vector<Eigen::Vector3d>) overloads. The geometry_msgs::Pose based
// overloads stay at the ROS boundary; the algorithm only needs these.

#ifndef MGG_CORE_TRAJECTORY_H_
#define MGG_CORE_TRAJECTORY_H_

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

/// Dynamic time warping distance between two paths.
double computeDTWDistance(const PathType& a, const PathType& b);

/// Heading implied by a path, weighted towards its early section.
double estimateDirectionFromPath(const PathType& path);

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
