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
/// KNOWN DEFECT, reproduced deliberately. The reference path this compares
/// against is built as
///
///     for (i = 0; i < n; ++i) path_ref.push_back(p0 + uvector * n);
///
/// using `n` where `i` was meant, so every reference point is identical and
/// the "direction" is a single point at p0 + heading * path_length rather
/// than a ray along the heading. The DTW is therefore degenerate: it measures
/// distance to one point, not deviation from a direction.
///
/// This is live. path_direction_penalty is 0.3 in three shipped configs and
/// 0.2 in the fourth, so the term is applied on every path. The planner does
/// still prefer forward paths, but by a different measure than intended.
/// Correcting it would change path selection, so it is left as-is pending a
/// decision. See ROS2_PORT_PLAN.md section 4.4.
double computeDistanceBetweenTrajectoryAndDirection(const PathType& path,
                                                    double heading,
                                                    double discrete_length,
                                                    bool scale_with_length);

}  // namespace mgg

#endif  // MGG_CORE_TRAJECTORY_H_
