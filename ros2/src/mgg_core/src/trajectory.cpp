#include "mgg_core/trajectory.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mgg {

void truncateYaw(double& x) {
  if (x > M_PI) x -= 2 * M_PI;
  else if (x < -M_PI) x += 2 * M_PI;
}

double computeDistanceBetweenTrajectoryAndDirection(
    const PathType& path, double heading, double discrete_length,
    bool scale_with_length) {
  if ((path.size() <= 0) || (discrete_length <= 0))
    return std::numeric_limits<double>::max();

  PathType path_intp;
  if (!interpolatePath(path, discrete_length, path_intp))
    return std::numeric_limits<double>::max();
  double path_length = getPathLength(path_intp);
  if (path_length == 0) return std::numeric_limits<double>::max();

  Eigen::Vector3d p0(path[0][0], path[0][1], path[0][2]);
  Eigen::Vector3d uvector =
      Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()) *
      Eigen::Vector3d(1, 0, 0) * discrete_length;
  std::vector<Eigen::Vector3d> path_ref;
  int n = (int)(path_length / discrete_length);
  for (int i = 0; i < n; ++i) {
    // `i`, not `n`. The ROS 1 code used n here, which made every reference
    // point identical, so the DTW measured distance to a single point instead
    // of deviation from a ray along the heading.
    Eigen::Vector3d ei = p0 + uvector * i;
    path_ref.push_back(ei);
  }

  // DTW distance.
  double dtw_dist = computeDTWDistance(path_intp, path_ref);
  // Scale with length or length square
  if (scale_with_length) dtw_dist /= (path_length * path_length);

  return dtw_dist;
}

double getPathLength(const PathType& path) {
  double total_len = 0;
  int path_size = path.size();
  for (int i = 0; i < (path_size - 1); ++i) {
    VectorType vec = path[i + 1] - path[i];
    total_len += vec.norm();
  }
  return total_len;
}

void shortenPath(PathType& path, double max_len) {
  double total_len = 0;
  const int path_size = static_cast<int>(path.size());
  for (int i = 0; i < (path_size - 1); ++i) {
    total_len += (path[i + 1] - path[i]).norm();
    if (total_len > max_len) {
      // i+1 to make sure the path has at least 1 segment.
      path.erase(path.begin() + i + 1, path.end());
      break;
    }
  }
}

double computeDistanceBetweenTwoTrajectories(const PathType& path_1,
                                             const PathType& path_2,
                                             double discrete_length,
                                             bool shorten_to_same_length,
                                             bool scale_with_length) {
  // Currently, only scale over length: spatial information only.
  double dist_ret = std::numeric_limits<double>::infinity();
  if (path_1.empty() || path_2.empty() || discrete_length <= 0) return dist_ret;

  PathType path_1_intp;
  if (!interpolatePath(path_1, discrete_length, path_1_intp)) return dist_ret;
  PathType path_2_intp;
  if (!interpolatePath(path_2, discrete_length, path_2_intp)) return dist_ret;

  // Cut trajectories to the same length.
  const double path_1_len = getPathLength(path_1_intp);
  const double path_2_len = getPathLength(path_2_intp);
  double common_length = std::min(path_1_len, path_2_len);
  if (shorten_to_same_length) {
    if (path_1_len > path_2_len) {
      common_length = path_2_len;
      shortenPath(path_1_intp, common_length);
    } else if (path_1_len < path_2_len) {
      common_length = path_1_len;
      shortenPath(path_2_intp, common_length);
    }
  }

  dist_ret = computeDTWDistance(path_1_intp, path_2_intp);
  if (scale_with_length) dist_ret /= (common_length * common_length);
  return dist_ret;
}

bool interpolatePath(const PathType& path, double discrete_length,
                                 PathType& path_intp) {
  path_intp.clear();
  if (discrete_length <= 0) return false;

  int path_size = path.size();
  if (path_size == 0) {
    return false;
  } else if (path_size == 1) {
    path_intp.push_back(path[0]);
    return true;
  }

  for (int i = 0; i < (path_size - 1); ++i) {
    // Interpolate along the segment.
    VectorType vec = path[i + 1] - path[i];
    double segment_len = vec.norm();
    if (std::abs(segment_len) < 0.01) {
      // Duplicated nodes. Add one only.
      path_intp.push_back(path[i]);
    } else {
      int n = (int)(segment_len / discrete_length);
      VectorType uvec = vec / segment_len * discrete_length;
      for (int j = 0; j <= n; ++j) {
        path_intp.push_back(path[i] + j * uvec);
      }
    }
  }
  return true;
}

double estimateDirectionFromPath(const PathType& path) {
  // First approach: sum up from each edge
  // Modified approach: sum up from first vertex towards each vertex
  const double kAlpha = 0.9;  // depends more on close vertices.
  if (path.size() <= 1) return 0.0;
  double yaw = 0;
  if (path.size() >= 2) {
    Eigen::Vector3d cur_dir(path[1][0] - path[0][0], path[1][1] - path[0][1],
                            path[1][2] - path[0][2]);
    yaw = atan2(cur_dir[1], cur_dir[0]);
  }

  for (int i = 2; i < path.size(); ++i) {
    // 1
    // Eigen::Vector3d cur_dir(path[i][0] - path[i - 1][0],
    //                         path[i][1] - path[i - 1][1],
    //                         path[i][2] - path[i - 1][2]);
    // 2
    Eigen::Vector3d cur_dir(path[i][0] - path[0][0], path[i][1] - path[0][1],
                            path[i][2] - path[0][2]);
    double yaw_tmp = atan2(cur_dir[1], cur_dir[0]);
    double dyaw = yaw_tmp - yaw;
    truncateYaw(dyaw);
    yaw = yaw + (1 - kAlpha) * dyaw;
    truncateYaw(yaw);
  }
  return yaw;
}

double computeDTWDistance(const PathType& pa, const PathType& pb) {
  const int n = static_cast<int>(pa.size());
  const int m = static_cast<int>(pb.size());
  if (n == 0 || m == 0) return std::numeric_limits<double>::max();

  // Two rolling rows rather than the full (n+1) x (m+1) table. The recurrence
  // only ever reads the previous row and the cell to the left, so the extra
  // rows are dead weight, and the ROS 1 version allocated one std::vector per
  // row on every call. This is the dominant cost of a planning cycle: the
  // direction penalty runs this once per leaf, and a representative graph has
  // a few hundred leaves with paths of a hundred-odd interpolated points.
  //
  // Results are identical; only the storage changed.
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> prev(m + 1, inf);
  std::vector<double> curr(m + 1, inf);
  prev[0] = 0.0;

  for (int i = 1; i <= n; ++i) {
    curr[0] = inf;
    for (int j = 1; j <= m; ++j) {
      const double d = (pa[i - 1] - pb[j - 1]).norm();
      curr[j] = d + std::min(std::min(prev[j - 1], prev[j]), curr[j - 1]);
    }
    prev.swap(curr);
  }
  return prev[m];
}



PathType shortcutPath(const PathType& path, const SegmentFreeFn& segment_free) {
  if (path.size() < 3 || !segment_free) return path;

  PathType out;
  out.push_back(path.front());
  size_t at = 0;
  while (at + 1 < path.size()) {
    // Farthest first: the point of the pass is to take the longest admissible
    // leap, and walking outwards from the near end would stop at the first
    // blocked pair even when a later one is clear (a segment that clips a
    // corner can be blocked while a longer one that passes wide of it is not).
    size_t next = at + 1;
    for (size_t candidate = path.size() - 1; candidate > at + 1; --candidate) {
      if (segment_free(path[at], path[candidate])) {
        next = candidate;
        break;
      }
    }
    out.push_back(path[next]);
    at = next;
  }
  return out;
}

}  // namespace mgg
