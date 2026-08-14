#include "mgg_core/trajectory.h"

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
    // NOTE: `n`, not `i`. Reproduced from the ROS 1 code, where it makes every
    // reference point identical so the comparison is against a single point
    // rather than a ray along the heading. See the header. Changing it to `i`
    // alters path selection in every shipped configuration.
    Eigen::Vector3d ei = p0 + uvector * n;
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
  int n = pa.size();
  int m = pb.size();

  std::vector<std::vector<double>> dist;
  dist.resize(n + 1);
  for (auto& v : dist) {
    v.resize(m + 1);
  }

  for (int i = 1; i <= n; ++i) {
    dist[i][0] = std::numeric_limits<double>::infinity();
  }
  for (int j = 1; j <= m; ++j) {
    dist[0][j] = std::numeric_limits<double>::infinity();
  }

  dist[0][0] = 0;
  for (int i = 1; i <= n; ++i) {
    for (int j = 1; j <= m; ++j) {
      double d = (pa[i - 1] - pb[j - 1]).norm();
      dist[i][j] = d + std::min(std::min(dist[i - 1][j - 1], dist[i - 1][j]),
                                dist[i][j - 1]);
    }
  }

  return dist[n][m];
}


}  // namespace mgg
