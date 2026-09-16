#include "mgg_core/ground_projection.h"

#include <cmath>

namespace mgg {

double GroundProjection::projectSample(Eigen::Vector3d& sample,
                                       VoxelStatus& status) const {
  int unknown_count = 0;
  double central_ray_len = 0.0;

  // The centre plus four local offsets, so a sample straddling a small hole
  // still finds ground without probing onto distant sidewalks or kerbs.
  const double probe_offset = std::max(0.20, 2.0 * map_.getResolution());
  const std::vector<Eigen::Vector3d> extra_samples = {
      {0.0, 0.0, probe_offset},
      {probe_offset, 0.0, probe_offset},
      {-probe_offset, 0.0, probe_offset},
      {0.0, probe_offset, probe_offset},
      {0.0, -probe_offset, probe_offset}};


  // The offset probes start slightly higher as well as to the side, so
  // the drop has to be measured from the sample itself rather than from where
  // the ray began. The ROS 1 code measured from the ray's own start, which
  // makes an offset hit report extra clearance; callers then place the point
  // too low. This function's contract is "how far below `sample` the ground
  // lies", so measure that.
  const double sample_z = sample(2);

  for (size_t i = 0; i < extra_samples.size(); ++i) {
    const Eigen::Vector3d start = sample + extra_samples[i];
    const Eigen::Vector3d end =
        start - Eigen::Vector3d(0.0, 0.0, max_projection_length);

    Eigen::Vector3d end_voxel;
    // Ground rays may cross unobserved air above the lidar. Ordinary body and edge
    // collision checks still run; only known occupied ground can support a
    // projected point.
    const VoxelStatus vs =
        map_.getGroundRayStatus(start, end, false, end_voxel);

    if (vs == VoxelStatus::kOccupied) {
      const double ray_len = sample_z - end_voxel(2);
      if (i == 0) central_ray_len = ray_len;
      status = VoxelStatus::kOccupied;
      return ray_len;
    }

    if (vs == VoxelStatus::kUnknown) {
      const double ray_len = sample_z - end_voxel(2);
      if (i == 0) central_ray_len = ray_len;
      ++unknown_count;
    }
  }

  if (unknown_count >= static_cast<int>(extra_samples.size())) {
    status = VoxelStatus::kUnknown;
    return central_ray_len;
  }
  status = VoxelStatus::kFree;
  return -1.0;
}

ProjectedEdgeStatus GroundProjection::getProjectedEdgeStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel,
    std::vector<Eigen::Vector3d>& projected_edge_out, bool is_hanging) const {
  const double step_size = 2.0 * map_.getResolution();
  const double max_inclination = params_.max_inclination;

  const Eigen::Vector3d ray = end - start;
  const double edge_incl =
      std::atan2(std::abs(ray(2)), std::abs(ray.head(2).norm()));
  if (std::abs(ray(2)) > params_.max_step_height + 1e-6 && edge_incl > max_inclination) return ProjectedEdgeStatus::kSteep;

  const double ray_len = ray.norm();
  if (ray_len < 1e-12) return ProjectedEdgeStatus::kAdmissible;
  const Eigen::Vector3d ray_normed = ray / ray_len;

  std::vector<Eigen::Vector3d> projected_edge;

  if (ray_len >= 2.0 * step_size) {
    Eigen::Vector3d last_point = start;
    for (double step = 0.0; step < ray_len; step += step_size) {
      Eigen::Vector3d edge_point = start + step * ray_normed;
      last_point = edge_point;
      VoxelStatus vs;
      const double ground_height = projectSample(edge_point, vs);
      // Intermediate points may hang only if an endpoint already does.
      if ((vs == VoxelStatus::kUnknown || ground_height < 0.0) &&
          !is_hanging) {
        return ProjectedEdgeStatus::kHanging;
      }
      if (vs != VoxelStatus::kOccupied || ground_height < 0.0) {
        projected_edge.push_back(edge_point);
        continue;
      }
      Eigen::Vector3d projected = edge_point;
      projected(2) -= (ground_height - params_.max_ground_height);
      projected_edge.push_back(projected);
    }
    // Drop the last sample when it nearly coincides with the endpoint, which
    // is appended below regardless.
    if (!projected_edge.empty() &&
        (last_point - end).norm() < 0.75 * step_size) {
      projected_edge.pop_back();  // erase(end()) in the original: UB
    }
  } else {
    VoxelStatus vs;
    Eigen::Vector3d start_m = start;
    const double start_ground = projectSample(start_m, vs);
    if ((vs == VoxelStatus::kUnknown || start_ground < 0.0) && !is_hanging) {
      return ProjectedEdgeStatus::kHanging;
    }
    if (vs == VoxelStatus::kOccupied && start_ground >= 0.0)
      start_m(2) -= (start_ground - params_.max_ground_height);
    projected_edge.push_back(start_m);
  }


  VoxelStatus vs;
  Eigen::Vector3d end_m = end;
  const double ground_height = projectSample(end_m, vs);
  if ((vs == VoxelStatus::kUnknown || ground_height < 0.0) && !is_hanging) {
    return ProjectedEdgeStatus::kHanging;
  }
  if (vs == VoxelStatus::kOccupied && ground_height >= 0.0)
    end_m(2) -= (ground_height - params_.max_ground_height);
  projected_edge.push_back(end_m);


  // Inclination of each segment. Cheaper than a collision check, so first.
  for (size_t i = 1; i < projected_edge.size(); ++i) {
    const Eigen::Vector3d segment = projected_edge[i] - projected_edge[i - 1];
    const double theta =
        std::atan2(std::abs(segment(2)), std::abs(segment.head(2).norm()));
    if (std::abs(segment(2)) > params_.max_step_height + 1e-6 &&
        std::abs(theta) > max_inclination) return ProjectedEdgeStatus::kSteep;
  }

  for (size_t i = 1; i < projected_edge.size(); ++i) {
    const VoxelStatus path = map_.getPathStatus(
        projected_edge[i - 1], projected_edge[i], box_size,
        stop_at_unknown_voxel);
    if (path == VoxelStatus::kUnknown) return ProjectedEdgeStatus::kUnknown;
    if (path == VoxelStatus::kOccupied) return ProjectedEdgeStatus::kOccupied;
  }

  projected_edge_out = projected_edge;
  return ProjectedEdgeStatus::kAdmissible;
}

}  // namespace mgg
