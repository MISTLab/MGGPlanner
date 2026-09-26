#include "mgg_core/ground_projection.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <utility>

namespace mgg {

namespace {

/// How far above the sample projectSample's rays start.
double probeOffset(const MapInterface& map) {
  return std::max(0.20, 2.0 * map.getResolution());
}

/// Most map cells footprintPlane considers around one point. A Bunker's
/// footprint spans about 150 of 0.2 m; a far larger one is not measured.
constexpr std::size_t kMaxFootprintCells = 1024;

/// Solids a goal's upward search steps through before giving up; each has
/// free space above it, so a column this tall is never met in practice.
constexpr int kMaxGoalGroundLayers = 32;

}  // namespace

double GroundProjection::projectSample(Eigen::Vector3d& sample,
                                       VoxelStatus& status) const {
  int unknown_count = 0;
  double central_ray_len = 0.0;

  // The centre plus four local offsets, so a sample straddling a small hole
  // still finds ground without probing onto distant sidewalks or kerbs.
  const double probe_offset = probeOffset(map_);
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

double GroundProjection::projectGoal(Eigen::Vector3d& sample,
                                     VoxelStatus& status) const {
  Eigen::Vector3d below_sample = sample;
  VoxelStatus below_status = VoxelStatus::kFree;
  const double below = projectSample(below_sample, below_status);
  const bool have_below = below_status == VoxelStatus::kOccupied;

  // projectSample covers everything up to where its rays start; look above
  // that, up to the bounded rise. From above the window down, each ray from
  // free space stops on top of the next solid; step through that solid and
  // look again, so the last top found is the lowest above the sample. The
  // walk starts a voxel above the bound so a top exactly at the bound is
  // met from free space; a top past the bound is stepped through unrecorded.
  const double rise = params_.max_goal_ground_rise;
  const double reach =
      std::isfinite(rise) ? std::clamp(rise, 0.0, kMaxGoalGroundRise) : 0.0;
  const double resolution = map_.getResolution();
  const double floor_z = sample.z() + probeOffset(map_);
  const double bound_z = sample.z() + reach;
  Eigen::Vector3d probe(sample.x(), sample.y(), bound_z + resolution);
  const Eigen::Vector3d end(sample.x(), sample.y(), floor_z);
  bool have_above = false;
  double above_z = 0.0;
  for (int layer = 0;
       layer < kMaxGoalGroundLayers && resolution > 0.0 && floor_z < bound_z;
       ++layer) {
    while (probe.z() > floor_z &&
           map_.getVoxelStatus(probe) == VoxelStatus::kOccupied) {
      probe.z() -= resolution;
    }
    if (probe.z() <= floor_z) break;
    Eigen::Vector3d hit;
    if (map_.getGroundRayStatus(probe, end, false, hit) !=
            VoxelStatus::kOccupied ||
        !(hit.z() > floor_z)) {
      break;
    }
    if (hit.z() <= bound_z + 1e-9) {
      have_above = true;
      above_z = hit.z();
    }
    // Just under the reported top, which a backend's measured surface may
    // put on the solid's upper face, so the step down starts inside it.
    probe.z() = hit.z() - 1e-6;
  }

  if (have_above && (!have_below || above_z - sample.z() < std::abs(below))) {
    status = VoxelStatus::kOccupied;
    return sample.z() - above_z;
  }
  sample = below_sample;
  status = below_status;
  return below;
}

ProjectedEdgeStatus GroundProjection::getProjectedEdgeStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel,
    std::vector<Eigen::Vector3d>& projected_edge_out, bool is_hanging,
    bool preserve_start_height, const EdgeBodyCheck* body,
    EdgeTravel travel) const {
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
      if (preserve_start_height && step == 0.0) {
        projected_edge.push_back(edge_point);
        continue;
      }
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
    if (!preserve_start_height) {
      const double start_ground = projectSample(start_m, vs);
      if ((vs == VoxelStatus::kUnknown || start_ground < 0.0) && !is_hanging) {
        return ProjectedEdgeStatus::kHanging;
      }
      if (vs == VoxelStatus::kOccupied && start_ground >= 0.0)
        start_m(2) -= (start_ground - params_.max_ground_height);
    }
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
    const VoxelStatus path =
        body != nullptr
            ? body->sweep(projected_edge[i - 1], projected_edge[i])
            : map_.getPathStatus(projected_edge[i - 1], projected_edge[i],
                                 box_size, stop_at_unknown_voxel);
    if (path == VoxelStatus::kUnknown) return ProjectedEdgeStatus::kUnknown;
    if (path == VoxelStatus::kOccupied) return ProjectedEdgeStatus::kOccupied;
  }

  // After the sweep, so the side probes start inside a footprint known not
  // to be occupied rather than inside a wall.
  const bool standing_at_start = body != nullptr && body->standing_at_start;
  if (params_.max_cross_slope < M_PI_2 &&
      crossSlope(projected_edge, box_size, standing_at_start) >
          params_.max_cross_slope) {
    return ProjectedEdgeStatus::kCrossSlope;
  }

  // The points the body is checked at: every point of the polyline and
  // between them, at most kFootprintSampleSpacing apart. A short edge's
  // polyline is its two ends, 0.4 to 0.57 m apart on the lattice, and which
  // of a rock's cells fell under a footprint then depended on where the
  // lattice lay (run 5, robot_2). Not the start where the robot stands.
  std::vector<Eigen::Vector3d> samples;
  for (std::size_t i = 1; i < projected_edge.size(); ++i) {
    const Eigen::Vector3d& from = projected_edge[i - 1];
    const Eigen::Vector3d& to = projected_edge[i];
    const int count = std::max(
        1, static_cast<int>(std::ceil((to - from).head<2>().norm() /
                                          kFootprintSampleSpacing -
                                      1e-9)));
    for (int k = (i == 1 && standing_at_start) ? 1 : 0; k < count; ++k) {
      samples.push_back(from + (to - from) * (double(k) / count));
    }
  }
  if (!projected_edge.empty()) samples.push_back(projected_edge.back());
  const Eigen::Vector2d heading =
      projected_edge.size() >= 2
          ? Eigen::Vector2d(
                (projected_edge.back() - projected_edge.front()).head<2>())
          : Eigen::Vector2d::Zero();

  const bool check_tilt = params_.max_footprint_tilt > 0.0;
  const bool check_step = params_.max_footprint_step > 0.0;
  const bool check_rise = params_.max_footprint_cell_rise > 0.0;
  if ((check_tilt || check_step || check_rise) && heading.norm() > 1e-9) {
    for (const Eigen::Vector3d& point : samples) {
      if (check_tilt || check_step) {
        const FootprintPlane plane = footprintPlane(point, heading, box_size);
        if (plane.measured &&
            ((check_tilt && plane.tilt > params_.max_footprint_tilt) ||
             (check_step &&
              plane.max_residual > params_.max_footprint_step))) {
          return ProjectedEdgeStatus::kFootprintPlane;
        }
      }
      // Only when set: the rise is not cached with the plane, whose key
      // carries no parameter (review r2, M-2).
      if (check_rise && footprintCellRise(point, heading, box_size) >
                            params_.max_footprint_cell_rise) {
        return ProjectedEdgeStatus::kFootprintPlane;
      }
    }
  }

  // Never onto unobserved ground (item 7): at every point, the half of the
  // footprint ahead must stand on observed ground. robot_0 was parked with
  // its front half over an unobserved 4 m pit in run 5 and tipped into it.
  // An edge that may hang, from a root the lidar has not yet seen the ground
  // under, is exempt but for its end, which is a vertex with ground. A
  // graph edge may be driven either way, and each way has its own half
  // ahead: checked one way only, a path driving it the other way could end
  // with the robot's front over a drop (review r2, I-1).
  const double min_ground = params_.min_observed_ground_fraction;
  if (min_ground > 0.0 && heading.norm() > 1e-9) {
    for (std::size_t i = 0; i < samples.size(); ++i) {
      if (is_hanging && i + 1 < samples.size()) continue;
      if (observedGroundAhead(samples[i], heading, box_size) <
              min_ground - 1e-9 ||
          (travel == EdgeTravel::kBothWays &&
           observedGroundAhead(samples[i], -heading, box_size) <
               min_ground - 1e-9)) {
        return ProjectedEdgeStatus::kGroundUnobserved;
      }
    }
  }

  projected_edge_out = projected_edge;
  return ProjectedEdgeStatus::kAdmissible;
}

double GroundProjection::observedGroundAhead(
    const Eigen::Vector3d& point, const Eigen::Vector2d& heading,
    const Eigen::Vector3d& box_size) const {
  const double half_length = 0.5 * box_size.x();
  const double half_width = 0.5 * box_size.y();
  if (!point.allFinite() || !(heading.norm() > 1e-9) ||
      !(half_length > 0.0) || !(half_width > 0.0)) {
    return 0.0;
  }
  const Eigen::Vector2d along = heading.normalized();
  const Eigen::Vector2d across(-along.y(), along.x());
  std::vector<XYCellCenter> candidates;
  if (!map_.getCircleIntersectingXYCellCenters(
          point.head<2>(), std::hypot(half_length, half_width),
          kMaxFootprintCells, candidates)) {
    return 0.0;
  }
  const double lowest = point.z() - 2.0 * params_.max_ground_height;
  int ahead = 0;
  int observed = 0;
  for (const XYCellCenter& cell : candidates) {
    const Eigen::Vector2d offset = cell.center - point.head<2>();
    const double forward = offset.dot(along);
    if (forward < -1e-9 || forward > half_length + 1e-9 ||
        std::abs(offset.dot(across)) > half_width + 1e-9) {
      continue;
    }
    ++ahead;
    Eigen::Vector3d ground;
    if (footprintGroundBelow(
            Eigen::Vector3d(cell.center.x(), cell.center.y(), point.z()),
            ground)) {
      if (ground.z() >= lowest) ++observed;
    } else if (standing_start_ && standing_start_->covers(cell.center)) {
      ++observed;
    }
  }
  return ahead > 0 ? static_cast<double>(observed) / ahead : 0.0;
}

double GroundProjection::footprintCellRise(
    const Eigen::Vector3d& point, const Eigen::Vector2d& heading,
    const Eigen::Vector3d& box_size) const {
  const double half_length = 0.5 * box_size.x();
  const double half_width = 0.5 * box_size.y();
  if (!point.allFinite() || !(heading.norm() > 1e-9) ||
      !(half_length > 0.0) || !(half_width > 0.0)) {
    return 0.0;
  }
  const Eigen::Vector2d along = heading.normalized();
  const Eigen::Vector2d across(-along.y(), along.x());
  std::vector<XYCellCenter> candidates;
  if (!map_.getCircleIntersectingXYCellCenters(
          point.head<2>(), std::hypot(half_length, half_width),
          kMaxFootprintCells, candidates)) {
    return 0.0;
  }
  // The ground of each cell under the footprint by its index on the map's
  // grid.
  std::map<std::pair<std::int64_t, std::int64_t>, double> ground_by_cell;
  for (const XYCellCenter& cell : candidates) {
    const Eigen::Vector2d offset = cell.center - point.head<2>();
    if (std::abs(offset.dot(along)) > half_length + 1e-9 ||
        std::abs(offset.dot(across)) > half_width + 1e-9) {
      continue;
    }
    Eigen::Vector3d ground;
    if (footprintGroundBelow(
            Eigen::Vector3d(cell.center.x(), cell.center.y(), point.z()),
            ground)) {
      ground_by_cell[{cell.grid_x, cell.grid_y}] = ground.z();
    }
  }
  double rise = 0.0;
  for (const auto& [index, z] : ground_by_cell) {
    for (const std::pair<std::int64_t, std::int64_t> next :
         {std::make_pair(index.first + 1, index.second),
          std::make_pair(index.first, index.second + 1)}) {
      const auto found = ground_by_cell.find(next);
      if (found != ground_by_cell.end()) {
        rise = std::max(rise, std::abs(found->second - z));
      }
    }
  }
  return rise;
}

bool GroundProjection::groundBelow(const Eigen::Vector3d& point,
                                   Eigen::Vector3d& ground) const {
  const Eigen::Vector3d end =
      point - Eigen::Vector3d(0.0, 0.0, max_projection_length);
  return map_.getGroundRayStatus(point, end, false, ground) ==
         VoxelStatus::kOccupied;
}

double GroundProjection::crossSlope(const std::vector<Eigen::Vector3d>& edge,
                                    const Eigen::Vector3d& box_size,
                                    bool skip_start) const {
  if (edge.size() < 2) return 0.0;
  const Eigen::Vector2d travel = (edge.back() - edge.front()).head<2>();
  const double half_track = 0.5 * std::min(box_size.x(), box_size.y());
  if (travel.norm() < 1e-9 || !(half_track > 0.0)) return 0.0;
  const Eigen::Vector2d unit = travel.normalized();
  const Eigen::Vector2d left_unit(-unit.y(), unit.x());
  const Eigen::Vector3d side(half_track * left_unit.x(),
                             half_track * left_unit.y(), 0.0);

  // The gradient from the right side to the left, per point that has ground
  // under both. It is taken between the ground points the map returns, not
  // the probes: a voxel map reports its cell's centre, and the cells either
  // side of a 0.83 m track may be 0.8 or 1.0 m apart.
  std::vector<Eigen::Vector2d> positions;
  std::vector<double> gradients;
  for (std::size_t i = skip_start ? 1 : 0; i < edge.size(); ++i) {
    const Eigen::Vector3d& point = edge[i];
    Eigen::Vector3d left;
    Eigen::Vector3d right;
    if (!groundBelow(point + side, left) || !groundBelow(point - side, right)) {
      continue;
    }
    const double across = (left - right).head<2>().dot(left_unit);
    if (across < half_track) continue;  // both sides in one cell
    positions.push_back(point.head<2>());
    gradients.push_back((left.z() - right.z()) / across);
  }
  const double body = std::max(box_size.x(), box_size.y());
  double steepest = 0.0;
  for (std::size_t first = 0; first < gradients.size(); ++first) {
    double sum = 0.0;
    std::size_t next = first;
    while (next < gradients.size() &&
           (positions[next] - positions[first]).norm() <= body + 1e-9) {
      sum += gradients[next++];
    }
    steepest = std::max(
        steepest, std::abs(sum / static_cast<double>(next - first)));
    // Every later run is part of this one.
    if (next == gradients.size()) break;
  }
  return std::atan(steepest);
}

namespace {

/// A length or angle as a whole number of millionths, for a cache key.
std::int64_t micro(double value) {
  return static_cast<std::int64_t>(std::llround(value * 1e6));
}

}  // namespace

bool GroundProjection::footprintGroundBelow(const Eigen::Vector3d& point,
                                            Eigen::Vector3d& ground) const {
  if (!cache_footprint_ground_) return groundBelow(point, ground);
  std::vector<GroundFromHeight>& column =
      ground_below_column_[ColumnKey{micro(point.x()), micro(point.y())}];
  for (const GroundFromHeight& earlier : column) {
    // A ray that starts between an earlier ray's start and the ground it
    // found crosses only cells that ray found empty, then that ground; its
    // end, 5 m below its start, is still below that ground. The ground
    // point lies in the cell that stopped the ray, so a start at or above
    // it is not past that cell. With no ground found, only the same start
    // gives the same answer: a lower ray reaches further down.
    const bool same = earlier.found ? earlier.ground.z() <= point.z() &&
                                          point.z() <= earlier.from_z
                                    : point.z() == earlier.from_z;
    if (same) {
      ground = earlier.ground;
      return earlier.found;
    }
  }
  GroundFromHeight cast;
  cast.from_z = point.z();
  cast.found = groundBelow(point, cast.ground);
  column.push_back(cast);
  ground = cast.ground;
  return cast.found;
}

FootprintPlane GroundProjection::footprintPlane(
    const Eigen::Vector3d& point, const Eigen::Vector2d& heading,
    const Eigen::Vector3d& box_size) const {
  if (!cache_footprint_ground_ || !point.allFinite() ||
      !(heading.norm() > 1e-9)) {
    return measureFootprintPlane(point, heading, box_size);
  }
  // The body's axis, folded into [0, pi): a heading and its reverse put the
  // footprint on the same cells.
  double axis = std::atan2(heading.y(), heading.x());
  if (axis < 0.0) axis += M_PI;
  if (axis >= M_PI) axis -= M_PI;
  const PlaneKey key{micro(point.x()), micro(point.y()), micro(point.z()),
                     micro(axis),      micro(box_size.x()),
                     micro(box_size.y())};
  const auto found = footprint_planes_.find(key);
  if (found != footprint_planes_.end()) return found->second;
  const FootprintPlane plane = measureFootprintPlane(point, heading, box_size);
  footprint_planes_.emplace(key, plane);
  return plane;
}

FootprintPlane GroundProjection::measureFootprintPlane(
    const Eigen::Vector3d& point, const Eigen::Vector2d& heading,
    const Eigen::Vector3d& box_size) const {
  FootprintPlane plane;
  const double half_length = 0.5 * box_size.x();
  const double half_width = 0.5 * box_size.y();
  if (!point.allFinite() || !(heading.norm() > 1e-9) ||
      !(half_length > 0.0) || !(half_width > 0.0)) {
    return plane;
  }
  const Eigen::Vector2d along = heading.normalized();
  const Eigen::Vector2d across(-along.y(), along.x());

  // Every map cell whose centre lies under the footprint, whatever the
  // footprint's heading. A lattice of probes rotated against the map's grid
  // can step over a cell entirely, and a rock in it (review r0).
  std::vector<XYCellCenter> candidates;
  if (!map_.getCircleIntersectingXYCellCenters(
          point.head<2>(), std::hypot(half_length, half_width),
          kMaxFootprintCells, candidates)) {
    return plane;
  }
  std::vector<Eigen::Vector3d> ground_points;
  ground_points.reserve(candidates.size());
  int cells_under = 0;
  for (const XYCellCenter& cell : candidates) {
    const Eigen::Vector2d offset = cell.center - point.head<2>();
    if (std::abs(offset.dot(along)) > half_length + 1e-9 ||
        std::abs(offset.dot(across)) > half_width + 1e-9) {
      continue;
    }
    ++cells_under;
    Eigen::Vector3d ground;
    if (footprintGroundBelow(Eigen::Vector3d(cell.center.x(),
                                             cell.center.y(), point.z()),
                             ground)) {
      ground_points.push_back(ground);
    }
  }
  plane.cells = static_cast<int>(ground_points.size());
  // With ground under fewer than half the cells, a plane would describe
  // only part of the body.
  if (2 * plane.cells < cells_under || plane.cells < 3) return plane;

  // z = c + a x + b y in coordinates centred on `point`, for conditioning,
  // solved from the 3 x 3 normal equations.
  auto row = [&](const Eigen::Vector3d& g) {
    return Eigen::Vector3d(1.0, g.x() - point.x(), g.y() - point.y());
  };
  Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
  Eigen::Vector3d moment = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d& g : ground_points) {
    const Eigen::Vector3d r = row(g);
    normal.noalias() += r * r.transpose();
    moment += r * g.z();
  }
  Eigen::FullPivLU<Eigen::Matrix3d> lu(normal);
  lu.setThreshold(1e-9);
  if (lu.rank() < 3) return plane;  // the points lie on a line
  const Eigen::Vector3d coefficients = lu.solve(moment);
  plane.measured = true;
  plane.tilt = std::atan(coefficients.tail<2>().norm());
  for (const Eigen::Vector3d& g : ground_points) {
    plane.max_residual = std::max(
        plane.max_residual, std::abs(g.z() - row(g).dot(coefficients)));
  }
  return plane;
}

}  // namespace mgg
