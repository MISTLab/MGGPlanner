#include "mgg_core/planning_cancellation.h"
#include "mgg_core/ground_projection.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
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

/// A length or angle as a whole number of millionths, for a cache key.
std::int64_t micro(double value) {
  return static_cast<std::int64_t>(std::llround(value * 1e6));
}

std::int64_t projectionBits(double value) {
  std::int64_t bits;
  if (value == 0.0) value = 0.0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

}  // namespace

double GroundProjection::projectSample(Eigen::Vector3d& sample,
                                       VoxelStatus& status) const {
  ProfileScope timed(profile_ ? &profile_->projection : nullptr);
  // The same sample again in one plan, from the same map: the same answer
  // (a lattice cell is projected by the sweep, the offer and expandGraph).
  if (!cache_footprint_ground_ || !sample.allFinite()) {
    return castProjection(sample, status);
  }
  const ProjectionKey key{projectionBits(sample.x()), projectionBits(sample.y()),
                          projectionBits(sample.z())};
  const auto known = projections_.find(key);
  if (known != projections_.end()) {
    if (profile_ != nullptr) ++profile_->projection_cache_hits;
    sample = known->second.sample;
    status = known->second.status;
    return known->second.below;
  }
  Projection projection;
  projection.below = castProjection(sample, status);
  projection.sample = sample;
  projection.status = status;
  projections_.emplace(key, projection);
  return projection.below;
}

double GroundProjection::castProjection(Eigen::Vector3d& sample,
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


  // With the per-plan column cache, the central ray is the one footprint
  // checks cast down the same column (footprintGroundBelow): a lattice
  // column's z levels, an edge's endpoints and a footprint's cells meet the
  // same ground. Only a found ground is taken from it; otherwise every ray
  // is cast as below.
  if (cache_footprint_ground_ && sample.allFinite()) {
    Eigen::Vector3d ground;
    // The column cache keeps the map's own answer; a floating solid it
    // found is passed below, as every ray does.
    if (footprintGroundBelow(sample + extra_samples[0], ground) &&
        !floatsAbove(ground, sample(2))) {
      status = VoxelStatus::kOccupied;
      return sample(2) - ground(2);
    }
  }

  // The offset probes start slightly higher as well as to the side, so
  // the drop has to be measured from the sample itself rather than from where
  // the ray began. The ROS 1 code measured from the ray's own start, which
  // makes an offset hit report extra clearance; callers then place the point
  // too low. This function's contract is "how far below `sample` the ground
  // lies", so measure that.
  const double sample_z = sample(2);

  const std::size_t rays = offset_probes_ ? extra_samples.size() : 1;
  for (size_t i = 0; i < rays; ++i) {
    checkpoint_.check();
    const Eigen::Vector3d start = sample + extra_samples[i];
    const Eigen::Vector3d end =
        start - Eigen::Vector3d(0.0, 0.0, max_projection_length);

    Eigen::Vector3d end_voxel;
    ProfileScope timed_ray(profile_ ? &profile_->ground_rays : nullptr);
    // Ground rays may cross unobserved air above the lidar. Ordinary body and edge
    // collision checks still run; only known occupied ground can support a
    // projected point.
    const VoxelStatus vs = groundRay(start, end, sample_z, end_voxel);

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

  if (unknown_count >= static_cast<int>(rays)) {
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

bool GroundProjection::groundStepsAdmissible(
    const std::vector<Eigen::Vector3d>& sweep_path,
    const Eigen::Vector3d* physical_root) const {
  const double resolution = map_.getResolution();
  if (!(resolution > 0.0) || sweep_path.size() < 2) return true;
  // Restore the physical start for terrain only; never move the collision
  // sweep back into the robot's exempt footprint or apply its body offset
  // to the robot's ground evidence.
  std::vector<Eigen::Vector3d> root_path;
  bool hanging_root = false;
  if (physical_root != nullptr) {
    if (!physical_root->allFinite()) return false;
    Eigen::Vector3d sample = *physical_root;
    VoxelStatus status = VoxelStatus::kUnknown;
    projectSample(sample, status);
    hanging_root = status != VoxelStatus::kOccupied;
    if (hanging_root) {
      root_path = sweep_path;
      root_path.front() = *physical_root;
    }
  }
  const auto& path = hanging_root ? root_path : sweep_path;
  std::vector<double> along(path.size(), 0.0);  // distance driven to each point
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (!path[i].allFinite()) return false;
    if (i > 0) along[i] = along[i - 1] + (path[i] - path[i - 1]).head<2>().norm();
  }
  const double length = along.back();
  const double baseline = 2.0 * resolution;
  // The ground under the path `s` along it, if known.
  const auto ground_at = [&](double s, double& ground) {
    checkpoint_.check();
    const std::size_t i = std::clamp<std::size_t>(
        std::upper_bound(along.begin(), along.end(), s) - along.begin(), 1,
        path.size() - 1);
    const double segment = along[i] - along[i - 1];
    const double t = segment > 1e-12 ? std::clamp((s - along[i - 1]) / segment, 0.0, 1.0)
                                     : 1.0;
    Eigen::Vector3d sample = path[i - 1] + (path[i] - path[i - 1]) * t;
    const double sample_z = sample.z();
    VoxelStatus status = VoxelStatus::kUnknown;
    const double below = projectSample(sample, status);
    ground = sample_z - below;
    return status == VoxelStatus::kOccupied;
  };
  // Sampling is along the whole polyline, independent of serialized pose
  // spacing. Include the endpoint even when it falls between cell samples.
  std::vector<double> samples;
  for (double s = 0.0; s < length - 1e-9; s += resolution) samples.push_back(s);
  samples.push_back(length);
  std::vector<double> ground(samples.size(), 0.0);
  std::vector<bool> known(samples.size(), false);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (i == 0 && hanging_root) {
      // The root's own ground, under the robot where the map has none.
      ground[0] = physical_root->z() - params_.max_ground_height;
      known[0] = true;
      continue;
    }
    known[i] = ground_at(samples[i], ground[i]);
  }
  // Every native-length window includes its interior, not just its ends.
  // Starting at each sample covers every distinct set of sampled heights.
  // Clipped windows (including a path shorter than two cells) retain the
  // native denominator: a quantised riser is not a raw adjacent-pose grade.
  for (std::size_t i = 0; i < samples.size(); ++i) {
    checkpoint_.check();
    double low = std::numeric_limits<double>::infinity();
    double high = -std::numeric_limits<double>::infinity();
    for (std::size_t j = i; j < samples.size() && samples[j] - samples[i] <= baseline + 1e-9; ++j) {
      if (!known[j]) continue;
      low = std::min(low, ground[j]);
      high = std::max(high, ground[j]);
    }
    const double rise = high - low;
    if (rise > params_.max_step_height + 1e-6 &&
        std::atan2(rise, baseline) > params_.max_inclination) return false;
  }
  return true;
}

ProjectedEdgeStatus GroundProjection::getProjectedEdgeStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel,
    std::vector<Eigen::Vector3d>& projected_edge_out, bool is_hanging,
    bool preserve_start_height, const EdgeBodyCheck* body,
    EdgeTravel travel, const Eigen::Vector3d* physical_root) const {
  ProfileScope timed(profile_ ? &profile_->edge_checks : nullptr);
  const double step_size = 2.0 * map_.getResolution();
  // The same projected-ground rule as final sent paths, including short
  // lattice edges. Endpoint pose heights are not a separate terrain rule,
  // except a preserved root's, which is its only ground evidence.
  if (!groundStepsAdmissible(start, end, physical_root)) {
    return ProjectedEdgeStatus::kSteep;
  }

  const Eigen::Vector3d ray = end - start;

  const double ray_len = ray.norm();
  if (ray_len < 1e-12) return ProjectedEdgeStatus::kAdmissible;
  const Eigen::Vector3d ray_normed = ray / ray_len;

  std::vector<Eigen::Vector3d> projected_edge;

  if (ray_len >= 2.0 * step_size) {
    Eigen::Vector3d last_point = start;
    for (double step = 0.0; step < ray_len; step += step_size) {
      checkpoint_.check();
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


  for (size_t i = 1; i < projected_edge.size(); ++i) {
    checkpoint_.check();
    ProfileScope timed_sweep(profile_ ? &profile_->body_sweeps : nullptr);
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
  if (params_.max_cross_slope < M_PI_2) {
    ProfileScope timed_slope(profile_ ? &profile_->cross_slope : nullptr);
    if (crossSlope(projected_edge, box_size, standing_at_start) >
        params_.max_cross_slope) {
      return ProjectedEdgeStatus::kCrossSlope;
    }
  }

  // The points the body is checked at: every point of the polyline and
  // between them, at most kFootprintSampleSpacing apart. A short edge's
  // polyline is its two ends, 0.4 to 0.57 m apart on the lattice, and which
  // of a rock's cells fell under a footprint then depended on where the
  // lattice lay (run 5, robot_2). Not the start where the robot stands.
  std::vector<Eigen::Vector3d> samples;
  for (std::size_t i = 1; i < projected_edge.size(); ++i) {
    checkpoint_.check();
    const Eigen::Vector3d& from = projected_edge[i - 1];
    const Eigen::Vector3d& to = projected_edge[i];
    const int count = std::max(
        1, static_cast<int>(std::ceil((to - from).head<2>().norm() /
                                          kFootprintSampleSpacing -
                                      1e-9)));
    for (int k = (i == 1 && standing_at_start) ? 1 : 0; k < count; ++k) {
      checkpoint_.check();
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
    ProfileScope timed_footprint(profile_ ? &profile_->footprint : nullptr);
    for (const Eigen::Vector3d& point : samples) {
      checkpoint_.check();
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
    ProfileScope timed_ahead(profile_ ? &profile_->ground_ahead : nullptr);
    for (std::size_t i = 0; i < samples.size(); ++i) {
      checkpoint_.check();
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

/// The map's XY cell grid round one footprint, from the cells
/// getCircleIntersectingXYCellCenters listed for it: a cell centre and one
/// cell's step along each of the grid's axes, which a map whose grid is
/// turned against the planner's frame (MolaMap) turns with it. A point is
/// taken to the centre of the cell it lies in, so that the ground under it
/// is looked up in the per-plan column cache as a footprint's own cells
/// are. Found from the cells when first needed.
struct GroundProjection::BridgeCells {
  explicit BridgeCells(const std::vector<XYCellCenter>& cells)
      : cells_(cells) {}

  /// Whether the grid's axes were found.
  bool valid() {
    if (!found_) find();
    return valid_;
  }

  /// The centre of the cell `point` lies in; valid() first.
  Eigen::Vector2d centerOf(const Eigen::Vector2d& point) const {
    const Eigen::Vector2d offset = point - origin_;
    return origin_ +
           std::round(offset.dot(step_x_) / step_x_.squaredNorm()) * step_x_ +
           std::round(offset.dot(step_y_) / step_y_.squaredNorm()) * step_y_;
  }

 private:
  void find() {
    found_ = true;
    // A cell with both its next neighbours listed; the cells lie in a
    // circle, so most have them.
    for (const XYCellCenter& cell : cells_) {
      planningCheckpoint();
      const XYCellCenter* next_x = nullptr;
      const XYCellCenter* next_y = nullptr;
      for (const XYCellCenter& other : cells_) {
        planningCheckpoint();
        if (other.grid_x == cell.grid_x + 1 && other.grid_y == cell.grid_y) {
          next_x = &other;
        } else if (other.grid_x == cell.grid_x &&
                   other.grid_y == cell.grid_y + 1) {
          next_y = &other;
        }
      }
      if (next_x == nullptr || next_y == nullptr) continue;
      origin_ = cell.center;
      step_x_ = next_x->center - cell.center;
      step_y_ = next_y->center - cell.center;
      valid_ = step_x_.squaredNorm() > 1e-12 && step_y_.squaredNorm() > 1e-12;
      return;
    }
  }

  const std::vector<XYCellCenter>& cells_;
  bool found_ = false;
  bool valid_ = false;
  Eigen::Vector2d origin_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d step_x_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d step_y_ = Eigen::Vector2d::Zero();
};

double GroundProjection::observedGroundAhead(
    const Eigen::Vector3d& point, const Eigen::Vector2d& heading,
    const Eigen::Vector3d& box_size) const {
  checkpoint_.check();
  if (!cache_footprint_ground_ || !point.allFinite() || !heading.allFinite() ||
      !(heading.norm() > 1e-9)) return measureObservedGroundAhead(point, heading, box_size);
  const Eigen::Vector2d along = heading.normalized();
  const GroundAheadKey key{projectionBits(point.x()), projectionBits(point.y()),
      projectionBits(point.z()), projectionBits(along.x()), projectionBits(along.y()),
      projectionBits(box_size.x()), projectionBits(box_size.y()),
      projectionBits(params_.max_ground_height), projectionBits(params_.max_inclination)};
  const auto found = ground_ahead_cache_.find(key);
  if (found != ground_ahead_cache_.end()) {
    if (profile_) ++profile_->ground_ahead_cache_hits;
    return found->second;
  }
  const double fraction = measureObservedGroundAhead(point, heading, box_size);
  ground_ahead_cache_.emplace(key, fraction);
  return fraction;
}

double GroundProjection::measureObservedGroundAhead(
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
  BridgeCells bridge(candidates);
  int ahead = 0;
  int observed = 0;
  for (const XYCellCenter& cell : candidates) {
    checkpoint_.check();
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
    } else if (groundBridged(bridge, cell.center, along, point.z(),
                             lowest)) {
      ++observed;
    }
  }
  return ahead > 0 ? static_cast<double>(observed) / ahead : 0.0;
}

bool GroundProjection::groundBridged(BridgeCells& cells,
                                     const Eigen::Vector2d& cell,
                                     const Eigen::Vector2d& along,
                                     double from_z, double lowest) const {
  const double resolution = map_.getResolution();
  const double grade = std::tan(params_.max_inclination);
  if (!(resolution > 0.0) || !std::isfinite(grade) || grade < 0.0 ||
      !cells.valid()) {
    return false;
  }
  // The observed ground nearest the cell on one side of it, within
  // kGroundBridgeCells. k cells out, the window reaches k cells' rise at
  // max_inclination further down than the footprint's.
  struct Side {
    bool found = false;
    Eigen::Vector2d at = Eigen::Vector2d::Zero();
    double z = 0.0;
  };
  const auto nearest = [&](const Eigen::Vector2d& direction) {
    Side side;
    for (int k = 1; k <= kGroundBridgeCells; ++k) {
      checkpoint_.check();
      const Eigen::Vector2d at =
          cells.centerOf(cell + k * resolution * direction);
      Eigen::Vector3d ground;
      if (footprintGroundBelow(Eigen::Vector3d(at.x(), at.y(), from_z),
                               ground) &&
          ground.z() >= lowest - k * resolution * grade) {
        side.found = true;
        side.at = at;
        side.z = ground.z();
        break;
      }
    }
    return side;
  };
  const Eigen::Vector2d across(-along.y(), along.x());
  for (const Eigen::Vector2d& direction : {along, across}) {
    checkpoint_.check();
    const Side ahead = nearest(direction);
    if (!ahead.found) continue;
    const Side behind = nearest(-direction);
    if (!behind.found) continue;
    const double to_ahead = (ahead.at - cell).norm();
    const double to_behind = (behind.at - cell).norm();
    const double rise = ahead.z - behind.z;
    if (std::abs(rise) > (to_ahead + to_behind) * grade + 1e-9) continue;
    const double bridged = behind.z + rise * to_behind / (to_ahead + to_behind);
    // A free voxel in the column from kGroundBridgeHoleDepth to
    // max_projection_length under the bridged ground is a hole the lidar
    // looked into (review r1, P2: 5.0 m, not 5.3).
    const bool hole = freeInColumn(cell, bridged - kGroundBridgeHoleDepth,
                                   bridged - max_projection_length);
    if (!hole) return true;
  }
  return false;
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
    checkpoint_.check();
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
    checkpoint_.check();
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
  ProfileScope timed_ray(profile_ ? &profile_->ground_rays : nullptr);
  return map_.getGroundRayStatus(point, end, false, ground) ==
         VoxelStatus::kOccupied;
}

std::optional<Eigen::Vector3d> GroundProjection::floatsAbove(
    const Eigen::Vector3d& hit, double level) const {
  const double resolution = map_.getResolution();
  if (!supported_ground_only_ || !(resolution > 0.0) ||
      !map_.observesFreeSpace())
    return std::nullopt;
  const Eigen::Vector3d under(
      hit.x(), hit.y(), (std::floor(hit.z() / resolution) - 0.5) * resolution);
  if (!(under.z() > level) || map_.getVoxelStatus(under) != VoxelStatus::kFree)
    return std::nullopt;
  return under;
}

VoxelStatus GroundProjection::groundRay(const Eigen::Vector3d& start,
                                        const Eigen::Vector3d& end,
                                        double level,
                                        Eigen::Vector3d& end_voxel) const {
  VoxelStatus status = map_.getGroundRayStatus(start, end, false, end_voxel);
  // Each pass looks on from the free voxel under one floating solid,
  // strictly lower each time, until the ray's end.
  double lowest = start.z();
  while (status == VoxelStatus::kOccupied) {
    checkpoint_.check();
    const auto under = floatsAbove(end_voxel, level);
    if (!under || !(under->z() < lowest)) break;
    lowest = under->z();
    if (under->z() <= end.z()) {
      end_voxel = end;
      return VoxelStatus::kFree;
    }
    status = map_.getGroundRayStatus(*under, end, false, end_voxel);
  }
  return status;
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
    checkpoint_.check();
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
    checkpoint_.check();
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


bool GroundProjection::footprintGroundBelow(const Eigen::Vector3d& point,
                                            Eigen::Vector3d& ground) const {
  if (!cache_footprint_ground_) return groundBelow(point, ground);
  std::vector<GroundFromHeight>& column =
      ground_below_column_[ColumnKey{projectionBits(point.x()), projectionBits(point.y())}];
  for (const GroundFromHeight& earlier : column) {
    checkpoint_.check();
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

bool GroundProjection::freeInColumn(const Eigen::Vector2d& cell, double top,
                                    double bottom) const {
  const double resolution = map_.getResolution();
  if (!(resolution > 0.0) || !std::isfinite(top) || !std::isfinite(bottom) ||
      !(bottom < top)) {
    return false;
  }
  // The column is sampled once per voxel, at heights (k + 1/2) resolution,
  // so that cached and uncached asks see the same samples. A voxel counts
  // when its sample lies between the bounds to within kSampleTolerance, and
  // the samples looked at are enumerated to that same tolerance: a sample
  // on a bound counts in either mode (review r2, P2).
  constexpr double kSampleTolerance = 1e-9;
  const auto sample = [&](double from, double to, std::vector<double>& free) {
    const auto highest = static_cast<std::int64_t>(
        std::floor((from + kSampleTolerance) / resolution - 0.5));
    const auto lowest = static_cast<std::int64_t>(
        std::ceil((to - kSampleTolerance) / resolution - 0.5));
    for (std::int64_t k = highest; k >= lowest; --k) {
      checkpoint_.check();
      const double z = (static_cast<double>(k) + 0.5) * resolution;
      if (map_.getVoxelStatus(Eigen::Vector3d(cell.x(), cell.y(), z)) ==
          VoxelStatus::kFree) {
        free.push_back(z);
      }
    }
  };
  const auto between = [&](double z) {
    return z <= top + kSampleTolerance && z >= bottom - kSampleTolerance;
  };
  if (!cache_footprint_ground_) {
    std::vector<double> free;
    sample(top, bottom, free);
    return std::any_of(free.begin(), free.end(), between);
  }
  // A column is asked about again from nearly the same height, for each
  // footprint and neighbour that bridges it: sampled once with a metre to
  // spare either way.
  FreeInColumn& column =
      free_in_column_[ColumnKey{micro(cell.x()), micro(cell.y())}];
  if (!(column.top >= top && column.bottom <= bottom)) {
    column.top = top + 1.0;
    column.bottom = bottom - 1.0;
    column.free_z.clear();
    sample(column.top, column.bottom, column.free_z);
  }
  return std::any_of(column.free_z.begin(), column.free_z.end(), between);
}

bool GroundProjection::clearanceHazard(const Eigen::Vector3d& cell,
                                       const Eigen::Vector3d& box_size) const {
  const double resolution = map_.getResolution();
  const auto height_bin = static_cast<std::int64_t>(std::floor(cell.z() / resolution));
  // Canonical voxel-centre height makes reuse independent of query order.
  // Quantisation is only for the soft cost; hard body/terrain checks are exact.
  Eigen::Vector3d sample = cell;
  sample.z() = (static_cast<double>(height_bin) + 0.5) * resolution;
  const ClearanceKey key{micro(cell.x()), micro(cell.y()), height_bin,
                     micro(box_size.z()), micro(params_.max_footprint_cell_rise),
                     micro(params_.max_ground_height)};
  if (cache_footprint_ground_) {
    const auto found = clearance_hazards_.find(key);
    if (found != clearance_hazards_.end()) return found->second;
  }
  bool hazard = map_.getBoxStatus(
      sample, {resolution, resolution, box_size.z()}, false) ==
      VoxelStatus::kOccupied;
  Eigen::Vector3d ground;
  if (!hazard && params_.max_footprint_cell_rise > 0.0 &&
      footprintGroundBelow(sample, ground)) {
    // Only the raised cell is a hazard, not its lower, drivable neighbour.
    // Compare actual support heights, not height above the robot: smooth
    // ramps keep their clearance and floors on another level do not alias.
    for (const Eigen::Vector2d& offset :
         {Eigen::Vector2d(resolution, 0), Eigen::Vector2d(-resolution, 0),
          Eigen::Vector2d(0, resolution), Eigen::Vector2d(0, -resolution)}) {
      Eigen::Vector3d neighbour;
      if (footprintGroundBelow(sample + Eigen::Vector3d(offset.x(), offset.y(), 0),
                               neighbour) &&
          ground.z() - neighbour.z() > params_.max_footprint_cell_rise) {
        hazard = true;
        break;
      }
    }
  }
  if (cache_footprint_ground_) clearance_hazards_.emplace(key, hazard);
  return hazard;
}

double GroundProjection::clearancePenalty(
    const Eigen::Vector3d& point, const Eigen::Vector2d& heading,
    const Eigen::Vector3d& box_size) const {
  // A cap bounds compute work even for a malformed deployment setting.
  const double margin = std::min(params_.path_clearance_margin, 1.0);
  if (!(margin > 0.0)) return 0.0;
  const double body_radius = 0.5 * box_size.head<2>().norm();
  const double cell_radius = map_.getResolution() / std::sqrt(2.0);
  std::vector<XYCellCenter> cells;
  // The enumerator bounds visited cells as well as output: botman's 0.1 m
  // grid needs a padded 35x35 box here. A cap of 1024 silently made every
  // clear-floor query report zero clearance, defeating direct shortcuts.
  if (!map_.getCircleIntersectingXYCellCenters(
          point.head<2>(), body_radius + margin + cell_radius, 2048, cells)) {
    // Unmeasurable clearance is the largest finite preference cost, not a
    // new collision rule. Even a very fine map can still use the passage.
    return 1.0;
  }
  const Eigen::Vector2d along =
      heading.norm() > 1e-9 ? Eigen::Vector2d(heading.normalized())
                            : Eigen::Vector2d::UnitX();
  const Eigen::Vector2d across(-along.y(), along.x());
  const Eigen::Vector2d half = 0.5 * box_size.head<2>();
  double clearance = margin;
  for (const auto& cell : cells) {
    checkpoint_.check();
    const Eigen::Vector2d offset = cell.center - point.head<2>();
    const Eigen::Vector2d local(std::abs(offset.dot(along)),
                                std::abs(offset.dot(across)));
    const double distance = std::max(
        0.0, (local - half).cwiseMax(0.0).norm() - cell_radius);
    if (distance >= clearance) continue;
    if (clearanceHazard({cell.center.x(), cell.center.y(), point.z()}, box_size)) {
      clearance = distance;
      if (clearance == 0.0) break;
    }
  }
  return 1.0 - clearance / margin;
}

std::vector<Eigen::Vector3d> GroundProjection::clearanceSamples(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    int maximum_samples) const {
  const double length = (end - start).norm();
  const int samples = std::max(1, std::min(std::clamp(maximum_samples, 1, 32),
      static_cast<int>(std::ceil(std::min(length, 6.4) / 0.2 - 1e-9))));
  std::vector<Eigen::Vector3d> points;
  points.reserve(samples);
  for (int i = 0; i < samples; ++i) {
    checkpoint_.check();
    Eigen::Vector3d point = start + ((i + 0.5) / samples) * (end - start);
    // Follow the support surface even on a shortcut spanning a ramp crest.
    Eigen::Vector3d ground;
    if (footprintGroundBelow(point, ground)) {
      point.z() = ground.z() + params_.max_ground_height;
    }
    points.push_back(point);
  }
  return points;
}

double GroundProjection::clearanceCost(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size, int maximum_samples) const {
  ProfileScope timed(profile_ ? &profile_->clearance : nullptr);
  const double length = (end - start).norm();
  if (!(params_.path_clearance_margin > 0.0) || !(length > 0.0)) return length;
  const Eigen::Vector2d heading = (end - start).head<2>();
  const std::vector<Eigen::Vector3d> points = clearanceSamples(start, end, maximum_samples);
  double penalty = 0.0;
  for (const Eigen::Vector3d& point : points) {
    penalty += clearancePenalty(point, heading, box_size);
  }
  return length * (1.0 + 4.0 * penalty / static_cast<double>(points.size()));
}

double GroundProjection::segmentClearance(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size) const {
  ProfileScope timed(profile_ ? &profile_->clearance : nullptr);
  const double margin = std::min(params_.path_clearance_margin, 1.0);
  if (!(margin > 0.0)) return 0.0;
  const Eigen::Vector2d heading = (end - start).head<2>();
  double worst = 0.0;
  for (const Eigen::Vector3d& point : clearanceSamples(start, end)) {
    worst = std::max(worst, clearancePenalty(point, heading, box_size));
    if (worst >= 1.0) break;
  }
  return margin * (1.0 - worst);
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
  if (found != footprint_planes_.end()) {
    if (profile_ != nullptr) ++profile_->footprint_cache_hits;
    return found->second;
  }
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
    checkpoint_.check();
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
    checkpoint_.check();
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
    checkpoint_.check();
    plane.max_residual = std::max(
        plane.max_residual, std::abs(g.z() - row(g).dot(coefficients)));
  }
  return plane;
}

}  // namespace mgg
