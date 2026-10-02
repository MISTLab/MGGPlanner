// Dropping samples and edges onto the ground, for wheeled and legged robots.
//
// Ported from Rrg::projectSample and Rrg::getProjectedEdgeStatus. Both depend
// only on the map and the planning parameters, so they move across cleanly.
//
// Two defects in the originals are fixed here:
//
//   * The enum value was spelled kOccipied.
//   * getProjectedEdgeStatus dropped a trailing point with
//     "projected_edge.erase(projected_edge.end())". Erasing end() is
//     undefined behaviour: std::vector::erase requires a dereferenceable
//     iterator, which end() is not. libstdc++ happens to treat it as
//     pop_back(), so it worked, but a build with _GLIBCXX_DEBUG asserts on
//     it. It is pop_back() here.

#ifndef MGG_CORE_GROUND_PROJECTION_H_
#define MGG_CORE_GROUND_PROJECTION_H_

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/planning_cancellation.h"

#include "mgg_core/map_interface.h"
#include "mgg_core/params.h"
#include "mgg_core/plan_profile.h"
#include "mgg_core/types.h"

namespace mgg {

enum class ProjectedEdgeStatus {
  kAdmissible = 0,
  kSteep,      ///< exceeds max_inclination
  kOccupied,   ///< hits an obstacle
  kUnknown,    ///< passes through unmapped space
  kHanging,    ///< no ground beneath it
  kCrossSlope,  ///< its ground slopes sideways past max_cross_slope
  kFootprintPlane,  ///< the ground under its footprint tilts past
                    ///< max_footprint_tilt or steps past max_footprint_step
  kGroundUnobserved,  ///< too little of the ground ahead of its footprint
                      ///< is observed (min_observed_ground_fraction)
};

/// The plane fitted to the ground under a robot's footprint at one point.
struct FootprintPlane {
  /// False when too few ground cells were found to fit a plane.
  bool measured = false;
  /// Angle between the plane and the horizontal, radians.
  double tilt = 0.0;
  /// Largest vertical distance of a ground cell from the plane, metres.
  double max_residual = 0.0;
  /// Distinct ground cells the plane was fitted to.
  int cells = 0;
};

/// The farthest apart, metres, that getProjectedEdgeStatus measures the
/// footprint plane along an edge: a map cell of MGG's 0.2 m MOLA grid.
inline constexpr double kFootprintSampleSpacing = 0.2;

/// The most PlanningParams::max_goal_ground_rise may reach, metres: about
/// two storeys, so a goal never snaps to a floor far above it.
inline constexpr double kMaxGoalGroundRise = 6.0;

/// How many map cells to either side of a cell with no observed ground
/// observedGroundAhead looks for observed ground to bridge it with: 0.6 m
/// on MGG's 0.2 m grid.
inline constexpr int kGroundBridgeCells = 3;

/// How far below the ground bridged over a cell, metres, a free voxel in
/// its column shows a hole the lidar looked into, which is not bridged. The
/// column is looked at from there down as far as ground projection looks
/// (GroundProjection::max_projection_length).
inline constexpr double kGroundBridgeHoleDepth = 0.3;

/// The collision check of the body moved straight between two points at
/// driving height, as MapInterface::getPathStatus checks a box.
using SegmentSweepFn = std::function<VoxelStatus(const Eigen::Vector3d&,
                                                 const Eigen::Vector3d&)>;

/// Which ways a robot may drive an edge getProjectedEdgeStatus checks, for
/// the ground it must have observed ahead (min_observed_ground_fraction).
enum class EdgeTravel {
  /// Either way: a graph edge, which is undirected, and routed over in
  /// whichever direction a path needs it.
  kBothWays,
  /// From `start` to `end` only: a departure, a shortcut segment, an edge
  /// out of the robot's own position.
  kForward,
};

/// Where a ground robot stands before it has moved: the disk of `radius`
/// round `center` (x, y) that its lidar cannot see the ground in, its
/// initial ground reach (the planner node's hanging_root_edge_length_max).
/// In run 6 robot_3 (Spot) stood 2.4 m from observed ground on every side
/// for the whole run, and robot_1 (Bunker) for 18 minutes: no way out
/// crossed observed ground (min_observed_ground_fraction), so they were
/// boxed in where they had been placed. The robot rests on that ground, so
/// in the disk a map cell with no ground found under it counts as observed
/// ground (observedGroundAhead, turnSpaceObserved); ground observed too far
/// down is still a drop. Elsewhere unknown stays unknown.
struct StandingStart {
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  double radius = 0.0;
  /// Whether `point` (x, y) lies in the disk.
  bool covers(const Eigen::Vector2d& point) const {
    return (point - center).norm() <= radius;
  }
  /// A first goal must not stop in the prior's disk after the allowance
  /// expires en route, including an early controller arrival.
  bool admitsGoal(const Eigen::Vector2d& point, double arrival_tolerance) const {
    return (point - center).norm() > radius + std::max(0.0, arrival_tolerance) + 1e-9;
  }
};

/// How getProjectedEdgeStatus checks the body on an edge other than with
/// the map's box sweep, as a boxed-in robot departs (findDeparture).
struct EdgeBodyCheck {
  /// Checks each segment of the edge instead of the map's box sweep.
  SegmentSweepFn sweep;
  /// The robot stands at the edge's start: neither the cross slope nor the
  /// footprint plane is measured there, where it already is.
  bool standing_at_start = false;
};

class GroundProjection {
 public:
  /// With `cache_footprint_ground`, footprintPlane remembers two things and
  /// reuses them for every later edge:
  ///   * the ground found under each map cell, with the height its ray
  ///     started from. A later ray down the same column that starts no
  ///     higher, and not below that ground, passes only cells the first
  ///     one found empty before it reached the ground, so it is not cast;
  ///   * the plane at each point for each body axis (a heading and its
  ///     reverse cover the same cells).
  /// A lattice checks each vertex's footprint from every edge that meets
  /// it, so this is most of the check's cost. The cache is only valid while
  /// the map stays the same. Build such a GroundProjection for one plan,
  /// while holding the map's read lease, and discard it with the plan. It
  /// is not thread-safe.
  GroundProjection(const MapInterface& map, const PlanningParams& params,
                   bool cache_footprint_ground = false)
      : map_(map),
        params_(params),
        cache_footprint_ground_(cache_footprint_ground) {}

  double maxStepHeight() const { return params_.max_step_height; }

  /// How far below `sample` the ground lies.
  ///
  /// Casts downward from `sample` and from four offsets around it. Returns the
  /// distance to the ground and sets `status` to kOccupied when ground was
  /// found, kUnknown when every ray ran into unmapped space, or kFree when
  /// nothing was hit within max_projection_length (returning -1).
  ///
  /// `sample` is updated to the x,y of whichever offset found ground, matching
  /// the original.
  double projectSample(Eigen::Vector3d& sample, VoxelStatus& status) const;

  /// The ground for a goal, whose height is only a hint: a 2-D goal is
  /// seeded at the robot's altitude, which may be on another level.
  ///
  /// The ground projectSample finds below `sample` and the lowest ground
  /// above it within max_goal_ground_rise (bounded to kMaxGoalGroundRise,
  /// a top exactly at the bound included) are the candidates; the one nearer `sample` wins, the one below on a
  /// tie. Ground above is looked for straight above `sample`, on top of each
  /// solid a downward ray meets. Same contract as projectSample: returns
  /// how far below `sample` the ground lies, negative when above.
  double projectGoal(Eigen::Vector3d& sample, VoxelStatus& status) const;

  /// Whether a robot could drive from `start` to `end` once both are dropped
  /// onto the ground, filling `projected_edge_out` with the ground-following
  /// polyline when it can.
  ///
  /// PRECONDITION, inherited from the ROS 1 code and easy to miss: `start` and
  /// `end` must ALREADY sit at driving height, i.e. max_ground_height above
  /// the ground. Intermediate samples are height-corrected as they are
  /// generated, but the endpoint is appended as given, without correction. Feed
  /// it raw endpoints and the final segment jumps by whatever the height
  /// mismatch is, and the edge is rejected as kSteep. Rrg::expandGraph
  /// satisfies this by projecting the new state before calling in.
  ///
  /// Once the edge is known to be clear, the ground under the robot's two
  /// sides is compared: see crossSlope. An edge whose cross slope exceeds
  /// max_cross_slope is kCrossSlope. Then, when max_footprint_tilt or
  /// max_footprint_step is set, the plane under the footprint is fitted at
  /// every point and between them, at most kFootprintSampleSpacing apart
  /// along the edge, short edges included: see footprintPlane. An edge with
  /// a point whose plane tilts or steps past either is kFootprintPlane, and
  /// so is one where two neighbouring ground cells under the footprint
  /// differ in height by more than max_footprint_cell_rise.
  /// With min_observed_ground_fraction set, at the same points (not the
  /// start where the robot stands, nor the points of an edge that may hang)
  /// observedGroundAhead must reach it, or the edge is kGroundUnobserved:
  /// ahead of the travel from `start` to `end`, and with EdgeTravel::
  /// kBothWays also the other way (review r2, I-1).
  ///
  /// max_inclination and max_cross_slope stay as coarse pre-filters.
  /// Neither sees the plane the chassis sits on: a segment between samples
  /// rising no more than max_step_height is exempt from max_inclination,
  /// and the cross slope is averaged over a body length.
  ///
  /// With `body`, each segment is checked with its sweep instead of the
  /// map's box sweep of `box_size`, which still sizes the cross slope and
  /// the footprint plane.
  ProjectedEdgeStatus getProjectedEdgeStatus(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end,
      const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel,
      std::vector<Eigen::Vector3d>& projected_edge_out, bool is_hanging,
      bool preserve_start_height = false,
      const EdgeBodyCheck* body = nullptr,
      EdgeTravel travel = EdgeTravel::kBothWays) const;

  /// Sideways slope of the ground under a ground-following polyline at
  /// driving height, relative to its heading from first to last point,
  /// radians. At each point the ground is found straight below the robot's
  /// two sides, half the smaller of `box_size` x and y out from the centre
  /// line, so both probes stay inside the footprint the edge was checked
  /// free for, and the gradient between the two ground points is taken;
  /// points where either side has no ground are skipped. The gradient is
  /// averaged over every run of points within the box's larger side, as the
  /// robot's body averages it, and the steepest run is returned. Zero when
  /// nothing can be measured. With `skip_start`, the first point is not
  /// measured, but still sets the heading.
  double crossSlope(const std::vector<Eigen::Vector3d>& edge,
                    const Eigen::Vector3d& box_size,
                    bool skip_start = false) const;

  /// The least-squares plane through the ground under a footprint centred
  /// on `point`, at driving height, and facing `heading` in the XY plane.
  /// The footprint is `box_size` x long along the heading and y wide.
  /// Every map cell whose centre lies under it is used, from the map's XY
  /// cell grid (MapInterface::getCircleIntersectingXYCellCenters); a map
  /// without one is not measured. The ground is found straight below each
  /// cell's centre, as crossSlope finds it, and the plane is fitted to the
  /// ground points the map returns. It is not measured when ground lies
  /// under fewer than half the cells, or the points do not span a plane.
  /// Such a point is skipped, as crossSlope skips a point with a side over
  /// no ground.
  FootprintPlane footprintPlane(const Eigen::Vector3d& point,
                                const Eigen::Vector2d& heading,
                                const Eigen::Vector3d& box_size) const;

  /// The fraction of the map cells under the leading half of a footprint
  /// (`box_size` x long along `heading`, y wide, centred on `point` at
  /// driving height; the cells whose centre lies under it, ahead of or on
  /// its centre line) with observed ground: an occupied voxel the ground ray
  /// meets within 2 max_ground_height below `point`, or, in the disk of a
  /// standing start (setStandingStart), no ground found at all. 0 when the
  /// map's cells cannot be enumerated.
  ///
  /// A cell with no ground found under it also counts when it lies in a gap
  /// of observed ground, which is interpolated, never extrapolated: within
  /// kGroundBridgeCells on both sides of it, along `heading` or across it,
  /// observed ground (the window reaching a cell's rise at max_inclination
  /// further down) whose two heights differ by no more than max_inclination
  /// over their distance, and no free voxel in the cell's column more than
  /// kGroundBridgeHoleDepth and no more than max_projection_length under
  /// the ground interpolated between them. The
  /// lidar sees a ramp down past a crest only in patches (diag-ramp, run
  /// 7: 84 % of its cells 0-1 m down, 17 % 4-5 m down), and the crest read
  /// as a ledge. A drop has no observed ground on its far side at a
  /// drivable height, and a hole looked into has free space under it.
  double observedGroundAhead(const Eigen::Vector3d& point,
                             const Eigen::Vector2d& heading,
                             const Eigen::Vector3d& box_size) const;

  /// The largest height difference between the ground of two neighbouring
  /// map cells (sharing a side on the map's grid) under a footprint, laid
  /// out as for footprintPlane, metres; 0 with no such pair. Not cached
  /// with the plane: getProjectedEdgeStatus asks for it only with
  /// max_footprint_cell_rise set, and the ground under each cell comes
  /// from the column cache.
  double footprintCellRise(const Eigen::Vector3d& point,
                           const Eigen::Vector2d& heading,
                           const Eigen::Vector3d& box_size) const;

  /// Length-weighted soft clearance cost (between length and 5*length).
  /// Samples at most 32 points and 1024 nearby cells per point. Obstacles
  /// at body height and support cells rising past the enabled cell-rise
  /// limit are hazards. Their distance beyond the body's rectangle, turned
  /// to the segment, is preferred, never required; unknown evidence adds no
  /// hazard. (The circumscribed circle it was measured from put a wall
  /// 0.5 m beside a 1.4 x 0.8 m body inside it, so every lattice edge down a
  /// 2 m corridor carried the full penalty and routes wove: botman,
  /// 2026-10-01.)
  /// maximum_samples bounds only this soft preference, never hard sweeps
  /// or segmentClearance's shortcut minimum. Clamped to [1, 32].
  double clearanceCost(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                       const Eigen::Vector3d& box_size,
                       int maximum_samples = 32) const;

  /// The least clearance, metres, between the body moved straight from
  /// `start` to `end` and the hazards clearanceCost counts, sampled as
  /// clearanceCost samples (at most 32 points following the ground), up to
  /// path_clearance_margin: a segment clear by the margin everywhere
  /// returns the margin. Measured from the body's rectangle turned to the
  /// segment. 0 with no margin set. A soft measure, never a collision check.
  double segmentClearance(const Eigen::Vector3d& start,
                          const Eigen::Vector3d& end,
                          const Eigen::Vector3d& box_size) const;

  /// The ground straight below `point`, false when none is mapped within
  /// max_projection_length.
  bool groundBelow(const Eigen::Vector3d& point, Eigen::Vector3d& ground) const;

  /// How far down projectSample looks. Was a bare 5.0 in the original.
  double max_projection_length = 5.0;

  /// The robot stands at its start: see StandingStart. Cleared with
  /// std::nullopt, once it has moved.
  void setStandingStart(const std::optional<StandingStart>& standing) {
    standing_start_ = standing;
    ground_ahead_cache_.clear();
  }
  /// Null when the robot is not standing at its start.
  const StandingStart* standingStart() const {
    return standing_start_ ? &*standing_start_ : nullptr;
  }

  /// Counts this projection's work into `profile` (null: none) until reset.
  void setProfile(PlanProfile* profile) { profile_ = profile; }
  PlanProfile* profile() const { return profile_; }

 private:
  // Shared by nested per-cell loops of this request-scoped projector: a
  // short cache lookup must not reset the throttle and check every cell.
  mutable PlanningCheckpointThrottle checkpoint_;
  using ColumnKey = std::array<std::int64_t, 2>;
  using PlaneKey = std::array<std::int64_t, 6>;
  struct CacheKeyHash {
    template <std::size_t N>
    std::size_t operator()(const std::array<std::int64_t, N>& key) const {
      std::size_t hash = 0;
      for (const std::int64_t part : key) {
        hash ^= std::hash<std::int64_t>()(part) + 0x9e3779b97f4a7c15ULL +
                (hash << 6) + (hash >> 2);
      }
      return hash;
    }
  };
  /// The free voxels of a map cell's column between two heights: the
  /// heights at which the column was sampled, once per voxel, and found
  /// free.
  struct FreeInColumn {
    double top = 0.0;
    double bottom = 0.0;
    std::vector<double> free_z;
  };
  /// One ground ray cast down a map cell's column.
  struct GroundFromHeight {
    double from_z = 0.0;  ///< where the ray started
    bool found = false;
    Eigen::Vector3d ground = Eigen::Vector3d::Zero();
  };
  /// groundBelow, through the cache when there is one.
  bool footprintGroundBelow(const Eigen::Vector3d& point,
                            Eigen::Vector3d& ground) const;
  /// The map's cell grid round one footprint, for groundBridged.
  struct BridgeCells;
  /// Whether the cell centred at `cell`, where no ground was found, lies in
  /// a gap of observed ground (observedGroundAhead), looking from driving
  /// height `from_z` for ground no lower than `lowest`.
  bool groundBridged(BridgeCells& cells, const Eigen::Vector2d& cell,
                     const Eigen::Vector2d& along, double from_z,
                     double lowest) const;
  /// Whether the map has a free voxel in the column of `cell` between the
  /// heights `bottom` and `top`: sampled once per voxel, at (k + 1/2)
  /// resolution, a voxel counts when its sample lies between them, bounds
  /// included to within 1e-9 m, cached or not. Through the cache when there
  /// is one.
  bool freeInColumn(const Eigen::Vector2d& cell, double top,
                    double bottom) const;
  /// projectSample without its per-plan memo.
  double castProjection(Eigen::Vector3d& sample, VoxelStatus& status) const;
  using ProjectionKey = std::array<std::int64_t, 3>;
  struct Projection {
    double below = 0.0;
    VoxelStatus status = VoxelStatus::kUnknown;
    Eigen::Vector3d sample = Eigen::Vector3d::Zero();
  };
  mutable std::unordered_map<ProjectionKey, Projection, CacheKeyHash>
      projections_;
  using GroundAheadKey = std::array<std::int64_t, 9>;
  mutable std::unordered_map<GroundAheadKey, double, CacheKeyHash> ground_ahead_cache_;
  double measureObservedGroundAhead(const Eigen::Vector3d& point,
                                    const Eigen::Vector2d& heading,
                                    const Eigen::Vector3d& box_size) const;
  FootprintPlane measureFootprintPlane(const Eigen::Vector3d& point,
                                       const Eigen::Vector2d& heading,
                                       const Eigen::Vector3d& box_size) const;

  /// 1 - clearance / margin at `point`, the body's rectangle turned to
  /// `heading` (unit, XY); 0 to 1.
  double clearancePenalty(const Eigen::Vector3d& point,
                          const Eigen::Vector2d& heading,
                          const Eigen::Vector3d& box_size) const;
  /// The points clearanceCost and segmentClearance sample, on the ground.
  std::vector<Eigen::Vector3d> clearanceSamples(const Eigen::Vector3d& start,
                                                const Eigen::Vector3d& end,
                                                int maximum_samples = 32) const;
  bool clearanceHazard(const Eigen::Vector3d& cell,
                       const Eigen::Vector3d& box_size) const;
  using ClearanceKey = std::array<std::int64_t, 6>;
  mutable std::unordered_map<ClearanceKey, bool, CacheKeyHash> clearance_hazards_;
  const MapInterface& map_;
  const PlanningParams& params_;
  const bool cache_footprint_ground_ = false;
  std::optional<StandingStart> standing_start_;
  PlanProfile* profile_ = nullptr;
  mutable std::unordered_map<ColumnKey, std::vector<GroundFromHeight>,
                             CacheKeyHash>
      ground_below_column_;
  mutable std::unordered_map<ColumnKey, FreeInColumn, CacheKeyHash>
      free_in_column_;
  mutable std::unordered_map<PlaneKey, FootprintPlane, CacheKeyHash>
      footprint_planes_;
};

}  // namespace mgg

#endif  // MGG_CORE_GROUND_PROJECTION_H_
