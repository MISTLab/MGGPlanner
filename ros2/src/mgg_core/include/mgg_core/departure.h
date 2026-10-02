// How a ground robot boxed in where it stands drives out: straight along
// its heading, ahead or back, to the first pose with room to turn in place.
//
// Not upstream. The body is checked as the robot's own rectangle turned to
// its heading, not as the smallest box aligned with the map that holds it,
// and the map cells under the body where it stands are not checked: the
// robot is standing there. In run 5 (2026-09-25) two Bunkers stopped with a
// wall 0.49 to 0.64 m away were refused every departure at the first step:
// the enlarged box at their start pose already met that wall.

#ifndef MGG_CORE_DEPARTURE_H_
#define MGG_CORE_DEPARTURE_H_

#include <vector>
#include <map>
#include <functional>

#include <Eigen/Dense>

#include "mgg_core/ground_projection.h"
#include "mgg_core/map_interface.h"
#include "mgg_core/params.h"
#include "mgg_core/types.h"

namespace mgg {

/// A departure ends at least this far out, past the controller's goal
/// tolerance and about a robot's length, metres...
inline constexpr double kDepartureMinM = 0.5;
/// ...and goes no farther than this.
inline constexpr double kDepartureMaxM = 2.0;
/// The most a departure turns in place before it drives, radians (30
/// degrees), in steps of kDepartureTurnStepRad (5 degrees), nearest first.
inline constexpr double kDepartureMaxTurnRad = 0.5235987755982988;
inline constexpr double kDepartureTurnStepRad = 0.08726646259971647;

/// An upright box standing at a pose: `size` x long along `heading`, y
/// wide, z tall, centred on `center`.
struct OrientedBox {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double heading = 0.0;
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
};

/// Query-only evidence of the robot's own physical volume on one pinned map.
/// Columns are keyed by the backend's XY cell centres; merged vertical
/// intervals do not change the map and NEVER exempt occupied voxels.
class KnownFreeBodyVolumes {
 public:
  void add(const MapInterface& map, const OrientedBox& body);
  /// Poses at driving height, in acquisition order, newest last. Samples
  /// actual chassis poses (not an enlarged swept AABB), newest 20 metres.
  void addTrajectory(const MapInterface& map, const RobotParams& robot,
                     const std::vector<StateVec>& poses);
  VoxelStatus strictColumnStatus(const MapInterface& map,
      const Eigen::Vector2d& cell, double lower, double upper) const;
 private:
  using ColumnKey = std::pair<long long, long long>;
  static ColumnKey key(const Eigen::Vector2d& cell) {
    return {std::llround(cell.x()*1e9), std::llround(cell.y()*1e9)};
  }
  std::map<ColumnKey, std::vector<std::pair<double, double>>> columns_;
};

/// Whether the closed XY square of a map cell, centred on `cell_center`
/// with sides `resolution` along x and y, meets `box`'s XY rectangle by
/// more than `touch` metres (negative for a closed test that counts a
/// shared edge).
bool cellMeetsBox(const Eigen::Vector2d& cell_center, double resolution,
                  const OrientedBox& box, double touch);

/// Whether `point` lies inside `box`'s XY rectangle, edges included.
bool pointInBox(const Eigen::Vector2d& point, const OrientedBox& box);

/// Occupancy of the volume `box` sweeps moved in a straight line from
/// `start` to `end`, keeping its heading. The sweep is split into steps of
/// at most a map cell; each step's box is lengthened along the heading and
/// widened across it by the step, and grown in z by its rise. Every map
/// cell whose square meets that box (MapInterface::
/// getCircleIntersectingXYCellCenters, the cells treated as squares along
/// the caller's x and y) is checked as a column over the box's height with
/// MapInterface::getBoxStatus; with `stop_at_unknown_voxel` an unknown
/// cell is kUnknown, unless an occupied one is met. With `standing`, where
/// the robot stands, a cell whose centre lies inside it is not checked at
/// any height: the robot's own body is there. A cell that only reaches
/// into it is checked where the moving body reaches into the cell beyond
/// the standing body (sampled every resolution / 8), so a post just past
/// the robot's front is not driven through, while a wall column beside it
/// does not stop the robot driving along it (review r0, I-1).
/// `box.center` is ignored. With clearance_prefilter, a known-free
/// circumscribed static AABB may accept early, after the dynamic sweep.
/// An occupied or unknown bound never decides: the exact sweep follows.
/// With unknown_above_center, all occupied volume still blocks, and the
/// volume below that height relative to each body centre is strictly known
/// free. Only the ground local-lattice sensor policy supplies this value;
/// aerial and global checks leave it unset. The pre-filter is disabled.
/// `known_free` subtracts ONLY unknown physical own-body intervals from the
/// strict check. With `standing_unknown_only`, `standing` uses this same
/// bounded-height rule and occupied cells still block. The default retains
/// the pre-existing boxed-in departure semantics described above.
VoxelStatus orientedBoxPathStatus(const MapInterface& map,
                                  const Eigen::Vector3d& start,
                                  const Eigen::Vector3d& end,
                                  const OrientedBox& box,
                                  bool stop_at_unknown_voxel,
                                  const OrientedBox* standing,
                                  bool clearance_prefilter = false,
                                  std::optional<double> unknown_above_center = std::nullopt,
                                  const KnownFreeBodyVolumes* known_free = nullptr,
                                  bool standing_unknown_only = false);

/// Aerial sweep from the physical root. The ordinary sweep permits unknown
/// volume only inside the original root body. On occupied-sweep failure a
/// capable backend may admit bounded outward-only occupied-root recovery
/// (MapInterface::aerialRootRecoveryTraversable), never an ordinary edge.
/// Dynamic peer/no-go departure policy is evaluated once over the full sweep.
/// Arguments are body centres (including RobotParams::center_offset).
bool aerialRootDepartureTraversable(const MapInterface& map,
                                     const Eigen::Vector3d& start,
                                     const Eigen::Vector3d& end,
                                     const Eigen::Vector3d& size);

/// Validates travel from `from` back to `to`, with the body facing the
/// opposite direction (the forward entry heading). Uses the forward edge's
/// oriented sweep and ground predicates, with no root/standing exemptions,
/// and applies the descent limit to the reverse direction. `ground` must
/// have no standing-start prior. `segment_admissible` checks current peers
/// and no-go zones on every projected segment.
bool reverseExitEdgeAdmissible(
    const MapInterface& map, const GroundProjection& ground,
    const RobotParams& robot, const PlanningParams& planning,
    const StateVec& from, const StateVec& to,
    const std::function<bool(const Eigen::Vector3d&, const Eigen::Vector3d&)>&
        segment_admissible = {});

/// A way out for a robot boxed in at a pose.
struct Departure {
  /// Where the robot stands, at the heading it departs with, then every
  /// pose out to the first with room to turn; each pose has that heading,
  /// so a departure back is driven in reverse.
  std::vector<StateVec> path;
  bool reverse = false;
  /// What the robot turns in place before it drives, radians,
  /// anticlockwise positive.
  double turn = 0.0;
};

/// Reference after a stationary ground-robot spin about its physical centre.
/// The planning vertical band and aerial reference are unchanged.
StateVec referenceAfterChassisSpin(const RobotParams& robot,
                                  const StateVec& start, double heading);

/// A departure for a ground robot boxed in at `start`, at driving height,
/// facing start[3]. Straight ahead along its heading, or else, with
/// PlanningParams::departure_reverse_allowed, straight back, in steps of
/// path_interpolation_distance (at most kDepartureMinM) up to
/// kDepartureMaxM, to the first pose at least kDepartureMinM out where
/// the robot has room to turn in place (roomToTurn). Each step is projected
/// to driving height and checked with getProjectedEdgeStatus, through
/// known free space only, its body the robot's planning box turned to the
/// heading (orientedBoxPathStatus, the cells under the robot where it
/// stands not checked, nor the cross slope and footprint plane there).
/// With no way out at its heading, the robot may
/// first turn in place by up to kDepartureMaxTurnRad, nearest first: at
/// every kDepartureTurnStepRad of the turn the body turned there must be
/// free, checked the same way. These are discrete poses, not the swept
/// arc: halfway between two steps a Bunker's corners reach about 2 cm
/// outside both (review r0, M-1). The path turns only in its first pose,
/// the start at the new heading; a controller such as SwarmDeck's DWB
/// blends that turn into the drive rather than turning in place first, so
/// the body it drives is not exactly the one checked, and its own costmap
/// stays the collision authority (review r0, M-6). The ground observed
/// ahead and the room to turn count the disk of `ground`'s standing start
/// (GroundProjection::setStandingStart) as observed ground. Returns false,
/// with `departure.path` empty, when there is no way out.
/// For aerial robots the original hover body may contain unknown cells;
/// bounded outward-only occupied-root recovery is backend-dependent. All
/// non-exempt swept cells and the endpoint must be observed-free. The body
/// uses RobotParams::center_offset.
bool findDeparture(const MapInterface& map, const GroundProjection& ground,
                   const RobotParams& robot, const PlanningParams& planning,
                   const StateVec& start, Departure& departure,
                   const std::function<bool(const std::vector<StateVec>&)>&
                       end_admissible = {},
                   double max_distance = kDepartureMaxM,
                   bool straight_only = false);

}  // namespace mgg

#endif  // MGG_CORE_DEPARTURE_H_
