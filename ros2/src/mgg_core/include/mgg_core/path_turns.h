// Where a ground robot's path turns, and whether it may turn there.
//
// Not upstream. On the SubT ramp, MGG sent a Spot, a Bunker and a Scout
// round turns on a 16 degree slope; they rolled 22 to 31 degrees and the
// Spot tipped over (live runs, 2026-09-23). A path over a slope has to run
// along the fall line or the ramp's axis and turn on level ground.
//
// The rule is applied to paths as they are chosen (selectBestPath), not to
// the lattice as it is built. The lattice is an undirected graph whose
// vertices carry no heading: whether a robot turns at a vertex depends on
// the edge it arrives by and the edge it leaves by, which only a path fixes.
// A lattice with heading in its state would be a different planner.
//
// A sharp turn also needs room: a skid-steer robot turns in place, sweeping
// the circle through its corners. MGG's body check is a box aligned with the
// map, 0.41 m either side for a Bunker whose corners reach 0.64 m, and it
// routed Bunkers along walls to turn where they could not (SubT, 2026-09-23).

#ifndef MGG_CORE_PATH_TURNS_H_
#define MGG_CORE_PATH_TURNS_H_

#include <array>
#include <functional>
#include <map>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/graph_manager.h"
#include "mgg_core/ground_projection.h"
#include "mgg_core/map_interface.h"
#include "mgg_core/params.h"
#include "mgg_core/types.h"

namespace mgg {

/// A turn sharper than this is a sharp turn, radians (45 degrees)...
constexpr double kSharpTurnRad = 0.7853981633974483;
/// ...and a sharp turn is not made where the ground slopes more than this,
/// radians (8 degrees).
constexpr double kLevelGroundSlopeRad = 0.13962634015954636;
/// The slope of ground that cannot be measured, radians (90 degrees): steep,
/// so no sharp turn is made where it is unknown.
constexpr double kUnknownSlopeRad = 1.5707963267948966;

/// Heading change at each of `points`, radians in [0, pi]. The heading into
/// a point is the direction from the first point at least `window` metres
/// back along the path (the path's start, when it is shorter), and into the
/// first point `start_heading`, the robot's yaw; the heading out of it is
/// towards the first point at least `window` ahead, or the path's end. The
/// last point has no turn. Measured over a window, a lattice staircase reads
/// as the straight line the shortcut makes of it, and a corner split over
/// several vertices as the one turn the robot makes.
std::vector<double> pathTurns(const std::vector<Eigen::Vector3d>& points,
                              double start_heading, double window);

/// Slope of the ground under `vertex`, radians: a plane fitted by least
/// squares to it and every vertex of `graph` within `radius` that has ground
/// beneath it (not is_hanging). Every vertex rides the same height above
/// its ground, so the plane has the ground's slope. kUnknownSlopeRad when
/// fewer than three of them span a plane: a sparse lattice, or one in a
/// line, says nothing about the slope across it.
double terrainSlope(GraphManager& graph, const Vertex& vertex, double radius);

/// Slope of the ground around `position`, radians, measured from the map
/// rather than from graph vertices: a plane fitted by least squares to the
/// ground straight below `position` and below eight points on a circle of
/// `radius` round it. kUnknownSlopeRad when fewer than three ground points
/// are found or they do not span a plane. For a route over the global
/// graph, whose vertices lie a metre apart along the tracks the robots
/// drove and so seldom span a plane within a robot's length.
double groundSlope(const GroundProjection& ground,
                   const Eigen::Vector3d& position, double radius);

/// Whether the robot has room to turn in place at `state`: no occupied voxel
/// within its turning radius (RobotParams::turningRadius), over the height
/// of its collision box. A ground robot's viewpointClear asks for more, so
/// every path end it allows passes. Unknown space passes, as it does there,
/// and so does a query the map cannot answer.
bool turnClear(const MapInterface& map, const RobotParams& robot,
               const StateVec& state);

/// Whether a path turns sharply only where it may.
using PathTurnsFn = std::function<bool(const std::vector<Vertex*>&)>;

/// Whether the robot has room to turn in place at a pose, e.g. turnClear.
using TurnRoomFn = std::function<bool(const StateVec&)>;

/// Whether the robot may turn sharply at a vertex: see
/// PathTurnCheck::sharpTurnAllowedAt.
using SharpTurnAllowedFn = std::function<bool(const Vertex&)>;

/// The slope of the ground at a position, radians, e.g. groundSlope.
using SlopeFn = std::function<double(const Eigen::Vector3d&)>;

/// Routes from a start vertex, found by findTurnCompliantRoutes.
struct TurnCompliantRoutes {
  struct Route {
    std::vector<Vertex*> path;  ///< the start vertex first
    std::vector<double> along;  ///< edge cost from the start to each vertex
  };
  /// By destination id; a destination with no route is absent.
  std::unordered_map<int, Route> to;
  /// Search states (vertex, vertex it was reached from) expanded.
  int states_expanded = 0;
  /// The search stopped at its bound; routes found by then are kept.
  bool capped = false;
  /// Arrivals at a destination refused for a sharp turn, less than `window`
  /// before it, where one is not allowed.
  int arrivals_refused = 0;
};

/// The cheapest route from vertex `start_id` of `graph`, by default vertex
/// 0, the lattice's root, to each of `destinations`
/// that turns sharply (kSharpTurnRad) only at vertices where
/// `sharp_turn_allowed`. The search runs over directed edges, a vertex
/// together with the one it was reached from, so a vertex whose shortest
/// route turns where it may not is reached the longer way round: up a ramp
/// to its level top and back, rather than across it. A route passes each
/// vertex once. The turn onto an edge is measured as pathTurns measures it
/// looking back, from the first vertex at least `window` back along the
/// route, and at the start from `start_heading`. A turn less than `window`
/// before a destination is measured towards it when the route arrives, and
/// an arrival that turns where it may not is refused while the search goes
/// on for another. The caller still checks a route with PathTurnCheck before
/// using it. At most `max_states` states are expanded.
TurnCompliantRoutes findTurnCompliantRoutes(
    GraphManager& graph, double start_heading, double window,
    const std::vector<int>& destinations,
    const SharpTurnAllowedFn& sharp_turn_allowed, int max_states,
    int start_id = 0);

/// The check selectBestPath applies to a ground robot's candidate paths
/// through `graph`: a sharp turn (kSharpTurnRad) is refused where the
/// terrain slopes more than kLevelGroundSlopeRad, and, with `room_to_turn`,
/// where the robot has no room to turn in place. The first vertex is where
/// the robot stands: a path that sets off sharply away from its heading
/// needs room there too. Turns are measured over the robot's length, the
/// larger of RobotParams::size x and y, and the terrain slope is fitted over
/// the same radius (terrainSlope), or given by `slope`. Slope and room are
/// found once per position.
class PathTurnCheck {
 public:
  PathTurnCheck(GraphManager& graph, const RobotParams& robot,
                TurnRoomFn room_to_turn = nullptr, SlopeFn slope = nullptr);

  /// The distance over which turns are measured, the robot's length.
  double window() const { return window_; }

  /// A candidate path through the graph, turning first from the heading of
  /// its first vertex (the root's is the robot's yaw). Counts refusals.
  bool operator()(const std::vector<Vertex*>& path);

  /// The same check on any polyline, e.g. a path after shortcutting, whose
  /// first point turns from `start_heading`. Counts nothing.
  bool admissible(const std::vector<Eigen::Vector3d>& points,
                  double start_heading);

  /// Whether a sharp turn may be made at `position`: the ground is level
  /// there and, with `room_to_turn`, the robot has room to turn.
  bool sharpTurnAllowedAt(const Eigen::Vector3d& position);

  /// Candidate paths refused for a sharp turn on a slope, and for one
  /// without room to turn.
  int refused_on_slope = 0;
  int refused_without_room = 0;

 private:
  enum class Refusal { kNone, kSlope, kRoom };
  using PositionKey = std::array<long long, 3>;

  Refusal firstRefusal(const std::vector<Eigen::Vector3d>& points,
                       double start_heading);
  double slopeAt(const Eigen::Vector3d& position);
  bool roomAt(const Eigen::Vector3d& position);

  GraphManager& graph_;
  double window_ = 0.0;
  TurnRoomFn room_to_turn_;
  SlopeFn slope_;
  std::map<PositionKey, double> slope_at_;
  std::map<PositionKey, bool> room_at_;
};

/// What chooseTurnCompliantRoute did with a route.
struct RouteTurnChoice {
  /// The route failed the check and a compliant one was looked for.
  bool searched = false;
  /// A longer route that passes the check replaced it.
  bool detour = false;
  /// None was found: the route is kept as it was, failing the check.
  bool fallback = false;
  /// What the search cost, and whether it stopped at its bound rather than
  /// running out of routes: a fallback after a capped search may have
  /// missed a compliant route.
  int states_expanded = 0;
  bool capped = false;
};

/// The turn rule for a route to a goal, as selectBestPath applies it to
/// exploration paths. `route` runs through `graph` from where the robot
/// joins it to the goal, after `lead_in`, the robot's own pose when it is
/// not on the route's first vertex; the robot's heading is
/// `start_heading`. A route that passes `check` is kept. Otherwise the
/// cheapest route between the same two vertices that turns sharply only
/// where `check` allows (findTurnCompliantRoutes, at most `max_states`
/// states) replaces it if it passes `check` too; and failing that the
/// route is kept, flagged fallback, so that the rule never leaves the robot
/// without a route it had.
RouteTurnChoice chooseTurnCompliantRoute(
    GraphManager& graph, PathTurnCheck& check,
    const std::vector<Eigen::Vector3d>& lead_in, double start_heading,
    std::vector<Vertex*>& route, int max_states);

}  // namespace mgg

#endif  // MGG_CORE_PATH_TURNS_H_
