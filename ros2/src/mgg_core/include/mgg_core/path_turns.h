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

#ifndef MGG_CORE_PATH_TURNS_H_
#define MGG_CORE_PATH_TURNS_H_

#include <functional>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "mgg_core/graph_manager.h"
#include "mgg_core/params.h"
#include "mgg_core/types.h"

namespace mgg {

/// A turn sharper than this is a sharp turn, radians (45 degrees)...
constexpr double kSharpTurnRad = 0.7853981633974483;
/// ...and a sharp turn is not made where the ground slopes more than this,
/// radians (8 degrees).
constexpr double kLevelGroundSlopeRad = 0.13962634015954636;

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
/// its ground, so the plane has the ground's slope. Zero when the vertices
/// do not span a plane.
double terrainSlope(GraphManager& graph, const Vertex& vertex, double radius);

/// Whether a path turns sharply only where it may.
using PathTurnsFn = std::function<bool(const std::vector<Vertex*>&)>;

/// The check selectBestPath applies to a ground robot's candidate paths
/// through `graph`: no sharp turn (kSharpTurnRad) where the terrain slopes
/// more than kLevelGroundSlopeRad. Turns are measured over the robot's
/// length, the larger of RobotParams::size x and y, and the terrain slope is
/// fitted over the same radius. Slopes are computed once per vertex.
class PathTurnCheck {
 public:
  PathTurnCheck(GraphManager& graph, const RobotParams& robot);

  bool operator()(const std::vector<Vertex*>& path);

  /// Candidate paths refused for a sharp turn on a slope.
  int refused_on_slope = 0;

 private:
  double slopeAt(const Vertex& vertex);

  GraphManager& graph_;
  double window_ = 0.0;
  std::unordered_map<int, double> slope_by_id_;
};

}  // namespace mgg

#endif  // MGG_CORE_PATH_TURNS_H_
