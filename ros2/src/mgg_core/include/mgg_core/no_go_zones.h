// No-go zones: places a robot must not drive into, such as where it tripped
// its tilt guard (run 8). Each is a disc of unbounded height; `reach` is its
// radius plus half the robot's body, so a centre line kept out of it keeps
// the body out of the disc.

#ifndef MGG_CORE_NO_GO_ZONES_H_
#define MGG_CORE_NO_GO_ZONES_H_

#include <vector>

#include <Eigen/Dense>

namespace mgg {

class NoGoZones {
 public:
  /// Replaces the zones; an empty set clears them.
  void set(std::vector<Eigen::Vector2d> centres, double reach);
  bool empty() const { return centres_.empty(); }
  const std::vector<Eigen::Vector2d>& centres() const { return centres_; }
  double reach() const { return reach_; }

  /// Whether `p` lies within a zone's reach.
  bool inside(const Eigen::Vector3d& p) const;

  /// Whether a path, driven from its first point, may be taken: from inside
  /// a zone only a monotonic outward departure (each segment moving no
  /// nearer its centre) until it is out, and then, as from outside, never
  /// within reach of any zone again; and it must not end inside one
  /// (review r0, I-3). A zone the path does not start in is never entered.
  bool pathAdmissible(const std::vector<Eigen::Vector3d>& path) const;

  /// Whether a graph edge, which a search may take either way, is closed:
  /// it passes within reach of a zone, unless the robot, at `robot`, stands
  /// in that zone and the edge's nearest point to the centre is one of its
  /// ends, no nearer than the robot: such an edge can be driven only away
  /// from the centre, on the robot's way out. A route found over the edges
  /// left open is still checked with pathAdmissible.
  bool blocksEdge(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                  const Eigen::Vector3d& robot) const;

 private:
  std::vector<Eigen::Vector2d> centres_;
  double reach_ = 0.0;
};

}  // namespace mgg

#endif  // MGG_CORE_NO_GO_ZONES_H_
