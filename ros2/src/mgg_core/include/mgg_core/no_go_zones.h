// No-go zones: places a robot must not drive into, such as where it tripped
// its tilt guard (run 8). Each is a disc of unbounded height; `reach` is its
// radius plus half the robot's body, so a centre line kept out of it keeps
// the body out of the disc.

#ifndef MGG_CORE_NO_GO_ZONES_H_
#define MGG_CORE_NO_GO_ZONES_H_

#include <vector>

#include <Eigen/Dense>

#include "mgg_core/graph_manager.h"

namespace mgg {

class NoGoZones {
 public:
  /// Replaces the zones; an empty set clears them.
  void set(std::vector<Eigen::Vector2d> centres, double reach);
  /// Per-disc centre-line reaches; no additional body inflation.
  void set(std::vector<Eigen::Vector2d> centres, std::vector<double> reaches);
  bool empty() const { return centres_.empty(); }
  const std::vector<Eigen::Vector2d>& centres() const { return centres_; }
  /// Largest reach (the common reach for the legacy uniform set).
  double reach() const { return reach_; }
  const std::vector<double>& reaches() const { return reaches_; }

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

  /// The zones a path is still departing, by index in centres(), in
  /// increasing order. Only zones holding the path's start are ever in it,
  /// so it is short whatever the number of zones; every other zone is
  /// simply blocked (review r2, R2-1: a 64-bit mask left the 65th zone and
  /// beyond undepartable).
  using Departing = std::vector<int>;
  /// The zones `p` lies within reach of: a path starting at `p` departs
  /// them.
  Departing departing(const Eigen::Vector3d& p) const;
  /// One segment of a path driven from `a` to `b` under pathAdmissible's
  /// rule, with `departing` the zones the path is still leaving: false
  /// when the segment is refused, else `departing` becomes the zones it is
  /// still leaving at `b`.
  bool step(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
            Departing& departing) const;

 private:
  std::vector<Eigen::Vector2d> centres_;
  std::vector<double> reaches_;
  double reach_ = 0.0;
};

/// The cheapest route over `graph` from vertex `source_id` to `target_id`
/// that pathAdmissible accepts when driven from `start` (where the robot
/// stands, the source or a point it departs from to the source): a
/// Dijkstra search over directed states, a vertex and the zones the route
/// is still leaving, so a departure goes outward only and nothing re-enters
/// a zone (review r1, R1-1). The vertices from source to target, or empty
/// when there is none. It walks edge_map_, skipping edges the graph no
/// longer holds and vertices out of service, and does not consult the
/// graph's edge filter, whose undirected rule it replaces.
std::vector<Vertex*> zoneRespectingRoute(GraphManager& graph, int source_id,
                                         int target_id,
                                         const Eigen::Vector3d& start,
                                         const NoGoZones& zones);

}  // namespace mgg

#endif  // MGG_CORE_NO_GO_ZONES_H_
