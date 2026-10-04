// Certified local verdicts kept across planning cycles over a changing map.
//
// Ground edge, turn-room and slope verdicts depend on map space well beyond
// the cells their points sit in: a projected edge on its body sweep, its
// ground rays and the bridged cells beside them; a turn on its turning disc
// and the ground under it; a slope on the ground ring around the pose. Each
// cached verdict carries such a dependency box, and a MapChange reaching it
// erases the verdict at once. Nothing erased is answered again until the
// wrapped check recomputes it, inside whatever planning budget calls it.

#ifndef MGG_CORE_CERTIFICATION_CACHE_H_
#define MGG_CORE_CERTIFICATION_CACHE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>

#include <Eigen/Geometry>

#include "mgg_core/dirty_region.h"
#include "mgg_core/graph_expansion.h"
#include "mgg_core/params.h"
#include "mgg_core/path_turns.h"

namespace mgg {

/// How far each verdict's dependency reaches beyond its points, metres.
/// `*_xy` is horizontal on every side, `*_below`/`*_above` vertical.
struct DependencyHalos {
  double edge_xy = 0.0;
  double edge_below = 0.0;
  double edge_above = 0.0;
  double turn_xy = 0.0;
  double turn_below = 0.0;
  double turn_above = 0.0;
  double slope_xy = 0.0;
  double slope_below = 0.0;
  double slope_above = 0.0;
};

/// Halos for a ground robot on a map of `resolution`, whose ground
/// projection looks `max_projection_length` down:
///  * edge: XY half the planning-box diagonal, plus the bridged ground cells
///    beside the sweep ((kGroundBridgeCells + 1) resolutions), plus twice the
///    planning offset (the reversed footprint of an offset body); below the
///    projection length; above the body height plus the offset height and
///    a resolution;
///  * turn: XY the turning radius plus the physical offset and a resolution;
///    below the deeper of the projection length and 2 max_ground_height,
///    above the body height, each plus the offset height and a resolution;
///  * slope (groundSlope over max(robot length, width)): XY that radius plus
///    a resolution; below the projection length plus a resolution; above a
///    resolution.
DependencyHalos dependencyHalos(const RobotParams& robot,
                                const PlanningParams& planning,
                                double resolution,
                                double max_projection_length);

/// The closed box an edge verdict between swept points `a` and `b` reads.
Eigen::AlignedBox3d edgeDependency(const Eigen::Vector3d& a,
                                   const Eigen::Vector3d& b,
                                   const DependencyHalos& halos);
/// The closed box a turn-room verdict at driving position `p` reads.
Eigen::AlignedBox3d turnRoomDependency(const Eigen::Vector3d& p,
                                       const DependencyHalos& halos);
/// The closed box a map slope at position `p` reads.
Eigen::AlignedBox3d slopeDependency(const Eigen::Vector3d& p,
                                    const DependencyHalos& halos);

/// Edge, turn-room and slope verdicts reused across planning cycles until a
/// map change reaches their dependency. Not thread-safe.
///
/// Every wrapper shares one table per kind: wrap one check of one map,
/// robot and policy. The cached slope must be map-only, groundSlope(...,
/// nullptr): a lattice-fitted slope also depends on the lattice. Changing
/// anything the keys do not hold (the standing start, no-go zones, the
/// wrapped checks' parameters) requires flushAll().
class CertificationCache {
 public:
  explicit CertificationCache(const DependencyHalos& halos);

  /// Supply as ExpandContext::edge_verdicts; outlives one build.
  EdgeVerdictCache& edges() { return edges_; }
  /// `check`, answered from this cache's turn-room table while the
  /// verdict's dependency is unchanged.
  TurnRoomFn turnRoom(TurnRoomFn check);
  /// `measure`, answered from this cache's slope table while the value's
  /// dependency is unchanged.
  SlopeFn slope(SlopeFn measure);

  /// Erases every verdict whose dependency `change` reaches (all of them
  /// for `everything`).
  void withdraw(const MapChange& change);
  /// Erases every verdict.
  void flushAll();

 private:
  struct BitsHash {
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
  using TurnKey = std::array<std::int64_t, 4>;
  using SlopeKey = std::array<std::int64_t, 3>;
  struct Tables {
    std::unordered_map<TurnKey, bool, BitsHash> turn_room;
    std::unordered_map<SlopeKey, double, BitsHash> slope;
  };

  DependencyHalos halos_;
  EdgeVerdictCache edges_;
  std::shared_ptr<Tables> tables_;
};

}  // namespace mgg

#endif  // MGG_CORE_CERTIFICATION_CACHE_H_
