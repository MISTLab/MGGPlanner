#include "mgg_core/certification_cache.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

#include "mgg_core/ground_projection.h"

namespace mgg {
namespace {

std::int64_t exactBits(double value) {
  std::int64_t bits;
  if (value == 0.0) value = 0.0;  // canonicalize signed zero
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

double fromBits(std::int64_t bits) {
  double value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

/// How far down turnSpaceObserved looks under a standing start
/// (path_turns.cpp, kStandingGroundDepth), whatever the projection length.
constexpr double kStandingTurnGroundDepth = 5.0;

Eigen::AlignedBox3d around(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi,
                           double xy, double below, double above) {
  return Eigen::AlignedBox3d(lo - Eigen::Vector3d(xy, xy, below),
                             hi + Eigen::Vector3d(xy, xy, above));
}

}  // namespace

DependencyHalos dependencyHalos(const RobotParams& robot,
                                const PlanningParams& planning,
                                double resolution,
                                double max_projection_length) {
  const Eigen::Vector3d box = robot.getPlanningSize();
  const double offset_xy = robot.center_offset.head<2>().norm();
  const double offset_z = std::abs(robot.center_offset.z());
  const double physical_offset_xy =
      robot.physicalOffsetForHeading(0.0).head<2>().norm();
  const double projection = max_projection_length;
  const double ground_height = planning.max_ground_height;
  // GroundProjection's projectSample probes start this far above and beside
  // a sample (ground_projection.cpp, probeOffset).
  const double probe = std::max(0.20, 2.0 * resolution);
  DependencyHalos halos;
  halos.edge_xy = std::max(0.5 * box.head<2>().norm() +
                               (kGroundBridgeCells + 1) * resolution,
                           probe + resolution) +
                  2.0 * offset_xy;
  // Every read of an edge check, from the swept key points down
  // (getProjectedEdgeStatus): a projected point stands max_ground_height
  // over ground its sample's ray found, at most a projection length and a
  // voxel below the sample; the body, the cross-slope and footprint rays
  // and the ground-ahead rays start at or above it and reach a projection
  // length further; a ground bridge's hole check (groundBridged) reads from
  // the bridged ground, which such a ray found, down another projection
  // length. Each step adds a voxel of slack.
  const double projected_drop =
      std::max(0.0, projection + resolution - ground_height);
  halos.edge_below = projected_drop + 2.0 * projection + 2.0 * resolution;
  // Upwards: a projected point rises at most max_ground_height over its
  // sample, with half the body above it; projectSample's rays start a probe
  // offset above the sample.
  halos.edge_above =
      std::max({box.z(), ground_height + 0.5 * box.z(), probe}) + offset_z +
      resolution;
  halos.turn_xy = robot.turningRadius() + physical_offset_xy + resolution;
  halos.turn_below =
      std::max({projection, 2.0 * ground_height, kStandingTurnGroundDepth}) +
      offset_z + resolution;
  halos.turn_above = box.z() + offset_z + resolution;
  halos.slope_xy = std::max(robot.size.x(), robot.size.y()) + resolution;
  halos.slope_below = projection + resolution;
  halos.slope_above = resolution;
  return halos;
}

Eigen::AlignedBox3d edgeDependency(const Eigen::Vector3d& a,
                                   const Eigen::Vector3d& b,
                                   const DependencyHalos& halos) {
  return around(a.cwiseMin(b), a.cwiseMax(b), halos.edge_xy, halos.edge_below,
                halos.edge_above);
}

Eigen::AlignedBox3d turnRoomDependency(const Eigen::Vector3d& p,
                                       const DependencyHalos& halos) {
  return around(p, p, halos.turn_xy, halos.turn_below, halos.turn_above);
}

Eigen::AlignedBox3d slopeDependency(const Eigen::Vector3d& p,
                                    const DependencyHalos& halos) {
  return around(p, p, halos.slope_xy, halos.slope_below, halos.slope_above);
}

CertificationCache::CertificationCache(const DependencyHalos& halos)
    : halos_(halos), tables_(std::make_shared<Tables>()) {}

TurnRoomFn CertificationCache::turnRoom(TurnRoomFn check) {
  return [tables = tables_, check = std::move(check)](const StateVec& state) {
    const TurnKey key{exactBits(state.x()), exactBits(state.y()),
                      exactBits(state.z()), exactBits(state[3])};
    const auto known = tables->turn_room.find(key);
    if (known != tables->turn_room.end()) return known->second;
    const bool room = check(state);
    tables->turn_room.emplace(key, room);
    return room;
  };
}

SlopeFn CertificationCache::slope(SlopeFn measure) {
  return [tables = tables_,
          measure = std::move(measure)](const Eigen::Vector3d& position) {
    const SlopeKey key{exactBits(position.x()), exactBits(position.y()),
                       exactBits(position.z())};
    const auto known = tables->slope.find(key);
    if (known != tables->slope.end()) return known->second;
    const double value = measure(position);
    tables->slope.emplace(key, value);
    return value;
  };
}

void CertificationCache::withdraw(const MapChange& change) {
  if (change.everything) {
    flushAll();
    return;
  }
  if (change.boxes.empty()) return;
  // A dependency that is not a finite box cannot be shown unreached.
  const auto reached = [&change](const Eigen::AlignedBox3d& dependency) {
    return !dependency.min().allFinite() || !dependency.max().allFinite() ||
           changeReaches(change, dependency);
  };
  edges_.eraseIf([&](const EdgeVerdictCache::Key& key) {
    const Eigen::Vector3d a(fromBits(key[0]), fromBits(key[1]),
                            fromBits(key[2]));
    const Eigen::Vector3d b(fromBits(key[3]), fromBits(key[4]),
                            fromBits(key[5]));
    return reached(edgeDependency(a, b, halos_));
  });
  for (auto it = tables_->turn_room.begin(); it != tables_->turn_room.end();) {
    const Eigen::Vector3d p(fromBits(it->first[0]), fromBits(it->first[1]),
                            fromBits(it->first[2]));
    if (reached(turnRoomDependency(p, halos_))) {
      it = tables_->turn_room.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = tables_->slope.begin(); it != tables_->slope.end();) {
    const Eigen::Vector3d p(fromBits(it->first[0]), fromBits(it->first[1]),
                            fromBits(it->first[2]));
    if (reached(slopeDependency(p, halos_))) {
      it = tables_->slope.erase(it);
    } else {
      ++it;
    }
  }
}

void CertificationCache::flushAll() {
  edges_.clear();
  tables_->turn_room.clear();
  tables_->slope.clear();
}

}  // namespace mgg
