// Map changes as dirty regions, shared by every map producer and consumer.
//
// A producer (RollingVoxelMap) returns one MapChange per mutating call; a
// consumer that cached a result over a spatial dependency withdraws it when
// the change reaches that dependency. Reach is inclusive: touching counts.
// Consumers take `const MapInterface&` plus `const MapChange&`, never the
// producing map, so mgg_core does not depend on mgg_map_octomap.

#ifndef MGG_CORE_DIRTY_REGION_H_
#define MGG_CORE_DIRTY_REGION_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Geometry>

namespace mgg {

/// Integer coordinates of a voxel of the world-anchored grid `floor(p/res)`.
/// Keys do not depend on any window, so they survive scrolling.
struct VoxelKey {
  std::int32_t x = 0, y = 0, z = 0;
  friend bool operator==(const VoxelKey& a, const VoxelKey& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
  }
  friend bool operator!=(const VoxelKey& a, const VoxelKey& b) {
    return !(a == b);
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& k) const {
    std::uint64_t h = std::uint64_t(std::uint32_t(k.x)) * 0x9e3779b97f4a7c15ULL;
    h ^= std::uint64_t(std::uint32_t(k.y)) * 0xc2b2ae3d27d4eb4fULL + (h << 6) +
         (h >> 2);
    h ^= std::uint64_t(std::uint32_t(k.z)) * 0x165667b19e3779f9ULL + (h << 6) +
         (h >> 2);
    return std::size_t(h);
  }
};

/// The voxel of edge `resolution` containing `p`. Coordinates beyond the
/// int32 range saturate; a non-finite coordinate maps to INT32_MIN.
VoxelKey keyOf(const Eigen::Vector3d& p, double resolution);

/// Centre of the voxel `key` of edge `resolution`.
Eigen::Vector3d centerOf(const VoxelKey& key, double resolution);

/// What one map call changed. `boxes` are closed regions whose ternary
/// voxel state changed, or that were evicted or entered by scrolling;
/// `everything` means the whole map changed (a reset). `revision` is the
/// map's revision after the call: it changes only when the map did.
struct MapChange {
  std::uint64_t revision = 0;
  std::vector<Eigen::AlignedBox3d> boxes;
  bool everything = false;
};

/// The change's boxes grown by `halo_m` on every side; one unbounded box for
/// `everything`.
std::vector<Eigen::AlignedBox3d> dirtyRegions(const MapChange& change,
                                              double halo_m);

/// Whether `change` reaches the closed box `dependency`, touching included.
bool changeReaches(const MapChange& change,
                   const Eigen::AlignedBox3d& dependency);

}  // namespace mgg

#endif  // MGG_CORE_DIRTY_REGION_H_
