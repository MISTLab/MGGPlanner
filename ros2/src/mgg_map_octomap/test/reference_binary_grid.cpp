#include "reference_binary_grid.h"

#include <algorithm>
#include <utility>

namespace mgg {
// The original status lookup, compiled into a separate grid type only for
// lattice comparisons. The rest of its geometry and traversal is identical.
template <class Cell>
class BinarySearchCellIndex {
 public:
  void build(const std::vector<Cell>&, const std::vector<Cell>&) {}
  int status(const Cell& key, const std::vector<Cell>& occupied,
             const std::vector<Cell>& free) const {
    if (std::binary_search(occupied.begin(), occupied.end(), key)) return 2;
    if (std::binary_search(free.begin(), free.end(), key)) return 1;
    return 0;
  }
};
}  // namespace mgg

// Compile the same NativeMolaGrid implementation into a separate test-only
// type, substituting *only* its index. Keeping walk, boxes and ground rays
// shared avoids a second independently drifting map implementation.
#undef MGG_MAP_OCTOMAP_NATIVE_MOLA_GRID_H_
#define NativeMolaGrid BinarySearchGrid
#define CellIndex BinarySearchCellIndex
#include "mgg_map_octomap/native_mola_grid.h"
#include "../src/native_mola_grid.cpp"
#undef CellIndex
#undef NativeMolaGrid

std::unique_ptr<mgg::MapInterface> makeReferenceBinaryGrid(
    double resolution, std::vector<mgg::NativeMolaGrid::Cell> occupied,
    std::vector<mgg::NativeMolaGrid::Cell> free,
    std::vector<mgg::NativeMolaGrid::Surface> surfaces) {
  std::vector<mgg::BinarySearchGrid::Cell> reference_occupied, reference_free;
  std::vector<mgg::BinarySearchGrid::Surface> reference_surfaces;
  for (const auto& c : occupied) reference_occupied.push_back({c.x, c.y, c.z});
  for (const auto& c : free) reference_free.push_back({c.x, c.y, c.z});
  for (const auto& s : surfaces)
    reference_surfaces.push_back({{s.cell.x, s.cell.y, s.cell.z}, s.max_z});
  return std::make_unique<mgg::BinarySearchGrid>(
      resolution, std::move(reference_occupied), std::move(reference_free),
      std::move(reference_surfaces));
}
