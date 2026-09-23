#ifndef MGG_MAP_OCTOMAP_TEST_REFERENCE_BINARY_GRID_H_
#define MGG_MAP_OCTOMAP_TEST_REFERENCE_BINARY_GRID_H_

#include <memory>
#include <vector>

#include "mgg_map_octomap/native_mola_grid.h"

// Compiled only into test_mola_map: the production grid's geometry and walk
// logic with a test-only binary-search status index.
std::unique_ptr<mgg::MapInterface> makeReferenceBinaryGrid(
    double resolution, std::vector<mgg::NativeMolaGrid::Cell> occupied,
    std::vector<mgg::NativeMolaGrid::Cell> free,
    std::vector<mgg::NativeMolaGrid::Surface> surfaces);

#endif
