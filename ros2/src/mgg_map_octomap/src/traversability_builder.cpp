#include "mgg_map_octomap/traversability_builder.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "mgg_core/map_interface.h"

namespace mgg {
namespace {

/// Bound on the cells a raster may hold: four million is a one kilometre
/// square at 0.5 m, well beyond one component of a MOLA map.
constexpr std::size_t kMaxRasterCells = std::size_t{4} * 1024u * 1024u;
/// Free-voxel layers tracked above the ground for the clearance rule.
constexpr int kClearanceLayers = 32;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

bool validParams(const RasterParams& params) {
  return std::isfinite(params.cell_size_m) && params.cell_size_m > 0.0 &&
         std::isfinite(params.body_radius_m) && params.body_radius_m >= 0.0 &&
         std::isfinite(params.max_step_height_m) &&
         params.max_step_height_m >= 0.0 &&
         std::isfinite(params.body_height_m) &&
         params.body_height_m >= params.max_step_height_m &&
         std::isfinite(params.min_clearance_m) && params.min_clearance_m >= 0.0 &&
         std::isfinite(params.step_tolerance_m) && params.step_tolerance_m >= 0.0;
}

}  // namespace

std::shared_ptr<const TraversabilityRaster> buildTraversabilityRaster(
    const NativeMolaGrid& grid, const RasterParams& params,
    const Eigen::Isometry3d& raster_from_grid) {
  const double resolution = grid.getResolution();
  if (!validParams(params) || !std::isfinite(resolution) || resolution <= 0.0 ||
      !raster_from_grid.matrix().allFinite() ||
      !authorityTiltAcceptable(raster_from_grid.linear())) {
    return nullptr;
  }
  const auto& occupied = grid.occupiedCells();
  const auto& surfaces = grid.surfaceMaxZ();
  if (occupied.empty() && surfaces.empty()) return nullptr;

  const double half = 0.5 * resolution;
  const auto centreOf = [&](const NativeMolaGrid::Cell& cell) {
    return Eigen::Vector3d(resolution * (static_cast<double>(cell.x) + 0.5),
                           resolution * (static_cast<double>(cell.y) + 0.5),
                           resolution * (static_cast<double>(cell.z) + 0.5));
  };
  const auto place = [&](const Eigen::Vector3d& point) {
    return Eigen::Vector3d(raster_from_grid * point);
  };

  // Extent of the raster: every occupied voxel and every surface, placed in
  // the raster frame, with a one-cell margin so the border cells are whole.
  Eigen::Vector2d min_xy = Eigen::Vector2d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector2d max_xy = -min_xy;
  const auto extend = [&](const Eigen::Vector3d& point) {
    min_xy = min_xy.cwiseMin(point.head<2>());
    max_xy = max_xy.cwiseMax(point.head<2>());
  };
  for (const auto& cell : occupied) extend(place(centreOf(cell)));
  for (const auto& entry : surfaces) {
    const Eigen::Vector3d centre = centreOf(entry.first);
    extend(place(Eigen::Vector3d(centre.x(), centre.y(), entry.second)));
  }
  if (!min_xy.allFinite() || !max_xy.allFinite()) return nullptr;

  const double cell = params.cell_size_m;
  auto raster = std::make_shared<TraversabilityRaster>();
  raster->cell_size = cell;
  raster->origin =
      Eigen::Vector2d(std::floor(min_xy.x() / cell) - 1.0,
                      std::floor(min_xy.y() / cell) - 1.0) *
      cell;
  const double width_d = std::ceil((max_xy.x() - raster->origin.x()) / cell) + 2.0;
  const double height_d = std::ceil((max_xy.y() - raster->origin.y()) / cell) + 2.0;
  if (!std::isfinite(width_d) || !std::isfinite(height_d) || width_d < 1.0 ||
      height_d < 1.0 || width_d * height_d > static_cast<double>(kMaxRasterCells)) {
    return nullptr;
  }
  raster->width = static_cast<std::size_t>(width_d);
  raster->height = static_cast<std::size_t>(height_d);
  const std::size_t cells = raster->size();
  if (cells == 0 || cells > kMaxRasterCells) return nullptr;
  raster->ground_z.assign(cells, kNaN);
  raster->state.assign(cells, RasterCellState::kUnknown);
  const auto indexOf = [&](const Eigen::Vector3d& point, std::size_t& index) {
    std::size_t x = 0;
    std::size_t y = 0;
    if (!raster->cellOf(point.head<2>(), x, y)) return false;
    index = raster->index(x, y);
    return true;
  };

  // Ground: the lowest surface height among the voxels the cell covers.
  for (const auto& entry : surfaces) {
    if (!std::isfinite(entry.second)) continue;
    const Eigen::Vector3d centre = centreOf(entry.first);
    const Eigen::Vector3d top =
        place(Eigen::Vector3d(centre.x(), centre.y(), entry.second));
    std::size_t index = 0;
    if (!indexOf(top, index)) continue;
    double& ground = raster->ground_z[index];
    if (!(ground <= top.z())) ground = top.z();
  }

  // Obstacles: an occupied voxel whose top rises above the step band and
  // whose bottom lies below the body band. The top is the measured surface
  // when the voxel has one, otherwise the voxel's geometric top.
  std::vector<std::uint8_t> obstacle(cells, 0);
  const double step_band = params.max_step_height_m + params.step_tolerance_m;
  for (const auto& voxel : occupied) {
    const Eigen::Vector3d centre = place(centreOf(voxel));
    std::size_t index = 0;
    if (!indexOf(centre, index)) continue;
    const double ground = raster->ground_z[index];
    if (!std::isfinite(ground)) continue;
    const auto surface = surfaces.find(voxel);
    double top = centre.z() + half;
    if (surface != surfaces.end() && std::isfinite(surface->second)) {
      const Eigen::Vector3d grid_centre = centreOf(voxel);
      top = place(Eigen::Vector3d(grid_centre.x(), grid_centre.y(),
                                  surface->second))
                .z();
    }
    const double bottom = centre.z() - half;
    if (top > ground + step_band && bottom < ground + params.body_height_m) {
      obstacle[index] = 1;
    }
  }

  // Clearance: with a requirement, the free voxels above the ground must
  // cover every layer up to it. Layer 0 is the voxel above the ground voxel.
  std::vector<std::uint32_t> free_layers;
  int required_layers = 0;
  if (params.min_clearance_m > 0.0) {
    required_layers = static_cast<int>(std::ceil(params.min_clearance_m / resolution - 1e-9));
    required_layers = std::clamp(required_layers, 1, kClearanceLayers);
    free_layers.assign(cells, 0u);
    for (const auto& voxel : grid.freeCells()) {
      const Eigen::Vector3d centre = place(centreOf(voxel));
      std::size_t index = 0;
      if (!indexOf(centre, index)) continue;
      const double ground = raster->ground_z[index];
      if (!std::isfinite(ground)) continue;
      const double relative = centre.z() - half - ground;
      if (relative <= -half) continue;
      const int layer = static_cast<int>(
          std::floor(std::max(0.0, relative) / resolution - 1e-6));
      if (layer < 0 || layer >= kClearanceLayers) continue;
      free_layers[index] |= 1u << static_cast<unsigned>(layer);
    }
  }

  for (std::size_t index = 0; index < cells; ++index) {
    if (!std::isfinite(raster->ground_z[index])) continue;
    if (obstacle[index] != 0) {
      raster->state[index] = RasterCellState::kObstacle;
      continue;
    }
    if (required_layers > 0) {
      const std::uint32_t needed =
          required_layers >= 32 ? 0xffffffffu
                                : ((1u << static_cast<unsigned>(required_layers)) - 1u);
      if ((free_layers[index] & needed) != needed) continue;
    }
    raster->state[index] = RasterCellState::kFree;
  }

  // Inflation: a free cell within the body radius of an obstacle cell is
  // marked inflated (the body may not fit there; the planner decides what
  // that costs). The distance is from the cell centre to the nearest point
  // of the obstacle cell's square, so the ring of neighbours counts as soon
  // as the body reaches past half a cell.
  struct Offset {
    long dx;
    long dy;
  };
  std::vector<Offset> offsets;
  const long reach = static_cast<long>(std::ceil(params.body_radius_m / cell)) + 1;
  for (long dy = -reach; dy <= reach; ++dy) {
    for (long dx = -reach; dx <= reach; ++dx) {
      if (dx == 0 && dy == 0) continue;
      const double gap_x = std::max(0.0, (std::abs(static_cast<double>(dx)) - 0.5) * cell);
      const double gap_y = std::max(0.0, (std::abs(static_cast<double>(dy)) - 0.5) * cell);
      if (std::hypot(gap_x, gap_y) <= params.body_radius_m + 1e-9) {
        offsets.push_back({dx, dy});
      }
    }
  }
  if (!offsets.empty()) {
    for (std::size_t index = 0; index < cells; ++index) {
      if (obstacle[index] == 0) continue;
      const long x = static_cast<long>(index % raster->width);
      const long y = static_cast<long>(index / raster->width);
      for (const Offset& offset : offsets) {
        const long nx = x + offset.dx;
        const long ny = y + offset.dy;
        if (nx < 0 || ny < 0 || nx >= static_cast<long>(raster->width) ||
            ny >= static_cast<long>(raster->height)) {
          continue;
        }
        const std::size_t neighbour = raster->index(
            static_cast<std::size_t>(nx), static_cast<std::size_t>(ny));
        if (raster->state[neighbour] == RasterCellState::kFree) {
          raster->state[neighbour] = RasterCellState::kInflated;
        }
      }
    }
  }
  return raster;
}

}  // namespace mgg
