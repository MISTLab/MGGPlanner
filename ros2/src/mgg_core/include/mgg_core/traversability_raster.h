// A 2.5D traversability raster: one ground height and one state per XY cell.
//
// This is the product the full-map global planner routes over. It is a pure
// data structure with no map or ROS dependency; the map layer builds it from
// its own product and the core plans over it. Cells are axis-aligned squares
// of `cell_size` metres; cell (0, 0) has its lower-left corner at `origin`
// and cell (x, y) covers [origin + x * cell_size, origin + (x + 1) * cell_size)
// along X and likewise along Y.

#ifndef MGG_CORE_TRAVERSABILITY_RASTER_H_
#define MGG_CORE_TRAVERSABILITY_RASTER_H_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Dense>

namespace mgg {

enum class RasterCellState : std::uint8_t {
  /// No ground was observed in the cell: nothing is known about it.
  kUnknown = 0,
  /// Observed ground with nothing in the body band above it.
  kFree,
  /// An obstacle stands in the body band above the ground, or the cell lies
  /// within the body radius of one that does.
  kObstacle,
};

struct TraversabilityRaster {
  double cell_size = 0.5;
  /// World XY of the lower-left corner of cell (0, 0).
  Eigen::Vector2d origin = Eigen::Vector2d::Zero();
  /// Cells along X.
  std::size_t width = 0;
  /// Cells along Y.
  std::size_t height = 0;
  /// Ground height per cell, row-major (index = y * width + x). NaN when the
  /// cell is kUnknown. Obstacle cells keep the ground they were classified
  /// against, so an endpoint may still adopt a neighbour's height.
  std::vector<double> ground_z;
  /// Cell state, row-major like ground_z.
  std::vector<RasterCellState> state;

  std::size_t size() const { return width * height; }
  bool empty() const { return width == 0 || height == 0; }
  bool contains(std::size_t x, std::size_t y) const {
    return x < width && y < height;
  }
  std::size_t index(std::size_t x, std::size_t y) const {
    return y * width + x;
  }
  /// World XY of the centre of cell (x, y).
  Eigen::Vector2d centre(std::size_t x, std::size_t y) const {
    return origin + cell_size * Eigen::Vector2d(static_cast<double>(x) + 0.5,
                                                static_cast<double>(y) + 0.5);
  }
  /// The cell containing a world XY. False when the point lies outside the
  /// raster or is not finite.
  bool cellOf(const Eigen::Vector2d& xy, std::size_t& x,
              std::size_t& y) const {
    if (!xy.allFinite() || !std::isfinite(cell_size) || cell_size <= 0.0) {
      return false;
    }
    const double fx = std::floor((xy.x() - origin.x()) / cell_size);
    const double fy = std::floor((xy.y() - origin.y()) / cell_size);
    if (!(fx >= 0.0) || !(fy >= 0.0) || fx >= static_cast<double>(width) ||
        fy >= static_cast<double>(height)) {
      return false;
    }
    x = static_cast<std::size_t>(fx);
    y = static_cast<std::size_t>(fy);
    return true;
  }
  RasterCellState stateAt(std::size_t x, std::size_t y) const {
    return contains(x, y) ? state[index(x, y)] : RasterCellState::kUnknown;
  }
  double groundAt(std::size_t x, std::size_t y) const {
    return contains(x, y) ? ground_z[index(x, y)]
                          : std::numeric_limits<double>::quiet_NaN();
  }
  /// State of the cell containing a world XY; kUnknown outside the raster.
  RasterCellState stateAtXY(const Eigen::Vector2d& xy) const {
    std::size_t x = 0;
    std::size_t y = 0;
    return cellOf(xy, x, y) ? state[index(x, y)] : RasterCellState::kUnknown;
  }
  /// Ground height of the cell containing a world XY; NaN outside the raster
  /// or when the cell is unknown.
  double groundAtXY(const Eigen::Vector2d& xy) const {
    std::size_t x = 0;
    std::size_t y = 0;
    return cellOf(xy, x, y) ? ground_z[index(x, y)]
                            : std::numeric_limits<double>::quiet_NaN();
  }
  /// True when the arrays match the declared dimensions and the geometry is
  /// finite, so a planner can index without further checks.
  bool consistent() const {
    return std::isfinite(cell_size) && cell_size > 0.0 &&
           origin.allFinite() && ground_z.size() == size() &&
           state.size() == size();
  }
};

}  // namespace mgg

#endif  // MGG_CORE_TRAVERSABILITY_RASTER_H_
