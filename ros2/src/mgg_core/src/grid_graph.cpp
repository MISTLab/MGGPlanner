#include "mgg_core/planning_cancellation.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/departure.h"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

namespace mgg {

bool LatticeColumnGround::holds(int i, int j, double z) const {
  const auto it = heights_.find(key(i, j));
  if (it == heights_.end()) return false;
  return std::any_of(it->second.begin(), it->second.end(),
                     [this, z](double h) { return std::abs(h - z) <= step_; });
}

void LatticeColumnGround::add(int i, int j, double z) {
  heights_[key(i, j)].push_back(z);
}

GridGraphResult buildGridGraph(GraphManager& graph, const StateVec& state,
                               const GridGraphParams& grid,
                               const ExpandContext& ctx, double heading) {
  GridGraphResult result;

  // The lattice is expressed relative to the robot, so it has to straddle it.
  if (grid.min_val.x() > 0.0 || grid.min_val.y() > 0.0 ||
      grid.min_val.z() > 0.0 || grid.max_val.x() < 0.0 ||
      grid.max_val.y() < 0.0 || grid.max_val.z() < 0.0 ||
      grid.resolution.x() == 0.0 || grid.resolution.y() == 0.0 ||
      grid.resolution.z() == 0.0) {
    result.status = GridGraphStatus::kInvalidBounds;
    return result;
  }

  // Snap the bounds outward to whole cells so the robot sits on a lattice
  // point.
  Eigen::Vector3d min_val = grid.min_val;
  Eigen::Vector3d max_val = grid.max_val;
  int num_nodes[3];
  for (int i = 0; i < 3; ++i) {
    min_val[i] = -grid.resolution[i] * std::ceil(-min_val[i] / grid.resolution[i]);
    max_val[i] = grid.resolution[i] * std::ceil(max_val[i] / grid.resolution[i]);
    num_nodes[i] =
        static_cast<int>((max_val[i] - min_val[i]) / grid.resolution[i]) + 1;
    if (num_nodes[i] == 0) num_nodes[i] = 1;
  }

  const double cos_h = heading != 0.0 ? std::cos(heading) : 1.0;
  const double sin_h = heading != 0.0 ? std::sin(heading) : 0.0;

  int loop_count = 0;
  int num_vertices = 1;
  int num_edges = 0;
  int vertex_id = 1;

  // Columns nearest the robot first, so a cap leaves out the farthest; at
  // equal distance, ahead before behind.
  const int i0 = static_cast<int>(std::lround(-min_val.x() / grid.resolution.x()));
  const int j0 = static_cast<int>(std::lround(-min_val.y() / grid.resolution.y()));
  std::vector<std::tuple<long long, int, int>> columns;
  columns.reserve(static_cast<std::size_t>(num_nodes[0]) * num_nodes[1]);
  for (int i = 0; i < num_nodes[0]; ++i) {
    for (int j = 0; j < num_nodes[1]; ++j) {
      const double dx = (i - i0) * grid.resolution.x();
      const double dy = (j - j0) * grid.resolution.y();
      columns.emplace_back(std::llround((dx * dx + dy * dy) * 1e6), i, j);
    }
  }
  std::stable_sort(columns.begin(), columns.end(),
                   [](const auto& a, const auto& b) {
                     return std::make_tuple(std::get<0>(a), -std::get<1>(a), std::get<2>(a)) <
                            std::make_tuple(std::get<0>(b), -std::get<1>(b), std::get<2>(b));
                   });

  const bool ground_robot =
      ctx.robot->type == RobotType::kGroundRobot && ctx.ground != nullptr;
  LatticeColumnGround column_ground(ctx.planning->max_step_height);
  if (ground_robot) column_ground.add(i0, j0, state.z());

  // A cell whose ground was found, more than a step above or below the
  // robot's, but whose edge to its nearest vertex was refused: on another
  // level, reached only by going outward first, its way in may not exist
  // yet. Offered again once the sweep is done. Cells on the robot's own
  // level are not, which keeps the lattice as it was where there is one.
  struct Retry {
    Eigen::Vector3d cell;
    int i, j;
  };
  std::vector<Retry> retries;

  // Charges one cell to the loop budget, as upstream charged every swept
  // cell before probing it, occupied or not; false, with hit_limit, when a
  // size or loop cap stops the sweep (review r1, R1-4).
  const auto charge = [&]() {
    planningCheckpoint();
    if (loop_count++ > ctx.planning->num_loops_max ||
        num_vertices >= ctx.planning->num_vertices_max ||
        num_edges >= ctx.planning->num_edges_max) {
      result.hit_limit = true;
      return false;
    }
    return true;
  };

  // Only lattice routing weights carry clearance; roadmap geometry and the
  // physical tree distance retain metres. GroundProjection is plan-scoped.
  ExpandContext weighted_ctx = ctx;
  if (ground_robot && ctx.planning->path_clearance_margin > 0.0) {
    weighted_ctx.edge_cost =
        [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
          return ctx.ground->clearanceCost(a, b, ctx.robot_box_size);
        };
  }

  // Offers one cell, already charged, to expandGraph.
  const auto offer = [&](const Eigen::Vector3d& cell, int i, int j,
                         bool first_pass, bool& added) {
    added = false;
    // The ground this cell would be dropped onto, as expandGraph drops it:
    // a vertex at its column already on that ground makes it the same place
    // again.
    bool other_level = false;
    if (ground_robot) {
      Eigen::Vector3d sample = cell;
      VoxelStatus ground_status;
      const double ground_height =
          ctx.ground->projectSample(sample, ground_status);
      const double driving_z =
          cell.z() - (ground_height - ctx.planning->max_ground_height);
      if (ground_status == VoxelStatus::kOccupied &&
          column_ground.holds(i, j, driving_z)) {
        if (first_pass) ++result.merged_duplicates;
        return false;
      }
      other_level = ground_status == VoxelStatus::kOccupied &&
                    std::abs(driving_z - state.z()) >
                        ctx.planning->max_step_height;
    }

    Vertex candidate(vertex_id++, StateVec(cell.x(), cell.y(), cell.z(), heading));
    candidate.robot_id = ctx.robot_id;
    ExpandGraphReport rep;
    expandGraph(graph, candidate, rep, weighted_ctx);
    if (first_pass) {
      ++result.rejected[static_cast<int>(rep.status)];
      if (rep.no_ground) ++result.no_ground;
      if (rep.projected_endpoint_status == VoxelStatus::kOccupied) {
        ++result.projected_endpoint_occupied;
      } else if (rep.projected_endpoint_status == VoxelStatus::kUnknown) {
        ++result.projected_endpoint_unknown;
      }
      for (int e = 0; e < 8; ++e) result.edge_status[e] += rep.edge_status[e];
    }
    if (rep.status == ExpandGraphStatus::kSuccess) {
      added = true;
      num_vertices += rep.num_vertices_added;
      num_edges += rep.num_edges_added;
      result.vertices_added += rep.num_vertices_added;
      result.edges_added += rep.num_edges_added;
      // A vertex clipped short of its cell stands elsewhere.
      if (ground_robot && rep.vertex_added != nullptr &&
          (rep.vertex_added->state.head<2>() - cell.head<2>()).norm() < 1e-6) {
        column_ground.add(i, j, rep.vertex_added->state.z());
      }
    } else if (first_pass && other_level && !rep.no_ground &&
               rep.status == ExpandGraphStatus::kErrorCollisionEdge) {
      retries.push_back({cell, i, j});
    }
    return rep.edge_status[static_cast<int>(ProjectedEdgeStatus::kOccupied)] > 0;
  };

  const auto try_cell = [&](const Eigen::Vector3d& candidate, int i, int j,
                            bool first_pass, bool& added) {
    OrientedBox body;
    body.heading = heading;
    StateVec query(candidate.x(), candidate.y(), candidate.z(), heading);
    Vertex* nearest = nullptr;
    if (ground_robot && graph.getNearestVertex(&query, &nearest) &&
        nearest != nullptr &&
        (candidate.head<2>() - nearest->state.head<2>()).norm() > 1e-9) {
      body.heading = std::atan2(candidate.y() - nearest->state.y(),
                                candidate.x() - nearest->state.x());
    }
    body.size = ctx.robot_box_size;
    const Eigen::Vector3d center = candidate + ctx.robot->center_offset;
    VoxelStatus status = ctx.robot->type == RobotType::kAerialRobot
        ? ctx.map->getStrictBoxStatus(center, ctx.robot_box_size)
        : ctx.map->getBoxStatus(center, ctx.robot_box_size,
                                !ctx.allow_unknown_lattice_body);
    if (ground_robot && status == VoxelStatus::kOccupied &&
        !ctx.map->dynamicBoxBlocked(center, ctx.robot_box_size)) {
      status = orientedBoxPathStatus(*ctx.map, center, center, body,
                                     !ctx.allow_unknown_lattice_body, nullptr);
    }
    if (status != VoxelStatus::kFree) return status == VoxelStatus::kOccupied;
    if (first_pass) ++result.free_cells;
    return offer(candidate, i, j, first_pass, added);
  };

  std::vector<Retry> nudges;
  for (const auto& [unused_distance, i, j] : columns) {
    (void)unused_distance;
    for (int k = 0; k < num_nodes[2]; ++k) {
      if (!charge()) return result;
      double x_val = min_val.x() + i * grid.resolution.x();
      double y_val = min_val.y() + j * grid.resolution.y();
      const double z_val = min_val.z() + k * grid.resolution.z();
      if (heading != 0.0) {
        const double rx = x_val * cos_h - y_val * sin_h;
        const double ry = x_val * sin_h + y_val * cos_h;
        x_val = rx;
        y_val = ry;
      }
      x_val += state.x();
      y_val += state.y();
      const double z_world = z_val + state.z();

      const Eigen::Vector3d cell(x_val, y_val, z_world);
      bool added = false;
      const bool refused = try_cell(cell, i, j, /*first_pass=*/true, added);
      if (ground_robot && refused && !added) nudges.push_back({cell, i, j});
    }
  }

  // Retry passes, nearest first again, while the last one joined a vertex.
  for (int pass = 0; pass < kGridGraphRetryPasses && !retries.empty(); ++pass) {
    std::vector<Retry> left;
    int joined = 0;
    for (const Retry& retry : retries) {
      // Retries are charged to the same budget.
      if (!charge()) return result;
      bool added = false;
      offer(retry.cell, retry.i, retry.j, /*first_pass=*/false, added);
      if (added) {
        ++joined;
      } else {
        left.push_back(retry);
      }
    }
    result.retried_joined += joined;
    retries.swap(left);
    if (joined == 0) break;
  }
  // Original cells and other-level links get their first chance before
  // nudges spend any of the shared budget or change nearest neighbours.
  for (const Retry& retry : nudges) {
    for (double offset : {0.1, -0.1, 0.2, -0.2}) {
      if (!charge()) return result;
      bool added = false;
      const Eigen::Vector3d shifted =
          retry.cell + offset * Eigen::Vector3d(-sin_h, cos_h, 0.0);
      try_cell(shifted, retry.i, retry.j, /*first_pass=*/false, added);
      if (added) break;
    }
  }
  return result;
}

}  // namespace mgg
