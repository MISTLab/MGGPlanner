#include "mgg_core/grid_graph.h"

#include <cmath>

namespace mgg {

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

  for (int i = 0; i < num_nodes[0]; ++i) {
    for (int j = 0; j < num_nodes[1]; ++j) {
      for (int k = 0; k < num_nodes[2]; ++k) {
        if (loop_count++ > ctx.planning->num_loops_max ||
            num_vertices >= ctx.planning->num_vertices_max ||
            num_edges >= ctx.planning->num_edges_max) {
          result.hit_limit = true;
          return result;
        }

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
        const VoxelStatus body_status = ctx.map->getBoxStatus(
            cell + ctx.robot->center_offset, ctx.robot_box_size,
            !ctx.allow_unknown_lattice_body);
        if (body_status != VoxelStatus::kFree) {
          continue;
        }
        ++result.free_cells;

        Vertex candidate(vertex_id++, StateVec(x_val, y_val, z_world, heading));
        candidate.robot_id = ctx.robot_id;
        ExpandGraphReport rep;
        expandGraph(graph, candidate, rep, ctx);
        ++result.rejected[static_cast<int>(rep.status)];
        if (rep.no_ground) ++result.no_ground;
        if (rep.projected_endpoint_status == VoxelStatus::kOccupied) {
          ++result.projected_endpoint_occupied;
        } else if (rep.projected_endpoint_status == VoxelStatus::kUnknown) {
          ++result.projected_endpoint_unknown;
        }
        for (int e = 0; e < 7; ++e) result.edge_status[e] += rep.edge_status[e];
        if (rep.status == ExpandGraphStatus::kSuccess) {
          num_vertices += rep.num_vertices_added;
          num_edges += rep.num_edges_added;
          result.vertices_added += rep.num_vertices_added;
          result.edges_added += rep.num_edges_added;
        }
      }
    }
  }
  return result;
}

}  // namespace mgg
