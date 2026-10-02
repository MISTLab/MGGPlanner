#include <set>
#include "mgg_core/planning_cancellation.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/departure.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace mgg {

namespace {

/// Exact finite input bits: sub-cell rounding can cross a voxel boundary.
std::int64_t exactBits(double value) {
  std::int64_t bits;
  if (value == 0.0) value = 0.0;  // canonicalize signed zero
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

struct KeyHash {
  std::size_t operator()(const std::array<std::int64_t, 4>& key) const {
    std::size_t hash = 0;
    for (const std::int64_t part : key) {
      hash ^= std::hash<std::int64_t>()(part) + 0x9e3779b97f4a7c15ULL +
              (hash << 6) + (hash >> 2);
    }
    return hash;
  }
};

}  // namespace

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
  PlanProfile* const profile = ctx.ground ? ctx.ground->profile() : nullptr;
  ProfileScope timed_lattice(profile ? &profile->lattice : nullptr);
  LatticeColumnGround column_ground(ctx.planning->max_step_height);
  if (ground_robot) column_ground.add(i0, j0, state.z());

  // Only offer columns beside a connected vertex. A disconnected column
  // is deferred, not discarded: connecting around a wall schedules it.
  // Ground projection/body queries for unreachable space used the entire
  // soft slice even after the connected component stopped growing.
  std::set<std::size_t> ready_columns;
  std::vector<bool> scheduled(columns.size(), false);
  const auto schedule_near = [&](const Eigen::Vector2d& position, double reach) {
    for (std::size_t n = 0; n < columns.size(); ++n) {
      if (scheduled[n]) continue;
      const auto [distance, i, j] = columns[n];
      (void)distance;
      const double x = (i-i0)*grid.resolution.x(), y = (j-j0)*grid.resolution.y();
      const Eigen::Vector2d world = state.head<2>() +
          Eigen::Vector2d(cos_h*x-sin_h*y, sin_h*x+cos_h*y);
      if ((world-position).norm() > reach+1e-9) continue;
      scheduled[n] = true;
      ready_columns.insert(n);
    }
  };
  if (ground_robot) schedule_near(state.head<2>(),
      std::max(ctx.planning->edge_length_max, ctx.hanging_root_edge_length_max));
  else for (std::size_t n=0; n<columns.size(); ++n) ready_columns.insert(n);

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
    if (ctx.deadline && std::chrono::steady_clock::now() >= *ctx.deadline) {
      result.hit_limit = true;
      result.hit_deadline = true;
      if (profile != nullptr) profile->budget_exhausted = true;
      return false;
    }
    return true;
  };

  // Only lattice routing weights carry clearance; roadmap geometry and the
  // physical tree distance retain metres. GroundProjection is plan-scoped.
  ExpandContext weighted_ctx = ctx;
  EdgeVerdictCache build_verdicts;
  if (ground_robot && weighted_ctx.edge_verdicts == nullptr) {
    weighted_ctx.edge_verdicts = &build_verdicts;
  }
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
    std::optional<double> ground_driving_z;
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
      if (ground_status == VoxelStatus::kOccupied) ground_driving_z = driving_z;
      other_level = ground_status == VoxelStatus::kOccupied &&
                    std::abs(driving_z - state.z()) >
                        ctx.planning->max_step_height;
    }

    Vertex candidate(vertex_id++, StateVec(cell.x(), cell.y(), cell.z(), heading));
    candidate.robot_id = ctx.robot_id;
    ExpandGraphReport rep;
    expandGraph(graph, candidate, rep, weighted_ctx);
    // Refused from its nearest vertex, a ground cell may still join from
    // another vertex beside it, nearest first: a cell past an obstacle's
    // corner, or on a level whose vertices the cell's lattice height is
    // farther from than another level's (kLatticeAlternateParents).
    ExpandGraphReport alternate;
    if (ground_robot && rep.status == ExpandGraphStatus::kErrorCollisionEdge &&
        !rep.no_ground && kLatticeAlternateParents > 0) {
      // Around where the cell's body will stand, at driving height over its
      // ground, not its lattice height: on another level the vertices of
      // that level are the ones it joins.
      StateVec standing = candidate.state;
      if (ground_driving_z) standing[2] = *ground_driving_z;
      std::vector<Vertex*> around;
      if (graph.getNearestVertices(&standing, ctx.planning->edge_length_max,
                                   &around)) {
        Vertex* tried_parent = nullptr;
        graph.getNearestVertex(&candidate.state, &tried_parent);
        std::sort(around.begin(), around.end(),
                  [&standing](const Vertex* a, const Vertex* b) {
                    return (a->state.head<3>() - standing.head<3>())
                               .squaredNorm() <
                           (b->state.head<3>() - standing.head<3>())
                               .squaredNorm();
                  });
        int alternates = 0;
        for (Vertex* parent : around) {
          if (parent == nullptr || parent == tried_parent) continue;
          if (alternates++ >= kLatticeAlternateParents) break;
          if (profile != nullptr) ++profile->alternate_parents;
          alternate = ExpandGraphReport();
          expandGraphFrom(graph, candidate, parent, alternate, weighted_ctx);
          if (alternate.status == ExpandGraphStatus::kSuccess) break;
        }
      }
    }
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
    if (rep.status != ExpandGraphStatus::kSuccess &&
        alternate.status == ExpandGraphStatus::kSuccess) {
      rep.status = alternate.status;
      rep.num_vertices_added = alternate.num_vertices_added;
      rep.num_edges_added = alternate.num_edges_added;
      rep.vertex_added = alternate.vertex_added;
      if (first_pass) ++result.joined_from_alternate;
    }
    if (rep.status == ExpandGraphStatus::kSuccess) {
      added = true;
      num_vertices += rep.num_vertices_added;
      num_edges += rep.num_edges_added;
      result.vertices_added += rep.num_vertices_added;
      if (ground_robot && rep.vertex_added)
        schedule_near(rep.vertex_added->state.head<2>(), ctx.planning->edge_length_max);
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

  // Precheck verdicts by where the body stands and how it is turned: the
  // z levels of a column that drop onto one ground, and its nudges' repeats,
  // are checked once (exact: the key is every input of the check).
  std::unordered_map<std::array<std::int64_t, 4>, VoxelStatus, KeyHash>
      prechecks;

  // A ground robot's cell is checked where its body will stand: at the
  // driving height over the ground under it (`driving_z`, when found), so
  // every z level of a column that drops onto the same ground gets the same
  // verdict, checked once (`prechecks`).
  const auto try_cell = [&](const Eigen::Vector3d& candidate, int i, int j,
                            bool first_pass, bool& added,
                            std::optional<double> driving_z) {
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
    Eigen::Vector3d center = candidate + ctx.robot->offsetForHeading(body.heading);
    if (driving_z) center.z() = *driving_z + ctx.robot->center_offset.z();
    std::optional<ProfileScope> timed_precheck;
    timed_precheck.emplace(profile ? &profile->cell_prechecks : nullptr);
    const std::array<std::int64_t, 4> precheck_key{
        exactBits(center.x()), exactBits(center.y()), exactBits(center.z()),
        exactBits(body.heading)};
    const auto known = ground_robot ? prechecks.find(precheck_key)
                                    : prechecks.end();
    VoxelStatus status = VoxelStatus::kUnknown;
    if (known != prechecks.end()) {
      status = known->second;
      if (profile != nullptr) ++profile->precheck_cache_hits;
    } else {
      if (ground_robot && (ctx.unknown_body_above_center || ctx.own_body_known_free)) {
        status = orientedBoxPathStatus(*ctx.map, center, center, body, !ctx.allow_unknown_lattice_body,
                                       nullptr, false, ctx.unknown_body_above_center, ctx.own_body_known_free.get());
      } else status = ctx.robot->type == RobotType::kAerialRobot
          ? ctx.map->getStrictBoxStatus(center, ctx.robot_box_size)
          : ctx.map->getBoxStatus(center, ctx.robot_box_size,
                                  !ctx.allow_unknown_lattice_body);
      if (ground_robot && !ctx.unknown_body_above_center && status == VoxelStatus::kOccupied &&
          !ctx.map->dynamicBoxBlocked(center, ctx.robot_box_size)) {
        status = orientedBoxPathStatus(*ctx.map, center, center, body,
                                       !ctx.allow_unknown_lattice_body,
                                       nullptr);
      }
      if (ground_robot) prechecks.emplace(precheck_key, status);
    }
    timed_precheck.reset();
    if (status != VoxelStatus::kFree) return status == VoxelStatus::kOccupied;
    if (first_pass) ++result.free_cells;
    return offer(candidate, i, j, first_pass, added);
  };

  // The ground a cell drops onto, at driving height; nothing when none is
  // mapped under it.
  const auto drivingHeight = [&](const Eigen::Vector3d& cell) {
    std::optional<double> driving_z;
    if (!ground_robot) return driving_z;
    Eigen::Vector3d sample = cell;
    VoxelStatus ground_status = VoxelStatus::kUnknown;
    const double ground_height =
        ctx.ground->projectSample(sample, ground_status);
    if (ground_status == VoxelStatus::kOccupied) {
      driving_z = cell.z() - (ground_height - ctx.planning->max_ground_height);
    }
    return driving_z;
  };

  std::vector<Retry> nudges;
  do {
  while (!ready_columns.empty()) {
    planningCheckpoint();
    const auto [unused_distance, i, j] = columns[*ready_columns.begin()];
    ready_columns.erase(ready_columns.begin());
    (void)unused_distance;
    // A ground robot's column is projected from its top level first: the
    // levels below start on that ray's way down and meet the same ground
    // (projectSample's column cache).
    if (ground_robot && num_nodes[2] > 1) {
      double x_val = min_val.x() + i * grid.resolution.x();
      double y_val = min_val.y() + j * grid.resolution.y();
      if (heading != 0.0) {
        const double rx = x_val * cos_h - y_val * sin_h;
        const double ry = x_val * sin_h + y_val * cos_h;
        x_val = rx;
        y_val = ry;
      }
      drivingHeight(Eigen::Vector3d(
          x_val + state.x(), y_val + state.y(),
          min_val.z() + (num_nodes[2] - 1) * grid.resolution.z() +
              state.z())).has_value();
    }
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
      const std::optional<double> driving_z = drivingHeight(cell);
      bool added = false;
      const bool refused =
          try_cell(cell, i, j, /*first_pass=*/true, added, driving_z);
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
      if (profile != nullptr) ++profile->retries;
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
  // A nudge is for a cell beside the lattice, that a vertex within an edge
  // could join where it is; farther out its edge would be clipped to where
  // the sweep's own cells already were.
  const double nudge_reach = ctx.planning->edge_length_max + 0.2;
  for (const Retry& retry : nudges) {
    StateVec query(retry.cell.x(), retry.cell.y(), retry.cell.z(), heading);
    Vertex* nearest = nullptr;
    if (!graph.getNearestVertex(&query, &nearest) || nearest == nullptr ||
        (nearest->state.head<2>() - retry.cell.head<2>()).norm() >
            nudge_reach) {
      continue;
    }
    for (double offset : {0.1, -0.1, 0.2, -0.2}) {
      if (!charge()) return result;
      if (profile != nullptr) ++profile->nudges;
      bool added = false;
      const Eigen::Vector3d shifted =
          retry.cell + offset * Eigen::Vector3d(-sin_h, cos_h, 0.0);
      try_cell(shifted, retry.i, retry.j, /*first_pass=*/false, added,
               drivingHeight(shifted));
      if (added) break;
    }
  }
  nudges.clear();
  retries.clear();
  } while (!ready_columns.empty());
  return result;
}

}  // namespace mgg
