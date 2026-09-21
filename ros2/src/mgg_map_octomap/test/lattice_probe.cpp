// Builds the local lattice against a real MOLA product, as the planner node
// does, and reports what the map says under and around the robot. Usage:
//   lattice_probe <peer_root> <component_id> <geometry_revision> \
//                 <T_component_navigation 16 row-major> <x> <y> <z> <yaw>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "mgg_core/graph_expansion.h"
#include "mgg_core/graph_manager.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/ground_projection.h"
#include "mgg_map_octomap/mola_map.h"

int main(int argc, char** argv) {
  if (argc < 24) {
    std::fprintf(stderr, "usage: see source\n");
    return 2;
  }
  mgg::MolaMapConfig cfg;
  cfg.peer_root = argv[1];
  cfg.resolution = 0.2;
  cfg.max_grid_bytes = 256 * 1024 * 1024;
  cfg.max_voxels = 2000000;
  cfg.max_load_time = std::chrono::milliseconds(10000);
  mgg::MolaMap map(cfg);
  mgg::MolaSnapshotRequest request;
  request.component_id = argv[2];
  request.geometry_revision = argv[3];
  request.graph_revision = std::strtoull(std::getenv("GRAPH_REV"), nullptr, 10);
  request.source_stamp_ns = std::strtoull(std::getenv("SOURCE_STAMP_NS"), nullptr, 10);
  Eigen::Matrix4d m;
  for (int i = 0; i < 16; ++i) m(i / 4, i % 4) = std::atof(argv[4 + i]);
  request.component_from_navigation = Eigen::Isometry3d::Identity();
  request.component_from_navigation.matrix() = m;
  map.requestSnapshot(request);
  for (int i = 0; i < 200 && !map.getStatus(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!map.getStatus()) {
    std::fprintf(stderr, "map unavailable: %s\n", map.lastError().c_str());
    return 1;
  }
  auto lease = map.acquireReadLease();
  const mgg::StateVec state(std::atof(argv[20]), std::atof(argv[21]),
                            std::atof(argv[22]), std::atof(argv[23]));

  mgg::PlanningParams planning;
  planning.max_ground_height = 0.525;
  planning.max_step_height = 0.15;
  planning.max_inclination = 0.5236;
  planning.edge_length_min = 0.05;
  planning.edge_length_max = 3.0;
  planning.edge_overshoot = 0.0;
  planning.num_vertices_max = 1500;
  planning.num_edges_max = 30000;
  planning.num_loops_max = 100000;
  mgg::RobotParams robot;
  robot.type = mgg::RobotType::kGroundRobot;
  robot.size = Eigen::Vector3d(1.023, 0.778, 0.4);
  robot.size_extension = Eigen::Vector3d(0.05, 0.05, 0.05);
  robot.size_extension_min.setZero();
  robot.safety_extension = Eigen::Vector3d(0.5, 0.75, 0.05);
  robot.bound_mode = mgg::BoundModeType::kExtendedBound;
  mgg::GroundProjection ground(map, planning);
  mgg::GeofenceManager geofence;
  mgg::GridGraphParams grid;
  grid.min_val = Eigen::Vector3d(-6.0, -6.0, -0.2);
  grid.max_val = Eigen::Vector3d(6.0, 6.0, 0.3);
  grid.resolution = Eigen::Vector3d(0.4, 0.4, 0.1);

  // What the map says at the robot and around it.
  Eigen::Vector3d here = state.head<3>();
  mgg::VoxelStatus vs;
  double gh = ground.projectSample(here, vs);
  std::printf("robot base (%.2f, %.2f, %.2f): ground status %d height %.3f -> projected z %.3f\n",
              state.x(), state.y(), state.z(), static_cast<int>(vs), gh,
              vs == mgg::VoxelStatus::kOccupied ? here.z() - (gh - planning.max_ground_height) : NAN);
  for (double r : {0.4, 0.8, 1.2, 2.0, 3.0}) {
    int occ = 0, unk = 0, fr = 0;
    double zsum = 0; int zn = 0;
    for (int k = 0; k < 16; ++k) {
      const double a = k * M_PI / 8.0;
      Eigen::Vector3d p(state.x() + r * std::cos(a), state.y() + r * std::sin(a), state.z());
      double g = ground.projectSample(p, vs);
      if (vs == mgg::VoxelStatus::kOccupied) { ++occ; zsum += p.z() - (g - planning.max_ground_height); ++zn; }
      else if (vs == mgg::VoxelStatus::kUnknown) ++unk; else ++fr;
    }
    std::printf("ring r=%.1f: ground occupied %d unknown %d free %d; mean driving z %.3f\n", r, occ, unk, fr, zn ? zsum / zn : NAN);
  }
  Eigen::Vector3d box = robot.getPlanningSize();
  std::printf("body box at base: strict %d, tolerant %d (box %.2f %.2f %.2f)\n",
              static_cast<int>(map.getBoxStatus(here, box, true)),
              static_cast<int>(map.getBoxStatus(here, box, false)), box.x(), box.y(), box.z());

  // Voxel columns around the root: which z layers are occupied per xy cell,
  // over the footprint the wider box adds.
  for (double dy : {-0.55, -0.45, -0.35, 0.35, 0.45, 0.55}) {
    std::printf("column dy=%.2f:", dy);
    for (double z = -0.4; z <= 0.8; z += 0.2) {
      Eigen::Vector3d p(state.x() + 0.0, state.y() + dy, z);
      std::printf(" z%.1f=%d", z, static_cast<int>(map.getVoxelStatus(p)));
    }
    std::printf("\n");
  }
  for (double dx : {-0.6, -0.5, 0.5, 0.6}) {
    std::printf("column dx=%.2f:", dx);
    for (double z = -0.4; z <= 0.8; z += 0.2) {
      Eigen::Vector3d p(state.x() + dx, state.y(), z);
      std::printf(" z%.1f=%d", z, static_cast<int>(map.getVoxelStatus(p)));
    }
    std::printf("\n");
  }
  // The sweep the lattice makes from the root to its east neighbour.
  for (double bxy : {0.0, 0.05}) {
    Eigen::Vector3d b(1.023 + bxy, 0.778 + bxy, 0.4);
    Eigen::Vector3d a(state.x(), state.y(), 0.198), e(state.x() + 0.4, state.y(), 0.225);
    std::printf("sweep box %.3f: path %d box_a %d box_e %d\n", b.x(),
                static_cast<int>(map.getPathStatus(a, e, b, false)),
                static_cast<int>(map.getBoxStatus(a, b, false)),
                static_cast<int>(map.getBoxStatus(e, b, false)));
  }
  for (int variant = 0; variant < 2; ++variant) {
    {
      const double reach = 3.0; const bool preserve = true;
      robot.bound_mode = variant == 0 ? mgg::BoundModeType::kExtendedBound : mgg::BoundModeType::kExactBound;
      std::printf("variant %s: ", variant == 0 ? "extended" : "exact");
      mgg::GraphManager graph;
      mgg::EdgeInclinations incl;
      mgg::ExpandContext ctx;
      ctx.map = &map; ctx.planning = &planning; ctx.robot = &robot; ctx.ground = &ground;
      ctx.geofence = &geofence; ctx.inclinations = &incl; ctx.robot_id = 1;
      box = robot.getPlanningSize(); ctx.robot_box_size = box;
      ctx.allow_unknown_lattice_body = true;
      ctx.hanging_root_edge_length_max = reach;
      ctx.preserve_hanging_root_start_height = preserve; ctx.root_footprint_exempt = true;
      mgg::StateVec root_state = state;
      Eigen::Vector3d pos = state.head<3>();
      double g = ground.projectSample(pos, vs);
      bool hanging = vs != mgg::VoxelStatus::kOccupied;
      if (!hanging) root_state[2] = pos.z() - (g - planning.max_ground_height);
      else root_state[2] += planning.max_ground_height - robot.size[2] / 2.0;
      auto* root = new mgg::Vertex(0, root_state);
      root->robot_id = 1; root->is_hanging = hanging;
      graph.addVertex(root);
      const mgg::GridGraphResult r = mgg::buildGridGraph(graph, root_state, grid, ctx, state[3]);
      std::printf("root z %.3f hanging %d -> free cells %d, vertices %d, edges %d; edge status ok %d steep %d occupied %d unmapped %d hanging %d; no_ground %d\n",
                  root_state[2], hanging, r.free_cells, r.vertices_added, r.edges_added,
                  r.edge_status[0], r.edge_status[1], r.edge_status[2], r.edge_status[3], r.edge_status[4], r.no_ground);
    }
  }
  return 0;
}
