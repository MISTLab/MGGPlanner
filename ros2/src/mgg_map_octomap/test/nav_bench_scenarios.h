// Botman's NAVIGATE and exploration scenarios on a MOLA planning product,
// shared by the mgg_nav_bench binary and the NavBench tests.

#ifndef MGG_MAP_OCTOMAP_TEST_NAV_BENCH_SCENARIOS_H_
#define MGG_MAP_OCTOMAP_TEST_NAV_BENCH_SCENARIOS_H_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "mgg_core/graph_manager.h"
#include "mgg_core/planning_cancellation.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/ground_projection.h"
#include "mgg_core/local_route.h"
#include "mgg_core/departure.h"
#include "mgg_core/path_selection.h"
#include "mgg_core/path_turns.h"
#include "mgg_core/plan_profile.h"
#include "mgg_core/trajectory.h"
#include "mgg_map_octomap/mola_map.h"

namespace mgg {
namespace nav_bench {

inline std::string readText(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

/// The MOLA product under `peer_root` (its mola/ directory), loaded through
/// MolaMap with the identity authority transform. Null with `error` set
/// when it does not load.
inline std::unique_ptr<MolaMap> loadProduct(const std::string& peer_root,
                                            std::string& error) {
  using nlohmann::json;
  json source, index;
  try {
    source = json::parse(readText(peer_root + "/mola/source.json"));
    index = json::parse(readText(peer_root + "/mola/index.json"));
  } catch (const std::exception& e) {
    error = e.what();
    return nullptr;
  }
  const json& manifest = source.at("manifests").at(0);
  const json& artifact = index.at("artifacts").at(0);
  MolaSnapshotRequest request;
  request.component_id = manifest.at("graph_revision").at("component_id");
  request.epoch = manifest.at("graph_revision").at("epoch");
  request.graph_revision = manifest.at("graph_revision").at("revision");
  request.geometry_revision = manifest.at("geometry_revision");
  for (const json& submap : manifest.at("submaps")) {
    request.source_stamp_ns =
        std::max<std::uint64_t>(request.source_stamp_ns,
                                submap.value("observed_at_ns", std::uint64_t{0}));
  }
  // The planner grid's resolution, from its metadata.
  const std::string grid = readText(peer_root + "/mola/" +
                                    artifact.at("planner").at("path").get<std::string>());
  if (grid.size() < 12) {
    error = "planner grid missing";
    return nullptr;
  }
  std::uint32_t metadata_size = 0;
  std::memcpy(&metadata_size, grid.data() + 8, 4);
  const json metadata = json::parse(grid.substr(12, metadata_size));
  MolaMapConfig config;
  config.peer_root = std::filesystem::absolute(peer_root).string();
  config.resolution = metadata.at("resolution_m");
  config.snapshot_ttl_sec = 1e6;
  config.max_load_time = std::chrono::milliseconds(10000);
  auto map = std::make_unique<MolaMap>(config);
  map->requestSnapshot(request);
  for (int i = 0; i < 400 && !map->getStatus(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!map->getStatus()) {
    error = map->lastError();
    return nullptr;
  }
  return map;
}

/// Botman's deployed planning parameters at the 0.10 m planner resolution:
/// deploy/mgg/config/hardware.yaml overridden as hardware.launch.py does
/// from adapters/adapter_ros2/config/bunker.yaml.
inline PlanningParams botmanPlanning() {
  PlanningParams p;
  p.rr_mode = RRModeType::kGraph;
  p.edge_length_min = 0.05;
  p.edge_length_max = 1.0;  // initial_ground_reach(0.61, 0, pi/4)
  p.edge_overshoot = 0.0;
  p.num_vertices_max = 500;
  p.num_edges_max = 10000;
  p.num_loops_cutoff = 2000;
  p.num_loops_max = 200000;
  p.nearest_range = 0.6;
  p.nearest_range_z = 0.15;
  p.nearest_range_min = 0.05;
  p.nearest_range_max = 1.0;
  p.max_ground_height = 0.61 + 0.15 + 0.175;
  p.max_step_height = 0.15;
  p.max_inclination = 27.0 * M_PI / 180.0;
  p.max_footprint_tilt = 20.0 * M_PI / 180.0;
  p.max_footprint_step = 0.10;
  p.path_clearance_margin = 0.6;
  p.path_interpolation_distance = 0.25;
  p.geofence_checking_enable = false;
  return p;
}

inline RobotParams botmanRobot() {
  RobotParams r;
  r.type = RobotType::kGroundRobot;
  // The box centred on the lidar that reaches the footprint's farthest
  // corner on each axis (hardware.launch.py), 2 x 0.61 m tall.
  r.size = Eigen::Vector3d(1.344, 0.778, 1.22);
  r.size_extension = Eigen::Vector3d(0.05, 0.05, 0.05);
  r.bound_mode = BoundModeType::kExtendedBound;
  return r;
}

inline GridGraphParams botmanGrid() {
  GridGraphParams g;
  g.min_val = Eigen::Vector3d(-6.0, -6.0, -0.2);
  g.max_val = Eigen::Vector3d(6.0, 6.0, 0.3);
  g.resolution = Eigen::Vector3d(0.4, 0.4, 0.1);
  return g;
}

struct Scenario {
  std::string name;
  bool navigate = true;
  /// The robot's odometry pose: x, y, z (the lidar, 0 in the capture
  /// frame), yaw.
  StateVec start = StateVec::Zero();
  StateVec goal = StateVec::Zero();
  /// Expectations, for the tests and the report: a route must be found
  /// (or, false, need not be).
  bool expect_route = true;
  double max_length_m = 0.0;     ///< 0: no bound
  double max_corner_deg = 0.0;   ///< 0: no bound
  double budget_ms = 0.0;        ///< x86 budget
};

/// Lab corridor scenarios of botman's 2026-10-01 run (mgg-3h.log).
inline std::vector<Scenario> scenarios() {
  std::vector<Scenario> s;
  auto nav = [&](const char* name, StateVec start, StateVec goal, bool expect,
                 double max_length, double max_corner, double budget) {
    Scenario c;
    c.name = name;
    c.start = start;
    c.goal = goal;
    c.expect_route = expect;
    c.max_length_m = max_length;
    c.max_corner_deg = max_corner;
    c.budget_ms = budget;
    s.push_back(c);
  };
  // A 2 m straight move down the corridor west of the start, where the
  // body clears the clutter on either side by 0.3 m.
  nav("corridor_west_2m", StateVec(0.0, 0.0, 0.0, M_PI),
      StateVec(-2.0, 0.0, 0.0, 0.0), true, 2.1, 10.0, 300.0);
  // A goal 3.5 m down that corridor, beside a bin at (-3.7, -0.55) and
  // short of the cabinet across it at x = -4.6 (4 m out the body's front
  // would be in it).
  nav("corridor_west_3.5m", StateVec(0.0, 0.0, 0.0, M_PI),
      StateVec(-3.5, 0.1, 0.0, 0.0), true, 3.6, 10.0, 300.0);
  // Logged goals botman was refused, "goal cannot be linked to the local
  // lattice". East of x = 2.3 the corridor narrows to 1.25 m beside a
  // cabinet; whether the body fits there is for the exact checks to say,
  // so these carry only the time budget.
  nav("log_1.13_to_4.60", StateVec(1.13, 0.07, 0.0, -0.08),
      StateVec(4.60, -0.38, 0.0, 0.0), false, 0.0, 0.0, 300.0);
  nav("log_0.60_to_4.21", StateVec(0.60, 0.06, 0.0, -0.05),
      StateVec(4.21, -0.08, 0.0, 0.0), false, 0.0, 0.0, 300.0);
  nav("log_-3.17_to_-8.04", StateVec(-3.17, -0.01, 0.0, 3.10),
      StateVec(-8.04, 0.17, 0.0, 0.0), false, 0.0, 0.0, 300.0);
  nav("log_-3.17_to_floor_0.66", StateVec(-3.17, -0.01, 0.0, 3.10),
      StateVec(0.66, 0.07, -0.68, 0.0), true, 4.2, 10.0, 300.0);
  nav("log_5.49_to_floor_0.20", StateVec(5.49, -0.20, 0.0, 3.12),
      StateVec(0.20, -0.08, -0.65, 0.0), false, 0.0, 0.0, 300.0);
  auto explore = [&](const char* name, StateVec start) {
    Scenario c;
    c.name = name;
    c.navigate = false;
    c.start = start;
    c.budget_ms = 350.0;
    s.push_back(c);
  };
  explore("explore_origin", StateVec(0.0, 0.0, 0.0, 0.0));
  explore("explore_1.13", StateVec(1.13, 0.07, 0.0, -0.08));
  explore("explore_-3.17", StateVec(-3.17, -0.01, 0.0, 3.10));
  return s;
}

struct Outcome {
  bool allow_unknown_body = true;
  double request_budget_ms = 500.0;
  bool routed = false;
  std::string reason;
  double total_ms = 0.0;
  double route_ms = 0.0;     ///< lattice, goal link, search
  double turn_ms = 0.0;      ///< turn rule
  double shortcut_ms = 0.0;  ///< shortcut and resample
  int lattice_vertices = 0;
  int lattice_edges = 0;
  GridGraphResult lattice;
  int route_vertices = 0;
  double graph_radius_m = 0.0;
  int no_room_refusals = 0;
  bool root_turn_clear = false;
  bool departure_found = false;
  double diagnostic_ms = 0.0;
  int corners = 0;           ///< points after shortcutting
  double length_m = 0.0;     ///< sent path length
  double max_corner_deg = 0.0;
  bool expectation_met = false;
  PlanProfile profile;
  std::vector<Eigen::Vector3d> path;
};

inline double polylineLength(const std::vector<Eigen::Vector3d>& p) {
  double length = 0.0;
  for (std::size_t i = 1; i < p.size(); ++i) length += (p[i] - p[i - 1]).norm();
  return length;
}

/// The largest heading change, degrees, between consecutive XY segments.
inline double maxCornerDeg(const std::vector<Eigen::Vector3d>& p) {
  double worst = 0.0;
  Eigen::Vector2d previous = Eigen::Vector2d::Zero();
  for (std::size_t i = 1; i < p.size(); ++i) {
    const Eigen::Vector2d d = (p[i] - p[i - 1]).head<2>();
    if (d.norm() < 1e-6) continue;
    if (previous.norm() > 0.0) {
      const double turn = std::abs(std::atan2(
          previous.x() * d.y() - previous.y() * d.x(), previous.dot(d)));
      worst = std::max(worst, turn * 180.0 / M_PI);
    }
    previous = d;
  }
  return worst;
}

inline Outcome run(const MapInterface& map, const Scenario& scenario,
                   bool allow_unknown_body = true, double request_budget_ms = 500.0) {
  using Clock = std::chrono::steady_clock;
  const auto ms = [](Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };
  Outcome out;
  out.allow_unknown_body = allow_unknown_body;
  out.request_budget_ms = request_budget_ms;
  const PlanningParams planning = botmanPlanning();
  const RobotParams robot = botmanRobot();
  const GridGraphParams grid = botmanGrid();
  GroundProjection ground(map, planning, /*cache_footprint_ground=*/true);
  ground.setProfile(&out.profile);
  EdgeInclinations inclinations;
  ExpandContext ctx;
  ctx.map = &map;
  ctx.planning = &planning;
  ctx.robot = &robot;
  ctx.ground = &ground;
  ctx.inclinations = &inclinations;
  ctx.robot_id = 1;
  ctx.robot_box_size = robot.getPlanningSize();
  ctx.allow_unknown_lattice_body = allow_unknown_body;
  ctx.hanging_root_edge_length_max = planning.edge_length_max;
  ctx.preserve_hanging_root_start_height = true;
  ctx.root_footprint_exempt = true;
  ctx.root_is_robot = true;
  GraphManager graph;
  const auto diagnostics = [&] {
    // Explicitly outside the measured production pipeline.
    PlanningCancellationScope unbudgeted([] { return false; });
    const auto started = Clock::now();
    for (const auto& entry : graph.vertices_map_) {
      out.graph_radius_m = std::max(out.graph_radius_m,
          (entry.second->state.head<2>() - scenario.start.head<2>()).norm());
    }
    GroundProjection diagnostic_ground(map, planning, true);
    ExpandContext diagnostic_ctx = ctx;
    diagnostic_ctx.ground = &diagnostic_ground;
    bool hanging = false;
    const StateVec root = localRouteRoot(diagnostic_ctx, scenario.start, hanging);
    out.root_turn_clear = roomToTurn(map, robot, planning, root, nullptr);
    Departure departure;
    out.departure_found = findDeparture(map, diagnostic_ground, robot, planning,
                                         root, departure);
    out.diagnostic_ms = ms(started, Clock::now());
  };
  const auto t0 = Clock::now();
  const auto deadline = t0 + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double, std::milli>(request_budget_ms));
  const auto* outer = planning_cancelled;
  PlanningCancellationScope budget([&] {
    return (outer && (*outer)()) || (request_budget_ms > 0 && Clock::now() >= deadline);
  });
  if (request_budget_ms > 0) ctx.deadline = deadline;
  try {
  if (!scenario.navigate) {
    ctx.deadline = t0 + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(kGroundExplorationLatticeBudgetS));
    bool hanging = false;
    const StateVec root_state = localRouteRoot(ctx, scenario.start, hanging);
    auto* root = new Vertex(0, root_state);
    root->robot_id = 1;
    root->is_hanging = hanging;
    graph.addVertex(root);
    const GridGraphResult r =
        buildGridGraph(graph, root_state, grid, ctx, scenario.start[3]);
    out.total_ms = out.route_ms = ms(t0, Clock::now());
    out.lattice = r;
    out.lattice_vertices = r.vertices_added;
    out.lattice_edges = r.edges_added;
    out.routed = r.vertices_added > 0;
    out.expectation_met = out.routed &&
                          (scenario.budget_ms <= 0.0 ||
                           out.total_ms <= scenario.budget_ms);
    diagnostics();
    return out;
  }
  LocalRouteResult local =
      routeOverLocalLattice(graph, scenario.start, scenario.goal, grid, ctx);
  const auto t_route = Clock::now();
  out.route_ms = ms(t0, t_route);
  out.lattice = local.lattice;
  out.lattice_vertices = local.lattice.vertices_added;
  out.lattice_edges = local.lattice.edges_added;
  out.routed = local.routed;
  out.reason = local.reason;
  if (local.routed) {
    out.route_vertices = static_cast<int>(local.route.size());
    // The planner node's turn rule (applyRouteTurnRule).
    PathTurnCheck check(graph, robot, [&](const StateVec& pose) {
      return roomToTurn(map, robot, planning, pose, nullptr);
    });
    check.setRobotTilt(local.route.front()->state.head<3>(), 0.0);
    chooseTurnCompliantRoute(graph, check, {}, scenario.start[3], local.route,
                             kMaxDetourSearchStates);
    const auto t_turn = Clock::now();
    out.turn_ms = ms(t_route, t_turn);
    // The planner node's shortcutAndResample, ground branch.
    PathType points;
    for (const Vertex* v : local.route) points.push_back(v->state.head<3>());
    GroundProjection shortcut_ground(map, planning, true);
    shortcut_ground.setProfile(&out.profile);
    ExpandContext shortcut_ctx = ctx;
    shortcut_ctx.ground = &shortcut_ground;
    // A local lattice route: the lattice's unknown policy.
    const auto segment_free = [&](const Eigen::Vector3d& a,
                                  const Eigen::Vector3d& b) {
      return groundShortcutSegmentAdmissible(shortcut_ctx, a, b,
                                             ctx.stop_at_unknown || !ctx.allow_unknown_lattice_body);
    };
    const PathOkFn turns_ok = [&](const PathType& trial) {
      return check.admissible(trial, scenario.start[3]);
    };
    const bool unshortcut_ok = turns_ok(points);
    const PathOkFn admissible = [&](const PathType& trial) {
      return !unshortcut_ok || turns_ok(trial);
    };
    const SegmentClearanceFn clearance = [&](const Eigen::Vector3d& a,
                                             const Eigen::Vector3d& b) {
      return shortcut_ground.segmentClearance(a, b, ctx.robot_box_size);
    };
    const PathType unshortcut = points;
    {
      ProfileScope timed(&out.profile.shortcut);
      points = shortcutPathKeepingClearance(points, segment_free, admissible,
                                            clearance);
    }
    out.corners = static_cast<int>(points.size());
    PathType resampled;
    if (interpolatePath(points, planning.path_interpolation_distance,
                        resampled) &&
        resampled.size() >= 2) {
      if ((resampled.back() - points.back()).norm() > 1e-6) {
        resampled.push_back(points.back());
      }
      points = resampled;
    }
    if (!admissible(points)) points = unshortcut;
    out.shortcut_ms = ms(t_turn, Clock::now());
    out.path.assign(points.begin(), points.end());
    out.length_m = polylineLength(out.path);
    out.max_corner_deg = maxCornerDeg(out.path);
    out.no_room_refusals = check.refused_without_room;
  }
  out.total_ms = ms(t0, Clock::now());
  diagnostics();
  out.expectation_met =
      (out.routed || !scenario.expect_route) &&
      (scenario.max_length_m <= 0.0 || out.length_m <= scenario.max_length_m) &&
      (scenario.max_corner_deg <= 0.0 ||
       out.max_corner_deg <= scenario.max_corner_deg) &&
      (scenario.budget_ms <= 0.0 || out.total_ms <= scenario.budget_ms);
  return out;
  } catch (const PlanningInterrupted&) {
    out.routed = false;
    out.path.clear();
    out.reason = "production request budget interrupted planning";
    out.total_ms = ms(t0, Clock::now());
    out.expectation_met = false;
    return out;
  }
}

inline std::string describe(const Scenario& s, const Outcome& o) {
  char buf[512];
  if (!s.navigate) {
    std::snprintf(buf, sizeof(buf),
                  "%-26s explore  %8.1f ms (budget %.0f) %s: %d vertices, %d "
                  "edges",
                  s.name.c_str(), o.total_ms, s.budget_ms,
                  o.expectation_met ? "ok  " : "MISS", o.lattice_vertices,
                  o.lattice_edges);
  } else {
    std::snprintf(
        buf, sizeof(buf),
        "%-26s navigate %8.1f ms (budget %.0f) %s: %s%s; route %.1f ms, turn "
        "%.1f ms, shortcut %.1f ms; lattice %d vertices %d edges; route %d "
        "vertices -> %d corners, %.2f m, max corner %.1f deg",
        s.name.c_str(), o.total_ms, s.budget_ms,
        o.expectation_met ? "ok  " : "MISS", o.routed ? "routed" : "refused",
        o.routed ? "" : (" (" + o.reason + ")").c_str(), o.route_ms, o.turn_ms,
        o.shortcut_ms, o.lattice_vertices, o.lattice_edges, o.route_vertices,
        o.corners, o.length_m, o.max_corner_deg);
  }
  const GridGraphResult& r = o.lattice;
  char lattice[256];
  std::snprintf(lattice, sizeof(lattice),
                "\n    lattice: %d free cells; edges %d ok, %d steep, %d "
                "occupied, %d unknown, %d hanging, %d cross-slope, %d "
                "footprint, %d ground unobserved; %d no ground",
                r.free_cells, r.edge_status[0], r.edge_status[1],
                r.edge_status[2], r.edge_status[3], r.edge_status[4],
                r.edge_status[5], r.edge_status[6], r.edge_status[7],
                r.no_ground);
  return std::string(buf) + lattice + "\n    " + o.profile.summary() +
      "\n    diagnostics (outside plan timing): no-room refusals " +
      std::to_string(o.no_room_refusals) + ", root turn " +
      (o.root_turn_clear ? "clear" : "refused") + ", departure " +
      (o.departure_found ? "found" : "refused") + ", " +
      std::to_string(o.diagnostic_ms) + " ms";
}

inline std::string toJson(const Scenario& s, const Outcome& o) {
  nlohmann::json j;
  j["allow_unknown_body"] = o.allow_unknown_body;
  j["request_budget_ms"] = o.request_budget_ms;
  j["scenario"] = s.name;
  j["kind"] = s.navigate ? "navigate" : "explore";
  j["routed"] = o.routed;
  j["reason"] = o.reason;
  j["total_ms"] = o.total_ms;
  j["budget_ms"] = s.budget_ms;
  j["route_ms"] = o.route_ms;
  j["turn_ms"] = o.turn_ms;
  j["shortcut_ms"] = o.shortcut_ms;
  j["length_m"] = o.length_m;
  j["max_corner_deg"] = o.max_corner_deg;
  j["corners"] = o.corners;
  j["expectation_met"] = o.expectation_met;
  j["graph_radius_m"] = o.graph_radius_m;
  j["profile"] = o.profile.summary();
  j["no_room_refusals"] = o.no_room_refusals;
  j["root_turn_clear"] = o.root_turn_clear;
  j["departure_found"] = o.departure_found;
  j["diagnostic_ms"] = o.diagnostic_ms;
  j["occupied_edges"] = o.lattice.edge_status[2];
  return j.dump();
}

}  // namespace nav_bench
}  // namespace mgg

#endif  // MGG_MAP_OCTOMAP_TEST_NAV_BENCH_SCENARIOS_H_
