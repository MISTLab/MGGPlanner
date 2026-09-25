// The footprint-plane edge check on maps of the SubT run of 2026-09-24.
//
// A Scout Mini drove from level floor onto a rock field and ended on the
// face of a rock pile at 33 degrees of tilt; every edge check MGG had
// passed it. A Bunker drove the 16 degree SubT ramp, which must stay open.
// Edges 0.8 m long are laid along each driven path every 0.2 m, as the
// diagnosis laid them (diag-run4 findings, Q2), and checked with the
// platform's parameters, first without the footprint plane, then with it.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/ground_projection.h"
#include "terrain_fixture.h"

namespace {

using mgg::GroundProjection;
using mgg::PlanningParams;
using mgg::ProjectedEdgeStatus;
using mgg::VoxelStatus;
using mgg_test::TerrainFixture;

double deg(double d) { return d * M_PI / 180.0; }

/// A simulated platform: its RobotSpec, the parameters SwarmDeck's fleet
/// launch derives from it (deploy/mgg/fleet.launch.py), and the limits
/// proposed for it. On MGG's 0.2 m map, with every cell under the body, the
/// ramp's footprint tilt passes 22 degrees from map noise, so the Scout's
/// limit is 25. The Spot's limits let it cross the rock face: it is stable
/// on rough ground and cannot tip over in simulation (operator, 2026-09-25).
struct Platform {
  const char* name;
  double length, width, base_height, max_step_height;
  double max_inclination_deg, max_cross_slope_deg;
  double max_footprint_tilt_deg, max_footprint_step;
};

const Platform kScout{"scout_mini", 0.612, 0.580, 0.1225, 0.15,
                      27.0,         18.0,  25.0,  0.15};
const Platform kBunker{"bunker", 1.023, 0.778, 0.200, 0.15,
                       27.0,     18.0,  22.0,  0.12};
const Platform kSpot{"spot", 1.100, 0.500, 0.500, 0.30,
                     30.0,   12.0,  25.0,  0.25};

PlanningParams planningFor(const Platform& p) {
  PlanningParams params;
  params.max_step_height = p.max_step_height;
  params.max_ground_height = p.base_height + p.max_step_height + 0.175;
  params.max_inclination = deg(p.max_inclination_deg);
  params.max_cross_slope = deg(p.max_cross_slope_deg);
  // The fixtures record the ground within 1.5 m of the path and no free
  // space; the unobserved-ground check is tested on its own.
  params.min_observed_ground_fraction = 0.0;
  return params;
}

Eigen::Vector3d boxFor(const Platform& p) {
  // RobotParams.size plus bistro.yaml's size_extension, kExtendedBound.
  return {p.length + 0.05, p.width + 0.05, 2.0 * p.base_height + 0.05};
}

struct Edge {
  double from = 0.0, to = 0.0;  ///< metres along the driven path
  ProjectedEdgeStatus without = ProjectedEdgeStatus::kAdmissible;
  ProjectedEdgeStatus with = ProjectedEdgeStatus::kAdmissible;
};

/// Every 0.8 m edge along the path, starting every 0.2 m.
std::vector<Edge> edgesAlongPath(const TerrainFixture& map,
                                 const Platform& platform) {
  const std::vector<Eigen::Vector3d>& poses = map.poses();
  std::vector<double> along{0.0};
  for (std::size_t i = 1; i < poses.size(); ++i) {
    along.push_back(along.back() +
                    (poses[i] - poses[i - 1]).head<2>().norm());
  }
  auto at = [&](double s) {
    std::size_t i = 1;
    while (i + 1 < poses.size() && along[i] < s) ++i;
    const double t = (s - along[i - 1]) / (along[i] - along[i - 1]);
    return Eigen::Vector3d(poses[i - 1] + t * (poses[i] - poses[i - 1]));
  };

  PlanningParams params = planningFor(platform);
  GroundProjection ground(map, params);
  const Eigen::Vector3d box = boxFor(platform);
  // An endpoint at driving height, as expandGraph places a vertex.
  auto driving = [&](double s, Eigen::Vector3d& point) {
    point = at(s) + Eigen::Vector3d(0.0, 0.0, 0.3);
    VoxelStatus status;
    const double drop = ground.projectSample(point, status);
    if (status != VoxelStatus::kOccupied) return false;
    point.z() -= drop - params.max_ground_height;
    return true;
  };

  std::vector<Edge> edges;
  for (double s = 0.0; s + 0.8 <= along.back() + 1e-9; s += 0.2) {
    Eigen::Vector3d start, end;
    if (!driving(s, start) || !driving(s + 0.8, end)) continue;
    Edge edge{s, s + 0.8};
    std::vector<Eigen::Vector3d> path;
    params.max_footprint_tilt = 0.0;
    params.max_footprint_step = 0.0;
    edge.without =
        ground.getProjectedEdgeStatus(start, end, box, false, path, false);
    params.max_footprint_tilt = deg(platform.max_footprint_tilt_deg);
    params.max_footprint_step = platform.max_footprint_step;
    edge.with =
        ground.getProjectedEdgeStatus(start, end, box, false, path, false);
    edges.push_back(edge);
  }
  return edges;
}

/// The rock pile's face, metres along robot_2's path: where the diagnosis
/// found the footprint tilting 21 to 29 degrees, just before the robot
/// stuck at 33 degrees.
constexpr double kFaceFrom = 3.5;
constexpr double kFaceTo = 4.5;

struct Tally {
  int admitted_without = 0;  ///< admissible without the footprint check
  int admitted_with = 0;     ///< admissible with it
  int face_without = 0;      ///< of those, overlapping the pile's face
  int face_with = 0;
};

Tally run(const char* fixture, const Platform& platform) {
  const TerrainFixture map(std::string(MGG_TEST_DATA_DIR) + "/" + fixture);
  const std::vector<Edge> edges = edgesAlongPath(map, platform);
  Tally t;
  std::string refused;
  for (const Edge& e : edges) {
    const bool face = e.from < kFaceTo && e.to > kFaceFrom;
    if (e.without == ProjectedEdgeStatus::kAdmissible) {
      ++t.admitted_without;
      if (face) ++t.face_without;
    }
    if (e.with == ProjectedEdgeStatus::kAdmissible) {
      ++t.admitted_with;
      if (face) ++t.face_with;
    } else if (e.with == ProjectedEdgeStatus::kFootprintPlane) {
      char span[32];
      std::snprintf(span, sizeof(span), " %.1f-%.1f", e.from, e.to);
      refused += span;
    }
  }
  std::printf("%s, %s: %zu edges, %d admissible without the footprint "
              "check, %d with it; it refused (m):%s\n",
              fixture, platform.name, edges.size(), t.admitted_without,
              t.admitted_with, refused.c_str());
  return t;
}

void expectRampOpen(const Platform& platform) {
  const Tally ramp = run("subt_ramp.txt", platform);
  ASSERT_GE(ramp.admitted_without, 70);  // of 101
  EXPECT_GE(ramp.admitted_with, 0.99 * ramp.admitted_without);
}

void expectRampOpenAndFaceRefused(const Platform& platform) {
  expectRampOpen(platform);

  // What MGG saw when it planned the drive onto the rocks: some edge over
  // the face passes every other check, and none passes this one.
  const Tally before = run("rock_field_preplan.txt", platform);
  EXPECT_GT(before.face_without, 0);
  EXPECT_EQ(before.face_with, 0);
  EXPECT_LT(before.admitted_with, before.admitted_without);

  // The map once the robot had stuck on the face.
  const Tally after = run("rock_field_now.txt", platform);
  EXPECT_EQ(after.face_with, 0);
}

TEST(FootprintPlane, ScoutMiniKeepsTheRampAndRefusesTheRockFace) {
  expectRampOpenAndFaceRefused(kScout);
}

TEST(FootprintPlane, BunkerKeepsTheRampAndRefusesTheRockFace) {
  expectRampOpenAndFaceRefused(kBunker);
}

TEST(FootprintPlane, SpotKeepsTheRamp) {
  expectRampOpen(kSpot);
  // Reported, not asserted: the Spot may cross the face.
  run("rock_field_preplan.txt", kSpot);
  run("rock_field_now.txt", kSpot);
}

TEST(FootprintPlane, APlanCacheGivesTheSameVerdictsAsNoCache) {
  // Every lattice-style edge, 0.4 m in 8 directions, from every 0.4 m point
  // along the ramp, checked by one projection that caches its footprint
  // lookups and by one that does not.
  const TerrainFixture map(std::string(MGG_TEST_DATA_DIR) + "/subt_ramp.txt");
  for (const Platform* platform : {&kScout, &kBunker, &kSpot}) {
    PlanningParams params = planningFor(*platform);
    params.max_footprint_tilt = deg(platform->max_footprint_tilt_deg);
    params.max_footprint_step = platform->max_footprint_step;
    const GroundProjection plain(map, params);
    const GroundProjection cached(map, params, true);
    const Eigen::Vector3d box = boxFor(*platform);
    int compared = 0, refused = 0;
    for (const Eigen::Vector3d& pose : map.poses()) {
      std::vector<Eigen::Vector3d> vertices;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          Eigen::Vector3d point = pose + Eigen::Vector3d(0.4 * dx, 0.4 * dy,
                                                         0.3);
          VoxelStatus status;
          const double drop = plain.projectSample(point, status);
          if (status != VoxelStatus::kOccupied) continue;
          point.z() -= drop - params.max_ground_height;
          vertices.push_back(point);
        }
      }
      for (const Eigen::Vector3d& a : vertices) {
        for (const Eigen::Vector3d& b : vertices) {
          if ((a - b).head<2>().norm() > 0.6 || (a - b).norm() < 1e-9) {
            continue;
          }
          std::vector<Eigen::Vector3d> p1, p2;
          const ProjectedEdgeStatus s1 =
              plain.getProjectedEdgeStatus(a, b, box, false, p1, false);
          const ProjectedEdgeStatus s2 =
              cached.getProjectedEdgeStatus(a, b, box, false, p2, false);
          ASSERT_EQ(s1, s2);
          ++compared;
          if (s1 == ProjectedEdgeStatus::kFootprintPlane) ++refused;
        }
      }
    }
    EXPECT_GT(compared, 1000);
    std::printf("%s: %d edges compared, %d refused by the footprint plane\n",
                platform->name, compared, refused);
  }
}

}  // namespace
