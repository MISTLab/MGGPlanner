#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <thread>

#include "mgg_core/ground_layer.h"
#include "terrain_fixture.h"

namespace mgg {
namespace {
using Clock = std::chrono::steady_clock;

std::map<std::pair<std::int64_t, std::int64_t>, double> terrain(
    const std::function<double(double)>& height) {
  std::map<std::pair<std::int64_t, std::int64_t>, double> tops;
  for (int x = -20; x < 20; ++x)
    for (int y = -20; y < 20; ++y)
      tops[{x, y}] = height((x + 0.5) * 0.2);
  return tops;
}

class LayerMap : public mgg_test::TerrainFixture {
 public:
  explicit LayerMap(const std::function<double(double)>& height)
      : TerrainFixture(0.2, terrain(height)) {}
  std::optional<Eigen::AlignedBox3d> windowBounds() const override {
    return bounds;
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool stop, Eigen::Vector3d& end) const override {
    ++ray_queries;
    if (!support_visible) {
      end = b;
      return VoxelStatus::kUnknown;
    }
    if (ceiling && a.z() >= 0.3 && b.z() <= 0.3) {
      end = {a.x(), a.y(), 0.3};
      return VoxelStatus::kOccupied;
    }
    return TerrainFixture::getRayStatus(a, b, stop, end);
  }
  VoxelStatus getBoxStatus(const Eigen::Vector3d& p,
                           const Eigen::Vector3d& size, bool stop) const override {
    if (slow_body) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    checked_bodies.push_back(p);
    if (unknown_body) return VoxelStatus::kUnknown;
    if (ceiling && (p.z() - size.z() / 2 <= 0.3) &&
        (p.z() + size.z() / 2 >= 0.3)) return VoxelStatus::kOccupied;
    return TerrainFixture::getBoxStatus(p, size, stop);
  }
  std::optional<Eigen::AlignedBox3d> bounds =
      Eigen::AlignedBox3d(Eigen::Vector3d(-2, -2, -2), Eigen::Vector3d(2, 2, 3));
  bool ceiling = false;
  bool support_visible = true;
  bool unknown_body = false;
  mutable int ray_queries = 0;
  bool slow_body = false;
  mutable std::vector<Eigen::Vector3d> checked_bodies;
};
// Voxel-centre ray hits, including an occupied start voxel, as in the
// production rolling map. Unlike TerrainFixture, the floor has thickness.
class StackedGroundMap : public LayerMap {
 public:
  explicit StackedGroundMap(bool raised_floor = false)
      : LayerMap([](double) { return 0.0; }), raised_floor_(raised_floor) {}

  VoxelStatus getVoxelStatus(const Eigen::Vector3d& p) const override {
    return occupied(keyOf(p, getResolution())) ? VoxelStatus::kOccupied
                                               : VoxelStatus::kFree;
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                           bool, Eigen::Vector3d& end) const override {
    // GroundLayer casts only vertical downward rays. Include the start
    // cell, even when a starts inside a buried occupied floor voxel.
    VoxelKey cell = keyOf(a, getResolution());
    const int last_z = keyOf(b, getResolution()).z;
    for (; cell.z >= last_z; --cell.z) {
      if (occupied(cell)) {
        end = centerOf(cell, getResolution());
        return VoxelStatus::kOccupied;
      }
    }
    end = b;
    return VoxelStatus::kFree;
  }
  VoxelStatus getBoxStatus(const Eigen::Vector3d& p,
                           const Eigen::Vector3d& size, bool) const override {
    const VoxelKey low = keyOf(p - size / 2, getResolution());
    const VoxelKey high = keyOf(p + size / 2, getResolution());
    for (int x = low.x; x <= high.x; ++x)
      for (int y = low.y; y <= high.y; ++y)
        for (int z = low.z; z <= high.z; ++z)
          if (occupied({x, y, z})) return VoxelStatus::kOccupied;
    return VoxelStatus::kFree;
  }

 private:
  bool occupied(const VoxelKey& cell) const {
    const int top = raised_floor_ && cell.x >= 4 ? 1 : 0;
    // A single-voxel neighbour alongside the two-voxel-thick seed column
    // also catches a false parent-height discontinuity after projection.
    return cell.z == top || (cell.x != 1 && cell.z == top - 1);
  }
  const bool raised_floor_;
};

PlanningParams planning() {
  PlanningParams p;
  p.max_ground_height = 0.6;
  p.max_step_height = 0.15;
  p.max_inclination = 20 * M_PI / 180;
  return p;
}
RobotParams robot() {
  RobotParams r;
  r.size = {1.2, 0.8, 0.5};
  return r;
}
void finish(GroundLayer& layer) { layer.recheck(Clock::now() + std::chrono::seconds(5)); }
int cost(const GroundLayer& layer, const Eigen::Vector2d& p) {
  const int width = static_cast<int>(std::sqrt(layer.occupancy().size()));
  const Eigen::Vector2d index = (p - layer.origin()) / 0.2;
  return layer.occupancy().at(static_cast<int>(std::floor(index.y())) * width +
                             static_cast<int>(std::floor(index.x())));
}
MapChange changeAt(const Eigen::Vector3d& p) {
  MapChange c;
  c.boxes.emplace_back(p - Eigen::Vector3d::Constant(0.01),
                       p + Eigen::Vector3d::Constant(0.01));
  return c;
}

TEST(GroundLayer, CertifiedTerrainMapping) {
  LayerMap flat([](double) { return 0.0; });
  GroundLayer flat_layer(flat, planning(), robot());
  flat_layer.reset({0.1, 0.1, 0.6}, 0);
  finish(flat_layer);
  EXPECT_EQ(cost(flat_layer, {0.1, 0.1}), 0);
  EXPECT_EQ(cost(flat_layer, {1.1, 0.1}), 0);

  LayerMap ramp([](double x) { return x * std::tan(16 * M_PI / 180); });
  GroundLayer ramp_layer(ramp, planning(), robot());
  ramp_layer.reset({0.1, 0.1, 0.6 + 0.1 * std::tan(16 * M_PI / 180)}, 0);
  finish(ramp_layer);
  EXPECT_EQ(cost(ramp_layer, {1.1, 0.1}), 0);

  LayerMap step([](double x) { return x >= 0.8 ? 0.3 : 0.0; });
  GroundLayer step_layer(step, planning(), robot());
  step_layer.reset({0.1, 0.1, 0.6}, 0);
  finish(step_layer);
  EXPECT_EQ(cost(step_layer, {0.7, 0.1}), 100);
  EXPECT_EQ(step_layer.verdict({0.7, 0.1}), GroundVerdict::kRefusedStepGrade);

  flat.ceiling = true;
  GroundLayer ceiling_layer(flat, planning(), robot());
  ceiling_layer.reset({0.1, 0.1, 0.6}, 0);
  finish(ceiling_layer);
  EXPECT_EQ(cost(ceiling_layer, {0.1, 0.1}), 100);
  EXPECT_EQ(cost(ceiling_layer, {1.1, 0.1}), 100);
  EXPECT_EQ(ceiling_layer.verdict({0.1, 0.1}), GroundVerdict::kRefusedOverhang);

  mgg_test::TerrainFixture unseen(0.2, {});
  GroundLayer unseen_layer(unseen, planning(), robot());
  unseen_layer.reset({0.1, 0.1, 0.6}, 0);
  finish(unseen_layer);
  EXPECT_EQ(cost(unseen_layer, {0.1, 0.1}), -1);
  EXPECT_EQ(unseen_layer.verdict({0.1, 0.1}), GroundVerdict::kUnknown);
}

TEST(GroundLayer, SeedToleratesLowAnchorAndRefreshesBeforeCertification) {
  LayerMap map([](double) { return 0.0; });
  GroundLayer layer(map, planning(), robot());
  layer.reset({0.1, 0.1, 0.57}, 0);
  finish(layer);
  EXPECT_EQ(cost(layer, {0.1, 0.1}), 0);  // hint 3 cm below the voxel bottom

  map.ceiling = true;
  layer.reset({0.1, 0.1, 0.57}, 0);
  finish(layer);
  EXPECT_EQ(cost(layer, {0.1, 0.1}), 100);
  map.ceiling = false;

  layer.reset({0.1, 0.1, -0.1}, 0);
  finish(layer);
  EXPECT_EQ(layer.verdict({0.1, 0.1}), GroundVerdict::kUnknown);
  layer.recenter({0.1, 0.1, 0.57}, 0);  // unchanged XY bounds, corrected odometry
  finish(layer);
  EXPECT_EQ(cost(layer, {0.1, 0.1}), 0);

  auto tops = terrain([](double) { return 0.0; });
  for (auto it = tops.begin(); it != tops.end();) {
    if (it->first.first < 5) it = tops.erase(it);
    else ++it;
  }
  mgg_test::TerrainFixture offset_support(0.2, tops);
  GroundLayer moved(offset_support, planning(), robot());
  moved.reset({0.1, 0.1, 0.6}, 0);
  finish(moved);
  EXPECT_EQ(moved.verdict({0.1, 0.1}), GroundVerdict::kUnknown);
  moved.recenter({1.1, 0.1, 0.6}, 0);
  finish(moved);
  EXPECT_EQ(cost(moved, {1.1, 0.1}), 0);
}

TEST(GroundLayer, StackedVoxelsKeepSurfaceAboveBuriedSupport) {
  StackedGroundMap map;
  Eigen::Vector3d buried;
  ASSERT_EQ(map.getRayStatus({0.1, 0.1, -0.02}, {0.1, 0.1, -1}, false, buried),
            VoxelStatus::kOccupied);
  EXPECT_NEAR(buried.z(), -0.1, 1e-9);  // the ray starts inside the lower voxel
  GroundLayer layer(map, planning(), robot());
  layer.reset({0.1, 0.1, 0.58}, 0);  // floor hint -0.02, surface centre +0.1
  finish(layer);
  EXPECT_EQ(cost(layer, {0.1, 0.1}), 0);
  EXPECT_EQ(cost(layer, {0.1, 0.3}), 0);  // stacked neighbour
  EXPECT_EQ(cost(layer, {0.3, 0.1}), 0);  // single-voxel neighbour
  layer.withdraw(changeAt({0.1, 0.1, 0.1}));
  finish(layer);
  EXPECT_EQ(cost(layer, {0.1, 0.1}), 0);
  EXPECT_EQ(cost(layer, {0.3, 0.1}), 0);
}

TEST(GroundLayer, StackedUphillChildUsesObservedSurface) {
  StackedGroundMap map(true);
  auto params = planning();
  params.max_step_height = 0.25;  // the quantised 0.2 m rise is admissible
  GroundLayer layer(map, params, robot());
  layer.reset({0.1, 0.1, 0.7}, 0.1);
  finish(layer);
  EXPECT_EQ(cost(layer, {0.1, 0.1}), 0);
  EXPECT_EQ(cost(layer, {0.9, 0.1}), 0);  // floor centres rise from 0.1 to 0.3
  EXPECT_EQ(layer.verdict({0.9, 0.1}), GroundVerdict::kAdmitted);
}

TEST(GroundLayer, CompletedUnknownIsNotPendingAndWaitsForChanges) {
  LayerMap map([](double) { return 0.0; });
  map.support_visible = false;
  GroundLayer layer(map, planning(), robot());
  layer.reset({0.1, 0.1, 0.6}, 0);
  finish(layer);
  EXPECT_EQ(layer.pendingCount(), 0);
  EXPECT_EQ(layer.verdict({0.1, 0.1}), GroundVerdict::kUnknown);
  const Eigen::AlignedBox3d region(Eigen::Vector3d(-2, -2, -1),
                                   Eigen::Vector3d(2, 2, 1));
  EXPECT_FALSE(layer.pending(region));
  const int queries = map.ray_queries;
  layer.recenter({0.1, 0.1, 0.6}, 0);  // same pose must not re-dirty unknown
  finish(layer);
  EXPECT_EQ(map.ray_queries, queries);

  map.support_visible = true;
  layer.withdraw(changeAt({0.1, 0.1, 0}));
  EXPECT_GT(layer.pendingCount(), 0);
  finish(layer);
  EXPECT_EQ(cost(layer, {1.7, 1.7}), 0);  // resumes into previously unreached cells
  EXPECT_EQ(layer.pendingCount(), 0);

  map.unknown_body = true;
  layer.withdraw(changeAt({0.1, 0.1, 0.3}));
  finish(layer);
  EXPECT_EQ(layer.verdict({0.1, 0.1}), GroundVerdict::kUnknown);
  EXPECT_EQ(layer.verdict({1.7, 1.7}), GroundVerdict::kUnknown);
  EXPECT_EQ(layer.pendingCount(), 0);
  const int body_queries = map.ray_queries;
  finish(layer);
  EXPECT_EQ(map.ray_queries, body_queries);
}

TEST(GroundLayer, DropEdgeCannotBorrowSupportFromRim) {
  auto tops = terrain([](double) { return 0.0; });
  for (auto it = tops.begin(); it != tops.end();) {
    if (it->first.first >= 4) it = tops.erase(it);
    else ++it;
  }
  mgg_test::TerrainFixture map(0.2, tops);
  GroundLayer layer(map, planning(), robot());
  layer.reset({0.1, 0.1, 0.6}, 0);
  finish(layer);
  EXPECT_EQ(cost(layer, {0.5, 0.1}), 0);
  EXPECT_EQ(cost(layer, {0.9, 0.1}), -1);
  EXPECT_EQ(layer.verdict({0.9, 0.1}), GroundVerdict::kUnknown);
  EXPECT_EQ(layer.pendingCount(), 0);
}

TEST(GroundLayer, RampAdmissionExercisesGradeLimit) {
  LayerMap map([](double x) { return x * std::tan(16 * M_PI / 180); });
  auto params = planning();
  params.max_step_height = 0.05;  // native two-cell rise exceeds the step limit
  const Eigen::Vector3d anchor(0.1, 0.1, 0.6 + 0.1 * std::tan(16 * M_PI / 180));
  GroundLayer admitted(map, params, robot());
  admitted.reset(anchor, 0);
  finish(admitted);
  EXPECT_EQ(cost(admitted, {1.1, 0.1}), 0);
  params.max_inclination = 10 * M_PI / 180;
  GroundLayer refused(map, params, robot());
  refused.reset(anchor, 0);
  finish(refused);
  EXPECT_EQ(cost(refused, {1.1, 0.1}), 100);
  EXPECT_EQ(refused.verdict({1.1, 0.1}), GroundVerdict::kRefusedStepGrade);
}

TEST(GroundLayer, ResetAndScrollWithdraw) {
  LayerMap map([](double) { return 0.0; });
  GroundLayer layer(map, planning(), robot());
  layer.reset({0.1, 0.1, 0.6}, 0);
  finish(layer);
  ASSERT_EQ(cost(layer, {0.1, 0.1}), 0);
  const Eigen::Vector2d before = layer.origin();
  map.bounds = Eigen::AlignedBox3d(Eigen::Vector3d(-1, -2, -2), Eigen::Vector3d(3, 2, 3));
  layer.recenter({1.1, 0.1, 0.6}, 0);
  EXPECT_NE(layer.origin(), before);
  EXPECT_EQ(cost(layer, {2.1, 0.1}), -1);
  // Its own column remains, but a ground-ray dependency was evicted.
  EXPECT_EQ(cost(layer, {-0.7, 0.1}), -1);
  EXPECT_GT(layer.pendingCount(), 0);
  EXPECT_EQ(cost(layer, {0.1, 0.1}), 0);  // safe overlap survives scrolling
  layer.reset({1.1, 0.1, 0.6}, 0);
  EXPECT_EQ(cost(layer, {0.1, 0.1}), -1);
  EXPECT_GT(layer.pendingCount(), 0);
}

TEST(GroundLayer, DeadlineNeverExportsPendingFree) {
  LayerMap map([](double x) { return x >= 0.8 ? 0.3 : 0.0; });
  GroundLayer layer(map, planning(), robot());
  layer.reset({0.1, 0.1, 0.6}, 0);
  finish(layer);
  ASSERT_EQ(cost(layer, {0.1, 0.1}), 0);
  ASSERT_EQ(cost(layer, {0.7, 0.1}), 100);
  MapChange all;
  all.everything = true;
  layer.withdraw(all);
  EXPECT_EQ(layer.verdict({0.1, 0.1}), GroundVerdict::kPending);
  layer.recheck(Clock::now() - std::chrono::seconds(1));
  EXPECT_EQ(cost(layer, {0.1, 0.1}), -1);
  EXPECT_EQ(cost(layer, {0.7, 0.1}), 100);
  EXPECT_TRUE(layer.pending(Eigen::AlignedBox3d(Eigen::Vector3d(0, 0, -1), Eigen::Vector3d(0.2, 0.2, 1))));
  EXPECT_FALSE(layer.pending(Eigen::AlignedBox3d(Eigen::Vector3d(20, 20, -1), Eigen::Vector3d(21, 21, 1))));
  finish(layer);
  EXPECT_EQ(cost(layer, {0.1, 0.1}), 0);
}

TEST(GroundLayer, ParentGroundRayWithdrawsDescendants) {
  LayerMap map([](double) { return 0.0; });
  GroundLayer layer(map, planning(), robot());
  layer.reset({0.1, 0.1, 0.6}, 0);
  finish(layer);
  ASSERT_EQ(cost(layer, {1.7, 1.7}), 0);
  layer.withdraw(changeAt({0.1, 0.1, -3.0}));
  EXPECT_EQ(cost(layer, {1.7, 1.7}), -1);
  finish(layer);
  EXPECT_EQ(cost(layer, {1.7, 1.7}), 0);
}

TEST(GroundLayer, InterruptedColumnNeverPublishesAdmission) {
  LayerMap map([](double) { return 0.0; });
  GroundLayer layer(map, planning(), robot());
  layer.reset({0.1, 0.1, 0.6}, 0);
  map.slow_body = true;
  layer.recheck(Clock::now() + std::chrono::milliseconds(2));
  EXPECT_EQ(cost(layer, {0.1, 0.1}), -1);
  EXPECT_EQ(layer.pendingCount(), 400);
  map.slow_body = false;
  finish(layer);
  EXPECT_EQ(cost(layer, {0.1, 0.1}), 0);
  EXPECT_EQ(layer.pendingCount(), 0);
}

TEST(GroundLayer, RechecksNearestFirstAndRetainsUnchangedAdmissions) {
  LayerMap map([](double) { return 0.0; });
  GroundLayer layer(map, planning(), robot());
  layer.reset({0.1, 0.1, 0.6}, 0);
  finish(layer);
  ASSERT_FALSE(map.checked_bodies.empty());
  EXPECT_TRUE(map.checked_bodies.front().head<2>().isApprox(Eigen::Vector2d(0.1, 0.1)));
  double distance = 0;
  for (const auto& body : map.checked_bodies) {
    const double next = (body.head<2>() - Eigen::Vector2d(0.1, 0.1)).norm();
    ASSERT_GE(next + 1e-9, distance);
    distance = next;
  }
  layer.withdraw(changeAt({1.9, 1.9, 0}));
  EXPECT_EQ(cost(layer, {-1.7, -1.7}), 0);
  EXPECT_FALSE(layer.pending(Eigen::AlignedBox3d(
      Eigen::Vector3d(-1.8, -1.8, -0.1), Eigen::Vector3d(-1.6, -1.6, 0.1))));
}

TEST(GroundLayer, BoundsAndExplicitFallback) {
  LayerMap map([](double) { return 0.0; });
  GroundLayer layer(map, planning(), robot());
  layer.reset({0.1, 0.1, 0.6}, 0);
  EXPECT_EQ(layer.occupancy().size(), 400u);
  EXPECT_TRUE(layer.origin().isApprox(Eigen::Vector2d(-2, -2)));
  map.bounds.reset();
  GroundLayer default_extent(map, planning(), robot());
  default_extent.reset({0.1, 0.1, 0.6}, 0);
  EXPECT_EQ(default_extent.occupancy().size(), 6400u);
  GroundLayerParams params;
  params.window_size_m = {6, 4};
  GroundLayer fallback(map, planning(), robot(), params);
  fallback.reset({1.1, 0.1, 0.6}, 0);
  EXPECT_EQ(fallback.occupancy().size(), 600u);
  EXPECT_TRUE(fallback.origin().isApprox(Eigen::Vector2d(-2, -2)));
}
}  // namespace
}  // namespace mgg
