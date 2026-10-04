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
    if (ceiling && (p.z() - size.z() / 2 <= 0.3) &&
        (p.z() + size.z() / 2 >= 0.3)) return VoxelStatus::kOccupied;
    return TerrainFixture::getBoxStatus(p, size, stop);
  }
  std::optional<Eigen::AlignedBox3d> bounds =
      Eigen::AlignedBox3d(Eigen::Vector3d(-2, -2, -2), Eigen::Vector3d(2, 2, 3));
  bool ceiling = false;
  bool slow_body = false;
  mutable std::vector<Eigen::Vector3d> checked_bodies;
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
    EXPECT_GE(next + 1e-9, distance);
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
