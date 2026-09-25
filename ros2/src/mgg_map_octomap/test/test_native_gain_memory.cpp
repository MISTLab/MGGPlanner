// Memory the native gain scan takes, measured by replacing the global
// allocation functions in this test binary alone.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <string>
#include <unordered_map>

#include "mgg_core/gain.h"
#include "mgg_map_octomap/native_mola_grid.h"

namespace {

std::atomic<std::size_t> live_bytes{0};
std::atomic<std::size_t> peak_bytes{0};

// A header before each block records its size; 16 bytes keeps the block
// aligned for any fundamental type.
constexpr std::size_t kHeader = 16;

void* allocate(std::size_t size) {
  void* block = std::malloc(size + kHeader);
  if (block == nullptr) throw std::bad_alloc();
  *static_cast<std::size_t*>(block) = size;
  const std::size_t now = live_bytes += size;
  std::size_t peak = peak_bytes.load();
  while (now > peak && !peak_bytes.compare_exchange_weak(peak, now)) {
  }
  return static_cast<char*>(block) + kHeader;
}

void release(void* pointer) noexcept {
  if (pointer == nullptr) return;
  void* block = static_cast<char*>(pointer) - kHeader;
  live_bytes -= *static_cast<std::size_t*>(block);
  std::free(block);
}

}  // namespace

void* operator new(std::size_t size) { return allocate(size); }
void* operator new[](std::size_t size) { return allocate(size); }
void operator delete(void* pointer) noexcept { release(pointer); }
void operator delete[](void* pointer) noexcept { release(pointer); }
void operator delete(void* pointer, std::size_t) noexcept { release(pointer); }
void operator delete[](void* pointer, std::size_t) noexcept {
  release(pointer);
}

namespace {

/// Bytes allocated at the peak of one aerial gain evaluation from a voxel
/// centre in all-unknown space, above what was live before it.
std::size_t peakOfGain(double vertical_fov, double step, double range) {
  mgg::NativeMolaGrid unknown_space(0.2, {}, {}, {});
  mgg::SensorParams sensor;
  sensor.type = mgg::SensorType::kLidar;
  sensor.max_range = range;
  sensor.fov = Eigen::Vector2d(2.0 * M_PI, vertical_fov);
  sensor.resolution = Eigen::Vector2d::Constant(step);
  sensor.frontier_percentage_threshold = 0.05;
  sensor.update();
  std::unordered_map<std::string, mgg::SensorParams> sensors{
      {"VLP16", sensor}};
  mgg::PlanningParams planning;
  planning.exp_sensor_list = {"VLP16"};
  mgg::BoundedSpaceParams space;
  space.min_val = Eigen::Vector3d::Constant(-100.0);
  space.max_val = Eigen::Vector3d::Constant(100.0);
  space.setCenter(Eigen::Vector3d(0.0, 0.0, 0.0), false);
  mgg::GainContext ctx;
  ctx.map = &unknown_space;
  ctx.planning = &planning;
  ctx.global_space = &space;
  ctx.sensors = &sensors;

  const std::size_t before = live_bytes.load();
  peak_bytes = before;
  mgg::VolumetricGain gain;
  mgg::computeVolumetricGain(mgg::StateVec(0.1, 0.1, 0.1, 0.0), gain, ctx);
  EXPECT_TRUE(gain.is_frontier);
  return peak_bytes.load() - before;
}

// Review r1 (P2): the scan's visited-cell table was sized from the ray
// count, 64 cells a ray, whatever the range. 64800 rays of 1 m, which
// reach under 500 distinct voxels, reserved 8.4 million slots, 232 MiB,
// and kept them. The table now grows with the distinct cells visited.
TEST(NativeGainMemory, ADenseShortRangeScanStaysSmall) {
  const std::size_t dense = peakOfGain(M_PI / 4.0, M_PI / 360.0, 1.0);
  std::printf("dense 0.5 degree, 1 m: peak %.2f MB\n", dense / 1.0e6);
  RecordProperty("dense_peak_bytes", std::to_string(dense));
  // The scan's 64890 ray endpoints alone are 1.6 MB of it.
  EXPECT_LT(dense, std::size_t(8) << 20);
}

// Not a limit: what the simulated VLP16 (5 degrees, 20 m) takes, for the
// record. The table grows to 262144 slots, and doubles through a rehash.
TEST(NativeGainMemory, TheSimulatedLidarsPeakIsReported) {
  const std::size_t bistro = peakOfGain(M_PI / 4.0, M_PI / 36.0, 20.0);
  std::printf("VLP16 5 degree, 20 m: peak %.2f MB\n", bistro / 1.0e6);
  RecordProperty("vlp16_peak_bytes", std::to_string(bistro));
  EXPECT_LT(bistro, std::size_t(32) << 20);
}

}  // namespace
