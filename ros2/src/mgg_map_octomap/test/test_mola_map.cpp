#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <openssl/evp.h>

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

#include "mgg_core/graph_manager.h"
#include "mgg_core/ground_projection.h"
#include "mgg_core/grid_graph.h"
#include "mgg_map_octomap/mola_map.h"

namespace {
using json = nlohmann::json;
using mgg::MolaMap;
using mgg::MolaMapConfig;
using mgg::MolaSnapshotRequest;
using mgg::VoxelStatus;

struct Voxel {
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;
};

std::string sha256(const std::string& bytes) {
  std::array<unsigned char, EVP_MAX_MD_SIZE> output{};
  unsigned int size = 0;
  EXPECT_EQ(EVP_Digest(bytes.data(), bytes.size(), output.data(), &size,
                       EVP_sha256(), nullptr), 1);
  std::ostringstream text;
  text << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < size; ++i)
    text << std::setw(2) << static_cast<unsigned int>(output[i]);
  return text.str();
}

void u32(std::string& bytes, std::uint32_t value) {
  for (unsigned int shift = 0; shift < 32; shift += 8)
    bytes.push_back(static_cast<char>((value >> shift) & 0xff));
}

void u64(std::string& bytes, std::uint64_t value) {
  for (unsigned int shift = 0; shift < 64; shift += 8)
    bytes.push_back(static_cast<char>((value >> shift) & 0xff));
}

void i64(std::string& bytes, std::int64_t value) {
  std::uint64_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));
  u64(bytes, raw);
}

void f64(std::string& bytes, double value) {
  std::uint64_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));
  u64(bytes, raw);
}

void write(const std::filesystem::path& path, const std::string& bytes) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(output.good());
  output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  ASSERT_TRUE(output.good());
}

std::vector<Voxel> freeBlock() {
  std::vector<Voxel> result;
  for (std::int64_t x = -10; x <= 10; ++x)
    for (std::int64_t y = -4; y <= 4; ++y)
      for (std::int64_t z = -1; z <= 2; ++z) result.push_back({x, y, z});
  return result;
}

class Publication {
 public:
  Publication() {
    root = std::filesystem::temp_directory_path() /
           ("mgg-mola-map-test-" + std::to_string(::getpid()) + "-" +
            std::to_string(sequence++));
    std::filesystem::create_directories(root / "mola" / "components");
  }
  ~Publication() { std::filesystem::remove_all(root); }

  MolaSnapshotRequest publish(std::uint64_t revision,
                              std::vector<Voxel> occupied,
                              std::vector<Voxel> free,
                              bool qualified = true,
                              Eigen::Isometry3d transform =
                                  Eigen::Isometry3d::Identity(),
                              std::string artifact_snapshot_id = {},
                              std::string artifact_source_digest = {},
                              double surface_fraction = 0.5) {
    std::sort(occupied.begin(), occupied.end(), less);
    std::sort(free.begin(), free.end(), less);
    free.erase(std::remove_if(free.begin(), free.end(), [&](const Voxel& value) {
                 return std::binary_search(occupied.begin(), occupied.end(), value, less);
               }), free.end());
    const std::string geometry(64, static_cast<char>('a' + revision));
    const std::string snapshot_id(64, static_cast<char>('1' + revision));
    const std::uint64_t source_stamp = 1000 + revision;
    const std::size_t points = occupied.size();
    const json chunk{{"sha256", std::string(64, 'c')},
                     {"size_bytes", 16 + 12 * points},
                     {"point_count", points},
                     {"encoding", "application/vnd.swarmdeck.xyz-f32.v1"}};
    json submap{{"observed_at_ns", source_stamp}, {"chunks", json::array({chunk})}};
    if (qualified) {
      submap["sensor_origins"] = json::array({json::array({0.0, 0.0, 0.0})});
      submap["ray_evidence"] = {
          {"return_semantics", "first_return"},
          {"deskew", "not_required"},
          {"origin_association", "single_capture"}};
    }
    json manifest{{"map_id", "onboard"},
                  {"layer_id", "persistent_geometry"},
                  {"frame_id", "component_test"},
                  {"graph_revision", {{"component_id", "component:test"},
                                      {"epoch", 1}, {"revision", revision}}},
                  {"geometry_revision", geometry},
                  {"submaps", json::array({submap})},
                  {"chunks", json::array({chunk})},
                  {"tombstones", json::array()}};
    json canonical_manifest = manifest;
    canonical_manifest["schema"] = "swarmdeck.autonomy.v1";
    json source{{"schema", "swarmdeck.autonomy.v1"},
                {"snapshot_id", snapshot_id},
                {"generated_at_ns", source_stamp},
                {"manifests", json::array({manifest})}};
    const std::string source_bytes = source.dump();
    const std::string source_digest = sha256(source_bytes);
    if (artifact_snapshot_id.empty()) artifact_snapshot_id = snapshot_id;
    if (artifact_source_digest.empty()) artifact_source_digest = source_digest;

    json metadata{
        {"schema", "swarmdeck.mola_planner_grid.v1"},
        {"graph_version", {{"component_id", "component:test"}, {"epoch", 1},
                           {"revision", revision}, {"digest", std::string(64, 'd')}}},
        {"identity", {{"geometry_revision", geometry},
                      {"native_geometry_digest", std::string(64, 'e')},
                      {"canonical_manifest_digest", sha256(canonical_manifest.dump())},
                      {"source_snapshot_id", artifact_snapshot_id},
                      {"source_sha256", artifact_source_digest},
                      {"reference_frame", "component_test"}}},
        {"source_stamp_ns", source_stamp},
        {"resolution_m", 0.2},
        {"ray_angular_resolution_rad", 0.08726646259971647},
        {"ray_step_fraction", 0.75},
        {"point_count", points},
        {"occupied_count", occupied.size()},
        {"free_count", free.size()},
        {"surface_count", points},
        {"ray_steps", free.empty() ? 0 : free.size()},
        {"qualified_ray_keyframes", qualified ? 1 : 0}};
    const std::string metadata_bytes = metadata.dump();
    std::string grid("SDMGRID1", 8);
    u32(grid, static_cast<std::uint32_t>(metadata_bytes.size()));
    grid += metadata_bytes;
    for (const auto& voxel : occupied) {
      i64(grid, voxel.x); i64(grid, voxel.y); i64(grid, voxel.z);
    }
    for (const auto& voxel : free) {
      i64(grid, voxel.x); i64(grid, voxel.y); i64(grid, voxel.z);
    }
    for (const auto& voxel : occupied) {
      i64(grid, voxel.x); i64(grid, voxel.y);
      f64(grid, (voxel.z + surface_fraction) * 0.2);
    }
    const std::string grid_digest = sha256(grid);
    json index{{"version", 1},
               {"source_snapshot_id", snapshot_id},
               {"source_sha256", source_digest},
               {"generated_at_ns", source_stamp},
               {"artifacts", json::array({
                   {{"component_id", "component:test"}, {"epoch", 1},
                    {"revision", revision}, {"geometry_revision", geometry},
                    // This is Python's canonical manifest digest in a real
                    // worker publication. It deliberately differs from the
                    // native nlohmann canonical digest carried by SDPG.
                    {"manifest_sha256", std::string(64, '9')},
                    {"path", "components/native.mola"}, {"size_bytes", 1},
                    {"sha256", std::string(64, 'f')},
                    {"planner", {{"path", "components/native.sdpg"},
                                 {"size_bytes", grid.size()},
                                 {"sha256", grid_digest},
                                 {"source_sha256", artifact_source_digest},
                                 {"source_snapshot_id", artifact_snapshot_id}}}}})}};
    write(root / "mola" / "components" / "native.sdpg", grid);
    write(root / "snapshot.json", source_bytes);
    write(root / "mola" / "index.json", index.dump());
    return {"component:test", 1, revision, geometry, source_stamp, transform};
  }

  std::filesystem::path root;

 private:
  static bool less(const Voxel& a, const Voxel& b) {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    return a.z < b.z;
  }
  static int sequence;
};
int Publication::sequence = 0;

bool waitFor(const std::function<bool()>& predicate) {
  for (int i = 0; i < 200; ++i) {
    if (predicate()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

MolaMapConfig config(const Publication& publication) {
  MolaMapConfig value;
  value.peer_root = publication.root.string();
  value.snapshot_ttl_sec = 2.0;
  value.max_load_time = std::chrono::milliseconds(1000);
  return value;
}

TEST(MolaMap, LoadsQualifiedTernaryMapAndAppliesFullSe3) {
  Publication publication;
  Eigen::Isometry3d component_from_navigation = Eigen::Isometry3d::Identity();
  component_from_navigation.linear() =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  component_from_navigation.translation() = Eigen::Vector3d(1.0, 2.0, 0.4);
  auto free = freeBlock();
  const Voxel endpoint{10, 0, 2};
  free.erase(std::remove_if(free.begin(), free.end(), [&](const Voxel& value) {
               return value.x == endpoint.x && value.y == endpoint.y &&
                      value.z == endpoint.z;
             }), free.end());
  const auto request = publication.publish(0, {endpoint}, free, true,
                                           component_from_navigation);
  MolaMap provider(config(publication));
  mgg::MapInterface* map = &provider;
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return map->getStatus(); })) << provider.lastError();
  const Eigen::Vector3d occupied_component(2.1, 0.1, 0.5);
  const Eigen::Vector3d occupied_navigation =
      component_from_navigation.inverse() * occupied_component;
  EXPECT_EQ(map->getVoxelStatus(occupied_navigation), VoxelStatus::kOccupied);
  const Eigen::Vector3d free_navigation =
      component_from_navigation.inverse() * Eigen::Vector3d(0.1, 0.1, 0.1);
  EXPECT_EQ(map->getVoxelStatus(free_navigation), VoxelStatus::kFree);
  EXPECT_EQ(map->getVoxelStatus(free_navigation + Eigen::Vector3d(0, 0, 20)),
            VoxelStatus::kUnknown);
  EXPECT_FALSE(map->augmentFreeBox(free_navigation + Eigen::Vector3d(5, 0, 0),
                                   Eigen::Vector3d::Ones()));
}

TEST(MolaMap, InvalidAndUnrepresentableQueriesRemainUnknown) {
  Publication publication;
  MolaMap provider(config(publication));
  const auto request = publication.publish(0, {{5, 0, 0}}, freeBlock());
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();

  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(provider.getVoxelStatus({nan, 0.0, 0.0}), VoxelStatus::kUnknown);
  EXPECT_EQ(provider.getVoxelStatus({1e12, 0.0, 0.0}), VoxelStatus::kUnknown);
  Eigen::Vector3d reached;
  EXPECT_EQ(provider.getRayStatus({0.1, 0.1, 0.1}, {nan, 0.0, 0.0}, true,
                                  reached),
            VoxelStatus::kUnknown);
  EXPECT_TRUE(reached.isApprox(Eigen::Vector3d(0.1, 0.1, 0.1)));
  EXPECT_EQ(provider.getRayStatus({0.1, 0.1, 0.1}, {1e12, 0.0, 0.0}, false,
                                  reached),
            VoxelStatus::kUnknown);
}

TEST(MolaMap, CorrectedSnapshotAtomicallyRetractsOldGeometry) {
  Publication publication;
  MolaMap provider(config(publication));
  auto first = publication.publish(0, {{5, 0, 0}}, freeBlock());
  provider.requestSnapshot(first);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); })) << provider.lastError();
  EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}), VoxelStatus::kOccupied);

  auto second = publication.publish(1, {{8, 0, 0}}, freeBlock());
  provider.requestSnapshot(second);
  // The fixture is small enough that the worker may already have installed
  // the correction when requestSnapshot returns. If it has, it must be the
  // new geometry; the retracted tree may never reappear.
  if (provider.getStatus()) {
    EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}), VoxelStatus::kFree);
    EXPECT_EQ(provider.getVoxelStatus({1.7, 0.1, 0.1}),
              VoxelStatus::kOccupied);
  }
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); })) << provider.lastError();
  EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}), VoxelStatus::kFree);
  EXPECT_EQ(provider.getVoxelStatus({1.7, 0.1, 0.1}), VoxelStatus::kOccupied);

  std::filesystem::remove(publication.root / "mola" / "index.json");
  provider.requestSnapshot(second);
  ASSERT_TRUE(waitFor([&]() {
    return !provider.getStatus() && !provider.lastError().empty();
  }));
  EXPECT_NE(provider.lastError().find("cannot open MOLA index"), std::string::npos);
}

TEST(MolaMap, ReadLeaseKeepsOneSnapshotAcrossAQueryTransaction) {
  Publication publication;
  MolaMap provider(config(publication));
  const auto first = publication.publish(0, {{5, 0, 0}}, freeBlock());
  provider.requestSnapshot(first);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();

  const auto second = publication.publish(1, {{8, 0, 0}}, freeBlock());
  std::atomic<bool> correction_started{false};
  std::atomic<bool> correction_returned{false};
  std::thread correction;
  {
    auto lease = provider.acquireReadLease();
    correction = std::thread([&]() {
      correction_started.store(true, std::memory_order_release);
      provider.requestSnapshot(second);
      correction_returned.store(true, std::memory_order_release);
    });
    ASSERT_TRUE(waitFor([&]() {
      return correction_started.load(std::memory_order_acquire);
    }));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_FALSE(correction_returned.load(std::memory_order_acquire));
    EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}),
              VoxelStatus::kOccupied);
    EXPECT_EQ(provider.getVoxelStatus({1.7, 0.1, 0.1}), VoxelStatus::kFree);
  }
  correction.join();
  EXPECT_FALSE(provider.getStatus());
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}), VoxelStatus::kFree);
  EXPECT_EQ(provider.getVoxelStatus({1.7, 0.1, 0.1}),
            VoxelStatus::kOccupied);
}

TEST(MolaMap, RejectsFreeCellsWithoutQualifiedRayEvidence) {
  Publication publication;
  MolaMap provider(config(publication));
  const auto request = publication.publish(0, {{5, 0, 0}}, {{0, 0, 0}}, false);
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return !provider.lastError().empty(); }));
  EXPECT_FALSE(provider.getStatus());
  EXPECT_NE(provider.lastError().find("qualified ray"), std::string::npos);
  EXPECT_EQ(provider.getVoxelStatus({0.1, 0.1, 0.1}), VoxelStatus::kUnknown);
}

TEST(MolaMap, AcceptsVerifiedReuseOfHistoricalComponentArtifact) {
  Publication publication;
  MolaMap provider(config(publication));
  // A global snapshot can advance because another component changed while
  // this component's immutable artifact is reused. The index binds current
  // source bytes; planner/SDPG provenance consistently names the historical
  // source that originally produced this same component artifact.
  const auto request = publication.publish(
      0, {{5, 0, 0}}, freeBlock(), true, Eigen::Isometry3d::Identity(),
      std::string(64, '7'), std::string(64, '8'));
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}),
            VoxelStatus::kOccupied);
}

TEST(MolaMap, HeartbeatRefreshDoesNotChurnButExpiryAndRecoveryDo) {
  Publication publication;
  auto map_config = config(publication);
  map_config.snapshot_ttl_sec = 0.1;
  MolaMap provider(map_config);
  const auto request = publication.publish(0, {{5, 0, 0}}, freeBlock());
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  const auto loaded_generation = provider.activeGeneration();
  provider.requestSnapshot(request);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_TRUE(provider.getStatus());
  EXPECT_EQ(provider.activeGeneration(), loaded_generation);

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_FALSE(provider.getStatus());
  EXPECT_GT(provider.activeGeneration(), loaded_generation);
  const auto expired_generation = provider.activeGeneration();
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  EXPECT_GT(provider.activeGeneration(), expired_generation);
}

TEST(MolaMap, DrivesCoreGridConstructionThroughMapInterface) {
  Publication publication;
  MolaMap provider(config(publication));
  const auto request = publication.publish(0, {{10, 10, 10}}, freeBlock());
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); })) << provider.lastError();

  mgg::RobotParams robot;
  robot.type = mgg::RobotType::kAerialRobot;
  robot.size = Eigen::Vector3d::Zero();
  mgg::PlanningParams planning;
  planning.edge_length_min = 0.05;
  planning.edge_length_max = 1.0;
  planning.edge_overshoot = 0.0;
  planning.nearest_range = 0.5;
  planning.nearest_range_min = 0.05;
  planning.nearest_range_max = 2.0;
  planning.nearest_range_z = 1.0;
  planning.num_vertices_max = 100;
  planning.num_edges_max = 1000;
  planning.num_loops_max = 1000;
  mgg::ExpandContext context;
  context.map = &provider;
  context.robot = &robot;
  context.planning = &planning;
  context.robot_box_size = Eigen::Vector3d::Zero();
  mgg::GraphManager graph;
  graph.addVertex(new mgg::Vertex(0, mgg::StateVec(0.1, 0.1, 0.1, 0.0)));
  mgg::GridGraphParams grid;
  grid.min_val = {-0.4, -0.4, 0.0};
  grid.max_val = {0.4, 0.4, 0.0};
  grid.resolution = {0.2, 0.2, 0.2};
  const auto result = mgg::buildGridGraph(
      graph, mgg::StateVec(0.1, 0.1, 0.1, 0.0), grid, context, 0.0);
  EXPECT_EQ(result.status, mgg::GridGraphStatus::kOk);
  EXPECT_GT(result.free_cells, 0);
  EXPECT_GT(result.vertices_added, 0);
}

TEST(MolaMap, PreservesMeasuredGroundAndRotatedFootprintCells) {
  Publication publication;
  Eigen::Isometry3d component_from_navigation = Eigen::Isometry3d::Identity();
  component_from_navigation.linear() =
      Eigen::AngleAxisd(0.37, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  component_from_navigation.translation() = Eigen::Vector3d(0.43, -0.27, -0.13);
  const auto request = publication.publish(
      0, {{0, 0, 0}}, {}, true, component_from_navigation, {}, {}, 0.1);
  MolaMap provider(config(publication));
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();

  const Eigen::Vector3d component_start(0.1, 0.1, 0.8);
  const Eigen::Vector3d component_end(0.1, 0.1, -0.8);
  Eigen::Vector3d navigation_hit;
  ASSERT_EQ(provider.getGroundRayStatus(
                component_from_navigation.inverse() * component_start,
                component_from_navigation.inverse() * component_end, false,
                navigation_hit),
            VoxelStatus::kOccupied);
  const Eigen::Vector3d component_hit =
      component_from_navigation * navigation_hit;
  EXPECT_NEAR(component_hit.z(), 0.02, 1e-6);

  const Eigen::Vector2d component_circle(0.13, -0.08);
  const Eigen::Vector3d navigation_circle_3 =
      component_from_navigation.inverse() *
      Eigen::Vector3d(component_circle.x(), component_circle.y(), 0.0);
  std::vector<mgg::XYCellCenter> navigation_cells;
  ASSERT_TRUE(provider.getCircleIntersectingXYCellCenters(
      navigation_circle_3.head<2>(), 0.46, 128, navigation_cells));
  EXPECT_GT(navigation_cells.size(), 4u);
  for (const mgg::XYCellCenter& navigation_cell : navigation_cells) {
    const Eigen::Vector3d component_cell = component_from_navigation *
        Eigen::Vector3d(navigation_cell.center.x(), navigation_cell.center.y(),
                        navigation_circle_3.z());
    EXPECT_NEAR(std::remainder(component_cell.x() - 0.1, 0.2), 0.0, 1e-6);
    EXPECT_NEAR(std::remainder(component_cell.y() - 0.1, 0.2), 0.0, 1e-6);
  }

  std::vector<mgg::XYCellCenter> bounded;
  EXPECT_FALSE(provider.getCircleIntersectingXYCellCenters(
      navigation_circle_3.head<2>(), 0.46, 1, bounded));
  EXPECT_TRUE(bounded.empty());
  EXPECT_FALSE(provider.getCircleIntersectingXYCellCenters(
      navigation_circle_3.head<2>(), 1e6, 4096, bounded));
  EXPECT_TRUE(bounded.empty());
}

TEST(MolaMap, SparseMeasuredRayCorridorGrowsFootprintValidatedGroundGraph) {
  Publication publication;
  std::vector<Voxel> floor;
  std::vector<Voxel> free;
  for (std::int64_t x = -8; x <= 8; ++x) {
    // Model one narrow group of measured floor returns and its ray-carved
    // body corridor rather than a fully observed floor/body volume.
    floor.push_back({x, 0, 0});
    for (std::int64_t y = -3; y <= 3; ++y) {
      for (std::int64_t z = 1; z <= 4; ++z) free.push_back({x, y, z});
    }
  }
  Eigen::Isometry3d component_from_navigation = Eigen::Isometry3d::Identity();
  component_from_navigation.linear() =
      Eigen::AngleAxisd(-0.31, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  component_from_navigation.translation() = Eigen::Vector3d(0.31, -0.22, -0.11);
  const auto request = publication.publish(
      0, floor, free, true, component_from_navigation, {}, {}, 0.1);
  MolaMap provider(config(publication));
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();

  mgg::RobotParams robot;
  robot.type = mgg::RobotType::kGroundRobot;
  robot.size = Eigen::Vector3d(1.023, 0.778, 0.40);
  robot.bound_mode = mgg::BoundModeType::kExactBound;
  mgg::PlanningParams planning;
  planning.max_ground_height = 0.475;
  planning.max_step_height = 0.15;
  planning.max_inclination = 0.52;
  planning.edge_length_min = 0.05;
  planning.edge_length_max = 0.8;
  planning.edge_overshoot = 0.0;
  planning.nearest_range = 0.5;
  planning.nearest_range_min = 0.05;
  planning.nearest_range_max = 2.0;
  planning.nearest_range_z = 1.0;
  planning.num_vertices_max = 100;
  planning.num_edges_max = 1000;
  planning.num_loops_max = 1000;
  mgg::GroundProjection ground(provider, planning);
  mgg::ExpandContext context;
  context.map = &provider;
  context.robot = &robot;
  context.planning = &planning;
  context.ground = &ground;
  context.robot_box_size = robot.getPlanningSize();
  context.projected_edge_admissible =
      [&](const std::vector<Eigen::Vector3d>& edge) {
        const double radius = 0.5 * context.robot_box_size.head<2>().norm();
        for (const Eigen::Vector3d& point : edge) {
          std::vector<mgg::XYCellCenter> footprint;
          if (!provider.getCircleIntersectingXYCellCenters(
                  point.head<2>(), radius, 4096, footprint)) {
            return false;
          }
          for (const mgg::XYCellCenter& cell : footprint) {
            Eigen::Vector3d hit;
            const VoxelStatus status = provider.getGroundRayStatus(
                {cell.center.x(), cell.center.y(), point.z()},
                {cell.center.x(), cell.center.y(), point.z() - 1.0}, false,
                hit);
            if (status == VoxelStatus::kUnknown ||
                (status == VoxelStatus::kOccupied &&
                 (!hit.allFinite() ||
                 std::abs(hit.z() - (point.z() - planning.max_ground_height)) >
                     planning.max_step_height + 1e-6))) {
              return false;
            }
          }
        }
        return true;
      };
  const Eigen::Vector3d component_root(0.1, 0.1, 0.495);
  const Eigen::Vector3d navigation_root =
      component_from_navigation.inverse() * component_root;
  mgg::GraphManager graph;
  graph.addVertex(new mgg::Vertex(
      0, mgg::StateVec(navigation_root.x(), navigation_root.y(),
                       navigation_root.z(), 0.0)));
  mgg::GridGraphParams grid;
  grid.min_val = {-0.4, -0.4, 0.0};
  grid.max_val = {0.4, 0.4, 0.0};
  grid.resolution = {0.2, 0.2, 0.2};
  const auto result = mgg::buildGridGraph(
      graph,
      mgg::StateVec(navigation_root.x(), navigation_root.y(),
                    navigation_root.z(), 0.0),
      grid, context, 0.0);
  EXPECT_EQ(result.status, mgg::GridGraphStatus::kOk);
  EXPECT_GT(result.free_cells, 0);
  EXPECT_GT(result.edge_status[0], 0);
  EXPECT_GT(result.vertices_added, 0)
      << "free=" << result.free_cells << " no_ground=" << result.no_ground
      << " edge_ok=" << result.edge_status[0];
}

}  // namespace
