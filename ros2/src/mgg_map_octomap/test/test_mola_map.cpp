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
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

#include "mgg_core/graph_manager.h"
#include "mgg_core/ground_projection.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/path_selection.h"
#include "mgg_map_octomap/mola_map.h"
#include "mgg_map_octomap/native_mola_grid.h"
#include "reference_binary_grid.h"

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

std::string readFile(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(input),
                     std::istreambuf_iterator<char>());
}

// Atomic like the mapping worker's publication: a reader sees the previous
// or the new file, never a truncated one. The loader may be mid-retry while a
// test publishes, and a truncated file would be a structural error.
void write(const std::filesystem::path& path, const std::string& bytes) {
  const std::filesystem::path staging = path.string() + ".staging";
  {
    std::ofstream output(staging, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(output.good());
    output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    ASSERT_TRUE(output.good());
  }
  std::filesystem::rename(staging, path);
}

std::vector<Voxel> freeBlock() {
  std::vector<Voxel> result;
  for (std::int64_t x = -10; x <= 10; ++x)
    for (std::int64_t y = -4; y <= 4; ++y)
      for (std::int64_t z = -1; z <= 2; ++z) result.push_back({x, y, z});
  return result;
}

// One mapping worker product: the exact source snapshot bytes it was built
// from, the index that names them, the planner grid, and the authority key
// that a heartbeat carries for it.
struct Product {
  MolaSnapshotRequest request;
  std::string source_bytes;
  std::string index_bytes;
  std::string grid_bytes;
};

// A peer root laid out like the deployment: the product lives under mola/
// (source.json, index.json, components/). The capture process's own
// snapshot.json in the peer root is not part of the product and is never
// written here; the loader must ignore it.
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
                              double surface_fraction = 0.5,
                              std::size_t retired = 0) {
    const Product product =
        build(revision, std::move(occupied), std::move(free), qualified,
              transform, std::move(artifact_snapshot_id),
              std::move(artifact_source_digest), surface_fraction, retired);
    publish(product);
    return product.request;
  }

  // Write a built product in the worker's order: the artifact, then the
  // source bytes, then the index that names them.
  void publish(const Product& product) {
    write(root / "mola" / "components" / "native.sdpg", product.grid_bytes);
    write(root / "mola" / "source.json", product.source_bytes);
    write(root / "mola" / "index.json", product.index_bytes);
  }

  Product build(std::uint64_t revision, std::vector<Voxel> occupied,
                std::vector<Voxel> free, bool qualified = true,
                Eigen::Isometry3d transform = Eigen::Isometry3d::Identity(),
                std::string artifact_snapshot_id = {},
                std::string artifact_source_digest = {},
                double surface_fraction = 0.5, std::size_t retired = 0) {
    std::sort(occupied.begin(), occupied.end(), less);
    std::sort(free.begin(), free.end(), less);
    free.erase(std::remove_if(free.begin(), free.end(), [&](const Voxel& value) {
                 return std::binary_search(occupied.begin(), occupied.end(), value, less);
               }), free.end());
    const std::string geometry(64, static_cast<char>('a' + revision));
    const std::string snapshot_id(64, static_cast<char>('1' + revision));
    const std::uint64_t source_stamp = 1000 + revision;
    // Every stored point is either a surface sample or an endpoint the builder
    // retired because later qualified rays saw through its voxel.
    const std::size_t points = occupied.size() + retired;
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
        {"schema", "swarmdeck.mola_planner_grid.v2"},
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
        {"source_point_count", points},
        {"occupied_count", occupied.size()},
        {"free_count", free.size()},
        {"surface_count", occupied.size()},
        {"retired_count", retired},
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
    return {{"component:test", 1, revision, geometry, source_stamp, transform},
            source_bytes, index.dump(), grid};
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

// Rewrites a published grid's SDMGRID1 metadata and restates the index
// descriptors, so a test can present one specific contract violation.
void rewriteGridMetadata(const std::filesystem::path& root,
                         const std::function<void(json&)>& mutate) {
  const auto grid_path = root / "mola" / "components" / "native.sdpg";
  const std::string grid = readFile(grid_path);
  ASSERT_GT(grid.size(), 12u);
  std::uint32_t metadata_size = 0;
  for (unsigned int i = 0; i < 4; ++i)
    metadata_size |= static_cast<std::uint32_t>(
                         static_cast<unsigned char>(grid.at(8 + i)))
                     << (8 * i);
  json metadata = json::parse(grid.substr(12, metadata_size));
  mutate(metadata);
  const std::string metadata_bytes = metadata.dump();
  std::string replacement("SDMGRID1", 8);
  u32(replacement, static_cast<std::uint32_t>(metadata_bytes.size()));
  replacement += metadata_bytes;
  replacement += grid.substr(12 + metadata_size);
  write(grid_path, replacement);
  json index = json::parse(readFile(root / "mola" / "index.json"));
  index["artifacts"][0]["planner"]["size_bytes"] = replacement.size();
  index["artifacts"][0]["planner"]["sha256"] = sha256(replacement);
  write(root / "mola" / "index.json", index.dump());
}

std::vector<Voxel> freeBlockWithout(const Voxel& endpoint) {
  auto result = freeBlock();
  result.erase(std::remove_if(result.begin(), result.end(),
                              [&](const Voxel& value) {
                                return value.x == endpoint.x &&
                                       value.y == endpoint.y &&
                                       value.z == endpoint.z;
                              }),
               result.end());
  return result;
}

TEST(MolaMap, TransientDiscsAreOccupiedUntilTheyExpire) {
  Publication publication;
  const Voxel endpoint{10, 0, 2};
  const auto request = publication.publish(0, {endpoint},
                                           freeBlockWithout(endpoint), true,
                                           Eigen::Isometry3d::Identity());
  MolaMap provider(config(publication));
  mgg::MapInterface* map = &provider;
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return map->getStatus(); })) << provider.lastError();

  const Eigen::Vector3d here(0.1, 0.1, 0.1);
  const Eigen::Vector3d there(0.5, 0.1, 0.1);
  const Eigen::Vector3d body(0.1, 0.1, 0.1);
  ASSERT_EQ(map->getBoxStatus(here, body, true), VoxelStatus::kFree);
  ASSERT_EQ(map->getPathStatus(here, there, body, true), VoxelStatus::kFree);

  // A neighbour stands beside the segment: nothing in the map says so.
  provider.setTransientDiscs({Eigen::Vector2d(0.3, 0.35)}, 0.25, 60.0);
  EXPECT_EQ(map->getBoxStatus(here, body, true), VoxelStatus::kFree);
  EXPECT_EQ(map->getPathStatus(here, there, body, true), VoxelStatus::kOccupied);
  EXPECT_EQ(map->getStrictPathStatus(here, there, body), VoxelStatus::kOccupied);
  EXPECT_EQ(map->getBoxStatus(Eigen::Vector3d(0.3, 0.2, 0.1), body, true),
            VoxelStatus::kOccupied);
  EXPECT_EQ(map->getOccupiedOnlyCylinderPathStatus(here, there, 0.1, 0.2),
            VoxelStatus::kOccupied);

  // A sweep that starts inside the neighbour's reach may leave it but not
  // approach it: two robots parked side by side must be able to drive apart.
  const Eigen::Vector3d beside(0.3, 0.2, 0.1);
  EXPECT_EQ(map->getBoxStatus(beside, body, true), VoxelStatus::kOccupied);
  EXPECT_EQ(map->getPathStatus(beside, Eigen::Vector3d(0.3, -0.4, 0.1), body,
                               true),
            VoxelStatus::kFree);
  EXPECT_EQ(map->getStrictPathStatus(beside, Eigen::Vector3d(0.3, -0.4, 0.1),
                                     body),
            VoxelStatus::kFree);
  EXPECT_EQ(map->getPathStatus(beside, Eigen::Vector3d(0.3, 0.3, 0.1), body,
                               true),
            VoxelStatus::kOccupied);
  EXPECT_EQ(map->getPathStatus(beside, Eigen::Vector3d(0.9, 0.6, 0.1), body,
                               true),
            VoxelStatus::kOccupied);

  // The neighbour left, or its reports stopped: the map decides again.
  provider.setTransientDiscs({}, 0.25, 60.0);
  EXPECT_EQ(map->getPathStatus(here, there, body, true), VoxelStatus::kFree);
  provider.setTransientDiscs({Eigen::Vector2d(0.3, 0.35)}, 0.25, 0.05);
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_EQ(map->getPathStatus(here, there, body, true), VoxelStatus::kFree);
}


TEST(MolaMap, ZeroSizePathVisitsEveryVoxelTheSegmentCrossesAndHonoursDiscs) {
  // The robot's own-pose link is checked along its centre line: a zero-size
  // sweep. It must still meet every voxel the segment crosses, here
  // [0,0.2) x [0.2,0.4) x [0,0.2), which it enters between fractions 0.8 and
  // 0.9 (review r0), and a neighbour's transient disc.
  Publication publication;
  const Voxel wall{0, 1, 0};
  const auto request = publication.publish(
      0, {wall}, freeBlockWithout(wall), true, Eigen::Isometry3d::Identity(),
      {}, {}, /*surface_fraction=*/0.9);
  MolaMap provider(config(publication));
  mgg::MapInterface* map = &provider;
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return map->getStatus(); }))
      << provider.lastError();
  const Eigen::Vector3d a(0.02, 0.04, 0.10);
  const Eigen::Vector3d b(0.22, 0.24, 0.10);
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  for (const bool stop_at_unknown : {false, true}) {
    EXPECT_EQ(map->getPathStatus(a, b, zero, stop_at_unknown),
              VoxelStatus::kOccupied);
    EXPECT_EQ(map->getPathStatus(b, a, zero, stop_at_unknown),
              VoxelStatus::kOccupied);
  }
  // Away from the wall the centre line is free, until a neighbour stands on
  // it.
  const Eigen::Vector3d c(0.5, 0.1, 0.1);
  const Eigen::Vector3d d(1.3, 0.1, 0.1);
  ASSERT_EQ(map->getPathStatus(c, d, zero, true), VoxelStatus::kFree);
  provider.setTransientDiscs({Eigen::Vector2d(0.9, 0.3)}, 0.25, 60.0);
  EXPECT_EQ(map->getPathStatus(c, d, zero, true), VoxelStatus::kOccupied);
}

// Visibility retirement. A peer robot captured beside this one leaves an
// occupied voxel and terrain surface samples that obstacle expiry alone cannot
// disprove, so the builder drops both once later qualified rays have seen
// through the voxel. Raw source provenance remains separate from bounded
// materialized surface and retired-extrema counts.
TEST(MolaMap, AcceptsVisibilityRetiredEndpointsAgainstStoredPointCount) {
  Publication publication;
  const Voxel endpoint{10, 0, 2};
  const auto request = publication.publish(
      0, {endpoint}, freeBlockWithout(endpoint), true,
      Eigen::Isometry3d::Identity(), {}, {}, 0.5, 1);
  MolaMap provider(config(publication));
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  EXPECT_EQ(provider.getVoxelStatus({2.1, 0.1, 0.5}), VoxelStatus::kOccupied);
  EXPECT_EQ(provider.getVoxelStatus({0.1, 0.1, 0.1}), VoxelStatus::kFree);
}

TEST(MolaMap, AcceptsCompactedSurfacesWithExactRawProvenance) {
  Publication publication;
  const Voxel endpoint{10, 0, 2};
  const auto request = publication.publish(
      0, {endpoint}, freeBlockWithout(endpoint), true,
      Eigen::Isometry3d::Identity(), {}, {}, 0.5, 1);
  rewriteGridMetadata(publication.root, [](json& metadata) {
    metadata["point_count"] = 1;
    metadata["retired_count"] = 0;
  });
  MolaMap provider(config(publication));
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  EXPECT_EQ(provider.getVoxelStatus({2.1, 0.1, 0.5}), VoxelStatus::kOccupied);
  EXPECT_EQ(provider.getVoxelStatus({0.1, 0.1, 0.1}), VoxelStatus::kFree);
}

TEST(MolaMap, RejectsSurfaceAndRetiredCountsBelowMaterializedPointCount) {
  Publication publication;
  const Voxel endpoint{10, 0, 2};
  const auto request = publication.publish(
      0, {endpoint}, freeBlockWithout(endpoint), true,
      Eigen::Isometry3d::Identity(), {}, {}, 0.5, 1);
  // Claim nothing was retired while the materialized count still includes it.
  rewriteGridMetadata(publication.root,
                      [](json& metadata) { metadata["retired_count"] = 0; });
  MolaMap provider(config(publication));
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return !provider.lastError().empty(); }));
  EXPECT_FALSE(provider.getStatus());
  EXPECT_EQ(provider.getVoxelStatus({2.1, 0.1, 0.5}), VoxelStatus::kUnknown);
}

TEST(MolaMap, RejectsAGridWithoutARetiredCount) {
  Publication publication;
  const auto request = publication.publish(0, {{5, 0, 0}}, freeBlock());
  rewriteGridMetadata(publication.root,
                      [](json& metadata) { metadata.erase("retired_count"); });
  MolaMap provider(config(publication));
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return !provider.lastError().empty(); }));
  EXPECT_FALSE(provider.getStatus());
  EXPECT_NE(provider.lastError().find("metadata fields are invalid"),
            std::string::npos)
      << provider.lastError();
}

TEST(MolaMap, RejectsRetirementWithoutQualifiedRayEvidence) {
  Publication publication;
  const auto request = publication.publish(
      0, {{5, 0, 0}}, {}, false, Eigen::Isometry3d::Identity(), {}, {}, 0.5, 1);
  MolaMap provider(config(publication));
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return !provider.lastError().empty(); }));
  EXPECT_FALSE(provider.getStatus());
  EXPECT_NE(provider.lastError().find("lack qualified ray evidence"),
            std::string::npos)
      << provider.lastError();
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

TEST(MolaMap, ViewpointBesideAnOccupiedColumnLacksClearanceThoughItsBoxIsFree) {
  // An occupied column over x=[1.0,1.2], y=[0.0,0.2], z=[0.0,0.6].
  Publication publication;
  MolaMap provider(config(publication));
  const auto request =
      publication.publish(0, {{5, 0, 0}, {5, 0, 1}, {5, 0, 2}}, freeBlock(),
                          true, Eigen::Isometry3d::Identity());
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  mgg::RobotParams robot;
  robot.size = Eigen::Vector3d(0.6, 0.4, 0.3);
  robot.bound_mode = mgg::BoundModeType::kExactBound;
  mgg::PlanningParams planning;
  planning.viewpoint_clearance_margin = 0.1;

  // 0.25 m beside the column. The body box, 0.2 m to either side, is free;
  // the inscribed radius plus margin, 0.3 m, reaches the column.
  const mgg::StateVec beside(1.1, 0.45, 0.3, 0.0);
  EXPECT_EQ(provider.getBoxStatus(beside.head<3>(), robot.getPlanningSize(),
                                  true),
            VoxelStatus::kFree);
  EXPECT_FALSE(mgg::viewpointClear(provider, robot, planning, beside));
  EXPECT_TRUE(mgg::viewpointClear(provider, robot, planning,
                                  mgg::StateVec(1.1, 0.55, 0.3, 0.0)));
  // The inscribed radius alone clears it.
  planning.viewpoint_clearance_margin = 0.0;
  EXPECT_TRUE(mgg::viewpointClear(provider, robot, planning, beside));
  // Only the body's height counts: a body riding above the column clears it.
  planning.viewpoint_clearance_margin = 0.1;
  EXPECT_TRUE(mgg::viewpointClear(provider, robot, planning,
                                  mgg::StateVec(1.1, 0.45, 0.9, 0.0)));
}

TEST(MolaMap, ExplorationInACorridorNarrowerThanTheClearanceStillHasAPath) {
  // A corridor 0.6 m wide along x: walls over y=[-0.4,-0.2] and [0.4,0.6].
  std::vector<Voxel> walls;
  std::vector<Voxel> corridor;
  for (std::int64_t x = 0; x <= 10; ++x) {
    for (std::int64_t z = 0; z <= 3; ++z) {
      walls.push_back({x, -2, z});
      walls.push_back({x, 2, z});
      for (std::int64_t y = -1; y <= 1; ++y) corridor.push_back({x, y, z});
    }
  }
  Publication publication;
  MolaMap provider(config(publication));
  provider.requestSnapshot(publication.publish(0, walls, corridor));
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();

  // Inscribed radius 0.25 m plus 0.1 m margin: wider than the corridor's
  // 0.3 m half-width, so no vertex along it has viewpoint clearance.
  mgg::RobotParams robot;
  robot.type = mgg::RobotType::kAerialRobot;
  robot.size = Eigen::Vector3d(0.5, 0.5, 0.2);
  mgg::PlanningParams planning;
  planning.viewpoint_clearance_margin = 0.1;
  planning.edge_length_min = 0.05;
  planning.edge_length_max = 1.0;
  planning.edge_overshoot = 0.0;
  planning.nearest_range = 0.5;
  planning.nearest_range_min = 0.05;
  planning.nearest_range_max = 2.0;
  planning.nearest_range_z = 1.0;
  mgg::ExpandContext context;
  context.map = &provider;
  context.robot = &robot;
  context.planning = &planning;
  context.robot_box_size = Eigen::Vector3d(0.2, 0.2, 0.2);
  const mgg::StateVec root(0.1, 0.1, 0.3, 0.0);
  mgg::GraphManager graph;
  graph.addVertex(new mgg::Vertex(0, root));
  mgg::GridGraphParams grid;
  grid.min_val = {0.0, 0.0, 0.0};
  grid.max_val = {1.6, 0.0, 0.0};
  grid.resolution = {0.2, 0.2, 0.2};
  ASSERT_EQ(mgg::buildGridGraph(graph, root, grid, context, 0.0).status,
            mgg::GridGraphStatus::kOk);
  ASSERT_GT(graph.getNumVertices(), 2);
  for (auto& entry : graph.vertices_map_) entry.second->vol_gain.gain = 1.0;

  const auto selection = mgg::selectBestPath(
      graph, planning, robot, mgg::EdgeInclinations(), 0.2, 0.0, {}, 0.0,
      [&](const mgg::Vertex& v) {
        return mgg::viewpointClear(provider, robot, planning, v.state);
      });
  ASSERT_FALSE(selection.best_path.empty());
  EXPECT_TRUE(selection.unclear_viewpoint);
  EXPECT_GT(selection.paths_without_clear_viewpoint, 0);
  EXPECT_GT(selection.best_path.back()->state.x(), 1.0);
}

TEST(MolaMap, OccupiedOnlyCylinderPreservesRotatedCapsuleGeometry) {
  Eigen::Isometry3d component_from_navigation = Eigen::Isometry3d::Identity();
  component_from_navigation.linear() =
      Eigen::AngleAxisd(0.37, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  component_from_navigation.translation() = Eigen::Vector3d(2.0, -1.0, 0.4);
  const Eigen::Vector3d component_start(0.1, 0.1, 0.1);
  const Eigen::Vector3d component_end(1.1, 0.1, 0.1);
  const Eigen::Vector3d navigation_start =
      component_from_navigation.inverse() * component_start;
  const Eigen::Vector3d navigation_end =
      component_from_navigation.inverse() * component_end;
  constexpr double radius = 0.30;
  constexpr double height = 0.40;
  const auto classify = [&](const Voxel& obstacle,
                            const Eigen::Vector3d& from,
                            const Eigen::Vector3d& to) {
    Publication publication;
    MolaMap provider(config(publication));
    const auto request = publication.publish(
        0, {obstacle}, freeBlock(), true, component_from_navigation);
    provider.requestSnapshot(request);
    if (!waitFor([&]() { return provider.getStatus(); })) {
      ADD_FAILURE() << provider.lastError();
      return VoxelStatus::kUnknown;
    }
    return provider.getOccupiedOnlyCylinderPathStatus(from, to, radius,
                                                       height);
  };

  // The {6,2,0} cell occupies x=[1.2,1.4], y=[0.4,0.6] in component
  // coordinates. Its closest corner is sqrt(0.1^2 + 0.3^2) from the end
  // cap, outside radius 0.3 while still inside its enclosing square.
  EXPECT_EQ(classify({6, 2, 0}, navigation_start, navigation_end),
            VoxelStatus::kFree);
  EXPECT_EQ(classify({6, 2, 0}, navigation_end, navigation_start),
            VoxelStatus::kFree);

  // These cell AABBs respectively overlap and touch the cylindrical side.
  EXPECT_EQ(classify({2, 1, 0}, navigation_start, navigation_end),
            VoxelStatus::kOccupied);
  EXPECT_EQ(classify({2, 2, 0}, navigation_start, navigation_end),
            VoxelStatus::kOccupied);
  EXPECT_EQ(classify({2, 1, 0}, navigation_end, navigation_start),
            VoxelStatus::kOccupied);
}

TEST(MolaMap, MeasuredGroundBelowBodyDoesNotInheritVoxelTop) {
  Publication publication;
  MolaMap provider(config(publication));
  // Cell z=-1 spans [-0.2,0]. Its measured return is -0.135, below the
  // body's -0.013 lower face even though the coarse occupied cell overlaps.
  const auto request = publication.publish(0, {{0, 0, -1}}, freeBlock(), true,
                                           Eigen::Isometry3d::Identity(), {}, {},
                                           0.325);
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); })) << provider.lastError();
  const Eigen::Vector3d body(0.1, 0.1, 0.1343);
  EXPECT_EQ(provider.getPathStatus(body, body, {0.2, 0.2, 0.295}, false),
            VoxelStatus::kFree);
  EXPECT_EQ(provider.getOccupiedOnlyCylinderPathStatus(body, body, 0.1, 0.295),
            VoxelStatus::kFree);
  EXPECT_EQ(provider.getStrictBoxStatus(body, {0.2, 0.2, 0.295}),
            VoxelStatus::kOccupied);
}

TEST(MolaMap, VisibleScanLiftsTheWallBandIntoTheComponentFrame) {
  // The component frame sits 1 m above navigation. A wall column over
  // x = [2.0, 2.2) holds one return at component z = [1.4, 1.6), navigation
  // z = [0.4, 0.6), and unknown gaps above it.
  Publication publication;
  MolaMap provider(config(publication));
  Eigen::Isometry3d component_from_navigation = Eigen::Isometry3d::Identity();
  component_from_navigation.translation() = Eigen::Vector3d(0.0, 0.0, 1.0);
  const auto request = publication.publish(0, {{10, 2, 7}}, freeBlock(), true,
                                           component_from_navigation);
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();

  // One ray, in navigation, rising through the gap at z = [0.6, 0.8).
  const Eigen::Vector3d origin(0.1, 0.5, 0.5);
  const std::vector<Eigen::Vector3d> ray{Eigen::Vector3d(10.1, 0.5, 1.3)};
  const auto behindTheWall = [&](const mgg::WallBand& wall) {
    mgg::GainCounts counts;
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> log;
    provider.getVisibleScanStatus(origin, ray, wall, counts, log, {});
    int behind = 0;
    for (const auto& entry : log)
      if (entry.second == VoxelStatus::kUnknown && entry.first.x() > 2.2)
        ++behind;
    return behind;
  };
  // The body band around the return, in navigation: the column is a wall.
  EXPECT_EQ(behindTheWall({0.35, 0.65}), 0);
  // A band below it: the ray passes the gap.
  EXPECT_GT(behindTheWall({-0.25, 0.05}), 30);
}

TEST(NativeMolaGrid, HighAndUnmeasuredReturnsRemainBlockingOnSlopedSweep) {
  using Grid = mgg::NativeMolaGrid;
  const Eigen::Vector3d start(0.1, 0.1, 0.1343);
  const Eigen::Vector3d end(0.5, 0.1, 0.3343);
  Grid low(0.2, {{0, 0, -1}}, {}, {{{0, 0, -1}, -0.135}});
  EXPECT_EQ(low.getOccupiedOnlyCylinderPathStatus(start, end, 0.1, 0.295),
            VoxelStatus::kFree);
  Grid high(0.2, {{0, 0, -1}}, {}, {{{0, 0, -1}, -0.0132}});
  EXPECT_EQ(high.getOccupiedOnlyCylinderPathStatus(start, end, 0.1, 0.295),
            VoxelStatus::kOccupied);
  Grid missing(0.2, {{0, 0, -1}}, {}, {});
  EXPECT_EQ(missing.getOccupiedOnlyCylinderPathStatus(start, end, 0.1, 0.295),
            VoxelStatus::kOccupied);
}

TEST(NativeMolaGrid, R2MeasuredGroundDoesNotBlockInflatedBody) {
  using Grid = mgg::NativeMolaGrid;
  Grid map(0.2, {{135, 163, -1}}, {}, {{{135, 163, -1}, -0.14447}});
  const Eigen::Vector3d center(26.8890352327, 32.8298884726, 0.1343023994);
  const Eigen::Vector3d box_size(0.913862134, 0.913862134, 0.295);
  EXPECT_EQ(map.getPathStatus(center, center, box_size, false),
            VoxelStatus::kFree);
  EXPECT_EQ(map.getOccupiedOnlyCylinderPathStatus(center, center, 0.456931067,
                                                   0.295),
            VoxelStatus::kFree);
  EXPECT_EQ(map.getStrictBoxStatus(center, box_size), VoxelStatus::kOccupied);
}

TEST(NativeMolaGrid, RaySupercoverIncludesCornerCellsInBothDirections) {
  using Grid = mgg::NativeMolaGrid;
  Grid map(0.2, {{1, 0, 0}}, {{0, 0, 0}, {1, 1, 0}}, {});
  const Eigen::Vector3d a(0.1, 0.1, 0.1);
  const Eigen::Vector3d b(0.3, 0.3, 0.1);
  EXPECT_EQ(map.getRayStatus(a, b, false), VoxelStatus::kOccupied);
  EXPECT_EQ(map.getRayStatus(b, a, false), VoxelStatus::kOccupied);
}

TEST(NativeMolaGrid, RaySupercoverIncludesStartFaceInBothDirections) {
  using Grid = mgg::NativeMolaGrid;
  const Eigen::Vector3d face(0.2, 0.1, 0.1);
  const Eigen::Vector3d interior(0.3, 0.1, 0.1);
  Grid occupied(0.2, {{0, 0, 0}}, {{1, 0, 0}}, {});
  EXPECT_EQ(occupied.getRayStatus(face, interior, false),
            VoxelStatus::kOccupied);
  EXPECT_EQ(occupied.getRayStatus(interior, face, false),
            VoxelStatus::kOccupied);

  Grid unknown(0.2, {}, {{1, 0, 0}}, {});
  EXPECT_EQ(unknown.getRayStatus(face, interior, true), VoxelStatus::kUnknown);
  EXPECT_EQ(unknown.getRayStatus(interior, face, true), VoxelStatus::kUnknown);
}

TEST(MolaMap, OccupiedOnlyCylinderRejectsTiltedOrExcessiveQueries) {
  Publication publication;
  Eigen::Isometry3d component_from_navigation = Eigen::Isometry3d::Identity();
  // Beyond kMaxAuthorityTiltRad (0.05): a merged frame tilts up to 1.4
  // degrees, this one 5.7.
  component_from_navigation.linear() =
      Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const auto request = publication.publish(
      0, {{2, 1, 0}}, freeBlock(), true, component_from_navigation);
  MolaMap provider(config(publication));
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();

  const Eigen::Vector3d start(0.1, 0.1, 0.1);
  const Eigen::Vector3d end(1.1, 0.1, 0.1);
  EXPECT_EQ(provider.getOccupiedOnlyCylinderPathStatus(start, end, 0.30, 0.40),
            VoxelStatus::kUnknown);
  EXPECT_EQ(provider.getOccupiedOnlyCylinderPathStatus(start, end, 4000.0,
                                                        0.40),
            VoxelStatus::kUnknown);
}

TEST(MolaMap, SmallAuthorityTiltIsTolerated) {
  // A peer SLAM correction between ground robots carries a few milliradians
  // of pitch and roll. Footprint queries must keep working under it, or a
  // merged fleet loses every lattice edge the moment its component frame
  // stops being its own map frame.
  Publication publication;
  Eigen::Isometry3d component_from_navigation = Eigen::Isometry3d::Identity();
  component_from_navigation.linear() =
      Eigen::AngleAxisd(0.005, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const auto request = publication.publish(
      0, {{2, 1, 0}}, freeBlock(), true, component_from_navigation);
  MolaMap provider(config(publication));
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();

  const Eigen::Vector3d start(0.1, 0.1, 0.1);
  const Eigen::Vector3d end(1.1, 0.1, 0.1);
  EXPECT_EQ(provider.getOccupiedOnlyCylinderPathStatus(start, end, 0.30, 0.40),
            VoxelStatus::kOccupied);
  std::vector<mgg::XYCellCenter> centers;
  EXPECT_TRUE(provider.getCircleIntersectingXYCellCenters(
      Eigen::Vector2d(0.5, 0.3), 0.3, 64, centers));
  EXPECT_FALSE(centers.empty());
  EXPECT_GT(mgg::authorityTiltRad(component_from_navigation.linear()), 0.004);
  EXPECT_LT(mgg::authorityTiltRad(component_from_navigation.linear()),
            mgg::kMaxAuthorityTiltRad);
}

TEST(MolaMap, MergedFrameTiltOfADegreeIsToleratedAndThreeDegreesIsNot) {
  // An inter-robot merge is a 6 DoF registration between base frames and
  // leaves up to 1.4 degrees of roll or pitch (benchbot 2026-09-20: robot_0's
  // frame at 1.174 degrees, 0.0205 rad, refused every plan at the earlier
  // 0.02 rad bound). Footprint queries accept it; a 3.4 degree tilt does not.
  for (const auto& [tilt, accepted] :
       {std::pair{0.0205, true}, std::pair{0.06, false}}) {
    Publication publication;
    Eigen::Isometry3d component_from_navigation = Eigen::Isometry3d::Identity();
    component_from_navigation.linear() =
        Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const auto request = publication.publish(
        0, {{2, 1, 0}}, freeBlock(), true, component_from_navigation);
    MolaMap provider(config(publication));
    provider.requestSnapshot(request);
    ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
        << provider.lastError();
    std::vector<mgg::XYCellCenter> centers;
    EXPECT_EQ(provider.getCircleIntersectingXYCellCenters(
                  Eigen::Vector2d(0.5, 0.3), 0.3, 64, centers),
              accepted)
        << "tilt " << tilt;
    EXPECT_EQ(centers.empty(), !accepted);
    EXPECT_EQ(mgg::authorityTiltAcceptable(component_from_navigation.linear()),
              accepted);
  }
}

TEST(MolaMap, IndexRacingBehindTheSnapshotResolvesWithinTheLoadBudget) {
  // The worker writes mola/source.json before mola/index.json. A request
  // that lands between the two must wait for the index rather than fail the
  // plan.
  Publication publication;
  publication.publish(0, {{5, 0, 0}}, freeBlock());
  const std::string stale_index = readFile(publication.root / "mola" / "index.json");
  const auto request = publication.publish(1, {{8, 0, 0}}, freeBlock());
  const std::string fresh_index = readFile(publication.root / "mola" / "index.json");
  write(publication.root / "mola" / "index.json", stale_index);

  MolaMapConfig settings = config(publication);
  settings.max_load_time = std::chrono::milliseconds(1500);
  MolaMap provider(settings);
  provider.requestSnapshot(request);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_FALSE(provider.getStatus());
  EXPECT_TRUE(provider.lastError().empty()) << provider.lastError();
  write(publication.root / "mola" / "index.json", fresh_index);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  EXPECT_EQ(provider.getVoxelStatus({1.7, 0.1, 0.1}), VoxelStatus::kOccupied);
}

TEST(MolaMap, PersistentIndexMismatchStillFailsAtTheLoadDeadline) {
  Publication publication;
  publication.publish(0, {{5, 0, 0}}, freeBlock());
  const std::string stale_index = readFile(publication.root / "mola" / "index.json");
  const auto request = publication.publish(1, {{8, 0, 0}}, freeBlock());
  write(publication.root / "mola" / "index.json", stale_index);

  MolaMapConfig settings = config(publication);
  settings.max_load_time = std::chrono::milliseconds(400);
  MolaMap provider(settings);
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return !provider.lastError().empty(); }));
  EXPECT_FALSE(provider.getStatus());
  EXPECT_NE(provider.lastError().find("not coherent"), std::string::npos)
      << provider.lastError();
}

// A peer correction moves the authority transform by one cell along x. In the
// corrected placement the first product's occupied cell (5,0,0) sits at
// navigation x=0.9 and the second's (8,0,0) at x=1.5.
Eigen::Isometry3d correctedTransform() {
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.translation() = Eigen::Vector3d(0.2, 0.0, 0.0);
  return value;
}

TEST(MolaMap, CorrectedSnapshotAtomicallyRetractsOldGeometry) {
  Publication publication;
  MolaMap provider(config(publication));
  auto first = publication.publish(0, {{5, 0, 0}}, freeBlock());
  provider.requestSnapshot(first);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); })) << provider.lastError();
  EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}), VoxelStatus::kOccupied);

  // The corrected transform makes the new key incompatible with the old
  // geometry, which is retracted before the product is decoded.
  auto second = publication.publish(1, {{8, 0, 0}}, freeBlock(), true,
                                    correctedTransform());
  provider.requestSnapshot(second);
  // The fixture is small enough that the worker may already have installed
  // the correction when requestSnapshot returns. If it has, it must be the
  // new geometry; the retracted tree may never reappear.
  if (provider.getStatus()) {
    EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}), VoxelStatus::kFree);
    EXPECT_EQ(provider.getVoxelStatus({1.5, 0.1, 0.1}),
              VoxelStatus::kOccupied);
  }
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); })) << provider.lastError();
  EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}), VoxelStatus::kFree);
  EXPECT_EQ(provider.getVoxelStatus({1.5, 0.1, 0.1}), VoxelStatus::kOccupied);

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
  std::atomic<bool> successor_started{false};
  std::atomic<bool> successor_returned{false};
  std::thread successor;
  {
    auto lease = provider.acquireReadLease();
    successor = std::thread([&]() {
      successor_started.store(true, std::memory_order_release);
      provider.requestSnapshot(second);
      successor_returned.store(true, std::memory_order_release);
    });
    ASSERT_TRUE(waitFor([&]() {
      return successor_started.load(std::memory_order_acquire);
    }));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_FALSE(successor_returned.load(std::memory_order_acquire));
    EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}),
              VoxelStatus::kOccupied);
    EXPECT_EQ(provider.getVoxelStatus({1.7, 0.1, 0.1}), VoxelStatus::kFree);
  }
  successor.join();
  // The successor is a newer revision of the same component under the same
  // transform, so the first geometry stays in service until the worker
  // installs the second; there is no gap without a map.
  EXPECT_TRUE(provider.getStatus());
  ASSERT_TRUE(waitFor([&]() {
    return provider.getStatus() &&
           provider.getVoxelStatus({1.7, 0.1, 0.1}) == VoxelStatus::kOccupied;
  })) << provider.lastError();
  EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}), VoxelStatus::kFree);
}

TEST(MolaMap, ReadLeasePinsAdmittedSnapshotAcrossTtlAndPublicationWindow) {
  Publication publication;
  auto map_config = config(publication);
  map_config.snapshot_ttl_sec = 0.05;
  MolaMap provider(map_config);
  const auto request = publication.publish(0, {{5, 0, 0}}, freeBlock());
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  const auto loaded_generation = provider.activeGeneration();

  {
    auto lease = provider.acquireReadLease();
    // Exercise the same RHS-before-move-assignment ordering used by planner
    // callback lease replacement. The nested acquisition inherits one pin.
    lease = provider.acquireReadLease();
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    EXPECT_TRUE(provider.getStatus());
    EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}),
              VoxelStatus::kOccupied);
    {
      auto nested = provider.acquireReadLease();
      EXPECT_TRUE(provider.getStatus());
    }

    lease.allowPublication();
    std::atomic<bool> other_status{true};
    std::thread other([&]() {
      auto other_lease = provider.acquireReadLease();
      other_status.store(provider.getStatus(), std::memory_order_release);
    });
    other.join();
    EXPECT_FALSE(other_status.load(std::memory_order_acquire));
    EXPECT_EQ(provider.activeGeneration(), loaded_generation);
    EXPECT_TRUE(provider.getStatus());
    lease.reacquirePublication();
  }

  EXPECT_FALSE(provider.getStatus());
  EXPECT_GT(provider.activeGeneration(), loaded_generation);
}

TEST(MolaMap, ExpiredSnapshotCannotBePinnedAtLeaseAdmission) {
  Publication publication;
  auto map_config = config(publication);
  map_config.snapshot_ttl_sec = 0.05;
  MolaMap provider(map_config);
  const auto request = publication.publish(0, {{5, 0, 0}}, freeBlock());
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  const auto loaded_generation = provider.activeGeneration();
  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  {
    auto stale = provider.acquireReadLease();
    EXPECT_FALSE(provider.getStatus());
    EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}),
              VoxelStatus::kUnknown);
  }
  EXPECT_GT(provider.activeGeneration(), loaded_generation);
}

TEST(MolaMap, HeartbeatRefreshReplacesExpiredActiveBesideOlderPin) {
  Publication publication;
  auto map_config = config(publication);
  map_config.snapshot_ttl_sec = 0.05;
  MolaMap provider(map_config);
  const auto request = publication.publish(0, {{5, 0, 0}}, freeBlock());
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  const auto loaded_generation = provider.activeGeneration();
  auto status_on_other_thread = [&]() {
    std::atomic<bool> status{false};
    std::thread other([&]() {
      auto lease = provider.acquireReadLease();
      status.store(provider.getStatus(), std::memory_order_release);
    });
    other.join();
    return status.load(std::memory_order_acquire);
  };

  {
    auto lease = provider.acquireReadLease();
    lease.allowPublication();
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    EXPECT_FALSE(status_on_other_thread());
    EXPECT_EQ(provider.activeGeneration(), loaded_generation);

    provider.requestSnapshot(request);
    ASSERT_TRUE(waitFor(status_on_other_thread)) << provider.lastError();
    EXPECT_EQ(provider.activeGeneration(), loaded_generation);
    // The transaction remains on its admitted immutable object even though a
    // coherent, freshly validated replacement is now active for new readers.
    EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}),
              VoxelStatus::kOccupied);
    lease.reacquirePublication();
  }

  EXPECT_TRUE(provider.getStatus());
  EXPECT_EQ(provider.activeGeneration(), loaded_generation);
}

TEST(MolaMap, CorrectionDuringPublicationWindowInvalidatesGenerationNotPin) {
  Publication publication;
  MolaMap provider(config(publication));
  const auto first = publication.publish(0, {{5, 0, 0}}, freeBlock());
  provider.requestSnapshot(first);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  const auto loaded_generation = provider.activeGeneration();
  // A moved authority transform is incompatible with the admitted geometry:
  // the generation advances before the product is decoded, while the
  // transaction keeps the snapshot it was admitted with.
  const auto second = publication.publish(1, {{8, 0, 0}}, freeBlock(), true,
                                          correctedTransform());

  {
    auto lease = provider.acquireReadLease();
    lease.allowPublication();
    provider.requestSnapshot(second);
    EXPECT_GT(provider.activeGeneration(), loaded_generation);
    EXPECT_TRUE(provider.getStatus());
    EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}),
              VoxelStatus::kOccupied);
    {
      auto nested = provider.acquireReadLease();
      EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}),
                VoxelStatus::kOccupied);
    }
    lease.reacquirePublication();
  }

  ASSERT_TRUE(waitFor([&]() {
    return provider.getStatus() &&
           provider.getVoxelStatus({1.1, 0.1, 0.1}) == VoxelStatus::kFree;
  })) << provider.lastError();
  EXPECT_EQ(provider.getVoxelStatus({1.5, 0.1, 0.1}),
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

TEST(MolaMap, HeartbeatStatRevalidationAndReplacementReload) {
  Publication publication;
  auto map_config = config(publication);
  map_config.snapshot_ttl_sec = 0.1;
  MolaMap provider(map_config);
  const auto request = publication.publish(0, {{5, 0, 0}}, freeBlock());
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  const auto loaded_generation = provider.activeGeneration();
  const auto initial_revalidations = provider.statRevalidationCount();

  // Repeating the same authority heartbeat only stats the product files:
  // it refreshes validity without decoding a grid or publishing a generation.
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() {
    return provider.statRevalidationCount() > initial_revalidations;
  }));
  EXPECT_TRUE(provider.getStatus());
  EXPECT_EQ(provider.activeGeneration(), loaded_generation);

  // Replacing index.json with identical bytes changes its inode. The next
  // identical heartbeat must therefore reload rather than trust the old stat.
  const auto index_path = publication.root / "mola" / "index.json";
  write(index_path, readFile(index_path));
  const auto before_reload_revalidations = provider.statRevalidationCount();
  provider.requestSnapshot(request);
  // The changed inode cannot take the stat-only path.
  EXPECT_EQ(provider.statRevalidationCount(), before_reload_revalidations);
  // Once the replacement is loaded, a repeated heartbeat can stat-confirm
  // that newly loaded publication. Poll by issuing that heartbeat until the
  // worker has finished, without assuming a semantic-generation bump.
  ASSERT_TRUE(waitFor([&]() {
    provider.requestSnapshot(request);
    return provider.statRevalidationCount() > before_reload_revalidations;
  })) << provider.lastError();
  EXPECT_TRUE(provider.getStatus());
  EXPECT_EQ(provider.activeGeneration(), loaded_generation);

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_FALSE(provider.getStatus());
  const auto expired_generation = provider.activeGeneration();
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  EXPECT_GT(provider.activeGeneration(), expired_generation);
}

// A short TTL and load budget, so successor loads run out of budget and the
// snapshot would expire within a test rather than within seconds.
MolaMapConfig successorConfig(const Publication& publication) {
  MolaMapConfig value = config(publication);
  value.snapshot_ttl_sec = 0.6;
  value.max_load_time = std::chrono::milliseconds(200);
  return value;
}

TEST(MolaMap, CompatibleSuccessorKeepsThePredecessorWhileItsProductIsAbsent) {
  // The authority key advances to a revision the mapping worker has not
  // published yet. The predecessor is the same component of the same epoch
  // under the same transform, so it stays in service across more than a TTL
  // of heartbeats, with no error reported and no generation churn.
  Publication publication;
  MolaMap provider(successorConfig(publication));
  const auto predecessor = publication.publish(0, {{5, 0, 0}}, freeBlock());
  provider.requestSnapshot(predecessor);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  const auto loaded_generation = provider.activeGeneration();

  const auto successor = publication.build(1, {{8, 0, 0}}, freeBlock());
  const auto start = std::chrono::steady_clock::now();
  const auto span = std::chrono::milliseconds(1500);
  while (std::chrono::steady_clock::now() - start < span) {
    provider.requestSnapshot(successor.request);
    EXPECT_TRUE(provider.getStatus());
    EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}),
              VoxelStatus::kOccupied);
    EXPECT_TRUE(provider.lastError().empty()) << provider.lastError();
    EXPECT_EQ(provider.activeGeneration(), loaded_generation);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_GT(provider.retainedPredecessorCount(), 0u);
  EXPECT_TRUE(provider.getStatus());
}

TEST(MolaMap, SuccessorProductLandingInstallsOnceBesideAPinnedPredecessor) {
  Publication publication;
  MolaMap provider(successorConfig(publication));
  const auto predecessor = publication.publish(0, {{5, 0, 0}}, freeBlock());
  provider.requestSnapshot(predecessor);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  const auto loaded_generation = provider.activeGeneration();
  const auto successor = publication.build(1, {{8, 0, 0}}, freeBlock());
  provider.requestSnapshot(successor.request);
  ASSERT_TRUE(waitFor([&]() {
    return provider.retainedPredecessorCount() > 0;
  }));
  EXPECT_TRUE(provider.getStatus());
  EXPECT_EQ(provider.activeGeneration(), loaded_generation);

  {
    // A transaction admitted on the predecessor stays on it while the
    // successor's product lands and the next heartbeat installs it.
    auto lease = provider.acquireReadLease();
    lease.allowPublication();
    publication.publish(successor);
    provider.requestSnapshot(successor.request);
    ASSERT_TRUE(waitFor([&]() {
      return provider.activeGeneration() != loaded_generation;
    })) << provider.lastError();
    EXPECT_EQ(provider.activeGeneration(), loaded_generation + 1);
    EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}),
              VoxelStatus::kOccupied);
    EXPECT_EQ(provider.getVoxelStatus({1.7, 0.1, 0.1}), VoxelStatus::kFree);
    lease.reacquirePublication();
  }
  EXPECT_TRUE(provider.getStatus());
  EXPECT_EQ(provider.activeGeneration(), loaded_generation + 1);
  EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}), VoxelStatus::kFree);
  EXPECT_EQ(provider.getVoxelStatus({1.7, 0.1, 0.1}), VoxelStatus::kOccupied);
  EXPECT_TRUE(provider.lastError().empty()) << provider.lastError();

  // Further heartbeats for the installed key reload without churn.
  provider.requestSnapshot(successor.request);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_TRUE(provider.getStatus());
  EXPECT_EQ(provider.activeGeneration(), loaded_generation + 1);
  EXPECT_TRUE(provider.lastError().empty()) << provider.lastError();
}

TEST(MolaMap, IncompatibleRequestRetractsTheActiveSnapshotAtOnce) {
  Publication publication;
  MolaMap provider(successorConfig(publication));
  const auto predecessor = publication.publish(0, {{5, 0, 0}}, freeBlock());
  provider.requestSnapshot(predecessor);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  auto generation = provider.activeGeneration();

  // Another epoch: the old geometry may not stand in for it, and the product
  // on disk cannot satisfy it, so the map is gone and the failure is reported.
  auto other_epoch = predecessor;
  other_epoch.epoch = 2;
  provider.requestSnapshot(other_epoch);
  EXPECT_FALSE(provider.getStatus());
  EXPECT_GT(provider.activeGeneration(), generation);
  ASSERT_TRUE(waitFor([&]() { return !provider.lastError().empty(); }));
  EXPECT_FALSE(provider.getStatus());
  EXPECT_NE(provider.lastError().find("does not match authority key"),
            std::string::npos)
      << provider.lastError();
  EXPECT_EQ(provider.retainedPredecessorCount(), 0u);

  // Back on the published key, then a moved authority transform.
  provider.requestSnapshot(predecessor);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  generation = provider.activeGeneration();
  const auto moved =
      publication.build(1, {{8, 0, 0}}, freeBlock(), true, correctedTransform());
  provider.requestSnapshot(moved.request);
  EXPECT_FALSE(provider.getStatus());
  EXPECT_GT(provider.activeGeneration(), generation);
  ASSERT_TRUE(waitFor([&]() { return !provider.lastError().empty(); }));
  EXPECT_FALSE(provider.getStatus());
  EXPECT_EQ(provider.retainedPredecessorCount(), 0u);
}

TEST(MolaMap, StructuralErrorInASuccessorStillDropsThePredecessor) {
  Publication publication;
  MolaMap provider(successorConfig(publication));
  const auto predecessor = publication.publish(0, {{5, 0, 0}}, freeBlock());
  provider.requestSnapshot(predecessor);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  const auto loaded_generation = provider.activeGeneration();

  // The successor's product is published but its grid does not match the
  // index: a broken product, not a race, so nothing may be served.
  const auto successor = publication.publish(1, {{8, 0, 0}}, freeBlock());
  const auto grid_path = publication.root / "mola" / "components" / "native.sdpg";
  write(grid_path, readFile(grid_path) + "x");
  provider.requestSnapshot(successor);
  ASSERT_TRUE(waitFor([&]() { return !provider.lastError().empty(); }));
  EXPECT_FALSE(provider.getStatus());
  EXPECT_GT(provider.activeGeneration(), loaded_generation);
  EXPECT_NE(provider.lastError().find("integrity check failed"),
            std::string::npos)
      << provider.lastError();
  EXPECT_EQ(provider.retainedPredecessorCount(), 0u);
}

TEST(MolaMap, NewerPeerSnapshotBesideAnOlderProductIsIgnored) {
  Publication publication;
  const auto older = publication.publish(0, {{5, 0, 0}}, freeBlock());
  const auto newer = publication.build(1, {{8, 0, 0}}, freeBlock());
  // The capture process has already moved the peer root's snapshot.json on
  // to a revision the worker has not published. Only mola/source.json
  // describes the product.
  write(publication.root / "snapshot.json", newer.source_bytes);

  MolaMap provider(successorConfig(publication));
  provider.requestSnapshot(older);
  ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); }))
      << provider.lastError();
  EXPECT_EQ(provider.getVoxelStatus({1.1, 0.1, 0.1}), VoxelStatus::kOccupied);
  EXPECT_TRUE(provider.lastError().empty()) << provider.lastError();

  // The key snapshot.json names has no product: a provider without a
  // predecessor reports the product's revision mismatch, not an index that
  // disagrees with snapshot.json.
  MolaMap fresh(successorConfig(publication));
  fresh.requestSnapshot(newer.request);
  ASSERT_TRUE(waitFor([&]() { return !fresh.lastError().empty(); }));
  EXPECT_FALSE(fresh.getStatus());
  EXPECT_NE(fresh.lastError().find("does not match authority key"),
            std::string::npos)
      << fresh.lastError();
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
  auto footprintAdmissible =
      [&](const mgg::MapInterface& map, const std::vector<Eigen::Vector3d>& edge) {
        const double radius = 0.5 * context.robot_box_size.head<2>().norm();
        for (const Eigen::Vector3d& point : edge) {
          std::vector<mgg::XYCellCenter> footprint;
          if (!map.getCircleIntersectingXYCellCenters(
                  point.head<2>(), radius, 4096, footprint)) {
            return false;
          }
          for (const mgg::XYCellCenter& cell : footprint) {
            Eigen::Vector3d hit;
            const VoxelStatus status = map.getGroundRayStatus(
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
  context.projected_edge_admissible =
      [&](const std::vector<Eigen::Vector3d>& edge) {
        return footprintAdmissible(provider, edge);
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

  // No recorded live map is available: build the synthetic tunnel through
  // the two index strategies on the same machine, then compare every vertex
  // and edge, including their computed floating-point values.
  std::vector<mgg::NativeMolaGrid::Cell> occupied_cells, free_cells;
  std::vector<mgg::NativeMolaGrid::Surface> surfaces;
  for (const Voxel& cell : floor) {
    occupied_cells.push_back({cell.x, cell.y, cell.z});
    surfaces.push_back({{cell.x, cell.y, cell.z}, 0.1});
  }
  for (const Voxel& cell : free)
    free_cells.push_back({cell.x, cell.y, cell.z});
  mgg::NativeMolaGrid indexed(0.2, occupied_cells, free_cells, surfaces);
  auto reference = makeReferenceBinaryGrid(0.2, occupied_cells, free_cells, surfaces);
  auto buildLattice = [&](mgg::MapInterface& map) {
    mgg::GroundProjection projection(map, planning);
    mgg::ExpandContext compared = context;
    compared.map = &map;
    compared.ground = &projection;
    compared.projected_edge_admissible =
        [&](const std::vector<Eigen::Vector3d>& edge) {
          return footprintAdmissible(map, edge);
        };
    auto lattice = std::make_unique<mgg::GraphManager>();
    const mgg::StateVec root(component_root.x(), component_root.y(),
                             component_root.z(), 0.0);
    lattice->addVertex(new mgg::Vertex(0, root));
    const auto outcome = mgg::buildGridGraph(*lattice, root, grid, compared, 0.0);
    EXPECT_EQ(outcome.status, mgg::GridGraphStatus::kOk);
    EXPECT_GT(outcome.vertices_added, 0);
    EXPECT_GT(outcome.edge_status[0], 0);
    return lattice;
  };
  auto indexed_graph = buildLattice(indexed);
  auto reference_graph = buildLattice(*reference);
  ASSERT_EQ(indexed_graph->vertices_map_.size(), reference_graph->vertices_map_.size())
      << "vertex count differs";
  for (const auto& [id, vertex] : reference_graph->vertices_map_) {
    const auto it = indexed_graph->vertices_map_.find(id);
    ASSERT_NE(it, indexed_graph->vertices_map_.end()) << "missing vertex " << id;
    for (int axis = 0; axis < vertex->state.size(); ++axis)
      ASSERT_EQ(it->second->state[axis], vertex->state[axis])
          << "vertex " << id << " axis " << axis;
  }
  ASSERT_EQ(indexed_graph->edge_map_.size(), reference_graph->edge_map_.size())
      << "edge source count differs";
  for (const auto& [id, edges] : reference_graph->edge_map_) {
    const auto it = indexed_graph->edge_map_.find(id);
    ASSERT_NE(it, indexed_graph->edge_map_.end()) << "missing edge source " << id;
    auto actual = it->second;
    auto expected = edges;
    std::sort(actual.begin(), actual.end());
    std::sort(expected.begin(), expected.end());
    ASSERT_EQ(actual.size(), expected.size()) << "edge count at vertex " << id;
    for (std::size_t i = 0; i < expected.size(); ++i) {
      ASSERT_EQ(actual[i].first, expected[i].first)
          << "edge from " << id << " at position " << i;
      ASSERT_EQ(actual[i].second, expected[i].second)
          << "edge " << id << " -> " << expected[i].first;
    }
  }
}

}  // namespace
