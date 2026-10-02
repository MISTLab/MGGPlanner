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
#include <memory>
#include <optional>
#include <sstream>
#include <set>
#include <tuple>
#include <unordered_map>
#include <string>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>
#include <unistd.h>

#include "mgg_core/departure.h"
#include "mgg_core/graph_manager.h"
#include "mgg_core/global_graph.h"
#include "mgg_core/graph_merge.h"
#include "mgg_core/ground_projection.h"
#include "mgg_core/grid_graph.h"
#include "mgg_core/path_selection.h"
#include "mgg_core/voxel_walk.h"
#include "mgg_core/path_turns.h"
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
          {"return_semantics", return_semantics},
          {"deskew", deskew},
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
  // The ray evidence a qualified product's manifest declares.
  std::string return_semantics = "first_return";
  std::string deskew = "not_required";

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


TEST(MolaMap, NoGoDiscsStayUntilReplacedAndCoexistWithPeerBodies) {
  // Run 8: a Scout tipped over debris its 0.2 m map showed as a low plateau,
  // three times in 4 s, each time sent back over it. SwarmDeck marks where
  // the tilt guard tripped; the planner must not drive there. Those discs
  // do not expire in the map, and neither disc set replaces the other.
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
  // A segment well away from the no-go disc, for the peer body.
  const Eigen::Vector3d west(-0.5, 0.1, 0.1);
  const Eigen::Vector3d far_west(-1.3, 0.1, 0.1);
  ASSERT_EQ(map->getPathStatus(here, there, body, true), VoxelStatus::kFree);
  ASSERT_EQ(map->getPathStatus(west, far_west, body, true), VoxelStatus::kFree);

  provider.setNoGoDiscs({Eigen::Vector2d(0.3, 0.35)}, 0.25);
  EXPECT_EQ(map->getPathStatus(here, there, body, true), VoxelStatus::kOccupied);
  EXPECT_EQ(map->getBoxStatus(Eigen::Vector3d(0.3, 0.2, 0.1), body, true),
            VoxelStatus::kOccupied);
  // A robot standing in one may leave it, as it may leave a peer's reach.
  EXPECT_EQ(map->getPathStatus(Eigen::Vector3d(0.3, 0.2, 0.1),
                               Eigen::Vector3d(0.3, -0.4, 0.1), body, true),
            VoxelStatus::kFree);

  mgg::OrientedBox oriented;
  oriented.size = body;
  oriented.heading = -M_PI / 2;
  const Eigen::Vector3d inside(0.3, 0.2, 0.1), outside(0.3, -0.4, 0.1);
  EXPECT_EQ(mgg::orientedBoxPathStatus(*map, inside, outside, oriented, true, nullptr),
            VoxelStatus::kFree);
  EXPECT_EQ(mgg::orientedBoxPathStatus(*map, outside, inside, oriented, true, nullptr),
            VoxelStatus::kOccupied);

  // A peer body elsewhere, with a short TTL: both sets hold at once.
  provider.setTransientDiscs({Eigen::Vector2d(-0.9, 0.1)}, 0.1, 0.05);
  EXPECT_EQ(map->getPathStatus(west, far_west, body, true),
            VoxelStatus::kOccupied);
  EXPECT_EQ(map->getPathStatus(here, there, body, true), VoxelStatus::kOccupied);
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  // The peer body expired; the no-go disc did not.
  EXPECT_EQ(map->getPathStatus(west, far_west, body, true), VoxelStatus::kFree);
  EXPECT_EQ(map->getPathStatus(here, there, body, true), VoxelStatus::kOccupied);
  // Peer bodies do not clear it either.
  provider.setTransientDiscs({}, 0.25, 60.0);
  EXPECT_EQ(map->getPathStatus(here, there, body, true), VoxelStatus::kOccupied);
  // An empty set clears it.
  provider.setNoGoDiscs({}, 0.25);
  EXPECT_EQ(map->getPathStatus(here, there, body, true), VoxelStatus::kFree);
}

TEST(MolaMap, LatticeCellsInsidePeerMarginsRemainOccupied) {
  Publication publication;
  const Voxel endpoint{10, 0, 2};
  const auto request = publication.publish(0, {endpoint},
      freeBlockWithout(endpoint), true, Eigen::Isometry3d::Identity());
  MolaMap map(config(publication));
  map.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return map.getStatus(); }));
  map.setTransientDiscs({Eigen::Vector2d(0.5, 0.1)}, 0.4, 60.0);
  mgg::RobotParams robot;
  robot.type = mgg::RobotType::kGroundRobot;
  robot.size = Eigen::Vector3d::Constant(0.1);
  mgg::PlanningParams planning;
  planning.num_loops_max = 100;
  mgg::GroundProjection ground(map, planning);
  mgg::ExpandContext ctx;
  ctx.map = &map; ctx.robot = &robot; ctx.planning = &planning;
  ctx.ground = &ground; ctx.robot_box_size = robot.getPlanningSize();
  ctx.allow_unknown_lattice_body = true;
  const mgg::StateVec root(-0.1, 0.1, 0.1, 0);
  mgg::GraphManager graph;
  graph.addVertex(new mgg::Vertex(0, root));
  mgg::GridGraphParams grid;
  grid.min_val = Eigen::Vector3d::Zero();
  grid.max_val = Eigen::Vector3d(0.6, 0, 0);
  grid.resolution = Eigen::Vector3d::Constant(0.6);
  // Root is clear. The other cell and every cross-offset are inside the
  // peer margin: none is a free place, even though a stationary sweep is
  // allowed to depart an already occupied margin.
  const auto result = mgg::buildGridGraph(graph, root, grid, ctx, 0);
  EXPECT_EQ(result.free_cells, 1);
  EXPECT_EQ(result.vertices_added, 0);
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

// A deskewed Ouster capture reports the strongest return per beam. Its
// keyframes qualify rays exactly as first returns do.
TEST(MolaMap, AcceptsDeskewedStrongestReturnRayEvidence) {
  Publication publication;
  publication.return_semantics = "strongest_return";
  publication.deskew = "deskewed";
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

TEST(MolaMap, RejectsStrongestReturnGridWithAMismatchedQualifiedCount) {
  Publication publication;
  publication.return_semantics = "strongest_return";
  publication.deskew = "deskewed";
  const auto request = publication.publish(0, {{5, 0, 0}}, freeBlock());
  rewriteGridMetadata(publication.root, [](json& metadata) {
    metadata["qualified_ray_keyframes"] = 2;
  });
  MolaMap provider(config(publication));
  provider.requestSnapshot(request);
  ASSERT_TRUE(waitFor([&]() { return !provider.lastError().empty(); }));
  EXPECT_FALSE(provider.getStatus());
  EXPECT_NE(provider.lastError().find("qualified-ray provenance"),
            std::string::npos)
      << provider.lastError();
}

TEST(MolaMap, OtherReturnSemanticsDoNotQualifyRays) {
  for (const char* semantics : {"unknown", "last_return", ""}) {
    Publication publication;
    publication.return_semantics = semantics;
    publication.deskew = "deskewed";
    // The grid claims one qualified keyframe; the manifest does not back it.
    const auto request = publication.publish(0, {{5, 0, 0}}, freeBlock());
    MolaMap provider(config(publication));
    provider.requestSnapshot(request);
    ASSERT_TRUE(waitFor([&]() { return !provider.lastError().empty(); }))
        << semantics;
    EXPECT_FALSE(provider.getStatus()) << semantics;
    EXPECT_NE(provider.lastError().find("qualified-ray provenance"),
              std::string::npos)
        << semantics << ": " << provider.lastError();
  }
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
  // No floor is mapped: the observed-space part is tested on its own.
  planning.min_observed_ground_fraction = 0.0;

  // 0.25 m beside the column. The body box, 0.2 m to either side, is free;
  // the turning radius, 0.36 m, plus the arrival slack and the margin,
  // 0.51 m, reaches the column.
  const mgg::StateVec beside(1.1, 0.45, 0.3, 0.0);
  EXPECT_EQ(provider.getBoxStatus(beside.head<3>(), robot.getPlanningSize(),
                                  true),
            VoxelStatus::kFree);
  EXPECT_FALSE(mgg::viewpointClear(provider, robot, planning, beside));
  EXPECT_TRUE(mgg::viewpointClear(provider, robot, planning,
                                  mgg::StateVec(1.1, 0.75, 0.3, 0.0)));
  // 0.45 m from the column: the margin is on top of the room to turn.
  const mgg::StateVec near(1.1, 0.65, 0.3, 0.0);
  EXPECT_FALSE(mgg::viewpointClear(provider, robot, planning, near));
  planning.viewpoint_clearance_margin = 0.0;
  EXPECT_TRUE(mgg::viewpointClear(provider, robot, planning, near));
  // Only the body's height counts: a body riding above the column clears it.
  planning.viewpoint_clearance_margin = 0.1;
  EXPECT_TRUE(mgg::viewpointClear(provider, robot, planning,
                                  mgg::StateVec(1.1, 0.45, 0.9, 0.0)));
}

// Real snapshot provider: exercise authority transforms and dynamic margins,
// not just the bare native grid used by the first aerial regressions.
struct AerialMolaScene {
  AerialMolaScene() : map(config(publication)) {
    robot.type = mgg::RobotType::kAerialRobot;
    robot.size = {0.5, 0.5, 0.3};
    robot.size_extension.setZero();
    robot.safety_extension.setZero();
    robot.bound_mode = mgg::BoundModeType::kExactBound;
    planning.edge_length_min = 0.05;
    planning.edge_length_max = 1.2;
    planning.edge_overshoot = 0.0;
    planning.nearest_range = 1.2;
    planning.nearest_range_min = 0.05;
    planning.nearest_range_max = 100;
    planning.nearest_range_z = 100;
    planning.path_interpolation_distance = 0.1;
    planning.num_vertices_max = 1500;
    planning.num_edges_max = 50000;
    planning.num_loops_max = 100000;
    ctx.map = &map;
    ctx.robot = &robot;
    ctx.planning = &planning;
    ctx.robot_box_size = robot.size;
    ctx.root_is_robot = true;
    ctx.root_footprint_exempt = true;
  }
  static std::vector<Voxel> observedRoom() {
    std::vector<Voxel> cells;
    for (int x = -10; x <= 20; ++x)
      for (int y = -10; y <= 10; ++y)
        for (int z = 0; z <= 15; ++z) cells.push_back({x, y, z});
    return cells;
  }
  void load(std::vector<Voxel> free, std::vector<Voxel> occupied = {},
            const Eigen::Isometry3d& transform = Eigen::Isometry3d::Identity()) {
    map.requestSnapshot(publication.publish(0, occupied, free, true, transform));
    ASSERT_TRUE(waitFor([&]() { return map.getStatus(); })) << map.lastError();
  }
  Publication publication;
  MolaMap map;
  mgg::RobotParams robot;
  mgg::PlanningParams planning;
  mgg::ExpandContext ctx;
  const mgg::StateVec hover{0.1, 0.1, 1.5, 0};
};

TEST(MolaMap, AerialRootDepartsPeerAndNoGoMarginsThroughObservedRoom) {
  for (int margin = 0; margin < 3; ++margin) {
    SCOPED_TRACE(margin);
    AerialMolaScene scene;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    scene.load(AerialMolaScene::observedRoom(), {}, transform);
    const Eigen::Vector2d disc = scene.hover.head<2>() +
        (margin == 1 ? Eigen::Vector2d(-0.2, 0.0) : Eigen::Vector2d::Zero());
    std::optional<MolaMap::TransientDiscPin> peers;
    if (margin < 2) peers.emplace(scene.map, std::vector<Eigen::Vector2d>{disc}, 0.6);
    else scene.map.setNoGoDiscs({disc}, 0.6);
    mgg::GraphManager graph;
    graph.addVertex(new mgg::Vertex(0, scene.hover));
    mgg::Vertex target(1, scene.hover + mgg::StateVec(1.0, 0, 0, 0));
    mgg::ExpandGraphReport report;
    mgg::expandGraph(graph, target, report, scene.ctx);
    EXPECT_EQ(report.num_vertices_added, 1);
    if (report.vertex_added) EXPECT_NEAR(report.vertex_added->state.x(), 1.1, 1e-9);
  }
}

TEST(MolaMap, AerialMergeDoesNotAnnounceReconnectionAfterFailedRefresh) {
  AerialMolaScene observed, unavailable;
  observed.load(AerialMolaScene::observedRoom());
  unavailable.load({});
  mgg::MapInterface* map = &observed.map;
  mgg::GraphManager graph;
  auto* root = new mgg::Vertex(0, observed.hover);
  auto* next = new mgg::Vertex(1, observed.hover + mgg::StateVec(.4, 0, 0, 0));
  root->robot_id = next->robot_id = 1;
  graph.addVertex(root); graph.addVertex(next); graph.addEdge(root, next, .4);
  mgg::GraphExchange incoming;
  for (int i = 0; i < 2; ++i) {
    mgg::GraphExchangeVertex v;
    v.id = i; v.robot_id = 2; v.state = observed.hover + mgg::StateVec(.6 + .5*i, 0, 0, 0);
    incoming.vertices.push_back(v);
  }
  incoming.edges = {{0, 1, .5}};
  mgg::StaticPoseSource poses;
  poses.setOffset(2, 0, 0);
  mgg::ReceiverPlatform platform;
  platform.type = mgg::RobotType::kAerialRobot;
  const auto merge = [&]() {
    return mgg::mergeNeighbourGraph(graph, incoming, poses,
        [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
          return map->getStaticStrictPathStatus(a, b, observed.robot.size) == VoxelStatus::kFree;
        }, 5, platform);
  };
  EXPECT_TRUE(merge().newly_connected);
  map = &unavailable.map;
  const auto lost = merge();
  EXPECT_FALSE(lost.merged);
  EXPECT_FALSE(lost.newly_connected);
  map = &observed.map;
  const auto restored = merge();
  EXPECT_TRUE(restored.merged);
  EXPECT_FALSE(restored.newly_connected);
}

TEST(MolaMap, AerialRoadmapLinkCannotBridgeUnknownAir) {
  AerialMolaScene scene;
  auto free = AerialMolaScene::observedRoom();
  free.erase(std::remove_if(free.begin(), free.end(),
                            [](const Voxel& v) { return v.x == 2; }), free.end());
  scene.load(free);
  scene.ctx.root_is_robot = false;
  scene.ctx.root_footprint_exempt = false;
  mgg::GraphManager graph;
  graph.addVertex(new mgg::Vertex(0, scene.hover));
  EXPECT_EQ(mgg::connectStateToGraph(graph, scene.hover + mgg::StateVec(1, 0, 0, 0),
                                     scene.ctx, 1.5, true), nullptr);
  EXPECT_EQ(graph.getNumEdges(), 0);
  EXPECT_NE(mgg::connectStateToGraph(graph, scene.hover - mgg::StateVec(1, 0, 0, 0),
                                     scene.ctx, 1.5, true), nullptr);
  // A nearby state must not silently snap onto an unobserved old vertex.
  mgg::GraphManager unknown;
  unknown.addVertex(new mgg::Vertex(0, mgg::StateVec(0.5, 0.1, 1.5, 0)));
  EXPECT_EQ(mgg::connectStateToGraph(unknown, mgg::StateVec(0.51, 0.1, 1.5, 0),
                                     scene.ctx, 1.5, false), nullptr);
}

TEST(MolaMap, AerialRoadmapLinkDoesNotBorrowPhysicalRootException) {
  AerialMolaScene scene;
  auto free = AerialMolaScene::observedRoom();
  free.erase(std::remove_if(free.begin(), free.end(), [](const Voxel& v) {
    return v.x == 0 && v.y == 0 && v.z == 7;
  }), free.end());
  scene.load(free);
  // Even a caller with local-root flags must not persist an unknown link
  // into the roadmap. The blind-hover allowance is for departure, not reuse.
  mgg::GraphManager graph;
  graph.addVertex(new mgg::Vertex(0, scene.hover));
  EXPECT_EQ(mgg::connectStateToGraph(graph, scene.hover + mgg::StateVec(1, 0, 0, 0),
                                     scene.ctx, 1.5, true), nullptr);
  EXPECT_EQ(graph.getNumEdges(), 0);
}

TEST(MolaMap, AerialNearbyUnknownVertexDoesNotHideAnotherStrictLink) {
  AerialMolaScene scene;
  auto free = AerialMolaScene::observedRoom();
  free.erase(std::remove_if(free.begin(), free.end(), [](const Voxel& v) {
    return v.x == -1 && v.y == 0 && v.z == 7;
  }), free.end());
  scene.load(free);
  mgg::GraphManager graph;
  graph.addVertex(new mgg::Vertex(0, mgg::StateVec(.24, .1, 1.5, 0)));
  auto* farther = new mgg::Vertex(1, mgg::StateVec(1.33, .1, 1.5, 0));
  graph.addVertex(farther);
  const auto* linked = mgg::connectStateToGraph(
      graph, mgg::StateVec(.33, .1, 1.5, 0), scene.ctx, 1.5, false);
  ASSERT_NE(linked, nullptr);
  EXPECT_EQ(linked->parent, farther);
  EXPECT_NEAR(linked->state.x(), .33, 1e-9);
}

TEST(MolaMap, AerialTakeoffOverPadLinksPastUnknownHomeWithoutPersistingEdge) {
  AerialMolaScene scene;
  auto free = AerialMolaScene::observedRoom();
  free.erase(std::remove_if(free.begin(), free.end(), [](const Voxel& v) {
    return v.x == 0 && v.y == 0 && v.z == 7;
  }), free.end());
  scene.load(free);
  mgg::GraphManager graph;
  graph.addVertex(new mgg::Vertex(0, scene.hover));  // home in the blind column
  auto* anchor = new mgg::Vertex(1, scene.hover + mgg::StateVec(1, 0, 0, 0));
  graph.addVertex(anchor);
  const auto link = mgg::linkDeparture(graph, scene.hover, scene.ctx, 1.5);
  EXPECT_EQ(link.vertex, anchor);
  EXPECT_TRUE(link.query_local);
  EXPECT_EQ(graph.getNumVertices(), 2);
  EXPECT_EQ(graph.getNumEdges(), 0);
}

TEST(MolaMap, AerialLinkDepartureFallbackRejectsWallButAllowsUnknownOwnBody) {
  for (bool wall : {false, true}) {
    SCOPED_TRACE(wall);
    AerialMolaScene scene;
    auto free = AerialMolaScene::observedRoom();
    free.erase(std::remove_if(free.begin(), free.end(), [](const Voxel& v) {
      return v.x == 0 && v.y == 0 && v.z == 7;
    }), free.end());
    scene.load(free, wall ? std::vector<Voxel>{{0, 0, 7}} : std::vector<Voxel>{});
    mgg::GraphManager graph;
    auto* anchor = new mgg::Vertex(0, scene.hover + mgg::StateVec(1, 0, 0, 0));
    graph.addVertex(anchor);
    const auto link = mgg::linkDeparture(graph, scene.hover, scene.ctx, 1.5);
    EXPECT_EQ(link.vertex, wall ? nullptr : anchor);
    EXPECT_EQ(link.query_local, !wall);
    EXPECT_EQ(graph.getNumVertices(), 1);
    EXPECT_EQ(graph.getNumEdges(), 0);
  }
}

TEST(MolaMap, AerialRebuildDoesNotJoinKeyframesAcrossUnknownAir) {
  AerialMolaScene scene;
  auto free = AerialMolaScene::observedRoom();
  free.erase(std::remove_if(free.begin(), free.end(),
                            [](const Voxel& v) { return v.x == 2; }), free.end());
  scene.load(free);
  mgg::GraphManager graph;
  mgg::RoadmapRebuildParams params;
  params.vertex_spacing = 1.0;
  params.max_offset = 0;
  const auto result = mgg::rebuildRoadmapFromTrajectory(
      graph, {{scene.hover}, {scene.hover + mgg::StateVec(1, 0, 0, 0)}},
      scene.ctx, params);
  ASSERT_EQ(graph.getNumVertices(), 2);
  EXPECT_EQ(graph.getNumEdges(), 0);
  EXPECT_EQ(result.home_component_vertices, 1);
  EXPECT_EQ(result.chain_edges, 0);
  EXPECT_EQ(result.link_edges, 0);
}

TEST(MolaMap, AerialRebuildDoesNotOffsetIntoUnobservedBodyVolume) {
  AerialMolaScene scene;
  auto free = AerialMolaScene::observedRoom();
  free.erase(std::remove_if(free.begin(), free.end(),
                            [](const Voxel& v) { return v.x >= 3; }), free.end());
  scene.load(free, {{5, 0, 7}});
  mgg::GraphManager graph;
  mgg::RoadmapRebuildParams params;
  params.vertex_spacing = 1.0;
  params.max_offset = 0.6;
  const auto result = mgg::rebuildRoadmapFromTrajectory(
      graph, {{scene.hover}, {scene.hover + mgg::StateVec(1, 0, 0, 0)}},
      scene.ctx, params);
  EXPECT_EQ(graph.getNumVertices(), 1);
  EXPECT_EQ(result.offset_vertices, 0);
  EXPECT_EQ(result.boxed_vertices, 1);
}

TEST(MolaMap, AerialStraightDepartureSkipsOnlyUnknownInsideOriginalRoot) {
  for (int obstacle = 0; obstacle < 3; ++obstacle) {
    SCOPED_TRACE(obstacle);
    AerialMolaScene scene;
    auto free = AerialMolaScene::observedRoom();
    // Unknown inside the root is allowed; occupied there never is. Unknown
    // beside the root must not become a progressively re-rooted exception.
    free.erase(std::remove_if(free.begin(), free.end(), [&](const Voxel& v) {
      return v.x == (obstacle == 2 ? 2 : 0) && v.y == 0 && v.z == 7;
    }), free.end());
    scene.load(free, obstacle == 1 ? std::vector<Voxel>{{0, 0, 7}}
                                  : std::vector<Voxel>{});
    mgg::GroundProjection ground(scene.map, scene.planning);
    mgg::Departure departure;
    scene.planning.departure_reverse_allowed = false;
    const bool found = mgg::findDeparture(
        scene.map, ground, scene.robot, scene.planning, scene.hover, departure,
        [](const auto&) { return true; }, 1.0, true);
    EXPECT_EQ(found, obstacle == 0);
    if (found) {
      ASSERT_GE(departure.path.size(), 2u);
      EXPECT_EQ(scene.map.getStrictBoxStatus(departure.path.back().head<3>(),
                                             scene.robot.size), VoxelStatus::kFree);
    }
  }
}

TEST(MolaMap, AerialDrone45NearFieldCarvePlansLocally) {
  AerialMolaScene scene;
  const mgg::StateVec hover(0.1, 0.1, 1.3, 0);
  const Eigen::Vector3d sensor = hover.head<3>() + Eigen::Vector3d(0, 0, -0.08);
  std::set<std::tuple<int, int, int>> occupied_cells, free_cells;
  const auto cell = [](const Eigen::Vector3d& p) {
    return std::make_tuple(int(std::floor(p.x() / 0.2)),
                           int(std::floor(p.y() / 0.2)),
                           int(std::floor(p.z() / 0.2)));
  };
  // drone_45: 17 rings over +/-45 degrees, in a room with walls at
  // x/y=+/-3, floor z=0 and ceiling z=3. A valid return is >=0.45 m away.
  // Like planner_map.cpp, sample from the sensor towards the return at
  // resolution*0.75 spacing, not from a sphere at the minimum return range.
  for (int az = 0; az < 72; ++az) {
    const double yaw = 2 * M_PI * az / 72;
    for (int ring = 0; ring < 17; ++ring) {
      const double pitch = -M_PI / 4 + ring * (M_PI / 2) / 16;
      const Eigen::Vector3d direction(std::cos(yaw) * std::cos(pitch),
                                       std::sin(yaw) * std::cos(pitch),
                                       std::sin(pitch));
      double distance = 100;
      for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(direction[axis]) < 1e-9) continue;
        const double wall = direction[axis] > 0 ? 3.0 : (axis == 2 ? 0.0 : -3.0);
        distance = std::min(distance, (wall - sensor[axis]) / direction[axis]);
      }
      ASSERT_GE(distance, 0.45);
      occupied_cells.insert(cell(sensor + distance * direction));
      const int steps = int(std::ceil(distance / (0.2 * 0.75)));
      for (int step = 1; step < steps; ++step)
        free_cells.insert(cell(sensor + distance * step / steps * direction));
    }
  }
  std::vector<Voxel> free, occupied;
  for (const auto& [x, y, z] : free_cells) free.push_back({x, y, z});
  for (const auto& [x, y, z] : occupied_cells) occupied.push_back({x, y, z});
  scene.load(free, occupied);
  mgg::GraphManager graph;
  graph.addVertex(new mgg::Vertex(0, hover));
  mgg::GridGraphParams grid;
  grid.min_val = {-1.2, -1.2, 0};
  grid.max_val = {1.2, 1.2, 0};
  grid.resolution = {0.4, 0.4, 0.2};
  mgg::buildGridGraph(graph, hover, grid, scene.ctx, 0);
  ASSERT_GT(graph.getNumVertices(), 2);
  mgg::SensorParams lidar;
  lidar.max_range = 4;
  lidar.fov = {2 * M_PI, M_PI / 2};
  lidar.resolution = {M_PI / 36, M_PI / 32};
  lidar.update();
  std::unordered_map<std::string, mgg::SensorParams> sensors{{"drone_45", lidar}};
  scene.planning.exp_sensor_list = {"drone_45"};
  scene.planning.unknown_voxel_gain = 1;
  mgg::BoundedSpaceParams bounds;
  bounds.min_val = {-4, -4, 0}; bounds.max_val = {4, 4, 3};
  bounds.setCenter(Eigen::Vector3d(0, 0, 0), false);
  mgg::GainContext gain;
  gain.map = &scene.map; gain.robot = &scene.robot;
  gain.planning = &scene.planning; gain.global_space = &bounds; gain.sensors = &sensors;
  mgg::computeExplorationGain(graph, gain, false, false);
  const auto chosen = mgg::selectBestPath(graph, scene.planning, scene.robot,
                                         mgg::EdgeInclinations(), 0.2, 0);
  ASSERT_GE(chosen.best_path.size(), 2u);
  EXPECT_GT((chosen.best_path.back()->state - hover).head<2>().norm(), 0.5);
  // The same blind near-field hover must also depart into the roadmap.
  mgg::GraphManager roadmap;
  auto* anchor = new mgg::Vertex(0, mgg::StateVec(1.1, 0.1, 1.3, 0));
  roadmap.addVertex(anchor);
  const auto link = mgg::linkDeparture(roadmap, hover, scene.ctx, 1.5);
  EXPECT_EQ(link.vertex, anchor);
  EXPECT_TRUE(link.query_local);
  EXPECT_EQ(roadmap.getNumVertices(), 1);
  EXPECT_EQ(roadmap.getNumEdges(), 0);
}

TEST(MolaMap, ExplorationInACorridorNarrowerThanTheClearanceStillHasAPath) {
  // A corridor 0.6 m wide along x: walls over y=[-0.4,-0.2] and [0.4,0.6].
  std::vector<Voxel> walls;
  std::vector<Voxel> corridor;
  // Include the closed rear face of the root body at x=0. This test is
  // about viewpoint clearance, not an unobserved aerial departure.
  for (std::int64_t x = -1; x <= 10; ++x) {
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

  // Circumscribed radius 0.354 m, with no aerial margin: wider than the
  // corridor's 0.3 m half-width, so no vertex along it has viewpoint
  // clearance.
  mgg::RobotParams robot;
  robot.type = mgg::RobotType::kAerialRobot;
  robot.size = Eigen::Vector3d(0.5, 0.5, 0.2);
  mgg::PlanningParams planning;
  planning.aerial_viewpoint_clearance_margin = 0.0;
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
  // Aerial blind-root departures allow unknown, never ground-specific
  // measured-surface relief of an occupied voxel.
  EXPECT_EQ(provider.getOccupiedOnlyPathStatus(body, body, {0.2, 0.2, 0.295}),
            VoxelStatus::kOccupied);
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

// Review r1 (P2) turned this up: a 45 degree ray ending on a voxel
// boundary, whose last y crossing rounds within the tie tolerance of an x
// crossing. The walk stepped y past the target cell with x and ran on to
// its work limit, 4 million voxels, and the query came back unknown.
TEST(NativeMolaGrid, RayEndingOnABoundaryAtATieEndsAtItsTarget) {
  using Grid = mgg::NativeMolaGrid;
  Grid map(0.2, {}, {}, {});
  const Eigen::Vector3d a(0.1, 0.1, 0.1);
  const Eigen::Vector3d b =
      a + Eigen::Vector3d(1.5000000000000002, -1.5, -2.1213203435596424);
  Eigen::Vector3d end;
  EXPECT_EQ(map.getRayStatus(a, b, false, end), VoxelStatus::kFree);
  EXPECT_TRUE(end.isApprox(b));
  // The walk ends at the target cell, and a return in it stops the ray.
  Grid wall(0.2, {{8, -7, -11}}, {}, {});
  EXPECT_EQ(wall.getRayStatus(a, b, false, end), VoxelStatus::kOccupied);
}

// Review r2 (P1): ending the walk at its target (713a4ab) stopped an axis
// already at its target from stepping with another whose crossing tied at
// the end, and the end cleanup added only voxels below the target along
// negative axes. From (0.1, 0.1, 0.1) to (0.2, 0.0, 0.1), the voxel
// (0, -1, 0) touching the end was missed, so a return in it went unseen.
TEST(NativeMolaGrid, RayEndingOnAnEdgeSeesTheVoxelDiagonallyAcrossIt) {
  using Grid = mgg::NativeMolaGrid;
  Grid map(0.2, {{0, -1, 0}}, {}, {});
  const Eigen::Vector3d a(0.1, 0.1, 0.1);
  const Eigen::Vector3d b(0.2, 0.0, 0.1);
  EXPECT_EQ(map.getRayStatus(a, b, false), VoxelStatus::kOccupied);
  EXPECT_EQ(map.getRayStatus(b, a, false), VoxelStatus::kOccupied);
}

// Every face, edge and corner of a voxel, in every mix of directions: a
// segment from the voxel's centre to it touches, and visits once each, the
// voxel and those across the face, edge or corner. A return in any one of
// them is seen, walking out or in.
TEST(NativeMolaGrid, RaysToEveryFaceEdgeAndCornerSeeEveryVoxelTouchingIt) {
  using Grid = mgg::NativeMolaGrid;
  const Eigen::Vector3d centre(0.1, 0.1, 0.1);
  for (int sx = -1; sx <= 1; ++sx)
    for (int sy = -1; sy <= 1; ++sy)
      for (int sz = -1; sz <= 1; ++sz) {
        if (sx == 0 && sy == 0 && sz == 0) continue;
        const Eigen::Vector3d end = centre + 0.1 * Eigen::Vector3d(sx, sy, sz);
        std::vector<std::array<std::int64_t, 3>> touching;
        for (int x : {0, sx})
          for (int y : {0, sy})
            for (int z : {0, sz}) {
              const std::array<std::int64_t, 3> cell{x, y, z};
              if (std::find(touching.begin(), touching.end(), cell) ==
                  touching.end())
                touching.push_back(cell);
            }
        const std::string where = std::to_string(sx) + "," +
                                  std::to_string(sy) + "," +
                                  std::to_string(sz);
        for (const auto& [from, to] :
             {std::pair{centre, end}, std::pair{end, centre}}) {
          std::vector<std::array<std::int64_t, 3>> visited;
          EXPECT_TRUE(mgg::walkVoxels(
              from, to, 0.2, 1u << 22, [&](const mgg::VoxelIndex& v) {
                visited.push_back({v.x, v.y, v.z});
                return true;
              }))
              << where;
          std::sort(visited.begin(), visited.end());
          std::sort(touching.begin(), touching.end());
          EXPECT_EQ(visited, touching) << where;
        }
        for (const auto& cell : touching) {
          Grid map(0.2, {{cell[0], cell[1], cell[2]}}, {}, {});
          EXPECT_EQ(map.getRayStatus(centre, end, false),
                    VoxelStatus::kOccupied)
              << where << " out, return in " << cell[0] << "," << cell[1]
              << "," << cell[2];
          EXPECT_EQ(map.getRayStatus(end, centre, false),
                    VoxelStatus::kOccupied)
              << where << " in, return in " << cell[0] << "," << cell[1]
              << "," << cell[2];
        }
      }
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

// The run-9 freshness measurement: one call per new grid installed, with
// the time since its authority heartbeat and the product's newest keyframe
// stamp; none for a heartbeat that only confirms the grid in service.
TEST(MolaMap, EachNewGridInstalledIsReportedOnce) {
  Publication publication;
  std::mutex mutex;
  std::vector<mgg::MolaInstallation> installs;
  MolaMapConfig cfg = config(publication);
  cfg.on_install = [&](const mgg::MolaInstallation& installed) {
    std::lock_guard<std::mutex> lock(mutex);
    installs.push_back(installed);
  };
  const auto count = [&]() {
    std::lock_guard<std::mutex> lock(mutex);
    return installs.size();
  };
  MolaMap provider(cfg);
  const auto first = publication.publish(0, {{5, 0, 0}}, freeBlock());
  ASSERT_GT(first.source_stamp_ns, 0u);
  provider.requestSnapshot(first);
  ASSERT_TRUE(waitFor([&]() { return count() == 1; })) << provider.lastError();
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(installs[0].source_stamp_ns, first.source_stamp_ns);
    EXPECT_EQ(installs[0].graph_revision, first.graph_revision);
    EXPECT_GE(installs[0].authority_to_install_s, 0.0);
    EXPECT_LT(installs[0].authority_to_install_s, 5.0);
  }
  // The same heartbeat again is confirmed by stat: nothing installed.
  provider.requestSnapshot(first);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(count(), 1u);
  // A successor is.
  const auto second = publication.publish(1, {{8, 0, 0}}, freeBlock());
  provider.requestSnapshot(second);
  ASSERT_TRUE(waitFor([&]() { return count() == 2; })) << provider.lastError();
  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(installs[1].graph_revision, second.graph_revision);
  EXPECT_EQ(installs[1].source_stamp_ns, second.source_stamp_ns);
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
  // One line of measured floor under a 0.78 m wide body: what the
  // unobserved-ground check (tested on its own) refuses.
  planning.min_observed_ground_fraction = 0.0;
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


TEST(NativeMolaGrid, AStraightDepartureAlongACorridorChecksTheBodyTurnedToItsHeading) {
  // mgg::findDeparture on the native MOLA grid, as mgg_ros PlannerNode
  // departs a boxed-in ground robot: 0.1 m steps checked with
  // getProjectedEdgeStatus, stopping at unknown space, with the robot's
  // rectangle turned to its heading, up to the first pose where
  // mgg::turnClear finds room to turn in place. The robot is 1.2 m long and
  // 0.4 m wide, facing +y in a corridor along y whose inside spans
  // x = [-0.2, 0.4]: it can drive along the corridor, but has no room to
  // turn in it. The corridor's walls end at y = 1.2.
  using Grid = mgg::NativeMolaGrid;
  std::vector<Grid::Cell> occupied;
  std::vector<Grid::Cell> free;
  std::vector<Grid::Surface> surfaces;
  for (std::int64_t x = -10; x <= 10; ++x) {
    for (std::int64_t y = -10; y <= 15; ++y) {
      occupied.push_back({x, y, -1});  // floor, its top at z = 0
      surfaces.push_back({{x, y, -1}, 0.0});
      const bool wall = (x == -2 || x == 2) && y <= 5;
      for (std::int64_t z = 0; z <= 2; ++z) {
        if (wall) {
          occupied.push_back({x, y, z});
          surfaces.push_back({{x, y, z}, (z + 1) * 0.2});
        } else {
          free.push_back({x, y, z});
        }
      }
    }
  }
  const Grid map(0.2, occupied, free, surfaces);

  mgg::RobotParams robot;
  robot.type = mgg::RobotType::kGroundRobot;
  robot.size = Eigen::Vector3d(1.2, 0.4, 0.15);
  robot.size_extension.setZero();
  robot.size_extension_min.setZero();
  robot.safety_extension.setZero();
  robot.bound_mode = mgg::BoundModeType::kExactBound;
  mgg::PlanningParams planning;
  planning.max_ground_height = 0.3;
  planning.max_step_height = 0.1;
  planning.max_inclination = 0.52;
  planning.path_interpolation_distance = 0.1;
  const mgg::GroundProjection ground(map, planning);

  // At driving height over the floor, facing +y.
  Eigen::Vector3d probe(0.1, 0.1, 0.3);
  VoxelStatus found = VoxelStatus::kUnknown;
  const double below = ground.projectSample(probe, found);
  ASSERT_EQ(found, VoxelStatus::kOccupied);
  const double driving_z = 0.3 - below + planning.max_ground_height;
  const mgg::StateVec start(0.1, 0.1, driving_z, M_PI / 2.0);
  ASSERT_FALSE(mgg::turnClear(map, robot, start));

  // Turned to face y the body is 0.4 m across the corridor: it drives out,
  // to room to turn past the walls' end. The turning circle, 0.632 m in
  // radius, clears the walls' corners 0.3 m to either side 0.556 m past
  // their end at y = 1.2: the first step there is y = 1.8, 1.7 m out.
  mgg::Departure departure;
  ASSERT_TRUE(
      mgg::findDeparture(map, ground, robot, planning, start, departure));
  EXPECT_FALSE(departure.reverse);
  EXPECT_EQ(departure.turn, 0.0);
  EXPECT_NEAR(departure.path.back().y() - start.y(), 1.7, 1e-9);
  EXPECT_NEAR(departure.path.back().x(), start.x(), 1e-9);
  // The box aligned with the map is its 1.2 m length across the corridor,
  // and meets the walls where the robot stands.
  EXPECT_EQ(map.getPathStatus(start.head<3>(),
                              start.head<3>() + Eigen::Vector3d(0, 0.1, 0),
                              robot.getPlanningSize(), true),
            VoxelStatus::kOccupied);
}

/// A planner grid cut from a SwarmDeck run around a robot that stood boxed
/// in (test/data/README.md), and where the robot stood, on the ground.
struct BoxedInFixture {
  std::unique_ptr<mgg::NativeMolaGrid> map;
  Eigen::Vector3d robot = Eigen::Vector3d::Zero();

  explicit BoxedInFixture(const std::string& name) {
    std::ifstream in(std::string(MGG_MAP_TEST_DATA_DIR) + "/" + name);
    EXPECT_TRUE(in.good()) << name;
    double resolution = 0.0;
    std::vector<mgg::NativeMolaGrid::Cell> occupied;
    std::vector<mgg::NativeMolaGrid::Cell> free;
    std::vector<mgg::NativeMolaGrid::Surface> surfaces;
    std::string line;
    while (std::getline(in, line)) {
      std::istringstream row(line);
      std::string tag;
      row >> tag;
      if (tag == "resolution") {
        row >> resolution;
      } else if (tag == "robot") {
        row >> robot.x() >> robot.y() >> robot.z();
      } else if (tag == "o" || tag == "f") {
        mgg::NativeMolaGrid::Cell cell;
        row >> cell.x >> cell.y >> cell.z;
        (tag == "o" ? occupied : free).push_back(cell);
        double top = 0.0;
        if (tag == "o" && row >> top) surfaces.push_back({cell, top});
      }
    }
    map = std::make_unique<mgg::NativeMolaGrid>(resolution, occupied, free,
                                                surfaces);
  }
};

/// A Bunker as SwarmDeck deploys MGG for it (deploy/mgg/fleet.launch.py,
/// run 5): 1.023 x 0.778 m, 0.4 m tall, 0.05 m of size extension.
mgg::RobotParams bunker() {
  mgg::RobotParams robot;
  robot.type = mgg::RobotType::kGroundRobot;
  robot.size = Eigen::Vector3d(1.023, 0.778, 0.4);
  robot.size_extension = Eigen::Vector3d::Constant(0.05);
  robot.bound_mode = mgg::BoundModeType::kExtendedBound;
  return robot;
}

mgg::PlanningParams bunkerPlanning() {
  mgg::PlanningParams planning;
  planning.max_ground_height = 0.2 + 0.15 + 0.175;
  planning.max_step_height = 0.15;
  planning.max_inclination = 27.0 * M_PI / 180.0;
  planning.max_cross_slope = 18.0 * M_PI / 180.0;
  planning.max_footprint_tilt = 22.0 * M_PI / 180.0;
  planning.max_footprint_step = 0.12;
  planning.path_interpolation_distance = 0.25;  // bistro.yaml
  return planning;
}

/// Where the robot of `fixture` stood, facing `yaw_deg`, at driving height.
mgg::StateVec boxedInStart(const BoxedInFixture& fixture,
                           const mgg::PlanningParams& planning,
                           double yaw_deg) {
  return mgg::StateVec(fixture.robot.x(), fixture.robot.y(),
                       fixture.robot.z() + planning.max_ground_height,
                       yaw_deg * M_PI / 180.0);
}

TEST(Departure, Run5Robot0TurnsAFewDegreesThenBacksOutOfTheCorner) {
  // Run 5, robot_0 (Bunker) at 1790345307: parked facing about 17 degrees
  // in a room's corner, a wall 0.84 m ahead, a wall notch 0.57 m to its
  // right and the south wall's foot about 0.9 m behind its right side. It
  // had no room to turn, and every departure was refused at the first step:
  // the box aligned with the map that holds the turned robot, 1.27 x 1.11 m,
  // met the notch where the robot stood. Its own rectangle does not. Still,
  // straight at 17 degrees there is no way out: ahead meets the east wall;
  // back heads for the south wall, whose foot comes within the turning
  // circle from 0.3 m out and meets the body's corner at 0.6 m. Turned 10
  // degrees clockwise in place, which the notch leaves room for, it backs
  // out straight west to room to turn 0.5 m out.
  const BoxedInFixture fixture("boxed_in_r0.txt");
  const mgg::RobotParams robot = bunker();
  const mgg::PlanningParams planning = bunkerPlanning();
  const mgg::GroundProjection ground(*fixture.map, planning);
  for (const double yaw : {15.5, 17.0, 20.0}) {
    SCOPED_TRACE(yaw);
    const mgg::StateVec start = boxedInStart(fixture, planning, yaw);
    ASSERT_FALSE(mgg::turnClear(*fixture.map, robot, start));
    mgg::Departure departure;
    ASSERT_TRUE(mgg::findDeparture(*fixture.map, ground, robot, planning,
                                   start, departure));
    // Not straight at its heading, which findDeparture tries first.
    EXPECT_NE(departure.turn, 0.0);
    EXPECT_LE(std::abs(departure.turn), mgg::kDepartureMaxTurnRad + 1e-9);
    EXPECT_TRUE(departure.reverse);
    const double out = (departure.path.back().head<2>() -
                        departure.path.front().head<2>())
                           .norm();
    EXPECT_GE(out, mgg::kDepartureMinM - 1e-9);
    EXPECT_LE(out, 1.0);
    EXPECT_TRUE(departure.path.front().head<2>().isApprox(start.head<2>()));
    EXPECT_TRUE(mgg::turnClear(*fixture.map, robot, departure.path.back()));
    std::printf("robot_0 at %.1f deg: turns %+.0f deg, backs out %.2f m\n",
                yaw, departure.turn * 180.0 / M_PI, out);
  }
}

TEST(Departure, Run5Robot1DepartsFromBesideTheWallColumn) {
  // Run 5, robot_1 (Bunker) from 1790345621: boxed in 43 times beside a
  // wall column 0.49 m from its centre, turning in place between 113 and
  // 174 degrees. That column lies inside its own planning rectangle, so
  // every check of the body where it stood failed. Those cells are not
  // checked where it stands, and at each heading it drives straight out.
  const BoxedInFixture fixture("boxed_in_r1.txt");
  const mgg::RobotParams robot = bunker();
  const mgg::PlanningParams planning = bunkerPlanning();
  const mgg::GroundProjection ground(*fixture.map, planning);
  for (const double yaw : {90.0, 113.0, 133.0, 174.0, 180.0}) {
    SCOPED_TRACE(yaw);
    const mgg::StateVec start = boxedInStart(fixture, planning, yaw);
    ASSERT_FALSE(mgg::turnClear(*fixture.map, robot, start));
    mgg::Departure departure;
    EXPECT_TRUE(mgg::findDeparture(*fixture.map, ground, robot, planning,
                                   start, departure));
    if (departure.path.empty()) continue;
    EXPECT_EQ(departure.turn, 0.0);
    const double out = (departure.path.back().head<2>() -
                        departure.path.front().head<2>())
                           .norm();
    EXPECT_GE(out, mgg::kDepartureMinM - 1e-9);
    std::printf("robot_1 at %.0f deg: turns %+.0f deg, %.2f m %s\n", yaw,
                departure.turn * 180.0 / M_PI, out,
                departure.reverse ? "back" : "ahead");
  }
}

TEST(ObservedGround, Run5Robot0IsNotParkedAtTheLedge) {
  // Run 5, robot_0 (Bunker) at 1790349295: at the end of a passage 1.2 m
  // wide it stood with its centre 0.1 m from a ledge, facing south, its
  // front half over a 4 m pit, and fell in. The pit floor lies below the
  // fixture's cut, as it lay unobserved before the fall. Of the cells under
  // the leading half of its footprint only 1 in 9 has observed ground: the
  // pose is refused, as is turning there. 0.4 m back it stood on observed
  // ground ahead.
  const BoxedInFixture fixture("ledge_r0.txt");
  const mgg::RobotParams robot = bunker();
  mgg::PlanningParams planning = bunkerPlanning();
  planning.max_footprint_tilt = 0.0;  // the passage's walls read as steps
  planning.max_footprint_step = 0.0;
  const mgg::GroundProjection ground(*fixture.map, planning);
  const auto at = [&](double x, double y) {
    Eigen::Vector3d p(x, y, fixture.robot.z() + planning.max_ground_height);
    VoxelStatus found = VoxelStatus::kUnknown;
    const double below = ground.projectSample(p, found);
    EXPECT_EQ(found, VoxelStatus::kOccupied);
    p.z() -= below - planning.max_ground_height;
    p.x() = x;
    p.y() = y;
    return p;
  };
  const Eigen::Vector2d south(0.0, -1.0);
  const Eigen::Vector3d ledge = at(137.83, -79.52);
  const Eigen::Vector3d back = at(137.80, -79.10);
  const double at_ledge =
      ground.observedGroundAhead(ledge, south, robot.getPlanningSize());
  const double at_back =
      ground.observedGroundAhead(back, south, robot.getPlanningSize());
  std::printf("robot_0 ledge: observed ground ahead %.2f at the ledge, %.2f "
              "0.4 m back\n", at_ledge, at_back);
  EXPECT_LT(at_ledge, planning.min_observed_ground_fraction);
  EXPECT_GE(at_back, planning.min_observed_ground_fraction);
  std::vector<Eigen::Vector3d> path;
  EXPECT_EQ(ground.getProjectedEdgeStatus(back, ledge,
                                          robot.getPlanningSize(), false,
                                          path, false),
            mgg::ProjectedEdgeStatus::kGroundUnobserved);
  EXPECT_FALSE(mgg::roomToTurn(
      *fixture.map, robot, planning,
      mgg::StateVec(ledge.x(), ledge.y(), ledge.z(), -M_PI / 2.0)));
  // robot_0 had driven 130 m from its start: its planner sets no standing
  // start there, restarted or not, whose disk would count the pit as
  // observed ground (PlannerNodeTest.
  // APlannerRestartedBesideAnUnobservedDropIsNotAtItsStart).
  // Without the check, nothing else refuses the edge onto the ledge.
  planning.min_observed_ground_fraction = 0.0;
  EXPECT_EQ(ground.getProjectedEdgeStatus(back, ledge,
                                          robot.getPlanningSize(), false,
                                          path, false),
            mgg::ProjectedEdgeStatus::kAdmissible);
}

TEST(ObservedGround, Run6StandingStartsHaveRoomToTurn) {
  // Run 6: robot_3 (Spot) was boxed in where it was placed for the whole
  // run, and robot_1 (Bunker), 2 m east of it, for 18 minutes. robot_3's
  // grid holds no ground within about 2.4 m of it, 4.8 m east behind
  // robot_1's body: every turning circle there was unobserved, so neither
  // had room to turn. robot_1's own grid was not kept; robot_3's leaves
  // robot_1's surroundings unobserved too, as robot_1's did. Standing at
  // their starts, the disk of their initial ground reach
  // (hanging_root_edge_length_max: 2.5 m for a Spot, 2.0 m for a Bunker)
  // counts as observed ground, and both have room to turn. Beyond it the
  // unobserved ground stays unobserved.
  const BoxedInFixture fixture("standing_r3.txt");
  mgg::RobotParams spot = bunker();
  spot.size = Eigen::Vector3d(1.1, 0.5, 1.0);
  mgg::PlanningParams spot_planning = bunkerPlanning();
  spot_planning.max_ground_height = 0.5 + 0.3 + 0.175;
  spot_planning.max_step_height = 0.3;
  const mgg::PlanningParams bunker_planning = bunkerPlanning();
  struct Standing {
    const char* name;
    mgg::RobotParams robot;
    const mgg::PlanningParams* planning;
    Eigen::Vector2d at;
    double reach;
  };
  for (const Standing& start :
       {Standing{"robot_3", spot, &spot_planning, {-2.02, -2.00}, 2.5},
        Standing{"robot_1", bunker(), &bunker_planning, {-0.01, -2.00}, 2.0}}) {
    SCOPED_TRACE(start.name);
    const mgg::PlanningParams& planning = *start.planning;
    const mgg::StateVec pose(start.at.x(), start.at.y(),
                             fixture.robot.z() + planning.max_ground_height,
                             0.0);
    EXPECT_TRUE(mgg::turnClear(*fixture.map, start.robot, pose));
    EXPECT_FALSE(mgg::roomToTurn(*fixture.map, start.robot, planning, pose));
    const mgg::StandingStart standing{start.at, start.reach};
    EXPECT_TRUE(mgg::roomToTurn(*fixture.map, start.robot, planning, pose,
                                &standing));
  }
  // 3 m east of robot_3, behind robot_1, out of robot_3's disk.
  const mgg::StandingStart robot_3{Eigen::Vector2d(-2.02, -2.00), 2.5};
  EXPECT_FALSE(mgg::turnSpaceObserved(
      *fixture.map, spot, spot_planning,
      mgg::StateVec(0.98, -2.00,
                    fixture.robot.z() + spot_planning.max_ground_height, 0.0),
      &robot_3));
}

TEST(ObservedGround, Run5Robot1HasNoRoomToTurnBesideAnUnobservedWall) {
  // Run 5, robot_1 (Bunker) at 1790348990.57 was sent a path starting with
  // a 170 degree turn at (76.60, -43.76), and rode onto a wall's foot. The
  // wall's cells within its turning circle, 0.43 to 0.62 m from its centre,
  // were not yet observed, and unknown space passes turnClear. The fixture
  // leaves them unknown: the turning circle holds unobserved columns, so
  // there is no room to turn.
  const BoxedInFixture fixture("turn_r1.txt");
  const mgg::RobotParams robot = bunker();
  const mgg::PlanningParams planning = bunkerPlanning();
  const mgg::StateVec root = boxedInStart(fixture, planning, 79.0);
  EXPECT_TRUE(mgg::turnClear(*fixture.map, robot, root));
  EXPECT_FALSE(mgg::turnSpaceObserved(*fixture.map, robot, planning, root));
  EXPECT_FALSE(mgg::roomToTurn(*fixture.map, robot, planning, root));
  // 0.6 m east, clear of the wall, the robot may turn.
  mgg::StateVec east = root;
  east.x() += 0.6;
  EXPECT_TRUE(mgg::roomToTurn(*fixture.map, robot, planning, east));
}

/// What sparseRampCrest cuts into the floor behind its crest.
enum class Trench {
  kNone,
  /// The floor at x = [-2.0, -1.6) is cut down to z = -1.0, its bottom
  /// unobserved and its inside carved free, as a lidar looking into it
  /// leaves it.
  kSeenInto,
  /// The same cells unobserved but for one free voxel, 1.4 to 1.6 m down
  /// unless sparseRampCrest says otherwise: a lidar looked into it only
  /// from far off, through the trench's width, and carved only its depth.
  kSeenDeepInto,
  /// The same cells unobserved, with nothing carved in them.
  kUnseen,
};

/// A crest at x = 0 as a Bunker's planner grid records it from the level
/// floor behind (diag-ramp, run 7): the floor, at z = 0 for x < 0, is
/// observed everywhere, with free air over it; past the crest a 16 degree
/// ramp falls 6 m along +x to a level bottom. The lidar meets the ramp in
/// rings across it: its ground is observed on every other row of cells
/// across the ramp, and between them on one cell in three, two thirds of
/// the cells, with free air carved over those cells only. The rest is
/// unknown. (Ground projection finds each unobserved cell's ground 0.4 m
/// to its side, at its height.) 3 m either side of y = 0. `trench` cuts
/// the floor behind the crest. From `wall_from_x`, a wall 0.4 m thick
/// crosses the ramp, observed; up to `air_seen_to_x`, the air over the
/// ramp's unobserved cells is observed free, their ground still not.
/// `seen_deep_iz` is the row of Trench::kSeenDeepInto's free voxel.
std::unique_ptr<mgg::NativeMolaGrid> sparseRampCrest(
    Trench trench = Trench::kNone,
    double wall_from_x = std::numeric_limits<double>::infinity(),
    double air_seen_to_x = -std::numeric_limits<double>::infinity(),
    std::int64_t seen_deep_iz = -8) {
  constexpr double kResolution = 0.2;
  const double grade = std::tan(16.0 * M_PI / 180.0);
  std::vector<mgg::NativeMolaGrid::Cell> occupied;
  std::vector<mgg::NativeMolaGrid::Cell> free;
  std::vector<mgg::NativeMolaGrid::Surface> surfaces;
  const auto cell = [&](double z) {
    return static_cast<std::int64_t>(std::floor(z / kResolution));
  };
  for (std::int64_t ix = -20; ix < 40; ++ix) {
    const double x = (ix + 0.5) * kResolution;
    const double ground = -std::clamp(x, 0.0, 6.0) * grade;
    const bool in_trench = trench != Trench::kNone && (ix == -10 || ix == -9);
    for (std::int64_t iy = -15; iy < 15; ++iy) {
      const bool observed =
          ix < 0 || ix % 2 == 0 || (iy % 3 + 3) % 3 == 2;
      if (in_trench) {
        for (std::int64_t iz = cell(-0.9);
             trench == Trench::kSeenInto && iz < cell(2.0); ++iz) {
          free.push_back({ix, iy, iz});
        }
        if (trench == Trench::kSeenDeepInto) {
          free.push_back({ix, iy, seen_deep_iz});
        }
        continue;
      }
      if (x >= wall_from_x && x < wall_from_x + 0.4) {
        for (std::int64_t iz = cell(ground - 1e-6); iz < cell(ground + 2.0);
             ++iz) {
          occupied.push_back({ix, iy, iz});
        }
        continue;
      }
      if (!observed) {
        for (std::int64_t iz = cell(ground - 1e-6) + 1;
             x <= air_seen_to_x && iz < cell(ground + 2.0); ++iz) {
          free.push_back({ix, iy, iz});
        }
        continue;
      }
      const mgg::NativeMolaGrid::Cell top{ix, iy, cell(ground - 1e-6)};
      occupied.push_back(top);
      surfaces.push_back({top, ground});
      for (std::int64_t iz = top.z + 1; iz < cell(ground + 2.0); ++iz) {
        free.push_back({ix, iy, iz});
      }
    }
  }
  return std::make_unique<mgg::NativeMolaGrid>(kResolution, occupied, free,
                                               surfaces);
}

/// The planner node's local plan for a Bunker on `map` from `start`, at
/// driving height, facing `yaw`, as buildLocalGraph makes it: the lattice,
/// PathTurnCheck, viewpointClear with the slope the lattice measures and
/// SlopeEndRetreat. Each vertex scores by how far it lies below
/// `crest_height` (a driving height). The lattice reaches `lattice_behind`
/// and `lattice_ahead` along x from `start`; without `room_anywhere` the
/// robot has room to turn nowhere.
struct SparseCrestPlan {
  mgg::PlanningParams planning;
  mgg::GraphManager graph;
  mgg::EdgeInclinations inclinations;
  std::unique_ptr<mgg::PathTurnCheck> turns;
  mgg::PathSelectionResult selection;
  /// Vertices more than 0.25 m below the crest.
  int down_the_ramp = 0;
};

std::unique_ptr<SparseCrestPlan> planOnSparseCrest(
    const mgg::NativeMolaGrid& map, const mgg::RobotParams& robot,
    const mgg::PlanningParams& planning, const Eigen::Vector3d& start,
    double yaw, double crest_height, double lattice_behind = 3.0,
    double lattice_ahead = 6.0, bool room_anywhere = true) {
  auto plan = std::make_unique<SparseCrestPlan>();
  mgg::PlanningParams& q = plan->planning;
  q = planning;
  q.edge_length_min = 0.05;
  q.edge_length_max = 2.0;
  q.nearest_range = 0.6;
  q.nearest_range_z = 0.15;
  q.nearest_range_min = 0.05;
  q.nearest_range_max = 1.0;
  q.num_vertices_max = 1500;
  q.num_edges_max = 50000;
  const mgg::GroundProjection ground(map, q, true);
  plan->graph.addVertex(
      new mgg::Vertex(0, mgg::StateVec(start.x(), start.y(), start.z(), yaw)));
  mgg::ExpandContext context;
  context.map = &map;
  context.planning = &q;
  context.robot = &robot;
  context.ground = &ground;
  context.inclinations = &plan->inclinations;
  context.robot_box_size = robot.getPlanningSize();
  context.allow_unknown_lattice_body = true;
  context.root_footprint_exempt = true;
  context.root_is_robot = true;
  mgg::GridGraphParams grid;
  grid.min_val = {-lattice_behind, -3.0, 0.0};
  grid.max_val = {lattice_ahead, 3.0, 0.0};
  grid.resolution = {0.4, 0.4, 0.1};
  mgg::buildGridGraph(plan->graph, plan->graph.getVertex(0)->state, grid,
                      context, yaw);
  for (auto& [id, vertex] : plan->graph.vertices_map_) {
    if (vertex == nullptr) continue;
    const double drop = crest_height - vertex->state.z();
    vertex->vol_gain.gain = 1.0 + 1000.0 * std::max(0.0, drop);
    if (drop > 0.25) ++plan->down_the_ramp;
  }
  const mgg::MapInterface& m = map;
  plan->turns = std::make_unique<mgg::PathTurnCheck>(
      plan->graph, robot,
      [&m, &robot, &q, room_anywhere](const mgg::StateVec& pose) {
        return room_anywhere && mgg::roomToTurn(m, robot, q, pose);
      });
  mgg::PathTurnCheck& turns = *plan->turns;
  mgg::SlopeEndRetreat retreat;
  retreat.admitted_on_slope = [&](const mgg::Vertex& v) {
    return mgg::slopeExemptsTurnSpace(turns.slopeAt(v.state.head<3>())) &&
           !mgg::turnSpaceObserved(map, robot, q, v.state);
  };
  retreat.refuge_admissible = [&](const auto& path, std::size_t, std::size_t refuge) {
    const auto& v = *path[refuge];
    return room_anywhere && mgg::roomToTurn(map, robot, q, v.state);
  };
  plan->selection = mgg::selectBestPath(
      plan->graph, q, robot, plan->inclinations, map.getResolution(), yaw, {},
      0.0,
      [&](const mgg::Vertex& v) {
        return mgg::viewpointClear(map, robot, q, v.state,
                                   turns.slopeAt(v.state.head<3>()));
      },
      std::ref(turns),
      [&](const mgg::Vertex& v) {
        return turns.sharpTurnAllowedAt(v.state.head<3>());
      },
      0.3, retreat);
  return plan;
}

/// Path length back from the end of `path` to its last vertex with room
/// to turn; infinity with none.
double wayBackAlong(const mgg::MapInterface& map, const mgg::RobotParams& robot,
                    const mgg::PlanningParams& planning,
                    const std::vector<mgg::Vertex*>& path) {
  double back = 0.0;
  for (std::size_t i = path.size(); i-- > 0;) {
    if (mgg::roomToTurn(map, robot, planning, path[i]->state)) return back;
    if (i > 0) back += (path[i]->state - path[i - 1]->state).head<3>().norm();
  }
  return std::numeric_limits<double>::infinity();
}

TEST(ObservedGround, ADownRampSeenInRingsPastACrestIsDriven) {
  // Run 7: the robots came to a 16 degree ramp's crest and turned back.
  // Past a crest the lidar meets the ramp in rings; under the leading half
  // of a footprint going down, 0.25 to 0.62 of the cells had observed
  // ground, and the edges down were refused (item 7). And the air over the
  // ramp is carved over the rings only: every end on it had wholly unknown
  // columns in its turning circle, and was pulled back to the crest. The
  // cells between the rings lie in gaps of observed ground, which are
  // bridged; an end on a measured slope, where the robot may not turn, does
  // not need its turn space observed.
  const auto map = sparseRampCrest();
  const mgg::RobotParams robot = bunker();
  mgg::PlanningParams planning = bunkerPlanning();
  ASSERT_GT(planning.min_observed_ground_fraction, 0.0);
  const mgg::GroundProjection ground(*map, planning, true);
  const Eigen::Vector3d box = robot.getPlanningSize();
  const double grade = std::tan(16.0 * M_PI / 180.0);
  const auto driving = [&](double x) {
    return Eigen::Vector3d(x, 0.1,
                           -std::clamp(x, 0.0, 6.0) * grade +
                               planning.max_ground_height);
  };
  const Eigen::Vector2d down(1.0, 0.0);
  for (const double x : {0.5, 1.5, 2.5, 3.5}) {
    SCOPED_TRACE(x);
    EXPECT_GE(ground.observedGroundAhead(driving(x), down, box),
              planning.min_observed_ground_fraction);
    EXPECT_GE(ground.observedGroundAhead(driving(x), -down, box),
              planning.min_observed_ground_fraction);
  }
  std::vector<Eigen::Vector3d> edge;
  EXPECT_EQ(ground.getProjectedEdgeStatus(driving(1.0), driving(1.4), box,
                                          false, edge, false),
            mgg::ProjectedEdgeStatus::kAdmissible);

  // 2 m down: the turning circle holds unknown columns. Measured as a
  // slope, the end is clear; unmeasured it is not, nor level.
  const Eigen::Vector3d end = driving(2.1);
  const mgg::StateVec end_state(end.x(), end.y(), end.z(), 0.0);
  EXPECT_FALSE(mgg::turnSpaceObserved(*map, robot, planning, end_state));
  EXPECT_TRUE(mgg::viewpointClear(*map, robot, planning, end_state,
                                  16.0 * M_PI / 180.0));
  EXPECT_FALSE(mgg::viewpointClear(*map, robot, planning, end_state));
  EXPECT_FALSE(mgg::viewpointClear(*map, robot, planning, end_state, 0.0));

  // The node's local plan from the level floor 0.5 m before the crest,
  // facing it, scored by depth: it ends down the ramp, within
  // kDepartureMaxM of room to turn back along its path. The lattice's
  // points lie on cell centres.
  const Eigen::Vector3d start = driving(-0.5);
  const auto plan = planOnSparseCrest(*map, robot, planning, start, 0.0,
                                      start.z());
  const mgg::PathSelectionResult& selection = plan->selection;
  ASSERT_FALSE(selection.best_path.empty());
  const mgg::StateVec& best = selection.best_path.back()->state;
  const double start_floor = start.z() - planning.max_ground_height;
  std::printf("sparse crest: %d vertices down the ramp, the plan ends at "
              "(%.2f, %.2f), %.2f m below the crest; %d slope ends without "
              "a way back\n",
              plan->down_the_ramp, best.x(), best.y(),
              start_floor - (best.z() - planning.max_ground_height),
              selection.slope_ends_without_way_back);
  EXPECT_GT(best.x(), 1.0);
  EXPECT_LT(best.z() - planning.max_ground_height, start_floor - 0.25);
  EXPECT_FALSE(selection.unclear_viewpoint);
  // The end's slope, measured over the lattice, exempts it; the root's,
  // on the level floor, does not.
  EXPECT_TRUE(
      mgg::slopeExemptsTurnSpace(plan->turns->slopeAt(best.head<3>())));
  EXPECT_FALSE(mgg::slopeExemptsTurnSpace(
      plan->turns->slopeAt(selection.best_path.front()->state.head<3>())));
  EXPECT_FALSE(mgg::turnSpaceObserved(*map, robot, planning, best));
  EXPECT_LE(wayBackAlong(*map, robot, planning, selection.best_path),
            mgg::kDepartureMaxM + 1e-9);
  EXPECT_GT(selection.slope_ends_without_way_back, 0);
}

TEST(ObservedGround, ASlopeEndWithNoWayBackIsNoFallback) {
  // Review r1 (P1): with no path ending clear, the best path was chosen as
  // it was, unclear, before the slope-end rule looked at it: a path ending
  // on the ramp with no room to turn within kDepartureMaxM behind was
  // still sent. The robot stands 1.5 m down the sparse ramp, the lattice
  // stays on the ramp, and the robot has room to turn nowhere: every end
  // admitted on the slope lacks a way back, and none is chosen, clear or
  // not.
  const auto map = sparseRampCrest();
  const mgg::RobotParams robot = bunker();
  const mgg::PlanningParams planning = bunkerPlanning();
  const double grade = std::tan(16.0 * M_PI / 180.0);
  const Eigen::Vector3d start(1.5, 0.1,
                              -1.5 * grade + planning.max_ground_height);
  const auto plan = planOnSparseCrest(*map, robot, planning, start, 0.0,
                                      start.z(), 0.0, 3.0, false);
  const mgg::PathSelectionResult& selection = plan->selection;
  const auto admitted_on_slope = [&](const mgg::StateVec& state) {
    return mgg::slopeExemptsTurnSpace(plan->turns->slopeAt(state.head<3>())) &&
           !mgg::turnSpaceObserved(*map, robot, plan->planning, state);
  };
  int on_slope = 0;
  for (const auto& [id, vertex] : plan->graph.vertices_map_) {
    if (vertex != nullptr && id != 0 && admitted_on_slope(vertex->state)) {
      ++on_slope;
    }
  }
  std::printf("no way back: %d vertices admitted on the slope, %d refused "
              "ends, best path %zu vertices%s\n",
              on_slope, selection.slope_ends_without_way_back,
              selection.best_path.size(),
              selection.unclear_viewpoint ? ", unclear" : "");
  ASSERT_GT(on_slope, 10);
  EXPECT_GT(selection.slope_ends_without_way_back, 0);
  if (!selection.best_path.empty()) {
    EXPECT_FALSE(admitted_on_slope(selection.best_path.back()->state));
  }
}

TEST(ObservedGround, ASlopeEndBlockedAheadBacksOutOrWaits) {
  // Review r0 (P1): a path end on the ramp, admitted without its turn space
  // observed, must leave the robot a way on. The robot arrives where the
  // first plan down the sparse crest ends, and finds a wall 1 m ahead. The
  // next cycle, as the node runs it (buildLocalGraph: selection, then the
  // boxed-in departure where the robot has no room to turn), must not turn
  // on the slope: a path that turns only where it may, a straight departure
  // back to room to turn, or no path, which the node answers as
  // kStatusNoPath, retried (PlannerNodeTest.
  // ExplorationBoxedInSendsNoPathThatStartsWithATurn), never as exploration
  // complete.
  const mgg::RobotParams robot = bunker();
  const mgg::PlanningParams planning = bunkerPlanning();
  const Eigen::Vector3d crest(-0.5, 0.1, planning.max_ground_height);
  const auto first_map = sparseRampCrest();
  const auto first =
      planOnSparseCrest(*first_map, robot, planning, crest, 0.0, crest.z());
  ASSERT_FALSE(first->selection.best_path.empty());
  const mgg::StateVec arrived = first->selection.best_path.back()->state;
  ASSERT_GT(arrived.x(), 0.5);
  const double wall = arrived.x() + 1.0;

  for (const bool air_seen : {false, true}) {
    // Unseen, the ramp's air stays as the crest saw it; seen, the lidar
    // carved it down to 0.6 m past the robot on its way down.
    SCOPED_TRACE(air_seen ? "air seen on the way down" : "air unseen");
    const auto map = sparseRampCrest(
        Trench::kNone, wall,
        air_seen ? arrived.x() + 0.6
                 : -std::numeric_limits<double>::infinity());
    const Eigen::Vector3d at = arrived.head<3>();
    ASSERT_FALSE(mgg::roomToTurn(*map, robot, planning, arrived));
    const auto plan =
        planOnSparseCrest(*map, robot, planning, at, 0.0, crest.z());
    const mgg::PathSelectionResult& selection = plan->selection;
    const bool goes_nowhere = mgg::pathGoesNowhere(selection, at, 0.3);
    std::printf("slope end (%.2f, %.2f), %.2f m down, wall at x %.2f, %s: "
                "%zu poses, fallback %d, nowhere %d, unclear %d\n",
                at.x(), at.y(), -(at.z() - crest.z()), wall,
                air_seen ? "air seen" : "air unseen",
                selection.best_path.size(), int(selection.sharp_turn_fallback),
                int(goes_nowhere), int(selection.unclear_viewpoint));
    if (selection.best_path.empty()) continue;  // no path, retried
    if (!selection.sharp_turn_fallback && !goes_nowhere) {
      // Sent: it turns only where it may, from the robot's heading on.
      std::vector<Eigen::Vector3d> points;
      for (const mgg::Vertex* v : selection.best_path) {
        points.push_back(v->state.head<3>());
      }
      EXPECT_TRUE(plan->turns->admissible(points, 0.0));
      for (const Eigen::Vector3d& p : points) EXPECT_LT(p.x(), wall);
      continue;
    }
    // Boxed in: a straight departure back, or none and no path. (Here
    // none: the departure's level body meets the ramp uphill of it.)
    mgg::Departure departure;
    const mgg::GroundProjection ground(*map, planning);
    if (mgg::findDeparture(*map, ground, robot, planning, arrived,
                           departure)) {
      std::printf("  departs %.2f m %s, turning %.0f deg\n",
                  (departure.path.back() - departure.path.front())
                      .head<2>()
                      .norm(),
                  departure.reverse ? "back" : "ahead",
                  departure.turn * 180.0 / M_PI);
      EXPECT_TRUE(departure.reverse);
      EXPECT_EQ(departure.turn, 0.0);
      EXPECT_TRUE(mgg::roomToTurn(*map, robot, planning,
                                  departure.path.back()));
      EXPECT_LT(departure.path.back().x(), arrived.x());
    } else {
      std::printf("  no departure: no path\n");
    }
  }
}

TEST(ObservedGround, AHoleTheLidarLookedIntoIsNotBridged) {
  // A trench two cells wide across the floor, looked into: its inside is
  // free and its bottom, 1 m down, was not seen. Observed floor lies on
  // both sides at one height, but the free space under it is a hole.
  const mgg::RobotParams robot = bunker();
  const mgg::PlanningParams planning = bunkerPlanning();
  const Eigen::Vector3d box = robot.getPlanningSize();
  const Eigen::Vector2d east(1.0, 0.0);
  const Eigen::Vector3d before(-2.3, 0.1, planning.max_ground_height);
  const auto seen_into = sparseRampCrest(Trench::kSeenInto);
  const mgg::GroundProjection ground(*seen_into, planning);
  EXPECT_LT(ground.observedGroundAhead(before, east, box),
            planning.min_observed_ground_fraction);
  std::vector<Eigen::Vector3d> edge;
  EXPECT_EQ(ground.getProjectedEdgeStatus(
                before, before + Eigen::Vector3d(0.4, 0.0, 0.0), box, false,
                edge, false),
            mgg::ProjectedEdgeStatus::kGroundUnobserved);
  // Review r0 (P1): seen free only 1.5 m down, under unknown, it is a hole
  // all the same, with or without the per-plan cache.
  const auto seen_deep = sparseRampCrest(Trench::kSeenDeepInto);
  for (const bool cached : {false, true}) {
    SCOPED_TRACE(cached ? "cached" : "uncached");
    const mgg::GroundProjection deep_ground(*seen_deep, planning, cached);
    EXPECT_LT(deep_ground.observedGroundAhead(before, east, box),
              planning.min_observed_ground_fraction);
  }
  // Review r1 (P2): free space vetoes the bridge down to 5.0 m under the
  // bridged floor (max_projection_length), not deeper: a free voxel 4.8 to
  // 5.0 m down is a hole, one 5.0 to 5.2 m down is not looked at.
  for (const bool cached : {false, true}) {
    for (const std::int64_t row : {std::int64_t(-25), std::int64_t(-26)}) {
      SCOPED_TRACE(std::string(cached ? "cached" : "uncached") + ", row " +
                   std::to_string(row));
      const auto at_depth = sparseRampCrest(
          Trench::kSeenDeepInto, std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity(), row);
      const mgg::GroundProjection depth_ground(*at_depth, planning, cached);
      ASSERT_DOUBLE_EQ(depth_ground.max_projection_length, 5.0);
      const double fraction =
          depth_ground.observedGroundAhead(before, east, box);
      if (row == -25) {
        EXPECT_LT(fraction, planning.min_observed_ground_fraction);
      } else {
        EXPECT_GE(fraction, planning.min_observed_ground_fraction);
      }
    }
  }
  // Review r2 (P2): on a bound, cached or not. Floor at z = 5.7, a trench
  // two cells wide in it, unknown but for one free voxel: row 3, whose
  // sample, 0.7, lies on the lower bound, 5.7 - 5.0 in floating point, is
  // a hole; row 2, 0.2 m under it, is not looked at.
  const auto raised = [](std::int64_t free_row) {
    constexpr double kResolution = 0.2;
    std::vector<mgg::NativeMolaGrid::Cell> occupied;
    std::vector<mgg::NativeMolaGrid::Cell> free;
    std::vector<mgg::NativeMolaGrid::Surface> surfaces;
    for (std::int64_t ix = -20; ix < 5; ++ix) {
      for (std::int64_t iy = -15; iy < 15; ++iy) {
        if (ix == -10 || ix == -9) {
          free.push_back({ix, iy, free_row});
          continue;
        }
        const mgg::NativeMolaGrid::Cell top{ix, iy, 28};  // [5.6, 5.8)
        occupied.push_back(top);
        surfaces.push_back({top, 5.7});
        for (std::int64_t iz = 29; iz < 39; ++iz) free.push_back({ix, iy, iz});
      }
    }
    return std::make_unique<mgg::NativeMolaGrid>(kResolution, occupied, free,
                                                 surfaces);
  };
  const Eigen::Vector3d raised_before(-2.3, 0.1,
                                      5.7 + planning.max_ground_height);
  for (const bool cached : {false, true}) {
    for (const std::int64_t row : {std::int64_t(3), std::int64_t(2)}) {
      SCOPED_TRACE(std::string(cached ? "cached" : "uncached") +
                   ", raised floor, row " + std::to_string(row));
      const auto map = raised(row);
      const mgg::GroundProjection raised_ground(*map, planning, cached);
      const double fraction =
          raised_ground.observedGroundAhead(raised_before, east, box);
      if (row == 3) {
        EXPECT_LT(fraction, planning.min_observed_ground_fraction);
      } else {
        EXPECT_GE(fraction, planning.min_observed_ground_fraction);
      }
    }
  }
  // Not looked into, the same gap is bridged: 0.4 m between observed floor
  // at one height is taken for floor.
  const auto unseen = sparseRampCrest(Trench::kUnseen);
  const mgg::GroundProjection unseen_ground(*unseen, planning);
  EXPECT_GE(unseen_ground.observedGroundAhead(before, east, box),
            planning.min_observed_ground_fraction);
}

}  // namespace

TEST(MolaMap, CentreLineDiscsUseTheirOwnReachAndAllowOutwardSweeps) {
  Publication publication;
  MolaMap provider(config(publication));
  provider.setNoGoDiscs({{10, 0}}, 0.5);
  provider.setNoGoCentreLineDiscs({{0, 0}}, {2.4});
  EXPECT_TRUE(provider.dynamicBoxBlocked({2.3, 0, 0}, {1, 1, 1}));
  EXPECT_FALSE(provider.dynamicBoxBlocked({2.5, 0, 0}, {1, 1, 1}));
  EXPECT_FALSE(provider.dynamicSweepBlocked({0, 0, 0}, {3, 0, 0}, 0.5));
  EXPECT_FALSE(provider.dynamicSweepBlocked({1.1, 0, 0}, {3, 0, 0}, 0.5));
  EXPECT_TRUE(provider.dynamicSweepBlocked({3, 0, 0}, {2.3, 0, 0}, 0.5));
  EXPECT_TRUE(provider.dynamicSweepBlocked({1.1, 0, 0}, {-3, 0, 0}, 0.5));
  provider.setNoGoCentreLineDiscs({}, {});
  EXPECT_FALSE(provider.dynamicBoxBlocked({0, 0, 0}, {1, 1, 1}));
  EXPECT_TRUE(provider.dynamicBoxBlocked({10, 0, 0}, {1, 1, 1}));
}


TEST(MolaMap, FullPadReachDepartureFromCentreAndNearWall) {
  for (bool wall : {false, true}) {
    SCOPED_TRACE(wall);
    Publication publication;
    std::vector<Voxel> occupied, free;
    for (int x = -25; x < 25; ++x) {
      for (int y = -8; y < 8; ++y) {
        occupied.push_back({x, y, -1});
        for (int z = 0; z < 8; ++z) {
          (wall && x == 4 ? occupied : free).push_back({x, y, z});
        }
      }
    }
    const auto request = publication.publish(0, occupied, free, true,
        Eigen::Isometry3d::Identity(), {}, {}, 0.5);
    MolaMap provider(config(publication));
    provider.requestSnapshot(request);
    ASSERT_TRUE(waitFor([&]() { return provider.getStatus(); })) << provider.lastError();
    provider.setNoGoCentreLineDiscs({{0, 0}}, {2.43});
    auto robot = bunker();
    auto planning = bunkerPlanning();
    planning.min_observed_ground_fraction = 0.0;
    const mgg::GroundProjection ground(provider, planning);
    const mgg::StateVec start(0, 0, planning.max_ground_height - 0.1, 0);
    mgg::Departure departure;
    ASSERT_TRUE(mgg::findDeparture(provider, ground, robot, planning, start, departure,
        [&](const auto& path) {
          return !provider.dynamicBoxBlocked(path.back().template head<3>(), robot.getPlanningSize())
              && mgg::roomToTurn(provider, robot, planning, path.back());
        }, 3.0, true));
    EXPECT_EQ(departure.reverse, wall);
    EXPECT_GE(std::abs(departure.path.back().x()), 2.43);
    EXPECT_LE(std::abs(departure.path.back().x()), 3.0);
    EXPECT_TRUE(provider.dynamicSweepBlocked(departure.path.back().head<3>(), start.head<3>(), 0.6));
  }
}

TEST(AerialRootRecovery, SyntheticWallOnlyOutwardDeparture) {
  Publication publication;
  std::vector<Voxel> occupied, free;
  for (int x = -10; x <= 10; ++x)
    for (int y = -5; y <= 5; ++y)
      for (int z = 0; z <= 10; ++z)
        (x == 2 ? occupied : free).push_back({x, y, z});
  MolaMap map(config(publication));
  map.requestSnapshot(publication.publish(0, occupied, free));
  ASSERT_TRUE(waitFor([&] { return map.getStatus(); }));
  const Eigen::Vector3d start(0.15, 0.1, 1.1), size(0.55, 0.55, 0.3);
  EXPECT_EQ(map.getStrictBoxStatus(start, size), VoxelStatus::kOccupied);
  EXPECT_TRUE(mgg::aerialRootDepartureTraversable(map, start, start - Eigen::Vector3d(1,0,0), size));
  EXPECT_FALSE(mgg::aerialRootDepartureTraversable(map, start, start + Eigen::Vector3d(1,0,0), size));
  EXPECT_FALSE(mgg::aerialRootDepartureTraversable(map, start, start + Eigen::Vector3d(0,0.8,0), size));
  // The exemption is not a map mutation or an ordinary edge allowance.
  EXPECT_EQ(map.getStrictPathStatus(start, start - Eigen::Vector3d(1,0,0), size), VoxelStatus::kOccupied);
}

TEST(AerialRootRecovery, ArchivedR6GridsAllowAwayButNotIntoPillar) {
  for (int revision : {39, 40}) {
    SCOPED_TRACE(revision);
    const auto fixture = json::parse(readFile(std::filesystem::path(MGG_MAP_TEST_DATA_DIR) /
        ("aerial_depart_r" + std::to_string(revision) + ".json")));
    std::vector<Voxel> occupied, free;
    for (const auto& c : fixture["occupied"]) occupied.push_back({c[0],c[1],c[2]});
    for (const auto& c : fixture["free"]) free.push_back({c[0],c[1],c[2]});
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c) transform.matrix()(r,c) = fixture["component_from_navigation"][r][c];
    const Eigen::Vector3d start(8.4509,1.6995,1.6168), size(0.55,0.55,0.3);
    Publication publication;
    MolaMap map(config(publication));
    map.requestSnapshot(publication.publish(0, occupied, free, true, transform));
    ASSERT_TRUE(waitFor([&] { return map.getStatus(); }));
    const Eigen::Vector3d root = transform * start;
    const Eigen::Vector3d half = transform.linear().cwiseAbs() * size / 2;
    int root_occupied = 0;
    for (const auto& c : occupied) {
      const Eigen::Vector3d center = 0.2 * Eigen::Vector3d(c.x + 0.5,c.y + 0.5,c.z + 0.5);
      if (((center - root).cwiseAbs().array() <= (half.array() + 0.1)).all()) ++root_occupied;
    }
    ASSERT_EQ(root_occupied, 13);
    const Eigen::Vector3d away = transform.linear().transpose() * Eigen::Vector3d(-0.8,0,0);
    ASSERT_EQ(map.getStrictBoxStatus(start + away, size), VoxelStatus::kFree);
    EXPECT_TRUE(mgg::aerialRootDepartureTraversable(map, start, start + away, size));
    EXPECT_FALSE(mgg::aerialRootDepartureTraversable(map, start, start - away, size));
    EXPECT_EQ(map.getStrictPathStatus(start, start + away, size), VoxelStatus::kOccupied);
    const auto stats = map.aerialRootRecoveryStats();
    EXPECT_EQ(stats.uses, 1u);
    EXPECT_GT(stats.exempted_cells, 0u);
    EXPECT_LE(stats.exempted_cells, 16u);
    RecordProperty("r" + std::to_string(revision) + "_exempted_cells", static_cast<int>(stats.exempted_cells));

    mgg::RobotParams robot;
    robot.type = mgg::RobotType::kAerialRobot;
    robot.size = size;
    robot.center_offset.setZero();
    mgg::PlanningParams planning;
    planning.edge_length_min = 0.05;
    planning.edge_length_max = 2.0;
    planning.edge_overshoot = 0;
    planning.nearest_range = 2.0;
    planning.nearest_range_min = 0.05;
    planning.nearest_range_max = 2.0;
    planning.nearest_range_z = 1.0;
    mgg::ExpandContext ctx;
    ctx.map = &map;
    ctx.robot = &robot;
    ctx.planning = &planning;
    ctx.robot_box_size = size;
    ctx.root_footprint_exempt = true;
    const mgg::StateVec root_state(start.x(),start.y(),start.z(),0);
    const mgg::StateVec target_state(start.x()+away.x(),start.y()+away.y(),start.z()+away.z(),0);
    for (bool physical_root : {false, true}) {
      mgg::GraphManager graph;
      graph.addVertex(new mgg::Vertex(0, root_state));
      ctx.root_is_robot = physical_root;
      mgg::Vertex target(1, target_state);
      mgg::ExpandGraphReport report;
      mgg::expandGraph(graph, target, report, ctx);
      EXPECT_EQ(report.num_vertices_added, physical_root ? 1 : 0);
    }
    // RETURN_HOME's root link is query-local, never an ordinary roadmap edge.
    mgg::GraphManager home_graph;
    home_graph.addVertex(new mgg::Vertex(0, target_state));
    ctx.root_is_robot = false;
    const auto link = mgg::linkDeparture(home_graph, root_state, ctx, 2.0);
    EXPECT_NE(link.vertex, nullptr);
    EXPECT_TRUE(link.query_local);
    EXPECT_EQ(home_graph.getNumVertices(), 1);
    EXPECT_EQ(home_graph.getNumEdges(), 0);
  }
}

TEST(AerialRootRecovery, StrictSafetyBoundsAndDynamicMargins) {
  // A finite post lets toward/along endpoints be free: rejection must come
  // from the sweep, not merely from the ordinary endpoint test.
  for (int mode = 0; mode < 8; ++mode) {
    SCOPED_TRACE(mode);
    Publication publication;
    std::vector<Voxel> occupied{{2,0,5}}, free;
    if (mode == 2) occupied.push_back({-2,0,5});  // new swept obstacle
    if (mode == 4) occupied.push_back({-5,0,5});  // occupied endpoint
    for (int x = -10; x <= 10; ++x)
      for (int y = -5; y <= 5; ++y)
        for (int z = 0; z <= 10; ++z) {
          if (mode == 1 && x == -2 && y == 0 && z == 5) continue;
          if (mode == 3 && x == 0 && y == 0 && z == 5) continue; // unknown root
          free.push_back({x,y,z});
        }
    MolaMap map(config(publication));
    map.requestSnapshot(publication.publish(0, occupied, free));
    ASSERT_TRUE(waitFor([&] { return map.getStatus(); }));
    const Eigen::Vector3d start(0.15,0.1,1.1), size(0.55,0.55,0.3);
    const Eigen::Vector3d away = start - Eigen::Vector3d(1,0,0);
    if (mode == 5) map.setTransientDiscs({{-0.4,0.1}}, 0.1, 60);
    if (mode == 6) map.setNoGoDiscs({{-0.4,0.1}}, 0.1);
    if (mode == 7) map.setNoGoCentreLineDiscs({{-0.4,0.1}}, {0.1});
    EXPECT_EQ(mgg::aerialRootDepartureTraversable(map, start, away, size), mode == 0);
    ASSERT_EQ(map.getStrictBoxStatus(start + Eigen::Vector3d(0,0.8,0), size), VoxelStatus::kFree);
    EXPECT_FALSE(mgg::aerialRootDepartureTraversable(map, start, start + Eigen::Vector3d(0,0.8,0), size));
    ASSERT_EQ(map.getStrictBoxStatus(start + Eigen::Vector3d(1,0,0), size), VoxelStatus::kFree);
    EXPECT_FALSE(mgg::aerialRootDepartureTraversable(map, start, start + Eigen::Vector3d(1,0,0), size));
    EXPECT_FALSE(mgg::aerialRootDepartureTraversable(map, start, start, size));
    // A short partial departure may not end still overlapping the root post.
    EXPECT_FALSE(mgg::aerialRootDepartureTraversable(map, start, start - Eigen::Vector3d(0.01,0,0), size));
    const auto stats = map.aerialRootRecoveryStats();
    EXPECT_EQ(stats.uses, mode == 0 ? 1u : 0u);
    if (mode == 0) {
      EXPECT_EQ(stats.exempted_cells, 1u);
      EXPECT_TRUE(stats.direction.isApprox(Eigen::Vector3d(-1,0,0)));
    }
  }
}

TEST(AerialRootRecovery, SixteenCellLimitAndInvalidQueriesFailClosed) {
  using Cell = mgg::NativeMolaGrid::Cell;
  const Eigen::Vector3d start(0.15,0.4,1.2), size(0.55,0.7,0.7);
  const Eigen::Vector3d end = start - Eigen::Vector3d(1,0,0);
  for (int count : {16, 17}) {
    std::vector<Cell> occupied, free;
    for (int y = 0; y < 4; ++y)
      for (int z = 4; z < 8; ++z) occupied.push_back({2,y,z});
    if (count == 17) occupied.push_back({2,4,5});
    for (int x = -10; x < 10; ++x)
      for (int y = -5; y < 10; ++y)
        for (int z = 0; z < 12; ++z) free.push_back({x,y,z});
    mgg::NativeMolaGrid map(0.2, occupied, free, {});
    const Eigen::Vector3d body = count == 16 ? size : Eigen::Vector3d(0.55,0.9,0.7);
    std::size_t exempted = 99;
    EXPECT_EQ(map.aerialRootRecovery(start, end, body, Eigen::Matrix3d::Identity(), exempted), count == 16);
    EXPECT_EQ(exempted, count == 16 ? 16u : 0u);
    EXPECT_FALSE(map.aerialRootRecovery(start, end, body, 2 * Eigen::Matrix3d::Identity(), exempted));
    EXPECT_FALSE(map.aerialRootRecovery(start, end, -body, Eigen::Matrix3d::Identity(), exempted));
    EXPECT_FALSE(map.aerialRootRecovery(start, end, Eigen::Vector3d::Constant(1e9), Eigen::Matrix3d::Identity(), exempted));
    EXPECT_FALSE(map.aerialRootRecovery(start, Eigen::Vector3d(NAN,0,0), body, Eigen::Matrix3d::Identity(), exempted));
  }
}

TEST(AerialRootRecovery, GroundCallSitesNeverUseAerialFallback) {
  class CountingMap : public MolaMap {
   public:
    using MolaMap::MolaMap;
    mutable int attempts = 0;
    bool aerialRootRecoveryTraversable(const Eigen::Vector3d& a,
        const Eigen::Vector3d& b, const Eigen::Vector3d& size) const override {
      ++attempts;
      return MolaMap::aerialRootRecoveryTraversable(a, b, size);
    }
  };
  Publication publication;
  std::vector<Voxel> occupied, free;
  for (int x = -10; x <= 10; ++x)
    for (int y = -5; y <= 5; ++y)
      for (int z = 0; z <= 8; ++z)
        (z == 0 || x == 2 ? occupied : free).push_back({x,y,z});
  CountingMap map(config(publication));
  map.requestSnapshot(publication.publish(0, occupied, free));
  ASSERT_TRUE(waitFor([&] { return map.getStatus(); }));
  mgg::RobotParams robot;
  robot.type = mgg::RobotType::kGroundRobot;
  robot.size = {0.55,0.55,0.3};
  robot.center_offset.setZero();
  mgg::PlanningParams planning;
  planning.max_ground_height = 0.4;
  planning.path_interpolation_distance = 0.1;
  planning.edge_length_min = 0.05;
  planning.edge_length_max = 2.0;
  planning.edge_overshoot = 0;
  planning.nearest_range = 2.0;
  planning.nearest_range_min = 0.05;
  planning.nearest_range_max = 2.0;
  planning.nearest_range_z = 1.0;
  mgg::GroundProjection ground(map, planning, true);
  const mgg::StateVec start(0.15,0.1,0.5,M_PI), end(-0.85,0.1,0.5,M_PI);
  mgg::Departure departure;
  mgg::findDeparture(map, ground, robot, planning, start, departure);
  mgg::ExpandContext ctx;
  ctx.map = &map;
  ctx.robot = &robot;
  ctx.planning = &planning;
  ctx.ground = &ground;
  ctx.robot_box_size = robot.size;
  ctx.root_is_robot = ctx.root_footprint_exempt = true;
  mgg::GraphManager graph;
  graph.addVertex(new mgg::Vertex(0, start));
  mgg::Vertex target(1, end);
  mgg::ExpandGraphReport report;
  mgg::expandGraph(graph, target, report, ctx);
  mgg::GraphManager home_graph;
  home_graph.addVertex(new mgg::Vertex(0, end));
  mgg::linkDeparture(home_graph, start, ctx, 2.0);
  EXPECT_EQ(map.attempts, 0);
  EXPECT_EQ(map.aerialRootRecoveryStats().uses, 0u);
}

// Lane drone-door (after run drone-r7): the drone's one clearance budget.
// Its footprint (0.354 m, a 0.71 m disc) holds the safety margin, so MGG's
// aerial body is the 0.5 m square it circumscribes, with no extension, held
// along the map's grid and swept exactly; the aerial lattice runs along the
// grid too, and nudges a refused cell to thread a narrow opening.
namespace door {

constexpr double kRes = 0.2;

// The planner's settings for a drone (planner_node, lane drone-door).
void alignBody(MolaMap& map) { map.setGridAlignedBody(true); }
double latticeHeading(const MolaMap& map) {
  double heading = 0.0;
  EXPECT_TRUE(map.gridHeading(heading));
  return heading;
}

// A wall in the component frame: its solid is the slab across `normal`
// from `base` to `base + 0.2 normal`, less a door `width` wide from
// `left` along the wall, from the floor to `height`; floor and ceiling
// close the room. The room spans x [-2.4, 6.4), y [-3, 3), z [0, 3) in the
// wall's own frame, turned by `turn` about z. Cells are occupied when a
// sample of them lies in the solid (as returns would mark them), observed
// free otherwise: the far side is observed too, unless `unknown_beyond`
// leaves cells more than 1.6 m past the wall unknown.
struct Wall {
  double width = 0.9, left = 0.05, height = 2.0, depth = 0.2, turn = 0.0;
  bool unknown_beyond = false;
  Eigen::Vector2d base{2.0, 0.0};
  Eigen::Vector2d along() const { return {-std::sin(turn), std::cos(turn)}; }
  Eigen::Vector2d normal() const { return {std::cos(turn), std::sin(turn)}; }
  bool solid(const Eigen::Vector3d& p) const {
    const Eigen::Vector2d d = p.head<2>() - base;
    const double n = d.dot(normal()), a = d.dot(along());
    if (p.z() < 0.0 || p.z() >= 3.0) return true;  // floor and ceiling
    if (n < 0.0 || n > depth) return false;
    return !(a > left && a < left + width && p.z() < height);
  }
  // The wall's solid boxes in its own frame (n, a, z): exact geometry for
  // the budget check, besides the voxels.
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> boxes() const {
    return {{{0.0, -10.0, 0.0}, {depth, left, 3.0}},
            {{0.0, left + width, 0.0}, {depth, 10.0, 3.0}},
            {{0.0, left, height}, {depth, left + width, 3.0}}};
  }
  void cells(std::vector<Voxel>& occupied, std::vector<Voxel>& free) const {
    for (int x = -20; x < 40; ++x)
      for (int y = -20; y < 20; ++y)
        for (int z = -1; z <= 15; ++z) {
          bool hit = false, room = false;
          for (int i = 0; i < 4 && !hit; ++i)
            for (int j = 0; j < 4 && !hit; ++j)
              for (int k = 0; k < 4 && !hit; ++k) {
                const Eigen::Vector3d p =
                    kRes * Eigen::Vector3d(x + (i + 0.5) / 4, y + (j + 0.5) / 4,
                                           z + (k + 0.5) / 4);
                const Eigen::Vector2d d = p.head<2>() - base;
                const double n = d.dot(normal()), a = d.dot(along());
                room = room || (n > -4.4 && n < 4.4 && std::abs(a) < 3.0);
                hit = solid(p);
              }
          if (!room) continue;
          const Eigen::Vector3d c = kRes * Eigen::Vector3d(x + .5, y + .5, z + .5);
          const double n = (c.head<2>() - base).dot(normal());
          if (hit) {
            occupied.push_back({x, y, z});
          } else if (!(unknown_beyond && n > depth + 1.6)) {
            free.push_back({x, y, z});
          }
        }
  }
};

struct Scene {
  explicit Scene(const Wall& wall, double frame_yaw) : aerial(), wall(wall) {
    alignBody(aerial.map);
    std::vector<Voxel> occupied, free;
    wall.cells(occupied, free);
    component_from_navigation.linear() =
        Eigen::AngleAxisd(frame_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    component_from_navigation.translation() = Eigen::Vector3d(-3.5, -4.0, -0.1);
    aerial.load(free, occupied, component_from_navigation);
  }
  Eigen::Vector3d nav(const Eigen::Vector3d& component) const {
    return component_from_navigation.inverse() * component;
  }
  // A point `n` before (negative) or past the wall's face, `a` along it,
  // at height `z`, in navigation coordinates.
  Eigen::Vector3d at(double n, double a, double z) const {
    const Eigen::Vector2d p = wall.base + n * wall.normal() + a * wall.along();
    return nav(Eigen::Vector3d(p.x(), p.y(), z));
  }
  AerialMolaScene aerial;
  Wall wall;
  Eigen::Isometry3d component_from_navigation = Eigen::Isometry3d::Identity();
};

mgg::GridGraphParams lattice() {
  // bistro.yaml's GridGraphLocal.
  mgg::GridGraphParams grid;
  grid.min_val = {-6.0, -6.0, -0.2};
  grid.max_val = {6.0, 6.0, 0.3};
  grid.resolution = {0.4, 0.4, 0.1};
  return grid;
}

// The swept upright box `size`, held along the component grid, meets the
// box [lo, hi] (component coordinates) exactly when the segment meets the
// box grown by half of it. Independent of the planner's own sweep.
bool sweepMeets(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                const Eigen::Vector3d& size, Eigen::Vector3d lo,
                Eigen::Vector3d hi) {
  lo -= size / 2;
  hi += size / 2;
  double first = 0, last = 1;
  for (int q = 0; q < 3; ++q) {
    const double d = b[q] - a[q];
    if (std::abs(d) < 1e-15) {
      if (a[q] < lo[q] || a[q] > hi[q]) return false;
      continue;
    }
    double x = (lo[q] - a[q]) / d, y = (hi[q] - a[q]) / d;
    if (x > y) std::swap(x, y);
    first = std::max(first, x);
    last = std::min(last, y);
    if (first > last) return false;
  }
  return true;
}

struct Route {
  std::vector<Eigen::Vector3d> points;  // navigation coordinates
  double lattice_s = 0;
  int vertices = 0;
  int nudged = 0;
  int passes = 0;
  bool hit_limit = false;
};

// NAVIGATE as planner_node's routeOverLocalLattice lays it out: the lattice
// round the drone, the goal linked exactly, the shortest path to it.
Route navigate(Scene& scene, const Eigen::Vector3d& start,
               const Eigen::Vector3d& goal) {
  Route route;
  mgg::GraphManager graph;
  const mgg::StateVec root(start.x(), start.y(), start.z(), 0);
  graph.addVertex(new mgg::Vertex(0, root));
  const auto begin = std::chrono::steady_clock::now();
  const auto built = mgg::buildGridGraph(graph, root, lattice(), scene.aerial.ctx,
                                         latticeHeading(scene.aerial.map));
  route.lattice_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
  route.vertices = graph.getNumVertices();
  route.nudged = built.aerial_nudged;
  route.passes = built.aerial_nudge_passes;
  route.hit_limit = built.hit_limit;
  mgg::Vertex* target = mgg::connectStateToGraph(
      graph, mgg::StateVec(goal.x(), goal.y(), goal.z(), 0), scene.aerial.ctx, 0.1,
      true);
  if (target == nullptr) return route;
  mgg::ShortestPathsReport report;
  if (!graph.findShortestPaths(0, report) ||
      report.parent_id_map.find(target->id) == report.parent_id_map.end())
    return route;
  std::vector<mgg::Vertex*> path;
  graph.getShortestPath(target->id, report, true, path);
  for (const auto* v : path) route.points.push_back(v->state.head<3>());
  return route;
}

// Every segment's exact sweep of the 0.5 m body (the budget's square: its
// corners 0.354 m out) stays off every occupied voxel and, for a wall
// along the grid, off the wall's exact solid.
void expectWithinBudget(const Scene& scene, const std::vector<Eigen::Vector3d>& path) {
  std::vector<Voxel> occupied, free;
  scene.wall.cells(occupied, free);
  const Eigen::Vector3d body = scene.aerial.robot.size;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector3d a = scene.component_from_navigation * path[i - 1];
    const Eigen::Vector3d b = scene.component_from_navigation * path[i];
    for (const auto& v : occupied) {
      const Eigen::Vector3d lo = kRes * Eigen::Vector3d(v.x, v.y, v.z);
      ASSERT_FALSE(sweepMeets(a, b, body, lo, lo + Eigen::Vector3d::Constant(kRes)))
          << "segment " << i << " meets voxel " << v.x << "," << v.y << "," << v.z;
    }
    if (scene.wall.turn != 0.0) continue;
    for (const auto& [lo, hi] : scene.wall.boxes()) {
      const Eigen::Vector3d shift(scene.wall.base.x(), scene.wall.base.y(), 0.0);
      ASSERT_FALSE(sweepMeets(a, b, body, lo + shift, hi + shift)) << "segment " << i;
    }
  }
}

bool crossesTheWall(const Scene& scene, const std::vector<Eigen::Vector3d>& path) {
  if (path.size() < 2) return false;
  const auto side = [&](const Eigen::Vector3d& p) {
    const Eigen::Vector3d c = scene.component_from_navigation * p;
    return (c.head<2>() - scene.wall.base).dot(scene.wall.normal());
  };
  return side(path.front()) < 0.0 && side(path.back()) > scene.wall.depth;
}

}  // namespace door

TEST(AerialDoor, NavigateThroughA09mDoorAtBothVoxelAlignmentsAndFrames) {
  // 0.9 m at 0.05 m from a voxel boundary leaves 0.6 m of free voxels
  // between the jambs; at 0.15 m, 0.8 m. The drone starts 1.4 m before the
  // wall, 0.45-0.6 m off the door's centre line (no lattice row through it).
  for (const double frame : {0.0, 0.38}) {
    for (const double left : {0.05, 0.15}) {
      SCOPED_TRACE(::testing::Message() << "frame " << frame << " left " << left);
      door::Wall wall;
      wall.left = left;
      door::Scene scene(wall, frame);
      const auto route =
          door::navigate(scene, scene.at(-1.4, 0.0, 1.3), scene.at(2.2, 0.5, 1.3));
      ASSERT_TRUE(door::crossesTheWall(scene, route.points))
          << route.vertices << " lattice vertices, " << route.nudged << " nudged, "
          << route.passes << " passes, hit limit " << route.hit_limit;
      EXPECT_GT(route.nudged, 0);
      door::expectWithinBudget(scene, route.points);
      RecordProperty("lattice_ms_frame" + std::to_string(int(frame * 100)) + "_left" +
                         std::to_string(int(left * 100)),
                     std::to_string(route.lattice_s * 1000));
      std::printf("[door] 0.9 m, frame %.2f rad, left %.2f: lattice %.1f ms, %d vertices, "
                  "%d nudged, route %zu poses\n",
                  frame, left, route.lattice_s * 1000, route.vertices, route.nudged,
                  route.points.size());
    }
  }
}

TEST(AerialDoor, ADoorNarrowerThanTheBodyIsRefusedEverywhere) {
  for (const double frame : {0.0, 0.38}) {
    for (const double left : {0.05, 0.1, 0.15}) {
      SCOPED_TRACE(::testing::Message() << "frame " << frame << " left " << left);
      door::Wall wall;
      wall.width = 0.6;
      wall.left = left;
      door::Scene scene(wall, frame);
      const auto route =
          door::navigate(scene, scene.at(-1.4, 0.0, 1.3), scene.at(2.2, 0.35, 1.3));
      EXPECT_FALSE(door::crossesTheWall(scene, route.points));
    }
  }
}

TEST(AerialDoor, AWiderDoorAt30DegreesToTheGridPasses) {
  for (const double frame : {0.0, 0.38}) {
    SCOPED_TRACE(frame);
    door::Wall wall;
    wall.width = 1.2;
    wall.left = -0.6;
    wall.turn = M_PI / 6;
    door::Scene scene(wall, frame);
    const auto route =
        door::navigate(scene, scene.at(-1.4, 0.0, 1.3), scene.at(2.2, 0.0, 1.3));
    ASSERT_TRUE(door::crossesTheWall(scene, route.points)) << route.vertices;
    door::expectWithinBudget(scene, route.points);
  }
}

TEST(AerialDoor, ExplorationRoutesThroughTheDoorToWhatIsUnseenBeyond) {
  for (const double frame : {0.0, 0.38}) {
    for (const double left : {0.05, 0.15}) {
      SCOPED_TRACE(::testing::Message() << "frame " << frame << " left " << left);
      door::Wall wall;
      wall.left = left;
      wall.unknown_beyond = true;
      door::Scene scene(wall, frame);
      auto& aerial = scene.aerial;
      const Eigen::Vector3d start = scene.at(-1.4, 0.0, 1.3);
      const mgg::StateVec root(start.x(), start.y(), start.z(), 0);
      mgg::GraphManager graph;
      graph.addVertex(new mgg::Vertex(0, root));
      mgg::buildGridGraph(graph, root, door::lattice(), aerial.ctx,
                          door::latticeHeading(aerial.map));
      mgg::SensorParams lidar;
      lidar.max_range = 4;
      lidar.fov = {2 * M_PI, M_PI / 2};
      lidar.resolution = {M_PI / 36, M_PI / 32};
      lidar.update();
      std::unordered_map<std::string, mgg::SensorParams> sensors{{"drone_45", lidar}};
      aerial.planning.exp_sensor_list = {"drone_45"};
      aerial.planning.unknown_voxel_gain = 1;
      mgg::BoundedSpaceParams bounds;
      bounds.min_val = {-20, -20, 0};
      bounds.max_val = {20, 20, 3};
      bounds.setCenter(Eigen::Vector3d(0, 0, 0), false);
      mgg::GainContext gain;
      gain.map = &aerial.map;
      gain.robot = &aerial.robot;
      gain.planning = &aerial.planning;
      gain.global_space = &bounds;
      gain.sensors = &sensors;
      mgg::computeExplorationGain(graph, gain, false, false);
      const auto chosen = mgg::selectBestPath(graph, aerial.planning, aerial.robot,
                                             mgg::EdgeInclinations(), 0.2, 0);
      std::vector<Eigen::Vector3d> path;
      for (const auto* v : chosen.best_path) path.push_back(v->state.head<3>());
      ASSERT_TRUE(door::crossesTheWall(scene, path)) << path.size() << " poses";
      door::expectWithinBudget(scene, path);
    }
  }
}
