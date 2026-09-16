#include "mgg_map_octomap/mola_map.h"

#include <nlohmann/json.hpp>
#include <openssl/evp.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <iomanip>
#include <iterator>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>
#include <unordered_set>

namespace mgg {
namespace {

using Clock = std::chrono::steady_clock;
using json = nlohmann::json;
constexpr char kSnapshotSchema[] = "swarmdeck.autonomy.v1";
constexpr char kGridSchema[] = "swarmdeck.mola_planner_grid.v1";
constexpr std::size_t kMetadataLimit = 64u * 1024u;
constexpr std::size_t kComponentLimit = 256u;
constexpr std::size_t kSnapshotLimit = 4u * 1024u * 1024u;
constexpr std::size_t kIndexLimit = 4u * 1024u * 1024u;
constexpr std::size_t kGridLimit = 256u * 1024u * 1024u;
constexpr std::size_t kVoxelLimit = 2000000u;

struct Voxel {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;
  friend bool operator==(const Voxel& a, const Voxel& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
  }
  friend bool operator<(const Voxel& a, const Voxel& b) {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    return a.z < b.z;
  }
};

bool isDigest(const std::string& value) {
  static const std::regex pattern("^[0-9a-f]{64}$");
  return std::regex_match(value, pattern);
}

std::string digest(const std::vector<std::uint8_t>& bytes) {
  EVP_MD_CTX* context = EVP_MD_CTX_new();
  if (context == nullptr) throw std::runtime_error("SHA-256 allocation failed");
  std::array<unsigned char, EVP_MAX_MD_SIZE> output{};
  unsigned int output_size = 0;
  const bool ok = EVP_DigestInit_ex(context, EVP_sha256(), nullptr) == 1 &&
                  EVP_DigestUpdate(context, bytes.data(), bytes.size()) == 1 &&
                  EVP_DigestFinal_ex(context, output.data(), &output_size) == 1;
  EVP_MD_CTX_free(context);
  if (!ok || output_size != 32) throw std::runtime_error("SHA-256 failed");
  std::ostringstream text;
  text << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < output_size; ++i)
    text << std::setw(2) << static_cast<unsigned int>(output[i]);
  return text.str();
}

std::string digest(const json& value) {
  const std::string encoded = value.dump();
  return digest(std::vector<std::uint8_t>(encoded.begin(), encoded.end()));
}

void checkDeadline(const Clock::time_point deadline) {
  if (Clock::now() > deadline)
    throw std::runtime_error("MOLA map load exceeded its time bound");
}

bool sameFile(const struct stat& a, const struct stat& b) {
  return a.st_dev == b.st_dev && a.st_ino == b.st_ino &&
         a.st_size == b.st_size && a.st_mtim.tv_sec == b.st_mtim.tv_sec &&
         a.st_mtim.tv_nsec == b.st_mtim.tv_nsec &&
         a.st_ctim.tv_sec == b.st_ctim.tv_sec &&
         a.st_ctim.tv_nsec == b.st_ctim.tv_nsec;
}

class FileDescriptor {
 public:
  explicit FileDescriptor(const int value) : value_(value) {}
  ~FileDescriptor() {
    if (value_ >= 0) ::close(value_);
  }
  FileDescriptor(const FileDescriptor&) = delete;
  FileDescriptor& operator=(const FileDescriptor&) = delete;
  int get() const { return value_; }

 private:
  int value_;
};

std::vector<std::uint8_t> stableRead(const std::filesystem::path& path,
                                     const std::size_t maximum,
                                     const char* label,
                                     const Clock::time_point deadline) {
  const int raw_fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (raw_fd < 0)
    throw std::runtime_error(std::string("cannot open ") + label + ": " +
                             std::strerror(errno));
  const FileDescriptor fd(raw_fd);
  struct stat opened {};
  if (::fstat(fd.get(), &opened) != 0 || !S_ISREG(opened.st_mode) ||
      opened.st_size <= 0 || static_cast<std::uint64_t>(opened.st_size) > maximum) {
    throw std::runtime_error(std::string(label) + " has invalid size or type");
  }
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(opened.st_size));
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const ssize_t count =
        ::read(fd.get(), bytes.data() + offset, bytes.size() - offset);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) {
      throw std::runtime_error(std::string(label) + " changed while reading");
    }
    offset += static_cast<std::size_t>(count);
    checkDeadline(deadline);
  }
  struct stat after {};
  struct stat named {};
  const bool stable = ::fstat(fd.get(), &after) == 0 &&
                      ::stat(path.c_str(), &named) == 0 && sameFile(opened, after) &&
                      sameFile(after, named);
  if (!stable) throw std::runtime_error(std::string(label) + " changed while reading");
  return bytes;
}

json decodeJson(const std::vector<std::uint8_t>& bytes, const char* label) {
  try {
    return json::parse(bytes.begin(), bytes.end(), nullptr, true, true);
  } catch (const json::exception& error) {
    throw std::runtime_error(std::string(label) + " is invalid JSON: " + error.what());
  }
}

std::uint64_t asUint(const json& value, const char* label) {
  if (!value.is_number_unsigned())
    throw std::runtime_error(std::string(label) + " must be an unsigned integer");
  return value.get<std::uint64_t>();
}

std::size_t asSize(const json& value, const char* label,
                   const std::size_t maximum) {
  const auto raw = asUint(value, label);
  if (raw > maximum) throw std::runtime_error(std::string(label) + " exceeds its bound");
  return static_cast<std::size_t>(raw);
}

std::uint32_t readU32(const std::vector<std::uint8_t>& bytes,
                      const std::size_t offset) {
  if (offset > bytes.size() || bytes.size() - offset < 4)
    throw std::runtime_error("MOLA planner grid is truncated");
  std::uint32_t result = 0;
  for (unsigned int i = 0; i < 4; ++i)
    result |= static_cast<std::uint32_t>(bytes[offset + i]) << (8u * i);
  return result;
}

std::uint64_t readU64(const std::vector<std::uint8_t>& bytes,
                      const std::size_t offset) {
  if (offset > bytes.size() || bytes.size() - offset < 8)
    throw std::runtime_error("MOLA planner grid is truncated");
  std::uint64_t result = 0;
  for (unsigned int i = 0; i < 8; ++i)
    result |= static_cast<std::uint64_t>(bytes[offset + i]) << (8u * i);
  return result;
}

std::int64_t readI64(const std::vector<std::uint8_t>& bytes,
                     const std::size_t offset) {
  const std::uint64_t raw = readU64(bytes, offset);
  std::int64_t result = 0;
  std::memcpy(&result, &raw, sizeof(result));
  return result;
}

double readF64(const std::vector<std::uint8_t>& bytes,
               const std::size_t offset) {
  const std::uint64_t raw = readU64(bytes, offset);
  double result = 0.0;
  std::memcpy(&result, &raw, sizeof(result));
  return result;
}

std::size_t checkedProduct(const std::size_t a, const std::size_t b) {
  if (a != 0 && b > std::numeric_limits<std::size_t>::max() / a)
    throw std::runtime_error("MOLA planner grid size overflow");
  return a * b;
}

std::size_t checkedSum(const std::size_t a, const std::size_t b) {
  if (b > std::numeric_limits<std::size_t>::max() - a)
    throw std::runtime_error("MOLA planner grid size overflow");
  return a + b;
}

bool sameIdentity(const MolaSnapshotRequest& a, const MolaSnapshotRequest& b) {
  return a.component_id == b.component_id && a.epoch == b.epoch &&
         a.graph_revision == b.graph_revision &&
         a.geometry_revision == b.geometry_revision &&
         a.source_stamp_ns == b.source_stamp_ns;
}

Eigen::Vector3d enclosingSize(const Eigen::Matrix3d& rotation,
                              const Eigen::Vector3d& size) {
  return rotation.cwiseAbs() * size;
}

void transformPoints(const Eigen::Isometry3d& transform,
                     std::vector<Eigen::Vector3d>& points) {
  for (auto& point : points) point = transform * point;
}

bool canRepresent(const OctomapMap& map, const Eigen::Vector3d& point) {
  if (!point.allFinite()) return false;
  octomap::OcTreeKey key;
  return map.tree()->coordToKeyChecked(
      octomap::point3d(static_cast<float>(point.x()),
                       static_cast<float>(point.y()),
                       static_cast<float>(point.z())),
      key);
}

}  // namespace

struct MolaMap::Snapshot {
  MolaSnapshotRequest request;
  std::shared_ptr<OctomapMap> map;
  std::string artifact_digest;
  Clock::time_point validated_at;
};

struct MolaMap::PendingRequest {
  MolaSnapshotRequest request;
  std::uint64_t generation = 0;
  Clock::time_point received_at;
};

MolaMap::MolaMap(MolaMapConfig config) : config_(std::move(config)) {
  if (config_.peer_root.empty() || !std::filesystem::path(config_.peer_root).is_absolute() ||
      !std::isfinite(config_.resolution) || config_.resolution <= 0.0 ||
      !std::isfinite(config_.snapshot_ttl_sec) || config_.snapshot_ttl_sec <= 0.0 ||
      config_.max_snapshot_bytes == 0 || config_.max_index_bytes == 0 ||
      config_.max_grid_bytes == 0 || config_.max_voxels == 0 ||
      config_.max_snapshot_bytes > kSnapshotLimit ||
      config_.max_index_bytes > kIndexLimit || config_.max_grid_bytes > kGridLimit ||
      config_.max_voxels > kVoxelLimit || config_.max_load_time.count() <= 0 ||
      config_.max_load_time > std::chrono::seconds(10))
    throw std::invalid_argument("MOLA map configuration is invalid or unbounded");
  worker_ = std::thread([this]() { workerLoop(); });
}

MolaMap::~MolaMap() {
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    stopping_ = true;
  }
  request_ready_.notify_one();
  if (worker_.joinable()) worker_.join();
}

void MolaMap::requestSnapshot(const MolaSnapshotRequest& request) {
  const Eigen::Matrix4d& request_matrix =
      request.component_from_navigation.matrix();
  if (request.component_id.empty() || !isDigest(request.geometry_revision) ||
      !request_matrix.allFinite() ||
      !request_matrix.row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0),
                                      1e-9) ||
      !request.component_from_navigation.linear().isUnitary(1e-5) ||
      request.component_from_navigation.linear().determinant() < 0.9999) {
    const std::lock_guard<std::recursive_mutex> publication_lock(
        publication_mutex_);
    std::lock_guard<std::mutex> request_lock(request_mutex_);
    pending_.reset();
    ++generation_;
    if (std::atomic_load(&active_) != nullptr) {
      std::atomic_store(&active_, std::shared_ptr<const Snapshot>());
      active_generation_.fetch_add(1, std::memory_order_release);
    }
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = "invalid MOLA snapshot request";
    return;
  }
  {
    const std::lock_guard<std::recursive_mutex> publication_lock(
        publication_mutex_);
    std::lock_guard<std::mutex> lock(request_mutex_);
    const auto active = std::atomic_load(&active_);
    if (active != nullptr &&
        (!sameIdentity(active->request, request) ||
         !active->request.component_from_navigation.matrix().isApprox(
             request.component_from_navigation.matrix(), 1e-9))) {
      // A corrected key or authority transform must never coexist with the old
      // geometry, even while the new product is being decoded.
      std::atomic_store(&active_, std::shared_ptr<const Snapshot>());
      active_generation_.fetch_add(1, std::memory_order_release);
    }
    pending_ = std::make_unique<PendingRequest>(
        PendingRequest{request, ++generation_, Clock::now()});
  }
  request_ready_.notify_one();
}

MolaMap::ReadLease MolaMap::acquireReadLease() const {
  return ReadLease(publication_mutex_);
}

std::string MolaMap::lastError() const {
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

std::uint64_t MolaMap::activeGeneration() const {
  return active_generation_.load(std::memory_order_acquire);
}

void MolaMap::failIfLatest(const std::uint64_t generation,
                           const std::string& error) {
  const std::lock_guard<std::recursive_mutex> publication_lock(
      publication_mutex_);
  std::lock_guard<std::mutex> request_lock(request_mutex_);
  if (generation != generation_) return;
  if (std::atomic_load(&active_) != nullptr) {
    std::atomic_store(&active_, std::shared_ptr<const Snapshot>());
    active_generation_.fetch_add(1, std::memory_order_release);
  }
  std::lock_guard<std::mutex> lock(error_mutex_);
  last_error_ = error;
}

void MolaMap::workerLoop() {
  while (true) {
    std::unique_ptr<PendingRequest> pending;
    {
      std::unique_lock<std::mutex> lock(request_mutex_);
      request_ready_.wait(lock, [this]() { return stopping_ || pending_ != nullptr; });
      if (stopping_) return;
      pending = std::move(pending_);
    }
    try {
      auto loaded = load(*pending);
      const std::lock_guard<std::recursive_mutex> publication_lock(
          publication_mutex_);
      std::lock_guard<std::mutex> request_lock(request_mutex_);
      if (pending->generation != generation_) {
        // An equivalent heartbeat that arrived during validation is covered
        // by the just-finished coherent read. Consume it to avoid starvation
        // when validation takes longer than the heartbeat interval.
        if (pending_ == nullptr ||
            !sameIdentity(pending_->request, loaded->request) ||
            !pending_->request.component_from_navigation.matrix().isApprox(
                loaded->request.component_from_navigation.matrix(), 1e-9))
          continue;
        pending_.reset();
      }
      const auto prior = std::atomic_load(&active_);
      const bool semantic_change =
          prior == nullptr || !sameIdentity(prior->request, loaded->request) ||
          prior->artifact_digest != loaded->artifact_digest ||
          !prior->request.component_from_navigation.matrix().isApprox(
              loaded->request.component_from_navigation.matrix(), 1e-9);
      std::atomic_store(&active_, std::move(loaded));
      if (semantic_change)
        active_generation_.fetch_add(1, std::memory_order_release);
      std::lock_guard<std::mutex> error_lock(error_mutex_);
      last_error_.clear();
    } catch (const std::exception& error) {
      failIfLatest(pending->generation, error.what());
    }
  }
}

std::shared_ptr<const MolaMap::Snapshot> MolaMap::load(
    const PendingRequest& pending) const {
  const Clock::time_point deadline = Clock::now() + config_.max_load_time;
  const std::filesystem::path root(config_.peer_root);
  const auto snapshot_bytes = stableRead(root / "snapshot.json",
                                         config_.max_snapshot_bytes,
                                         "mapping snapshot", deadline);
  const auto index_bytes = stableRead(root / "mola" / "index.json",
                                      config_.max_index_bytes,
                                      "MOLA index", deadline);
  const json source = decodeJson(snapshot_bytes, "mapping snapshot");
  const json index = decodeJson(index_bytes, "MOLA index");
  const std::string source_digest = digest(snapshot_bytes);
  if (!source.is_object() || source.value("schema", std::string()) != kSnapshotSchema ||
      !source.contains("manifests") || !source["manifests"].is_array() ||
      source["manifests"].size() > kComponentLimit)
    throw std::runtime_error("mapping snapshot schema is invalid");
  const std::string snapshot_id = source.value("snapshot_id", std::string());
  if (!isDigest(snapshot_id)) throw std::runtime_error("mapping snapshot ID is invalid");
  std::unordered_set<std::string> component_ids;
  const json* requested_manifest = nullptr;
  for (const auto& raw : source["manifests"]) {
    if (!raw.is_object()) throw std::runtime_error("mapping manifest is invalid");
    json manifest = raw;
    if (!manifest.contains("schema")) manifest["schema"] = kSnapshotSchema;
    if (manifest.value("schema", std::string()) != kSnapshotSchema ||
        !manifest.contains("graph_revision") ||
        !manifest["graph_revision"].is_object())
      throw std::runtime_error("mapping manifest schema is invalid");
    const auto& graph = raw["graph_revision"];
    const std::string component = graph.value("component_id", std::string());
    if (component.empty() || !component_ids.emplace(component).second)
      throw std::runtime_error("mapping snapshot component identity is invalid");
    if (component == pending.request.component_id) requested_manifest = &raw;
  }
  if (requested_manifest == nullptr)
    throw std::runtime_error("requested component is absent from mapping snapshot");
  const auto& manifest_graph = (*requested_manifest)["graph_revision"];
  if (asUint(manifest_graph.at("epoch"), "manifest epoch") != pending.request.epoch ||
      asUint(manifest_graph.at("revision"), "manifest revision") !=
          pending.request.graph_revision ||
      requested_manifest->value("geometry_revision", std::string()) !=
          pending.request.geometry_revision)
    throw std::runtime_error("mapping manifest does not match authority key");

  if (!index.is_object() || index.value("version", 0) != 1 ||
      index.value("source_snapshot_id", std::string()) != snapshot_id ||
      index.value("source_sha256", std::string()) != source_digest ||
      !index.contains("artifacts") || !index["artifacts"].is_array() ||
      index["artifacts"].size() != source["manifests"].size())
    throw std::runtime_error("MOLA index is not coherent with the mapping snapshot");
  const json* requested_artifact = nullptr;
  std::unordered_set<std::string> artifact_components;
  for (const auto& artifact : index["artifacts"]) {
    if (!artifact.is_object()) throw std::runtime_error("MOLA artifact is invalid");
    const std::string component = artifact.value("component_id", std::string());
    if (!component_ids.count(component) || !artifact_components.emplace(component).second)
      throw std::runtime_error("MOLA index component coverage is invalid");
    if (component == pending.request.component_id) requested_artifact = &artifact;
  }
  if (artifact_components != component_ids || requested_artifact == nullptr)
    throw std::runtime_error("MOLA index does not cover the coherent snapshot");
  json canonical_manifest = *requested_manifest;
  canonical_manifest["schema"] = kSnapshotSchema;
  const std::string manifest_digest = digest(canonical_manifest);
  const std::string index_manifest_digest =
      requested_artifact->value("manifest_sha256", std::string());
  if (asUint(requested_artifact->at("epoch"), "artifact epoch") !=
          pending.request.epoch ||
      asUint(requested_artifact->at("revision"), "artifact revision") !=
          pending.request.graph_revision ||
      requested_artifact->value("geometry_revision", std::string()) !=
          pending.request.geometry_revision ||
      !isDigest(index_manifest_digest) ||
      !requested_artifact->contains("planner") ||
      !(*requested_artifact)["planner"].is_object())
    throw std::runtime_error("MOLA artifact does not match the mapping manifest");

  const auto& planner = (*requested_artifact)["planner"];
  const std::string relative_text = planner.value("path", std::string());
  const std::filesystem::path relative(relative_text);
  if (relative.empty() || relative.is_absolute() || relative.extension() != ".sdpg" ||
      relative.parent_path() != "components" || relative.filename().empty())
    throw std::runtime_error("MOLA planner path is invalid");
  const std::string expected_grid_digest = planner.value("sha256", std::string());
  const std::string planner_snapshot_id =
      planner.value("source_snapshot_id", std::string());
  const std::string planner_source_digest =
      planner.value("source_sha256", std::string());
  if (!isDigest(expected_grid_digest) || !isDigest(planner_snapshot_id) ||
      !isDigest(planner_source_digest))
    throw std::runtime_error("MOLA planner identity is invalid");
  const std::size_t expected_size = asSize(planner.at("size_bytes"),
                                           "planner size", config_.max_grid_bytes);
  const auto grid_bytes = stableRead(root / "mola" / relative, config_.max_grid_bytes,
                                     "MOLA planner grid", deadline);
  if (grid_bytes.size() != expected_size || digest(grid_bytes) != expected_grid_digest)
    throw std::runtime_error("MOLA planner grid integrity check failed");
  // Re-read both transaction markers after the artifact. This catches a
  // source correction or index replacement during decode preparation.
  if (stableRead(root / "snapshot.json", config_.max_snapshot_bytes,
                 "mapping snapshot", deadline) != snapshot_bytes ||
      stableRead(root / "mola" / "index.json", config_.max_index_bytes,
                 "MOLA index", deadline) != index_bytes)
    throw std::runtime_error("MOLA publication changed during validation");

  if (grid_bytes.size() < 12 ||
      std::memcmp(grid_bytes.data(), "SDMGRID1", 8) != 0)
    throw std::runtime_error("MOLA planner grid magic is invalid");
  const std::size_t metadata_size = readU32(grid_bytes, 8);
  if (metadata_size == 0 || metadata_size > kMetadataLimit ||
      metadata_size > grid_bytes.size() - 12)
    throw std::runtime_error("MOLA planner metadata size is invalid");
  const std::vector<std::uint8_t> metadata_bytes(
      grid_bytes.begin() + 12, grid_bytes.begin() + 12 + metadata_size);
  const json metadata = decodeJson(metadata_bytes, "MOLA planner metadata");
  if (!metadata.is_object() || metadata.value("schema", std::string()) != kGridSchema ||
      !metadata.contains("graph_version") || !metadata["graph_version"].is_object() ||
      !metadata.contains("identity") || !metadata["identity"].is_object())
    throw std::runtime_error("MOLA planner metadata schema is invalid");
  const auto& graph = metadata["graph_version"];
  const auto& identity = metadata["identity"];
  if (graph.size() != 4 || identity.size() != 6 ||
      !isDigest(graph.value("digest", std::string())) ||
      !isDigest(identity.value("native_geometry_digest", std::string())) ||
      graph.value("component_id", std::string()) != pending.request.component_id ||
      asUint(graph.at("epoch"), "grid epoch") != pending.request.epoch ||
      asUint(graph.at("revision"), "grid revision") != pending.request.graph_revision ||
      identity.value("geometry_revision", std::string()) !=
          pending.request.geometry_revision ||
      identity.value("canonical_manifest_digest", std::string()) != manifest_digest ||
      identity.value("source_snapshot_id", std::string()) != planner_snapshot_id ||
      identity.value("source_sha256", std::string()) != planner_source_digest ||
      identity.value("reference_frame", std::string()) !=
          requested_manifest->value("frame_id", std::string()) ||
      asUint(metadata.at("source_stamp_ns"), "grid source stamp") !=
          pending.request.source_stamp_ns)
    throw std::runtime_error("MOLA planner metadata does not match authority identity");
  const double resolution = metadata.value("resolution_m", 0.0);
  const double angular_resolution =
      metadata.value("ray_angular_resolution_rad", 0.0);
  const double ray_step_fraction = metadata.value("ray_step_fraction", 0.0);
  if (!std::isfinite(resolution) ||
      std::abs(resolution - config_.resolution) > 1e-9 ||
      !std::isfinite(angular_resolution) ||
      std::abs(angular_resolution - 0.08726646259971647) > 1e-12 ||
      ray_step_fraction != 0.75)
    throw std::runtime_error("MOLA planner resolution does not match configuration");
  static const std::unordered_set<std::string> metadata_fields{
      "schema", "graph_version", "identity", "source_stamp_ns",
      "resolution_m", "ray_angular_resolution_rad", "ray_step_fraction",
      "point_count", "occupied_count", "free_count", "surface_count",
      "ray_steps", "qualified_ray_keyframes"};
  if (metadata.size() != metadata_fields.size())
    throw std::runtime_error("MOLA planner metadata fields are invalid");
  for (auto it = metadata.begin(); it != metadata.end(); ++it)
    if (!metadata_fields.count(it.key()))
      throw std::runtime_error("MOLA planner metadata fields are invalid");
  if (!requested_manifest->contains("submaps") ||
      !(*requested_manifest)["submaps"].is_array())
    throw std::runtime_error("mapping manifest submaps are invalid");
  std::uint64_t expected_points = 0;
  std::uint64_t expected_source_stamp = 0;
  std::size_t expected_qualified = 0;
  std::size_t provenance_records = 0;
  for (const auto& submap : (*requested_manifest)["submaps"]) {
    if (!submap.is_object() || !submap.contains("chunks") ||
        !submap["chunks"].is_array())
      throw std::runtime_error("mapping submap provenance is invalid");
    expected_source_stamp = std::max(
        expected_source_stamp,
        asUint(submap.value("observed_at_ns", json(0)), "observed_at_ns"));
    for (const auto& chunk : submap["chunks"]) {
      if (!chunk.is_object()) throw std::runtime_error("mapping chunk is invalid");
      const auto count = asUint(chunk.at("point_count"), "chunk point count");
      if (count > std::numeric_limits<std::uint64_t>::max() - expected_points)
        throw std::runtime_error("mapping point count overflow");
      expected_points += count;
      if ((++provenance_records & 4095u) == 0) checkDeadline(deadline);
    }
    const auto evidence = submap.find("ray_evidence");
    const auto origins = submap.find("sensor_origins");
    if (evidence != submap.end() && evidence->is_object() &&
        evidence->value("return_semantics", std::string()) == "first_return" &&
        (evidence->value("deskew", std::string()) == "deskewed" ||
         evidence->value("deskew", std::string()) == "not_required") &&
        evidence->value("origin_association", std::string()) == "single_capture" &&
        origins != submap.end() && origins->is_array() && origins->size() == 1)
      ++expected_qualified;
  }
  if (asUint(metadata.at("point_count"), "grid point count") != expected_points ||
      asUint(metadata.at("source_stamp_ns"), "grid source stamp") !=
          expected_source_stamp || expected_source_stamp != pending.request.source_stamp_ns)
    throw std::runtime_error("MOLA planner source count or stamp is invalid");
  const std::size_t occupied_count = asSize(metadata.at("occupied_count"),
                                            "occupied count", config_.max_voxels);
  const std::size_t free_count = asSize(metadata.at("free_count"), "free count",
                                        config_.max_voxels);
  if (free_count > config_.max_voxels - occupied_count)
    throw std::runtime_error("MOLA planner voxel count exceeds its bound");
  const std::size_t surface_count = asSize(metadata.at("surface_count"),
                                           "surface count", config_.max_voxels);
  const std::size_t qualified = asSize(metadata.at("qualified_ray_keyframes"),
                                       "qualified ray count", config_.max_voxels);
  const std::size_t ray_steps = asSize(metadata.at("ray_steps"), "ray steps",
                                       std::numeric_limits<std::size_t>::max());
  if (qualified != expected_qualified)
    throw std::runtime_error("MOLA planner qualified-ray provenance is invalid");
  if (surface_count != expected_points)
    throw std::runtime_error("MOLA planner surfaces do not match source points");
  if (qualified == 0 && (free_count != 0 || ray_steps != 0))
    throw std::runtime_error("MOLA free voxels lack qualified ray evidence");
  std::size_t expected_body = checkedProduct(occupied_count + free_count, 24u);
  expected_body = checkedSum(expected_body, checkedProduct(surface_count, 24u));
  const std::size_t body = 12u + metadata_size;
  if (body > grid_bytes.size() || expected_body != grid_bytes.size() - body)
    throw std::runtime_error("MOLA planner body size is invalid");

  std::vector<Voxel> occupied;
  std::vector<Voxel> free;
  occupied.reserve(occupied_count);
  free.reserve(free_count);
  std::size_t offset = body;
  auto readVoxels = [&](const std::size_t count, std::vector<Voxel>& out,
                        const char* label) {
    for (std::size_t i = 0; i < count; ++i, offset += 24) {
      const Voxel value{readI64(grid_bytes, offset), readI64(grid_bytes, offset + 8),
                        readI64(grid_bytes, offset + 16)};
      if (!out.empty() && !(out.back() < value))
        throw std::runtime_error(std::string("MOLA ") + label +
                                 " voxels are not strictly sorted");
      out.push_back(value);
      if ((i & 4095u) == 0) checkDeadline(deadline);
    }
  };
  readVoxels(occupied_count, occupied, "occupied");
  readVoxels(free_count, free, "free");
  std::size_t occupied_index = 0;
  std::size_t free_index = 0;
  while (occupied_index < occupied.size() && free_index < free.size()) {
    if (occupied[occupied_index] == free[free_index])
      throw std::runtime_error("MOLA occupied and free voxels overlap");
    if (occupied[occupied_index] < free[free_index])
      ++occupied_index;
    else
      ++free_index;
    if (((occupied_index + free_index) & 4095u) == 0) checkDeadline(deadline);
  }
  struct SurfaceRecord {
    std::int64_t x;
    std::int64_t y;
    double z;
  };
  std::vector<SurfaceRecord> surfaces;
  surfaces.reserve(surface_count);
  std::array<std::int64_t, 2> prior_xy{};
  double prior_z = 0.0;
  bool have_prior = false;
  for (std::size_t i = 0; i < surface_count; ++i, offset += 24) {
    const std::int64_t x = readI64(grid_bytes, offset);
    const std::int64_t y = readI64(grid_bytes, offset + 8);
    const double z = readF64(grid_bytes, offset + 16);
    if (!std::isfinite(z) ||
        (have_prior && (x < prior_xy[0] ||
                        (x == prior_xy[0] && (y < prior_xy[1] ||
                         (y == prior_xy[1] && z < prior_z))))))
      throw std::runtime_error("MOLA surface records are invalid or unsorted");
    const double scaled_z = std::floor(z / resolution);
    if (!std::isfinite(scaled_z) ||
        scaled_z < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
        scaled_z >= -static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
        !std::binary_search(occupied.begin(), occupied.end(),
                            Voxel{x, y, static_cast<std::int64_t>(scaled_z)}))
      throw std::runtime_error("MOLA surface has no occupied endpoint");
    prior_xy = {x, y};
    prior_z = z;
    have_prior = true;
    surfaces.push_back({x, y, z});
    if ((i & 4095u) == 0) checkDeadline(deadline);
  }

  const auto previous = std::atomic_load(&active_);
  std::shared_ptr<OctomapMap> map;
  if (previous != nullptr && previous->artifact_digest == expected_grid_digest) {
    map = previous->map;
  } else {
    OctomapConfig map_config;
    map_config.resolution = resolution;
    map_config.max_range = -1.0;
    map = std::make_shared<OctomapMap>(map_config);
    const auto install = [&](const std::vector<Voxel>& voxels, const bool occupied_value) {
      const float log_odds = occupied_value ? map->tree()->getClampingThresMaxLog()
                                            : map->tree()->getClampingThresMinLog();
      for (std::size_t i = 0; i < voxels.size(); ++i) {
        const auto& voxel = voxels[i];
        const Eigen::Vector3d centre = resolution *
            (Eigen::Vector3d(static_cast<double>(voxel.x),
                             static_cast<double>(voxel.y),
                             static_cast<double>(voxel.z)) +
             Eigen::Vector3d::Constant(0.5));
        octomap::OcTreeKey key;
        if (!centre.allFinite() || !map->tree()->coordToKeyChecked(
                octomap::point3d(static_cast<float>(centre.x()),
                                 static_cast<float>(centre.y()),
                                 static_cast<float>(centre.z())), key))
          throw std::runtime_error("MOLA voxel lies outside OctoMap bounds");
        map->tree()->setNodeValue(key, log_odds, true);
        if ((i & 4095u) == 0) checkDeadline(deadline);
      }
    };
    install(free, false);
    install(occupied, true);
    for (std::size_t i = 0; i < surfaces.size(); ++i) {
      const auto& surface = surfaces[i];
      const double voxel_z = std::floor(surface.z / resolution);
      const Eigen::Vector3d occupied_center = resolution *
          (Eigen::Vector3d(static_cast<double>(surface.x),
                           static_cast<double>(surface.y), voxel_z) +
           Eigen::Vector3d::Constant(0.5));
      if (!map->setMeasuredSurfaceZ(occupied_center, surface.z)) {
        throw std::runtime_error(
            "MOLA measured surface could not bind to occupied endpoint");
      }
      if ((i & 4095u) == 0) checkDeadline(deadline);
    }
    // Maximum-depth leaves are queried directly; a full inner-occupancy pass
    // is unnecessary and would be one long OctoMap call that cannot observe
    // the load deadline. Lazy insertion keeps every loop interruptible.
  }
  checkDeadline(deadline);
  // Tree construction may dominate the request. Ensure the source/index pair
  // is still the pair validated above before this candidate can become live.
  if (stableRead(root / "snapshot.json", config_.max_snapshot_bytes,
                 "mapping snapshot", deadline) != snapshot_bytes ||
      stableRead(root / "mola" / "index.json", config_.max_index_bytes,
                 "MOLA index", deadline) != index_bytes)
    throw std::runtime_error("MOLA publication changed during tree construction");
  auto result = std::make_shared<Snapshot>();
  result->request = pending.request;
  result->map = std::move(map);
  result->artifact_digest = expected_grid_digest;
  result->validated_at = Clock::now();
  return result;
}

std::shared_ptr<const MolaMap::Snapshot> MolaMap::current() const {
  auto value = std::atomic_load(&active_);
  if (value == nullptr) return nullptr;
  if (std::chrono::duration<double>(Clock::now() - value->validated_at).count() >
      config_.snapshot_ttl_sec) {
    const std::lock_guard<std::recursive_mutex> lock(publication_mutex_);
    value = std::atomic_load(&active_);
    if (value != nullptr &&
        std::chrono::duration<double>(Clock::now() - value->validated_at).count() >
            config_.snapshot_ttl_sec) {
      std::atomic_store(&active_, std::shared_ptr<const Snapshot>());
      active_generation_.fetch_add(1, std::memory_order_release);
      return nullptr;
    }
  }
  return value;
}

double MolaMap::getResolution() const { return config_.resolution; }

bool MolaMap::getCircleIntersectingXYCellCenters(
    const Eigen::Vector2d& circle_center, const double radius,
    const std::size_t maximum_cells,
    std::vector<XYCellCenter>& centers) const {
  centers.clear();
  const auto snapshot = current();
  if (snapshot == nullptr || !circle_center.allFinite() ||
      !std::isfinite(radius) || radius < 0.0 || maximum_cells == 0) {
    return false;
  }
  const auto& transform = snapshot->request.component_from_navigation;
  // A circle remains a circle under the yaw and translation used between the
  // navigation and MOLA component frames. Tilt would turn the XY footprint
  // into an ellipse and cannot satisfy this ground-query contract.
  const Eigen::Vector3d component_up =
      transform.linear() * Eigen::Vector3d::UnitZ();
  if (!component_up.isApprox(Eigen::Vector3d::UnitZ(), 1e-6)) return false;
  const Eigen::Vector3d navigation_reference(circle_center.x(),
                                              circle_center.y(), 0.0);
  const Eigen::Vector3d component_reference =
      transform * navigation_reference;
  std::vector<XYCellCenter> component_centers;
  if (!snapshot->map->getCircleIntersectingXYCellCenters(
          component_reference.head<2>(), radius, maximum_cells,
          component_centers)) {
    return false;
  }
  centers.reserve(component_centers.size());
  const Eigen::Isometry3d navigation_from_component = transform.inverse();
  for (const XYCellCenter& component_cell : component_centers) {
    const Eigen::Vector3d navigation = navigation_from_component *
        Eigen::Vector3d(component_cell.center.x(), component_cell.center.y(),
                        component_reference.z());
    if (!navigation.allFinite() || centers.size() >= maximum_cells) {
      centers.clear();
      return false;
    }
    centers.push_back(
        {navigation.head<2>(), component_cell.grid_x, component_cell.grid_y});
  }
  return !centers.empty();
}

bool MolaMap::getStatus() const { return current() != nullptr; }

VoxelStatus MolaMap::getVoxelStatus(const Eigen::Vector3d& position) const {
  const auto snapshot = current();
  if (snapshot == nullptr) return VoxelStatus::kUnknown;
  const Eigen::Vector3d component =
      snapshot->request.component_from_navigation * position;
  return canRepresent(*snapshot->map, component)
             ? snapshot->map->getVoxelStatus(component)
             : VoxelStatus::kUnknown;
}

VoxelStatus MolaMap::getRayStatus(const Eigen::Vector3d& view_point,
                                  const Eigen::Vector3d& voxel_to_test,
                                  const bool stop_at_unknown_voxel) const {
  Eigen::Vector3d ignored;
  return getRayStatus(view_point, voxel_to_test, stop_at_unknown_voxel, ignored);
}

VoxelStatus MolaMap::getRayStatus(const Eigen::Vector3d& view_point,
                                  const Eigen::Vector3d& voxel_to_test,
                                  const bool stop_at_unknown_voxel,
                                  Eigen::Vector3d& end_voxel) const {
  const auto snapshot = current();
  if (snapshot == nullptr) {
    end_voxel = view_point;
    return VoxelStatus::kUnknown;
  }
  const Eigen::Vector3d component_start =
      snapshot->request.component_from_navigation * view_point;
  const Eigen::Vector3d component_target =
      snapshot->request.component_from_navigation * voxel_to_test;
  if (!canRepresent(*snapshot->map, component_start) ||
      !canRepresent(*snapshot->map, component_target)) {
    end_voxel = view_point;
    return VoxelStatus::kUnknown;
  }
  Eigen::Vector3d component_end;
  const auto status = snapshot->map->getRayStatus(
      component_start, component_target, stop_at_unknown_voxel, component_end);
  end_voxel = snapshot->request.component_from_navigation.inverse() * component_end;
  return status;
}

VoxelStatus MolaMap::getGroundRayStatus(
    const Eigen::Vector3d& view_point,
    const Eigen::Vector3d& voxel_to_test,
    const bool stop_at_unknown_voxel, Eigen::Vector3d& end_voxel) const {
  const auto snapshot = current();
  if (snapshot == nullptr) {
    end_voxel = view_point;
    return VoxelStatus::kUnknown;
  }
  const auto& transform = snapshot->request.component_from_navigation;
  const Eigen::Vector3d component_start = transform * view_point;
  const Eigen::Vector3d component_target = transform * voxel_to_test;
  if (!canRepresent(*snapshot->map, component_start) ||
      !canRepresent(*snapshot->map, component_target)) {
    end_voxel = view_point;
    return VoxelStatus::kUnknown;
  }
  Eigen::Vector3d component_end;
  const VoxelStatus status = snapshot->map->getGroundRayStatus(
      component_start, component_target, stop_at_unknown_voxel,
      component_end);
  end_voxel = transform.inverse() * component_end;
  return status;
}

VoxelStatus MolaMap::getBoxStatus(const Eigen::Vector3d& center,
                                  const Eigen::Vector3d& size,
                                  const bool stop_at_unknown_voxel) const {
  const auto snapshot = current();
  if (snapshot == nullptr) return VoxelStatus::kUnknown;
  if (!center.allFinite() || !size.allFinite() || (size.array() < 0.0).any())
    return VoxelStatus::kUnknown;
  const auto& transform = snapshot->request.component_from_navigation;
  return snapshot->map->getBoxStatus(transform * center,
                                     enclosingSize(transform.linear(), size),
                                     stop_at_unknown_voxel);
}

VoxelStatus MolaMap::getPathStatus(const Eigen::Vector3d& start,
                                   const Eigen::Vector3d& end,
                                   const Eigen::Vector3d& box_size,
                                   const bool stop_at_unknown_voxel) const {
  const auto snapshot = current();
  if (snapshot == nullptr) return VoxelStatus::kUnknown;
  if (!start.allFinite() || !end.allFinite() || !box_size.allFinite() ||
      (box_size.array() < 0.0).any())
    return VoxelStatus::kUnknown;
  const auto& transform = snapshot->request.component_from_navigation;
  return snapshot->map->getPathStatus(transform * start, transform * end,
                                      enclosingSize(transform.linear(), box_size),
                                      stop_at_unknown_voxel);
}

VoxelStatus MolaMap::getStrictBoxStatus(const Eigen::Vector3d& center,
                                        const Eigen::Vector3d& size) const {
  const auto snapshot = current();
  if (snapshot == nullptr) return VoxelStatus::kUnknown;
  if (!center.allFinite() || !size.allFinite() || (size.array() < 0.0).any())
    return VoxelStatus::kUnknown;
  const auto& transform = snapshot->request.component_from_navigation;
  return snapshot->map->getStrictBoxStatus(
      transform * center, enclosingSize(transform.linear(), size));
}

VoxelStatus MolaMap::getStrictPathStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size) const {
  const auto snapshot = current();
  if (snapshot == nullptr) return VoxelStatus::kUnknown;
  if (!start.allFinite() || !end.allFinite() || !box_size.allFinite() ||
      (box_size.array() < 0.0).any())
    return VoxelStatus::kUnknown;
  const auto& transform = snapshot->request.component_from_navigation;
  return snapshot->map->getStrictPathStatus(
      transform * start, transform * end,
      enclosingSize(transform.linear(), box_size));
}

void MolaMap::getScanStatus(
    const Eigen::Vector3d& pos,
    const std::vector<Eigen::Vector3d>& multiray_endpoints, GainCounts& gain,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
    const SensorModel& sensor) {
  const auto snapshot = current();
  gain = GainCounts{};
  voxel_log.clear();
  if (snapshot == nullptr) return;
  const auto& transform = snapshot->request.component_from_navigation;
  std::vector<Eigen::Vector3d> endpoints = multiray_endpoints;
  transformPoints(transform, endpoints);
  snapshot->map->getScanStatus(transform * pos, endpoints, gain, voxel_log, sensor);
  const Eigen::Isometry3d inverse = transform.inverse();
  for (auto& item : voxel_log) item.first = inverse * item.first;
}

void MolaMap::getScanStatusIterative(
    const Eigen::Vector3d& pos,
    const std::vector<Eigen::Vector3d>& multiray_endpoints, GainCounts& gain,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
    const SensorModel& sensor) {
  const auto snapshot = current();
  gain = GainCounts{};
  voxel_log.clear();
  if (snapshot == nullptr) return;
  const auto& transform = snapshot->request.component_from_navigation;
  std::vector<Eigen::Vector3d> endpoints = multiray_endpoints;
  transformPoints(transform, endpoints);
  snapshot->map->getScanStatusIterative(transform * pos, endpoints, gain, voxel_log,
                                        sensor);
  const Eigen::Isometry3d inverse = transform.inverse();
  for (auto& item : voxel_log) item.first = inverse * item.first;
}

bool MolaMap::augmentFreeBox(const Eigen::Vector3d&, const Eigen::Vector3d&) {
  // Only the qualified free set in SDMGRID1 may establish known free space.
  return false;
}
void MolaMap::augmentFreeFrustum() {}

void MolaMap::resetMap() {
  const std::lock_guard<std::recursive_mutex> publication_lock(
      publication_mutex_);
  std::lock_guard<std::mutex> lock(request_mutex_);
  pending_.reset();
  ++generation_;
  if (std::atomic_load(&active_) != nullptr) {
    std::atomic_store(&active_, std::shared_ptr<const Snapshot>());
    active_generation_.fetch_add(1, std::memory_order_release);
  }
}

void MolaMap::extractLocalMap(
    const Eigen::Vector3d& center, const Eigen::Vector3d& bounding_box_size,
    std::vector<Eigen::Vector3d>& occupied_voxels,
    std::vector<Eigen::Vector3d>& free_voxels) {
  occupied_voxels.clear();
  free_voxels.clear();
  const auto snapshot = current();
  if (snapshot == nullptr) return;
  const auto& transform = snapshot->request.component_from_navigation;
  snapshot->map->extractLocalMap(transform * center,
      enclosingSize(transform.linear(), bounding_box_size), occupied_voxels, free_voxels);
  const Eigen::Isometry3d inverse = transform.inverse();
  transformPoints(inverse, occupied_voxels);
  transformPoints(inverse, free_voxels);
  const auto outside = [&](const Eigen::Vector3d& p) {
    return ((p - center).array().abs() > bounding_box_size.array() / 2.0).any();
  };
  occupied_voxels.erase(std::remove_if(occupied_voxels.begin(), occupied_voxels.end(),
                                       outside), occupied_voxels.end());
  free_voxels.erase(std::remove_if(free_voxels.begin(), free_voxels.end(), outside),
                    free_voxels.end());
}

void MolaMap::extractLocalMapAlongAxis(
    const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
    const Eigen::Vector3d& bounding_box_size,
    std::vector<Eigen::Vector3d>& occupied_voxels,
    std::vector<Eigen::Vector3d>& free_voxels) {
  const double radius = bounding_box_size.norm();
  extractLocalMap(center, Eigen::Vector3d::Constant(radius), occupied_voxels,
                  free_voxels);
  const Eigen::Vector3d direction = axis.norm() > 1e-9 ? axis.normalized()
                                                       : Eigen::Vector3d::UnitX();
  const Eigen::Matrix3d frame = Eigen::Quaterniond::FromTwoVectors(
      Eigen::Vector3d::UnitX(), direction).toRotationMatrix();
  const auto outside = [&](const Eigen::Vector3d& p) {
    return ((frame.transpose() * (p - center)).array().abs() >
            bounding_box_size.array() / 2.0).any();
  };
  occupied_voxels.erase(std::remove_if(occupied_voxels.begin(), occupied_voxels.end(),
                                       outside), occupied_voxels.end());
  free_voxels.erase(std::remove_if(free_voxels.begin(), free_voxels.end(), outside),
                    free_voxels.end());
}

void MolaMap::getLocalPointcloud(const Eigen::Vector3d& center, const double range,
                                 const double yaw,
                                 std::vector<Eigen::Vector3d>& points,
                                 const bool include_unknown_voxels) {
  points.clear();
  const auto snapshot = current();
  if (snapshot == nullptr) return;
  const auto& transform = snapshot->request.component_from_navigation;
  snapshot->map->getLocalPointcloud(transform * center, range, yaw, points,
                                    include_unknown_voxels);
  transformPoints(transform.inverse(), points);
}

void MolaMap::getFreeSpacePointCloud(
    const std::vector<Eigen::Vector3d>& multiray_endpoints, const StateVec& state,
    std::vector<Eigen::Vector3d>& points) {
  points.clear();
  const auto snapshot = current();
  if (snapshot == nullptr) return;
  const auto& transform = snapshot->request.component_from_navigation;
  std::vector<Eigen::Vector3d> endpoints = multiray_endpoints;
  transformPoints(transform, endpoints);
  StateVec component_state = state;
  component_state.head<3>() = transform * state.head<3>();
  snapshot->map->getFreeSpacePointCloud(endpoints, component_state, points);
  transformPoints(transform.inverse(), points);
}

void MolaMap::setRaycastingParams(bool, double) {}
void MolaMap::setRobotRadius(double) {}

}  // namespace mgg
