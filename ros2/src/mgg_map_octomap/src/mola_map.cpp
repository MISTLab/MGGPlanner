#include "mgg_map_octomap/mola_map.h"

#include <nlohmann/json.hpp>
#include <openssl/evp.h>

#include <algorithm>
#include <thread>
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
constexpr char kGridSchema[] = "swarmdeck.mola_planner_grid.v2";
constexpr std::size_t kMetadataLimit = 64u * 1024u;
constexpr std::size_t kComponentLimit = 256u;
constexpr std::size_t kSnapshotLimit = 64u * 1024u * 1024u;
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

// Thrown when the publication on disk is not the one the request names: the
// source and index describe different publications (the worker writes the two
// files a moment apart), a file was replaced while it was being read, or the
// product carries another revision than the authority key. The loader retries
// these within its budget; after the budget the worker keeps a compatible
// predecessor in service instead of dropping the map.
struct CoherenceRace : std::runtime_error {
  using std::runtime_error::runtime_error;
};
// Thrown when the load budget runs out; load() reports it as the race that
// consumed the budget when one was seen.
struct DeadlineExceeded : std::runtime_error {
  using std::runtime_error::runtime_error;
};
constexpr std::chrono::milliseconds kCoherenceRetryInterval{100};

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
    throw DeadlineExceeded("MOLA map load exceeded its time bound");
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
                                     const Clock::time_point deadline,
                                     struct stat* identity = nullptr) {
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
      throw CoherenceRace(std::string(label) + " changed while reading");
    }
    offset += static_cast<std::size_t>(count);
    checkDeadline(deadline);
  }
  struct stat after {};
  struct stat named {};
  const bool stable = ::fstat(fd.get(), &after) == 0 &&
                      ::stat(path.c_str(), &named) == 0 && sameFile(opened, after) &&
                      sameFile(after, named);
  // A file replaced or truncated under the reader is a publication in
  // progress, not a broken product; the caller retries within its budget.
  if (!stable) throw CoherenceRace(std::string(label) + " changed while reading");
  if (identity != nullptr) *identity = named;
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

bool sameTransform(const MolaSnapshotRequest& a, const MolaSnapshotRequest& b) {
  return a.component_from_navigation.matrix().isApprox(
      b.component_from_navigation.matrix(), 1e-9);
}

// Two requests are compatible when one geometry may stand in for the other
// while a newer revision is decoded: the same component of the same epoch,
// placed by the same authority transform. A revision, geometry revision or
// source stamp that differs only means the product is a few keyframes behind.
bool compatible(const MolaSnapshotRequest& a, const MolaSnapshotRequest& b) {
  return a.component_id == b.component_id && a.epoch == b.epoch &&
         sameTransform(a, b);
}

Eigen::Vector3d enclosingSize(const Eigen::Matrix3d& rotation,
                              const Eigen::Vector3d& size) {
  return rotation.cwiseAbs() * size;
}

void transformPoints(const Eigen::Isometry3d& transform,
                     std::vector<Eigen::Vector3d>& points) {
  for (auto& point : points) point = transform * point;
}

bool canRepresent(const NativeMolaGrid& map, const Eigen::Vector3d& point) {
  if (!point.allFinite()) return false;
  Eigen::Vector2d ignored;
  return map.getAxisAlignedXYCellCenter(point.head<2>(), ignored) &&
         std::isfinite(point.z() / map.getResolution()) &&
         std::abs(point.z() / map.getResolution()) <
             double(std::numeric_limits<std::int64_t>::max()) / 4.0;
}

}  // namespace

struct MolaMap::Snapshot {
  MolaSnapshotRequest request;
  std::shared_ptr<NativeMolaGrid> map;
  std::string artifact_digest;
  // The publication files exactly as read for this geometry. A heartbeat
  // that repeats the active identity is revalidated against these by stat:
  // the same inode, size and times are the same bytes, which were verified
  // when the geometry was built, so nothing is re-read or re-hashed.
  struct stat source_identity {};
  struct stat index_identity {};
  // Steady-clock nanoseconds of the last on-disk confirmation that this
  // geometry may be served: set by the load that built it and refreshed in
  // place when a compatible successor heartbeat ends in a coherence race.
  // The geometry itself stays immutable.
  mutable std::atomic<std::int64_t> validated_at_ns{0};
  mutable std::atomic<std::size_t> reader_pins{0};

  void markValidated(const Clock::time_point when) const {
    validated_at_ns.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            when.time_since_epoch()).count(),
        std::memory_order_release);
  }
  bool expired(const Clock::time_point now, const double ttl_sec) const {
    const std::chrono::nanoseconds validated(
        validated_at_ns.load(std::memory_order_acquire));
    return std::chrono::duration<double>(now.time_since_epoch() - validated)
               .count() > ttl_sec;
  }
};

struct MolaMap::PendingRequest {
  MolaSnapshotRequest request;
  std::uint64_t generation = 0;
  Clock::time_point received_at;
};

thread_local std::vector<MolaMap::ThreadPin> MolaMap::thread_pins_;

MolaMap::ReadLease::ReadLease(const MolaMap& owner)
    : owner_(&owner),
      owner_thread_(std::this_thread::get_id()),
      lock_(owner.publication_mutex_) {
  snapshot_ = owner.beginReadLease();
}

MolaMap::ReadLease::ReadLease(ReadLease&& other) noexcept
    : owner_(other.owner_),
      snapshot_(std::move(other.snapshot_)),
      owner_thread_(other.owner_thread_),
      lock_(std::move(other.lock_)) {
  if (owner_ != nullptr && owner_thread_ != std::this_thread::get_id())
    std::terminate();
  other.owner_ = nullptr;
  other.owner_thread_ = std::thread::id{};
}

MolaMap::ReadLease& MolaMap::ReadLease::operator=(ReadLease&& other) noexcept {
  if (this == &other) return *this;
  if (other.owner_ != nullptr &&
      other.owner_thread_ != std::this_thread::get_id())
    std::terminate();
  release();
  owner_ = other.owner_;
  snapshot_ = std::move(other.snapshot_);
  owner_thread_ = other.owner_thread_;
  lock_ = std::move(other.lock_);
  other.owner_ = nullptr;
  other.owner_thread_ = std::thread::id{};
  return *this;
}

MolaMap::ReadLease::~ReadLease() { release(); }

void MolaMap::ReadLease::allowPublication() {
  if (owner_ == nullptr || !lock_.owns_lock()) return;
  if (owner_thread_ != std::this_thread::get_id())
    throw std::logic_error("MOLA read lease used from another thread");
  lock_.unlock();
}

void MolaMap::ReadLease::reacquirePublication() {
  if (owner_ == nullptr || lock_.owns_lock()) return;
  if (owner_thread_ != std::this_thread::get_id())
    throw std::logic_error("MOLA read lease used from another thread");
  lock_.lock();
}

void MolaMap::ReadLease::release() noexcept {
  if (owner_ == nullptr) return;
  if (owner_thread_ != std::this_thread::get_id()) std::terminate();
  if (!lock_.owns_lock()) lock_.lock();
  owner_->endReadLease(snapshot_);
  lock_.unlock();
  owner_ = nullptr;
  snapshot_.reset();
  owner_thread_ = std::thread::id{};
}

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
    if (active != nullptr && pending_ == nullptr &&
        sameIdentity(active->request, request) &&
        sameTransform(active->request, request) &&
        publicationUnchanged(*active)) {
      // The heartbeat repeats the served identity and the product files it
      // was read from are the same files: confirmed on disk without a load.
      active->markValidated(Clock::now());
      stat_revalidation_count_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    if (active != nullptr && !compatible(active->request, request)) {
      // Another component or epoch, or a moved authority transform, must never
      // coexist with the old geometry, even while the new product is being
      // decoded. A compatible successor (a newer revision of the same
      // component under the same transform) keeps the active snapshot in
      // service until the worker installs the successor or retains it.
      std::atomic_store(&active_, std::shared_ptr<const Snapshot>());
      active_generation_.fetch_add(1, std::memory_order_release);
    }
    pending_ = std::make_unique<PendingRequest>(
        PendingRequest{request, ++generation_, Clock::now()});
  }
  request_ready_.notify_one();
}

MolaMap::ReadLease MolaMap::acquireReadLease() const {
  return ReadLease(*this);
}

std::shared_ptr<const MolaMap::Snapshot> MolaMap::beginReadLease() const {
  for (auto& pin : thread_pins_) {
    if (pin.owner != this) continue;
    ++pin.depth;
    if (pin.snapshot != nullptr)
      pin.snapshot->reader_pins.fetch_add(1, std::memory_order_relaxed);
    return pin.snapshot;
  }
  const auto snapshot = current();
  thread_pins_.push_back(ThreadPin{this, snapshot, 1});
  if (snapshot != nullptr)
    snapshot->reader_pins.fetch_add(1, std::memory_order_relaxed);
  return snapshot;
}

void MolaMap::endReadLease(
    const std::shared_ptr<const Snapshot>& snapshot) const {
  for (auto pin = thread_pins_.begin(); pin != thread_pins_.end(); ++pin) {
    if (pin->owner != this) continue;
    if (pin->depth == 0 || pin->snapshot != snapshot) std::terminate();
    if (snapshot != nullptr &&
        snapshot->reader_pins.fetch_sub(1, std::memory_order_acq_rel) == 0)
      std::terminate();
    if (--pin->depth != 0) return;
    thread_pins_.erase(pin);
    // If this was the final pin of an expired active snapshot, restore the
    // ordinary expiry behavior now. A coherent heartbeat may already have
    // installed a fresh replacement, in which case current() preserves it.
    (void)current();
    return;
  }
  std::terminate();
}

std::string MolaMap::lastError() const {
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

std::uint64_t MolaMap::activeGeneration() const {
  return active_generation_.load(std::memory_order_acquire);
}

std::uint64_t MolaMap::retainedPredecessorCount() const {
  return retained_predecessor_count_.load(std::memory_order_relaxed);
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

void MolaMap::retainPredecessorOrFail(const PendingRequest& pending,
                                      const std::string& error) {
  const std::lock_guard<std::recursive_mutex> publication_lock(
      publication_mutex_);
  std::lock_guard<std::mutex> request_lock(request_mutex_);
  const auto active = std::atomic_load(&active_);
  if (active != nullptr && compatible(active->request, pending.request)) {
    // The product on disk still describes another revision, or the
    // source/index pair is mid-replacement. The active geometry is the same
    // component of the same epoch under the same authority transform, so it
    // stays the right map to serve until the successor lands. Refresh its
    // validity clock so current() does not expire it: this heartbeat confirmed
    // the authority is alive on this component. This also covers a request a
    // newer heartbeat has superseded, because every request since the
    // install was compatible (an incompatible one would have retracted it)
    // and the next attempt may not finish before the clock would run out.
    // last_error_ stays untouched: it is empty whenever a snapshot is active.
    active->markValidated(Clock::now());
    retained_predecessor_count_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (pending.generation != generation_) return;
  if (active != nullptr) {
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
        // A newer heartbeat arrived during validation. An incompatible one
        // (or a reset) has already retracted the active snapshot, and this
        // geometry must not resurface under it. A compatible successor keeps
        // being served from this snapshot, a fresher predecessor than the one
        // it would otherwise retain, and stays queued for its own load. An
        // equivalent heartbeat is covered by the just-finished coherent read:
        // consume it to avoid starvation when validation takes longer than
        // the heartbeat interval.
        if (pending_ == nullptr ||
            !compatible(pending_->request, loaded->request))
          continue;
        if (sameIdentity(pending_->request, loaded->request)) pending_.reset();
      }
      const auto prior = std::atomic_load(&active_);
      // A retained predecessor is replaced here by its successor: the
      // identity differs, so the generation advances exactly once for it.
      const bool semantic_change =
          prior == nullptr || !sameIdentity(prior->request, loaded->request) ||
          prior->artifact_digest != loaded->artifact_digest ||
          !sameTransform(prior->request, loaded->request);
      std::atomic_store(&active_, std::move(loaded));
      if (semantic_change)
        active_generation_.fetch_add(1, std::memory_order_release);
      std::lock_guard<std::mutex> error_lock(error_mutex_);
      last_error_.clear();
    } catch (const CoherenceRace& race) {
      retainPredecessorOrFail(*pending, race.what());
    } catch (const std::exception& error) {
      failIfLatest(pending->generation, error.what());
    }
  }
}

std::shared_ptr<const MolaMap::Snapshot> MolaMap::load(
    const PendingRequest& pending) const {
  const Clock::time_point deadline = Clock::now() + config_.max_load_time;
  // The mapping worker publishes a product as two atomic writes a moment
  // apart: mola/source.json, the exact mapping snapshot bytes it built from,
  // and then mola/index.json, whose source digest and snapshot ID name those
  // bytes. A request that reads between the two writes sees files that are
  // each valid but do not yet describe each other. The authority key only
  // names revisions with a published product, but the worker publishes a few
  // seconds behind the newest revision, so a heartbeat can also reach a
  // product that still carries the previous revision. Retry within the load
  // budget instead of failing the plan request on a race that resolves by
  // itself; when the budget runs out the race is reported as such, and the
  // worker keeps a compatible predecessor in service.
  std::string race_seen;
  for (;;) {
    try {
      return loadOnce(pending, deadline);
    } catch (const CoherenceRace& race) {
      if (Clock::now() + kCoherenceRetryInterval >= deadline) throw;
      race_seen = race.what();
      std::this_thread::sleep_for(kCoherenceRetryInterval);
    } catch (const DeadlineExceeded&) {
      // The budget went to waiting out a race that never resolved; report the
      // race, not a slow read, so the predecessor is retained rather than
      // dropped. A first attempt that runs out of time is a slow load.
      if (race_seen.empty()) throw;
      throw CoherenceRace(race_seen);
    }
  }
}

std::shared_ptr<const MolaMap::Snapshot> MolaMap::loadOnce(
    const PendingRequest& pending, const Clock::time_point deadline) const {
  // The product describes itself: mola/source.json holds the exact mapping
  // snapshot bytes the worker built from, and mola/index.json names those
  // bytes. The capture process's own snapshot.json in the peer root may
  // already be ahead of the product and is never read.
  const std::filesystem::path root(config_.peer_root);
  const std::filesystem::path source_path = root / "mola" / "source.json";
  const std::filesystem::path index_path = root / "mola" / "index.json";
  const auto source_bytes = stableRead(source_path, config_.max_snapshot_bytes,
                                       "MOLA source snapshot", deadline);
  const auto index_bytes = stableRead(index_path, config_.max_index_bytes,
                                      "MOLA index", deadline);
  const json source = decodeJson(source_bytes, "MOLA source snapshot");
  const json index = decodeJson(index_bytes, "MOLA index");
  const std::string source_digest = digest(source_bytes);
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
    throw CoherenceRace("mapping manifest does not match authority key");

  if (!index.is_object() || index.value("version", 0) != 1 ||
      index.value("source_snapshot_id", std::string()) != snapshot_id ||
      index.value("source_sha256", std::string()) != source_digest ||
      !index.contains("artifacts") || !index["artifacts"].is_array() ||
      index["artifacts"].size() != source["manifests"].size())
    throw CoherenceRace("MOLA index is not coherent with the mapping snapshot");
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
  // source or index replacement during decode preparation; the pair read
  // above may then no longer be the publication the artifact belongs to.
  if (stableRead(source_path, config_.max_snapshot_bytes,
                 "MOLA source snapshot", deadline) != source_bytes ||
      stableRead(index_path, config_.max_index_bytes, "MOLA index",
                 deadline) != index_bytes)
    throw CoherenceRace("MOLA publication changed during validation");

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
      "point_count", "source_point_count", "occupied_count", "free_count", "surface_count",
      "retired_count", "ray_steps", "qualified_ray_keyframes"};
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
  if (asUint(metadata.at("source_point_count"), "grid source point count") != expected_points ||
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
  const std::size_t retired_count = asSize(metadata.at("retired_count"),
                                           "retired count", config_.max_voxels);
  const std::size_t qualified = asSize(metadata.at("qualified_ray_keyframes"),
                                       "qualified ray count", config_.max_voxels);
  const std::size_t ray_steps = asSize(metadata.at("ray_steps"), "ray steps",
                                       std::numeric_limits<std::size_t>::max());
  if (qualified != expected_qualified)
    throw std::runtime_error("MOLA planner qualified-ray provenance is invalid");
  // Raw provenance stays exact while the product holds bounded surface extrema.
  const auto materialized_points = asSize(
      metadata.at("point_count"), "grid point count", config_.max_voxels);
  if (materialized_points > expected_points ||
      static_cast<std::uint64_t>(surface_count) +
          static_cast<std::uint64_t>(retired_count) != materialized_points)
    throw std::runtime_error("MOLA planner surfaces do not match materialized points");
  // Retirement rests on the same evidence free space does: only a qualified
  // capture's rays can prove an endpoint was seen through.
  if (qualified == 0 && (free_count != 0 || ray_steps != 0 || retired_count != 0))
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
  std::shared_ptr<NativeMolaGrid> map;
  if (previous != nullptr && previous->artifact_digest == expected_grid_digest) {
    map = previous->map;
  } else {
    std::vector<NativeMolaGrid::Cell> native_occupied, native_free;
    std::vector<NativeMolaGrid::Surface> native_surfaces;
    native_occupied.reserve(occupied.size()); native_free.reserve(free.size());
    native_surfaces.reserve(surfaces.size());
    for (const auto& v : occupied) native_occupied.push_back({v.x, v.y, v.z});
    for (const auto& v : free) native_free.push_back({v.x, v.y, v.z});
    for (const auto& s : surfaces) {
      native_surfaces.push_back({{s.x, s.y,
          static_cast<std::int64_t>(std::floor(s.z / resolution))}, s.z});
    }
    checkDeadline(deadline);
    map = std::make_shared<NativeMolaGrid>(resolution,
        std::move(native_occupied), std::move(native_free),
        std::move(native_surfaces));
  }
  checkDeadline(deadline);
  // Tree construction may dominate the request. Ensure the source/index pair
  // is still the pair validated above before this candidate can become live.
  struct stat source_identity {};
  struct stat index_identity {};
  if (stableRead(source_path, config_.max_snapshot_bytes,
                 "MOLA source snapshot", deadline, &source_identity) !=
          source_bytes ||
      stableRead(index_path, config_.max_index_bytes, "MOLA index", deadline,
                 &index_identity) != index_bytes)
    throw CoherenceRace("MOLA publication changed during grid construction");
  auto result = std::make_shared<Snapshot>();
  result->request = pending.request;
  result->map = std::move(map);
  result->artifact_digest = expected_grid_digest;
  result->source_identity = source_identity;
  result->index_identity = index_identity;
  result->markValidated(Clock::now());
  return result;
}

bool MolaMap::publicationUnchanged(const Snapshot& snapshot) const {
  const std::filesystem::path root(config_.peer_root);
  struct stat source {};
  struct stat index {};
  return ::stat((root / "mola" / "source.json").c_str(), &source) == 0 &&
         ::stat((root / "mola" / "index.json").c_str(), &index) == 0 &&
         sameFile(snapshot.source_identity, source) &&
         sameFile(snapshot.index_identity, index);
}

std::uint64_t MolaMap::statRevalidationCount() const {
  return stat_revalidation_count_.load(std::memory_order_relaxed);
}

std::shared_ptr<const MolaMap::Snapshot> MolaMap::current() const {
  for (const auto& pin : thread_pins_) {
    if (pin.owner == this) return pin.snapshot;
  }
  auto value = std::atomic_load(&active_);
  if (value == nullptr) return nullptr;
  if (value->expired(Clock::now(), config_.snapshot_ttl_sec)) {
    const std::lock_guard<std::recursive_mutex> lock(publication_mutex_);
    value = std::atomic_load(&active_);
    if (value != nullptr &&
        value->expired(Clock::now(), config_.snapshot_ttl_sec)) {
      // A transaction admitted while this exact snapshot was fresh may finish
      // on it, but another thread may neither use nor expire that transaction's
      // snapshot. The final pin release performs ordinary expiry if no coherent
      // heartbeat has installed a refreshed replacement or refreshed this
      // snapshot's validity clock in place.
      if (value->reader_pins.load(std::memory_order_acquire) != 0)
        return nullptr;
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
  // navigation and MOLA component frames. A large tilt would turn the XY
  // footprint into an ellipse and cannot satisfy this ground-query contract;
  // the few milliradians a peer SLAM correction carries are within the
  // documented tolerance.
  if (!authorityTiltAcceptable(transform.linear())) return false;
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

void MolaMap::setTransientDiscs(std::vector<Eigen::Vector2d> centres,
                                const double radius_m, const double ttl_s) {
  auto discs = std::make_shared<TransientDiscs>();
  if (std::isfinite(radius_m) && radius_m > 0.0 && std::isfinite(ttl_s) &&
      ttl_s > 0.0) {
    for (const Eigen::Vector2d& centre : centres) {
      if (centre.allFinite()) discs->centres.push_back(centre);
    }
    discs->radius_m = radius_m;
    discs->expires =
        Clock::now() + std::chrono::duration_cast<Clock::duration>(
                           std::chrono::duration<double>(ttl_s));
  }
  std::atomic_store(&transient_discs_,
                    std::shared_ptr<const TransientDiscs>(std::move(discs)));
}

bool MolaMap::discsBlockSweep(const Eigen::Vector3d& start,
                              const Eigen::Vector3d& end,
                              const double half_width) const {
  const auto discs = std::atomic_load(&transient_discs_);
  if (discs == nullptr || discs->centres.empty() ||
      Clock::now() > discs->expires) {
    return false;
  }
  const Eigen::Vector2d a = start.head<2>();
  const Eigen::Vector2d delta = end.head<2>() - a;
  const double length_squared = delta.squaredNorm();
  const double reach = discs->radius_m + half_width;
  for (const Eigen::Vector2d& centre : discs->centres) {
    const double t =
        length_squared > 1e-12
            ? std::clamp((centre - a).dot(delta) / length_squared, 0.0, 1.0)
            : 0.0;
    const double closest = (centre - (a + t * delta)).squaredNorm();
    if (closest >= reach * reach) continue;
    // A sweep that starts inside a disc's reach begins where the robot (or
    // a lattice cell it can already reach) stands beside that neighbour: the
    // disc is a margin, not a measured collision, and two robots that stop
    // 0.9 m apart must be able to drive away from each other. Only a sweep
    // that brings the body closer to the neighbour than its start already is
    // stays blocked; one that keeps or increases the distance may leave.
    // Measured on benchbot 2026-09-18: robot_0 and robot_2 parked 0.9 m apart
    // and every outgoing lattice edge of both was rejected as a collision
    // (4851 of 4852 cells) until the trial ended.
    const double at_start = (centre - a).squaredNorm();
    if (at_start < reach * reach && closest >= at_start - 1e-9) continue;
    return true;
  }
  return false;
}

bool MolaMap::discsBlockBox(const Eigen::Vector3d& center,
                            const Eigen::Vector3d& size) const {
  // A box is a place, not a departure: inside a neighbour's reach it is
  // occupied, whichever way a sweep through it might be allowed to leave.
  const auto discs = std::atomic_load(&transient_discs_);
  if (discs == nullptr || discs->centres.empty() ||
      Clock::now() > discs->expires) {
    return false;
  }
  const double reach = discs->radius_m + 0.5 * std::max(size.x(), size.y());
  const Eigen::Vector2d at = center.head<2>();
  for (const Eigen::Vector2d& centre : discs->centres) {
    if ((centre - at).squaredNorm() < reach * reach) return true;
  }
  return false;
}

VoxelStatus MolaMap::getBoxStatus(const Eigen::Vector3d& center,
                                  const Eigen::Vector3d& size,
                                  const bool stop_at_unknown_voxel) const {
  const auto snapshot = current();
  if (snapshot == nullptr) return VoxelStatus::kUnknown;
  if (!center.allFinite() || !size.allFinite() || (size.array() < 0.0).any())
    return VoxelStatus::kUnknown;
  if (discsBlockBox(center, size)) return VoxelStatus::kOccupied;
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
  if (discsBlockSweep(start, end, 0.5 * std::max(box_size.x(), box_size.y())))
    return VoxelStatus::kOccupied;
  const auto& transform = snapshot->request.component_from_navigation;
  return snapshot->map->getPathStatus(transform * start, transform * end,
                                      enclosingSize(transform.linear(), box_size),
                                      stop_at_unknown_voxel);
}

VoxelStatus MolaMap::getOccupiedOnlyCylinderPathStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const double radius, const double height) const {
  const auto snapshot = current();
  if (snapshot == nullptr || !start.allFinite() || !end.allFinite() ||
      !std::isfinite(radius) || radius < 0.0 || !std::isfinite(height) ||
      height < 0.0) {
    return VoxelStatus::kUnknown;
  }
  if (discsBlockSweep(start, end, radius)) return VoxelStatus::kOccupied;
  const auto& transform = snapshot->request.component_from_navigation;
  // The swept cylinder is upright in the navigation frame; see
  // kMaxAuthorityTiltRad for the tilt this contract tolerates.
  if (!authorityTiltAcceptable(transform.linear())) {
    return VoxelStatus::kUnknown;
  }
  return snapshot->map->getOccupiedOnlyCylinderPathStatus(
      transform * start, transform * end, radius, height);
}

VoxelStatus MolaMap::getStrictBoxStatus(const Eigen::Vector3d& center,
                                        const Eigen::Vector3d& size) const {
  const auto snapshot = current();
  if (snapshot == nullptr) return VoxelStatus::kUnknown;
  if (!center.allFinite() || !size.allFinite() || (size.array() < 0.0).any())
    return VoxelStatus::kUnknown;
  if (discsBlockBox(center, size)) return VoxelStatus::kOccupied;
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
  if (discsBlockSweep(start, end, 0.5 * std::max(box_size.x(), box_size.y())))
    return VoxelStatus::kOccupied;
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
