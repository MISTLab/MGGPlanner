#include "mgg_ros/local_replay.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <limits>
#include <tuple>
#include <set>
#include <sstream>
#include <variant>

#include <openssl/evp.h>
#include <nlohmann/json.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <tf2/buffer_core.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "mgg_ros/conversions.h"
#include "mgg_ros/local_events.h"
#include "mgg_ros/local_path_conversions.h"
#include "mgg_ros/param_loader.h"
#include "mgg_ros/scan_input.h"

namespace mgg {
namespace {
namespace fs = std::filesystem;
using nlohmann::json;
using Clock = std::chrono::steady_clock;
using Cloud = sensor_msgs::msg::PointCloud2;
using Odom = nav_msgs::msg::Odometry;
using TF = tf2_msgs::msg::TFMessage;
using Guidance = mgg_msgs::msg::GlobalGuidance;
using Feedback = mgg_msgs::msg::LocalPathFeedback;
using Zones = geometry_msgs::msg::PoseArray;
using Message = std::variant<Cloud, Odom, TF, Guidance, Feedback, Zones>;

void require(bool ok, const std::string& why) {
  if (!ok) throw std::runtime_error(why);
}
std::string read(const fs::path& file) {
  std::ifstream in(file, std::ios::binary);
  require(bool(in), "cannot read " + file.string());
  std::ostringstream bytes;
  bytes << in.rdbuf();
  require(!in.bad(), "cannot read " + file.string());
  return bytes.str();
}
std::string digest(const std::string& bytes) {
  std::array<unsigned char, EVP_MAX_MD_SIZE> hash{};
  unsigned int size = 0;
  require(EVP_Digest(bytes.data(), bytes.size(), hash.data(), &size,
                     EVP_sha256(), nullptr) == 1, "SHA-256 failed");
  std::ostringstream out;
  for (unsigned int i = 0; i < size; ++i)
    out << std::hex << std::setfill('0') << std::setw(2) << unsigned(hash[i]);
  return out.str();
}
std::string fileDigest(const fs::path& file) {
  std::ifstream in(file, std::ios::binary);
  require(bool(in), "cannot read " + file.string());
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(
      EVP_MD_CTX_new(), EVP_MD_CTX_free);
  require(context && EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) == 1,
          "SHA-256 initialization failed");
  std::array<char, 65536> bytes{};
  while (in) {
    in.read(bytes.data(), bytes.size());
    require(EVP_DigestUpdate(context.get(), bytes.data(), in.gcount()) == 1,
            "SHA-256 update failed");
  }
  require(in.eof(), "cannot read " + file.string());
  std::array<unsigned char, EVP_MAX_MD_SIZE> hash{};
  unsigned int size = 0;
  require(EVP_DigestFinal_ex(context.get(), hash.data(), &size) == 1,
          "SHA-256 finalization failed");
  std::ostringstream out;
  for (unsigned int i = 0; i < size; ++i)
    out << std::hex << std::setfill('0') << std::setw(2) << unsigned(hash[i]);
  return out.str();
}
fs::path inputPath(const fs::path& root, const std::string& name) {
  const fs::path relative(name);
  require(!name.empty() && !relative.is_absolute() && name.find('\\') == name.npos,
          "invalid input path: " + name);
  fs::path prefix = root;
  for (const auto& part : relative) {
    require(part != "..", "input escapes bundle: " + name);
    prefix /= part;
    require(!fs::is_symlink(fs::symlink_status(prefix)),
            "symlink in input: " + name);
  }
  require(fs::is_regular_file(prefix), "missing input: " + name);
  return prefix;
}
double milliseconds(Clock::duration time) {
  return std::chrono::duration<double, std::milli>(time).count();
}
struct BagInput {
  double time;
  std::string topic;
  Message message;
};
struct Event {
  double time;
  std::size_t line;
  json value;
  std::optional<RecordedLocalRequest> request;
};
struct Bundle {
  json manifest;
  json consumed_inputs = json::array(), hash_only_inputs = json::array();
  std::vector<BagInput> bag;
  std::vector<Event> events;
  LocalPlanningParams params;
  std::string params_hash;
  // Identity metadata only: pre-indexing never exposes future map inputs.
  std::map<std::pair<std::string, std::string>, std::set<std::uint64_t>> epochs;
};

template <typename T>
Message decode(const rosbag2_storage::SerializedBagMessage& message) {
  rclcpp::SerializedMessage serialized(*message.serialized_data);
  T value;
  rclcpp::Serialization<T>().deserialize_message(&serialized, &value);
  return value;
}

LocalPlanningParams parameters(const fs::path& file, const std::string& robot) {
  require(!file.empty(), "missing local planner parameters: --local-params FILE required");
  require(fs::is_regular_file(file), "missing local parameters: " + file.string());
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context).use_global_arguments(false)
      .automatically_declare_parameters_from_overrides(true)
      .start_parameter_services(false).start_parameter_event_publisher(false)
      .arguments({"--ros-args", "--params-file", file.string()});
  auto node = std::make_shared<rclcpp::Node>(
      "mggplanner_node", "/" + robot + "/mgg/local_planner", options);
  mgg_ros::ParamLoader loader(node.get());
  LocalPlanningParams p;
  // Refuse a YAML for a different node rather than accepting struct defaults.
  require(node->has_parameter("RobotParams.type") &&
              node->has_parameter("RobotParams.size"),
          "local parameters lack RobotParams for mggplanner_node");
  require(mgg_ros::loadRobotParams(loader, "RobotParams", p.robot), "invalid RobotParams");
  require(mgg_ros::loadPlanningParams(loader, "PlanningParams", p.planning),
          "invalid PlanningParams");
  std::vector<std::string> sensors;
  if (loader.get("SensorParams/sensor_list", sensors) && !sensors.empty())
    require(mgg_ros::loadSensorParams(loader, "SensorParams/" + sensors.front(), p.sensor),
            "invalid SensorParams");
  loader.get("local_map/resolution", p.window.resolution);
  loader.get("ground_recheck_s", p.ground_recheck_s);
  p.epoch = 1;  // Recorded path identities are translated, never guessed.
  node.reset();
  context->shutdown("parameters loaded");
  return p;
}

Bundle load(const fs::path& manifest_path, const fs::path& params_path) {
  Bundle b;
  b.manifest = json::parse(read(manifest_path));
  const auto& j = b.manifest;
  for (const char* field : {"version", "run", "robot", "robot_type", "architecture",
                             "session_id", "inputs", "config", "scenarios", "git"})
    require(j.contains(field), std::string("manifest missing ") + field);
  require(j.at("inputs").is_array() && !j.at("inputs").empty() &&
              j.at("scenarios").is_array(), "invalid manifest inputs/scenarios");
  require(j.at("version") == 1 && j.at("robot_type") == "ground" &&
          j.at("architecture") == "v2", "expected ground v2 manifest v1");
  const auto root = fs::absolute(manifest_path).parent_path();
  const std::string robot = j.at("robot");
  b.params = parameters(params_path, robot);
  b.params_hash = fileDigest(params_path);
  std::set<std::string> names;
  std::set<fs::path> opened_inputs;
  std::vector<fs::path> bags;
  fs::path event_file;
  for (const auto& entry : j.at("inputs")) {
    const std::string name = entry.at("path"), kind = entry.at("kind");
    require(names.insert(name).second, "duplicate input " + name);
    const fs::path file = inputPath(root, name);
    require(fileDigest(file) == entry.at("sha256"), "sha256 mismatch: " + name);
    require(kind == "bag" || kind == "events" || kind == "map_product",
            "unknown input kind: " + kind);
    if (kind == "bag" && file.extension() == ".mcap") bags.push_back(file);
    if (kind == "events") {
      require(event_file.empty(), "multiple events inputs");
      event_file = file;
    }
  }
  require(!event_file.empty() && !bags.empty(), "missing events or MCAP bag input");
  // Open hash-verified MCAP files directly: metadata must never cause a reader
  // to follow an unlisted (and therefore unverified) storage file.
  std::sort(bags.begin(), bags.end());
  const std::string prefix = "/" + robot + "/";
  std::map<std::string, std::string> types{
      {prefix + "scan/points", "sensor_msgs/msg/PointCloud2"},
      {prefix + "odom", "nav_msgs/msg/Odometry"},
      {prefix + "tf", "tf2_msgs/msg/TFMessage"},
      {prefix + "tf_static", "tf2_msgs/msg/TFMessage"},
      {prefix + "mgg/global_guidance", "mgg_msgs/msg/GlobalGuidance"},
      {prefix + "mgg/no_go_zones", "geometry_msgs/msg/PoseArray"},
      {prefix + "mgg/local_planner/local_path_feedback", "mgg_msgs/msg/LocalPathFeedback"}};
  bool scans = false, odometry = false;
  for (const auto& file : bags) {
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage;
    storage.uri = file.string(); storage.storage_id = "mcap";
    reader.open(storage);
    opened_inputs.insert(file);
    for (const auto& topic : reader.get_all_topics_and_types()) {
      const auto it = types.find(topic.name);
      if (it != types.end()) require(topic.type == it->second,
                                    "wrong message type for " + topic.name);
    }
    while (reader.has_next()) {
      const auto m = reader.read_next();
      const auto it = types.find(m->topic_name);
      if (it == types.end()) continue;  // Other recorder topics are not local inputs.
      Message value;
      if (it->second == "sensor_msgs/msg/PointCloud2") { value = decode<Cloud>(*m); scans = true; }
      else if (it->second == "nav_msgs/msg/Odometry") { value = decode<Odom>(*m); odometry = true; }
      else if (it->second == "tf2_msgs/msg/TFMessage") value = decode<TF>(*m);
      else if (it->second == "mgg_msgs/msg/GlobalGuidance") value = decode<Guidance>(*m);
      else if (it->second == "mgg_msgs/msg/LocalPathFeedback") value = decode<Feedback>(*m);
      else value = decode<Zones>(*m);
      b.bag.push_back({m->recv_timestamp * 1e-9, m->topic_name, std::move(value)});
    }
  }
  require(scans && odometry, "missing scan/points or odom messages");
  std::stable_sort(b.bag.begin(), b.bag.end(),
                   [](const auto& a, const auto& c) { return a.time < c.time; });
  std::ifstream stream(event_file);
  require(bool(stream), "cannot read " + event_file.string());
  opened_inputs.insert(event_file);
  std::vector<json> requests;
  double previous = -std::numeric_limits<double>::infinity();
  std::size_t line = 0, cycles = 0;
  for (std::string text; std::getline(stream, text); ++line) {
    auto e = json::parse(text);
    const double time = e.at("sim_s").get<double>();
    require(std::isfinite(time) && time >= previous, "events not in exported time order");
    previous = time;
    require(e.at("robot_id") == robot, "event belongs to another robot");
    const std::string kind = e.at("kind");
    if (kind == "local_request") requests.push_back(e.at("payload"));
    if (kind == "local_plan") {
      ++cycles;
      const auto& p = e.at("payload");
      if (p.contains("epoch"))
        b.epochs[{e.at("boot").get<std::string>(), p.at("session_id").get<std::string>()}]
            .insert(p.at("epoch").get<std::uint64_t>());
    }
    b.events.push_back({time, line, std::move(e), std::nullopt});
  }
  require(cycles > 0 && !requests.empty(), "missing local_plan or local_request events");
  const auto assembled = recordedLocalRequests(requests);
  std::map<std::pair<std::string, std::string>, RecordedLocalRequest> by_id;
  for (const auto& r : assembled)
    by_id.emplace(std::make_pair(r.request.session_id, r.request.request_id), r);
  for (auto& e : b.events) {
    if (e.value.at("kind") != "local_request") continue;
    const auto& p = e.value.at("payload");
    const auto key = std::make_pair(p.at("session_id").get<std::string>(),
                                  p.at("request_id").get<std::string>());
    const auto found = by_id.find(key);
    if (found != by_id.end()) { e.request = found->second; by_id.erase(found); }
  }
  for (const auto& window : j.at("scenarios")) {
    const double start = window.at("start_s"), end = window.at("end_s");
    require(std::isfinite(start) && std::isfinite(end) && start <= end &&
            window.at("tag").is_string(), "invalid scenario window");
  }
  // Detect any file changed during loading, including the explicit params.
  for (const auto& entry : j.at("inputs")) {
    const auto file = inputPath(root, entry.at("path"));
    require(fileDigest(file) == entry.at("sha256"),
            "input changed during loading: " + entry.at("path").get<std::string>());
    // Report actual reader/stream opens separately from integrity-only reads.
    // Bag metadata and unused map products must not claim replay consumption.
    (opened_inputs.count(file) ? b.consumed_inputs : b.hash_only_inputs)
        .push_back(entry);
  }
  require(fileDigest(params_path) == b.params_hash, "local params changed during loading");
  return b;
}

json pathOutput(const std::optional<LocalPathPlan>& plan) {
  if (!plan) return nullptr;
  const auto& p = *plan;
  json poses = json::array();
  for (const auto& q : p.poses) poses.push_back({q.x(), q.y(), q.z(), q.w()});
  return {{"poses", poses}, {"reverse", p.reverse}, {"kind", static_cast<int>(p.kind)},
          {"prefix_length", p.prefix_length}, {"commit_length_m", p.commit_length_m},
          {"reaches_goal", p.reaches_goal}, {"sequence_id", p.sequence_id},
          {"extends_sequence_id", p.extends_sequence_id},
          {"splice_point", poses.at(p.prefix_length ? p.prefix_length - 1 : 0)}};
}

struct Replay {
  const Bundle& bundle;
  std::unique_ptr<LocalPlanningCore> core;
  tf2::BufferCore tf;
  std::optional<StateVec> base;
  std::string odom_frame, boot;
  using Identity = std::tuple<std::string, std::uint64_t, std::uint64_t>;
  std::map<Identity, std::uint64_t> publications;
  json sessions = json::array(), counts = json::object();
  std::size_t resets = 0, unmatched_idle_feedback = 0;
  std::size_t dropped_scans = 0, integrated_scans = 0, cycles_with_usable_scan = 0;
  std::size_t ignored_odometry = 0, ignored_guidance = 0, ignored_no_go_centres = 0;
  std::size_t ignored_stale_feedback = 0, callback_count = 0;
  bool core_has_scan = false;
  double accumulated_ms = 0;
  // One sim-clock tick can stamp a plan and the feedback it caused alike
  // (the recorder stamps callbacks with its latest /clock). Within a tick,
  // EXECUTING feedback waits for the local_plan event it names; later
  // feedback queues behind it to keep receipt order.
  double tick = std::numeric_limits<double>::quiet_NaN();
  std::set<Identity> tick_publications;  // recorded, not yet replayed
  std::deque<BagInput> deferred_feedback;

  explicit Replay(const Bundle& b) : bundle(b) { newCore(); }
  void newCore() {
    core_has_scan = false;
    core = std::make_unique<LocalPlanningCore>(bundle.params);
    core->setFrameLookup([this](const std::string& frame) -> std::optional<Eigen::Isometry3d> {
      if (odom_frame.empty()) return std::nullopt;
      if (frame == odom_frame) return Eigen::Isometry3d::Identity();
      try {
        const auto t = tf.lookupTransform(odom_frame, frame, tf2::TimePointZero);
        const auto& p = t.transform.translation;
        const auto& q = t.transform.rotation;
        Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
        result.translation() = Eigen::Vector3d(p.x, p.y, p.z);
        result.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
        return result;
      } catch (const tf2::TransformException&) { return std::nullopt; }
    });
  }
  void count(const std::string& name) { counts[name] = counts.value(name, 0u) + 1; }
  bool active() const {
    return !core->sessionId().empty() && core->mode() != LocalMode::kIdle;
  }
  void clearTiming() {
    accumulated_ms = 0;
    callback_count = 0;
  }
  /// Starts the tick of events[first], whose local_plan identities a
  /// same-tick feedback may wait for.
  void beginTick(const std::vector<Event>& events, std::size_t first) {
    tick = events[first].time;
    tick_publications.clear();
    for (auto i = first; i < events.size() && events[i].time == tick; ++i) {
      const auto& e = events[i].value;
      const auto& p = e.at("payload");
      if (e.at("kind") == "local_plan" && p.contains("sequence_id") &&
          p.contains("epoch"))
        tick_publications.insert({p.at("session_id").get<std::string>(),
                                  p.at("epoch").get<std::uint64_t>(),
                                  p.at("sequence_id").get<std::uint64_t>()});
    }
  }
  bool awaitsPublication(const BagInput& input) const {
    const auto* m = std::get_if<Feedback>(&input.message);
    return m && m->state == Feedback::EXECUTING &&
           tick_publications.count({m->session_id, m->epoch, m->sequence_id});
  }
  /// Delivers deferred feedback whose publication has replayed. At the end
  /// of the tick nothing waits: unmatched identities fail as usual.
  void releaseFeedback(bool end_of_tick) {
    if (end_of_tick) tick_publications.clear();
    while (!deferred_feedback.empty() &&
           !awaitsPublication(deferred_feedback.front())) {
      const auto input = std::move(deferred_feedback.front());
      deferred_feedback.pop_front();
      deliver(input);
    }
  }
  void apply(const BagInput& input) {
    if (std::holds_alternative<Feedback>(input.message) && input.time == tick &&
        (!deferred_feedback.empty() || awaitsPublication(input))) {
      deferred_feedback.push_back(input);
      return;
    }
    deliver(input);
  }
  void deliver(const BagInput& input) {
    const bool measured = active();
    const auto start = Clock::now();
    std::visit([&](const auto& m) { applyMessage(m, input.topic); }, input.message);
    // Integrate even while idle, as production does, but bill only callbacks
    // in the active cycle window, not pre-session or IDLE background work.
    if (measured) {
      accumulated_ms += milliseconds(Clock::now() - start);
      ++callback_count;
    }
    count(input.topic);
  }
  void applyMessage(const TF& m, const std::string& topic) {
    const bool fixed = topic == "/" + bundle.manifest.at("robot").get<std::string>() + "/tf_static";
    for (const auto& t : m.transforms)
      require(tf.setTransform(t, "replay", fixed), "invalid TF: " + t.child_frame_id);
  }
  void applyMessage(const Odom& m, const std::string&) {
    const auto pose = mgg_ros::fromPoseMsg(m.pose.pose);
    if (!pose.allFinite() || m.header.frame_id.empty() ||
        (!odom_frame.empty() && odom_frame != m.header.frame_id)) {
      ++ignored_odometry;
      return;
    }
    odom_frame = m.header.frame_id; base = pose;
    if (core->onOdometry(pose).everything) {
      ++resets;
      core_has_scan = false;
    }
  }
  void applyMessage(const Cloud& m, const std::string&) {
    if (!base || odom_frame.empty()) {
      ++dropped_scans;
      return;
    }
    const auto scan = scanInOdom(m, tf, odom_frame);
    if (!scan) {
      ++dropped_scans;
      return;
    }
    const auto dropped_before = core->droppedScans();
    core->onScan(*scan, *base);
    if (core->droppedScans() != dropped_before) {
      ++dropped_scans;
      return;
    }
    ++integrated_scans;
    core_has_scan = core_has_scan || !scan->points.empty();
  }
  void applyMessage(const Guidance& m, const std::string&) {
    try {
      core->setGuidance(fromGuidanceMsg(m));
    } catch (const std::invalid_argument&) {
      ++ignored_guidance;
    }
  }
  void applyMessage(const Zones& m, const std::string&) {
    std::vector<Eigen::Vector2d> centres;
    for (const auto& p : m.poses) {
      if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y)) {
        ++ignored_no_go_centres;
        continue;
      }
      centres.emplace_back(p.position.x, p.position.y);
    }
    const auto box = core->params().robot.getPlanningSize();
    NoGoZones zones;
    zones.set(std::move(centres), core->params().planning.no_go_radius_m +
              0.5 * std::max(box.x(), box.y()));
    core->setNoGoZones(zones, m.header.frame_id);
  }
  void applyMessage(const Feedback& m, const std::string&) {
    auto feedback = fromFeedbackMsg(m);
    // Preserve foreign-session rejection without translating into a live path.
    if (feedback.session_id != core->sessionId()) { core->onFeedback(feedback); return; }
    const auto current_epochs = bundle.epochs.find({boot, m.session_id});
    if (current_epochs == bundle.epochs.end() || !current_epochs->second.count(m.epoch)) {
      for (const auto& entry : bundle.epochs) {
        if (entry.first.first != boot && entry.first.second == m.session_id &&
            entry.second.count(m.epoch)) {
          // A known other process's epoch is foreign to the current core,
          // even when the adapter kept the same session across the restart.
          ++ignored_stale_feedback;
          return;
        }
      }
    }
    require(m.state == Feedback::EXECUTING || m.state == Feedback::IDLE,
            "unknown local_path_feedback state");
    if (feedback.executing || m.sequence_id != 0) {
      const auto it = publications.find({m.session_id, m.epoch, m.sequence_id});
      require(it != publications.end(), "unmatched feedback epoch/sequence for " + m.session_id);
      feedback.sequence_id = it->second;
    } else {
      const auto epochs = bundle.epochs.find({boot, m.session_id});
      if (epochs == bundle.epochs.end() || !epochs->second.count(m.epoch)) {
        // A pre-publication IDLE can name a service-response epoch absent
        // from telemetry. It cannot be translated safely: leave state alone.
        ++unmatched_idle_feedback;
        return;
      }
    }
    feedback.epoch = core->epoch();
    core->onFeedback(feedback);
  }
  void request(const Event& event) {
    const auto start = Clock::now();
    const std::string current_boot = event.value.at("boot");
    const auto& recorded = *event.request;
    if (!active() || recorded.request.mode == LocalMode::kIdle ||
        recorded.request.session_id != core->sessionId() || boot != current_boot)
      clearTiming();
    if (!boot.empty() && boot != current_boot) {
      newCore(); publications.clear(); ++resets;
      if (base) core->onOdometry(*base);
    }
    boot = current_boot;
    require(recorded.continuation == (recorded.request.session_id == core->sessionId()),
            "local_request continuation mismatch");
    const auto result = core->setMode(recorded.request);
    require(result.accepted, "local_request rejected: " + result.reason);
    if (active()) accumulated_ms += milliseconds(Clock::now() - start);
    sessions.push_back(core->sessionId());
    count("local_request");
  }
  json cycle(const Event& event) {
    const auto& p = event.value.at("payload");
    require(!core->sessionId().empty() && p.at("session_id") == core->sessionId() &&
            p.at("request_id") == core->requestId() && event.value.at("boot") == boot,
            "local_plan has no matching local_request");
    const auto start = Clock::now();
    const auto deadline = start + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(kLocalPlanningBudgetS + 2 * bundle.params.ground_recheck_s));
    const double progress = core->progress();
    if (core_has_scan) ++cycles_with_usable_scan;
    const auto result = core->plan(deadline);
    const auto invalidations = core->takeInvalidations();
    const double duration = accumulated_ms + milliseconds(Clock::now() - start);
    const auto measured_callbacks = callback_count;
    clearTiming();
    if (result.path && p.contains("sequence_id") && p.contains("epoch")) {
      const Identity key{core->sessionId(), p.at("epoch").get<std::uint64_t>(),
                         p.at("sequence_id").get<std::uint64_t>()};
      require(publications.emplace(key, result.path->sequence_id).second,
              "duplicate recorded path identity");
    }
    if (p.contains("sequence_id") && p.contains("epoch"))
      tick_publications.erase({p.at("session_id").get<std::string>(),
                               p.at("epoch").get<std::uint64_t>(),
                               p.at("sequence_id").get<std::uint64_t>()});
    const json path = pathOutput(result.path);
    const std::string status = statusName(result.status);
    json tags = json::array();
    for (const auto& w : bundle.manifest.at("scenarios"))
      if (event.time >= w.at("start_s").get<double>() && event.time <= w.at("end_s").get<double>() &&
          (!w.contains("session_id") || w.at("session_id").is_null() ||
           w.at("session_id") == core->sessionId()))
        tags.push_back(w.at("tag"));
    const auto occupancy = core->ground().occupancy();
    return {{"component", "local"}, {"measured_scope", "local_map_ground_invalidation_search"},
            {"session_id", core->sessionId()}, {"input_seq", event.line}, {"sim_s", event.time},
            {"map_revision", core->map().revision()}, {"duration_ms", duration},
            {"complete", result.checks_complete}, {"status", status},
            {"reason", result.reason},
            {"output_digest", digest(json({{"path", path}, {"status", status}}).dump())},
            {"path", path}, {"progress_m", progress}, {"tags", tags},
            {"callback_count", measured_callbacks},
            {"reset_count", resets}, {"invalidations", invalidations.size()},
            {"admitted_cells", std::count(occupancy.begin(), occupancy.end(), 0)}};
  }
};
}  // namespace

int runLocalReplay(const fs::path& manifest, const fs::path& output) {
  return runLocalReplay(manifest, output, {});
}
int runLocalReplay(const fs::path& manifest, const fs::path& output,
                   const fs::path& local_params) {
  try {
    const auto started = Clock::now();
    const Bundle b = load(manifest, local_params);
    const double load_ms = milliseconds(Clock::now() - started);
    Replay replay(b);
    std::vector<json> rows;
    std::size_t bag_index = 0;
    // Stable bag receipt order, then exported event order at a tie. Nothing
    // recorded later is visible to a cycle, including dynamic TF.
    for (std::size_t i = 0; i < b.events.size(); ++i) {
      const auto& event = b.events[i];
      if (i == 0 || b.events[i - 1].time != event.time) replay.beginTick(b.events, i);
      while (bag_index < b.bag.size() && b.bag[bag_index].time <= event.time)
        replay.apply(b.bag[bag_index++]);
      if (event.request) replay.request(event);
      if (event.value.at("kind") == "local_plan") rows.push_back(replay.cycle(event));
      replay.releaseFeedback(i + 1 == b.events.size() || b.events[i + 1].time != event.time);
    }
    // Consume trailing callbacks too, including drops and stale feedback,
    // without allowing a trailing scan to qualify unobserved planning cycles.
    while (bag_index < b.bag.size()) replay.apply(b.bag[bag_index++]);
    require(!rows.empty(), "no local planning cycles");
    require(replay.integrated_scans > 0, "no scan was integrated (missing usable scan/TF input)");
    require(replay.cycles_with_usable_scan > 0, "no planning cycle had a usable scan");
    const json summary = {{"summary", true}, {"component", "local"},
        {"load_ms", load_ms}, {"cycles", rows.size()}, {"failures", 0},
        {"runner_version", "mgg_local_replay/1"}, {"consumed_inputs", b.consumed_inputs},
        {"hash_only_inputs", b.hash_only_inputs},
        {"core_sessions", replay.sessions}, {"input_counts", replay.counts},
        {"unmatched_idle_feedback", replay.unmatched_idle_feedback},
        {"dropped_scans", replay.dropped_scans}, {"integrated_scans", replay.integrated_scans},
        {"cycles_with_usable_scan", replay.cycles_with_usable_scan},
        {"ignored_odometry", replay.ignored_odometry}, {"ignored_guidance", replay.ignored_guidance},
        {"ignored_no_go_centres", replay.ignored_no_go_centres},
        {"ignored_stale_feedback", replay.ignored_stale_feedback},
        {"local_params", fs::absolute(local_params).string()}, {"local_params_sha256", b.params_hash}};
    // Never expose a partially successful run as a C5 report.
    const fs::path temporary = output.string() + ".tmp";
    std::ofstream sink(temporary);
    require(bool(sink), "cannot write " + temporary.string());
    for (const auto& row : rows) sink << row.dump() << '\n';
    sink << summary.dump() << '\n';
    sink.close();
    require(bool(sink), "failed writing " + temporary.string());
    fs::rename(temporary, output);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "mgg_local_replay: " << error.what() << '\n';
    return 1;
  }
}
}  // namespace mgg
