// mgg_exploration_replay: the legacy exploration lattice over a replay
// bundle's recorded requests, the timing baseline of the C5 replay protocol
// (SwarmDeck docs/operations/orin-exploration-replay.md). Baseline only: its
// cycles are never complete, so it never qualifies a planner's timing.
//
// Usage:
//   mgg_exploration_replay --manifest FILE --component legacy --output FILE
//                          [--map-product DIR] [--legacy-config FILE]
//
// The manifest (v1) must verify: every input relative to its directory,
// inside it with no symlink on the way, a regular file with its recorded
// SHA-256. Each `local_request` event of the manifest's robot in
// events.jsonl, in its recorded line order (never re-sorted), plans one
// exploration lattice at its `pose` {x, y, z, yaw} through nav_bench::run.
// Every lattice is planned from scratch on the map, so nothing outlives a
// session or a map change.
//
// The map is a MOLA planning product: the manifest's `map_product` inputs,
// which must include every file the product load reads (mola/source.json,
// mola/index.json and each planner grid the index names) and are hashed
// again after the load, or `--map-product DIR`, outside the manifest's
// hashes. The parameters are Botman's overlaid by the manifest's
// `config.legacy` or `--legacy-config FILE` (legacyParams()). Giving one
// both ways is an error, and so is missing either: nothing is replayed then.
//
// The output holds one C5 cycle record per request and the summary line,
// written only when every request was planned: a request that throws fails
// the run instead of recording a timing it does not have. `duration_ms` is
// the plan's own time (Outcome::total_ms); manifest verification, event
// parsing and the product load are `load_ms`.

#include <openssl/evp.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "nav_bench_scenarios.h"

namespace mgg {
namespace exploration_replay {

namespace fs = std::filesystem;
using nlohmann::json;

inline constexpr char kComponent[] = "legacy";
inline constexpr char kMeasuredScope[] = "legacy_lattice";
inline constexpr char kRunnerVersion[] = "mgg_exploration_replay/1";

inline std::string sha256Hex(const std::string& bytes) {
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int size = 0;
  if (EVP_Digest(bytes.data(), bytes.size(), digest.data(), &size, EVP_sha256(),
                 nullptr) != 1) {
    throw std::runtime_error("SHA-256 failed");
  }
  std::ostringstream text;
  text << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < size; ++i) {
    text << std::setw(2) << static_cast<unsigned int>(digest[i]);
  }
  return text.str();
}

inline bool readFile(const fs::path& path, std::string& bytes) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  std::stringstream buffer;
  buffer << in.rdbuf();
  bytes = buffer.str();
  return static_cast<bool>(in) || in.eof();
}

/// One recorded `local_request`.
struct Request {
  std::size_t input_seq = 0;  ///< 0-based line of events.jsonl
  double sim_s = 0.0;
  std::string session_id;
  std::string request_id;
  StateVec pose = StateVec::Zero();
};

/// A verified replay bundle.
struct Bundle {
  bool accepted = false;
  std::string error;
  std::string robot;
  std::string robot_type;
  std::vector<Request> requests;  ///< recorded order
  fs::path map_product;           ///< peer root; empty without map_product inputs
  /// The map_product inputs: path relative to the peer root -> SHA-256.
  std::map<std::string, std::string> map_product_files;
  std::optional<json> legacy_config;  ///< the manifest's config.legacy
};

namespace detail {

inline bool number(const json& value) {
  return value.is_number() && std::isfinite(value.get<double>());
}

inline Request parseRequest(const json& event, std::size_t line) {
  const json& payload = event.at("payload");
  Request request;
  request.input_seq = line;
  const json& time = event.contains("sim_s") ? event.at("sim_s") : event.at("stamp");
  if (!number(time)) throw std::runtime_error("no simulated time");
  request.sim_s = time.get<double>();
  const json& session = payload.at("session_id");
  if (!session.is_string() || session.get<std::string>().empty()) {
    throw std::runtime_error("invalid session_id");
  }
  request.session_id = session.get<std::string>();
  const json& id = payload.at("request_id");
  if (id.is_string() && !id.get<std::string>().empty()) {
    request.request_id = id.get<std::string>();
  } else if (id.is_number_unsigned()) {
    request.request_id = std::to_string(id.get<std::uint64_t>());
  } else {
    throw std::runtime_error("invalid request_id");
  }
  const json& pose = payload.at("pose");
  int axis = 0;
  for (const char* key : {"x", "y", "z", "yaw"}) {
    if (!pose.is_object() || !pose.contains(key) || !number(pose.at(key))) {
      throw std::runtime_error(std::string("pose lacks a finite ") + key);
    }
    request.pose[axis++] = pose.at(key).get<double>();
  }
  return request;
}

}  // namespace detail

/// The bundle of `manifest`, its inputs verified; `accepted` is false with
/// `error` set on any missing, escaping, unreadable or altered input.
inline Bundle loadBundle(const fs::path& manifest) {
  Bundle bundle;
  const auto fail = [&](const std::string& message) {
    bundle.accepted = false;
    bundle.error = message;
    bundle.requests.clear();
    return bundle;
  };
  std::string text;
  if (!readFile(manifest, text)) return fail("cannot read manifest " + manifest.string());
  try {
    const json data = json::parse(text);
    if (data.at("version") != 1) return fail("unsupported manifest version");
    bundle.robot = data.at("robot").get<std::string>();
    bundle.robot_type = data.at("robot_type").get<std::string>();
    const fs::path root = fs::absolute(manifest).parent_path();
    std::optional<fs::path> events;
    for (const json& input : data.at("inputs")) {
      const std::string name = input.at("path").get<std::string>();
      const std::string kind = input.at("kind").get<std::string>();
      const fs::path relative(name);
      bool escapes = name.empty() || relative.is_absolute() ||
                     name.find('\\') != std::string::npos;
      for (const auto& part : relative) escapes = escapes || part == "..";
      if (escapes) return fail("input path escapes the bundle: " + name);
      fs::path prefix = root;
      for (const auto& part : relative) {
        prefix /= part;
        if (fs::is_symlink(fs::symlink_status(prefix))) {
          return fail("input path escapes the bundle through a symlink: " + name);
        }
      }
      const fs::path file = root / relative;
      const fs::path canonical_root = fs::weakly_canonical(root);
      const fs::path canonical = fs::weakly_canonical(file);
      if (std::mismatch(canonical_root.begin(), canonical_root.end(), canonical.begin(),
                        canonical.end()).first != canonical_root.end()) {
        return fail("input path escapes the bundle: " + name);
      }
      const auto status = fs::symlink_status(file);
      if (!fs::is_regular_file(status)) return fail("input is not a regular file: " + name);
      std::string bytes;
      if (!readFile(file, bytes)) return fail("cannot read input " + name);
      if (sha256Hex(bytes) != input.at("sha256").get<std::string>()) {
        return fail("sha256 mismatch for " + name);
      }
      if (kind == "events") {
        if (events) return fail("the manifest has more than one events input");
        events = file;
      } else if (kind == "map_product") {
        // <peer root>/mola/<...>: every product file shares one peer root.
        fs::path peer_root;
        bool found = false;
        for (auto it = relative.begin(); it != relative.end(); ++it) {
          if (*it == "mola" && std::next(it) != relative.end()) {
            found = true;
            break;
          }
          peer_root /= *it;
        }
        if (!found) return fail("map_product input is not under <root>/mola/: " + name);
        if (!bundle.map_product.empty() && bundle.map_product != root / peer_root) {
          return fail("map_product inputs name more than one peer root");
        }
        bundle.map_product = root / peer_root;
        const fs::path in_product =
            peer_root.empty() ? relative : relative.lexically_relative(peer_root);
        bundle.map_product_files[in_product.lexically_normal().generic_string()] =
            input.at("sha256").get<std::string>();
      } else if (kind != "bag") {
        return fail("unknown input kind " + kind);
      }
    }
    if (!events) return fail("the manifest has no events input");
    const json& config = data.at("config");
    if (config.is_object() && config.contains("legacy")) {
      bundle.legacy_config = config.at("legacy");
    }
    std::ifstream lines(*events);
    std::string line;
    for (std::size_t number = 0; std::getline(lines, line); ++number) {
      json event;
      try {
        event = json::parse(line);
      } catch (const std::exception& e) {
        return fail("events.jsonl:" + std::to_string(number + 1) + ": " + e.what());
      }
      if (event.value("kind", "") != "local_request" ||
          event.value("robot_id", "") != bundle.robot) {
        continue;
      }
      try {
        bundle.requests.push_back(detail::parseRequest(event, number));
      } catch (const std::exception& e) {
        return fail("events.jsonl:" + std::to_string(number + 1) +
                    ": local_request: " + e.what());
      }
    }
  } catch (const std::exception& e) {
    return fail(std::string("malformed manifest: ") + e.what());
  }
  bundle.accepted = true;
  return bundle;
}

/// The product files a load of `bundle.map_product` reads: mola/source.json,
/// mola/index.json and every planner grid the index names. Empty with
/// `error` naming the first that is not a verified map_product input.
inline std::vector<std::string> productInputs(const Bundle& bundle, std::string& error) {
  std::vector<std::string> required{"mola/source.json", "mola/index.json"};
  for (const std::string& name : required) {
    if (!bundle.map_product_files.count(name)) {
      error = "map_product input missing from the manifest: " + name;
      return {};
    }
  }
  try {
    std::string text;
    if (!readFile(bundle.map_product / "mola" / "index.json", text)) {
      throw std::runtime_error("unreadable");
    }
    const json index = json::parse(text);
    for (const json& artifact : index.at("artifacts")) {
      if (!artifact.contains("planner")) continue;
      required.push_back("mola/" + artifact.at("planner").at("path").get<std::string>());
    }
  } catch (const std::exception& e) {
    error = std::string("map_product mola/index.json names no planner grids: ") + e.what();
    return {};
  }
  for (const std::string& name : required) {
    if (!bundle.map_product_files.count(fs::path(name).lexically_normal().generic_string())) {
      error = "map_product input missing from the manifest: " + name;
      return {};
    }
  }
  return required;
}

/// True when every map_product input still has its recorded SHA-256.
inline bool productUnchanged(const Bundle& bundle, std::string& error) {
  for (const auto& [name, digest] : bundle.map_product_files) {
    std::string bytes;
    if (!readFile(bundle.map_product / name, bytes) || sha256Hex(bytes) != digest) {
      error = "map_product input changed during the load: " + name;
      return false;
    }
  }
  return true;
}

/// Botman's parameters and the benchmark's product loading, overlaid by the
/// recorded `legacy` object: {"planning": {...}, "robot": {...}, "grid":
/// {...}, "request": {...}, "map": {...}}, each key one named below. Any
/// other key is an error.
inline bool legacyParams(const json& legacy, nav_bench::RecordedParams& params,
                         MolaMapConfig& map, std::string& error) {
  PlanningParams& p = params.planning;
  const std::map<std::string, double*> planning{
      {"edge_length_min", &p.edge_length_min},
      {"edge_length_max", &p.edge_length_max},
      {"edge_overshoot", &p.edge_overshoot},
      {"num_vertices_max", &p.num_vertices_max},
      {"num_edges_max", &p.num_edges_max},
      {"num_loops_cutoff", &p.num_loops_cutoff},
      {"num_loops_max", &p.num_loops_max},
      {"nearest_range", &p.nearest_range},
      {"nearest_range_z", &p.nearest_range_z},
      {"nearest_range_min", &p.nearest_range_min},
      {"nearest_range_max", &p.nearest_range_max},
      {"max_ground_height", &p.max_ground_height},
      {"max_step_height", &p.max_step_height},
      {"max_inclination", &p.max_inclination},
      {"max_footprint_tilt", &p.max_footprint_tilt},
      {"max_footprint_step", &p.max_footprint_step},
      {"path_clearance_margin", &p.path_clearance_margin},
      {"path_interpolation_distance", &p.path_interpolation_distance}};
  const auto vector3 = [](const json& value, Eigen::Vector3d& out) {
    if (!value.is_array() || value.size() != 3) return false;
    for (int i = 0; i < 3; ++i) {
      if (!detail::number(value[i])) return false;
      out[i] = value[i].get<double>();
    }
    return true;
  };
  try {
    if (!legacy.is_object()) {
      error = "legacy parameters must be an object";
      return false;
    }
    for (const auto& [section, values] : legacy.items()) {
      if (!values.is_object()) {
        error = section + " must be an object";
        return false;
      }
      for (const auto& [key, value] : values.items()) {
        const std::string field = section + "." + key;
        bool ok = false;
        if (section == "planning" && planning.count(key)) {
          ok = detail::number(value);
          if (ok) *planning.at(key) = value.get<double>();
        } else if (section == "robot" && (key == "size" || key == "size_extension")) {
          ok = vector3(value, key == "size" ? params.robot.size : params.robot.size_extension);
        } else if (section == "robot" &&
                   (key == "physical_size" || key == "physical_center_offset")) {
          Eigen::Vector3d v;
          ok = vector3(value, v);
          if (ok) (key == "physical_size" ? params.robot.physical_size
                                          : params.robot.physical_center_offset) = v;
        } else if (section == "grid" &&
                   (key == "min_val" || key == "max_val" || key == "resolution")) {
          ok = vector3(value, key == "min_val" ? params.grid.min_val
                             : key == "max_val" ? params.grid.max_val
                                                : params.grid.resolution);
        } else if (section == "request" && key == "allow_unknown_body") {
          ok = value.is_boolean();
          if (ok) params.allow_unknown_body = value.get<bool>();
        } else if (section == "request" &&
                   (key == "request_budget_ms" || key == "lattice_budget_ms")) {
          ok = detail::number(value) && value.get<double>() >= 0.0;
          if (ok) (key == "request_budget_ms" ? params.request_budget_ms
                                              : params.lattice_budget_ms) = value.get<double>();
        } else if (section == "request" && key == "sensor_height") {
          ok = value.is_null() || (detail::number(value) && value.get<double>() > 0.0);
          if (ok) params.sensor_height = value.is_null() ? std::nullopt
                                         : std::optional<double>(value.get<double>());
        } else if (section == "map" && key == "snapshot_ttl_sec") {
          ok = detail::number(value) && value.get<double>() > 0.0;
          if (ok) map.snapshot_ttl_sec = value.get<double>();
        } else if (section == "map" && key == "max_load_time_ms") {
          ok = detail::number(value) && value.get<double>() > 0.0;
          if (ok) map.max_load_time = std::chrono::milliseconds(
              static_cast<std::int64_t>(value.get<double>()));
        } else {
          error = "unknown field " + field;
          return false;
        }
        if (!ok) {
          error = "invalid value for " + field;
          return false;
        }
      }
    }
  } catch (const std::exception& e) {
    error = e.what();
    return false;
  }
  return true;
}

/// One replayed request.
struct Cycle {
  std::string status;
  double duration_ms = 0.0;
  /// The lattice's edges (nav_bench::latticeEdgeKeys) and their SHA-256.
  std::vector<std::string> edge_keys;
  std::string output_digest;
};

/// The legacy exploration lattice of `request` on `map`, planned from
/// scratch as the planner node does for every exploration request.
inline Cycle replayRequest(const MapInterface& map, nav_bench::RecordedParams params,
                           const Request& request) {
  params.collect_lattice_edges = true;
  nav_bench::Scenario scenario;
  scenario.name = "replay_" + request.request_id;
  scenario.navigate = false;
  scenario.start = request.pose;
  const nav_bench::Outcome out = nav_bench::run(map, scenario, params);
  Cycle cycle;
  cycle.duration_ms = out.total_ms;
  cycle.status = !out.reason.empty()        ? "interrupted"
                 : out.lattice.hit_deadline ? "deadline"
                 : out.routed               ? "lattice"
                                            : "no_lattice";
  cycle.edge_keys = out.lattice_edge_keys;
  std::string keys;
  for (const std::string& key : cycle.edge_keys) keys += key + "\n";
  cycle.output_digest = sha256Hex(keys);
  return cycle;
}

/// The C5 cycle record of `cycle`.
inline json cycleRecord(const Request& request, const Cycle& cycle,
                        std::uint64_t map_revision) {
  return {{"component", kComponent},       {"measured_scope", kMeasuredScope},
          {"session_id", request.session_id}, {"input_seq", request.input_seq},
          {"sim_s", request.sim_s},        {"map_revision", map_revision},
          {"duration_ms", cycle.duration_ms}, {"complete", false},
          {"status", cycle.status},        {"output_digest", cycle.output_digest}};
}

/// The runner; `args[0]` is the program name. Returns the exit code: 2 for
/// usage, 1 when an input is missing or invalid (naming it), else 0.
inline int runMain(const std::vector<std::string>& args, std::ostream& err) {
  using Clock = std::chrono::steady_clock;
  const auto started = Clock::now();
  std::map<std::string, std::string> options;
  for (std::size_t i = 1; i < args.size(); i += 2) {
    const std::string& key = args[i];
    if (i + 1 >= args.size() ||
        (key != "--manifest" && key != "--component" && key != "--output" &&
         key != "--map-product" && key != "--legacy-config") ||
        options.count(key)) {
      err << "usage: mgg_exploration_replay --manifest FILE --component legacy "
             "--output FILE [--map-product DIR] [--legacy-config FILE]\n";
      return 2;
    }
    options[key] = args[i + 1];
  }
  if (!options.count("--manifest") || !options.count("--output") ||
      options["--component"] != kComponent) {
    err << "mgg_exploration_replay: needs --manifest, --output and --component "
        << kComponent << "\n";
    return 2;
  }
  const Bundle bundle = loadBundle(options["--manifest"]);
  if (!bundle.accepted) {
    err << "mgg_exploration_replay: " << bundle.error << "\n";
    return 1;
  }
  if (bundle.robot_type != "ground") {
    err << "mgg_exploration_replay: the legacy lattice replays ground robots, not "
        << bundle.robot_type << "\n";
    return 1;
  }
  fs::path product = bundle.map_product;
  std::string error;
  if (!product.empty() && productInputs(bundle, error).empty()) {
    err << "mgg_exploration_replay: " << error << "\n";
    return 1;
  }
  if (options.count("--map-product")) {
    if (!product.empty()) {
      err << "mgg_exploration_replay: the manifest has map_product inputs; "
             "--map-product would replace them\n";
      return 1;
    }
    product = options["--map-product"];
  }
  if (product.empty()) {
    err << "mgg_exploration_replay: missing input map_product: the manifest has "
           "none and --map-product was not given\n";
    return 1;
  }
  std::optional<json> legacy = bundle.legacy_config;
  std::string source = "config.legacy";
  if (options.count("--legacy-config")) {
    if (legacy) {
      err << "mgg_exploration_replay: the manifest has config.legacy; "
             "--legacy-config would replace it\n";
      return 1;
    }
    source = "--legacy-config";
    std::string text;
    if (!readFile(options["--legacy-config"], text)) {
      err << "mgg_exploration_replay: cannot read --legacy-config "
          << options["--legacy-config"] << "\n";
      return 1;
    }
    try {
      legacy = json::parse(text);
    } catch (const std::exception& e) {
      err << "mgg_exploration_replay: --legacy-config: " << e.what() << "\n";
      return 1;
    }
  }
  if (!legacy) {
    err << "mgg_exploration_replay: missing input config.legacy: the manifest has "
           "no recorded legacy parameters and --legacy-config was not given\n";
    return 1;
  }
  nav_bench::RecordedParams params;
  MolaMapConfig map_config = nav_bench::benchProductConfig();
  if (!legacyParams(*legacy, params, map_config, error)) {
    err << "mgg_exploration_replay: " << source << ": " << error << "\n";
    return 1;
  }
  if (bundle.requests.empty()) {
    err << "mgg_exploration_replay: missing input: no local_request events of "
        << bundle.robot << " in the bundle\n";
    return 1;
  }
  std::unique_ptr<MolaMap> map;
  try {
    map = nav_bench::loadProduct(product.string(), map_config, error);
  } catch (const std::exception& e) {
    error = e.what();
  }
  if (!map) {
    err << "mgg_exploration_replay: cannot load map product " << product.string()
        << ": " << error << "\n";
    return 1;
  }
  if (!bundle.map_product.empty() && !productUnchanged(bundle, error)) {
    err << "mgg_exploration_replay: " << error << "\n";
    return 1;
  }
  const auto lease = map->acquireReadLease();
  std::uint64_t revision = 0;
  try {
    std::string text;
    readFile(product / "mola" / "source.json", text);
    revision = json::parse(text)
                   .at("manifests").at(0).at("graph_revision").at("revision")
                   .get<std::uint64_t>();
  } catch (const std::exception& e) {
    err << "mgg_exploration_replay: map product has no graph revision: " << e.what()
        << "\n";
    return 1;
  }
  const double load_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - started).count();

  std::vector<std::string> lines;
  for (const Request& request : bundle.requests) {
    Cycle cycle;
    try {
      cycle = replayRequest(*map, params, request);
    } catch (const std::exception& e) {
      err << "mgg_exploration_replay: request " << request.request_id
          << " (events.jsonl line " << request.input_seq + 1
          << ") was not measured: " << e.what() << "\n";
      return 1;
    }
    lines.push_back(cycleRecord(request, cycle, revision).dump());
  }
  lines.push_back(json({{"summary", true}, {"component", kComponent},
                        {"load_ms", load_ms}, {"cycles", bundle.requests.size()},
                        {"failures", 0}, {"runner_version", kRunnerVersion}})
                      .dump());
  std::ofstream out(options["--output"], std::ios::trunc);
  for (const std::string& line : lines) out << line << "\n";
  out.flush();
  if (!out) {
    err << "mgg_exploration_replay: writing " << options["--output"] << " failed\n";
    return 1;
  }
  return 0;
}

}  // namespace exploration_replay
}  // namespace mgg

#ifndef MGG_EXPLORATION_REPLAY_NO_MAIN
int main(int argc, char** argv) {
  return mgg::exploration_replay::runMain(
      std::vector<std::string>(argv, argv + argc), std::cerr);
}
#endif
