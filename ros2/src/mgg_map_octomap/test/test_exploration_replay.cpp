// The legacy exploration replay runner (exploration_replay.cpp, C5): bundle
// inputs in recorded order behind their hashes, cold and reset sessions, and
// missing inputs.

#define MGG_EXPLORATION_REPLAY_NO_MAIN
#include "exploration_replay.cpp"

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <iterator>
#include <optional>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "mgg_map_octomap/native_mola_grid.h"

namespace {

namespace fs = std::filesystem;
namespace replay = mgg::exploration_replay;
using nlohmann::json;

std::atomic<int> sequence{0};

class BundleDir {
 public:
  BundleDir() {
    root = fs::temp_directory_path() /
           ("mgg-exploration-replay-" + std::to_string(::getpid()) + "-" +
            std::to_string(sequence++));
    fs::create_directories(root);
  }
  ~BundleDir() { fs::remove_all(root); }

  void write(const std::string& relative, const std::string& bytes) const {
    fs::create_directories((root / relative).parent_path());
    std::ofstream out(root / relative, std::ios::binary | std::ios::trunc);
    out << bytes;
  }

  json input(const std::string& relative, const std::string& kind) const {
    std::ifstream in(root / relative, std::ios::binary);
    std::stringstream bytes;
    bytes << in.rdbuf();
    return {{"path", relative}, {"sha256", replay::sha256Hex(bytes.str())},
            {"kind", kind}};
  }

  fs::path root;
};

json requestEvent(const std::string& robot, const std::string& request_id,
                  const std::string& session, double sim_s, double x) {
  return {{"v", 1}, {"kind", "local_request"}, {"robot_id", robot},
          {"stamp", sim_s}, {"sim_s", sim_s}, {"wall", 1000.0 + sim_s},
          {"boot", std::string(32, 'a')}, {"seq", 0},
          {"payload", {{"request_id", request_id}, {"session_id", session},
                       {"continuation", false}, {"stamp", sim_s},
                       {"pose", {{"x", x}, {"y", 0.0}, {"z", 0.0}, {"yaw", 0.0}}}}}};
}

// A ground bundle: a recorded events.jsonl whose requests are deliberately
// not in time order, and a map product. Returns the manifest path.
// `omit` names a product file that is written but not listed as an input.
fs::path writeBundle(const BundleDir& dir, bool map_product = true,
                     bool legacy_config = true, bool requests = true,
                     const std::string& omit = "") {
  std::string events =
      json({{"v", 1}, {"kind", "local_plan"}, {"robot_id", "robot_0"},
            {"stamp", 1.0}, {"sim_s", 1.0}, {"wall", 1001.0},
            {"boot", std::string(32, 'a')}, {"seq", 1},
            {"payload", {{"session_id", "s1"}}}}).dump() + "\n";
  if (requests) {
    events += requestEvent("robot_0", "3", "s1", 5.0, 0.0).dump() + "\n";
    events += requestEvent("robot_1", "9", "s1", 4.0, 0.0).dump() + "\n";
    events += requestEvent("robot_0", "1", "s1", 2.0, 0.4).dump() + "\n";
    events += requestEvent("robot_0", "2", "s2", 6.0, 0.8).dump() + "\n";
  }
  dir.write("events.jsonl", events);
  dir.write("bag/bag_0.mcap", "mcap");
  json inputs = json::array({dir.input("bag/bag_0.mcap", "bag"),
                             dir.input("events.jsonl", "events")});
  if (map_product) {
    dir.write("map/mola/source.json", "{}");
    dir.write("map/mola/index.json",
              json({{"artifacts", {{{"planner", {{"path", "components/native.sdpg"}}}}}}})
                  .dump());
    dir.write("map/mola/components/native.sdpg", "grid");
    for (const char* file : {"map/mola/source.json", "map/mola/index.json",
                             "map/mola/components/native.sdpg"}) {
      if (omit != file) inputs.push_back(dir.input(file, "map_product"));
    }
  }
  json config = {{"scenario", "configs/x.yaml"}};
  if (legacy_config) config["legacy"] = {{"request", {{"lattice_budget_ms", 250.0}}}};
  const json manifest = {
      {"version", 1}, {"run", "run"}, {"robot", "robot_0"},
      {"robot_type", "ground"}, {"architecture", "v2"}, {"session_id", nullptr},
      {"inputs", inputs}, {"config", config},
      {"scenarios", json::array()}, {"git", {{"swarmdeck", nullptr}, {"mgg", nullptr}}}};
  dir.write("manifest.json", manifest.dump());
  return dir.root / "manifest.json";
}

std::vector<std::string> requestIds(const replay::Bundle& bundle) {
  std::vector<std::string> ids;
  for (const auto& request : bundle.requests) ids.push_back(request.request_id);
  return ids;
}

int runMain(std::vector<std::string> args, std::string& errors) {
  std::ostringstream err;
  args.insert(args.begin(), "mgg_exploration_replay");
  const int code = replay::runMain(args, err);
  errors = err.str();
  return code;
}

// A flat floor 0.61 m under the lidar, as mgg_nav_bench --synthetic-floor,
// built in `map`; with `wall`, a 2 m high wall across x = 1.5..1.7 m.
void floorGrid(std::optional<mgg::NativeMolaGrid>& map, bool wall) {
  std::vector<mgg::NativeMolaGrid::Cell> occupied, free;
  std::vector<mgg::NativeMolaGrid::Surface> surfaces;
  for (int x = -60; x <= 60; ++x) {
    for (int y = -60; y <= 60; ++y) {
      occupied.push_back({x, y, -7});
      surfaces.push_back({{x, y, -7}, -0.61});
      const bool walled = wall && x >= 15 && x <= 16 && y >= -30 && y <= 30;
      for (int z = -6; z <= 12; ++z) {
        if (walled) {
          occupied.push_back({x, y, z});
        } else {
          free.push_back({x, y, z});
        }
      }
    }
  }
  // Emplacing again reuses the same storage: the changed map has the same
  // address as the one it replaces.
  map.emplace(0.1, std::move(occupied), std::move(free), std::move(surfaces));
}

std::vector<std::string> minus(const std::vector<std::string>& a,
                               const std::vector<std::string>& b) {
  std::vector<std::string> out;
  std::set_difference(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(out));
  return out;
}

replay::Request request(const std::string& session, const std::string& id, double x) {
  replay::Request r;
  r.session_id = session;
  r.request_id = id;
  r.pose = mgg::StateVec(x, 0.0, 0.0, 0.0);
  return r;
}

}  // namespace

TEST(ExplorationReplay, RecordedOrderAndHash) {
  BundleDir dir;
  const fs::path manifest = writeBundle(dir);
  const replay::Bundle bundle = replay::loadBundle(manifest);
  ASSERT_TRUE(bundle.accepted) << bundle.error;
  // The runner never re-sorts: requests 3, 1, 2 in their recorded lines,
  // another robot's request skipped.
  const std::vector<std::string> recorded_order{"3", "1", "2"};
  const std::vector<std::string> input_order = requestIds(bundle);
  EXPECT_EQ(input_order, recorded_order);
  ASSERT_EQ(bundle.requests.size(), 3u);
  EXPECT_EQ(bundle.requests[0].input_seq, 1u);
  EXPECT_EQ(bundle.requests[1].input_seq, 3u);
  EXPECT_EQ(bundle.requests[2].input_seq, 4u);
  EXPECT_DOUBLE_EQ(bundle.requests[1].sim_s, 2.0);
  EXPECT_DOUBLE_EQ(bundle.requests[1].pose.x(), 0.4);
  EXPECT_EQ(bundle.map_product, fs::path(dir.root / "map"));
  ASSERT_TRUE(bundle.legacy_config.has_value());

  dir.write("events.jsonl", "{}\n");
  const replay::Bundle tampered = replay::loadBundle(manifest);
  EXPECT_FALSE(tampered.accepted);
  EXPECT_NE(tampered.error.find("sha256"), std::string::npos) << tampered.error;

  BundleDir product_dir;
  const fs::path second = writeBundle(product_dir);
  product_dir.write("map/mola/index.json", "{\"changed\":1}");
  EXPECT_FALSE(replay::loadBundle(second).accepted);

  BundleDir escape_dir;
  const fs::path third = writeBundle(escape_dir);
  json data = json::parse(std::ifstream(third));
  data["inputs"][0]["path"] = "../elsewhere/bag_0.mcap";
  escape_dir.write("manifest.json", data.dump());
  const replay::Bundle escaping = replay::loadBundle(third);
  EXPECT_FALSE(escaping.accepted);
  EXPECT_NE(escaping.error.find("escapes"), std::string::npos) << escaping.error;

  // A directory symlink inside the bundle cannot reach outside it, even
  // with a matching hash.
  BundleDir link_dir, outside;
  const fs::path fourth = writeBundle(link_dir);
  outside.write("bag_0.mcap", "mcap");
  fs::create_directory_symlink(outside.root, link_dir.root / "linked");
  data = json::parse(std::ifstream(fourth));
  data["inputs"][0]["path"] = "linked/bag_0.mcap";
  link_dir.write("manifest.json", data.dump());
  const replay::Bundle linked = replay::loadBundle(fourth);
  EXPECT_FALSE(linked.accepted);
  EXPECT_NE(linked.error.find("symlink"), std::string::npos) << linked.error;
}

TEST(ExplorationReplay, ColdResetAndWholeWindowInvalidation) {
  // Session b plans after a wall went up inside the lattice window. A warm
  // runner that planned session a on the old map must give exactly the
  // cold runner's lattices, keeping no edge the wall invalidated.
  mgg::nav_bench::RecordedParams params;
  params.request_budget_ms = 0.0;      // no request budget: deterministic lattices
  params.lattice_budget_ms = 1.0e6;
  const std::vector<replay::Request> session_a{request("a", "1", 0.0),
                                               request("a", "2", 0.8)};
  const std::vector<replay::Request> session_b{request("b", "3", 0.0),
                                               request("b", "4", 0.8)};
  std::optional<mgg::NativeMolaGrid> map;
  const auto replayed = [&](const std::vector<replay::Request>& requests) {
    std::vector<replay::Cycle> cycles;
    for (const auto& r : requests) {
      cycles.push_back(replay::replayRequest(*map, params, r));
      EXPECT_EQ(cycles.back().status, "lattice");
      EXPECT_FALSE(cycles.back().edge_keys.empty());
    }
    return cycles;
  };
  const auto digests = [](const std::vector<replay::Cycle>& cycles) {
    std::vector<std::string> out;
    for (const auto& cycle : cycles) out.push_back(cycle.output_digest);
    return out;
  };

  floorGrid(map, /*wall=*/true);
  const std::vector<replay::Cycle> cold_cycles = replayed(session_b);
  const std::vector<std::string> cold = digests(cold_cycles);

  floorGrid(map, /*wall=*/false);
  const std::vector<replay::Cycle> before = replayed(session_a);
  floorGrid(map, /*wall=*/true);
  const std::vector<replay::Cycle> warm_cycles = replayed(session_b);
  const std::vector<std::string> warm_reset = digests(warm_cycles);
  EXPECT_EQ(cold, warm_reset);

  std::size_t invalidated_edges = 0, reused_invalid_edges = 0;
  for (std::size_t i = 0; i < before.size(); ++i) {
    // Edges of the old window the wall made invalid.
    const auto invalidated = minus(before[i].edge_keys, cold_cycles[i].edge_keys);
    invalidated_edges += invalidated.size();
    reused_invalid_edges +=
        invalidated.size() - minus(invalidated, warm_cycles[i].edge_keys).size();
    EXPECT_EQ(warm_cycles[i].edge_keys, cold_cycles[i].edge_keys);
  }
  EXPECT_GT(invalidated_edges, 0u);  // the wall did invalidate edges
  EXPECT_EQ(reused_invalid_edges, 0u);

  const json record = replay::cycleRecord(session_b[0], warm_cycles[0], 7);
  for (const char* field : {"component", "measured_scope", "session_id", "input_seq",
                            "sim_s", "map_revision", "duration_ms", "complete",
                            "status", "output_digest"}) {
    EXPECT_TRUE(record.contains(field)) << field;
  }
  EXPECT_EQ(record["component"], "legacy");
  EXPECT_EQ(record["measured_scope"], "legacy_lattice");
  EXPECT_EQ(record["complete"], false);
  EXPECT_EQ(record["map_revision"], 7);
}

TEST(ExplorationReplay, MissingInputFailsNamingIt) {
  std::string errors;
  {
    BundleDir dir;
    const fs::path manifest = writeBundle(dir, /*map_product=*/false);
    EXPECT_NE(runMain({"--manifest", manifest.string(), "--component", "legacy",
                       "--output", (dir.root / "out.jsonl").string()}, errors), 0);
    EXPECT_NE(errors.find("map_product"), std::string::npos) << errors;
    EXPECT_FALSE(fs::exists(dir.root / "out.jsonl"));
  }
  for (const std::string omitted : {"map/mola/index.json",
                                    "map/mola/components/native.sdpg"}) {
    // A product file the load reads must be a hashed manifest input.
    BundleDir dir;
    const fs::path manifest = writeBundle(dir, true, true, true, omitted);
    EXPECT_NE(runMain({"--manifest", manifest.string(), "--component", "legacy",
                       "--output", (dir.root / "out.jsonl").string()}, errors), 0);
    EXPECT_NE(errors.find("missing from the manifest: " + omitted.substr(4)),
              std::string::npos) << errors;
    EXPECT_FALSE(fs::exists(dir.root / "out.jsonl"));
  }
  {
    BundleDir dir;
    const fs::path manifest = writeBundle(dir, true, /*legacy_config=*/false);
    EXPECT_NE(runMain({"--manifest", manifest.string(), "--component", "legacy",
                       "--output", (dir.root / "out.jsonl").string()}, errors), 0);
    EXPECT_NE(errors.find("config.legacy"), std::string::npos) << errors;
  }
  {
    BundleDir dir;
    const fs::path manifest = writeBundle(dir, true, true, /*requests=*/false);
    EXPECT_NE(runMain({"--manifest", manifest.string(), "--component", "legacy",
                       "--output", (dir.root / "out.jsonl").string()}, errors), 0);
    EXPECT_NE(errors.find("local_request"), std::string::npos) << errors;
  }
  {
    // --map-product and --legacy-config supply what the manifest lacks; the
    // empty product then fails to load, naming it.
    BundleDir dir;
    const fs::path manifest = writeBundle(dir, false, false);
    dir.write("legacy.json", json({{"grid", {{"resolution", {0.4, 0.4, 0.1}}}}}).dump());
    fs::create_directories(dir.root / "product" / "mola");
    EXPECT_NE(runMain({"--manifest", manifest.string(), "--component", "legacy",
                       "--output", (dir.root / "out.jsonl").string(),
                       "--map-product", (dir.root / "product").string(),
                       "--legacy-config", (dir.root / "legacy.json").string()}, errors), 0);
    EXPECT_NE(errors.find("map product"), std::string::npos) << errors;
    EXPECT_EQ(errors.find("config.legacy"), std::string::npos) << errors;
  }
  {
    BundleDir dir;
    const fs::path manifest = writeBundle(dir, true, false);
    dir.write("legacy.json", json({{"planning", {{"no_such_field", 1.0}}}}).dump());
    EXPECT_NE(runMain({"--manifest", manifest.string(), "--component", "legacy",
                       "--output", (dir.root / "out.jsonl").string(),
                       "--legacy-config", (dir.root / "legacy.json").string()}, errors), 0);
    EXPECT_NE(errors.find("--legacy-config"), std::string::npos) << errors;
  }
  BundleDir dir;
  const fs::path manifest = writeBundle(dir);
  EXPECT_EQ(runMain({"--manifest", manifest.string(), "--component", "local",
                     "--output", (dir.root / "out.jsonl").string()}, errors), 2);
}
