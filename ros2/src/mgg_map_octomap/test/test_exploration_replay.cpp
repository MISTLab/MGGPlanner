// The legacy exploration replay runner (exploration_replay.cpp, C5): bundle
// inputs in recorded order behind their hashes, cold and reset sessions, and
// missing inputs.

#define MGG_EXPLORATION_REPLAY_NO_MAIN
#include "exploration_replay.cpp"

#include <unistd.h>

#include <atomic>
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
fs::path writeBundle(const BundleDir& dir, bool map_product = true,
                     bool legacy_config = true, bool requests = true) {
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
    dir.write("map/mola/index.json", "{}");
    inputs.push_back(dir.input("map/mola/source.json", "map_product"));
    inputs.push_back(dir.input("map/mola/index.json", "map_product"));
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

// A flat floor 0.61 m under the lidar, as mgg_nav_bench --synthetic-floor.
mgg::NativeMolaGrid floorGrid() {
  std::vector<mgg::NativeMolaGrid::Cell> occupied, free;
  std::vector<mgg::NativeMolaGrid::Surface> surfaces;
  for (int x = -60; x <= 60; ++x) {
    for (int y = -60; y <= 60; ++y) {
      occupied.push_back({x, y, -7});
      surfaces.push_back({{x, y, -7}, -0.61});
      for (int z = -6; z <= 12; ++z) free.push_back({x, y, z});
    }
  }
  return mgg::NativeMolaGrid(0.1, std::move(occupied), std::move(free),
                             std::move(surfaces));
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
}

TEST(ExplorationReplay, ColdResetAndWholeWindowInvalidation) {
  const mgg::NativeMolaGrid map = floorGrid();
  mgg::nav_bench::RecordedParams params;
  params.request_budget_ms = 0.0;      // no request budget: deterministic lattices
  params.lattice_budget_ms = 1.0e6;

  const std::vector<replay::Request> session_b{request("b", "3", 0.0),
                                               request("b", "4", 0.8)};
  using Result = std::vector<std::pair<std::string, std::size_t>>;
  const auto replayed = [&](replay::LegacyReplay& runner,
                            const std::vector<replay::Request>& requests) {
    Result result;
    for (const auto& r : requests) {
      const replay::Cycle cycle = runner.step(r);
      EXPECT_EQ(cycle.status, "lattice");
      result.emplace_back(cycle.output_digest, cycle.reused_edges);
    }
    return result;
  };

  replay::LegacyReplay cold_runner(map, params);
  const Result cold = replayed(cold_runner, session_b);
  ASSERT_EQ(cold.size(), 2u);
  EXPECT_EQ(cold[0].second, 0u);
  // The second request of the window overlaps the first one's lattice.
  EXPECT_GT(cold[1].second, 0u);

  replay::LegacyReplay warm_runner(map, params);
  replayed(warm_runner, {request("a", "1", 0.0), request("a", "2", 0.8)});
  const Result warm_reset = replayed(warm_runner, session_b);
  EXPECT_EQ(cold, warm_reset);
  const std::size_t reused_invalid_edges = warm_reset[0].second;
  EXPECT_EQ(reused_invalid_edges, 0u);

  // Without a new session the same pose reuses its window's edges.
  replay::LegacyReplay continuing(map, params);
  replayed(continuing, {request("a", "1", 0.0)});
  EXPECT_GT(replayed(continuing, {request("a", "2", 0.0)})[0].second, 0u);

  const json record = replay::cycleRecord(session_b[0], continuing.step(session_b[0]), 7);
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
