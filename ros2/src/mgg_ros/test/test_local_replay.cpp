#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unistd.h>

#include <gtest/gtest.h>
#include <openssl/evp.h>
#include <nlohmann/json.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include "local_planner_scene.h"
#include "mgg_ros/conversions.h"
#include "mgg_ros/local_events.h"
#include "mgg_ros/local_path_conversions.h"
#include "mgg_ros/local_replay.h"

namespace {
using nlohmann::json;
namespace fs = std::filesystem;

std::string hashFile(const fs::path& file) {
  std::ifstream stream(file, std::ios::binary);
  std::string bytes{std::istreambuf_iterator<char>(stream), {}};
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int size;
  EVP_Digest(bytes.data(), bytes.size(), digest, &size, EVP_sha256(), nullptr);
  std::ostringstream out;
  for (unsigned int i = 0; i < size; ++i)
    out << std::hex << std::setfill('0') << std::setw(2) << unsigned(digest[i]);
  return out.str();
}

enum class Extra {
  None, KnownIdle, UnknownIdle, GuidanceAndZones, MissingTf,
  DroppedInputs, IdleGap, RestartFeedback, LateScans, SameTickFeedback,
  SameTickUnmatchedFeedback, UnreproducedFeedback
};

struct Bundle {
  fs::path root, manifest, params, output;
  json events = json::array(), data;
  mgg_test::Corridor corridor;
  rosbag2_cpp::Writer writer;
  static int serial;

  explicit Bundle(bool feedback = true, bool wall = true, Extra extra = Extra::None) {
    root = fs::temp_directory_path() /
        ("local-replay-" + std::to_string(getpid()) + "-" + std::to_string(++serial));
    fs::create_directories(root);
    manifest = root / "manifest.json";
    params = root / "params.yaml";
    output = root / "out.jsonl";
    std::ofstream(params) << R"(/**/mggplanner_node:
  ros__parameters:
    RobotParams:
      type: kGroundRobot
      size: [1.0, 0.6, 0.4]
      size_extension: [0.0, 0.0, 0.0]
      size_extension_min: [0.0, 0.0, 0.0]
      safety_extension: [0.0, 0.0, 0.0]
      bound_mode: kExactBound
    PlanningParams:
      max_ground_height: 0.5
      max_inclination: 0.7
      max_negative_inclination: 0.5
      min_observed_ground_fraction: 0.75
      v_max: 0.6
      no_go_radius_m: 0.3
    SensorParams:
      sensor_list: [lidar]
      lidar:
        type: kLidar
        max_range: 8.0
        fov: [6.28, 0.2]
        resolution: [0.4, 0.2]
)";
    rosbag2_storage::StorageOptions storage;
    storage.uri = (root / "bag").string();
    storage.storage_id = "mcap";
    writer.open(storage);
    tf2_msgs::msg::TFMessage tf;
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "lidar";
    transform.transform.rotation.w = 1;
    const auto origin = mgg_test::lidarOrigin(mgg_test::basePose(0.1, 0.1));
    transform.transform.translation.x = origin.x();
    transform.transform.translation.y = origin.y();
    transform.transform.translation.z = origin.z();
    tf.transforms.push_back(transform);
    if (extra != Extra::MissingTf) write(tf, "tf_static", 0.9);
    if (extra == Extra::DroppedInputs) scan(0.8);
    odometry(1, 0.1);
    for (int i = 0; i < 12; ++i)
      scan((extra == Extra::LateScans ? 5.1 : 1.01) + i * 0.05);
    if (extra == Extra::DroppedInputs) {
      scan(1.65, "unavailable_sensor");
      nav_msgs::msg::Odometry ignored;
      ignored.header.frame_id = "other_odom";
      ignored.pose.pose = mgg_ros::toPoseMsg(mgg_test::basePose(50, 0));
      write(ignored, "odom", 1.7);
      ignored.header.frame_id = "odom";
      ignored.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
      write(ignored, "odom", 1.71);
      mgg_msgs::msg::GlobalGuidance invalid;
      invalid.kind = 255;
      write(invalid, "mgg/global_guidance", 1.72);
      geometry_msgs::msg::PoseArray zones;
      zones.header.frame_id = "odom";
      geometry_msgs::msg::Pose centre;
      centre.position.x = std::numeric_limits<double>::infinity();
      zones.poses.push_back(centre);
      write(zones, "mgg/no_go_zones", 1.73);
    }
    request("s1", "r1", 1.9);
    if (extra == Extra::GuidanceAndZones) {
      events.back()["payload"]["mode"] = "explore";
      mgg_msgs::msg::GlobalGuidance guidance;
      guidance.header.frame_id = "odom";
      guidance.session_id = "s1";
      guidance.sequence_id = 10;
      guidance.kind = mgg_msgs::msg::GlobalGuidance::TARGET;
      guidance.target.x = 6.1; guidance.target.y = 0.1; guidance.target.z = 0.4;
      write(guidance, "mgg/global_guidance", 1.8);
      geometry_msgs::msg::PoseArray zones;
      zones.header.frame_id = "odom";
      geometry_msgs::msg::Pose centre;
      centre.position.x = 3.1; centre.position.y = 0.1;
      zones.poses.push_back(centre);
      write(zones, "mgg/no_go_zones", 2.6);
    }
    cycle("s1", "r1", 2, 1);
    if (feedback) {
      mgg_msgs::msg::LocalPathFeedback fb;
      fb.session_id = "s1";
      fb.epoch = 777;
      // Same tick: the recorder stamps callbacks with its latest /clock.
      const bool same_tick = extra == Extra::SameTickFeedback ||
                             extra == Extra::SameTickUnmatchedFeedback;
      fb.sequence_id = extra == Extra::SameTickUnmatchedFeedback ? 2 : 1;
      fb.state = "executing";
      fb.progress_m = 0.5;
      fb.speed_mps = 0.3;
      fb.deceleration_mps2 = 0.5;
      fb.latency_s = 0.2;
      fb.speed_cap_mps = 0.6;
      write(fb, "mgg/local_planner/local_path_feedback", same_tick ? 2.0 : 2.5);
    }
    if (extra == Extra::KnownIdle || extra == Extra::UnknownIdle) {
      mgg_msgs::msg::LocalPathFeedback idle;
      idle.session_id = "s1";
      idle.epoch = extra == Extra::KnownIdle ? 777 : 999;
      idle.state = "idle";
      write(idle, "mgg/local_planner/local_path_feedback", 1.95);
      write(idle, "mgg/local_planner/local_path_feedback", 2.6);
    }
    cycle("s1", "r1", 3, 2);
    if (wall) {
      corridor.obstacle = Eigen::AlignedBox3d(Eigen::Vector3d(3.1, -2.5, -0.1), Eigen::Vector3d(3.5, 2.5, 1.0));
      for (int i = 0; i < 4; ++i) scan(3.1 + i * 0.1);
    }
    cycle("s1", "r1", 4, 3);
    if (extra == Extra::UnreproducedFeedback) {
      // A live path the replay does not reproduce (here: its cycle replays
      // in IDLE), then the executor's report on it.
      request("s1", "idle", 4.1);
      events.back()["payload"]["mode"] = "idle";
      events.back()["payload"]["continuation"] = true;
      cycle("s1", "idle", 4.15, 50);
      mgg_msgs::msg::LocalPathFeedback fb;
      fb.session_id = "s1";
      fb.epoch = 777;
      fb.sequence_id = 50;
      fb.state = "executing";
      fb.progress_m = 0.5;
      write(fb, "mgg/local_planner/local_path_feedback", 4.2);
    }
    if (extra == Extra::IdleGap) {
      request("s1", "idle", 4.1);
      events.back()["payload"]["mode"] = "idle";
      events.back()["payload"]["continuation"] = true;
      for (int i = 0; i < 20; ++i) scan(4.11 + i * 0.01);
    }
    odometry(4.5, 20.1);
    request("s2", "r2", 4.8);
    if (extra == Extra::IdleGap) scan(4.85);
    cycle("s2", "r2", 5, 4);
    if (extra == Extra::RestartFeedback) {
      // Same adapter session, new planner process, distinct recorded epoch.
      for (std::size_t i = events.size() - 2; i < events.size(); ++i) {
        events[i]["boot"] = "boot2";
        events[i]["payload"]["session_id"] = "s1";
      }
      events.back()["payload"]["epoch"] = 888;
      mgg_msgs::msg::LocalPathFeedback stale;
      stale.session_id = "s1";
      stale.epoch = 777;
      stale.sequence_id = 1;
      stale.state = "executing";
      stale.progress_m = 9.0;
      write(stale, "mgg/local_planner/local_path_feedback", 4.9);
    }
    writer.close();
    save();
  }
  ~Bundle() { fs::remove_all(root); }
  template <class T> void write(const T& value, const std::string& topic, double t) {
    writer.write(value, "/robot_0/" + topic,
                 rclcpp::Time(static_cast<int64_t>(t * 1e9)));
  }
  void odometry(double t, double x) {
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "odom";
    odom.pose.pose = mgg_ros::toPoseMsg(mgg_test::basePose(x, 0.1));
    write(odom, "odom", t);
  }
  void scan(double t, const std::string& frame = "lidar") {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame;
    cloud.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
    const auto scan = corridor.odomScan(mgg_test::basePose(0.1, 0.1));
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(scan.points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (const auto& point : scan.points) {
      const Eigen::Vector3d relative = point - scan.origin;
      *x = relative.x(); *y = relative.y(); *z = relative.z();
      ++x; ++y; ++z;
    }
    write(cloud, "scan/points", t);
  }
  void event(const std::string& kind, double t, const json& payload) {
    events.push_back({{"kind", kind}, {"robot_id", "robot_0"}, {"boot", "boot1"},
                      {"seq", events.size()}, {"sim_s", t}, {"payload", payload}});
  }
  void request(const std::string& session, const std::string& id, double t) {
    mgg::LocalModeRequest req;
    req.session_id = session; req.request_id = id;
    req.mode = mgg::LocalMode::kFollowRoute;
    req.goal = mgg_test::drivingPoint(6.1, 0.1);
    req.tolerance_m = 0.3;
    event("local_request", t, mgg::localRequestPayload(req, false,
        mgg_test::basePose(session == "s1" ? 0.1 : 20.1, 0.1), t));
  }
  void cycle(const std::string& session, const std::string& id, double t, int seq) {
    event("local_plan", t, {{"session_id", session}, {"request_id", id},
        {"outcome", "path"}, {"epoch", 777}, {"sequence_id", seq}});
  }
  void save() {
    std::ofstream stream(root / "events.jsonl");
    for (const auto& e : events) stream << e.dump() << '\n';
    stream.close();
    data = {{"version", 1}, {"run", "fixture"}, {"robot", "robot_0"},
        {"robot_type", "ground"}, {"architecture", "v2"}, {"session_id", nullptr},
        {"config", json::object()}, {"git", json::object()},
        {"inputs", json::array()}, {"scenarios", json::array({
          {{"tag", "cold"}, {"start_s", 1.9}, {"end_s", 2}, {"session_id", "s1"}},
          {{"tag", "steady"}, {"start_s", 3}, {"end_s", 4}, {"session_id", "s1"}},
          {{"tag", "whole_window_invalidation"}, {"start_s", 4.8}, {"end_s", 5}, {"session_id", "s2"}}})}};
    for (const auto& entry : fs::directory_iterator(root / "bag"))
      data["inputs"].push_back({{"path", "bag/" + entry.path().filename().string()},
          {"kind", "bag"}, {"sha256", hashFile(entry.path())}});
    data["inputs"].push_back({{"path", "events.jsonl"}, {"kind", "events"},
                             {"sha256", hashFile(root / "events.jsonl")}});
    std::ofstream(manifest) << data.dump();
  }
  std::vector<json> run() {
    EXPECT_EQ(mgg::runLocalReplay(manifest, output, params), 0);
    std::ifstream stream(output);
    std::vector<json> rows;
    for (std::string line; std::getline(stream, line);) rows.push_back(json::parse(line));
    return rows;
  }
};
int Bundle::serial = 0;

TEST(LocalReplay, RealSearchAndTimingScope) {
  Bundle wall, clear(true, false);
  const auto rows = wall.run(), open = clear.run();
  ASSERT_EQ(rows.size(), 5u); ASSERT_EQ(open.size(), 5u);
  EXPECT_FALSE(rows[0]["path"].is_null());
  EXPECT_NE(rows[2]["output_digest"], open[2]["output_digest"]);
  // The wall stands beyond the commitment: only the suffix is withdrawn.
  EXPECT_EQ(rows[2]["invalidations"], 0);
  EXPECT_GT(rows[2]["suffix_withdrawals"], 0);
  EXPECT_EQ(rows[3]["admitted_cells"], 0);
  EXPECT_TRUE(rows[3]["path"].is_null());
  EXPECT_EQ(rows[3]["status"], "waiting_for_map");
  EXPECT_GT(rows[3]["reset_count"], 0);
  EXPECT_EQ(rows[0]["measured_scope"], "local_map_ground_invalidation_search");
  EXPECT_GT(rows.back()["load_ms"].get<double>(), 0);
  json consumed = json::array(), hash_only = json::array();
  for (const auto& entry : wall.data["inputs"]) {
    const fs::path path = entry["path"].get<std::string>();
    if (path.extension() == ".mcap" || path == "events.jsonl") consumed.push_back(entry);
    else hash_only.push_back(entry);
  }
  EXPECT_EQ(rows.back()["consumed_inputs"], consumed);
  EXPECT_EQ(rows.back()["hash_only_inputs"], hash_only);
  EXPECT_EQ(rows[0]["tags"], json::array({"cold"}));
  EXPECT_EQ(rows[3]["tags"], json::array({"whole_window_invalidation"}));
  EXPECT_GT(rows[0]["duration_ms"].get<double>(), 0);
  EXPECT_EQ(rows[0]["callback_count"], 0);
  EXPECT_EQ(rows[1]["callback_count"], 1);  // active feedback
  EXPECT_EQ(rows[2]["callback_count"], 4);  // four active wall scans
  EXPECT_EQ(rows[3]["callback_count"], 0);  // new session starts a fresh window
}
TEST(LocalReplay, RequestsFromEvents) {
  Bundle bundle;
  const auto rows = bundle.run();
  ASSERT_EQ(rows.size(), 5u);
  EXPECT_EQ(rows.back()["core_sessions"], json::array({"s1", "s2"}));
  EXPECT_EQ(rows[0]["session_id"], "s1");
  EXPECT_EQ(rows[3]["session_id"], "s2");
  EXPECT_EQ(rows[0]["input_seq"], 1);
  EXPECT_EQ(rows[0]["sim_s"], 2);
}
TEST(LocalReplay, MultipartRequestsAreReassembledOrFailClosed) {
  Bundle b;
  const auto head = b.events[0];
  json first = head, last = head;
  first["payload"]["route_part"] = 0;
  first["payload"]["route_parts"] = 2;
  first["payload"]["route"] = json::array({{{"x", 3.1}, {"y", 0.1}, {"z", 0.4}}});
  last["payload"]["route_part"] = 1;
  last["payload"]["route_parts"] = 2;
  last["payload"]["route"] = json::array({{{"x", 6.1}, {"y", 0.1}, {"z", 0.4}}});
  b.events[0] = first;
  b.events.insert(b.events.begin() + 1, last);
  b.save();
  const auto rows = b.run();
  ASSERT_EQ(rows.size(), 5u);
  EXPECT_EQ(rows.back()["input_counts"]["local_request"], 2);
  EXPECT_EQ(rows[0]["input_seq"], 2);
  fs::remove(b.output);
  b.events.erase(b.events.begin() + 1);
  b.save();
  EXPECT_NE(mgg::runLocalReplay(b.manifest, b.output, b.params), 0);
  EXPECT_FALSE(fs::exists(b.output));
}
TEST(LocalReplay, FeedbackDrivesSplicing) {
  Bundle with, without(false);
  const auto a = with.run(), b = without.run();
  ASSERT_EQ(a.size(), 5u); ASSERT_EQ(b.size(), 5u);
  EXPECT_EQ(a[1]["progress_m"], 0.5);
  EXPECT_EQ(b[1]["progress_m"], 0.0);
  ASSERT_FALSE(a[1]["path"].is_null()); ASSERT_FALSE(b[1]["path"].is_null());
  EXPECT_NE(a[1]["path"]["splice_point"], b[1]["path"]["splice_point"]);
}
TEST(LocalReplay, HashMismatchFails) {
  Bundle b;
  std::ofstream(b.root / "events.jsonl", std::ios::app) << "tampered\n";
  EXPECT_NE(mgg::runLocalReplay(b.manifest, b.output, b.params), 0);
  EXPECT_FALSE(fs::exists(b.output));
}
TEST(LocalReplay, MissingParamsAndUnmatchedFeedbackFail) {
  Bundle b;
  EXPECT_NE(mgg::runLocalReplay(b.manifest, b.output), 0);
  b.events[1]["payload"]["sequence_id"] = 99;
  b.save();
  EXPECT_NE(mgg::runLocalReplay(b.manifest, b.output, b.params), 0);
  EXPECT_FALSE(fs::exists(b.output));
}
TEST(LocalReplay, SameTickFeedbackFollowsItsPublication) {
  Bundle same(true, true, Extra::SameTickFeedback), later;
  const auto rows = same.run(), reference = later.run();
  ASSERT_EQ(rows.size(), 5u); ASSERT_EQ(reference.size(), 5u);
  EXPECT_EQ(rows.back()["failures"], 0);
  EXPECT_EQ(rows[0]["progress_m"], 0.0);
  EXPECT_EQ(rows[1]["progress_m"], 0.5);
  ASSERT_FALSE(rows[1]["path"].is_null());
  EXPECT_EQ(rows[1]["path"]["kind"], static_cast<int>(mgg::LocalPathKind::kExtend));
  EXPECT_EQ(rows[1]["path"]["splice_point"], reference[1]["path"]["splice_point"]);
}
TEST(LocalReplay, SameTickFeedbackWithoutItsPublicationFails) {
  Bundle b(true, true, Extra::SameTickUnmatchedFeedback);
  testing::internal::CaptureStderr();
  const int result = mgg::runLocalReplay(b.manifest, b.output, b.params);
  const auto error = testing::internal::GetCapturedStderr();
  EXPECT_NE(result, 0);
  EXPECT_NE(error.find("unmatched feedback epoch/sequence"), std::string::npos);
  EXPECT_FALSE(fs::exists(b.output));
}
// p1a-acc-2 robot_0's production replay failed "unmatched feedback
// epoch/sequence": its wall-clock budgets need not reproduce every live
// path. Feedback naming a recorded publication whose cycle replayed
// without a path is counted, never mistaken for another path; one naming
// no recorded publication still fails
// (SameTickFeedbackWithoutItsPublicationFails).
TEST(LocalReplay, FeedbackOnAnUnreproducedPathIsCounted) {
  Bundle b(true, true, Extra::UnreproducedFeedback);
  const auto rows = b.run();
  ASSERT_EQ(rows.size(), 6u);
  EXPECT_TRUE(rows[3]["path"].is_null());
  EXPECT_EQ(rows.back()["unreproduced_feedback"], 1);
  EXPECT_EQ(rows.back()["failures"], 0);
}
TEST(LocalReplay, IdleEpochMappingAndUnmatchedIdleCount) {
  Bundle known(true, false, Extra::KnownIdle), unknown(true, false, Extra::UnknownIdle);
  const auto a = known.run(), b = unknown.run();
  ASSERT_EQ(a.size(), 5u); ASSERT_EQ(b.size(), 5u);
  EXPECT_EQ(a[1]["progress_m"], 0.0);
  EXPECT_EQ(b[1]["progress_m"], 0.5);
  EXPECT_EQ(a.back()["unmatched_idle_feedback"], 0);
  EXPECT_EQ(b.back()["unmatched_idle_feedback"], 2);
  EXPECT_EQ(a[1]["path"]["kind"], static_cast<int>(mgg::LocalPathKind::kStart));
  EXPECT_EQ(b[1]["path"]["kind"], static_cast<int>(mgg::LocalPathKind::kExtend));
}
TEST(LocalReplay, RecordedGuidanceAndNoGoZonesAreApplied) {
  Bundle b(true, false, Extra::GuidanceAndZones);
  const auto rows = b.run();
  ASSERT_EQ(rows.size(), 5u);
  ASSERT_FALSE(rows[0]["path"].is_null());
  ASSERT_FALSE(rows[1]["path"].is_null());
  EXPECT_NE(rows[0]["path"]["poses"], rows[1]["path"]["poses"]);
  std::vector<mgg::StateVec> poses;
  for (const auto& p : rows[1]["path"]["poses"])
    poses.emplace_back(p[0].get<double>(), p[1].get<double>(), p[2].get<double>(), p[3].get<double>());
  EXPECT_FALSE(mgg_test::pathCrosses(poses, {3.1, 0.1}, 0.8));
  EXPECT_EQ(rows.back()["input_counts"]["/robot_0/mgg/global_guidance"], 1);
  EXPECT_EQ(rows.back()["input_counts"]["/robot_0/mgg/no_go_zones"], 1);
}
TEST(LocalReplay, MissingStampedTfFails) {
  Bundle b(false, true, Extra::MissingTf);
  testing::internal::CaptureStderr();
  const int result = mgg::runLocalReplay(b.manifest, b.output, b.params);
  const auto error = testing::internal::GetCapturedStderr();
  EXPECT_NE(result, 0);
  EXPECT_NE(error.find("no scan was integrated"), std::string::npos);
  EXPECT_FALSE(fs::exists(b.output));
}
TEST(LocalReplay, ProductionDropsAreCountedWithoutAborting) {
  Bundle b(true, true, Extra::DroppedInputs);
  const auto rows = b.run();
  ASSERT_EQ(rows.size(), 5u);
  EXPECT_FALSE(rows[0]["path"].is_null());
  EXPECT_EQ(rows[1]["progress_m"], 0.5);
  EXPECT_EQ(rows.back()["dropped_scans"], 2);
  EXPECT_EQ(rows.back()["integrated_scans"], 16);
  EXPECT_EQ(rows.back()["ignored_odometry"], 2);
  EXPECT_EQ(rows.back()["ignored_guidance"], 1);
  EXPECT_EQ(rows.back()["ignored_no_go_centres"], 1);
  EXPECT_EQ(rows.back()["cycles_with_usable_scan"], 3);
}
TEST(LocalReplay, PreSessionAndIdleWorkIsNotBilledToCycles) {
  Bundle b(false, false, Extra::IdleGap);
  const auto rows = b.run();
  ASSERT_EQ(rows.size(), 5u);
  // Initial 12 scans and the 20 scans in IDLE are really integrated, but only
  // the scan after the second active request belongs to that cold cycle.
  EXPECT_EQ(rows.back()["integrated_scans"], 33);
  EXPECT_EQ(rows[0]["callback_count"], 0);
  EXPECT_EQ(rows[1]["callback_count"], 0);
  EXPECT_EQ(rows[2]["callback_count"], 0);
  EXPECT_EQ(rows[3]["callback_count"], 1);
}
TEST(LocalReplay, TrailingScansCannotQualifyUnobservedCycles) {
  Bundle b(false, false, Extra::LateScans);
  testing::internal::CaptureStderr();
  const int result = mgg::runLocalReplay(b.manifest, b.output, b.params);
  const auto error = testing::internal::GetCapturedStderr();
  EXPECT_NE(result, 0);
  EXPECT_NE(error.find("no planning cycle had a usable scan"), std::string::npos);
  EXPECT_FALSE(fs::exists(b.output));
}
TEST(LocalReplay, HashOnlyInputsAreNotReportedAsConsumed) {
  Bundle b;
  std::ofstream(b.root / "unused-map.json") << "{}";
  const json extra = {{"path", "unused-map.json"}, {"kind", "map_product"},
                      {"sha256", hashFile(b.root / "unused-map.json")}};
  b.data["inputs"].push_back(extra);
  std::ofstream(b.manifest) << b.data.dump();
  const auto rows = b.run();
  ASSERT_EQ(rows.size(), 5u);
  const auto& consumed = rows.back()["consumed_inputs"];
  ASSERT_TRUE(consumed.is_array());
  ASSERT_EQ(consumed.size(), 2u);  // one MCAP, one events stream
  for (const auto& input : consumed) {
    const fs::path path = input["path"].get<std::string>();
    EXPECT_TRUE(path.extension() == ".mcap" || path == "events.jsonl");
  }
  const auto& hash_only = rows.back()["hash_only_inputs"];
  ASSERT_TRUE(hash_only.is_array());
  ASSERT_EQ(hash_only.size(), 2u);  // bag metadata and unused map product
  EXPECT_NE(std::find(hash_only.begin(), hash_only.end(), extra), hash_only.end());
}
TEST(LocalReplay, KnownForeignEpochAfterRestartIsIgnored) {
  Bundle b(true, false, Extra::RestartFeedback);
  const auto rows = b.run();
  ASSERT_EQ(rows.size(), 5u);
  EXPECT_EQ(rows[3]["session_id"], "s1");
  EXPECT_EQ(rows[3]["progress_m"], 0.0);
  EXPECT_EQ(rows.back()["ignored_stale_feedback"], 1);
  EXPECT_TRUE(rows[3]["path"].is_null());
}
TEST(LocalReplay, InputSymlinkAndEscapingPathFail) {
  Bundle b;
  fs::rename(b.root / "events.jsonl", b.root / "real-events.jsonl");
  fs::create_symlink(b.root / "real-events.jsonl", b.root / "events.jsonl");
  EXPECT_NE(mgg::runLocalReplay(b.manifest, b.output, b.params), 0);
  b.data["inputs"].back()["path"] = "../outside.jsonl";
  std::ofstream(b.manifest) << b.data.dump();
  EXPECT_NE(mgg::runLocalReplay(b.manifest, b.output, b.params), 0);
}
}  // namespace
