#include "mgg_core/local_route.h"
#include "mgg_core/planning_cancellation.h"
// NAVIGATE objectives for a Bunker-sized ground robot (botman) on MOLA
// planning products at its deployed 0.10 m resolution: what the lattice,
// the goal link and the shortcut make of open floor, a wall and a ramp.
// The mola_snapshot backend is always built, so these run with and without
// OctoMap.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <rclcpp/rclcpp.hpp>

#include "mgg_map_octomap/mola_map.h"
#include "mgg_ros/planner_node.h"

namespace mgg_ros {

namespace {

std::string sha256(const std::string& bytes) {
  std::array<unsigned char, EVP_MAX_MD_SIZE> output{};
  unsigned int size = 0;
  EVP_Digest(bytes.data(), bytes.size(), output.data(), &size, EVP_sha256(),
             nullptr);
  std::ostringstream text;
  text << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < size; ++i) {
    text << std::setw(2) << static_cast<unsigned int>(output[i]);
  }
  return text.str();
}

template <typename T>
void append(std::string& bytes, T value) {
  std::uint64_t raw = 0;
  std::memcpy(&raw, &value, sizeof(T));
  for (std::size_t i = 0; i < 8; ++i) {
    bytes.push_back(static_cast<char>((raw >> (8 * i)) & 0xff));
  }
}

void write(const std::filesystem::path& path, const std::string& bytes) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

/// How high over the ground a product's free space is observed, metres.
/// Botman's body reaches 1.57 m; above about 1.2 m its lidar's rays rarely
/// pass over open floor, so the top of the body box stands in unknown air,
/// as on the robot's own products (botman_0, 2026-10-01).
constexpr double kObservedFreeAboveM = 1.2;

/// An upright block standing on the ground over [x0, x1] x [y0, y1], `top`
/// metres above it.
struct Block {
  double x0, x1, y0, y1, top;
};

}  // namespace

/// A MOLA planning product at `resolution`: ground at `ground(x, y)` over
/// [x0, x1] x [y0, y1] (one occupied voxel, its measured surface the exact
/// height), observed free space up to `free_above` metres over it, and
/// blocks on it. Everything else is unknown.
class MolaTerrainProduct {
 public:
  MolaTerrainProduct(double resolution, double x0, double x1, double y0,
                     double y1, const std::function<double(double, double)>& ground,
                     const std::vector<Block>& blocks = {},
                     double free_above = kObservedFreeAboveM,
                     const std::function<bool(double,double,double)>& unknown = {})
      : resolution_(resolution) {
    static int sequence = 0;
    root_ = std::filesystem::temp_directory_path() /
            ("mgg-navigation-mola-" + std::to_string(::getpid()) + "-" +
             std::to_string(sequence++));
    std::filesystem::create_directories(root_ / "mola" / "components");
    using json = nlohmann::json;
    struct Voxel {
      std::int64_t x, y, z;
      bool operator<(const Voxel& o) const {
        return std::tie(x, y, z) < std::tie(o.x, o.y, o.z);
      }
    };
    std::vector<Voxel> occupied, free;
    std::vector<double> surface;  // per occupied voxel, after sorting
    std::vector<std::pair<Voxel, double>> occupied_with_top;
    const auto index = [&](double v) {
      return static_cast<std::int64_t>(std::floor(v / resolution + 1e-9));
    };
    for (std::int64_t x = index(x0); x < index(x1); ++x) {
      for (std::int64_t y = index(y0); y < index(y1); ++y) {
        const double cx = (x + 0.5) * resolution, cy = (y + 0.5) * resolution;
        const double h = ground(cx, cy);
        if (!std::isfinite(h)) continue;
        const auto gz = static_cast<std::int64_t>(std::floor(h / resolution));
        occupied_with_top.push_back({{x, y, gz}, h});
        double block_top = -1.0;
        for (const Block& b : blocks) {
          if (cx >= b.x0 && cx < b.x1 && cy >= b.y0 && cy < b.y1) {
            block_top = std::max(block_top, b.top);
          }
        }
        const auto top_z = static_cast<std::int64_t>(
            std::floor((h + free_above) / resolution));
        for (std::int64_t z = gz + 1; z <= top_z; ++z) {
          const double zc = (z + 0.5) * resolution;
          if (block_top > 0.0 && zc - h <= block_top) {
            occupied_with_top.push_back({{x, y, z}, zc});
          } else if (!unknown || !unknown(cx,cy,zc)) {
            free.push_back({x, y, z});
          }
        }
      }
    }
    std::sort(occupied_with_top.begin(), occupied_with_top.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    for (const auto& [voxel, top] : occupied_with_top) {
      occupied.push_back(voxel);
      surface.push_back(top);
    }
    std::sort(free.begin(), free.end());

    const std::string geometry(64, 'a');
    const std::string snapshot_id(64, '1');
    const std::uint64_t stamp = 1000;
    const std::size_t points = occupied.size();
    const json chunk{{"sha256", std::string(64, 'c')},
                     {"size_bytes", 16 + 12 * points},
                     {"point_count", points},
                     {"encoding", "application/vnd.swarmdeck.xyz-f32.v1"}};
    json submap{{"observed_at_ns", stamp}, {"chunks", json::array({chunk})}};
    submap["sensor_origins"] = json::array({json::array({0.0, 0.0, 0.0})});
    submap["ray_evidence"] = {{"return_semantics", "first_return"},
                              {"deskew", "not_required"},
                              {"origin_association", "single_capture"}};
    json manifest{{"map_id", "onboard"},
                  {"layer_id", "persistent_geometry"},
                  {"frame_id", "component_test"},
                  {"graph_revision", {{"component_id", "component:test"},
                                      {"epoch", 1}, {"revision", 0}}},
                  {"geometry_revision", geometry},
                  {"submaps", json::array({submap})},
                  {"chunks", json::array({chunk})},
                  {"tombstones", json::array()}};
    json canonical = manifest;
    canonical["schema"] = "swarmdeck.autonomy.v1";
    const json source{{"schema", "swarmdeck.autonomy.v1"},
                      {"snapshot_id", snapshot_id},
                      {"generated_at_ns", stamp},
                      {"manifests", json::array({manifest})}};
    const std::string source_bytes = source.dump();
    const std::string source_digest = sha256(source_bytes);
    const json metadata{
        {"schema", "swarmdeck.mola_planner_grid.v2"},
        {"graph_version", {{"component_id", "component:test"}, {"epoch", 1},
                           {"revision", 0}, {"digest", std::string(64, 'd')}}},
        {"identity", {{"geometry_revision", geometry},
                      {"native_geometry_digest", std::string(64, 'e')},
                      {"canonical_manifest_digest", sha256(canonical.dump())},
                      {"source_snapshot_id", snapshot_id},
                      {"source_sha256", source_digest},
                      {"reference_frame", "component_test"}}},
        {"source_stamp_ns", stamp},
        {"resolution_m", resolution},
        {"ray_angular_resolution_rad", 0.08726646259971647},
        {"ray_step_fraction", 0.75},
        {"point_count", points},
        {"source_point_count", points},
        {"occupied_count", occupied.size()},
        {"free_count", free.size()},
        {"surface_count", occupied.size()},
        {"retired_count", 0},
        {"ray_steps", free.size()},
        {"qualified_ray_keyframes", 1}};
    const std::string metadata_bytes = metadata.dump();
    std::string grid("SDMGRID1", 8);
    const auto size32 = static_cast<std::uint32_t>(metadata_bytes.size());
    for (int i = 0; i < 4; ++i) grid.push_back(static_cast<char>((size32 >> (8 * i)) & 0xff));
    grid += metadata_bytes;
    for (const Voxel& v : occupied) { append(grid, v.x); append(grid, v.y); append(grid, v.z); }
    for (const Voxel& v : free) { append(grid, v.x); append(grid, v.y); append(grid, v.z); }
    for (std::size_t i = 0; i < occupied.size(); ++i) {
      append(grid, occupied[i].x);
      append(grid, occupied[i].y);
      append(grid, surface[i]);
    }
    const json index_json{
        {"version", 1},
        {"source_snapshot_id", snapshot_id},
        {"source_sha256", source_digest},
        {"generated_at_ns", stamp},
        {"artifacts",
         json::array({{{"component_id", "component:test"}, {"epoch", 1},
                       {"revision", 0}, {"geometry_revision", geometry},
                       {"manifest_sha256", std::string(64, '9')},
                       {"path", "components/native.mola"}, {"size_bytes", 1},
                       {"sha256", std::string(64, 'f')},
                       {"planner", {{"path", "components/native.sdpg"},
                                    {"size_bytes", grid.size()},
                                    {"sha256", sha256(grid)},
                                    {"source_sha256", source_digest},
                                    {"source_snapshot_id", snapshot_id}}}}})}};
    write(root_ / "mola" / "components" / "native.sdpg", grid);
    write(root_ / "mola" / "source.json", source_bytes);
    write(root_ / "mola" / "index.json", index_json.dump());
    request_ = {"component:test", 1, 0, geometry, stamp,
                Eigen::Isometry3d::Identity()};
  }
  ~MolaTerrainProduct() { std::filesystem::remove_all(root_); }

  const std::filesystem::path& root() const { return root_; }
  const mgg::MolaSnapshotRequest& request() const { return request_; }
  double resolution() const { return resolution_; }

 private:
  double resolution_;
  std::filesystem::path root_;
  mgg::MolaSnapshotRequest request_;
};

class PlannerNodeTestPeer {
 public:
  /// Botman as deployed (deploy/mgg hardware.yaml with bunker.yaml): the
  /// 1.344 x 0.778 m box centred on the lidar, 0.61 m over the floor, 2 x
  /// 0.61 m tall, extended by 0.05 m; the lattice and terrain limits.
  static void configureBotman(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.robot_params_.type = mgg::RobotType::kGroundRobot;
    node.robot_params_.size = Eigen::Vector3d(1.344, 0.778, 1.22);
    node.robot_params_.size_extension = Eigen::Vector3d(0.05, 0.05, 0.05);
    node.robot_params_.size_extension_min.setZero();
    node.robot_params_.safety_extension.setZero();
    node.robot_params_.bound_mode = mgg::BoundModeType::kExtendedBound;
    node.robot_params_.center_offset.setZero();
    node.robot_params_.physical_size = Eigen::Vector3d(1.023, .778, .660);
    node.robot_params_.physical_center_offset = Eigen::Vector3d(-.16, 0, .330 - .935);
    mgg::PlanningParams& p = node.planning_params_;
    p.rr_mode = mgg::RRModeType::kGraph;
    p.edge_length_min = 0.05;
    p.edge_length_max = 1.0;
    p.edge_overshoot = 0.0;
    p.num_vertices_max = 500;
    p.num_edges_max = 10000;
    p.num_loops_cutoff = 2000;
    p.num_loops_max = 200000;
    p.nearest_range = 0.6;
    p.nearest_range_z = 0.15;
    p.nearest_range_min = 0.05;
    p.nearest_range_max = 1.0;
    p.max_ground_height = 0.61 + 0.15 + 0.175;
    p.max_step_height = 0.15;
    p.max_inclination = 27.0 * M_PI / 180.0;
    p.max_footprint_tilt = 20.0 * M_PI / 180.0;
    p.max_footprint_step = 0.10;
    p.path_clearance_margin = 0.6;
    p.path_interpolation_distance = 0.25;
    p.traverse_length_max = 20.0;
    node.allow_unknown_lattice_body_ = true;
    node.hanging_root_edge_length_max_ = 1.0;
    node.grid_params_.min_val = Eigen::Vector3d(-6.0, -6.0, -0.2);
    node.grid_params_.max_val = Eigen::Vector3d(6.0, 6.0, 0.3);
    node.grid_params_.resolution = Eigen::Vector3d(0.4, 0.4, 0.1);
    node.global_space_.setBound(Eigen::Vector3d(-20.0, -20.0, -3.0),
                                Eigen::Vector3d(20.0, 20.0, 5.0));
    node.odometry_stale_s_ = 3600.0;
  }
  /// The product in service, as a heartbeat naming it would put it.
  static bool serve(PlannerNode& node, const MolaTerrainProduct& product) {
    node.mola_map_->requestSnapshot(product.request());
    for (int i = 0; i < 400 && !node.mola_map_->getStatus(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.mapping_snapshot_.component_id = "component:test";
    node.mapping_snapshot_.epoch = 1;
    node.mapping_snapshot_.component_from_navigation.rotation.w = 1.0;
    node.have_mapping_snapshot_ = true;
    return node.mola_map_->getStatus();
  }
  /// Odometry at the lidar, 0.61 m over the ground at (x, y), facing yaw.
  static void standAt(PlannerNode& node, double x, double y, double ground,
                      double yaw, double stamp_s) {
    auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    msg->header.stamp.sec = static_cast<std::int32_t>(stamp_s);
    msg->pose.pose.position.x = x;
    msg->pose.pose.position.y = y;
    msg->pose.pose.position.z = ground + 0.61;
    msg->pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg->pose.pose.orientation.w = std::cos(yaw / 2.0);
    node.onOdometry(msg);
  }
  static bool serveFixture(PlannerNode& node, const std::string& root) {
    nlohmann::json source;
    std::ifstream(root + "/mola/source.json") >> source;
    const auto& manifest = source.at("manifests").at(0);
    mgg::MolaSnapshotRequest request;
    request.component_id = manifest.at("graph_revision").at("component_id");
    request.epoch = manifest.at("graph_revision").at("epoch");
    request.graph_revision = manifest.at("graph_revision").at("revision");
    request.geometry_revision = manifest.at("geometry_revision");
    for (const auto& submap : manifest.at("submaps"))
      request.source_stamp_ns = std::max<std::uint64_t>(request.source_stamp_ns,
          submap.value("observed_at_ns", std::uint64_t{0}));
    node.mola_map_->requestSnapshot(request);
    for (int i=0; i<400 && !node.mola_map_->getStatus(); ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    node.mapping_snapshot_.component_id = request.component_id;
    node.mapping_snapshot_.epoch = request.epoch;
    node.mapping_snapshot_.component_from_navigation.rotation.w = 1;
    node.have_mapping_snapshot_ = true;
    mgg::SensorParams sensor;
    sensor.type = mgg::SensorType::kLidar;
    sensor.max_range = 20;
    sensor.fov = Eigen::Vector2d(2*M_PI, M_PI/3);
    sensor.resolution = Eigen::Vector2d(M_PI/36, M_PI/36);
    sensor.frontier_percentage_threshold = .05;
    sensor.update();
    node.sensors_["VLP16"] = sensor;
    node.planning_params_.exp_sensor_list = {"VLP16"};
    return node.mola_map_->getStatus();
  }
  static std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> explore(PlannerNode& node) {
    auto request = std::make_shared<mgg_msgs::srv::PlannerSrv::Request>();
    request->bound_mode = request->EXTENDED_BOUND;
    auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
    node.onPlanRequest(request, response);
    return response;
  }
  static void objectiveIdentity(const PlannerNode& node, mgg_msgs::srv::PlanObjective::Request& request) {
    request.component_id = node.mapping_snapshot_.component_id;
    request.map_epoch = node.mapping_snapshot_.epoch;
  }
  static nlohmann::json explorationMetrics(PlannerNode& node) {
    double radius = 0, length = 0;
    for (const auto& entry : node.local_graph_->vertices_map_)
      radius = std::max(radius, (entry.second->state.head<2>() - node.current_state_.head<2>()).norm());
    for (std::size_t i = 1; i < node.best_path_.size(); ++i)
      length += (node.best_path_[i].head<3>() - node.best_path_[i-1].head<3>()).norm();
    double selected_gain = 0, distance = std::numeric_limits<double>::infinity();
    if (!node.best_path_.empty()) for (const auto& entry : node.local_graph_->vertices_map_) {
      const double d = (entry.second->state.head<3>() - node.best_path_.back().head<3>()).norm();
      if (d < distance) { distance = d; selected_gain = entry.second->vol_gain.gain; }
    }
    return {{"lattice_radius_m",radius},{"viewpoints",node.local_graph_->getNumVertices()},
      {"best_path_length_m",length},{"endpoint_gain",selected_gain}};
  }
  static void explorationSlice(PlannerNode& node, double seconds) {
    node.ground_exploration_lattice_budget_s_ = seconds;
  }
  /// Ground gain sensor model (PlanningParams ground_gain_*); 0 = real sensor.
  static void groundGainModel(PlannerNode& node, double degrees, double range) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.planning_params_.ground_gain_angular_resolution_deg = degrees;
    node.planning_params_.ground_gain_max_range = range;
  }
  static nlohmann::json groundGainModel(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return {{"ground_gain_step_deg", node.planning_params_.ground_gain_angular_resolution_deg},
            {"ground_gain_range_m", node.planning_params_.ground_gain_max_range}};
  }
  static nlohmann::json globalLinks(PlannerNode& node) {
    mgg::StateVec goal(-2,0,0,0);
    node.projectGoalToDrivingHeight(goal);
    const auto strict = node.makeGlobalContext(), bounded = node.makeContext();
    nlohmann::json links = nlohmann::json::array();
    std::vector<mgg::Vertex*> near;
    node.global_graph_->getNearestVertices(&goal, 1.0, &near);
    const mgg::Vertex target(-1,goal);
    for (const auto* v : near) {
      mgg::ExpandGraphReport a,b;
      const bool sa = mgg::roadmapEdgeTraversable(strict,*v,target,a);
      const bool sb = mgg::roadmapEdgeTraversable(bounded,*v,target,b);
      nlohmann::json upper_unknown = nlohmann::json::array();
      if (sb && bounded.unknown_body_above_center) {
        const auto size=node.robot_params_.getPlanningSize();
        const double yaw=std::atan2(goal.y()-v->state.y(),goal.x()-v->state.x());
        mgg::OrientedBox body{goal.head<3>(),yaw,size};
        std::vector<mgg::XYCellCenter> cells;
        node.map_->getCircleIntersectingXYCellCenters(goal.head<2>(),size.head<2>().norm()/2,4096,cells);
        const double plane=goal.z()+*bounded.unknown_body_above_center;
        for (const auto& cell:cells) if(mgg::cellMeetsBox(cell.center,.1,body,-1e-9)) {
          for (double z=std::floor(plane/.1)*.1+.15; z<goal.z()+size.z()/2;z+=.1) {
            if(upper_unknown.size()<8 && node.map_->getVoxelStatus({cell.center.x(),cell.center.y(),z})==mgg::VoxelStatus::kUnknown)
              upper_unknown.push_back({cell.center.x(),cell.center.y(),z});
          }
        }
      }
      links.push_back({{"upper_unknown_voxels",upper_unknown},{"id",v->id},{"position",{v->state.x(),v->state.y(),v->state.z()}},
        {"strict",sa},{"bounded",sb},{"strict_status",a.edge_status},{"bounded_status",b.edge_status}});
    }
    return {{"vertices",node.global_graph_->getNumVertices()},
      {"edges",node.global_graph_->getNumEdges()},{"goal_links",links}};
  }
  static void mappedGlobalPath(PlannerNode& node) {
    node.applyLatestOdometry();
    node.global_graph_->reset();
    auto* a = new mgg::Vertex(0, mgg::StateVec(0,0,.935,M_PI));
    node.global_graph_->addVertex(a);
    auto* b = new mgg::Vertex(node.global_graph_->generateVertexID(), mgg::StateVec(-1.2,0,.935,M_PI));
    node.global_graph_->addVertex(b);
    node.global_graph_->addEdge(a,b,1.2);
    node.global_root_supported_ = true;
  }
  static std::pair<int,int> globalSize(PlannerNode& node) {
    return {node.global_graph_->getNumVertices(), node.global_graph_->getNumEdges()};
  }
  static void historyIdentity(PlannerNode& node, const std::string& component, int epoch, double x = 2) {
    class OwnSource : public KeyframeTrajectorySource {
     public:
      KeyframeTrajectory trajectory;
      bool read(KeyframeTrajectory& out, std::string&) override { out=trajectory; return true; }
    };
    auto source = std::make_unique<OwnSource>();
    source->trajectory.component_id=component; source->trajectory.epoch=epoch;
    auto pose=Eigen::Isometry3d::Identity(); pose.translation()=Eigen::Vector3d(x,0,.61);
    source->trajectory.poses={pose};
    node.keyframe_source_=std::move(source);
  }
  static mgg::VoxelStatus historicalUnknown(PlannerNode& node, double low = .55, double high = .6) {
    node.applyLatestOdometry();
    PlannerNode::StandingStartScope scope(node);
    const auto ctx=node.makeContext();
    const Eigen::Vector2d cell(2.05,.05);
    return ctx.own_body_known_free->strictColumnStatus(*node.map_,cell,low,high);
  }
  static nlohmann::json joinWitness(PlannerNode& node, const Eigen::Vector3d& from,
                                     const Eigen::Vector3d& to) {
    node.applyLatestOdometry();
    PlannerNode::StandingStartScope scope(node);
    const auto ctx=node.makeContext();
    nlohmann::json row, sweeps=nlohmann::json::array();
    mgg::OrientedBox body;
    body.heading=std::atan2(to.y()-from.y(),to.x()-from.x());
    body.size=ctx.robot_box_size;
    mgg::EdgeBodyCheck check;
    check.sweep=[&](const Eigen::Vector3d& a,const Eigen::Vector3d& b) {
      const auto status=mgg::orientedBoxPathStatus(*node.map_,a,b,body,true,
          nullptr,true,ctx.unknown_body_above_center,ctx.own_body_known_free.get());
      nlohmann::json sweep{{"from",{a.x(),a.y(),a.z()}},{"to",{b.x(),b.y(),b.z()}},
                            {"body_status",int(status)}};
      nlohmann::json witnesses=nlohmann::json::array();
      const double resolution=node.map_->getResolution();
      const int steps=std::max(1,int(std::ceil((b-a).norm()/resolution)));
      const Eigen::Vector3d step=(b-a)/steps;
      const Eigen::Vector2d along(std::cos(body.heading),std::sin(body.heading));
      mgg::OrientedBox swept=body;
      swept.size+=Eigen::Vector3d(std::abs(step.head<2>().dot(along)),
          std::abs(step.head<2>().dot(Eigen::Vector2d(-along.y(),along.x()))),std::abs(step.z()));
      for(int i=0;i<steps;++i) {
        swept.center=a+(i+.5)*step;
        std::vector<mgg::XYCellCenter> cells;
        node.map_->getCircleIntersectingXYCellCenters(swept.center.head<2>(),
            swept.size.head<2>().norm()/2,4096,cells);
        const double lo=swept.center.z()-swept.size.z()/2;
        const double hi=std::min(swept.center.z()+swept.size.z()/2,
            swept.center.z()+ctx.unknown_body_above_center.value_or(swept.size.z()/2)+std::abs(step.z())/2);
        for(const auto& cell:cells) {
          if(!mgg::cellMeetsBox(cell.center,resolution,swept,-1e-9))continue;
          if(ctx.own_body_known_free->strictColumnStatus(*node.map_,cell.center,lo,hi)==mgg::VoxelStatus::kFree)continue;
          for(double z=std::floor(lo/resolution)*resolution+resolution/2;
              z-resolution/2<=hi;z+=resolution) {
            const auto voxel=node.map_->getVoxelStatus({cell.center.x(),cell.center.y(),z});
            if(voxel==mgg::VoxelStatus::kFree)continue;
            const auto masked=ctx.own_body_known_free->strictColumnStatus(*node.map_,cell.center,
                std::max(lo,z-resolution/2+1e-7),std::min(hi,z+resolution/2-1e-7));
            if(masked==mgg::VoxelStatus::kFree)continue;
            if(witnesses.size()<12) witnesses.push_back({{"voxel",{cell.center.x(),cell.center.y(),z}},
                {"status",int(voxel)},{"masked_status",int(masked)},{"required_band",{lo,hi}}});
          }
        }
      }
      sweep["witnesses"]=witnesses; sweeps.push_back(sweep);
      return status;
    };
    std::vector<Eigen::Vector3d> projected;
    const auto offset=ctx.robot->offsetForHeading(body.heading);
    row["projected_status"]=int(ctx.ground->getProjectedEdgeStatus(from+offset,to+offset,
        body.size,true,projected,false,false,&check,mgg::EdgeTravel::kForward));
    row["sweeps"]=sweeps;
    return row;
  }
  static bool postSpin(PlannerNode& node, std::vector<mgg::StateVec>& path) {
    node.applyLatestOdometry();
    return node.startPathAfterChassisSpin(path,true);
  }
  static mgg::Departure turnedDeparture(PlannerNode& node) {
    node.applyLatestOdometry();
    mgg::StateVec start(0,0,.935,M_PI/2);
    mgg::Departure departure;
    // Reject endpoints at the original heading; exercise the first small
    // turned departure without coupling the test to search scoring.
    EXPECT_TRUE(mgg::findDeparture(*node.map_,*node.ground_,node.robot_params_,
        node.planning_params_,start,departure,[](const auto& path) {
          return std::abs(path.front()[3]-M_PI/2)>1e-3;
        }));
    return departure;
  }
  static std::vector<mgg::StateVec> storedOffsetReverse(PlannerNode& node) {
    node.applyLatestOdometry();
    const mgg::StateVec start(0,0,.935,.2);
    node.stored_reverse_exit_ = {start, {-.5,0,.935,0}, {-1,0,.935,0}, {-2,0,.935,0}};
    node.reverse_exit_entry_path_ = {node.stored_reverse_exit_.rbegin(),node.stored_reverse_exit_.rend()};
    std::vector<mgg::StateVec> path;
    std::string note;
    EXPECT_TRUE(node.validateStoredReverseExit(start,path,note)) << note;
    EXPECT_FALSE(path.empty()) << note;
    return path;
  }
  static bool refugeBand(PlannerNode& node, double stored_yaw) {
    node.applyLatestOdometry();
    return node.refugeArrivalBandAdmissible({{0,0,.935,stored_yaw},
                                            {1,0,.935,stored_yaw}});
  }
  static bool postSpinRecordsTurnBack(PlannerNode& node) {
    node.applyLatestOdometry();
    node.best_path_ = {{0,0,.935,M_PI},{.25,0,.935,0},{1,0,.935,0}};
    node.lattice_path_ = node.best_path_;
    node.best_path_from_global_graph_ = false;
    node.lattice_selection_direction_ = M_PI;
    if (!node.startPathAfterChassisSpin(node.best_path_,true)) return false;
    node.recordSentPath();
    return node.turn_back_hysteresis_.lastTurnedBack();
  }
  static bool drivenReturn(PlannerNode& node, bool history) {
    node.applyLatestOdometry();
    PlannerNode::StandingStartScope scope(node);
    auto ctx=node.makeContext();
    if (!history) ctx.own_body_known_free.reset();
    return mgg::groundShortcutSegmentAdmissible(ctx,{2,0,.935},{1.8,0,.935},true);
  }
  static bool hasOwnBodyMask(PlannerNode& node, bool physical, bool strict = true) {
    node.applyLatestOdometry();
    if (!physical) {
      node.robot_params_.physical_size.reset();
      node.robot_params_.physical_center_offset.reset();
    }
    if (strict) node.unknown_body_policy_ = "strict";
    node.allow_unknown_lattice_body_ = false;
    const auto ctx = node.makeContext();
    return ctx.standing_body.has_value() || bool(ctx.own_body_known_free);
  }
  static bool globalUsesUpperPolicy(PlannerNode& node) {
    return node.makeGlobalContext().unknown_body_above_center.has_value();
  }
  static void forceGlobalFallback(PlannerNode& node, bool force) {
    node.grid_params_.min_val.x() = force ? -1 : -6;
    node.grid_params_.max_val.x() = force ? 1 : 6;
  }
  static void sensorPolicy(PlannerNode& node, double mount = .61) {
    node.allow_unknown_lattice_body_ = false;
    node.unknown_body_policy_ = "above_sensor_fov";
    auto sensor = node.sensors_["VLP16"];
    sensor.mount_height = mount;
    sensor.fov = Eigen::Vector2d(2*M_PI, M_PI/2);
    if (sensor.resolution.minCoeff() > 0) sensor.update();
    node.sensors_["VLP16"] = sensor;
  }
  static void hardwarePolicy(PlannerNode& node) { node.allow_unknown_lattice_body_ = false; }
  static void shortcut(PlannerNode& node, std::vector<mgg::StateVec>& path) {
    node.shortcutAndResample(path, {}, {}, true);
  }

  static void expireRequest(PlannerNode& node) {
    node.lattice_deadline_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  }
  static void setBudget(PlannerNode& node, double seconds) {
    node.lattice_time_budget_s_ = seconds;
    node.navigate_time_budget_s_ = seconds;
    node.global_route_time_budget_s_ = seconds;
  }
  // Inject cooperative work at real request checkpoints, not an uninterruptible
  // sleep or a machine-dependent graph size. No production timing hook.
  static bool deadlineActive(const PlannerNode& node) {
    return node.lattice_deadline_.has_value();
  }
  static void cancel(PlannerNode& node) { node.cancelPlanning(); }
  static double& rawRouteCertificationMs(PlannerNode& node) {
    return node.raw_route_certification_ms_;
  }
  static void resetPolish(PlannerNode& node) {
    node.path_shortcut_from_ = 0;
    node.path_shortcut_to_ = 0;
  }
  static bool polishing(const PlannerNode& node) { return node.path_shortcut_from_ > 2; }
  // The shortcut has finished: the polished route is being certified.
  // shortcutAndResample sets path_shortcut_to_ to its input size on entry
  // and to the sent size only once it is done, so a positive value alone is
  // still the shortcut (mgg-budget review r1 P2). The budget roadmaps'
  // routes are resampled to more poses than their graph corners; a route
  // whose sizes matched would never report this phase, and the tests
  // assert it was reached.
  static bool certifyingPolish(const PlannerNode& node) {
    return polishing(node) && node.path_shortcut_to_ != node.path_shortcut_from_;
  }
  // The raw graph route first doubles back to a vertex inside the chassis
  // spin's offset circle, so its spin joins nothing before an initial-leg
  // corner. Its shortcut runs straight ahead along the robot's heading. The
  // long edge keeps the link expansion from bypassing that first vertex.
  static void rawCornerRoadmap(PlannerNode& node) {
    node.applyLatestOdometry();
    node.global_graph_->reset();
    const std::vector<Eigen::Vector2d> at{{0, 0}, {.16, .1}, {-1.2, 0}, {-2, 0}};
    for (std::size_t i = 0; i < at.size(); ++i) {
      auto* v = new mgg::Vertex(static_cast<int>(i), mgg::StateVec(at[i].x(), at[i].y(), .935, M_PI));
      node.global_graph_->addVertex(v);
      if (i) node.global_graph_->addEdge(node.global_graph_->getVertex(static_cast<int>(i) - 1), v,
                                         (at[i] - at[i - 1]).norm());
    }
    node.global_root_supported_ = true;
    resetPolish(node);
  }
  static void budgetRoadmap(PlannerNode& node) {
    node.applyLatestOdometry();
    node.global_graph_->reset();
    for (int i = 0; i < 5; ++i) {
      auto* v = new mgg::Vertex(i, mgg::StateVec(-i * .5, 0, .935, M_PI));
      node.global_graph_->addVertex(v);
      if (i) node.global_graph_->addEdge(node.global_graph_->getVertex(i-1), v, .5);
    }
    node.global_root_supported_ = true;
    resetPolish(node);
  }
  static void exploreImpl(PlannerNode& node,
      std::shared_ptr<mgg_msgs::srv::PlannerSrv::Response> response) {
    node.onPlanRequestImpl(std::make_shared<mgg_msgs::srv::PlannerSrv::Request>(), response);
  }
  static void objective(
      PlannerNode& node,
      std::shared_ptr<mgg_msgs::srv::PlanObjective::Request> request,
      std::shared_ptr<mgg_msgs::srv::PlanObjective::Response> response) {
    node.onObjectiveRequest(request, response);
  }
};

namespace {

using Service = mgg_msgs::srv::PlanObjective;

std::shared_ptr<PlannerNode> botmanNode(const std::string& name,
                                        const MolaTerrainProduct& product,
                                        std::vector<rclcpp::Parameter> extra = {}) {
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=" + name});
  std::vector<rclcpp::Parameter> parameters{
      rclcpp::Parameter("map.backend", "mola_snapshot"),
      rclcpp::Parameter("map.resolution", product.resolution()),
      rclcpp::Parameter("map.mola.peer_root", product.root().string()),
      rclcpp::Parameter("map.mola.snapshot_ttl_sec", 60.0),
      rclcpp::Parameter("PlanningParams.global_frame_id", "world"),
  };
  parameters.insert(parameters.end(), extra.begin(), extra.end());
  options.parameter_overrides(parameters);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<PlannerNode>(options);
  PlannerNodeTestPeer::configureBotman(*node);
  EXPECT_TRUE(PlannerNodeTestPeer::serve(*node, product));
  return node;
}

std::shared_ptr<Service::Response> navigate(PlannerNode& node, double x,
                                            double y, double z = 0.0) {
  auto request = std::make_shared<Service::Request>();
  request->objective = Service::Request::NAVIGATE;
  PlannerNodeTestPeer::objectiveIdentity(node, *request);
  request->goal.position.x = x;
  request->goal.position.y = y;
  request->goal.position.z = z;
  request->goal.orientation.w = 1.0;
  auto response = std::make_shared<Service::Response>();
  PlannerNodeTestPeer::objective(node, request, response);
  return response;
}

double pathLength(const std::vector<geometry_msgs::msg::Pose>& path) {
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += std::hypot(path[i].position.x - path[i - 1].position.x,
                         path[i].position.y - path[i - 1].position.y);
  }
  return length;
}

/// The largest heading change between consecutive segments, degrees.
double maxCornerDeg(const std::vector<geometry_msgs::msg::Pose>& path) {
  double worst = 0.0;
  Eigen::Vector2d previous = Eigen::Vector2d::Zero();
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector2d d(path[i].position.x - path[i - 1].position.x,
                            path[i].position.y - path[i - 1].position.y);
    if (d.norm() < 1e-6) continue;
    if (previous.norm() > 0.0) {
      worst = std::max(worst, std::abs(std::atan2(
                                  previous.x() * d.y() - previous.y() * d.x(),
                                  previous.dot(d))) *
                                  180.0 / M_PI);
    }
    previous = d;
  }
  return worst;
}

double flat(double, double) { return 0.0; }

}  // namespace

class PlannerNavigationTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

namespace {
// Work injected once inside the active deadline, checked every millisecond.
class CooperativeBudgetWork {
 public:
  CooperativeBudgetWork(PlannerNode& node, double seconds, std::atomic<bool>* admitted = nullptr,
                        bool during_polish = false)
      : scope_([&, seconds, admitted, during_polish] {
          if (!entered && PlannerNodeTestPeer::deadlineActive(node) &&
              (!during_polish || PlannerNodeTestPeer::polishing(node))) {
            entered = true;
            if (admitted) *admitted = true;
            const auto until = std::chrono::steady_clock::now() +
                std::chrono::duration<double>(seconds);
            while (std::chrono::steady_clock::now() < until) {
              mgg::planningCheckpoint();
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
          }
          return false;
        }) {}
  std::atomic<bool> entered{false};
 private:
  mgg::PlanningCancellationScope scope_;
};
std::shared_ptr<Service::Request> budgetRequest(PlannerNode& node, int objective) {
  auto request = std::make_shared<Service::Request>();
  request->objective = objective;
  PlannerNodeTestPeer::objectiveIdentity(node, *request);
  request->goal.position.x = -2;
  request->goal.orientation.w = 1;
  return request;
}
}  // namespace

TEST_F(PlannerNavigationTest, ObjectiveBudgetsAllowReturnHomeBeyondHalfSecond) {
  MolaTerrainProduct product(.1, -4, 3, -3, 3, flat, {}, 2.5);
  auto node = botmanNode("budget_home", product);
  PlannerNodeTestPeer::standAt(*node, 0, 0, 0, M_PI, 1);
  PlannerNodeTestPeer::budgetRoadmap(*node);
  CooperativeBudgetWork work(*node, .75);
  auto response = std::make_shared<Service::Response>();
  const auto start = std::chrono::steady_clock::now();
  PlannerNodeTestPeer::objective(*node, budgetRequest(*node, Service::Request::RETURN_HOME), response);
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  EXPECT_GT(elapsed, .5);
  EXPECT_LT(elapsed, 5.0);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  ASSERT_FALSE(response->path.empty());
  EXPECT_NEAR(response->path.back().position.x, -2, .001);
}

TEST_F(PlannerNavigationTest, ObjectiveBudgetsNavigateHasOneSecondNotHalfSecond) {
  MolaTerrainProduct product(.1, -4, 3, -3, 3, flat, {}, 2.5);
  for (double duration : {.65, 1.2}) {
    SCOPED_TRACE(duration);
    auto node = botmanNode("budget_navigate", product);
    PlannerNodeTestPeer::standAt(*node, 0, 0, 0, M_PI, 1);
    CooperativeBudgetWork work(*node, duration);
    const auto start = std::chrono::steady_clock::now();
    auto response = navigate(*node, -2, 0);
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    if (duration < 1) {
      EXPECT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
      EXPECT_FALSE(response->path.empty());
    } else {
      EXPECT_EQ(response->status, Service::Response::UNREACHABLE);
      EXPECT_TRUE(response->path.empty());
      EXPECT_NE(response->reason.find("budget exceeded"), std::string::npos);
      EXPECT_GE(elapsed, 1.0);
      EXPECT_LT(elapsed, 1.2);
    }
  }
}

TEST_F(PlannerNavigationTest, ObjectiveBudgetsExplorationStillStopsAtHalfSecond) {
  MolaTerrainProduct product(.1, -4, 3, -3, 3, flat, {}, 2.5);
  auto node = botmanNode("budget_explore", product);
  PlannerNodeTestPeer::standAt(*node, 0, 0, 0, M_PI, 1);
  CooperativeBudgetWork work(*node, 1.2);
  auto response = std::make_shared<mgg_msgs::srv::PlannerSrv::Response>();
  const auto start = std::chrono::steady_clock::now();
  PlannerNodeTestPeer::exploreImpl(*node, response);
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  EXPECT_TRUE(work.entered);
  EXPECT_GE(elapsed, .5);
  EXPECT_LT(elapsed, .7);
  EXPECT_TRUE(response->path.empty());
}

TEST_F(PlannerNavigationTest, ObjectiveBudgetsCancelThreeSecondHomeWithin200ms) {
  using namespace std::chrono_literals;
  MolaTerrainProduct product(.1, -4, 3, -3, 3, flat, {}, 2.5);
  auto node = botmanNode("budget_cancel_home", product);
  PlannerNodeTestPeer::standAt(*node, 0, 0, 0, M_PI, 1);
  PlannerNodeTestPeer::budgetRoadmap(*node);
  auto response = std::make_shared<Service::Response>();
  std::atomic<bool> entered{false};
  auto future = std::async(std::launch::async, [&] {
    CooperativeBudgetWork work(*node, 3.0, &entered);
    PlannerNodeTestPeer::objective(*node, budgetRequest(*node, Service::Request::RETURN_HOME), response);
  });
  const auto by = std::chrono::steady_clock::now() + 2s;
  while (!entered && std::chrono::steady_clock::now() < by) std::this_thread::sleep_for(1ms);
  std::this_thread::sleep_for(700ms);
  EXPECT_EQ(future.wait_for(0ms), std::future_status::timeout);
  const auto start = std::chrono::steady_clock::now();
  PlannerNodeTestPeer::cancel(*node);
  EXPECT_EQ(future.wait_for(200ms), std::future_status::ready);
  future.get();
  EXPECT_LT(std::chrono::steady_clock::now()-start, 200ms);
  EXPECT_TRUE(response->path.empty());
  EXPECT_EQ(response->status, Service::Response::BLOCKED) << response->reason;
}

TEST_F(PlannerNavigationTest, ObjectiveBudgetsKeepCertifiedGlobalRouteWhenPolishExpires) {
  MolaTerrainProduct product(.1, -4, 3, -3, 3, flat, {}, 2.5);
  for (int objective : {Service::Request::RETURN_HOME, Service::Request::NAVIGATE}) {
    auto node = botmanNode("budget_anytime", product);
    PlannerNodeTestPeer::standAt(*node, 0, 0, 0, M_PI, 1);
    PlannerNodeTestPeer::budgetRoadmap(*node);
    PlannerNodeTestPeer::forceGlobalFallback(*node, true);
    bool expired = false;
    mgg::PlanningCancellationScope instrument([&] {
      if (!expired && PlannerNodeTestPeer::polishing(*node)) {
        expired = true;
        PlannerNodeTestPeer::expireRequest(*node);
      }
      return false;
    });
    auto response = std::make_shared<Service::Response>();
    PlannerNodeTestPeer::objective(*node, budgetRequest(*node, objective), response);
    EXPECT_TRUE(expired);
    ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
    ASSERT_GE(response->path.size(), 2u);
    EXPECT_NEAR(response->path.back().position.x, -2, .001);
    EXPECT_NE(response->reason.find("certified"), std::string::npos);
  }
}

TEST_F(PlannerNavigationTest, ObjectiveBudgetsGlobalNavigateRunsBeyondOneSecond) {
  MolaTerrainProduct product(.1, -4, 3, -3, 3, flat, {}, 2.5);
  auto node = botmanNode("budget_global_navigate", product);
  PlannerNodeTestPeer::standAt(*node, 0, 0, 0, M_PI, 1);
  PlannerNodeTestPeer::budgetRoadmap(*node);
  PlannerNodeTestPeer::forceGlobalFallback(*node, true);
  CooperativeBudgetWork work(*node, 1.2, nullptr, /*during_polish=*/true);
  const auto start = std::chrono::steady_clock::now();
  const auto response = navigate(*node, -2, 0);
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  EXPECT_TRUE(work.entered);
  EXPECT_GT(elapsed, 1.0);
  EXPECT_LT(elapsed, 5.0);
  EXPECT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  EXPECT_FALSE(response->path.empty());
}

TEST_F(PlannerNavigationTest, ObjectiveBudgetsFiveSecondExpiryKeepsCertifiedHome) {
  MolaTerrainProduct product(.1, -4, 3, -3, 3, flat, {}, 2.5);
  auto node = botmanNode("budget_five_second_home", product);
  PlannerNodeTestPeer::standAt(*node, 0, 0, 0, M_PI, 1);
  PlannerNodeTestPeer::budgetRoadmap(*node);
  CooperativeBudgetWork work(*node, 6.0, nullptr, /*during_polish=*/true);
  const auto start = std::chrono::steady_clock::now();
  auto response = std::make_shared<Service::Response>();
  PlannerNodeTestPeer::objective(*node, budgetRequest(*node, Service::Request::RETURN_HOME), response);
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  EXPECT_TRUE(work.entered);
  EXPECT_GE(elapsed, 5.0);
  EXPECT_LT(elapsed, 5.2);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  ASSERT_FALSE(response->path.empty());
  EXPECT_NEAR(response->path.back().position.x, -2, .001);
  EXPECT_NE(response->reason.find("certified"), std::string::npos);
}

TEST_F(PlannerNavigationTest, ObjectiveBudgetsAreIndependentParameters) {
  MolaTerrainProduct product(.1, -4, 3, -3, 3, flat, {}, 2.5);
  auto node = botmanNode("budget_parameters", product, {
      rclcpp::Parameter("lattice_time_budget_s", .02),
      rclcpp::Parameter("navigate_time_budget_s", .03),
      rclcpp::Parameter("global_route_time_budget_s", .2)});
  EXPECT_DOUBLE_EQ(node->get_parameter("lattice_time_budget_s").as_double(), .02);
  EXPECT_DOUBLE_EQ(node->get_parameter("navigate_time_budget_s").as_double(), .03);
  EXPECT_DOUBLE_EQ(node->get_parameter("global_route_time_budget_s").as_double(), .2);
  PlannerNodeTestPeer::standAt(*node, 0, 0, 0, M_PI, 1);
  PlannerNodeTestPeer::budgetRoadmap(*node);
  CooperativeBudgetWork work(*node, .08);
  auto response = std::make_shared<Service::Response>();
  PlannerNodeTestPeer::objective(*node, budgetRequest(*node, Service::Request::RETURN_HOME), response);
  EXPECT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  EXPECT_FALSE(response->path.empty());
}

TEST_F(PlannerNavigationTest, ObjectiveBudgetsCancellationDuringPolishNeverReturnsRoute) {
  MolaTerrainProduct product(.1, -4, 3, -3, 3, flat, {}, 2.5);
  auto node = botmanNode("budget_polish_cancel", product);
  PlannerNodeTestPeer::standAt(*node, 0, 0, 0, M_PI, 1);
  PlannerNodeTestPeer::budgetRoadmap(*node);
  mgg::PlanningCancellationScope instrument([&] { return PlannerNodeTestPeer::polishing(*node); });
  auto response = std::make_shared<Service::Response>();
  PlannerNodeTestPeer::objective(*node, budgetRequest(*node, Service::Request::RETURN_HOME), response);
  EXPECT_EQ(response->status, Service::Response::BLOCKED);
  EXPECT_TRUE(response->path.empty());
}

// Review r0 P1: the base flow polished and then certified. A raw graph route
// failing the post-spin join must not hide its passing shortcut.
TEST_F(PlannerNavigationTest, ObjectiveBudgetsAcceptPolishedRouteWhoseRawCornersFail) {
  MolaTerrainProduct product(.1, -4, 3, -3, 3, flat, {}, 2.5);
  for (int objective : {Service::Request::RETURN_HOME, Service::Request::NAVIGATE}) {
    SCOPED_TRACE(objective);
    auto node = botmanNode("budget_raw_corner", product);
    PlannerNodeTestPeer::standAt(*node, 0, 0, 0, M_PI, 1);
    PlannerNodeTestPeer::rawCornerRoadmap(*node);
    PlannerNodeTestPeer::forceGlobalFallback(*node, true);
    auto response = std::make_shared<Service::Response>();
    PlannerNodeTestPeer::objective(*node, budgetRequest(*node, objective), response);
    ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
    ASSERT_GE(response->path.size(), 2u);
    EXPECT_NEAR(response->path.back().position.x, -2, .001);
    EXPECT_EQ(response->reason.find("certified"), std::string::npos) << response->reason;
    // The up-front raw check here runs the post-spin join to its corner.
    RecordProperty(objective == Service::Request::RETURN_HOME
                       ? "home_raw_route_certification_ms" : "navigate_raw_route_certification_ms",
                   std::to_string(PlannerNodeTestPeer::rawRouteCertificationMs(*node)));
    // The polished route never visits the doubled-back vertex behind the robot.
    for (const auto& pose : response->path) EXPECT_LT(pose.position.x, .05);
  }
}

TEST_F(PlannerNavigationTest, ObjectiveBudgetsExpiryDuringPolishCertificationKeepsCertifiedRoute) {
  MolaTerrainProduct product(.1, -4, 3, -3, 3, flat, {}, 2.5);
  auto node = botmanNode("budget_polish_certify_expiry", product);
  PlannerNodeTestPeer::standAt(*node, 0, 0, 0, M_PI, 1);
  PlannerNodeTestPeer::budgetRoadmap(*node);
  PlannerNodeTestPeer::resetPolish(*node);
  bool expired = false;
  mgg::PlanningCancellationScope instrument([&] {
    if (!expired && PlannerNodeTestPeer::certifyingPolish(*node)) {
      expired = true;
      PlannerNodeTestPeer::expireRequest(*node);
    }
    return false;
  });
  auto response = std::make_shared<Service::Response>();
  PlannerNodeTestPeer::objective(*node, budgetRequest(*node, Service::Request::RETURN_HOME), response);
  // Expired during the polished route's final checks, not its shortcut.
  ASSERT_TRUE(expired);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_NEAR(response->path.back().position.x, -2, .001);
  EXPECT_NE(response->reason.find("certified"), std::string::npos) << response->reason;
}

// Both routes fail: polish cut by the deadline leaves the raw route's reason.
TEST_F(PlannerNavigationTest, ObjectiveBudgetsBothRoutesFailingReportRawReasonAfterExpiry) {
  MolaTerrainProduct product(.1, -4, 3, -3, 3, flat, {}, 2.5);
  auto node = botmanNode("budget_both_fail", product);
  PlannerNodeTestPeer::standAt(*node, 0, 0, 0, M_PI, 1);
  PlannerNodeTestPeer::rawCornerRoadmap(*node);
  bool expired = false;
  mgg::PlanningCancellationScope instrument([&] {
    if (!expired && PlannerNodeTestPeer::polishing(*node)) {
      expired = true;
      PlannerNodeTestPeer::expireRequest(*node);
    }
    return false;
  });
  auto response = std::make_shared<Service::Response>();
  PlannerNodeTestPeer::objective(*node, budgetRequest(*node, Service::Request::RETURN_HOME), response);
  EXPECT_TRUE(expired);
  EXPECT_EQ(response->status, Service::Response::UNREACHABLE) << response->reason;
  EXPECT_TRUE(response->path.empty());
  EXPECT_NE(response->reason.find("post-spin reference cannot join"), std::string::npos)
      << response->reason;
  EXPECT_NE(response->reason.find("polish stopped at the request deadline"), std::string::npos)
      << response->reason;
}

TEST_F(PlannerNavigationTest, ObjectiveBudgetsCancelWhileCertifyingPolishNeverReturnsRoute) {
  MolaTerrainProduct product(.1, -4, 3, -3, 3, flat, {}, 2.5);
  auto node = botmanNode("budget_polish_certify_cancel", product);
  PlannerNodeTestPeer::standAt(*node, 0, 0, 0, M_PI, 1);
  PlannerNodeTestPeer::rawCornerRoadmap(*node);
  PlannerNodeTestPeer::resetPolish(*node);
  bool cancelled = false;
  mgg::PlanningCancellationScope instrument([&] {
    cancelled = cancelled || PlannerNodeTestPeer::certifyingPolish(*node);
    return cancelled;
  });
  auto response = std::make_shared<Service::Response>();
  PlannerNodeTestPeer::objective(*node, budgetRequest(*node, Service::Request::RETURN_HOME), response);
  // Cancelled during the polished route's final checks, not its shortcut.
  EXPECT_TRUE(cancelled);
  EXPECT_EQ(response->status, Service::Response::BLOCKED) << response->reason;
  EXPECT_TRUE(response->path.empty());
}

TEST_F(PlannerNavigationTest, HardwareNavigateFitsNorthDoorwayWithObservedBody) {
  MolaTerrainProduct product(.1, -5, 5, -3, 7, flat,
      {{-5, -.6, 1.5, 1.8, 2}, {.6, 5, 1.5, 1.8, 2}}, 2.0);
  auto node = botmanNode("hardware_doorway", product);
  PlannerNodeTestPeer::hardwarePolicy(*node);
  PlannerNodeTestPeer::standAt(*node, 0, 0, 0, M_PI/2, 1);
  const auto response = navigate(*node, 0, 4);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  EXPECT_LE(pathLength(response->path), 4.2);
}

TEST_F(PlannerNavigationTest, BotmanPostSpinJoinWitness) {
  const char* root=std::getenv("MGG_NAV_BENCH_PRODUCT");
  if(!root || !*root)GTEST_SKIP()<<"MGG_NAV_BENCH_PRODUCT is not set";
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("map.backend","mola_snapshot"),
      rclcpp::Parameter("map.resolution",.1),
      rclcpp::Parameter("map.mola.peer_root",std::string(root)),
      rclcpp::Parameter("roadmap_rebuild.robot_id","botman_0"),
      rclcpp::Parameter("map.mola.snapshot_ttl_sec",60.0)});
  options.automatically_declare_parameters_from_overrides(true);
  auto node=std::make_shared<PlannerNode>(options);
  PlannerNodeTestPeer::configureBotman(*node);
  ASSERT_TRUE(PlannerNodeTestPeer::serveFixture(*node,root));
  PlannerNodeTestPeer::sensorPolicy(*node);
  for(bool east:{true,false}) {
    PlannerNodeTestPeer::standAt(*node,east?1.13:-3.17,east?.07:-.01,-.61,east?-.08:3.10,east?1:2);
    const Eigen::Vector3d from=east?Eigen::Vector3d(.811023,.095573,.333106):Eigen::Vector3d(-2.850277,-.023306,.379410);
    const Eigen::Vector3d to=east?Eigen::Vector3d(.631601,.109957,.366102):Eigen::Vector3d(-2.670572,-.030785,.372892);
    auto row=PlannerNodeTestPeer::joinWitness(*node,from,to);
    row["start_x"]=east?1.13:-3.17;
    std::cout<<"JOIN_WITNESS "<<row.dump()<<'\n';
    EXPECT_NE(row["projected_status"].get<int>(),int(mgg::ProjectedEdgeStatus::kAdmissible));
  }
}

TEST_F(PlannerNavigationTest, BotmanFixtureFullServicesBenchmark) {
  const char* root = std::getenv("MGG_NAV_BENCH_PRODUCT");
  if (!root || !*root) GTEST_SKIP() << "MGG_NAV_BENCH_PRODUCT is not set";
  const int repeat = std::getenv("MGG_SERVICE_BENCH_REPEAT")
      ? std::max(1, std::atoi(std::getenv("MGG_SERVICE_BENCH_REPEAT"))) : 3;
  // Optional ground gain model "degrees,range_m" for the Orin benchmark, as
  // the PlanningParams overlay would set it (e.g. "7.5,10"); default dense.
  double gain_step_deg = 0, gain_range_m = 0;
  if (const char* model = std::getenv("MGG_GROUND_GAIN_MODEL")) {
    char* end = nullptr;
    gain_step_deg = std::strtod(model, &end);
    ASSERT_EQ(*end, ',') << "MGG_GROUND_GAIN_MODEL must be degrees,range_m: " << model;
    gain_range_m = std::strtod(end + 1, &end);
    ASSERT_EQ(*end, '\0') << "MGG_GROUND_GAIN_MODEL must be degrees,range_m: " << model;
    ASSERT_TRUE(std::isfinite(gain_step_deg) && gain_step_deg >= 0 && gain_step_deg <= 180 &&
                std::isfinite(gain_range_m) && gain_range_m >= 0) << model;
  }
  for (int run = 0; run < repeat; ++run) for (const std::string policy : {"strict", "above_sensor_fov", "legacy_relaxed"}) {
    const bool allow_unknown = policy == "legacy_relaxed";
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("map.backend", "mola_snapshot"),
        rclcpp::Parameter("map.resolution", .1),
        rclcpp::Parameter("map.mola.peer_root", std::string(root)),
        rclcpp::Parameter("roadmap_rebuild.robot_id", "botman_0"),
        rclcpp::Parameter("map.mola.snapshot_ttl_sec", 60.0)});
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<PlannerNode>(options);
    PlannerNodeTestPeer::configureBotman(*node);
    PlannerNodeTestPeer::groundGainModel(*node, gain_step_deg, gain_range_m);
    if (const char* slice = std::getenv("MGG_SERVICE_LATTICE_MS"))
      PlannerNodeTestPeer::explorationSlice(*node, std::atof(slice)/1000);
    if (!allow_unknown) PlannerNodeTestPeer::hardwarePolicy(*node);
    ASSERT_TRUE(PlannerNodeTestPeer::serveFixture(*node, root));
    if (policy == "above_sensor_fov") PlannerNodeTestPeer::sensorPolicy(*node);
    PlannerNodeTestPeer::standAt(*node, 0, 0, -.61, M_PI, run+1);
    for (const std::string mode : {"navigate_local", "navigate_global_fallback", "navigate_15m", "explore_origin", "explore_1.13", "explore_-3.17"}) {
      PlannerNodeTestPeer::forceGlobalFallback(*node, mode == "navigate_global_fallback");
      PlannerNodeTestPeer::rawRouteCertificationMs(*node) = -1;
      const auto started = std::chrono::steady_clock::now();
      int status = 0;
      std::size_t poses = 0;
      std::string reason;
      const bool exploring = mode.find("explore_") == 0;
      if (exploring) {
        const double x = mode == "explore_1.13" ? 1.13 : mode == "explore_-3.17" ? -3.17 : 0;
        const double y = mode == "explore_1.13" ? .07 : mode == "explore_-3.17" ? -.01 : 0;
        const double heading = mode == "explore_1.13" ? -.08 : mode == "explore_-3.17" ? 3.10 : M_PI;
        PlannerNodeTestPeer::standAt(*node, x, y, -.61, heading, run*10 + (x == 0 ? 2 : x > 0 ? 3 : 4));
        const auto result = PlannerNodeTestPeer::explore(*node);
        status = result->status; poses = result->path.size();
      } else {
        // Force the observed -2 m goal through the global route too;
        // separately measure the unobserved 15 m goal's clean refusal.
        const auto result = navigate(*node, mode == "navigate_15m" ? -15 : -2, 0);
        status = result->status; poses = result->path.size(); reason = result->reason;
      }
      const double elapsed = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
      nlohmann::json row{{"service",mode},{"repeat",run},{"unknown_body_policy",policy},{"allow_unknown_body",allow_unknown},
          {"exploration_budget_ms",500},{"navigate_budget_ms",1000},
          {"global_route_budget_ms",5000},{"total_ms",elapsed},{"status",status},{"poses",poses},
          {"reason",reason},{"x86_budget_met",elapsed <= 300}};
      if (exploring) row.update(PlannerNodeTestPeer::explorationMetrics(*node));
      row.update(PlannerNodeTestPeer::groundGainModel(*node));
      if (mode == "navigate_global_fallback") row["global_diagnostic"] = PlannerNodeTestPeer::globalLinks(*node);
      if (PlannerNodeTestPeer::rawRouteCertificationMs(*node) >= 0)
        row["raw_route_certification_ms"] = PlannerNodeTestPeer::rawRouteCertificationMs(*node);
      std::printf("SERVICE_BENCH %s\n", row.dump().c_str());
      EXPECT_LT(elapsed, 1000);  // hard envelope, not a claim of the x86 target
    }
  }
}

TEST_F(PlannerNavigationTest, OwnBodyHistoryCannotCrossComponentOrEpoch) {
  MolaTerrainProduct product(.1,-3,5,-3,3,flat,{},.5);
  auto node=botmanNode("history_identity",product);
  PlannerNodeTestPeer::sensorPolicy(*node);
  PlannerNodeTestPeer::standAt(*node,0,0,0,0,1);
  PlannerNodeTestPeer::historyIdentity(*node,"component:test",1);
  EXPECT_EQ(PlannerNodeTestPeer::historicalUnknown(*node),mgg::VoxelStatus::kFree);
  PlannerNodeTestPeer::historyIdentity(*node,"component:other",1);
  EXPECT_EQ(PlannerNodeTestPeer::historicalUnknown(*node),mgg::VoxelStatus::kUnknown);
  PlannerNodeTestPeer::historyIdentity(*node,"component:test",2);
  EXPECT_EQ(PlannerNodeTestPeer::historicalUnknown(*node),mgg::VoxelStatus::kUnknown);
}

TEST_F(PlannerNavigationTest, StrictPolicyHasNoOwnBodyUnknownExceptions) {
  MolaTerrainProduct product(.1,-3,5,-3,3,flat,{},.5);
  auto node=botmanNode("strict_own_body",product);
  PlannerNodeTestPeer::standAt(*node,0,0,0,0,1);
  EXPECT_FALSE(PlannerNodeTestPeer::hasOwnBodyMask(*node,true));
  EXPECT_FALSE(PlannerNodeTestPeer::hasOwnBodyMask(*node,false));
}

TEST_F(PlannerNavigationTest, PhysicalEvidenceEndsAtLidarHousingNotPlanningCeiling) {
  MolaTerrainProduct product(.1,-3,5,-3,3,flat,{},.5);
  auto node=botmanNode("physical_z",product);
  PlannerNodeTestPeer::sensorPolicy(*node);
  PlannerNodeTestPeer::standAt(*node,0,0,0,0,1);
  PlannerNodeTestPeer::historyIdentity(*node,"component:test",1);
  EXPECT_EQ(PlannerNodeTestPeer::historicalUnknown(*node,.64,.659),mgg::VoxelStatus::kFree);
  EXPECT_EQ(PlannerNodeTestPeer::historicalUnknown(*node,.661,.69),mgg::VoxelStatus::kUnknown);
}

TEST_F(PlannerNavigationTest, RequestHistoryNeverConcatenatesSeparateReads) {
  MolaTerrainProduct product(.1,-3,6,-3,3,flat,{},.5);
  auto node=botmanNode("fresh_history",product);
  PlannerNodeTestPeer::sensorPolicy(*node);
  PlannerNodeTestPeer::standAt(*node,0,0,0,0,1);
  PlannerNodeTestPeer::historyIdentity(*node,"component:test",1,2);
  EXPECT_EQ(PlannerNodeTestPeer::historicalUnknown(*node),mgg::VoxelStatus::kFree);
  PlannerNodeTestPeer::historyIdentity(*node,"component:test",1,5);
  EXPECT_EQ(PlannerNodeTestPeer::historicalUnknown(*node),mgg::VoxelStatus::kUnknown);
}

TEST_F(PlannerNavigationTest, TurnedDepartureStartsAtPostSpinReference) {
  MolaTerrainProduct product(.1,-3,5,-3,3,flat,{},2.0);
  auto node=botmanNode("turned_departure",product);
  PlannerNodeTestPeer::standAt(*node,0,0,0,M_PI/2,1);
  const auto departure=PlannerNodeTestPeer::turnedDeparture(*node);
  ASSERT_GT(departure.path.size(),1u);
  ASSERT_NE(departure.turn,0);
  const double yaw=departure.path.front()[3];
  const Eigen::Vector2d centre = departure.path.front().head<2>() +
      Eigen::Vector2d(-.16*std::cos(yaw),-.16*std::sin(yaw));
  EXPECT_NEAR(centre.x(),0,1e-9);
  EXPECT_NEAR(centre.y(),-.16,1e-9);
  EXPECT_GT(departure.path.front().head<2>().norm(),.001);
}

TEST_F(PlannerNavigationTest, StoredReverseStartsAtRotatedOffsetReference) {
  MolaTerrainProduct product(.1,-4,2,-2,2,flat,{{0,.1,-.6,-.5,1.5}},2.0);
  auto node=botmanNode("stored_spin",product);
  PlannerNodeTestPeer::standAt(*node,0,0,0,.2,1);
  const auto path=PlannerNodeTestPeer::storedOffsetReverse(*node);
  ASSERT_GE(path.size(),2u);
  const auto& post=path.front();
  EXPECT_NEAR(post.x()-.16*std::cos(post[3]),-.16*std::cos(.2),1e-8);
  EXPECT_NEAR(post.y()-.16*std::sin(post[3]),-.16*std::sin(.2),1e-8);
  const double travel=std::atan2(path[1].y()-post.y(),path[1].x()-post.x());
  EXPECT_NEAR(std::remainder(travel-post[3]-M_PI,2*M_PI),0,1e-8);
}

TEST_F(PlannerNavigationTest, RefugeBandUsesReverseChassisYawNotStoredYaw) {
  // Reverse travel is +X, so the chassis centre is +.16 from the reference.
  // The obstacle behind the arrival lies in a yaw-zero circle, not that circle.
  MolaTerrainProduct product(.1,-1,3,-2,2,flat,{{.1,.2,.5,.6,1.5}},2.0);
  auto node=botmanNode("refuge_chassis_yaw",product);
  PlannerNodeTestPeer::standAt(*node,0,0,0,0,1);
  EXPECT_TRUE(PlannerNodeTestPeer::refugeBand(*node,0));
  EXPECT_TRUE(PlannerNodeTestPeer::refugeBand(*node,M_PI));
}

TEST_F(PlannerNavigationTest, OffsetStartSpinPreservesTurnBackHysteresis) {
  MolaTerrainProduct product(.1,-3,5,-3,3,flat,{},2.0);
  auto node=botmanNode("spin_hysteresis",product);
  PlannerNodeTestPeer::standAt(*node,0,0,0,M_PI,1);
  EXPECT_TRUE(PlannerNodeTestPeer::postSpinRecordsTurnBack(*node));
}

TEST_F(PlannerNavigationTest, BoundedPolicyWithoutPhysicalPairHasNoMask) {
  MolaTerrainProduct product(.1,-3,5,-3,3,flat,{},.5);
  auto node=botmanNode("bounded_no_physical",product);
  PlannerNodeTestPeer::sensorPolicy(*node);
  PlannerNodeTestPeer::standAt(*node,0,0,0,0,1);
  EXPECT_FALSE(PlannerNodeTestPeer::hasOwnBodyMask(*node,false,false));
}

TEST_F(PlannerNavigationTest, StartSpinMovesSentReferenceButNotChassisCentre) {
  MolaTerrainProduct product(.1,-3,5,-3,3,flat,{},2.0);
  auto node=botmanNode("post_spin",product);
  PlannerNodeTestPeer::standAt(*node,0,0,0,M_PI,1);
  std::vector<mgg::StateVec> path{{0,0,.935,M_PI},{.25,0,.935,0},{1,0,.935,0}};
  ASSERT_TRUE(PlannerNodeTestPeer::postSpin(*node,path));
  ASSERT_GE(path.size(),2u);
  EXPECT_NEAR(path.front().x(),.32,1e-6);
  EXPECT_NEAR(path.front().y(),0,1e-6);
  EXPECT_NEAR(path.front()[3],0,1e-6);
  EXPECT_GT(path[1].x(),path.front().x());
}

TEST_F(PlannerNavigationTest, OwnDrivenHistoryAdmitsReturnSweepOnlyWithEvidence) {
  MolaTerrainProduct product(.1,-3,5,-3,3,flat,{},.7,
      [](double x,double y,double z) {
        return x > 1.8 && x < 1.9 && y > 0 && y < .1 && z > .4 && z < .5;
      });
  auto node=botmanNode("driven_return",product);
  PlannerNodeTestPeer::sensorPolicy(*node);
  PlannerNodeTestPeer::standAt(*node,0,0,0,0,1);
  PlannerNodeTestPeer::historyIdentity(*node,"component:test",1);
  EXPECT_FALSE(PlannerNodeTestPeer::drivenReturn(*node,false));
  EXPECT_TRUE(PlannerNodeTestPeer::drivenReturn(*node,true));
}

TEST_F(PlannerNavigationTest, InvalidSensorPolicyWarnsAtParameterLoad) {
  MolaTerrainProduct product(.1,-1,1,-1,1,flat);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("map.backend","mola_snapshot"),
      rclcpp::Parameter("map.resolution",.1),
      rclcpp::Parameter("map.mola.peer_root",product.root().string()),
      rclcpp::Parameter("unknown_body_policy","above_sensor_fov"),
      rclcpp::Parameter("unknown_body_sensor","missing_sensor")});
  options.automatically_declare_parameters_from_overrides(true);
  testing::internal::CaptureStderr();
  { auto node=std::make_shared<PlannerNode>(options); }
  const auto log=testing::internal::GetCapturedStderr();
  EXPECT_NE(log.find("CANNOT APPLY"),std::string::npos);
  EXPECT_NE(log.find("USING STRICT UNKNOWN POLICY"),std::string::npos);
}

TEST_F(PlannerNavigationTest, GlobalQueryConnectorUsesUpperAirPolicyWithoutPersisting) {
  MolaTerrainProduct product(.1,-5,3,-3,3,flat,{},.7);
  auto node = botmanNode("global_query_upper", product);
  PlannerNodeTestPeer::sensorPolicy(*node);
  PlannerNodeTestPeer::standAt(*node,0,0,0,M_PI,1);
  PlannerNodeTestPeer::mappedGlobalPath(*node);
  PlannerNodeTestPeer::forceGlobalFallback(*node,true);
  const auto before = PlannerNodeTestPeer::globalSize(*node);
  EXPECT_FALSE(PlannerNodeTestPeer::globalUsesUpperPolicy(*node));
  const auto result = navigate(*node,-2,0);
  EXPECT_EQ(result->status, Service::Response::SUCCEEDED) << result->reason;
  EXPECT_NE(result->reason.find("request-only goal connector: above_sensor_fov"), std::string::npos);
  EXPECT_EQ(PlannerNodeTestPeer::globalSize(*node), before);
}

TEST_F(PlannerNavigationTest, GlobalQueryConnectorKeepsLowerUnknownAndOccupiedBlocked) {
  for (bool occupied : {false,true}) {
    MolaTerrainProduct product(.1,-5,3,-3,3,flat,
        occupied ? std::vector<Block>{{-2.2,-1.8,-3,3,.75}} : std::vector<Block>{},
        occupied ? .9 : .5);
    auto node = botmanNode("global_query_blocked", product);
    PlannerNodeTestPeer::sensorPolicy(*node);
    PlannerNodeTestPeer::standAt(*node,0,0,0,M_PI,1);
    PlannerNodeTestPeer::mappedGlobalPath(*node);
    PlannerNodeTestPeer::forceGlobalFallback(*node,true);
    EXPECT_NE(navigate(*node,-2,0)->status, Service::Response::SUCCEEDED);
  }
}

TEST_F(PlannerNavigationTest, FullyObservedGoalBeyondLocalLatticeUsesGlobalGraph) {
  MolaTerrainProduct product(.1,-5,3,-3,3,flat,{},2.0);
  auto node = botmanNode("global_observed", product);
  PlannerNodeTestPeer::hardwarePolicy(*node);
  PlannerNodeTestPeer::standAt(*node,0,0,0,M_PI,1);
  PlannerNodeTestPeer::mappedGlobalPath(*node);
  PlannerNodeTestPeer::forceGlobalFallback(*node,true);
  EXPECT_EQ(navigate(*node,-2,0)->status, Service::Response::SUCCEEDED);
}

TEST_F(PlannerNavigationTest, HardwareShortcutDoesNotCrossUnknownBodyVolume) {
  MolaTerrainProduct product(.1, -4, 6, -4, 6, flat);
  auto node = botmanNode("hardware_shortcut", product);
  PlannerNodeTestPeer::hardwarePolicy(*node);
  std::vector<mgg::StateVec> path{{0, 0, .935, 0}, {0, 2, .935, 0}, {2, 2, .935, 0}};
  PlannerNodeTestPeer::shortcut(*node, path);
  double length = 0;
  for (std::size_t i = 1; i < path.size(); ++i)
    length += (path[i].head<3>() - path[i-1].head<3>()).norm();
  EXPECT_NEAR(length, 4.0, 1e-9);
}

TEST_F(PlannerNavigationTest, SensorFovPolicyToleratesOnlyUpperUnknownAir) {
  for (double observed : {.4, .5, .599, .61, .7, 2.0}) {
    SCOPED_TRACE(observed);
    MolaTerrainProduct product(.1, -3, 5, -3, 3, flat, {}, observed);
    auto node = botmanNode("sensor_unknown_air", product);
    PlannerNodeTestPeer::sensorPolicy(*node);
    PlannerNodeTestPeer::standAt(*node, 0, 0, 0, 0, 1);
    const auto response = navigate(*node, 2, 0);
    EXPECT_EQ(response->status == Service::Response::SUCCEEDED, observed >= .61);
  }
}

TEST_F(PlannerNavigationTest, SensorFovPolicyBlocksTableEdgeAt075Metres) {
  MolaTerrainProduct product(.1, -3, 5, -3, 3, flat, {{1.4, 1.8, -3, 3, .75}}, .9);
  auto node = botmanNode("sensor_table_edge", product);
  PlannerNodeTestPeer::sensorPolicy(*node);
  PlannerNodeTestPeer::standAt(*node, 0, 0, 0, 0, 1);
  EXPECT_NE(navigate(*node, 2, 0)->status, Service::Response::SUCCEEDED);
}

TEST_F(PlannerNavigationTest, SensorFovPolicyWithoutConfiguredMountFailsClosed) {
  MolaTerrainProduct product(.1, -3, 5, -3, 3, flat, {}, .7);
  auto node = botmanNode("sensor_missing_mount", product);
  PlannerNodeTestPeer::sensorPolicy(*node, 0);
  PlannerNodeTestPeer::standAt(*node, 0, 0, 0, 0, 1);
  EXPECT_NE(navigate(*node, 2, 0)->status, Service::Response::SUCCEEDED);
}

TEST_F(PlannerNavigationTest, SensorFovPolicyBelowBodyBottomFailsClosed) {
  for (double mount : {.1, .3}) {
    MolaTerrainProduct product(.1, -3, 5, -3, 3, flat, {}, .7);
    auto node = botmanNode("sensor_low_mount", product);
    PlannerNodeTestPeer::sensorPolicy(*node, mount);
    PlannerNodeTestPeer::standAt(*node, 0, 0, 0, 0, 1);
    EXPECT_NE(navigate(*node, 2, 0)->status, Service::Response::SUCCEEDED);
  }
}

TEST_F(PlannerNavigationTest, RequestBudgetRefusesWithoutPublishingAPartialRoute) {
  MolaTerrainProduct product(0.1, -8, 8, -8, 8,
                              [](double, double) { return -0.61; });
  auto node = botmanNode("budget_navigation", product);
  PlannerNodeTestPeer::standAt(*node, 0, 0, -0.61, 0, 1);
  PlannerNodeTestPeer::setBudget(*node, 1e-9);
  const auto started = std::chrono::steady_clock::now();
  const auto response = navigate(*node, 4, 0);
  EXPECT_EQ(response->status, Service::Response::UNREACHABLE);
  EXPECT_TRUE(response->path.empty());
  EXPECT_NE(response->reason.find("planning budget exceeded"), std::string::npos);
  EXPECT_NE(response->reason.find("ms"), std::string::npos);
  EXPECT_LT(std::chrono::steady_clock::now() - started, std::chrono::seconds(1));
}

TEST_F(PlannerNavigationTest, RequestBudgetPreservesOuterCancellation) {
  MolaTerrainProduct product(0.1, -8, 8, -8, 8,
                              [](double, double) { return -0.61; });
  auto node = botmanNode("cancel_navigation", product);
  PlannerNodeTestPeer::standAt(*node, 0, 0, -0.61, 0, 1);
  mgg::PlanningCancellationScope cancelled([] { return true; });
  const auto response = navigate(*node, 4, 0);
  EXPECT_EQ(response->status, Service::Response::BLOCKED);
  EXPECT_TRUE(response->path.empty());
  EXPECT_NE(response->reason.find("cancelled"), std::string::npos);
  EXPECT_EQ(response->reason.find("budget exceeded"), std::string::npos);
}

TEST_F(PlannerNavigationTest, ATwoMetreMoveOnOpenFloorComesOutStraight) {
  // The robot faces 0.3 rad, so its lattice is turned against the goal's
  // bearing (0.93 rad): the lattice route is a staircase, which the
  // shortcut must straighten.
  MolaTerrainProduct product(0.1, -4.0, 7.0, -4.0, 5.0, flat);
  auto node = botmanNode("botman_open_2m", product);
  PlannerNodeTestPeer::standAt(*node, 0.0, 0.0, 0.0, 0.3, 1.0);
  const auto response = navigate(*node, 1.2, 1.6);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  EXPECT_NEAR(response->path.back().position.x, 1.2, 0.05);
  EXPECT_NEAR(response->path.back().position.y, 1.6, 0.05);
  EXPECT_LE(pathLength(response->path), 2.1);
  EXPECT_LE(maxCornerDeg(response->path), 10.0);
}

TEST_F(PlannerNavigationTest, AGoalFourMetresAwayInObservedFreeSpaceSucceeds) {
  MolaTerrainProduct product(0.1, -4.0, 8.0, -4.0, 6.0, flat);
  auto node = botmanNode("botman_open_4m", product);
  PlannerNodeTestPeer::standAt(*node, 0.0, 0.0, 0.0, -0.2, 1.0);
  const auto response = navigate(*node, 3.3, 2.27);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  EXPECT_NEAR(response->path.back().position.x, 3.3, 0.05);
  EXPECT_NEAR(response->path.back().position.y, 2.27, 0.05);
  EXPECT_LE(pathLength(response->path), 4.2);
  EXPECT_LE(maxCornerDeg(response->path), 10.0);
}

TEST_F(PlannerNavigationTest, AGoalBehindAWallRoutesAroundIt) {
  // A wall 3 m wide across the way, 1.8 m tall; the goal is behind it.
  const Block wall{2.0, 2.3, -1.5, 1.5, 1.8};
  MolaTerrainProduct product(0.1, -3.0, 7.0, -4.0, 4.0, flat, {wall});
  auto node = botmanNode("botman_wall", product);
  PlannerNodeTestPeer::standAt(*node, 0.0, 0.0, 0.0, 0.0, 1.0);
  const auto response = navigate(*node, 4.5, 0.0);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  // No pose's body centre within half the planning width of the wall.
  for (const auto& pose : response->path) {
    const double dx = std::max({wall.x0 - pose.position.x, 0.0,
                                pose.position.x - wall.x1});
    const double dy = std::max({wall.y0 - pose.position.y, 0.0,
                                pose.position.y - wall.y1});
    EXPECT_GT(std::hypot(dx, dy), 0.414)
        << "pose at " << pose.position.x << ", " << pose.position.y;
  }
  // Retain the original path-quality bound with the oriented body and
  // turn rule intact, rather than relaxing it to accommodate the detour.
  std::printf("wall_path_length_m=%.6f\n", pathLength(response->path));
  EXPECT_LT(pathLength(response->path), 6.8);
}

TEST_F(PlannerNavigationTest, ARampRouteStaysWithinTheSlopeLimits) {
  // Floor, then a 15 degree ramp up from x = 1 to x = 4, then a deck.
  const double grade = std::tan(15.0 * M_PI / 180.0);
  const auto ramp = [grade](double x, double) {
    return std::clamp(x - 1.0, 0.0, 3.0) * grade;
  };
  MolaTerrainProduct product(0.1, -3.0, 7.0, -3.0, 3.0, ramp);
  auto node = botmanNode("botman_ramp", product);
  PlannerNodeTestPeer::standAt(*node, 0.0, 0.0, 0.0, 0.0, 1.0);
  const auto response = navigate(*node, 5.0, 0.4, ramp(5.0, 0.4));
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  const double limit = std::tan(27.0 * M_PI / 180.0);
  for (std::size_t i = 1; i < response->path.size(); ++i) {
    const auto& a = response->path[i - 1].position;
    const auto& b = response->path[i].position;
    const double run = std::hypot(b.x - a.x, b.y - a.y);
    if (run < 0.2) continue;
    EXPECT_LE(std::abs(b.z - a.z) / run, limit + 1e-6);
    // Driving height over the ground under it.
    EXPECT_NEAR(b.z - ramp(b.x, b.y), 0.935, 0.12);
  }
  EXPECT_LE(maxCornerDeg(response->path), 10.0);
}

TEST_F(PlannerNavigationTest, RobotSizedMapObstacleIsUnreachableNotPeerBlocked) {
  // An occupied robot-sized block at the goal has no special identity or
  // retry-only peer status. A later free map admits the same objective.
  for (const bool occupied : {true, false}) {
    SCOPED_TRACE(occupied);
    MolaTerrainProduct product(.1, -3, 5, -3, 3, flat,
        occupied ? std::vector<Block>{{1.6, 2.4, -.3, .3, .6}} : std::vector<Block>{});
    auto node = botmanNode("mapped_robot_obstacle", product);
    PlannerNodeTestPeer::standAt(*node, 0, 0, 0, 0, 1);
    const auto response = navigate(*node, 2, 0);
    EXPECT_EQ(response->status, occupied ? Service::Response::UNREACHABLE
                                        : Service::Response::SUCCEEDED) << response->reason;
    EXPECT_EQ(node->count_subscribers("peer_bodies"), 0u);
    EXPECT_EQ(node->count_subscribers("aerial_peer_bodies"), 0u);
  }
}

}  // namespace mgg_ros
