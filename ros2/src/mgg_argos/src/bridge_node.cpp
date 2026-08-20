#include "mgg_argos/bridge_node.h"

#include "mgg_argos/protocol.h"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace mgg_argos {
namespace {

using namespace protocol;

/// Reads a little-endian scalar out of a byte buffer and advances the cursor.
template <typename T>
bool take(const std::vector<std::uint8_t>& buf, size_t& at, T& out) {
  if (at + sizeof(T) > buf.size()) return false;
  std::memcpy(&out, buf.data() + at, sizeof(T));
  at += sizeof(T);
  return true;
}

geometry_msgs::msg::TransformStamped makeTransform(
    const std::string& parent, const std::string& child,
    const rclcpp::Time& stamp, const double pose[7]) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.translation.x = pose[0];
  tf.transform.translation.y = pose[1];
  tf.transform.translation.z = pose[2];
  tf.transform.rotation.w = pose[3];
  tf.transform.rotation.x = pose[4];
  tf.transform.rotation.y = pose[5];
  tf.transform.rotation.z = pose[6];
  return tf;
}

}  // namespace

/****************************************/

BridgeNode::BridgeNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mgg_argos_bridge", options) {
  socket_path_ = declare_parameter("socket_path", std::string("/tmp/mgg_argos.sock"));
  world_frame_ = declare_parameter("world_frame", std::string("map"));
  base_frame_suffix_ = declare_parameter("base_frame_suffix", std::string("base_link"));
  lidar_frame_suffix_ = declare_parameter("lidar_frame_suffix", std::string("lidar"));
  lidar_translation_ = declare_parameter("lidar_translation",
                                         std::vector<double>{0.0, 0.0, 0.3});
  miss_range_scale_ = declare_parameter("miss_range_scale", miss_range_scale_);
  real_time_factor_ = declare_parameter("real_time_factor", real_time_factor_);
  publish_ground_truth_ =
      declare_parameter("publish_ground_truth", publish_ground_truth_);
  send_overlays_ = declare_parameter("send_overlays", send_overlays_);
  max_overlay_edges_ =
      static_cast<int>(declare_parameter("max_overlay_edges",
                                         max_overlay_edges_));
  const auto ids =
      declare_parameter("robots", std::vector<std::string>{"r0"});
  if (lidar_translation_.size() != 3) {
    throw std::runtime_error("lidar_translation needs exactly 3 values");
  }
  if (miss_range_scale_ <= 1.0) {
    // At exactly 1.0 octomap's `norm() <= max_range` test passes and every
    // miss becomes a phantom obstacle in a shell around the robot.
    throw std::runtime_error("miss_range_scale must be greater than 1");
  }

  clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock",
                                                           rclcpp::ClockQoS());
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  robots_.resize(ids.size());
  for (size_t i = 0; i < ids.size(); ++i) {
    Robot& robot = robots_[i];
    robot.id = ids[i];
    robot_index_[robot.id] = i;
    const std::string ns = robot.id + "/";
    robot.odom_pub =
        create_publisher<nav_msgs::msg::Odometry>(ns + "odometry", rclcpp::QoS(10));
    robot.cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
        ns + "pointcloud", rclcpp::SensorDataQoS());
    robot.imu_pub =
        create_publisher<sensor_msgs::msg::Imu>(ns + "imu", rclcpp::SensorDataQoS());
    if (publish_ground_truth_) {
      robot.truth_pub = create_publisher<nav_msgs::msg::Odometry>(
          ns + "ground_truth", rclcpp::QoS(10));
    }
    const std::string id = robot.id;
    robot.path_sub = create_subscription<nav_msgs::msg::Path>(
        ns + "path", rclcpp::QoS(10),
        [this, id](const nav_msgs::msg::Path& msg) { onPath(id, msg); });
    if (send_overlays_) {
      // The planner publishes these only when something subscribes, so this
      // subscription is also what switches the marker work on.
      robot.markers_sub =
          create_subscription<visualization_msgs::msg::MarkerArray>(
              ns + "graph_markers", rclcpp::QoS(1),
              [this, id](const visualization_msgs::msg::MarkerArray& msg) {
                onMarkers(id, msg);
              });
    }
    // The lidar mount is fixed on the robot, so it is a static transform.
    // It has to agree with the <photorealistic_lidar position="..."> in the
    // experiment file; there is no way to discover it from this side.
    const double mount[7] = {lidar_translation_[0], lidar_translation_[1],
                             lidar_translation_[2], 1.0, 0.0, 0.0, 0.0};
    static_tf_broadcaster_->sendTransform(
        makeTransform(robot.id + "/" + base_frame_suffix_,
                      robot.id + "/" + lidar_frame_suffix_,
                      rclcpp::Time(0, 0, RCL_ROS_TIME), mount));
  }

  server_thread_ = std::thread([this] { listenAndServe(); });
  RCLCPP_INFO(get_logger(), "listening on %s for %zu robot(s)",
              socket_path_.c_str(), robots_.size());
}

/****************************************/

BridgeNode::~BridgeNode() {
  running_ = false;
  if (listen_fd_ >= 0) {
    ::shutdown(listen_fd_, SHUT_RDWR);
    ::close(listen_fd_);
    listen_fd_ = -1;
  }
  if (server_thread_.joinable()) server_thread_.join();
  ::unlink(socket_path_.c_str());
}

/****************************************/

void BridgeNode::onPath(const std::string& id, const nav_msgs::msg::Path& msg) {
  const auto it = robot_index_.find(id);
  if (it == robot_index_.end()) return;
  std::lock_guard<std::mutex> lock(paths_mutex_);
  Robot& robot = robots_[it->second];
  const rclcpp::Time stamp(msg.header.stamp);
  if (msg.poses.empty()) {
    robot.stop_pending = true;
    robot.path_pending = false;
    return;
  }
  // Only forward genuinely new paths. Sending the same one again would reset
  // the robot to waypoint zero, and it would shuffle on the spot forever.
  std::vector<std::array<double, 3>> points;
  points.reserve(msg.poses.size());
  for (const auto& pose : msg.poses) {
    points.push_back({pose.pose.position.x, pose.pose.position.y,
                      pose.pose.position.z});
  }
  const bool changed = !robot.have_forwarded ||
                       stamp != robot.last_path_stamp ||
                       points != robot.last_path_points;
  if (robot.path_pending || changed) {
    robot.path = msg;
    robot.path_points = std::move(points);
    robot.path_pending = true;
  }
}

/****************************************/

void BridgeNode::onMarkers(const std::string& id,
                           const visualization_msgs::msg::MarkerArray& msg) {
  const auto it = robot_index_.find(id);
  if (it == robot_index_.end()) return;
  std::vector<float> local_edges;
  std::vector<float> global_edges;
  std::vector<float> merge_edges;
  std::vector<float> points;
  for (const auto& marker : msg.markers) {
    if (marker.type == visualization_msgs::msg::Marker::LINE_LIST) {
      std::vector<float>* target = &local_edges;
      if (marker.ns == "global_graph_edges") {
        target = &global_edges;
      } else if (marker.ns == "graph_merges") {
        target = &merge_edges;
      }
      const size_t pairs = marker.points.size() / 2;
      target->reserve(target->size() + pairs * 6);
      for (size_t i = 0; i + 1 < marker.points.size(); i += 2) {
        if (target == &local_edges &&
            static_cast<int>(target->size() / 6) >= max_overlay_edges_) break;
        const auto& a = marker.points[i];
        const auto& b = marker.points[i + 1];
        target->push_back(static_cast<float>(a.x));
        target->push_back(static_cast<float>(a.y));
        target->push_back(static_cast<float>(a.z));
        target->push_back(static_cast<float>(b.x));
        target->push_back(static_cast<float>(b.y));
        target->push_back(static_cast<float>(b.z));
      }
    } else if (marker.type == visualization_msgs::msg::Marker::POINTS ||
               marker.type == visualization_msgs::msg::Marker::SPHERE_LIST) {
      points.reserve(points.size() + marker.points.size() * 3);
      for (const auto& p : marker.points) {
        if (static_cast<int>(points.size() / 3) >= 2000) break;
        points.push_back(static_cast<float>(p.x));
        points.push_back(static_cast<float>(p.y));
        points.push_back(static_cast<float>(p.z));
      }
    }
  }
  std::lock_guard<std::mutex> lock(overlay_mutex_);
  robots_[it->second].graph_edges = std::move(local_edges);
  robots_[it->second].global_graph_edges = std::move(global_edges);
  robots_[it->second].merge_edges = std::move(merge_edges);
  robots_[it->second].overlay_points = std::move(points);
}

/****************************************/

void BridgeNode::appendOverlays(Robot& robot, std::vector<std::uint8_t>& out) {
  const auto append = [&out](const void* data, size_t n) {
    const auto* in = static_cast<const std::uint8_t*>(data);
    out.insert(out.end(), in, in + n);
  };
  const auto block = [&](std::uint8_t type, const std::vector<float>& payload,
                         std::uint32_t count) {
    append(&type, 1);
    const auto length =
        std::uint32_t(sizeof(std::uint32_t) + payload.size() * sizeof(float));
    append(&length, sizeof(length));
    append(&count, sizeof(count));
    if (!payload.empty()) {
      append(payload.data(), payload.size() * sizeof(float));
    }
  };

  if (!send_overlays_) {
    const std::uint8_t none = 0;
    append(&none, 1);
    return;
  }
  std::lock_guard<std::mutex> lock(overlay_mutex_);
  std::uint8_t count = 0;
  if (!robot.drawn_path.empty()) ++count;
  if (!robot.graph_edges.empty()) ++count;
  if (!robot.global_graph_edges.empty()) ++count;
  if (!robot.merge_edges.empty()) ++count;
  if (!robot.overlay_points.empty()) ++count;
  append(&count, 1);

  if (!robot.drawn_path.empty()) {
    block(kOverlayPath, robot.drawn_path,
          std::uint32_t(robot.drawn_path.size() / 3));
  }
  if (!robot.graph_edges.empty()) {
    block(kOverlayGraphEdges, robot.graph_edges,
          std::uint32_t(robot.graph_edges.size() / 6));
  }
  if (!robot.global_graph_edges.empty()) {
    block(kOverlayGlobalGraph, robot.global_graph_edges,
          std::uint32_t(robot.global_graph_edges.size() / 6));
  }
  if (!robot.merge_edges.empty()) {
    block(kOverlayMerge, robot.merge_edges,
          std::uint32_t(robot.merge_edges.size() / 6));
  }
  if (!robot.overlay_points.empty()) {
    block(kOverlayPoints, robot.overlay_points,
          std::uint32_t(robot.overlay_points.size() / 3));
  }
}



/****************************************/

void BridgeNode::listenAndServe() {
  ::unlink(socket_path_.c_str());
  listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    RCLCPP_FATAL(get_logger(), "cannot create socket: %s", ::strerror(errno));
    return;
  }
  struct sockaddr_un addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  if (socket_path_.size() >= sizeof(addr.sun_path)) {
    RCLCPP_FATAL(get_logger(), "socket path is too long");
    return;
  }
  std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);
  if (::bind(listen_fd_, reinterpret_cast<struct sockaddr*>(&addr),
             sizeof(addr)) < 0) {
    RCLCPP_FATAL(get_logger(), "cannot bind %s: %s", socket_path_.c_str(),
                 ::strerror(errno));
    return;
  }
  /* bind() applies the umask, and this node runs as root inside a container
   * while ARGoS connects from the host as an ordinary user. Connecting to a
   * Unix socket needs write permission on it, so the default 0755 leaves the
   * simulator with "Permission denied". The uf_link and swarm_slam bridges
   * both widen their socket the same way for the same reason. */
  if (::chmod(socket_path_.c_str(), 0777) < 0) {
    RCLCPP_WARN(get_logger(), "cannot chmod %s: %s; the simulator may not be "
                "able to connect", socket_path_.c_str(), ::strerror(errno));
  }
  if (::listen(listen_fd_, 1) < 0) {
    RCLCPP_FATAL(get_logger(), "cannot listen: %s", ::strerror(errno));
    return;
  }
  while (running_) {
    const int fd = ::accept(listen_fd_, nullptr, nullptr);
    if (fd < 0) {
      if (!running_) break;
      if (errno == EINTR) continue;
      RCLCPP_ERROR(get_logger(), "accept failed: %s", ::strerror(errno));
      break;
    }
    RCLCPP_INFO(get_logger(), "ARGoS connected");
    serveConnection(fd);
    ::close(fd);
    RCLCPP_INFO(get_logger(), "ARGoS disconnected");
    // A new run can connect to the same bridge without a restart.
    pace_started_ = false;
  }
}

/****************************************/

bool BridgeNode::recvAll(int fd, void* data, size_t n) {
  auto* out = static_cast<std::uint8_t*>(data);
  size_t got = 0;
  while (got < n) {
    const ssize_t r = ::recv(fd, out + got, n - got, 0);
    if (r == 0) return false;
    if (r < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    got += static_cast<size_t>(r);
  }
  return true;
}

bool BridgeNode::sendAll(int fd, const void* data, size_t n) {
  const auto* in = static_cast<const std::uint8_t*>(data);
  size_t sent = 0;
  while (sent < n) {
    const ssize_t w = ::send(fd, in + sent, n - sent, MSG_NOSIGNAL);
    if (w <= 0) {
      if (errno == EINTR) continue;
      return false;
    }
    sent += static_cast<size_t>(w);
  }
  return true;
}

/****************************************/

void BridgeNode::serveConnection(int fd) {
  std::vector<std::uint8_t> reply;
  while (running_ && rclcpp::ok()) {
    char magic[4];
    if (!recvAll(fd, magic, sizeof(magic))) return;
    if (std::memcmp(magic, kObservationMagic, 4) != 0) {
      RCLCPP_ERROR(get_logger(), "bad observation magic; stream out of sync");
      return;
    }
    std::uint16_t version = 0;
    std::uint32_t tick = 0, ticks_per_second = 0, robot_count = 0;
    if (!recvAll(fd, &version, sizeof(version))) return;
    if (version != kVersion) {
      RCLCPP_FATAL(get_logger(), "ARGoS speaks protocol version %u, this build "
                   "speaks %u", version, kVersion);
      return;
    }
    if (!recvAll(fd, &tick, sizeof(tick))) return;
    if (!recvAll(fd, &ticks_per_second, sizeof(ticks_per_second))) return;
    if (!recvAll(fd, &robot_count, sizeof(robot_count))) return;

    std::vector<Observation> observations(robot_count);
    for (std::uint32_t r = 0; r < robot_count; ++r) {
      Observation& obs = observations[r];
      std::uint8_t id_length = 0;
      if (!recvAll(fd, &id_length, 1)) return;
      obs.id.resize(id_length);
      if (id_length > 0 && !recvAll(fd, obs.id.data(), id_length)) return;
      std::uint8_t block_count = 0;
      if (!recvAll(fd, &block_count, 1)) return;
      for (std::uint8_t b = 0; b < block_count; ++b) {
        std::uint8_t type = 0;
        std::uint32_t length = 0;
        if (!recvAll(fd, &type, 1)) return;
        if (!recvAll(fd, &length, sizeof(length))) return;
        std::vector<std::uint8_t> payload(length);
        if (length > 0 && !recvAll(fd, payload.data(), length)) return;
        size_t at = 0;
        switch (type) {
          case kBlockOdometry:
            obs.has_odometry = true;
            for (double& v : obs.odometry) take(payload, at, v);
            break;
          case kBlockGroundTruth:
            obs.has_truth = true;
            for (double& v : obs.truth) take(payload, at, v);
            break;
          case kBlockImu:
            obs.has_imu = true;
            for (double& v : obs.imu) take(payload, at, v);
            break;
          case kBlockLidar: {
            obs.has_scan = true;
            take(payload, at, obs.rings);
            take(payload, at, obs.azimuths);
            take(payload, at, obs.elevation_min);
            take(payload, at, obs.elevation_max);
            take(payload, at, obs.max_range);
            const size_t rays = size_t(obs.rings) * obs.azimuths;
            obs.ranges.resize(rays);
            obs.hits.resize(rays);
            for (size_t i = 0; i < rays; ++i) take(payload, at, obs.ranges[i]);
            if (at + rays <= payload.size()) {
              std::memcpy(obs.hits.data(), payload.data() + at, rays);
            }
            break;
          }
          default:
            // Length-prefixed precisely so an unknown block can be skipped
            // rather than desynchronising the stream.
            break;
        }
      }
    }

    publish(tick, ticks_per_second, observations);

    reply.clear();
    buildCommands(tick, reply);
    if (!sendAll(fd, reply.data(), reply.size())) return;

    // Pace the run against wall-clock. Without this the simulation advances
    // as fast as this node can publish, and the planner - which works off
    // /clock - would be handed simulated minutes per wall-clock second and
    // plan on data it never had time to consume.
    if (real_time_factor_ > 0.0 && ticks_per_second > 0) {
      if (!pace_started_) {
        pace_origin_ = std::chrono::steady_clock::now();
        pace_origin_tick_ = tick;
        pace_started_ = true;
      } else {
        const double elapsed_sim =
            double(tick - pace_origin_tick_) / double(ticks_per_second);
        const auto target = pace_origin_ + std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(elapsed_sim / real_time_factor_));
        std::this_thread::sleep_until(target);
      }
    }
  }
}

/****************************************/

void BridgeNode::publish(std::uint32_t tick, std::uint32_t ticks_per_second,
                         const std::vector<Observation>& observations) {
  if (ticks_per_second == 0) ticks_per_second = 1;
  const double seconds = double(tick) / double(ticks_per_second);
  const rclcpp::Time stamp(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);

  // /clock first: everything stamped below belongs to this tick, and a
  // subscriber that sees the data before the clock reaches it will reject it
  // as coming from the future.
  rosgraph_msgs::msg::Clock clock;
  clock.clock = stamp;
  clock_pub_->publish(clock);

  for (const Observation& obs : observations) {
    const auto it = robot_index_.find(obs.id);
    if (it == robot_index_.end()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "observation for unconfigured robot '%s'",
                           obs.id.c_str());
      continue;
    }
    const Robot& robot = robots_[it->second];
    const std::string base_frame = robot.id + "/" + base_frame_suffix_;

    if (obs.has_odometry) {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = world_frame_;
      odom.child_frame_id = base_frame;
      odom.pose.pose.position.x = obs.odometry[0];
      odom.pose.pose.position.y = obs.odometry[1];
      odom.pose.pose.position.z = obs.odometry[2];
      odom.pose.pose.orientation.w = obs.odometry[3];
      odom.pose.pose.orientation.x = obs.odometry[4];
      odom.pose.pose.orientation.y = obs.odometry[5];
      odom.pose.pose.orientation.z = obs.odometry[6];
      robot.odom_pub->publish(odom);
      // The planner localises against this, so the TF tree has to carry the
      // same estimate the odometry topic does, drift included.
      tf_broadcaster_->sendTransform(
          makeTransform(world_frame_, base_frame, stamp, obs.odometry));
    }
    if (obs.has_truth && robot.truth_pub) {
      nav_msgs::msg::Odometry truth;
      truth.header.stamp = stamp;
      truth.header.frame_id = world_frame_;
      truth.child_frame_id = base_frame + "_truth";
      truth.pose.pose.position.x = obs.truth[0];
      truth.pose.pose.position.y = obs.truth[1];
      truth.pose.pose.position.z = obs.truth[2];
      truth.pose.pose.orientation.w = obs.truth[3];
      truth.pose.pose.orientation.x = obs.truth[4];
      truth.pose.pose.orientation.y = obs.truth[5];
      truth.pose.pose.orientation.z = obs.truth[6];
      robot.truth_pub->publish(truth);
    }
    if (obs.has_imu) {
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = stamp;
      imu.header.frame_id = base_frame;
      imu.angular_velocity.x = obs.imu[0];
      imu.angular_velocity.y = obs.imu[1];
      imu.angular_velocity.z = obs.imu[2];
      imu.linear_acceleration.x = obs.imu[3];
      imu.linear_acceleration.y = obs.imu[4];
      imu.linear_acceleration.z = obs.imu[5];
      robot.imu_pub->publish(imu);
    }
    if (obs.has_scan) publishCloud(robot, obs, stamp);
  }
}

/****************************************/

void BridgeNode::publishCloud(const Robot& robot, const Observation& obs,
                              const rclcpp::Time& stamp) {
  const size_t rays = obs.ranges.size();
  if (rays == 0) return;

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = robot.id + "/" + lidar_frame_suffix_;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  struct Point3D {
    float x, y, z;
  };
  std::vector<Point3D> points;
  points.reserve(rays);

  const double elevation_step =
      obs.rings > 1
          ? (double(obs.elevation_max) - obs.elevation_min) / double(obs.rings - 1)
          : 0.0;
  const double azimuth_step =
      obs.azimuths > 0 ? 2.0 * M_PI / double(obs.azimuths) : 0.0;

  for (std::uint32_t a = 0; a < obs.azimuths; ++a) {

    const double azimuth = azimuth_step * a;
    const double cos_azimuth = std::cos(azimuth);
    const double sin_azimuth = std::sin(azimuth);
    for (std::uint32_t r = 0; r < obs.rings; ++r) {
      const size_t i = size_t(a) * obs.rings + r;
      const double elevation = obs.elevation_min + elevation_step * r;
      const double cos_elevation = std::cos(elevation);

      if (obs.hits[i]) {
        // Real obstacle or ground return (ignore any stray returns on robot chassis < 0.50m)
        if (obs.ranges[i] >= 0.50f) {
          const double range = double(obs.ranges[i]);
          points.push_back({
              float(range * cos_elevation * cos_azimuth),
              float(range * cos_elevation * sin_azimuth),
              float(range * std::sin(elevation))});
        }
      } else if (obs.ranges[i] > 0.0f) {
        // True open-air miss: push past max range so OctoMap clears free space.
        // Rays occluded by host robot body have range == 0.0 and are dropped
        // so they do not falsely clear the ground underneath the chassis.
        const double range = double(obs.max_range) * miss_range_scale_;
        points.push_back({
            float(range * cos_elevation * cos_azimuth),
            float(range * cos_elevation * sin_azimuth),
            float(range * std::sin(elevation))});
      }
    }
  }

  if (points.empty()) return;
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
  for (const auto& pt : points) {
    *it_x = pt.x;
    *it_y = pt.y;
    *it_z = pt.z;
    ++it_x;
    ++it_y;
    ++it_z;
  }

  robot.cloud_pub->publish(cloud);
}

/****************************************/

void BridgeNode::buildCommands(std::uint32_t tick,
                               std::vector<std::uint8_t>& out) {
  const auto append = [&out](const void* data, size_t n) {
    const auto* in = static_cast<const std::uint8_t*>(data);
    out.insert(out.end(), in, in + n);
  };
  append(kCommandMagic, 4);
  const std::uint16_t version = kVersion;
  append(&version, sizeof(version));
  append(&tick, sizeof(tick));
  const auto robot_count = std::uint32_t(robots_.size());
  append(&robot_count, sizeof(robot_count));

  std::lock_guard<std::mutex> lock(paths_mutex_);
  for (Robot& robot : robots_) {
    const auto id_length = std::uint8_t(robot.id.size());
    append(&id_length, 1);
    append(robot.id.data(), robot.id.size());
    if (robot.stop_pending) {
      const std::uint8_t type = kCommandStop;
      append(&type, 1);
      robot.stop_pending = false;
      {
        std::lock_guard<std::mutex> lock(overlay_mutex_);
        robot.drawn_path.clear();
      }
      appendOverlays(robot, out);
      continue;
    }
    if (!robot.path_pending) {
      const std::uint8_t type = kCommandNone;
      append(&type, 1);
      appendOverlays(robot, out);
      continue;
    }
    const std::uint8_t type = kCommandPath;
    append(&type, 1);
    const auto count = std::uint32_t(robot.path.poses.size());
    append(&count, sizeof(count));
    for (const auto& pose : robot.path.poses) {
      // Yaw is carried for completeness; the differential-drive follower
      // derives its heading from the direction of travel instead.
      const auto& q = pose.pose.orientation;
      const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      const double values[4] = {pose.pose.position.x, pose.pose.position.y,
                                pose.pose.position.z, yaw};
      append(values, sizeof(values));
    }
    robot.path_pending = false;
    robot.have_forwarded = true;
    robot.last_path_stamp = rclcpp::Time(robot.path.header.stamp);
    robot.last_path_points = robot.path_points;
    {
      std::lock_guard<std::mutex> lock(overlay_mutex_);
      robot.drawn_path.clear();
      robot.drawn_path.reserve(robot.path.poses.size() * 3);
      for (const auto& pose : robot.path.poses) {
        robot.drawn_path.push_back(float(pose.pose.position.x));
        robot.drawn_path.push_back(float(pose.pose.position.y));
        robot.drawn_path.push_back(float(pose.pose.position.z));
      }
    }
    appendOverlays(robot, out);
    RCLCPP_INFO(get_logger(), "forwarded a %u-waypoint path to %s", count,
                robot.id.c_str());
  }
}

}  // namespace mgg_argos
