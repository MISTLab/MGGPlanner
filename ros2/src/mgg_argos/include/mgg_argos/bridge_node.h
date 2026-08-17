/// The ROS 2 end of the ARGoS bridge.
///
/// Listens on a Unix socket, turns each observation into ROS messages
/// (/clock, per-robot odometry, point cloud and TF) and replies with whatever
/// path the planner has published for each robot. ARGoS blocks on that reply,
/// so nothing is ever published into a void: see include/mgg_argos/protocol.h.
///
/// Why a bridge rather than linking ARGoS against rclcpp: ARGoS plugins are
/// loaded into a process that must keep building without ROS present, and the
/// two have conflicting ideas about who owns the main loop. A socket keeps the
/// dependency one-directional.

#ifndef MGG_ARGOS_BRIDGE_NODE_H
#define MGG_ARGOS_BRIDGE_NODE_H

#include <array>
#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace mgg_argos {

class BridgeNode : public rclcpp::Node {
 public:
  explicit BridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~BridgeNode() override;

 private:
  /// Everything published or remembered for one robot.
  struct Robot {
    std::string id;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_pub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    /// Graph geometry for drawing, from the planner's marker array. Display
    /// only: nothing the robot does depends on it.
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
        markers_sub;
    /// Flattened edge endpoints, six floats per segment. Guarded by
    /// overlay_mutex_.
    std::vector<float> graph_edges;
    /// Global swarm graph edges, six floats per segment.
    std::vector<float> global_graph_edges;
    /// Graph merge beacon & connection edges, six floats per segment.
    std::vector<float> merge_edges;
    /// Points of interest (frontiers, viewpoints), three floats per point.
    /// Guarded by overlay_mutex_.
    std::vector<float> overlay_points;
    /// The path last forwarded, kept so it can be redrawn every tick rather
    /// than only on the tick it changed.
    std::vector<float> drawn_path;


    /// Latest path from the planner, and whether it still has to be sent.
    /// Guarded by paths_mutex_.
    nav_msgs::msg::Path path;
    std::vector<std::array<double, 3>> path_points;
    bool path_pending = false;
    /// The last path forwarded, so a republished identical one does not
    /// restart the robot at waypoint zero every tick.
    ///
    /// Both the stamp and the waypoints, because the stamp alone is not
    /// enough: an unstamped path carries 0, which is also what "nothing
    /// forwarded yet" would look like, so a zero-stamped path would be
    /// dropped forever. Publishers that leave the header alone are common
    /// enough - ros2 topic pub does it by default - that this has to work.
    rclcpp::Time last_path_stamp{0, 0, RCL_ROS_TIME};
    std::vector<std::array<double, 3>> last_path_points;
    bool have_forwarded = false;
    bool stop_pending = false;
  };

  /// One decoded per-robot observation.
  struct Observation {
    std::string id;
    bool has_odometry = false;
    double odometry[7] = {0, 0, 0, 1, 0, 0, 0};
    bool has_truth = false;
    double truth[7] = {0, 0, 0, 1, 0, 0, 0};
    bool has_imu = false;
    double imu[6] = {0, 0, 0, 0, 0, 0};
    bool has_scan = false;
    std::uint32_t rings = 0;
    std::uint32_t azimuths = 0;
    float elevation_min = 0.0f;
    float elevation_max = 0.0f;
    float max_range = 0.0f;
    std::vector<float> ranges;
    std::vector<std::uint8_t> hits;
  };

  void listenAndServe();
  void serveConnection(int fd);
  /// Reads exactly n bytes; false when the peer closed or errored.
  bool recvAll(int fd, void* data, size_t n);
  bool sendAll(int fd, const void* data, size_t n);

  void publish(std::uint32_t tick, std::uint32_t ticks_per_second,
               const std::vector<Observation>& observations);
  void publishCloud(const Robot& robot, const Observation& obs,
                    const rclcpp::Time& stamp);
  /// Builds the reply for this tick into `out`.
  void buildCommands(std::uint32_t tick, std::vector<std::uint8_t>& out);
  void onPath(const std::string& id, const nav_msgs::msg::Path& msg);
  void onMarkers(const std::string& id,
                 const visualization_msgs::msg::MarkerArray& msg);
  /// Appends this robot's overlay blocks to the reply.
  void appendOverlays(Robot& robot, std::vector<std::uint8_t>& out);

  std::string socket_path_;
  std::string world_frame_;
  std::string base_frame_suffix_;
  std::string lidar_frame_suffix_;
  std::vector<double> lidar_translation_;
  /// Miss endpoints go out at max_range * this. Strictly above 1 so octomap
  /// treats them as beyond-range measurements, which clear the ray without
  /// marking an obstacle at the end of it.
  double miss_range_scale_ = 1.02;
  /// Wall-clock seconds per simulated second. 0 runs as fast as possible.
  double real_time_factor_ = 1.0;
  bool publish_ground_truth_ = false;

  std::vector<Robot> robots_;
  std::map<std::string, size_t> robot_index_;
  std::mutex paths_mutex_;
  /// Guards the overlay geometry, which arrives on marker callbacks and is
  /// read when the reply is built.
  std::mutex overlay_mutex_;
  /// Whether to send overlay geometry at all. Off costs nothing; on costs a
  /// few hundred kB a tick for a large graph.
  bool send_overlays_ = true;
  /// Cap on edges sent per robot per tick, so a graph that grows without
  /// bound cannot swamp the socket.
  int max_overlay_edges_ = 20000;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  std::thread server_thread_;
  std::atomic<bool> running_{true};
  int listen_fd_ = -1;
  /// Wall-clock reference for pacing, set when the first tick arrives.
  std::chrono::steady_clock::time_point pace_origin_;
  std::uint32_t pace_origin_tick_ = 0;
  bool pace_started_ = false;
};

}  // namespace mgg_argos

#endif  // MGG_ARGOS_BRIDGE_NODE_H
