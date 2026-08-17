/// Wire format between ARGoS and ROS 2.
///
/// ARGoS deliberately does not link against ROS 2, so the simulator and the
/// planner talk over a Unix domain socket and this header is the only thing
/// they share. It pulls in nothing but <cstdint> precisely so both ends can
/// include it: the ARGoS side stays ROS-free and the ROS side stays
/// ARGoS-free.
///
/// The exchange is lockstep. Every tick ARGoS writes one observation message
/// and then blocks reading one command message. The simulation cannot advance
/// until the planner has answered, which decouples the run from wall-clock
/// time: when planning takes two seconds, the simulator waits two seconds
/// instead of dropping the data on the floor. This is the same lockstep the
/// swarm_slam_bridge in argos3-examples uses; the protocol differs because
/// this one has to carry commands back.
///
/// All integers are little-endian and unaligned; all floats are IEEE 754.
/// Strings are a u8 length followed by that many bytes, not terminated.
///
/// Observation, ARGoS to ROS:
///
///   "MGGB"                 magic
///   u16   version          kVersion
///   u32   tick
///   u32   ticks_per_second so the far side can build stamps
///   u32   robot_count
///   per robot:
///     str   id             "r0", "r1", ...
///     u8    block_count
///     per block:
///       u8  type           one of EBlockType
///       u32 length         payload bytes
///       u8  payload[length]
///
/// The length prefix on every block is what makes this extensible: a reader
/// that does not know a block type skips it and keeps parsing. Phase 8 can add
/// RGB-D for cslam without either end needing to move in lockstep with the
/// other.
///
/// Command, ROS to ARGoS:
///
///   "MGGC"                 magic
///   u16   version          kVersion
///   u32   tick             echoes the observation, to catch desync
///   u32   robot_count
///   per robot:
///     str   id
///     u8    type           one of ECommandType
///     if kCommandPath:
///       u32 waypoint_count
///       per waypoint: f64 x, y, z, yaw   (world frame, radians)
///     u8    block_count    overlay blocks that follow
///     per block:
///       u8  type           one of EOverlayType
///       u32 length         payload bytes
///       u8  payload[length]
///
/// A robot may be omitted from the command message, which means the same as
/// kCommandNone: carry on with the current path.
///
/// The overlay blocks are length-prefixed for the same reason the observation
/// blocks are: a reader that does not know a type skips it, so the two ends
/// can be upgraded separately. They carry geometry purely so the simulator can
/// draw what the planner is thinking; nothing the robot does depends on them,
/// and a bridge that sends none is perfectly valid.

#ifndef MGG_ARGOS_PROTOCOL_H
#define MGG_ARGOS_PROTOCOL_H

#include <cstdint>

namespace mgg_argos {
namespace protocol {

/// Bumped whenever the layout changes incompatibly. Both ends check it and
/// refuse to run on a mismatch, because the failure mode of silently
/// misparsing a binary stream is far worse than not starting.
inline constexpr std::uint16_t kVersion = 3;

inline constexpr char kObservationMagic[4] = {'M', 'G', 'G', 'B'};
inline constexpr char kCommandMagic[4] = {'M', 'G', 'G', 'C'};

enum EBlockType : std::uint8_t {
  /// f64 x, y, z, qw, qx, qy, qz. The robot's estimated pose, from the
  /// drift-injected odometry sensor rather than ground truth.
  kBlockOdometry = 1,
  /// A full lidar revolution:
  ///   u32 num_rings
  ///   u32 num_azimuths
  ///   f32 elevation_min, elevation_max   (radians)
  ///   f32 max_range                      (meters)
  ///   f32 range[num_azimuths * num_rings]
  ///   u8  hit[num_azimuths * num_rings]
  /// Ordered azimuth-major with the ring index fastest, as a VLP-16 emits.
  /// The ray directions are not sent: the pattern is a regular grid, so the
  /// five scalars above define it exactly and save 300 kB a tick.
  ///
  /// Misses are carried as a flag rather than dropped. A ray that hits
  /// nothing still says the space it crossed is empty, which is most of what
  /// makes a frontier a frontier, but it must not leave an obstacle at its
  /// endpoint.
  kBlockLidar = 2,
  /// Reserved: RGB-D for Swarm-SLAM (phase 8).
  kBlockRgbd = 3,
  /// f64 wx, wy, wz, ax, ay, az.
  kBlockImu = 4,
  /// f64 x, y, z, qw, qx, qy, qz. Ground truth, for evaluation only; the
  /// planner never sees it.
  kBlockGroundTruth = 5,
};

/// Geometry the simulator draws so a human can watch the planner work. Purely
/// for display: it reaches the viewer's overlay layer and never a sensor.
enum EOverlayType : std::uint8_t {
  /// The path the robot is following:
  ///   u32 count
  ///   f32 x, y, z  per point
  /// Sent alongside a path command rather than derived from it, so that a
  /// bridge which drops overlays does not change what the robot does.
  kOverlayPath = 1,
  /// The planner's graph, as disjoint segments rather than a polyline:
  ///   u32 count
  ///   f32 x0, y0, z0, x1, y1, z1  per segment
  /// f32 because this is the largest thing on the wire by far - a local graph
  /// runs to thousands of edges every cycle - and it is being drawn, not
  /// measured.
  kOverlayGraphEdges = 2,
  /// Points of interest (frontiers, viewpoints), drawn as small crosses:
  ///   u32 count
  ///   f32 x, y, z  per point
  kOverlayPoints = 3,
};

enum ECommandType : std::uint8_t {
  /// Keep following the current path.
  kCommandNone = 0,
  /// Replace the current path with the waypoints that follow.
  kCommandPath = 1,
  /// Stop and discard the current path.
  kCommandStop = 2,
};

}  // namespace protocol
}  // namespace mgg_argos

#endif  // MGG_ARGOS_PROTOCOL_H
