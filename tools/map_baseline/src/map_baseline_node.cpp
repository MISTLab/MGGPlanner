// Records a behavioural baseline of the map layer (phase 0 of
// ROS2_PORT_PLAN.md).
//
// Why this exists: section 4 of the plan replaces voxblox with a ternary
// occupancy backend. TSDF truncation and log-odds raycasting disagree about
// free space near surfaces, so frontier positions and volumetric gain will
// shift. This harness pins down what the current voxblox implementation
// answers for a fixed, synthetic input, so the replacement can be compared
// against it rather than eyeballed.
//
// Everything is deterministic and simulator-free: the scene is analytic, the
// sensor poses are fixed, and no bag or Gazebo world is involved. The same
// scene generator can be reimplemented on the ROS 2 side to produce a
// directly comparable CSV.
//
// The map is fed through the normal ROS ingest path (a PointCloud2 on
// "pointcloud" plus TF) rather than by calling the integrator directly,
// because MapManagerVoxblox keeps its server private. That has the side
// benefit of exercising the real ingest path.

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "planner_common/map_manager_voxblox_impl.h"

namespace {

// ---------------------------------------------------------------- the scene
//
// A closed room with a floor, four walls, a ceiling, and one box obstacle
// off-centre. Chosen so the query battery below straddles all three voxel
// states: inside the room is free, the box and walls are occupied, and
// everything outside the room stays unknown because no ray reaches it.

struct Box {
  Eigen::Vector3d min_c;
  Eigen::Vector3d max_c;
};

const Box kRoom{{-10.0, -10.0, 0.0}, {10.0, 10.0, 4.0}};
const Box kObstacle{{2.0, -1.5, 0.0}, {4.0, 1.5, 2.0}};

// Slab method. Returns the smallest positive t where the ray enters or exits.
bool rayBoxHit(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
               const Box& box, bool inside, double* t_hit) {
  double t_near = -std::numeric_limits<double>::infinity();
  double t_far = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 3; ++i) {
    if (std::fabs(dir(i)) < 1e-12) {
      if (origin(i) < box.min_c(i) || origin(i) > box.max_c(i)) return false;
      continue;
    }
    double t1 = (box.min_c(i) - origin(i)) / dir(i);
    double t2 = (box.max_c(i) - origin(i)) / dir(i);
    if (t1 > t2) std::swap(t1, t2);
    t_near = std::max(t_near, t1);
    t_far = std::min(t_far, t2);
    if (t_near > t_far) return false;
  }
  // Looking outward from within the room we want the exit; for the obstacle
  // we want the entry.
  double t = inside ? t_far : t_near;
  if (t <= 1e-6) return false;
  *t_hit = t;
  return true;
}

// One synthetic lidar sweep from origin, in the sensor frame.
pcl::PointCloud<pcl::PointXYZ> sweep(const Eigen::Vector3d& origin,
                                     double max_range, int n_az, int n_el,
                                     double el_min, double el_max) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int ia = 0; ia < n_az; ++ia) {
    double az = -M_PI + 2.0 * M_PI * ia / n_az;
    for (int ie = 0; ie < n_el; ++ie) {
      double el = el_min + (el_max - el_min) * ie / std::max(1, n_el - 1);
      Eigen::Vector3d dir(std::cos(el) * std::cos(az),
                          std::cos(el) * std::sin(az), std::sin(el));
      double t_room, t_obs;
      bool hit_room = rayBoxHit(origin, dir, kRoom, true, &t_room);
      bool hit_obs = rayBoxHit(origin, dir, kObstacle, false, &t_obs);
      double t;
      if (hit_obs && (!hit_room || t_obs < t_room)) {
        t = t_obs;
      } else if (hit_room) {
        t = t_room;
      } else {
        continue;
      }
      if (t > max_range) continue;  // beyond range: leaves space unknown
      Eigen::Vector3d p = origin + t * dir;
      cloud.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    }
  }
  return cloud;
}

const char* statusName(MapManager::VoxelStatus s) {
  switch (s) {
    case MapManager::VoxelStatus::kUnknown: return "unknown";
    case MapManager::VoxelStatus::kOccupied: return "occupied";
    case MapManager::VoxelStatus::kFree: return "free";
  }
  return "?";
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_baseline_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string out_path;
  nh_private.param<std::string>("out", out_path, "/results/baseline.csv");

  // Sensor poses the synthetic sweeps are taken from. Fixed on purpose.
  const std::vector<Eigen::Vector3d> kSensorPoses = {
      {0.0, 0.0, 1.0}, {5.0, 5.0, 1.0}, {-5.0, -5.0, 1.5}, {0.0, 6.0, 1.0}};

  auto map = new MapManagerVoxblox<MapManagerVoxbloxServer,
                                   MapManagerVoxbloxVoxel>(nh, nh_private);

  ros::Publisher pc_pub =
      nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 10);
  tf2_ros::StaticTransformBroadcaster static_tf;

  // Let the server's subscriber connect before anything is sent.
  ros::Duration(2.0).sleep();
  ros::spinOnce();

  for (size_t i = 0; i < kSensorPoses.size(); ++i) {
    const Eigen::Vector3d& o = kSensorPoses[i];
    const std::string frame = "sensor_" + std::to_string(i);

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = frame;
    tf_msg.transform.translation.x = o.x();
    tf_msg.transform.translation.y = o.y();
    tf_msg.transform.translation.z = o.z();
    tf_msg.transform.rotation.w = 1.0;
    static_tf.sendTransform(tf_msg);
    ros::Duration(0.5).sleep();
    ros::spinOnce();

    // Points are expressed in the sensor frame, so subtract the origin.
    pcl::PointCloud<pcl::PointXYZ> cloud_w =
        sweep(o, 30.0, 360, 32, -M_PI / 6.0, M_PI / 6.0);
    pcl::PointCloud<pcl::PointXYZ> cloud_s;
    for (const auto& p : cloud_w) {
      cloud_s.push_back(
          pcl::PointXYZ(p.x - o.x(), p.y - o.y(), p.z - o.z()));
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud_s, msg);
    msg.header.frame_id = frame;
    msg.header.stamp = ros::Time::now();
    pc_pub.publish(msg);
    std::fprintf(stderr, "[baseline] sweep %zu from (%.1f, %.1f, %.1f): %zu points\n",
                 i, o.x(), o.y(), o.z(), cloud_s.size());

    // Integration happens in the server's callback.
    for (int k = 0; k < 40; ++k) {
      ros::spinOnce();
      ros::Duration(0.05).sleep();
    }
  }

  std::fprintf(stderr, "[baseline] map resolution: %f\n", map->getResolution());

  std::ofstream out(out_path);
  if (!out) {
    ROS_ERROR("[baseline] cannot write %s", out_path.c_str());
    return 1;
  }
  out << "# MGGPlanner map-layer baseline (voxblox TSDF, ROS 1)\n";
  out << "# scene: room " << kRoom.min_c.transpose() << " .. "
      << kRoom.max_c.transpose() << "; obstacle " << kObstacle.min_c.transpose()
      << " .. " << kObstacle.max_c.transpose() << "\n";
  out << std::unitbuf;  // never lose the tail of the file on a crash
  out << "kind,x,y,z,arg,result\n";
  out << "resolution,0,0,0,0," << map->getResolution() << "\n";

  // 1. Voxel status over a coarse lattice spanning inside and outside.
  int n_unknown = 0, n_free = 0, n_occupied = 0;
  for (double x = -14.0; x <= 14.0; x += 1.0) {
    for (double y = -14.0; y <= 14.0; y += 1.0) {
      for (double z = 0.5; z <= 4.5; z += 1.0) {
        Eigen::Vector3d p(x, y, z);
        auto s = map->getVoxelStatus(p);
        if (s == MapManager::VoxelStatus::kUnknown) ++n_unknown;
        else if (s == MapManager::VoxelStatus::kFree) ++n_free;
        else ++n_occupied;
        out << "voxel," << x << "," << y << "," << z << ",0," << statusName(s)
            << "\n";
      }
    }
  }
  std::fprintf(stderr, "[baseline] voxel lattice: %d unknown, %d free, %d occupied\n",
               n_unknown, n_free, n_occupied);

  // 2. Box status for a set of fixed centres and one fixed size.
  const Eigen::Vector3d box_size(0.8, 0.8, 0.8);
  for (double x = -8.0; x <= 8.0; x += 2.0) {
    Eigen::Vector3d c(x, 0.0, 1.0);
    out << "box," << c.x() << "," << c.y() << "," << c.z() << ",0.8,"
        << statusName(map->getBoxStatus(c, box_size, true)) << "\n";
  }

  // 3. Path status along segments that cross the obstacle and the walls.
  const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> segments = {
      {{-8.0, 0.0, 1.0}, {8.0, 0.0, 1.0}},    // through the obstacle
      {{0.0, -8.0, 1.0}, {0.0, 8.0, 1.0}},    // clear of it
      {{0.0, 0.0, 1.0}, {14.0, 0.0, 1.0}},    // out through a wall
      {{-5.0, -5.0, 1.0}, {5.0, 5.0, 1.0}},   // diagonal
  };
  for (size_t i = 0; i < segments.size(); ++i) {
    out << "path," << segments[i].first.x() << "," << segments[i].first.y()
        << "," << segments[i].first.z() << "," << i << ","
        << statusName(map->getPathStatus(segments[i].first, segments[i].second,
                                         box_size, true))
        << "\n";
  }

  // 4. Ray status from the room centre outward.
  Eigen::Vector3d ray_origin(0.0, 0.0, 1.0);
  for (int a = 0; a < 16; ++a) {
    double az = 2.0 * M_PI * a / 16.0;
    Eigen::Vector3d target = ray_origin + 12.0 * Eigen::Vector3d(std::cos(az),
                                                                 std::sin(az),
                                                                 0.0);
    out << "ray," << target.x() << "," << target.y() << "," << target.z()
        << "," << a << ","
        << statusName(map->getRayStatus(ray_origin, target, true)) << "\n";
  }

  // 5. Volumetric gain. This is the number the exploration planner actually
  //    acts on, so it is the most important row in the file.
  //
  // The sensor MUST be initialised through loadParams rather than by setting
  // fields directly: updateFrustumEndpoints() multiplies by the private
  // rot_B2S member, which only loadParams ever computes. Populating the
  // struct by hand leaves it uninitialised, the frustum endpoints come out as
  // garbage, and castRay then tries to allocate an astronomically long index
  // vector (std::bad_alloc). Loading the planner's own SensorParams block also
  // keeps the baseline faithful to the real configuration.
  SensorParamsBase sensor;
  const std::string sensor_ns = ros::this_node::getName() + "/SensorParams/VLP16";
  if (!sensor.loadParams(sensor_ns)) {
    ROS_ERROR("[baseline] could not load sensor params from %s",
              sensor_ns.c_str());
    return 1;
  }
  {
    // Cheap sanity check on the endpoint count before raycasting to them.
    StateVec probe(0.0, 0.0, 1.0, 0.0);
    std::vector<Eigen::Vector3d> probe_ep;
    sensor.getFrustumEndpoints(probe, probe_ep);
    std::fprintf(stderr, "[baseline] frustum endpoints: %zu\n",
                 probe_ep.size());
    if (probe_ep.empty() || probe_ep.size() > 1000000) {
      ROS_ERROR("[baseline] implausible endpoint count; aborting");
      return 1;
    }
  }

  const std::vector<StateVec> gain_states = {
      StateVec(0.0, 0.0, 1.0, 0.0), StateVec(6.0, 0.0, 1.0, 0.0),
      StateVec(0.0, 6.0, 1.0, 0.0), StateVec(-6.0, -6.0, 1.0, 0.0),
      StateVec(8.0, 8.0, 1.0, 0.0)};
  for (size_t i = 0; i < gain_states.size(); ++i) {
    StateVec s = gain_states[i];
    std::vector<Eigen::Vector3d> endpoints;
    sensor.getFrustumEndpoints(s, endpoints);
    std::tuple<int, int, int> gain_log;
    std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>> voxel_log;
    Eigen::Vector3d pos(s[0], s[1], s[2]);
    map->getScanStatus(pos, endpoints, gain_log, voxel_log, sensor);
    char buf[128];
    std::snprintf(buf, sizeof(buf), "unknown=%d;occupied=%d;free=%d",
                  std::get<0>(gain_log), std::get<1>(gain_log),
                  std::get<2>(gain_log));
    out << "gain," << s[0] << "," << s[1] << "," << s[2] << "," << i << ","
        << buf << "\n";
    std::fprintf(stderr, "[baseline] gain at (%.1f, %.1f, %.1f): %s\n",
                 s[0], s[1], s[2], buf);
  }

  out.close();
  std::fprintf(stderr, "[baseline] wrote %s\n", out_path.c_str());
  return 0;
}
