// Ground scan input (cross-plan contract C5a): the robot's own cloud,
// transformed into its odometry frame with TF at the cloud's stamp. Ground
// robots never use scan/points_odom.

#ifndef MGG_ROS_SCAN_INPUT_H_
#define MGG_ROS_SCAN_INPUT_H_

#include <optional>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/buffer_core.h>

#include "mgg_ros/local_planning_core.h"

namespace mgg {

/// The cloud's finite returns and the sensor frame's origin in `odom_frame`,
/// both from TF at the cloud's stamp; nullopt when TF has no transform at
/// that stamp (never the latest transform instead), the stamp is zero (TF's
/// "latest"), or the cloud has no float x/y/z fields.
std::optional<OdomScan> scanInOdom(const sensor_msgs::msg::PointCloud2& cloud,
                                   const tf2::BufferCore& buffer,
                                   const std::string& odom_frame);

}  // namespace mgg

#endif  // MGG_ROS_SCAN_INPUT_H_
