#pragma once

#include <string>

#include <eigen3/Eigen/Dense>
#include <pcl/search/impl/search.hpp>

#include "pcl_ros/point_cloud.h"
#include "planner_common/map_manager_voxblox_impl.h"
#include "planner_common/params.h"
#include "sensor_msgs/PointCloud2.h"

enum AdaptiveObbType { kPca = 0, kMvbb, kAabb };

class AdaptiveObb {
 public:
  AdaptiveObb(MapManagerVoxblox<MapManagerVoxbloxServer,
                                MapManagerVoxbloxVoxel>* map_manager)
      : map_manager_(map_manager) {}
  ~AdaptiveObb();

  void computeBounds(Eigen::Vector3d& min_val, Eigen::Vector3d& max_val,
                     const Eigen::Vector3d& offset,
                     const Eigen::Matrix3d& rot_w2b,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud);
  double computeVolume(const Eigen::Matrix3d& rot_w2b,
                       const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud);
  bool rotationsSafetyCheck(Eigen::Matrix3d& rot_w2b, Eigen::Vector3d& eig_val);

  void computeVarianceInReferenceFrame(
      Eigen::Vector3d& variance,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,
      const Eigen::Matrix3d& rot_w2b);

  void constructBoundingBox(const Eigen::Vector3d& pos,
                            Eigen::Vector3d& min_val, Eigen::Vector3d& max_val,
                            Eigen::Vector3d& rotations,
                            Eigen::Vector3d& mean_val,
                            Eigen::Vector3d& std_val);

  bool loadParams(std::string ns);

 private:
  MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>*
      map_manager_;

  AdaptiveObbType type_;
  double local_pointcloud_range_;
  double bounding_box_size_max_;
  double distribution_scaling_max_;
  double voxel_filter_leaf_size_;
};
