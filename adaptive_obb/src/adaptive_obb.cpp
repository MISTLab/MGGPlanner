#include "adaptive_obb/adaptive_obb.h"

#include <limits>

#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/convex_hull.h>
#include <ros/ros.h>

void AdaptiveObb::computeBounds(
    Eigen::Vector3d& min_val, Eigen::Vector3d& max_val,
    const Eigen::Vector3d& offset, const Eigen::Matrix3d& rot_w2b,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud) {
  double inf = std::numeric_limits<double>::max();

  min_val = Eigen::Vector3d::Ones() * inf;
  max_val = -Eigen::Vector3d::Ones() * inf;

  for (auto& p : pointcloud->points) {
    Eigen::Vector3d point_w(p.x, p.y, p.z);
    Eigen::Vector3d point_b = rot_w2b * (point_w - offset);

    for (int i = 0; i < 3; i++) {
      if (point_b[i] < min_val[i]) min_val[i] = point_b[i];
      if (point_b[i] > max_val[i]) max_val[i] = point_b[i];
    }
  }
}

double AdaptiveObb::computeVolume(
    const Eigen::Matrix3d& rot_w2b,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud) {
  Eigen::Vector3d min_val;
  Eigen::Vector3d max_val;

  computeBounds(min_val, max_val, Eigen::Vector3d::Zero().eval(), rot_w2b,
                pointcloud);

  return (max_val[0] - min_val[0]) * (max_val[1] - min_val[1]) *
         (max_val[2] - min_val[2]);
}

bool AdaptiveObb::rotationsSafetyCheck(Eigen::Matrix3d& rot_w2b,
                                       Eigen::Vector3d& eig_val) {
  if (abs(rot_w2b(2, 0)) > 0.8) {
    // To avoid the problem of gimbal lock, the rot_w2b is modified s.t
    // new x axis <- old y axis
    // new y axis <- old z axis
    // new z axis <- old x axis
    // Note: row vectors
    Eigen::Matrix3d rot_swap;
    rot_swap << 0, 1, 0, 0, 0, 1, 1, 0, 0;

    rot_w2b = rot_swap * rot_w2b;
    eig_val = rot_swap * eig_val;
    return true;
  }
  return false;
}

void AdaptiveObb::computeVarianceInReferenceFrame(
    Eigen::Vector3d& variance,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,
    const Eigen::Matrix3d& rot_w2b) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.topLeftCorner(3, 3) = rot_w2b.cast<float>();

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());

  pcl::transformPointCloud(*pointcloud, *transformed_cloud, transform);

  Eigen::Vector4f centroid_out;
  pcl::compute3DCentroid(*transformed_cloud, centroid_out);

  Eigen::Matrix3f covariance_matrix;
  computeCovarianceMatrix(*transformed_cloud, centroid_out, covariance_matrix);

  variance << covariance_matrix(0, 0), covariance_matrix(1, 1),
      covariance_matrix(2, 2);

  variance = variance / (pointcloud->size() - 1.0);
}

void AdaptiveObb::constructBoundingBox(const Eigen::Vector3d& pos,
                                       Eigen::Vector3d& min_val,
                                       Eigen::Vector3d& max_val,
                                       Eigen::Vector3d& rotations,
                                       Eigen::Vector3d& mean_val,
                                       Eigen::Vector3d& std_val) {
  // TODO:
  // For now surface points in line of sight of the robot are used to construct
  // the bounding box. This may be an issue in
  // 1. Open space -> possible solution: sample endpoint and unknown voxels
  // 2. Narrow (highly non-convex) environments -> possible solution: random
  // reflection model
  //      - Issue is now less important after changing bound computations

  ros::Time getlocalpcl_time;
  // Get local pointcloud
  pcl::PointCloud<pcl::PointXYZI>* local_cloud =
      new pcl::PointCloud<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_cloud_ptr(local_cloud);

  START_TIMER(getlocalpcl_time);
  map_manager_->getLocalPointcloud(pos, local_pointcloud_range_, 0,
                                   *local_cloud, false);
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "getLocalPointcloud time: %f", GET_ELAPSED_TIME(getlocalpcl_time));
  // Filter cloud to reduce effect of error sources from uneven density
  pcl::PointCloud<pcl::PointXYZI>* local_filtered =
      new pcl::PointCloud<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_filtered_ptr(local_filtered);

  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  voxel_filter.setInputCloud(local_cloud_ptr);
  voxel_filter.setLeafSize(voxel_filter_leaf_size_, voxel_filter_leaf_size_,
                           voxel_filter_leaf_size_);
  voxel_filter.filter(*local_filtered);

  // Compute rotation matrix depending on selected method
  Eigen::Matrix3d rot_w2b;
  Eigen::Vector3d variance;

  if (type_ == AdaptiveObbType::kPca) {
    // Matrix constructed from the vectors of the principal component analysis
    // Works bad in self-similar environments
    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(local_filtered_ptr);
    rot_w2b = pca.getEigenVectors().cast<double>();
    rot_w2b = rot_w2b / rot_w2b.determinant();  // Ensure right-handedness
    rot_w2b.transposeInPlace();  // For the matrix to be body to world, the
                                 // vector is tranposed

    double cloud_size = local_filtered->size();
    variance = pca.getEigenValues().cast<double>() / (cloud_size - 1);
    variance.cwiseAbs();
  }

  // Change to a representation convertible to euler angles
  rotationsSafetyCheck(rot_w2b, variance);

  // Compute offset to center of bounding box
  // The center coincides with center of poincloud
  Eigen::Vector4f centroid_out;
  pcl::compute3DCentroid(*local_filtered, centroid_out);

  Eigen::Vector3d pos_c = centroid_out.head(3).cast<double>();
  Eigen::Vector3d offset = rot_w2b * (pos_c - pos);
  // Eigen::Vector3d offset = Eigen::Vector3d::Zero();

  // Compute bounds by using standard deviation in each direction of the
  // reference frame
  Eigen::Vector3d sigma = variance.cwiseSqrt();
  double sigma_max = sigma.maxCoeff();

  min_val = offset - sigma / sigma_max * bounding_box_size_max_ / 2;
  max_val = offset + sigma / sigma_max * bounding_box_size_max_ / 2;

  rotations = rot_w2b.eulerAngles(2, 1, 0);

  // Scaling parameter for sample distributions
  std_val = sigma / sigma_max * distribution_scaling_max_;
  mean_val = Eigen::Vector3d::Zero();
}

bool AdaptiveObb::loadParams(std::string ns) {
  ROSPARAM_INFO("Loading: " + ns);
  std::string param_name;

  std::string parse_str;

  param_name = ns + "/type";
  ros::param::get(param_name, parse_str);
  if (!parse_str.compare("kPca"))
    type_ = AdaptiveObbType::kPca;
  else if (!parse_str.compare("kMvbb"))
    type_ = AdaptiveObbType::kMvbb;
  else if (!parse_str.compare("kAabb"))
    type_ = AdaptiveObbType::kAabb;
  else {
    ROSPARAM_ERROR(param_name);
    return false;
  }

  param_name = ns + "/local_pointcloud_range";
  if (!ros::param::get(param_name, local_pointcloud_range_)) {
    local_pointcloud_range_ = 15.0;
    ROSPARAM_WARN(param_name, local_pointcloud_range_);
  }

  param_name = ns + "/bounding_box_size_max";
  if (!ros::param::get(param_name, bounding_box_size_max_)) {
    bounding_box_size_max_ = 30.0;
    ROSPARAM_WARN(param_name, bounding_box_size_max_);
  }

  param_name = ns + "/distribution_scaling_max";
  if (!ros::param::get(param_name, distribution_scaling_max_)) {
    distribution_scaling_max_ = 10.0;
    ROSPARAM_WARN(param_name, distribution_scaling_max_);
  }

  param_name = ns + "/voxel_filter_leaf_size";
  if (!ros::param::get(param_name, voxel_filter_leaf_size_)) {
    voxel_filter_leaf_size_ = 1.0;
    ROSPARAM_WARN(param_name, voxel_filter_leaf_size_);
  }

  return true;
}
