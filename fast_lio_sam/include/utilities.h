#ifndef FAST_LIO_SAM_UTILITY_H
#define FAST_LIO_SAM_UTILITY_H

///// common headers
#include <string>
///// ROS
#include <tf2/LinearMath/Transform.h>  // to Quaternion_to_euler
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
///// PCL
#include <pcl/point_types.h>                  //pt
#include <pcl/point_cloud.h>                  //cloud
#include <pcl/conversions.h>                  //ros<->pcl
#include <pcl_conversions/pcl_conversions.h>  //ros<->pcl
#include <pcl/common/transforms.h>
///// Eigen
#include <Eigen/Eigen>  // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
///// GTSAM
#include <gtsam/geometry/Pose3.h>

///// conversions
void tf2_to_eigen(const tf2::Transform &a, Eigen::Matrix4d &b);

void eigen_to_tf2(const Eigen::Matrix4d &a, tf2::Transform &b);

gtsam::Pose3 pose_eig_to_gtsam_pose(const Eigen::Matrix4d &pose_eig_in);

Eigen::Matrix4d gtsam_pose_to_pose_eig(const gtsam::Pose3 &gtsam_pose_in);

geometry_msgs::msg::PoseStamped pose_eig_to_pose_stamped(
    const Eigen::Matrix4d &pose_eig_in, std::string frame_id = "map");

geometry_msgs::msg::PoseStamped gtsam_pose_to_pose_stamped(
    const gtsam::Pose3 &gtsam_pose_in, std::string frame_id = "map");

template <typename T>
sensor_msgs::msg::PointCloud2 pcl_to_pcl_ros(pcl::PointCloud<T> cloud,
                                             std::string frame_id = "map") {
  sensor_msgs::msg::PointCloud2 cloud_ROS;
  pcl::toROSMsg(cloud, cloud_ROS);
  cloud_ROS.header.frame_id = frame_id;
  return cloud_ROS;
}

///// transformation
template <typename T>
pcl::PointCloud<T> tf_pcd(const pcl::PointCloud<T> &cloud_in,
                          const Eigen::Matrix4d &pose_tf) {
  if (cloud_in.size() == 0) return cloud_in;
  pcl::PointCloud<T> pcl_out_ = cloud_in;
  pcl::transformPointCloud(cloud_in, pcl_out_, pose_tf);
  return pcl_out_;
}

#endif