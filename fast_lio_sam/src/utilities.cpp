#include "utilities.h"

///// ROS
#include "tf2/LinearMath/Quaternion.h"  // to Quaternion_to_euler
#include <tf2/LinearMath/Matrix3x3.h>   // to Quaternion_to_euler

///// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>

using namespace std;
//////////////////////////////////////////////////////////////////////
///// conversions
void tf2_to_eigen(const tf2::Transform &a, Eigen::Matrix4d &b) {
  tf2::Matrix3x3 rot(a.getRotation());
  for (size_t i = 0, rows = 3, cols = 3; i < cols; ++i) {
    for (size_t j = 0; j < rows; ++j) {
      b(j, i) = rot[i][j];
    }
  }
  b(0, 3) = a.getOrigin().getX();
  b(1, 3) = a.getOrigin().getY();
  b(2, 3) = a.getOrigin().getZ();
  b(3, 3) = 1;
}

void eigen_to_tf2(const Eigen::Matrix4d &a, tf2::Transform &b) {
  tf2::Matrix3x3 rot;
  for (size_t i = 0, rows = 3, cols = 3; i < cols; ++i) {
    for (size_t j = 0; j < rows; ++j) {
      rot[i][j] = a(j, i);
    }
  }
  b.setBasis(rot);
  b.setOrigin(tf2::Vector3(a(0, 3), a(1, 3), a(2, 3)));
}

gtsam::Pose3 pose_eig_to_gtsam_pose(const Eigen::Matrix4d &pose_eig_in) {
  double r_, p_, y_;
  tf2::Transform transform_;
  eigen_to_tf2(pose_eig_in, transform_);
  return gtsam::Pose3(
      gtsam::Rot3::Quaternion(
          transform_.getRotation().getW(), transform_.getRotation().getX(),
          transform_.getRotation().getY(), transform_.getRotation().getZ()),
      gtsam::Point3(pose_eig_in(0, 3), pose_eig_in(1, 3), pose_eig_in(2, 3)));
}
Eigen::Matrix4d gtsam_pose_to_pose_eig(const gtsam::Pose3 &gtsam_pose_in) {
  Eigen::Matrix4d pose_eig_out_ = Eigen::Matrix4d::Identity();
  tf2::Quaternion quat_;
  quat_.setRPY(gtsam_pose_in.rotation().roll(),
               gtsam_pose_in.rotation().pitch(),
               gtsam_pose_in.rotation().yaw());
  tf2::Matrix3x3 mat_(quat_);

  pose_eig_out_(0, 0) = mat_[0][0];
  pose_eig_out_(0, 1) = mat_[0][1];
  pose_eig_out_(0, 2) = mat_[0][2];
  pose_eig_out_(1, 0) = mat_[1][0];
  pose_eig_out_(1, 1) = mat_[1][1];
  pose_eig_out_(1, 2) = mat_[1][2];
  pose_eig_out_(2, 0) = mat_[2][0];
  pose_eig_out_(2, 1) = mat_[2][1];
  pose_eig_out_(2, 2) = mat_[2][2];

  pose_eig_out_(0, 3) = gtsam_pose_in.translation().x();
  pose_eig_out_(1, 3) = gtsam_pose_in.translation().y();
  pose_eig_out_(2, 3) = gtsam_pose_in.translation().z();
  return pose_eig_out_;
}
geometry_msgs::msg::PoseStamped pose_eig_to_pose_stamped(
    const Eigen::Matrix4d &pose_eig_in, string frame_id) {
  double r_, p_, y_;
  tf2::Transform transform_;
  eigen_to_tf2(pose_eig_in, transform_);
  tf2::Quaternion quat_ = transform_.getRotation();
  geometry_msgs::msg::PoseStamped pose_;
  pose_.header.frame_id = frame_id;
  pose_.pose.position.x = pose_eig_in(0, 3);
  pose_.pose.position.y = pose_eig_in(1, 3);
  pose_.pose.position.z = pose_eig_in(2, 3);
  pose_.pose.orientation.w = quat_.getW();
  pose_.pose.orientation.x = quat_.getX();
  pose_.pose.orientation.y = quat_.getY();
  pose_.pose.orientation.z = quat_.getZ();
  return pose_;
}
geometry_msgs::msg::PoseStamped gtsam_pose_to_pose_stamped(
    const gtsam::Pose3 &gtsam_pose_in, string frame_id) {
  tf2::Quaternion quat_;
  quat_.setRPY(gtsam_pose_in.rotation().roll(),
               gtsam_pose_in.rotation().pitch(),
               gtsam_pose_in.rotation().yaw());
  geometry_msgs::msg::PoseStamped pose_;
  pose_.header.frame_id = frame_id;
  pose_.pose.position.x = gtsam_pose_in.translation().x();
  pose_.pose.position.y = gtsam_pose_in.translation().y();
  pose_.pose.position.z = gtsam_pose_in.translation().z();

  pose_.pose.orientation.w = quat_.getW();
  pose_.pose.orientation.x = quat_.getX();
  pose_.pose.orientation.y = quat_.getY();
  pose_.pose.orientation.z = quat_.getZ();

  return pose_;
}