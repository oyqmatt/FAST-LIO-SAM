#ifndef FAST_LIO_SAM_MAIN_H
#define FAST_LIO_SAM_MAIN_H

///// coded headers
// #include "utilities.h"
///// common headers
#include <time.h>
#include <math.h>
#include <cmath>
#include <chrono>  //time check
#include <vector>
#include <mutex>
#include <string>
#include <utility>  // pair, make_pair
///// ROS
#include <rclcpp/rclcpp.hpp>
// #include <ros/package.h> // get package_path
#include <rosbag2_cpp/writer.hpp>
#include <tf2/LinearMath/Quaternion.h>      // to Quaternion_to_euler
#include <tf2/LinearMath/Matrix3x3.h>       // to Quaternion_to_euler
#include <tf2/transform_datatypes.h>        // createQuaternionFromRPY
#include <tf2_eigen/tf2_eigen.h>            // tf <-> eigen
#include <tf2_ros/transform_broadcaster.h>  // broadcaster
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
///// PCL
#include <pcl/point_types.h>                  //pt
#include <pcl/point_cloud.h>                  //cloud
#include <pcl/common/transforms.h>            //transformPointCloud
#include <pcl/conversions.h>                  //ros<->pcl
#include <pcl_conversions/pcl_conversions.h>  //ros<->pcl
#include <pcl/filters/voxel_grid.h>           //voxelgrid
#include <pcl/registration/icp.h>             //icp
#include <pcl/io/pcd_io.h>                    // save map
///// Eigen
#include <Eigen/Eigen>  // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
///// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

using namespace std;
using namespace std::chrono;
typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>
    odom_pcd_sync_pol;

////////////////////////////////////////////////////////////////////////////////////////////////////
struct pose_pcd {
  pcl::PointCloud<pcl::PointXYZI> pcd;
  Eigen::Matrix4d pose_eig = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d pose_corrected_eig = Eigen::Matrix4d::Identity();
  double timestamp;
  int idx;
  bool processed = false;
  pose_pcd(){};
  pose_pcd(const nav_msgs::msg::Odometry &odom_in,
           const sensor_msgs::msg::PointCloud2 &pcd_in, const int &idx_in);
};
////////////////////////////////////////////////////////////////////////////////////////////////////
class FAST_LIO_SAM_CLASS : public rclcpp::Node {
 private:
  ///// basic params
  string m_map_frame;
  string m_package_path;
  ///// shared data - odom and pcd
  mutex m_realtime_pose_mutex, m_keyframes_mutex;
  mutex m_graph_mutex, m_vis_mutex;
  bool m_init = false;
  int m_current_keyframe_idx = 0;
  pose_pcd m_current_frame;
  vector<pose_pcd> m_keyframes;
  pose_pcd m_not_processed_keyframe;
  Eigen::Matrix4d m_last_corrected_pose = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d m_odom_delta = Eigen::Matrix4d::Identity();
  ///// graph and values
  shared_ptr<gtsam::ISAM2> m_isam_handler = nullptr;
  gtsam::NonlinearFactorGraph m_gtsam_graph;
  gtsam::Values m_init_esti;
  gtsam::Values m_corrected_esti;
  double m_keyframe_thr;
  ///// loop
  pcl::VoxelGrid<pcl::PointXYZI> m_voxelgrid, m_voxelgrid_vis;
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> m_icp;
  double m_icp_score_thr;
  double m_loop_det_radi;
  double m_loop_det_tdiff_thr;
  int m_sub_key_num;
  double m_loop_update_hz_, m_vis_hz_;
  vector<pair<int, int>> m_loop_idx_pairs;  // for vis
  bool m_loop_added_flag = false;           // for opt
  bool m_loop_added_flag_vis = false;       // for vis
  ///// visualize
  std::unique_ptr<tf2_ros::TransformBroadcaster> m_broadcaster_;
  pcl::PointCloud<pcl::PointXYZ> m_odoms, m_corrected_odoms;
  nav_msgs::msg::Path m_odom_path, m_corrected_path;
  bool m_global_map_vis_switch = true;
  ///// results
  bool m_save_map_bag = false, m_save_map_pcd = false;
  ///// ros
  // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub,
      m_corrected_path_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_odom_pub,
      m_corrected_current_pcd_pub, m_corrected_pcd_map_pub,
      m_corrected_odom_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      m_loop_detection_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      m_realtime_pose_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_debug_src_pub,
      m_debug_dst_pub, m_debug_aligned_pub;
  rclcpp::TimerBase::SharedPtr m_loop_timer_, m_vis_timer_;
  // odom, pcd sync subscriber
  shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>>
      m_sub_odom_pcd_sync_ = nullptr;
  message_filters::Subscriber<nav_msgs::msg::Odometry> m_sub_odom_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_sub_pcd_;

  ///// functions
 public:
  FAST_LIO_SAM_CLASS(rclcpp::NodeOptions options);
  ~FAST_LIO_SAM_CLASS();

 private:
  rclcpp::TimerBase::SharedPtr m_timer, m_vis_timer;
  // methods
  void update_vis_vars(const pose_pcd &pose_pcd_in);
  void voxelize_pcd(pcl::VoxelGrid<pcl::PointXYZI> &voxelgrid,
                    pcl::PointCloud<pcl::PointXYZI> &pcd_in);
  bool check_if_keyframe(const pose_pcd &pose_pcd_in,
                         const pose_pcd &latest_pose_pcd);
  int get_closest_keyframe_idx(const pose_pcd &front_keyframe,
                               const vector<pose_pcd> &keyframes);
  void icp_key_to_subkeys(const pose_pcd &front_keyframe,
                          const int &closest_idx,
                          const vector<pose_pcd> &keyframes);
  visualization_msgs::msg::Marker get_loop_markers(
      const gtsam::Values &corrected_esti_in);
  // cb
  void odom_pcd_cb(
      const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg);
  void on_timer_callback();
  void on_vis_timer_callback();
};

#endif