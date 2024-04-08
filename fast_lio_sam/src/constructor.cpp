#include "main.h"
#include "utilities.h"

pose_pcd::pose_pcd(const nav_msgs::msg::Odometry &odom_in,
                   const sensor_msgs::msg::PointCloud2 &pcd_in,
                   const int &idx_in) {
  tf2::Transform tf(
      tf2::Quaternion(
          odom_in.pose.pose.orientation.x, odom_in.pose.pose.orientation.y,
          odom_in.pose.pose.orientation.z, odom_in.pose.pose.orientation.z),
      tf2::Vector3(odom_in.pose.pose.position.x, odom_in.pose.pose.position.y,
                   odom_in.pose.pose.position.z));
  tf2_to_eigen(tf, pose_corrected_eig);
  pose_eig = pose_corrected_eig;
  pcl::PointCloud<pcl::PointXYZI> tmp_pcd_;
  pcl::fromROSMsg(pcd_in, tmp_pcd_);
  pcd =
      tf_pcd(tmp_pcd_,
             pose_corrected_eig.inverse());  // FAST-LIO publish data in world
                                             // frame, so save it in LiDAR frame
  timestamp = odom_in.header.stamp.sec + odom_in.header.stamp.nanosec * 1e-9;
  idx = idx_in;
}

FAST_LIO_SAM_CLASS::FAST_LIO_SAM_CLASS(rclcpp::NodeOptions options)
    : Node("fast_lio_sam", options) {
  ////// ROS params
  // temp vars
  // double loop_update_hz_, vis_hz_;
  // get params
  m_map_frame = this->declare_parameter<std::string>("map_frame", "map");
  m_keyframe_thr = this->declare_parameter<double>("keyframe_threshold", 1.0);
  m_loop_det_radi =
    this->declare_parameter<double>("loop_detection_radius", 15.0);
  m_loop_det_tdiff_thr = this->declare_parameter<double>(
    "loop_detection_timediff_threshold", 10.0);
  m_icp_score_thr =
    this->declare_parameter<double>("icp_score_threshold", 10.0);
  m_sub_key_num = this->declare_parameter<int>("subkeyframes_number", 5);
  m_loop_update_hz_ = this->declare_parameter<double>("loop_update_hz", 1.0);
  m_vis_hz_ = this->declare_parameter<double>("vis_hz", 0.5);
  /* results */
  m_save_map_bag = this->declare_parameter<bool>("result/save_map_bag", false);
  m_save_map_pcd = this->declare_parameter<bool>("result/save_map_pcd", true);

  m_map_frame = this->get_parameter("map_frame").as_string();
  m_keyframe_thr = this->get_parameter("keyframe_threshold").as_double();
  m_loop_det_radi = this->get_parameter("loop_detection_radius").as_double();
  m_loop_det_tdiff_thr = this->get_parameter("loop_detection_timediff_threshold").as_double();
  m_icp_score_thr = this->get_parameter("icp_score_threshold").as_double();
  m_sub_key_num = this->get_parameter("subkeyframes_number").as_int();
  m_loop_update_hz_ = this->get_parameter("loop_update_hz").as_double();
  m_vis_hz_ = this->get_parameter("vis_hz").as_double();

  m_save_map_bag = this->get_parameter("result/save_map_bag").as_bool();
  m_save_map_pcd = this->get_parameter("result/save_map_pcd").as_bool();


  ////// GTSAM init
  gtsam::ISAM2Params isam_params_;
  isam_params_.relinearizeThreshold = 0.01;
  isam_params_.relinearizeSkip = 1;
  m_isam_handler = std::make_shared<gtsam::ISAM2>(isam_params_);
  ////// loop init
  m_voxelgrid.setLeafSize(0.3, 0.3, 0.3);
  m_voxelgrid_vis.setLeafSize(0.2, 0.2, 0.2);
  m_icp.setMaxCorrespondenceDistance(m_loop_det_radi * 2.0);
  m_icp.setTransformationEpsilon(1e-2);
  m_icp.setEuclideanFitnessEpsilon(1e-2);
  m_icp.setMaximumIterations(100);
  m_icp.setRANSACIterations(0);

  ////// ROS things
  m_odom_path.header.frame_id = m_map_frame;
  m_corrected_path.header.frame_id = m_map_frame;
  m_package_path = ".";
  // m_package_path = rclcpp::package::getPath("fast_lio_sam");
  m_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  // publishers
  m_odom_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("ori_odom", 10);
  m_path_pub = this->create_publisher<nav_msgs::msg::Path>("ori_path", 10);
  m_corrected_odom_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "corrected_odom", 10);
  m_corrected_path_pub =
      this->create_publisher<nav_msgs::msg::Path>("corrected_path", 10);
  m_corrected_pcd_map_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("corrected_map",
                                                            10);
  m_corrected_current_pcd_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "corrected_current_pcd", 10);
  m_loop_detection_pub =
      this->create_publisher<visualization_msgs::msg::Marker>("loop_detection",
                                                              10);
  m_realtime_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "pose_stamped", 10);
  m_debug_src_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("src", 10);
  m_debug_dst_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("dst", 10);
  m_debug_aligned_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("aligned", 10);
  // subscribers
  m_sub_odom_.subscribe(this, "odom");
  // =
  //     std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this,
  //     "/odom", 10);
  //         // , "/Odometry", 10);
  m_sub_pcd_.subscribe(this, "cloud_registered");
  m_sub_odom_pcd_sync_ =
      std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(
          odom_pcd_sync_pol(10), m_sub_odom_, m_sub_pcd_);
  m_sub_odom_pcd_sync_->registerCallback(
      std::bind(&FAST_LIO_SAM_CLASS::odom_pcd_cb, this, std::placeholders::_1,
                std::placeholders::_2));
  // Timers at the end
  m_loop_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1 / m_loop_update_hz_),
      std::bind(&FAST_LIO_SAM_CLASS::on_timer_callback, this));
  m_vis_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1 / m_vis_hz_),
      std::bind(&FAST_LIO_SAM_CLASS::on_vis_timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Main class, starting node...");
}

FAST_LIO_SAM_CLASS::~FAST_LIO_SAM_CLASS() {
  // save map
  if (m_save_map_bag) {
    auto bag = std::make_unique<rosbag2_cpp::Writer>();
    bag->open(m_package_path + "/result.bag");
    {
      lock_guard<mutex> lock(m_keyframes_mutex);
      for (int i = 0; i < m_keyframes.size(); ++i) {
        rclcpp::Time time(m_keyframes[i].timestamp);
        // time = m_keyframes[i].timestamp;
        bag->write(pcl_to_pcl_ros(m_keyframes[i].pcd, m_map_frame),
                   "/keyframe_pcd", time);
        bag->write(pose_eig_to_pose_stamped(m_keyframes[i].pose_corrected_eig),
                   "/keyframe_pose", time);
      }
    }
    // bag->close();
    cout << "\033[36;1mResult saved in .bag format!!!\033[0m" << endl;
  }
  if (m_save_map_pcd) {
    pcl::PointCloud<pcl::PointXYZI> corrected_map_;
    {
      lock_guard<mutex> lock(m_keyframes_mutex);
      for (int i = 0; i < m_keyframes.size(); ++i) {
        corrected_map_ +=
            tf_pcd(m_keyframes[i].pcd, m_keyframes[i].pose_corrected_eig);
      }
    }
    pcl::io::savePCDFileASCII<pcl::PointXYZI>(m_package_path + "/result.pcd",
                                              corrected_map_);
    cout << "\033[32;1mResult saved in .pcd format!!!\033[0m" << endl;
  }
}