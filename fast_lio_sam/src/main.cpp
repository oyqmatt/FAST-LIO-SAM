#include "main.h"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // ros::NodeHandle nh_private("~");

  auto node = std::make_shared<FAST_LIO_SAM_CLASS>(rclcpp::NodeOptions());

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  
  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  spin_thread.join();
  
  node.reset();

  return 0;
}