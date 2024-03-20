cmake_minimum_required(VERSION 3.4)
project(fast_lio_sam)

### set compiler
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -pthread -fexceptions -g -ggdb") #c++17, for parallel programming 
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

### get packages
## ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)

## C++
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
if(PCL_FOUND)
  message(WARNING "PCL_VER: ${PCL_VERSION}, and PCL_DIR: ${PCL_INCLUDE_DIRS}")
endif()
if(Eigen3_FOUND)
  message(WARNING "EIGEN_VER: ${EIGEN3_VERSION_STRING}, and Eigen DIR: ${EIGEN3_INCLUDE_DIR}")
endif()
if(GTSAM_FOUND)
  message(WARNING "GTSAM_VER: ${GTSAM_VERSION}, and GTSAM DIR: ${GTSAM_INCLUDE_DIR}")
endif()

### get packages' headers
include_directories(
  include
	${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)

###########
## Build ##
###########
### main
add_library(${PROJECT_NAME}_constructor src/constructor.cpp)
target_link_libraries(${PROJECT_NAME}_constructor ${rclcpp_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBS} gtsam)
add_library(${PROJECT_NAME}_methods src/methods.cpp)
target_link_libraries(${PROJECT_NAME}_methods ${rclcpp_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBS} gtsam)
add_library(${PROJECT_NAME}_callbacks src/callbacks.cpp)
target_link_libraries(${PROJECT_NAME}_callbacks ${rclcpp_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBS} gtsam)
add_library(${PROJECT_NAME}_utilities src/utilities.cpp)
target_link_libraries(${PROJECT_NAME}_utilities ${rclcpp_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBS} gtsam)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node 
  ${rclcpp_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${EIGEN3_LIBS} 
  gtsam 
  ${PROJECT_NAME}_constructor 
  ${PROJECT_NAME}_callbacks 
  ${PROJECT_NAME}_methods 
  ${PROJECT_NAME}_utilities
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  builtin_interfaces
  rosbag2_cpp
  rosbag2_storage
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
  message_filters
)

install(TARGETS
  ${PROJECT_NAME}_constructor
  ${PROJECT_NAME}_methods
  ${PROJECT_NAME}_callbacks
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()