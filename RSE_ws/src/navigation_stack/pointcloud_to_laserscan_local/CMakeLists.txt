cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_to_laserscan_local)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions pcl_ros
)

## Build the executable
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_to_laserscan src/pointcloud_to_laserscan.cpp)
target_link_libraries(pointcloud_to_laserscan ${catkin_LIBRARIES})

