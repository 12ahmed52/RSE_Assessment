cmake_minimum_required(VERSION 3.0.2)
project(gui_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(tcp_server_node src/gui_ros.cpp src/TCPServer.cpp) # Add TCPServer.cpp here
target_link_libraries(tcp_server_node ${catkin_LIBRARIES})

