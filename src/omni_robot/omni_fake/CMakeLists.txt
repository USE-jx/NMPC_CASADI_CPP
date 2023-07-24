cmake_minimum_required(VERSION 3.0.2)
project(omni_fake)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

# catkin_python_setup()

catkin_package()
include_directories( ${catkin_INCLUDE_DIRS} )

add_executable(omni_fake_node src/omni_fake_node.cpp)
target_link_libraries(omni_fake_node ${catkin_LIBRARIES})

add_executable(odom_path_node src/odom_path_node.cpp)
target_link_libraries(odom_path_node ${catkin_LIBRARIES})