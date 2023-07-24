cmake_minimum_required(VERSION 3.0.2)
project(omni_control)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move2pose src/move_to_pose.cpp)
target_link_libraries(move2pose ${catkin_LIBRARIES})

add_executable(pure_pursuit src/pure_pursuit.cpp)
target_link_libraries(pure_pursuit ${catkin_LIBRARIES})