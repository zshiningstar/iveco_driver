cmake_minimum_required(VERSION 3.0.2)
project(iveco_drive)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  driverless_common
  can2serial
  serial
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iveco_drive
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(base_control_node  src/base_control.cpp)
add_dependencies(base_control_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_control_node ${catkin_LIBRARIES})
