cmake_minimum_required(VERSION 2.8.3)
project(whac_lidar)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_runtime
  std_msgs
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf
  )

catkin_python_setup()


catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf
  )