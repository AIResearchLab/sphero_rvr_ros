cmake_minimum_required(VERSION 3.10.2)
project(rvr_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  std_msgs
  tf
)
