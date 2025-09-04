cmake_minimum_required(VERSION 2.8.3)
project(obstacle_tracking)

find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  geometry_msgs
  roscpp
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
