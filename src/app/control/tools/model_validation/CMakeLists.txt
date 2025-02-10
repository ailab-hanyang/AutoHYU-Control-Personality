cmake_minimum_required(VERSION 2.8.3)
project(model_validation)

## Compile as C++14, supported in ROS Kinetic and newer

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    autohyu_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
