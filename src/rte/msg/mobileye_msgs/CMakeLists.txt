
cmake_minimum_required(VERSION 2.8.3)
project(mobileye_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
    FILES

    # ADAS_DRV
    ADAS_DRV.msg
    
    # C_RDR
    C_RDR_Genrl.msg
    C_RDR.msg

    # RDR
    RDR_Genrl.msg
    RDR.msg

    # CMR
    CMR_Genrl.msg
    CMR_Lane.msg
    CMR_Obj.msg

    # Common
    ObjectState.msg
    LaneInfo.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs
   geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs 
  nav_msgs
  geometry_msgs
  message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)