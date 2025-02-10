cmake_minimum_required(VERSION 2.8.3)
project(carmaker)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  tf
  std_msgs
  autohyu_msgs
  carmaker_msgs

  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

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
#  LIBRARIES trajectory
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}

  # BSW
  ${CMAKE_SOURCE_DIR}/bsw/system/ini_parser
  ${CMAKE_SOURCE_DIR}/bsw/system/task_manager
  ${CMAKE_SOURCE_DIR}/bsw/system/interface
  ${CMAKE_SOURCE_DIR}/bsw/system/ros_bridge 
  ${CMAKE_SOURCE_DIR}/bsw/system/util_function
  
  # Library
  ${CMAKE_SOURCE_DIR}/lib/spline
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/trajectory.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(vehicle_driver_carmaker
  # BSW
  ${CMAKE_SOURCE_DIR}/bsw/system/ini_parser/ini_parser.cpp

  # Library
  ${CMAKE_SOURCE_DIR}/lib/spline/spline.cpp

  src/vehicle_driver/vehicle_driver_node.cpp
)

add_executable(object_driver_carmaker
  # BSW
  ${CMAKE_SOURCE_DIR}/bsw/system/ini_parser/ini_parser.cpp

  # Library
  ${CMAKE_SOURCE_DIR}/lib/spline/spline.cpp

  src/object_driver/object_driver_node.cpp
)

add_executable(vehicle_control_carmaker
  # BSW
  ${CMAKE_SOURCE_DIR}/bsw/system/ini_parser/ini_parser.cpp

  # Library
  ${CMAKE_SOURCE_DIR}/lib/spline/spline.cpp

  src/vehicle_control/vehicle_control_node.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above

add_dependencies(vehicle_driver_carmaker  
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  autohyu_msgs_generate_messages_cpp
  carmaker_msgs_generate_messages_cpp
)

add_dependencies(object_driver_carmaker  
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  autohyu_msgs_generate_messages_cpp
  carmaker_msgs_generate_messages_cpp
)

add_dependencies(vehicle_control_carmaker  
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  autohyu_msgs_generate_messages_cpp
  carmaker_msgs_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(vehicle_driver_carmaker
    ${catkin_LIBRARIES}
)

target_link_libraries(object_driver_carmaker
    ${catkin_LIBRARIES}
)

target_link_libraries(vehicle_control_carmaker
    ${catkin_LIBRARIES}
)