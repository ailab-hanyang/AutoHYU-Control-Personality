cmake_minimum_required(VERSION 2.8.3)
project(can_parser)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    std_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    autohyu_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

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
    INCLUDE_DIRS include
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
    ${EIGEN3_INCLUDE_DIRS}

    # BSW
    ${CMAKE_SOURCE_DIR}/bsw/system/ini_parser
    ${CMAKE_SOURCE_DIR}/bsw/system/task_manager
    ${CMAKE_SOURCE_DIR}/bsw/system/interface
    ${CMAKE_SOURCE_DIR}/bsw/system/ros_bridge 
    ${CMAKE_SOURCE_DIR}/bsw/system/util_function
    
    # Library
    ${CMAKE_SOURCE_DIR}/lib/spline
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} 
    # BSW
    ${CMAKE_SOURCE_DIR}/bsw/system/ini_parser/ini_parser.cpp

    # Library
    ${CMAKE_SOURCE_DIR}/lib/spline/spline.cpp

    # Node and Algorithm
    src/can_parser_node.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
    autohyu_msgs_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
