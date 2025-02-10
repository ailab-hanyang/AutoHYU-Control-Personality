cmake_minimum_required(VERSION 3.0.2)
project(acados_scc_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rospy
    tf
    std_msgs
    autohyu_msgs
    visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)

catkin_python_setup()

catkin_package()
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/lib/nanoflann

    )


  
add_library(libfrenet_scc SHARED
    src/utils/frenet.cpp
)

set_target_properties(libfrenet_scc PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

target_link_libraries(libfrenet_scc
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)


install(TARGETS libfrenet_scc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/acados_scc_control_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
