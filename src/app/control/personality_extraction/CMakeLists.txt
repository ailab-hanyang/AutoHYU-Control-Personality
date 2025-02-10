cmake_minimum_required(VERSION 2.8.3)
project(personality_extraction)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    autohyu_msgs
    visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header fisdfles
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

    # BSW
    ${CMAKE_SOURCE_DIR}/bsw/system/ini_parser
    ${CMAKE_SOURCE_DIR}/bsw/system/task_manager
    ${CMAKE_SOURCE_DIR}/bsw/system/interface
    ${CMAKE_SOURCE_DIR}/bsw/system/ros_bridge 
    ${CMAKE_SOURCE_DIR}/bsw/system/util_function

    ${PYTHON_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/lib/nanoflann
    
    # Library
    # ${CMAKE_SOURCE_DIR}/lib/spline
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_library(libfrenet2 SHARED
    src/utils/frenet.cpp
)

set_target_properties(libfrenet2 PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

target_link_libraries(libfrenet2
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)


install(TARGETS libfrenet2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/personality_weight_bayesopt_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#############
## personality time window node
add_executable(personality_time_window
    # BSW
    ${CMAKE_SOURCE_DIR}/bsw/system/ini_parser/ini_parser.cpp

    # Library
    # ${CMAKE_SOURCE_DIR}/lib/spline/spline.cpp

    # Node and Algorithm
    src/personality_time_window_node.cpp    
    src/personality_extraction_algorithm/data_aquisition.cpp    
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(personality_time_window
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
    autohyu_msgs_generate_messages_cpp
)

target_link_libraries(personality_time_window
    ${catkin_LIBRARIES}
)



############################################# Matplotlib cpp #############################################
# Find Python
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
include_directories(${Python3_INCLUDE_DIRS})
# Find matplotlib
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import matplotlib; print(matplotlib.__file__)"
    OUTPUT_VARIABLE MATPLOTLIB_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
get_filename_component(MATPLOTLIB_PATH ${MATPLOTLIB_PATH} DIRECTORY)

# Try to find backend_agg.h
find_path(MATPLOTLIB_INCLUDE_DIR
    NAMES backend_agg.h agg_backend.h
    PATHS
        ${MATPLOTLIB_PATH}
        ${MATPLOTLIB_PATH}/backends
        ${MATPLOTLIB_PATH}/backends/qt_compat
        ${Python3_SITELIB}/matplotlib/backends
        ${Python3_SITELIB}/matplotlib/backends/qt_compat
    PATH_SUFFIXES matplotlib/backends
)

if(NOT MATPLOTLIB_INCLUDE_DIR)
    message(WARNING "matplotlib backend header not found. Plotting functionality might be limited.")
else()
    message(STATUS "Found matplotlib backend header at: ${MATPLOTLIB_INCLUDE_DIR}")
    include_directories(${MATPLOTLIB_INCLUDE_DIR})
    include_directories(${MATPLOTLIB_PATH})
    include_directories(${MATPLOTLIB_PATH}/core)
endif()

include_directories(${MATPLOTLIB_PATH})
include_directories(${MATPLOTLIB_PATH}/core)


# Add matplotlib-cpp
target_include_directories(personality_time_window      PRIVATE     ${CMAKE_SOURCE_DIR}/lib/matplotlib-cpp)
target_link_libraries(personality_time_window
    ${Python3_LIBRARIES}
    ${Python3_NumPy_LIBRARIES}
)
target_compile_definitions(personality_time_window      PRIVATE
    WITHOUT_NUMPY
    MPL_strcasecmp=strcasecmp
)
##########################################################################################################
