# @junhee.lee

cmake_minimum_required(VERSION 2.8.3)
project(lateral_control)

add_compile_options(-std=c++17)

## acados c code genration
set(ACADOS_INSTALL_DIR ${CMAKE_SOURCE_DIR}/lib/acados)
# Add acados include directories
include_directories(${ACADOS_INSTALL_DIR}/include)
include_directories(${ACADOS_INSTALL_DIR}/include/acados)
include_directories(${ACADOS_INSTALL_DIR}/include/blasfeo/include)
include_directories(${ACADOS_INSTALL_DIR}/include/hpipm/include)

# Add the path to generated acados code
set(LATERAL_KINEMATIC_BICYCLE_NMPC scripts/lateral_kinematic_bicycle_nmpc)

# Run acados python script to generate c code ocp
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(Python3_EXECUTABLE /usr/bin/python3) 
add_custom_target(run_lateral_kinematic_bicycle_nmpc ALL
    COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/${LATERAL_KINEMATIC_BICYCLE_NMPC}/lateral_kinematic_bicycle_nmpc.py
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/${LATERAL_KINEMATIC_BICYCLE_NMPC}
    COMMENT "Running Python script to generate c code ocp..."
)

set(LATERAL_KINEMATIC_BICYCLE_NMPC_GENERATED_DIR ${PROJECT_SOURCE_DIR}/${LATERAL_KINEMATIC_BICYCLE_NMPC}/c_generated_code)
include_directories(${LATERAL_KINEMATIC_BICYCLE_NMPC_GENERATED_DIR})
include_directories(${LATERAL_KINEMATIC_BICYCLE_NMPC_GENERATED_DIR}/lateral_kinematic_bicycle_nmpc_cost)
include_directories(${LATERAL_KINEMATIC_BICYCLE_NMPC_GENERATED_DIR}/lateral_kinematic_bicycle_nmpc_model)
# Goto target_link_libraries setting

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    autohyu_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}

    ${CMAKE_SOURCE_DIR}/bsw/system/ini_parser
    ${CMAKE_SOURCE_DIR}/bsw/system/task_manager
    ${CMAKE_SOURCE_DIR}/bsw/system/interface
    ${CMAKE_SOURCE_DIR}/bsw/system/ros_bridge 
    ${CMAKE_SOURCE_DIR}/bsw/system/util_function
    
    ## spline
    ${CMAKE_SOURCE_DIR}/lib/spline

    ## hpipm
    ${CMAKE_SOURCE_DIR}/lib/hpipm/blasfeo/lib/include
    ${CMAKE_SOURCE_DIR}/lib/hpipm/hpipm/lib/include
    ${CMAKE_SOURCE_DIR}/lib/hpipm/include

    ## core/fallback controllers
    ${PROJECT_SOURCE_DIR}/include/lateral_control
)

# Collect all .cpp files in the src directory and its subdirectories
file(GLOB_RECURSE HPIPM_WRAPPER_SOURCES "${CMAKE_SOURCE_DIR}/lib/hpipm/src/*.cpp")
file(GLOB_RECURSE LATERAL_CONTROLLER "${PROJECT_SOURCE_DIR}/src/*.cpp")

add_executable(${PROJECT_NAME} 
    # ini
    ${CMAKE_SOURCE_DIR}/bsw/system/ini_parser/ini_parser.cpp

    # spline
    ${CMAKE_SOURCE_DIR}/lib/spline/spline.cpp

    # hpipm wrapper
    ${HPIPM_WRAPPER_SOURCES}

    # Node and Algorithms (if it dose not recognize the new contents, re-save the CMakeList.)
    ${LATERAL_CONTROLLER}
)

add_dependencies(${PROJECT_NAME} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
    autohyu_msgs_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME} 
    ${LATERAL_KINEMATIC_BICYCLE_NMPC_GENERATED_DIR}/libacados_ocp_solver_lateral_kinematic_bicycle_nmpc.so  # BE AWARE of your .so file name! 
    ${ACADOS_INSTALL_DIR}/lib/libacados.so
    ${ACADOS_INSTALL_DIR}/lib/libblasfeo.so
    ${ACADOS_INSTALL_DIR}/lib/libhpipm.so
    ${CMAKE_SOURCE_DIR}/lib/hpipm/hpipm/lib/lib/libhpipm.a 
    ${CMAKE_SOURCE_DIR}/lib/hpipm/blasfeo/lib/lib/libblasfeo.a
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
target_include_directories(${PROJECT_NAME}      PRIVATE     ${CMAKE_SOURCE_DIR}/lib/matplotlib-cpp)
target_link_libraries(${PROJECT_NAME}      
    ${Python3_LIBRARIES}
    ${Python3_NumPy_LIBRARIES}
)
target_compile_definitions(${PROJECT_NAME}      PRIVATE
    WITHOUT_NUMPY
    MPL_strcasecmp=strcasecmp
)
##########################################################################################################

# ldd devel/lib/lateral_control/lateral_control
# echo $LD_LIBRARY_PATH