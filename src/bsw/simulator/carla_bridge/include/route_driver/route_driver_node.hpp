/**
 * @file        route_driver_node.hpp
 * @brief       carla route driver node hpp file
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-08-01 created by Jeonghun Kang
 * 
 */

#ifndef __CARLA_ROUTE_DRIVER_HPP__
#define __CARLA_ROUTE_DRIVER_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <random>

// ROS Header
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// ROS Message Header
#include <carla_msgs/CarlaRoute.h>
#include <carla_msgs/CarlaGnssRoute.h>
#include <autohyu_msgs/Reference.h>
// #include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <autohyu_msgs/GoalPoints.h>
#include <geometry_msgs/Point.h>
#include "ros_bridge_geometry_msgs.hpp"

// System Header
#include <task_manager.hpp>
#include <ini_parser.h>

// Utility header
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_projection/UTM.h>

// Interface Header

// Ros Bridge Header
#include "ros_bridge_vehicle_state.hpp"
#include "ros_bridge_time.hpp"
#include "ros_bridge_trajectories.hpp"

// Parameter Header
#include "route_driver_config.hpp"

// Namespace
using namespace ros;
using namespace tf;
using namespace std;
using namespace interface;

class RouteDriver : public TaskManager {
    public:
        // Constructor
        explicit RouteDriver(std::string node_name, double period);
        // Destructor
        virtual ~RouteDriver();

        void Run();
        void Publish();
        void ProcessINI();
        void ProcessRosparam(const ros::NodeHandle& nh);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions

        // Callback functions for subscribe variables
        inline void CallbackCARLARoute(const carla_msgs::CarlaRoute::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_carla_route_);
            i_carla_route_ = GetCARLARoute(*msg);
            is_initialized = true;
        }
        inline void CallbackCARLAGNSSRoute(const carla_msgs::CarlaGnssRoute::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_carla_gnss_route_);
            i_carla_gnss_route_ = *msg;
            is_initialized = true;
        }
        inline void CallbackReference(const autohyu_msgs::Reference::ConstPtr& msg) {
            mutex_ref_origin_.lock();
            i_ref_origin_ = ros_bridge::GetReference(*msg);
            b_is_ref_origin_ = true;
            mutex_ref_origin_.unlock();
        }

        Trajectory GetCARLARoute(const carla_msgs::CarlaRoute& msg);
        Trajectory ProjectCARLAGNSSRoute(const carla_msgs::CarlaGnssRoute& msg);

        // Update functions for publish variables
        void UpdateGoalPoints(const Trajectory& trajectory);
        void UpdateRvizCARLARoute(const Trajectory& trajectory);
        void UpdateRvizCARLAGNSSRoute(const Trajectory& trajectory);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables     

        // Subscriber
        Subscriber s_carla_route_;
        Subscriber s_carla_gnss_route_;
        Subscriber s_ref_origin_;

        // Publisher
        Publisher p_goal_points_;
        Publisher p_rviz_route_;
        Publisher p_rviz_gnss_route_;

        // Inputs
        Trajectory i_carla_route_;
        carla_msgs::CarlaGnssRoute i_carla_gnss_route_;
        Reference i_ref_origin_;

        mutex mutex_carla_route_;
        mutex mutex_carla_gnss_route_;
        mutex mutex_ref_origin_;

        // Outputs
        autohyu_msgs::GoalPoints o_goal_points_;
        visualization_msgs::MarkerArray o_rviz_route_;
        visualization_msgs::MarkerArray o_rviz_gnss_route_;

        // Environments
        IniParser util_ini_parser_;
        // Algorithms

        // Configuration parameters
        CARLARouteDriverConfig cfg_;
        bool is_initialized = false;
        bool b_is_ref_origin_ = false;
};

#endif  // __CARLA_ROUTE_DRIVER_HPP__