/**
 * @file        traffic_driver_node.hpp
 * @brief       morai traffic driver node hpp file
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-05-16 created by Jeonghun Kang
 * 
 */

#ifndef __MORAI_TRAFFIC_DRIVER_HPP__
#define __MORAI_TRAFFIC_DRIVER_HPP__
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
#include <morai_msgs/GetTrafficLightStatus.h>
#include <autohyu_msgs/TrafficLight.h>

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
#include "interface_traffic_light.hpp"

// Ros Bridge Header
#include "ros_bridge_vehicle_state.hpp"
#include "ros_bridge_time.hpp"

// Parameter Header
#include "traffic_driver_config.hpp"

// Namespace
using namespace ros;
using namespace tf;
using namespace std;
using namespace interface;

class TrafficDriver : public TaskManager {
    public:
        // Constructor
        explicit TrafficDriver(std::string node_name, double period);
        // Destructor
        virtual ~TrafficDriver();

        void Run();
        void Publish();
        void ProcessINI();
        void ProcessRosparam(const ros::NodeHandle& nh);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions

        // Callback functions for subscribe variables
        inline void CallbackMRTrafficLight(const morai_msgs::GetTrafficLightStatus::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_mr_traffic_light_);
            i_mr_traffic_light_ = GetMRTrafficLight(*msg);
            is_initialized = true;
        }

        TrafficLight GetMRTrafficLight(const morai_msgs::GetTrafficLightStatus& msg);

        // Update functions for publish variables
        void UpdateTrafficLight(const TrafficLight& traffic_light);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables     

        // Subscriber
        Subscriber s_mr_traffic_light_;

        // Publisher
        Publisher p_traffic_light_;

        // Inputs
        TrafficLight i_mr_traffic_light_;

        mutex mutex_mr_traffic_light_;

        // Outputs
        autohyu_msgs::TrafficLight o_traffic_light_;

        // Environments
        IniParser util_ini_parser_;
        
        // Algorithms

        // Configuration parameters
        MRTrafficDriverConfig cfg_;
        bool is_initialized = false;
};

#endif  // __MORAI_TRAFFIC_DRIVER_HPP__