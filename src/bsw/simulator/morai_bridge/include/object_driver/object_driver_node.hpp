/**
 * @file        object_driver_node.hpp
 * @brief       carmaker object driver node hpp file
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-05-16 created by Jeonghun Kang
 * 
 */
#ifndef __MORAI_OBJECT_DRIVER_HPP__
#define __MORAI_OBJECT_DRIVER_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// System Header
#include <task_manager.hpp>
#include <ini_parser.h>

// Bridge Header
#include "ros_bridge_vehicle_state.hpp"
#include "ros_bridge_objects.hpp"
#include "ros_bridge_simulator.hpp"

// Parameter Header
#include "object_driver_config.hpp"

// Algorithm Header

// Namespace
using namespace ros;
using namespace tf;
using namespace std;

class ObjectDriver : public TaskManager {
    public:
        // Constructor
        explicit ObjectDriver(std::string node_name, double period);
        // Destructor
        virtual ~ObjectDriver();
;
        void Run();
        void Publish();
        void ProcessINI();
        void ProcessRosparam(const ros::NodeHandle& nh);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions

        // Callback functions for subscribe variables
        inline void CallbackVehicleState(const autohyu_msgs::VehicleState::ConstPtr& msg) {
            lock_guard<mutex> lock(mutex_vehicle_state_);
            i_vehicle_state_ = ros_bridge::GetVehicleState(*msg);
            b_is_vehicle_state_ = true;
        }
        inline void CallbackMoraiObjects(const morai_msgs::ObjectStatusList::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_morai_objects_);
            i_morai_objects_ = *msg;
            b_is_morai_objects_ = true;
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables     

        // Subscriber
        ros::Subscriber s_vehicle_state_;
        ros::Subscriber s_morai_objects_;

        // Input
        interface::VehicleState i_vehicle_state_;
        morai_msgs::ObjectStatusList i_morai_objects_;

        // Mutex
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_morai_objects_;

        // Publisher
        ros::Publisher p_track_objects_;
        ros::Publisher p_rviz_track_objects_;

        // Outputs
        autohyu_msgs::TrackObjects o_tracked_objects_;
        visualization_msgs::MarkerArray o_rviz_objects_;
        
        // Algorithm

        // Util and Configuration
        IniParser util_ini_parser_;   
        MRObjectDriverConfig cfg_;   
         
        // Flags
        bool b_is_vehicle_state_ = false;
        bool b_is_morai_objects_ = false;

        // Global Variables
        // ADD GLOBAL VARIABLES HERE
};

#endif  // __MORAI_OBJECT_DRIVER_HPP__