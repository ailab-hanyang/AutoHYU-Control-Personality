/**
 * @file        object_driver_node.hpp
 * @brief       carla object driver node hpp file
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-08-01 created by Jeonghun Kang
 * 
 */
#ifndef __CARLA_OBJECT_DRIVER_HPP__
#define __CARLA_OBJECT_DRIVER_HPP__
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
#include "ros_bridge_header.hpp"
#include "ros_bridge_vehicle_state.hpp"
#include "ros_bridge_objects.hpp"
#include "interface_constants.hpp"

// Utility header
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_projection/UTM.h>

// Message Header
#include <derived_object_msgs/ObjectArray.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <carla_msgs/CarlaActorInfo.h>
#include <carla_msgs/CarlaActorList.h>
#include <nav_msgs/Odometry.h>

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
        inline void CallbackCARLAObjects(const derived_object_msgs::ObjectArray::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_carla_objects_);
            i_carla_objects_ = *msg;
            b_is_carla_objects_ = true;
        }
        inline void CallbackCARLAEgoVehicleInfo(const carla_msgs::CarlaEgoVehicleInfo::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_ego_vehicle_info_);
            i_ego_vehicle_info_ = *msg;
            b_is_carla_ego_vehicle_info_ = true;
        }
        inline void CallbackReference(const autohyu_msgs::Reference::ConstPtr& msg) {
            mutex_ref_origin_.lock();
            i_ref_origin_ = ros_bridge::GetReference(*msg);
            b_is_ref_origin_ = true;
            mutex_ref_origin_.unlock();
        }
        inline void CallbackCARLAVehicleOdometry(const nav_msgs::Odometry::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_ego_vehicle_odometry_);
            i_ego_vehicle_odometry_ = *msg;
            b_is_carla_ego_vehicle_odometry_ = true;
        }
        inline void CallbackCARLAActorList(const carla_msgs::CarlaActorList::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_carla_actor_list_);
            i_carla_actor_list_ = *msg;
            b_is_carla_actor_list_info_ = true;
        }
        interface::TrackObjects GetCARLAObjects(const derived_object_msgs::ObjectArray& msg, const interface::VehicleState& vehicle_state);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables     

        // Subscriber
        ros::Subscriber s_vehicle_state_;
        ros::Subscriber s_carla_objects_;
        ros::Subscriber s_ref_origin_;
        ros::Subscriber s_ego_vehicle_info_;
        ros::Subscriber s_ego_vehicle_odometry_;
        ros::Subscriber s_carla_actor_list_;

        // Input
        interface::VehicleState i_vehicle_state_;
        derived_object_msgs::ObjectArray i_carla_objects_;
        interface::Reference i_ref_origin_;
        carla_msgs::CarlaEgoVehicleInfo i_ego_vehicle_info_;
        nav_msgs::Odometry i_ego_vehicle_odometry_;
        carla_msgs::CarlaActorList i_carla_actor_list_;

        // Mutex
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_carla_objects_;
        std::mutex mutex_ref_origin_;
        std::mutex mutex_ego_vehicle_info_;
        std::mutex mutex_ego_vehicle_odometry_;
        std::mutex mutex_carla_actor_list_;

        // Publisher
        ros::Publisher p_track_objects_;
        ros::Publisher p_rviz_track_objects_;

        // Outputs
        autohyu_msgs::TrackObjects o_tracked_objects_;
        visualization_msgs::MarkerArray o_rviz_objects_;
        
        // Algorithm

        // Util and Configuration
        IniParser util_ini_parser_;   
        CARLAObjectDriverConfig cfg_;   
         
        // Flags
        bool b_is_vehicle_state_ = false;
        bool b_is_carla_objects_ = false;
        bool b_is_ref_origin_ = false;
        bool b_is_carla_ego_vehicle_info_ = false;
        bool b_is_carla_ego_vehicle_odometry_ = false;
        bool b_is_carla_actor_list_info_ = false;

        // Global Variables
        // ADD GLOBAL VARIABLES HERE
};

#endif  // __CARLA_OBJECT_DRIVER_HPP__