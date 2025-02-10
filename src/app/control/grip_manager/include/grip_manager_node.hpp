/**
 * @file        grip_manager_node.hpp
 * @brief       
 * 
 * @authors     Junhee Lee (998jun@gmail.com)         
 * 
 * @date        2023-10-04 created by Junhee Lee
 * 
 */

#ifndef __GRIP_MANAGER_NODE_HPP__
#define __GRIP_MANAGER_NODE_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// System Header
#include "task_manager.hpp"
#include "ini_parser.h"

// Bridge Header
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "ros_bridge_vehicle_state.hpp"

// Parameter Header
#include <grip_manager_config.hpp>

// Algorithm Header
#include <grip_manager/grip_manager.hpp>

// Namespace
using namespace ros;
using namespace tf;
using namespace std;

class GripManagerNode : public TaskManager {
    public:
        // Constructor
        explicit GripManagerNode(std::string node_name, double period);
        // Destructor
        virtual ~GripManagerNode();

        void Run();
        void Publish();
        void ProcessINI();
        void ProcessRosparam(const ros::NodeHandle& nh);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        void UpdateCommand(const double& steering_tire_angle_deg, const double& torque, const double& accel);

        // Callback functions for subscribe variables        
        inline void CallbackVehicleState(const autohyu_msgs::VehicleState::ConstPtr& msg) {
            mutex_vehicle_state_.lock();
            i_vehicle_state_ = ros_bridge::GetVehicleState(*msg);
            b_is_vehicle_state_ = true;
            mutex_vehicle_state_.unlock();
        }
        inline void CallbackTargetSteering(const std_msgs::Float32::ConstPtr& msg){
            std::lock_guard<std::mutex> lock(mutex_target_steering_);
            i_target_steering_ = msg->data;
        }
        inline void CallbackTargetTorque(const std_msgs::Float32::ConstPtr& msg){
            std::lock_guard<std::mutex> lock(mutex_target_torque_);
            i_target_torque_ = msg->data;
        }
        inline void CallbackTargetAccel(const std_msgs::Float32::ConstPtr& msg){
            std::lock_guard<std::mutex> lock(mutex_target_accel_);
            i_target_accel_ = msg->data;
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        
        // Subscriber
        ros::Subscriber s_vehicle_state_;
        ros::Subscriber s_target_steering_;
        ros::Subscriber s_target_torque_;
        ros::Subscriber s_target_accel_;

        // Input
        interface::VehicleState     i_vehicle_state_;
        double                      i_target_steering_;
        double                      i_target_torque_;
        double                      i_target_accel_;

        // Mutex
        std::mutex mutex_vehicle_state_;
        mutex mutex_target_steering_;
        mutex mutex_target_torque_;
        mutex mutex_target_accel_;
        
        // Publisher
        ros::Publisher p_gripped_steering_;
        ros::Publisher p_gripped_torque_;
        ros::Publisher p_gripped_accel_;

        // Output
        std_msgs::Float32 o_gripped_steering_;
        std_msgs::Float32 o_gripped_torque_;
        std_msgs::Float32 o_gripped_accel_;

        // Algorithm
        std::unique_ptr<GripManager> ptr_grip_manager;

        // Util and Configuration
        IniParser util_ini_parser_;
        GripManagerNodeConfig cfg_;

        // Flags
        double d_time_stamp_;
        bool b_is_vehicle_state_ = false;        

        // Global Variables
        // ADD GLOBAL VARIABLES HERE

};

#endif  // __TEMPLATE_NODE_NODE_HPP__