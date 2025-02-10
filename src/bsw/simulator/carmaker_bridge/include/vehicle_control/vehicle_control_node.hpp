/**
 * @file        vehicle_control_node.hpp
 * @brief       carmaker vehicle control node hpp file
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-05-16 created by Jeonghun Kang
 * 
 */
#ifndef __CARMAKER_VEHICLE_CONTROL_HPP__
#define __CARMAKER_VEHICLE_CONTROL_HPP__
#pragma once

// STD header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// ROS header
#include <ros/ros.h>

// ROS Message Header
#include <carmaker_msgs/Control_Signal.h>
#include <autohyu_msgs/VehicleState.h>
#include <autohyu_msgs/VehicleCmd.h>
#include <std_msgs/Float32.h>

// System Header
#include <task_manager.hpp>
#include <ini_parser.h>

// Interface Header
#include "interface_vehicle_state.hpp"
#include "interface_control.hpp"
#include "interface_constants.hpp"

// Ros Bridge Header
#include "ros_bridge_vehicle_state.hpp"

// Util Function Header

// Parameter Header
#include "vehicle_control_config.hpp"

// Namespace
using namespace ros;
using namespace tf;
using namespace std;
using namespace interface;

class VehicleControl : public TaskManager {
    public:
        // Constructor
        explicit VehicleControl(std::string node_name, double period);
        // Destructor
        virtual ~VehicleControl();

        void Run();
        void Publish();
        void ProcessINI();
        void ProcessRosparam(const ros::NodeHandle& nh);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions

        // Callback functions for subscribe variables
        inline void CallbackVehicleState(const autohyu_msgs::VehicleState::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
            i_vehicle_state_ = ros_bridge::GetVehicleState(*msg);
        }
        inline void CallbackCommandTorque(const std_msgs::Float32::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_command_torque_);
            i_command_torque_ = msg->data;
        }
        inline void CallbackCommandSteer(const std_msgs::Float32::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_command_steer_);
            i_command_steer_ = msg->data;
        }
        inline void CallbackCommandAccel(const std_msgs::Float32::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_command_accel_);
            i_command_accel_ = msg->data;
        }
        inline void CallbackCommandSpeed(const std_msgs::Float32::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_command_speed_);
            i_command_speed_ = msg->data;
        }
        inline void CallbackKeyboardCommand(const autohyu_msgs::VehicleCmd::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_keyboard_command_);
            i_keyboard_command_.steering_angle = msg->steering_angle;
            i_keyboard_command_.front_tire_angle = msg->front_tire_angle;
            i_keyboard_command_.accel = msg->accel;
            i_keyboard_command_.gas = msg->gas;
            i_keyboard_command_.brake = msg->brake;
            i_keyboard_command_.torque = msg->torque;
        }
        std::tuple<double, double, double, double> CalculateCommand(const VehicleState& vehicle_state,
                                                            double& command_accel,
                                                            const double& command_torque,
                                                            const double& command_speed, 
                                                            const double& command_steer);

        // Update functions for publish variables
        void UpdateCommand(const double& steer, const double& gas, const double& brake, const double& accel);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables     

        // Subscriber
        Subscriber s_vehicle_state_;
        Subscriber s_command_steer_;
        Subscriber s_command_torque_;
        Subscriber s_command_speed_;
        Subscriber s_command_accel_;
        Subscriber s_keyboard_command_;
        // Publisher
        Publisher p_control_signal_;
        Publisher p_commands_;

        // Inputs
        VehicleState i_vehicle_state_;
        double i_command_torque_;
        double i_command_steer_;
        double i_command_speed_;
        double i_command_accel_;
        VehicleCmd i_keyboard_command_;

        mutex mutex_command_torque_;
        mutex mutex_command_steer_;
        mutex mutex_command_speed_;
        mutex mutex_command_accel_;
        mutex mutex_vehicle_state_;
        mutex mutex_keyboard_command_;
        // Outputs
        carmaker_msgs::Control_Signal o_control_signal_;
        autohyu_msgs::VehicleCmd o_vehicle_cmd_;

        // Environments
        IniParser util_ini_parser_;
        
        // Algorithms
        
        // Configuration parameters   
        CMVehicleControlConfig cfg_;

};

#endif  // __CARMAKER_VEHICLE_CONTROL_HPP__