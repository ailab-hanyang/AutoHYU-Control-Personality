/**
 * @file        can_transmitter_node.hpp
 * @brief       node cpp file for can transmitter node
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-09-03 created by Yuseung Na
 * 
 */

#ifndef __CAN_TRANSMITTER_NODE_HPP__
#define __CAN_TRANSMITTER_NODE_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// ROS Header
#include <ros/ros.h>

// ROS Message Header
#include <autohyu_msgs/FrameFD.h>
#include <autohyu_msgs/VehicleCmd.h>
#include <autohyu_msgs/ADModeInput.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

// System Header
#include "task_manager.hpp"
#include "ini_parser.h"

// Interface Header
#include "ros_bridge_vehicle_state.hpp"
#include "interface_vehicle_state.hpp"
#include "interface_candb.hpp"
#include "interface_control.hpp"

// Parameter Header

// Algorithm Header
#include "aps_table/TorqueMap.h"
#include "wheel_to_steering/wheel_to_steering.h"

// Namespace
using namespace ros;
using namespace std;

class CANTransmitter : public TaskManager {
    public:
        // Constructor
        explicit CANTransmitter(std::string node_name, double period);
        // Destructor
        virtual ~CANTransmitter();

    public:
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
            b_is_vehicle_state_ = true;
        }
        inline void CallbackCommandAccel(const std_msgs::Float32::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_command_accel_);
            i_command_accel_ = msg->data;
        }
        inline void CallbackCommandTorque(const std_msgs::Float32::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_command_torque_);
            i_command_torque_ = msg->data;
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
        inline void CallbackCommandSteer(const std_msgs::Float32::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_command_steer_);
            i_command_steer_ = msg->data;
        }
        inline void CallbackHealth(const std_msgs::UInt8::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_health_);
            i_health_sequence_ = msg->data;

            b_is_health_sequence_ = true;
        }
        inline void CallbackADModeInput(const autohyu_msgs::ADModeInput::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_ad_mode_input_);
            i_ad_mode_input_.ready_mode = msg->ready_mode;
            i_ad_mode_input_.lateral_mode = msg->lateral_mode;
            i_ad_mode_input_.longitudinal_mode = msg->longitudinal_mode;
            i_ad_mode_input_.manual_mode = msg->manual_mode;

            b_is_ad_mode_input_ = true;
        }

        // Update functions for subscribe variables
        void UpdateVehicleCommand(const interface::VehicleState& vehicle_state, 
                                  const double& accel, 
                                  const double& torque, 
                                  const double& speed, 
                                  const double& steer);
        void UpdateKeyboardCommand(const interface::VehicleState& vehicle_state, 
                                   const interface::VehicleCmd& keyboard_command);
        void UpdateAutokuCanHealth(const uint8_t& health_sequence);
        void UpdateAutokuCanAdMode(const interface::ADModeInput& ad_mode_input);    
        void UpdateEFlag();

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        bool b_is_vehicle_state_ = false;
        bool b_is_ad_mode_input_ = false;
        bool b_is_health_sequence_ = false;
        bool b_is_vehicle_cmd_ = false;
        
        bool b_use_keyboard_input = false;
        bool b_use_steering_speed_limit = false;
        double d_steering_speed_threshold = 800;

        // Subscriber
        Subscriber s_vehicle_state_;
        Subscriber s_command_accel_;
        Subscriber s_command_torque_;
        Subscriber s_command_speed_;
        Subscriber s_keyboard_command_;
        Subscriber s_command_steer_;
        Subscriber s_health_;
        Subscriber s_ad_mode_input_;

        // Publisher
        Publisher p_steering_wheel_angle_;
        Publisher p_gas_;
        Publisher p_brake_;
        Publisher p_commands_;
        Publisher p_can_tx_;

        // Inputs        
        interface::VehicleState i_vehicle_state_;
        double i_command_accel_ = 0.0;
        double i_command_torque_ = 0.0;
        double i_command_speed_ = 0.0;
        double i_command_steer_ = 0.0;
        interface::VehicleCmd i_keyboard_command_;
        uint8_t i_health_sequence_;
        interface::ADModeInput i_ad_mode_input_;

        // Outputs
        std_msgs::Float32 o_steering_wheel_angle_;
        std_msgs::Float32 o_gas_;
        std_msgs::Float32 o_brake_;
        autohyu_msgs::VehicleCmd o_vehicle_cmd_;
        
        autohyu_msgs::FrameFD o_vehicle_cmd_can_tx_;
        autohyu_msgs::FrameFD o_health_can_tx_;
        autohyu_msgs::FrameFD o_ad_mode_can_tx_;
        autohyu_msgs::FrameFD o_pseudo_e_flag_can_tx_;
        
        // Mutex
        mutex mutex_vehicle_state_;
        mutex mutex_command_accel_;
        mutex mutex_command_torque_;
        mutex mutex_command_speed_;
        mutex mutex_command_steer_;
        mutex mutex_keyboard_command_;
        mutex mutex_health_;
        mutex mutex_ad_mode_input_;

        // Algorithm
        unique_ptr<TorqueMap> ptr_aps_table_;
        unique_ptr<Wheel2Steering> ptr_wheel_to_steering_;

        // Utils
        IniParser util_ini_parser_;

        // Configuration and Parameters
        bool b_is_valid_ = true;
};

#endif // __CAN_TRANSMITTER_NODE_HPP__