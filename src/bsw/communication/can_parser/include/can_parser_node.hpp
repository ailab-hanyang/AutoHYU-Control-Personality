/**
 * @file        can_parser_node.hpp
 * @brief       node hpp file for can parser node
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-04-12 created by Yuseung Na
 * 
 */

#ifndef __CAN_PARSER_NODE_HPP__
#define __CAN_PARSER_NODE_HPP__
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
#include <autohyu_msgs/VehicleCAN.h>

// System Header
#include "task_manager.hpp"
#include "ini_parser.h"

// Interface Header
#include "interface_candb.hpp"

// Parameter Header
#include "can_parser_config.hpp"

// Util Function Header
#include "function_vehicle_state.hpp"

// Namespace
using namespace ros;
using namespace std;

class CANParser : public TaskManager {
    public:
        // Constructor
        explicit CANParser(std::string node_name, double period);
        // Destructor
        virtual ~CANParser();

        void Run();
        void Publish();
        void ProcessINI();
        void ProcessRosparam(const ros::NodeHandle& nh);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions

        // Callback functions for subscribe variables        
        inline void CallbackFrameFD0(const autohyu_msgs::FrameFD::ConstPtr& msg) {
            mutex_frame_fd_0_.lock();
            if (cfg_.vehicle == "lab_ioniq") {
                UpdateLabIONIQCAN(*msg);
            }
            else if (cfg_.vehicle == "hmg_ioniq") {
                UpdateHMGIONIQCAN(*msg);
            }
            else {
                ROS_ERROR("Invalid target car name");
            }
            mutex_frame_fd_0_.unlock();
        }
        // inline void CallbackFrameFD1(const autohyu_msgs::FrameFD::ConstPtr& msg) {
        //     mutex_frame_fd_1_.lock();
        //     UpdateVehicleCAN(*msg);
        //     mutex_frame_fd_1_.unlock();
        // }

        // Update functions for publish variables
        void UpdateLabIONIQCAN(const autohyu_msgs::FrameFD& msg);
        void UpdateHMGIONIQCAN(const autohyu_msgs::FrameFD& msg);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        
        // Subscriber
        ros::Subscriber s_frame_fd_0_;
        ros::Subscriber s_frame_fd_1_;

        // Input
        

        // Mutex
        std::mutex mutex_frame_fd_0_;
        std::mutex mutex_frame_fd_1_;
        
        // Publisher
        ros::Publisher p_vehicle_can_;

        // Output
        autohyu_msgs::VehicleCAN   o_vehicle_can_;

        // Algorithm

        // Util and Configuration
        IniParser util_ini_parser_;
        CANParserConfig cfg_;

        // Flags

        // Global Variables
        // ADD GLOBAL VARIABLES HERE

};

#endif  // __CAN_PARSER_NODE_HPP__