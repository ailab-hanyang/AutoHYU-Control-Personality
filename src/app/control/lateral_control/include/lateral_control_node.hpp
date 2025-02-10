/**
 * @file        lateral_control.hpp
 * @brief       lateral control node
 * 
 * @authors     Junhee Lee (998jun@gmail.com)         
 */

#ifndef __LATERAL_CONTROL_NODE_HPP__
#define __LATERAL_CONTROL_NODE_HPP__
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
#include <autohyu_msgs/VehicleState.h>
#include <autohyu_msgs/Trajectories.h>
#include <autohyu_msgs/VehicleCmd.h>
#include <autohyu_msgs/ControlInfo.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>

// System Header
#include "task_manager.hpp"
#include "ini_parser.h"

// Bridge Header
#include "ros_bridge_control.hpp"
#include "ros_bridge_time.hpp"
#include "ros_bridge_trajectories.hpp"
#include "ros_bridge_vehicle_state.hpp"

// Parameter Header
#include "lateral_control_config.hpp"

// Algorithm Header
#include "spline.h"
#include "lateral_control/lateral_control.hpp"

// Namespace
using namespace ros;
using namespace tf;
using namespace std;

class LateralControlNode : public TaskManager {
    public:
        // Constructor
        explicit LateralControlNode(std::string task_node, double period);
        // Destructor
        virtual ~LateralControlNode();

        void Run();
        void Publish();
        void ProcessINI();

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions

        // Callback functions for subscribe variables        
        inline void CallbackVehicleState(const autohyu_msgs::VehicleState::ConstPtr& msg) {
            mutex_vehicle_state_.lock();
            i_vehicle_state_ = ros_bridge::GetVehicleState(*msg);
            b_is_vehicle_state_ = true;
            mutex_vehicle_state_.unlock();
        }
        inline void CallbackTrajectory(const autohyu_msgs::Trajectory::ConstPtr& msg) {
            mutex_trajectory_.lock();
            i_trajectory_ = ros_bridge::GetTrajectory(*msg);
            b_is_trajectory_ = true;
            mutex_trajectory_.unlock();
        }

        // Update functions for publish variables
        void UpdateSteeringTireCommand(const float& steering_tire_command);
        void UpdatePathTrackingInfos(const PathTrackingInfos& path_tracking_errors);
        
        void UpdateRvizControlPrediction(const ControlTrajectory& control_trajectory);
        void UpdateRvizControlReference(const ControlTrajectory& trajectory);
    

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        
        // Subscriber
        ros::Subscriber s_vehicle_state_;
        ros::Subscriber s_trajectory_;

        // Input
        interface::VehicleState     i_vehicle_state_;
        interface::Trajectory       i_trajectory_;

        // Mutex
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_trajectory_;
        
        // Publisher
        ros::Publisher p_command_steer_;
        ros::Publisher p_control_trajectory_;
        ros::Publisher p_reference_path_;
        ros::Publisher p_control_info_;
        ros::Publisher p_current_understeer_gradient_;


        // Output
        std_msgs::Float32 o_command_steer_;
        autohyu_msgs::ControlInfo o_control_info_;
        visualization_msgs::MarkerArray o_control_trajectory_;
        visualization_msgs::MarkerArray o_reference_path_;      
        std_msgs::Float32 o_current_understeer_gradient_;

        // Algorithm
        std::unique_ptr<LateralControl> ptr_lateral_control_;

        // Util and Configuration
        IniParser util_ini_parser_;
        // TemplateNodeConfig cfg_;
        double d_time_stamp_;

        // Flags
        bool b_is_vehicle_state_ = false;        
        bool b_is_trajectory_ = false;
        bool b_use_predicted_path_  = false; 
        // Global Variables
        // ADD GLOBAL VARIABLES HERE

};

#endif  // __LATERAL_CONTROL_NODE_HPP__