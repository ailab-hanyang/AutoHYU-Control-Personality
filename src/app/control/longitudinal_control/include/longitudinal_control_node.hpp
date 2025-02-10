/**
 * @file        longitudinal_control.hpp
 * @brief       longitudinal control node
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)    
 *              Junhee Lee (998jun@gmail.com)      
 * 
 * @date        2023-02-02 Yuseung Na - Created. (Longitudinal MPC using CVXGEN)
 *              2023-03-10 Junhee Lee - Add dynamic MPC model
 *              2023-04-12 Junhee Lee - Add dynamic MPC du model (HPIPM)
 *              2023-08-06 Junhee Lee - Code Cleanup
 */

#ifndef __LONGITUDINAL_CONTROL_NODE_HPP__
#define __LONGITUDINAL_CONTROL_NODE_HPP__
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
#include <autohyu_msgs/DetectObjects3D.h>
#include <autohyu_msgs/TrackObjects.h>
#include <autohyu_msgs/PredictObjects.h>
#include <autohyu_msgs/Trajectories.h>
#include <autohyu_msgs/VehicleCmd.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>

// System Header
#include "task_manager.hpp"
#include "ini_parser.h"

// Bridge Header
#include "ros_bridge_control.hpp"
#include "ros_bridge_objects.hpp"
#include "ros_bridge_time.hpp"
#include "ros_bridge_trajectories.hpp"
#include "ros_bridge_vehicle_state.hpp"

// Parameter Header

// Algorithm Header
#include "longitudinal_control/longitudinal_control.hpp"

// Namespace
using namespace ros;
using namespace interface;
using namespace std;

class LongitudinalControlNode : public TaskManager {
    public:
        // Constructor
        explicit LongitudinalControlNode(std::string task_node, double period);
        // Destructor
        virtual ~LongitudinalControlNode();

        void Run();
        void Publish();
        void ProcessINI();
        void ProcessRosparam(const ros::NodeHandle& nh);

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
        void UpdateControlCommand(const ControlCommand& cmd);
        void UpdateSpeedTrackingInfos(const PathTrackingInfos& speed_tracking_errors,
                            const VehicleState& vehicle_state,
                            const ControlTrajectory& trajectory);
        
        void UpdateRvizPredictedSpeedProfile(const ControlTrajectory& control_trajectory);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        
        // Subscriber
        ros::Subscriber s_vehicle_state_;
        ros::Subscriber s_predict_objects_;
        ros::Subscriber s_trajectory_;

        // Input
        interface::VehicleState    i_vehicle_state_;
        interface::Trajectory      i_trajectory_;

        // Mutex
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_predict_objects_;
        std::mutex mutex_trajectory_;
        
        // Publisher
        ros::Publisher p_command_torque_;
        ros::Publisher p_command_acceleration_;
        
        ros::Publisher p_target_speed_;
        ros::Publisher p_target_ax_;
        ros::Publisher p_current_speed_;
        ros::Publisher p_current_speederror_;
        ros::Publisher p_current_axerror_;
        ros::Publisher p_compensated_torque_;
        ros::Publisher p_speed_compensation_error_;
        ros::Publisher p_predicted_speed_profile_;

        // Output
        std_msgs::Float32 o_command_torque_;
        std_msgs::Float32 o_command_acceleration_;
 
        std_msgs::Float32 o_target_speed_;
        std_msgs::Float32 o_target_ax_;        
        std_msgs::Float32 o_current_speed_;        
        std_msgs::Float32 o_current_speederror_;        
        std_msgs::Float32 o_current_accelerror_;        
        std_msgs::Float32 o_compensated_torque_;        
        std_msgs::Float32 o_speed_compensation_error_;  

        visualization_msgs::MarkerArray o_predicted_speed_profile_;

        // Algorithm
        unique_ptr<LongitudinalControl> ptr_longitudinal_control_;

        // Util and Configuration
        IniParser util_ini_parser_;
        // TemplateNodeConfig cfg_;

        // Flags
        double d_time_stamp_;

        bool b_is_vehicle_state_ = false;        
        bool b_is_trajectory_ = false;
        bool b_use_predicted_speed_ = false; 

        // Global Variables
        // ADD GLOBAL VARIABLES HERE

};

#endif  // __LONGITUDINAL_CONTROL_NODE_HPP__