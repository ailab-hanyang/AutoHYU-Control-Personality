/**
 * @file        template_node_node.hpp
 * @brief       template node hpp file for template node
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-04-01 created by Yuseung Na
 * 
 */

#ifndef __TEMPLATE_NODE_NODE_HPP__
#define __TEMPLATE_NODE_NODE_HPP__
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
#include "ros_bridge_control.hpp"
#include "ros_bridge_lanelet.hpp"
#include "ros_bridge_novatel.hpp"
#include "ros_bridge_objects.hpp"
#include "ros_bridge_header.hpp"
#include "ros_bridge_trajectories.hpp"
#include "ros_bridge_vehicle_state.hpp"

// Parameter Header
#include "template_node_config.hpp"

// Algorithm Header
#include "template_algorithm/template_algorithm_algorithm.hpp"

// Namespace
using namespace ros;
using namespace tf;
using namespace std;

class TemplateNode : public TaskManager {
    public:
        // Constructor
        explicit TemplateNode(std::string node_name, double period);
        // Destructor
        virtual ~TemplateNode();

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
        inline void CallbackDetectObjects(const autohyu_msgs::DetectObjects3D::ConstPtr& msg) {
            mutex_detect_objects_.lock();
            i_detect_objects_ = ros_bridge::GetDetectObjects3D(*msg);
            mutex_detect_objects_.unlock();
        }
        inline void CallbackTrackObjects(const autohyu_msgs::TrackObjects::ConstPtr& msg) {
            mutex_track_objects_.lock();
            i_track_objects_ = ros_bridge::GetTrackObjects(*msg);
            mutex_track_objects_.unlock();
        }
        inline void CallbackPredictObjects(const autohyu_msgs::PredictObjects::ConstPtr& msg) {
            mutex_predict_objects_.lock();
            i_predict_objects_ = ros_bridge::GetPredictObjects(*msg);
            mutex_predict_objects_.unlock();
        }
        inline void CallbackTrajectory(const autohyu_msgs::Trajectory::ConstPtr& msg) {
            mutex_trajectory_.lock();
            i_trajectory_ = ros_bridge::GetTrajectory(*msg);
            b_is_trajectory_ = true;
            mutex_trajectory_.unlock();
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        
        // Subscriber
        ros::Subscriber s_vehicle_state_;
        ros::Subscriber s_detect_objects_;
        ros::Subscriber s_track_objects_;
        ros::Subscriber s_predict_objects_;
        ros::Subscriber s_trajectory_;

        // Input
        interface::VehicleState    i_vehicle_state_;
        interface::DetectObjects3D   i_detect_objects_;
        interface::TrackObjects    i_track_objects_;
        interface::PredictObjects  i_predict_objects_;
        interface::Trajectory      i_trajectory_;

        // Mutex
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_detect_objects_;
        std::mutex mutex_track_objects_;
        std::mutex mutex_predict_objects_;
        std::mutex mutex_trajectory_;
        
        // Publisher
        ros::Publisher p_predict_objects_;
        ros::Publisher p_trajectories_;

        // Output
        autohyu_msgs::Trajectory   o_trajectory_;
        autohyu_msgs::Trajectories o_trajectories_;

        // Algorithm
        std::unique_ptr<TemplateAlgorithm> alg_physics_based_;

        // Util and Configuration
        IniParser util_ini_parser_;
        TemplateNodeConfig cfg_;

        // Flags
        bool b_is_vehicle_state_ = false;        
        bool b_is_trajectory_ = false;

        // Global Variables
        // ADD GLOBAL VARIABLES HERE

};

#endif  // __TEMPLATE_NODE_NODE_HPP__