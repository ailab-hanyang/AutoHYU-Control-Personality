/**
 * @file        state_estimation_node.hpp
 * @brief       node hpp file for state estimation node
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)
 * 
 * @date        2024-04-11 created by Yuseung Na
 *              2024-04-25 updated by Yuseung Na: Update reference managing system
 * 
 */

#ifndef __STATE_ESTIMATION_NODE_HPP__
#define __STATE_ESTIMATION_NODE_HPP__
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
#include "ros_bridge_novatel.hpp"
#include "ros_bridge_vehicle_state.hpp"

// Parameter Header
#include "state_estimation_config.hpp"

// Algorithm Header
#include "state_using_novatel/state_using_novatel_algorithm.hpp"

// Namespace
using namespace ros;
using namespace tf;
using namespace std;

class StateEstimation : public TaskManager {
    public:
        // Constructor
        explicit StateEstimation(std::string node_name, double period);
        // Destructor
        virtual ~StateEstimation();

        void Run();
        void Publish();
        void ProcessINI();
        void ProcessRosparam(const ros::NodeHandle& nh);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions

        // Callback functions for subscribe variables        
        inline void CallbackNovatelInspvax(const novatel_oem7_msgs::INSPVAX::ConstPtr& msg) {
            mutex_novatel_inspvax_.lock();
            i_novatel_inspvax_      = ros_bridge::GetNovatelInspvax(*msg);
            b_is_novatel_inspavx_   = true;
            mutex_novatel_inspvax_.unlock();
        }
        inline void CallbackNovatelCorrimu(const novatel_oem7_msgs::CORRIMU::ConstPtr& msg) {
            mutex_novatel_corrimu_.lock();
            i_novatel_corrimu_      = ros_bridge::GetNovatelCorrimu(*msg);
            b_is_novatel_corrimu_   = true;
            mutex_novatel_corrimu_.unlock();
        }
        inline void CallbackVehicleCAN(const autohyu_msgs::VehicleCAN::ConstPtr& msg) {
            mutex_vehicle_can_.lock();
            i_vehicle_can_ = ros_bridge::GetVehicleCAN(*msg);
            b_is_vehicle_can_ = true;
            mutex_vehicle_can_.unlock();
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        
        // Subscriber
        ros::Subscriber s_novatel_inspvax_;
        ros::Subscriber s_novatel_corrimu_;
        ros::Subscriber s_vehicle_can_;

        // Input
        interface::INSPVAX i_novatel_inspvax_;
        interface::CORRIMU i_novatel_corrimu_;
        interface::VehicleCAN i_vehicle_can_;

        // Mutex
        std::mutex mutex_novatel_inspvax_;
        std::mutex mutex_novatel_corrimu_;
        std::mutex mutex_vehicle_can_;
        
        // Publisher
        ros::Publisher p_vehicle_state_;
        ros::Publisher p_reference_point_;

        // Output
        autohyu_msgs::VehicleState o_vehicle_state_;
        autohyu_msgs::Reference    o_reference_point_;

        // Algorithm
        std::unique_ptr<StateUsingNovatel> alg_state_using_novatel_;

        // Util and Configuration
        IniParser util_ini_parser_;
        StateEstimationConfig cfg_;

        // Flags
        bool b_is_novatel_inspavx_ = false;        
        bool b_is_novatel_corrimu_ = false;
        bool b_is_vehicle_can_ = false;

        // Global Variables
        // ADD GLOBAL VARIABLES HERE

};

#endif  // __STATE_ESTIMATION_NODE_HPP__