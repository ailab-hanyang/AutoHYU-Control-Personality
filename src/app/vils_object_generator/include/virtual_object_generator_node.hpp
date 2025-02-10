/**
 * Module:      virtual_object_generator_node.hpp
 * Description: virtual object generator node
 * 
 * Authors: Kyungwoon So (bigcow1999@gmail.com)
 *          Yuseung Na (ys.na0220@gmail.com) 
 * 
 * Revision History
 *      Sep. 07, 2023: Kyungwoon So - Created
 *      Sep. 21, 2023: Yuseung Na - Modify for real vehicle test
 */

#ifndef __VIRTUAL_OBJECT_GENERATOR_NODE_HPP__
#define __VIRTUAL_OBJECT_GENERATOR_NODE_HPP__
#pragma once

// STD header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// System Header
#include "task_manager.hpp"
#include "ini_parser.h"

// Bridge Header
#include "ros_bridge_objects.hpp"
#include "ros_bridge_trajectories.hpp"
#include "ros_bridge_vehicle_state.hpp"

// Parameter Header
#include "virtual_object_generator_config.hpp"

// Algorithm header
#include "scenario_parser/scenario_parser_algorithm.hpp"

// Namespace
using namespace ros;
using namespace std;
using namespace tf;

class VirtualObjectGenerator : public TaskManager {
    public:
        // Constructor
        explicit VirtualObjectGenerator(std::string task_node, double period);
        // Destructor
        virtual ~VirtualObjectGenerator();

    public:
        void Run();
        void Publish();
        void Terminate();
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
        inline void CallbackBehaviorTrajectory(const autohyu_msgs::BehaviorTrajectory::ConstPtr& msg) {
            mutex_behavior_trajectory_.lock();
            i_behavior_trajectory_ = ros_bridge::GetBehaviorTrajectory(*msg);
            b_is_behavior_trajectory_ = true;
            mutex_behavior_trajectory_.unlock();
        }

        // Update functions for publish variables
        void UpdateVirtualObjects(const interface::VehicleState& vehicle_state,
                                  const interface::TrackObjects& objects,
                                  const vector<double> ego_frenet,
                                  tk::Map& road_map,
                                  tk::spline& left_boundary,
                                  tk::spline& right_boundary,
                                  const VirtualObjectGeneratorParams params);
        void UpdateRvizVirtualObjects(const interface::TrackObjects& virtual_objects);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables

        // Subscriber
        Subscriber s_vehicle_state_;
        Subscriber s_behavior_trajectory_;

        // Publisher
        Publisher p_virtual_objects_;
        Publisher p_rviz_virtual_objects_;

        // Inputs
        interface::VehicleState i_vehicle_state_;
        interface::BehaviorTrajectory i_behavior_trajectory_;

        // Mutex
        mutex mutex_vehicle_state_;
        mutex mutex_behavior_trajectory_;

        // Outputs
        autohyu_msgs::TrackObjects o_virtual_objects_;
        visualization_msgs::MarkerArray o_rviz_virtual_objects_;

        // Algorithms
        std::unique_ptr<ScenarioParser> ptr_scenario_parser_;

        // Utils
        IniParser util_ini_parser_;

        // Configuration parameters
        bool b_is_vehicle_state_ = false;
        bool b_is_behavior_trajectory_ = false;
        VirtualObjectGeneratorParams params_;
};

#endif  // __VIRTUAL_OBJECT_GENERATOR_NODE_HPP__