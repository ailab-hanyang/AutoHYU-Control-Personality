/****************************************************************************/
// Module:      delayed_state_node.hpp
// Description: delayed_state node
//
// Authors: seounghoon
/****************************************************************************/

#ifndef __DELAYED_STATE_NODE_HPP__
#define __DELAYED_STATE_NODE_HPP__
#pragma once

// STD header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// ROS header
#include <ros/package.h>
#include <ros/ros.h>

// ROS Message Header
#include <autohyu_msgs/VehicleState.h>

// Utility Header
#include <task_manager.hpp>
#include <ini_parser.h>

// Interface Header
#include "ros_bridge_vehicle_state.hpp"

// Parameter Header
#include "delayed_state_params.hpp"

// State estimation algorithm
#include "delayed_state_algorithm.hpp"

// Namespace
using namespace ros;
using namespace tf;
using namespace std;
using namespace interface;

class DelayedStateNode: public TaskManager {
public:
    // Constructor
    explicit DelayedStateNode(std::string task_node, double period);
    // Destructor
    virtual ~DelayedStateNode();

public:
    void Init();
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
        VehicleState vehicle_state = ros_bridge::GetVehicleState(*msg);
        i_vehicle_state_vec_.push_back(vehicle_state);
        b_is_vehicle_state_ = true;
        mutex_vehicle_state_.unlock();
    }
    
    // Update functions for publish variables
    void UpdateDelayedState(const VehicleState& vehicle_state);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Variables

    // Subscriber
    Subscriber s_vehicle_state_;

    // Publisher
    Publisher        p_delayed_state_;
    
    // Inputs
    std::vector<VehicleState>     i_vehicle_state_vec_;
    
    // Mutex
    mutex           mutex_vehicle_state_;
    
    // Variables
    
    // Outputs
    autohyu_msgs::VehicleState     o_delayed_state_;
    
    // Algorithms
    std::shared_ptr<DelayedStateAlgorithm> ptr_delayed_state_algorithm_;

    // Utils
    IniParser util_ini_parser_;

    // Configuration and Parameters
    DelayedStateParams params_;

    bool b_is_vehicle_state_;
};

#endif // __DELAYED_STATE_NODE_HPP__