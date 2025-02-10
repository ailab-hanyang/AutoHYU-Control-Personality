/**
 * @file        ros_bridge_control.hpp
 * @brief       ROS bridge for control
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2023-07-14 created by Yuseung Na
 *              2024-04-01 updated by Yuseung Na: Refactoring 
 */

#ifndef __ROS_BRIDGE_CONTROL__
#define __ROS_BRIDGE_CONTROL__
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Message Header
#include <autohyu_msgs/Trajectories.h>

// Interface Header
#include "interface_control.hpp"

// Bridge Header
#include "ros_bridge_header.hpp"

namespace ros_bridge {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
 
} // namespace ros_bridge

#endif  // __ROS_BRIDGE_CONTROL__