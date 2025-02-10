/****************************************************************************/
// Module:      rosbridge_time.hpp
// Description: ROS bridge for time
//
// Authors: Yuseung Na (yuseungna@hanyang.ac.kr)
//          Jeonghun Kang (kjhoon9674@gmail.com)
// Version: 1.1
//
// Revision History
//      June 14,  2023: Yuseung Na - Created.
//      March 17, 2024: Jeonghun Kang - Updated to autohyu
/****************************************************************************/

#ifndef __ROS_BRIDGE_TIME_HPP__
#define __ROS_BRIDGE_TIME_HPP__
#pragma once

// ROS header
#include <ros/ros.h>

// ROS Message Header

// Interface Header

namespace ros_bridge {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    double GetTimeStamp(const ros::Time& stamp) {
        return (double)stamp.sec + (double)stamp.nsec * 1e-9;
    }
} // namespace ros_bridge

#endif  // __ROS_BRIDGE_TIME_HPP__