/**
 * @file        ros_bridge_header.hpp
 * @brief       ROS bridge for header
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2023-07-14 created by Yuseung Na
 *              2024-04-01 updated by Yuseung Na: Refactoring 
 */

#ifndef __ROS_BRIDGE_HEADER__
#define __ROS_BRIDGE_HEADER__
#pragma once

// ROS header
#include <ros/ros.h>

// ROS Message Header
#include <std_msgs/Header.h>

// Interface Header
#include "interface_header.hpp"
#include "interface_constants.hpp"

// Bridge Header
#include "ros_bridge_time.hpp"

namespace ros_bridge {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    interface::Header GetHeader(const std_msgs::Header& msg) {
        interface::Header header;

        header.seq      = msg.seq;
        header.stamp    = ros_bridge::GetTimeStamp(msg.stamp);
        header.frame_id = msg.frame_id;

        return header;
    }
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //    
    std_msgs::Header UpdateHeader(const interface::Header& header) {
        std_msgs::Header msg;

        msg.seq      = header.seq;
        msg.stamp    = ros::Time(header.stamp);
        msg.frame_id = header.frame_id;

        return msg;
    }

} // namespace ros_bridge

#endif  // __ROS_BRIDGE_HEADER__