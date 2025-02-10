/**
 * @file        ros_bridge_novatel.hpp
 * @brief       ROS bridge for novatel GNSS/INS
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2023-10-05 created by Yuseung Na
 *              2024-04-01 updated by Yuseung Na: Refactoring 
 */

#ifndef __ROS_BRIDGE_NOVATEL__
#define __ROS_BRIDGE_NOVATEL__
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Message Header
#include <novatel_oem7_msgs/CORRIMU.h>
#include <novatel_oem7_msgs/INSPVAX.h>
#include <novatel_oem7_msgs/BESTPOS.h>
#include <novatel_oem7_msgs/BESTVEL.h>

// Interface Header
#include "interface_novatel.hpp"

// Bridge Header
#include "ros_bridge_header.hpp"

namespace ros_bridge {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    interface::CORRIMU GetNovatelCorrimu(const novatel_oem7_msgs::CORRIMU& msg) {
        interface::CORRIMU corrimu;
        corrimu.header                           = GetHeader(msg.header);       
        corrimu.nov_header.message_name          = msg.nov_header.message_name;
        corrimu.nov_header.message_id            = msg.nov_header.message_id;
        corrimu.nov_header.message_type          = msg.nov_header.message_type;
        corrimu.nov_header.sequence_number       = msg.nov_header.sequence_number;
        corrimu.nov_header.time_status           = msg.nov_header.time_status;
        corrimu.nov_header.gps_week_number       = msg.nov_header.gps_week_number;
        corrimu.nov_header.gps_week_milliseconds = msg.nov_header.gps_week_milliseconds;
        corrimu.imu_data_count                   = msg.imu_data_count;
        corrimu.pitch_rate                       = msg.pitch_rate;
        corrimu.roll_rate                        = msg.roll_rate;
        corrimu.yaw_rate                         = msg.yaw_rate;
        corrimu.lateral_acc                      = msg.lateral_acc;
        corrimu.longitudinal_acc                 = msg.longitudinal_acc;
        corrimu.vertical_acc                     = msg.vertical_acc;
        corrimu.reserved1                        = msg.reserved1;
        corrimu.reserved2                        = msg.reserved2;

        return corrimu;
    }

    interface::INSPVAX GetNovatelInspvax(const novatel_oem7_msgs::INSPVAX& msg) {
        interface::INSPVAX inspvax;
        inspvax.header                           = GetHeader(msg.header);   
        inspvax.nov_header.message_name          = msg.nov_header.message_name;
        inspvax.nov_header.message_id            = msg.nov_header.message_id;
        inspvax.nov_header.message_type          = msg.nov_header.message_type;
        inspvax.nov_header.sequence_number       = msg.nov_header.sequence_number;
        inspvax.nov_header.time_status           = msg.nov_header.time_status;
        inspvax.nov_header.gps_week_number       = msg.nov_header.gps_week_number;
        inspvax.nov_header.gps_week_milliseconds = msg.nov_header.gps_week_milliseconds;
        inspvax.ins_status                       = static_cast<interface::InertialSolutionStatus>(msg.ins_status.status);
        inspvax.pos_type                         = static_cast<interface::PositionOrVelocityType>(msg.pos_type.type);
        inspvax.latitude                         = msg.latitude;
        inspvax.longitude                        = msg.longitude;
        inspvax.height                           = msg.height;
        inspvax.undulation                       = msg.undulation;
        inspvax.north_velocity                   = msg.north_velocity;
        inspvax.east_velocity                    = msg.east_velocity;
        inspvax.up_velocity                      = msg.up_velocity;
        inspvax.roll                             = msg.roll;
        inspvax.pitch                            = msg.pitch;
        inspvax.azimuth                          = msg.azimuth;
        inspvax.latitude_stdev                   = msg.latitude_stdev;
        inspvax.longitude_stdev                  = msg.longitude_stdev;
        inspvax.height_stdev                     = msg.height_stdev;
        inspvax.north_velocity_stdev             = msg.north_velocity_stdev;
        inspvax.east_velocity_stdev              = msg.east_velocity_stdev;
        inspvax.up_velocity_stdev                = msg.up_velocity_stdev;
        inspvax.roll_stdev                       = msg.roll_stdev;
        inspvax.pitch_stdev                      = msg.pitch_stdev;
        inspvax.azimuth_stdev                    = msg.azimuth_stdev;
        inspvax.ext_sol_status                   = static_cast<interface::INSExtendedSolutionStatus>(msg.ext_sol_status.status);
        inspvax.time_since_update                = msg.time_since_update;

        return inspvax;
    }
} // namespace ros_bridge

#endif // __ROS_BRIDGE_NOVATEL__