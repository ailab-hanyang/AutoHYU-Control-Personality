/**
 * @file        ros_bridge_vehicle_state.hpp
 * @brief       ROS bridge for vehicle state
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2023-07-14 created by Yuseung Na
 *              2024-04-01 updated by Yuseung Na: Refactoring 
 */

#ifndef __ROS_BRIDGE_VEHICLE_STATE__
#define __ROS_BRIDGE_VEHICLE_STATE__
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Message Header
#include <autohyu_msgs/VehicleState.h>

// Interface Header
#include "interface_constants.hpp"
#include "interface_vehicle_state.hpp"

// Bridge Header
#include "ros_bridge_header.hpp"

namespace ros_bridge {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    interface::VehicleCAN GetVehicleCAN(const autohyu_msgs::VehicleCAN& msg) {
        interface::VehicleCAN vehicle_can;
        vehicle_can.header = GetHeader(msg.header);

        vehicle_can.lateral_accel       = msg.lateral_accel;
        vehicle_can.longitudinal_accel  = msg.longitudinal_accel;
        vehicle_can.yaw_rate            = msg.yaw_rate;

        vehicle_can.steering_wheel_angle    = msg.steering_wheel_angle;
        vehicle_can.steering_tire_angle     = msg.steering_tire_angle;
        vehicle_can.steering_speed          = msg.steering_speed;
        vehicle_can.steering_torque         = msg.steering_torque;
        vehicle_can.steering_state          = static_cast<interface::MdpsState>(msg.steering_state);

        vehicle_can.wheel_velocity_fl       = msg.wheel_velocity_fl;
        vehicle_can.wheel_velocity_fr       = msg.wheel_velocity_fr;
        vehicle_can.wheel_velocity_rl       = msg.wheel_velocity_rl;
        vehicle_can.wheel_velocity_rr       = msg.wheel_velocity_rr;
        vehicle_can.wheel_velocity_f_avg    = msg.wheel_velocity_f_avg;
        vehicle_can.wheel_velocity_r_avg    = msg.wheel_velocity_r_avg;
        vehicle_can.wheel_velocity_avg      = msg.wheel_velocity_avg;

        vehicle_can.motor_torque_f          = msg.motor_torque_f;
        vehicle_can.motor_torque_r          = msg.motor_torque_r;
        vehicle_can.motor_torque_total      = msg.motor_torque_total;

        vehicle_can.accel_position          = msg.accel_position;
        vehicle_can.brake_pressure          = msg.brake_pressure;
        vehicle_can.brake_active            = msg.brake_active;
        vehicle_can.gear_select             = static_cast<interface::Gear>(msg.gear_select);

        vehicle_can.operation_mode                  = static_cast<interface::OperationMode>(msg.operation_mode);
        vehicle_can.lateral_autonomous_mode         = static_cast<interface::AutonomousMode>(msg.lateral_autonomous_mode);
        vehicle_can.longitudinal_autonomous_mode    = static_cast<interface::AutonomousMode>(msg.longitudinal_autonomous_mode);

        return vehicle_can;
    }

    interface::WGS84 GetWGS84(const autohyu_msgs::WGS84& msg) {
        interface::WGS84 wgs84;
        wgs84.latitude  = msg.latitude;
        wgs84.longitude = msg.longitude;
        wgs84.altitude  = msg.altitude;

        return wgs84;
    }

    interface::Reference GetReference(const autohyu_msgs::Reference& msg) {
        interface::Reference reference;
        reference.projection        = msg.projection;
        reference.wgs84.latitude    = msg.wgs84.latitude;
        reference.wgs84.longitude   = msg.wgs84.longitude;
        reference.wgs84.altitude    = msg.wgs84.altitude;

        return reference;
    }

    interface::VehicleState GetVehicleState(const autohyu_msgs::VehicleState& msg) {
        interface::VehicleState vehicle_state;
        vehicle_state.header    = GetHeader(msg.header);

        vehicle_state.reference     = GetReference(msg.reference);
        vehicle_state.pos_type      = static_cast<interface::PositionOrVelocityType>(msg.pos_type);
        vehicle_state.gnss          = GetWGS84(msg.gnss);
        vehicle_state.gnss_stdev    = GetWGS84(msg.gnss_stdev);

        vehicle_state.x = msg.x;
        vehicle_state.y = msg.y;
        vehicle_state.z = msg.z;

        vehicle_state.vx = msg.vx;
        vehicle_state.vy = msg.vy;
        vehicle_state.vz = msg.vz;

        vehicle_state.ax = msg.ax;
        vehicle_state.ay = msg.ay;
        vehicle_state.az = msg.az;

        vehicle_state.roll = msg.roll;
        vehicle_state.pitch = msg.pitch;
        vehicle_state.yaw = msg.yaw;

        vehicle_state.roll_stdev = msg.roll_stdev;
        vehicle_state.pitch_stdev = msg.pitch_stdev;
        vehicle_state.yaw_stdev = msg.yaw_stdev;

        vehicle_state.roll_vel = msg.roll_vel;
        vehicle_state.pitch_vel = msg.pitch_vel;
        vehicle_state.yaw_vel = msg.yaw_vel;        

        vehicle_state.vehicle_can = GetVehicleCAN(msg.vehicle_can);

        return vehicle_state;
    }
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    autohyu_msgs::VehicleCAN UpdateVehicleCAN(const interface::VehicleCAN& vehicle_can) {
        autohyu_msgs::VehicleCAN msg;
        msg.header = UpdateHeader(vehicle_can.header);

        msg.lateral_accel                   = vehicle_can.lateral_accel;
        msg.longitudinal_accel              = vehicle_can.longitudinal_accel;
        msg.yaw_rate                        = vehicle_can.yaw_rate;

        msg.steering_wheel_angle            = vehicle_can.steering_wheel_angle;
        msg.steering_tire_angle             = vehicle_can.steering_tire_angle;
        msg.steering_speed                  = vehicle_can.steering_speed;
        msg.steering_torque                 = vehicle_can.steering_torque;
        msg.steering_state                  = static_cast<uint8_t>(vehicle_can.steering_state);

        msg.wheel_velocity_fl               = vehicle_can.wheel_velocity_fl;
        msg.wheel_velocity_fr               = vehicle_can.wheel_velocity_fr;
        msg.wheel_velocity_rl               = vehicle_can.wheel_velocity_rl;
        msg.wheel_velocity_rr               = vehicle_can.wheel_velocity_rr;
        msg.wheel_velocity_f_avg            = vehicle_can.wheel_velocity_f_avg;
        msg.wheel_velocity_r_avg            = vehicle_can.wheel_velocity_r_avg;
        msg.wheel_velocity_avg              = vehicle_can.wheel_velocity_avg;

        msg.motor_torque_f                  = vehicle_can.motor_torque_f;
        msg.motor_torque_r                  = vehicle_can.motor_torque_r;
        msg.motor_torque_total              = vehicle_can.motor_torque_total;

        msg.accel_position                  = vehicle_can.accel_position;
        msg.brake_pressure                  = vehicle_can.brake_pressure;
        msg.brake_active                    = vehicle_can.brake_active;
        msg.gear_select                     = static_cast<uint8_t>(vehicle_can.gear_select);

        msg.operation_mode                  = static_cast<uint8_t>(vehicle_can.operation_mode);
        msg.lateral_autonomous_mode         = static_cast<uint8_t>(vehicle_can.lateral_autonomous_mode);
        msg.longitudinal_autonomous_mode    = static_cast<uint8_t>(vehicle_can.longitudinal_autonomous_mode);

        return msg;
    }

    autohyu_msgs::WGS84 UpdateWGS84(const interface::WGS84& wgs84) {
        autohyu_msgs::WGS84 msg;
        msg.latitude = wgs84.latitude;
        msg.longitude = wgs84.longitude;
        msg.altitude = wgs84.altitude;

        return msg;
    }

    autohyu_msgs::Reference UpdateReference(const interface::Reference& reference) {
        autohyu_msgs::Reference msg;
        msg.projection = reference.projection;
        msg.wgs84 = UpdateWGS84(reference.wgs84);

        return msg;
    }

    autohyu_msgs::VehicleState UpdateVehicleState(const interface::VehicleState& vehicle_state) {
        autohyu_msgs::VehicleState msg;
        msg.header = UpdateHeader(vehicle_state.header);

        msg.reference   = UpdateReference(vehicle_state.reference);
        msg.pos_type    = static_cast<uint16_t>(vehicle_state.pos_type);
        msg.gnss        = UpdateWGS84(vehicle_state.gnss);
        msg.gnss_stdev  = UpdateWGS84(vehicle_state.gnss_stdev);

        msg.x = vehicle_state.x;
        msg.y = vehicle_state.y;
        msg.z = vehicle_state.z;

        msg.vx = vehicle_state.vx;
        msg.vy = vehicle_state.vy;
        msg.vz = vehicle_state.vz;

        msg.ax = vehicle_state.ax;
        msg.ay = vehicle_state.ay;
        msg.az = vehicle_state.az;

        msg.roll = vehicle_state.roll;
        msg.pitch = vehicle_state.pitch;
        msg.yaw = vehicle_state.yaw;

        msg.roll_stdev = vehicle_state.roll_stdev;
        msg.pitch_stdev = vehicle_state.pitch_stdev;
        msg.yaw_stdev = vehicle_state.yaw_stdev;

        msg.roll_vel = vehicle_state.roll_vel;
        msg.pitch_vel = vehicle_state.pitch_vel;
        msg.yaw_vel = vehicle_state.yaw_vel;

        msg.vehicle_can = UpdateVehicleCAN(vehicle_state.vehicle_can);

        return msg;
    }

} // namespace ros_bridge

#endif  // __ROS_BRIDGE_VEHICLE_STATE__