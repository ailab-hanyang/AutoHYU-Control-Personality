/**
 * @file        grip_manager_node_config.hpp
 * @brief       
 * 
 * @authors     Junhee Lee (998jun@gmail.com)         
 * 
 * @date        2023-10-04 created by Junhee Lee
 * 
 */

#ifndef __GRIP_MANAGER_CONFIG_HPP__
#define __GRIP_MANAGER_CONFIG_HPP__
#pragma once

// STD Header
#include <stdint.h>
#include <string>

typedef struct {  
    double  friction_coeff;
    int     use_steering_limiter;
    double  max_front_tire_slip_angle;

    int     use_torque_limiter;

    int     use_tire_slip_rear_angle_based_long_acc_limit;
    double  tire_slip_rear_start_angle_deg;
    double  tire_slip_rear_end_angle_deg;
    double  tire_slip_rear_start_accel;
    double  tire_slip_rear_end_accel;

    int     use_acc_based_long_acc_limit; 
    double  lat_acc_window_size;
    double  maximum_accel;
    double  longitudinal_accel_lower_bound; 

} GripManagerNodeConfig;

#endif // __GRIP_MANAGER_CONFIG_HPP__