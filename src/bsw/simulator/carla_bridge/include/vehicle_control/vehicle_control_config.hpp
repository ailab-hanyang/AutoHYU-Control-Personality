/**
 * @file        vehicle_control_config.hpp
 * @brief       configuration hpp file for carla vehicle control node
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-08-14 created by Jeonghun Kang
 * 
 */

#ifndef __CARLA_VEHICLE_CONTROL_CONFIG_HPP__
#define __CARLA_VEHICLE_CONTROL_CONFIG_HPP__
#pragma once

// STD Header
#include <string>

typedef struct {  
    // Node Config
    std::string vehicle_state_topic{"/app/loc/vehicle_state"};
    std::string role_name;
    bool use_keyboard_input{false};
    int longitudinal_control_method;
    double p_gain{0.0};
    double i_gain{0.0};
    double d_gain{0.0};
} CARLAVehicleControlConfig;

typedef enum {
    PID     = 0,
    WHEELTORQUE_MAP = 1,
} LongitudinalControlMethod;

#endif // __CARLA_VEHICLE_CONTROL_CONFIG_HPP__