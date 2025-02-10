/**
 * @file        vehicle_control_config.hpp
 * @brief       configuration hpp file for carmaker vehicle control node
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-05-16 created by Jeonghun Kang
 * 
 */

#ifndef __CARMAKER_VEHICLE_CONTROL_CONFIG_HPP__
#define __CARMAKER_VEHICLE_COTNROL_CONFIG_HPP__
#pragma once

// STD Header
#include <string>

typedef struct {  
    // Node Config
    std::string vehicle_state_topic{"/app/loc/vehicle_state"};
    bool use_keyboard_input{false};
} CMVehicleControlConfig;

#endif // __CARMAKER_VEHICLE_COTNROL_CONFIG_HPP__