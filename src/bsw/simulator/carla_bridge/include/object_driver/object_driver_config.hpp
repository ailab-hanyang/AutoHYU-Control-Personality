/**
 * @file        object_driver_config.hpp
 * @brief       configuration hpp file for carla object driver node
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-08-01 created by Jeonghun Kang
 * 
 */

#ifndef __CARLA_OBJECT_DRIVER_CONFIG_HPP__
#define __CARLA_OBJECT_DRIVER_CONFIG_HPP__
#pragma once

// STD Header
#include <string>

typedef struct {  
    // Node Config
    double roi;
    std::string role_name;
    bool use_actor_list_filter{false};
    std::string ego_veicle_type;
    int ego_vehicle_id;
} CARLAObjectDriverConfig;

#endif // __CARLA_OBJECT_DRIVER_CONFIG_HPP__