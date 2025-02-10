/**
 * @file        rotue_driver_config.hpp
 * @brief       configuration hpp file for carla route driver node
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-08-01 created by Jeonghun Kang
 * 
 */

#ifndef __CARLA_ROUTE_DRIVER_CONFIG_HPP__
#define __CARLA_ROUTE_DRIVER_CONFIG_HPP__
#pragma once

// STD Header
#include <string>

typedef struct {  
    std::string role_name;
} CARLARouteDriverConfig;

#endif // __CARLA_ROUTE_DRIVER_CONFIG_HPP__