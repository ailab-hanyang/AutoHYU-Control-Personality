/**
 * @file        object_driver_config.hpp
 * @brief       configuration hpp file for carmaker object driver node
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-05-16 created by Jeonghun Kang
 * 
 */

#ifndef __CARMAKER_OBJECT_DRIVER_CONFIG_HPP__
#define __CARMAKER_OBJECT_DRIVER_CONFIG_HPP__
#pragma once

// STD Header
#include <string>

typedef struct {  
    // Node Config
    double roi;     
} CMObjectDriverConfig;

#endif // __CARMAKER_OBJECT_DRIVER_CONFIG_HPP__