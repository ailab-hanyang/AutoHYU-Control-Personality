/**
 * @file        object_driver_config.hpp
 * @brief       configuration hpp file for carmaker object driver node
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-05-16 created by Jeonghun Kang
 * 
 */

#ifndef __MORAI_OBJECT_DRIVER_CONFIG_HPP__
#define __MORAI_OBJECT_DRIVER_CONFIG_HPP__
#pragma once

// STD Header
#include <string>

typedef struct {  
    // Node Config
    double roi;
    double ref_lat;
    double ref_lon; 
} MRObjectDriverConfig;

#endif // __MORAI_OBJECT_DRIVER_CONFIG_HPP__