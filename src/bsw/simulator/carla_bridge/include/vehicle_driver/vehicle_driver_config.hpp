/**
 * @file        vehicle_driver_config.hpp
 * @brief       configuration hpp file for carla vehicle driver node
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-07-26 created by Jeonghun Kang
 * 
 */

#ifndef __CARLA_VEHICLE_DRIVER_CONFIG_HPP__
#define __CARLA_VEHICLE_DRIVER_CONFIG_HPP__
#pragma once

// STD Header
#include <string>

typedef struct {  
    // Node Config
    double ref_lat;
    double ref_lon;
    std::string role_name;
    std::string projection_mode;
    int     i_sim_vx_vy_time_window;
    int     i_sim_ax_ay_time_window;
    int     i_sim_yaw_vel_time_window;
    int     cnt;
    double  ay_lpf;
    double  vy_lpf;
    double  ay_noise;
    double  vy_noise;
} CARLAVehicleDriverConfig;

#endif // __CARLA_VEHICLE_DRIVER_CONFIG_HPP__