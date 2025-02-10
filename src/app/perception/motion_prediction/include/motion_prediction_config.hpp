/**
 * @file        motion_prediction_config.hpp
 * @brief       configuration hpp file for motion prediction node
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-04-01 created by Yuseung Na
 * 
 */

#ifndef __MOTION_PREDICTION_CONFIG_HPP__
#define __MOTION_PREDICTION_CONFIG_HPP__
#pragma once

// STD Header
#include <string>

typedef struct {  
    // Node Config
    double dt{0.1};
    double prediction_horizon{4.0};
    int prediction_algorithm;
    int track_buffer_size;
    double track_erase_time_sec;
    double track_filter_alpha;
    double yawrate_zero_vel_ms; 

    // Physics prediction model
    std::string physics_model{"CV"};

    // Maneuver prediction model
    std::string longitudinal_model{"CV"};
    std::string lateral_model{"CV"};

    // Planning-based
    int dr_model;
    bool adaptive_k_gain;
    double k_gain;
    double ks_gain;
    double max_k_gain;
    double min_k_gain;
    double max_curvature;
    double min_curvature;
    double max_ax;
    double max_ay;
    double max_wheel_angle_deg;
    double max_delta_wheel_angle_deg;

} MotionPredictionConfig;

#endif // __MOTION_PREDICTION_CONFIG_HPP__