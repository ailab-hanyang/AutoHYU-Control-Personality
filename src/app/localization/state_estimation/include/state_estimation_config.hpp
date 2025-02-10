/**
 * @file        state_estimation_config.hpp
 * @brief       configuration hpp file for state estimation node
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)
 * 
 * @date        2024-04-11 created by Yuseung Na
 * 
 */

#ifndef __STATE_ESTIMATION_CONFIG_HPP__
#define __STATE_ESTIMATION_CONFIG_HPP__
#pragma once

// STD Header
#include <string>

typedef struct {  
    // Node Config
    std::string mode{"real"};           // real, carla, morai, carmaker, ...
    std::string location{"kcity"};      // kcity, katri, grandpark, ...
    double reference_latitude{0.0};
    double reference_longitude{0.0};

    // Algorithm Config
    std::string projector{"local_cartesian"}; // local_cartesian, utm

    // 

} StateEstimationConfig;

#endif // __STATE_ESTIMATION_CONFIG_HPP__