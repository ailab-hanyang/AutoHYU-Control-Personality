/**
 * Module:      virtual_object_generator_config.hpp
 * Description: config of virtual object generator node
 * 
 * Authors: Kyungwoon So (bigcow1999@gmail.com)
 *          Yuseung Na (ys.na0220@gmail.com) 
 * 
 * Revision History
 *      Sep. 07, 2023: Kyungwoon So - Created
 *      Sep. 21, 2023: Yuseung Na - Modify for real vehicle test
 */

#ifndef __VIRTUAL_OBJECT_GENERATOR_CONFIG_HPP__
#define __VIRTUAL_OBJECT_GENERATOR_CONFIG_HPP__

#pragma once

enum GenerateMode {
    OFF = 0,
    START = 1,
    PAUSE = 2,
    INIT = 3,
};

typedef struct{
    int generate_mode;

    std::vector<std::string> csv_path;
    std::vector<std::string> csv_noise_path;

    double roi_s_front;
    double roi_s_rear;

    double noise_x;
    double noise_y;
    double noise_yaw;
    double noise_v_x;
    double noise_a_x;

    int noise_mode;
    
} VirtualObjectGeneratorParams;


#endif // __VIRTUAL_OBJECT_GENERATOR_CONFIG_HPP__