/**
 * @file        interface_personality.hpp
 * @brief       interface for personality bayesopt
 * 
 * @authors     Seounghoon Park(sunghoon8585@gmail.com)          
 * 
 * @date        2024-11-19 created by Seounghoon Park
 * 
 */

#ifndef __INTERFACE_PERSONALITY_HPP__
#define __INTERFACE_PERSONALITY_HPP__
#pragma once

// STD Header
#include <stdint.h>
#include <utility>
#include <vector>

// Interface Header
#include "interface_header.hpp"

// Novatel struct
namespace interface {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // enum
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //    
    typedef enum{
        ACCEL_SCENE = 0,
        DECEL_SCENE,
        CORNER_SCENE
    } PersonalitySceneType;

    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //    
    typedef struct {
        double x;
        double y;
        double yaw;
        double vel_mps;
        double delta;       // steering tire angle  [rad]
        double ax;          // acceleration         [m/s^2]
        double ay;          // lateral acceleration [m/s^2]
        double jx;          // jerk                 [m/s^3]
        double jy;          // lateral jerk         [m/s^3]
    } PersonalityScenePoint;

    typedef struct {
        unsigned int scene_type;    // maybe deprecated?
        std::vector<PersonalityScenePoint> scene_points;
    } PersonalityScene;

    typedef struct {
        double avg;
        double std_dev;
        double min = -9999.99;
        double max;
    } Stats;

    typedef struct {
        Stats ax;
        Stats ax_plus;
        Stats ax_minus;
        Stats ay;
        Stats jx;
        Stats jx_plus;
        Stats jx_minus;
        Stats jy;

        std::vector<PersonalityScene> accel_scenes;
        std::vector<PersonalityScene> decel_scenes;
        std::vector<PersonalityScene> corner_scenes;
    } PersonalityTimeWindow;

} // namespace interface

#endif // __INTERFACE_NOVATEL_HPP__