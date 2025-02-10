/**
 * @file        interface_traffic_light.hpp
 * @brief       interface for traffic light
 * 
 * @authors     Seongjae Jeong (jsj19990602@gmail.com)
 *              Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-04-24 created by Seongjae Jeong
 *              2024-04-01 updated by Yuseung Na: Refactoring 
 */

#ifndef __INTERFACE_TRAFFIC_LIGHT_HPP__
#define __INTERFACE_TRAFFIC_LIGHT_HPP__
#pragma once

// STD Header
#include <stdint.h>
#include <map>
#include <utility>
#include <vector>

// Interface Header
#include "interface_header.hpp"

// Image struct
namespace interface {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // enum
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //    
    typedef enum {
        GREEN       = 0,
        YELLOW      = 1,
        RED         = 2,
        TURN        = 3,
    } Color;

    typedef enum {
        RYG = 0,
        RYGwL = 1,
        RYGwLG = 2,
        YYY = 100
    } MRTrafficLightType;

    typedef enum {
        R = 1,
        Y = 4,
        G = 16,
        G_W_L = 32
    } MRTrafficLightStatus;
    
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    typedef struct {
        Header  header;
        Color   color;
    } TrafficLight;

    typedef struct {
        Header header;
        std::string trafficLightIndex;

        int trafficLightType;
        int trafficLightStatus;
    } MRTrafficLight;

    
} // namespace interface

#endif // __INTERFACE_TRAFFIC_LIGHT_HPP__