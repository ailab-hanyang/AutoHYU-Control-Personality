/**
 * @file        interface_traffic_sign.hpp
 * @brief       interface for traffic sign
 * 
 * @authors     Seongjae Jeong (jsj19990602@gmail.com)
 *              Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-04-24 created by Seongjae Jeong
 *              2024-04-01 updated by Yuseung Na: Refactoring 
 */

#ifndef __INTERFACE_TRAFFIC_SIGN_HPP__
#define __INTERFACE_TRAFFIC_SIGN_HPP__
#pragma once

// STD Header
#include <stdint.h>
#include <map>
#include <utility>
#include <vector>

// Image struct
namespace interface {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // enum
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //    
    typedef enum {
        SPEEDLIMIT  = 0,
        HIGHWAY     = 1,
        SCHOOLZONE  = 2,
        SPEEDBUMP   = 3,
    } Sign;
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //    
    typedef struct {
        std::string frame_id;
        uint32_t    sequence_number;
        double      time_stamp{0.0};
    } TrafficSignHeader;

    typedef struct {
        TrafficSignHeader   traffic_sign_header;
        Sign                sign;
    } TrafficSign;

    
} // namespace interface

#endif // __INTERFACE_TRAFFIC_SIGN_HPP__