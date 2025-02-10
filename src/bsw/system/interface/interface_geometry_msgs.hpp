/**
 * @file        interface_geometry_msgs.hpp
 * @brief       interface for geometry_msgs
 * 
 * @authors     Seheon Ha (seheonha@hanyang.ac.kr)          
 * 
 * @date        2024-04-29 created by Seheon Ha
 */

#ifndef __INTERFACE_GEOMETRY_MSGS_HPP__
#define __INTERFACE_GEOMETRY_MSGS_HPP__
#pragma once

// STD Header
#include <stdint.h>
#include <map>
#include <utility>
#include <vector>

#include "interface_header.hpp"

namespace interface {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // enum
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //     

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    typedef struct {
        double x;
        double y;
        double z;
        
    } Point;
    inline bool operator==(const Point a, const Point b){
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }
    typedef struct {
        double x;
        double y;
        double z;
        double w;
    } Quaternion;

    typedef struct{
        Point position;
        Quaternion orientation;
    } Pose;

    typedef struct {
        Header header;
        Point point;
    } PointStamped;

    typedef struct {
        Header header;
        Pose pose;
    } PoseStamped;

} // namespace interface

#endif // __INTERFACE_GEOMETRY_MSGS_HPP__
