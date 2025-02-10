/**
 * @file        interface_lanelet.hpp
 * @brief       interface for binary-lanelet
 * 
 * @authors     Junhee Lee (998jun@gmail.com)      
 *              Yuseung Na (yuseungna@hanyang.ac.kr)
 * 
 * @date        2024-04-12 created by Junhee Lee
 *              2024-06-20 updated by Yuseung Na: add LaneletPath, LaneletRoute
 *  
 */

#ifndef __INTERFACE_LANELET_HPP__
#define __INTERFACE_LANELET_HPP__
#pragma once


// STD Header
#include <stdint.h>
#include <map>
#include <utility>
#include <vector>

// Interface Header
#include "interface_header.hpp"
#include "interface_geometry_msgs.hpp"

namespace interface {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // enum
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //     
    typedef enum {
        LL_NONE = 0,
        LL_SUCCESSOR = 1,
        LL_LEFT = 2,
        LL_RIGHT = 3,
        LL_ADJACENT_LEFT = 4,
        LL_ADJACENT_RIGHT = 5,
        LL_CONFLICTING = 6,
        LL_AREA = 7,
        LL_DIVERGE_STRAIGHT = 8,
        LL_DIVERGE_LEFT = 9,
        LL_DIVERGE_RIGHT = 10,
        LL_MERGE_STRAIGHT = 11,
        LL_MERGE_LEFT = 12,
        LL_MERGE_RIGHT = 13,
    } LaneletRelationType;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //  
    typedef struct {
        Header                  header;
        std::vector<int8_t>     data;
    } MapBin;

    typedef struct{
        Header                  header;
        int64_t                 id_ego;
        std::vector<int64_t>    id_path;
        interface::Point        goal_point;
    } LaneletPath;

    // typedef struct {
    //     Header                  header;
    //     int64_t                 id_ego;
    //     std::vector<int64_t>    id_route;
    //     interface::Point        goal_point;
    // } LaneletRoute;

    typedef struct {
        int64_t id;
        LaneletRelationType type;
    } LaneletRelation;

    typedef struct {
        int64_t id;
        bool is_ego;
        bool is_goal;
        bool is_shortest_path;
        int64_t left_boundary_line_id;
        int64_t right_boundary_line_id;
        std::vector<LaneletRelation> relation;
    } Lanelet;

    typedef struct {
        double x;
        double y;
        double z;
    } LaneletLinePoint;

    typedef struct {
        int64_t id;
        std::vector<LaneletLinePoint> point;
    } LaneletLine;

    typedef struct {
        Header                  header;
        std::vector<Lanelet>    lanelet;
        std::vector<LaneletLine>    left_boundary;
        std::vector<LaneletLine>    right_boundary;
        std::vector<LaneletLine>    center_line;
    } LaneletRoute;
} // namespace interface

#endif // __INTERFACE_LANELET_HPP__
