/**
 * @file        interface_trajectories.hpp
 * @brief       interface for trajectories
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2023-07-14 created by Yuseung Na
 *              2024-04-01 updated by Yuseung Na: Refactoring 
 */

#ifndef __INTERFACE_TRAJECTORIES_HPP__
#define __INTERFACE_TRAJECTORIES_HPP__
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
        LANE_KEEPING = 0,
        LEFT_LANE_CHANGING = 1,
        RIGHT_LANE_CHANGING = 2,
        INLANE_OVERTAKING = 3,
        CAR_FOLLOWING = 4,
        STOP = 5,
    } BehaviorType;

    typedef enum{
        CENTER_LANE  = 0,
        RIGHT_LANE   = 1,
        LEFT_LANE    = 2,
    } LaneType;
    typedef enum{
        SINGLE_LANE  = 0,
        NEXT_RIGHT   = 1,
        NEXT_LEFT    = 2,
        WRONG        = 3,
    } CenterLaneType;

    typedef enum{
        CENTER_LINE  = 0,
        RIGHT_LINE   = 1,
        LEFT_LINE    = 2,
    } LineType;

    typedef enum {
        FORWARD = 0,
        BACKWARD = 1,
    } TrajectoryDirection;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //  

    typedef struct {
        std::vector<Point> point;
    } GoalPoints;
    inline bool operator==(const GoalPoints& a, const GoalPoints& b) {
        if (a.point.size() != b.point.size()) {
            return false;
        }
        for (size_t i = 0; i < a.point.size(); ++i) {
            if (!(a.point[i] == b.point[i])) {
                return false;
            }
        }
        return true; 
    }

    typedef struct {
        double x;
        double y;
        double z;
    } SearchSpaceBoundaryPoint;
    
    typedef struct {
        uint64_t id;        
        std::vector<uint64_t> prev_id;
        std::vector<uint64_t> next_id;
        uint16_t on_ego; // 0: ego, 1: left, 2: right
        bool on_shortest_path;
        double x;
        double y;
        double z;
        double yaw;
        double curvature;
        double speed;
        SearchSpaceBoundaryPoint lb_point;
        SearchSpaceBoundaryPoint rb_point;
    } SearchSpacePoint;

    typedef struct {
        Header       header;
        std::vector<SearchSpacePoint> point;
        std::vector<SearchSpaceBoundaryPoint> left_boundary;
        std::vector<SearchSpaceBoundaryPoint> right_boundary;
    } SearchSpace;

    typedef struct {
        double x;
        double y;
        double z;
        double s;
        double n;
    } SamplingBoundaryLinePoint;

    typedef struct{
        LineType type;
        std::vector<SamplingBoundaryLinePoint> point;
    } SamplingBoundaryLine;

    typedef struct{
        bool is_exist;
        LaneType type;
        SamplingBoundaryLine center;
        SamplingBoundaryLine right;
        SamplingBoundaryLine left;
    } SamplingBoundaryLane;

    typedef struct{
        uint64_t id;
        CenterLaneType center_type;
        std::vector<SamplingBoundaryLane> center;
        SamplingBoundaryLane right;
        SamplingBoundaryLane left;
    }SamplingBoundarySegment;

    typedef struct{
        Header       header;
        std::vector<SamplingBoundarySegment> segment;
    }SamplingBoundary;
    
    typedef struct {
        double x;
        double y;
        double z;
    } RoutePoint;

    typedef struct {
        int64_t id;
        bool is_shortest_path;
        int64_t left_boundary_id;
        int64_t right_boundary_id;
        std::vector<int64_t> prev_id;
        std::vector<int64_t> next_id;

        std::vector<RoutePoint> point;
        std::vector<RoutePoint> left_point;
        std::vector<RoutePoint> right_point;
    } RouteSegment;

    typedef struct {
        uint16_t on_ego; // 0: ego, 1: left, 2: right
        std::vector<RouteSegment> segment;
    } RouteCandidate;

    typedef struct {
        RouteCandidate ego_route;
        RouteCandidate left_route;
        RouteCandidate right_route;
    } RouteCandidates;

    typedef struct {
        double x;
        double y;
        double z;
        double s;
        double n;
    } BehaviorBoundaryLinePoint;

    typedef struct {
        uint64_t id;
        uint16_t on_ego; // 0: ego, 1: left, 2: right
        bool on_shortest_path;
        double time;
        double distance;
        double x;
        double y;
        double z;
        double yaw;
        double curvature;
        double ref_speed;
        double speed;
        double acceleration;
        BehaviorBoundaryLinePoint lb_point;
        BehaviorBoundaryLinePoint rb_point;
    } BehaviorTrajectoryPoint;

    typedef struct {
        Header header;

        double lateral_difference;
        double lateral_distance;
        double lateral_accel;
        double longitudinal_distance;
        double collision;
        double stability;
        double drivable_area;
        bool   is_collision;

        double total;
    } TrajectoryCost;

    typedef struct {
        Header header;
        uint32_t id;
        TrajectoryCost cost;
        std::vector<double> st;
        std::vector<double> vt;
        std::vector<double> at;
        std::vector<BehaviorTrajectoryPoint> point;
        std::vector<BehaviorBoundaryLinePoint> reference_map;
        std::vector<BehaviorBoundaryLinePoint> right_boundary;
        std::vector<BehaviorBoundaryLinePoint> left_boundary;
    } BehaviorTrajectory;

    typedef struct {
        Header header;
        std::vector<BehaviorTrajectory> trajectory;
    } BehaviorTrajectories;
    
    typedef struct {
        float x;
        float y;
        float z;
        float s;
        float n;
    } TrajectoryBoundaryPoint;

    typedef struct {
        std::vector<TrajectoryBoundaryPoint> point;
    } TrajectoryBoundary;

    typedef struct {
        float               time;
        float               distance;
        TrajectoryDirection direction;
        float               x;
        float               y;
        float               z;
        float               yaw;
        float               curvature;
        float               speed;
        float               acceleration;
    } TrajectoryPoint;

    typedef struct {
        Header       header;
        uint16_t     id;
        
        TrajectoryBoundary           left_boundary;
        TrajectoryBoundary           right_boundary;
        std::vector<TrajectoryPoint> point;
    } Trajectory;

    typedef struct {
        Header header;
        std::vector<Trajectory> trajectory;
    } Trajectories;
    
    typedef struct {
        bool on_ego;
        bool collision;
        double x;
        double y;
        double r;
    } CollisionBoundary;
    
    typedef struct{
        Header header;
        std::vector<CollisionBoundary> boundary;
    } CollisionBoundaries;
}

#endif // __INTERFACE_TRAJECTORIES_HPP__
