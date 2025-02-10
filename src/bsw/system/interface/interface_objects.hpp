/**
 * @file        interface_objects.hpp
 * @brief       interface for objects
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2023-07-14 created by Yuseung Na
 *              2024-04-01 updated by Yuseung Na: Refactoring 
 *              2024-04-30 updated by Yuseung Na: Add 2D objects
 */

#ifndef __INTERFACE_OBJECTS_HPP__
#define __INTERFACE_OBJECTS_HPP__
#pragma once

// STD Header
#include <stdint.h>
#include <map>
#include <utility>
#include <vector>

// Interface Header
#include "interface_header.hpp"

namespace interface {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // enum
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //     
    typedef enum {
        UNKNOWN = 0,
        CAR = 1,
        TRUCK = 2,
        MOTORCYCLE = 3,
        PEDESTRIAN = 4,
        BARRIER = 5,
        TRAFFIC_LIGHT = 6,
        TRAFFIC_SIGN = 7,
    } ObjectClass;

    typedef enum {
        UNKNOWN_STATE = 0,
        STATIC = 1,
        DYNAMIC = 2,
    } ObjectDynamicState;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    typedef struct {
        Header header;
        float x;
        float y;
    } Object2DState;

    typedef struct {
        Header header;
        float x;
        float y;
        float z;
        float roll;
        float pitch;
        float yaw;
        float v_x;
        float v_y;
        float v_z;
        float a_x;
        float a_y;
        float a_z;
        float roll_rate;
        float pitch_rate;
        float yaw_rate;
    } Object3DState;

    typedef struct {
        float length;
        float width;
        float height;
    } ObjectDimension;

    typedef struct {
        uint32_t    id;
        float       confidence_score;
        ObjectClass classification;

        ObjectDimension dimension;  
        Object2DState   state;
    } DetectObject2D;

    typedef struct {        
        Header header;
        std::vector<DetectObject2D> object;
    } DetectObjects2D;

    typedef struct {
        uint32_t    id;
        float       confidence_score;
        ObjectClass classification;

        ObjectDimension dimension;        
        Object3DState   state;
    } DetectObject3D;

    typedef struct {        
        Header header;
        std::vector<DetectObject3D> object;
    } DetectObjects3D;

    typedef struct {
        uint32_t            id;
        ObjectClass         classification;
        ObjectDynamicState  dynamic_state;

        ObjectDimension dimension;
        Object3DState   state;
        Object3DState   state_covariance;
    } TrackObject;

    typedef struct {
        Header header;
        std::vector<TrackObject> object;
    } TrackObjects;

    typedef struct {
        double x;
        double y;
    } ReferenceTrajectoryPoint;

    typedef struct {
        std::vector<ReferenceTrajectoryPoint> point;
    } ReferenceTrajectory;

    typedef struct {
        double                      probability;
        std::vector<Object3DState>  state;
    } PredictObjectMultimodal;

    typedef struct {
        uint32_t            id;
        ObjectClass         classification;
        ObjectDynamicState  dynamic_state;
        std::vector<ReferenceTrajectory> reference;
        
        ObjectDimension                 dimension;

        // Multi-modal Prediction
        std::vector<PredictObjectMultimodal> state_multi;

        // Highest Probability
        std::vector<Object3DState>      state;
    } PredictObject;

    typedef struct {
        Header header;
        std::vector<PredictObject> object;
    } PredictObjects;

    typedef struct {
        Header header;

        double dist;

        double s;
        double ds;

        double n;
        double dn;
    } TargetFrenetObject;
} // namespace interface

#endif // __INTERFACE_OBJECTS_HPP__
