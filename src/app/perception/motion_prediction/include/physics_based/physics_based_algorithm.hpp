/**
 * @file        physics_based_algorithm.hpp
 * @brief       physics based algorithm hpp file for motion prediction
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-04-13 created by Yuseung Na
 * 
 */

#ifndef __PHYSICS_BASED_ALGORITHM_HPP__
#define __PHYSICS_BASED_ALGORITHM_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// Interface Header
#include "interface_objects.hpp"

// Util Function Header
#include "function_objects.hpp"

// Library Header
#include "spline.h"

// Parameter Header
#include "motion_prediction_config.hpp"

class PhysicsBased {
    public:
        explicit PhysicsBased();
        virtual ~PhysicsBased();

    public:
        interface::PredictObjects RunAlgorithm(const interface::PredictObjects& processed_objects,
                                               const MotionPredictionConfig& cfg);
    
    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
	    // Functions
        interface::PredictObjects PredictMotion(const interface::PredictObjects& processed_objects,
                                                const MotionPredictionConfig& cfg);
        interface::Object3DState PredictWithCV(const interface::Object3DState& prev_state,
                                               const MotionPredictionConfig& cfg);
        interface::Object3DState PredictWithCA(const interface::Object3DState& prev_state,
                                               const MotionPredictionConfig& cfg);
        interface::Object3DState PredictWithCTRV(const interface::Object3DState& prev_state,
                                                 const MotionPredictionConfig& cfg);
        interface::Object3DState PredictWithCTRA(const interface::Object3DState& prev_state,
                                                 const MotionPredictionConfig& cfg);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables        

        // Outputs
        interface::PredictObjects o_predict_objects_;

        // Global Variables
        // ADD GLOBAL VARIABLES HERE

};

#endif // __PHYSICS_BASED_ALGORITHM_HPP__