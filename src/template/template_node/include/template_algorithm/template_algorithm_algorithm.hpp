/**
 * @file        template_algorithm_algorithm.hpp
 * @brief       template algorithm hpp file for template node algorithm
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-04-01 created by Yuseung Na
 * 
 */

#ifndef __TEMPLATE_ALGORITHM_ALGORITHM_HPP__
#define __TEMPLATE_ALGORITHM_ALGORITHM_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// Interface Header
#include "interface_candb.hpp"
#include "interface_constants.hpp"
#include "interface_control.hpp"
#include "interface_lanelet.hpp"
#include "interface_novatel.hpp"
#include "interface_objects.hpp"
#include "interface_trajectories.hpp"
#include "interface_vehicle_state.hpp"

// Util Function Header
#include "function_control.hpp"
#include "function_objects.hpp"
#include "function_print.hpp"
#include "function_trajectories.hpp"
#include "function_vehicle_state.hpp"

// Library Header
#include "spline.h"

// Parameter Header
#include "template_node_config.hpp"

class TemplateAlgorithm {
    public:
        explicit TemplateAlgorithm();
        virtual ~TemplateAlgorithm();

    public:
        interface::Trajectory RunAlgorithm(const interface::VehicleState& vehicle_state,
                                           const interface::DetectObjects3D& detect_objects,
                                           const interface::TrackObjects& track_objects,
                                           const interface::PredictObjects& predict_objects,
                                           const interface::Trajectory& trajectory,
                                           const TemplateNodeConfig& cfg);
    
    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
	    // Functions

        double DummyAlgorithm1(const interface::VehicleState& vehicle_state,
                               const interface::DetectObjects3D& detect_objects,
                               const TemplateNodeConfig& cfg);
        int    DummyAlgorithm2(const interface::VehicleState& vehicle_state,
                               const interface::TrackObjects& track_objects,
                               const TemplateNodeConfig& cfg);
        interface::Trajectory DummyAlgorithm3(const interface::VehicleState& vehicle_state,
                                              const interface::PredictObjects& predict_objects,
                                              const TemplateNodeConfig& cfg);
        interface::Trajectory DummyAlgorithm4(const interface::VehicleState& vehicle_state,
                                              const interface::Trajectory& trajectory,
                                              const TemplateNodeConfig& cfg);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables        

        // Outputs
        interface::Trajectory o_trajectory_;

        // Global Variables
        // ADD GLOBAL VARIABLES HERE

};

#endif // __TEMPLATE_ALGORITHM_ALGORITHM_HPP__