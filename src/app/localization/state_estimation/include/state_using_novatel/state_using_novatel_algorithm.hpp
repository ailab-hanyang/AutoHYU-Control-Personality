/**
 * @file        state_using_novatel_algorithm.hpp
 * @brief       algorithm hpp file forstate using novatel algorithm
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)       
 * 
 * @date        2024-04-01 created by Yuseung Na
 * 
 */

#ifndef __STATE_USING_NOVATEL_ALGORITHM_HPP__
#define __STATE_USING_NOVATEL_ALGORITHM_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// Interface Header
#include "interface_constants.hpp"
#include "interface_novatel.hpp"
#include "interface_vehicle_state.hpp"

// Util Function Header
#include "function_print.hpp"
#include "function_vehicle_state.hpp"

// Library Header
#include "spline.h"
#include "GeographicLib/LocalCartesian.hpp"
#include "GeographicLib/UTMUPS.hpp"
#include "GeographicLib/Geocentric.hpp"

// Parameter Header
#include "state_estimation_config.hpp"

class StateUsingNovatel {
    public:
        explicit StateUsingNovatel();
        virtual ~StateUsingNovatel();

    public:
        interface::VehicleState RunAlgorithm(const interface::INSPVAX& inspvax,
                                             const interface::CORRIMU& corrimu,
                                             const interface::VehicleCAN& vehicle_can,
                                             const StateEstimationConfig& cfg);
    
    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
	    // Functions
        interface::VehicleState MergeInspvaxCorrimu(const interface::INSPVAX& inspvax,
                                                    const interface::CORRIMU& corrimu,
                                                    const StateEstimationConfig& cfg);
        interface::VehicleState MergeNovatelCAN(const interface::VehicleState& vehicle_state,
                                                const interface::VehicleCAN& vehicle_can,
                                                const StateEstimationConfig& cfg);
        
        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables        

        // Outputs
        interface::VehicleState o_vehicle_state_;

        // Global Variables
        // ADD GLOBAL VARIABLES HERE

};

#endif // __STATE_USING_NOVATEL_ALGORITHM_HPP__