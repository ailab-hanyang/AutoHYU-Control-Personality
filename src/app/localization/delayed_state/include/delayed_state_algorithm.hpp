// ****************************************************************************/
// Module:      delayed_state_algorithm.hpp
// Description: delayed_state algorithm
//
// Authors: seounghoon
/****************************************************************************/

#ifndef __DELAYED_STATE_ALGORITHM_HPP__
#define __DELAYED_STATE_ALGORITHM_HPP__
#pragma once

// Standard header
#include <algorithm>
#include <string>
#include <deque>
#include <cmath>
#include <random>
#include <iostream> // TODO : erase

// Interface header
#include <interface_constants.hpp>
#include <interface_vehicle_state.hpp>


// Utility header
#include <ini_parser.h>
#include <delayed_state_params.hpp>

// Defines


using namespace interface;
// Class
class DelayedStateAlgorithm {

public:
    typedef struct {
        double x;
        double y;
    } xy;

    // Constructor
    DelayedStateAlgorithm();
    // Destructor
    ~DelayedStateAlgorithm();

public:
    bool Init(double dt);

public:
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Functions

    VehicleState MakeNoisyDelayedState(
        const std::vector<VehicleState>& vehicle_state_vec,
        const DelayedStateParams& params);
    
protected:
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Variables

    double GenerateGaussianNoise(double mean, double std_dev);

    // std::deque<xy> dq_state_xy_window_;
    std::deque<VehicleState> dq_state_xy_window_;
    VehicleState prev_vehicle_state_;
    double d_dt_sec_;

};

#endif // __DELAYED_STATE_ALGORITHM_HPP__