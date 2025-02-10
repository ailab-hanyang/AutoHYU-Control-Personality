/**
 * @file        grip_manager.hpp
 * @brief       
 * 
 * @authors     Junhee Lee (998jun@gmail.com)         
 * 
 * @date        2023-10-04 created by Junhee Lee
 * 
 */

#ifndef __GRIP_MANAGER_HPP__
#define __GRIP_MANAGER_HPP__
#pragma once

// STD Header
#include <iostream>
#include <vector>
#include <list>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <mutex>
#include <utility>
#include <memory>
#include <chrono>
#include <tuple>
#include <deque>

// Interface Header
#include "interface_constants.hpp"
#include "interface_vehicle_state.hpp"

// Util Function Header
#include "function_print.hpp"



// Parameter Header
#include "grip_manager_config.hpp"

using namespace std;
using namespace interface;

typedef struct {
	double vehicle_side_slip;
    double tire_slip_front;
    double tire_slip_rear;
    double yawrate_measured;
    double yawrate_from_steering;
    double yawrate_max;   
} SlipState;

typedef struct {
	double steering_tire_angle;
} LateralCommand;

typedef struct {
	double torque;
    double ax;
} LongitudinalCommand;

typedef enum {
    YAWRATE_BASED= 0,
    FRONT_TIRE_SLIP_ANGLE_BASED
} SteeringLimiterMode;




class GripManager {
    public:
        explicit GripManager();
        virtual ~GripManager();

    public:
        std::tuple<double, double, double> RunAlgorithm(const VehicleState& vehicle_state,
                                                        const double& target_steering_tire_angle, 
                                                        const double& target_torque,
                                                        const double& target_accel,
                                                        const GripManagerNodeConfig& cfg);
    
    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
	    // Functions
        SlipState CalculateSlipState(const VehicleState& vehicle_state, const double& steering_tire_angle, const GripManagerNodeConfig& cfg);

        void TorqueLimiter(LongitudinalCommand& longi_cmd,
                            const SlipState& slip_state,
                            const VehicleState& vehicle_state,
                            const GripManagerNodeConfig& cfg);

		void SteeringLimiter(LateralCommand& lateral_cmd,
                            const SlipState& slip_state,
                            const VehicleState& vehicle_state,
                            const GripManagerNodeConfig& cfg);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables        

        // Inputs
        std::shared_ptr<const VehicleState> i_vehicle_state_;
		std::shared_ptr<const double> i_target_steering_tire_angle_;
		std::shared_ptr<const double> i_target_torque_;
		std::shared_ptr<const double> i_target_accel_;


        // Global Variables
        // ADD GLOBAL VARIABLES HERE

};

#endif // __TEMPLATE_ALGORITHM_ALGORITHM_HPP__