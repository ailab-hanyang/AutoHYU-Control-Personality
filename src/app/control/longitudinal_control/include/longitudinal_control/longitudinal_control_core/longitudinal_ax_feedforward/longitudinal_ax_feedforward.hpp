#ifndef __LONGITUDINAL_AX_FF_HPP__
#define __LONGITUDINAL_AX_FF_HPP__
#pragma once

// STD header
#include <iostream>
#include <vector>
#include <list>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <mutex>
#include <utility>

// Utility header
#include <ini_parser.h>
#include <spline.h>

// Interface Header
#include "interface_constants.hpp"
#include "interface_trajectories.hpp"
#include "interface_vehicle_state.hpp"
#include "interface_control.hpp"

#include <longitudinal_control/longitudinal_control_core/longitudinal_control_interface.hpp>

// Parameter Header
#include <longitudinal_control_config.hpp>

using namespace std;
using namespace interface;
using namespace longitudinal_controller;

class LongitudinalAxFeedForward: public LongitudinalControllerInterface{	
	public:
        explicit LongitudinalAxFeedForward();
        virtual ~LongitudinalAxFeedForward();

    public:

        void ProcessINI();		
		void BuildControllerVehicleState(const VehicleState& vehicle_state, tk::Map road_map) override;
	   	ControlCommand CalculateOptimalLongitudinalCommand(const ControlTrajectory& ref) override;


		
	private:
	    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
	    // Functions


		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
		// Variables
		uint8_t live_cnt_=0;

		// Inputs
		VehicleState i_vehicle_state_;
		ControlTrajectory i_reference_trajectory_;
		tk::Map i_road_map_;
		// Outputs
		
        // Environments
        IniParser util_ini_parser_;

        // Configuration parameters
		AxFeedForwardParams params_;



    public:
		inline std::string GetTargetFrame(){
			return "CG";
		};
		inline int GetControlTrajectoryStepNum(){
			return params_.step_num;
		};
		inline double GetControlTrajectoryStepTime(){
			return params_.step_dt;
		};
};

#endif  // __LONGITUDINAL_AX_HPP__
