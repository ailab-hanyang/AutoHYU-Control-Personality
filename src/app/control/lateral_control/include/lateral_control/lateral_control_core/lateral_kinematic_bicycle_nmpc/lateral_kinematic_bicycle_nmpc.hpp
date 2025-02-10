#ifndef __LATERAL_KINEMATIC_BICYCLE_NMPC_HPP__
#define __LATERAL_KINEMATIC_BICYCLE_NMPC_HPP__
#pragma once

// STD header
#include <iostream>
#include <vector>
#include <tuple>
#include <list>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <mutex>
#include <utility>
#include <memory>
// Utility header
#include <ini_parser.h>
#include <spline.h>
#include "matplotlibcpp.h"

// Interface Header
#include "interface_constants.hpp"
#include "interface_trajectories.hpp"
#include "interface_vehicle_state.hpp"
#include "interface_control.hpp"

#include <lateral_control/lateral_control_core/lateral_control_interface.hpp>


// Parameter Header
#include <lateral_control_config.hpp>

// ACADOS Header 
#include <lateral_control/lateral_control_core/lateral_kinematic_bicycle_nmpc/lateral_kinematic_bicycle_nmpc_acados_wrapper.hpp>

using namespace std;
using namespace interface;
using namespace lateral_controller;
// using namespace nmpc_model;
class LateralKinematicBicycleNMPC: public LateralControllerInterface{	
	public:
        explicit LateralKinematicBicycleNMPC();
        virtual ~LateralKinematicBicycleNMPC();

    public:

        void ProcessINI();		
		void BuildControllerVehicleState(const VehicleState& vehicle_state, tk::Map road_map) override;
	   	boost::optional<ControlCommand> CalculateOptimalTireSteering(const ControlTrajectory& ref) override;
		void ComputeOptimalSteeringAngle();

	private:
		typedef std::vector<double> State;
		typedef std::vector<double> Input;
		typedef std::pair<State, Input> OptVariables;

		const string target_frame_ = "ego_frame"; 

        // acados
        std::unique_ptr<LateralKinematicBicycleNMPCAcadosWrapper> solver_;
	    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
	    // Functions
		vector<OptVariables> SolveAcados(const ControlTrajectory& ref);
        void CheckSolvedStateTwisted(const ControlTrajectory &ref, const ControlCommand &control_command);
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
		// Variables
		uint8_t live_cnt_=0;
		bool is_solved_ = false;
		bool is_steering_fault_ = false;

		int 	step_num_ = 0;
		double 	step_dt_ = 0;
		
		vector<State> optimal_states_;

		// Inputs
		VehicleState i_vehicle_state_;
		ControlTrajectory i_reference_trajectory_;
		tk::Map i_road_map_;
		// Outputs
		
        // Environments
        IniParser util_ini_parser_;

        // Configuration parameters
		LateralKinematicBicycleNMPCParams params_;

	

    public:
		inline std::string GetTargetFrame(){
			return target_frame_;
		};
		inline int GetControlTrajectoryStepNum(){
			return step_num_;
		};
		inline double GetControlTrajectoryStepTime(){
			return step_dt_;
		};
};

#endif  // __LATERAL_KINEMATIC_BICYCLE_NMPC_HPP__
