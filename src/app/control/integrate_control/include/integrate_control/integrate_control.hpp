#ifndef __INTEGRATE_CONTROL_HPP__
#define __INTEGRATE_CONTROL_HPP__
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
#include "ini_parser.h"
#include "function_control.hpp"
#include "function_objects.hpp"
#include "function_print.hpp"
#include "function_trajectories.hpp"
#include "function_vehicle_state.hpp"

// Library Header
#include <spline.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

// Parameter Header
#include <integrate_control_config.hpp>

// Algorithm Header
#include <integrate_control_core/integrate_control_interface.hpp>
#include <integrate_control_core/integrate_kinematic_bicycle_nmpc/integrate_kinematic_bicycle_nmpc.hpp>
#include <integrate_control_core/integrate_dynamic_bicycle_nmpc/integrate_dynamic_bicycle_nmpc.hpp>
#include <integrate_control_fallback/integrate_control_fallback.hpp>

using namespace std;
using namespace interface;
using namespace integrate_controller;

class Control{	
	public:
        explicit Control();
        virtual ~Control();

        bool Init();
		void ProcessINI();		

		std::tuple<ControlCommand, ControlTrajectory, PathTrackingInfos> Run (
			const VehicleState& vehicle_state,
			const Trajectory& trajectory);
	   
	private:
	    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
	    // Functions
		void CreateController();
		Trajectory TransformTrajectoryToEgo(const Trajectory& trajectory, const VehicleState& vehicle_state);
		ControlTrajectory TransformTrajectoryToWorld(const ControlTrajectory& trajectory, const VehicleState& vehicle_state);
		tk::Map GenerateRoadMap(const Trajectory& trajectory);
		ControlTrajectory GenerateTargetTrajectory(const Trajectory& trajectory, tk::Map road_map,
										const std::string& frame_out = "ego_frame");
				
		double CompensateUndersteerGradient(const ControlTrajectory& ref);

		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
		// Variables
		std::shared_ptr<ControllerInterface> ptr_integrate_controller_; 
		std::unique_ptr<ControlFallback>     ptr_integrate_fallback_; 

		PathTrackingInfos path_tracking_info_;
		int prev_control_method_ = -1;

		double sum_yaw_rate_error_ = 0.;
		double prev_yaw_rate_error_ = 0.;

		// Inputs
		std::shared_ptr<const VehicleState> i_vehicle_state_;
		std::shared_ptr<const Trajectory> 	i_reference_trajectory_;
		
        // Environments
        IniParser util_ini_parser_;	

		// Parameter
		ControlParams params_;
};

#endif  // __DYNAMIC_MPC_HPP__
