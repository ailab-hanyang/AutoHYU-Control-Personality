#ifndef __LONGITUDINAL_CONTROL_HPP__
#define __LONGITUDINAL_CONTROL_HPP__
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

// ROS header
#include <ros/ros.h>

// ROS Message Header
#include <autohyu_msgs/Trajectory.h>
#include <autohyu_msgs/VehicleState.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

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
#include <ini_parser.h>
#include "function_control.hpp"
#include "function_objects.hpp"
#include "function_print.hpp"
#include "function_trajectories.hpp"
#include "function_vehicle_state.hpp"

// Library Header
#include "spline.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

// Parameter Header
#include <longitudinal_control_config.hpp>

// Algorithm header
#include <longitudinal_control/longitudinal_control_core/longitudinal_control_interface.hpp>
#include <longitudinal_control/longitudinal_control_core/longitudinal_delayed_dynamic_lmpc/longitudinal_delayed_dynamic_lmpc.hpp>
#include <longitudinal_control/longitudinal_control_core/longitudinal_delayed_kinematic_lmpc/longitudinal_delayed_kinematic_lmpc.hpp>
#include <longitudinal_control/longitudinal_control_core/longitudinal_dynamic_lmpc/longitudinal_dynamic_lmpc.hpp>
#include <longitudinal_control/longitudinal_control_core/longitudinal_pid/longitudinal_pid.hpp>
#include <longitudinal_control/longitudinal_control_core/longitudinal_ax_feedforward/longitudinal_ax_feedforward.hpp>

using namespace std;
using namespace interface;
using namespace longitudinal_controller;

class LongitudinalControl {
    public:
        explicit LongitudinalControl();
        virtual ~LongitudinalControl();

		bool Init();
		void ProcessINI();

    public:
        std::tuple<ControlCommand, ControlTrajectory, PathTrackingInfos> Run(
			const interface::VehicleState& vehicle_state,
			const interface::Trajectory& trajectory);
    
    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
	    // Functions
		void CreateLongitudinalController();
		Trajectory TransformTrajectory(const Trajectory& trajectory, const VehicleState& vehicle_state, 
										const std::string& frame_out = "ego_frame");
		tk::Map GenerateRoadMap(const Trajectory& trajectory);
		ControlTrajectory GenerateTargetSpeedProfile(const Trajectory& trajectory, tk::Map road_map,
										const std::string& frame_out = "ego_frame");
		void CalculateSpeedTrackingErrors(const ControlTrajectory& profile, const VehicleState& vehicle_state);
		float SpeedCompensator(const ControlTrajectory& profile, const VehicleState& vehicle_state, const double& dt);
		void FaultCheck(ControlCommand& control_command);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables     

		std::shared_ptr<LongitudinalControllerInterface> ptr_longitudinal_controller_; 
		PathTrackingInfos speed_tracking_infos_;
		double sum_speed_error_ = 0.0;
		double prev_speed_error_ = 0.0;
		int prev_control_method_ = -1;
		double cte_error_ = 0.0;

		// Inputs
		std::shared_ptr<const VehicleState> i_vehicle_state_;
		std::shared_ptr<const Trajectory> 	i_reference_trajectory_;

		// Environments
        IniParser util_ini_parser_;	

		// Parameter
		LongitudinalControlParams params_;

        // Global Variables
        // ADD GLOBAL VARIABLES HERE

};

#endif // __LONGITUDINAL_CONTROL_HPP__