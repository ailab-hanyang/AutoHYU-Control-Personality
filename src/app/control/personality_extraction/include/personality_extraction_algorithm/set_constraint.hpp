/**
 * 
 */

#ifndef __SET_CONSTRAINT_HPP__
#define __SET_CONSTRAINT_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <iostream>
#include <algorithm>

// Interface Header
// #include "interface_candb.hpp"
// #include "interface_constants.hpp"
// #include "interface_control.hpp"
#include "interface_vehicle_state.hpp"

// Util Function Header
// #include "function_control.hpp"
#include "ini_parser.h"

// Library Header
// #include "spline.h"

// Parameter Header
#include "personality_extraction_config.hpp"

class SetConstraint {
    public:
        explicit SetConstraint();
        virtual ~SetConstraint();

    public:
        void Run(const interface::VehicleState& vehicle_state, const double dt, const PersonalityExtractionConfig& cfg);
        void RunByGettingBuffer(const std::vector<double>& lon_acc_buf,
                                const std::vector<double>& lat_acc_buf,
                                const std::vector<double>& lon_jerk_buf,
                                const std::vector<double>& lat_jerk_buf,
                                const PersonalityExtractionConfig& cfg);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
	    // Functions
        void FindAccJerkMax(const interface::VehicleState& vehicle_state, const double dt, const PersonalityExtractionConfig& cfg);
        void FindAccJerkMax(const std::vector<double>& lon_acc_buf,
                            const std::vector<double>& lat_acc_buf,
                            const std::vector<double>& lon_jerk_buf,
                            const std::vector<double>& lat_jerk_buf,
                            const PersonalityExtractionConfig& cfg);

        std::vector<double> FindFilteredMinMax(std::vector<double>& datas, bool is_iqr, bool is_sorting = false);

        void ProcessINI();


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        
        // for longitudinal constraints
        std::vector<double> longitudinal_minus_accel_buf_;
        std::vector<double> longitudinal_plus_accel_buf_;
        std::vector<double> longitudinal_minus_jerk_buf_;
        std::vector<double> longitudinal_plus_jerk_buf_;
        double min_longitudinal_accel_;
        double max_longitudinal_accel_;
        double min_longitudinal_jerk_;
        double max_longitudinal_jerk_;
        double prev_longitudinal_accel_ = 0.0;

        // for lateral constraints
        std::vector<double> lateral_accel_buf_;
        std::vector<double> lateral_jerk_buf_;
        double max_lateral_accel_;
        double max_lateral_jerk_;
        double prev_lateral_accel_ = 0.0;

        // Outputs
        // interface::Trajectory o_trajectory_;

        // Global Variables
        // ADD GLOBAL VARIABLES HERE

        // Utils
        IniExporter util_ini_exporter_;
        bool b_is_config_export_ = true;

};

#endif // __SET_CONSTRAINT_HPP__