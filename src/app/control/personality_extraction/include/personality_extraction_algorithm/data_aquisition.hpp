/**
 * @file        data_aquisition.hpp
 * @brief       output : only the change of event count
 * 
 * @authors     sunghoon8585@gmail.com
 * 
 * @date        2024-10-25 created by Seounghoon Park
 *              2024-11-11 edit to make time window
 * 
 */

#ifndef __DATA_AQUISITION_HPP__
#define __DATA_AQUISITION_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <iostream>
#include <algorithm>

// Interface Header
#include "interface_constants.hpp"
#include "interface_vehicle_state.hpp"
#include "interface_personality.hpp"

// Util Function Header

// Parameter Header
#include "personality_extraction_config.hpp"

using namespace interface;

class DataAquisition {
    public:
        explicit DataAquisition();
        virtual ~DataAquisition();

    public:
        int Run(const interface::VehicleState& vehicle_state, const double dt, const PersonalityExtractionConfig& cfg);
        std::tuple<int, int, int> EventCountOut();

        std::vector<double> LonAccBufferOut();
        std::vector<double> LonPlusAccBufferOut();
        std::vector<double> LonMinusAccBufferOut();
        std::vector<double> LatAccBufferOut();
        std::vector<double> LatAbsAccBufferOut();
        std::vector<double> LonJerkBufferOut();
        std::vector<double> LonPlusJerkBufferOut();
        std::vector<double> LonMinusJerkBufferOut();
        std::vector<double> LatJerkBufferOut();
        std::vector<double> LatAbsJerkBufferOut();
        std::vector<PersonalityScenePoint> SceneTrajectoryOut();
        void LonAccJerkBufferClear();
        void LatAccJerkBufferClear();

        double MvAvgOut(const std::vector<double>& data);
        double StdDevOut(const std::vector<double>& data);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
	    // Functions
        void FindAccEvent(const interface::VehicleState& vehicle_state, const double dt, const PersonalityExtractionConfig& cfg);
        void FindDccEvent(const interface::VehicleState& vehicle_state, const double dt, const PersonalityExtractionConfig& cfg);
        void FindSteerEvent(const interface::VehicleState& vehicle_state, const double dt, const PersonalityExtractionConfig& cfg);

        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        // mov avg filter buf for output
        std::vector<double> lon_mov_avg_window_;
        std::vector<double> lat_mov_avg_window_;

        // for checking event
        std::vector<double> vel_buf_for_acc_;      
        double ref_acc_vel_mps_ = 1.0;

        std::vector<double> vel_buf_for_dcc_;      
        double ref_dcc_vel_mps_ = -1.0;

        std::vector<double> curvature_buf_;        
        double ref_curvature_ = 0.005;
        
        

        //// Outputs
        // counts
        bool get_data_ = false;
        int acc_event_count_ = 0;
        int dcc_event_count_ = 0;
        int steer_event_count_ = 0;
        
        // personality extraction input
        std::vector<double> o_lon_acc_buf_;     // ax
        std::vector<double> o_lat_acc_buf_;     // ay
        std::vector<double> o_lon_jerk_buf_;    // jx
        std::vector<double> o_lat_jerk_buf_;    // jy
        std::vector<PersonalityScenePoint> o_scene_points_;


        // Global Variables
        // ADD GLOBAL VARIABLES HERE
        bool b_is_curve_before_ = false;
        bool b_is_accel_before_ = false;
        bool b_is_decel_before_ = false;
        const double cut_off_freq_ = 2.0;
        int event_count_threshold_ = 10;
        // Utils

};

#endif // __SET_CONSTRAINT_HPP__