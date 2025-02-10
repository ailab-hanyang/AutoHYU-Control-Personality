/**
 * @file        personality_time_window_node.hpp
 * @brief       get trajectory of each scene & get total ax ay jx jy's avg & stddev
 * 
 * @authors     sunghoon8585@gmail.com
 * 
 * @date        2024-11-11 created by Seounghoon Park
 * 
 */


#ifndef __PERSONALITY_TIME_WINDOW_NODE_HPP__
#define __PERSONALITY_TIME_WINDOW_NODE_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <string>

// System Header
#include "task_manager.hpp"
#include "ini_parser.h"

// Bridge Header
// #include "ros_bridge_control.hpp"
#include "ros_bridge_vehicle_state.hpp"
#include "ros_bridge_personality.hpp"

// Parameter Header
#include "personality_extraction_config.hpp"

// Algorithm Header
#include "personality_extraction_algorithm/data_aquisition.hpp"

// matplotlib visualization
#define _USE_MATH_DEFINES
#include <cmath>
#include "matplotlibcpp.h"
#include <boost/thread/thread.hpp>

// Namespace
using namespace ros;
using namespace tf;
using namespace std;

using namespace interface;

class PersonalityTimeWindowNode : public TaskManager {
    public:
        // Constructor
        explicit PersonalityTimeWindowNode(std::string node_name, double period);
        // Destructor
        virtual ~PersonalityTimeWindowNode();

        void Run();
        void Publish();
        void ProcessINI();
        void ProcessRosparam(const ros::NodeHandle& nh);


        void PlotScene();
    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        void SetMinMaxAvgStddev();
        
        void UpdateWindow();



        // TODO : seperate layer by algorithm
        std::vector<double> FindFilteredMinMax(
            std::vector<double>& datas, bool is_iqr, bool is_sorting);

        // Callback functions for subscribe variables        
        inline void CallbackVehicleState(const autohyu_msgs::VehicleState::ConstPtr& msg) {
            mutex_vehicle_state_.lock();
            i_vehicle_state_ = ros_bridge::GetVehicleState(*msg);
            b_is_vehicle_state_ = true;
            mutex_vehicle_state_.unlock();
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        std::vector<double> o_plus_lon_acc_buf_;
        std::vector<double> o_minus_lon_acc_buf_;
        std::vector<double> o_lat_acc_buf_;     
        std::vector<double> o_plus_lon_jerk_buf_;    
        std::vector<double> o_minus_lon_jerk_buf_;   
        std::vector<double> o_lat_jerk_buf_; 
        
        std::vector<PersonalityScene> accel_scene_list_;
        std::vector<PersonalityScene> decel_scene_list_;
        std::vector<PersonalityScene> corner_scene_list_;

        double ax_min_;
        double ax_max_;
        double ax_mean_;
        double ax_stddev_;

        double ax_plus_mean_;
        double ax_plus_stddev_;
        double ax_minus_mean_;
        double ax_minus_stddev_;

        double ay_max_;
        double ay_mean_;
        double ay_stddev_;

        double jx_min_;
        double jx_max_;
        double jx_mean_;
        double jx_stddev_;

        double jx_plus_mean_;
        double jx_plus_stddev_;
        double jx_minus_mean_;
        double jx_minus_stddev_;

        double jy_max_;
        double jy_mean_;
        double jy_stddev_;
        
        
        // Subscriber
        ros::Subscriber             s_vehicle_state_;

        // Input
        interface::VehicleState     i_vehicle_state_;

        // Mutex
        std::mutex mutex_vehicle_state_;
        
        // Publisher
        ros::Publisher p_personality_time_window_;
   

        // Output
        interface::PersonalityTimeWindow o_personality_time_window_;

        // Algorithm
        std::unique_ptr<DataAquisition> alg_data_aquisition_;

        // Util and Configuration
        IniParser util_ini_parser_;
        IniExporter util_ini_exporter_;
        PersonalityExtractionConfig cfg_;

        // Flags
        bool b_is_vehicle_state_ = false; 
        bool b_get_scene_ = false;
        
        // Global Variables
        double before_time_stamp_s_ = 0.0;
        int before_acc_event_ = 0;
        int before_dcc_event_ = 0;
        int before_steer_event_ = 0;
        int loop_count_ = 0;       

        
        int plot_i_ = 0;

};

#endif  // __PERSONALITY_EXTRACTION_NODE_HPP__