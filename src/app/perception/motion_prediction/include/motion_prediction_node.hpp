/**
 * @file        motion_prediction_node.hpp
 * @brief       motion prediction node hpp file to predict surrounding vehicles' motion
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-04-13 created by Yuseung Na
 * 
 */

#ifndef __MOTION_PREDICTION_NODE_HPP__
#define __MOTION_PREDICTION_NODE_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// System Header
#include "task_manager.hpp"
#include "ini_parser.h"

// Bridge Header
#include "ros_bridge_objects.hpp"
#include "ros_bridge_vehicle_state.hpp"
#include "ros_bridge_trajectories.hpp"
#include "ros_bridge_lanelet.hpp"

// Parameter Header
#include "motion_prediction_config.hpp"

// Algorithm Header
#include "preprocess/preprocess_algorithm.hpp"
#include "physics_based/physics_based_algorithm.hpp"

// Namespace
using namespace ros;
using namespace tf;
using namespace std;

class MotionPrediction : public TaskManager {
    public:
        // Constructor
        explicit MotionPrediction(std::string node_name, double period);
        // Destructor
        virtual ~MotionPrediction();

        void Run();
        void Publish();
        void ProcessINI();
        void ProcessRosparam(const ros::NodeHandle& nh);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions

        // Callback functions for subscribe variables
        // inline void CallbackMapBin(const autohyu_msgs::MapBin& msg){
        //     mutex_map_bin_.lock();
        //     i_map_bin_ = ros_bridge::GetLaneletMapBin(msg);
        //     i_map_lanelet_ =  util_function::DecodeLaneletMapBin(i_map_bin_.data);
        //     i_map_seq_ = msg.header.seq;
        //     b_is_map_ = true;
        //     mutex_map_bin_.unlock();
        // }      
        inline void CallbackVehicleState(const autohyu_msgs::VehicleState::ConstPtr& msg) {
            mutex_vehicle_state_.lock();
            i_vehicle_state_ = ros_bridge::GetVehicleState(*msg);
            b_is_vehicle_state_ = true;
            mutex_vehicle_state_.unlock();
        }
        inline void CallbackTrackObjects(const autohyu_msgs::TrackObjects::ConstPtr& msg) {
            mutex_track_objects_.lock();
            i_track_objects_ = ros_bridge::GetTrackObjects(*msg);
            b_is_track_objects_ = true;
            mutex_track_objects_.unlock();
        }
        inline void CallbackTrajectory(const autohyu_msgs::Trajectory::ConstPtr& msg) {
            mutex_trajectory_.lock();
            i_trajectory_ = ros_bridge::GetTrajectory(*msg);
            b_is_trajectory_ = true;
            mutex_trajectory_.unlock();
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        
        // Subscriber
        ros::Subscriber s_vehicle_state_;
        ros::Subscriber s_track_objects_;
        // ros::Subscriber s_map_bin_;
        ros::Subscriber s_trajectory_;
        ros::Subscriber s_goal_point_;

        // Input
        // interface::MapBin           i_map_bin_;
        interface::VehicleState     i_vehicle_state_;
        interface::TrackObjects     i_track_objects_;
        interface::Trajectory       i_trajectory_;
        lanelet::LaneletMapPtr      i_map_lanelet_;
        uint32_t                    i_map_seq_ = 0;

        // Mutex
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_track_objects_;
        // std::mutex mutex_map_bin_;
        std::mutex mutex_goal_point_;
        std::mutex mutex_trajectory_;
        
        // Publisher
        ros::Publisher p_predict_objects_;
        ros::Publisher p_rviz_predict_objects_;
        ros::Publisher p_map_request_;

        // Output
        autohyu_msgs::PredictObjects o_predict_objects_;
        visualization_msgs::MarkerArray o_rviz_predict_objects_;

        // Algorithm
        std::unique_ptr<Preprocess> alg_preprocess_;
        std::unique_ptr<PhysicsBased> alg_physics_based_;


        // Util and Configuration
        IniParser util_ini_parser_;
        MotionPredictionConfig cfg_;

        // Flags
        bool b_is_map_           = false;
        bool b_is_vehicle_state_ = false;
        bool b_is_track_objects_ = false;
        bool b_is_trajectory_    = false;

        // Global Variables
        // ADD GLOBAL VARIABLES HERE

        //** about lanelet
        float request_num_ = 1;
};

#endif  // __MOTION_PREDICTION_NODE_HPP__