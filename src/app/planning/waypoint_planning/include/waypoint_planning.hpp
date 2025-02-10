/**
 * @file        waypoint_planning.hpp
 * @brief       waypoint planning node
 * 
 * @authors     Junhee Lee (998jun@gmail.com)
 *              
 * 
 * @date        2023-06-25 Junhee Lee - Created.
 *              2023-08-08 Junhee Lee - Add function to get Lanelet reference speed
 */

#ifndef __WAYPOINT_PLANNING_HPP__
#define __WAYPOINT_PLANNING_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <boost/filesystem.hpp>

// ROS Header
#include <ros/ros.h>

// ROS Message Header
#include <autohyu_msgs/VehicleState.h>
#include <autohyu_msgs/Trajectories.h>
#include <autohyu_msgs/BehaviorTrajectories.h>
#include <autohyu_msgs/VehicleCmd.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>

// System Header
#include "task_manager.hpp"
#include "ini_parser.h"

// Library Header
#include "spline.h"

// Bridge Header
#include "ros_bridge_control.hpp"
#include "ros_bridge_lanelet.hpp"
#include "ros_bridge_novatel.hpp"
#include "ros_bridge_objects.hpp"
#include "ros_bridge_time.hpp"
#include "ros_bridge_trajectories.hpp"
#include "ros_bridge_vehicle_state.hpp"

// Parameter Header
#include <waypoint_planning_config.hpp>

// Algorithm Header
#include <waypoint_generator/waypoint_generator.hpp>

// Namespace
using namespace ros;
using namespace tf;
using namespace std;

class WaypointPlanning : public TaskManager {
    public:
        // Constructor
        explicit WaypointPlanning(int id, std::string task_node, double period);
        // Destructor
        virtual ~WaypointPlanning();

        void Run();
        void Publish();
        void ProcessINI();
        void ProcessRosparam(const ros::NodeHandle& nh);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        void CheckMapUpdate();

        // Callback functions for subscribe variables        
        inline void CallbackVehicleState(const autohyu_msgs::VehicleState::ConstPtr& msg) {
            mutex_vehicle_state_.lock();
            i_vehicle_state_ = ros_bridge::GetVehicleState(*msg);
            b_is_vehicle_state_ = true;
            mutex_vehicle_state_.unlock();
        }

        // Update functions for publish variables
        void UpdateTrajectory(const Trajectory& optimal_trajectory, const Trajectory& race_trajectory);
        void UpdateRvizTrajectory(const Trajectory& optimal_trajectory, const Trajectory& race_trajectory);
        void UpdateRvizMap(const lanelet::LaneletMapPtr& map );

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        Trajectory prev_trajectory_;
        double d_prev_map_update_time_;
        
        // Subscriber
        ros::Subscriber s_vehicle_state_;

        // Input
        interface::VehicleState i_vehicle_state_;

        // Mutex
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_detect_objects_;
        std::mutex mutex_track_objects_;
        std::mutex mutex_predict_objects_;
        std::mutex mutex_trajectory_;
        
        // Publisher
        ros::Publisher p_trajectory_;
        ros::Publisher p_bhv_trajectory_;
        ros::Publisher p_race_trajectory_;
        ros::Publisher p_rviz_map_;  
        ros::Publisher p_rviz_trajectory_;
        ros::Publisher p_rviz_race_trajectory_;
        // ros::Publisher p_trajectories_;

        // Output
        autohyu_msgs::Trajectory o_trajectory_;
        autohyu_msgs::Trajectory o_race_trajectory_;
        autohyu_msgs::BehaviorTrajectory o_bhv_trajectory_;
        visualization_msgs::MarkerArray o_rviz_trajectory_;
        visualization_msgs::MarkerArray o_rviz_race_trajectory_;
        visualization_msgs::MarkerArray o_rviz_map_;

        // Algorithm
        unique_ptr<WaypointGenerator> ptr_waypoint_generator_;

        // Util and Configuration
        IniParser util_ini_parser_;
        WaypointPlanningParams params_;

        // Flags
        double time_stamp_;
        bool b_is_vehicle_state_ = false;        
        bool b_is_map_update_ = false;

        // Global Variables
        // ADD GLOBAL VARIABLES HERE

};

#endif  // __TEMPLATE_NODE_NODE_HPP__