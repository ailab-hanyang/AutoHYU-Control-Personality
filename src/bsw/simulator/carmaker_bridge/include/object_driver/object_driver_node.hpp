/**
 * @file        object_driver_node.hpp
 * @brief       carmaker object driver node hpp file
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-05-16 created by Jeonghun Kang
 * 
 */
#ifndef __CARMAKER_OBJECT_DRIVER_HPP__
#define __CARMAKER_OBJECT_DRIVER_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// ROS Header
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <rosgraph_msgs/Clock.h>

// ROS Message Header
#include <visualization_msgs/MarkerArray.h>
#include <carmaker_msgs/Objects.h>
#include <autohyu_msgs/DetectObjects3D.h>
#include <autohyu_msgs/TrackObjects.h>
#include <mobileye_msgs/C_RDR.h>

// System Header
#include <task_manager.hpp>
#include <ini_parser.h>

// Utility Header
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_projection/LocalCartesian.h>

// Interface Header
#include "interface_vehicle_state.hpp"
#include "interface_objects.hpp"
#include "interface_trajectories.hpp"
#include "interface_constants.hpp"

// Ros Bridge Header
#include "ros_bridge_vehicle_state.hpp"
#include "ros_bridge_objects.hpp"
#include "ros_bridge_trajectories.hpp"

// Util Function Header
#include "function_trajectories.hpp"

// Parameter Header
#include "object_driver_config.hpp"

// Namespace
using namespace ros;
using namespace tf;
using namespace std;
using namespace interface;

class ObjectDriver : public TaskManager {
    public:
        // Constructor
        explicit ObjectDriver(std::string node_name, double period);
        // Destructor
        virtual ~ObjectDriver();
;
        void Run();
        void Publish();
        void ProcessINI();
        void ProcessRosparam(const ros::NodeHandle& nh);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions

        // Callback functions for subscribe variables
        inline void CallbackVehicleState(const autohyu_msgs::VehicleState::ConstPtr& msg) {
            lock_guard<mutex> lock(mutex_vehicle_state_);
            i_vehicle_state_ = ros_bridge::GetVehicleState(*msg);
            b_is_vehicle_state_ = true;
        }
        inline void CallbackCMObjects(const carmaker_msgs::Objects::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_cm_objects_);
            i_cm_objects_ = *msg;
            // std::cout << "Get object" << std::endl;
        }
        inline void CallbackReference(const autohyu_msgs::Reference::ConstPtr& msg) {
            mutex_ref_origin_.lock();
            i_ref_origin_ = ros_bridge::GetReference(*msg);
            b_is_ref_origin_ = true;
            mutex_ref_origin_.unlock();
        }
        // inline void CallbackGlobalTrajectory(const autohyu_msgs::Trajectory::ConstPtr& msg) {
        //     lock_guard<mutex> lock(mutex_global_trajectory_);
        //     i_global_trajectory_ = ros_bridge::GetTrajectory(*msg);
        //     b_is_global_trajectory_ = true;
        // }

        // Get functions for subscribe variables
        // DetectObjects3D GetObjects(const VehicleState& vehicle_state, const Trajectory& global_trajectory);
        DetectObjects3D GetObjects(const VehicleState& vehicle_state);



        // Update functions for publish variables
        void UpdateObjects(const DetectObjects3D& objects);     
        void UpdateRvizObjects(const DetectObjects3D& objects);
        void UpdateMobileyeCornerRadar(const DetectObjects3D& objects, const VehicleState& vehicle_state);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables     

        // Subscriber
        Subscriber s_vehicle_state_;
        Subscriber s_cm_objects_;
        Subscriber s_ref_origin_;
        // Subscriber s_global_trajectory_;

        // Publisher
        Publisher p_lidar_objects_;
        Publisher p_track_objects_;
        Publisher p_rviz_track_objects_;

        // %add
        Publisher p_mobileye_corner_radar_;

        // Inputs
        VehicleState i_vehicle_state_;
        Reference i_ref_origin_;
        carmaker_msgs::Objects i_cm_objects_;
        // Trajectory i_global_trajectory_;

        mutex mutex_cm_objects_;
        mutex mutex_vehicle_state_;
        mutex mutex_ref_origin_;
        // mutex mutex_global_trajectory_;

        // Outputs
        autohyu_msgs::DetectObjects3D o_lidar_objects_;
        autohyu_msgs::TrackObjects o_tracked_objects_;
        visualization_msgs::MarkerArray o_rviz_objects_;
        mobileye_msgs::C_RDR o_mobileye_corner_radar_;


        // Environments
        IniParser util_ini_parser_;
        std::map<int, TrackObject> v_fusion_objects_;
        
        // Algorithms        
         
        // Configuration parameters
        CMObjectDriverConfig cfg_;
        // bool b_is_global_trajectory_ = false;
        bool b_is_vehicle_state_ = false;
        bool b_is_ref_origin_ = false;
};

#endif  // __CARMAKER_OBJECT_DRIVER_HPP__