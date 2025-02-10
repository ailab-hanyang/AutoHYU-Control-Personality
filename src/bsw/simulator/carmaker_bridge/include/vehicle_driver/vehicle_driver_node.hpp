/**
 * @file        vehicle_driver_node.hpp
 * @brief       carmaker vehicle driver node hpp file
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-05-16 created by Jeonghun Kang
 * 
 */

#ifndef __CARMAKER_VEHICLE_DRIVER_HPP__
#define __CARMAKER_VEHICLE_DRIVER_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <random>

// ROS Header
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// ROS Message Header
#include <carmaker_msgs/GNSS.h>
#include <carmaker_msgs/UAQ_Out.h>
#include <autohyu_msgs/VehicleState.h>

// System Header
#include <task_manager.hpp>
#include <ini_parser.h>

// Utility Header
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_projection/UTM.h>

// Interface Header
#include "interface_vehicle_state.hpp"

// Ros Bridge Header
#include "ros_bridge_vehicle_state.hpp"
#include "ros_bridge_time.hpp"

// Parameter Header
#include "vehicle_driver_config.hpp"

// Namespace
using namespace ros;
using namespace tf;
using namespace std;
using namespace interface;

class VehicleDriver : public TaskManager {
    public:
        // Constructor
        explicit VehicleDriver(std::string node_name, double period);
        // Destructor
        virtual ~VehicleDriver();

        void Run();
        void Publish();
        void ProcessINI();
        void ProcessRosparam(const ros::NodeHandle& nh);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions

        // Callback functions for subscribe variables
        inline void CallbackCMUAQOut(const carmaker_msgs::UAQ_Out::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_cm_uaq_out_);
            i_cm_uaq_out_ = GetCMUAQOut(*msg);
            is_initialized = true;
        }
        inline void CallbackCMGNSS(const carmaker_msgs::GNSS::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_cm_gnss_);            
            i_cm_gnss_ = GetCMGNSS(*msg);
        }

        VehicleState GetCMUAQOut(const carmaker_msgs::UAQ_Out& msg);
        Gnss GetCMGNSS(const carmaker_msgs::GNSS& msg);
        double GenerateGaussianNoise(double mean, double std_dev);

        // Update functions for publish variables
        void UpdateVehicleState(const Position& position, 
                                const VehicleState& cm_vehicle_state);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables     

        // Subscriber
        Subscriber s_cm_gnss_;
        Subscriber s_cm_vehicle_state_;

        // Publisher
        Publisher p_vehicle_state_;
        Publisher p_vehicle_state_filtered_;
        Publisher p_reference_point_;

        // Inputs
        Gnss i_cm_gnss_;
        VehicleState i_cm_uaq_out_;

        mutex mutex_cm_uaq_out_;
        mutex mutex_cm_gnss_;

        // Outputs
        autohyu_msgs::VehicleState o_vehicle_state_;
        autohyu_msgs::VehicleState o_vehicle_state_filtered_;
        autohyu_msgs::Reference    o_reference_point_;

        // Environments
        IniParser util_ini_parser_;
        TransformListener tf_listener_;
        lanelet::GPSPoint v_gps_point_;
        
        // Algorithms
        std::deque<double> deq_d_sim_ax_time_window_;
        std::deque<double> deq_d_sim_ay_time_window_;
        std::deque<double> deq_d_sim_vx_time_window_;
        std::deque<double> deq_d_sim_vy_time_window_;
        std::deque<double> deq_d_sim_yaw_vel_time_window_;

        // Configuration parameters
        bool is_initialized = false;
        CMVehicleDriverConfig cfg_;
};

#endif  // __CARMAKER_VEHICLE_DRIVER_HPP__