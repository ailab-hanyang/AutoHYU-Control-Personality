/**
 * @file        vehicle_driver_node.hpp
 * @brief       carla vehicle driver node hpp file
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-07-26 created by Jeonghun Kang
 * 
 */

#ifndef __CARLA_VEHICLE_DRIVER_HPP__
#define __CARLA_VEHICLE_DRIVER_HPP__
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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Quaternion.h>
#include <autohyu_msgs/VehicleState.h>

// System Header
#include <task_manager.hpp>
#include <ini_parser.h>

// Utility header
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_projection/UTM.h>

// Interface Header
#include "interface_vehicle_state.hpp"
#include "interface_constants.hpp"

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
        inline void CallbackCARLAIMU(const sensor_msgs::Imu::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_carla_imu_);
            i_carla_imu_ = GetCARLAIMU(*msg);
            is_initialized = true;
        }
        inline void CallbackCARLAGPS(const sensor_msgs::NavSatFix::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_carla_gps_);            
            i_carla_gps_ = GetCARLAGPS(*msg);
            is_initialized = true;
        }
        inline void CallbackCARLAVehicleStatus(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_carla_vehicle_status_);
            i_carla_vehicle_status_ = GetCARLAVehicleStatus(*msg);
            is_initialized = true;
        }
        inline void CallbackCARLAVehicleOdometry(const nav_msgs::Odometry::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mutex_carla_vehicle_odometry_);
            i_carla_vehicle_odometry_.vx = msg->twist.twist.linear.x;
            i_carla_vehicle_odometry_.vy = msg->twist.twist.linear.y;
            i_carla_vehicle_odometry_.vz = msg->twist.twist.linear.z;
            is_initialized = true;
        }

        Gnss GetCARLAGPS(const sensor_msgs::NavSatFix& msg);
        Motion GetCARLAIMU(const sensor_msgs::Imu& msg);
        VehicleState GetCARLAVehicleStatus(const carla_msgs::CarlaEgoVehicleStatus& msg);
        double GenerateGaussianNoise(double mean, double std_dev);

        // Update functions for publish variables
        void UpdateVehicleState(const Position& position,
                                const Motion& motion, 
                                const VehicleState& carla_vehicle_state);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables     

        // Subscriber
        Subscriber s_carla_gps_;
        Subscriber s_carla_imu_;
        Subscriber s_carla_ego_vehicle_status_;
        Subscriber s_carla_ego_vehicle_odometry_;

        // Publisher
        Publisher p_vehicle_state_;
        Publisher p_vehicle_state_filtered_;
        Publisher p_reference_point_;

        // Inputs
        Motion i_carla_imu_;
        Gnss i_carla_gps_;
        VehicleState i_carla_vehicle_status_;
        VehicleState i_carla_vehicle_odometry_;     // for getting seperated vx, vy, and vz

        mutex mutex_carla_imu_;
        mutex mutex_carla_gps_;
        mutex mutex_carla_vehicle_status_;
        mutex mutex_carla_vehicle_odometry_;

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
        CARLAVehicleDriverConfig cfg_;
        bool is_initialized = false;
};

#endif  // __CARLA_VEHICLE_DRIVER_HPP__