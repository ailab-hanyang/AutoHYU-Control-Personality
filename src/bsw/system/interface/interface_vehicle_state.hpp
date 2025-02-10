/**
 * @file        interface_vehicle_state.hpp
 * @brief       interface for vehicle state
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2023-07-14 created by Yuseung Na
 *              2024-04-01 updated by Yuseung Na: Refactoring 
 */

#ifndef __INTERFACE_VEHICLE_STATE_HPP__
#define __INTERFACE_VEHICLE_STATE_HPP__
#pragma once

// STD Header
#include <stdint.h>
#include <map>
#include <utility>
#include <vector>

// Interface Header
#include "interface_novatel.hpp"
#include "interface_header.hpp"

namespace interface {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // enum
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    typedef enum {
        MANUAL     = 0,
        AUTONOMOUS = 1,
        FAIL       = 2,
    } OperationMode;

    typedef enum {
        READY = 0,
        RUN   = 1,
    } AutonomousMode;

    typedef enum {
        GEAR_P = 0,
        GEAR_R = 7,
        GEAR_N = 6,
        GEAR_D = 5
    } Gear;

    typedef enum {
        MDPS_INIT             = 1,
        MDPS_READY            = 2,
        MDPS_STANDBY          = 3,
        MDPS_ACTIVATION_START = 4,
        MDPS_ACTIVATE         = 5,
        MDPS_ERROR            = 6,
        MDPS_ABORTED          = 7,
    } MdpsState;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    typedef struct {
        double latitude;
        double longitude;
        double altitude;
    } WGS84;

    typedef struct {
        Header header;
        
        double x;
        double y;
        double z;
    } Position;

    typedef struct {
        Header header;

        double latitude;
        double longitude;
        double altitude;
        double heading;
        double latitude_sigma;
        double longitude_sigma;
        double altitude_sigma;
        double heading_sigma;
        double a_sigma;
        double b_sigma;
        int    quality;
        int    number_of_satellites;
        double HDOP;
    } Gnss;

    typedef struct {
        Header header;

        double vx;
        double vy;
        double vz;
        double ax;
        double ay;
        double az;

        double roll;
        double pitch;
        double yaw;
        double roll_vel;
        double pitch_vel;
        double yaw_vel;
    } Motion;

    typedef struct {
        WGS84 wgs84;
        std::string projection;
    } Reference;

    typedef struct {
        Header          header;

        double          lateral_accel;                  // VehicleCan: lateral_acceleration [m/s2]
        double          longitudinal_accel;             // VehicleCan: longitudinal_accel [m/s2]
        double          yaw_rate;                       // VehicleCan: yaw_rate [deg/s]

        double          steering_wheel_angle;           // Vehicle steering wheel angle [rad]
        double          steering_tire_angle;            // Tire direction angle [rad]
        double          steering_speed;                 // Steering wheel rotation speed [rad/s]
        double          steering_torque;                // Steering torque [Nm]
        MdpsState       steering_state;                 // MDPS steering state

        double          wheel_velocity_fl;              // Wheel speed front left [m/s]
        double          wheel_velocity_fr;              // Wheel speed front right [m/s]
        double          wheel_velocity_rl;              // Wheel speed rear left [m/s]
        double          wheel_velocity_rr;              // Wheel speed rear right [m/s]
        double          wheel_velocity_f_avg;           // Average wheel speed front [m/s]
        double          wheel_velocity_r_avg;           // Average wheel speed rear [m/s]
        double          wheel_velocity_avg;             // Average wheel speed [m/s]


        double          motor_torque_f;                 // Motor front torque [Nm]
        double          motor_torque_r;                 // Motor rear torque [Nm]
        double          motor_torque_total;             // Motor torque addition of front and rear  [Nm]

        double          accel_position;                 // Acceleration padal position
        double          brake_pressure;                 // Brake fluid presure [Pa]
        uint8_t         brake_active;                   // Brake active signal
        Gear            gear_select;                    // Gear position P|R|N|D

        OperationMode   operation_mode;                 // Operation mode [manual | AUTONOMOUS | FAIL]
        AutonomousMode  lateral_autonomous_mode;        // Lateral autonomous mode [READY | RUN]
        AutonomousMode  longitudinal_autonomous_mode;   // Longitudinal autonomous mode [READY | RUN]
    } VehicleCAN;

    typedef struct {
        Header                  header;

        // Reference        
        Reference               reference;

        // Novatel type
        PositionOrVelocityType  pos_type;

        // GNSS
        WGS84                   gnss;
        WGS84                   gnss_stdev;
        
        // position and motion
        double x;               // East position in ENU frame [m]
        double y;               // North position in ENU frame [m]
        double z;               // Up position in ENU frame [m]

        double vx;              // Vehicle longitudinal velocity [m/s]
        double vy;              // Vehicle lateral velocity [m/s]
        double vz;              // Vehicle up velocity [m/s]

        double ax;              // Vehicle longitudinal acceleration [m/s2]
        double ay;              // Vehicle lateral acceleration [m/s2]
        double az;              // Vehicle up acceleration [m/s2]

        double roll;            // World to vehicle roll [rad]
        double pitch;           // World to vehicle pitch [rad]
        double yaw;             // World to vehicle yaw [rad]

        double roll_vel;        // Vehicle x axis rotation rate [rad/s]
        double pitch_vel;       // Vehicle y axis rotation rate [rad/s]
        double yaw_vel;         // Vehicle z axis rotation rate [rad/s]

        double roll_stdev;
        double pitch_stdev;
        double yaw_stdev;

        // vehicle can
        VehicleCAN vehicle_can;
        
    } VehicleState;

    typedef struct {
        //double time_stamp{0.0};
        Header header;

        double s;
        double ds;
        double dds;

        double n;
        double dn;
        double ddn;

        double mu;
        double dmu;
        double ddmu;
    } FrenetVehicleState;
} // namespace interface

#endif // __INTERFACE_VEHICLE_STATE_HPP__
