/**
 * @file        interface_novatel.hpp
 * @brief       interface for novatel
 * 
 * @authors     Jiwon Seok (pauljiwon96@gmail.com)
 *              Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2023-10-05 created by Jiwon Seok
 *              2024-04-01 updated by Yuseung Na: Refactoring 
 */

#ifndef __INTERFACE_NOVATEL_HPP__
#define __INTERFACE_NOVATEL_HPP__
#pragma once

// STD Header
#include <stdint.h>
#include <map>
#include <utility>
#include <vector>

// Interface Header
#include "interface_header.hpp"

// Novatel struct
namespace interface {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // enum
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //    
    typedef enum {
        INS_INACTIVE            = 0,
        INS_ALIGNING            = 1,
        INS_HIGH_VARIANCE       = 2,
        INS_SOLUTION_GOOD       = 3,
        INS_SOLUTION_FREE       = 6,
        INS_ALIGNMENT_COMPLETE  = 7,
        DETERMINING_ORIENTATION = 8,
        WAITING_INITIAL_POS     = 9,
        WAITING_AZIMUTH         = 10,
        INITIALIZING_BIASES     = 11,
        MOTION_DETECT           = 12,
    } InertialSolutionStatus;

    typedef enum {
        NONE                     = 0,
        FIXEDPOS                 = 1,
        FIXEDHEIGHT              = 2,
        DOPPLER_VELOCITY         = 8,
        SINGLE                   = 16,
        PSRDIFF                  = 17,
        WAAS                     = 18,
        PROPAGATED               = 19,
        L1_FLOAT                 = 32,
        NARROW_FLOAT             = 34,
        L1_INT                   = 48,
        WIDE_INT                 = 49,
        NARROW_INT               = 50,
        RTK_DIRECT_INS           = 51,
        INS_SBAS                 = 52,
        INS_PSRSP                = 53,
        INS_PSRDIFF              = 54,
        INS_RTKFLOAT             = 55,
        INS_RTKFIXED             = 56,
        PPP_CONVERGING           = 68,
        PPP                      = 69,
        OPERATIONAL              = 70,
        WARNING                  = 71,
        OUT_OF_BOUNDS            = 72,
        INS_PPP_CONVERGING       = 73,
        INS_PPP                  = 74,
        PPP_BASIC_CONVERGING     = 77,
        PPP_BASIC                = 78,
        INS_PPP_BASIC_CONVERGING = 79,
        INS_PPP_BASIC            = 80,
    } PositionOrVelocityType;

    typedef enum {
        POSITION_UPDATE              = 1,            // 0x00000001
        PHASE_UPDATE                 = 2,            // 0x00000002
        ZERO_VELOCITY_UPDATE         = 4,            // 0x00000004
        WHEEL_SEONSOR_UPDATE         = 8,            // 0x00000008
        ALIGN_UPDATE                 = 16,           // 0x00000010
        EXTERNAL_POSITION_UPDATE     = 32,           // 0x00000020
        INS_SOLUTIN_CONVERGENCE_FLAG = 64,           // 0x00000040
        DOPPLER_UPDATE               = 128,          // 0x00000080
        PSEUDORANGE_UPDATE           = 256,          // 0x00000100
        VELOCITY_UPDATE              = 512,          // 0x00000200
        RESERVED_1                   = 1024,         // 0x00000400
        DEAD_RECONING_UPDATE         = 2048,         // 0x00000800
        PHASE_WIND_UP_UPDATE         = 4096,         // 0x00001000
        COURSE_OVER_GROUND_UPDATE    = 8192,         // 0x00002000
        EXTERNAL_VELOCITY_UPDATE     = 16384,        // 0x00004000
        EXTERNAL_ATTITUDE_UPDATE     = 32768,        // 0x00008000
        EXTERNAL_HEADING_UPDATE      = 65535,        // 0x00010000
        EXTERNAL_HEIGHT_UPDATE       = 131072,       // 0x00020000
        RESERVED_2                   = 262144,       // 0x00040000
        RESERVED_3                   = 524288,       // 0x00080000
        ROVER_POSITION_UPDATE        = 1048576,      // 0x00100000
        ROVER_POSITION_UPDATE_TYPE   = 2097152,      // 0x00200000

        RESERVED_4                   = 4194304,      // 0x00400000
        RESERVED_5                   = 8388608,      // 0x00800000
        TURN_ON_BIASES_ESTIMATED     = 16777216,     // 0x01000000
        ALIGNMENT_DIRECTION_VERIFIED = 33554432,     // 0x02000000
        ALIGNMENT_INDICATION_1       = 67108864,     // 0x04000000
        ALIGNMENT_INDICATION_2       = 134217728,    // 0x08000000
        ALIGNMENT_INDICATION_3       = 268435456,    // 0x10000000
        NVM_SEED_INDICATION_1        = 538870912,    // 0x20000000
        NVM_SEED_INDICATION_2        = 1073741824,   // 0x40000000
        NVM_SEED_INDICATION_3        = 2147483648,   // 0x80000000

        // Alignment indication
        ALIGNMENT_INCOMPLETE_ALIGNMENT = 0,
        ALIGNMENT_STATIC               = 1,
        ALIGNMENT_KINETMATIC           = 2,
        ALIGNMENT_DUAL_ANTENNA         = 3,
        ALIGNMENT_USER_COMMAND         = 4,
        ALIGNMENT_NVM_SEED             = 5,

        // NVM Seed Indication
        NVM_SEED_INACTIVE                          = 0,
        NVM_SEED_STORED_INVALID                    = 1,
        NVM_SEED_PENDING_VALIDATION                = 2,
        NVM_SEED_INJECTED                          = 4,
        NVM_SEEED_DATA_IGNORED                     = 5,
        NVM_SEED_ERROR_MODEL_DATA_INJECTED         = 6,
    } INSExtendedSolutionStatus;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //    
    typedef struct {
        std::string message_name;
        uint16_t    message_id;
        uint8_t     message_type;
        uint32_t    sequence_number;
        uint8_t     time_status;
        uint16_t    gps_week_number;
        uint32_t    gps_week_milliseconds;
    } Oem7Header;

    typedef struct {
        Header      header;
        Oem7Header  nov_header;
        uint32_t    imu_data_count;
        double      pitch_rate;
        double      roll_rate;
        double      yaw_rate;
        double      lateral_acc;
        double      longitudinal_acc;
        double      vertical_acc;
        uint32_t    reserved1;
        uint32_t    reserved2;
    } CORRIMU;

    typedef struct {
        Header                      header;
        Oem7Header                  nov_header;
        InertialSolutionStatus      ins_status;
        PositionOrVelocityType      pos_type;
        double                      latitude;
        double                      longitude;
        double                      height;
        float                       undulation;
        double                      north_velocity;
        double                      east_velocity;
        double                      up_velocity;
        double                      roll;
        double                      pitch;
        double                      azimuth;
        float                       latitude_stdev;
        float                       longitude_stdev;
        float                       height_stdev;
        float                       north_velocity_stdev;
        float                       east_velocity_stdev;
        float                       up_velocity_stdev;
        float                       roll_stdev;
        float                       pitch_stdev;
        float                       azimuth_stdev;
        INSExtendedSolutionStatus   ext_sol_status;
        uint16_t                    time_since_update;
    } INSPVAX;
} // namespace interface

#endif // __INTERFACE_NOVATEL_HPP__