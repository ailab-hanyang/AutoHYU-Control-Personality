#ifndef __INTEGRATE_CONTROL_CONFIG_HPP__
#define __INTEGRATE_CONTROL_CONFIG_HPP__
#pragma once

// STD Header
#include <stdint.h>
#include <string>


typedef enum {
    KINEMATIC_BICYCLE_NMPC = 0,
    DYNAMIC_BICYCLE_NMPC,

} ControlMethod;


typedef struct {    
    int     integrate_control_method;
    int     vehicle_state_origin;
    double  reference_time_offset;
    int     use_reference_speed;
    double  time_delay_for_compensators;

    int     use_understeer_gradient;
    double  understeer_gradient_lpf;
    double  understeer_gradient_rate_limit;
    double  understeer_gradient_kv;

    int     use_vehicle_twisting_compensator;
    double  vehicle_twisting_constant;
    double  max_vehicle_twisting_compensation;
    
} ControlParams;



typedef struct {    
    double  step_dt;
    int     shift_input_step;    
    double  weight_x;
    double  weight_y;
    double  weight_yaw;
    double  weight_speed;
    double  weight_steer;
    double  weight_accel;
    double  weight_x_end;
    double  weight_y_end;
    double  weight_yaw_end;
    double  weight_speed_end;
    double  weight_steer_end;
    double  weight_accel_end;
    double  weight_dsteer;
    double  weight_jerk;
    double  avg_distance_error_max;
    double  avg_yaw_error_max;
} IntegrateKinematicBicycleNMPCParams;


typedef struct {    
    double  step_dt;
    int     shift_input_step;    
    double  weight_x;
    double  weight_y;
    double  weight_yaw;
    double  weight_speed;
    double  weight_steer;
    double  weight_accel;
    double  weight_x_end;
    double  weight_y_end;
    double  weight_yaw_end;
    double  weight_speed_end;
    double  weight_steer_end;
    double  weight_accel_end;
    double  weight_dsteer;
    double  weight_jerk;
    double  avg_distance_error_max;
    double  avg_yaw_error_max;
} IntegrateDynamicBicycleNMPCParams;


typedef struct {    
    double  min_look_ahead;
    double  look_ahead_gain;
    double  step_dt;
    int     step_num;
} PurePursuitParams;





#endif // __INTEGRATE_CONTROL_CONFIG_HPP__