#ifndef __LONGITUDINAL_CONTROL_CONFIG_HPP__
#define __LONGITUDINAL_CONTROL_CONFIG_HPP__

#include <stdint.h>
#include <string>

typedef enum {
    DELAYED_DYNAMIC_LMPC= 0,
    DYNAMIC_LMPC,
    PID,
    AX_FEEDFORWARD,
    DELAYED_KINEMATIC_LMPC
} LongitudinalControlMethod;

typedef struct {    
    int     longitudinal_control_method;
    double  reference_time_offset;
    int     use_user_speed;
    double  user_speed;
    double  zero_speed;
    int     use_speed_compensator;
    double  look_ahead_time;
    double  ref_speed_lpf;
    double  vel_kp;
    double  vel_ki;
    double  vel_kd;
    double  max_i_error;

    int use_cte_decceleration;
    double decceleration_min_cte;
    double decceleration_min_accel;
    double decceleration_max_cte;
    double decceleration_max_accel;

} LongitudinalControlParams;

typedef struct {    
    double  step_dt;
    int     step_num;
    int     shift_input_step;   
    double  time_constant;
    double  max_dF; 
    double  weight_damping_ratio;
    double  weight_v;
    double  weight_F;
    double  weight_v_end;
    double  weight_F_end;
    double  weight_dF;
} LongitudinalDelayedDynamicLMPCParams;

typedef struct {    
    double  step_dt;
    int     step_num;
    int     shift_input_step;   
    double  time_constant;
    double  max_jerk; 
    double  weight_damping_ratio;
    double  weight_v;
    double  weight_a;
    double  weight_v_end;
    double  weight_a_end;
    double  weight_jerk;
}LongitudinalDelayedKinematicLMPCParams;

typedef struct {    
    double  step_dt;
    int     step_num;
    int     shift_input_step;   
    double  max_dF; 
    double  weight_damping_ratio;
    double  weight_v;
    double  weight_F;
    double  weight_v_end;
    double  weight_F_end;
    double  weight_dF;
} LongitudinalDynamicLMPCParams;

typedef struct {    
    double  look_ahead_time;
    double  step_dt;
    int     step_num;
    double  vel_kp;
    double  vel_ki;
    double  vel_kd;
    double  max_i_error;
    double  ref_speed_lpf;

} PIDParams;

typedef struct {    
    double  look_ahead_time;
    double  step_dt;
    int     step_num;
    double  ref_speed_lpf;
} AxFeedForwardParams;
#endif // __LONGITUDINAL_CONTROL_CONFIG_HPP__