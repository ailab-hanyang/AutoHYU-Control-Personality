#ifndef __DELAYED_STATE_PARAMS_HPP__
#define __DELAYED_STATE_PARAMS_HPP__

#include <stdint.h>
#include <string>

typedef struct {    
    double cfg_delay_time_s;
    double cfg_noise_std_dev_x;
    double cfg_noise_std_dev_y;
    double cfg_noise_std_dev_yaw;
    double cfg_noise_std_dev_vx;
    double cfg_noise_std_dev_vy;
    double cfg_noise_std_dev_yaw_vel;
} DelayedStateParams;

#endif // __DELAYED_STATE_PARAMS_HPP__