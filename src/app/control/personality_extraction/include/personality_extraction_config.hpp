/**
 */

#ifndef __PERSONALITY_EXTRACTION_CONFIG_HPP__
#define __PERSONALITY_EXTRACTION_CONFIG_HPP__
#pragma once

// STD Header
#include <string>

typedef struct {  
    // changed from ini parser
    int     auto_stop   = 0;
    int     get_data    = 0;
    
    double  min_ax      = 0.0;
    double  max_ax      = 0.0;
    double  min_jx      = 0.0;
    double  max_jx      = 0.0;
    
    double  max_ay      = 0.0;
    double  max_jy      = 0.0;
    

    unsigned int max_scene_queue_size = 0;
    double scene_size_in_sec = 0.0;
    double ax_mean = 0.0;
    double ax_stddev = 0.0;    
    double ay_mean = 0.0;
    double ay_stddev = 0.0;
    double jx_mean = 0.0;
    double jx_stddev = 0.0;
    double jy_mean = 0.0;
    double jy_stddev = 0.0;

    // changed from rosparam
    double  loop_rate   = 0.0;
    std::string user_name = "";
} PersonalityExtractionConfig;

#endif 