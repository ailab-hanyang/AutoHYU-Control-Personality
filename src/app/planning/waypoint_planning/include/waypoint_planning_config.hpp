#ifndef __WAYPOINT_PLANNING_CONFIG_HPP__
#define __WAYPOINT_PLANNING_CONFIG_HPP__

#include <stdint.h>
#include <string>

typedef struct {    
    std::string vehicle_state_topic{"/app/loc/vehicle_state"};

    std::string map_file_path;
    double ref_lat;
    double ref_lon;

    int    use_lanelet_speed;
    double max_speed_kph;
    double max_ay_mps2;
    int    use_backward_forward_smoothing;
    double max_ax_mps2;
    double min_ax_mps2;

    double trajectory_time_horizon_sec = 5; 
    double min_search_distance_m = 5;
    double min_roi_distance_m = 50;
    int max_iteration;
    int max_overlap_region = 20;
    int smooth_start_and_stop = 0;
    int lookback_index  = 0;
} WaypointPlanningParams;

#endif // __WAYPOINT_PLANNING_CONFIG_HPP__
