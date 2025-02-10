/**
 * @file        grip_manager.cpp
 * @brief       
 * 
 * @authors     Junhee Lee (998jun@gmail.com)         
 * 
 * @date        2023-10-04 created by Junhee Lee
 * 
 */

#include <grip_manager/grip_manager.hpp>


GripManager::GripManager(){
    
    i_vehicle_state_ = std::make_shared<const VehicleState>();
    i_target_steering_tire_angle_ = std::make_shared<const double>();
    i_target_torque_ = std::make_shared<const double>();
  
}

GripManager::~GripManager(){

}


std::tuple<double, double, double> GripManager::RunAlgorithm(
                            const VehicleState& vehicle_state, 
                            const double& target_steering_tire_angle, 
                            const double& target_torque,
                            const double& target_accel, 
                            const GripManagerNodeConfig& cfg){

    i_vehicle_state_ = std::make_shared<const VehicleState>(vehicle_state);
    i_target_steering_tire_angle_ = std::make_shared<const double>(target_steering_tire_angle);
    i_target_torque_ = std::make_shared<const double>(target_torque);
    i_target_accel_ = std::make_shared<const double>(target_accel);

    LongitudinalCommand target_longitudinal_cmd = {*i_target_torque_, *i_target_accel_};
    LateralCommand target_lateral_cmd = {*i_target_steering_tire_angle_};

    SlipState slip_state = CalculateSlipState(*i_vehicle_state_,*i_target_steering_tire_angle_, cfg);

    if(cfg.use_torque_limiter){
        TorqueLimiter(target_longitudinal_cmd, slip_state, *i_vehicle_state_, cfg);
    }
    if(cfg.use_steering_limiter){
        SteeringLimiter(target_lateral_cmd, slip_state, *i_vehicle_state_, cfg);
    }

    double gripped_steering_tire_angle_deg = target_lateral_cmd.steering_tire_angle*RAD2DEG;
    double gripped_torque = target_longitudinal_cmd.torque;
    double gripped_accel  = target_longitudinal_cmd.ax;

    return {gripped_steering_tire_angle_deg, gripped_torque, gripped_accel};
}



SlipState GripManager::CalculateSlipState(const VehicleState& vehicle_state,
                                            const double& steering_tire_angle,
                                            const GripManagerNodeConfig& cfg){
    SlipState slip_state;
    double v_cg = sqrt(pow(vehicle_state.vx, 2)+pow(vehicle_state.vy, 2));

    double beta = atan2(vehicle_state.vy, vehicle_state.vx);

    // watch out for inf! 
    // yawrate and velocities are measured from rear axle (ego frame)
    double alpha_f = -beta + vehicle_state.vehicle_can.steering_tire_angle
                 -VEHICLE_WHEEL_BASE*vehicle_state.yaw_vel/vehicle_state.vx;
    double alpha_r = -beta
                 +0.0*vehicle_state.yaw_vel/vehicle_state.vx;
    
    double yawrate_from_steering = vehicle_state.vx*steering_tire_angle/
                (VEHICLE_WHEEL_BASE + (VEHICLE_MASS*v_cg*v_cg*
                (REAR_CORNERING_STIFFNESS*VEHICLE_REAR_AXLE_TO_CG-FRONT_CORNERING_STIFFNESS*VEHICLE_WHEEL_BASE)/(2*REAR_CORNERING_STIFFNESS*FRONT_CORNERING_STIFFNESS*VEHICLE_WHEEL_BASE) ));
    
    slip_state.vehicle_side_slip     = beta; 
    slip_state.tire_slip_front       = alpha_f;
    slip_state.tire_slip_rear        = alpha_r;
    slip_state.yawrate_measured      = vehicle_state.yaw_vel;
    slip_state.yawrate_from_steering = yawrate_from_steering;
    slip_state.yawrate_max           = 0.85*cfg.friction_coeff*GRAVITY_COEFF/vehicle_state.vx; // ref : Rajamani yawrate max
    
    return slip_state;
}


void GripManager::TorqueLimiter(LongitudinalCommand& longi_cmd,
                                const SlipState& slip_state,
                                const VehicleState& vehicle_state,
                                const GripManagerNodeConfig& cfg){

    if(fabs(vehicle_state.vx) < 7.0*KPH2MPS )
        return;    

    // Get slip angle
    double tire_slip_ang_deg = fabs(RAD2DEG*slip_state.tire_slip_rear);
    // ROS_INFO_STREAM("vehicle_slip_angle_deg: "<< tire_slip_ang_deg);

    // Slip angle limit condition 3: curr slip angle > slip_start_angle_deg
    if( tire_slip_ang_deg > cfg.tire_slip_rear_start_angle_deg 
            && cfg.use_tire_slip_rear_angle_based_long_acc_limit){
        
        // Limit longitudinal acceleration
        // Limitation modeling
        double slip_angle_area_ratio = (tire_slip_ang_deg - cfg.tire_slip_rear_start_angle_deg)     // 0.5
                            /(cfg.tire_slip_rear_end_angle_deg-cfg.tire_slip_rear_start_angle_deg);  //2.5
        double slip_angle_limit_long_acc = cfg.tire_slip_rear_start_accel + slip_angle_area_ratio * (cfg.tire_slip_rear_end_accel-cfg.tire_slip_rear_start_accel); // -5*0.2
        
        slip_angle_limit_long_acc = max(slip_angle_limit_long_acc, cfg.tire_slip_rear_end_accel);
        
        double long_acc_lim = min(slip_angle_limit_long_acc, longi_cmd.ax); 

        //// Apply compensated throttle command
        // Convert Accelration to torque. 
        double long_torque_lim = (VEHICLE_MASS*
                                    (long_acc_lim
                                    + ROLLING_RESISTANCE*GRAVITY_COEFF*cos(-vehicle_state.pitch)
                                    + GRAVITY_COEFF*sin(-vehicle_state.pitch))
                                    + 0.5*AIR_DENSITY*VEHICLE_FRONT_AREA*AIR_DRAG_COEFF
                                    *vehicle_state.vx*vehicle_state.vx)*VEHICLE_WHEEL_RADIUS;
        cout << "rear" <<  tire_slip_ang_deg << " slip_angle_area_ratio " << slip_angle_area_ratio<< ", " <<longi_cmd.torque << ", " << long_torque_lim<< endl;
        longi_cmd.torque = long_torque_lim;
        longi_cmd.ax = long_acc_lim;
    }

    if(cfg.use_acc_based_long_acc_limit){
        static std::deque<double> vehicle_lat_acc_window = {0.0};    
        double ax = longi_cmd.ax;
        double ay = vehicle_state.ay;
        vehicle_lat_acc_window.push_front(ay);
        // Limit deque length
        while(vehicle_lat_acc_window.size() > cfg.lat_acc_window_size)
            vehicle_lat_acc_window.pop_back();

        // Calculate window average
        double lat_acc_sum = 0.0;
        for(int win_idx = 0; win_idx < vehicle_lat_acc_window.size(); win_idx++)
        {
            lat_acc_sum += vehicle_lat_acc_window[win_idx];
        }
        double lat_acc_filtered = lat_acc_sum / vehicle_lat_acc_window.size();

        //// calculate longitudinal acceleration limit.
        double long_acc_lim_by_friction = cfg.longitudinal_accel_lower_bound;
        
        if(fabs(cfg.maximum_accel) > fabs(lat_acc_filtered)){
            long_acc_lim_by_friction = sqrt(pow(cfg.maximum_accel, 2.0) - pow(lat_acc_filtered,2.0));
        }

        double long_acc_lim = min(long_acc_lim_by_friction, longi_cmd.ax); // 1.0 is minimum longitudinal acceleration limit
        // cout << lat_acc_filtered << " long_acc_lim " << long_acc_lim<< endl;

        //// Apply compensated throttle command
        // Convert Accelration to torque. 
        double long_torque_lim = (VEHICLE_MASS*
                                    (long_acc_lim
                                    + ROLLING_RESISTANCE*GRAVITY_COEFF*cos(-vehicle_state.pitch)
                                    + GRAVITY_COEFF*sin(-vehicle_state.pitch))
                                    + 0.5*AIR_DENSITY*VEHICLE_FRONT_AREA*AIR_DRAG_COEFF
                                    *vehicle_state.vx*vehicle_state.vx)*VEHICLE_WHEEL_RADIUS;
        longi_cmd.torque = long_torque_lim;
        longi_cmd.ax = long_acc_lim;
    }

}

void GripManager::SteeringLimiter(LateralCommand& lateral_cmd,
                                    const SlipState& slip_state,
                                    const VehicleState& vehicle_state,
                                    const GripManagerNodeConfig& cfg){
   
    if(vehicle_state.vx > 5.0){
            
        // Steering limit by front tire slip angle    
        util_function::DebugPrintError("Steer :: Front tire slip angle over");  

        // double alpha_f = -beta + vehicle_state.steering_tire_angle
        //              -VEHICLE_WHEEL_BASE*vehicle_state.yaw_vel/vehicle_state.vx;

        double front_vx = vehicle_state.vx;
        double front_vy = vehicle_state.vy+VEHICLE_WHEEL_BASE*vehicle_state.yaw_vel;

        double front_tire_direction = atan2(front_vy,front_vx);

        double front_steer_angle_upper_bound = front_tire_direction + cfg.max_front_tire_slip_angle*DEG2RAD;
        double front_steer_angle_lower_bound = front_tire_direction - cfg.max_front_tire_slip_angle*DEG2RAD;
        lateral_cmd.steering_tire_angle = min(max(lateral_cmd.steering_tire_angle,front_steer_angle_lower_bound) ,front_steer_angle_upper_bound);

    }
}