#include <longitudinal_control/longitudinal_control_core/longitudinal_pid/longitudinal_pid.hpp>

LongitudinalPID::LongitudinalPID() :LongitudinalControllerInterface() {   
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/control.ini");  
    util_ini_parser_.Init((dir+ini_dir).c_str());

    // Parameters initialization
    ProcessINI();
    std::cout << "init PID" << std::endl;
}

LongitudinalPID::~LongitudinalPID() {
}

void LongitudinalPID::BuildControllerVehicleState(const VehicleState& vehicle_state, tk::Map road_map){
    std::cout << "PID : " << (int)live_cnt_++ << std::endl;

    ProcessINI();
    i_vehicle_state_ = vehicle_state;
    i_road_map_      = road_map;
}

ControlCommand LongitudinalPID::CalculateOptimalLongitudinalCommand(const ControlTrajectory& ref){
    ControlCommand control_command;
    if (i_vehicle_state_.vehicle_can.operation_mode != OperationMode::AUTONOMOUS){
        // initialize
        control_command.control_trajectory.control_point.clear();
        control_command.trq = 0.0;
        control_command.ax  = 0.0;
    }
    else{
        int target_idx = min((int)(params_.look_ahead_time/params_.step_dt), params_.step_num );
        static double ref_speed = i_vehicle_state_.vx;

        double calculated_ref_speed = ref.control_point.at(target_idx).vx;       
        if(!isnan(calculated_ref_speed)){
            ref_speed = params_.ref_speed_lpf*calculated_ref_speed + (1.0-params_.ref_speed_lpf)*ref_speed;
        }


       
        double current_speed = sqrt(pow((i_vehicle_state_.vx),2) + pow((i_vehicle_state_.vy),2));

        double speed_error = ref_speed - current_speed;
        sum_speed_error_ += speed_error;
        if(!i_vehicle_state_.vehicle_can.longitudinal_autonomous_mode){
            sum_speed_error_ = 0;
        }
        // if(prev_speed_error_ * speed_error < 0){
        //     sum_speed_error_ = 0.0;
        // }
        if(sum_speed_error_ > params_.max_i_error){
            sum_speed_error_ = params_.max_i_error;
        }
        else if(sum_speed_error_ < -params_.max_i_error){
            sum_speed_error_ = -params_.max_i_error;
        }


        prev_speed_error_ = speed_error;

        double velocity_pid_output =  params_.vel_kp * speed_error
                                    + params_.vel_ki * sum_speed_error_
                                    + params_.vel_kd * (speed_error-prev_speed_error_); 
        if(fabs(i_vehicle_state_.vx) < 40*KPH2MPS && speed_error > 0){
            // low speed overshoot compensation
            velocity_pid_output =  1.5 * speed_error; 
        }


        double control_accel  = velocity_pid_output;
        double control_Fx     = VEHICLE_MASS*control_accel 
                                + VEHICLE_MASS*GRAVITY_COEFF*sin(-i_vehicle_state_.pitch) 
                                + VEHICLE_MASS*ROLLING_RESISTANCE*GRAVITY_COEFF*cos(-i_vehicle_state_.pitch)
                                + 0.5*AIR_DENSITY*VEHICLE_FRONT_AREA*AIR_DRAG_COEFF*i_vehicle_state_.vx*i_vehicle_state_.vx;
        double control_torque = control_Fx * VEHICLE_WHEEL_RADIUS;


        // DR of pid
        control_command.control_trajectory.frame_id = "CG";
        ControlPoint point;
        point.x     = 0.0;
        point.y     = 0.0;
        point.vx    = current_speed;
        control_command.control_trajectory.control_point.push_back(point);    

        control_command.ax  = control_accel;
        control_command.trq = control_torque;
    }
    return control_command;
}

void LongitudinalPID::ProcessINI() {
    if (util_ini_parser_.IsFileUpdated()){
        util_ini_parser_.ParseConfig("PID", "look_ahead_time",    params_.look_ahead_time );
        util_ini_parser_.ParseConfig("PID", "step_dt",            params_.step_dt         );
        util_ini_parser_.ParseConfig("PID", "step_num",           params_.step_num        );
        util_ini_parser_.ParseConfig("PID", "vel_kp",             params_.vel_kp          );
        util_ini_parser_.ParseConfig("PID", "vel_ki",             params_.vel_ki          );
        util_ini_parser_.ParseConfig("PID", "vel_kd",             params_.vel_kd          );
        util_ini_parser_.ParseConfig("PID", "max_i_error",        params_.max_i_error     );
        util_ini_parser_.ParseConfig("PID", "ref_speed_lpf",      params_.ref_speed_lpf     );

        // cout << params_.look_ahead_time << ",\n" <<
        //         params_.step_dt         << ",\n" <<
        //         params_.step_num        << ",\n" <<
        //         params_.vel_kp          << ",\n" <<
        //         params_.vel_ki          << ",\n" <<
        //         params_.vel_kd          << endl;
    }
}