#include <longitudinal_control/longitudinal_control_core/longitudinal_ax_feedforward/longitudinal_ax_feedforward.hpp>

LongitudinalAxFeedForward::LongitudinalAxFeedForward() :LongitudinalControllerInterface() {   
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/control.ini");  
    util_ini_parser_.Init((dir+ini_dir).c_str());

    // Parameters initialization
    ProcessINI();
    std::cout << "init ax feedforward" << std::endl;
}

LongitudinalAxFeedForward::~LongitudinalAxFeedForward() {
}

void LongitudinalAxFeedForward::BuildControllerVehicleState(const VehicleState& vehicle_state, tk::Map road_map){
    std::cout << "Longitudinal ax FeedForward : " << (int)live_cnt_++ << std::endl;

    ProcessINI();
    i_vehicle_state_ = vehicle_state;
    i_road_map_      = road_map;
}

ControlCommand LongitudinalAxFeedForward::CalculateOptimalLongitudinalCommand(const ControlTrajectory& ref){
    ControlCommand control_command;
    if (i_vehicle_state_.vehicle_can.operation_mode != OperationMode::AUTONOMOUS){
        // initialize
        control_command.control_trajectory.control_point.clear();
        control_command.trq = 0.0;
        control_command.ax  = 0.0;
    }
    else{
        int target_idx = min((int)(params_.look_ahead_time/params_.step_dt), params_.step_num );
        double ref_ax    = ref.control_point.at(target_idx).ax;
        
        double control_Fx     = VEHICLE_MASS*ref_ax 
                                + VEHICLE_MASS*GRAVITY_COEFF*sin(-i_vehicle_state_.pitch) 
                                + VEHICLE_MASS*ROLLING_RESISTANCE*GRAVITY_COEFF*cos(-i_vehicle_state_.pitch)
                                + 0.5*AIR_DENSITY*VEHICLE_FRONT_AREA*AIR_DRAG_COEFF*i_vehicle_state_.vx*i_vehicle_state_.vx;
        double control_torque = control_Fx * VEHICLE_WHEEL_RADIUS;

        double current_speed = sqrt(pow((i_vehicle_state_.vx),2) + pow((i_vehicle_state_.vy),2));

        // DR of pid
        control_command.control_trajectory.frame_id = "CG";
        ControlPoint point;
        point.x     = 0.0;
        point.y     = 0.0;
        point.vx    = current_speed;
        control_command.control_trajectory.control_point.push_back(point);    

        control_command.ax  = ref_ax;
        control_command.trq = control_torque;
        cout << "ax" << control_command.ax   << ", trq"<< control_command.trq << endl;
    }
    return control_command;
}

void LongitudinalAxFeedForward::ProcessINI() {
    if (util_ini_parser_.IsFileUpdated()){
        util_ini_parser_.ParseConfig("Ax Feed Forward", "look_ahead_time",    params_.look_ahead_time );
        util_ini_parser_.ParseConfig("Ax Feed Forward", "step_dt",            params_.step_dt         );
        util_ini_parser_.ParseConfig("Ax Feed Forward", "step_num",           params_.step_num        );

        // cout << params_.look_ahead_time << ",\n" <<
        //         params_.step_dt         << ",\n" <<
        //         params_.step_num        << ",\n" <<
        //         params_.vel_kp          << ",\n" <<
        //         params_.vel_ki          << ",\n" <<
        //         params_.vel_kd          << endl;
    }
}