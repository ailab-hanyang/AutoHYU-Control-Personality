#include <integrate_control/integrate_control_core/integrate_kinematic_bicycle_nmpc/integrate_kinematic_bicycle_nmpc.hpp>

IntegrateKinematicBicycleNMPC::IntegrateKinematicBicycleNMPC() :ControllerInterface() {   
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/control.ini");  
    util_ini_parser_.Init((dir+ini_dir).c_str());

    // Parameters initialization
    ProcessINI();
    std::cout << "init IntegrateKinematicBicycleNMPC" << std::endl;

    solver_ = std::make_unique<IntegrateKinematicBicycleNMPCAcadosWrapper>(step_dt_);
    step_num_ = INTEGRATE_KINEMATIC_BICYCLE_NMPC_N+1;
}

IntegrateKinematicBicycleNMPC::~IntegrateKinematicBicycleNMPC() {
}

void IntegrateKinematicBicycleNMPC::BuildControllerVehicleState(const VehicleState& vehicle_state, tk::Map road_map){
    // std::cout << "IntegrateKinematicBicycleNMPC : " << (int)live_cnt_++ << std::endl;

    ProcessINI();
    i_vehicle_state_ = vehicle_state;
    i_road_map_      = road_map;
}

boost::optional<ControlCommand> IntegrateKinematicBicycleNMPC::CalculateOptimalCommand(const ControlTrajectory& ref){
    ControlCommand control_command;
    if (i_vehicle_state_.vehicle_can.operation_mode != OperationMode::AUTONOMOUS || 
        ( i_vehicle_state_.vehicle_can.lateral_autonomous_mode == AutonomousMode::RUN && 
          i_vehicle_state_.vehicle_can.steering_state == MdpsState::MDPS_ACTIVATION_START && is_steering_fault_==true) ){
        
        // initialize
        control_command.control_trajectory.control_point.clear();
        control_command.steering_tire_angle = i_vehicle_state_.vehicle_can.steering_tire_angle*RAD2DEG;
        control_command.ax = i_vehicle_state_.ax;
        control_command.trq = control_command.ax * VEHICLE_MASS * VEHICLE_WHEEL_RADIUS;
        optimal_states_.clear();
        is_steering_fault_ = false;
    }
    else{
        // Verify that the trajectory has enough points for the solver
        if (static_cast<int>(ref.control_point.size()) < step_num_) {
            // Return a boost::none if not enough points are available
            std::cout << "reference has not enough points for the solver" << std::endl;
            return boost::none;
        }
        // acados solver can not update time step online 
        if(step_dt_ != solver_->getTimeStep()){
            std::cout << "step dt does not match. Reinitialize acados " << step_dt_ << " != " << solver_->getTimeStep() << std::endl;
            solver_->initializeSolver(step_dt_);
            step_num_ = INTEGRATE_KINEMATIC_BICYCLE_NMPC_N+1;
        }

        vector<OptVariables> opt_vals = SolveAcados(ref);   

        control_command.control_trajectory.control_point.clear();
        control_command.control_trajectory.frame_id = target_frame_;
        for(int i = 0; i<opt_vals.size() ; i++){
            ControlPoint point ; 
            point.x = opt_vals[i].first[0];
            point.y = opt_vals[i].first[1];
            point.yaw = opt_vals[i].first[2];
            point.vx  = opt_vals[i].first[3];
            point.steer  = opt_vals[i].first[4];
            point.ax = opt_vals[i].first[5];

            if(i < opt_vals.size() -1 ){
                point.dsteer = opt_vals[i].second[0];
                point.jerkx = opt_vals[i].second[1];
            }

            control_command.control_trajectory.control_point.push_back(point);
        }
        if(opt_vals.size() >= 1){
            control_command.steering_tire_angle = opt_vals[1+params_.shift_input_step].first[4]*RAD2DEG;
            control_command.ax = opt_vals[1+params_.shift_input_step].first[5];
            control_command.trq = control_command.ax * VEHICLE_MASS * VEHICLE_WHEEL_RADIUS;
        }
        else{
            control_command.steering_tire_angle = 0.0;
            control_command.ax = -1.0;
            control_command.trq = control_command.ax * VEHICLE_MASS * VEHICLE_WHEEL_RADIUS;
        }
        CheckSolvedStateTwisted(ref, control_command);

    }

    if(is_solved_ == true){
        return control_command;
    }
    else{
        return boost::none;
    }
}


vector<IntegrateKinematicBicycleNMPC::OptVariables> IntegrateKinematicBicycleNMPC::SolveAcados(const ControlTrajectory& ref){

    // set initial state [x, y, yaw, v, delta, a]
    State x0;
    if(optimal_states_.empty()){
        x0 = {0.0, 0.0, 0.0, i_vehicle_state_.vx, i_vehicle_state_.vehicle_can.steering_tire_angle, i_vehicle_state_.ax};
    }
    else{
        x0 = {0.0, 0.0, 0.0, i_vehicle_state_.vx, optimal_states_.at(1)[4], optimal_states_.at(1)[5]};
    }
   
    solver_->setInitialState(x0);

    for (int i = 0; i < step_num_-1; ++i) { // Indices: 0 to 38
        // Set input constraints
        std::vector<double> lbu = {-MAX_STEERING_TIRE_SPEED*DEG2RAD, -500};
        std::vector<double> ubu = {+MAX_STEERING_TIRE_SPEED*DEG2RAD, +500};
        solver_->setInputConstraints(i, lbu, ubu);

        // Set state constraints : 1 to 39
        std::vector<double> lbx = {-MAX_STEERING_TIRE_ANGLE*DEG2RAD};
        std::vector<double> ubx = {+MAX_STEERING_TIRE_ANGLE*DEG2RAD};
        solver_->setStateConstraints(i+1, lbx, ubx);
    }

    // set parameter, reference and initial guess
    for (int i = 0; i < step_num_; ++i) { // Indices: 0 to 39
                
        // Set solver parameters based on the candidate trajectory's speed
        std::vector<double> param = { ref.control_point.at(i).x,
            ref.control_point.at(i).y,
            ref.control_point.at(i).yaw,
            ref.control_point.at(i).vx,
            params_.weight_x, params_.weight_y, params_.weight_yaw, params_.weight_speed, params_.weight_steer, params_.weight_accel,
            params_.weight_dsteer,params_.weight_jerk };
        solver_->setParameters(i, param);

        std::vector<double> x_guess = {
                    ref.control_point.at(i).x,
                    ref.control_point.at(i).y,
                    ref.control_point.at(i).yaw,
                    ref.control_point.at(i).vx,
                    0,// atan2(ref.control_point.at(i).curvature,VEHICLE_WHEEL_BASE) // unstable because of curvature jittering
                    0
                };
        
        solver_->setInitialGuess(i, x_guess);
    }

    
    // Execute the MPC solver
    int opt_status = solver_->solve();
    // std::cout << solver_->getSolveTime() << std::endl;

    vector<IntegrateKinematicBicycleNMPC::OptVariables> opt_vals;
    if (opt_status == ACADOS_SUCCESS)
    {
        is_solved_ = true;
        optimal_states_.clear();
        for (int i=0; i<step_num_; ++i) { // Indices: 1 to 39
            IntegrateKinematicBicycleNMPC::OptVariables opt_var;
            opt_var.first = solver_->getState(i);
            if(i < step_num_-1){
                opt_var.second = solver_->getControl(i);
            }
            
            
            opt_vals.push_back(opt_var);           
            optimal_states_.push_back(opt_var.first);
        }
    }
    else
    {
        std::cout << "CAN'T SOLVE: ";
        switch (opt_status)
        {
            case ACADOS_NAN_DETECTED:
                std::cout << "NaN detected." << std::endl;
                break;
            case ACADOS_MAXITER:
                std::cout << "Maximum iterations reached." << std::endl;
                break;
            case ACADOS_MINSTEP:
                std::cout << "Minimum step size reached." << std::endl;
                break;
            case ACADOS_QP_FAILURE:
                std::cout << "QP solver failure." << std::endl;
                break;
            case ACADOS_READY:
                std::cout << "Ready status." << std::endl;
                break;
            case ACADOS_UNBOUNDED:
                std::cout << "Problem is unbounded." << std::endl;
                break;
            default:
                std::cout << "Unknown error." << std::endl;
                break;
        }

        for (int i=0; i<step_num_; ++i) {
            IntegrateKinematicBicycleNMPC::OptVariables opt_var;
            opt_var.first = optimal_states_[i];
            opt_vals.push_back(opt_var);           
        }
        optimal_states_.clear();
    }    
    
    return opt_vals;
}

void IntegrateKinematicBicycleNMPC::CheckSolvedStateTwisted(const ControlTrajectory& ref, const ControlCommand& control_command){
    // Calculate errors
    // 1. distance error
    // 2. yaw error
    double sum_distance_error = 0.0;
    double sum_yaw_error      = 0.0;

    for(int i = 0; i < control_command.control_trajectory.control_point.size(); i++){
        double distance_error = sqrt(pow(ref.control_point.at(i).x-control_command.control_trajectory.control_point.at(i).x,2)
                                    +pow(ref.control_point.at(i).y-control_command.control_trajectory.control_point.at(i).y,2));
        double yaw_error      = ref.control_point.at(i).yaw-control_command.control_trajectory.control_point.at(i).yaw;

        sum_distance_error += distance_error;
        sum_yaw_error += yaw_error*RAD2DEG;
    }

    double avg_distance_error = fabs(sum_distance_error/control_command.control_trajectory.control_point.size());
    double avg_yaw_error = fabs(sum_yaw_error/control_command.control_trajectory.control_point.size());

    // cout << "dist_e " <<  sum_distance_error/control_command.control_trajectory.control_point.size()  << ",\t yaw_e:" << sum_yaw_error/control_command.control_trajectory.control_point.size() << endl;
    
    if(avg_distance_error > params_.avg_distance_error_max && avg_yaw_error > params_.avg_yaw_error_max){
        std::cout << "solve twisted!!!!!!!" << std::endl;
        is_solved_ = false;
        optimal_states_.clear();
    }

    static double prev_timestamp = i_vehicle_state_.header.stamp;
    double curr_timestamp = i_vehicle_state_.header.stamp;

    if(curr_timestamp < prev_timestamp){
        std::cout << "timestamp backward!!!!!!!" << std::endl;
        is_solved_ = false;
        optimal_states_.clear();
    }

    prev_timestamp = curr_timestamp;

}
void IntegrateKinematicBicycleNMPC::ProcessINI() {
    if (util_ini_parser_.IsFileUpdated()){
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "step_dt",               params_.step_dt             );
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "shift_input_step",      params_.shift_input_step    );
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "weight_x",              params_.weight_x            );
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "weight_y",              params_.weight_y            );
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "weight_yaw",            params_.weight_yaw          );
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "weight_speed",          params_.weight_speed        );
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "weight_steer",          params_.weight_steer        );
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "weight_accel",          params_.weight_accel        );
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "weight_x_end",          params_.weight_x_end        );
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "weight_y_end",          params_.weight_y_end        );
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "weight_yaw_end",        params_.weight_yaw_end      );
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "weight_speed_end",      params_.weight_speed_end    );
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "weight_steer_end",      params_.weight_steer_end    );
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "weight_accel_end",      params_.weight_accel_end    );
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "weight_dsteer",         params_.weight_dsteer       );
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "weight_jerk",           params_.weight_jerk         );

        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "avg_distance_error_max",params_.avg_distance_error_max  );
        util_ini_parser_.ParseConfig("Integrate Kinematic Bicycle NMPC", "avg_yaw_error_max",     params_.avg_yaw_error_max       );

        step_dt_ = params_.step_dt;
        cout << "Update Integrate Kinematic Bicycle NMPC ini" << endl;

    }
}