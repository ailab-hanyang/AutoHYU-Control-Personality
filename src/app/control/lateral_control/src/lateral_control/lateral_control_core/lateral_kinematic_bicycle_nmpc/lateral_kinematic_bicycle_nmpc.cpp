#include <lateral_control/lateral_control_core/lateral_kinematic_bicycle_nmpc/lateral_kinematic_bicycle_nmpc.hpp>

LateralKinematicBicycleNMPC::LateralKinematicBicycleNMPC() :LateralControllerInterface() {   
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/control.ini");  
    util_ini_parser_.Init((dir+ini_dir).c_str());

    // Parameters initialization
    ProcessINI();
    std::cout << "init LateralKinematicBicycleNMPC" << std::endl;

    solver_ = std::make_unique<LateralKinematicBicycleNMPCAcadosWrapper>(step_dt_);
    step_num_ = LATERAL_KINEMATIC_BICYCLE_NMPC_N+1;
}

LateralKinematicBicycleNMPC::~LateralKinematicBicycleNMPC() {
}

void LateralKinematicBicycleNMPC::BuildControllerVehicleState(const VehicleState& vehicle_state, tk::Map road_map){
    std::cout << "LateralKinematicBicycleNMPC : " << (int)live_cnt_++ << std::endl;

    ProcessINI();
    i_vehicle_state_ = vehicle_state;
    i_road_map_      = road_map;
}

boost::optional<ControlCommand> LateralKinematicBicycleNMPC::CalculateOptimalTireSteering(const ControlTrajectory& ref){
    ControlCommand control_command;
    if (i_vehicle_state_.vehicle_can.operation_mode != OperationMode::AUTONOMOUS || 
        ( i_vehicle_state_.vehicle_can.lateral_autonomous_mode == AutonomousMode::RUN && 
          i_vehicle_state_.vehicle_can.steering_state == MdpsState::MDPS_ACTIVATION_START && is_steering_fault_==true) ){
        
        // initialize
        control_command.control_trajectory.control_point.clear();
        control_command.steering_tire_angle = i_vehicle_state_.vehicle_can.steering_tire_angle*RAD2DEG;
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
            step_num_ = LATERAL_KINEMATIC_BICYCLE_NMPC_N+1;
        }

        vector<OptVariables> opt_vals = SolveAcados(ref);   

        control_command.control_trajectory.control_point.clear();
        control_command.control_trajectory.frame_id = target_frame_;
        for(int i = 0; i<opt_vals.size() ; i++){
            ControlPoint point = {opt_vals[i].first[0], opt_vals[i].first[1], opt_vals[i].first[2],  0};

            control_command.control_trajectory.control_point.push_back(point);
        }
        if(opt_vals.size() >= 1){
            control_command.steering_tire_angle = opt_vals[params_.shift_input_step].first[3]*RAD2DEG;
        }
        else{
            control_command.steering_tire_angle = 0.0;
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


vector<LateralKinematicBicycleNMPC::OptVariables> LateralKinematicBicycleNMPC::SolveAcados(const ControlTrajectory& ref){

    // set initial state [x, y, yaw, delta]
    State x0;
    if(optimal_states_.empty()){
        x0 = {0.0, 0.0, 0.0, 0.0};
    }
    else{
        x0 = {0.0, 0.0, 0.0, optimal_states_.front()[3]};
    }
    solver_->setInitialState(x0);

    for (int i = 0; i < step_num_-1; ++i) { // Indices: 0 to 38
        // Set input constraints
        std::vector<double> lbu = {-MAX_STEERING_TIRE_SPEED*DEG2RAD};
        std::vector<double> ubu = {+MAX_STEERING_TIRE_SPEED*DEG2RAD};
        solver_->setInputConstraints(i, lbu, ubu);

        // Set state constraints : 1 to 39
        std::vector<double> lbx = {-MAX_STEERING_TIRE_ANGLE*DEG2RAD};
        std::vector<double> ubx = {+MAX_STEERING_TIRE_ANGLE*DEG2RAD};
        solver_->setStateConstraints(i+1, lbx, ubx);
        
        // Set cost weights
        std::vector<double> Q_diag = {params_.weight_x, params_.weight_y, params_.weight_yaw, params_.weight_steer};
        std::vector<double> R_diag = {params_.weight_dsteer};
        solver_->setWeights(i, Q_diag, R_diag);
    }
    // Set terminal cost weights
    std::vector<double> Qe_diag = {params_.weight_x_end, params_.weight_y_end, params_.weight_yaw_end, params_.weight_steer_end};
    solver_->setTerminalWeights(Qe_diag);

    // set parameter, reference and initial guess
    for (int i = 0; i < step_num_; ++i) { // Indices: 0 to 39
        
        // Set solver parameters based on the candidate trajectory's speed
        std::vector<double> param = { ref.control_point.at(i).vx };
        solver_->setParameters(i, param);

        // Define the reference state: [x, y, yaw, yaw_rate]
        std::vector<double> yref = {
            ref.control_point.at(i).x,
            ref.control_point.at(i).y,
            ref.control_point.at(i).yaw,
            0.0// atan2(ref.control_point.at(i).curvature,VEHICLE_WHEEL_BASE) // unstable because of curvature jittering
        };
        solver_->setReference(i, yref);

        // std::cout << i << " " << ref.control_point.at(i).x << " " <<ref.control_point.at(i).y << " "<< ref.control_point.at(i).yaw*RAD2DEG << " " << ref.control_point.at(i).vx << std::endl;

        // Optionally set an initial guess for the solver (using yref)
        solver_->setInitialGuess(i, yref);
    }

    
    // Execute the MPC solver
    int opt_status = solver_->solve();
    // std::cout << solver_->getSolveTime() << std::endl;

    vector<LateralKinematicBicycleNMPC::OptVariables> opt_vals;
    if (opt_status == ACADOS_SUCCESS)
    {
        is_solved_ = true;
        optimal_states_.clear();
        for (int i=1; i<step_num_; ++i) { // Indices: 1 to 39
            LateralKinematicBicycleNMPC::OptVariables opt_var;
            opt_var.first = solver_->getState(i);
            opt_var.second = solver_->getControl(i-1);

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
            LateralKinematicBicycleNMPC::OptVariables opt_var;
            opt_var.first = optimal_states_[i];
            opt_vals.push_back(opt_var);           
        }
        optimal_states_.clear();
    }    
    return opt_vals;
}

void LateralKinematicBicycleNMPC::CheckSolvedStateTwisted(const ControlTrajectory& ref, const ControlCommand& control_command){
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
void LateralKinematicBicycleNMPC::ProcessINI() {
    if (util_ini_parser_.IsFileUpdated()){
        util_ini_parser_.ParseConfig("Lateral Kinematic Bicycle NMPC", "step_dt",               params_.step_dt    );
        util_ini_parser_.ParseConfig("Lateral Kinematic Bicycle NMPC", "shift_input_step",      params_.shift_input_step    );
        util_ini_parser_.ParseConfig("Lateral Kinematic Bicycle NMPC", "weight_x",              params_.weight_x            );
        util_ini_parser_.ParseConfig("Lateral Kinematic Bicycle NMPC", "weight_y",              params_.weight_y            );
        util_ini_parser_.ParseConfig("Lateral Kinematic Bicycle NMPC", "weight_yaw",            params_.weight_yaw          );
        util_ini_parser_.ParseConfig("Lateral Kinematic Bicycle NMPC", "weight_steer",          params_.weight_steer        );
        util_ini_parser_.ParseConfig("Lateral Kinematic Bicycle NMPC", "weight_x_end",          params_.weight_x_end        );
        util_ini_parser_.ParseConfig("Lateral Kinematic Bicycle NMPC", "weight_y_end",          params_.weight_y_end        );
        util_ini_parser_.ParseConfig("Lateral Kinematic Bicycle NMPC", "weight_yaw_end",        params_.weight_yaw_end      );
        util_ini_parser_.ParseConfig("Lateral Kinematic Bicycle NMPC", "weight_steer_end",      params_.weight_steer_end    );
        util_ini_parser_.ParseConfig("Lateral Kinematic Bicycle NMPC", "weight_dsteer",         params_.weight_dsteer       );

        util_ini_parser_.ParseConfig("Lateral Kinematic Bicycle NMPC", "avg_distance_error_max",params_.avg_distance_error_max  );
        util_ini_parser_.ParseConfig("Lateral Kinematic Bicycle NMPC", "avg_yaw_error_max",     params_.avg_yaw_error_max       );

        step_dt_ = params_.step_dt;
        cout << "Update Lateral Kinematic Bicycle NMPC ini" << endl;

    }
}