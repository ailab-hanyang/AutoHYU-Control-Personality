#include <lateral_control/lateral_control_core/lateral_dynamic_error_lmpc/lateral_dynamic_error_lmpc.hpp>

LateralDynamicErrorLMPC::LateralDynamicErrorLMPC() :LateralControllerInterface() {   
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/control.ini");  
    util_ini_parser_.Init((dir+ini_dir).c_str());

    // Parameters initialization
    ProcessINI();
    std::cout << "init LateralDynamicErrorLMPC" << std::endl;
}

LateralDynamicErrorLMPC::~LateralDynamicErrorLMPC() {
}



void LateralDynamicErrorLMPC::BuildControllerVehicleState(const VehicleState& vehicle_state, tk::Map road_map){
    std::cout << "LateralDynamicErrorLMPC : " << (int)live_cnt_++ << std::endl;

    ProcessINI();
    i_vehicle_state_ = vehicle_state;
    i_road_map_      = road_map;
    
    FrenetVehicleState vehicle_frenet_state;
    std::vector<double> sn = road_map.ToFrenet(0., 0.); // because it is vehicle frame
    std::vector<double> dsn = road_map.ToFrenetVelocity(i_vehicle_state_.vx, i_vehicle_state_.vy, 0.);
    double mu = -road_map.GetSlope(sn[0]);
     
    vehicle_frenet_state.s     = sn[0];   
    vehicle_frenet_state.ds    = dsn[0];   
    vehicle_frenet_state.n     = sn[1];   
    vehicle_frenet_state.dn    = dsn[1];   
    vehicle_frenet_state.mu    = mu;   
    vehicle_frenet_state.dmu   = (mu - i_vehicle_frenet_state_.mu); // ignore step_dt_

    i_vehicle_frenet_state_ = vehicle_frenet_state;

}

boost::optional<ControlCommand> LateralDynamicErrorLMPC::CalculateOptimalTireSteering(const ControlTrajectory& ref){
    ControlCommand control_command;
    if (i_vehicle_state_.vehicle_can.operation_mode != OperationMode::AUTONOMOUS){
        // initialize
        control_command.control_trajectory.control_point.clear();
        control_command.steering_tire_angle = i_vehicle_state_.vehicle_can.steering_tire_angle*RAD2DEG;
    }
    else{
        vector<State> operation_point = GenerateOperationPoints(ref); 
        vector<OptVariables> opt_vals = SolveHPIPM(operation_point,ref);   

        control_command.control_trajectory.control_point.clear();
        control_command.control_trajectory.frame_id = target_frame_;
        for(int i = 0; i<opt_vals.size() ; i++){
            vector<double> xy = i_road_map_.ToCartesian( i_vehicle_frenet_state_.s + ref.control_point[i+1].distance, opt_vals[i].first(0));
            ControlPoint point = {xy[0], xy[1], 0,  0};
            control_command.control_trajectory.control_point.push_back(point);
        }
        if(opt_vals.size() >= 1){
            control_command.steering_tire_angle = opt_vals[params_.shift_input_step].first(4)*RAD2DEG;
        }
        else{
            control_command.steering_tire_angle = 0.0;
        }

    }
    return control_command;
}

vector<LateralDynamicErrorLMPC::State> LateralDynamicErrorLMPC::GenerateOperationPoints(const ControlTrajectory& ref){
    vector<LateralDynamicErrorLMPC::State> operation_points;

    if (operation_inputs_.empty()){
        std::cout<<"empty operation input"<<std::endl;
        LateralDynamicErrorLMPC::Input operate_input;
        for(uint8_t i=0; i<step_num_+1; i++){
            operate_input.setZero();     
            operation_inputs_.push_back(operate_input);
        } 
    }    

    operation_points.push_back(GetInitState(optimal_states_.empty()));
    for(uint8_t i=0; i<step_num_+1; i++) 
    {
        // MODEL VARIANT
        double ref_v = (double)(ref.control_point[i].vx );
        double ref_k = (double)(ref.control_point[i].curvature );
        State curr_state = operation_points.back();
        Input input = operation_inputs_[i];
        State state_derive = GetContinuousModelDeriv(curr_state , input, ref_v, ref_k);
        State next_state = curr_state + state_derive * step_dt_;
        operation_points.push_back(next_state);
    }
    
    return operation_points;
}

LateralDynamicErrorLMPC::State LateralDynamicErrorLMPC::GetReferecneState(const ControlTrajectory& ref, const uint32_t& idx){
    State x_ref;
    const double Kv = VEHICLE_REAR_AXLE_TO_CG * VEHICLE_MASS / (2 * FRONT_CORNERING_STIFFNESS * VEHICLE_WHEEL_BASE) 
                      - VEHICLE_FRONT_AXLE_TO_CG * VEHICLE_MASS / (2 * REAR_CORNERING_STIFFNESS * VEHICLE_WHEEL_BASE);
    const double u_ref = VEHICLE_WHEEL_BASE * ref.control_point[idx].curvature 
                         + Kv * ref.control_point[idx].vx * ref.control_point[idx].vx * ref.control_point[idx].curvature ;
    
    x_ref <<    0.0,
                0.0,
                0.0,
                0.0,
                u_ref;
    return x_ref;
}


LateralDynamicErrorLMPC::State LateralDynamicErrorLMPC::GetInitState(bool is_prev_state_empty){
    State x0;
    if(is_prev_state_empty){
        x0 <<   i_vehicle_frenet_state_.n,
                i_vehicle_frenet_state_.dn,
                i_vehicle_frenet_state_.mu,
                0.0,
                0.0;
    }
    else{
        x0 <<   i_vehicle_frenet_state_.n,
                i_vehicle_frenet_state_.dn,
                i_vehicle_frenet_state_.mu,
                0.0,
                optimal_states_.front()(4);
    }
    return x0;
};

LateralDynamicErrorLMPC::State LateralDynamicErrorLMPC::GetContinuousModelDeriv(const State &x,const Input &u, const double &v, const double &curvature)
{
    // MODEL VARIANT
    const double cf = FRONT_CORNERING_STIFFNESS;
    const double cr = REAR_CORNERING_STIFFNESS;
    const double lf  = VEHICLE_FRONT_AXLE_TO_CG;
    const double lr  = VEHICLE_REAR_AXLE_TO_CG;
    const double mass = VEHICLE_MASS;
    const double iz  = VEHICLE_MOMENT_OF_INERTIA;
    const double vel   = std::max(v, 0.01);
    const double kappa = curvature;

    // State : n, dn, mu, dmu, steer
    State f; 
    f(0) = x(1);
    f(1) = (x(2)*(cf+cr))/mass-kappa*vel*(vel+(cf*lf-cr*lr)/(mass*vel))+(cf*x(4))/mass-(x(1)*(cf+cr))/(mass*vel)-(x(3)*(cf*lf-cr*lr))/(mass*vel);
    f(2) = x(3);
    f(3) =-(kappa*(cf*(lf*lf)+cr*(lr*lr)))/iz+(x(2)*(cf*lf-cr*lr))/iz+(cf*x(4)*lf)/iz-(x(3)*(cf*(lf*lf)+cr*(lr*lr)))/(iz*vel)-(x(1)*(cf*lf-cr*lr))/(iz*vel);
    f(4) = u(0);
    return f;
}

LateralDynamicErrorLMPC::LinModelMatrix LateralDynamicErrorLMPC::GetDiscreteLinModelMatrix(const State &x, const Input &u, const double &v, const double &curvature){
    double const cf = FRONT_CORNERING_STIFFNESS;
    double const cr = REAR_CORNERING_STIFFNESS;
    double const lf = VEHICLE_FRONT_AXLE_TO_CG;
    double const lr = VEHICLE_REAR_AXLE_TO_CG;
    double const mass = VEHICLE_MASS;
    double const iz  = VEHICLE_MOMENT_OF_INERTIA;
    const double vel = std::max(v, 0.01);
    const double kappa = curvature;


    /*
    * x[k+1] = a_d*x[k] + b_d*u + w_d
    */
    A_MPC a_d = A_MPC::Zero();
    a_d(0, 1) = 1.0;
    a_d(1, 1) = -(cf + cr) / (mass * vel);
    a_d(1, 2) = (cf + cr) / mass;
    a_d(1, 3) = (lr * cr - lf * cf) / (mass * vel);
    a_d(1, 4) = cf/mass;
    a_d(2, 3) = 1.0;
    a_d(3, 1) = (lr * cr - lf * cf) / (iz * vel);
    a_d(3, 2) = (lf * cf - lr * cr) / iz;
    a_d(3, 3) = -(lf * lf * cf + lr * lr * cr) / (iz * vel);
    a_d(3, 4) = lf * cf / iz;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim_state_, dim_state_);
    Eigen::MatrixXd a_d_inverse = (I - step_dt_ * 0.5 * a_d).inverse();

    a_d = a_d_inverse * (I + step_dt_ * 0.5 * a_d);  // bilinear discretization

    B_MPC b_d = B_MPC::Zero();
    b_d(4, 0) = 1.0;

    C_MPC c_d = C_MPC::Zero();
    c_d(0, 0) = 0.0;
    c_d(1, 0) = (lr * cr - lf * cf) / (mass * vel) - vel;
    c_d(2, 0) = 0.0;
    c_d(3, 0) = -(lf * lf * cf + lr * lr * cr) / (iz * vel);
    c_d(4, 0) = 0.0;

    b_d = (a_d_inverse * step_dt_) * b_d;
    c_d = (a_d_inverse * step_dt_ * kappa * vel) * c_d;

   return {a_d,b_d,c_d};
}


vector<LateralDynamicErrorLMPC::OptVariables> LateralDynamicErrorLMPC::SolveHPIPM(const vector<State>& operation_points, const ControlTrajectory& ref){
    
    std::vector<hpipm::OcpQp> qp(step_num_+2);
    
    Eigen::Matrix<double,dim_state_,dim_state_> Q;
    Eigen::Matrix<double,dim_state_,dim_state_> Qf;
    Eigen::Matrix<double,dim_input_,dim_input_> R;
    Eigen::Matrix<double,dim_state_,dim_input_> S;

    // cost
    // MODEL VARIANT //
    Q.setZero();  Q.diagonal()  << params_.weight_n,     params_.weight_dn,    params_.weight_mu,     params_.weight_dmu,     params_.weight_steer;
    Qf.setZero(); Qf.diagonal() << params_.weight_n_end, params_.weight_dn_end,params_.weight_mu_end, params_.weight_dmu_end, params_.weight_steer_end;
    S.setZero();
    R.setZero();  R.diagonal()  << params_.weight_dsteer;

    Eigen::VectorXd x_ref(dim_state_);
    const Eigen::VectorXd r = Eigen::VectorXd::Zero(dim_input_);
    for (int i=0; i<step_num_+1; ++i) {
        x_ref = GetReferecneState(ref,i); 

        qp[i].Q = Q.transpose();
        qp[i].R = R.transpose();
        qp[i].S = S.transpose();
        qp[i].q = (- Q * x_ref).transpose();
        qp[i].r = r;
    }
    {
        x_ref = GetReferecneState(ref,step_num_+1); 
        // put final cost
        qp[step_num_+1].Q = Qf.transpose();
        qp[step_num_+1].q = (- Qf * x_ref).transpose();

    }
    

    // constraints
    // dynamics
    for (int i=0; i<step_num_+1; ++i) {
        auto [a_d, b_d, c_d] = GetDiscreteLinModelMatrix(operation_points.at(i),operation_inputs_.at(i),
                                            (double)ref.control_point[i].vx,(double)ref.control_point[i].curvature);
        qp[i].A = a_d;
        qp[i].B = b_d;
        qp[i].b = c_d;
    }

    // state constraint
    for (int i=1; i<=step_num_+1; ++i) {
        // n dn mu dmu delta
        qp[i].idxbx = {4};
        qp[i].lbx = (Eigen::VectorXd(1) << -MAX_STEERING_WHEEL_ANGLE*DEG2RAD/STEERING_RATIO).finished();
        qp[i].ubx = (Eigen::VectorXd(1) <<  MAX_STEERING_WHEEL_ANGLE*DEG2RAD/STEERING_RATIO).finished();
    }

    //input constraint
    for (int i=0; i<step_num_+1; ++i) {
        qp[i].idxbu = {0};
        qp[i].lbu = (Eigen::VectorXd(1) << -MAX_STEERING_WHEEL_SPEED*DEG2RAD/STEERING_RATIO).finished();
        qp[i].ubu = (Eigen::VectorXd(1) <<  MAX_STEERING_WHEEL_SPEED*DEG2RAD/STEERING_RATIO).finished();
    }
    ////////////////////

    hpipm::OcpQpIpmSolverSettings solver_settings;
    solver_settings.mode = hpipm::HpipmMode::Robust;
    solver_settings.iter_max = 100;
    solver_settings.alpha_min = 1e-8;
    solver_settings.mu0 = 1e2;
    solver_settings.tol_stat = 1e-04;
    solver_settings.tol_eq = 1e-04;
    solver_settings.tol_ineq = 1e-04;
    solver_settings.tol_comp = 1e-04;
    solver_settings.reg_prim = 1e-12;
    solver_settings.warm_start = 1;
    solver_settings.pred_corr = 1;
    solver_settings.ric_alg = 0;
    solver_settings.split_step = 1;

    std::vector<hpipm::OcpQpSolution> solution(step_num_+2);
    hpipm::OcpQpIpmSolver solver(qp, solver_settings);
    
    Eigen::VectorXd x = Eigen::VectorXd::Zero(dim_state_);
    for (int i=0; i<step_num_+1; ++i) {
        constexpr double u0 = 0.0;
        solution[i].x = x;
        solution[i].u.resize(dim_input_);
        solution[i].u.fill(u0);
    }
    solution[step_num_+1].x = x;

    Eigen::VectorXd x0 = GetInitState(optimal_states_.empty());

    auto opt_sol = solver.solve(x0, qp, solution);
   
    vector<LateralDynamicErrorLMPC::OptVariables> opt_vals;
    if(opt_sol != hpipm::HpipmStatus::Success){
        std::cout << "!!!!!!!!!!!!!!!! CAN'T SOLVE !!!!!!!!!!!!!!!!"<< std::endl;
        for (int i=0; i<step_num_+1; ++i) {
            LateralDynamicErrorLMPC::OptVariables opt_var;
            opt_var.first = optimal_states_[i];
            opt_var.second = operation_inputs_[i];

            opt_vals.push_back(opt_var);           
        }
        optimal_states_.clear();
        operation_inputs_.clear();
        return opt_vals;
    }
    else{
        optimal_states_.clear();
        operation_inputs_.clear();
        for (int i=1; i<step_num_+2; ++i) {
            LateralDynamicErrorLMPC::OptVariables opt_var;
            opt_var.first = solution[i].x;
            opt_var.second = solution[i-1].u;

            opt_vals.push_back(opt_var);           
            optimal_states_.push_back(opt_var.first);
            operation_inputs_.push_back(opt_var.second);
        }
    }
    
    return opt_vals;
}

void LateralDynamicErrorLMPC::ProcessINI() {
    if (util_ini_parser_.IsFileUpdated()){
        util_ini_parser_.ParseConfig("Lateral Dynamic Error LMPC", "step_dt",               params_.step_dt             );
        util_ini_parser_.ParseConfig("Lateral Dynamic Error LMPC", "step_num",              params_.step_num            );
        util_ini_parser_.ParseConfig("Lateral Dynamic Error LMPC", "shift_input_step",      params_.shift_input_step    );
        util_ini_parser_.ParseConfig("Lateral Dynamic Error LMPC", "weight_n",              params_.weight_n            );
        util_ini_parser_.ParseConfig("Lateral Dynamic Error LMPC", "weight_dn",             params_.weight_dn           );
        util_ini_parser_.ParseConfig("Lateral Dynamic Error LMPC", "weight_mu",             params_.weight_mu           );
        util_ini_parser_.ParseConfig("Lateral Dynamic Error LMPC", "weight_dmu",            params_.weight_dmu          );
        util_ini_parser_.ParseConfig("Lateral Dynamic Error LMPC", "weight_steer",          params_.weight_steer        );
        util_ini_parser_.ParseConfig("Lateral Dynamic Error LMPC", "weight_n_end",          params_.weight_n_end        );
        util_ini_parser_.ParseConfig("Lateral Dynamic Error LMPC", "weight_dn_end",         params_.weight_dn_end       );
        util_ini_parser_.ParseConfig("Lateral Dynamic Error LMPC", "weight_mu_end",         params_.weight_mu_end       );
        util_ini_parser_.ParseConfig("Lateral Dynamic Error LMPC", "weight_dmu_end",        params_.weight_dmu_end      );
        util_ini_parser_.ParseConfig("Lateral Dynamic Error LMPC", "weight_steer_end",      params_.weight_steer_end    );
        util_ini_parser_.ParseConfig("Lateral Dynamic Error LMPC", "weight_dsteer",         params_.weight_dsteer       );

        step_num_ = params_.step_num;
        step_dt_ = params_.step_dt;

        cout << "Update Lateral Dynamic Error LMPC ini" << endl;

    }
}