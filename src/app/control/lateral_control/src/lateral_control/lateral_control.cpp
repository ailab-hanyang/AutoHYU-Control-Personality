#include <lateral_control/lateral_control.hpp>

using namespace std;
using namespace tk;

LateralControl::LateralControl(){
    // Make unique pointer
}

LateralControl::~LateralControl(){
}

bool LateralControl::Init(){
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/control.ini");    
    util_ini_parser_.Init((dir+ini_dir).c_str());

    // Parameters initialization
    ProcessINI();
    
    i_vehicle_state_ = std::make_shared<const VehicleState>();
    i_reference_trajectory_ = std::make_shared<const Trajectory>();

    CreateLateralController();
    ptr_lateral_fallback_ = make_unique<LateralControlFallback>();
    ptr_lateral_fallback_->Init();

    return true;
}

std::tuple<ControlCommand, ControlTrajectory, PathTrackingInfos> LateralControl::Run(
                            const VehicleState& vehicle_state, const Trajectory& trajectory)
{
    i_vehicle_state_ = std::make_shared<const VehicleState>(vehicle_state);
    i_reference_trajectory_ = std::make_shared<const Trajectory>(trajectory);
    ProcessINI();
    CreateLateralController();

    // TF reference trajectory to controller's target frame
    Trajectory ego_trajectory = TransformTrajectory(*i_reference_trajectory_, *i_vehicle_state_, ptr_lateral_controller_->GetTargetFrame());
    
    // Generate frenet frame based on transformed trajectory
    tk::Map road_map =  GenerateRoadMap(ego_trajectory);
    
    // Resample and calculate reference attributes
    ControlTrajectory controllable_trajectory = GenerateTargetTrajectory(ego_trajectory, road_map, ptr_lateral_controller_->GetTargetFrame());
    
    // Assign current vehicle state to controller
    ptr_lateral_controller_->BuildControllerVehicleState(*i_vehicle_state_,road_map);
    
    auto start = chrono::high_resolution_clock::now();
    boost::optional<ControlCommand> control_command = ptr_lateral_controller_->CalculateOptimalTireSteering(controllable_trajectory);
    auto end = chrono::high_resolution_clock::now();
    
    auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
    path_tracking_info_.total_time = duration.count()*0.001;
    
    if(control_command){
        if(params_.use_understeer_gradient){
            // Compensate lateral slip for kinematic model based controllers
            control_command->steering_tire_angle += CompensateUndersteerGradient(controllable_trajectory);
        }
        else{
            path_tracking_info_.compensation_lateral = 0.0;
        }
        if(params_.use_vehicle_twisting_compensator){
            // Compensate accident aftereffect for 2023 HMG AutoKU IONIQ5 (aka. banana car) 
            double vehicle_twisting_steer = params_.vehicle_twisting_constant*i_vehicle_state_->vehicle_can.motor_torque_total;
            vehicle_twisting_steer = min(params_.max_vehicle_twisting_compensation,vehicle_twisting_steer);
            vehicle_twisting_steer = max(0.0,vehicle_twisting_steer);
            control_command->steering_tire_angle -= vehicle_twisting_steer;
        }
        return {*control_command, controllable_trajectory, path_tracking_info_};
    }
    else{
        // Fallback controller activate when control_command is boost::none
        std::cout << "FallBack Controller Activated!!!!!!!!!!!" << std::endl;
        ControlCommand backup_cmd = ptr_lateral_fallback_->PurePursuitSteeringAngle( *i_vehicle_state_,*i_reference_trajectory_);
        return {backup_cmd, controllable_trajectory, path_tracking_info_};
    }
}

Trajectory LateralControl::TransformTrajectory(const Trajectory& trajectory, const VehicleState& vehicle_state,
                                                const std::string& frame_out){
    Trajectory ego_trajectory;    
    for(auto point : trajectory.point){
        TrajectoryPoint ego_point;
        Eigen::Affine3d point_global_tf, point_local_tf, ego_frame_tf;
        point_global_tf = Eigen::Affine3d::Identity();
        point_local_tf = Eigen::Affine3d::Identity();
        ego_frame_tf = Eigen::Affine3d::Identity();

        point_global_tf.translate(Eigen::Vector3d(point.x, point.y, 0.0));
        point_global_tf.rotate( Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) * 
                                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(point.yaw, Eigen::Vector3d::UnitZ()));

        ego_frame_tf.translation()[0] = vehicle_state.x;
        ego_frame_tf.translation()[1] = vehicle_state.y;
        ego_frame_tf.translation()[2] = 0.0;

        ego_frame_tf = Eigen::Translation3d(ego_frame_tf.translation()) *
                        (Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(vehicle_state.yaw, Eigen::Vector3d::UnitZ()));

        point_local_tf = ego_frame_tf.inverse() * point_global_tf;
        Eigen::Vector3d ego_point_rpy = point_local_tf.rotation().eulerAngles(0,1,2);

        ego_point.x   = point_local_tf.translation().x();
        ego_point.y   = point_local_tf.translation().y();
        ego_point.yaw = ego_point_rpy(2);
        ego_point.speed  = point.speed;

        // Translate x w.r.t target frame (default is rear axle)
        // It is based on assuming the vehicle state comes from the rear axle
        // Deprecated
        // if(frame_out == "CG"){
        //     ego_point.x -= VEHICLE_REAR_AXLE_TO_CG;
        // }
        // else if(frame_out == "front_axle"){
        //     ego_point.x -= VEHICLE_WHEEL_BASE;
        // }
        
        ego_trajectory.point.push_back(ego_point);
    }

    // return x,y,(yaw),speed only trajectory
    return ego_trajectory;
    
}

tk::Map LateralControl::GenerateRoadMap(const Trajectory& trajectory) {
    if ((uint16_t)trajectory.point.size() < 4) {
        tk::Map road_map;
        std::cout<<"Cannot generate road map to spline!"<<std::endl;
        return road_map;
    } else {
        std::vector<double> sx, sy, ss, sdx, sdy;
        double sum_s = 0.0;
        for (uint16_t i = 0; i < (uint16_t)trajectory.point.size(); i++) {
            double x = trajectory.point[i].x;
            double y = trajectory.point[i].y;
            double dx = i == 0 ? 0.0 : x - trajectory.point[i - 1].x;
            double dy = i == 0 ? 0.0 : y - trajectory.point[i - 1].y;
            sum_s += sqrt(dx * dx + dy * dy);
            sx.push_back(x);
            sy.push_back(y);
            ss.push_back(sum_s);
            sdx.push_back(dx);
            sdy.push_back(dy);
        }

        return tk::Map(sx, sy, ss, sdx, sdy);
    }
}


ControlTrajectory LateralControl::GenerateTargetTrajectory(const Trajectory& trajectory, tk::Map road_map, const std::string& frame_out){
    // We only use x, y and (speed) from reference trajectory for controller independence 
    
    // Convert ego cartesian to frenet coordinate
    // TODO: 원형 경로 추종시 closest point를 잘못뽑을 수 있음
    std::vector<double> ego_sn = road_map.ToFrenet(0.0, 0.0, 0.2);
    path_tracking_info_.cross_track_error = ego_sn[1];
    path_tracking_info_.yaw_error = -road_map.GetSlope(ego_sn[0])*RAD2DEG;

    ControlTrajectory interpolated_trajectory;
    interpolated_trajectory.frame_id = frame_out;
    vector<double> time, speed, distance;
    double sum_s = 0.;
    double sum_time = 0.;
    for (uint16_t i = 0; i < (uint16_t)trajectory.point.size(); i++) {

        double dx = i == 0 ? 0.0 : trajectory.point[i].x - trajectory.point[i - 1].x;
        double dy = i == 0 ? 0.0 : trajectory.point[i].y - trajectory.point[i - 1].y;
        double ds = sqrt(dx * dx + dy * dy);
        double dt;

        if(params_.use_reference_speed == true){
            // reference speed based on reference trajectory speed
            // if you need to use longitudinal control please remove the max condition
            dt = i == 0 ? 0.0 : 2*ds/max(trajectory.point[i].speed + trajectory.point[i-1].speed , (float)0.1);
            speed.push_back(trajectory.point[i].speed);

        }
        else{
            // reference speed based on vehicle current speed
            dt = ds/max(i_vehicle_state_->vx,0.1);
            speed.push_back(i_vehicle_state_->vx);
        }

        sum_s += ds;
        sum_time += dt;

        distance.push_back(sum_s);
        time.push_back(sum_time);
    }

    if ((uint16_t)time.size() < 2 || (uint16_t)speed.size() < 2 || (uint16_t)distance.size() < 2) {
        std::cout<<"Cannot generate target interpolated_trajectory!"<<std::endl;
        std::cout<<"Size of time: " << (int)time.size() << 
                    ", speed: "<<(int)speed.size() << ", dist: %d"<< (int)distance.size() <<std::endl;
        
        return interpolated_trajectory;
    }

    tk::spline st, ts, tv;
    if(i_vehicle_state_->vx < 0.1){
        // linear interpolation at low speed
        st.set_points(distance, time, false);
        ts.set_points(time, distance, false);
        tv.set_points(time, speed, false);
    }
    else{
        st.set_points(distance, time, true);
        ts.set_points(time, distance, true);
        tv.set_points(time, speed, true);
    }
    double time_offset = st(ego_sn[0]) + params_.reference_time_offset;

    int step_num     = ptr_lateral_controller_->GetControlTrajectoryStepNum();
    double step_time = ptr_lateral_controller_->GetControlTrajectoryStepTime();

    // yaw unwrapping (important for initial guess and cost function!)
    double previous_yaw = 0.0; // Initialize with the first raw yaw value
    double offset = 0.0;  // Angle adjustment value

    for (uint16_t i = 0; i < step_num + 2; i++) {
        ControlPoint dp;
        double target_distance = ts(step_time * i + time_offset);
        std::vector<double> xy = road_map.ToCartesian(target_distance, 0.0);
        dp.x = xy[0];
        dp.y = xy[1];

        // Calculate current yaw value (within -pi to pi range)
        double current_yaw = road_map.GetSlope(target_distance);

        // Compute the difference between current yaw and previous yaw
        double delta_yaw = current_yaw - previous_yaw;

        // Adjust the offset based on the delta to unwrap the yaw
        if (delta_yaw > M_PI) {
            offset -= 2 * M_PI;
        } else if (delta_yaw < -M_PI) {
            offset += 2 * M_PI;
        }

        // Apply the offset to get the unwrapped yaw
        double unwrapped_yaw = current_yaw + offset;
        dp.yaw = unwrapped_yaw;

        // Update previous_yaw with the current raw yaw (before unwrapping)
        previous_yaw = current_yaw;

        dp.curvature = road_map.GetCurvature(target_distance);
        dp.vx = tv(step_time * i + time_offset);
        dp.time = step_time * i;
        dp.distance = target_distance - ts(time_offset);
        interpolated_trajectory.control_point.push_back(dp);
    }
    return interpolated_trajectory;
}

double LateralControl::CompensateUndersteerGradient(const ControlTrajectory& ref){
    static double understeer_gradient = 0.0; // "static local variable"
    static double prev_understeer_gradient = 0.0;
    static auto prev_command_time = std::chrono::system_clock::now();

    // yawrate and velocities are measured from rear axle (ego frame)
    double beta = atan2(i_vehicle_state_->vy,i_vehicle_state_->vx);
    double alpha_f = -beta+i_vehicle_state_->vehicle_can.steering_tire_angle
                     -VEHICLE_WHEEL_BASE*i_vehicle_state_->yaw_vel/i_vehicle_state_->vx;
    double alpha_r = -beta+0.0*i_vehicle_state_->yaw_vel/i_vehicle_state_->vx;
    
    // double Kv = VEHICLE_REAR_AXLE_TO_CG*VEHICLE_MASS/(FRONT_CORNERING_STIFFNESS*VEHICLE_WHEEL_BASE)
    //             -VEHICLE_FRONT_AXLE_TO_CG*VEHICLE_MASS/(REAR_CORNERING_STIFFNESS*VEHICLE_WHEEL_BASE);
    double Kv = params_.understeer_gradient_kv;

    int delay_sample = max(1, (int)(params_.time_delay_for_compensators/ptr_lateral_controller_->GetControlTrajectoryStepTime()));

    double target_curvature = ref.control_point[delay_sample].curvature;
    if(isnan(target_curvature)){
        target_curvature = 0.0;
    }
    double calculated_understeer_gradient = 0.0;
    if(fabs(i_vehicle_state_->vx) > 1.0){
        if(fabs(alpha_f) - fabs(alpha_r) > -1.0){
            // calculated_understeer_gradient = (alpha_f - alpha_r)*RAD2DEG;
            if (fabs(target_curvature) > 0.001){
                double ay = target_curvature*i_vehicle_state_->vx*i_vehicle_state_->vx;
                calculated_understeer_gradient = Kv*ay*RAD2DEG;
            }
        }
    }
    double target_yawrate = i_vehicle_state_->vx*target_curvature;      
    double yaw_rate_error =  target_yawrate - i_vehicle_state_->yaw_vel;

    // lpf understeer_gradient
    understeer_gradient = params_.understeer_gradient_lpf*calculated_understeer_gradient + (1.0-params_.understeer_gradient_lpf)*understeer_gradient;
    // std::cout << "UndersteerGradient "<<  understeer_gradient << "deg"<< std::endl;
    
    auto current_command_time = std::chrono::system_clock::now();
    double dt = std::chrono::duration<double>(current_command_time-prev_command_time).count();
    double understeer_gradient_rate = (understeer_gradient-prev_understeer_gradient)/dt;

    if(understeer_gradient_rate > params_.understeer_gradient_rate_limit){
        understeer_gradient = prev_understeer_gradient + params_.understeer_gradient_rate_limit*dt;
    }
    else if(understeer_gradient_rate < -params_.understeer_gradient_rate_limit){
        understeer_gradient = prev_understeer_gradient - params_.understeer_gradient_rate_limit*dt;
    }
    prev_understeer_gradient = understeer_gradient; 
    prev_command_time = current_command_time;

    path_tracking_info_.compensation_lateral = understeer_gradient;
    return understeer_gradient;
}

void LateralControl::CreateLateralController(){
    if(prev_control_method_ == params_.lateral_control_method){
        return;
    }   
    ptr_lateral_controller_.reset();
    switch (params_.lateral_control_method)
    {
    case LateralControlMethod::DELAYED_KINEMATIC_BICYCLE_LMPC:
        ptr_lateral_controller_ = std::make_shared<LateralDelayedKinematicBicycleLMPC>();
        break;
    case LateralControlMethod::KINEMATIC_BICYCLE_LMPC:
        ptr_lateral_controller_ = std::make_shared<LateralKinematicBicycleLMPC>();
        break;
    case LateralControlMethod::KINEMATIC_BICYCLE_NMPC:
        ptr_lateral_controller_ = std::make_shared<LateralKinematicBicycleNMPC>();
        break;
    case LateralControlMethod::DYNAMIC_ERROR_LMPC:
        ptr_lateral_controller_ = std::make_shared<LateralDynamicErrorLMPC>();
        break;
    case LateralControlMethod::PURE_PURSUIT:
        ptr_lateral_controller_ = std::make_shared<LateralPurePursuit>();
        break;
    default:
        break;
    }

    prev_control_method_ = params_.lateral_control_method;

}

void LateralControl::ProcessINI(){
	if (util_ini_parser_.IsFileUpdated()){
		util_ini_parser_.ParseConfig("Lateral Control", "lateral_control_method", params_.lateral_control_method);
		util_ini_parser_.ParseConfig("Lateral Control", "reference_time_offset" , params_.reference_time_offset);
		util_ini_parser_.ParseConfig("Lateral Control", "use_reference_speed"   , params_.use_reference_speed);
		util_ini_parser_.ParseConfig("Lateral Control", "time_delay_for_compensators"   , params_.time_delay_for_compensators);

		util_ini_parser_.ParseConfig("Lateral Control", "use_understeer_gradient"   , params_.use_understeer_gradient);
        util_ini_parser_.ParseConfig("Lateral Control", "understeer_gradient_lpf"   , params_.understeer_gradient_lpf);
        util_ini_parser_.ParseConfig("Lateral Control", "understeer_gradient_rate_limit" , params_.understeer_gradient_rate_limit);
        util_ini_parser_.ParseConfig("Lateral Control", "understeer_gradient_kv"   , params_.understeer_gradient_kv);

        util_ini_parser_.ParseConfig("Lateral Control", "use_vehicle_twisting_compensator"   , params_.use_vehicle_twisting_compensator);
        util_ini_parser_.ParseConfig("Lateral Control", "vehicle_twisting_constant"   , params_.vehicle_twisting_constant);
        util_ini_parser_.ParseConfig("Lateral Control", "max_vehicle_twisting_compensation"   , params_.max_vehicle_twisting_compensation);


		std::cout<<"[Lateral Control] Ini file is updated!"<<std::endl;
    }
}
