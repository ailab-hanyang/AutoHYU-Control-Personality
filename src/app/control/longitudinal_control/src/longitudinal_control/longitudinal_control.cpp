/**
 * @file        template_algorithm_algorithm.cpp
 * @brief       template algorithm cpp file for template node algorithm
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-04-01 created by Yuseung Na
 * 
 */

#include <longitudinal_control/longitudinal_control.hpp>

LongitudinalControl::LongitudinalControl(){
}

LongitudinalControl::~LongitudinalControl(){
    // terminate
}

bool LongitudinalControl::Init(){
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/control.ini");    
    util_ini_parser_.Init((dir+ini_dir).c_str());

    // Parameters initialization
    ProcessINI();
    
    i_vehicle_state_ = std::make_shared<const VehicleState>();
    i_reference_trajectory_ = std::make_shared<const Trajectory>();
    
    CreateLongitudinalController();

    return true;
}


std::tuple<ControlCommand, ControlTrajectory, PathTrackingInfos> LongitudinalControl::Run(
        const interface::VehicleState& vehicle_state,
        const interface::Trajectory& trajectory)
{
    i_vehicle_state_ = std::make_shared<const VehicleState>(vehicle_state);
    i_reference_trajectory_ = std::make_shared<const Trajectory>(trajectory);

    ProcessINI();
    CreateLongitudinalController();

    Trajectory ego_trajectory = TransformTrajectory(*i_reference_trajectory_, *i_vehicle_state_, ptr_longitudinal_controller_->GetTargetFrame());
    tk::Map road_map =  GenerateRoadMap(ego_trajectory);
    ControlTrajectory controllable_trajectory = GenerateTargetSpeedProfile(ego_trajectory, road_map, ptr_longitudinal_controller_->GetTargetFrame());
    CalculateSpeedTrackingErrors(controllable_trajectory,*i_vehicle_state_);
    ptr_longitudinal_controller_->BuildControllerVehicleState(*i_vehicle_state_,road_map);
    ControlCommand control_command = ptr_longitudinal_controller_->CalculateOptimalLongitudinalCommand(controllable_trajectory);
    if(params_.use_speed_compensator){
        float comp_trq = SpeedCompensator(controllable_trajectory,*i_vehicle_state_,ptr_longitudinal_controller_->GetControlTrajectoryStepTime());
        speed_tracking_infos_.compensation_longitudinal = comp_trq;
        control_command.trq += comp_trq;
        control_command.ax += comp_trq/(VEHICLE_MASS*VEHICLE_WHEEL_RADIUS);
    }
    else{
        speed_tracking_infos_.compensation_longitudinal = 0.0;
    }

    FaultCheck(control_command);

    return {control_command, controllable_trajectory, speed_tracking_infos_};
}


Trajectory LongitudinalControl::TransformTrajectory(const Trajectory& trajectory, const VehicleState& vehicle_state,
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

tk::Map LongitudinalControl::GenerateRoadMap(const Trajectory& trajectory) {
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

ControlTrajectory LongitudinalControl::GenerateTargetSpeedProfile(const Trajectory& trajectory, tk::Map road_map, const std::string& frame_out){
    // Convert Ego Cartesian to Frenet coordinate , Only need <X, Y, Speed> 
    std::vector<double> ego_sn = road_map.ToFrenet(0.0, 0.0, 0.2);
    std::vector<double> start = {ego_sn[0], ego_sn[1]};
    cte_error_ = start[1];

    ControlTrajectory interpolated_trajectory;
    interpolated_trajectory.frame_id = frame_out;
    vector<double> time, speed, distance;
    double sum_s = 0.;
    double sum_time = 0.;
    
    // bool is_stop = true;
    
    for (uint16_t i = 0; i < (uint16_t)trajectory.point.size(); i++) {
        double x = trajectory.point[i].x;
        double y = trajectory.point[i].y;
        double dx = i == 0 ? 0.0 : x - trajectory.point[i - 1].x;
        double dy = i == 0 ? 0.0 : y - trajectory.point[i - 1].y;
        double ds = sqrt(dx * dx + dy * dy);
        double dt;
        double vx;

        if(params_.use_user_speed == false){
            vx = trajectory.point[i].speed;
            dt = i == 0 ? 0.0 : 2*ds/max(trajectory.point[i].speed + trajectory.point[i-1].speed , (float)0.1);
            
            if(vx < 0.1){
                vx = params_.zero_speed;
            }
        }
        else{
            vx = params_.user_speed;
            dt = ds/max(params_.user_speed,0.1);
        }

        sum_s += ds;
        sum_time += dt;

        distance.push_back(sum_s);
        time.push_back(sum_time);
        speed.push_back(vx);
    }

    if ((uint16_t)time.size() < 2 || (uint16_t)speed.size() < 2 || (uint16_t)distance.size() < 2) {
        std::cout<<"Cannot generate target interpolated_trajectory!"<<std::endl;
        std::cout<<"Size of time: " << (int)time.size() << 
                    ", speed: "<<(int)speed.size() << ", dist: %d"<< (int)distance.size() <<std::endl;
        
        return interpolated_trajectory;
    }

    tk::spline st, ts, tv;
    st.set_points(distance, time, false);
    ts.set_points(time, distance, false);
    tv.set_points(time, speed, false);
    
    double time_offset = st(start[0])+ params_.reference_time_offset;
    // std::cout <<"time_offset " << time_offset  << std::endl;

    int step_num     = ptr_longitudinal_controller_->GetControlTrajectoryStepNum();
    double step_time = ptr_longitudinal_controller_->GetControlTrajectoryStepTime();

    for (uint16_t i = 0; i < step_num+2; i++) {
        ControlPoint dp;
        double target_distance = ts(step_time * i + time_offset);
        vector<double> xy = road_map.ToCartesian(target_distance, 0.0);
        dp.x = xy[0]; 
        dp.y = xy[1];
        // dp.yaw = road_map.GetSlope(target_distance);
        // dp.curvature = road_map.GetCurvature(target_distance);
        dp.vx = tv(step_time * i + time_offset);
        dp.ax = (tv(step_time * (i+1) + time_offset) - dp.vx)/step_time;
        dp.time = step_time * i ;
        // dp.distance = target_distance - ts(time_offset);
        interpolated_trajectory.control_point.push_back(dp);
    }
    return interpolated_trajectory;
}

float LongitudinalControl::SpeedCompensator(const ControlTrajectory& profile, const VehicleState& vehicle_state, const double& dt){
    

    int target_idx = min((int)(params_.look_ahead_time/dt), (int)profile.control_point.size() );
    static double ref_speed = i_vehicle_state_->vx;
    
    double calculated_ref_speed = profile.control_point.at(target_idx).vx;
    
    if(!isnan(calculated_ref_speed)){
        ref_speed = params_.ref_speed_lpf*calculated_ref_speed + (1.0-params_.ref_speed_lpf)*ref_speed;
    }
    
    double current_speed = sqrt(pow((vehicle_state.vx),2) + pow((vehicle_state.vy),2));

    double speed_error = ref_speed - current_speed;
    sum_speed_error_ += speed_error;

    if(prev_speed_error_ * speed_error < 0){
        sum_speed_error_ = 0.0;
    }
    if(sum_speed_error_ > params_.max_i_error){
        sum_speed_error_ = params_.max_i_error;
    }
    else if(sum_speed_error_ < -params_.max_i_error){
        sum_speed_error_ = -params_.max_i_error;
    }

    prev_speed_error_ = speed_error;
    
    speed_tracking_infos_.speed_compensation_error  = (ref_speed-current_speed)*MPS2KPH;
    
    float comp_trq    =  params_.vel_kp * speed_error
                        + params_.vel_ki * sum_speed_error_
                        + params_.vel_kd * (speed_error-prev_speed_error_); 
    // std::cout << "comped_trq" << comp_trq << std::endl;
    return comp_trq;
}

void LongitudinalControl::FaultCheck(ControlCommand& control_command){
    if(isnan(control_command.trq)){
        // Stop smooth ... if nan
        control_command.trq = -2500.0;
        control_command.ax = -3.0;
    }
    else if(i_vehicle_state_->vehicle_can.lateral_autonomous_mode == AutonomousMode::RUN && i_vehicle_state_->vehicle_can.steering_state != MdpsState::MDPS_ACTIVATE){
        if(i_vehicle_state_->vx > 5.0*KPH2MPS){
            control_command.trq = min(control_command.trq, (float)-2500.0);
            control_command.ax  = min(control_command.ax, (float)-3.0);
        }
        else{
            control_command.ax  = min(control_command.ax, (float)0.5);
            double control_Fx = VEHICLE_MASS*control_command.ax 
                                + VEHICLE_MASS*GRAVITY_COEFF*sin(-i_vehicle_state_->pitch) 
                                + VEHICLE_MASS*ROLLING_RESISTANCE*GRAVITY_COEFF*cos(-i_vehicle_state_->pitch)
                                + 0.5*AIR_DENSITY*VEHICLE_FRONT_AREA*AIR_DRAG_COEFF*i_vehicle_state_->vx*i_vehicle_state_->vx;
            double control_torque = control_Fx * VEHICLE_WHEEL_RADIUS;

            control_command.trq = min(control_command.trq, (float)control_torque);
        }
    }

    if(params_.use_cte_decceleration){
        double target_cte = fabs(cte_error_);
        if(target_cte > params_.decceleration_min_cte){
            double cte_ratio = (params_.decceleration_max_accel - params_.decceleration_min_accel)/(params_.decceleration_max_cte-params_.decceleration_min_cte);
            
            target_cte = max(target_cte, params_.decceleration_min_cte);
            target_cte = min(target_cte, params_.decceleration_max_cte);
            
            double target_accel = params_.decceleration_min_accel + cte_ratio*(target_cte-params_.decceleration_min_cte);
            
            control_command.ax  = min(control_command.ax, (float)target_accel);
            double control_Fx = VEHICLE_MASS*control_command.ax 
                                + VEHICLE_MASS*GRAVITY_COEFF*sin(-i_vehicle_state_->pitch) 
                                + VEHICLE_MASS*ROLLING_RESISTANCE*GRAVITY_COEFF*cos(-i_vehicle_state_->pitch)
                                + 0.5*AIR_DENSITY*VEHICLE_FRONT_AREA*AIR_DRAG_COEFF*i_vehicle_state_->vx*i_vehicle_state_->vx;
            double control_torque = control_Fx * VEHICLE_WHEEL_RADIUS;

            control_command.trq = min(control_command.trq, (float)control_torque);
        }
    }
}

void LongitudinalControl::CalculateSpeedTrackingErrors(const ControlTrajectory& profile, const VehicleState& vehicle_state){
    if(profile.control_point.size()>0){
        speed_tracking_infos_.speed_error  = (profile.control_point[0].vx - vehicle_state.vx)*MPS2KPH;
        speed_tracking_infos_.accel_error =  profile.control_point[0].ax - vehicle_state.ax;
    }
    else{
        std::cout << "Not enough ref profile to calculate errors" << std::endl;
    }
}

void LongitudinalControl::CreateLongitudinalController(){
    if(prev_control_method_ == params_.longitudinal_control_method){
        return;
    }
    ptr_longitudinal_controller_.reset();
    switch (params_.longitudinal_control_method)
    {
    case LongitudinalControlMethod::DELAYED_DYNAMIC_LMPC:
        ptr_longitudinal_controller_ = std::make_shared<LongitudinalDelayedDynamicLMPC>();
        break;
    case LongitudinalControlMethod::DYNAMIC_LMPC:
        ptr_longitudinal_controller_ = std::make_shared<LongitudinalDynamicLMPC>();
        break;
    case LongitudinalControlMethod::PID:
        ptr_longitudinal_controller_ = std::make_shared<LongitudinalPID>();
        break;
    case LongitudinalControlMethod::AX_FEEDFORWARD:
        ptr_longitudinal_controller_ = std::make_shared<LongitudinalAxFeedForward>();
        break;
    case LongitudinalControlMethod::DELAYED_KINEMATIC_LMPC:
        ptr_longitudinal_controller_ = std::make_shared<LongitudinalDelayedKinematicLMPC>();
        break;
    default:
        break;
    }

    prev_control_method_ = params_.longitudinal_control_method;

}

void LongitudinalControl::ProcessINI(){
	if (util_ini_parser_.IsFileUpdated()){
		util_ini_parser_.ParseConfig("Longitudinal Control", "longitudinal_control_method", params_.longitudinal_control_method);
		util_ini_parser_.ParseConfig("Longitudinal Control", "reference_time_offset" , params_.reference_time_offset);
		util_ini_parser_.ParseConfig("Longitudinal Control", "use_user_speed"        , params_.use_user_speed);
		util_ini_parser_.ParseConfig("Longitudinal Control", "user_speed"            , params_.user_speed);

		util_ini_parser_.ParseConfig("Longitudinal Control", "use_speed_compensator" , params_.use_speed_compensator);
		util_ini_parser_.ParseConfig("Longitudinal Control", "look_ahead_time"       , params_.look_ahead_time);
		util_ini_parser_.ParseConfig("Longitudinal Control", "vel_kp"                , params_.vel_kp);
		util_ini_parser_.ParseConfig("Longitudinal Control", "vel_ki"                , params_.vel_ki);
		util_ini_parser_.ParseConfig("Longitudinal Control", "vel_kd"                , params_.vel_kd);
		util_ini_parser_.ParseConfig("Longitudinal Control", "max_i_error"           , params_.max_i_error);
		util_ini_parser_.ParseConfig("Longitudinal Control", "ref_speed_lpf"         , params_.ref_speed_lpf);

		util_ini_parser_.ParseConfig("Longitudinal Control", "zero_speed"            , params_.zero_speed);

		util_ini_parser_.ParseConfig("Longitudinal Control", "use_cte_decceleration"            , params_.use_cte_decceleration);
		util_ini_parser_.ParseConfig("Longitudinal Control", "decceleration_min_cte"            , params_.decceleration_min_cte);
		util_ini_parser_.ParseConfig("Longitudinal Control", "decceleration_min_accel"            , params_.decceleration_min_accel);
		util_ini_parser_.ParseConfig("Longitudinal Control", "decceleration_max_cte"            , params_.decceleration_max_cte);
		util_ini_parser_.ParseConfig("Longitudinal Control", "decceleration_max_accel"            , params_.decceleration_max_accel);

        params_.user_speed =  params_.user_speed*KPH2MPS;

		std::cout<<"[Longitudinal Control] Ini file is updated!"<<std::endl;
    }
}