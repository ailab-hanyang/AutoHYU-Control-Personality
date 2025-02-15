/**
 * @file        pcan_router_node.cpp
 * @brief       node cpp file for pcan router node
 * 
 * @authors     Seheon Ha (seheonha@hanyang.ac.kr)          
 * 
 * @date        2025-02-14 created by Seheon Ha
 * 
 */

#include "pcan_router_node.hpp"

PCANRouter::PCANRouter(std::string node_name, double period)
    : TaskManager(node_name, period) {
    // Initialize
    ROS_WARN_STREAM("[" << node_name << "] Initialize node (Period: " << 1/period << " Hz)");

    // Node init
    ros::NodeHandle nh;

    // Ini init
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/bsw.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());

    // Parameter init
    ProcessINI();
    ProcessRosparam(nh);
    
    // Subscriber init
    s_can_rx_ = nh.subscribe("can_rx0", 10, &PCANRouter::CallbackCANRX, this);
    s_can_tx_ = nh.subscribe("can_tx0", 10, &PCANRouter::CallbackCANTX, this);

    // Publisher init
    p_can_tx_ = nh.advertise<autohyu_msgs::FrameFD>("can_tx1", 10); // Change Kvaser Topic can_tx1

    // Algorithm init
    ptr_auto_ku_can_process_ = make_unique<AutoKuCanProcess>();
}

PCANRouter::~PCANRouter() {
    // Terminate
}

void PCANRouter::Run() {
    ProcessINI();
    b_is_valid_ = true;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if(SYSTIME_DIFF(timer_10ms, SYSTIME_NOW) > TIME_11_MS)
    {
        timer_10ms = SYSTEM_NOW - (SYSTIME_DIFF(timer_10ms, SYSTIME_NOW) - TIME_11_MS);
        ptr_auto_ku_can_process_->Timer10msProcess(SYSTEM_NOW);
        
        b_is_ADCMD_updated_ = true;
    }
    
    if(SYSTIME_DIFF(timer_100ms, SYSTIME_NOW) > TIME_100_MS)
    {
        timer_100ms = SYSTIME_NOW - (SYSTEIME_DIFF(timer_100ms, SYSTIME_NOW) - TIME_100_MS);
        ptr_auto_ku_can_process_->Timer100msProcess();
        b_is_AutoKUSTA_updated_ = true;
    }

}


void PCANRouter::Publish() {
    if(b_is_ADCMD_updated_ == true){
        p_can_tx_.publish();
        b_is_ADCMD_updated_ = false;
    }
    
    if(b_is_AutoKUSTA_updated_ == true){
        p_can_tx_.publish();
        b_is_AutoKUSTA_updated_ = false;
    }
}

void PCANRouter::ProcessINI() {
    if (util_ini_parser_.IsFileUpdated() ) {
        ROS_WARN("[CAN Transmitter] Ini file is updated!");
        util_ini_parser_.ParseConfig("CAN Transmitter", "use_keyboard_input", b_use_keyboard_input);
        util_ini_parser_.ParseConfig("CAN Transmitter", "use_steering_speed_limit", b_use_steering_speed_limit);
        util_ini_parser_.ParseConfig("CAN Transmitter", "steering_speed_threshold", d_steering_speed_threshold);		
    }
}

void PCANRouter::ProcessRosparam(const ros::NodeHandle& nh) {
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
// Update functions for publish variables
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
void PCANRouter::UpdateKeyboardCommand(const interface::VehicleState& vehicle_state, 
                                           const interface::VehicleCmd& keyboard_command) {
    internal_can::INTERNAL_CAN_CMD autoku_cmd;
    o_vehicle_cmd_can_tx_.data.resize(internal_can::DLC_INTERNAL_CMD);
    
    // autoku_cmd.str.target_steering_angle = int16_t(keyboard_command.steering_angle/hmg_ioniq::FACTOR_TARGET_STEERING);
    autoku_cmd.str.target_steering_angle = int16_t(ptr_wheel_to_steering_->ConvertWheelToSteeringAngle(keyboard_command.steering_angle)/hmg_ioniq::FACTOR_TARGET_STEERING);

    autoku_cmd.str.target_acceleration = int16_t(keyboard_command.accel/hmg_ioniq::FACTOR_TARGET_ACCEL);
    autoku_cmd.str.target_speed = uint16_t(keyboard_command.speed/hmg_ioniq::FACTOR_TARGET_SPEED);    
    autoku_cmd.str.target_acc_pedal_pos = uint8_t(keyboard_command.gas/hmg_ioniq::FACTOR_TARGET_PEDAL_POSITION);
    autoku_cmd.str.target_brake_pedal_pos = uint8_t(keyboard_command.brake/hmg_ioniq::FACTOR_TARGET_PEDAL_POSITION);
    
    autoku_cmd.str.target_gear = uint8_t(0);
    
    autoku_cmd.str.reserve1 = 0;
    autoku_cmd.str.reserve2 = 0;
    autoku_cmd.str.reserve3 = 0;
    autoku_cmd.str.reserve4 = 0;
    autoku_cmd.str.reserve5 = 0;
    
    o_vehicle_cmd_can_tx_.id = internal_can::CANID_INTERNAL_CMD;
    o_vehicle_cmd_can_tx_.dlc = internal_can::DLC_INTERNAL_CMD;
    o_vehicle_cmd_can_tx_.is_canfd = true;
    memcpy(&o_vehicle_cmd_can_tx_.data[0], &autoku_cmd.data[0], size_t(internal_can::DLC_INTERNAL_CMD));
    
    o_vehicle_cmd_.header.stamp = ros::Time(vehicle_state.header.stamp);
    // o_vehicle_cmd_.steering_angle = keyboard_command.steering_angle;
    o_vehicle_cmd_.steering_angle = ptr_wheel_to_steering_->ConvertWheelToSteeringAngle(keyboard_command.steering_angle);
    o_vehicle_cmd_.front_tire_angle = keyboard_command.steering_angle;
    o_vehicle_cmd_.speed = keyboard_command.speed;
    o_vehicle_cmd_.accel = keyboard_command.accel;
    o_vehicle_cmd_.torque = keyboard_command.torque;
    o_vehicle_cmd_.gas = keyboard_command.gas;
    o_vehicle_cmd_.brake = keyboard_command.brake;
}

void PCANRouter::UpdateVehicleCommand(const interface::VehicleState& vehicle_state, 
                                          const double& accel, const double& torque, 
                                          const double& speed, const double& steer) {    
    static auto prev_command_time = std::chrono::system_clock::now();
    static double prev_steering_angle = vehicle_state.vehicle_can.steering_wheel_angle*interface::RAD2DEG;

    if(vehicle_state.vehicle_can.lateral_autonomous_mode == false){
        prev_steering_angle = vehicle_state.vehicle_can.steering_wheel_angle*interface::RAD2DEG;
    }

    internal_can::INTERNAL_CAN_CMD autoku_cmd;
    o_vehicle_cmd_can_tx_.data.resize(internal_can::DLC_INTERNAL_CMD);

    double target_steering_angle = ptr_wheel_to_steering_->ConvertWheelToSteeringAngle(steer);
    // double target_steering_angle = autoku_functions::TireAngleToWheelAngle(steer);
    auto current_command_time = std::chrono::system_clock::now();
    double dt = std::chrono::duration<double>(current_command_time-prev_command_time).count();
    double steering_speed = (target_steering_angle-prev_steering_angle)/dt;
    
    if (b_use_steering_speed_limit){
        if(vehicle_state.vehicle_can.steering_state == interface::MdpsState::MDPS_ACTIVATE){ 
            if(steering_speed > d_steering_speed_threshold){
                target_steering_angle = prev_steering_angle + d_steering_speed_threshold*dt;
                steering_speed = d_steering_speed_threshold;
                ROS_ERROR_STREAM("OVER STEERING SPEED "<< steering_speed << " dt " <<dt << " max "<<d_steering_speed_threshold);
            }
            else if(steering_speed < -d_steering_speed_threshold){
                target_steering_angle = prev_steering_angle - d_steering_speed_threshold*dt;
                steering_speed = -d_steering_speed_threshold;
                ROS_ERROR_STREAM("OVER STEERING SPEED "<< steering_speed << " dt " <<dt<< " max "<<d_steering_speed_threshold);
            }
        }
    }

    prev_command_time   = current_command_time;
    prev_steering_angle = target_steering_angle;

    autoku_cmd.str.target_steering_angle = int16_t(target_steering_angle/hmg_ioniq::FACTOR_TARGET_STEERING);
    autoku_cmd.str.target_acceleration = int16_t(accel/hmg_ioniq::FACTOR_TARGET_ACCEL);
    autoku_cmd.str.target_speed = uint16_t(speed/hmg_ioniq::FACTOR_TARGET_SPEED);

    // Transfer torque to pedal pos

    ptr_aps_table_->TorqueMap_U.In1 = torque; // Torque
    ptr_aps_table_->TorqueMap_U.In2 = vehicle_state.vx*interface::MPS2KPH; // Speed

    // double prev_time = ros::Time::now().toSec();
    ptr_aps_table_->step();
    
    double target_aps = std::min(99.0, ptr_aps_table_->TorqueMap_Y.Out1);
    double target_bps = 0.;
    if(target_aps< 0.01 || torque < interface::T_NEUTRAL){
        // target_bps = std::min(100.0, torque*BRAKE1_1+ BRAKE1_2);
        target_bps = std::min(100.0, pow(torque,3)*interface::BRAKE3_1
                                        + pow(torque,2)*interface::BRAKE3_2
                                        + pow(torque,1)*interface::BRAKE3_3
                                        + interface::BRAKE3_4);
        // if(torque < -BRAKE_Q1){
        //     target_bps =  (BRAKE_P1*torque*torque + BRAKE_P2*torque + BRAKE_P3) / (torque + BRAKE_Q1);
        // }
        // else{
        //     target_bps = 0.0;
        // }
        
        if(target_bps < 0.0){
            target_bps = 0.0;
        }
        if(target_bps > 100.0){
            target_bps = 100.0;
        }
        target_aps = 0.0;
    }
    // if(target_aps > 95.0){
    //     target_aps = 100.0;
    // }
    // if(target_bps > 99.0){
    //     target_bps = 99.0;
    // }
    
    
    // ROS_INFO_STREAM("target_aps : " << target_aps << " target_bps : " << target_bps);
    autoku_cmd.str.target_acc_pedal_pos = uint8_t(target_aps/hmg_ioniq::FACTOR_TARGET_PEDAL_POSITION);
    autoku_cmd.str.target_brake_pedal_pos = uint8_t(target_bps/hmg_ioniq::FACTOR_TARGET_PEDAL_POSITION);
    
    autoku_cmd.str.target_gear = uint8_t(4);
    
    autoku_cmd.str.reserve1 = 0;
    autoku_cmd.str.reserve2 = 0;
    autoku_cmd.str.reserve3 = 0;
    autoku_cmd.str.reserve4 = 0;
    autoku_cmd.str.reserve5 = 0;
    
    o_steering_wheel_angle_.data = target_steering_angle;
    o_gas_.data = target_aps;
    o_brake_.data = target_bps;

    o_vehicle_cmd_can_tx_.id = internal_can::CANID_INTERNAL_CMD;
    o_vehicle_cmd_can_tx_.dlc = internal_can::DLC_INTERNAL_CMD;
    o_vehicle_cmd_can_tx_.is_canfd = true;
    memcpy(&o_vehicle_cmd_can_tx_.data[0], &autoku_cmd.data[0], size_t(internal_can::DLC_INTERNAL_CMD));

    // for (int i = 0; i < sizeof(autoku_cmd.data); i++) {
    //     ROS_INFO_STREAM("can: " << i << ": " << static_cast<int>(o_vehicle_cmd_can_tx_.data[i]));
    // }
    
    o_vehicle_cmd_.header.stamp = ros::Time(vehicle_state.header.stamp);
    o_vehicle_cmd_.steering_angle = target_steering_angle;
    o_vehicle_cmd_.front_tire_angle = steer;
    o_vehicle_cmd_.speed = speed;
    o_vehicle_cmd_.accel = accel;
    o_vehicle_cmd_.torque = torque;
    o_vehicle_cmd_.gas = target_aps;
    o_vehicle_cmd_.brake = target_bps;
}

void PCANRouter::UpdateAutokuCanHealth(const uint8_t& health_sequence) {
    static uint8_t hs = 0;
    internal_can::INTERNAL_CAN_HEALTH autoku_health;
    o_health_can_tx_.data.resize(internal_can::DLC_INTERNAL_HEALTH);
    autoku_health.str.health_sequence = uint8_t(hs);
    o_health_can_tx_.id = internal_can::CANID_INTERNAL_HEALTH;
    o_health_can_tx_.dlc = internal_can::DLC_INTERNAL_HEALTH;
    o_health_can_tx_.is_canfd = true;
    hs ++;
      

    memcpy(&o_health_can_tx_.data[0], &autoku_health.data[0], size_t(internal_can::DLC_INTERNAL_HEALTH));

}

void PCANRouter::UpdateAutokuCanAdMode(const interface::ADModeInput& ad_mode_input) {
    internal_can::INTERNAL_CAN_AD_MODE autoku_ad_mode;
    o_ad_mode_can_tx_.data.resize(internal_can::DLC_INTERNAL_AD_MODE);

    autoku_ad_mode.str.operation_mode = uint8_t(ad_mode_input.ready_mode);
    autoku_ad_mode.str.lateral_mode = uint8_t(ad_mode_input.lateral_mode);
    autoku_ad_mode.str.longitudinal_mode = uint8_t(ad_mode_input.longitudinal_mode);
    autoku_ad_mode.str.manual_mode = uint8_t(ad_mode_input.manual_mode);

    o_ad_mode_can_tx_.id = internal_can::CANID_INTERNAL_AD_MODE;
    o_ad_mode_can_tx_.dlc = internal_can::DLC_INTERNAL_AD_MODE;
    o_ad_mode_can_tx_.is_canfd = true;
    memcpy(&o_ad_mode_can_tx_.data[0], &autoku_ad_mode.data[0], size_t(internal_can::DLC_INTERNAL_AD_MODE));
}

void PCANRouter::UpdateEFlag() {
    internal_can::INTERNAL_CAN_E_FLAG pseudo_e_flag;

    o_pseudo_e_flag_can_tx_.data.resize(internal_can::DLC_E_FLAG); // CAN 메시지의 DLC 설정
    
    pseudo_e_flag.str.e_flag = uint8_t(1);  // e_flag를 1로 설정

    o_pseudo_e_flag_can_tx_.id = internal_can::CANID_E_FLAG;
    o_pseudo_e_flag_can_tx_.dlc = internal_can::DLC_E_FLAG;
    o_pseudo_e_flag_can_tx_.is_canfd = true;

    memcpy(&o_pseudo_e_flag_can_tx_.data[0], &pseudo_e_flag.data[0], size_t(internal_can::DLC_E_FLAG));

}


int main(int argc, char** argv) {
    // Initialize node
    std::string node_name = "pcan_router";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_can_transmitter", period)) {
        period = 1.0;
    }
    
    PCANRouter main_task(node_name, period);
    main_task.Exec();

    return 0;
}