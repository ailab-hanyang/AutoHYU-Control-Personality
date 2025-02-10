/**
 * @file        can_parser_node.cpp
 * @brief       node cpp file for can parser node
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-04-12 created by Yuseung Na
 * 
 */

#include "can_parser_node.hpp"

CANParser::CANParser(std::string node_name, double period)
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
    s_frame_fd_0_ = nh.subscribe(
        "can_rx0", 100, &CANParser::CallbackFrameFD0, this);
    // s_frame_fd_1_ = nh.subscribe(
    //     "can_rx1", 100, &CANParser::CallbackFrameFD1, this);

    // Publisher init
    p_vehicle_can_ = nh.advertise<autohyu_msgs::VehicleCAN>(
        "bsw/vehicle_can", 10);

    // Algorithm init

}

CANParser::~CANParser() {
    // Terminate
}

void CANParser::Run() {
    ProcessINI();
}

void CANParser::Publish() {}

void CANParser::UpdateLabIONIQCAN(const autohyu_msgs::FrameFD& msg) {
    // Update Vehicle CAN
    o_vehicle_can_.steering_speed = 0.0;
    o_vehicle_can_.steering_state = interface::MdpsState::MDPS_ACTIVATE;
    o_vehicle_can_.motor_torque_f = 0.0;
    o_vehicle_can_.motor_torque_r = 0.0;
    o_vehicle_can_.accel_position = 0.0;
    o_vehicle_can_.brake_pressure = 0.0;

    // DynamicState (0x10)
    if (msg.id == lab_ioniq::CANID_DynamicState) {        
        lab_ioniq::VCAN_DYNAMIC_STATE vcan_dynamic_state;
        memcpy(vcan_dynamic_state.data, &msg.data[0], size_t(lab_ioniq::DLC_DynamicState));

        // long_acceleration [m/s^2]
        o_vehicle_can_.longitudinal_accel = ((double)vcan_dynamic_state.str.longitudinal_acceleration) * (0.01) * 9.81;

        // lat_acceleration [m/s^2]
        o_vehicle_can_.lateral_accel = ((double)vcan_dynamic_state.str.lateral_acceleration) * (0.01)* 9.81;

        // yaw_rate [rad/s]
        o_vehicle_can_.yaw_rate = ((double)vcan_dynamic_state.str.yaw_rate) * (0.1) * interface::DEG2RAD;       

        o_vehicle_can_.header = msg.header;
        p_vehicle_can_.publish(o_vehicle_can_); 
    }
    // WheelState (0x11)
    else if (msg.id == lab_ioniq::CANID_WheelState) {
        lab_ioniq::VCAN_WHEEL_STATE vcan_wheel_state;
        memcpy(vcan_wheel_state.data, &msg.data[0], size_t(lab_ioniq::DLC_WheelState));

        // wheel_speed_fl [m/s]
        o_vehicle_can_.wheel_velocity_fl = ((double)vcan_wheel_state.str.wheel_speed_fl) * (0.01);

        // wheel_speed_fr [m/s]
        o_vehicle_can_.wheel_velocity_fr = ((double)vcan_wheel_state.str.wheel_speed_fr) * (0.01);

        // wheel_speed_rl [m/s]
        o_vehicle_can_.wheel_velocity_rl = ((double)vcan_wheel_state.str.wheel_speed_rl) * (0.01);

        // wheel_speed_rr [m/s]
        o_vehicle_can_.wheel_velocity_rr = ((double)vcan_wheel_state.str.wheel_speed_rr) * (0.01);

        o_vehicle_can_.wheel_velocity_f_avg = (o_vehicle_can_.wheel_velocity_fr + o_vehicle_can_.wheel_velocity_fl) / 2.0;
        o_vehicle_can_.wheel_velocity_r_avg = (o_vehicle_can_.wheel_velocity_rr + o_vehicle_can_.wheel_velocity_rl) / 2.0;
        o_vehicle_can_.wheel_velocity_avg = (o_vehicle_can_.wheel_velocity_f_avg + o_vehicle_can_.wheel_velocity_r_avg) / 2.0;               
        
        o_vehicle_can_.header = msg.header;
        p_vehicle_can_.publish(o_vehicle_can_); 
    }
    // SteeringState (0x12)
    else if (msg.id == lab_ioniq::CANID_SteeringState) {
        lab_ioniq::VCAN_STEERING_STATE vcan_steering_state;
        memcpy(vcan_steering_state.data, &msg.data[0], size_t(lab_ioniq::DLC_SteeringState));

        // steering_wheel_angle [deg]
        o_vehicle_can_.steering_wheel_angle = ((double)vcan_steering_state.str.steering_wheel_angle) * (0.1);
        o_vehicle_can_.steering_tire_angle = util_function::WheelAngleToTireAngle(o_vehicle_can_.steering_wheel_angle);

        // steering_torque [Nm]
        o_vehicle_can_.steering_torque = ((double)vcan_steering_state.str.steering_torque) * (0.01);     
        
        o_vehicle_can_.header = msg.header;
        p_vehicle_can_.publish(o_vehicle_can_);      
    }
    // LongitudinalState (0x13)
    else if (msg.id == lab_ioniq::CANID_LongitudinalState) {
        lab_ioniq::VCAN_LONGITUDINAL_STATE vcan_longitudinal_state;
        memcpy(vcan_longitudinal_state.data, &msg.data[0], size_t(lab_ioniq::DLC_LongitudinalState));

        // acceleration_pedal_status [1: released, 2: pushed]

        // brake_pedal_status [1: released, 2: pushed]
        o_vehicle_can_.brake_active = (vcan_longitudinal_state.str.brake_pedal_status) - 1;

        o_vehicle_can_.header = msg.header;
        p_vehicle_can_.publish(o_vehicle_can_); 
    }
    // GearState (0x15)
    else if (msg.id == lab_ioniq::CANID_GearState) {
        lab_ioniq::VCAN_GEAR_STATE vcan_gear_state;
        memcpy(vcan_gear_state.data, &msg.data[0], size_t(lab_ioniq::DLC_GearState));

        // gear_status [0: n/a, 1: P, 2: R, 3: N, 4: D]
        o_vehicle_can_.gear_select = vcan_gear_state.str.gear_status;;           
        
        o_vehicle_can_.header = msg.header;
        p_vehicle_can_.publish(o_vehicle_can_);   
    }
    // // ModeStatus (0x0F)
    // else if (msg.id == lab_ioniq::CANID_ModeStatus) {
    //     lab_ioniq::VCAN_MODE_STATUS vcan_mode_status;
    //     memcpy(vcan_mode_status.data, &msg.data[0], size_t(lab_ioniq::DLC_ModeStatus));
        
    //     // operation_mode [0: n/a, 1: operation mode]
    //     o_vehicle_can_.operation_mode = vcan_mode_status.str.operation_mode;

    //     // autonomous_mode [0: Ready, 1: Run, 2: Pause]
    //     o_vehicle_can_.lateral_autonomous_mode      = vcan_mode_status.str.autonomous_mode;     
    //     o_vehicle_can_.longitudinal_autonomous_mode = vcan_mode_status.str.autonomous_mode;     
        
    //     o_vehicle_can_.header = msg.header;
    //     p_vehicle_can_.publish(o_vehicle_can_); 
    // }    
    else if (msg.id == internal_can::CANID_INTERNAL_STA) {
        internal_can::_INTERNAL_CAN_STA_ autoku_can_sta;
        memcpy(autoku_can_sta.data, &msg.data[0], size_t(internal_can::DLC_INTERNAL_STA));

        // AD Mode
        o_vehicle_can_.operation_mode = static_cast<interface::OperationMode>(autoku_can_sta.str.operation_mode);
        
        // Lateral Mode
        o_vehicle_can_.lateral_autonomous_mode = static_cast<interface::AutonomousMode>(autoku_can_sta.str.lateral_mode);
        o_vehicle_can_.longitudinal_autonomous_mode = static_cast<interface::AutonomousMode>(autoku_can_sta.str.longitudinal_mode);

        // Check if AD Mode is on
        if (o_vehicle_can_.operation_mode == 0) {
            o_vehicle_can_.lateral_autonomous_mode = 0;
            o_vehicle_can_.longitudinal_autonomous_mode = 0; 
        }
        
        o_vehicle_can_.header = msg.header;
        p_vehicle_can_.publish(o_vehicle_can_);
    }
}

void CANParser::UpdateHMGIONIQCAN(const autohyu_msgs::FrameFD& msg) {
    // Parse CAN data
    if (msg.id == hmg_ioniq::CANID_GWAY1) {
        hmg_ioniq::External_CAN_GWAY1 external_can_gway1;
        memcpy(external_can_gway1.data, &msg.data[0], size_t(hmg_ioniq::DLC_GWAY1));
        
        // Gway_Lateral_Accel_Speed [g] -> [m/s^2]
        o_vehicle_can_.lateral_accel = ((double)external_can_gway1.str.Gway_Lateral_Accel_Speed) * (0.000127465) + (-4.17677);
        o_vehicle_can_.lateral_accel = o_vehicle_can_.lateral_accel * 9.81;

        // Gway_Longitudinal_Accel_Speed [g] -> [m/s^2]
        o_vehicle_can_.longitudinal_accel = ((double)external_can_gway1.str.Gway_Longitudinal_Accel_Speed) * (0.000127465) + (-4.17677);
        o_vehicle_can_.longitudinal_accel = o_vehicle_can_.longitudinal_accel * 9.81;
        
        // Gway_Yaw_Rate_Sensor [rad/s]
        o_vehicle_can_.yaw_rate = ((double)external_can_gway1.str.Gway_Yaw_Rate_Sensor) * (0.005) + (-163.84);
        o_vehicle_can_.yaw_rate = o_vehicle_can_.yaw_rate * interface::DEG2RAD;

        // Gway_SAS_Angle [deg]
        o_vehicle_can_.steering_wheel_angle = ((double)external_can_gway1.str.Gway_SAS_Angle) * (0.1);
        o_vehicle_can_.steering_tire_angle = util_function::WheelAngleToTireAngle(o_vehicle_can_.steering_wheel_angle);
        
        // Gway_SAS_Speed [deg/s]
        o_vehicle_can_.steering_speed = ((double)external_can_gway1.str.Gway_SAS_Speed) * (4.0);
        
        // Gway_Steering_Tq [Nm]
        o_vehicle_can_.steering_torque = ((double)external_can_gway1.str.Gway_Steering_Tq) * (0.005) + (-20.48);

        // Gway_Steering_Status [0: n/a, 1: Normal, 2: Fail]
        o_vehicle_can_.steering_state = static_cast<interface::MdpsState>(external_can_gway1.str.Gway_Steering_Status);

        // Gway_Wheel_Velocity_FR [m/s]
        o_vehicle_can_.wheel_velocity_fr = ((double)external_can_gway1.str.Gway_Wheel_Velocity_FR) * (0.03125) * interface::KPH2MPS;

        // Gway_Wheel_Velocity_RL [m/s]
        o_vehicle_can_.wheel_velocity_rl = ((double)external_can_gway1.str.Gway_Wheel_Velocity_RL) * (0.03125) * interface::KPH2MPS;

        // Gway_Wheel_Velocity_RR [m/s]
        o_vehicle_can_.wheel_velocity_rr = ((double)external_can_gway1.str.Gway_Wheel_Velocity_RR) * (0.03125) * interface::KPH2MPS;

        // Gway_Wheel_Velocity_FL [m/s]
        o_vehicle_can_.wheel_velocity_fl = ((double)external_can_gway1.str.Gway_Wheel_Velocity_FL) * (0.03125) * interface::KPH2MPS;

        o_vehicle_can_.wheel_velocity_f_avg = (o_vehicle_can_.wheel_velocity_fr + o_vehicle_can_.wheel_velocity_fl) / 2.0;
        o_vehicle_can_.wheel_velocity_r_avg = (o_vehicle_can_.wheel_velocity_rr + o_vehicle_can_.wheel_velocity_rl) / 2.0;
        o_vehicle_can_.wheel_velocity_avg = (o_vehicle_can_.wheel_velocity_f_avg + o_vehicle_can_.wheel_velocity_r_avg) / 2.0;   
        
        // F_MCU_Torque [Nm]
        o_vehicle_can_.motor_torque_f = ((double)external_can_gway1.str.F_MCU_Torque) * (0.125);

        // R_MCU_Torque [Nm]
        o_vehicle_can_.motor_torque_r = ((double)external_can_gway1.str.R_MCU_Torque) * (0.125);     

        // Gway_Accel_Pedal_Position [%]
        o_vehicle_can_.accel_position = ((double)external_can_gway1.str.Gway_Accel_Pedal_Position) * (0.392157);
        
        // Gway_BrakeCylinder_Pressure [Bar]
        o_vehicle_can_.brake_pressure = ((double)external_can_gway1.str.Gway_BrakeCylinder_Pressure) * (0.1);

        // Gway_Brake_Active [0: n/a, 1: Active]
        o_vehicle_can_.brake_active = external_can_gway1.str.Gway_Brake_Active;

        // Gway_GearSelDisp [P:0, R: 7, N: 6, D: 5]
        o_vehicle_can_.gear_select = static_cast<interface::Gear>(external_can_gway1.str.Gway_GearSelDisp);
    
        o_vehicle_can_.header = msg.header;
        p_vehicle_can_.publish(o_vehicle_can_);
    }
    else if (msg.id == internal_can::CANID_INTERNAL_STA) {
        internal_can::_INTERNAL_CAN_STA_ autoku_can_sta;
        memcpy(autoku_can_sta.data, &msg.data[0], size_t(internal_can::DLC_INTERNAL_STA));

        // AD Mode
        o_vehicle_can_.operation_mode = static_cast<interface::OperationMode>(autoku_can_sta.str.operation_mode);
        
        // Lateral Mode
        o_vehicle_can_.lateral_autonomous_mode = static_cast<interface::AutonomousMode>(autoku_can_sta.str.lateral_mode);
        o_vehicle_can_.longitudinal_autonomous_mode = static_cast<interface::AutonomousMode>(autoku_can_sta.str.longitudinal_mode);

        // Check if AD Mode is on
        if (o_vehicle_can_.operation_mode == 0) {
            o_vehicle_can_.lateral_autonomous_mode = 0;
            o_vehicle_can_.longitudinal_autonomous_mode = 0; 
        }
        
        o_vehicle_can_.header = msg.header;
        p_vehicle_can_.publish(o_vehicle_can_);
    }
}

void CANParser::ProcessRosparam(const ros::NodeHandle& nh) {
    nh.getParam("/can_parser/mode",    cfg_.mode);
    nh.getParam("/can_parser/vehicle", cfg_.vehicle);

    ROS_WARN_STREAM("\nMode: " << cfg_.mode 
                 << "\nVehicle: " << cfg_.vehicle);
}

void CANParser::ProcessINI() {}

int main(int argc, char** argv) {
    std::string node_name = "can_parser";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_can_parser", period)) {
        period = 1.0;
    }

    CANParser main_task(node_name, period);
    main_task.Exec();

    return 0;
}