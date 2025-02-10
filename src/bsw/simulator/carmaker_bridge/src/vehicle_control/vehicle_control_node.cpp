/**
 * @file        vehicle_control_node.hpp
 * @brief       carmaker vehicle control node hpp file
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-05-16 created by Jeonghun Kang
 * 
 */
#include "vehicle_control/vehicle_control_node.hpp"

VehicleControl::VehicleControl(std::string node_name, double period)
        : TaskManager(node_name, period) {
    // Node initialization
    ros::NodeHandle nh;

    // Ini init
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/simulation.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());

    // ROS param init

    // Subscriber init    
    s_vehicle_state_ = nh.subscribe(
            "app/loc/vehicle_state", 10, &VehicleControl::CallbackVehicleState, this);
    s_command_steer_ = nh.subscribe(
            "app/con/gripped_steer", 10, &VehicleControl::CallbackCommandSteer, this);             
    s_command_torque_ = nh.subscribe(
            "app/con/gripped_torque", 10, &VehicleControl::CallbackCommandTorque, this);
    s_command_speed_ = nh.subscribe(
            "hmi/con/target_speed", 10, &VehicleControl::CallbackCommandSpeed, this);
    s_command_accel_ = nh.subscribe(
            "app/con/gripped_accel", 10, &VehicleControl::CallbackCommandAccel, this);
    s_keyboard_command_ = nh.subscribe(
            "keyboard/vehicle_cmd", 10, &VehicleControl::CallbackKeyboardCommand, this);
    // Publisher init
    p_control_signal_ = nh.advertise<carmaker_msgs::Control_Signal>(
            "/carmaker/control_signal", 10);
    p_commands_ = nh.advertise<autohyu_msgs::VehicleCmd>(
            "bsw/can/vehicle_cmd", 1);

    i_command_steer_ = 0.0;
    i_command_torque_ = -10000.0;
    ProcessINI();
    ProcessRosparam(nh);
}

VehicleControl::~VehicleControl() {}

void VehicleControl::ProcessRosparam(const ros::NodeHandle& nh) {
    // ROS param init
    // if (!nh.getParam("/object_driver/ref_lat", cfg_.ref_lat)) {
    //     cfg_.ref_lat = 0.0;
    // }
    // ROS_WARN_STREAM("Reference Lat: " << cfg_.ref_lat << ", Lon: " << cfg_.ref_lon);
}

void VehicleControl::Run() {
    ProcessINI();
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    VehicleState vehicle_state; {
        std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
        vehicle_state = i_vehicle_state_;
    }
    double command_steer = 0.0; {
        std::lock_guard<std::mutex> lock(mutex_command_steer_);
        command_steer = i_command_steer_;
    }
    double command_torque = -10000.0; {
        std::lock_guard<std::mutex> lock(mutex_command_torque_);
        command_torque = i_command_torque_;
    }
    double command_speed = 0.0; {
        std::lock_guard<std::mutex> lock(mutex_command_speed_);
        command_speed = i_command_speed_;
    }
    double command_accel = 0.0; {
        std::lock_guard<std::mutex> lock(mutex_command_accel_);
        command_accel = i_command_accel_;
        if(command_accel > 10.1 ){
            command_accel = 10.1;
        }
        else if(command_accel  < -10.1){
            command_accel  = -10.1;
        }
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //    
    // calculate latency adapted command   
    std::tuple<double, double, double, double> updated_command;
    if (cfg_.use_keyboard_input == true) {
        VehicleCmd keyboard_command; {
            std::lock_guard<std::mutex> lock(mutex_keyboard_command_);
            keyboard_command = i_keyboard_command_;
        }
        updated_command = std::make_tuple(keyboard_command.steering_angle*DEG2RAD, keyboard_command.gas, keyboard_command.brake, keyboard_command.accel);

        o_vehicle_cmd_.header.stamp = ros::Time(vehicle_state.header.stamp);
        o_vehicle_cmd_.steering_angle = keyboard_command.steering_angle;
        o_vehicle_cmd_.gas = keyboard_command.gas;
        o_vehicle_cmd_.brake = keyboard_command.brake;
        o_vehicle_cmd_.accel = keyboard_command.accel;
    }
    else{
        if (vehicle_state.vx < 0.0 && command_torque <= T_NEUTRAL_CM) {
            updated_command = std::make_tuple(command_steer, 0.0, 1.0, command_accel);
        }    
        else if (std::isnan(command_torque) == false && std::isnan(command_steer) == false) {
            updated_command = CalculateCommand(vehicle_state, 
                                                command_accel,
                                                command_torque,
                                                command_speed, 
                                                command_steer);
        }
        else if (std::isnan(command_torque) == true) {
            updated_command = std::make_tuple(command_steer, 0.0, 1.0, command_accel);
        }
        else {
            updated_command = std::make_tuple(0.0, 0.0, 1.0, command_accel);
        }
    }



    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //    
    UpdateCommand(std::get<0>(updated_command), std::get<1>(updated_command), std::get<2>(updated_command), std::get<3>(updated_command));
}

void VehicleControl::Publish() {
    p_control_signal_.publish(o_control_signal_);
    p_commands_.publish(o_vehicle_cmd_);
}

void VehicleControl::ProcessINI() {
    if (util_ini_parser_.IsFileUpdated()) { 
        util_ini_parser_.ParseConfig("Carmaker Vehicle control", "use_keyboard_input", cfg_.use_keyboard_input);  
        ROS_INFO("Ini file is updated!");
    }
}

std::tuple<double, double, double, double> VehicleControl::CalculateCommand(const VehicleState& vehicle_state, 
                                                                    double& command_accel,
                                                                    const double& command_torque,
                                                                    const double& command_speed, 
                                                                    const double& command_steer) {
    double front_tire_steer_deg = command_steer;
    double gas = 0.;
    double brake = 0.;
    
    if(command_torque >= T_NEUTRAL_CM){       
        double limit_trq = min(G_L1*pow(vehicle_state.vx*MPS2KPH,4)
                             + G_L2*pow(vehicle_state.vx*MPS2KPH,3)
                             + G_L3*pow(vehicle_state.vx*MPS2KPH,2)
                             + G_L4*vehicle_state.vx*MPS2KPH
                             + G_L5 , T_UL);

        // if(command_torque > limit_trq){
        //     command_torque = limit_trq;
        // }

        gas = min(1.0, GAS_1* command_torque + GAS_2);
        brake = 0.0;
    }
    else if(command_torque < T_NEUTRAL_CM) {
        gas = 0.0;
        brake = min(1.0, BRAKE_1_CM* command_torque + BRAKE_2_CM);
    }

    if(vehicle_state.vx < 0.1 && command_accel < 0.0){
        command_accel = 0.0;
    }

    o_vehicle_cmd_.header.stamp = ros::Time(vehicle_state.header.stamp);
    o_vehicle_cmd_.steering_angle = front_tire_steer_deg*STEERING_RATIO_CARMAKER;
    o_vehicle_cmd_.front_tire_angle = front_tire_steer_deg;
    o_vehicle_cmd_.speed = command_speed;
    o_vehicle_cmd_.accel = command_accel;
    o_vehicle_cmd_.torque = command_torque;
    o_vehicle_cmd_.gas = 100.0*gas;
    o_vehicle_cmd_.brake = 100.0*brake;

    return std::make_tuple(front_tire_steer_deg*DEG2RAD*STEERING_RATIO_CARMAKER, gas, brake, command_accel);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
// Update functions for publish variables
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
void VehicleControl::UpdateCommand(const double& steer, const double& gas, const double& brake, const double& accel) {
    // Update command    
    o_control_signal_.steerangle = steer;
    o_control_signal_.gas        = gas;
    o_control_signal_.brake      = brake;
    // o_control_signal_.accel      = accel;
}

int main(int argc, char** argv) {
    std::string node_name = "vehicle_control";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_vehicle_control", period)){
        period = 1.0;
    }

    VehicleControl main_task(node_name, period);
    main_task.Exec();

    return 0;
}