/**
 * @file        grip_manager_node_node.cpp
 * @brief       
 * 
 * @authors     Junhee Lee (998jun@gmail.com)         
 * 
 * @date        2023-10-04 created by Junhee Lee
 * 
 */

#include "grip_manager_node.hpp"

GripManagerNode::GripManagerNode(std::string node_name, double period)
    : TaskManager(node_name, period) {
    // Initialize
    ROS_WARN_STREAM("[" << node_name << "] Initialize node (Period: " << 1/period << " Hz)");

    // Node init
    ros::NodeHandle nh;

    // Ini init
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/control.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());

    // Parameter init
    ProcessINI();
    ProcessRosparam(nh);

    // Subscriber init
    s_vehicle_state_ = nh.subscribe(
        "app/loc/vehicle_state", 10, &GripManagerNode::CallbackVehicleState, this);
    s_target_steering_ = nh.subscribe(
        "app/con/command_steer", 10, &GripManagerNode::CallbackTargetSteering, this);
    s_target_torque_ = nh.subscribe(
        "app/con/command_torque", 10, &GripManagerNode::CallbackTargetTorque, this);
    s_target_accel_ = nh.subscribe(
        "app/con/command_accel", 10, &GripManagerNode::CallbackTargetAccel, this);
    

    // Publisher init
    p_gripped_steering_ = nh.advertise<std_msgs::Float32>(
        "app/con/gripped_steer",  10);
    p_gripped_torque_ = nh.advertise<std_msgs::Float32>(
        "app/con/gripped_torque", 10);
    p_gripped_accel_ = nh.advertise<std_msgs::Float32>(
        "app/con/gripped_accel",  10);  
    

    // Algorithm init
    ptr_grip_manager  = make_unique<GripManager>();

}

GripManagerNode::~GripManagerNode() {
    // Terminate
}

void GripManagerNode::UpdateCommand(const double& steering_tire_angle_deg, const double& torque, const double& accel){
    o_gripped_steering_.data = steering_tire_angle_deg;
    o_gripped_torque_.data = torque;
    o_gripped_accel_.data = accel;
}

void GripManagerNode::Run() {
    ProcessINI();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if (!b_is_vehicle_state_) {
        ROS_ERROR_THROTTLE(1.0,"Wait for Vehicle State...");
        return;
    }


    interface::VehicleState vehicle_state; {
        mutex_vehicle_state_.lock();
        vehicle_state = i_vehicle_state_;
        mutex_vehicle_state_.unlock();
    }
    double target_steering_tire_angle_rad; {
        mutex_target_steering_.lock();
        target_steering_tire_angle_rad = i_target_steering_ * DEG2RAD;
        mutex_target_steering_.unlock();
    }
    double target_torque; {
        mutex_target_torque_.lock();
        target_torque = i_target_torque_;
        mutex_target_torque_.unlock();
    }
    double target_accel; {
        mutex_target_accel_.lock();
        target_accel = i_target_accel_;
        mutex_target_accel_.unlock();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    // Algorithm 1
    const auto [steering_tire_angle_deg, wheel_torque, accel] =
        ptr_grip_manager->RunAlgorithm(vehicle_state, target_steering_tire_angle_rad, target_torque, target_accel, cfg_);

    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    UpdateCommand(steering_tire_angle_deg, wheel_torque, accel);
}

void GripManagerNode::Publish() {
    if (!b_is_vehicle_state_) {
        return;
    }
    p_gripped_steering_.publish(o_gripped_steering_);
    p_gripped_torque_.publish(o_gripped_torque_);
    p_gripped_accel_.publish(o_gripped_accel_);
}

void GripManagerNode::ProcessRosparam(const ros::NodeHandle& nh) {
    
}


void GripManagerNode::ProcessINI() {
    if (util_ini_parser_.IsFileUpdated()){        
        util_ini_parser_.ParseConfig("Grip Manager", "friction_coeff" , cfg_.friction_coeff);
		util_ini_parser_.ParseConfig("Grip Manager", "use_steering_limiter" , cfg_.use_steering_limiter);
		util_ini_parser_.ParseConfig("Grip Manager", "max_front_tire_slip_angle" , cfg_.max_front_tire_slip_angle);

		util_ini_parser_.ParseConfig("Grip Manager", "use_torque_limiter" , cfg_.use_torque_limiter);

		util_ini_parser_.ParseConfig("Grip Manager", "use_tire_slip_rear_angle_based_long_acc_limit" , cfg_.use_tire_slip_rear_angle_based_long_acc_limit);
		util_ini_parser_.ParseConfig("Grip Manager", "tire_slip_rear_start_angle_deg" , cfg_.tire_slip_rear_start_angle_deg  );
		util_ini_parser_.ParseConfig("Grip Manager", "tire_slip_rear_end_angle_deg" ,   cfg_.tire_slip_rear_end_angle_deg      );
		util_ini_parser_.ParseConfig("Grip Manager", "tire_slip_rear_start_accel" ,     cfg_.tire_slip_rear_start_accel          );
		util_ini_parser_.ParseConfig("Grip Manager", "tire_slip_rear_end_accel" ,       cfg_.tire_slip_rear_end_accel               );

        util_ini_parser_.ParseConfig("Grip Manager", "use_acc_based_long_acc_limit" , cfg_.use_acc_based_long_acc_limit);
		util_ini_parser_.ParseConfig("Grip Manager", "maximum_accel" , cfg_.maximum_accel);
		util_ini_parser_.ParseConfig("Grip Manager", "lat_acc_window_size" , cfg_.lat_acc_window_size);
		util_ini_parser_.ParseConfig("Grip Manager", "longitudinal_accel_lower_bound" , cfg_.longitudinal_accel_lower_bound);


		std::cout<<"[Grip Manager] Ini file is updated!"<<std::endl;
    }
}



int main(int argc, char** argv) {
    std::string node_name = "grip_manager";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_grip_manager", period)) {
        period = 1.0;
    }

    GripManagerNode main_task(node_name, period);
    main_task.Exec();

    return 0;
}