/**
 * @file        state_estimation_node.cpp
 * @brief       node cpp file for state estimation node
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)         
 * 
 * @date        2024-04-11 created by Yuseung Na
 * 
 */

#include "state_estimation_node.hpp"

StateEstimation::StateEstimation(std::string node_name, double period)
    : TaskManager(node_name, period) {
    // Initialize
    ROS_WARN_STREAM("[" << node_name << "] Initialize node (Period: " << 1/period << " Hz)");

    // Node init
    ros::NodeHandle nh;

    // Ini init
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/localization.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());

    // Parameter init
    ProcessINI();    
    ProcessRosparam(nh);

    // Subscriber init
    s_novatel_inspvax_ = nh.subscribe(
        "/novatel/oem7/inspvax", 10, &StateEstimation::CallbackNovatelInspvax, this);
    s_novatel_corrimu_ = nh.subscribe(
        "/novatel/oem7/corrimu", 10, &StateEstimation::CallbackNovatelCorrimu, this);
    s_vehicle_can_ = nh.subscribe(
        "/bsw/vehicle_can", 10, &StateEstimation::CallbackVehicleCAN, this);
    
    // Publisher init
    p_vehicle_state_ = nh.advertise<autohyu_msgs::VehicleState>(
        "app/loc/vehicle_state", 10);
    p_reference_point_ = nh.advertise<autohyu_msgs::Reference>(
        "app/loc/reference_point", 10);

    // Algorithm init
    alg_state_using_novatel_ = std::make_unique<StateUsingNovatel>();
}

StateEstimation::~StateEstimation() {
    // Terminate
}

void StateEstimation::Run() {
    ProcessINI();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if (!b_is_novatel_inspavx_) {
        ROS_ERROR("Wait for Novatel INSPVAX...");
        return;
    }
    if (!b_is_novatel_corrimu_) {
        ROS_ERROR("Wait for Novatel CORRIMU...");
        return;
    }
    if (!b_is_vehicle_can_) {
        ROS_ERROR("Wait for Vehicle CAN...");
        return;
    }

    interface::INSPVAX inspvax; {
        mutex_novatel_inspvax_.lock();
        inspvax = i_novatel_inspvax_;
        mutex_novatel_inspvax_.unlock();
    }
    interface::CORRIMU corrimu; {
        mutex_novatel_corrimu_.lock();
        corrimu = i_novatel_corrimu_;
        mutex_novatel_corrimu_.unlock();
    }
    interface::VehicleCAN vehicle_can; {
        mutex_vehicle_can_.lock();
        vehicle_can = i_vehicle_can_;
        mutex_vehicle_can_.unlock();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    // Algorithm 1
    interface::VehicleState o_vehicle_state 
        = alg_state_using_novatel_->RunAlgorithm(inspvax, corrimu, vehicle_can, cfg_);
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    o_vehicle_state_   = ros_bridge::UpdateVehicleState(o_vehicle_state);
    o_reference_point_ = ros_bridge::UpdateReference(o_vehicle_state.reference);
}

void StateEstimation::Publish() {
    if (!b_is_novatel_inspavx_ || !b_is_novatel_corrimu_) {
        return;
    }
    
    p_vehicle_state_.publish(o_vehicle_state_);
    p_reference_point_.publish(o_reference_point_);
}

void StateEstimation::ProcessRosparam(const ros::NodeHandle& nh) {
    nh.getParam("/state_estimation/mode",          cfg_.mode);    
    nh.getParam("/state_estimation/location",      cfg_.location);
    nh.getParam("/state_estimation/ref_latitude",  cfg_.reference_latitude);
    nh.getParam("/state_estimation/ref_longitude", cfg_.reference_longitude);

    ROS_WARN_STREAM("\nMode: " << cfg_.mode
                 << "\nLocation: " << cfg_.location
                 << "\nReference Latitude: " << cfg_.reference_latitude << ", Longitude: " << cfg_.reference_longitude);
}

void StateEstimation::ProcessINI() {
    util_ini_parser_.ParseConfig("StateUsingNovatel", "projector", cfg_.projector);
}

int main(int argc, char** argv) {
    std::string node_name = "state_estimation";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_state_estimation", period)) {
        period = 1.0;
    }

    StateEstimation main_task(node_name, period);
    main_task.Exec();

    return 0;
}