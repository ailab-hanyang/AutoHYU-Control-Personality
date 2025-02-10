

#include "delayed_state_node.hpp"

DelayedStateNode::DelayedStateNode(std::string task_node, double period):
    TaskManager(task_node, period),
    ptr_delayed_state_algorithm_(NULL) {

    Init();
}

DelayedStateNode::~DelayedStateNode() {}

void DelayedStateNode::Init() {
    // Node init
    NodeHandle nh;

    // Ini init
    std::string dir(ros::package::getPath("delayed_state") + "/../../../../");
    std::string ini_path("/config/localization.ini");
    util_ini_parser_.Init((dir + ini_path).c_str());

    // Parameter init
    ProcessINI();

    double localization_period;
    // Algorithm Init
    if ( !nh.getParam("task_period/period_state_estimation", localization_period) ) {
        localization_period = 0.01;
    }
    
    ptr_delayed_state_algorithm_.reset(new DelayedStateAlgorithm());
    ptr_delayed_state_algorithm_->Init(localization_period);
        

    // ROS param init
    // if ( !nh.getParam("/state_estimation/ref_lat", params_.d_ref_latitude) ) {
    //     params_.d_ref_latitude = 0.0;
    // }

    // Subscriber init
    s_vehicle_state_     = nh.subscribe("/app/loc/vehicle_state", 10, &DelayedStateNode::CallbackVehicleState, this);
    
    // Publisher init
    p_delayed_state_ = nh.advertise<autohyu_msgs::VehicleState>(
        "/app/loc/delayed_state", 10);


}

void DelayedStateNode::Run() {

    ProcessINI();

    if (!b_is_vehicle_state_) {
        ROS_ERROR_THROTTLE(1.0,"Wait for Vehicle State...");
        return;
    }
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    std::vector<VehicleState> vehicle_state_vec;
    {
        std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
        vehicle_state_vec = i_vehicle_state_vec_;
    }
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update variables & Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    //// Current time estimation
    ros::Time        curr_ros_time                  = ros::Time::now();
    static ros::Time prev_ros_time                  = ros::Time::now();
    // actually not used.. TODO : use ros time

    
    //// Update vehicle state
    if ( ptr_delayed_state_algorithm_ == NULL ) {
        ROS_ERROR_STREAM("[DelayedState] DelayedState Algorithm is not init");
        return;
    }
    
    VehicleState delayed_vehicle_state;
    delayed_vehicle_state    = ptr_delayed_state_algorithm_->MakeNoisyDelayedState(vehicle_state_vec, params_);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    UpdateDelayedState(delayed_vehicle_state);

    
    // clear input buf
    {
        std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
        i_vehicle_state_vec_.clear();
    }

    return;
}

void DelayedStateNode::Publish() {
    p_delayed_state_.publish(o_delayed_state_);
}

void DelayedStateNode::Terminate() {}

void DelayedStateNode::ProcessINI() {
    if ( util_ini_parser_.IsFileUpdated() ) {

        util_ini_parser_.ParseConfig("Delayed State", "cfg_delay_time_s",           params_.cfg_delay_time_s);
        
        util_ini_parser_.ParseConfig("Delayed State", "cfg_noise_std_dev_x",        params_.cfg_noise_std_dev_x);
        util_ini_parser_.ParseConfig("Delayed State", "cfg_noise_std_dev_y",        params_.cfg_noise_std_dev_y);
        util_ini_parser_.ParseConfig("Delayed State", "cfg_noise_std_dev_yaw",      params_.cfg_noise_std_dev_yaw);
        util_ini_parser_.ParseConfig("Delayed State", "cfg_noise_std_dev_vx",       params_.cfg_noise_std_dev_vx);
        util_ini_parser_.ParseConfig("Delayed State", "cfg_noise_std_dev_vy",       params_.cfg_noise_std_dev_vy);
        util_ini_parser_.ParseConfig("Delayed State", "cfg_noise_std_dev_yaw_vel",  params_.cfg_noise_std_dev_yaw_vel);
        
        ROS_WARN("[Delayed State] Ini file is updated!");
    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
// Update functions for publish variables
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
void DelayedStateNode::UpdateDelayedState(const VehicleState& vehicle_state) {
    
    o_delayed_state_ = ros_bridge::UpdateVehicleState(vehicle_state);
    return;
}



int main(int argc, char** argv) {
    // Initialize node
    std::string node_name = "delayed_state";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if ( !nh.getParam("task_period/period_delayed_state", period) ) {
        period = 1.0;
    }

    // Exec algorithm
    DelayedStateNode main_task(node_name, period);
    main_task.Exec();

    return 0;
}
