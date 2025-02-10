/**
 * @file        template_node_node.cpp
 * @brief       template node cpp file for template node
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-04-01 created by Yuseung Na
 * 
 */

#include "template_node_node.hpp"

TemplateNode::TemplateNode(std::string node_name, double period)
    : TaskManager(node_name, period) {
    // Initialize
    ROS_WARN_STREAM("[" << node_name << "] Initialize node (Period: " << 1/period << " Hz)");

    // Node init
    ros::NodeHandle nh;

    // Ini init
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/template.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());

    // Parameter init
    ProcessINI();
    ProcessRosparam(nh);

    // Subscriber init
    s_vehicle_state_ = nh.subscribe(
        "app/loc/vehicle_state", 10, &TemplateNode::CallbackVehicleState, this);
    s_detect_objects_ = nh.subscribe(
        "app/perc/detect_objects", 10, &TemplateNode::CallbackDetectObjects, this);
    s_track_objects_ = nh.subscribe(
        "app/perc/track_objects", 10, &TemplateNode::CallbackTrackObjects, this);
    s_predict_objects_ = nh.subscribe(
        "app/perc/predict_objects", 10, &TemplateNode::CallbackPredictObjects, this);        
    s_trajectory_ = nh.subscribe(
        "app/pla/trajectory", 10, &TemplateNode::CallbackTrajectory, this);

    // Publisher init
    p_predict_objects_ = nh.advertise<autohyu_msgs::Trajectory>(
        "app/pla/trajectory", 10);

    // Algorithm init
    alg_physics_based_ = std::make_unique<TemplateAlgorithm>();
}

TemplateNode::~TemplateNode() {
    // Terminate
}

void TemplateNode::Run() {
    ProcessINI();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if (!b_is_vehicle_state_) {
        ROS_ERROR_THROTTLE(1.0,"Wait for Vehicle State...");
        return;
    }
    if (!b_is_trajectory_) {
        ROS_ERROR_THROTTLE(1.0,"Wait for Trajectory...");
        return;
    }

    interface::VehicleState vehicle_state; {
        mutex_vehicle_state_.lock();
        vehicle_state = i_vehicle_state_;
        mutex_vehicle_state_.unlock();
    }
    interface::DetectObjects3D detect_objects; {
        mutex_detect_objects_.lock();
        detect_objects = i_detect_objects_;
        mutex_detect_objects_.unlock();
    }
    interface::TrackObjects track_objects; {
        mutex_track_objects_.lock();
        track_objects = i_track_objects_;
        mutex_track_objects_.unlock();
    }
    interface::PredictObjects predict_objects; {
        mutex_predict_objects_.lock();
        predict_objects = i_predict_objects_;
        mutex_predict_objects_.unlock();
    }
    interface::Trajectory trajectory; {
        mutex_trajectory_.lock();
        trajectory = i_trajectory_;
        mutex_trajectory_.unlock();
    }    

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    // Algorithm 1
    interface::Trajectory o_trajectory = alg_physics_based_->RunAlgorithm(
        vehicle_state, detect_objects, track_objects, predict_objects, trajectory, cfg_);

    // Algorithm 2
    // ...
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    o_trajectory_ = ros_bridge::UpdateTrajectory(o_trajectory);
}

void TemplateNode::Publish() {
    if (!b_is_vehicle_state_ || !b_is_trajectory_) {
        return;
    }
    
    p_predict_objects_.publish(o_trajectory_);
}

void TemplateNode::ProcessRosparam(const ros::NodeHandle& nh) {
    nh.getParam("/template_node/mode",                        cfg_.mode);
    nh.getParam("/template_node/location",                    cfg_.location);
    nh.getParam("/map_file/"  + cfg_.location,                cfg_.map);
    cfg_.map = getenv("PWD")  + cfg_.map;
    nh.getParam("/reference/" + cfg_.location + "/latitude",  cfg_.reference_latitude);
    nh.getParam("/reference/" + cfg_.location + "/longitude", cfg_.reference_longitude);

    ROS_WARN_STREAM("\nMode: " << cfg_.mode 
                 << "\nLocation: " << cfg_.location 
                 << "\nMap: " << cfg_.map
                 << "\nReference Latitude: " << cfg_.reference_latitude << ", Longitude: " << cfg_.reference_longitude);
}

void TemplateNode::ProcessINI() {
    util_ini_parser_.ParseConfig("TemplateNode", "config1", cfg_.node_config1);
    util_ini_parser_.ParseConfig("TemplateNode", "config2", cfg_.node_config2);
    util_ini_parser_.ParseConfig("TemplateNode", "config3", cfg_.node_config3);

    util_ini_parser_.ParseConfig("Algorithm1", "config1", cfg_.algorithm1_config1);
    util_ini_parser_.ParseConfig("Algorithm1", "config2", cfg_.algorithm1_config2);
    util_ini_parser_.ParseConfig("Algorithm1", "config3", cfg_.algorithm1_config3);

    util_ini_parser_.ParseConfig("Algorithm2", "config1", cfg_.algorithm2_config1);
    util_ini_parser_.ParseConfig("Algorithm2", "config2", cfg_.algorithm2_config2);
    util_ini_parser_.ParseConfig("Algorithm2", "config3", cfg_.algorithm2_config3);
}

int main(int argc, char** argv) {
    std::string node_name = "template_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_template_node", period)) {
        period = 1.0;
    }

    TemplateNode main_task(node_name, period);
    main_task.Exec();

    return 0;
}