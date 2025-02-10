/**
 * @file        motion_prediction_node.cpp
 * @brief       motion prediction node cpp file to predict surrounding vehicles' motion
 * 
 * @authors     Yuseung Na (ys.na0220@gmail.com)          
 *              Seounghoon Park (sunghoon8585@gmail.com)
 * 
 * @date        2024-04-13 created by Yuseung Na
 *              2024-12-11 change to get vils object by PSH
 * 
 */

#include "motion_prediction_node.hpp"
// #include <random>

MotionPrediction::MotionPrediction(std::string node_name, double period)
    : TaskManager(node_name, period) {
    // Initialize
    ROS_WARN_STREAM("[" << node_name << "] Initialize node (Period: " << 1/period << " Hz)");

    // Node init
    ros::NodeHandle nh;

    // Ini init
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/perception.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());

    // Parameter init
    ProcessINI();
    ProcessRosparam(nh);

    // Subscriber init
    // TODO: Subscribe Lanelet Map for Generate Reference Trajectory of Each Objects
    s_vehicle_state_ = nh.subscribe(
        "app/loc/vehicle_state", 10, &MotionPrediction::CallbackVehicleState, this);
    s_track_objects_ = nh.subscribe(
        "/bsw/vils/track_objects", 10, &MotionPrediction::CallbackTrackObjects, this);
    s_trajectory_ = nh.subscribe(
        "/app/pla/trajectory", 10, &MotionPrediction::CallbackTrajectory, this);
    // s_map_bin_       = nh.subscribe(
    //     "app/pla/map", 10, &MotionPrediction::CallbackMapBin, this);

    // Publisher init
    p_predict_objects_      = nh.advertise<autohyu_msgs::PredictObjects>(
        "app/perc/predict_objects", 10);
    p_rviz_predict_objects_ = nh.advertise<visualization_msgs::MarkerArray>(
        "hmi/perc/predict_objects", 10);
    p_map_request_          = nh.advertise<std_msgs::Header>(
        "app/pla/map_request", 1);

    // Algorithm init
    alg_preprocess_ = std::make_unique<Preprocess>();
    alg_physics_based_ = std::make_unique<PhysicsBased>();
    // alg_maneuver_based_ = std::make_unique<ManeuverBased>();
    // alg_planning_based_ = std::make_unique<PlanningBased>();
}

MotionPrediction::~MotionPrediction() {
    // Terminate
}

void MotionPrediction::Run() {
    ProcessINI();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //    
    // if (!b_is_map_){
    //     ROS_ERROR_THROTTLE(1.0, "Wait for Map...");
    //     std_msgs::Header header;
    //     header.seq      =   request_num_;
    //     header.frame_id =   "motion_prediction";
    //     header.stamp    =   ros::Time::now();
    //     p_map_request_.publish(header);
    //     request_num_ += 1;
    //     return;
    // }
    if (!b_is_vehicle_state_) {
        ROS_ERROR_THROTTLE(1.0, "Wait for Vehicle State...");
        return;
    }
    if (!b_is_track_objects_) {
        ROS_ERROR_THROTTLE(1.0, "Wait for Track Objects...");
        return;
    }
    if (!b_is_trajectory_) {
        ROS_ERROR_THROTTLE(1.0, "Wait for Trajectory...");
        return;
    }

    
    // lanelet::LaneletMapPtr map_lanelet; 
    // uint32_t map_seq; {
    //     mutex_map_bin_.lock();
    //     map_lanelet = i_map_lanelet_;
    //     map_seq = i_map_seq_;
    //     mutex_map_bin_.unlock();
    // }
    interface::VehicleState vehicle_state; {
        mutex_vehicle_state_.lock();
        vehicle_state = i_vehicle_state_;
        mutex_vehicle_state_.unlock();
    }
    interface::TrackObjects track_objects; {
        mutex_track_objects_.lock();
        track_objects = i_track_objects_;
        mutex_track_objects_.unlock();
    }
    interface::Trajectory trajectory; {
        mutex_trajectory_.lock();
        trajectory = i_trajectory_;
        mutex_trajectory_.unlock();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    // Pre-process
    // interface::PredictObjects processed_objects
    //    = alg_preprocess_->RunAlgorithm(vehicle_state.header.stamp, track_objects, map_lanelet , map_seq, cfg_);
    interface::PredictObjects processed_objects = alg_preprocess_->RunAlgorithm(
        vehicle_state.header.stamp, track_objects, trajectory, cfg_);

    // Predict motion 
    interface::PredictObjects o_predict_objects;
    switch (cfg_.prediction_algorithm) {
        // Physics-based prediction
        case 0: default:
            o_predict_objects = alg_physics_based_->RunAlgorithm(processed_objects, cfg_);
            break;
        // // Manuever-based prediction
        // case 1:
        //     o_predict_objects = alg_maneuver_based_->RunAlgorithm(
        //         vehicle_state, processed_objects, cfg_);
        //     break;
        // // Planning-based prediction
        // case 2:
        //     o_predict_objects = alg_planning_based_->RunAlgorithm(
        //         vehicle_state, processed_objects, cfg_);
        //     break;
    }
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    o_predict_objects_ = ros_bridge::UpdatePredictObjects(o_predict_objects);
    o_rviz_predict_objects_ = ros_bridge::UpdateRvizPredictObjects(o_predict_objects, 150*KPH2MPS);
}


void MotionPrediction::Publish() {
    if (!b_is_vehicle_state_ || !b_is_track_objects_) {    
        return;
    }
    p_predict_objects_.publish(o_predict_objects_);
    p_rviz_predict_objects_.publish(o_rviz_predict_objects_);
}

void MotionPrediction::ProcessRosparam(const ros::NodeHandle& nh) {

}

void MotionPrediction::ProcessINI() {
    util_ini_parser_.ParseConfig("MotionPrediction", "dt", cfg_.dt);
    util_ini_parser_.ParseConfig("MotionPrediction", "prediction_horizon", cfg_.prediction_horizon);
    util_ini_parser_.ParseConfig("MotionPrediction", "prediction_algorithm", cfg_.prediction_algorithm);   
    util_ini_parser_.ParseConfig("MotionPrediction", "track_buffer_size", cfg_.track_buffer_size);
    util_ini_parser_.ParseConfig("MotionPrediction", "track_erase_time_sec", cfg_.track_erase_time_sec);
    util_ini_parser_.ParseConfig("MotionPrediction", "track_filter_alpha", cfg_.track_filter_alpha);
    util_ini_parser_.ParseConfig("MotionPrediction", "yawrate_zero_vel_ms", cfg_.yawrate_zero_vel_ms);

    // Physics-based
    util_ini_parser_.ParseConfig("PhysicsBased", "physics_model", cfg_.physics_model);

    // Maneuver-based
    util_ini_parser_.ParseConfig("ManeuverBased", "longitudinal_model", cfg_.longitudinal_model);
    util_ini_parser_.ParseConfig("ManeuverBased", "lateral_model", cfg_.lateral_model);

    // Planning-based
    util_ini_parser_.ParseConfig("PlanningBased", "dr_model", cfg_.dr_model);
    util_ini_parser_.ParseConfig("PlanningBased", "adaptive_k_gain", cfg_.adaptive_k_gain);
    util_ini_parser_.ParseConfig("PlanningBased", "max_k_gain", cfg_.max_k_gain);
    util_ini_parser_.ParseConfig("PlanningBased", "min_k_gain", cfg_.min_k_gain);
    util_ini_parser_.ParseConfig("PlanningBased", "max_curvature", cfg_.max_curvature);
    util_ini_parser_.ParseConfig("PlanningBased", "min_curvature", cfg_.min_curvature);
    util_ini_parser_.ParseConfig("PlanningBased", "k_gain", cfg_.k_gain);
    util_ini_parser_.ParseConfig("PlanningBased", "ks_gain", cfg_.ks_gain);
    util_ini_parser_.ParseConfig("PlanningBased", "max_ax", cfg_.max_ax);
    util_ini_parser_.ParseConfig("PlanningBased", "max_ay", cfg_.max_ay);
    util_ini_parser_.ParseConfig("PlanningBased", "max_wheel_angle_deg", cfg_.max_wheel_angle_deg);
    util_ini_parser_.ParseConfig("PlanningBased", "max_delta_wheel_angle_deg", cfg_.max_delta_wheel_angle_deg);
}

int main(int argc, char** argv) {
    std::string node_name = "motion_prediction";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_motion_prediction", period)) {
        period = 1.0;
    }

    MotionPrediction main_task(node_name, period);
    main_task.Exec();

    return 0;
}