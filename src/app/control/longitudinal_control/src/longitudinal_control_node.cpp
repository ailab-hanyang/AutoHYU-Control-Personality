/**
 * @file        longitudinal_control.cpp
 * @brief       longitudinal control node
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)    
 *              Junhee Lee (998jun@gmail.com)      
 * 
 * @date        2023-02-02 Yuseung Na - Created. (Longitudinal MPC using CVXGEN)
 *              2023-03-10 Junhee Lee - Add dynamic MPC model
 *              2023-04-12 Junhee Lee - Add dynamic MPC du model (HPIPM)
 *              2023-08-06 Junhee Lee - Code Cleanup
 */

#include "longitudinal_control_node.hpp"

LongitudinalControlNode::LongitudinalControlNode(std::string task_node, double period)
    : TaskManager(task_node, period) {
    // Initialize
    ROS_WARN_STREAM("[" << task_node << "] Initialize node (Period: " << 1/period << " Hz)");

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
        "app/loc/vehicle_state", 10, &LongitudinalControlNode::CallbackVehicleState, this);
    if(b_use_predicted_speed_){
        s_trajectory_ = nh.subscribe(
            "app/con/predicted_trajectory", 10, &LongitudinalControlNode::CallbackTrajectory, this);
    }else{
        s_trajectory_ = nh.subscribe(
            "app/pla/trajectory", 10, &LongitudinalControlNode::CallbackTrajectory, this);
    }

    // Publisher init
    p_command_torque_           = nh.advertise<std_msgs::Float32>("app/con/command_torque", 10);
    p_command_acceleration_     = nh.advertise<std_msgs::Float32>("app/con/command_accel", 10);  
    
    p_target_speed_             = nh.advertise<std_msgs::Float32>("hmi/con/target_speed", 10);
    p_target_ax_                = nh.advertise<std_msgs::Float32>("hmi/con/target_ax", 10);
    p_current_speed_            = nh.advertise<std_msgs::Float32>("hmi/con/current_speed", 10);
    p_current_speederror_       = nh.advertise<std_msgs::Float32>("hmi/con/current_speederror", 10);
    p_current_axerror_          = nh.advertise<std_msgs::Float32>("hmi/con/current_axerror", 10);
    p_compensated_torque_       = nh.advertise<std_msgs::Float32>("hmi/con/compensated_torque", 10);
    p_speed_compensation_error_ = nh.advertise<std_msgs::Float32>("hmi/con/speed_comp_error", 10);

    p_predicted_speed_profile_  = nh.advertise<visualization_msgs::MarkerArray>("hmi/con/control_speed_profile", 10);

    // Algorithm init
    ptr_longitudinal_control_ = std::make_unique<LongitudinalControl>();
    ptr_longitudinal_control_->Init();
}

LongitudinalControlNode::~LongitudinalControlNode() {
    // Terminate
}

void LongitudinalControlNode::Run() {
    ProcessINI();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if (!b_is_vehicle_state_) {
        ROS_ERROR("Wait for Vehicle State...");
        return;
    }
    if (!b_is_trajectory_) {
        ROS_ERROR("Wait for Trajectory...");
        return;
    }

    interface::VehicleState vehicle_state; {
        mutex_vehicle_state_.lock();
        vehicle_state = i_vehicle_state_;
        mutex_vehicle_state_.unlock();
        d_time_stamp_ = vehicle_state.header.stamp;
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
    const auto [control_command, control_reference, speed_tracking_infos] = ptr_longitudinal_control_->Run (
        vehicle_state, trajectory);

    // Algorithm 2
    // ...
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    UpdateControlCommand(control_command);
    UpdateRvizPredictedSpeedProfile(control_command.control_trajectory);
    UpdateSpeedTrackingInfos(speed_tracking_infos,vehicle_state,control_reference);
}

void LongitudinalControlNode::Publish() {
    if (!b_is_vehicle_state_ || !b_is_trajectory_) {
        return;
    }
    
    p_command_torque_.publish(o_command_torque_);
    p_command_acceleration_.publish(o_command_acceleration_);

    p_predicted_speed_profile_.publish(o_predicted_speed_profile_);

    p_target_speed_.publish(o_target_speed_);
    p_target_ax_.publish(o_target_ax_);
    p_current_speed_.publish(o_current_speed_);
    p_current_axerror_.publish(o_current_accelerror_);
    p_current_speederror_.publish(o_current_speederror_);
    p_compensated_torque_.publish(o_compensated_torque_);
    p_speed_compensation_error_.publish(o_speed_compensation_error_);
}

void LongitudinalControlNode::ProcessRosparam(const ros::NodeHandle& nh) {
    // nh.getParam("/template_node/mode",                        cfg_.mode);
    // nh.getParam("/template_node/location",                    cfg_.location);
    // nh.getParam("/map_file/"  + cfg_.location,                cfg_.map);
    // cfg_.map = getenv("PWD")  + cfg_.map;
    // nh.getParam("/reference/" + cfg_.location + "/latitude",  cfg_.reference_latitude);
    // nh.getParam("/reference/" + cfg_.location + "/longitude", cfg_.reference_longitude);

    // ROS_WARN_STREAM("\nMode: " << cfg_.mode 
    //              << "\nLocation: " << cfg_.location 
    //              << "\nMap: " << cfg_.map
    //              << "\nReference Latitude: " << cfg_.reference_latitude << ", Longitude: " << cfg_.reference_longitude);
}

void LongitudinalControlNode::ProcessINI() {
    if ( util_ini_parser_.IsFileUpdated() ) {
        
        ROS_WARN("[Longitudinal control] Ini file is updated!");
        util_ini_parser_.ParseConfig("Low Level Control", "use_predicted_speed",  b_use_predicted_speed_  );

    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
// Update functions for publish variables
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
void LongitudinalControlNode::UpdateControlCommand(const ControlCommand& cmd){
    o_command_torque_.data       = cmd.trq;    
    o_command_acceleration_.data = cmd.ax;
}

void LongitudinalControlNode::UpdateRvizPredictedSpeedProfile(const ControlTrajectory& control_trajectory) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker      point_marker;
    geometry_msgs::Point            point;

    point_marker.header.frame_id = control_trajectory.frame_id;
    point_marker.header.stamp    = ros::Time(d_time_stamp_);
    point_marker.ns              = "control_trajectory";
    point_marker.action          = visualization_msgs::Marker::ADD;
    point_marker.type            = visualization_msgs::Marker::CYLINDER;
    point_marker.lifetime        = Duration(0.1);

    point_marker.scale.x = 0.15;
    point_marker.scale.y = 0.15;
    point_marker.scale.z = 0.01;

    point_marker.color.r = 1.0f;
    point_marker.color.g = 0.078f;
    point_marker.color.b = 0.57;
    point_marker.color.a = 0.3f;

    uint16_t marker_id = 0;
    for ( auto control_point : control_trajectory.control_point ) {
        point_marker.id              = marker_id++;
        point_marker.pose.position.x = control_point.x;
        point_marker.pose.position.y = control_point.y;
        point_marker.pose.position.z = control_point.vx *KPH2MPS/2;
        point_marker.scale.z         = control_point.vx *KPH2MPS;

        point_marker.pose.orientation.x = 0.0;
        point_marker.pose.orientation.y = 0.0;
        point_marker.pose.orientation.z = 0.0;
        point_marker.pose.orientation.w = 1.0;

        marker_array.markers.push_back(point_marker);
    }
    o_predicted_speed_profile_       = marker_array;
}

void LongitudinalControlNode::UpdateSpeedTrackingInfos(const PathTrackingInfos& tracking_errors,
                                            const VehicleState& vehicle_state,
                                            const ControlTrajectory& ref) {
    o_current_accelerror_.data = tracking_errors.accel_error;
    o_current_speederror_.data = tracking_errors.speed_error;
    o_compensated_torque_.data = tracking_errors.compensation_longitudinal;
    o_speed_compensation_error_.data = tracking_errors.speed_compensation_error;
    o_target_speed_.data       = ref.control_point[0].vx*MPS2KPH;
    o_target_ax_.data       = ref.control_point[0].ax;
    o_current_speed_.data      = sqrt(pow(vehicle_state.vx,2)+pow(vehicle_state.vy,2))*MPS2KPH;
}

int main(int argc, char** argv) {
    std::string node_name = "longitudinal_control";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    

    double period;
    if ( !nh.getParam("task_period/period_longitudinal_control", period) ) {
        period = 1.0;
    }

    LongitudinalControlNode main_task(node_name, period);
    main_task.Exec();

    return 0;
}