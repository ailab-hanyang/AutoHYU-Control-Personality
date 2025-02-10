/**
 * @file        lateral_control_node.cpp
 * @brief       lateral_control_node
 * 
 * @authors     Junhee Lee (998jun@gmail.com)      
 * 
 * @date        2023-02-02 Yuseung Na - Created. (ACELAB's Lateral MPC using NLOPT)
 *              2023-05-17 Junhee Lee - Add du kinematic MPC using HPIPM.
 *              2023-07-11 Junhee Lee - Add actuator aware kinematic MPC using HPIPM
 *              2023-07-20 Junhee Lee - Add time-delay considered MPC using HPIPM.
 *              2023-07-23 Junhee Lee - Add error dynamic MPC using HPIPM
 *              2023-07-30 Junhee Lee - Clean up lateral controller!
 *              2023-09-03 Junhee Lee - Undergradient + Yawrate Compensator
 *              2024-06-25 Junhee Lee - Remove Yawrate Compensator
 *              2024-10-25 Junhee Lee - Debugging topics --> Control info msg
 *              2024-10-28 Junhee Lee - Update acados controllers!
 *              2024-10-29 Junhee Lee - Unwarp reference trajectory
 */

#include "lateral_control_node.hpp"

LateralControlNode::LateralControlNode(std::string task_node, double period)
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

    // Subscriber init
    s_vehicle_state_ = nh.subscribe(
        "app/loc/vehicle_state", 10, &LateralControlNode::CallbackVehicleState, this);     
    
    if(b_use_predicted_path_){
        s_trajectory_ = nh.subscribe(
            "app/con/predicted_trajectory", 10, &LateralControlNode::CallbackTrajectory, this);
    }else{
        s_trajectory_ = nh.subscribe(
            "app/pla/trajectory", 10, &LateralControlNode::CallbackTrajectory, this);
    }

    // Publisher init
    p_command_steer_    = nh.advertise<std_msgs::Float32>("app/con/command_steer", 10);
    
    p_control_info_     = nh.advertise<autohyu_msgs::ControlInfo>("hmi/con/lat/control_info", 10);
    p_control_trajectory_     = nh.advertise<visualization_msgs::MarkerArray>("hmi/con/lat/prediction", 10);
    p_reference_path_   = nh.advertise<visualization_msgs::MarkerArray>("hmi/con/lat/reference", 10);
    p_current_understeer_gradient_ = nh.advertise<std_msgs::Float32>("hmi/con/lat/current_understeer_gradient", 10);
    // Algorithm init
    ptr_lateral_control_ = std::make_unique<LateralControl>();
    ptr_lateral_control_->Init();
}

LateralControlNode::~LateralControlNode() {
    // Terminate
}

void LateralControlNode::Run() {
    ProcessINI();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if (!b_is_vehicle_state_) {
        ROS_ERROR("[Lateral Control] Wait for Vehicle State...");
        return;
    }
    if (!b_is_trajectory_) {
        ROS_ERROR("[Lateral Control] Wait for Trajectory...");
        return;
    }

    interface::VehicleState vehicle_state; {
        mutex_vehicle_state_.lock();
        vehicle_state = i_vehicle_state_;
        d_time_stamp_ = vehicle_state.header.stamp;
        mutex_vehicle_state_.unlock();
    }
    interface::Trajectory trajectory; {
        mutex_trajectory_.lock();
        trajectory = i_trajectory_;
        mutex_trajectory_.unlock();
    }    

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    const auto [control_command, control_reference, path_tracking_infos] = ptr_lateral_control_->Run(
            vehicle_state, trajectory);
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    UpdateSteeringTireCommand(control_command.steering_tire_angle); //Deg
    UpdateRvizControlPrediction(control_command.control_trajectory);
    UpdateRvizControlReference(control_reference);
    UpdatePathTrackingInfos(path_tracking_infos);
}

// output msg update functions
void LateralControlNode::UpdateSteeringTireCommand(const float& steering_tire_command) {
    o_command_steer_.data = steering_tire_command;
}

void LateralControlNode::UpdateRvizControlPrediction(const ControlTrajectory& control_trajectory) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker      line_marker, point_marker;
    geometry_msgs::Point            point;

    line_marker.header.frame_id = control_trajectory.frame_id;
    line_marker.header.stamp    = ros::Time(d_time_stamp_);
    line_marker.ns              = "control_trajectory";
    line_marker.action          = visualization_msgs::Marker::ADD;
    line_marker.type            = visualization_msgs::Marker::LINE_STRIP;
    line_marker.lifetime        = Duration(0.1);

    line_marker.scale.x = 0.15;
    line_marker.color.r = 1.0f;
    line_marker.color.g = 0.078f;
    line_marker.color.b = 0.57;
    line_marker.color.a = 0.3f;

    point_marker.header.frame_id = control_trajectory.frame_id;
    point_marker.header.stamp    = ros::Time(d_time_stamp_);
    point_marker.ns              = "control_points";
    point_marker.action          = visualization_msgs::Marker::ADD;
    point_marker.type            = visualization_msgs::Marker::CYLINDER;
    point_marker.lifetime        = Duration(0.1);

    point_marker.scale.x = 0.15;
    point_marker.scale.y = 0.15;
    point_marker.scale.z = 0.01;

    point_marker.color.r = 1.0f;
    point_marker.color.g = 0.078f;
    point_marker.color.b = 0.57;
    point_marker.color.a = 0.8f;

    uint16_t marker_id = 0;
    for ( auto control_point : control_trajectory.control_point ) {
        point.x = control_point.x;
        point.y = control_point.y;
        point.z = 0.2;
        line_marker.points.push_back(point);

        point_marker.id              = marker_id++;
        point_marker.pose.position.x = control_point.x;
        point_marker.pose.position.y = control_point.y;
        point_marker.pose.position.z = 0.25;
        point_marker.scale.z         = 0.5;

        point_marker.pose.orientation.x = 0.0;
        point_marker.pose.orientation.y = 0.0;
        point_marker.pose.orientation.z = 0.0;
        point_marker.pose.orientation.w = 1.0;

        marker_array.markers.push_back(point_marker);
    }
    marker_array.markers.push_back(line_marker);
    o_control_trajectory_       = marker_array;
}

void LateralControlNode::UpdateRvizControlReference(const ControlTrajectory& trajectory) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker      marker;
    marker.header.frame_id = trajectory.frame_id;
    marker.header.stamp    = ros::Time(d_time_stamp_);
    marker.action          = visualization_msgs::Marker::ADD;
    marker.type            = visualization_msgs::Marker::CYLINDER;
    marker.lifetime        = ros::Duration(0.1);

    marker.scale.x = 0.15;
    marker.scale.y = 0.15;

    marker.color.r = 0.0f;
    marker.color.g = 0.78f;
    marker.color.b = 0.58f;
    marker.color.a = 0.5f;

    uint16_t marker_id = 0;
    for ( auto dp : trajectory.control_point ) {
        marker.id              = marker_id++;
        marker.ns              = "curvature";
        marker.color.r = 0.0f;
        marker.color.g = 0.78f;
        marker.color.b = 0.58f;
        marker.pose.position.x = dp.x;
        marker.pose.position.y = dp.y;
        marker.pose.position.z = dp.curvature*50; // scale up for viz
        marker.scale.x         = 0.15;
        marker.scale.y         = 0.15;
        marker.scale.z         = dp.curvature*100;
        marker.pose.orientation.x = 0.;
        marker.pose.orientation.y = 0.;
        marker.pose.orientation.z = 0.;
        marker.pose.orientation.w = 1.;
        marker_array.markers.push_back(marker);

        marker.id              = marker_id++;
        marker.ns              = "yaw";
        marker.color.r = 1.0f;
        marker.pose.position.z = dp.yaw*RAD2DEG/4; // scale up for viz
        marker.scale.z         = dp.yaw*RAD2DEG/2;
        marker_array.markers.push_back(marker);

        marker.id              = marker_id++;
        marker.ns              = "speed";
        marker.color.r = 0.6f;
        marker.color.g = 0.6f;
        marker.pose.position.z = dp.vx*KPH2MPS/2; // scale up for viz
        marker.scale.z         = dp.vx*KPH2MPS;
        marker_array.markers.push_back(marker);
    }
    o_reference_path_ = marker_array;
}

void LateralControlNode::UpdatePathTrackingInfos(const PathTrackingInfos& tracking_errors) {
    o_control_info_.cross_track_error = tracking_errors.cross_track_error;
    o_control_info_.yaw_error = tracking_errors.yaw_error;
    o_control_info_.total_time = tracking_errors.total_time;
    o_control_info_.cross_track_error = tracking_errors.cross_track_error;
    o_control_info_.cross_track_error = tracking_errors.cross_track_error;
    o_current_understeer_gradient_.data  = tracking_errors.compensation_lateral;
}

void LateralControlNode::Publish() {
    if (!b_is_vehicle_state_ || !b_is_trajectory_) {
        return;
    }
    
    p_command_steer_.publish(o_command_steer_);
    p_control_info_.publish(o_control_info_);
    p_control_trajectory_.publish(o_control_trajectory_);
    p_reference_path_.publish(o_reference_path_);
    p_current_understeer_gradient_.publish(o_current_understeer_gradient_);
}


void LateralControlNode::ProcessINI() {
    if ( util_ini_parser_.IsFileUpdated() ) {
        ROS_WARN("[Lateral control] Ini file is updated!");
        util_ini_parser_.ParseConfig("Low Level Control", "use_predicted_path",  b_use_predicted_path_  );
    }
}

int main(int argc, char** argv) {
    std::string node_name = "lateral_control";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_lateral_control", period)) {
        period = 1.0;
    }

    LateralControlNode main_task(node_name, period);
    main_task.Exec();

    return 0;
}