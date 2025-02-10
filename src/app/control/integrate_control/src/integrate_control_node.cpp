/**
 * @file        integrate_control_node.cpp
 * @brief       integrate_control_node
 * 
 * @authors     Junhee Lee (998jun@gmail.com)      
 * 
 * @date        
 *              2024-10-31 Junhee Lee - create from lateral control
 *              2024-11-05 Junhee Lee - Update acados based dynamic MPC (stable)
 *              2024-11-14 Junhee Lee - Add hirachical control option (upper: integrated MPC, lower: lateral MPC / longitudinal PID)
 */

#include "integrate_control_node.hpp"

ControlNode::ControlNode(std::string task_node, double period)
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
        "app/loc/vehicle_state", 10, &ControlNode::CallbackVehicleState, this);     
    s_trajectory_ = nh.subscribe(
        "app/pla/trajectory", 10, &ControlNode::CallbackTrajectory, this);

    // Publisher init
    p_command_steer_    = nh.advertise<std_msgs::Float32>("app/con/command_steer", 10);
    p_command_torque_       = nh.advertise<std_msgs::Float32>("app/con/command_torque", 10);
    p_command_acceleration_ = nh.advertise<std_msgs::Float32>("app/con/command_accel", 10);  

    // For low level trajectory tracking controller
    p_predicted_trajectory_ = nh.advertise<autohyu_msgs::Trajectory>("app/con/predicted_trajectory", 10);  

    p_control_info_     = nh.advertise<autohyu_msgs::ControlInfo>("hmi/con/control_info", 10);
    p_control_trajectory_     = nh.advertise<visualization_msgs::MarkerArray>("hmi/con/prediction", 10);
    p_reference_path_   = nh.advertise<visualization_msgs::MarkerArray>("hmi/con/reference", 10);
    p_current_understeer_gradient_ = nh.advertise<std_msgs::Float32>("hmi/con/current_understeer_gradient", 10);
    // Algorithm init
    ptr_integrate_control_ = std::make_unique<Control>();
    ptr_integrate_control_->Init();
}

ControlNode::~ControlNode() {
    // Terminate
}

void ControlNode::Run() {
    ProcessINI();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if (!b_is_vehicle_state_) {
        ROS_ERROR("[Integrate Control] Wait for Vehicle State...");
        return;
    }
    if (!b_is_trajectory_) {
        ROS_ERROR("[Integrate Control] Wait for Trajectory...");
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

    const auto [control_command, control_reference, path_tracking_infos] = ptr_integrate_control_->Run(
            vehicle_state, trajectory);
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    UpdateControlCommand(control_command); //Deg
    UpdatePredictedTrajectory(control_command);
    UpdateRvizControlPrediction(control_command.control_trajectory);
    UpdateRvizControlReference(control_reference);
    UpdatePathTrackingInfos(path_tracking_infos);
}

// output msg update functions
void ControlNode::UpdateControlCommand(const ControlCommand& cmd) {
    o_command_steer_.data        = cmd.steering_tire_angle;    
    o_command_torque_.data       = cmd.trq;    
    o_command_acceleration_.data = cmd.ax;
}

void ControlNode::UpdatePredictedTrajectory(const ControlCommand& cmd) {
    autohyu_msgs::Trajectory o_trajectory;
    o_trajectory.header.frame_id = "world";

    for (auto point : cmd.control_trajectory.control_point) {
        autohyu_msgs::TrajectoryPoint tpoint;
        tpoint.time = point.time;
        tpoint.x = point.x;
        tpoint.y = point.y;
        tpoint.z = 0.0;
        tpoint.yaw = point.yaw;
        tpoint.curvature = point.curvature;
        // tpoint.distance = point.distance - optimal_trajectory.point.at(0).distance;
        tpoint.speed = point.vx;
        // tpoint.acceleration = point.dds;
        tpoint.acceleration = point.ax;

        o_trajectory.point.push_back(tpoint);
    }
    o_predicted_trajectory_ = o_trajectory;
}

void ControlNode::UpdateRvizControlPrediction(const ControlTrajectory& control_trajectory) {
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

    point_marker.pose.orientation.x = 0.0;
    point_marker.pose.orientation.y = 0.0;
    point_marker.pose.orientation.z = 0.0;
    point_marker.pose.orientation.w = 1.0;

    uint16_t marker_id = 0;
    for ( auto control_point : control_trajectory.control_point ) {
        point.x = control_point.x;
        point.y = control_point.y;
        point.z = 0.2;
        line_marker.points.push_back(point);

        point_marker.ns              = "predicted_speed";
        point_marker.id              = marker_id++;
        point_marker.pose.position.x = control_point.x;
        point_marker.pose.position.y = control_point.y;
        point_marker.pose.position.z = control_point.vx*KPH2MPS/2.0;
        point_marker.scale.z         = control_point.vx*KPH2MPS;
        marker_array.markers.push_back(point_marker);

        point_marker.ns              = "predicted_ax/Fx";
        point_marker.id              = marker_id++;
        point_marker.pose.position.z = control_point.ax/2.0;
        point_marker.scale.z         = control_point.ax;
        marker_array.markers.push_back(point_marker);

        point_marker.ns              = "predicted_jerk/dFx";
        point_marker.id              = marker_id++;
        point_marker.pose.position.z = control_point.jerkx/2.0;
        point_marker.scale.z         = control_point.jerkx;
        marker_array.markers.push_back(point_marker);

        point_marker.ns              = "predicted_steer";
        point_marker.id              = marker_id++;
        point_marker.pose.position.z = control_point.steer*RAD2DEG/2.0;
        point_marker.scale.z         = control_point.steer*RAD2DEG;
        marker_array.markers.push_back(point_marker);


        point_marker.ns              = "predicted_dsteer";
        point_marker.id              = marker_id++;
        point_marker.pose.position.z = control_point.dsteer*RAD2DEG/2.0;
        point_marker.scale.z         = control_point.dsteer*RAD2DEG;
        marker_array.markers.push_back(point_marker);
    }
    marker_array.markers.push_back(line_marker);
    o_control_trajectory_       = marker_array;
}

void ControlNode::UpdateRvizControlReference(const ControlTrajectory& trajectory) {
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

void ControlNode::UpdatePathTrackingInfos(const PathTrackingInfos& tracking_errors) {
    o_control_info_.cross_track_error = tracking_errors.cross_track_error;
    o_control_info_.yaw_error = tracking_errors.yaw_error;
    o_control_info_.total_time = tracking_errors.total_time;
    o_control_info_.cross_track_error = tracking_errors.cross_track_error;
    o_control_info_.cross_track_error = tracking_errors.cross_track_error;
    o_current_understeer_gradient_.data  = tracking_errors.compensation_lateral;
}

void ControlNode::Publish() {
    if (!b_is_vehicle_state_ || !b_is_trajectory_) {
        return;
    }

    p_predicted_trajectory_.publish(o_predicted_trajectory_);
    if(!b_use_predicted_path_){
        p_command_steer_.publish(o_command_steer_);
    }
    if(!b_use_predicted_speed_){
        p_command_torque_.publish(o_command_torque_);
        p_command_acceleration_.publish(o_command_acceleration_);
    }


    p_control_info_.publish(o_control_info_);
    p_control_trajectory_.publish(o_control_trajectory_);
    p_reference_path_.publish(o_reference_path_);
    p_current_understeer_gradient_.publish(o_current_understeer_gradient_);
}


void ControlNode::ProcessINI() {
    if ( util_ini_parser_.IsFileUpdated() ) {
        ROS_WARN("[Integrate control] Ini file is updated!");
        util_ini_parser_.ParseConfig("Low Level Control", "use_predicted_path", b_use_predicted_path_);
        util_ini_parser_.ParseConfig("Low Level Control", "use_predicted_speed", b_use_predicted_speed_);

    }
}

int main(int argc, char** argv) {
    std::string node_name = "integrate_control";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_integrate_control", period)) {
        period = 1.0;
    }

    ControlNode main_task(node_name, period);
    main_task.Exec();

    return 0;
}