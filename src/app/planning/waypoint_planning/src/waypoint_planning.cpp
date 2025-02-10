/**
 * @file        waypoint_planning.cpp
 * @brief       waypoint planning node
 * 
 * @authors     Junhee Lee (998jun@gmail.com)
 *              
 * 
 * @date        2023-06-25 Junhee Lee - Created.
 *              2023-08-08 Junhee Lee - Add function to get Lanelet reference speed
 */

#include "waypoint_planning.hpp"

WaypointPlanning::WaypointPlanning(int id, std::string task_node, double period)
    : TaskManager(task_node,  period) {
    // Initialize
    ROS_WARN_STREAM("[" << task_name_ << "] Initialize node (ID: " 
                                        << ", Period: " << 1/task_period_ << " Hz)");

    // Node init
    ros::NodeHandle nh;

    // Ini init
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/planning.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());
    

    // Parameter init
    ProcessINI();
    ProcessRosparam(nh);

    // Subscriber init
    s_vehicle_state_ = nh.subscribe(
        "app/loc/vehicle_state", 10, &WaypointPlanning::CallbackVehicleState, this);   

    // Publisher init
    p_trajectory_ = nh.advertise<autohyu_msgs::Trajectory>(
            "app/pla/trajectory", 10);
    p_bhv_trajectory_ = nh.advertise<autohyu_msgs::BehaviorTrajectory>(
            "app/pla/behavior_trajectory", 10);
    p_race_trajectory_ = nh.advertise<autohyu_msgs::Trajectory>(
            "app/pla/race_trajectory", 10);
    p_rviz_map_ = nh.advertise<visualization_msgs::MarkerArray>(
            "hmi/pla/map", 10);
    p_rviz_trajectory_ = nh.advertise<visualization_msgs::MarkerArray>(
            "hmi/pla/trajectory", 10);
    p_rviz_race_trajectory_ = nh.advertise<visualization_msgs::MarkerArray>(
            "hmi/pla/race_trajectory", 10);

    // Algorithm init
    ptr_waypoint_generator_ = make_unique<WaypointGenerator>();
    ptr_waypoint_generator_->Init(params_);
    lanelet::LaneletMapPtr lanelet_map = ptr_waypoint_generator_->GetLaneletMap();
    b_is_map_update_ = true;
    d_prev_map_update_time_ = ros::Time::now().toSec();
    UpdateRvizMap(lanelet_map);
}

WaypointPlanning::~WaypointPlanning() {
    // Terminate
}

void WaypointPlanning::Run() {
    ProcessINI();
    // ProcessRosparam(); // just for init
    CheckMapUpdate();
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if (!b_is_vehicle_state_) {
        ROS_ERROR_THROTTLE(1.0,"[Trajectory Planning] Wait for Vehicle State...");
        return;
    }

    interface::VehicleState vehicle_state; {
        mutex_vehicle_state_.lock();
        vehicle_state = i_vehicle_state_;
        mutex_vehicle_state_.unlock();
        time_stamp_ = vehicle_state.header.stamp;
    }
  
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //


    pair<Trajectory,Trajectory> race_trajectory =  ptr_waypoint_generator_->GenerateWaypoint(&vehicle_state, &params_);
    lanelet::LaneletMapPtr lanelet_map = ptr_waypoint_generator_->GetLaneletMap();
    if ((ros::Time::now().toSec() - d_prev_map_update_time_) > 0.9 || (ros::Time::now().toSec() - d_prev_map_update_time_) < -0.9 ) {
        UpdateRvizMap(lanelet_map);
        d_prev_map_update_time_ = ros::Time::now().toSec();
    }
    UpdateTrajectory(race_trajectory.first, race_trajectory.second);
    UpdateRvizTrajectory(race_trajectory.first, race_trajectory.second);
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // not yet defined

}

// instead of processrosparam
void WaypointPlanning::CheckMapUpdate(){
    ros::NodeHandle nh;

    string in_map_file_path;
    if (nh.getParam("/waypoint_planning/map_file", in_map_file_path)) {
        if(in_map_file_path != params_.map_file_path){
            ROS_INFO_STREAM("New map! : "<<in_map_file_path);
            if (!boost::filesystem::exists(boost::filesystem::path(in_map_file_path))) {
                std::cout << "Could not find lanelet map under "<<  in_map_file_path<< endl;
                return;
            }
            params_.map_file_path = in_map_file_path;
            b_is_map_update_ = true;
        }
        else{
            b_is_map_update_ = false;
        }
    }  
}

void WaypointPlanning::Publish() {
    if (!b_is_vehicle_state_) {
        return;
    }
    
    p_trajectory_.publish(o_trajectory_);
    p_bhv_trajectory_.publish(o_bhv_trajectory_);
    p_race_trajectory_.publish(o_race_trajectory_);
    p_rviz_trajectory_.publish(o_rviz_trajectory_);
    p_rviz_race_trajectory_.publish(o_rviz_race_trajectory_);
}

void WaypointPlanning::ProcessRosparam(const ros::NodeHandle& nh) {
    if (!nh.getParam("/waypoint_planning/map_file", params_.map_file_path)) {
        ROS_ERROR_STREAM(params_.map_file_path);
        params_.map_file_path = "";
    }  
    else{
        ROS_INFO_STREAM(params_.map_file_path);
    }
    if (!nh.getParam("/waypoint_planning/ref_lat", params_.ref_lat)) {
        params_.ref_lat = 0.0;
    }
    if (!nh.getParam("/waypoint_planning/ref_lon", params_.ref_lon)) {
        params_.ref_lon = 0.0;
    }
    ROS_WARN_STREAM("Reference Lat: " << params_.ref_lat << ", Lon: " << params_.ref_lon);

}

void WaypointPlanning::ProcessINI() {
    if (util_ini_parser_.IsFileUpdated()) {
        ROS_WARN("[Waypoint planning] Ini file is updated!");
    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
// Update functions for publish variables
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
void WaypointPlanning::UpdateTrajectory(const Trajectory& optimal_trajectory,
                                          const Trajectory& race_trajectory) {

    prev_trajectory_ = optimal_trajectory;
    autohyu_msgs::Trajectory o_trajectory;
    o_trajectory.header.frame_id = "world";
    o_trajectory.header.stamp = ros::Time(optimal_trajectory.header.stamp);

    for (auto point : optimal_trajectory.point) {
        autohyu_msgs::TrajectoryPoint tpoint;
        tpoint.time = point.time;
        tpoint.x = point.x;
        tpoint.y = point.y;
        tpoint.z = point.z;
        tpoint.yaw = point.yaw;
        tpoint.curvature = point.curvature;
        tpoint.distance = point.distance - optimal_trajectory.point.at(0).distance;
        tpoint.speed = point.speed;
        // tpoint.acceleration = point.dds;
        tpoint.acceleration = point.acceleration;

        o_trajectory.point.push_back(tpoint);
    }
    o_trajectory_ = o_trajectory;


    autohyu_msgs::BehaviorTrajectory o_bhv_trajectory;
    o_trajectory.header.frame_id = "world";
    o_trajectory.header.stamp = ros::Time(optimal_trajectory.header.stamp);

    for (auto point : optimal_trajectory.point) {
        autohyu_msgs::BehaviorTrajectoryPoint tpoint;
        tpoint.x = point.x;
        tpoint.y = point.y;
        tpoint.z = point.z;
        tpoint.yaw = point.yaw;
        tpoint.curvature = point.curvature;
        tpoint.speed = point.speed;

        o_bhv_trajectory.point.push_back(tpoint);
    }


    for (auto point : optimal_trajectory.point) {
        autohyu_msgs::BehaviorBoundaryPoint tpoint;
        tpoint.x = point.x;
        tpoint.y = point.y;
        tpoint.z = point.z;

        o_bhv_trajectory.reference_map.push_back(tpoint);
    }


    for (auto point : optimal_trajectory.left_boundary.point) {
        autohyu_msgs::BehaviorBoundaryPoint tpoint;
        tpoint.x = point.x;
        tpoint.y = point.y;
        tpoint.z = point.z;

        o_bhv_trajectory.left_boundary.push_back(tpoint);
    }


    for (auto point : optimal_trajectory.right_boundary.point) {
        autohyu_msgs::BehaviorBoundaryPoint tpoint;
        tpoint.x = point.x;
        tpoint.y = point.y;
        tpoint.z = point.z;
        

        o_bhv_trajectory.right_boundary.push_back(tpoint);
    }


    o_bhv_trajectory_ = o_bhv_trajectory;



    
    autohyu_msgs::Trajectory o_race_trajectory;
    o_race_trajectory.header.frame_id = "world";
    o_race_trajectory.header.stamp = ros::Time(optimal_trajectory.header.stamp);

    for (auto point : race_trajectory.point) {
        autohyu_msgs::TrajectoryPoint tpoint;
        tpoint.time = point.time;
        tpoint.x = point.x;
        tpoint.y = point.y;
        tpoint.z = point.z;
        tpoint.yaw = point.yaw;
        tpoint.curvature = point.curvature;
        tpoint.distance = point.distance - race_trajectory.point.at(0).distance;
        tpoint.speed = point.speed;
        tpoint.acceleration = point.acceleration;

        o_race_trajectory.point.push_back(tpoint);
    }

    o_race_trajectory_ = o_race_trajectory;
}

void WaypointPlanning::UpdateRvizTrajectory(const Trajectory& optimal_trajectory,
                                              const Trajectory& race_trajectory) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker path_marker;
    visualization_msgs::Marker speed_marker;

    path_marker.header.frame_id = "world";
    path_marker.header.stamp = ros::Time(time_stamp_);
    path_marker.ns = "spatial";
    path_marker.id = 0;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.lifetime = ros::Duration(0.1);

    speed_marker.header.frame_id = "world";
    speed_marker.header.stamp = ros::Time(time_stamp_);
    speed_marker.ns = "temporal";
    speed_marker.id = 1;
    speed_marker.action = visualization_msgs::Marker::ADD;
    speed_marker.type = visualization_msgs::Marker::LINE_LIST;
    speed_marker.lifetime = ros::Duration(0.1);

    // Line width
    path_marker.scale.x = 0.25f;
    speed_marker.scale = path_marker.scale;

    // Color space
    path_marker.color.r = 0.0f;
    path_marker.color.g = 0.78f;
    path_marker.color.b = 0.58f;
    path_marker.color.a = 0.33f;
    speed_marker.color = path_marker.color;

    for (uint16_t i = 0; i < (uint16_t)optimal_trajectory.point.size(); i++) {
        geometry_msgs::Point point;
        point.x = optimal_trajectory.point[i].x;
        point.y = optimal_trajectory.point[i].y;
        point.z = optimal_trajectory.point[i].z;
        path_marker.points.push_back(point);
        speed_marker.points.push_back(point);
        point.z = optimal_trajectory.point[i].speed * KPH2MPS;
        speed_marker.points.push_back(point);
    }
    marker_array.markers.push_back(path_marker);
    marker_array.markers.push_back(speed_marker);


    visualization_msgs::Marker marker_boundary;
    marker_boundary.header.stamp = ros::Time(time_stamp_);
    marker_boundary.header.frame_id = "world";
    marker_boundary.scale.x         = 0.2;
    marker_boundary.color.a         = 0.85;
    marker_boundary.id              = 0;
    marker_boundary.type            = visualization_msgs::Marker::LINE_STRIP;
    marker_boundary.action          = visualization_msgs::Marker::ADD;

    marker_boundary.ns              = "left_boundary";
    marker_boundary.color.r         = 0.7;
    marker_boundary.color.g         = 0.7;
    marker_boundary.color.b         = 0.0;
    marker_boundary.lifetime        = ros::Duration(0.1);

    for (const auto& lbp: optimal_trajectory.left_boundary.point) {
        geometry_msgs::Point point;
        point.x = lbp.x;
        point.y = lbp.y;
        point.z = lbp.z;
        marker_boundary.points.push_back(point);
    }
    marker_array.markers.push_back(marker_boundary);

    marker_boundary.ns              = "right_boundary";
    marker_boundary.color.r         = 0.0;
    marker_boundary.color.g         = 0.0;
    marker_boundary.color.b         = 0.7;
    marker_boundary.lifetime        = ros::Duration(0.1);
    marker_boundary.points.clear();
    for (const auto& rbp: optimal_trajectory.right_boundary.point) {
        geometry_msgs::Point point;
        point.x = rbp.x;
        point.y = rbp.y;
        point.z = rbp.z;
        marker_boundary.points.push_back(point);
    }
    marker_array.markers.push_back(marker_boundary);

    o_rviz_trajectory_ = marker_array;
}

void WaypointPlanning::UpdateRvizMap(const lanelet::LaneletMapPtr& ptr_lanelet_map ) {
    // if(b_is_map_update_==false){
    //     return;
    // }
    lanelet::LaneletMap &map = *ptr_lanelet_map;

    visualization_msgs::MarkerArray marker_array;

    vector<lanelet::Id> lane_left_ids;
    vector<lanelet::Id> lane_right_ids;
    vector<lanelet::Id> lane_centerline_ids;

    // Check lanelet's boundary
    for (auto ll : map.laneletLayer){
       lane_left_ids.push_back(ll.leftBound2d().id());
       lane_right_ids.push_back(ll.rightBound2d().id());
       lane_centerline_ids.push_back(ll.centerline2d().id());
    }

    for (auto lineString : map.lineStringLayer) {
        visualization_msgs::Marker marker;
        double x = 0.2, r = 0.3, g = 0.3, b = 0.3, a = 0.5;

        std::string lineString_ns = "default";
        for(auto id : lane_left_ids ){
            if(lineString.id() == id){
                lineString_ns = "left";
                x = 0.1, r = 0.6, g = 0.6, b = 0.6, a = 1.0;
                break;
            }
        }
        for(auto id : lane_right_ids ){
            if(lineString.id() == id){
                lineString_ns = "right";
                x = 0.1, r = 0.6, g = 0.6, b = 0.6, a = 1.0;
                break;
            }
        }
        for(auto id : lane_centerline_ids ){
            if(lineString.id() == id){
                lineString_ns = "centerline";
                x = 0.1, r = 0.3, g = 0.3, b = 0.3, a = 0.7;
                break;
            }
        }
        
        marker.header.stamp    = ros::Time(time_stamp_);
        marker.header.frame_id = "world";
        marker.ns              = lineString_ns;
        marker.id              = (int32_t)lineString.id();
        marker.type            = visualization_msgs::Marker::LINE_STRIP;
        marker.action          = visualization_msgs::Marker::ADD;
        marker.scale.x         = x;
        marker.color.r         = r;
        marker.color.g         = g;
        marker.color.b         = b;
        marker.color.a         = a;
        // marker.lifetime        = ros::Duration(0);
        marker.lifetime        = ros::Duration(1.2);

        for (auto i_ls = 0; i_ls < lineString.size(); i_ls++) {
            geometry_msgs::Point point;
            point.x = lineString[i_ls].x();
            point.y = lineString[i_ls].y();
            point.z = lineString[i_ls].z();
            marker.points.push_back(point);
        }
        marker_array.markers.push_back(marker);
    }
    o_rviz_map_ = marker_array;
    p_rviz_map_.publish(o_rviz_map_);
}



int main(int argc, char** argv) {
    std::string node_name = "waypoint_planning";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ROS_INFO("Initialize node, get parameters...");

    int id;
    if (!nh.getParam("task_id/trajectory_planning", id)) {
        id = 0;
    }       
    double period;
    if (!nh.getParam("task_period/period_trajectory_planning", period)) {
        period = 1.0;
    }
    ROS_INFO("Complete to get parameters! (ID: %d, Period: %.3f)", id, period);

    WaypointPlanning main_task(id, node_name, period);
    main_task.Exec();
    
    return 0;
}