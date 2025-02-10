/**
 * @file        route_driver_node.hpp
 * @brief       carla route driver node hpp file
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-08-01 created by Jeonghun Kang
 * 
 */
#include "route_driver/route_driver_node.hpp"

RouteDriver::RouteDriver(std::string node_name, double period)
    : TaskManager(node_name, period){
    // Node initialization
    ros::NodeHandle nh;

    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/simulation.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());

    // Parameter init
    ProcessINI();
    ProcessRosparam(nh);

    // Subscriber init
    s_carla_route_ = nh.subscribe(
            "carla/" + cfg_.role_name + "/global_plan", 10, &RouteDriver::CallbackCARLARoute, this);
    s_carla_gnss_route_ = nh.subscribe(
            "carla/" + cfg_.role_name + "/global_plan_gnss", 10, &RouteDriver::CallbackCARLAGNSSRoute, this);
    s_ref_origin_ = nh.subscribe(
        "app/loc/reference_point", 10, &RouteDriver::CallbackReference, this);

    // Publisher init
    p_goal_points_ = nh.advertise<autohyu_msgs::GoalPoints>(
            "sim/carla/goal_points", 10);
    p_rviz_route_ = nh.advertise<visualization_msgs::MarkerArray>(
            "hmi/carla/route", 10);
    p_rviz_gnss_route_ = nh.advertise<visualization_msgs::MarkerArray>(
            "hmi/carla/gnss_route", 10);

    // Algorithm init
}

RouteDriver::~RouteDriver() {}

void RouteDriver::ProcessRosparam(const ros::NodeHandle& nh) {
    // ROS param init
    if (!nh.getParam("/route_driver/role_name", cfg_.role_name)) {
        cfg_.role_name = "";
    }
    ROS_WARN_STREAM("Role name for Ego : " << cfg_.role_name);
}
void RouteDriver::Run() {
    ProcessINI();
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if (!b_is_ref_origin_) {
        ROS_ERROR_THROTTLE(1.0, "Wait for Reference Point...");
        return;
    }
    if (!is_initialized) {
        ROS_ERROR_THROTTLE(1.0, "Wait for CARLA Route...");
        return;
    }

    Trajectory carla_rotue; {
        std::lock_guard<std::mutex> lock(mutex_carla_route_);
        carla_rotue = i_carla_route_;
    }
    // Trajectory carla_gnss_route; {
    //     std::lock_guard<std::mutex> lock(mutex_carla_gnss_route_);
    //     carla_gnss_route = i_carla_gnss_route_;
    // }
    // Reference reference_point; {
    //     std::lock_guard<std::mutex> lock(mutex_ref_origin_);
    //     reference_point = i_ref_origin_;
    // }
    Trajectory carla_gnss_route = ProjectCARLAGNSSRoute(i_carla_gnss_route_);
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    UpdateRvizCARLARoute(carla_rotue);
    UpdateGoalPoints(carla_gnss_route);
    UpdateRvizCARLAGNSSRoute(carla_gnss_route);
}

void RouteDriver::Publish() {
    p_goal_points_.publish(o_goal_points_);
    p_rviz_route_.publish(o_rviz_route_);
    p_rviz_gnss_route_.publish(o_rviz_gnss_route_);
}

void RouteDriver::ProcessINI() {
    if (util_ini_parser_.IsFileUpdated()) {
        ROS_WARN("[CARLA Route driver] Ini file is updated!");
        // util_ini_parser_.ParseConfig("Morai Vehicle driver", "cfg_i_sim_ax_ay_time_window", cfg_.i_sim_ax_ay_time_window);
    }
}

Trajectory RouteDriver::GetCARLARoute(const carla_msgs::CarlaRoute& msg) {
    carla_msgs::CarlaRoute i_carla_route = msg;
    Trajectory calra_trajectory;
    calra_trajectory.header.stamp = ros_bridge::GetTimeStamp(i_carla_route.header.stamp);
    for (const auto& poses : i_carla_route.poses) {
        TrajectoryPoint tp;
        // tp.time = point.time;
        tp.x = poses.position.x;
        tp.y = poses.position.y;
        // tp.z = poses.position.z;
        tp.z = 0.0;
        tf::Quaternion quat(
            poses.orientation.x, // x
            poses.orientation.y, // y
            poses.orientation.z, // z
            poses.orientation.w  // w
        );
        double roll,pitch,yaw;
        tf::Matrix3x3 quaternion_mat(quat);
        quaternion_mat.getRPY(roll,pitch,yaw);

        tp.yaw = yaw;
        // tp.curvature = point.curvature;
        // tp.distance = point.distance;
        // tp.speed = point.speed;
        // tp.acceleration = point.acceleration;

        calra_trajectory.point.push_back(tp);
    }
    return calra_trajectory;
}

Trajectory RouteDriver::ProjectCARLAGNSSRoute(const carla_msgs::CarlaGnssRoute& msg) {
    carla_msgs::CarlaGnssRoute i_carla_gnss_route = msg;
    Trajectory calra_gnss_trajectory;
    calra_gnss_trajectory.header.stamp = ros_bridge::GetTimeStamp(i_carla_gnss_route.header.stamp);
    if(b_is_ref_origin_){
        // ROS_WARN_STREAM("Size of Gnss Route " << i_carla_gnss_route.coordinates.size());
        for (const auto& coordinate : i_carla_gnss_route.coordinates) {
            TrajectoryPoint tp;
            // tp.time = point.time;
            lanelet::GPSPoint v_gps_point;
            Position position;
            v_gps_point.lat = coordinate.latitude;
            v_gps_point.lon = coordinate.longitude;
            v_gps_point.ele = coordinate.altitude;
            if(i_ref_origin_.projection == "utm"){
                lanelet::projection::UtmProjector projector(lanelet::Origin({i_ref_origin_.wgs84.latitude, i_ref_origin_.wgs84.longitude}));
                lanelet::BasicPoint3d utm_projpos = projector.forward(v_gps_point);
                position.x = utm_projpos.x();
                position.y = utm_projpos.y();
                position.z = utm_projpos.z();
            }else if(i_ref_origin_.projection == "local_cartesian"){
                lanelet::projection::LocalCartesianProjector projector(lanelet::Origin({i_ref_origin_.wgs84.latitude, i_ref_origin_.wgs84.longitude}));
                lanelet::BasicPoint3d localcartesian_projpos = projector.forward(v_gps_point);
                position.x = localcartesian_projpos.x();
                position.y = localcartesian_projpos.y();
                position.z = localcartesian_projpos.z();
            }
            tp.x = position.x;
            tp.y = position.y;
            tp.z = 0.0;

            // tp.yaw = yaw;
            // tp.curvature = point.curvature;
            // tp.distance = point.distance;
            // tp.speed = point.speed;
            // tp.acceleration = point.acceleration;

            calra_gnss_trajectory.point.push_back(tp);
        }
    }else{
        ROS_ERROR("Wait for Reference Point...");
    }
    return calra_gnss_trajectory;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
// Update functions for publish variables
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
void RouteDriver::UpdateRvizCARLARoute(const Trajectory& trajectory){
    visualization_msgs::MarkerArray o_rviz_route;

    visualization_msgs::Marker path_marker;
    visualization_msgs::Marker point_marker;
    path_marker.header.stamp = ros::Time(trajectory.header.stamp);
    path_marker.header.frame_id = "world";
    point_marker.header = path_marker.header;

    path_marker.ns = "spatial";
    path_marker.id = 0;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.lifetime = ros::Duration(0.1);

    point_marker.ns = "temporal";
    point_marker.id = 1;
    point_marker.action = visualization_msgs::Marker::ADD;
    point_marker.type = visualization_msgs::Marker::LINE_LIST;
    point_marker.lifetime = ros::Duration(0.1);

    // Line width
    path_marker.scale.x = 0.5f;
    point_marker.scale.x = 0.5f;

    // Color space - Pink
    path_marker.color.r = 1.0f;
    path_marker.color.g = 0.4f;
    path_marker.color.b = 0.7f;
    path_marker.color.a = 1.0f;
    point_marker.color = path_marker.color;

    for (uint16_t i = 0; i < (uint16_t)trajectory.point.size(); i++) {
        geometry_msgs::Point point;
        point.x = trajectory.point[i].x;
        point.y = trajectory.point[i].y;
        point.z = trajectory.point[i].z;
        path_marker.points.push_back(point);
        point_marker.points.push_back(point);
        point.z = 2.0;
        point_marker.points.push_back(point);
    }
    o_rviz_route.markers.push_back(path_marker);
    o_rviz_route.markers.push_back(point_marker);

    o_rviz_route_ = o_rviz_route;
}

void RouteDriver::UpdateGoalPoints(const Trajectory& trajectory){
    autohyu_msgs::GoalPoints goal_points;

    for (uint16_t i = 0; i < (uint16_t)trajectory.point.size(); i++) {
        geometry_msgs::Point point;
        point.x = trajectory.point[i].x;
        point.y = trajectory.point[i].y;
        point.z = trajectory.point[i].z;
        goal_points.goal_points.push_back(point);
    }

    o_goal_points_ = goal_points;
}

void RouteDriver::UpdateRvizCARLAGNSSRoute(const Trajectory& trajectory){
    visualization_msgs::MarkerArray o_rviz_gnss_route;

    visualization_msgs::Marker path_marker;
    visualization_msgs::Marker point_marker;
    path_marker.header.stamp = ros::Time(trajectory.header.stamp);
    path_marker.header.frame_id = "world";
    point_marker.header = path_marker.header;

    path_marker.ns = "spatial";
    path_marker.id = 2;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.lifetime = ros::Duration(0.1);

    point_marker.ns = "temporal";
    point_marker.id = 3;
    point_marker.action = visualization_msgs::Marker::ADD;
    point_marker.type = visualization_msgs::Marker::LINE_LIST;
    point_marker.lifetime = ros::Duration(0.1);

    // Line width
    path_marker.scale.x = 0.5f;
    point_marker.scale.x = 0.5f;

    // Color space - Pink
    path_marker.color.r = 0.0f;
    path_marker.color.g = 1.0f;
    path_marker.color.b = 1.0f;
    path_marker.color.a = 1.0f;
    point_marker.color = path_marker.color;

    for (uint16_t i = 0; i < (uint16_t)trajectory.point.size(); i++) {
        geometry_msgs::Point point;
        point.x = trajectory.point[i].x;
        point.y = trajectory.point[i].y;
        point.z = trajectory.point[i].z;
        path_marker.points.push_back(point);
        point_marker.points.push_back(point);
        point.z = 2.0;
        point_marker.points.push_back(point);
    }
    o_rviz_gnss_route.markers.push_back(path_marker);
    o_rviz_gnss_route.markers.push_back(point_marker);

    o_rviz_gnss_route_ = o_rviz_gnss_route;
}

int main(int argc, char** argv) {
    std::string node_name = "route_driver";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_route_driver", period)){
        period = 1.0;
    }
    
    RouteDriver main_task(node_name, period);
    main_task.Exec();

    return 0;
}