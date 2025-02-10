/**
 * Module:      virtual_object_generator_node.cpp
 * Description: virtual object generator node
 *
 * Authors: Kyungwoon So (bigcow1999@gmail.com)
 *          Yuseung Na (ys.na0220@gmail.com)
 *
 * Revision History
 *      Sep. 07, 2023: Kyungwoon So - Created
 *      Sep. 21, 2023: Yuseung Na - Modify for real vehicle test
 */

#include "virtual_object_generator_node.hpp"

VirtualObjectGenerator::VirtualObjectGenerator(std::string task_node, double period):
    TaskManager(task_node, period) {
    // Make unique pointer
    ptr_scenario_parser_ = std::make_unique<ScenarioParser>();

    // Node init
    NodeHandle nh;

    // Ini init
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/perception.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());

    // Parameter init
    ProcessINI();

    // ROS param init
    std::string csv_path_string;
    if ( nh.getParam("/vils_object_generator/csv_path", csv_path_string) ) {
        std::stringstream ss(csv_path_string);
        std::string       item;
        while ( std::getline(ss, item, ',') ) {
            item.erase(std::remove(item.begin(), item.end(), ' '), item.end()); // remove spaces
            item.erase(std::remove(item.begin(), item.end(), '['), item.end()); // remove opening bracket
            item.erase(std::remove(item.begin(), item.end(), ']'), item.end()); // remove closing bracket

            params_.csv_path.push_back(item);
        }

        for ( const auto& path : params_.csv_path ) {
            ROS_INFO("csv_path: %s", path.c_str());
        }
    }
    else {
        ROS_ERROR("Failed to get csv_path");
    }

    std::string csv_noise_path_string;
    if ( nh.getParam("/vils_object_generator/csv_noise_path", csv_noise_path_string) ) {
        std::stringstream ss(csv_noise_path_string);
        std::string       item;
        while ( std::getline(ss, item, ',') ) {
            item.erase(std::remove(item.begin(), item.end(), ' '), item.end()); // remove spaces
            item.erase(std::remove(item.begin(), item.end(), '['), item.end()); // remove opening bracket
            item.erase(std::remove(item.begin(), item.end(), ']'), item.end()); // remove closing bracket

            params_.csv_noise_path.push_back(item);
        }

        for ( const auto& path : params_.csv_noise_path ) {
            ROS_INFO("csv_noise_path: %s", path.c_str());
        }
    }
    else {
        ROS_ERROR("Failed to get csv_noise_path");
    }

    // Subscriber init
    s_vehicle_state_     = nh.subscribe("app/loc/vehicle_state", 10, &VirtualObjectGenerator::CallbackVehicleState, this);
    s_behavior_trajectory_ = nh.subscribe("app/pla/behavior_trajectory", 10, &VirtualObjectGenerator::CallbackBehaviorTrajectory, this);

    // Publisher init
    p_virtual_objects_      = nh.advertise<autohyu_msgs::TrackObjects>("bsw/vils/track_objects", 10);
    p_rviz_virtual_objects_ = nh.advertise<visualization_msgs::MarkerArray>("hmi/vils/track_objects", 10);

    // Algorithm init
    ptr_scenario_parser_->Init(params_);
}

VirtualObjectGenerator::~VirtualObjectGenerator() {}

void VirtualObjectGenerator::Run() {
    ProcessINI();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if ( b_is_vehicle_state_ == false ) {
        ROS_WARN("[Object Generator] No vehicle state!");
        return;
    }
    if ( b_is_behavior_trajectory_ == false ) {
        ROS_WARN("[Object Generator] No global trajectory!");
        return;
    }

    interface::VehicleState vehicle_state;
    {
        mutex_vehicle_state_.lock();
        vehicle_state = i_vehicle_state_;
        mutex_vehicle_state_.unlock();
    }
    interface::BehaviorTrajectory behavior_trajectory;
    {
        mutex_behavior_trajectory_.lock();
        behavior_trajectory = i_behavior_trajectory_;
        mutex_behavior_trajectory_.unlock();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    tk::Map road_map;
    std::vector<interface::Point> points;
    for(auto& rlp : behavior_trajectory.reference_map){
        interface::Point point;
        point.x = rlp.x;
        point.y = rlp.y;
        points.push_back(point);
    }
    util_function::GenerateSparseRoadMap(points, road_map, 0.01);
    tk::spline left_boundary, right_boundary;
    util_function::GenerateBoundarySpline(behavior_trajectory, road_map, left_boundary, right_boundary);
    std::vector<double> ego_frenet = road_map.ToFrenet(vehicle_state.x, vehicle_state.y);

    interface::TrackObjects virtual_vehicles = ptr_scenario_parser_->ParseScenario(vehicle_state, ego_frenet, road_map,
                                                                        left_boundary, right_boundary, params_);

    // util_function::DebugPrintInfo(
    //     "Size of virtual objects : " + std::to_string(virtual_vehicles.object.size())
    // );

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    UpdateVirtualObjects(vehicle_state, virtual_vehicles, ego_frenet, road_map, left_boundary, right_boundary, params_);
    UpdateRvizVirtualObjects(virtual_vehicles);
}

void VirtualObjectGenerator::Publish() {
    // if ((GenerateMode)params_.generate_mode == GenerateMode::OFF) {
    //     return;
    // }
    if ( b_is_vehicle_state_ == false ) {
        ROS_WARN("[Virtual Object Generator] No vehicle state!");
        return;
    }
    if ( b_is_behavior_trajectory_ == false ) {
        ROS_WARN("[Virtual Object Generator] No global trajectory!");
        return;
    }

    p_virtual_objects_.publish(o_virtual_objects_);
    p_rviz_virtual_objects_.publish(o_rviz_virtual_objects_);
}

void VirtualObjectGenerator::Terminate() {
    autohyu_msgs::TrackObjects virtual_objects;
    p_virtual_objects_.publish(virtual_objects);

    visualization_msgs::MarkerArray rviz_virtual_objects;
    p_rviz_virtual_objects_.publish(rviz_virtual_objects);
}

void VirtualObjectGenerator::ProcessINI() {
    if ( util_ini_parser_.IsFileUpdated() ) {
        util_ini_parser_.ParseConfig("Virtual object generator", "generate_mode", params_.generate_mode);
        util_ini_parser_.ParseConfig("Virtual object generator", "roi_s_front", params_.roi_s_front);
        util_ini_parser_.ParseConfig("Virtual object generator", "roi_s_rear", params_.roi_s_rear);
        util_ini_parser_.ParseConfig("Virtual object generator", "noise_x", params_.noise_x);
        util_ini_parser_.ParseConfig("Virtual object generator", "noise_y", params_.noise_y);
        util_ini_parser_.ParseConfig("Virtual object generator", "noise_yaw", params_.noise_yaw);
        util_ini_parser_.ParseConfig("Virtual object generator", "noise_v_x", params_.noise_v_x);
        util_ini_parser_.ParseConfig("Virtual object generator", "noise_a_x", params_.noise_a_x);
        util_ini_parser_.ParseConfig("Virtual object generator", "noise_mode", params_.noise_mode);

        ROS_WARN("[Virtual object generator] Ini file is updated!");
    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
// Update functions for publish variables
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
void VirtualObjectGenerator::UpdateVirtualObjects(const interface::VehicleState& vehicle_state, const interface::TrackObjects& objects,
                                                  const vector<double> ego_frenet, tk::Map& road_map, tk::spline& left_boundary,
                                                  tk::spline& right_boundary, const VirtualObjectGeneratorParams params) {
    autohyu_msgs::TrackObjects o_objects;
    o_objects.header.frame_id = "world";
    o_objects.header.stamp    = ros::Time(objects.header.stamp);

    for ( auto object : objects.object ) {
        // if ( road_map.ClosestDistance(object.state.x, object.state.y) > 15.0 ) {
        //     continue;
        // }

        // Circle roi based check
        double distance = sqrt(pow(vehicle_state.x - object.state.x, 2) + pow(vehicle_state.y - object.state.y, 2));
        if ( distance > params.roi_s_front ) {
            continue;
        }

        // Frenet roi based check
        vector<double> object_frenet = road_map.ToFrenet(object.state.x, object.state.y);
        if ( object_frenet.at(0) > ego_frenet.at(0) + params.roi_s_front || object_frenet.at(0) < ego_frenet.at(0) - params.roi_s_rear ) {
            continue;
        }

        // Boundary check
        if ( (object_frenet.at(1) > left_boundary(object_frenet.at(0)) + VEHICLE_WIDTH) ||
             (object_frenet.at(1) < right_boundary(object_frenet.at(0)) - VEHICLE_WIDTH) ) {
            continue;
        }

        autohyu_msgs::TrackObject o_object;
        o_object.id               = object.id;
        o_object.classification   = static_cast<ObjectClass>(object.classification);
        o_object.dynamic_state    = static_cast<ObjectDynamicState>(object.dynamic_state);
        o_object.dimension.length = object.dimension.length;
        o_object.dimension.width  = object.dimension.width;
        o_object.dimension.height = object.dimension.height;

        autohyu_msgs::Object3DState state;
        o_object.state.header.stamp = ros::Time(object.state.header.stamp);
        o_object.state.x            = object.state.x;
        o_object.state.y            = object.state.y;
        o_object.state.yaw          = object.state.yaw;
        o_object.state.v_x          = object.state.v_x;
        o_object.state.v_y          = object.state.v_y;
        o_object.state.a_x          = object.state.a_x;
        o_object.state.a_y          = object.state.a_y;

        // int out_count = 0;
        // // Can be seen or not
        // for (int i = 0; i < int(distance/0.5); i++) {
        //     double dx = (object.state.x - vehicle_state.x)/distance;
        //     double dy = (object.state.y - vehicle_state.y)/distance;

        //     double x = vehicle_state.x + dx*i*0.5;
        //     double y = vehicle_state.y + dy*i*0.5;

        //     vector<double> frenet_xy = road_map.ToFrenet(x, y);

        //     if ((frenet_xy.at(1) > left_boundary(frenet_xy.at(0)) + VEHICLE_WIDTH) ||
        //         (frenet_xy.at(1) < right_boundary(frenet_xy.at(0)) - VEHICLE_WIDTH)) {

        //         out_count++;
        //     }
        // }

        // if (out_count >= 2) {
        //     continue;
        // }

        o_objects.object.push_back(o_object);
    }

    o_virtual_objects_ = o_objects;
}

void VirtualObjectGenerator::UpdateRvizVirtualObjects(const interface::TrackObjects& objects) {
    o_rviz_virtual_objects_.markers.clear();
    visualization_msgs::MarkerArray marker_array;

    for ( auto object : objects.object ) {
        visualization_msgs::Marker position_marker;
        visualization_msgs::Marker track_info_marker;

        position_marker.header.frame_id = "world";
        position_marker.header.stamp    = ros::Time(objects.header.stamp);
        position_marker.ns              = "position";
        position_marker.id              = object.id;
        position_marker.action          = visualization_msgs::Marker::ADD;
        position_marker.type            = visualization_msgs::Marker::CUBE;
        position_marker.lifetime        = ros::Duration(0.1);

        track_info_marker.header.frame_id = "world";
        track_info_marker.header.stamp    = ros::Time(objects.header.stamp);
        track_info_marker.ns              = "velocity";
        track_info_marker.id              = object.id;
        track_info_marker.action          = visualization_msgs::Marker::ADD;
        track_info_marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
        track_info_marker.lifetime        = ros::Duration(0.1);

        double             track_velocity = sqrt(object.state.v_x * object.state.v_x + object.state.v_y * object.state.v_y);
        std::ostringstream display_text;
        display_text << "ID (" << object.id << ")"
                     << "\nVel:   " << std::fixed << std::setprecision(3) << std::round(track_velocity * 1000.0) / 1000.0 << "m/s"
                     << "\nAccel: " << std::fixed << std::setprecision(3) << std::round(object.state.a_x * 1000.0) / 1000.0 << "m/s^2";

        track_info_marker.text = display_text.str();

        // Line width
        position_marker.scale.x   = object.dimension.length;
        position_marker.scale.y   = object.dimension.width;
        position_marker.scale.z   = object.dimension.height;
        track_info_marker.scale.z = 1.0;

        // Color space
        position_marker.color.r   = 0.0f;
        position_marker.color.g   = 1.0f;
        position_marker.color.b   = 1.0f;
        position_marker.color.a   = 0.5f;
        track_info_marker.color.r = 1.0f;
        track_info_marker.color.g = 1.0f;
        track_info_marker.color.b = 1.0f;
        track_info_marker.color.a = 1.0f;

        position_marker.pose.position.x  = object.state.x;
        position_marker.pose.position.y  = object.state.y;
        position_marker.pose.position.z  = object.dimension.height / 2;
        position_marker.pose.orientation = tf::createQuaternionMsgFromYaw(object.state.yaw);

        track_info_marker.pose.position.x  = object.state.x;
        track_info_marker.pose.position.y  = object.state.y;
        track_info_marker.pose.position.z  = 3.0;
        track_info_marker.pose.orientation = tf::createQuaternionMsgFromYaw(object.state.yaw);

        marker_array.markers.push_back(position_marker);
        marker_array.markers.push_back(track_info_marker);
    }

    o_rviz_virtual_objects_ = marker_array;
}

int main(int argc, char** argv) {
    // Initialize node
    std::string node_name = "vils_object_generator";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Get and period
    double period;
    if ( !nh.getParam("task_period/period_vils_object_generator", period) ) {
        period = 1.0;
    }

    // Exec algorithm
    VirtualObjectGenerator main_task(node_name, period);
    main_task.Exec();

    return 0;
}