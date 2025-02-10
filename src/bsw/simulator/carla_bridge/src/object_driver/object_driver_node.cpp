/**
 * @file        object_driver_node.cpp
 * @brief       node cpp file for carla object driver
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)
 *              Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-08-01 created by Jeonghun Kang
 *              
 * 
 */

#include "object_driver/object_driver_node.hpp"

ObjectDriver::ObjectDriver(std::string node_name, double period)
    : TaskManager(node_name, period) {
    // Initialize
    ROS_WARN_STREAM("[" << node_name << "] Initialize node (Period: " << 1/period << " Hz)");

    // Node init
    ros::NodeHandle nh;

    // Ini init
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/simulation.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());

    // Parameter init
    ProcessINI();
    ProcessRosparam(nh);
    
    // Subscriber init
    s_vehicle_state_ = nh.subscribe(
            "app/loc/vehicle_state", 10, &ObjectDriver::CallbackVehicleState, this);   
    s_carla_objects_ = nh.subscribe(
            "carla/" + cfg_.role_name + "/objects", 10, &ObjectDriver::CallbackCARLAObjects, this);
    s_ref_origin_ = nh.subscribe(
        "app/loc/reference_point", 10, &ObjectDriver::CallbackReference, this);
    s_ego_vehicle_info_ = nh.subscribe(
        "carla/" + cfg_.role_name + "/vehicle_info", 10, &ObjectDriver::CallbackCARLAEgoVehicleInfo, this);
    s_ego_vehicle_odometry_ = nh.subscribe(
        "carla/" + cfg_.role_name + "/odometry", 10, &ObjectDriver::CallbackCARLAVehicleOdometry, this);
    // s_carla_actor_list_ = nh.subscribe(
    //     "carla/" + cfg_.role_name + "/actor_list", 10, &ObjectDriver::CallbackCARLAActorList, this);

    // Publisher init
    p_track_objects_ = nh.advertise<autohyu_msgs::TrackObjects>(
            "app/perc/track_objects", 10);
    p_rviz_track_objects_ = nh.advertise<visualization_msgs::MarkerArray>(
            "hmi/perc/track_objects", 10);
            
    // Algorithm init    
}

ObjectDriver::~ObjectDriver() {
    // Terminate
}

void ObjectDriver::Run() {
    ProcessINI();    

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //    
    if (b_is_vehicle_state_ == false) {
        ROS_ERROR_THROTTLE(1.0,"Wait for Vehicle State...");
        return;
    }
    if (b_is_carla_objects_ == false) {
        ROS_ERROR_THROTTLE(1.0,"Wait for CARLA Objects...");
        return;
    }
    if (b_is_ref_origin_ == false) {
        ROS_ERROR_THROTTLE(1.0,"Wait for Reference Points...");
        return;
    }
    if (b_is_carla_ego_vehicle_info_ == false) {
        ROS_ERROR_THROTTLE(1.0,"Wait for Ego Vehicle Info...");
        return;
    }
    if (b_is_carla_ego_vehicle_odometry_ == false) {
        ROS_ERROR_THROTTLE(1.0,"Wait for Ego Vehicle Odometry...");
        return;
    }
    // if ((cfg_.use_actor_list_filter == true) && (b_is_carla_actor_list_info_ == false)) {
    //     ROS_ERROR_THROTTLE(1.0,"Wait for Ego Vehicle Odometry...");
    //     return;
    // }

    interface::VehicleState vehicle_state; {
        std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
        vehicle_state = i_vehicle_state_;
    }
    interface::TrackObjects track_objects; {
        std::lock_guard<std::mutex> lock(mutex_carla_objects_);
        track_objects = GetCARLAObjects(i_carla_objects_, vehicle_state);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    o_tracked_objects_ = ros_bridge::UpdateTrackObjects(track_objects);
    o_rviz_objects_ = ros_bridge::UpdateRvizTrackObjects(track_objects);
}

interface::TrackObjects ObjectDriver::GetCARLAObjects(const derived_object_msgs::ObjectArray& msg,const interface::VehicleState& vehicle_state){
    interface::TrackObjects objects;
    objects.header = ros_bridge::GetHeader(msg.header);
    objects.header.frame_id = "world";
    for (const auto& i_object : msg.objects) {
        double distance_with_vehicle = 
            sqrt(
                  pow(i_ego_vehicle_odometry_.pose.pose.position.x - i_object.pose.position.x,2) 
                + pow(i_ego_vehicle_odometry_.pose.pose.position.y - i_object.pose.position.y,2)
            );
        if (distance_with_vehicle > cfg_.roi 
            || distance_with_vehicle < 1.0 
            || i_object.id == i_ego_vehicle_info_.id
            || (cfg_.use_actor_list_filter == true && i_object.id == cfg_.ego_vehicle_id)){ // 
            continue;
        }
        // || (i_object.classification < 4 || i_object.classification > 9)
        interface::TrackObject object;
        object.id = i_object.id;
        switch (i_object.classification)
        {
        case 0:
            object.classification == interface::ObjectClass::UNKNOWN;
            break;
        case 1:
            object.classification == interface::ObjectClass::UNKNOWN;
            break;
        case 2:
            object.classification == interface::ObjectClass::UNKNOWN;
            break;
        case 3:
            object.classification == interface::ObjectClass::UNKNOWN;
            break;
        case 4:
            object.classification == interface::ObjectClass::PEDESTRIAN;
            break;
        case 5:
            object.classification == interface::ObjectClass::MOTORCYCLE; // Bike
            break;
        case 6:
            object.classification == interface::ObjectClass::CAR;
            break;
        case 7:
            object.classification == interface::ObjectClass::TRUCK;
            break;
        case 8:
            object.classification == interface::ObjectClass::MOTORCYCLE; 
            break;
        case 9:
            object.classification == interface::ObjectClass::CAR; // Other Vehicle
            break;
        case 10:
            object.classification == interface::ObjectClass::BARRIER;
            break;
        case 11:
            object.classification == interface::ObjectClass::TRAFFIC_SIGN;
            break;
        default:
            object.classification == interface::ObjectClass::UNKNOWN;
            break;
        }
        // object.classification = ;//interface::ObjectClass::CAR;
        // object.dynamic_state;

        object.dimension.length = i_object.shape.dimensions[0];
        object.dimension.width  = i_object.shape.dimensions[1];
        object.dimension.height = i_object.shape.dimensions[2];

        object.state.header = objects.header;

        tf::Quaternion quat(
            i_object.pose.orientation.x, // x
            i_object.pose.orientation.y, // y
            i_object.pose.orientation.z, // z
            i_object.pose.orientation.w  // w
        );
        double roll,pitch,yaw;
        tf::Matrix3x3 quaternion_mat(quat);
        quaternion_mat.getRPY(roll,pitch,yaw);

        // Convert ego_yaw to radians
        double theta = 0.0;//yaw;

        // Rotation matrix elements
        double cos_theta = cos(theta);
        double sin_theta = sin(theta);

        // Perform rotation
        double global_vx = i_object.twist.linear.x * cos_theta 
                            - i_object.twist.linear.y * sin_theta;
        double global_vy = i_object.twist.linear.x * sin_theta 
                            + i_object.twist.linear.y * cos_theta;

        double global_ax = i_object.accel.linear.x * cos_theta 
                            - i_object.accel.linear.y * sin_theta;
        double global_ay = i_object.accel.linear.x * sin_theta 
                            + i_object.accel.linear.y * cos_theta;

        object.state.header = objects.header;
        object.state.yaw    = yaw;
        object.state.v_x    = global_vx;
        object.state.v_y    = global_vy;
        object.state.a_x    = global_ax;
        object.state.a_y    = global_ay;

        // 1. CARLA Postion to GPS (reverse projection)
        lanelet::GPSPoint object_gps_point;
        double scale = std::cos(i_ref_origin_.wgs84.latitude * M_PI / 180.0);
        double mx = scale * i_ref_origin_.wgs84.longitude * M_PI * interface::EARTH_RADIUS_EQUA / 180.0;
        double my = scale * interface::EARTH_RADIUS_EQUA * std::log(std::tan((90.0 + i_ref_origin_.wgs84.latitude) * M_PI / 360.0));
        mx += i_object.pose.position.x;
        my -= -i_object.pose.position.y;

        object_gps_point.lon = mx * 180.0 / (M_PI * interface::EARTH_RADIUS_EQUA * scale);
        object_gps_point.lat = 360.0 * std::atan(std::exp(my / (interface::EARTH_RADIUS_EQUA * scale))) / M_PI - 90.0;
        object_gps_point.ele = i_object.pose.position.z;
        // 2. GPS to World Position (forward projection)
        lanelet::BasicPoint3d object_autohyu_projpos;
        if(i_ref_origin_.projection == "utm"){
            lanelet::projection::UtmProjector autohyu_projector(lanelet::Origin({i_ref_origin_.wgs84.latitude, i_ref_origin_.wgs84.longitude}));
            object_autohyu_projpos = autohyu_projector.forward(object_gps_point);
        }else if(i_ref_origin_.projection == "local_cartesian"){
            lanelet::projection::LocalCartesianProjector autohyu_projector(lanelet::Origin({i_ref_origin_.wgs84.latitude, i_ref_origin_.wgs84.longitude}));
            object_autohyu_projpos = autohyu_projector.forward(object_gps_point);
        }

        object.state.x = object_autohyu_projpos.x();
        object.state.y = object_autohyu_projpos.y();
        object.state.z = object_autohyu_projpos.z();

        // object.state.x = i_object.pose.position.x;
        // object.state.y = i_object.pose.position.y;
        // object.state.z = i_object.pose.position.z;

        objects.object.push_back(object);
    }

    return objects;
}

void ObjectDriver::Publish() {
    p_track_objects_.publish(o_tracked_objects_);
    p_rviz_track_objects_.publish(o_rviz_objects_);
}

void ObjectDriver::ProcessRosparam(const ros::NodeHandle& nh) {
    // ROS param init
    if (!nh.getParam("/object_driver/roi", cfg_.roi)) {
        cfg_.roi = 100.0;
    }
    if (!nh.getParam("/object_driver/role_name", cfg_.role_name)) {
        cfg_.role_name = "hero";
    }
    ROS_WARN_STREAM("Get Ros Param... ROI: " << cfg_.roi);
}

void ObjectDriver::ProcessINI() {
    if (util_ini_parser_.IsFileUpdated()) { 
        util_ini_parser_.ParseConfig("Carla Object driver", "use_actor_list_filter", cfg_.use_actor_list_filter);  
        util_ini_parser_.ParseConfig("Carla Object driver", "ego_veicle_type", cfg_.ego_veicle_type);  
        util_ini_parser_.ParseConfig("Carla Object driver", "ego_vehicle_id", cfg_.ego_vehicle_id);  
        ROS_INFO("[Carla Vehicle control] Ini file is updated!");
    }
}

int main(int argc, char** argv) {
    std::string node_name = "object_driver";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_object_driver", period))
        period = 1.0;

    ObjectDriver main_task(node_name, period);
    main_task.Exec();

    return 0;
}