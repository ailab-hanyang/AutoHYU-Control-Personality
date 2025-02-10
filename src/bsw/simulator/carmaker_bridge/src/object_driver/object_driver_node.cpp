#include "object_driver/object_driver_node.hpp"

ObjectDriver::ObjectDriver(std::string node_name, double period)
    : TaskManager(node_name, period) {
    // Initialize
    ROS_WARN_STREAM("[" << node_name << "] Initialize node (Period: " << 1/period << " Hz)");
    // Node initialization
    ros::NodeHandle nh;

    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/carmaker.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());

    // Parameter init
    ProcessINI();
    ProcessRosparam(nh);
    
    // Subscriber init
    s_vehicle_state_ = nh.subscribe(
            "app/loc/vehicle_state", 10, &ObjectDriver::CallbackVehicleState, this);   
    s_cm_objects_ = nh.subscribe(
            "carmaker/objects", 10, &ObjectDriver::CallbackCMObjects, this);
    s_ref_origin_ = nh.subscribe(
        "app/loc/reference_point", 10, &ObjectDriver::CallbackReference, this);
    // s_global_trajectory_ = nh.subscribe(
    //         "app/pla/global_trajectory", 10, &ObjectDriver::CallbackGlobalTrajectory, this);

    // Publisher init
    p_lidar_objects_ = nh.advertise<autohyu_msgs::DetectObjects3D>(
            "app/perc/lidar_objects", 10);
    p_track_objects_ = nh.advertise<autohyu_msgs::TrackObjects>(
            "app/perc/track_objects", 10);
    p_rviz_track_objects_ = nh.advertise<visualization_msgs::MarkerArray>(
            "hmi/perc/track_objects", 10);
    p_mobileye_corner_radar_ = nh.advertise<mobileye_msgs::C_RDR>(
            "app/perc/mobileye_c_radar",10);
            
    // Algorithm init    

}

ObjectDriver::~ObjectDriver() {}

void ObjectDriver::ProcessRosparam(const ros::NodeHandle& nh) {
    // ROS param init
    if (!nh.getParam("/object_driver/roi", cfg_.roi)) {
        cfg_.roi = 0.0;
    }
    ROS_WARN_STREAM("ROI : " << cfg_.roi);
}

void ObjectDriver::Run() {
    ProcessINI();    
    if (b_is_vehicle_state_ == false) {
        ROS_ERROR_THROTTLE(1.0,"[Object Driver] No vehicle state...");
        return;
    }
    if (b_is_ref_origin_ == false) {
        ROS_ERROR_THROTTLE(1.0,"[Object Driver] Wait for Reference Points...");
        return;
    }
    // if (b_is_global_trajectory_ == false) {
    //     ROS_WARN("[Object Driver] No global trajectory!");
    //     return;
    // }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //      
    VehicleState vehicle_state; {
        std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
        vehicle_state = i_vehicle_state_;
    } 
    // Trajectory global_trajectory; {
    //     std::lock_guard<std::mutex> lock(mutex_global_trajectory_);
    //     global_trajectory = i_global_trajectory_;
    // }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // DetectObjects3D object = GetObjects(vehicle_state, global_trajectory);
    DetectObjects3D object = GetObjects(vehicle_state);
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    UpdateObjects(object);
    UpdateMobileyeCornerRadar(object, vehicle_state);
}

void ObjectDriver::Publish() {
    p_lidar_objects_.publish(o_lidar_objects_);
    p_track_objects_.publish(o_tracked_objects_);
    p_rviz_track_objects_.publish(o_rviz_objects_);
    p_mobileye_corner_radar_.publish(o_mobileye_corner_radar_);
}

void ObjectDriver::ProcessINI() {}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
// Get functions for subscribe variables
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
DetectObjects3D ObjectDriver::GetObjects(const VehicleState& vehicle_state) {
    mutex_cm_objects_.lock();
    carmaker_msgs::Objects i_objects = i_cm_objects_;
    mutex_cm_objects_.unlock();

    // tk::spline left_boundary; tk::spline right_boundary;
    // tk::Map road_map;
    // util_function::GenerateRoadMap(global_trajectory, road_map);
    // util_function::GenerateBoundarySpline(global_trajectory, left_boundary, right_boundary);
    // std::vector<double> ego_frenet = road_map.ToFrenet(vehicle_state.x, vehicle_state.y);

    DetectObjects3D objects;
    objects.header.stamp = (double)i_objects.header.stamp.sec
                       + (double)i_objects.header.stamp.nsec * 1e-9; 
    for (auto i_object : i_objects.object) {
        std::cout << "here"<<std::endl;
        DetectObject3D object;
        object.id = i_object.id;
        object.classification = (ObjectClass)i_object.classification;

        object.dimension.length = i_object.length;
        object.dimension.width = i_object.width;
        object.dimension.height = i_object.height;

        object.state.header.stamp = objects.header.stamp;
        object.state.yaw = i_object.state.yaw;
        object.state.v_x = i_object.state.v_x;
        object.state.v_y = i_object.state.v_y;
        object.state.a_x = i_object.state.a_x;
        object.state.a_y = i_object.state.a_y;

        // if (sqrt(pow(object.state.v_x, 2) + pow(object.state.v_y, 2)) < 0.1) {
        //     object.dynamic_state = ObjectDynamicState::STATIC;
        // }
        // else {
        //     object.dynamic_state = ObjectDynamicState::DYNAMIC;
        // }
        
        lanelet::GPSPoint object_gps_point;
        object_gps_point.lat = i_object.state.latitude;
        object_gps_point.lon = i_object.state.longitude;
        object_gps_point.ele = i_object.state.elevation;

        lanelet::BasicPoint3d object_autohyu_projpos;
        if(i_ref_origin_.projection == "utm"){
            lanelet::projection::UtmProjector autohyu_projector(lanelet::Origin({i_ref_origin_.wgs84.latitude, i_ref_origin_.wgs84.longitude}));
            object_autohyu_projpos = autohyu_projector.forward(object_gps_point);
        }else if(i_ref_origin_.projection == "local_cartesian"){
            lanelet::projection::LocalCartesianProjector autohyu_projector(lanelet::Origin({i_ref_origin_.wgs84.latitude, i_ref_origin_.wgs84.longitude}));
            object_autohyu_projpos = autohyu_projector.forward(object_gps_point);
        }

        object.state.x = object_autohyu_projpos.x() + object.dimension.length/2.0 * cos(object.state.yaw);
        object.state.y = object_autohyu_projpos.y() + object.dimension.length/2.0 * sin(object.state.yaw);

        object.state.x = i_object.state.x;
        object.state.y = i_object.state.y;

        double distance_with_vehicle = 
            sqrt(
                  pow(vehicle_state.x - object.state.x,2) 
                + pow(vehicle_state.y - object.state.y,2)
            );
        
        if(distance_with_vehicle > cfg_.roi){
            continue;
        }

        // if (road_map.ClosestDistance(object.state.x, object.state.y) > 15.0) {
        //     continue;
        // }

        // Frenet roi based check
        // vector<double> object_frenet = road_map.ToFrenet(object.state.x, object.state.y);
        // if (object_frenet.at(0) > ego_frenet.at(0) + 100.0 
        //     || object_frenet.at(0) < ego_frenet.at(0) - 100.0) {
            
        //     continue;
        // }

        // Boundary check
        // if ((object_frenet.at(1) > left_boundary(object_frenet.at(0)) + VEHICLE_WIDTH) ||
        //     (object_frenet.at(1) < right_boundary(object_frenet.at(0)) - VEHICLE_WIDTH)) {

        //     continue;
        // }

        // int out_count = 0;
        // double distance = sqrt(pow(vehicle_state.x - object.state.x, 2) + pow(vehicle_state.y - object.state.y, 2));

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

        objects.object.push_back(object);
    }

    return objects;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
// Update functions for publish variables
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
void ObjectDriver::UpdateObjects(const DetectObjects3D& objects) {    
    UpdateRvizObjects(objects);

    autohyu_msgs::DetectObjects3D o_objects;
    o_objects.header.frame_id = "world";
    o_objects.header.stamp = ros::Time(objects.header.stamp);

    autohyu_msgs::TrackObjects o_fusion_objects;
    o_fusion_objects.header = o_objects.header;

    for (auto object : objects.object) {
        autohyu_msgs::DetectObject3D o_object;
        o_object.id = object.id;
        o_object.classification = ObjectClass::CAR;
        // o_object.dynamic_state = static_cast<ObjectDynamicState>(object.dynamic_state);

        o_object.dimension.length = object.dimension.length;
        o_object.dimension.width = object.dimension.width;
        o_object.dimension.height = object.dimension.height;
        
        o_object.state.header.stamp = ros::Time(object.state.header.stamp);
        o_object.state.x = object.state.x;
        o_object.state.y = object.state.y;
        o_object.state.yaw = object.state.yaw;
        o_object.state.v_x = object.state.v_x;
        o_object.state.v_y = object.state.v_y;
        o_object.state.a_x = object.state.a_x;
        o_object.state.a_y = object.state.a_y;

        o_objects.object.push_back(o_object);

        // Update Fusion DetectObjects3D
        autohyu_msgs::TrackObject fusion_object;
        fusion_object.id = object.id;
        fusion_object.classification = object.classification;
        // fusion_object.dynamic_state = static_cast<ObjectDynamicState>(object.dynamic_state);

        fusion_object.dimension.length = object.dimension.length;
        fusion_object.dimension.width = object.dimension.width;
        fusion_object.dimension.height = object.dimension.height;
        fusion_object.state = o_object.state;

        o_fusion_objects.object.push_back(fusion_object); 
    }
    o_lidar_objects_ = o_objects;
    o_tracked_objects_ = o_fusion_objects;
}

void ObjectDriver::UpdateMobileyeCornerRadar(const DetectObjects3D& objects, const VehicleState& vehicle_state) {    
    UpdateRvizObjects(objects);

    mobileye_msgs::C_RDR o_objects;
    o_objects.header.frame_id = "world";
    o_objects.header.stamp = ros::Time(objects.header.stamp);

    int idx = 0;

    for (auto object : objects.object) {
        mobileye_msgs::ObjectState o_object_state;
        o_object_state.ID = object.id + 1;

        o_object_state.Length = object.dimension.length;
        o_object_state.Width  = object.dimension.width;

        
        o_object_state.header.stamp = ros::Time(object.state.header.stamp);

        double cos_yaw = cos(-vehicle_state.yaw);
        double sin_yaw = sin(-vehicle_state.yaw);
        double dx = object.state.x - vehicle_state.x;
        double dy = object.state.y - vehicle_state.y;
        double local_object_x   = dx * cos_yaw - dy * sin_yaw;
        double local_object_y   = dx * sin_yaw + dy * cos_yaw;
        double local_object_yaw = object.state.yaw - vehicle_state.yaw;

        o_object_state.RelPosX      = local_object_x;
        o_object_state.RelPosY      = local_object_y;
        o_object_state.HeadingAngle = local_object_yaw;

        double cos_yaw_op = cos(vehicle_state.yaw);
        double sin_yaw_op = sin(vehicle_state.yaw);

        double ego_global_vx = vehicle_state.vx * cos_yaw_op - vehicle_state.vy * sin_yaw_op;
        double ego_global_vy = vehicle_state.vx * sin_yaw_op + vehicle_state.vy * cos_yaw_op;

        double relative_global_vx = object.state.v_x - ego_global_vx;
        double relative_global_vy = object.state.v_y - ego_global_vy;

        double relative_vx = relative_global_vx * cos_yaw - relative_global_vy * sin_yaw;
        double relative_vy = relative_global_vx * sin_yaw + relative_global_vy * cos_yaw;

        double ego_global_ax = vehicle_state.ax * cos_yaw_op - vehicle_state.ay * sin_yaw_op;
        double ego_global_ay = vehicle_state.ax * sin_yaw_op - vehicle_state.ay * cos_yaw_op;
        
        double relative_global_ax = object.state.a_x - ego_global_ax;
        double relative_global_ay = object.state.a_y - ego_global_ay;

        double relative_ax = relative_global_ax * cos_yaw - relative_global_ay * sin_yaw;
        double relative_ay = relative_global_ax * sin_yaw + relative_global_ay * cos_yaw;
                

        o_object_state.RelVelX = relative_vx;
        o_object_state.RelVelY = relative_vy;
        o_object_state.RelAccelX = relative_ax;
        o_object_state.RelAccelY = relative_ay;

        // o_object.state.x = object.state.x;
        // o_object.state.y = object.state.y;
        // o_object.state.yaw = object.state.yaw;
        // o_object.state.v_x = object.state.v_x;
        // o_object.state.v_y = object.state.v_y;
        // o_object.state.a_x = object.state.a_x;
        // o_object.state.a_y = object.state.a_y;

        if(idx < 15){
            o_objects.Object[idx] = o_object_state;
            idx += 1;
        }
    }

    if(idx < 15){
        int empty_object_num = 15 - o_objects.Object.size();
        for(int i = 0; i < empty_object_num; i++){
            mobileye_msgs::ObjectState o_object_state;
            o_object_state.ID           = 0;
            o_object_state.Length       = 0;
            o_object_state.Width        = 0;
            o_object_state.RelPosX      = 0;
            o_object_state.RelPosY      = 0;
            o_object_state.RelVelX      = 0;
            o_object_state.RelVelY      = 0;
            o_object_state.RelAccelX    = 0;
            o_object_state.RelAccelY    = 0;
            o_object_state.HeadingAngle = 0;
            o_objects.Object[idx] = o_object_state;
            idx += 1;
        }
        
    }
    o_mobileye_corner_radar_ = o_objects;
}

void ObjectDriver::UpdateRvizObjects(const DetectObjects3D& objects) {
    visualization_msgs::MarkerArray marker_array;
    
    for (auto object : objects.object) {
        visualization_msgs::Marker position_marker;
        visualization_msgs::Marker gt_motion_info_marker;
        
        position_marker.header.frame_id = "world";
        position_marker.header.stamp = ros::Time(objects.header.stamp);
        position_marker.ns = "position";
        position_marker.id = object.id;
        position_marker.action = visualization_msgs::Marker::ADD;
        position_marker.type = visualization_msgs::Marker::CUBE;
        position_marker.lifetime = ros::Duration(0.1);

        gt_motion_info_marker.header.frame_id = "world";
        gt_motion_info_marker.header.stamp = ros::Time::now();
        gt_motion_info_marker.ns = "velocity";
        gt_motion_info_marker.id = object.id;
        gt_motion_info_marker.action = visualization_msgs::Marker::ADD;
        gt_motion_info_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        gt_motion_info_marker.lifetime = ros::Duration(0.1);

        double gt_velocity = sqrt(object.state.v_x*object.state.v_x + object.state.v_y*object.state.v_y);
        double gt_aceeleration = sqrt(object.state.a_x*object.state.a_x + object.state.a_y*object.state.a_y);
        gt_motion_info_marker.text = "V(GT): " + std::to_string(gt_velocity) + "m/s";

        // Line width
        position_marker.scale.x = object.dimension.length;
        position_marker.scale.y = object.dimension.width;
        position_marker.scale.z = object.dimension.height;
        gt_motion_info_marker.scale.z = 1.0;

        // Color space
        position_marker.color.r = 0.5f;
        position_marker.color.g = 0.5f;
        position_marker.color.b = 1.0f;
        position_marker.color.a = 0.3f;
        gt_motion_info_marker.color.r = 0.5f;
        gt_motion_info_marker.color.g = 0.5f;
        gt_motion_info_marker.color.b = 1.0f;
        gt_motion_info_marker.color.a = 1.0f;

        position_marker.pose.position.x = object.state.x;
        position_marker.pose.position.y = object.state.y;
        position_marker.pose.position.z = object.dimension.height/2;
        position_marker.pose.orientation = tf::createQuaternionMsgFromYaw(object.state.yaw);

        gt_motion_info_marker.pose.position.x = object.state.x + 5;
        gt_motion_info_marker.pose.position.y = object.state.y + 5;
        gt_motion_info_marker.pose.position.z = 3.0;
        gt_motion_info_marker.pose.orientation = tf::createQuaternionMsgFromYaw(object.state.yaw);

        marker_array.markers.push_back(position_marker);
        marker_array.markers.push_back(gt_motion_info_marker);
    }

    o_rviz_objects_ = marker_array;
}

int main(int argc, char** argv) {
    std::string node_name = "object_driver";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_object_driver", period)){
        period = 1.0;
    }

    ObjectDriver main_task(node_name, period);
    main_task.Exec();

    return 0;
}