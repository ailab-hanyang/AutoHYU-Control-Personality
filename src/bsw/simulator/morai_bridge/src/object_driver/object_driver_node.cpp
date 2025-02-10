/**
 * @file        object_driver_node.cpp
 * @brief       node cpp file for morai object driver
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)
 *              Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-05-16 created by Jeonghun Kang
 *              2024-07-17 updated by Yuseung Na: Clean up
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
    std::string ini_dir("/config/morai.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());

    // Parameter init
    ProcessINI();
    ProcessRosparam(nh);
    
    // Subscriber init
    s_vehicle_state_ = nh.subscribe(
            "app/loc/vehicle_state", 10, &ObjectDriver::CallbackVehicleState, this);   
    s_morai_objects_ = nh.subscribe(
            "/Object_topic", 10, &ObjectDriver::CallbackMoraiObjects, this);

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
    if (b_is_morai_objects_ == false) {
        ROS_ERROR_THROTTLE(1.0,"Wait for MORAI Objects...");
        return;
    }

    interface::VehicleState vehicle_state; {
        std::lock_guard<std::mutex> lock(mutex_vehicle_state_);
        vehicle_state = i_vehicle_state_;
    }
    interface::TrackObjects track_objects; {
        std::lock_guard<std::mutex> lock(mutex_morai_objects_);
        track_objects = ros_bridge::GetMoraiObjects(i_morai_objects_, cfg_.ref_lat, cfg_.ref_lon);
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

void ObjectDriver::Publish() {
    p_track_objects_.publish(o_tracked_objects_);
    p_rviz_track_objects_.publish(o_rviz_objects_);
}

void ObjectDriver::ProcessRosparam(const ros::NodeHandle& nh) {
    // ROS param init
    if (!nh.getParam("/object_driver/ref_lat", cfg_.ref_lat)) {
        cfg_.ref_lat = 0.0;
    }
    if (!nh.getParam("/object_driver/ref_lon", cfg_.ref_lon)) {
        cfg_.ref_lon = 0.0;
    }
    ROS_WARN_STREAM("Reference Lat: " << cfg_.ref_lat << ", Lon: " << cfg_.ref_lon);

    if (!nh.getParam("/object_driver/roi", cfg_.roi)) {
        cfg_.roi = 100.0;
    }
    ROS_WARN_STREAM("ROI: " << cfg_.roi);
}

void ObjectDriver::ProcessINI() {
    
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