/**
 * @file        traffic_driver_node.hpp
 * @brief       morai traffic driver node hpp file
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-06-20 created by Jeonghun Kang
 * 
 */
#include "traffic_driver/traffic_driver_node.hpp"

TrafficDriver::TrafficDriver(std::string node_name, double period)
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
    s_mr_traffic_light_ = nh.subscribe(
            "GetTrafficLightStatus", 10, &TrafficDriver::CallbackMRTrafficLight, this);

    // Publisher init
    p_traffic_light_ = nh.advertise<autohyu_msgs::TrafficLight>(
            "app/perc/traffic_light", 10);

    // Algorithm init
    is_initialized = false;
}

TrafficDriver::~TrafficDriver() {}

void TrafficDriver::ProcessRosparam(const ros::NodeHandle& nh) {
    // ROS param init
    // if (!nh.getParam("/vehicle_driver/ref_lat", cfg_.ref_lat)) {
    //     cfg_.ref_lat = 0.0;
    // }
    // ROS_WARN_STREAM("Reference Lat: " << cfg_.ref_lat << ", Lon: " << cfg_.ref_lon);
}
void TrafficDriver::Run() {
    ProcessINI();
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    TrafficLight traffic_light; {
        std::lock_guard<std::mutex> lock(mutex_mr_traffic_light_);
        traffic_light = i_mr_traffic_light_;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    UpdateTrafficLight(traffic_light);
}

void TrafficDriver::Publish() {
    p_traffic_light_.publish(o_traffic_light_);
}

void TrafficDriver::ProcessINI() {
    if (util_ini_parser_.IsFileUpdated()) {
        ROS_WARN("[MORAI Traffic driver] Ini file is updated!");
        // util_ini_parser_.ParseConfig("Morai Vehicle driver", "cfg_i_sim_ax_ay_time_window", cfg_.i_sim_ax_ay_time_window);
    }
}

TrafficLight TrafficDriver::GetMRTrafficLight(const morai_msgs::GetTrafficLightStatus& msg) {
    morai_msgs::GetTrafficLightStatus i_traffic_light = msg;
    TrafficLight traffic_light;
    traffic_light.header.stamp = ros_bridge::GetTimeStamp(i_traffic_light.header.stamp);
    switch (i_traffic_light.trafficLightStatus)
    {
    case MRTrafficLightStatus::R:
        traffic_light.color = Color::RED;
        break;
    case MRTrafficLightStatus::Y:
        traffic_light.color = Color::YELLOW;
        break;
    case MRTrafficLightStatus::G:
        traffic_light.color = Color::GREEN;
        break;
    case MRTrafficLightStatus::G_W_L:
        traffic_light.color = Color::TURN;
        break;
    default:
        traffic_light.color = Color::RED;
        break;
    }
    return traffic_light;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
// Update functions for publish variables
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
void TrafficDriver::UpdateTrafficLight(const TrafficLight& traffic_light){
    autohyu_msgs::TrafficLight o_traffic_light;

    o_traffic_light.header.stamp = ros::Time(traffic_light.header.stamp);
    o_traffic_light.color = traffic_light.color;

    o_traffic_light_ = o_traffic_light;
}

int main(int argc, char** argv) {
    std::string node_name = "traffic_driver";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_traffic_driver", period)){
        period = 1.0;
    }
    
    TrafficDriver main_task(node_name, period);
    main_task.Exec();

    return 0;
}