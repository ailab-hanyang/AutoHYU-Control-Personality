/**
 * @file        vehicle_driver_node.hpp
 * @brief       carmaker vehicle driver node hpp file
 * 
 * @authors     Jeonghun Kang (kjhoon9674@gmail.com)          
 * 
 * @date        2024-05-16 created by Jeonghun Kang
 * 
 */
#include "vehicle_driver/vehicle_driver_node.hpp"

VehicleDriver::VehicleDriver(std::string node_name, double period)
    : TaskManager(node_name, period),
    deq_d_sim_ax_time_window_(),
    deq_d_sim_ay_time_window_(),
    deq_d_sim_vx_time_window_(),
    deq_d_sim_vy_time_window_(),
    deq_d_sim_yaw_vel_time_window_() {
    // Initialize
    ROS_WARN_STREAM("[" << node_name << "] Initialize node (Period: " << 1/period << " Hz)");

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
    s_cm_vehicle_state_ = nh.subscribe(
            "carmaker/uaq_out", 10, &VehicleDriver::CallbackCMUAQOut, this);
    s_cm_gnss_ = nh.subscribe(
            "carmaker/gnss", 10, &VehicleDriver::CallbackCMGNSS, this);

    // Publisher init
    p_vehicle_state_ = nh.advertise<autohyu_msgs::VehicleState>(
            "app/loc/vehicle_state", 10);
    p_vehicle_state_filtered_ = nh.advertise<autohyu_msgs::VehicleState>(
            "app/loc/vehicle_state_filtered", 10);
    p_reference_point_ = nh.advertise<autohyu_msgs::Reference>(
        "app/loc/reference_point", 10);

    // Algorithm init
    is_initialized = false;
    i_cm_gnss_.latitude = 0.0;
    i_cm_gnss_.longitude = 0.0;    
}

VehicleDriver::~VehicleDriver() {}

void VehicleDriver::ProcessRosparam(const ros::NodeHandle& nh) {
    // ROS param init
    if (!nh.getParam("/vehicle_driver/ref_lat", cfg_.ref_lat)) {
        cfg_.ref_lat = 0.0;
    }
    if (!nh.getParam("/vehicle_driver/ref_lon", cfg_.ref_lon)) {
        cfg_.ref_lon = 0.0;
    }
    if (!nh.getParam("/vehicle_driver/proj_mode", cfg_.projection_mode)) {
        cfg_.projection_mode = "";
    }
    ROS_WARN_STREAM("Reference Lat: " << cfg_.ref_lat << ", Lon: " << cfg_.ref_lon);
}

void VehicleDriver::Run() {
    ProcessINI();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    Gnss gnss; {
        std::lock_guard<std::mutex> lock(mutex_cm_gnss_);
        gnss = i_cm_gnss_;
    }
    VehicleState vehicle_state; {
        std::lock_guard<std::mutex> lock(mutex_cm_uaq_out_);
        vehicle_state = i_cm_uaq_out_;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    v_gps_point_.lat = gnss.latitude;
    v_gps_point_.lon = gnss.longitude;
    v_gps_point_.ele = 0.0;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm 
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    Position position;
    Reference reference_point;
    position.header.stamp = gnss.header.stamp;
    if (gnss.latitude != 0.0 && gnss.longitude != 0.0) {
        
        
        if(cfg_.projection_mode == "utm"){
            lanelet::projection::UtmProjector projector(lanelet::Origin({cfg_.ref_lat, cfg_.ref_lon}));
            lanelet::BasicPoint3d utm_projpos = projector.forward(v_gps_point_);
            position.x = utm_projpos.x();
            position.y = utm_projpos.y();
            position.z = utm_projpos.z();
        }else if(cfg_.projection_mode == "local_cartesian"){
            lanelet::projection::LocalCartesianProjector projector(lanelet::Origin({cfg_.ref_lat, cfg_.ref_lon}));
            lanelet::BasicPoint3d localcartesian_projpos = projector.forward(v_gps_point_);
            position.x = localcartesian_projpos.x();
            position.y = localcartesian_projpos.y();
            position.z = localcartesian_projpos.z();
        }
        
        UpdateVehicleState(position, vehicle_state);
    }
    reference_point.wgs84.latitude  = cfg_.ref_lat;
    reference_point.wgs84.longitude = cfg_.ref_lon;
    reference_point.projection      = cfg_.projection_mode;
    o_reference_point_ = ros_bridge::UpdateReference(reference_point);
    
}

void VehicleDriver::Publish() {
    p_vehicle_state_.publish(o_vehicle_state_);
    p_vehicle_state_filtered_.publish(o_vehicle_state_filtered_);
    p_reference_point_.publish(o_reference_point_);
}

void VehicleDriver::ProcessINI() {
    if (util_ini_parser_.IsFileUpdated()) {
        ROS_WARN("[Vehicle driver] Ini file is updated!");
        util_ini_parser_.ParseConfig("Carmaker Vehicle driver", "cfg_i_sim_ax_ay_time_window", cfg_.i_sim_ax_ay_time_window);
        util_ini_parser_.ParseConfig("Carmaker Vehicle driver", "cfg_i_sim_vx_vy_time_window", cfg_.i_sim_vx_vy_time_window);
        util_ini_parser_.ParseConfig("Carmaker Vehicle driver", "cfg_i_sim_yaw_vel_time_window_", cfg_.i_sim_yaw_vel_time_window);
        util_ini_parser_.ParseConfig("Carmaker Vehicle driver", "cfg_cnt", cfg_.cnt);
        util_ini_parser_.ParseConfig("Carmaker Vehicle driver", "cfg_ay_lpf", cfg_.ay_lpf);
        util_ini_parser_.ParseConfig("Carmaker Vehicle driver", "cfg_vy_lpf", cfg_.vy_lpf);
        util_ini_parser_.ParseConfig("Carmaker Vehicle driver", "cfg_ay_noise", cfg_.ay_noise);
        util_ini_parser_.ParseConfig("Carmaker Vehicle driver", "cfg_vy_noise", cfg_.vy_noise);
    }
}


VehicleState VehicleDriver::GetCMUAQOut(const carmaker_msgs::UAQ_Out& msg){
    carmaker_msgs::UAQ_Out i_uaq_out = msg;

    VehicleState vehicle_state;
    vehicle_state.header.stamp = ros_bridge::GetTimeStamp(i_uaq_out.time);
    // vehicle_state.quality = NovatelPosType::INS_RTKFIXED;
    
    vehicle_state.vx = i_uaq_out.Sensor_Inertial_0_Vel_B_x;
    vehicle_state.vy = i_uaq_out.Sensor_Inertial_0_Vel_B_y;
    vehicle_state.vz = i_uaq_out.Sensor_Inertial_0_Vel_B_z;
    
    vehicle_state.ax = i_uaq_out.Sensor_Inertial_0_Acc_B_x;
    vehicle_state.ay = i_uaq_out.Sensor_Inertial_0_Acc_B_y;
    vehicle_state.az = i_uaq_out.Sensor_Inertial_0_Acc_B_z;

    vehicle_state.roll  = i_uaq_out.Car_Roll;
    vehicle_state.pitch = i_uaq_out.Car_Pitch;
    vehicle_state.yaw   = tfNormalizeAngle(i_uaq_out.Car_Yaw);

    vehicle_state.roll_vel  = i_uaq_out.Sensor_Inertial_0_Omega_B_x;
    vehicle_state.pitch_vel = i_uaq_out.Sensor_Inertial_0_Omega_B_y;
    vehicle_state.yaw_vel   = i_uaq_out.Sensor_Inertial_0_Omega_B_z;

    vehicle_state.vehicle_can.lateral_accel         = i_uaq_out.Sensor_Inertial_0_Acc_B_y;
    vehicle_state.vehicle_can.longitudinal_accel    = i_uaq_out.Sensor_Inertial_0_Acc_B_x;
    vehicle_state.vehicle_can.yaw_rate              = i_uaq_out.Sensor_Inertial_0_Omega_B_z;
            
    vehicle_state.vehicle_can.steering_wheel_angle  = i_uaq_out.Steer_WhlAng;
    vehicle_state.vehicle_can.steering_tire_angle   = i_uaq_out.Steer_WhlAng/STEERING_RATIO_CARMAKER;//(i_uaq_out.Vhcl_FL_rz + i_uaq_out.Vhcl_FR_rz)/2.0;
    vehicle_state.vehicle_can.steering_speed        = 0.0; // TODO
    vehicle_state.vehicle_can.steering_torque       = 0.0; // TODO
    vehicle_state.vehicle_can.steering_state        = MdpsState::MDPS_ACTIVATE;

    vehicle_state.vehicle_can.wheel_velocity_fl = 0.0; // TODO
    vehicle_state.vehicle_can.wheel_velocity_fr = 0.0; // TODO
    vehicle_state.vehicle_can.wheel_velocity_rl = 0.0; // TODO
    vehicle_state.vehicle_can.wheel_velocity_rr = 0.0; // TODO

    vehicle_state.vehicle_can.motor_torque_f     = 0.0; // TODO
    vehicle_state.vehicle_can.motor_torque_r     = 0.0; // TODO
    vehicle_state.vehicle_can.motor_torque_total = 0.0; // TODO

    vehicle_state.vehicle_can.accel_position     = i_uaq_out.VC_Gas;
    vehicle_state.vehicle_can.brake_pressure     = i_uaq_out.VC_Brake;
    vehicle_state.vehicle_can.brake_active       = 0.0; // TODO
    vehicle_state.vehicle_can.gear_select        = Gear::GEAR_D; // TODO

    vehicle_state.vehicle_can.operation_mode                = OperationMode::AUTONOMOUS;
    vehicle_state.vehicle_can.lateral_autonomous_mode       = AutonomousMode::RUN;
    vehicle_state.vehicle_can.longitudinal_autonomous_mode  = AutonomousMode::RUN;

    return vehicle_state;
}

Gnss VehicleDriver::GetCMGNSS(const carmaker_msgs::GNSS& msg) {
    carmaker_msgs::GNSS i_gnss = msg;

    Gnss gnss;
    gnss.header.stamp = ros_bridge::GetTimeStamp(i_gnss.header.stamp);
    gnss.latitude = i_gnss.latitude;
    gnss.longitude = i_gnss.longitude;
    gnss.altitude = i_gnss.altitude;
    gnss.HDOP = i_gnss.hdop;

    // ROS_WARN_STREAM(i_gnss.latitude << " " << i_gnss.longitude << " " << i_gnss.altitude);

    return gnss;
}

double VehicleDriver::GenerateGaussianNoise(double mean, double std_dev) {
    // This function is for generating gaussian noise
    static std::random_device        rd;
    std::default_random_engine       gen(rd());
    std::normal_distribution<double> dist(mean, std_dev);

    return dist(gen);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
// Update functions for publish variables
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
void VehicleDriver::UpdateVehicleState(const Position& position, 
                                       const VehicleState& cm_vehicle_state){
    autohyu_msgs::VehicleState vehicle_state;

    static double vy_noise = 0.0;
    static double ay_noise = 0.0;

    static double vy_lpf_noise = 0.0;
    static double ay_lpf_noise = 0.0;

    static uint16_t cnt = 0;

    if(cnt > cfg_.cnt){
        vy_noise = GenerateGaussianNoise(0.0, cfg_.vy_noise);
        ay_noise = GenerateGaussianNoise(0.0, cfg_.ay_noise);

        cnt = 0;
    }
    cnt++;

    vy_lpf_noise = cfg_.vy_lpf*vy_noise + (1.0-cfg_.vy_lpf)*vy_lpf_noise;
    ay_lpf_noise = cfg_.ay_lpf*ay_noise + (1.0-cfg_.ay_lpf)*ay_lpf_noise;

    vehicle_state.header.stamp = ros::Time(position.header.stamp);

    vehicle_state.x = position.x;
    vehicle_state.y = position.y;
    vehicle_state.z = position.z;

    vehicle_state.vx = cm_vehicle_state.vx + GenerateGaussianNoise(0.0, 0.01);
    vehicle_state.vy = cm_vehicle_state.vy + vy_lpf_noise; 
    vehicle_state.vz = cm_vehicle_state.vz;

    vehicle_state.ax = cm_vehicle_state.ax ;
    vehicle_state.ay = cm_vehicle_state.ay + ay_lpf_noise;
    vehicle_state.az = cm_vehicle_state.az;

    vehicle_state.roll  = cm_vehicle_state.roll;
    vehicle_state.pitch = cm_vehicle_state.pitch;
    vehicle_state.yaw   = cm_vehicle_state.yaw;
    
    vehicle_state.roll_vel  = cm_vehicle_state.roll_vel;
    vehicle_state.pitch_vel = cm_vehicle_state.pitch_vel;
    vehicle_state.yaw_vel   = cm_vehicle_state.yaw_vel + GenerateGaussianNoise(0.0, 0.01);

    vehicle_state.vehicle_can.lateral_accel         = cm_vehicle_state.vehicle_can.lateral_accel;
    vehicle_state.vehicle_can.longitudinal_accel    = cm_vehicle_state.vehicle_can.longitudinal_accel;
    vehicle_state.vehicle_can.yaw_rate              = cm_vehicle_state.vehicle_can.yaw_rate;

    vehicle_state.vehicle_can.steering_wheel_angle  = cm_vehicle_state.vehicle_can.steering_wheel_angle;
    vehicle_state.vehicle_can.steering_tire_angle   = cm_vehicle_state.vehicle_can.steering_tire_angle;
    vehicle_state.vehicle_can.steering_speed        = cm_vehicle_state.vehicle_can.steering_speed;
    vehicle_state.vehicle_can.steering_torque       = cm_vehicle_state.vehicle_can.steering_torque;
    vehicle_state.vehicle_can.steering_state        = static_cast<MdpsState>(cm_vehicle_state.vehicle_can.steering_state);

    vehicle_state.vehicle_can.wheel_velocity_fl = cm_vehicle_state.vehicle_can.wheel_velocity_fl;
    vehicle_state.vehicle_can.wheel_velocity_fr = cm_vehicle_state.vehicle_can.wheel_velocity_fr;
    vehicle_state.vehicle_can.wheel_velocity_rl = cm_vehicle_state.vehicle_can.wheel_velocity_rl;
    vehicle_state.vehicle_can.wheel_velocity_rr = cm_vehicle_state.vehicle_can.wheel_velocity_rr;

    vehicle_state.vehicle_can.motor_torque_f     = cm_vehicle_state.vehicle_can.motor_torque_f;
    vehicle_state.vehicle_can.motor_torque_r     = cm_vehicle_state.vehicle_can.motor_torque_r;
    vehicle_state.vehicle_can.motor_torque_total = cm_vehicle_state.vehicle_can.motor_torque_f + cm_vehicle_state.vehicle_can.motor_torque_r;

    vehicle_state.vehicle_can.accel_position = cm_vehicle_state.vehicle_can.accel_position;
    vehicle_state.vehicle_can.brake_pressure = cm_vehicle_state.vehicle_can.brake_pressure;
    vehicle_state.vehicle_can.brake_active   = cm_vehicle_state.vehicle_can.brake_active;
    vehicle_state.vehicle_can.gear_select    = static_cast<Gear>(cm_vehicle_state.vehicle_can.gear_select);

    vehicle_state.vehicle_can.operation_mode                = static_cast<OperationMode>(cm_vehicle_state.vehicle_can.operation_mode);
    vehicle_state.vehicle_can.lateral_autonomous_mode       = static_cast<AutonomousMode>(cm_vehicle_state.vehicle_can.lateral_autonomous_mode);
    vehicle_state.vehicle_can.longitudinal_autonomous_mode  = static_cast<AutonomousMode>(cm_vehicle_state.vehicle_can.longitudinal_autonomous_mode);
    
    o_vehicle_state_ = vehicle_state;
    o_vehicle_state_filtered_ = vehicle_state;

    // Sim vx,vy filtering
    double window_vx = 0.0;
    double window_vy = 0.0;

    // Vx Vy 
    if ( cfg_.i_sim_vx_vy_time_window <= 1 ) {
        window_vx = vehicle_state.vx;
        window_vy = vehicle_state.vy;
    }
    else {
        // Buffer manage
        deq_d_sim_vx_time_window_.push_back(vehicle_state.vx);
        deq_d_sim_vy_time_window_.push_back(vehicle_state.vy);

        while ( deq_d_sim_vx_time_window_.size() > cfg_.i_sim_vx_vy_time_window )
            deq_d_sim_vx_time_window_.pop_front();
        while ( deq_d_sim_vy_time_window_.size() > cfg_.i_sim_vx_vy_time_window )
            deq_d_sim_vy_time_window_.pop_front();
        // moving average
        window_vx =
                std::accumulate(deq_d_sim_vx_time_window_.begin(), deq_d_sim_vx_time_window_.end(), 0.0) /
                deq_d_sim_vx_time_window_.size();
        window_vy =
                std::accumulate(deq_d_sim_vy_time_window_.begin(), deq_d_sim_vy_time_window_.end(), 0.0) /
                deq_d_sim_vy_time_window_.size();
    }
    o_vehicle_state_filtered_.vx = window_vx;
    o_vehicle_state_filtered_.vy = window_vy;

    // Sim ax,ay filtering
    double window_ax = 0.0;
    double window_ay = 0.0;
    
    // Ax Ay 
    if ( cfg_.i_sim_ax_ay_time_window <= 1 ) {
        window_ax = vehicle_state.ax;
        window_ay = vehicle_state.ay;
    }
    else {
        // Buffer manage
        deq_d_sim_ax_time_window_.push_back(vehicle_state.ax);
        deq_d_sim_ay_time_window_.push_back(vehicle_state.ay);

        while ( deq_d_sim_ax_time_window_.size() > cfg_.i_sim_ax_ay_time_window )
            deq_d_sim_ax_time_window_.pop_front();
        while ( deq_d_sim_ay_time_window_.size() > cfg_.i_sim_ax_ay_time_window )
            deq_d_sim_ay_time_window_.pop_front();
        // moving average
        window_ax =
                std::accumulate(deq_d_sim_ax_time_window_.begin(), deq_d_sim_ax_time_window_.end(), 0.0) /
                deq_d_sim_ax_time_window_.size();
        window_ay =
                std::accumulate(deq_d_sim_ay_time_window_.begin(), deq_d_sim_ay_time_window_.end(), 0.0) /
                deq_d_sim_ay_time_window_.size();
    }
    o_vehicle_state_filtered_.ax = window_ax;
    o_vehicle_state_filtered_.ay = window_ay;

    // Sim ax,ay filtering
    double window_yaw_vel = 0.0;
    
    // Yaw vel
    if ( cfg_.i_sim_yaw_vel_time_window <= 1 ) {
        window_yaw_vel = vehicle_state.yaw_vel;
    }
    else {
        // Buffer manage
        deq_d_sim_yaw_vel_time_window_.push_back(vehicle_state.yaw_vel);

        while ( deq_d_sim_yaw_vel_time_window_.size() > cfg_.i_sim_yaw_vel_time_window )
            deq_d_sim_yaw_vel_time_window_.pop_front();
        // moving average
        window_yaw_vel =
                std::accumulate(deq_d_sim_yaw_vel_time_window_.begin(), deq_d_sim_yaw_vel_time_window_.end(), 0.0) /
                deq_d_sim_yaw_vel_time_window_.size();
    }
    o_vehicle_state_filtered_.yaw_vel = window_yaw_vel;

}

int main(int argc, char** argv) {
    std::string node_name = "vehicle_driver";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_vehicle_driver", period)){
        period = 1.0;
    }
        
    VehicleDriver main_task(node_name, period);
    main_task.Exec();

    return 0;
}