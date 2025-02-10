/**
 * @file        vehicle_driver_node.hpp
 * @brief       morai vehicle driver node hpp file
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
    s_mr_ego_vehicle_status_ = nh.subscribe(
        "Ego_topic", 10, &VehicleDriver::CallbackMRVehicleStatus, this);
    s_mr_gps_ = nh.subscribe(
        "gps", 10, &VehicleDriver::CallbackMRGPS, this);
    s_mr_imu_ = nh.subscribe(
        "imu", 10, &VehicleDriver::CallbackMRIMU, this);

    // Publisher init
    p_vehicle_state_ = nh.advertise<autohyu_msgs::VehicleState>(
        "app/loc/vehicle_state", 10);
    p_vehicle_state_filtered_ = nh.advertise<autohyu_msgs::VehicleState>(
        "app/loc/vehicle_state_filtered", 10);
    p_reference_point_ = nh.advertise<autohyu_msgs::Reference>(
        "app/loc/reference_point", 10);
    // Algorithm init
    is_initialized = false;
    i_mr_gps_.latitude = 0.0;
    i_mr_gps_.longitude = 0.0;   
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
        std::lock_guard<std::mutex> lock(mutex_mr_gps_);
        gnss = i_mr_gps_;
    }
    Motion motion; {
        std::lock_guard<std::mutex> lock(mutex_mr_imu_);
        motion = i_mr_imu_;
    }
    VehicleState vehicle_state; {
        std::lock_guard<std::mutex> lock(mutex_mr_vehicle_status_);
        vehicle_state = i_mr_vehicle_status_;
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
    interface::Position  position;
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
        
        UpdateVehicleState(position, motion, vehicle_state);
    }

    interface::Reference reference_point;
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
        util_ini_parser_.ParseConfig("Morai Vehicle driver", "cfg_i_sim_ax_ay_time_window", cfg_.i_sim_ax_ay_time_window);
        util_ini_parser_.ParseConfig("Morai Vehicle driver", "cfg_i_sim_vx_vy_time_window", cfg_.i_sim_vx_vy_time_window);
        util_ini_parser_.ParseConfig("Morai Vehicle driver", "cfg_i_sim_yaw_vel_time_window_", cfg_.i_sim_yaw_vel_time_window);
        util_ini_parser_.ParseConfig("Morai Vehicle driver", "cfg_cnt", cfg_.cnt);
        util_ini_parser_.ParseConfig("Morai Vehicle driver", "cfg_ay_lpf", cfg_.ay_lpf);
        util_ini_parser_.ParseConfig("Morai Vehicle driver", "cfg_vy_lpf", cfg_.vy_lpf);
        util_ini_parser_.ParseConfig("Morai Vehicle driver", "cfg_ay_noise", cfg_.ay_noise);
        util_ini_parser_.ParseConfig("Morai Vehicle driver", "cfg_vy_noise", cfg_.vy_noise);
    }
}

VehicleState VehicleDriver::GetMRVehicleStatus(const morai_msgs::EgoVehicleStatus& msg) {
    morai_msgs::EgoVehicleStatus i_ego_vehicle_status = msg;

    VehicleState vehicle_state;
    vehicle_state.header.stamp = ros_bridge::GetTimeStamp(i_ego_vehicle_status.header.stamp);
    // vehicle_state.quality = NovatelPosType::INS_RTKFIXED;
    
    vehicle_state.vx                    = i_ego_vehicle_status.velocity.x;
    vehicle_state.vy                    = i_ego_vehicle_status.velocity.y;
    vehicle_state.vz                    = i_ego_vehicle_status.velocity.z;
    
    vehicle_state.ax                    = i_ego_vehicle_status.acceleration.x;
    vehicle_state.ay                    = i_ego_vehicle_status.acceleration.y;
    vehicle_state.az                    = i_ego_vehicle_status.acceleration.z;

    vehicle_state.vehicle_can.longitudinal_accel    = i_ego_vehicle_status.acceleration.x;
    vehicle_state.vehicle_can.lateral_accel         = i_ego_vehicle_status.acceleration.y;
    
    vehicle_state.yaw                   = i_ego_vehicle_status.heading*DEG2RAD;

    vehicle_state.vehicle_can.steering_wheel_angle  = i_ego_vehicle_status.wheel_angle*STEERING_RATIO_MORAI;
    vehicle_state.vehicle_can.steering_tire_angle   = i_ego_vehicle_status.wheel_angle;
    
    vehicle_state.vehicle_can.steering_speed        = 0.0; // TODO
    vehicle_state.vehicle_can.steering_torque       = 0.0; // TODO
    vehicle_state.vehicle_can.steering_state        = MdpsState::MDPS_ACTIVATE;

    vehicle_state.vehicle_can.wheel_velocity_fl          = 0.0; // TODO
    vehicle_state.vehicle_can.wheel_velocity_fr          = 0.0; // TODO
    vehicle_state.vehicle_can.wheel_velocity_rl          = 0.0; // TODO
    vehicle_state.vehicle_can.wheel_velocity_rr          = 0.0; // TODO

    vehicle_state.vehicle_can.motor_torque_f            = 0.0; // TODO
    vehicle_state.vehicle_can.motor_torque_r            = 0.0; // TODO
    vehicle_state.vehicle_can.motor_torque_total        = 0.0; // TODO

    vehicle_state.vehicle_can.accel_position        = i_ego_vehicle_status.accel;
    vehicle_state.vehicle_can.brake_pressure        = i_ego_vehicle_status.brake;
    vehicle_state.vehicle_can.brake_active          = 0.0; // TODO
    vehicle_state.vehicle_can.gear_select           = Gear::GEAR_D; // TODO

    vehicle_state.vehicle_can.operation_mode        = OperationMode::AUTONOMOUS;
    vehicle_state.vehicle_can.lateral_autonomous_mode = AutonomousMode::RUN;
    vehicle_state.vehicle_can.longitudinal_autonomous_mode = AutonomousMode::RUN;
    return vehicle_state;
}
Gnss VehicleDriver::GetMRGPS(const morai_msgs::GPSMessage& msg) {
    morai_msgs::GPSMessage i_gnss = msg;
    
    Gnss gnss;
    gnss.header.stamp = ros_bridge::GetTimeStamp(i_gnss.header.stamp);
    gnss.latitude = i_gnss.latitude;
    gnss.longitude = i_gnss.longitude;
    gnss.altitude = i_gnss.altitude;
    // gnss.HDOP = i_gnss.hdop;

    return gnss;
}

Motion VehicleDriver::GetMRIMU(const sensor_msgs::Imu& msg) {
    sensor_msgs::Imu i_imu = msg;

    Motion motion;
    motion.header.stamp = ros_bridge::GetTimeStamp(i_imu.header.stamp);

    tf::Quaternion quat(
        i_imu.orientation.x, // x
        i_imu.orientation.y, // y
        i_imu.orientation.z, // z
        i_imu.orientation.w  // w
    );
    double roll,pitch,yaw;
    tf::Matrix3x3 quaternion_mat(quat);
    quaternion_mat.getRPY(roll,pitch,yaw);
    motion.roll = roll;
    motion.pitch = pitch;
    motion.yaw = yaw;

    motion.roll_vel = i_imu.angular_velocity.x;
    motion.pitch_vel = i_imu.angular_velocity.y;
    motion.yaw_vel = i_imu.angular_velocity.z;

    return motion;
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
                                       const Motion& motion, 
                                       const VehicleState& mr_vehicle_state){
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

    vehicle_state.x                     = position.x;
    vehicle_state.y                     = position.y;
    vehicle_state.z                     = position.z;

    vehicle_state.vx                    = mr_vehicle_state.vx;// + GenerateGaussianNoise(0.0, 0.01);
    vehicle_state.vy                    = mr_vehicle_state.vy;// + vy_lpf_noise; 
    vehicle_state.vz                    = mr_vehicle_state.vz;

    vehicle_state.ax                    = mr_vehicle_state.ax;
    vehicle_state.ay                    = mr_vehicle_state.ay;// + ay_lpf_noise;
    vehicle_state.az                    = mr_vehicle_state.az;

    vehicle_state.roll                  = motion.roll;
    vehicle_state.pitch                 = motion.pitch;
    vehicle_state.yaw                   = mr_vehicle_state.yaw;
    
    vehicle_state.roll_vel              = motion.roll_vel;
    vehicle_state.pitch_vel             = motion.pitch_vel;
    vehicle_state.yaw_vel               = motion.yaw_vel;// + GenerateGaussianNoise(0.0, 0.01);

    vehicle_state.vehicle_can.lateral_accel         = mr_vehicle_state.vehicle_can.lateral_accel;
    vehicle_state.vehicle_can.longitudinal_accel    = mr_vehicle_state.vehicle_can.longitudinal_accel;
    vehicle_state.vehicle_can.yaw_rate              = motion.yaw_vel;

    vehicle_state.vehicle_can.steering_wheel_angle  = mr_vehicle_state.vehicle_can.steering_wheel_angle;
    vehicle_state.vehicle_can.steering_tire_angle   = mr_vehicle_state.vehicle_can.steering_tire_angle;
    vehicle_state.vehicle_can.steering_speed        = mr_vehicle_state.vehicle_can.steering_speed;
    vehicle_state.vehicle_can.steering_torque       = mr_vehicle_state.vehicle_can.steering_torque;
    vehicle_state.vehicle_can.steering_state        = static_cast<MdpsState>(mr_vehicle_state.vehicle_can.steering_state);

    vehicle_state.vehicle_can.wheel_velocity_fl          = mr_vehicle_state.vehicle_can.wheel_velocity_fl;
    vehicle_state.vehicle_can.wheel_velocity_fr          = mr_vehicle_state.vehicle_can.wheel_velocity_fr;
    vehicle_state.vehicle_can.wheel_velocity_rl          = mr_vehicle_state.vehicle_can.wheel_velocity_rl;
    vehicle_state.vehicle_can.wheel_velocity_rr          = mr_vehicle_state.vehicle_can.wheel_velocity_rr;

    vehicle_state.vehicle_can.motor_torque_f            = mr_vehicle_state.vehicle_can.motor_torque_f;
    vehicle_state.vehicle_can.motor_torque_r            = mr_vehicle_state.vehicle_can.motor_torque_r;
    vehicle_state.vehicle_can.motor_torque_total        = mr_vehicle_state.vehicle_can.motor_torque_f + mr_vehicle_state.vehicle_can.motor_torque_r;

    vehicle_state.vehicle_can.accel_position        = mr_vehicle_state.vehicle_can.accel_position;
    vehicle_state.vehicle_can.brake_pressure        = mr_vehicle_state.vehicle_can.brake_pressure;
    vehicle_state.vehicle_can.brake_active          = mr_vehicle_state.vehicle_can.brake_active;
    vehicle_state.vehicle_can.gear_select           = static_cast<Gear>(mr_vehicle_state.vehicle_can.gear_select);

    vehicle_state.vehicle_can.operation_mode        = static_cast<OperationMode>(mr_vehicle_state.vehicle_can.operation_mode);
    vehicle_state.vehicle_can.lateral_autonomous_mode = static_cast<AutonomousMode>(mr_vehicle_state.vehicle_can.lateral_autonomous_mode);
    vehicle_state.vehicle_can.longitudinal_autonomous_mode = static_cast<AutonomousMode>(mr_vehicle_state.vehicle_can.longitudinal_autonomous_mode);
    
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