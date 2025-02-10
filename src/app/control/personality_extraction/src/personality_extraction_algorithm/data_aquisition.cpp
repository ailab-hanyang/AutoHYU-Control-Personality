/**
 * @file        data_aquisition.cpp
 * @brief       output : only the change of event count
 * 
 * @authors     sunghoon8585@gmail.com
 * 
 * @date        2024-10-25 created by Seounghoon Park
 *              2024-11-11 edit to make time window
 * 
 */

#include "personality_extraction_algorithm/data_aquisition.hpp"

DataAquisition::DataAquisition() {
    // Initialize
}

DataAquisition::~DataAquisition() {
    // Terminate
}



int DataAquisition::Run(const interface::VehicleState& vehicle_state, const double dt, const PersonalityExtractionConfig& cfg) {
    // Run Algorithm
    // generated in only initialization
    if (dt < cfg.loop_rate/2.0) {
        int get_data = 0;
        return get_data;
    }


    if (vel_buf_for_acc_.empty() == true) {
        // int size_of_2sec = 2*(1.0 / cfg.loop_rate);
        int buf_size = cfg.scene_size_in_sec / cfg.loop_rate;

        PersonalityScenePoint scene_point;
        scene_point.x = vehicle_state.x;
        scene_point.y = vehicle_state.y;
        scene_point.yaw = vehicle_state.yaw;
        scene_point.vel_mps = vehicle_state.vx;
        scene_point.ax = vehicle_state.vehicle_can.longitudinal_accel;
        scene_point.ay = vehicle_state.vehicle_can.lateral_accel;
        scene_point.jx = 0.0;
        scene_point.jy = 0.0;


        for (int i = 0; i < buf_size; i++) {
            vel_buf_for_acc_.push_back(vehicle_state.vx);
            vel_buf_for_dcc_.push_back(vehicle_state.vx);
            
            curvature_buf_.push_back(0.0);

            o_lon_acc_buf_.push_back(vehicle_state.vehicle_can.longitudinal_accel);
            o_lat_acc_buf_.push_back(vehicle_state.vehicle_can.lateral_accel);
            o_lon_jerk_buf_.push_back(0.0);
            o_lat_jerk_buf_.push_back(0.0);

            o_scene_points_.push_back(scene_point);
        }

        int mv_avg_window_size = 1.0 / (2 * cut_off_freq_ * cfg.loop_rate);
        for (int i = 0; i < mv_avg_window_size; i++) {
            lon_mov_avg_window_.push_back(vehicle_state.vehicle_can.longitudinal_accel);
            lat_mov_avg_window_.push_back(vehicle_state.vehicle_can.lateral_accel);
        }

        int get_data = 1;
        return get_data;

    }

    else {
        
        FindAccEvent(vehicle_state, dt, cfg);
        FindDccEvent(vehicle_state, dt, cfg);
        FindSteerEvent(vehicle_state, dt, cfg);


        // >>>>>>>>> SAVING DATA IS INDEPENDENT WHETHER cfg.get_data is true or not <<<<<<<<<<<<!!!
        
        // setting output of filtered acceleration data & jerk
        lon_mov_avg_window_.insert(lon_mov_avg_window_.begin(), vehicle_state.vehicle_can.longitudinal_accel);
        lon_mov_avg_window_.pop_back();
        o_lon_acc_buf_.insert(o_lon_acc_buf_.begin(), MvAvgOut(lon_mov_avg_window_));
        o_lon_acc_buf_.pop_back();
        
        
        lat_mov_avg_window_.insert(lat_mov_avg_window_.begin(), vehicle_state.vehicle_can.lateral_accel); 
        lat_mov_avg_window_.pop_back();
        o_lat_acc_buf_.insert(o_lat_acc_buf_.begin(), MvAvgOut(lat_mov_avg_window_));
        o_lat_acc_buf_.pop_back();
        
        // setting output of jerk data from filtered accel
        if (o_lon_acc_buf_.size() >= 2) {
            double lon_jerk = (o_lon_acc_buf_[0] - o_lon_acc_buf_[1]) / dt;
            double lat_jerk = (o_lat_acc_buf_[0] - o_lat_acc_buf_[1]) / dt;
            if (o_lon_acc_buf_[0] == o_lon_acc_buf_[1]) {
                lon_jerk = 0.0;
            }
            if (o_lat_acc_buf_[0] == o_lat_acc_buf_[1]) {
                lat_jerk = 0.0;
            }
            // std::cout << "lat jerk : " << lat_jerk << std::endl;
            o_lon_jerk_buf_.insert(o_lon_jerk_buf_.begin(), lon_jerk);
            o_lon_jerk_buf_.pop_back();
            o_lat_jerk_buf_.insert(o_lat_jerk_buf_.begin(), lat_jerk);
            o_lat_jerk_buf_.pop_back();
        }        

        // setting trajectory of vehicle movement
        PersonalityScenePoint scene_point;
        scene_point.x = vehicle_state.x;
        scene_point.y = vehicle_state.y;
        scene_point.yaw = vehicle_state.yaw;
        scene_point.vel_mps = vehicle_state.vx;
        scene_point.ax = vehicle_state.vehicle_can.longitudinal_accel;
        scene_point.ay = vehicle_state.vehicle_can.lateral_accel;
        scene_point.jx = o_lon_jerk_buf_[0];
        scene_point.jy = o_lat_jerk_buf_[0];
        scene_point.delta = vehicle_state.vehicle_can.steering_tire_angle;

        o_scene_points_.insert(o_scene_points_.begin(), scene_point);
        o_scene_points_.pop_back();


        // for defining end of data aquisition
        // setting to data get or not
        // not used now
        int get_data = 0;
        if (acc_event_count_ >= event_count_threshold_ && 
            dcc_event_count_ >= event_count_threshold_ &&  
            steer_event_count_ >= event_count_threshold_) {
            get_data = 0;
            return get_data;
        }
        else  {
            get_data = 1;
            return get_data;
        }
    }
}






void DataAquisition::FindAccEvent(const interface::VehicleState& vehicle_state, const double dt, const PersonalityExtractionConfig& cfg) {
    vel_buf_for_acc_.insert(vel_buf_for_acc_.begin(), vehicle_state.vx);
    vel_buf_for_acc_.pop_back();

    int buf_quarter = vel_buf_for_acc_.size() / 4;
    int buf_3quarter = 3 * buf_quarter;

    bool is_acc_now = false;
    if (b_is_accel_before_ == false) {
        if (vel_buf_for_acc_[buf_quarter] - vel_buf_for_acc_[buf_3quarter] >= ref_acc_vel_mps_) {
            is_acc_now = true;
            
            acc_event_count_++;

            b_is_accel_before_ = is_acc_now;
        }
    }
    else if (b_is_accel_before_ == true) {
        if (vel_buf_for_acc_[buf_quarter] - vel_buf_for_acc_[buf_3quarter] >= ref_acc_vel_mps_) {
            is_acc_now = true;
        }
        b_is_accel_before_ = is_acc_now;
    }
    
}

void DataAquisition::FindDccEvent(const interface::VehicleState& vehicle_state, const double dt, const PersonalityExtractionConfig& cfg) {
    vel_buf_for_dcc_.insert(vel_buf_for_dcc_.begin(), vehicle_state.vx);
    vel_buf_for_dcc_.pop_back();

    int buf_quarter = vel_buf_for_dcc_.size() / 4;
    int buf_3quarter = 3 * buf_quarter;

    bool is_dcc_now = false;
    if (b_is_decel_before_ == false) {
        if (vel_buf_for_dcc_[buf_quarter] - vel_buf_for_dcc_[buf_3quarter] <= ref_dcc_vel_mps_) {
            is_dcc_now = true;
            dcc_event_count_++;

            b_is_decel_before_ = is_dcc_now;
        }
    }
    else if (b_is_decel_before_ == true) {
        if (vel_buf_for_dcc_[buf_quarter] - vel_buf_for_dcc_[buf_3quarter] <= ref_dcc_vel_mps_) {
            is_dcc_now = true;
        }
        b_is_decel_before_ = is_dcc_now;
    }

}

void DataAquisition::FindSteerEvent(const interface::VehicleState& vehicle_state, const double dt, const PersonalityExtractionConfig& cfg) {
    double L = interface::VEHICLE_FRONT_AXLE_TO_CG + interface::VEHICLE_REAR_AXLE_TO_CG;
    double kappa = tan(vehicle_state.vehicle_can.steering_tire_angle) / L;
    curvature_buf_.insert(curvature_buf_.begin(), kappa);
    curvature_buf_.pop_back();

    bool is_curve_now = false;

    int buf_quarter = curvature_buf_.size() / 4;
    int buf_3quarter = 3 * buf_quarter;

    if (b_is_curve_before_ == false) {
        // checking is curve now
        is_curve_now = true;
        for (int i = buf_quarter; i < buf_3quarter; i++) {
            if (std::abs(curvature_buf_[i]) < ref_curvature_) {
                is_curve_now = false;
            }
        }

        if (is_curve_now == true) {
            steer_event_count_++;
            b_is_curve_before_ = is_curve_now;
        }
    }
    else if (b_is_curve_before_ == true) {
        // checking is curve now
        is_curve_now = true;
        for (int i = buf_quarter; i < buf_3quarter; i++) {
            if (std::abs(curvature_buf_[i]) < ref_curvature_) {
                is_curve_now = false;
            }
        }
        b_is_curve_before_ = is_curve_now;
    }
}





double DataAquisition::MvAvgOut(const std::vector<double>& data) {
    double sum = 0.0;
    for (int i = 0; i < data.size(); i++) {
        sum += data[i];
    }
    return sum / data.size();
}
double DataAquisition::StdDevOut(const std::vector<double>& data) {
    double mean = MvAvgOut(data);
    double sum = 0.0;
    for (int i = 0; i < data.size(); i++) {
        sum += (data[i] - mean) * (data[i] - mean);
    }
    return sqrt(sum / data.size());
}

std::tuple<int, int, int> DataAquisition::EventCountOut() {
    std::tuple<int, int, int> out(acc_event_count_, dcc_event_count_, steer_event_count_);
    return out;
}
std::vector<double> DataAquisition::LonAccBufferOut() {
    return o_lon_acc_buf_;
}
std::vector<double> DataAquisition::LonPlusAccBufferOut() {
    std::vector<double> lon_plus_acc_buf;
    for (int i = 0; i < o_lon_acc_buf_.size(); i++) {
        if (o_lon_acc_buf_[i] > 0.0) {
            lon_plus_acc_buf.push_back(o_lon_acc_buf_[i]);
        }
    }
    return lon_plus_acc_buf;
}
std::vector<double> DataAquisition::LonMinusAccBufferOut() {
    std::vector<double> lon_minus_acc_buf;
    for (int i = 0; i < o_lon_acc_buf_.size(); i++) {
        if (o_lon_acc_buf_[i] < 0.0) {
            lon_minus_acc_buf.push_back(o_lon_acc_buf_[i]);
        }
    }
    return lon_minus_acc_buf;
}
std::vector<double> DataAquisition::LatAccBufferOut() {
    return o_lat_acc_buf_;
}
std::vector<double> DataAquisition::LatAbsAccBufferOut() {
    std::vector<double> lat_abs_acc_buf;
    for (int i = 0; i < o_lat_acc_buf_.size(); i++) {
        lat_abs_acc_buf.push_back(std::abs(o_lat_acc_buf_[i]));
    }
    return lat_abs_acc_buf;
}
std::vector<double> DataAquisition::LonJerkBufferOut() {
    return o_lon_jerk_buf_;
}
std::vector<double> DataAquisition::LonPlusJerkBufferOut() {
    std::vector<double> lon_plus_jerk_buf;
    for (int i = 0; i < o_lon_jerk_buf_.size(); i++) {
        if (o_lon_jerk_buf_[i] > 0.0) {
            lon_plus_jerk_buf.push_back(o_lon_jerk_buf_[i]);
        }
    }
    return lon_plus_jerk_buf;
}
std::vector<double> DataAquisition::LonMinusJerkBufferOut() {
    std::vector<double> lon_minus_jerk_buf;
    for (int i = 0; i < o_lon_jerk_buf_.size(); i++) {
        if (o_lon_jerk_buf_[i] < 0.0) {
            lon_minus_jerk_buf.push_back(o_lon_jerk_buf_[i]);
        }
    }
    return lon_minus_jerk_buf;
}
std::vector<double> DataAquisition::LatJerkBufferOut() {
    return o_lat_jerk_buf_;
}
std::vector<double> DataAquisition::LatAbsJerkBufferOut() {
    std::vector<double> lat_abs_jerk_buf;
    for (int i = 0; i < o_lat_jerk_buf_.size(); i++) {
        lat_abs_jerk_buf.push_back(std::abs(o_lat_jerk_buf_[i]));
    }
    return lat_abs_jerk_buf;
}
std::vector<PersonalityScenePoint> DataAquisition::SceneTrajectoryOut() {
    return o_scene_points_;
}
void DataAquisition::LonAccJerkBufferClear() {
    o_lon_acc_buf_.clear();
    o_lon_jerk_buf_.clear();
}
void DataAquisition::LatAccJerkBufferClear() {
    o_lat_acc_buf_.clear();
    o_lat_jerk_buf_.clear();
}