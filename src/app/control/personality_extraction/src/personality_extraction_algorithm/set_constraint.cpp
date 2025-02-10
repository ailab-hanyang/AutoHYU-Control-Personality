/**
 * 
 */

#include "personality_extraction_algorithm/set_constraint.hpp"

SetConstraint::SetConstraint() {
    // Initialize

    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/personality.ini");
    util_ini_exporter_.Init((dir + ini_dir).c_str());

}

SetConstraint::~SetConstraint() {
    // Terminate
}



void SetConstraint::Run(const interface::VehicleState& vehicle_state, const double dt, const PersonalityExtractionConfig& cfg) {
    // Run Algorithm
    if (cfg.get_data == 1) {
        FindAccJerkMax(vehicle_state, dt, cfg);
        if (b_is_config_export_ == true)
            std::cout << "\n\n[personality_extraction_node] Start extracting acc & jerk\n\n" << std::endl;
        b_is_config_export_ = false;
    }
    else if (cfg.get_data == 0) {
        if (b_is_config_export_ == false) {
            
            ProcessINI();
            b_is_config_export_ = true;
        }
        else {
            prev_longitudinal_accel_ = vehicle_state.vehicle_can.longitudinal_accel;
            prev_lateral_accel_ = vehicle_state.vehicle_can.lateral_accel;
            lateral_accel_buf_.clear();
            lateral_jerk_buf_.clear();
            longitudinal_minus_accel_buf_.clear();
            longitudinal_plus_accel_buf_.clear();
            longitudinal_minus_jerk_buf_.clear();
            longitudinal_plus_jerk_buf_.clear();

            max_longitudinal_accel_ = 0.0;
            max_longitudinal_jerk_ = 0.0;
            max_lateral_accel_ = 0.0;
            max_lateral_jerk_ = 0.0;
        }
    }
    else {
        std::cout << "[personality_extraction_node] wrong cfg value!!" << std::endl;
    }
}



// Sorting & Removing Outliar by IQR
void SetConstraint::FindAccJerkMax(const interface::VehicleState& vehicle_state, const double dt, const PersonalityExtractionConfig& cfg) {
    
    if (dt < cfg.loop_rate/2.0) {
        return;
    }
    
    double longitudinal_jerk = (vehicle_state.vehicle_can.longitudinal_accel - prev_longitudinal_accel_) / dt;
    double lateral_jerk      = (vehicle_state.vehicle_can.lateral_accel - prev_lateral_accel_) / dt;

    // put longitudinal datas
    if (vehicle_state.vehicle_can.longitudinal_accel > 0.1)
        longitudinal_plus_accel_buf_.push_back(vehicle_state.vehicle_can.longitudinal_accel);
    else if (vehicle_state.vehicle_can.longitudinal_accel < -0.1)
        longitudinal_minus_accel_buf_.push_back(vehicle_state.vehicle_can.longitudinal_accel);

    if (std::abs(longitudinal_jerk) <= 20.0 && std::abs(longitudinal_jerk) >= 0.1) {
        if (longitudinal_jerk > 0.0)
            longitudinal_plus_jerk_buf_.push_back(longitudinal_jerk);
        else 
            longitudinal_minus_jerk_buf_.push_back(longitudinal_jerk);
    }

    // put lateral datas
    if (std::abs(vehicle_state.vehicle_can.longitudinal_accel) >= 0.1)
        lateral_accel_buf_.push_back(std::abs(vehicle_state.vehicle_can.longitudinal_accel));
    if (std::abs(lateral_jerk) >= 0.1 && std::abs(lateral_jerk) <= 20.0) 
        lateral_jerk_buf_.push_back(std::abs(lateral_jerk));


    // TODO : check sorting in loop is efficient or not
    std::sort(lateral_accel_buf_.begin(), lateral_accel_buf_.end());
    std::sort(lateral_jerk_buf_.begin(), lateral_jerk_buf_.end());
    std::sort(longitudinal_plus_accel_buf_.begin(), longitudinal_plus_accel_buf_.end());
    std::sort(longitudinal_minus_accel_buf_.begin(), longitudinal_minus_accel_buf_.end());
    std::sort(longitudinal_plus_jerk_buf_.begin(), longitudinal_plus_jerk_buf_.end());
    std::sort(longitudinal_minus_jerk_buf_.begin(), longitudinal_minus_jerk_buf_.end());
    
    bool is_iqr = false;
    max_lateral_accel_ = FindFilteredMinMax(lateral_accel_buf_, is_iqr).at(1);
    min_longitudinal_accel_ = FindFilteredMinMax(longitudinal_minus_accel_buf_, is_iqr).at(0);
    max_longitudinal_accel_ = FindFilteredMinMax(longitudinal_plus_accel_buf_ , is_iqr).at(1);
    
    is_iqr = true;
    min_longitudinal_jerk_ = FindFilteredMinMax(longitudinal_minus_jerk_buf_, is_iqr).at(0);
    max_longitudinal_jerk_ = FindFilteredMinMax(longitudinal_plus_jerk_buf_, is_iqr).at(1);
    max_lateral_jerk_ = FindFilteredMinMax(lateral_jerk_buf_, is_iqr).at(1);

    // std::cout << "dt : " << dt << std::endl;
    // std::cout << "max ax : " << max_longitudinal_accel_ << std::endl;
    // std::cout << "max jx : " << max_longitudinal_jerk_ << std::endl;
    // std::cout << "max ay : " << max_lateral_accel_ << std::endl;
    // std::cout << "max jy : " << max_lateral_jerk_ << std::endl;
    // std::cout << std::endl;
    
    prev_longitudinal_accel_ = vehicle_state.vehicle_can.longitudinal_accel;
    prev_lateral_accel_ = vehicle_state.vehicle_can.lateral_accel;
}



std::vector<double> SetConstraint::FindFilteredMinMax(std::vector<double>& datas, bool is_iqr, bool is_sorting) {
    if (datas.empty() == true) {
        std::vector<double> ret = {0.0, 0.0};
        return ret;    
    }

    if (is_sorting == true) {
        std::sort(datas.begin(), datas.end());
    }
    
    double lower_bound = 0.0;
    double upper_bound = 0.0;

    if (is_iqr == true) {
        int idx_25 = datas.size() * (0.25);
        int idx_75 = datas.size() * (0.75);
        double IQR = datas[idx_75] - datas[idx_25];

        lower_bound = datas[idx_25] - 1.5*IQR;
        upper_bound = datas[idx_75] + 1.5*IQR;
    }
    else if (is_iqr == false) {
        int idx_5 = datas.size() * (0.05);
        int idx_95 = datas.size() * (0.95);

        lower_bound = datas[idx_5];
        upper_bound = datas[idx_95];
    }
    
    std::vector<double> minmax;
    minmax.push_back(lower_bound);
    minmax.push_back(upper_bound);

    return minmax;
}

void SetConstraint::RunByGettingBuffer(const std::vector<double>& lon_acc_buf,
                                        const std::vector<double>& lat_acc_buf,
                                        const std::vector<double>& lon_jerk_buf,
                                        const std::vector<double>& lat_jerk_buf,
                                        const PersonalityExtractionConfig& cfg) {
    // Run Algorithm
    if (cfg.get_data == 1) {
        FindAccJerkMax(lon_acc_buf, lat_acc_buf, lon_jerk_buf, lat_jerk_buf, cfg);
        if (b_is_config_export_ == true)
            std::cout << "\n\n[personality_extraction_node] Start extracting acc & jerk\n\n" << std::endl;
        b_is_config_export_ = false;
    }
    else if (cfg.get_data == 0) {
        if (b_is_config_export_ == false) {
            
            ProcessINI();
            b_is_config_export_ = true;
        }
        else {
            lateral_accel_buf_.clear();
            lateral_jerk_buf_.clear();
            longitudinal_minus_accel_buf_.clear();
            longitudinal_plus_accel_buf_.clear();
            longitudinal_minus_jerk_buf_.clear();
            longitudinal_plus_jerk_buf_.clear();

            max_longitudinal_accel_ = 0.0;
            max_longitudinal_jerk_ = 0.0;
            max_lateral_accel_ = 0.0;
            max_lateral_jerk_ = 0.0;
        }
    }
    else {
        std::cout << "[personality_extraction_node] wrong cfg value!!" << std::endl;
    }
}


void SetConstraint::FindAccJerkMax(const std::vector<double>& lon_acc_buf,
                                    const std::vector<double>& lat_acc_buf,
                                    const std::vector<double>& lon_jerk_buf,
                                    const std::vector<double>& lat_jerk_buf,
                                    const PersonalityExtractionConfig& cfg) {
    
    // put longitudinal datas
    for (int i = 0; i < lon_acc_buf.size(); i++) {
        if (lon_acc_buf[i] > 0.0)
            longitudinal_plus_accel_buf_.push_back(lon_acc_buf[i]);
        else if (lon_acc_buf[i] < 0.0)
            longitudinal_minus_accel_buf_.push_back(lon_acc_buf[i]);
    }
    for (int i = 0; i < lon_jerk_buf.size(); i++) {
        if (lon_jerk_buf[i] > 0.0)
            longitudinal_plus_jerk_buf_.push_back(lon_jerk_buf[i]);
        else 
            longitudinal_minus_jerk_buf_.push_back(lon_jerk_buf[i]);
    }
    

    // put lateral datas
    for (int i = 0; i < lat_acc_buf.size(); i++) {
        lateral_accel_buf_.push_back(std::abs(lat_acc_buf[i]));
    }
    for (int i = 0; i < lat_jerk_buf.size(); i++) {
        lateral_jerk_buf_.push_back(std::abs(lat_jerk_buf[i]));
    }
    

    // TODO : check sorting in loop is efficient or not
    std::sort(lateral_accel_buf_.begin(), lateral_accel_buf_.end());
    std::sort(lateral_jerk_buf_.begin(), lateral_jerk_buf_.end());
    std::sort(longitudinal_plus_accel_buf_.begin(), longitudinal_plus_accel_buf_.end());
    std::sort(longitudinal_minus_accel_buf_.begin(), longitudinal_minus_accel_buf_.end());
    std::sort(longitudinal_plus_jerk_buf_.begin(), longitudinal_plus_jerk_buf_.end());
    std::sort(longitudinal_minus_jerk_buf_.begin(), longitudinal_minus_jerk_buf_.end());
    
    bool is_iqr = false;
    max_lateral_accel_ = FindFilteredMinMax(lateral_accel_buf_, is_iqr).at(1);
    min_longitudinal_accel_ = FindFilteredMinMax(longitudinal_minus_accel_buf_, is_iqr).at(0);
    max_longitudinal_accel_ = FindFilteredMinMax(longitudinal_plus_accel_buf_ , is_iqr).at(1);
    min_longitudinal_jerk_ = FindFilteredMinMax(longitudinal_minus_jerk_buf_, is_iqr).at(0);
    max_longitudinal_jerk_ = FindFilteredMinMax(longitudinal_plus_jerk_buf_, is_iqr).at(1);
    max_lateral_jerk_ = FindFilteredMinMax(lateral_jerk_buf_, is_iqr).at(1);

    // std::cout << "dt : " << dt << std::endl;
    // std::cout << "max ax : " << max_longitudinal_accel_ << std::endl;
    // std::cout << "max jx : " << max_longitudinal_jerk_ << std::endl;
    // std::cout << "max ay : " << max_lateral_accel_ << std::endl;
    // std::cout << "max jy : " << max_lateral_jerk_ << std::endl;
    // std::cout << std::endl;
    
}


void SetConstraint::ProcessINI() {
    if (!util_ini_exporter_.ExportConfig("PersonalityExtraction", "min_ax", min_longitudinal_accel_)  ||
        !util_ini_exporter_.ExportConfig("PersonalityExtraction", "max_ax", max_longitudinal_accel_)  ||
        !util_ini_exporter_.ExportConfig("PersonalityExtraction", "min_jx", min_longitudinal_jerk_)   ||
        !util_ini_exporter_.ExportConfig("PersonalityExtraction", "max_jx", max_longitudinal_jerk_)   ||
        !util_ini_exporter_.ExportConfig("PersonalityExtraction", "max_ay", max_lateral_accel_)       ||
        !util_ini_exporter_.ExportConfig("PersonalityExtraction", "max_jy", max_lateral_jerk_)      ) {

        std::cout << "[personality_extraction_node] cannot change file!" << std::endl;
    }
    else {
        std::cout << "\n\n[personality_extraction_node] changed ax & jerk to personality.ini\n\n" << std::endl;
    }
}