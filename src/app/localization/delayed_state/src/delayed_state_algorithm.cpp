
#include <delayed_state_algorithm.hpp>

DelayedStateAlgorithm::DelayedStateAlgorithm() {}

DelayedStateAlgorithm::~DelayedStateAlgorithm() {}

bool DelayedStateAlgorithm::Init(double dt) {
    d_dt_sec_ = dt;
    return true;
}

// make noise (x, y, yaw, vx, vy, yawrate)
VehicleState DelayedStateAlgorithm::MakeNoisyDelayedState(
        const std::vector<VehicleState>& vehicle_state_vec,
        const DelayedStateParams& params) {

    // first initializing dq
    if (dq_state_xy_window_.empty() == true) {
        int q_size = std::floor(params.cfg_delay_time_s / d_dt_sec_);

        for (int i = 0; i < q_size; i++) {
            // xy point;
            // point.x = vehicle_state.x;
            // point.y = vehicle_state.y;
            // dq_state_xy_window_.push_front(point);
            VehicleState blank_state;
            dq_state_xy_window_.push_front(blank_state);
            // dq_state_xy_window_.push_front(vehicle_state_vec[0]);
        }
    }


    // if no vehicle state received in a loop
    if (vehicle_state_vec.empty() == true) {
        VehicleState output;
        output = dq_state_xy_window_.front();
        
        // make noise
        output.x = GenerateGaussianNoise(output.x, params.cfg_noise_std_dev_x);
        output.y = GenerateGaussianNoise(output.y, params.cfg_noise_std_dev_y);
        output.yaw = GenerateGaussianNoise(output.yaw, params.cfg_noise_std_dev_yaw);
        output.vx = GenerateGaussianNoise(output.vx, params.cfg_noise_std_dev_vx);
        output.vy = GenerateGaussianNoise(output.vy, params.cfg_noise_std_dev_vy);
        output.yaw_vel = GenerateGaussianNoise(output.yaw_vel, params.cfg_noise_std_dev_yaw_vel);
        dq_state_xy_window_.pop_front();

        return output;
    }




    // check delay changed
    // add or erase deque datas
    int changed_q_size;
    if (params.cfg_delay_time_s <= 0.0) {     // if no delay!
        changed_q_size = dq_state_xy_window_.size();
        dq_state_xy_window_.clear();
        for (int i = 0; i < changed_q_size; i++) {
            // xy point;
            // point.x = vehicle_state.x;
            // point.y = vehicle_state.y;
            // dq_state_xy_window_.push_front(point);
            dq_state_xy_window_.push_front(vehicle_state_vec[0]);
        }
    }
    else {
        changed_q_size = std::floor(params.cfg_delay_time_s / d_dt_sec_);
    }

    if (changed_q_size > dq_state_xy_window_.size()) {
        int add_idx = changed_q_size - dq_state_xy_window_.size();
        for (int i = 0; i < add_idx; i++) {
            // xy point;
            // point.x = prev_vehicle_state_.x;
            // point.y = prev_vehicle_state_.y;
            // dq_state_xy_window_.push_back(point);
            dq_state_xy_window_.push_back(prev_vehicle_state_);
        }
    }
    else if (changed_q_size < dq_state_xy_window_.size()) {
        int erase_idx = dq_state_xy_window_.size() - changed_q_size;
        for (int i = 0; i < erase_idx; i++) {
            
            dq_state_xy_window_.pop_front();
        }
    }

    // save prev data
    prev_vehicle_state_ = vehicle_state_vec[0];


    // update data 
    for(int i = 0; i < vehicle_state_vec.size(); i++) {
        dq_state_xy_window_.push_back(vehicle_state_vec[i]);
    }
    // std::cout << vehicle_state_vec.size() << std::endl;
    

    VehicleState output;
    output = dq_state_xy_window_.front();
    
    // make noise
    output.x = GenerateGaussianNoise(output.x, params.cfg_noise_std_dev_x);
    output.y = GenerateGaussianNoise(output.y, params.cfg_noise_std_dev_y);
    output.yaw = GenerateGaussianNoise(output.yaw, params.cfg_noise_std_dev_yaw);
    output.vx = GenerateGaussianNoise(output.vx, params.cfg_noise_std_dev_vx);
    output.vy = GenerateGaussianNoise(output.vy, params.cfg_noise_std_dev_vy);
    output.yaw_vel = GenerateGaussianNoise(output.yaw_vel, params.cfg_noise_std_dev_yaw_vel);

    dq_state_xy_window_.pop_front();

    // std::cout << dq_state_xy_window_.size() << std::endl;

    return output;
}



double DelayedStateAlgorithm::GenerateGaussianNoise(double mean, double std_dev) {
    // This function is for generating gaussian noise
    static std::random_device        rd;
    std::default_random_engine       gen(rd());
    std::normal_distribution<double> dist(mean, std_dev);

    return dist(gen);
}