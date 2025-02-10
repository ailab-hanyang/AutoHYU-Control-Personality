/**
 * @file        personality_time_window_node.cpp
 * @brief       get trajectory of each scene & get total ax ay jx jy's avg & stddev
 * 
 * @authors     sunghoon8585@gmail.com
 * 
 * @date        2024-11-11 created by Seounghoon Park
 * 
 */

#include "personality_time_window_node.hpp"

namespace plt = matplotlibcpp;



PersonalityTimeWindowNode::PersonalityTimeWindowNode(std::string node_name, double period)
    : TaskManager(node_name, period) {
    // Initialize
    ROS_WARN_STREAM("[" << node_name << "] Initialize node (Period: " << 1/period << " Hz)");

    // Node init
    ros::NodeHandle nh;

    // Ini init
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/personality.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());
    util_ini_exporter_.Init((dir + ini_dir).c_str());

    // Parameter init
    ProcessRosparam(nh);
    ProcessINI();
    
    // Subscriber init
    s_vehicle_state_ = nh.subscribe(
        "app/loc/vehicle_state", 10, &PersonalityTimeWindowNode::CallbackVehicleState, this);
        
    p_personality_time_window_ = nh.advertise<personality_msgs::PersonalityTimeWindow>(
        "/personality/time_window", 10);

    // Algorithm init
    alg_data_aquisition_ = std::make_unique<DataAquisition>();

    // Matplotlib init
    plt::ion();
}

PersonalityTimeWindowNode::~PersonalityTimeWindowNode() {
    // Terminate
}

void PersonalityTimeWindowNode::Run() {
    ProcessINI();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if (!b_is_vehicle_state_) {
        ROS_ERROR_THROTTLE(1.0,"Wait for Vehicle State...");
        return;
    }

    interface::VehicleState vehicle_state; {
        mutex_vehicle_state_.lock();
        vehicle_state = i_vehicle_state_;
        mutex_vehicle_state_.unlock();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    double dt = vehicle_state.header.stamp - before_time_stamp_s_;


    // overriding get_data value
    if (cfg_.get_data == true) {
        
        alg_data_aquisition_->Run(vehicle_state, dt, cfg_);
        
        std::tuple<int, int, int> event_count = alg_data_aquisition_->EventCountOut();
        int acc_count = std::get<0>(event_count);
        int dcc_count = std::get<1>(event_count);
        int steer_count = std::get<2>(event_count);

        // visualization
        loop_count_++;
        if (loop_count_ % 20 == 0) {    // 0.05 * 20 = 1[s] of visualization
            std::cout <<
                "event acc / dcc / steer : " << 
                acc_count << " " << dcc_count << " " << steer_count << " " <<
            std::endl;
        }


        
        // saving time window
        b_get_scene_ = false;
        PersonalityScene scene;

        // put scene in vector(works like queue)
        // steering event highest priority!
        if (steer_count > before_steer_event_) {    
            scene.scene_type = interface::CORNER_SCENE;
            scene.scene_points = alg_data_aquisition_->SceneTrajectoryOut();
            corner_scene_list_.insert(corner_scene_list_.begin(), scene);
            b_get_scene_ = true;
        }
        else if (acc_count > before_acc_event_) {
            scene.scene_type = interface::ACCEL_SCENE;
            scene.scene_points = alg_data_aquisition_->SceneTrajectoryOut();
            accel_scene_list_.insert(accel_scene_list_.begin(), scene);
            b_get_scene_ = true;
        }
        else if (dcc_count > before_dcc_event_) {
            scene.scene_type = interface::DECEL_SCENE;
            scene.scene_points = alg_data_aquisition_->SceneTrajectoryOut();
            decel_scene_list_.insert(decel_scene_list_.begin(), scene);
            b_get_scene_ = true;
        }
        else {
            b_get_scene_ = false;
        }

        if (accel_scene_list_.size() > cfg_.max_scene_queue_size)
            accel_scene_list_.pop_back();
        if (decel_scene_list_.size() > cfg_.max_scene_queue_size)
            decel_scene_list_.pop_back();
        if (corner_scene_list_.size() > cfg_.max_scene_queue_size)
            corner_scene_list_.pop_back();

        
        // calculate mean & stddev & minmax
        if (b_get_scene_ == true) {
            
            
            SetMinMaxAvgStddev();
            std::cout <<
                "ax mean   : " << ax_mean_ << "\n" <<
                "ax stddev : " << ax_stddev_ << "\n" <<
                "ay mean   : " << ay_mean_ << "\n" <<
                "ay stddev : " << ay_stddev_ << "\n" <<
                "jx mean   : " << jx_mean_ << "\n" <<
                "jx stddev : " << jx_stddev_ << "\n" <<
                "jy mean   : " << jy_mean_ << "\n" <<
                "jy stddev : " << jy_stddev_ << "\n" <<
            std::endl;

        }

        before_acc_event_ = acc_count;
        before_dcc_event_ = dcc_count;
        before_steer_event_ = steer_count;
    }

    before_time_stamp_s_ = vehicle_state.header.stamp;   
    
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    UpdateWindow();

}

void PersonalityTimeWindowNode::SetMinMaxAvgStddev() {
    o_plus_lon_acc_buf_.clear();
    o_minus_lon_acc_buf_.clear();
    o_lat_acc_buf_.clear();
    o_plus_lon_jerk_buf_.clear();
    o_minus_lon_jerk_buf_.clear();
    o_lat_jerk_buf_.clear();
    
    for (int i = 0; i < accel_scene_list_.size(); i++) {
        for (int j = 0; j < accel_scene_list_[i].scene_points.size(); j++) {
            double ax = accel_scene_list_[i].scene_points[j].ax;
            if (ax > 0.0)       o_plus_lon_acc_buf_.push_back(ax);
            else                o_minus_lon_acc_buf_.push_back(ax);

            o_lat_acc_buf_.push_back(std::abs(accel_scene_list_[i].scene_points[j].ay));

            double jx = accel_scene_list_[i].scene_points[j].jx;
            if (jx > 0.0)       o_plus_lon_jerk_buf_.push_back(jx);
            else                o_minus_lon_jerk_buf_.push_back(jx);

            o_lat_jerk_buf_.push_back(std::abs(accel_scene_list_[i].scene_points[j].jy));
        }
    }

    for (int i = 0; i < decel_scene_list_.size(); i++) {
        for (int j = 0; j < decel_scene_list_[i].scene_points.size(); j++) {
            double ax = decel_scene_list_[i].scene_points[j].ax;
            if (ax > 0.0)       o_plus_lon_acc_buf_.push_back(ax);
            else                o_minus_lon_acc_buf_.push_back(ax);

            o_lat_acc_buf_.push_back(std::abs(decel_scene_list_[i].scene_points[j].ay));

            double jx = decel_scene_list_[i].scene_points[j].jx;
            if (jx > 0.0)       o_plus_lon_jerk_buf_.push_back(jx);
            else                o_minus_lon_jerk_buf_.push_back(jx);

            o_lat_jerk_buf_.push_back(std::abs(decel_scene_list_[i].scene_points[j].jy));
        }
    }

    for (int i = 0; i < corner_scene_list_.size(); i++) {
        for (int j = 0; j < corner_scene_list_[i].scene_points.size(); j++) {
            double ax = corner_scene_list_[i].scene_points[j].ax;
            if (ax > 0.0)       o_plus_lon_acc_buf_.push_back(ax);
            else                o_minus_lon_acc_buf_.push_back(ax);

            o_lat_acc_buf_.push_back(std::abs(corner_scene_list_[i].scene_points[j].ay));

            double jx = corner_scene_list_[i].scene_points[j].jx;
            if (jx > 0.0)       o_plus_lon_jerk_buf_.push_back(jx);
            else                o_minus_lon_jerk_buf_.push_back(jx);

            o_lat_jerk_buf_.push_back(std::abs(corner_scene_list_[i].scene_points[j].jy));
        }
    }


    ax_min_ = FindFilteredMinMax(o_minus_lon_acc_buf_, false, true)[0];
    ax_max_ = FindFilteredMinMax(o_plus_lon_acc_buf_, false, true)[1];
    ay_max_ = FindFilteredMinMax(o_lat_acc_buf_, false, true)[1];
    jx_min_ = FindFilteredMinMax(o_minus_lon_jerk_buf_, false, true)[0];
    jx_max_ = FindFilteredMinMax(o_plus_lon_jerk_buf_, false, true)[1];
    jy_max_ = FindFilteredMinMax(o_lat_jerk_buf_, false, true)[1];


    std::vector<double> ax_buf, ay_buf, jx_buf, jy_buf;
    ax_buf.insert(ax_buf.end(), o_plus_lon_acc_buf_.begin(), o_plus_lon_acc_buf_.end());
    ax_buf.insert(ax_buf.end(), o_minus_lon_acc_buf_.begin(), o_minus_lon_acc_buf_.end());

    jx_buf.insert(jx_buf.end(), o_plus_lon_jerk_buf_.begin(), o_plus_lon_jerk_buf_.end());
    jx_buf.insert(jx_buf.end(), o_minus_lon_jerk_buf_.begin(), o_minus_lon_jerk_buf_.end());


    ax_mean_ = alg_data_aquisition_->MvAvgOut(ax_buf);
    ax_stddev_ = alg_data_aquisition_->StdDevOut(ax_buf);
    ax_plus_mean_ = alg_data_aquisition_->MvAvgOut(o_plus_lon_acc_buf_);
    ax_plus_stddev_ = alg_data_aquisition_->StdDevOut(o_plus_lon_acc_buf_);
    ax_minus_mean_ = alg_data_aquisition_->MvAvgOut(o_minus_lon_acc_buf_);
    ax_minus_stddev_ = alg_data_aquisition_->StdDevOut(o_minus_lon_acc_buf_);
    ay_mean_ = alg_data_aquisition_->MvAvgOut(o_lat_acc_buf_);
    ay_stddev_ = alg_data_aquisition_->StdDevOut(o_lat_acc_buf_);

    jx_mean_ = alg_data_aquisition_->MvAvgOut(jx_buf);
    jx_stddev_ = alg_data_aquisition_->StdDevOut(jx_buf);
    jx_plus_mean_ = alg_data_aquisition_->MvAvgOut(o_plus_lon_jerk_buf_);
    jx_plus_stddev_ = alg_data_aquisition_->StdDevOut(o_plus_lon_jerk_buf_);
    jx_minus_mean_ = alg_data_aquisition_->MvAvgOut(o_minus_lon_jerk_buf_);
    jx_minus_stddev_ = alg_data_aquisition_->StdDevOut(o_minus_lon_jerk_buf_);
    jy_mean_ = alg_data_aquisition_->MvAvgOut(o_lat_jerk_buf_);
    jy_stddev_ = alg_data_aquisition_->StdDevOut(o_lat_jerk_buf_);

}


/*
 * is_iqr == true : IQR method
 * is_iqr == false : 5% 95% method
 * 
 * is_sorting == true : sorting before find minmax
 * is_sorting == false : no sorting
 */
std::vector<double> PersonalityTimeWindowNode::FindFilteredMinMax(std::vector<double>& datas, bool is_iqr, bool is_sorting) {
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

void PersonalityTimeWindowNode::UpdateWindow() {
    o_personality_time_window_.ax.min = ax_min_;
    o_personality_time_window_.ax.max = ax_max_;
    o_personality_time_window_.ax.avg = ax_mean_;
    o_personality_time_window_.ax.std_dev = ax_stddev_;

    o_personality_time_window_.ax_plus.min = -9999.99;
    o_personality_time_window_.ax_plus.max = 9999.99;
    o_personality_time_window_.ax_plus.avg = ax_plus_mean_;
    o_personality_time_window_.ax_plus.std_dev = ax_plus_stddev_;

    o_personality_time_window_.ax_minus.min = -9999.99;
    o_personality_time_window_.ax_minus.max = 9999.99;
    o_personality_time_window_.ax_minus.avg = ax_minus_mean_;
    o_personality_time_window_.ax_minus.std_dev = ax_minus_stddev_;

    o_personality_time_window_.ay.max = ay_max_;
    o_personality_time_window_.ay.avg = ay_mean_;
    o_personality_time_window_.ay.std_dev = ay_stddev_;

    o_personality_time_window_.jx.min = jx_min_;
    o_personality_time_window_.jx.max = jx_max_;
    o_personality_time_window_.jx.avg = jx_mean_;
    o_personality_time_window_.jx.std_dev = jx_stddev_;

    o_personality_time_window_.jx_plus.min = -9999.99;
    o_personality_time_window_.jx_plus.max = 9999.99;
    o_personality_time_window_.jx_plus.avg = jx_plus_mean_;
    o_personality_time_window_.jx_plus.std_dev = jx_plus_stddev_;

    o_personality_time_window_.jx_minus.min = -9999.99;
    o_personality_time_window_.jx_minus.max = 9999.99;
    o_personality_time_window_.jx_minus.avg = jx_minus_mean_;
    o_personality_time_window_.jx_minus.std_dev = jx_minus_stddev_;

    o_personality_time_window_.jy.max = jy_max_;
    o_personality_time_window_.jy.avg = jy_mean_;
    o_personality_time_window_.jy.std_dev = jy_stddev_;

    o_personality_time_window_.accel_scenes = accel_scene_list_;
    o_personality_time_window_.decel_scenes = decel_scene_list_;
    o_personality_time_window_.corner_scenes = corner_scene_list_;
}


void PersonalityTimeWindowNode::Publish() {
    if (!b_is_vehicle_state_ ||
        !b_get_scene_) {
        return;
    }

    personality_msgs::PersonalityTimeWindow msg =
        ros_bridge::UpdateTimeWindow(o_personality_time_window_);
    p_personality_time_window_.publish(msg);
}

// multithreaded function
void PersonalityTimeWindowNode::PlotScene() {
	
    while(true) {
        
        // Clear previous plot
        plt::clf();

        if (accel_scene_list_.size() == 0 && 
            decel_scene_list_.size() == 0 &&
            corner_scene_list_.size() == 0) {
            plt::title("Scenes");
            plt::subplot(2, 5, 1);
            plt::pause(0.1);
            continue;
        }


        for (int i = 0; i < 5; i++) {
            std::vector<double> x, y, v;
            if (i < accel_scene_list_.size()) {
                
                plt::subplot(3, 5, i+1);
                
                for (int j = 0; j < accel_scene_list_[i].scene_points.size(); j++) {
                    x.push_back(accel_scene_list_[i].scene_points[j].x);
                    y.push_back(accel_scene_list_[i].scene_points[j].y);
                    v.push_back(accel_scene_list_[i].scene_points[j].vel_mps);
                }
                plt::title("ACCEL");

                // plt::plot(x, y);
                plt::scatter_colored(x, y, v, 40, {{"cmap", "rainbow"}});
                
                std::vector<double> start_x, start_y;
                start_x.push_back(x[0]); 
                start_y.push_back(y[0]);
                // plt::plot(start_x, start_y, "k*");
                plt::scatter(start_x, start_y, 50, {{"color", "black"}});
                
            }
        }

        for (int i = 0; i < 5; i++) {
            std::vector<double> x, y, v;
            if (i < decel_scene_list_.size()) {
                
                plt::subplot(3, 5, i+6);
                
                for (int j = 0; j < decel_scene_list_[i].scene_points.size(); j++) {
                    x.push_back(decel_scene_list_[i].scene_points[j].x);
                    y.push_back(decel_scene_list_[i].scene_points[j].y);
                    v.push_back(decel_scene_list_[i].scene_points[j].vel_mps);
                }
                plt::title("DECEL");

                // plt::plot(x, y);
                plt::scatter_colored(x, y, v, 40, {{"cmap", "rainbow"}});
                
                std::vector<double> start_x, start_y;
                start_x.push_back(x[0]); 
                start_y.push_back(y[0]);
                // plt::plot(start_x, start_y, "k*");
                plt::scatter(start_x, start_y, 50, {{"color", "black"}});
                
            }
        }

        for (int i = 0; i < 5; i++) {
            std::vector<double> x, y, v;
            if (i < corner_scene_list_.size()) {
                
                plt::subplot(3, 5, i+11);
                
                for (int j = 0; j < corner_scene_list_[i].scene_points.size(); j++) {
                    x.push_back(corner_scene_list_[i].scene_points[j].x);
                    y.push_back(corner_scene_list_[i].scene_points[j].y);
                    v.push_back(corner_scene_list_[i].scene_points[j].vel_mps);
                }
                plt::title("CORNER");

                // plt::plot(x, y);
                plt::scatter_colored(x, y, v, 40, {{"cmap", "rainbow"}});
                
                std::vector<double> start_x, start_y;
                start_x.push_back(x[0]); 
                start_y.push_back(y[0]);
                // plt::plot(start_x, start_y, "k*");
                plt::scatter(start_x, start_y, 50, {{"color", "black"}});
                
            }
        }

        // plt::colorbar();
        plt::pause(0.1);
    }
    
}

void PersonalityTimeWindowNode::ProcessRosparam(const ros::NodeHandle& nh) {
    nh.getParam("/personality_time_window/user_name", cfg_.user_name);
    nh.getParam("task_period/period_personality_extraction", cfg_.loop_rate);
    ROS_WARN_STREAM("\nperiod : " << cfg_.loop_rate); 
}

void PersonalityTimeWindowNode::ProcessINI() {
    util_ini_parser_.ParseConfig("PersonalityTimeWindow", "get_data",               cfg_.get_data);
    util_ini_parser_.ParseConfig("PersonalityTimeWindow", "max_scene_queue_size",   cfg_.max_scene_queue_size);
    util_ini_parser_.ParseConfig("PersonalityTimeWindow", "scene_size_in_sec",      cfg_.scene_size_in_sec);

    if (b_get_scene_ == true) {
        // export min & max
        // util_ini_exporter_.ExportConfig("PersonalityExtraction", "ax_min", ax_min_);
        util_ini_exporter_.ExportConfig("PersonalityExtraction", "ax_max", ax_max_);
        util_ini_exporter_.ExportConfig("PersonalityExtraction", "ay_max", ay_max_);
        util_ini_exporter_.ExportConfig("PersonalityExtraction", "jx_min", jx_min_);
        util_ini_exporter_.ExportConfig("PersonalityExtraction", "jx_max", jx_max_);
        util_ini_exporter_.ExportConfig("PersonalityExtraction", "jy_max", jy_max_);

        // export avg & stddev
        util_ini_exporter_.ExportConfig("PersonalityTimeWindow", "ax_mean", ax_mean_);
        util_ini_exporter_.ExportConfig("PersonalityTimeWindow", "ax_stddev", ax_stddev_);
        util_ini_exporter_.ExportConfig("PersonalityTimeWindow", "ay_mean", ay_mean_);
        util_ini_exporter_.ExportConfig("PersonalityTimeWindow", "ay_stddev", ay_stddev_);
        util_ini_exporter_.ExportConfig("PersonalityTimeWindow", "jx_mean", jx_mean_);
        util_ini_exporter_.ExportConfig("PersonalityTimeWindow", "jx_stddev", jx_stddev_);
        util_ini_exporter_.ExportConfig("PersonalityTimeWindow", "jy_mean", jy_mean_);
        util_ini_exporter_.ExportConfig("PersonalityTimeWindow", "jy_stddev", jy_stddev_);

        // Save user result
        util_ini_exporter_.ExportConfig(cfg_.user_name, "ax_max", ax_max_);
        util_ini_exporter_.ExportConfig(cfg_.user_name, "ay_max", ay_max_);
        util_ini_exporter_.ExportConfig(cfg_.user_name, "jx_min", jx_min_);
        util_ini_exporter_.ExportConfig(cfg_.user_name, "jx_max", jx_max_);
        util_ini_exporter_.ExportConfig(cfg_.user_name, "jy_max", jy_max_);

        // export avg & stddev
        util_ini_exporter_.ExportConfig(cfg_.user_name, "ax_mean", ax_mean_);
        util_ini_exporter_.ExportConfig(cfg_.user_name, "ax_stddev", ax_stddev_);
        util_ini_exporter_.ExportConfig(cfg_.user_name, "ay_mean", ay_mean_);
        util_ini_exporter_.ExportConfig(cfg_.user_name, "ay_stddev", ay_stddev_);
        util_ini_exporter_.ExportConfig(cfg_.user_name, "jx_mean", jx_mean_);
        util_ini_exporter_.ExportConfig(cfg_.user_name, "jx_stddev", jx_stddev_);
        util_ini_exporter_.ExportConfig(cfg_.user_name, "jy_mean", jy_mean_);
        util_ini_exporter_.ExportConfig(cfg_.user_name, "jy_stddev", jy_stddev_);
    }

}

int main(int argc, char** argv) {
    std::string node_name = "personality_extraction_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_personality_extraction", period)) {
        period = 1.0;
    }

    PersonalityTimeWindowNode main_task(node_name, period);
    boost::thread main_thread(boost::bind(&TaskManager::MainLoop, &main_task));

    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    boost::thread plot_thread(boost::bind(&PersonalityTimeWindowNode::PlotScene, &main_task));

    ros::waitForShutdown();

    main_thread.join();
    plot_thread.join();

    return 0;
}