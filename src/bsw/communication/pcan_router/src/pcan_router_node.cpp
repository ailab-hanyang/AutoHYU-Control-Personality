/**
 * @file        pcan_router_node.cpp
 * @brief       node cpp file for pcan router node
 * 
 * @authors     Seheon Ha (seheonha@hanyang.ac.kr)          
 * 
 * @date        2025-02-14 created by Seheon Ha
 * 
 */

#include "pcan_router_node.hpp"

PCANRouter::PCANRouter(std::string node_name, double period)
    : TaskManager(node_name, period) {
    // Initialize
    ROS_WARN_STREAM("[" << node_name << "] Initialize node (Period: " << 1/period << " Hz)");

    // Node init
    ros::NodeHandle nh;

    // Ini init
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/bsw.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());

    // Parameter init
    ProcessINI();
    ProcessRosparam(nh);
    
    // Subscriber init
    s_can_rx_ = nh.subscribe("can_rx0", 10, &PCANRouter::CallbackCANRX, this);
    s_can_tx_ = nh.subscribe("can_tx0", 10, &PCANRouter::CallbackCANTX, this);

    // Publisher init
    p_can_tx_ = nh.advertise<autohyu_msgs::FrameFD>("pcan_tx0", 10); // Change Kvaser Topic can_tx1

    // Set system time (ms)
    current_time_ = ros::Time::now().nsec / 1000000;
    timer_10ms_   = current_time_;
    timer_100ms_  = current_time_;

    // Algorithm init
    ptr_auto_ku_can_process_ = make_unique<AutoKuCanProcess>();
}

PCANRouter::~PCANRouter() {
    // Terminate
}

void PCANRouter::Run() {
    ProcessINI();
    b_is_valid_ = true;

    // Update timer
    current_time_ = ros::Time::now().nsec / 1000000;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if(GetTimeDifference(current_time_, timer_10ms_) > 10)
    {
        std::cout << "Update ADCMD ROS msg, 10ms time assumed: " << GetTimeDifference(current_time_, timer_10ms_) << std::endl;
        timer_10ms_ = current_time_ - (GetTimeDifference(current_time_, timer_10ms_) - 10);
        ptr_auto_ku_can_process_->Timer10msProcess(current_time_);
        UpdateADCMD();
        b_is_ADCMD_updated_ = true;
    }
    
    if(GetTimeDifference(current_time_, timer_100ms_) > 100)
    {
        std::cout << "Update STA ROS msg, 100ms time assumed: " << GetTimeDifference(current_time_, timer_100ms_) << std::endl;
        timer_100ms_ = current_time_ - (GetTimeDifference(current_time_, timer_100ms_) - 100);
        ptr_auto_ku_can_process_->Timer100msProcess();
        UpdateAutoKUSTA();
        b_is_AutoKUSTA_updated_ = true;
    }

}

void PCANRouter::Publish() {
    if(b_is_ADCMD_updated_ == true){
        p_can_tx_.publish(o_can_ADCMD_tx_);
        b_is_ADCMD_updated_ = false;
    }
    
    if(b_is_AutoKUSTA_updated_ == true){
        p_can_tx_.publish(o_can_AutoKUSTA_tx_);
        b_is_AutoKUSTA_updated_ = false;
    }
}



void PCANRouter::ProcessINI() {
    if (util_ini_parser_.IsFileUpdated() ) {

    }
}

void PCANRouter::ProcessRosparam(const ros::NodeHandle& nh) {
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
// Update functions for publish variables
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //



void PCANRouter::UpdateADCMD(){
    o_can_ADCMD_tx_.id        =  hmg_ioniq::CANID_ADCMD;
    o_can_ADCMD_tx_.dlc       =  hmg_ioniq::DLC_ADCMD;
    o_can_ADCMD_tx_.is_canfd  =  true;
    o_can_ADCMD_tx_.data.resize(hmg_ioniq::DLC_ADCMD);
    memcpy(&o_can_ADCMD_tx_.data[0], &ptr_auto_ku_can_process_->o_vehicle_can_adcmd_.data[0] , size_t(hmg_ioniq::DLC_ADCMD));
}

void PCANRouter::UpdateAutoKUSTA(){
    o_can_AutoKUSTA_tx_.id       = internal_can::CANID_INTERNAL_STA;
    o_can_AutoKUSTA_tx_.dlc      = internal_can::DLC_INTERNAL_STA;
    o_can_AutoKUSTA_tx_.is_canfd = true;
    o_can_AutoKUSTA_tx_.data.resize(internal_can::DLC_INTERNAL_STA);
    memcpy(&o_can_AutoKUSTA_tx_.data[0], &ptr_auto_ku_can_process_->o_autoku_can_sta_.data[0], size_t(internal_can::DLC_INTERNAL_STA));
}

// Get time difference
uint32_t PCANRouter::GetTimeDifference(const uint32_t& current_time, const uint32_t& prev_time){
    uint32_t time_difference;
    time_difference = current_time - prev_time;
    return time_difference;
}

int main(int argc, char** argv) {
    // Initialize node
    std::string node_name = "pcan_router";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    double period;
    if (!nh.getParam("task_period/period_pcan_router", period)) {
        period = 1.0;
    }
    
    PCANRouter main_task(node_name, period);
    main_task.Exec();

    return 0;
}