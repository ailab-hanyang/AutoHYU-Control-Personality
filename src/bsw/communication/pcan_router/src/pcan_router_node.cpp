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
    p_can_tx_ = nh.advertise<autohyu_msgs::FrameFD>("can_tx1", 10); // Change Kvaser Topic can_tx1

    // Algorithm init
    ptr_auto_ku_can_process_ = make_unique<AutoKuCanProcess>();
}

PCANRouter::~PCANRouter() {
    // Terminate
}

void PCANRouter::Run() {
    ProcessINI();
    b_is_valid_ = true;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    if(SYSTIME_DIFF(timer_10ms, SYSTIME_NOW) > TIME_11_MS)
    {
        timer_10ms = SYSTIME_NOW - (SYSTIME_DIFF(timer_10ms, SYSTIME_NOW) - TIME_11_MS);
        ptr_auto_ku_can_process_->Timer10msProcess(SYSTIME_NOW);
        UpdateADCMD();
        b_is_ADCMD_updated_ = true;
    }
    
    if(SYSTIME_DIFF(timer_100ms, SYSTIME_NOW) > TIME_100_MS)
    {
        timer_100ms = SYSTIME_NOW - (SYSTIME_DIFF(timer_100ms, SYSTIME_NOW) - TIME_100_MS);
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