/**
 * @file        pcan_router_node.hpp
 * @brief       node cpp file for can router node & function of this code is P CAN Router
 * 
 * @authors     Seheon Ha (seheonha@hanyang.ac.kr)          
 * 
 * @date        2025-02-13 created by Seheon Ha
 * 
 */

#ifndef __PCAN_ROUTER_NODE_HPP__
#define __PCAN_ROUTER_NODE_HPP__
#pragma once


// get the current time
#define	SYSTIME_NOW		( TIM2->CNT)
// max for this MCU
#define	SYSTIME_MAX		0xFFFFFFFF
// calc a timediff
#define	SYSTIME_DIFF( first, second)		(((first)<=(second)) ? ((second)-(first)):((SYSTIME_MAX-(first))+(second)+1U))
// this is our basetype for time calculations
#define	SYSTIME_t	uint32_t

#define TIME_100_MS 1e5
#define TIME_10_MS 1e4
#define TIME_9_MS 9e3
#define TIME_11_MS 11e3


// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// ROS Header
#include <ros/ros.h>

// ROS Message Header
#include <autohyu_msgs/FrameFD.h>
#include <autohyu_msgs/VehicleCmd.h>
#include <autohyu_msgs/ADModeInput.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

// System Header
#include "task_manager.hpp"
#include "ini_parser.h"

// Interface Header
#include "ros_bridge_vehicle_state.hpp"
#include "interface_vehicle_state.hpp"
#include "interface_candb.hpp"
#include "interface_control.hpp"

// Parameter Header

// Algorithm Header
#include <autoku_can/autoku_can_process.hpp>


// Namespace
using namespace ros;
using namespace std;



class PCANRouter : public TaskManager {
    public:
        // Constructor
        explicit PCANRouter(std::string node_name, double period);
        // Destructor
        virtual ~PCANRouter();

    public:
        void Run();
        void Publish();
        void ProcessINI();
        void ProcessRosparam(const ros::NodeHandle& nh);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        
        // Callback functions for subscribe variables

        // Get Gway_data
        inline void CallbackCANRX(const autohyu_msgs::FrameFD& msg)
        {
            mutex_can_rx_.lock();
            switch (msg.id)
            {
                case CANID_GWAY1:
                    ptr_auto_ku_can_process_->CallbackGway1(&msg.data[0]);
                    break;
                // case CANID_AutoKUCMD:
                //     ptr_auto_ku_can_process_->CallbackAutoKuCmd(msg.data.data());
                //     break;
                case CANID_AutoKU_HELTH:
                    ptr_auto_ku_can_process_->CallbackAutoKuHelth(&msg.data[0]);
                    break;
                case CANID_AutoKU_ADMODE:
                    ptr_auto_ku_can_process_->CallbackAdMode(&msg.data[0]);
                    break;
                case CANID_AutoKU_SIREN_ON_OFF:
                    ptr_auto_ku_can_process_->CallbackSirenLedConfig(&msg.data[0]);
                    break;
                case CANID_E_FLAG:
                    ptr_auto_ku_can_process_->CallbackEFlag(&msg.data[0]);
                    break;
                default:
                    break;
            }
            mutex_can_rx_.unlock();
        };

        inline void CallbackCANTX(const autohyu_msgs::FrameFD& msg)
        {
            mutex_can_tx_.lock();
            ptr_auto_ku_can_process_->CallbackAutoKuCmd(&msg.data[0]);
            mutex_can_tx_.unlock();
        }

    
        // Update functions for subscribe variables
        void UpdateADCMD();
        void UpdateAutoKUSTA();

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        SYSTIME_t timer_10ms  = SYSTIME_NOW;
        SYSTIME_t timer_100ms = SYSTIME_NOW; 

        // Subscriber
        Subscriber s_can_rx_; // Get Gway CAN data from kvaser
        Subscriber s_can_tx_; // Get ADCMD CAN data from transmitter

        // Publisher
        Publisher p_can_tx_; // Publish ADCMD CAN data after adapt CAN DB

        // Inputs


        // Outputs
        autohyu_msgs::FrameFD o_can_ADCMD_tx_;  // ADCMD
        autohyu_msgs::FrameFD o_can_AutoKUSTA_tx_; // AutoKUSTA

        // Mutex
        mutex mutex_can_rx_;
        mutex mutex_can_tx_;

        // Algorithm
        unique_ptr<AutoKuCanProcess> ptr_auto_ku_can_process_;

        // Utils
        IniParser util_ini_parser_;

        // Configuration and Parameters
        bool b_is_valid_ = true;
        bool b_is_ADCMD_updated_ = false;
        bool b_is_AutoKUSTA_updated_ = false;
};

#endif // __PCAN_ROUTER_NODE_HPP__