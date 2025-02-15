#ifndef _AUTOKU_CAN_PROCESS_HPP_
#define _AUTOKU_CAN_PROCESS_HPP_

#include "interface_candb.hpp"

const float STOP_CONDITION_VEL_KPH = 10.0f;
const uint32_t PARKING_TIME_CONDITION_US = 5e6; // 5 sec
const uint32_t RE_DRIVE_TIME_CONDITION_US = 1e6; // 1 sec
const uint8_t SHIFT_COMMAND_SKIP_COUNT = 17; // Some non ilregular number.
const float SHIFT_ENABLE_BRAKE_PRESURE = 30.0; // Consider with SHIFTING_BRAKE_PRESURE in autoku_can_msg.hpp


/////////////////////////////////////////////////////////////////
// CAN Factor and Scale
// GWAY1
const float GWAY1_SAS_ANGLE_SCALE = -0.1;
const float GWAY1_SAS_SPEED_SCALE = 4.0;
const float GWAY1_CLUSTER_ODOMETER_SCALE = 0.1;
const float GWAY1_LATERAL_ACCEL_SPEED_SCALE = 0.000127465;
const float GWAY1_LATERAL_ACCEL_SPEED_OFFSET = -4.17677312;
const float GWAY1_LONGITUDINAL_ACCEL_SPEED_SCALE = 0.000127465;
const float GWAY1_LONGITUDINAL_ACCEL_SPEED_OFFSET = -4.17677312;
const float GWAY1_YAW_RATE_SENSOR_SCALE = 0.005;
const float GWAY1_YAW_RATE_SENSOR_OFFSET = -163.84;
const float GWAY1_WHEEL_VELOCITY_SCALE = 0.03125;
const float GWAY1_STEERING_ANGLE_SCALE = -0.1;
const float GWAY1_STEERING_TQ_SCALE = 0.005;
const float GWAY1_STEERING_TQ_OFFSET = -20.48;
const float GWAY1_ACCEL_PEDAL_POSITION_SCALE = 0.3921568627;
const float GWAY1_MCU_TORQUE_SCALE = 0.125;

// ADCMD
const float ADCMD_BRK1_SCALE = 0.390625;
const float ADCMD_BRK2V_SCALE = 0.390625;
const float ADCMD_ACC_APS_SCALE = 0.2985;
const float ADCMD_ACC_APS_OFFSET = -22.3875;
const float ADCMD_APS2_SCALE = 0.33479853;
const float ADCMD_APS2_OFFSET = 308;
const float ADCMD_STR_ANGLE_REQ_SCALE = -0.1;

// AutoKUCMD
const float AutoKUCMD_SPEED_SCALE = 0.03125;
const float AutoKUCMD_BRAKE_PEDAL_POS_SCALE = 0.392157;
const float AutoKUCMD_ACC_PEDAL_POS_SCALE = 0.392157;
const float AutoKUCMD_ACCELERATION_SCALE = 0.01;
const float AutoKUCMD_STEERING_ANGLE_SCALE = -0.1; // Why "minus" 0.1  ....

/////////////////////////////////////////////////////////////////
// CAN ID
const unsigned int CANID_ADCMD = 0x50;
const unsigned int CANID_GWAY1 = 0x100;
const unsigned int CANID_E_FLAG = 0x200;
const unsigned int CANID_AutoKUCMD = 0x303;
const unsigned int CANID_AutoKUSTA = 0x304;
const unsigned int CANID_AutoKU_HELTH = 0x305;
const unsigned int CANID_AutoKU_ADMODE = 0x306;
const unsigned int CANID_AutoKU_SIREN_ON_OFF = 0x307;

/////////////////////////////////////////////////////////////////
// CAN DataLength and DataLengthCode
const unsigned int DL_ADCMD = 32;
const unsigned int DLC_ADCMD = 0xD;
const unsigned int DL_GWAY1 = 32;
const unsigned int DLC_GWAY1 = 0xD;
const unsigned int DL_E_FLAG = 8;
const unsigned int DLC_E_FLAG = 0x8;
const unsigned int DL_AutoKUCMD = 12;
const unsigned int DLC_AutoKUCMD = 0x9;
const unsigned int DL_AutoKUSTA = 3;
const unsigned int DLC_AutoKUSTA = 0x3;
const unsigned int DL_AutoKU_HEALTH = 1;
const unsigned int DLC_AutoKU_HEALTH = 0x1;
const unsigned int DL_AutoKU_ADMODE = 4;
const unsigned int DLC_AutoKU_ADMODE = 0x4;
const unsigned int DL_AutoKU_SIREN_ON_OFF = 2;
const unsigned int DLC_AutoKU_SIREN_ON_OFF = 0x2;

/////////////////////////////////////////////////////////////////
// Control constants
const float INITIAL_BRAKE_POSITION = 20.0;
const float SHIFTING_BRAKE_PRESURE = 40.0; // Bar
const uint32_t AutoKUCMD_TIMEOUT_US = 5e5; // 500 ms

/////////////////////////////////////////////////////////////////
// Decoded Gway messages Define
typedef struct
{
    float Gway_SAS_Angle;                   // Deg
    float Gway_SAS_Speed;                   // Deg/s
    float Gway_Cluster_Odometer;            // km
    float Gway_Lateral_Accel_Speed;         // g
    float Gway_BrakeMasterCylinder_Pressur; // Bar
    uint8_t Gway_Steering_Status;           //  0 "Reserved" 1 "Steering still in initialization phase" 2 "Steering ready, waits for PA command" 3 "Steering set in standby by PA" 4 "Steering requested to go to first activation step" 5 "Steering requested to go to final activation step" 6 "Steering went to error internally" 7 "Steering aborted the automatic function" 14 "Not Used" 15 "Error Indicator"
    float Gway_Longitudinal_Accel_Speed;    // g
    float Gway_Yaw_Rate_Sensor;             // deg/s
    float Gway_Wheel_Velocity_FR;           // km/h
    uint8_t Gway_Brake_Active;              // 0 "Brake pedal not pressed" 1 "Brake pedal pressed" 2 "Not used" 3 "Error Indicator" ;
    float Gway_Wheel_Velocity_RL;           // km/h
    float Gway_Wheel_Velocity_RR;           // km/h
    float Gway_Wheel_Velocity_FL;           // km/h
    float Gway_Steering_Angle;              // Deg
    float Gway_Steering_Tq;                 // Nm
    float Gway_Accel_Pedal_Position;        // %
    uint8_t Gway_GearSelDisp;               // 0 "P" 1 "B" 2 "Not Used" 3 "Not Used" 4 "Not Used" 5 "D" 6 "N" 7 "R" 8 "Not Used" 9 "Not Display at Cluster" 10 "Reserved" 11 "Reserved" 12 "Manual Shift Mode on Retromode" 13 "Reserved" 14 "Intermediate Position" 15 "Fault" ;
    float F_MCU_Torque;                     // Nm
    float R_MCU_Torque;                     // Nm
} External_Decoded_GWAY1;
/////////////////////////////////////////////////////////////////


class AutoKuCanProcess
{
    /////////////////////////////////////////////////////////////////
    // Constructor
public:
    AutoKuCanProcess();
    ~AutoKuCanProcess();

    /////////////////////////////////////////////////////////////////
    // CAN Factor and Scale

    /////////////////////////////////////////////////////////////////
    // Input Variables
    // Autonomous Mode
    bool i_b_mode_updated_;
    INTERNAL_CAN_AD_MODE i_autoku_ad_mode_;

    // Autonomous Command
    uint8_t i_ui8_health_seq_number_;
    INTERNAL_CAN_CMD i_autoku_can_cmd_;

    // Vehicle gateway
    External_CAN_GWAY1 i_vehicle_can_gway1_;
    External_Decoded_GWAY1 i_vehicle_gway1_;

    // Siren and LED configuration
    INTERNAL_CAN_SIREN_ON_OFF i_autoku_siren_on_off_;

    // Three Sec E Flag
    THREE_SECS_E_FLAG i_three_secs_e_flag_can_;

    /////////////////////////////////////////////////////////////////
    // Output Variables
    // Autonomous Mode
    INTERNAL_CAN_STA o_autoku_can_sta_;

    // Autonomous Command

    // Vehicle gateway
    External_CAN_ADCMD o_vehicle_can_adcmd_;

    // Siren and LED configuration

    /////////////////////////////////////////////////////////////////
    // Parameter (configuration) Variables
    // Autonomous Mode
    bool param_b_ad_ready_;
    bool param_b_lateral_enable_;
    bool param_b_longitudinal_enable_;
    bool param_b_ad_run_user_command_;

    // Autonomous Command
    bool param_b_vehicle_command_health_is_good_;

    // Vehicle gateway
    uint8_t param_ui8_input_steering_status_; // 1: Init, 2: NewStart, 3: Standby, 4: Active, 5: AngleControlActive, 6:Failure

    // Siren and LED configuration
    uint8_t param_b_siren_on_;      // 0: off, 1: on, 2: blink, 7: e_flag_mode
    uint8_t param_b_led_blue_on_;   // 0: off, 1: on, 2: blink, 7: e_flag_mode
    uint8_t param_b_led_orange_on_; // 0: off, 1: on, 2: blink, 7: e_flag_mode

    /////////////////////////////////////////////////////////////////
    // Variables
    // Autonomous Mode
    uint8_t ui8_prev_seq_number_;

    // Autonomous Command
    uint32_t ui32_time_last_cmd_seq_updated_us_;
    uint32_t ui32_time_diff_from_last_cmd_seq_updated_us_;

    // Vehicle gateway

    // Siren and LED configuration

    /////////////////////////////////////////////////////////////////
    // Fuctions

    // Autonomous Mode
    void AutonomousModeUpdate();
    void CheckDriverOveride();
    void AdCmdHelthSeqChecker(uint32_t time_now_us);
    // INTERNAL_CAN_STA GetAutoKuCanSta();
    void UpdateAutoKuCanSta();

    // Autonomous Command
    void SteeringModeStatusUpdate();
    void AutoKuCmdUpdate();
    void AutoKuLatCmdUpdate(External_CAN_ADCMD input_command);
    void AutoKuLonCmdUpdate(External_CAN_ADCMD input_command);
    bool AutoKuCmdValidation();

    void GearShiftCmdUpdate(uint32_t time_now_us);

    void AutonomousCloseCmdUpdate();

    // Add 10ms worker
    void Timer10msProcess(uint32_t time_now_us);
    // Add 100ms worker
    void Timer100msProcess();


    // Vehicle gateway

    /////////////////////////////////////////////////////////////////
    // Callback Fuctions

    // Autonomous Mode
    void CallbackAdMode(uint8_t *data);

    // Autonomous Command
    void CallbackAutoKuHelth(uint8_t *autoku_can_health_data);
    void CallbackAutoKuCmd(uint8_t *autoku_can_cmd_data);

    // Vehicle gateway
    void CallbackGway1(uint8_t *gway1_data);
    
    // Siren and LED configuration
    void CallbackSirenLedConfig(uint8_t *siren_speaker_data);

    // Three Sec E Flag
    void CallbackEFlag(uint8_t *e_flag_data);

    /////////////////////////////////////////////////////////////////
    // Tools

    /////////////////////////////////////////////////////////////////
    // CAN message function
    External_Decoded_GWAY1 DecodeDataCanGway1(const External_CAN_GWAY1 &can);
    void InjectCanAutoKUCmdToCanAdcmd(const INTERNAL_CAN_CMD &autoku_cmd, External_CAN_ADCMD &adcmd);
};

#endif // _AUTOKU_CAN_PROCESS_HPP_
