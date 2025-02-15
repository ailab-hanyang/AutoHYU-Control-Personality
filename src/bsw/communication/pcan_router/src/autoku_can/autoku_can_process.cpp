#include "autoku_can_process.hpp"

/////////////////////////////////////////////////////////////////
// Constructor
AutoKuCanProcess::AutoKuCanProcess() : i_b_mode_updated_(false),
                                       i_autoku_ad_mode_(),
                                       i_ui8_health_seq_number_(0),
                                       i_autoku_can_cmd_(),
                                       i_vehicle_can_gway1_(),
                                       i_vehicle_gway1_(),
                                       i_autoku_siren_on_off_(),
                                       i_three_secs_e_flag_can_(),
                                       o_autoku_can_sta_(),
                                       o_vehicle_can_adcmd_(),
                                       param_b_ad_ready_(false),
                                       param_b_lateral_enable_(false),
                                       param_b_longitudinal_enable_(false),
                                       param_b_ad_run_user_command_(false),
                                       param_b_vehicle_command_health_is_good_(false),
                                       param_ui8_input_steering_status_(0),
                                       param_b_siren_on_(7),
                                       param_b_led_blue_on_(7),
                                       param_b_led_orange_on_(7),
                                       ui8_prev_seq_number_(0),
                                       ui32_time_last_cmd_seq_updated_us_(0),
                                       ui32_time_diff_from_last_cmd_seq_updated_us_(0)
{
    memset(&i_autoku_can_cmd_, 0, sizeof(INTERNAL_CAN_CMD));
    memset(&i_vehicle_can_gway1_, 0, sizeof(External_CAN_GWAY1));
    memset(&i_vehicle_gway1_, 0, sizeof(External_Decoded_GWAY1));
    memset(&o_autoku_can_sta_, 0, sizeof(INTERNAL_CAN_STA));
}
AutoKuCanProcess::~AutoKuCanProcess() {}

/////////////////////////////////////////////////////////////////
// Fuctions
// Autonomous Mode

/**
 * @brief Autonomous drive mode update from PC
 * @param param_b_ad_mode_: Can enable when vehicle stop and brake pressed
 * @param param_b_lateral_enable_: Can enable when vehicle stop and brake not pressed
 * @param param_b_longitudinal_enable_: Can enable when vehicle stop and brake not pressed
 */
void AutoKuCanProcess::AutonomousModeUpdate()
{
    if (i_b_mode_updated_ == true) // User command udpate
    {
        // Update detected
        i_b_mode_updated_ = false;

        // Manual mode disable all ad modes
        if (i_autoku_ad_mode_.str.manual_mode == 1)
        {
            param_b_ad_ready_ = false;
            param_b_lateral_enable_ = false;
            param_b_longitudinal_enable_ = false;
            param_b_ad_run_user_command_ = false;
            return;
        }

        // Autonomous operation ready
        if (i_autoku_ad_mode_.str.operation_mode == 1)
        {
            // Autonomous drive ready mode enable when vehicle stoped (<FLT_MIN) and brake bedal activated (1)
            if (param_b_ad_ready_ == false &&
                i_vehicle_gway1_.Gway_Wheel_Velocity_FL < STOP_CONDITION_VEL_KPH)
            {
                param_b_ad_ready_ = true;
            }
        }

        if (param_b_ad_ready_ == false)
        {
            param_b_lateral_enable_ = false;
            param_b_longitudinal_enable_ = false;
            return;
        }

        // Lateral control mode
        if (i_autoku_ad_mode_.str.lateral_mode == 1)
        {
            // Lateral control enable when velocity is stop and brake not brake press.
            if (i_vehicle_gway1_.Gway_Wheel_Velocity_FL < STOP_CONDITION_VEL_KPH)
            {
                param_b_lateral_enable_ = true;
            }
        }

        // Lonagitudinal control mode
        if (i_autoku_ad_mode_.str.longitudinal_mode == 1)
        {
            // Lonagitudinal control enable when velocity is stop and brake not brake press.
            if (i_vehicle_gway1_.Gway_Wheel_Velocity_FL < STOP_CONDITION_VEL_KPH)
            {
                param_b_longitudinal_enable_ = true;
            }
        }

        if (param_b_lateral_enable_ == true && param_b_longitudinal_enable_ == true)
        {
            param_b_ad_run_user_command_ = true;
        }
    }

    if (param_b_ad_run_user_command_ == true)
    {
        if (i_three_secs_e_flag_can_.str.e_flag == 1 || i_three_secs_e_flag_can_.str.e_flag == 2 || i_three_secs_e_flag_can_.str.e_flag == 3)
        {
            param_b_ad_ready_ = true;
            param_b_lateral_enable_ = true;
            param_b_longitudinal_enable_ = true;
        }
    }

    return;
}
/**
 * @brief User overide check function. Brake activated then autonomous false
 *
 */
void AutoKuCanProcess::CheckDriverOveride()
{
    // Autonomous mode user overide
    if (param_b_lateral_enable_ == true || param_b_longitudinal_enable_ == true)
    {
        if (i_vehicle_gway1_.Gway_Brake_Active == 1)
        {
            param_b_longitudinal_enable_ = false;
            // Non competition mode
            if ((i_three_secs_e_flag_can_.str.e_flag == 1 ||
                 (i_three_secs_e_flag_can_.str.e_flag == 2 ||
                  i_three_secs_e_flag_can_.str.e_flag == 3)) == false)
            {
                param_b_ad_run_user_command_ = false; // ad_run_user_command is not diable when competition mode
                param_b_lateral_enable_ = false;      // lateral control not diable when competition
            }
        }
    }

    // Fail mode user overide
    if (param_b_vehicle_command_health_is_good_ == false && param_b_ad_ready_ == true)
    {
        // Emergency case, brake mean off the system
        if (i_vehicle_gway1_.Gway_Brake_Active == 1)
        {
            param_b_ad_ready_ = false;
            param_b_lateral_enable_ = false;
            param_b_longitudinal_enable_ = false;
            // Non competition mode
            if ((i_three_secs_e_flag_can_.str.e_flag == 1 ||
                 (i_three_secs_e_flag_can_.str.e_flag == 2 ||
                  i_three_secs_e_flag_can_.str.e_flag == 3)) == false)
            {
                param_b_ad_run_user_command_ = false; // ad_run_user_command is not diable when competition mode
            }
        }
    }
    return;
}

/**
 * @brief Check input seqence is changed and determine input command validate with timeout
 *
 * @param time_now_us: last sequance change detected time (current time)
 * @param parma_b_vehicle_command_health_is_gool_: false when command is not refresh (timeout, 500ms)
 */
void AutoKuCanProcess::AdCmdHelthSeqChecker(uint32_t time_now_us)
{
    /////////////////////////////////////////////////////////////////
    // Check input sequance is change
    if (ui8_prev_seq_number_ != i_ui8_health_seq_number_)
    {
        // Refresh updated time to now
        ui32_time_last_cmd_seq_updated_us_ = time_now_us;
        ui8_prev_seq_number_ = i_ui8_health_seq_number_;
    }

    /////////////////////////////////////////////////////////////////
    // Determine command disconnect with timeout
    if (ui32_time_last_cmd_seq_updated_us_ == time_now_us)
    {
        param_b_vehicle_command_health_is_good_ = true;
        return;
    }

    // Check time difference
    if (ui32_time_last_cmd_seq_updated_us_ > time_now_us)
        ui32_time_diff_from_last_cmd_seq_updated_us_ = ui32_time_last_cmd_seq_updated_us_ - time_now_us;
    else
        ui32_time_diff_from_last_cmd_seq_updated_us_ = time_now_us - ui32_time_last_cmd_seq_updated_us_;

    // Check command is healthy
    if (ui32_time_diff_from_last_cmd_seq_updated_us_ > AutoKUCMD_TIMEOUT_US)
        param_b_vehicle_command_health_is_good_ = false;
    else
        param_b_vehicle_command_health_is_good_ = true;

    return;
}

/**
 * @brief Get AutoKU CAN Status (operation mode, lateral | longitudinal mode)
 *
 * @return AutoKU_CAN_STA
 */
// INTERNAL_CAN_STA AutoKuCanProcess::GetAutoKuCanSta()
// {
//     memset(o_autoku_can_sta_.data, 0, DL_AutoKUSTA);
//     o_autoku_can_sta_.str.operation_mode = param_b_ad_ready_;
//     o_autoku_can_sta_.str.lateral_mode = param_b_lateral_enable_;
//     o_autoku_can_sta_.str.longitudinal_mode = param_b_longitudinal_enable_;
//     return o_autoku_can_sta_;
// }
void AutoKuCanProcess::UpdateAutoKuCanSta()
{
    memset(o_autoku_can_sta_.data, 0, DL_AutoKUSTA);
    o_autoku_can_sta_.str.operation_mode = param_b_ad_ready_;
    o_autoku_can_sta_.str.lateral_mode = param_b_lateral_enable_;
    o_autoku_can_sta_.str.longitudinal_mode = param_b_longitudinal_enable_;

}

// Autonomous Command
/**
 * @brief Steering mode status update with interacting vehicle steering status
 *
 * @param param_ui8_input_steering_status_: target steering control status
 * @param i_vehicle_gway1_.Gway_Steering_Status: vehicle steering status (0 "Reserved" 1 "Steering still in initialization phase" 2 "Steering ready, waits for PA command" 3 "Steering set in standby by PA" 4 "Steering requested to go to first activation step" 5 "Steering requested to go to final activation step" 6 "Steering went to error internally" 7 "Steering aborted the automatic function" 14 "Not Used" 15 "Error Indicator")
 *
 */
void AutoKuCanProcess::SteeringModeStatusUpdate()
{
    if (param_b_lateral_enable_ == 1)
    {
        // Lateral control
        switch (param_ui8_input_steering_status_)
        {
        case 1: // Init
            param_ui8_input_steering_status_ = 2;
            break;

        case 2: // New Start
            if (i_vehicle_gway1_.Gway_Steering_Status <= 3 ||
                i_vehicle_gway1_.Gway_Steering_Status == 6 ||
                i_vehicle_gway1_.Gway_Steering_Status == 7)
            {
                param_ui8_input_steering_status_ = 3;
            }
            break;

        case 3: // Stand By
            if (i_vehicle_gway1_.Gway_Steering_Status == 3)
            {
                param_ui8_input_steering_status_ = 4;
            }
            break;

        case 4: // Active
            if (i_vehicle_gway1_.Gway_Steering_Status == 4)
            {
                param_ui8_input_steering_status_ = 5;
            }
            break;

        case 5: // Angle Control Active
            if (i_vehicle_gway1_.Gway_Steering_Status == 6 ||
                i_vehicle_gway1_.Gway_Steering_Status == 7 ||
                i_vehicle_gway1_.Gway_Steering_Status == 3)
            {
                param_ui8_input_steering_status_ = 1;
            }
            break;

        default:
            param_ui8_input_steering_status_ = 1;
            break;
        } // end of switch
    }
    else
    {
        param_ui8_input_steering_status_ = 1;
    }
    return;
}

/**
 * @brief Vehicle control command generation
 *
 */
void AutoKuCanProcess::AutoKuCmdUpdate()
{
    // Initialize vehicle command
    memset(o_vehicle_can_adcmd_.data, 0, DL_ADCMD);
    External_CAN_ADCMD tmp_control_command;
    memset(tmp_control_command.data, 0, DL_ADCMD);
    InjectCanAutoKUCmdToCanAdcmd(i_autoku_can_cmd_, tmp_control_command);

    // Lateral command input
    AutoKuLatCmdUpdate(tmp_control_command);

    // Longitudinal command input
    AutoKuLonCmdUpdate(tmp_control_command);

    // Validate vehicle command
    AutoKuCmdValidation();

    return;
}

/**
 * @brief Lateral vehicle control value inject to o_vehicle_can_adcmd_
 *
 * @param input_command: input vehicle control command can message
 * @param o_vehicle_can_adcmd_: target of vehicle control command injection
 */
void AutoKuCanProcess::AutoKuLatCmdUpdate(External_CAN_ADCMD input_command)
{
    if (param_b_lateral_enable_ == 1)
    {
        o_vehicle_can_adcmd_.str.ADSteer_Mod1 = 2;
        o_vehicle_can_adcmd_.str.ADSteer_Be = 1;
        o_vehicle_can_adcmd_.str.ADSteer_StrAnglReq = input_command.str.ADSteer_StrAnglReq;
    }
    else
    {
        o_vehicle_can_adcmd_.str.ADSteer_Mod1 = 0;
        o_vehicle_can_adcmd_.str.ADSteer_Be = 0;
        o_vehicle_can_adcmd_.str.ADSteer_StrAnglReq = 0;
    }
    o_vehicle_can_adcmd_.str.ADSteer_Sta = param_ui8_input_steering_status_;
    o_vehicle_can_adcmd_.str.ADSteer_TestModSta = static_cast<uint8_t>(param_b_lateral_enable_);
    return;
}

/**
 * @brief Longitudinal vehicle control value inject to o_vehicle_can_adcmd_
 *
 * @param input_command: input vehicle control command can message
 * @param o_vehicle_can_adcmd_: target of vehicle control command injection
 */
void AutoKuCanProcess::AutoKuLonCmdUpdate(External_CAN_ADCMD input_command)
{
    // Control command set
    if (param_b_longitudinal_enable_ == 1)
    {
        // Gear set
        o_vehicle_can_adcmd_.str.ADCMD_ShifterRND = input_command.str.ADCMD_ShifterRND;
        o_vehicle_can_adcmd_.str.ADCMD_ShifterP = input_command.str.ADCMD_ShifterP;

        o_vehicle_can_adcmd_.str.ADCMD_ModeAct = 2; // Brake
        o_vehicle_can_adcmd_.str.ADCMD_Enable = 1;  // Throttle

        o_vehicle_can_adcmd_.str.ADCMD_AccAps = input_command.str.ADCMD_AccAps;
        o_vehicle_can_adcmd_.str.ADCMD_Brk1 = input_command.str.ADCMD_Brk1;
    }
    else
    {
        // Gear set
        o_vehicle_can_adcmd_.str.ADCMD_ShifterRND = 0;
        o_vehicle_can_adcmd_.str.ADCMD_ShifterP = 0;

        o_vehicle_can_adcmd_.str.ADCMD_ModeAct = 1; // Brake
        o_vehicle_can_adcmd_.str.ADCMD_Enable = 0;  // Throttle

        o_vehicle_can_adcmd_.str.ADCMD_AccAps = 0;
        o_vehicle_can_adcmd_.str.ADCMD_Brk1 = 0;
    }

    return;
}

/**
 * @brief Vehicle door management with parking. Parking when stop vehicle over PARKING_TIME_CONDITION_US. Drive when longitudinal_enable.
 * @brief This function need for door lock release with parking and re shift to origin gear.
 *
 */
void AutoKuCanProcess::GearShiftCmdUpdate(uint32_t time_now_us)
{
    static uint32_t last_moving_time_us = time_now_us;
    static uint32_t last_parking_time_us = time_now_us;
    static uint32_t loop_count_after_stop = 0;
    if (i_vehicle_gway1_.Gway_Wheel_Velocity_FL > __FLT_MIN__) // Vehicle moving condition
    {
        last_moving_time_us = time_now_us;
        loop_count_after_stop = 0;
    }
    else
    {
        // Stop count increase
        loop_count_after_stop++;
    }

    // Initial stop step for stop transient detection
    static bool b_is_parking_changed_this_stop = false;
    if (loop_count_after_stop == 1)
    {
        // Flag initializer when vehicle is stop then start detect parking in this stop session
        b_is_parking_changed_this_stop = false;
    }

    // Gear shifting command is not continuous
    static uint8_t shift_update_delay_count = 0;
    bool gear_shift_enable = false;
    shift_update_delay_count++;
    if (shift_update_delay_count < SHIFT_COMMAND_SKIP_COUNT)
    {
        gear_shift_enable = false;
    }
    else
    {
        shift_update_delay_count = 0;
        gear_shift_enable = true;
    }

    // Parking Gear shifter
    if (param_b_ad_ready_ == true && b_is_parking_changed_this_stop == false)
    {
        uint32_t time_after_stop_us = time_now_us - last_moving_time_us;
        if (time_after_stop_us > PARKING_TIME_CONDITION_US)
        {
            if (i_vehicle_gway1_.Gway_GearSelDisp != 0)
            {
                // Gear shift need ADCMD_Enable to true
                o_vehicle_can_adcmd_.str.ADCMD_ModeAct = 2;                                      // Brake enable
                o_vehicle_can_adcmd_.str.ADCMD_Enable = 1;                                       // Accelrator enable
                o_vehicle_can_adcmd_.str.ADCMD_AccAps = 86;                                      // 0 position
                o_vehicle_can_adcmd_.str.ADCMD_Brk1 = INITIAL_BRAKE_POSITION / ADCMD_BRK1_SCALE; // Brake for drive gear shifting
                if (gear_shift_enable == true)                                                   // Not Parking(0)
                {
                    o_vehicle_can_adcmd_.str.ADCMD_ShifterRND = 0;
                    o_vehicle_can_adcmd_.str.ADCMD_ShifterP = 1;
                }
            }
            else
            {
                // Longitudinal accelration disable when vehicle stop (if not it has problem when reshiftering to Drive is not operated)
                o_vehicle_can_adcmd_.str.ADCMD_Enable = 0;
                o_vehicle_can_adcmd_.str.ADCMD_AccAps = 0;
                b_is_parking_changed_this_stop = true;
            }
        }
    }

    // Detect parking gear shifting
    static uint8_t prev_gear_sel = i_vehicle_gway1_.Gway_GearSelDisp;
    static uint8_t origin_gear_sel = i_vehicle_gway1_.Gway_GearSelDisp; // For gear reverting
    if (prev_gear_sel != 0 && i_vehicle_gway1_.Gway_GearSelDisp == 0)
    {
        // Gear parking shift transient condition
        last_parking_time_us = time_now_us;
        origin_gear_sel = i_vehicle_gway1_.Gway_GearSelDisp;
        b_is_parking_changed_this_stop = true;
    }
    prev_gear_sel = i_vehicle_gway1_.Gway_GearSelDisp;

    // Drive gear shifter
    if (param_b_longitudinal_enable_ == true)
    {
        uint32_t time_after_parking_us = time_now_us - last_parking_time_us;
        if (i_vehicle_gway1_.Gway_GearSelDisp != 5) // origin_gear_sel) // Currently consider only drive gear
        {
            // Longitudinal accelration disable when vehicle stop (if not it has problem when reshiftering to Drive is not operated)
            o_vehicle_can_adcmd_.str.ADCMD_Enable = 0;
            o_vehicle_can_adcmd_.str.ADCMD_AccAps = 0;
            if (time_after_parking_us > RE_DRIVE_TIME_CONDITION_US) // After shift parking some seconds
            {
                // Gear shift need ADCMD_Enable to true
                o_vehicle_can_adcmd_.str.ADCMD_ModeAct = 2;                                      // Brake enable
                o_vehicle_can_adcmd_.str.ADCMD_Enable = 1;                                       // Accelrator enable
                o_vehicle_can_adcmd_.str.ADCMD_AccAps = 86;                                      // 0 position
                o_vehicle_can_adcmd_.str.ADCMD_Brk1 = SHIFTING_BRAKE_PRESURE / ADCMD_BRK1_SCALE; // over 30 bar
                if (gear_shift_enable == true &&
                    i_vehicle_gway1_.Gway_BrakeMasterCylinder_Pressur > SHIFT_ENABLE_BRAKE_PRESURE) // Shifting to drive require the brake is pressed)
                {
                    o_vehicle_can_adcmd_.str.ADCMD_ShifterRND = 5; // origin_gear_sel; // Currently only update drive mode
                    o_vehicle_can_adcmd_.str.ADCMD_ShifterP = 0;
                    last_moving_time_us = time_now_us; // Moving time reset for not shift P.
                    last_parking_time_us = time_now_us;
                }
            }
        }
    }

    return;
}

/**
 * @brief Check vehicle command is validate use health seq timeout condition and set failsafe condition
 *
 * @return true: TBD
 * @return false: TBD
 */
bool AutoKuCanProcess::AutoKuCmdValidation()
{
    // Timeout validation
    if (param_b_ad_ready_ == true &&
        param_b_vehicle_command_health_is_good_ == false)
    {
        // Disable accelration and brake on initial setted brake force (lateral not changed)
        o_vehicle_can_adcmd_.str.ADCMD_ShifterRND = 4; // N
        o_vehicle_can_adcmd_.str.ADCMD_ShifterP = 0;   // non P

        o_vehicle_can_adcmd_.str.ADCMD_ModeAct = 2; // Brake enable
        o_vehicle_can_adcmd_.str.ADCMD_Enable = 0;  // Throttle disable

        o_vehicle_can_adcmd_.str.ADCMD_AccAps = 0;
        o_vehicle_can_adcmd_.str.ADCMD_Brk1 = INITIAL_BRAKE_POSITION / ADCMD_BRK1_SCALE;
    }
    return false;
}

void AutoKuCanProcess::AutonomousCloseCmdUpdate()
{
    External_CAN_ADCMD tmp_control_command;
    memset(tmp_control_command.data, 0, DL_ADCMD);

    // Longitudinal
    // Gear set
    tmp_control_command.str.ADCMD_ShifterRND = 0;
    tmp_control_command.str.ADCMD_ShifterP = 0;

    // Control command set
    tmp_control_command.str.ADCMD_ModeAct = 1; // Brake
    tmp_control_command.str.ADCMD_Enable = 0;  // Throttle

    tmp_control_command.str.ADCMD_AccAps = 0;
    tmp_control_command.str.ADCMD_Brk1 = 0;

    // Lateral
    tmp_control_command.str.ADSteer_Mod1 = 0;
    tmp_control_command.str.ADSteer_Be = 0;
    tmp_control_command.str.ADSteer_StrAnglReq = 0;
    tmp_control_command.str.ADSteer_Sta = 3;
    tmp_control_command.str.ADSteer_TestModSta = 0;

    o_vehicle_can_adcmd_ = tmp_control_command;

    return;
}

// Add 10ms worker
void AutoKuCanProcess::Timer10msProcess(uint32_t time_now_us){
    
    ////////////// Autonomous Mode //////////////
    // Update autonomous mode from user input
    AutonomousModeUpdate();
    // Detect driver overide
    CheckDriverOveride();
    // Update algorithhm input health signal timer
    AdCmdHelthSeqChecker(time_now_us);

    ////////////// Autonomous Command generation //////////////
    // Steering mode update
    SteeringModeStatusUpdate();
    // Vehicle command update
    AutoKuCmdUpdate();
    // Vehicle door update
    GearShiftCmdUpdate(time_now_us);

    ////////////// Output vehicle adcmd can message //////////////


}

// Add 100ms worker
void AutoKuCanProcess::Timer100msProcess(){
    UpdateAutoKuCanSta();
}


// Vehicle gateway

/////////////////////////////////////////////////////////////////
// Callback Fuctions

// Autonomous Mode
void AutoKuCanProcess::CallbackAdMode(uint8_t *data)
{
    if (data == NULL)
        return;
    memset(&i_autoku_ad_mode_, 0, DL_AutoKU_ADMODE);
    memcpy(i_autoku_ad_mode_.data, data, DL_AutoKU_ADMODE);
    i_b_mode_updated_ = true;
    return;
}

// Autonomous Command
void AutoKuCanProcess::CallbackAutoKuHelth(uint8_t *autoku_can_health_data)
{
    if (autoku_can_health_data == NULL)
        return;
    AutoKU_CAN_HELTH health;
    memcpy(health.data, autoku_can_health_data, DL_AutoKU_HEALTH);

    i_ui8_health_seq_number_ = health.str.health_sequence;
    return;
}
void AutoKuCanProcess::CallbackAutoKuCmd(uint8_t *autoku_can_cmd_data)
{
    if (autoku_can_cmd_data == NULL)
        return;
    memset(&i_autoku_can_cmd_, 0, DL_AutoKUCMD);
    memcpy(i_autoku_can_cmd_.data, autoku_can_cmd_data, DL_AutoKUCMD);
    return;
}

// Vehicle gateway
void AutoKuCanProcess::CallbackGway1(uint8_t *gway1_data)
{
    if (gway1_data == NULL)
        return;
    memset(&i_vehicle_can_gway1_, 0, DL_GWAY1);
    memcpy(i_vehicle_can_gway1_.data, gway1_data, DL_GWAY1);
    i_vehicle_gway1_ = DecodeDataCanGway1(i_vehicle_can_gway1_);
    return;
}

// Siren and LED configuration
void AutoKuCanProcess::CallbackSirenLedConfig(uint8_t *siren_speaker_data)
{
    if (siren_speaker_data == NULL)
        return;
    memset(&i_autoku_siren_on_off_, 0, DL_AutoKU_SIREN_ON_OFF);
    memcpy(i_autoku_siren_on_off_.data, siren_speaker_data, DL_AutoKU_SIREN_ON_OFF);

    // Apply configuration

    // Siren
    switch (i_autoku_siren_on_off_.str.siren_on_off)
    {
    case 1:
        param_b_siren_on_ = 1; // on
        break;
    case 2:
        param_b_siren_on_ = 2; // blink
        break;
    case 0:
        param_b_siren_on_ = 0; // off
        break;
    case 7:                    // 7: e_flag_mode
        param_b_siren_on_ = 7; // 7: e_flag_mode
        break;
    default:
        break;
    }

    // LED blue
    switch (i_autoku_siren_on_off_.str.led_on_off)
    {
    case 1:
    case 5:
        param_b_led_blue_on_ = 1; // on
        break;
    case 2:
    case 6:
        param_b_led_blue_on_ = 2; // blink
        break;
    case 0:
    case 3:
    case 4:
        param_b_led_blue_on_ = 0; // off
        break;
    case 7:                       // 7: e_flag_mode
        param_b_led_blue_on_ = 7; // 7: e_flag_mode
        break;
    default:
        break;
    }

    // LED orange
    switch (i_autoku_siren_on_off_.str.led_on_off)
    {
    case 3:
    case 5:
        param_b_led_orange_on_ = 1; // on
        break;
    case 4:
    case 6:
        param_b_led_orange_on_ = 2; // blink
        break;
    case 0:
    case 1:
    case 2:
        param_b_led_orange_on_ = 0; // off
        break;
    case 7:                         // 7: e_flag_mode
        param_b_led_orange_on_ = 7; // 7: e_flag_mode
        break;
    default:
        break;
    }

    return;
}

// Three Sec E Flag
void AutoKuCanProcess::CallbackEFlag(uint8_t *e_flag_data)
{
    if (e_flag_data == NULL)
        return;
    memset(&i_three_secs_e_flag_can_, 0, DL_E_FLAG);
    memcpy(i_three_secs_e_flag_can_.data, e_flag_data, DL_E_FLAG);
    return;
}


External_Decoded_GWAY1 DecodeDataCanGway1(const External_CAN_GWAY1 &can)
{
    External_Decoded_GWAY1 decode_gway1;

    decode_gway1.Gway_SAS_Angle = static_cast<float>(can.str.Gway_SAS_Angle) * GWAY1_SAS_ANGLE_SCALE;
    decode_gway1.Gway_SAS_Speed = static_cast<float>(can.str.Gway_SAS_Speed) * GWAY1_SAS_SPEED_SCALE;
    decode_gway1.Gway_Cluster_Odometer = static_cast<float>(can.str.Gway_Cluster_Odometer) * GWAY1_CLUSTER_ODOMETER_SCALE;
    decode_gway1.Gway_Lateral_Accel_Speed = static_cast<float>(can.str.Gway_Lateral_Accel_Speed) * GWAY1_LATERAL_ACCEL_SPEED_SCALE + GWAY1_LATERAL_ACCEL_SPEED_OFFSET;
    decode_gway1.Gway_BrakeMasterCylinder_Pressur = can.str.Gway_BrakeCylinder_Pressure;
    decode_gway1.Gway_Steering_Status = can.str.Gway_Steering_Status;
    decode_gway1.Gway_Longitudinal_Accel_Speed = static_cast<float>(can.str.Gway_Longitudinal_Accel_Speed) * GWAY1_LONGITUDINAL_ACCEL_SPEED_SCALE + GWAY1_LONGITUDINAL_ACCEL_SPEED_OFFSET;
    decode_gway1.Gway_Yaw_Rate_Sensor = static_cast<float>(can.str.Gway_Yaw_Rate_Sensor) * GWAY1_YAW_RATE_SENSOR_SCALE + GWAY1_YAW_RATE_SENSOR_OFFSET;
    decode_gway1.Gway_Wheel_Velocity_FR = static_cast<float>(can.str.Gway_Wheel_Velocity_FR) * GWAY1_WHEEL_VELOCITY_SCALE;
    decode_gway1.Gway_Brake_Active = can.str.Gway_Brake_Active;
    decode_gway1.Gway_Wheel_Velocity_RL = static_cast<float>(can.str.Gway_Wheel_Velocity_RL) * GWAY1_WHEEL_VELOCITY_SCALE;
    decode_gway1.Gway_Wheel_Velocity_RR = static_cast<float>(can.str.Gway_Wheel_Velocity_RR) * GWAY1_WHEEL_VELOCITY_SCALE;
    decode_gway1.Gway_Wheel_Velocity_FL = static_cast<float>(can.str.Gway_Wheel_Velocity_FL) * GWAY1_WHEEL_VELOCITY_SCALE;
    decode_gway1.Gway_Steering_Angle = static_cast<float>(can.str.Gway_Steering_Angle) * GWAY1_STEERING_ANGLE_SCALE;
    decode_gway1.Gway_Steering_Tq = static_cast<float>(can.str.Gway_Steering_Tq) * GWAY1_STEERING_TQ_SCALE + GWAY1_STEERING_TQ_OFFSET;
    decode_gway1.Gway_Accel_Pedal_Position = static_cast<float>(can.str.Gway_Accel_Pedal_Position) * GWAY1_ACCEL_PEDAL_POSITION_SCALE;
    decode_gway1.Gway_GearSelDisp = can.str.Gway_GearSelDisp;
    decode_gway1.F_MCU_Torque = static_cast<float>(can.str.F_MCU_Torque) * GWAY1_MCU_TORQUE_SCALE;
    decode_gway1.R_MCU_Torque = static_cast<float>(can.str.R_MCU_Torque) * GWAY1_MCU_TORQUE_SCALE;

    return decode_gway1;
}

void InjectCanAutoKUCmdToCanAdcmd(const INTERNAL_CAN_CMD &autoku_cmd, External_CAN_ADCMD &adcmd)
{
    float target_steering_angle = autoku_cmd.str.target_steering_angle * AutoKUCMD_STEERING_ANGLE_SCALE;
    // float target_acceleration = autoku_cmd.str.target_acceleration * AutoKUCMD_ACCELERATION_SCALE; // Used in LAB ONIQ ver
    // float target_speed = autoku_cmd.str.target_speed * AutoKUCMD_SPEED_SCALE; // Used in LAB ONIQ ver
    float target_acc_pedal_pos = autoku_cmd.str.target_acc_pedal_pos * AutoKUCMD_ACC_PEDAL_POS_SCALE;
    float target_brake_pedal_pos = autoku_cmd.str.target_brake_pedal_pos * AutoKUCMD_BRAKE_PEDAL_POS_SCALE / 2.0; // "/ 2.0" is on origin code autoku_vehicle_can.can in vector canoe repository (https://github.com/ailab-konkuk/vector_canoe)
    uint8_t target_gear = autoku_cmd.str.target_gear;

    // Set gear
    uint8_t target_gear_shifter_p = 0;
    uint8_t target_gear_shifter_rnd = 0;
    if (target_gear == 1) // P
    {
        target_gear_shifter_p = 1;
        target_gear_shifter_rnd = 0;
    }
    else if (target_gear == 2) // R
    {
        target_gear_shifter_p = 0;
        target_gear_shifter_rnd = 1;
    }
    else if (target_gear == 3) // N
    {
        target_gear_shifter_p = 0;
        target_gear_shifter_rnd = 4;
    }
    else if (target_gear == 4) // D
    {
        target_gear_shifter_p = 0;
        target_gear_shifter_rnd = 5;
    }
    else // NONE
    {
        target_gear_shifter_p = 0;
        target_gear_shifter_rnd = 0;
    }

    // Inject AutoKUCMD to ADCMD
    //(target_acc_pedal_pos + ADCMD_ACC_APS_OFFSET) / ADCMD_ACC_APS_SCALE;
    adcmd.str.ADCMD_AccAps = 3.009 * target_acc_pedal_pos + 87.12; // Value need to check. Not equal to can dbc.
    adcmd.str.ADCMD_Brk1 = target_brake_pedal_pos / ADCMD_BRK1_SCALE;

    // adcmd.str.ADCMD_ShifterRND = target_gear_shifter_rnd;
    // adcmd.str.ADCMD_ShifterP = target_gear_shifter_p;

    adcmd.str.ADSteer_StrAnglReq = target_steering_angle / ADCMD_STR_ANGLE_REQ_SCALE;

    return;
}