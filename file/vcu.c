/**
 * @file vcuSys.c
 * @brief This file contains the implementation of the VCU system functions.
 *
 * This file implements the VCU system functions, including reading user inputs, sending drive commands, and monitoring powertrain.
 */

#include "vcuSys.h"

/**
 * @brief Flag indicating whether debug mode is enabled or not.
 */
bool debugFlag = true;

/**
 * @brief Global variables for VCU system state data.
 */
extern vcuSysState_t vcuStateData;

/**
 * @brief Global variables for FlexCAN and MCAN data information.
 */
extern flexcan_data_info_t FlexCAN_genericTransmitData;
extern mcan_data_info_t MCAN_genericTransmitData;

/**
 * @brief Global variables for manual and OBC drive command CAN data.
 */
genericCANframeData_t manualDriveCommandCANdata, OBC_DriveCommandCANdata;

/**
 * @brief Global variables for throttle channel voltage.
 */
float throttleCh1Volt, throttleCh2Volt;

/**
 * @brief Reads user inputs from various sensors and updates VCU state data.
 *
 * This function reads the throttle voltage, drive switch input, reverse switch input, foot brake input, handbrake input,
 * and emergency stop input. It then updates the VCU state data accordingly.
 */
void readUserInputs()
{
    ADC_StartGroupConversion(&ADC_THROTTLE_instance, 0U);
    throttleCh1Volt = ((float)ADC_THROTTLE_Results00[0]) * THROTTLE_VOLTAGE_CONVERSION_FACTOR;
    throttleCh2Volt = ((float)ADC_THROTTLE_Results00[1]) * THROTTLE_VOLTAGE_CONVERSION_FACTOR;
    if (throttleCh1Volt < SEVCON_THROTTLE_START_VOLTAGE)
    {
        throttleCh1Volt = SEVCON_THROTTLE_START_VOLTAGE ;
    }
    else if (throttleCh1Volt > SEVCON_THROTTLE_END_VOLTAGE)
    {
        throttleCh2Volt = SEVCON_THROTTLE_END_VOLTAGE ;
    }

    vcuStateData.throttleInputVolt = throttleCh1Volt;

    pins_channel_type_t tempGPIOInputVal = PINS_DRV_ReadPins(DriveSwitchInputPin_PORT);
    vcuStateData.driveSwitchInputFlag = READ_ACTIVE_LOW_PIN(tempGPIOInputVal, DriveSwitchInputPin_PIN);
    vcuStateData.reverseSwitchInputFlag = READ_ACTIVE_LOW_PIN(tempGPIOInputVal, ReverseSwitchInputPin_PIN);

    //tempGPIOInputVal = PINS_DRV_ReadPins(FootbrakeInputPin_PORT);
    vcuStateData.FootBrakeInputFlag = READ_ACTIVE_HIGH_PIN(tempGPIOInputVal, FootbrakeInputPin_PIN);
    vcuStateData.handbrakeInputFlag = READ_ACTIVE_LOW_PIN(tempGPIOInputVal, HandbrakeInputPin_PIN);

    tempGPIOInputVal = PINS_DRV_ReadPins(EmergencyStopInputPin_PORT);
    vcuStateData.emergencyStopInputFlag = READ_ACTIVE_HIGH_PIN(tempGPIOInputVal, EmergencyStopInputPin_PIN);

    updateManualDriveCommandCANdata(&vcuStateData, &manualDriveCommandCANdata);
}

/**
 * @brief Sends the drive command over the CAN bus based on the VCU mode.
 *
 * This function sends the drive command over the CAN bus based on the current VCU mode. If the VCU mode is MANUAL_MODE,
 * it sends the manual drive command to both VHCL_CANBUS1 and VHCL_CANBUS2. If the VCU mode is AUTONOMOUS_MODE,
 * it sends the OBC drive command to VHCL_CANBUS1 and the manual drive command to VHCL_CANBUS2.
 */
void sendDriveCommand()
{
    if(vcuStateData.VCU_MODE == MANUAL_MODE)
    {
        sendCANFrame(VHCL_CANBUS1, &manualDriveCommandCANdata);
        sendCANFrame(VHCL_CANBUS2, &manualDriveCommandCANdata);
    }
    else if (vcuStateData.VCU_MODE == AUTONOMOUS_MODE)
    {
        sendCANFrame(VHCL_CANBUS1, &OBC_DriveCommandCANdata);
        sendCANFrame(VHCL_CANBUS2, &manualDriveCommandCANdata);
    }
}

/**
 * @brief Sends a CAN frame based on the given VCU CAN instance and CAN data.
 *
 * This function sends a CAN frame based on the given VCU CAN instance and CAN data. If the VCU CAN instance is VHCL_CANBUS1,
 * it uses FlexCAN to send the CAN frame. If the VCU CAN instance is VHCL_CANBUS2, it uses MCAN to send the CAN frame.
 *
 * @param[in] vcuCANinstant The VCU CAN instance.
 * @param[in] canData The CAN data to be sent.
 */
inline void sendCANFrame(vcuCANinstant_t vcuCANinstant, genericCANframeData_t *canData)
{
    if (VHCL_CANBUS1 == vcuCANinstant)
    {
        FlexCAN_genericTransmitData.data_length = canData->can_dataLength;
        FlexCAN_genericTransmitData.msg_id_type = canData->isStandardFrame ?
                                                FLEXCAN_MSG_ID_STD : FLEXCAN_MSG_ID_EXT;
        FLEXCAN_DRV_Send(CAN4, 0, &FlexCAN_genericTransmitData,
                        canData->can_id, canData->dataArray);
    }
    else if (VHCL_CANBUS2 == vcuCANinstant)
    {
        MCAN_genericTransmitData.data_length = canData->can_dataLength;
        MCAN_genericTransmitData.msg_id_type = canData->isStandardFrame ?
                                                MCAN_MSG_ID_STD : MCAN_MSG_ID_EXT;
        MCAN_DRV_Send(CAN3, 0, &MCAN_genericTransmitData, canData->can_id, canData->dataArray);
    }
}

/**
 * @brief Updates the manual drive command CAN data based on the VCU state data.
 *
 * This function updates the manual drive command CAN data based on the current VCU state data. It converts the throttle input
 * voltage to a scaled value and updates the appropriate data array elements. It also updates the PT switch input data based
 * on the drive switch input, reverse switch input, foot brake input, and handbrake input.
 *
 * @param[in] vcuStateData Pointer to the VCU system state data.
 * @param[out] manualDriveCommandCANdata Pointer to the manual drive command CAN data.
 */
inline void updateManualDriveCommandCANdata(vcuSysState_t *vcuStateData,
                                            genericCANframeData_t *manualDriveCommandCANdata)
{
    uint16_t tempThrottleScaledValue = (uint16_t)(vcuStateData->throttleInputVolt *
                                                  SEVCON_THROTTLE_VOLTAGE_SCALING_FACTOR);
    manualDriveCommandCANdata->dataArray[0] = tempThrottleScaledValue & 0x00FF;
    manualDriveCommandCANdata->dataArray[1] = (tempThrottleScaledValue & 0xFF00) >> 8;

    uint8_t tempPTSwitchInputCANdata1 = 0;
    if (vcuStateData->driveSwitchInputFlag)
    {
        tempPTSwitchInputCANdata1 |= 0x01;
    }
    else if (vcuStateData->reverseSwitchInputFlag)
    {
        tempPTSwitchInputCANdata1 |= 0x02;
    }
    else
    {
        tempPTSwitchInputCANdata1 |= 0x00;
    }

    if (vcuStateData->FootBrakeInputFlag)
    {
        tempPTSwitchInputCANdata1 |= 0x04;
    }
    if (vcuStateData->handbrakeInputFlag)
    {
        tempPTSwitchInputCANdata1 |= 0x08;
    }

    manualDriveCommandCANdata->dataArray[2] = tempPTSwitchInputCANdata1;
}

/**
 * @brief Monitors the powertrain and updates the VCU state data.
 *
 * This function monitors the powertrain by analyzing the received CAN data. Based on the CAN ID, it updates the relevant
 * VCU state data. Currently, it only handles the MOTOR_RPM_CANID and updates the motor RPM value.
 *
 * @param[in] vcuState Pointer to the VCU system state data.
 * @param[in] CANdata Pointer to the received CAN data.
 */
void monitorPowertrain(vcuSysState_t *vcuState, genericCANframeData_t *CANdata)
{
    switch (CANdata->can_id)
    {
    case MOTOR_RPM_CANID:
        vcuState->motorRPM = CANdata->dataArray[7];
        vcuState->motorRPM = ((vcuState->motorRPM << 8) & 0x0000FF00) | CANdata->dataArray[6];
        vcuState->motorRPM = ((vcuState->motorRPM << 8) & 0x00FFFF00) | CANdata->dataArray[5];
        vcuState->motorRPM = ((vcuState->motorRPM << 8) & 0xFFFFFF00) | CANdata->dataArray[4];
        break;

    default:
        break;
    }
}
