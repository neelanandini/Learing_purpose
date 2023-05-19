/**
 * @file vcuSys.h
 *
 * @brief Header file for the Vehicle Control Unit (VCU) system.
 *
 * This file contains the definitions and declarations related to the VCU system.
 *
 * @date Jan 2, 2023
 * @author kanhu
 */

#ifndef VCUSYS_H_
#define VCUSYS_H_

#include "Cpu.h"
#include "Debug_Print.h"
#include "HSD_DRV.h"
#include "sevcon.h"

/**
 * @brief Macro to read an active low pin in a GPIO input value.
 *
 * @param GPIOinputValue The GPIO input value.
 * @param PIN The pin number.
 * @return True if the pin is active low, false otherwise.
 */
#define READ_ACTIVE_LOW_PIN(GPIOinputValue, PIN)  ((~((GPIOinputValue >> PIN) | 0xFFFFFFFE)) ? true : false)

/**
 * @brief Macro to read an active high pin in a GPIO input value.
 *
 * @param GPIOinputValue The GPIO input value.
 * @param PIN The pin number.
 * @return True if the pin is active high, false otherwise.
 */
#define READ_ACTIVE_HIGH_PIN(GPIOinputValue, PIN)  (((GPIOinputValue >> PIN) & 0x00000001) ? true : false)

/**
 * @brief Number of receive buffers for VHCL_CAN1.
 */
#define VHCL_CAN1_NUMBER_OF_RECEIVE_BUFFERS 10

/**
 * @brief Number of receive buffers for VHCL_CAN2.
 */
#define VHCL_CAN2_NUMBER_OF_RECEIVE_BUFFERS 10

/**
 * @brief Conversion factor for throttle voltage.
 */
#define THROTTLE_VOLTAGE_CONVERSION_FACTOR 0.000305 // 5.0 / 16385.0

/**
 * @brief Highest RPM in the forward direction for Sevcon.
 */
#define SEVCON_HIGHEST_RPM_IN_FORWARD_DIRECTION 4000 // In RPM

/**
 * @brief Highest RPM in the reverse direction for Sevcon.
 */
#define SEVCON_HIGHEST_RPM_IN_REVERSE_DIRECTION 2000 // In RPM

/**
 * @brief Scaling factor for Sevcon throttle voltage in CAN message.
 */
#define SEVCON_THROTTLE_VOLTAGE_SCALING_FACTOR 256 // (Analog value * scaling factor) for CAN message

/**
 * @brief Start voltage for Sevcon throttle.
 */
#define SEVCON_THROTTLE_START_VOLTAGE 1.8 // As configured in Sevcon

/**
 * @brief End voltage for Sevcon throttle.
 */
#define SEVCON_THROTTLE_END_VOLTAGE 4.3 // As configured in Sevcon

/**
 * @brief CAN ID for VCU drive command.
 */
#define VCU_DRIVE_COMMAND_CANID 0x188

/**
 * @brief CAN instance options for VCU.
 */
typedef enum
{
    VHCL_CANBUS2 = 3, // CAN3 - MCAN1
    VHCL_CANBUS1 = 4, // CAN4 - FLEXCANC
} vcuCANinstant_t;

/**
 * @brief VCU operation modes.
 */
typedef enum
{
    MANUAL_MODE = 0,
    AUTONOMOUS_MODE = 1
} vcuMode_t;

/**
 * @brief Structure representing the OBC interface data.
 */
typedef struct
{
    bool handshakeInitiatedFlag;

    // OBC input request status
    bool OBCfootBrakeRequestFlag;
    bool OBCdriveSwitchRequestFlag;
    bool OBCreverseSwitchRequestFlag;
    bool OBCheadlampRequestFlag;
    bool OBCleftIndicatorRequestFlag;
    bool OBCrightIndicatorRequestFlag;
    bool OBChornRequestFlag;

    float throttleVoltage;
} OBC_interfaceStruct_t;

/**
 * @brief Structure representing the VCU system state.
 */
typedef struct
{
    vcuMode_t VCU_MODE;

    // System input status
    bool FootBrakeInputFlag;
    bool driveSwitchInputFlag;
    bool reverseSwitchInputFlag;
    bool handbrakeInputFlag;
    bool headlampInputFlag;
    bool leftIndicatorInputFlag;
    bool rightIndicatorInputFlag;
    bool hornInputFlag;

    bool emergencyStopInputFlag;

    bool keyPos1InputFlag;
    bool keyPos2InputFlag;

    float throttleInputVolt;

    // Powertrain status
    int32_t motorRPM;

    OBC_interfaceStruct_t OBC_interfaceData;

} vcuSysState_t;

/**
 * @brief Structure representing a receive buffer for FlexCAN messages.
 */
typedef struct
{
    vcuCANinstant_t VCU_CAN_INST;

    flexcan_msgbuff_t flexCANdata;
    uint32_t timestamp;
    bool isFull;

} flexCANreceiveBuffer_t;

/**
 * @brief Structure representing a receive buffer for MCAN messages.
 */
typedef struct
{
    vcuCANinstant_t VCU_CAN_INST;

    mcan_msgbuff_t MCANdata;
    uint32_t timestamp;
    bool isFull;

} MCANreceiveBuffer_t;

/**
 * @brief Structure representing a generic CAN frame.
 */
typedef struct
{
    uint32_t can_id;
    bool isStandardFrame;
    uint8_t can_dataLength;
    uint8_t dataArray[8];
} genericCANframeData_t;

/**
 * @brief Reads user inputs and updates the VCU system state.
 */
void readUserInputs();

/**
 * @brief Sends the drive command over the CAN bus.
 */
void sendDriveCommand();

/**
 * @brief Sends a CAN frame over the specified CAN instance.
 *
 * @param vcuCANinstant The CAN instance to send the frame on.
 * @param canData The CAN frame data to be sent.
 */
void sendCANFrame(vcuCANinstant_t vcuCANinstant, genericCANframeData_t *canData);

/**
 * @brief Updates the manual drive command CAN data based on the VCU state.
 *
 * @param vcuStateData The VCU system state.
 * @param manualDriveCommandCANdata The manual drive command CAN data to be updated.
 */
void updateManualDriveCommandCANdata(vcuSysState_t *vcuStateData, genericCANframeData_t *manualDriveCommandCANdata);

/**
 * @brief Monitors the powertrain and updates the VCU state based on the received CAN data.
 *
 * @param vcuState The VCU system state.
 * @param CANdata The received CAN data.
 */
void monitorPowertrain(vcuSysState_t *vcuState, genericCANframeData_t *CANdata);

#endif /* VCUSYS_H_ */
