/*
 * vcuSys.h
 *
 *  Created on: Jan 2, 2023
 *      Author: kanhu
 */

#ifndef VCUSYS_H_
#define VCUSYS_H_

#include "Cpu.h"
#include "Debug_Print.h"
#include "HSD_DRV.h"
#include "LSD_DRV.h"
#include "sevcon.h"

#define OE_EPAS //OE_EPAS, RANE_EPAS
#define FNR_ACTIVE_HIGH //FNR_ACTIVE_HIGH, FNR_ACTIVE_LOW

#define READ_ACTIVE_LOW_PIN(GPIOinputValue, PIN)  ((~((GPIOinputValue >> PIN) | 0xFFFFFFFE)) ? true : false)

#define READ_ACTIVE_HIGH_PIN(GPIOinputValue, PIN)  (((GPIOinputValue >> PIN) & 0x00000001) ? true : false)


#define VHCL_CAN1_NUMBER_OF_RECEIVE_BUFFERS 10
#define VHCL_CAN2_NUMBER_OF_RECEIVE_BUFFERS 10

#define THROTTLE_VOLTAGE_CONVERSION_FACTOR 0.000305 // 5.0 /16385.0

#define SEVCON_HIGHEST_RPM_IN_FORWARD_DIRECTION   4000  //In RPM
#define SEVCON_HIGHEST_RPM_IN_REVERSE_DIRECTION   1500  //In RPM

#define SEVCON_THROTTLE_VOLTAGE_SCALING_FACTOR 256  //( Analog value * scaling factor ) for CAN message

#define SEVCON_THROTTLE_START_VOLTAGE 1.9 //As configured in Sevcon
#define SEVCON_THROTTLE_END_VOLTAGE 4.59   //As configured in Sevcon

#define VCU_DRIVE_COMMAND_CANID 0x188

#define OBC_REQUEST_CANID 					0x501
#define OBC_VCU_RESPONSE_CANID				0x503
#define OBC_DRIVE_COMMAND_CANID             0x510

#if defined(OE_EPAS)
	#define EPAS_STATUS_MSG1 					0x18F
    #define EPAS_COMMAND_MSG1 					0x314

	#define EPAS_MAX_ANG_VELOCITY               200 // min: 50, max: 540 degree/sec
	#define EPAS_MAX_ANG_INCREMENT              200
	#define EPAS_MAX_ANGLE_LEFT					510
	#define EPAS_MAX_ANGLE_RIGHT				-510

#elif defined(RANE_EPAS)
	#define EPAS_STATUS_MSG1 					0x290
	#define EPAS_STATUS_MSG2 					0x292
    #define EPAS_COMMAND_MSG1 					0x298

#endif

#define OBC_STEERING_COMMAND_CANID          	EPAS_COMMAND_MSG1
#define OBC_HANDSHAKE_REQUEST_CODE 				101


#define CAN1 	INST_FLEXCANA
#define CAN2 	INST_FLEXCAND
#define CAN3 	INST_MCAN1
#define CAN4	INST_FLEXCANC
#define CAN5	INST_MCAN0
#define CAN6	INST_FLEXCANB


typedef enum
{
	//VCU_CAN1 = 1, //CAN1 - FlexCANA
	//VCU_CAN2 = 2, //CAN2 - FLexCAND

	VHCL_CANBUS2 = 3, //CAN3 - MCAN1
	VHCL_CANBUS1 = 4, //CAN4 - FLEXCANC

	//VCU_CAN5 = 5, //CAN5 - MCAN0
	//VCU_CAN6 = 6, //CAN6 - FLEXCANB -> SBC_CAN

}vcuCANinstant_t;


typedef enum
{
	MANUAL_MODE = 0,
	AUTONOMOUS_MODE = 1
}vcuMode_t;

typedef struct
{
	bool handshakeInitiatedFlag;

	//OBC input request status
	bool OBCfootBrakeRequestFlag;
	bool OBCdriveSwitchRequestFlag;
	bool OBCreverseSwitchRequestFlag;
	bool OBCheadlampRequestFlag;
	bool OBCleftIndicatorRequestFlag;
	bool OBCrightIndicatorRequestFlag;
	bool OBChornRequestFlag;

	float throttleVoltage;
}OBC_interfaceStruct_t;

typedef struct
{
	vcuMode_t VCU_MODE;

	//System input status
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

	//Powertrain status
	int32_t motorRPM;

	OBC_interfaceStruct_t OBC_interfaceData;

}vcuSysState_t;

typedef struct
{
	vcuCANinstant_t VCU_CAN_INST;

	flexcan_msgbuff_t flexCANdata;
	uint32_t timestamp;
	bool isFull;

}flexCANreceiveBuffer_t;

typedef struct
{
	vcuCANinstant_t VCU_CAN_INST;

	mcan_msgbuff_t MCANdata;
	uint32_t timestamp;
	bool isFull;

}MCANreceiveBuffer_t;

typedef struct
{
	uint32_t can_id;
	bool isStandardFrame;
	uint8_t can_dataLength;
	uint8_t dataArray[8];
}genericCANframeData_t;


void readUserInputs();
void sendDriveCommand();
int32_t sendCANFrame(vcuCANinstant_t vcuCANinstant, genericCANframeData_t *canData);
void updateManualDriveCommandCANdata(vcuSysState_t *vcuStateData, genericCANframeData_t *manualDriveCommandCANdata);
void monitorPowertrain(vcuSysState_t *vcuState, genericCANframeData_t *CANdata);
void VCUmodeChange(vcuMode_t destinationMode, uint32_t timestamp_ms);


#endif /* VCUSYS_H_ */
