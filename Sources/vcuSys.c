/*
 * vcuSys.c
 *
 *  Created on: Jan 2, 2023
 *      Author: kanhu
 */

#include "vcuSys.h"

bool debugFlag = true;

vcuSysState_t vcuStateData;

extern flexcan_data_info_t FlexCAN_genericTransmitData;
extern mcan_data_info_t MCAN_genericTransmitData;

genericCANframeData_t manualDriveCommandCANdata, OBC_DriveCommandCANdata;

float throttleCh1Volt, throttleCh2Volt;


void readUserInputs()
{
	ADC_StartGroupConversion(&ADC_THROTTLE_instance, 0U);
	throttleCh1Volt = ((float)ADC_THROTTLE_Results00[0]) * THROTTLE_VOLTAGE_CONVERSION_FACTOR;
	throttleCh2Volt = ((float)ADC_THROTTLE_Results00[1]) * THROTTLE_VOLTAGE_CONVERSION_FACTOR;
	if (throttleCh1Volt < SEVCON_THROTTLE_START_VOLTAGE)
	{
		throttleCh1Volt = SEVCON_THROTTLE_START_VOLTAGE ;
	}
	else if(throttleCh1Volt > SEVCON_THROTTLE_END_VOLTAGE)
	{
		throttleCh2Volt = SEVCON_THROTTLE_END_VOLTAGE ;
	}

	vcuStateData.throttleInputVolt = throttleCh1Volt;


	pins_channel_type_t tempGPIOInputVal = PINS_DRV_ReadPins(DriveSwitchInputPin_PORT);

	#if defined(FNR_ACTIVE_HIGH)

	vcuStateData.driveSwitchInputFlag = READ_ACTIVE_HIGH_PIN(tempGPIOInputVal, DriveSwitchInputPin_PIN);
	vcuStateData.reverseSwitchInputFlag = READ_ACTIVE_HIGH_PIN(tempGPIOInputVal, ReverseSwitchInputPin_PIN);

	#elif defined(FNR_ACTIVE_LOW)
	vcuStateData.driveSwitchInputFlag = READ_ACTIVE_LOW_PIN(tempGPIOInputVal, DriveSwitchInputPin_PIN);
	vcuStateData.reverseSwitchInputFlag = READ_ACTIVE_LOW_PIN(tempGPIOInputVal, ReverseSwitchInputPin_PIN);

	#endif

	//tempGPIOInputVal = PINS_DRV_ReadPins(FootbrakeInputPin_PORT);
	vcuStateData.FootBrakeInputFlag = READ_ACTIVE_HIGH_PIN(tempGPIOInputVal, FootbrakeInputPin_PIN);
	vcuStateData.handbrakeInputFlag = READ_ACTIVE_LOW_PIN(tempGPIOInputVal, HandbrakeInputPin_PIN);

	tempGPIOInputVal = PINS_DRV_ReadPins(EmergencyStopInputPin_PORT);
	vcuStateData.emergencyStopInputFlag = READ_ACTIVE_HIGH_PIN(tempGPIOInputVal, EmergencyStopInputPin_PIN);


	updateManualDriveCommandCANdata(&vcuStateData, &manualDriveCommandCANdata);

}


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

int32_t sendCANFrame(vcuCANinstant_t vcuCANinstant, genericCANframeData_t *canData)
{
	if (VHCL_CANBUS1 == vcuCANinstant)
	{
		FlexCAN_genericTransmitData.data_length = canData->can_dataLength;
		FlexCAN_genericTransmitData.msg_id_type = canData->isStandardFrame ?
											FLEXCAN_MSG_ID_STD : FLEXCAN_MSG_ID_EXT;
		if (FLEXCAN_DRV_Send(CAN4, 0, &FlexCAN_genericTransmitData,
						canData->can_id, canData->dataArray) == STATUS_SUCCESS)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else if (VHCL_CANBUS2 == vcuCANinstant)
	{
		MCAN_genericTransmitData.data_length = canData->can_dataLength;
		MCAN_genericTransmitData.msg_id_type = canData->isStandardFrame ?
												MCAN_MSG_ID_STD : MCAN_MSG_ID_EXT;

		if (MCAN_DRV_Send(CAN3, 0, &MCAN_genericTransmitData, canData->can_id, canData->dataArray)
				== STATUS_SUCCESS)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}

	return 0;

}


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

	default :
		break;
	}
}


void VCUmodeChange(vcuMode_t destinationMode, uint32_t timestamp_ms)
{
	if (destinationMode == AUTONOMOUS_MODE)
	{
		vcuStateData.VCU_MODE = AUTONOMOUS_MODE;
		vcuStateData.OBC_interfaceData.throttleVoltage = SEVCON_THROTTLE_START_VOLTAGE;
		vcuStateData.OBC_interfaceData.OBCdriveSwitchRequestFlag = false;
		vcuStateData.OBC_interfaceData.OBCfootBrakeRequestFlag = false;
		vcuStateData.OBC_interfaceData.OBCheadlampRequestFlag = false;
		vcuStateData.OBC_interfaceData.OBChornRequestFlag = false;
		vcuStateData.OBC_interfaceData.OBCleftIndicatorRequestFlag = false;
		vcuStateData.OBC_interfaceData.OBCreverseSwitchRequestFlag = false;
		vcuStateData.OBC_interfaceData.OBCrightIndicatorRequestFlag = false;
		vcuStateData.OBC_interfaceData.handshakeInitiatedFlag = false;
	}
	else if (destinationMode == MANUAL_MODE)
	{
		vcuStateData.VCU_MODE = MANUAL_MODE;
	}
}





