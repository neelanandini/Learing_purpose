/*
 * OBC_CANinterface.c
 *
 *  Created on: Jan 15, 2023
 *      Author: kanhu
 */

#include "OBC_CANinterface.h"

genericCANframeData_t tempCANtransmitData;

volatile uint32_t handshakeInitTimestamp, lastOBCdriveCommandTimestamp,
			lastStatusCheckMsgReceiveTimestamp, lastStatusCheckMsgTransmitTimestamp,
			lastAutonomousModeExitTimestamp;
bool statusCheckMsgSent;

uint32_t statusCheckMsgMissedCounter, commandMsgMissedCounter;

vcuSysState_t vcuStateData_atAutonomousEntry;

extern genericCANframeData_t OBC_DriveCommandCANdata;

void updateOBCdriveCommand(vcuSysState_t *vcuState, genericCANframeData_t *OBC_DriveCommandCANdata);
void manualToAutonomousTransition(vcuSysState_t *currentVcuStateData, uint32_t timestamp);


OBCinterfaceStatus_t OBC_CANinterface(genericCANframeData_t *CANdataFromOBC,
						  vcuSysState_t *vcuStateData, uint32_t timestamp)
{
	if (vcuStateData->VCU_MODE == MANUAL_MODE && ((timestamp - lastAutonomousModeExitTimestamp) > 5000))
	{
		if(CANdataFromOBC->can_id == OBC_REQUEST_CANID)
		{
			if ((!vcuStateData->OBC_interfaceData.handshakeInitiatedFlag)
				&& (CANdataFromOBC->dataArray[0] == OBC_HANDSHAKE_REQUEST_CODE)
				&& (CANdataFromOBC->dataArray[1] == 0)
				&& (CANdataFromOBC->dataArray[2] == 0))
			{
				if((vcuStateData->motorRPM < MOTOR_RPM_THRESHOLD_FOR_SWITCHING_TO_AUTONOMOUS)
					&& (vcuStateData->throttleInputVolt < THROTTLE_INPUT_THRESHOLD_FOR_SWITCHING_TO_AUTONOMOUS)
					&& (!vcuStateData->FootBrakeInputFlag) && (!vcuStateData->handbrakeInputFlag)
					&& (!vcuStateData->emergencyStopInputFlag))
				{
					tempCANtransmitData.can_id = OBC_VCU_RESPONSE_CANID;
					tempCANtransmitData.can_dataLength = 8;
					tempCANtransmitData.isStandardFrame = true;
					tempCANtransmitData.dataArray[0] = OBC_HANDSHAKE_REQUEST_CODE;
					tempCANtransmitData.dataArray[1] = 0;
					tempCANtransmitData.dataArray[2] = 1;
					sendCANFrame(VHCL_CANBUS2, &tempCANtransmitData);

					vcuStateData->OBC_interfaceData.handshakeInitiatedFlag = true;
					handshakeInitTimestamp = timestamp;
					return OBC_HANDSHAKE_INITIATED;
				}
				else
				{
					tempCANtransmitData.can_id = OBC_VCU_RESPONSE_CANID;
					tempCANtransmitData.can_dataLength = 8;
					tempCANtransmitData.isStandardFrame = true;
					tempCANtransmitData.dataArray[0] = OBC_HANDSHAKE_REQUEST_CODE;
					if (vcuStateData->motorRPM > MOTOR_RPM_THRESHOLD_FOR_SWITCHING_TO_AUTONOMOUS)
					{
						tempCANtransmitData.dataArray[1] = MOTOR_RPM_ABOVE_THRESHOLD ;
					}
					else if (vcuStateData->throttleInputVolt > THROTTLE_INPUT_THRESHOLD_FOR_SWITCHING_TO_AUTONOMOUS)
					{
						tempCANtransmitData.dataArray[1] = THROTTLE_INPUT_ABOVE_THRESHOLD ;
					}
					else if (vcuStateData->FootBrakeInputFlag)
					{
						tempCANtransmitData.dataArray[1] = FOOT_BRAKE_ACTIVE ;
					}
					else if (vcuStateData->handbrakeInputFlag)
					{
						tempCANtransmitData.dataArray[1] = HAND_BRAKE_ACTIVE ;
					}
					else if (vcuStateData->emergencyStopInputFlag)
					{
						tempCANtransmitData.dataArray[1] = EMERGENCY_STOP_ACTIVE ;
					}
					tempCANtransmitData.dataArray[2] = 3;
					sendCANFrame(VHCL_CANBUS2, &tempCANtransmitData);

					vcuStateData->OBC_interfaceData.handshakeInitiatedFlag = false;
					return OBC_HANDSHAKE_INIT_REQUEST_DENIED;
				}
			}
			else if((vcuStateData->OBC_interfaceData.handshakeInitiatedFlag)
					&& (CANdataFromOBC->dataArray[0] == OBC_HANDSHAKE_REQUEST_CODE)
					&& (CANdataFromOBC->dataArray[1] == 0)
					&& (CANdataFromOBC->dataArray[2] == 2))
			{
				tempCANtransmitData.can_id = OBC_VCU_RESPONSE_CANID;
				tempCANtransmitData.can_dataLength = 8;
				tempCANtransmitData.isStandardFrame = true;
				tempCANtransmitData.dataArray[0] = OBC_HANDSHAKE_REQUEST_CODE;
				tempCANtransmitData.dataArray[1] = 0;
				tempCANtransmitData.dataArray[2] = 4;
				sendCANFrame(VHCL_CANBUS2, &tempCANtransmitData);

				manualToAutonomousTransition(vcuStateData, timestamp);

				return OBC_HANDSHAKE_SUCCESSFUL;
			}
		}
	}
	else if (vcuStateData->VCU_MODE == AUTONOMOUS_MODE)
	{
		if (CANdataFromOBC->can_id == OBC_DRIVE_COMMAND_CANID)
		{
			lastOBCdriveCommandTimestamp = timestamp;
			commandMsgMissedCounter = 0;

			updateOBCdriveCommand(vcuStateData, CANdataFromOBC);
			return OBC_DRIVE_COMMAND_DATA_RECEIVED;
		}
		else if(CANdataFromOBC->can_id == OBC_REQUEST_CANID)
		{
			if (statusCheckMsgSent
				&& (CANdataFromOBC->dataArray[0] == OBC_HANDSHAKE_REQUEST_CODE)
				&& (CANdataFromOBC->dataArray[1] == 0)
				&& (CANdataFromOBC->dataArray[2] == 5))
			{
				statusCheckMsgSent = false;
				lastStatusCheckMsgReceiveTimestamp = timestamp;
				statusCheckMsgMissedCounter = 0;

				return OBC_STATUS_CHECK_RESPONSE_RECEIVED;
			}
			else if ((CANdataFromOBC->dataArray[0] == OBC_HANDSHAKE_REQUEST_CODE)
				&& (CANdataFromOBC->dataArray[1] == 1)
				&& (CANdataFromOBC->dataArray[2] == 0))
			{
				tempCANtransmitData.can_id = OBC_VCU_RESPONSE_CANID;
				tempCANtransmitData.can_dataLength = 8;
				tempCANtransmitData.isStandardFrame = true;
				tempCANtransmitData.dataArray[0] = OBC_HANDSHAKE_REQUEST_CODE;
				tempCANtransmitData.dataArray[1] = ABORTED_BY_USER;
				tempCANtransmitData.dataArray[2] = 3;
				sendCANFrame(VHCL_CANBUS2, &tempCANtransmitData);

				lastAutonomousModeExitTimestamp = timestamp;
				VCUmodeChange(MANUAL_MODE, timestamp);

				return OBC_AUTOMODE_TERMINATED_BY_USER;
			}
		}
	}

	return OBC_NO_APPROPRIATE_MSG;
}

OBCinterfaceStatus_t OBCwatchdog(vcuSysState_t *currentVcuStateData, uint32_t timestamp)
{
	if (currentVcuStateData->VCU_MODE == MANUAL_MODE)
	{
		if (currentVcuStateData->OBC_interfaceData.handshakeInitiatedFlag)
		{
			if ((timestamp - handshakeInitTimestamp) > OBC_AUTOMODE_REQUEST_TIMEOUT_THRESHOLD)
			{
				currentVcuStateData->OBC_interfaceData.handshakeInitiatedFlag = false;

				tempCANtransmitData.can_id = OBC_VCU_RESPONSE_CANID;
				tempCANtransmitData.can_dataLength = 8;
				tempCANtransmitData.isStandardFrame = true;
				tempCANtransmitData.dataArray[0] = OBC_HANDSHAKE_REQUEST_CODE;
				tempCANtransmitData.dataArray[1] = AUTOMODE_REQUEST_TIMEOUT;
				tempCANtransmitData.dataArray[2] = 3;
				sendCANFrame(VHCL_CANBUS2, &tempCANtransmitData);

				return OBC_HANDSHAKE_TIMEOUT;
			}
		}
	}
	else if (currentVcuStateData->VCU_MODE == AUTONOMOUS_MODE)
	{
		if (statusCheckMsgSent)
		{
			if ((timestamp - lastStatusCheckMsgTransmitTimestamp) > OBC_STATUS_CHECK_LISTEN_TIMER_THRESHOLD)
			{
				statusCheckMsgSent = false;
				statusCheckMsgMissedCounter++;
			}
		}
		else
		{
			if ((timestamp - lastStatusCheckMsgReceiveTimestamp) > OBC_STATUS_CHECK_INTERVAL)
			{
				tempCANtransmitData.can_id = OBC_VCU_RESPONSE_CANID;
				tempCANtransmitData.can_dataLength = 8;
				tempCANtransmitData.isStandardFrame = true;
				tempCANtransmitData.dataArray[0] = OBC_HANDSHAKE_REQUEST_CODE;
				tempCANtransmitData.dataArray[1] = 1;
				tempCANtransmitData.dataArray[2] = 4;

				if (sendCANFrame(VHCL_CANBUS2, &tempCANtransmitData) == 1)
				{
					lastStatusCheckMsgTransmitTimestamp = timestamp;
					statusCheckMsgSent = true;

					return OBC_STATUS_CHECK_INITIATED;
				}
			}
		}

		if (((timestamp - lastOBCdriveCommandTimestamp) > OBC_WATCHDOG_TIMER_THRESHOLD)
				|| (statusCheckMsgMissedCounter > OBC_STATUS_COMM_MISSED_THRESHOLD))
		{
			lastAutonomousModeExitTimestamp = timestamp;
			VCUmodeChange(MANUAL_MODE, timestamp);

			tempCANtransmitData.can_id = OBC_VCU_RESPONSE_CANID;
			tempCANtransmitData.can_dataLength = 8;
			tempCANtransmitData.isStandardFrame = true;
			tempCANtransmitData.dataArray[0] = OBC_HANDSHAKE_REQUEST_CODE;
			tempCANtransmitData.dataArray[2] = 3;
			if ((timestamp - lastOBCdriveCommandTimestamp) > OBC_WATCHDOG_TIMER_THRESHOLD)
			{
				tempCANtransmitData.dataArray[1] = WATCHDOG_TIMER_OVERFLOWED;
				sendCANFrame(VHCL_CANBUS2, &tempCANtransmitData);

				return OBC_WATCHDOG_TIMEOUT;
			}
			else
			{
				tempCANtransmitData.dataArray[1] = STATUS_MISSED_COUNTER_OVERFLOWED;
				sendCANFrame(VHCL_CANBUS2, &tempCANtransmitData);

				return OBC_STATUS_CHECK_UNRESPONSIVE;
			}
		}

		tempCANtransmitData.can_id = OBC_VCU_RESPONSE_CANID;
		tempCANtransmitData.can_dataLength = 8;
		tempCANtransmitData.isStandardFrame = true;
		tempCANtransmitData.dataArray[0] = OBC_HANDSHAKE_REQUEST_CODE;
		tempCANtransmitData.dataArray[2] = 3;
		if (currentVcuStateData->emergencyStopInputFlag)
		{
			lastAutonomousModeExitTimestamp = timestamp;
			VCUmodeChange(MANUAL_MODE, timestamp);
			tempCANtransmitData.dataArray[1] = USER_PRESSED_EMERGENCY_STOP;
			sendCANFrame(VHCL_CANBUS2, &tempCANtransmitData);

			return OBC_USER_INTERRUPTED;
		}
		else if (currentVcuStateData->FootBrakeInputFlag)
		{
			lastAutonomousModeExitTimestamp = timestamp;
			VCUmodeChange(MANUAL_MODE, timestamp);
			tempCANtransmitData.dataArray[1] = USER_PRESSED_FOOT_BRAKE;
			sendCANFrame(VHCL_CANBUS2, &tempCANtransmitData);

			return OBC_USER_INTERRUPTED;
		}
		else if (currentVcuStateData->handbrakeInputFlag)
		{
			lastAutonomousModeExitTimestamp = timestamp;
			VCUmodeChange(MANUAL_MODE, timestamp);
			tempCANtransmitData.dataArray[1] = USER_ACTIVATED_HAND_BRAKE;
			sendCANFrame(VHCL_CANBUS2, &tempCANtransmitData);

			return OBC_USER_INTERRUPTED;
		}
		else if (currentVcuStateData->throttleInputVolt > THROTTLE_PRESS_INTERRUPT_THRESHOLD)
		{
			lastAutonomousModeExitTimestamp = timestamp;
			VCUmodeChange(MANUAL_MODE, timestamp);
			tempCANtransmitData.dataArray[1] = USER_PRESSED_THROTTLE;
			sendCANFrame(VHCL_CANBUS2, &tempCANtransmitData);

			return OBC_USER_INTERRUPTED;
		}
		else if ((vcuStateData_atAutonomousEntry.driveSwitchInputFlag != currentVcuStateData->driveSwitchInputFlag)
				|| (vcuStateData_atAutonomousEntry.reverseSwitchInputFlag != currentVcuStateData->reverseSwitchInputFlag))
		{
			lastAutonomousModeExitTimestamp = timestamp;
			VCUmodeChange(MANUAL_MODE, timestamp);
			tempCANtransmitData.dataArray[1] = USER_CHANGED_FNR_STATE;
			sendCANFrame(VHCL_CANBUS2, &tempCANtransmitData);

			return OBC_USER_INTERRUPTED;
		}
	}

	return OBC_NO_EVENT;
}

inline void updateOBCdriveCommand(vcuSysState_t *vcuStateData,
		                     genericCANframeData_t *CANdataFromOBC)
{
	uint16_t requestedRPM = 0;
	requestedRPM = CANdataFromOBC->dataArray[1];
	requestedRPM = ((requestedRPM << 8) & 0xFF00) | CANdataFromOBC->dataArray[0];

	uint8_t requestedSpeedLimitSanity = 0;
	if ((CANdataFromOBC->dataArray[2] & 0x01) == 0x01)
	{
		vcuStateData->OBC_interfaceData.OBCdriveSwitchRequestFlag = true;
		vcuStateData->OBC_interfaceData.OBCreverseSwitchRequestFlag = false;
		if (requestedRPM > HIGHEST_FORWARD_RPM_IN_AUTO_MODE)
		{
			requestedSpeedLimitSanity = 1;
			requestedRPM = 0;
		}
	}
	else if ((CANdataFromOBC->dataArray[2] & 0x02) == 0x02)
	{
		vcuStateData->OBC_interfaceData.OBCdriveSwitchRequestFlag = false;
		vcuStateData->OBC_interfaceData.OBCreverseSwitchRequestFlag = true;
		if (requestedRPM > HIGHEST_REVERSE_RPM_IN_AUTO_MODE)
		{
			requestedSpeedLimitSanity = 2;
			requestedRPM = 0;
		}
	}
	else
	{
		vcuStateData->OBC_interfaceData.OBCdriveSwitchRequestFlag = false;
		vcuStateData->OBC_interfaceData.OBCreverseSwitchRequestFlag = false;
	}

	if (requestedSpeedLimitSanity != 0)
	{
		tempCANtransmitData.can_id = OBC_VCU_RESPONSE_CANID;
		tempCANtransmitData.can_dataLength = 8;
		tempCANtransmitData.isStandardFrame = true;
		tempCANtransmitData.dataArray[0] = 0;
		tempCANtransmitData.dataArray[1] = requestedSpeedLimitSanity;
		tempCANtransmitData.dataArray[2] = 6;
		sendCANFrame(VHCL_CANBUS2, &tempCANtransmitData);
	}

	vcuStateData->OBC_interfaceData.throttleVoltage = ((requestedRPM *
														(SEVCON_THROTTLE_END_VOLTAGE - SEVCON_THROTTLE_START_VOLTAGE))
													  /SEVCON_HIGHEST_RPM_IN_FORWARD_DIRECTION)
													+
													SEVCON_THROTTLE_START_VOLTAGE ;

	vcuStateData->OBC_interfaceData.OBCfootBrakeRequestFlag = ((CANdataFromOBC->dataArray[2] & 0x04) == 0x04) ?
																true : false;

	vcuStateData->OBC_interfaceData.OBCheadlampRequestFlag = ((CANdataFromOBC->dataArray[3] & 0x01) == 0x01) ?
																true : false;

	vcuStateData->OBC_interfaceData.OBCrightIndicatorRequestFlag = ((CANdataFromOBC->dataArray[3] & 0x02) == 0x02) ?
																true : false;

	vcuStateData->OBC_interfaceData.OBCleftIndicatorRequestFlag = ((CANdataFromOBC->dataArray[3] & 0x04) == 0x04) ?
																true : false;

	vcuStateData->OBC_interfaceData.OBChornRequestFlag = ((CANdataFromOBC->dataArray[3] & 0x08) == 0x08) ?
																	true : false;

	//Prepare CAN data
	uint16_t tempThrottleScaledValue = (uint16_t)(vcuStateData->OBC_interfaceData.throttleVoltage *
													  SEVCON_THROTTLE_VOLTAGE_SCALING_FACTOR);
	OBC_DriveCommandCANdata.dataArray[0] = tempThrottleScaledValue & 0x00FF;
	OBC_DriveCommandCANdata.dataArray[1] = (tempThrottleScaledValue & 0xFF00) >> 8;

	uint8_t tempPTinputCANdata1 = 0;
	if (vcuStateData->OBC_interfaceData.OBCdriveSwitchRequestFlag)
	{
		tempPTinputCANdata1 |= 0x01;
	}
	else if (vcuStateData->OBC_interfaceData.OBCreverseSwitchRequestFlag)
	{
		tempPTinputCANdata1 |= 0x02;
	}
	else
	{
		tempPTinputCANdata1 &= 0xFC;
	}

	if (vcuStateData->OBC_interfaceData.OBCfootBrakeRequestFlag)
	{
		tempPTinputCANdata1 |= 0x04;
	}

	OBC_DriveCommandCANdata.dataArray[2] = tempPTinputCANdata1;
}


inline void manualToAutonomousTransition(vcuSysState_t *currentVcuStateData, uint32_t timestamp)
{
	lastOBCdriveCommandTimestamp = timestamp;
	lastStatusCheckMsgReceiveTimestamp = timestamp;
	lastStatusCheckMsgTransmitTimestamp = timestamp;
	statusCheckMsgMissedCounter = 0;
	commandMsgMissedCounter = 0;
	statusCheckMsgSent = false;

	VCUmodeChange(AUTONOMOUS_MODE, timestamp);

	vcuStateData_atAutonomousEntry.driveSwitchInputFlag = currentVcuStateData->FootBrakeInputFlag;
	vcuStateData_atAutonomousEntry.reverseSwitchInputFlag = currentVcuStateData->handbrakeInputFlag;

	//CAN data
	uint16_t tempThrottleScaledValue = (uint16_t)(currentVcuStateData->OBC_interfaceData.throttleVoltage *
														  SEVCON_THROTTLE_VOLTAGE_SCALING_FACTOR);

	OBC_DriveCommandCANdata.dataArray[0] = tempThrottleScaledValue & 0x00FF;
	OBC_DriveCommandCANdata.dataArray[1] = (tempThrottleScaledValue & 0xFF00) >> 8;
	OBC_DriveCommandCANdata.dataArray[2] = 0;
}

