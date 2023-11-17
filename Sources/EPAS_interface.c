/*
 * EPAS_interface.c
 *
 *  Created on: May 24, 2023
 *      Author: kanhu
 */

#include "EPAS_interface.h"

genericCANframeData_t EPAS_commandMsg;

EPAS_dataStruct EPAS_data;

volatile bool isEPAS_controlEnabled = false, isExitMsgPending = false;

volatile uint32_t lastOBCcommandSentTimestamp_ms;

int16_t previousOBCTargetAngle;

void EPAS_targetAngleControl(int16_t EPAStargetAngle);
uint32_t transmitEPAScommand(genericCANframeData_t *EPAS_command, uint32_t timestamp);

void EPAS_init()
{
	EPAS_commandMsg.can_id = EPAS_COMMAND_MSG1;
	EPAS_commandMsg.can_dataLength = 8;
	EPAS_commandMsg.isStandardFrame = true;
	for (uint8_t index = 0; index < 8; index++)
	{
		EPAS_commandMsg.dataArray[index] = 0;
	}

	#if defined(OE_EPAS)
	uint16_t maxAngularVelocity = EPAS_MAX_ANG_VELOCITY;
	EPAS_commandMsg.dataArray[3] = (maxAngularVelocity >> 8) & 0x00FF;
	EPAS_commandMsg.dataArray[4] = maxAngularVelocity & 0x00FF;
	#endif
}


void EPAScontrolEntry(uint32_t timestamp)
{
	isEPAS_controlEnabled = true;

	EPAS_commandMsg.dataArray[0] = 1;
	EPAS_targetAngleControl(EPAS_data.EPAS_angle);
}

void EPAScontrolExit(uint32_t timestamp)
{
	isEPAS_controlEnabled = false;
	EPAS_commandMsg.dataArray[0] = 0;
	if (transmitEPAScommand(&EPAS_commandMsg, timestamp) == 1)
	{
		isExitMsgPending = false;
	}
	else
	{
		isExitMsgPending = true;
	}
}


void EPAS_interface(genericCANframeData_t *EPAS_interfacesMsg, uint32_t timestamp)
{
	if (EPAS_interfacesMsg->can_id == EPAS_STATUS_MSG1)
	{
		#if defined(OE_EPAS)
		int16_t tempEPASAngle = EPAS_interfacesMsg->dataArray[1];
		tempEPASAngle =	((tempEPASAngle <<8) & 0xFF00);
		tempEPASAngle |= EPAS_interfacesMsg->dataArray[2];

		EPAS_data.EPAS_angle = tempEPASAngle;

		#endif
	}
	else if (EPAS_interfacesMsg->can_id == EPAS_COMMAND_MSG1)
	{
		if (isEPAS_controlEnabled)
		{

			#if defined(OE_EPAS)
				EPAS_commandMsg.dataArray[0] = EPAS_interfacesMsg->dataArray[0] & 0x01 ;

				int16_t tempOBCTargetAngle = EPAS_interfacesMsg->dataArray[1];
				tempOBCTargetAngle =	((tempOBCTargetAngle <<8) & 0xFF00);
				tempOBCTargetAngle |= EPAS_interfacesMsg->dataArray[2];

				EPAS_targetAngleControl(tempOBCTargetAngle);

				previousOBCTargetAngle = tempOBCTargetAngle;

				transmitEPAScommand(&EPAS_commandMsg, timestamp);


			#elif defined(RANE_EPAS)
				transmitEPAScommand(EPAS_interfacesMsg, timestamp);
			#endif

		}
	}
}

void EPAS_targetAngleControl(int16_t EPAStargetAngle)
{
	int16_t angleDiff = EPAStargetAngle - EPAS_data.EPAS_angle;

	int16_t tempActualTargetAngle = 0;
	if (angleDiff > (int16_t)EPAS_MAX_ANG_INCREMENT)
	{
		tempActualTargetAngle = EPAS_data.EPAS_angle + (int16_t)EPAS_MAX_ANG_INCREMENT;
	}
	else if (angleDiff < (int16_t)-EPAS_MAX_ANG_INCREMENT)
	{
		tempActualTargetAngle = EPAS_data.EPAS_angle - (int16_t)EPAS_MAX_ANG_INCREMENT;
	}
	else if ((angleDiff >= -2) && (angleDiff <= 2))
	{
		tempActualTargetAngle = EPAStargetAngle; //EPAS_data.EPAS_angle
	}
	else
	{
		tempActualTargetAngle = EPAStargetAngle;
	}

	//Sanity check for target angle to stay within max EPAS limits
	if (tempActualTargetAngle > (int16_t)EPAS_MAX_ANGLE_LEFT)
	{
		tempActualTargetAngle = EPAS_MAX_ANGLE_LEFT;
	}
	else if (tempActualTargetAngle < (int16_t)EPAS_MAX_ANGLE_RIGHT)
	{
		tempActualTargetAngle = EPAS_MAX_ANGLE_RIGHT;
	}

	EPAS_commandMsg.dataArray[1] = (tempActualTargetAngle >> 8) & 0x00FF;
	EPAS_commandMsg.dataArray[2] = tempActualTargetAngle & 0x00FF;

//	uint16_t maxAngularVelocity = EPAS_MAX_ANG_VELOCITY;
//	EPAS_commandMsg.dataArray[3] = (maxAngularVelocity >> 8) & 0x00FF;
//	EPAS_commandMsg.dataArray[4] = maxAngularVelocity & 0x00FF;

	EPAS_commandMsg.dataArray[5] = 0;
	EPAS_commandMsg.dataArray[6] = 0;
	EPAS_commandMsg.dataArray[7] = 0;

}

uint32_t transmitEPAScommand(genericCANframeData_t *EPAS_command, uint32_t timestamp)
{
	if ((timestamp - lastOBCcommandSentTimestamp_ms) > 20)
		{
			if (sendCANFrame(VHCL_CANBUS1, EPAS_command) == 1)
				{
					lastOBCcommandSentTimestamp_ms = timestamp;
					return 1;
				}
			else
			{
				return 0;
			}
		}
	else
	{
		return 0;
	}

}

void EPAS_watchdog(uint32_t timestamp)
{
	if (isExitMsgPending)
	{
		EPAScontrolExit(timestamp);
	}

	if (isEPAS_controlEnabled && ((timestamp - lastOBCcommandSentTimestamp_ms) > 20))
	{
		EPAS_commandMsg.dataArray[0] = 1;
		EPAS_targetAngleControl(previousOBCTargetAngle);
		transmitEPAScommand(&EPAS_commandMsg, timestamp);
	}
	else if ((timestamp - lastOBCcommandSentTimestamp_ms) > 100)
	{
		EPAS_commandMsg.dataArray[0] = 0;
		transmitEPAScommand(&EPAS_commandMsg, timestamp);
	}

}
