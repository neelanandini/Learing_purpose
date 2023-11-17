/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
/* ###################################################################
**     Filename    : main.c
**     Processor   : MPC5777C
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.00
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */


/* Including necessary module. Cpu.h contains other modules needed for compiling.*/
#include "Cpu.h"
#include "Debug_Print.h"

#include "vcuSys.h"
#include "mcuRoutines.h"
#include "OBC_CANinterface.h"
#include "EPAS_interface.h"

extern vcuSysState_t vcuStateData;
extern uint32_t VHCL_CANBUS1_receivedMsgs, VHCL_CANBUS2_receivedMsgs;
extern flexCANreceiveBuffer_t flexCANC_receiveBufferArray[];
extern MCANreceiveBuffer_t MCAN1_receiveBufferArray[];
genericCANframeData_t genericCANframeDataReceiveBuffer, genericCANframeDataTransmitBuffer;

uint32_t currentTimestamp, lastTimestamp100ms, lastTimestamp50ms, lastTimestamp20ms;

uint16_t previousLSDoutputflag = 0;

uint8_t updateVcuHsdOutputFlags(vcuSysState_t *vcuStateData);
uint16_t updateVcuLsdOutputFlags(vcuSysState_t *vcuStateData);

  volatile int exit_code = 0;


/* User includes (#include below this line is not maintained by Processor Expert) */

/*! 
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
*/
int main(void)
{
  /* Write your local variable definition here */



  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  /* For example: for(;;) { } */
    boardInit();
    LSD_init(0);
    HSD_init(0);
    HSD_init(1);
    EPAS_init();
    while(1)
    {
    	//OSIF_TimeDelay(1);
    	currentTimestamp = OSIF_GetMilliseconds();
    	if((currentTimestamp - lastTimestamp50ms) >= 50)
		{
			lastTimestamp50ms = currentTimestamp;
			readUserInputs();

			updateHSDoutputs(0, updateVcuHsdOutputFlags(&vcuStateData));
			updateHSDoutputs(1, updateVcuHsdOutputFlags(&vcuStateData));

		}

    	if((currentTimestamp - lastTimestamp100ms) >= 100)
    	{
    		lastTimestamp100ms = currentTimestamp;
    		sendDriveCommand();

    		//LSD switching control
    		uint16_t LSDoutputFlag = updateVcuLsdOutputFlags(&vcuStateData);
			if (previousLSDoutputflag != LSDoutputFlag)
			{
				updateLSDoutputs(0, LSDoutputFlag);
				previousLSDoutputflag = LSDoutputFlag;
			}
			//updateLSDoutputs(0, LSDoutputFlag);

    	}

    	while(VHCL_CANBUS1_receivedMsgs > 0)
    	{
    		for(uint8_t index = 0; index <VHCL_CAN1_NUMBER_OF_RECEIVE_BUFFERS; index++)
			{
				if (flexCANC_receiveBufferArray[index].isFull)
				{
					genericCANframeDataReceiveBuffer.can_id = flexCANC_receiveBufferArray[index].flexCANdata.msgId;
					genericCANframeDataReceiveBuffer.isStandardFrame = ((genericCANframeDataReceiveBuffer.can_id & 0xFFFFF800) == 0x00000000)
																		? true: false;
					genericCANframeDataReceiveBuffer.can_dataLength = flexCANC_receiveBufferArray[index].flexCANdata.dataLen;
					for (uint8_t dataIndex = 0; dataIndex < genericCANframeDataReceiveBuffer.can_dataLength; dataIndex++)
					{
						genericCANframeDataReceiveBuffer.dataArray[dataIndex] = flexCANC_receiveBufferArray[index].flexCANdata.data[dataIndex];
					}
					flexCANC_receiveBufferArray[index].isFull = false;
					VHCL_CANBUS1_receivedMsgs--;

					sendCANFrame(VHCL_CANBUS2, &genericCANframeDataReceiveBuffer);

					switch (genericCANframeDataReceiveBuffer.can_id)
					{
					case MOTOR_RPM_CANID:
						monitorPowertrain(&vcuStateData, &genericCANframeDataReceiveBuffer);
						break;

					case EPAS_STATUS_MSG1:
						EPAS_interface(&genericCANframeDataReceiveBuffer, currentTimestamp);

					default:
						break;
					}
				}

				if (VHCL_CANBUS1_receivedMsgs == 0)
				{
					break;
				}
			}
    	}
    	while(VHCL_CANBUS2_receivedMsgs > 0)
		{
    		for(uint8_t index = 0; index <VHCL_CAN2_NUMBER_OF_RECEIVE_BUFFERS; index++)
    		{
    			if (MCAN1_receiveBufferArray[index].isFull)
				{
					genericCANframeDataReceiveBuffer.can_id = MCAN1_receiveBufferArray[index].MCANdata.msgId ;
					genericCANframeDataReceiveBuffer.isStandardFrame = ((genericCANframeDataReceiveBuffer.can_id & 0xFFFFF800) == 0x00000000)
																		? true: false;
					genericCANframeDataReceiveBuffer.can_dataLength = MCAN1_receiveBufferArray[index].MCANdata.dataLen;
					for (uint8_t dataIndex = 0; dataIndex < genericCANframeDataReceiveBuffer.can_dataLength; dataIndex++)
					{
						genericCANframeDataReceiveBuffer.dataArray[dataIndex] = MCAN1_receiveBufferArray[index].MCANdata.data[dataIndex];
					}
					MCAN1_receiveBufferArray[index].isFull = false;
					VHCL_CANBUS2_receivedMsgs--;

					switch (genericCANframeDataReceiveBuffer.can_id)
					{
					case OBC_REQUEST_CANID:
						OBC_CANinterface(&genericCANframeDataReceiveBuffer, &vcuStateData,
								currentTimestamp);
						break;

					case OBC_STEERING_COMMAND_CANID:
						if (vcuStateData.VCU_MODE == AUTONOMOUS_MODE)
						{
							EPAS_interface(&genericCANframeDataReceiveBuffer, currentTimestamp);
						}
						break;

					case OBC_DRIVE_COMMAND_CANID:
						OBC_CANinterface(&genericCANframeDataReceiveBuffer, &vcuStateData,
										currentTimestamp);
						break;

					default :
						break;
					}
				}

    			if (VHCL_CANBUS2_receivedMsgs == 0)
				{
					break;
				}
    		}
		}

    	volatile static vcuMode_t lastVCUmode = MANUAL_MODE;
    	if (vcuStateData.VCU_MODE == MANUAL_MODE)
    	{
    		if (lastVCUmode == MANUAL_MODE)
    		{

    		}
    		else if (lastVCUmode == AUTONOMOUS_MODE)
    		{
    			EPAScontrolExit(currentTimestamp);
    		}

    		lastVCUmode = MANUAL_MODE;
    	}
    	else if (vcuStateData.VCU_MODE == AUTONOMOUS_MODE)
    	{
    		if (lastVCUmode == MANUAL_MODE)
			{
    			EPAScontrolEntry(currentTimestamp);
			}
    		else if (lastVCUmode == AUTONOMOUS_MODE)
    		{

    		}


    		lastVCUmode = AUTONOMOUS_MODE;
    	}
    	OBCwatchdog(&vcuStateData, currentTimestamp);
    	EPAS_watchdog(currentTimestamp);
    }

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the NXP C55 series of microcontrollers.
**
** ###################################################################
*/

uint8_t updateVcuHsdOutputFlags(vcuSysState_t *vcuStateData)
{
	static uint8_t outputVar = 0x00;
	uint32_t currentTime = OSIF_GetMilliseconds();

	if (vcuStateData->VCU_MODE == MANUAL_MODE)
	{
		outputVar &= 0xEF;	//Amber light reset
		outputVar &= 0xF3;  //Indicators reset
		outputVar &= 0xDF;  //Headlamp reset

		if (vcuStateData->FootBrakeInputFlag || vcuStateData->handbrakeInputFlag)
		{
			outputVar |= 0x01;	//Brake light
		}
		else
		{
			outputVar &= 0xFE;
		}
		if (vcuStateData->reverseSwitchInputFlag)
		{
			outputVar |= 0x02;  //Reverse light and buzzer
		}
		else
		{
			outputVar &= 0xFD;
		}
	}
	else if (vcuStateData->VCU_MODE == AUTONOMOUS_MODE)
	{
		outputVar |= 0x10;	//Amber light

		if ( vcuStateData->OBC_interfaceData.OBCfootBrakeRequestFlag
				|| vcuStateData->FootBrakeInputFlag
				|| vcuStateData->handbrakeInputFlag )
		{
			outputVar |= 0x01;	//Brake light
		}
		else
		{
			outputVar &= 0xFE;
		}

		if (vcuStateData->OBC_interfaceData.OBCreverseSwitchRequestFlag)
		{
			outputVar |= 0x02;  //Reverse light and buzzer
		}
		else
		{
			outputVar &= 0xFD;
		}

		//Indicator Action
		uint8_t tempIndicatorStatus = 0;
		if (vcuStateData->OBC_interfaceData.OBCrightIndicatorRequestFlag)
		{
			tempIndicatorStatus |= 0x04;
		}
		else
		{
			tempIndicatorStatus &= 0xFB;
		}

		if (vcuStateData->OBC_interfaceData.OBCleftIndicatorRequestFlag)
		{
			tempIndicatorStatus |= 0x08;
		}
		else
		{
			tempIndicatorStatus &= 0xF7;
		}
		//Indicator toggle action
		static uint32_t indicatorToggleTimer;
		static bool indicatorOnFlag;
		if ((tempIndicatorStatus & 0x0C) != 0x00)
		{
			if (!indicatorOnFlag)
			{
				outputVar |= (tempIndicatorStatus & 0x0C);
				indicatorOnFlag = true;
				indicatorToggleTimer = currentTime;

			}
			else
			{
				if (((outputVar & 0x0C) != 0x00)
					&& ((currentTime - indicatorToggleTimer) > 1000))
				{
					outputVar &= 0xF3;
					indicatorToggleTimer = currentTime;
				}

				if (((outputVar & 0x0C) == 0x00)
					&& (currentTime - indicatorToggleTimer) > 1000)
				{
					outputVar |= (tempIndicatorStatus & 0x0C);
					indicatorToggleTimer = currentTime;
				}
			}
		}
		else
		{
			outputVar &= 0xF3;
			indicatorOnFlag = false;
		}

		//Headlamp action
		if (vcuStateData->OBC_interfaceData.OBCheadlampRequestFlag)
		{
			outputVar |= 0x20;
		}
		else
		{
			outputVar &= 0xDF;
		}

	}

	return outputVar;
}

uint16_t updateVcuLsdOutputFlags(vcuSysState_t *vcuStateData)
{
	static uint8_t outputVar = 0x0000;

	uint32_t currentTime = OSIF_GetMilliseconds();

	//Horn action
	static uint32_t hornToggleTimer;
	static bool hornONflag;
	if (((vcuStateData->VCU_MODE == AUTONOMOUS_MODE)
		&& (vcuStateData->OBC_interfaceData.OBChornRequestFlag)))
	{
		if(!hornONflag)
		{
			outputVar |= 0x0007;
			hornONflag = true;
			hornToggleTimer = currentTime;
		}
		else
		{
			if (((outputVar & 0x0007) == 0x0007)
				&& ((currentTime - hornToggleTimer) > 1000))
			{
				outputVar &= 0xFFF8;
				hornToggleTimer = currentTime;
			}

			if (((outputVar & 0x0007) == 0x0000)
				&& (currentTime - hornToggleTimer) > 2000)
			{
				outputVar |= 0x0007;
				hornToggleTimer = currentTime;
			}

		}

	}
	else
	{
		outputVar &= 0xFFF8;
		hornONflag = false;
	}

	return outputVar;
}
