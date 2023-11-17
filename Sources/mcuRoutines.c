/*
 * mcuRoutines.c
 *
 *  Created on: Jan 2, 2023
 *      Author: kanhu
 */
#include "mcuRoutines.h"

extern bool debugFlag;

uint32_t VHCL_CANBUS1_receivedMsgs= 0;
flexcan_data_info_t FlexCAN_genericTransmitData, FlexCANCtransmitMsgInfo, FlexCANCreceiveMsgInfo1, FlexCANCreceiveMsgInfo2;
flexCANreceiveBuffer_t flexCANC_receiveBufferArray[VHCL_CAN1_NUMBER_OF_RECEIVE_BUFFERS] ;

uint32_t VHCL_CANBUS2_receivedMsgs = 0;
mcan_data_info_t MCAN_genericTransmitData, MCAN1transmitMsgInfo, MCAN1receiveMsgInfo1, MCAN1receiveMsgInfo2;
MCANreceiveBuffer_t MCAN1_receiveBufferArray[VHCL_CAN2_NUMBER_OF_RECEIVE_BUFFERS] ;

extern vcuSysState_t vcuStateData;
extern genericCANframeData_t manualDriveCommandCANdata, OBC_DriveCommandCANdata;

void boardInit()
{
	//SystemInit();
	//SystemCoreClockUpdate();
	CLOCK_DRV_Init(&clockMan1_InitConfig0);
	PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);


	#if defined(FNR_ACTIVE_HIGH)
		PINS_DRV_SetPullSel(PORTG,
				DriveSwitchInputPin_PIN, PORT_INTERNAL_PULL_DOWN_ENABLED); //DriveSwitchInputPin_PORT
		PINS_DRV_SetPullSel(PORTG,
				ReverseSwitchInputPin_PIN, PORT_INTERNAL_PULL_DOWN_ENABLED);  //ReverseSwitchInputPin_PORT

	#elif defined(FNR_ACTIVE_LOW)
		PINS_DRV_SetPullSel(PORTG,
						DriveSwitchInputPin_PIN, PORT_INTERNAL_PULL_UP_ENABLED); //DriveSwitchInputPin_PORT
		PINS_DRV_SetPullSel(PORTG,
						ReverseSwitchInputPin_PIN, PORT_INTERNAL_PULL_UP_ENABLED);  //ReverseSwitchInputPin_PORT

	#endif

	if ((UART_Init(&debugUARTC_instance, &debugUARTC_Config0) == STATUS_SUCCESS)
			&& (debugFlag))
	{
		printf("Boson-VCU MPC5775B.\n");
	}
	else
	{
		while(1);
	}

	//VCU system variable initialization
	VCUsysVarInit();

	//VCU Peripherals
	ADC_Init(&ADC_THROTTLE_instance, &ADC_Throttle_InitConfig0);
	ADC_StartGroupConversion(&ADC_THROTTLE_instance, 0U);

	//FlexCAN initialization
	/* *************************FlexCAN initialisation********************************** */
	FLEXCAN_DRV_Init(INST_FLEXCANC, &FlexCANC_State, &FlexCANC_InitConfig0);
	FLEXCAN_DRV_InstallEventCallback(INST_FLEXCANC, FlexCANC_CallbackFunc, NULL);

	FLEXCAN_DRV_SetRxMbGlobalMask(INST_FLEXCANC, FLEXCAN_MSG_ID_STD, 0x00000000); //To receive all CAN IDs including extended type
	//FLEXCAN_DRV_SetRxIndividualMask(INST_FLEXCANC, FLEXCAN_MSG_ID_STD, 2, 0x00000000);
	//FLEXCAN_DRV_SetRxMaskType(INST_FLEXCANC, FLEXCAN_RX_MASK_INDIVIDUAL);

	FLEXCAN_DRV_ConfigTxMb(INST_FLEXCANC, 0, &FlexCANCtransmitMsgInfo, 0x69);
	FLEXCAN_DRV_ConfigTxMb(INST_FLEXCANC, 1, &FlexCANCtransmitMsgInfo, 0x69);

	FLEXCAN_DRV_ConfigRxMb(INST_FLEXCANC, 2, &FlexCANCreceiveMsgInfo1, 0x210);
	FLEXCAN_DRV_ConfigRxMb(INST_FLEXCANC, 3, &FlexCANCreceiveMsgInfo1, 0x210);
	FLEXCAN_DRV_ConfigRxMb(INST_FLEXCANC, 4, &FlexCANCreceiveMsgInfo1, 0x210);
	FLEXCAN_DRV_ConfigRxMb(INST_FLEXCANC, 5, &FlexCANCreceiveMsgInfo2, 0x210);
	FLEXCAN_DRV_ConfigRxMb(INST_FLEXCANC, 6, &FlexCANCreceiveMsgInfo2, 0x210);

	//FLEXCAN_DRV_Init(INST_FLEXCANB, &FlexCANB_State, &FlexCANB_InitConfig0);
	//FLEXCAN_DRV_Init(INST_FLEXCANA, &FlexCANC_State, &FlexCANC_InitConfig0);
	//FLEXCAN_DRV_Init(INST_FLEXCAND, &FlexCAND_State, &FlexCAND_InitConfig0);

	FLEXCAN_DRV_Receive(INST_FLEXCANC, 2, &flexCANC_receiveBufferArray[0].flexCANdata);
	FLEXCAN_DRV_Receive(INST_FLEXCANC, 3, &flexCANC_receiveBufferArray[0].flexCANdata);
	FLEXCAN_DRV_Receive(INST_FLEXCANC, 4, &flexCANC_receiveBufferArray[0].flexCANdata);
	FLEXCAN_DRV_Receive(INST_FLEXCANC, 5, &flexCANC_receiveBufferArray[0].flexCANdata);
	FLEXCAN_DRV_Receive(INST_FLEXCANC, 6, &flexCANC_receiveBufferArray[0].flexCANdata);

	/* *************************MCAN initialisation********************************** */
	//MCAN_DRV_Init(INST_MCAN0, &MCAN0_State, &MCAN0_mcan_user_config_t0);

	MCAN_DRV_Init(INST_MCAN1, &MCAN1_State, &MCAN1_mcan_user_config_t0);
	MCAN_DRV_InstallEventCallback(INST_MCAN1, MCAN1_CallbackFunc, NULL);

	MCAN_DRV_ConfigTxMb(INST_MCAN1, 0, &MCAN1transmitMsgInfo, 0X69);
	MCAN_DRV_ConfigTxMb(INST_MCAN1, 1, &MCAN1transmitMsgInfo, 0X69);

	MCAN_DRV_ConfigRxMb(INST_MCAN1, 2, &MCAN1receiveMsgInfo1, OBC_REQUEST_CANID);
	MCAN_DRV_ConfigRxMb(INST_MCAN1, 3, &MCAN1receiveMsgInfo1, OBC_DRIVE_COMMAND_CANID);
	MCAN_DRV_ConfigRxMb(INST_MCAN1, 4, &MCAN1receiveMsgInfo1, OBC_STEERING_COMMAND_CANID);
	MCAN_DRV_ConfigRxMb(INST_MCAN1, 5, &MCAN1receiveMsgInfo2, 0x69);
	MCAN_DRV_ConfigRxMb(INST_MCAN1, 6, &MCAN1receiveMsgInfo2, 0x69);

	MCAN_DRV_Receive(INST_MCAN1, 2, &MCAN1_receiveBufferArray[0].MCANdata);
	MCAN_DRV_Receive(INST_MCAN1, 3, &MCAN1_receiveBufferArray[0].MCANdata);
	MCAN_DRV_Receive(INST_MCAN1, 4, &MCAN1_receiveBufferArray[0].MCANdata);
	MCAN_DRV_Receive(INST_MCAN1, 5, &MCAN1_receiveBufferArray[0].MCANdata);
	MCAN_DRV_Receive(INST_MCAN1, 6, &MCAN1_receiveBufferArray[0].MCANdata);

	//HSD SPI Init
	DSPI_MasterInit(INST_HSD_SPIB, &HSD_SPIBState, &HSD_SPIB_MasterInitConfig0);
	DSPI_MasterSetDelay(INST_HSD_SPIB, 2, 2, 2);

	//LSD SPI Init
	DSPI_MasterInit(INST_LSD_SPIC, &LSD_SPICState, &LSD_SPIC_MasterInitConfig0);
	DSPI_MasterSetDelay(INST_LSD_SPIC, 2, 2, 2);

	//LSD PWM initialization
	PWM_PAL0Configs.pwmChannels->period = 25000; //100000: 250Hz, 250000: 100Hz
	PWM_PAL0Configs.pwmChannels->duty = 12500;   //

	PWM_Init(&PWM_PAL0Instance,&PWM_PAL0Configs);
	//PWM_UpdatePeriod(&PWM_PAL0Instance, 3, 100000);
	//PWM_UpdateDuty(&PWM_PAL0Instance, 3, 90000);

}


void throttleADCcallbackFunc(const adc_callback_info_t * const callbackInfo, void * userData)
  {
  	(void) userData;

  	//groupConvDone = true;
  	//resultLastOffset = callbackInfo->resultBufferTail;
  }


void MCAN1_CallbackFunc(uint8_t instance, mcan_event_type_t eventType,
        uint32_t buffIdx, mcan_state_t *mcanState)
{

	if(MCAN_EVENT_RX_COMPLETE == eventType)
	{
		if(VHCL_CANBUS2_receivedMsgs < VHCL_CAN2_NUMBER_OF_RECEIVE_BUFFERS)
		{
			for(uint8_t index = 0; index <VHCL_CAN2_NUMBER_OF_RECEIVE_BUFFERS; index++)
			{
				if(!MCAN1_receiveBufferArray[index].isFull)
				{
					MCAN1_receiveBufferArray[index].timestamp = OSIF_GetMilliseconds();
					MCAN_DRV_Receive(instance, buffIdx, &MCAN1_receiveBufferArray[index].MCANdata);
					MCAN1_receiveBufferArray[index].VCU_CAN_INST = VHCL_CANBUS2;
					MCAN1_receiveBufferArray[index].isFull = true;
					VHCL_CANBUS2_receivedMsgs++;
					return;
				}
			}
		}
		else
		{
			MCAN_DRV_Receive(instance, buffIdx, &MCAN1_receiveBufferArray[0].MCANdata);
			MCAN1_receiveBufferArray[0].VCU_CAN_INST = VHCL_CANBUS2;
			MCAN1_receiveBufferArray[0].isFull = true;
		}
	}

}

void FlexCANC_CallbackFunc(uint8_t instance, flexcan_event_type_t eventType,
        uint32_t buffIdx, flexcan_state_t *flexcanState)
{
	if(FLEXCAN_EVENT_RX_COMPLETE == eventType)
	{
		if(VHCL_CANBUS1_receivedMsgs < VHCL_CAN1_NUMBER_OF_RECEIVE_BUFFERS)
		{
			for(uint8_t index = 0; index <VHCL_CAN1_NUMBER_OF_RECEIVE_BUFFERS; index++)
			{
				if (!flexCANC_receiveBufferArray[index].isFull)
				{
					flexCANC_receiveBufferArray[index].timestamp = OSIF_GetMilliseconds();
					FLEXCAN_DRV_Receive(instance, buffIdx, &flexCANC_receiveBufferArray[index].flexCANdata);
					flexCANC_receiveBufferArray[index].VCU_CAN_INST = VHCL_CANBUS1;
					flexCANC_receiveBufferArray[index].isFull = true;
					VHCL_CANBUS1_receivedMsgs++;
					return;
				}

			}

		}
		else
		{
			FLEXCAN_DRV_Receive(instance, buffIdx, &flexCANC_receiveBufferArray[0].flexCANdata);
			flexCANC_receiveBufferArray[0].VCU_CAN_INST = VHCL_CANBUS1;
			flexCANC_receiveBufferArray[0].isFull = true;
		}
	}
}



void VCUsysVarInit()
{
	//CAN variables
	//Changing bitrate configs and code generators overwrites to wrong values
	FlexCANC_InitConfig0.bitrate.propSeg = 7;
	FlexCANC_InitConfig0.bitrate.phaseSeg1 = 4;
	FlexCANC_InitConfig0.bitrate.phaseSeg2 = 1;
	FlexCANC_InitConfig0.bitrate.preDivider = 9;
	FlexCANC_InitConfig0.bitrate.rJumpwidth = 1;

	VHCL_CANBUS1_receivedMsgs = 0;
	for (uint8_t index = 0; index< VHCL_CAN1_NUMBER_OF_RECEIVE_BUFFERS; index++)
	{
		flexCANC_receiveBufferArray[index].isFull = false;
	}

	FlexCAN_genericTransmitData.is_remote = false;

	FlexCANCtransmitMsgInfo.data_length = 8;
	FlexCANCtransmitMsgInfo.is_remote = false;
	FlexCANCtransmitMsgInfo.msg_id_type = FLEXCAN_MSG_ID_STD;

	FlexCANCreceiveMsgInfo1.data_length = 8;
	FlexCANCreceiveMsgInfo1.is_remote = false;
	FlexCANCreceiveMsgInfo1.msg_id_type = FLEXCAN_MSG_ID_STD;

	FlexCANCreceiveMsgInfo1.data_length = 8;
	FlexCANCreceiveMsgInfo1.is_remote = false;
	FlexCANCreceiveMsgInfo1.msg_id_type = FLEXCAN_MSG_ID_EXT;



	VHCL_CANBUS2_receivedMsgs = 0;
	for (uint8_t index = 0; index< VHCL_CAN2_NUMBER_OF_RECEIVE_BUFFERS; index++)
	{
		MCAN1_receiveBufferArray[index].isFull = false;
	}

	MCAN_genericTransmitData.is_remote = false;

	MCAN1transmitMsgInfo.data_length = 8;
	MCAN1transmitMsgInfo.enable_brs = false;
	MCAN1transmitMsgInfo.fd_enable = false;
	MCAN1transmitMsgInfo.fd_padding = false;
	MCAN1transmitMsgInfo.is_remote = false;
	MCAN1transmitMsgInfo.msg_id_type = MCAN_MSG_ID_STD;

	MCAN1receiveMsgInfo1.data_length = 8;
	MCAN1receiveMsgInfo1.enable_brs = false;
	MCAN1receiveMsgInfo1.fd_enable = false;
	MCAN1receiveMsgInfo1.fd_padding = false;
	MCAN1receiveMsgInfo1.is_remote = false;
	MCAN1receiveMsgInfo1.msg_id_type = MCAN_MSG_ID_STD;

	MCAN1receiveMsgInfo2.data_length = 8;
	MCAN1receiveMsgInfo2.enable_brs = false;
	MCAN1receiveMsgInfo2.fd_enable = false;
	MCAN1receiveMsgInfo2.fd_padding = false;
	MCAN1receiveMsgInfo2.is_remote = false;
	MCAN1receiveMsgInfo2.msg_id_type = MCAN_MSG_ID_EXT;


	//Application variables
	vcuStateData.VCU_MODE = MANUAL_MODE;
	vcuStateData.OBC_interfaceData.handshakeInitiatedFlag = false;
	vcuStateData.motorRPM = 0;

	manualDriveCommandCANdata.can_id = VCU_DRIVE_COMMAND_CANID;
	manualDriveCommandCANdata.can_dataLength = 8;
	manualDriveCommandCANdata.isStandardFrame = true;
	manualDriveCommandCANdata.dataArray[0] = 0;
	manualDriveCommandCANdata.dataArray[1] = 0;
	manualDriveCommandCANdata.dataArray[2] = 0;

	OBC_DriveCommandCANdata.can_id = VCU_DRIVE_COMMAND_CANID;
	OBC_DriveCommandCANdata.can_dataLength = 8;
	OBC_DriveCommandCANdata.isStandardFrame = true;
	OBC_DriveCommandCANdata.dataArray[0] = 0;
	OBC_DriveCommandCANdata.dataArray[1] = 0;
	OBC_DriveCommandCANdata.dataArray[2] = 0;



}

