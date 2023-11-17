/*
 * LSD_DRV.c
 *
 *  Created on: Aug 22, 2023
 *      Author: kanhu
 */
#include "LSD_DRV.h"

uint8_t LSD_trnsmtMsgBuff[6], LSD_rcvMsgBuff[6]; //24bit communication and 48bit communication


int32_t updateLSDoutputs(uint8_t LSD_INSTANT, uint16_t outputFlags)
{
	status_t LSDcommStatus = STATUS_SUCCESS;
	switch (LSD_INSTANT)
	{
	case 0:
		//Update PCS
		DSPI_UpdateCS(INST_LSD_SPIC, LSD0_PCS);

		break;

	default:
		break;
	}

	//Enable load
	LSD_trnsmtMsgBuff[0] = 0x00; //ON - OFF command
	LSD_trnsmtMsgBuff[1] = (uint8_t)((outputFlags >> 8) & 0xFF);
	LSD_trnsmtMsgBuff[2] = (uint8_t)(outputFlags & 0xFF);

	//SPI transfer
	LSDcommStatus = DSPI_MasterTransferBlocking(INST_LSD_SPIC, &LSD_trnsmtMsgBuff, &LSD_rcvMsgBuff, 3, 5);
	if (LSDcommStatus != STATUS_SUCCESS)
	{
		return -1;
	}
	else
	{
		return 1;
	}

}

int32_t LSD_init(uint8_t LSD_INSTANT)
{
	status_t LSDcommStatus = STATUS_SUCCESS;
	switch (LSD_INSTANT)
	{
	case 0:
		/*RST pin reset*/
		PINS_DRV_WritePin(LSD_RSTB_PORT, LSD_RSTB_PIN, 0);
		OSIF_TimeDelay(1);
		PINS_DRV_WritePin(LSD_RSTB_PORT, LSD_RSTB_PIN, 1);
		OSIF_TimeDelay(1);

		DSPI_UpdateCS(INST_LSD_SPIC, LSD0_PCS);

		break;

	default:
		break;
	}
	//SPI integrity check
	int32_t LSDintegrityCheckStatus;
	LSDintegrityCheckStatus = LSD_SPIcheck(LSD_INSTANT);

	if (LSDintegrityCheckStatus == 1)
	{
		//OSIF_TimeDelay(1);
		//Enable open load current detection
		LSD_trnsmtMsgBuff[0] = 0x04; //Openload control command
		LSD_trnsmtMsgBuff[1] = 0x00; //0xFF to enable,0x00 to disable
		LSD_trnsmtMsgBuff[2] = 0x00; //0xFF to enable,0x00 to disable

		//SPI transfer
		LSDcommStatus = DSPI_MasterTransferBlocking(INST_LSD_SPIC, &LSD_trnsmtMsgBuff, &LSD_rcvMsgBuff, 3, 5);
		if (LSDcommStatus != STATUS_SUCCESS)
		{
			return -1;
		}


		//OSIF_TimeDelay(1);
		//Configure gloabl shutdown strategy
		LSD_trnsmtMsgBuff[0] = 0x08; //Global shutdown control command
		LSD_trnsmtMsgBuff[0] |= (0x00 | 0x00); // 0x01 for overvoltage shutdown retry enable,  0x02 for thermal shutdown retry enable
		LSD_trnsmtMsgBuff[1] = 0x00;
		LSD_trnsmtMsgBuff[2] = 0x00;

		//SPI transfer
		LSDcommStatus = DSPI_MasterTransferBlocking(INST_LSD_SPIC, &LSD_trnsmtMsgBuff, &LSD_rcvMsgBuff, 3, 5);
		if (LSDcommStatus != STATUS_SUCCESS)
		{
			return -1;
		}

		//OSIF_TimeDelay(1);
		//SFPD Control
		LSD_trnsmtMsgBuff[0] = 0x0C; //SFPD control command
		LSD_trnsmtMsgBuff[1] = 0x00; //0x00 - Short fault protect enable,  0xFF - only thermal protection
		LSD_trnsmtMsgBuff[2] = 0x00;

		//SPI transfer
		LSDcommStatus = DSPI_MasterTransferBlocking(INST_LSD_SPIC, &LSD_trnsmtMsgBuff, &LSD_rcvMsgBuff, 3, 5);
		if (LSDcommStatus != STATUS_SUCCESS)
		{
			return -1;
		}

		//OSIF_TimeDelay(1);
		//PWM Control
		LSD_trnsmtMsgBuff[0] = 0x10; //PWM control command
		LSD_trnsmtMsgBuff[1] = 0x00; //0x00 - PWM disable,  0xFF - PWM enable
		LSD_trnsmtMsgBuff[2] = 0x00;

		//SPI transfer
		LSDcommStatus = DSPI_MasterTransferBlocking(INST_LSD_SPIC, &LSD_trnsmtMsgBuff, &LSD_rcvMsgBuff, 3, 5);
		if (LSDcommStatus != STATUS_SUCCESS)
		{
			return -1;
		}

		//OSIF_TimeDelay(1);
		//PWM-SPI AND/OR Control
		LSD_trnsmtMsgBuff[0] = 0x14; //AND-OR control command
		LSD_trnsmtMsgBuff[1] = 0x00; //0x00 - AND,  0xFF - OR
		LSD_trnsmtMsgBuff[2] = 0x00;

		//SPI transfer
		LSDcommStatus = DSPI_MasterTransferBlocking(INST_LSD_SPIC, &LSD_trnsmtMsgBuff, &LSD_rcvMsgBuff, 3, 5);
		if (LSDcommStatus != STATUS_SUCCESS)
		{
			return -1;
		}

		return 1;
	}
	else
	{
		return LSDintegrityCheckStatus;
	}
}


int32_t LSD_SPIcheck(uint8_t LSD_INSTANT)
{
	status_t SPIcommStatus = STATUS_SUCCESS;
	switch (LSD_INSTANT)
	{
	case 0:
		DSPI_UpdateCS(INST_LSD_SPIC, LSD0_PCS);

		break;

	default:
		break;

	}

	//Integrity check values
	LSD_trnsmtMsgBuff[0] = 0xAA;
	LSD_trnsmtMsgBuff[1] = 0xAA;
	LSD_trnsmtMsgBuff[2] = 0xAA;

	/* software reset */
	LSD_trnsmtMsgBuff[3] = 0x18; //Reset command
	LSD_trnsmtMsgBuff[4] = 0x00;
	LSD_trnsmtMsgBuff[5] = 0x00;

	//SPI transfer

	SPIcommStatus = DSPI_MasterTransferBlocking(INST_LSD_SPIC, &LSD_trnsmtMsgBuff, &LSD_rcvMsgBuff, 6, 10);

	//Check SPI communication result
	if (SPIcommStatus != STATUS_SUCCESS)
	{
		//SPI comm. failed
		return -1;
	}
	else if ((LSD_rcvMsgBuff[3] != LSD_trnsmtMsgBuff[0])
			|| (LSD_rcvMsgBuff[4] != LSD_trnsmtMsgBuff[1])
			|| (LSD_rcvMsgBuff[5] != LSD_trnsmtMsgBuff[2]))
	{
		//SPI data integrity check failed
		return 0;
	}
	else
	{
		//SPI data integrity check passed.
		return 1;
	}
}
