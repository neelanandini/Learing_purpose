/*
 * HSD_DRV.c
 *
 *  Created on: Jan 17, 2023
 *      Author: kanhu
 */

#include "HSD_DRV.h"

uint16_t HSD_trnsmtMsgBuff[2], HSD_rcvMsgBuff[2];

void HSD_init(uint8_t HSD_INSTANT)
{
	switch (HSD_INSTANT)
	{
	case 0:
		PINS_DRV_WritePin(HSD_RSTB_PORT, HSD_RSTB_PIN, 0);
		OSIF_TimeDelay(1);
		PINS_DRV_WritePin(HSD_RSTB_PORT, HSD_RSTB_PIN, 1);
		OSIF_TimeDelay(1); //tENBL not mentioned

		HSD_trnsmtMsgBuff[0] = 0x0C18;
		HSD_trnsmtMsgBuff[1] = 0x0418;
		DSPI_UpdateCS(INST_HSD_SPIB, HSD_PCS);
		DSPI_MasterTransferBlocking(INST_HSD_SPIB, &HSD_trnsmtMsgBuff, &HSD_rcvMsgBuff, 2, 10);

		break;

	case 1:
		PINS_DRV_WritePin(HSD1_RSTB_PORT, HSD1_RSTB_PIN, 0);
		OSIF_TimeDelay(1);
		PINS_DRV_WritePin(HSD1_RSTB_PORT, HSD1_RSTB_PIN, 1);
		OSIF_TimeDelay(1); //tENBL not mentioned

		HSD_trnsmtMsgBuff[0] = 0x0C18;
		HSD_trnsmtMsgBuff[1] = 0x0418;
		DSPI_UpdateCS(INST_HSD_SPIB, HSD1_PCS);
		DSPI_MasterTransferBlocking(INST_HSD_SPIB, &HSD_trnsmtMsgBuff, &HSD_rcvMsgBuff, 2, 10);

		break;

	case 2:
		PINS_DRV_WritePin(HSD2_RSTB_PORT, HSD2_RSTB_PIN, 0);
		OSIF_TimeDelay(1);
		PINS_DRV_WritePin(HSD2_RSTB_PORT, HSD2_RSTB_PIN, 1);
		OSIF_TimeDelay(1); //tENBL not mentioned

		HSD_trnsmtMsgBuff[0] = 0x0C18;
		HSD_trnsmtMsgBuff[1] = 0x0418;
		DSPI_UpdateCS(INST_HSD_SPIB, HSD2_PCS);
		DSPI_MasterTransferBlocking(INST_HSD_SPIB, &HSD_trnsmtMsgBuff, &HSD_rcvMsgBuff, 2, 10);

		break;

	case 3:
		PINS_DRV_WritePin(HSD3_RSTB_PORT, HSD3_RSTB_PIN, 0);
		OSIF_TimeDelay(1);
		PINS_DRV_WritePin(HSD3_RSTB_PORT, HSD3_RSTB_PIN, 1);
		OSIF_TimeDelay(1); //tENBL not mentioned

		HSD_trnsmtMsgBuff[0] = 0x0C18;
		HSD_trnsmtMsgBuff[1] = 0x0418;
		DSPI_UpdateCS(INST_HSD_SPIB, HSD3_PCS);
		DSPI_MasterTransferBlocking(INST_HSD_SPIB, &HSD_trnsmtMsgBuff, &HSD_rcvMsgBuff, 2, 10);

		break;

	default:
		break;
	}
}

inline void updateHSDoutputs(uint8_t HSD_INSTANT, uint8_t outputFlags)
{
	switch (HSD_INSTANT)
	{
	case 0:
		HSD_trnsmtMsgBuff[0] = 0x8800 | (outputFlags & 0x1F);
		HSD_trnsmtMsgBuff[1] = 0x8000 | (outputFlags & 0x1F);
		DSPI_UpdateCS(INST_HSD_SPIB, HSD_PCS);
		DSPI_MasterTransferBlocking(INST_HSD_SPIB, &HSD_trnsmtMsgBuff, &HSD_rcvMsgBuff, 2, 10);
		break;

	case 1:
		HSD_trnsmtMsgBuff[0] = 0x8800 | ((outputFlags >> 5) & 0x1F);
		HSD_trnsmtMsgBuff[1] = 0x8000 | ((outputFlags >> 5) & 0x1F);
		DSPI_UpdateCS(INST_HSD_SPIB, HSD1_PCS);
		DSPI_MasterTransferBlocking(INST_HSD_SPIB, &HSD_trnsmtMsgBuff, &HSD_rcvMsgBuff, 2, 10);
		break;

	default:
		break;

	}
}

