/*
 * LSD_DRV.h
 *
 *  Created on: Aug 22, 2023
 *      Author: kanhu
 */

#ifndef LSD_DRV_H_
#define LSD_DRV_H_


#include "Cpu.h"
#include "Debug_Print.h"

#define LSD0_PCS  3


int32_t updateLSDoutputs(uint8_t LSD_INSTANT, uint16_t outputFlags);
int32_t LSD_init(uint8_t LSD_INSTANT);
int32_t LSD_SPIcheck(uint8_t LSD_INSTANT);


#endif /* LSD_DRV_H_ */
