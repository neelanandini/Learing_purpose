/*
 * HSD_DRV.h
 *
 *  Created on: Jan 17, 2023
 *      Author: kanhu
 */

#ifndef HSD_DRV_H_
#define HSD_DRV_H_

#include "Cpu.h"
#include "Debug_Print.h"


#define HSD_PCS   0
#define HSD1_PCS  2
#define HSD2_PCS  1
#define HSD3_PCS  3


void HSD_init(uint8_t HSD_INSTANT);
void updateHSDoutputs(uint8_t HSD_INSTANT, uint8_t outputFlags);


#endif /* HSD_DRV_H_ */
