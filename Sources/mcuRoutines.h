/*
 * mcuRoutines.h
 *
 *  Created on: Jan 2, 2023
 *      Author: kanhu
 */

#ifndef MCUROUTINES_H_
#define MCUROUTINES_H_

#include "Cpu.h"
#include "vcuSys.h"

#include "Debug_Print.h"



void boardInit();

void throttleADCcallbackFunc(const adc_callback_info_t * const callbackInfo, void * userData);

void MCAN1_CallbackFunc(uint8_t instance, mcan_event_type_t eventType,
        uint32_t buffIdx, mcan_state_t *mcanState);

void FlexCANC_CallbackFunc(uint8_t instance, flexcan_event_type_t eventType,
        uint32_t buffIdx, flexcan_state_t *flexcanState);

void VCUsysVarInit();



#endif /* MCUROUTINES_H_ */
