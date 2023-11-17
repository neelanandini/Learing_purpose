/*
 * EPAS_interface.h
 *
 *  Created on: May 24, 2023
 *      Author: kanhu
 */

#ifndef EPAS_INTERFACE_H_
#define EPAS_INTERFACE_H_

#include "Cpu.h"
#include "vcuSys.h"

#include "Debug_Print.h"


typedef struct
{
	#if defined(OE_EPAS)

	int16_t EPAS_angle;
	int8_t EPAS_temperature;

	#elif defined(RANE_EPAS)

	uint16_t EPAS_angle;

	#endif
}EPAS_dataStruct;



void EPAS_init();

void EPAScontrolEntry(uint32_t timestamp);

void EPAScontrolExit(uint32_t timestamp);

void EPAS_interface(genericCANframeData_t *EPAS_interfacesMsg, uint32_t timestamp);

void EPAS_watchdog(uint32_t timestamp);



#endif /* EPAS_INTERFACE_H_ */
