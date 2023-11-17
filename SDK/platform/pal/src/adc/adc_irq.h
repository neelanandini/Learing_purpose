/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
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

#ifndef ADC_IRQ_H
#define ADC_IRQ_H

#include "device_registers.h"
#include "adc_pal_mapping.h"


/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#if defined (ADC_PAL_S32K1xx)

void ADC_S32K1xx_IrqHandler(const uint32_t instIdx);

#endif /* defined(ADC_PAL_S32K1xx) */


#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_SAR)

#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
#if ADC_PAL_BCTU_CONV_UPDATE_IRQ_UNROLLED
extern const IRQn_Type bctuConvUpdateIrqNum[BCTU_INSTANCE_COUNT][FEATURE_BCTU_NUM_ADC];
extern void (* const bctuConvUpdateIrqHandlers[BCTU_INSTANCE_COUNT][FEATURE_BCTU_NUM_ADC])(void);
#else
extern const IRQn_Type bctuConvUpdateIrqNum[BCTU_INSTANCE_COUNT];
#endif /* ADC_PAL_BCTU_CONV_UPDATE_IRQ_UNROLLED */


#if ADC_PAL_BCTU_CONV_UPDATE_IRQ_UNROLLED

/* Interrupt handlers for HW triggered groups with a single conversion */
/* Install custom interrupt handler for BCTU because default one has different names depending on device */
#if ADC_INSTANCE_COUNT > 0u
void BCTU0_ADC0_IRQHandler(void);
#endif
#if ADC_INSTANCE_COUNT > 1u
void BCTU0_ADC1_IRQHandler(void);
#endif
#if ADC_INSTANCE_COUNT > 2u
void BCTU0_ADC2_IRQHandler(void);
#endif
#if ADC_INSTANCE_COUNT > 3u
void BCTU0_ADC3_IRQHandler(void);
#endif

#else

/* Interrupt handler for HW triggered groups with a single conversion */
/* Install custom interrupt handler for BCTU because default one has different names depending on device */
void BCTU_ConvUpdate_IRQHandler(void);

#endif /* ADC_PAL_BCTU_CONV_UPDATE_IRQ_UNROLLED */

/* Interrupt handler for HW triggered groups with multiple conversions */
/* Install custom interrupt handler for BCTU because default one has different names depending on device */
void BCTU_ListLast_IRQHandler(void);

#endif /* defined (ADC_PAL_TYPE_ADC_SAR_BCTU) */

void ADC_SAR_BCTU_HwTrigIrqHandler(const uint32_t instIdx,
                                       const uint8_t adcIdx);

#endif /* defined(ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_SAR) */

#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)

void ADC_SAR_CTU_HwTrigIrqHandler(const uint32_t instIdx,
                                  const uint8_t fifoIdx);

#endif /* defined(ADC_PAL_TYPE_ADC_SAR_CTU) */

#if (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_SAR))

void ADC_SAR_SwTrigIrqHandler(const uint32_t instIdx,
                              const uint8_t adcIdx);

/* Prototypes for ADC EOC custom interrupt handlers*/
#if (ADC_INSTANCE_COUNT > 0u)
void ADC0_EOC_IRQHandler(void);
#endif /* (ADC_INSTANCE_COUNT > 0u) */

#if (ADC_INSTANCE_COUNT > 1u)
void ADC1_EOC_IRQHandler(void);
#endif /* (ADC_INSTANCE_COUNT > 1u) */

#if (ADC_INSTANCE_COUNT > 2u)
void ADC2_EOC_IRQHandler(void);
#endif /* (ADC_INSTANCE_COUNT > 2u) */

#if (ADC_INSTANCE_COUNT > 3u)
void ADC3_EOC_IRQHandler(void);
#endif /* (ADC_INSTANCE_COUNT > 3u) */

/* Array of function pointers initialized to ADC interrupt handlers to be registered */
extern void (* const adcEocIrqHandlers[ADC_INSTANCE_COUNT])(void);

#endif /* (defined(ADC_PAL_TYPE_ADC_SAR_BCTU) || defined(ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_SAR)) */


#if defined (ADC_PAL_TYPE_SDADC)

void SDADC_SwTrigIrqHandler(const uint32_t instIdx);

#if FEATURE_SDADC_INSTANCES_HAVE_COMMON_INT_VECTOR
extern bool sdadcInstEnabled[SDADC_INSTANCE_COUNT];
#endif /* FEATURE_SDADC_INSTANCES_HAVE_COMMON_INT_VECTOR */

/* Prototypes for ADC EOC custom interrupt handlers*/
#if (SDADC_INSTANCE_COUNT > 0u)
void SDADC0_IRQHandler(void);
#endif /* (SDADC_INSTANCE_COUNT > 0u) */

#if (SDADC_INSTANCE_COUNT > 1u)
void SDADC1_IRQHandler(void);
#endif /* (SDADC_INSTANCE_COUNT > 1u) */

#if (SDADC_INSTANCE_COUNT > 2u)
void SDADC2_IRQHandler(void);
#endif /* (SDADC_INSTANCE_COUNT > 2u) */

#if (SDADC_INSTANCE_COUNT > 3u)
void SDADC3_IRQHandler(void);
#endif /* (SDADC_INSTANCE_COUNT > 3u) */

/* Array of function pointers initialized to ADC interrupt handlers to be registered */
extern void (* const sdadcIrqHandlers[SDADC_INSTANCE_COUNT])(void);

#endif /* defined (ADC_PAL_TYPE_SDADC) */



#if defined (ADC_PAL_TYPE_EQADC)

void EQADC_CfifoFillRequest(const uint32_t instIdx, const uint32_t fifoIdx);
void EQADC_RfifoDrainRequest(const uint32_t instIdx, const uint32_t fifoIdx);

/* Number of FIFOS (result/commmand) used by the ADC PAL
 * Currently 1 cfifo and 1 rfifo are used for SW triggered group, and 1 cfifo and 1 rfifo for HW triggered group */
#define EQADC_NUM_USED_FIFOS         (2u)

/* Prototypes for EQADC custom interrupt handlers*/
void EQADC0_CFIFO0_InterruptHandler(void);
void EQADC0_RFIFO0_InterruptHandler(void);
void EQADC0_CFIFO1_InterruptHandler(void);
void EQADC0_RFIFO1_InterruptHandler(void);

#if EQADC_INSTANCE_COUNT > 1u
void EQADC1_CFIFO0_InterruptHandler(void);
void EQADC1_RFIFO0_InterruptHandler(void);
void EQADC1_CFIFO1_InterruptHandler(void);
void EQADC1_RFIFO1_InterruptHandler(void);
#endif /* EQADC_INSTANCE_COUNT > 1u */

extern void (* const eqadcCfifoFillHandlers[EQADC_INSTANCE_COUNT][EQADC_NUM_USED_FIFOS])(void);
extern void (* const eqadcRfifoFillHandlers[EQADC_INSTANCE_COUNT][EQADC_NUM_USED_FIFOS])(void);

#endif /* defined (ADC_PAL_TYPE_EQADC) */

#endif /* ADC_IRQ_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
