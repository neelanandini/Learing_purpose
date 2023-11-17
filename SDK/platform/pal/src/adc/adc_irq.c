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
/*!
 * @file adc_irq.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, external symbol defined without a prior
 * declaration.
 * Default IRQHandler symbols are weak symbols declared in platform startup files (.s).
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Default IRQHandler symbols are weak symbols declared in platform startup files (.s).
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, Could define variable at block scope.
 * Array with pointers to interrupt handler functions are defined in this file to improve code readability of pal source file.
 *
 */

#include "adc_pal_cfg.h"
#include "adc_irq.h"
#include "adc_pal_mapping.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Define default interrupt handlers for ADC instances.
 * Default interrupt handlers already declared in startup files. */

#if defined (ADC_PAL_S32K1xx)

#if (ADC_INSTANCE_COUNT >= 1u)
void ADC0_IRQHandler(void);

void ADC0_IRQHandler(void)
{
    ADC_S32K1xx_IrqHandler(0u);
}
#endif /* (ADC_INSTANCE_COUNT >= 1u) */

#if (ADC_INSTANCE_COUNT >= 2u)
void ADC1_IRQHandler(void);

void ADC1_IRQHandler(void)
{
    ADC_S32K1xx_IrqHandler(1u);
}
#endif /* (ADC_INSTANCE_COUNT >= 2u) */

#endif /* defined(ADC_PAL_S32K1xx) */


#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_SAR)

#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
#if ADC_PAL_BCTU_CONV_UPDATE_IRQ_UNROLLED
const IRQn_Type bctuConvUpdateIrqNum[BCTU_INSTANCE_COUNT][FEATURE_BCTU_NUM_ADC] = {
{
#if FEATURE_BCTU_NUM_ADC > 0u
          BCTU0_ADC0_IRQn,
#endif
#if FEATURE_BCTU_NUM_ADC > 1u
          BCTU0_ADC1_IRQn,
#endif
#if FEATURE_BCTU_NUM_ADC > 2u
          BCTU0_ADC2_IRQn,
#endif
#if FEATURE_BCTU_NUM_ADC > 3u
          BCTU0_ADC3_IRQn
#endif
}
#if BCTU_INSTANCE_COUNT > 1u
#error "Number of BCTU instances is not supported by ADC PAL"
#endif
};

void (* const bctuConvUpdateIrqHandlers[BCTU_INSTANCE_COUNT][FEATURE_BCTU_NUM_ADC])(void) = {
{
#if FEATURE_BCTU_NUM_ADC > 0u
        BCTU0_ADC0_IRQHandler,
#endif
#if FEATURE_BCTU_NUM_ADC > 1u
        BCTU0_ADC1_IRQHandler,
#endif
#if FEATURE_BCTU_NUM_ADC > 2u
        BCTU0_ADC2_IRQHandler,
#endif
#if FEATURE_BCTU_NUM_ADC > 3u
        BCTU0_ADC3_IRQHandler
#endif
}
#if BCTU_INSTANCE_COUNT > 1u
#error "Number of BCTU instances is not supported by ADC PAL"
#endif
};


#else
const IRQn_Type bctuConvUpdateIrqNum[BCTU_INSTANCE_COUNT] = {
        BCTU_ConvUpdate_IRQn
#if BCTU_INSTANCE_COUNT > 1u
#error "Number of BCTU instances is not supported by ADC PAL"
#endif
};
#endif /* ADC_PAL_BCTU_CONV_UPDATE_IRQ_UNROLLED */


#if ADC_PAL_BCTU_CONV_UPDATE_IRQ_UNROLLED
/* Interrupt handlers for HW triggered groups with a single conversion */
#if ADC_INSTANCE_COUNT > 0u
void BCTU0_ADC0_IRQHandler(void)
{
    const uint8_t adcIdx = 0u;
    const uint32_t bctuIdx = 0u; /* For ADC_PAL_TYPE_ADC_SAR_BCTU, ADC PAL instances are mapped 1:1 with BCTU instances. */

    ADC_SAR_BCTU_HwTrigIrqHandler(bctuIdx, adcIdx);
}
#endif /* ADC_INSTANCE_COUNT > 0 */
#if ADC_INSTANCE_COUNT > 1u
void BCTU0_ADC1_IRQHandler(void)
{
    const uint8_t adcIdx = 1u;
    const uint32_t bctuIdx = 0u; /* For ADC_PAL_TYPE_ADC_SAR_BCTU, ADC PAL instances are mapped 1:1 with BCTU instances. */

    ADC_SAR_BCTU_HwTrigIrqHandler(bctuIdx, adcIdx);
}
#endif /* ADC_INSTANCE_COUNT > 1 */
#if ADC_INSTANCE_COUNT > 2u
void BCTU0_ADC2_IRQHandler(void)
{
    const uint8_t adcIdx = 2u;
    const uint32_t bctuIdx = 0u; /* For ADC_PAL_TYPE_ADC_SAR_BCTU, ADC PAL instances are mapped 1:1 with BCTU instances. */

    ADC_SAR_BCTU_HwTrigIrqHandler(bctuIdx, adcIdx);
}
#endif /* ADC_INSTANCE_COUNT > 2 */
#if ADC_INSTANCE_COUNT > 3u
void BCTU0_ADC3_IRQHandler(void)
{
    const uint8_t adcIdx = 3u;
    const uint32_t bctuIdx = 0u; /* For ADC_PAL_TYPE_ADC_SAR_BCTU, ADC PAL instances are mapped 1:1 with BCTU instances. */

    ADC_SAR_BCTU_HwTrigIrqHandler(bctuIdx, adcIdx);
}
#endif /* ADC_INSTANCE_COUNT > 3 */

#else

/* For HW triggered groups with a single conversion */
void BCTU_ConvUpdate_IRQHandler(void)
{
    uint8_t adcIdx = 0u;
    const uint32_t bctuIdx = 0u; /* For ADC_PAL_TYPE_ADC_SAR_BCTU, ADC PAL instances are mapped 1:1 with BCTU instances. */

    for (adcIdx = 0u; adcIdx < FEATURE_BCTU_NUM_ADC; adcIdx++)
    {
        /* ADC PAL doesn't trigger conversions in parallel on multiple ADCs,
         * so only one ADC should have new data available. */
        if (BCTU_DRV_GetStatusFlag(bctuIdx, adcIdx, BCTU_FLAG_NEW_DATA_AVAILABLE))
        {
            ADC_SAR_BCTU_HwTrigIrqHandler(bctuIdx, adcIdx);

            break;
        }
    }
}
#endif /* ADC_PAL_BCTU_CONV_UPDATE_IRQ_UNROLLED */

/* For HW triggered groups with multiple conversions */
void BCTU_ListLast_IRQHandler(void)
{
    uint8_t adcIdx = 0u;
    const uint32_t bctuIdx = 0u; /* For ADC_PAL_TYPE_ADC_SAR_BCTU, ADC PAL instances are mapped 1:1 with BCTU instances. */

    for (adcIdx = 0u; adcIdx < FEATURE_BCTU_NUM_ADC; adcIdx++)
    {
        /* ADC PAL doesn't trigger conversions in parallel on multiple ADCs,
         * so only one ADC should have new data available. */
        if (BCTU_DRV_GetStatusFlag(bctuIdx, adcIdx, BCTU_FLAG_LIST_LAST_CONV))
        {
            ADC_SAR_BCTU_HwTrigIrqHandler(bctuIdx, adcIdx);

            break;
        }
    }
}
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_BCTU) */

#if (ADC_INSTANCE_COUNT > 0u)
void ADC0_EOC_IRQHandler(void)
{
    /* For ADC_PAL_TYPE_ADC_SAR_BCTU, ADC PAL instances are mapped 1:1 with BCTU instances.
     * ADC0 is connected to BCTU 0. */
    const uint32_t palIdx = 0u;
    ADC_SAR_SwTrigIrqHandler(palIdx, 0u);
}
#endif /* (ADC_INSTANCE_COUNT > 0u) */

#if (ADC_INSTANCE_COUNT > 1u)
void ADC1_EOC_IRQHandler(void)
{
    /* For ADC_PAL_TYPE_ADC_SAR_BCTU, ADC PAL instances are mapped 1:1 with BCTU instances.
     * ADC1 is connected to BCTU 0. */
    const uint32_t palIdx = 0u;
    ADC_SAR_SwTrigIrqHandler(palIdx, 1u);
}
#endif /* (ADC_INSTANCE_COUNT > 1u) */

#if (ADC_INSTANCE_COUNT > 2u)
void ADC2_EOC_IRQHandler(void)
{
    /* For ADC_PAL_TYPE_ADC_SAR_BCTU, ADC PAL instances are mapped 1:1 with BCTU instances.
     * ADC2 is connected to BCTU 0. */
    const uint32_t palIdx = 0u;
    ADC_SAR_SwTrigIrqHandler(palIdx, 2u);
}
#endif /* (ADC_INSTANCE_COUNT > 2u) */

#if (ADC_INSTANCE_COUNT > 3u)
void ADC3_EOC_IRQHandler(void)
{
    /* For ADC_PAL_TYPE_ADC_SAR_BCTU, ADC PAL instances are mapped 1:1 with BCTU instances.
     * ADC3 is connected to BCTU 0. */
    const uint32_t palIdx = 0u;
    ADC_SAR_SwTrigIrqHandler(palIdx, 3u);
}
#endif /* (ADC_INSTANCE_COUNT > 3u) */

#endif /* defined(ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_SAR) */


#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)

#if (ADC_PAL_MAX_NUM_HW_GROUPS_EN >= 1u)

void CTU0_FIFO0_IRQHandler(void)
{
    const uint32_t palIdx   = 0u;
    ADC_SAR_CTU_HwTrigIrqHandler(palIdx, 0u);
}

#endif /* (ADC_PAL_MAX_NUM_HW_GROUPS_EN >= 1u) */

#if (ADC_PAL_MAX_NUM_HW_GROUPS_EN >= 2u)

void CTU0_FIFO1_IRQHandler(void)
{
    const uint32_t palIdx   = 0u;
    ADC_SAR_CTU_HwTrigIrqHandler(palIdx, 1u);
}

#endif /* (ADC_PAL_MAX_NUM_HW_GROUPS_EN >= 2u) */

#if (ADC_PAL_MAX_NUM_HW_GROUPS_EN >= 3u)

void CTU0_FIFO2_IRQHandler(void)
{
    const uint32_t palIdx   = 0u;
    ADC_SAR_CTU_HwTrigIrqHandler(palIdx, 2u);
}

#endif /* (ADC_PAL_MAX_NUM_HW_GROUPS_EN >= 3u) */

#if (ADC_PAL_MAX_NUM_HW_GROUPS_EN >= 4u)

void CTU0_FIFO3_IRQHandler(void)
{
    const uint32_t palIdx   = 0u;
    ADC_SAR_CTU_HwTrigIrqHandler(palIdx, 3u);
}

#endif /* (ADC_PAL_MAX_NUM_HW_GROUPS_EN >= 4u) */


#if CTU_INSTANCE_COUNT > 1u

#if (ADC_PAL_MAX_NUM_HW_GROUPS_EN >= 1u)

void CTU1_FIFO0_IRQHandler(void)
{
    const uint32_t palIdx   = 1u;
    ADC_SAR_CTU_HwTrigIrqHandler(palIdx, 0u);
}

#endif /* (ADC_PAL_MAX_NUM_HW_GROUPS_EN >= 1u) */

#if (ADC_PAL_MAX_NUM_HW_GROUPS_EN >= 2u)

void CTU1_FIFO1_IRQHandler(void)
{
    const uint32_t palIdx   = 1u;
    ADC_SAR_CTU_HwTrigIrqHandler(palIdx, 1u);
}

#endif /* (ADC_PAL_MAX_NUM_HW_GROUPS_EN >= 2u) */

#if (ADC_PAL_MAX_NUM_HW_GROUPS_EN >= 3u)

void CTU1_FIFO2_IRQHandler(void)
{
    const uint32_t palIdx   = 1u;
    ADC_SAR_CTU_HwTrigIrqHandler(palIdx, 2u);
}

#endif /* (ADC_PAL_MAX_NUM_HW_GROUPS_EN >= 3u) */

#if (ADC_PAL_MAX_NUM_HW_GROUPS_EN >= 4u)

void CTU1_FIFO3_IRQHandler(void)
{
    const uint32_t palIdx   = 1u;
    ADC_SAR_CTU_HwTrigIrqHandler(palIdx, 3u);
}

#endif /* (ADC_PAL_MAX_NUM_HW_GROUPS_EN >= 4u) */
#endif /* CTU_INSTANCE_COUNT > 1u */

void ADC0_EOC_IRQHandler(void)
{
    /* For ADC_PAL_TYPE_ADC_SAR_CTU, ADC PAL instances are mapped 1:1 with CTU instances.
     * ADC0 is connected to CTU 0. */
    const uint32_t palIdx = 0u;
    ADC_SAR_SwTrigIrqHandler(palIdx, 0u);
}

#if ADC_INSTANCE_COUNT > 1u

void ADC1_EOC_IRQHandler(void)
{
    /* For ADC_PAL_TYPE_ADC_SAR_CTU, ADC PAL instances are mapped 1:1 with CTU instances.
     * ADC1 is connected to CTU 0. */
    const uint32_t palIdx = 0u;
    ADC_SAR_SwTrigIrqHandler(palIdx, 1u);
}

#endif /* ADC_INSTANCE_COUNT > 1u */

#if ADC_INSTANCE_COUNT > 2u

void ADC2_EOC_IRQHandler(void)
{
    /* For ADC_PAL_TYPE_ADC_SAR_CTU, ADC PAL instances are mapped 1:1 with CTU instances.
     * ADC2 is connected to CTU 1. */
    const uint32_t palIdx = 1u;
    ADC_SAR_SwTrigIrqHandler(palIdx, 2u);
}

#endif /* ADC_INSTANCE_COUNT > 2u */

#if ADC_INSTANCE_COUNT > 3u

void ADC3_EOC_IRQHandler(void)
{
    /* For ADC_PAL_TYPE_ADC_SAR_CTU, ADC PAL instances are mapped 1:1 with CTU instances.
     * ADC3 is connected to CTU 1. */
    const uint32_t palIdx = 1u;
    ADC_SAR_SwTrigIrqHandler(palIdx, 3u);
}

#endif /* ADC_INSTANCE_COUNT > 3u */

#endif /* defined(ADC_PAL_TYPE_ADC_SAR_CTU) */


#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_SAR)

/* Array of function pointers initialized to ADC interrupt handlers to be registered */
void (* const adcEocIrqHandlers[ADC_INSTANCE_COUNT])(void) = {
#if (ADC_INSTANCE_COUNT > 0u)
        ADC0_EOC_IRQHandler,
#endif /* (ADC_INSTANCE_COUNT > 0u) */
#if (ADC_INSTANCE_COUNT > 1u)
        ADC1_EOC_IRQHandler,
#endif /* (ADC_INSTANCE_COUNT > 1u) */
#if (ADC_INSTANCE_COUNT > 2u)
        ADC2_EOC_IRQHandler,
#endif /* (ADC_INSTANCE_COUNT > 2u) */
#if (ADC_INSTANCE_COUNT > 3u)
        ADC3_EOC_IRQHandler
#endif /* (ADC_INSTANCE_COUNT > 3u) */
};

#endif /* defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_SAR) */


#if defined (ADC_PAL_TYPE_SDADC)

#if FEATURE_SDADC_INSTANCES_HAVE_COMMON_INT_VECTOR
bool sdadcInstEnabled[SDADC_INSTANCE_COUNT];

void SDADC1234_IRQHandler(void)
{
    uint32_t palIdx;

	/* For ADC_PAL_TYPE_SDADC, ADC PAL instances are mapped 1:1 with SDADC instances. */

    for(palIdx = 0u; palIdx < SDADC_INSTANCE_COUNT; palIdx++)
    {
    	/* Check which SDADC instance are enabled and have the interrupt asserted */
    	if(sdadcInstEnabled[palIdx] == true)
    	{
    		if((SDADC_DRV_GetStatusFlags(palIdx) & SDADC_EVENT_FIFO_FULL) != 0u)
    		{
    			SDADC_SwTrigIrqHandler(palIdx);
    		}
    	}
    }
}

/* Array of function pointers initialized to ADC interrupt handlers to be registered */
void (* const sdadcIrqHandlers[SDADC_INSTANCE_COUNT])(void) = {
#if (SDADC_INSTANCE_COUNT > 0u)
		SDADC1234_IRQHandler,
#endif /* (SDADC_INSTANCE_COUNT > 0u) */
#if (SDADC_INSTANCE_COUNT > 1u)
		SDADC1234_IRQHandler,
#endif /* (SDADC_INSTANCE_COUNT > 1u) */
#if (SDADC_INSTANCE_COUNT > 2u)
		SDADC1234_IRQHandler,
#endif /* (SDADC_INSTANCE_COUNT > 2u) */
#if (SDADC_INSTANCE_COUNT > 3u)
		SDADC1234_IRQHandler
#endif /* (SDADC_INSTANCE_COUNT > 3u) */
};

#else
void SDADC0_IRQHandler(void)
{
    /* For ADC_PAL_TYPE_SDADC, ADC PAL instances are mapped 1:1 with SDADC instances. */
    const uint32_t palIdx = 0u;
    SDADC_SwTrigIrqHandler(palIdx);
}

#if SDADC_INSTANCE_COUNT > 1u

void SDADC1_IRQHandler(void)
{
    /* For ADC_PAL_TYPE_SDADC, ADC PAL instances are mapped 1:1 with SDADC instances. */
    const uint32_t palIdx = 1u;
    SDADC_SwTrigIrqHandler(palIdx);
}

#endif /* ADC_INSTANCE_COUNT > 1u */

#if SDADC_INSTANCE_COUNT > 2u

void SDADC2_IRQHandler(void)
{
    /* For ADC_PAL_TYPE_SDADC, ADC PAL instances are mapped 1:1 with SDADC instances. */
    const uint32_t palIdx = 2u;
    SDADC_SwTrigIrqHandler(palIdx);
}

#endif /* ADC_INSTANCE_COUNT > 2u */

#if SDADC_INSTANCE_COUNT > 3u

void SDADC3_IRQHandler(void)
{
    /* For ADC_PAL_TYPE_SDADC, ADC PAL instances are mapped 1:1 with SDADC instances. */
    const uint32_t palIdx = 3u;
    SDADC_SwTrigIrqHandler(palIdx);
}

#endif /* ADC_INSTANCE_COUNT > 3u */


/* Array of function pointers initialized to ADC interrupt handlers to be registered */
void (* const sdadcIrqHandlers[SDADC_INSTANCE_COUNT])(void) = {
#if (SDADC_INSTANCE_COUNT > 0u)
        SDADC0_IRQHandler,
#endif /* (SDADC_INSTANCE_COUNT > 0u) */
#if (SDADC_INSTANCE_COUNT > 1u)
        SDADC1_IRQHandler,
#endif /* (SDADC_INSTANCE_COUNT > 1u) */
#if (SDADC_INSTANCE_COUNT > 2u)
        SDADC2_IRQHandler,
#endif /* (SDADC_INSTANCE_COUNT > 2u) */
#if (SDADC_INSTANCE_COUNT > 3u)
        SDADC3_IRQHandler
#endif /* (SDADC_INSTANCE_COUNT > 3u) */
};

#endif /* FEATURE_SDADC_INSTANCES_HAVE_COMMON_INT_VECTOR */

#endif /* defined (ADC_PAL_TYPE_SDADC) */


#if defined (ADC_PAL_TYPE_EQADC)

void EQADC0_CFIFO0_InterruptHandler(void)
{
    /* For ADC_PAL_TYPE_EQADC, ADC PAL instances are mapped 1:1 with EQADC instances. */
    const uint32_t palIdx = 0u;
    const uint32_t fifoIdx = 0u;
    EQADC_CfifoFillRequest(palIdx, fifoIdx);
}

void EQADC0_RFIFO0_InterruptHandler(void)
{
    /* For ADC_PAL_TYPE_EQADC, ADC PAL instances are mapped 1:1 with EQADC instances. */
    const uint32_t palIdx = 0u;
    const uint32_t fifoIdx = 0u;
    EQADC_RfifoDrainRequest(palIdx, fifoIdx);
}

void EQADC0_CFIFO1_InterruptHandler(void)
{
    /* For ADC_PAL_TYPE_EQADC, ADC PAL instances are mapped 1:1 with EQADC instances. */
    const uint32_t palIdx = 0u;
    const uint32_t fifoIdx = 1u;
    EQADC_CfifoFillRequest(palIdx, fifoIdx);
}

void EQADC0_RFIFO1_InterruptHandler(void)
{
    /* For ADC_PAL_TYPE_EQADC, ADC PAL instances are mapped 1:1 with EQADC instances. */
    const uint32_t palIdx = 0u;
    const uint32_t fifoIdx = 1u;
    EQADC_RfifoDrainRequest(palIdx, fifoIdx);
}

#if EQADC_INSTANCE_COUNT > 1u

void EQADC1_CFIFO0_InterruptHandler(void)
{
    /* For ADC_PAL_TYPE_EQADC, ADC PAL instances are mapped 1:1 with EQADC instances. */
    const uint32_t palIdx = 1u;
    const uint32_t fifoIdx = 0u;
    EQADC_CfifoFillRequest(palIdx, fifoIdx);
}

void EQADC1_RFIFO0_InterruptHandler(void)
{
    /* For ADC_PAL_TYPE_EQADC, ADC PAL instances are mapped 1:1 with EQADC instances. */
    const uint32_t palIdx = 1u;
    const uint32_t fifoIdx = 0u;
    EQADC_RfifoDrainRequest(palIdx, fifoIdx);
}

void EQADC1_CFIFO1_InterruptHandler(void)
{
    /* For ADC_PAL_TYPE_EQADC, ADC PAL instances are mapped 1:1 with EQADC instances. */
    const uint32_t palIdx = 1u;
    const uint32_t fifoIdx = 1u;
    EQADC_CfifoFillRequest(palIdx, fifoIdx);
}

void EQADC1_RFIFO1_InterruptHandler(void)
{
    /* For ADC_PAL_TYPE_EQADC, ADC PAL instances are mapped 1:1 with EQADC instances. */
    const uint32_t palIdx = 1u;
    const uint32_t fifoIdx = 1u;
    EQADC_RfifoDrainRequest(palIdx, fifoIdx);
}

#endif /* EQADC_INSTANCE_COUNT > 1u */

#if EQADC_INSTANCE_COUNT > 2u
#error "Unsupported number for EQADC instances in ADC PAL"
#endif /* EQADC_INSTANCE_COUNT > 2u */


/* Array of function pointers initialized to ADC interrupt handlers to be registered */
void (* const eqadcCfifoFillHandlers[EQADC_INSTANCE_COUNT][EQADC_NUM_USED_FIFOS])(void) = {
#if (EQADC_INSTANCE_COUNT > 0u)
    {
        EQADC0_CFIFO0_InterruptHandler,
        EQADC0_CFIFO1_InterruptHandler
    },
#endif /* (EQADC_INSTANCE_COUNT > 0u) */
#if (EQADC_INSTANCE_COUNT > 1u)
    {
        EQADC1_CFIFO0_InterruptHandler,
        EQADC1_CFIFO1_InterruptHandler
    },
#endif /* (EQADC_INSTANCE_COUNT > 1u) */
};

/* Array of function pointers initialized to ADC interrupt handlers to be registered */
void (* const eqadcRfifoFillHandlers[EQADC_INSTANCE_COUNT][EQADC_NUM_USED_FIFOS])(void) = {
#if (EQADC_INSTANCE_COUNT > 0u)
    {
        EQADC0_RFIFO0_InterruptHandler,
        EQADC0_RFIFO1_InterruptHandler
    },
#endif /* (EQADC_INSTANCE_COUNT > 0u) */
#if (EQADC_INSTANCE_COUNT > 1u)
    {
        EQADC1_RFIFO0_InterruptHandler,
        EQADC1_RFIFO1_InterruptHandler
    },
#endif /* (EQADC_INSTANCE_COUNT > 1u) */
};


#endif /* defined (ADC_PAL_TYPE_EQADC) */


/*******************************************************************************
 * EOF
 ******************************************************************************/
