/*
** ###################################################################
**     Processor:           MPC5775B
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright 2018-2019 NXP
**     All rights reserved.
**
**     THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
**     IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
**     OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
**     IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
**     INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
**     SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
**     HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
**     STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
**     IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
**     THE POSSIBILITY OF SUCH DAMAGE.
**
**
** ###################################################################
*/

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, An object should be defined at block
 * scope if its identifier only appears in a single function.
 * An object with static storage duration declared at block scope cannot be
 * accessed directly from outside the block.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required to initialize a pointer with an unsigned int define,
 * representing a memory-mapped address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from pointer to unsigned int.
 * The cast is required to initialize a pointer with an unsigned int define,
 * representing a memory-mapped address.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.1, Conversions shall not be performed
 * between a pointer to a function and any other type.
 * This is required in order to write the prefix of the interrupt vector table.
 *
 */

/*!
 * @file MPC5775B
 * @version 1.0
 * @date 2017-02-14
 * @brief Device specific configuration file for MPC5775B (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "system_MPC5775B.h"


/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/*FUNCTION**********************************************************************
 *
 * Function Name : SystemInit
 * Description   : Typically this function disables the watchdog and enables FPU
 * that are parts of the microcontroller device. SystemInit is called from
 * startup_device file.
 *
 * Implements    : SystemInit_Activity
 *END**************************************************************************/
void SystemInit(void)
{
#if defined(START_SECONDARY_CORES)
#if defined(TURN_ON_CPU1)
    uint32_t cpu1_bootAddr = 0U;
    #if defined(__cpu1_boot_addr__)
        cpu1_bootAddr = __cpu1_boot_addr__;
    #else
        #if defined(START_FROM_FLASH)
            cpu1_bootAddr = 0x00A00000U;
        #else
            cpu1_bootAddr = 0x40040000U;
        #endif /* defined(START_FROM_FLASH) */
    #endif /* defined(__cpu1_boot_addr__) */
    /* Set Start address for core 1: Will reset and start */
    SIU->RSTVEC1 = cpu1_bootAddr | 0x1U;
#endif /* defined(TURN_ON_CPU1) */
#endif /* defined(START_SECONDARY_CORES) */

#if INIT_INTERRUPT_CONTROLLER
    /* initialize interrupt controller for current core */
    uint8_t coreId = GET_CORE_ID();
    switch (coreId)
    {
        case 0U:
            /* Software vector mode used for core 0 */
            INTC->MCR  &= ~(INTC_MCR_HVEN_PRC0_MASK);
            /* Lower core 0's INTC current priority to 0 */
            INTC->CPR0 = 0U;
            break;
        case 1U:
            /* Software vector mode used for core 1 */
            INTC->MCR  &= ~(INTC_MCR_HVEN_PRC1_MASK);
            /* Lower core 1's INTC current priority to 0 */
            INTC->CPR1 = 0U;
            break;
        default:
            /* invalid core number */
            DEV_ASSERT(false);
            break;
    }
    /* Initialize core's spr IVPR register*/
    MTSPR(63,(uint32_t)&VTABLE);
#endif

/**************************************************************************/
            /* GRANT ACCESS TO PERIPHERALS FOR DMA MASTER */
/**************************************************************************/
#if ENABLE_DMA_ACCESS_TO_PERIPH
  /* DMA trusted for read/writes in supervisor & user modes on peripheral bridge A */
  AIPS_0->MPRA |= AIPS_MPRA_MPL2_MASK;
  AIPS_0->MPRA |= AIPS_MPRA_MTW2_MASK;
  AIPS_0->MPRA |= AIPS_MPRA_MTR2_MASK;
  /* DMA trusted for read/writes in supervisor & user modes on peripheral bridge B */
  AIPS_1->MPRA |= AIPS_MPRA_MPL2_MASK;
  AIPS_1->MPRA |= AIPS_MPRA_MTW2_MASK;
  AIPS_1->MPRA |= AIPS_MPRA_MTR2_MASK;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SystemCoreClockUpdate
 * Description   : This function must be called whenever the core clock is changed
 * during program execution. It evaluates the clock register settings and calculates
 * the current core clock.
 *
 * Implements    : SystemCoreClockUpdate_Activity
 *END**************************************************************************/
void SystemCoreClockUpdate(void)
{
    uint32_t SYSClock = 0U;
    uint32_t PLL_0Clock = 0U;
    uint32_t PLL_1Clock = 0U;
    uint32_t divPLL_0_PHI_0, divPLL_0_PHI_1, divPLL_1_PHI_0, predivPLL_0, multiPLL_0, multiPLL_1;
    bool validSystemClockSource = true;
    bool validPLL_0ClockSource = true;
    bool validPLL_1ClockSource = true;

    /* Multiplication factor divider for PLL0 and PLL1 */
    multiPLL_0 = (PLLDIG->PLL0DV & PLLDIG_PLL0DV_MFD_MASK) >> PLLDIG_PLL0DV_MFD_SHIFT;
    multiPLL_1 = (PLLDIG->PLL1DV & PLLDIG_PLL1DV_MFD_MASK) >> PLLDIG_PLL1DV_MFD_SHIFT;

    /* Input clock pre-divider */
    if(((PLLDIG->PLL0DV & PLLDIG_PLL0DV_PREDIV_MASK) >> PLLDIG_PLL0DV_PREDIV_SHIFT) != 0U)
    {
        predivPLL_0 = (PLLDIG->PLL0DV & PLLDIG_PLL0DV_PREDIV_MASK) >> PLLDIG_PLL0DV_PREDIV_SHIFT;
    }
    else
    {
        predivPLL_0 = 0;
        validPLL_0ClockSource = false;
    }

    /* PHI reduced frequency divider */
    divPLL_0_PHI_0 = (PLLDIG->PLL0DV & PLLDIG_PLL0DV_RFDPHI_MASK) >> PLLDIG_PLL0DV_RFDPHI_SHIFT;
    divPLL_0_PHI_1 = (PLLDIG->PLL0DV & PLLDIG_PLL0DV_RFDPHI1_MASK) >> PLLDIG_PLL0DV_RFDPHI1_SHIFT;
    divPLL_1_PHI_0 = (PLLDIG->PLL1DV & PLLDIG_PLL1DV_RFDPHI_MASK) >> PLLDIG_PLL1DV_RFDPHI_SHIFT;

    /* PLL0 out clock */
    if ((SIU->SYSDIV & SIU_SYSDIV_PLL0SEL_MASK) == 0U)
    {
        PLL_0Clock = CPU_EXT_FAST_CLK_HZ;
    }
    else
    {
        PLL_0Clock = CPU_INT_FAST_CLK_HZ;
    }

    /* PLL1 out clock */
    if ((SIU->SYSDIV & SIU_SYSDIV_PLL1SEL_MASK) != 0U)
    {
        if ((validPLL_0ClockSource) && (predivPLL_0 != 0U))
        {
            PLL_1Clock = PLL_0Clock * multiPLL_0 / (predivPLL_0 * divPLL_0_PHI_1);
        }
        else
        {
            validPLL_1ClockSource = false;
        }
    }
    else
    {
        PLL_1Clock = CPU_EXT_FAST_CLK_HZ;
    }

    /* System out clock */
    switch ((SIU->SYSDIV & SIU_SYSDIV_SYSCLKSEL_MASK) >> SIU_SYSDIV_SYSCLKSEL_SHIFT)
    {
    case 0x3:
        /* primary PLL */
        if ((validPLL_0ClockSource) && (predivPLL_0 != 0U))
        {
            SYSClock = PLL_0Clock * multiPLL_0 / (predivPLL_0 * divPLL_0_PHI_0);
        }
        else
        {
            validSystemClockSource = false;
        }
        break;
    case 0x2:
        /* secondary PLL */
        if (validPLL_1ClockSource)
        {
            SYSClock = PLL_1Clock * multiPLL_1/divPLL_1_PHI_0;
        }
        else
        {
            validSystemClockSource = false;
        }
        break;
    case 0x1:
        /* XOSC */
        SYSClock = CPU_EXT_FAST_CLK_HZ;
        break;
    default:
        /* 16MHz IRCOSC */
        SYSClock = CPU_INT_FAST_CLK_HZ;
        break;
    }

    if (validSystemClockSource == true)
    {
        SystemCoreClock = SYSClock;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SystemSoftwareReset
 * Description   : This function is used to initiate a 'functional' reset event
 * to the microcontroller. The reset module will do a state machine from
 * PHASE1->PHASE2->PHASE3->IDLE.
 *
 * Implements    : SystemSoftwareReset_Activity
 *END**************************************************************************/
void SystemSoftwareReset(void)
{
    /* Set SSR bit to generate a software internal system reset */
    SIU->SRCR |= SIU_SRCR_SSR_MASK;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
