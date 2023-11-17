/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : clockMan1.c
**     Project     : Boson_VCU_MPC5775B_Z7_0
**     Processor   : MPC5775B_416
**     Component   : clock_manager
**     Version     : Component SDK_S32_PA_15, Driver 01.00, CPU db: 3.00.000
**     Repository  : SDK_S32_PA_15
**     Compiler    : GNU C Compiler
**     Date/Time   : 2023-08-25, 11:46, # CodeGen: 53
**
**     Copyright 1997 - 2015 Freescale Semiconductor, Inc. 
**     Copyright 2016-2017 NXP 
**     All Rights Reserved.
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
** ###################################################################*/
/*!
** @file clockMan1.c
** @version 01.00
*/         
/*!
**  @addtogroup clockMan1_module clockMan1 module documentation
**  @{
*/         

/* clockMan1. */

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External variable could be made static.
 * The external variables will be used in other source files, with the same initialized values.
 */

#include "clockMan1.h"

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 9.4, Duplicate initialization of object element.
 * It's the only way to initialize an array that is member of struct.
 *
 */


/* *************************************************************************    
 * Configuration structure for Clock Configuration 0
 * ************************************************************************* */
/*! @brief User Configuration structure clock_managerCfg_0 */     
clock_manager_user_config_t clockMan1_InitConfig0 = {

    .siuConfig =
    {
        .SIU_CRC                  = false,                           /* CRC                               */
        .SIU_DSPIA                = true,                            /* DSPIA                             */
        .SIU_DSPIB                = true,                            /* DSPIB                             */
        .SIU_DSPIC                = true,                            /* DSPIC                             */
        .SIU_DSPID                = true,                            /* DSPID                             */
        .SIU_ENET                 = false,                           /* ENET                              */
        .SIU_FLEXCANA             = true,                            /* FLEXCANA                          */
        .SIU_FLEXCANB             = true,                            /* FLEXCANB                          */
        .SIU_FLEXCANC             = true,                            /* FLEXCANC                          */
        .SIU_FLEXCAND             = true,                            /* FLEXCAND                          */
        .SIU_EMIOS0               = true,                            /* EMIOS0                            */
        .SIU_EMIOS1               = true,                            /* EMIOS1                            */
        .SIU_CSE                  = false,                           /* CSE                               */
        .SIU_PSI                  = false,                           /* PSI                               */
        .SIU_ESCIA                = true,                            /* ESCIA                             */
        .SIU_ESCIB                = true,                            /* ESCIB                             */
        .SIU_ESCIC                = true,                            /* ESCIC                             */
        .SIU_ESCID                = true,                            /* ESCID                             */
        .SIU_ESCIE                = true,                            /* ESCIE                             */
        .SIU_ESCIF                = true,                            /* ESCIF                             */
        .SIU_PSI5A                = false,                           /* PSI5A                             */
        .SIU_PSI5B                = false,                           /* PSI5B                             */
        .SIU_DECFIL               = false,                           /* DECFIL                            */
        .SIU_ETPUC                = false,                           /* ETPUC                             */
        .SIU_NPC                  = false,                           /* NPC                               */
        .SIU_PIT                  = true,                            /* PIT                               */
        .SIU_STCU                 = false,                           /* STCU                              */
        .SIU_SRX0                 = false,                           /* SRX0                              */
        .SIU_SRX1                 = false,                           /* SRX1                              */
        .SIU_EQADCA               = true,                            /* EQADCA                            */
        .SIU_EQADCB               = true,                            /* EQADCB                            */
        .SIU_SDD                  = true,                            /* SDD                               */
        .SIU_SIPI                 = false,                           /* SIPI                              */
        .SIU_SDA                  = true,                            /* SDA                               */
        .SIU_SDB                  = true,                            /* SDB                               */
        .SIU_SDC                  = true,                            /* SDC                               */
        .SIU_MCANB                = true,                            /* MCANB                             */
        .SIU_EBI                  = false,                           /* EBI                               */
        .SIU_ETPUA                = false,                           /* ETPUA                             */
        .SIU_DSPIE                = true,                            /* DSPIE                             */
        .SIU_MCANA                = true,                            /* MCANA                             */
	
        .pll0Reference            = PLL_REFERENCE_XOSC,              /* PLL0 Reference                    */
        .pll1Reference            = PLL_REFERENCE_XOSC,              /* PLL1 Reference                    */
	
        .scs                      = SIU_SYSTEM_CLOCK_SRC_PLL0_PHI0,  /* System Clock Source               */
	
        .coreClk                  = SIU_CLOCK_DIV_BY_1,              /* Core Clock                        */
        .pbridgeClk               = SIU_CLOCK_DIV_BY_2,              /* Pbridge Clock                     */
        .perClkSel                = SIU_PER_CLK_SEL_CORE_CLK,        /* Peripheral Clock Selector         */
        .perClk                   = SIU_CLOCK_DIV_BY_2,              /* Peripheral Clock                  */
        .etpuClk                  = SIU_CLOCK_DIV_BY_1,              /* ETPU Clock                        */
        .adcsdClk                 = SIU_CLOCK_DIV_BY_50,             /* ADC Sigma Delta Clock             */
        .psi5Rx                   = SIU_CLOCK_DIV_BY_1,              /* PSI5 Clock                        */
        .psi5Rx1M                 = SIU_CLOCK_DIV_BY_1,              /* PSI5 1M Clock                     */
        .lfastSel                 = SIU_LFASTx_SEL_PER_CLK,          /* LFAST Selector Clock              */
        .lfastClk                 = SIU_CLOCK_DIV_BY_1,              /* LFAST Clock                       */
        .mcanSel                  = SIU_MCAN_CLK_SEL_XOSC_CLK,       /* CAN Selector Clock                */
	
        .clkout                   = SIU_CLOCK_DIV_BY_1,              /* Clockout                          */
        .engClkoutSel             = SIU_ENG_CLOCKOUT_XOSC_CLK,       /* Engineering Clockout Selector     */
        .engClkout                = SIU_CLOCK_DIV_BY_1,              /* Engineering Clockout              */
    },
        
    .clockSourcesConfig =
    {
        .xosc0Config =
        {
            .freq                 = 40000000,                        /* XOSC Frequency                    */
            .startupDelay         = 1,                               /* XOSC Startup Delay                */
            .bypassOption         = XOSC_USE_CRYSTAL,                /* Bypass Option                     */
        },
        .pll0Config =
        {
            .enable               = true,                            /* PLL0 Enabled                      */
            .predivider           = PLLDIG_CLOCK_PREDIV_BY_2,        /* PLL0 Predivider                   */
            .mulFactorDiv         = 20,                              /* PLL0 Multiplier                   */
            .phi0Divider          = PLLDIG_PHI_DIV_BY_2,             /* PLL0PHI0 Divider                  */
            .phi1Divider          = PLLDIG_PHI_DIV_BY_4,             /* PLL0PHI1 Divider                  */
        },
        .pll1Config =
        {
            .enable               = true,                            /* PLL1 Enabled                      */
            .mulFactorDiv         = 25,                              /* PLL1 Multiplier                   */
            .fracDivider          = true,                            /* PLL1 divider enable               */
            .fracDividerValue     = 0,                               /* PLL1 Divider Value                */
            .phi0Divider          = PLLDIG_PHI_DIV_BY_5,             /* PLL1PHI0 Divider                  */
            .modulation           = false,                           /* Modulation Enable                 */
            .modulationType       = CENTRE_SPREAD_MODULATION,        /* Modulation Type                   */
            .modulationPeriod     = 0,                               /* Modulation Period                 */
            .incrementStep        = 0,                               /* Modulation Increment Step         */
            .rectangularDitherControl            = false,            /* Rectangular Dither Enable         */
            .rectangularDitherControlValue       = 0,                /* Rectangular Dither Value          */
            .triangularDitherControl             = false,            /* Triangular Dither Enable          */
            .triangularDitherControlValue        = 0,                /* Triangular Dither Value           */
        },
	
        .sipiRefClkFreq0               = 0U,
	
    },
	
    .cmuConfig =
    {
        .cmu_rcdiv                = CMU_LO_FREQ_1,                   /* Lowest accepted frequency divider */

        .cmu = 
        {
            {
                .enable           = false,                           /* CMU_0 disabled                    */
                .lo_freq          = 0,                               /* Lowest accepted frequency         */
                .hi_freq          = 4095,                            /* Highest accepted frequency        */
            },
            {
                .enable           = false,                           /* CMU_1 disabled                    */
                .lo_freq          = 0,                               /* Lowest accepted frequency         */
                .hi_freq          = 4095,                            /* Highest accepted frequency        */
            },
            {
                .enable           = false,                           /* CMU_2 disabled                    */
                .lo_freq          = 0,                               /* Lowest accepted frequency         */
                .hi_freq          = 4095,                            /* Highest accepted frequency        */
            },
            {
                .enable           = false,                           /* CMU_3 disabled                    */
                .lo_freq          = 0,                               /* Lowest accepted frequency         */
                .hi_freq          = 4095,                            /* Highest accepted frequency        */
            },
            {
                .enable           = false,                           /* CMU_4 disabled                    */
                .lo_freq          = 0,                               /* Lowest accepted frequency         */
                .hi_freq          = 4095,                            /* Highest accepted frequency        */
            },
            {
                .enable           = false,                           /* CMU_5 disabled                    */
                .lo_freq          = 0,                               /* Lowest accepted frequency         */
                .hi_freq          = 4095,                            /* Highest accepted frequency        */
            },
            {
                .enable           = false,                           /* CMU_6 disabled                    */
                .lo_freq          = 0,                               /* Lowest accepted frequency         */
                .hi_freq          = 4095,                            /* Highest accepted frequency        */
            },
            {
                .enable           = false,                           /* CMU_7 disabled                    */
                .lo_freq          = 0,                               /* Lowest accepted frequency         */
                .hi_freq          = 4095,                            /* Highest accepted frequency        */
            },
            {
                .enable           = false,                           /* CMU_8 disabled                    */
                .lo_freq          = 0,                               /* Lowest accepted frequency         */
                .hi_freq          = 4095,                            /* Highest accepted frequency        */
            },
        },
    },
    
};        

/*! @brief Array of pointers to User configuration structures */
clock_manager_user_config_t const * g_clockManConfigsArr[] = {
    &clockMan1_InitConfig0
};
/*! @brief Array of pointers to User defined Callbacks configuration structures */
clock_manager_callback_user_config_t * g_clockManCallbacksArr[] = {(void*)0};
/* END clockMan1. */

/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the NXP C55 series of microcontrollers.
**
** ###################################################################
*/
