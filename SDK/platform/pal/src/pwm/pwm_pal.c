/*
 * Copyright 2017-2019 NXP
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
 * @file pwm_pal.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. A source of error in
 * writing dynamic code is that the stack segment may be different from the data
 * segment.
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.7, Symbol not referenced
 * These symbols are referenced only for some platforms.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, Could define variable at block scope
 * The variable must be global because it can't be allocated on stack.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, essential type
 * Expression assigned to a narrower or different essential type.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.1, Unpermitted operand to operator.
 * This is required to get the right returned status from the
 * initialization function.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.7, other operand.
 * Composite expression with smaller essential type than other operand.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from
 * 'essentially enum<i>' to 'essentially enum<i>'.
 * The cast is performed in order to map the values from the peripheral abstraction
 * layer to the types used in the driver.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.3, Cast performed between a pointer to object type
 * and a pointer to a different object type.
 * This is needed for the extension of the user configuration structure, for which the actual type
 * cannot be known.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.5, Conversion from pointer to void to pointer to other type
 * This is needed for the extension of the user configuration structure, for which the actual type
 * cannot be known.
 *
 */

#include "pwm_pal.h"

/* Include ETIMER special functions. */
#if (defined (PWM_OVER_ETIMER))
/* nothing here */
#endif

/* Include EMIOS special functions. */
#if (defined (PWM_OVER_EMIOS))
#if (FEATURE_PWMPAL_EMIOS_HAS_CHANNEL_MAPPING)
    static const uint8_t s_VrChannelMapping[FEATURE_EMIOS_CH_COUNT] = PWMPAL_INDEX_2_HW_CHANNELS;
#endif
    static status_t PWM_EMIOS_VrChannelMapping(uint8_t vrChannel, uint8_t * hwChannel);
    static bool PWM_EMIOS_ValidateChannel(uint8_t inChVal, uint8_t * outChVal);
#endif

/* Define state structures for FTM */
#if (defined(PWM_OVER_FTM))
    /*! @brief FTM state structures */
    static ftm_state_t FtmState[NO_OF_FTM_INSTS_FOR_PWM];
    /*! @brief FTM state-instance matching */
    static uint32_t FtmStateInstanceMapping[NO_OF_FTM_INSTS_FOR_PWM];
    /*! @brief FTM  available resources table */
    static bool FtmStateIsAllocated[NO_OF_FTM_INSTS_FOR_PWM];
    /*! @brief Store the flag of channel is used in combined PWM signal */
    static bool pwmPalCombChnFlag[FTM_INSTANCE_COUNT][FEATURE_FTM_CHANNEL_COUNT];


/*FUNCTION**********************************************************************
 *
 * Function Name : PwmAllocateState
 * Description   : Allocates one of the available state structure.
 *
 *END**************************************************************************/
static uint8_t PwmAllocateState(bool* isAllocated, uint32_t* instanceMapping, const pwm_instance_t * const instance, uint8_t numberOfinstances)
{
    uint8_t i;
    /* Allocate one of the FTM state structure for this instance */
    for (i = 0;i < numberOfinstances;i++)
    {
        if (isAllocated[i] == false)
        {
            instanceMapping[i] = instance->instIdx;
            isAllocated[i] = true;
            break;
        }
    }
    return i;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PwmFreeState
 * Description   : Deallocates one of the available state structure.
 *
 *END**************************************************************************/
static void PwmFreeState(bool* isAllocated, const uint32_t* instanceMapping, const pwm_instance_t * const instance, uint8_t numberOfinstances)
{
    uint8_t i;
    /* Allocate one of the FTM state structure for this instance */
    for (i = 0;i < numberOfinstances;i++)
    {
        if (instanceMapping[i] == instance->instIdx)
        {
            isAllocated[i] = false;
            break;
        }
    }
}
#endif

#if (defined(PWM_OVER_FTM))
static inline status_t PWM_FTM_Init(const pwm_instance_t * const instance, const pwm_global_config_t* config)
{
    /* Declare internal variables */
    uint8_t channel;
    uint8_t channelID;
    uint8_t index;
    uint32_t frequency;
    uint32_t dutyPercent;
    status_t status = STATUS_ERROR;
    
    DEV_ASSERT(instance->instIdx < FTM_INSTANCE_COUNT);
    /* Declare the configuration structure for FTM */
    ftm_pwm_param_t ftmPwmConfig;
    ftm_independent_ch_param_t pwmIndependentChannelConfig[FEATURE_FTM_CHANNEL_COUNT];
    ftm_combined_ch_param_t pwmCombinedChannelConfig[FEATURE_FTM_CHANNEL_COUNT >> 1U];
    ftm_user_config_t ftmGlobalConfig;
#if (FTM_FEATURE_FAULT_CHANNELS > 1U)
    /* Fault control feature is not supported, but this structure is filled with values which disable fault control */
    ftm_pwm_fault_param_t faultConfig =
    {
        false, /* Output pin state on fault */
        false, /* PWM fault interrupt state */
        0U, /* Fault filter value */
        FTM_FAULT_CONTROL_DISABLED,  /* Fault mode */
        {
            {
                false, /* Fault channel state (Enabled/Disabled) */
                false, /* Fault channel filter state (Enabled/Disabled) */
                FTM_POLARITY_LOW, /* Fault channel state (Enabled/Disabled) */
            },
            {
                false, /* Fault channel state (Enabled/Disabled) */
                false, /* Fault channel filter state (Enabled/Disabled) */
                FTM_POLARITY_LOW, /* Fault channel state (Enabled/Disabled) */
            },
            {
                false, /* Fault channel state (Enabled/Disabled) */
                false, /* Fault channel filter state (Enabled/Disabled) */
                FTM_POLARITY_LOW, /* Fault channel state (Enabled/Disabled) */
            },
            {
                false, /* Fault channel state (Enabled/Disabled) */
                false, /* Fault channel filter state (Enabled/Disabled) */
                FTM_POLARITY_LOW, /* Fault channel state (Enabled/Disabled) */
            }
        }
    };
    ftmPwmConfig.faultConfig = &faultConfig;
#endif
    ftmPwmConfig.pwmCombinedChannelConfig = pwmCombinedChannelConfig;
    ftmPwmConfig.pwmIndependentChannelConfig = pwmIndependentChannelConfig;

    /* Because FTM has only one timebase first channel is used to configure FTM clocking*/
    ftmGlobalConfig.ftmClockSource = ((pwm_ftm_timebase_t*)(config->pwmChannels[0].timebase))->sourceClock;
    ftmGlobalConfig.ftmPrescaler = ((pwm_ftm_timebase_t*)(config->pwmChannels[0].timebase))->prescaler;
    ftmPwmConfig.deadTimePrescaler = ((pwm_ftm_timebase_t*)(config->pwmChannels[0].timebase))->deadtimePrescaler;

    /* Configure FTM mode according to first channel setup.
    * All PWM channels must be PWM_EDGE_ALIGNED/PWM_Shifted or PWM_CENTER_ALIGNED */
    if ((config->pwmChannels[0].channelType) == PWM_CENTER_ALIGNED)
    {
        ftmPwmConfig.mode = FTM_MODE_CEN_ALIGNED_PWM;
    }
    else
    {
        ftmPwmConfig.mode = FTM_MODE_EDGE_ALIGNED_PWM;
    }
    ftmGlobalConfig.ftmMode =  ftmPwmConfig.mode;

    /* The synchronization for duty, period and phase shift will be update when
    * signal period is done. Only overwrite function shall take effect immediate.
    */
    ftmGlobalConfig.syncMethod.softwareSync = true;
    ftmGlobalConfig.syncMethod.hardwareSync1 = false;
    ftmGlobalConfig.syncMethod.hardwareSync2 = false;
    ftmGlobalConfig.syncMethod.hardwareSync0 = false;
    ftmGlobalConfig.syncMethod.autoClearTrigger = false;
    ftmGlobalConfig.syncMethod.maskRegSync = FTM_SYSTEM_CLOCK;
    ftmGlobalConfig.syncMethod.initCounterSync = FTM_PWM_SYNC;
    ftmGlobalConfig.syncMethod.inverterSync = FTM_PWM_SYNC;
    ftmGlobalConfig.syncMethod.outRegSync = FTM_SYSTEM_CLOCK;
    ftmGlobalConfig.syncMethod.maxLoadingPoint = true;
    ftmGlobalConfig.syncMethod.minLoadingPoint = false;
    ftmGlobalConfig.syncMethod.syncPoint = FTM_WAIT_LOADING_POINTS;
    ftmGlobalConfig.isTofIsrEnabled = false;
    ftmGlobalConfig.BDMMode = FTM_BDM_MODE_00;

    /* Configure FTM channels */
    ftmPwmConfig.nNumCombinedPwmChannels = 0;
    ftmPwmConfig.nNumIndependentPwmChannels = 0;

    /* Configure dead-time insertion. For them the dead time configuration is available for all channels. */
    ftmPwmConfig.deadTimePrescaler = ((pwm_ftm_timebase_t*)(config->pwmChannels[0].timebase))->deadtimePrescaler;
    ftmPwmConfig.deadTimeValue = config->pwmChannels[0].deadtime;

    for (channel = 0; channel < config->numberOfPwmChannels; channel++)
    {
        if (config->pwmChannels[channel].enableComplementaryChannel == false)
        {
            /* Configure duty cycle in percents */
            dutyPercent = (FTM_MAX_DUTY_CYCLE * config->pwmChannels[channel].duty) / config->pwmChannels[channel].period;
            pwmIndependentChannelConfig[ftmPwmConfig.nNumIndependentPwmChannels].uDutyCyclePercent = (uint16_t)dutyPercent;

            /* Configure channel number */
            pwmIndependentChannelConfig[ftmPwmConfig.nNumIndependentPwmChannels].hwChannelId = config->pwmChannels[channel].channel;

            /* Configure channel polarity */
            if (config->pwmChannels[channel].polarity == PWM_ACTIVE_HIGH)
            {
                pwmIndependentChannelConfig[ftmPwmConfig.nNumIndependentPwmChannels].polarity = FTM_POLARITY_HIGH;
                pwmIndependentChannelConfig[ftmPwmConfig.nNumIndependentPwmChannels].safeState = FTM_LOW_STATE;
            }
            else
            {
                pwmIndependentChannelConfig[ftmPwmConfig.nNumIndependentPwmChannels].polarity = FTM_POLARITY_LOW;
                pwmIndependentChannelConfig[ftmPwmConfig.nNumIndependentPwmChannels].safeState = FTM_LOW_STATE;
            }

            /* Configure default value for fail safe value. */
            pwmIndependentChannelConfig[ftmPwmConfig.nNumIndependentPwmChannels].enableExternalTrigger = false;
            pwmIndependentChannelConfig[ftmPwmConfig.nNumIndependentPwmChannels].deadTime = false;
            pwmIndependentChannelConfig[ftmPwmConfig.nNumIndependentPwmChannels].enableSecondChannelOutput = false;

            ftmPwmConfig.nNumIndependentPwmChannels++;
        }
        else
        {
            /* Configure duty cycle in percents */
            dutyPercent = (FTM_MAX_DUTY_CYCLE * config->pwmChannels[channel].duty) / config->pwmChannels[channel].period;
            pwmCombinedChannelConfig[ftmPwmConfig.nNumCombinedPwmChannels].firstEdge = 0;
            pwmCombinedChannelConfig[ftmPwmConfig.nNumCombinedPwmChannels].secondEdge = (uint16_t)dutyPercent;
#if FEATURE_FTM_HAS_MODIFIED_COMBINE_MODE
            pwmCombinedChannelConfig[ftmPwmConfig.nNumCombinedPwmChannels].enableModifiedCombine = false;
#endif
            pwmCombinedChannelConfig[ftmPwmConfig.nNumCombinedPwmChannels].enableSecondChannelOutput = true;

            /* Configure channel number */
            pwmCombinedChannelConfig[ftmPwmConfig.nNumCombinedPwmChannels].hwChannelId = config->pwmChannels[channel].channel;

            if (config->pwmChannels[channel].polarity == PWM_ACTIVE_HIGH)
            {
                pwmCombinedChannelConfig[ftmPwmConfig.nNumCombinedPwmChannels].mainChannelPolarity = FTM_POLARITY_HIGH;
                pwmCombinedChannelConfig[ftmPwmConfig.nNumCombinedPwmChannels].mainChannelSafeState = FTM_LOW_STATE;
                pwmCombinedChannelConfig[ftmPwmConfig.nNumCombinedPwmChannels].secondChannelSafeState = FTM_LOW_STATE;

            }
            else
            {
                pwmCombinedChannelConfig[ftmPwmConfig.nNumCombinedPwmChannels].mainChannelPolarity = FTM_POLARITY_LOW;
                pwmCombinedChannelConfig[ftmPwmConfig.nNumCombinedPwmChannels].mainChannelSafeState = FTM_LOW_STATE;
                pwmCombinedChannelConfig[ftmPwmConfig.nNumCombinedPwmChannels].secondChannelSafeState = FTM_LOW_STATE;
            }

            if (config->pwmChannels[channel].complementaryChannelPolarity == PWM_DUPLICATED)
            {
                pwmCombinedChannelConfig[ftmPwmConfig.nNumCombinedPwmChannels].secondChannelPolarity = FTM_MAIN_DUPLICATED;
            }
            else
            {
                pwmCombinedChannelConfig[ftmPwmConfig.nNumCombinedPwmChannels].secondChannelPolarity = FTM_MAIN_INVERTED;
            }
            /* Configure default value for fail safe value. */
            pwmCombinedChannelConfig[ftmPwmConfig.nNumCombinedPwmChannels].enableExternalTrigger = false;
            pwmCombinedChannelConfig[ftmPwmConfig.nNumCombinedPwmChannels].deadTime = config->pwmChannels[channel].insertDeadtime;
            pwmCombinedChannelConfig[ftmPwmConfig.nNumCombinedPwmChannels].enableExternalTriggerOnNextChn = false;
            ftmPwmConfig.nNumCombinedPwmChannels++;

            /* Set the flag as true when it is configured in combined channel */
            channelID = config->pwmChannels[channel].channel;
            pwmPalCombChnFlag[instance->instIdx][channelID] = true;
        }
    }
    /* Initialize FTM */
    index = PwmAllocateState(FtmStateIsAllocated, FtmStateInstanceMapping, instance, NO_OF_FTM_INSTS_FOR_PWM);
    /* No permit if all elements in PWM_PAL available resources table are chosen */
    DEV_ASSERT(index < NO_OF_FTM_INSTS_FOR_PWM);
    (void)FTM_DRV_Init(instance->instIdx, &ftmGlobalConfig, (ftm_state_t*)(&FtmState[index]));
    /* Configure PWM frequency */
    frequency = FTM_DRV_GetFrequency(instance->instIdx);
    ftmPwmConfig.uFrequencyHZ = frequency / config->pwmChannels[0].period;
    /* Start PWM signal generation */
    status = FTM_DRV_InitPwm(instance->instIdx, &ftmPwmConfig);
	
	return status;
}
#endif

#if (defined(PWM_OVER_EMIOS))
static inline status_t PWM_EMIOS_Init(const pwm_instance_t * const instance, const pwm_global_config_t* config)
{
    status_t status = STATUS_ERROR;
    uint8_t hwChannel = 0U;

    /* Configure the default configuration for eMIOS */
    emios_common_param_t pwmGlobalConfig =
    {
        false, /*!< If true, all channel in eMIOS group can enter debug mode */
        false, /*!< Low power mode or normal mode */
        1U, /*!< Select the clock divider value for the global prescaler in range (1-256) */
        false, /*!< Enable or disable global prescaler */
        false, /*!< Enable or disable global timebase */
#if defined(FEATURE_EMIOS_STAC_CLIENT)
        false,             /* Enable external timebase or disable */
        0U                 /* Select the address of a specific STAC server to which the STAC is assigned */
#endif
    };

    /* Initialize global configuration of the EMIOS. */
    EMIOS_DRV_InitGlobal((uint8_t)instance->instIdx, &pwmGlobalConfig);

    /* Default timebase setup which will be changed accroding to config structure */
    emios_mc_mode_param_t pwmTimebaseConfig =
    {
        EMIOS_MODE_MCB_UP_COUNTER_EXT_CLK,
        10UL, /*!< Period value */
        EMIOS_CLOCK_DIVID_BY_1, /*!< Internal prescaler value */
        false, /*!< Enable internal prescaler */
        EMIOS_INPUT_FILTER_BYPASS, /*!< Filter value, ignore if not select external clock mode */
        false, /*!< Input capture filter state, ignore if not select external clock mode */
        EMIOS_TRIGGER_EDGE_FALLING /*!< Input signal trigger mode, ignore if not select external clock mode */
    };

    emios_pwm_param_t pwmChannelConfig=
    {
        EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX1,
        EMIOS_CLOCK_DIVID_BY_1, /*!< Internal prescaler value */
        true, /*!< Internal prescaler is enabled */
        EMIOS_NEGATIVE_PULSE, /*!< Output active value, active low or high level */
        10000UL, /*!< Period count */
        5000UL, /*!< Duty cycle count */
        EMIOS_BUS_SEL_BCDE, /*!< Counter bus selected */
        1UL, /*!< Ideal duty cycle value using to compare with the selected time base */
        0UL, /*!< The dead time value and is compared against the internal counter */
#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
        0UL /*!< Trigger Event placement */
#endif
    };

    /* Configure all timebases */
    uint8_t i;
    for (i = 0; i < config->numberOfPwmChannels; i++)
    {
        /* Configure timebase counter mode. */
        if(config->pwmChannels[i].channelType == PWM_EDGE_ALIGNED)
        {
            pwmTimebaseConfig.mode = EMIOS_MODE_MCB_UP_COUNTER_INT_CLK;
            pwmChannelConfig.mode = EMIOS_MODE_OPWMB_FLAGX1;

            /* Configure timebase period. */
            pwmTimebaseConfig.period = config->pwmChannels[i].period;
        }
        else
        {
            pwmTimebaseConfig.mode = EMIOS_MODE_MCB_UPDOWN_CNT_FLAGX1_INT_CLK;
            pwmChannelConfig.mode = EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX1;

            if(config->pwmChannels[i].insertDeadtime){
                /* Default mode is pwmChannelConfig.mode = EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX1;*/
                if(config->pwmChannels[i].deadtimeEdge == PWM_DEADTIME_LEAD_EDGE){
                    pwmChannelConfig.mode = EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX1;
                }
                pwmChannelConfig.deadTime = config->pwmChannels[i].deadtime;
            }

            /* Configure timebase period. */
            pwmTimebaseConfig.period = config->pwmChannels[i].period;
        }

        /* Configure timebase divider. */
        pwmTimebaseConfig.internalPrescalerEn = true;
        pwmTimebaseConfig.internalPrescaler = ((pwm_emios_timebase_t*)(config->pwmChannels[i].timebase))->internalPrescaler;
        pwmChannelConfig.internalPrescalerEn = true;
        pwmChannelConfig.internalPrescaler = config->pwmChannels[i].deadtimePrescaler;


        /* Initialize the counter mode */
        status = EMIOS_DRV_MC_InitCounterMode((uint8_t)(instance->instIdx),((pwm_emios_timebase_t*)(config->pwmChannels[i].timebase))->name,&pwmTimebaseConfig);
        if(STATUS_SUCCESS != status)
        {
            DEV_ASSERT(false);
        }

        /* Select timebase for current channel */
        if (((pwm_emios_timebase_t*)(config->pwmChannels[i].timebase))->name == EMIOS_CNT_BUSA_DRIVEN)
        {
            pwmChannelConfig.timebase = EMIOS_BUS_SEL_A;
        }
#if FEATURE_EMIOS_BUS_F_SELECT
        else if (((pwm_emios_timebase_t*)(config->pwmChannels[i].timebase))->name == EMIOS_CNT_BUSF_DRIVEN)
        {
            pwmChannelConfig.timebase = EMIOS_BUS_SEL_F;
        }
#endif
        else
        {
            pwmChannelConfig.timebase = EMIOS_BUS_SEL_BCDE;
        }

        /* Select channel polarity */
        if (config->pwmChannels[i].polarity == PWM_ACTIVE_HIGH)
        {
            pwmChannelConfig.outputActiveMode = EMIOS_POSITIVE_PULSE;
            /* Configure duty */
            pwmChannelConfig.dutyCycleCount = config->pwmChannels[i].duty;
            pwmChannelConfig.idealDutyCycle = config->pwmChannels[i].duty;
        }
        else
        {
            pwmChannelConfig.outputActiveMode = EMIOS_NEGATIVE_PULSE;
            /* Configure duty */
            if (config->pwmChannels[i].duty <= config->pwmChannels[i].period)
            {
                pwmChannelConfig.dutyCycleCount = config->pwmChannels[i].period - config->pwmChannels[i].duty;
                pwmChannelConfig.idealDutyCycle = pwmChannelConfig.dutyCycleCount;
            }
            else
            {
                pwmChannelConfig.dutyCycleCount = 0UL;
                pwmChannelConfig.idealDutyCycle = 0UL;
            }
        }

        /* Get Virtual Channel from Hardware Channel */
        status = PWM_EMIOS_VrChannelMapping(config->pwmChannels[i].channel, &hwChannel);
        if(STATUS_SUCCESS != status)
        {
            DEV_ASSERT(false);
        }

        /* Configure channel */
        (void)EMIOS_DRV_PWM_InitMode((uint8_t)(instance->instIdx),(uint8_t)hwChannel, &pwmChannelConfig);
    }
    EMIOS_DRV_EnableGlobalEmios((uint8_t)(instance->instIdx));
    status = STATUS_SUCCESS;
    
    return status;
}
#endif
    
#if (defined (PWM_OVER_ETIMER))
static inline status_t PWM_ETIMER_Init(const pwm_instance_t * const instance, const pwm_global_config_t* config)
{
    status_t status = STATUS_ERROR;
    /* intial sanity check */
    DEV_ASSERT(config->numberOfPwmChannels < ETIMER_CH_COUNT);

    /* default configuration for ETIMER for PWM mode */
    etimer_user_channel_config_t etimerPwmPalCfg ;
    ETIMER_DRV_GetDefaultChannelConfigVariableFreqPwm(&etimerPwmPalCfg);
    etimerPwmPalCfg.compareValues[0] = 32767;
    etimerPwmPalCfg.compareValues[1] = 65535;
    etimerPwmPalCfg.primaryInput.source = ETIMER_IN_SRC_CLK_DIV_32;
    etimerPwmPalCfg.compareLoading[0] = ETIMER_CLC_FROM_CMPLD1_WHEN_COMP2;
    etimerPwmPalCfg.compareLoading[1] = ETIMER_CLC_FROM_CMPLD2_WHEN_COMP2;

    /* initialize ETIMERx peripheral */
    ETIMER_DRV_Init(instance->instIdx);
    /* Configure all timebases */
    uint8_t i;
    uint16_t enmask=0;
    for (i = 0; i < config->numberOfPwmChannels; i++)
    {
        /* sanity checks */
        DEV_ASSERT(config->pwmChannels[i].channelType == PWM_EDGE_ALIGNED);
        DEV_ASSERT(config->pwmChannels[i].enableComplementaryChannel == false);
        DEV_ASSERT(config->pwmChannels[i].insertDeadtime == false);
        DEV_ASSERT(config->pwmChannels[i].period >= config->pwmChannels[i].duty);
        DEV_ASSERT(config->pwmChannels[i].duty <= 0xFFFFu);
        DEV_ASSERT((config->pwmChannels[i].period - config->pwmChannels[i].duty) <= 0xFFFFu);
        /* double frequency ? */
        if( (((pwm_etimer_timebase_t *)(config->pwmChannels[i].timebase))->halfClkPeriod) == true )
        {
            etimerPwmPalCfg.countMode = ETIMER_CNTMODE_PRIMARY_BOTH_EDGES;
        }
        else
        {
            etimerPwmPalCfg.countMode = ETIMER_CNTMODE_PRIMARY;
        }
        /* clock source */
        etimerPwmPalCfg.primaryInput.polarity = ((pwm_etimer_timebase_t *)(config->pwmChannels[i].timebase))->pwmClkSrc.polarity;
        etimerPwmPalCfg.primaryInput.source = ((pwm_etimer_timebase_t *)(config->pwmChannels[i].timebase))->pwmClkSrc.source;
        /* enable output ? */
        if( (((pwm_etimer_timebase_t *)(config->pwmChannels[i].timebase))->enableOutSig) == true )
        {
            etimerPwmPalCfg.outputPin.enable = true;
        }
        else
        {
            etimerPwmPalCfg.outputPin.enable = false;
        }
        /* output polarity */
        /* eTimer ETIMER_POLARITY_POSITIVE has the first output the low period, then the high
         * it is opposite of the PWM PAL ACTIVE_HIGH and ACTIVE_LOW concepts
         */
        if(config->pwmChannels[i].polarity == PWM_ACTIVE_HIGH)
        {
            etimerPwmPalCfg.outputPin.polarity = ETIMER_POLARITY_NEGATIVE;
        }
        else
        {
            etimerPwmPalCfg.outputPin.polarity = ETIMER_POLARITY_POSITIVE;
        }
        /* COMP1 and COMP2 */
        uint16_t ton = config->pwmChannels[i].duty;
        if (ton == 0u)
        {
            /* duty 0% is not supported */
            ton = 1u;
        }
        else if (ton == config->pwmChannels[i].period)
        {
            /* duty 100% is not supported */
            ton = config->pwmChannels[i].period - 1u;
        }
        else
        {
            /* ton does not need any adjustment */
        }
        uint16_t toff = config->pwmChannels[i].period - ton;
        etimerPwmPalCfg.compareValues[0] = ton - 1u;
        etimerPwmPalCfg.compareValues[1] = toff - 1u;
        /* setup ETIMERx channels */
        ETIMER_DRV_InitChannel(instance->instIdx, config->pwmChannels[i].channel, &etimerPwmPalCfg);
        /* mask the enable bit */
        enmask|=(1UL<<(config->pwmChannels[i].channel));
    }
    /* start channel operation */
    ETIMER_DRV_StartTimerChannels(instance->instIdx, enmask);
    /* all ok */
    status = STATUS_SUCCESS;
    
    return status;
}
#endif   

#if (defined (PWM_OVER_FLEXPWM))
static inline status_t PWM_FLEXPWM_Init(const pwm_instance_t * const instance, const pwm_global_config_t* config)
{
    status_t status = STATUS_ERROR;
    
    flexpwm_module_setup_t flexpwm_channel_settings =
    {
        .cntrInitSel = InitSrcLocalSync,
        .clkSrc = ClkSrcPwmPeriphClk,
        .prescaler = PwmDividedBy1,
        .clockFreq = 0U,
        .chnlPairOper = FlexPwmIndependent,
        .complementarySourceSel = FlexPwmComplementarySource23,
        .reloadLogic = FlexPwmReloadImmediate,
        .reloadSource = FLEXPWM_LOCAL_RELOAD_SIGNAL,
        .reloadFreq = PwmLoadEvery1Oportunity,
        .forceTrig = ForceOutputLocalForce
    };
    flexpwm_module_signal_setup_t flexpwm_signal_settings =
    {
        .pwmPeriod = 10000U,
        .flexpwmType = FlexPwmEdgeAligned,
        .pwmAPulseWidth = 5000U,
        .pwmBPulseWidth = 0U,
        .pwmAShift = 0U,
        .pwmBShift = 0U,
        .pwmAOuten = true,
        .pwmBOuten = false,
        .pwmXOuten = false,
        .pwmAPolarity = false,
        .pwmBPolarity = false,
        .pwmXPolarity = false,
        .pwmAFaultDisableMask = 0U,
        .pwmAfaultState = FLEXPWM_OUTPUT_STATE_LOGIC_0,
        .pwmBFaultDisableMask = 0U,
        .pwmBfaultState = FLEXPWM_OUTPUT_STATE_LOGIC_0,
        .pwmXFaultDisableMask = 0U,
        .pwmXfaultState = FLEXPWM_OUTPUT_STATE_LOGIC_0
    };

    /* inItial sanity check */
    DEV_ASSERT(config->numberOfPwmChannels <= FlexPWM_SUB_COUNT);

    /* Configure all channels */
    uint8_t i;
    for (i = 0; i < config->numberOfPwmChannels; i++)
    {
        /* sanity checks */
        DEV_ASSERT(config->pwmChannels[i].channelType == PWM_EDGE_ALIGNED);
        DEV_ASSERT(config->pwmChannels[i].period >= config->pwmChannels[i].duty);
        DEV_ASSERT(config->pwmChannels[i].duty <= 0xFFFFu);
        DEV_ASSERT((config->pwmChannels[i].period - config->pwmChannels[i].duty) <= 0xFFFFu);

        flexpwm_signal_settings.pwmPeriod = config->pwmChannels[i].period;
        flexpwm_signal_settings.pwmAPulseWidth = config->pwmChannels[i].duty;
        flexpwm_signal_settings.pwmAPolarity = config->pwmChannels[i].polarity;

        flexpwm_channel_settings.clkSrc = ((pwm_flexpwm_timebase_t *)(config->pwmChannels[i].timebase))->pwmClkSrc;
        flexpwm_channel_settings.prescaler = ((pwm_flexpwm_timebase_t *)(config->pwmChannels[i].timebase))->pwmClkPrsc;

        if(config->pwmChannels[i].insertDeadtime){
            flexpwm_channel_settings.chnlPairOper = FlexPwmComplementary;
        }

        FLEXPWM_DRV_SetupPwm(instance->instIdx, (flexpwm_module_t)(config->pwmChannels[i].channel),
                             &flexpwm_channel_settings, &flexpwm_signal_settings);

        if(config->pwmChannels[i].insertDeadtime){
            FLEXPWM_DRV_SetDeadtime(instance->instIdx, (flexpwm_module_t)(config->pwmChannels[i].channel),
                    config->pwmChannels[i].deadtime,FLEXPWM_DEADTIME_COUNTER_0);
        }

        FLEXPWM_DRV_CounterStart(instance->instIdx, (flexpwm_module_t)(config->pwmChannels[i].channel));

    }
    /* all ok */
    status = STATUS_SUCCESS;
    
    return status;
}
#endif
 
/*FUNCTION**********************************************************************
 *
 * Function Name : PWM_Init
 * Description   : Initialize PWM channels based on config parameter.
 *
 * Implements    : PWM_Init_Activity
 *END**************************************************************************/
status_t PWM_Init(const pwm_instance_t * const instance, const pwm_global_config_t* config)
{
    DEV_ASSERT(instance != NULL);
    DEV_ASSERT(config != NULL);
#if (defined(DEV_ERROR_DETECT) || defined(CUSTOM_DEVASSERT))
    {
        uint8_t idx;
        
        for (idx = 0; idx < config->numberOfPwmChannels; idx++)
        {
            DEV_ASSERT(config->pwmChannels[idx].duty <= config->pwmChannels[idx].period);
        }
    }
#endif

    status_t status = STATUS_ERROR;

    #if (defined(PWM_OVER_FTM))
    if(instance->instType == PWM_INST_TYPE_FTM)
    {
        status = PWM_FTM_Init(instance, config);
    }
    else
    #endif

    #if (defined(PWM_OVER_EMIOS))
    if (instance->instType == PWM_INST_TYPE_EMIOS)
    {
        status = PWM_EMIOS_Init(instance, config);
    }
    else
    #endif

    #if (defined (PWM_OVER_ETIMER))
    if (instance->instType == PWM_INST_TYPE_ETIMER)
    {
        status = PWM_ETIMER_Init(instance, config);
    }
    else
    #endif

    #if (defined (PWM_OVER_FLEXPWM))
    if (instance->instType == PWM_INST_TYPE_FLEXPWM)
    {
        status = PWM_FLEXPWM_Init(instance, config);
    }
    else
    #endif

    {
        DEV_ASSERT(false);
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PWM_UpdateDuty
 * Description   : Update duty cycle. The measurement unit for duty is clock ticks.
 * Implements    : PWM_UpdateDuty_Activity
 *
 *END**************************************************************************/
status_t PWM_UpdateDuty(const pwm_instance_t * const instance, uint8_t channel, uint32_t duty)
{
    DEV_ASSERT(instance != NULL);
    status_t status = STATUS_ERROR;

    #if (defined(PWM_OVER_FTM))
    if (instance->instType == PWM_INST_TYPE_FTM)
    {
        DEV_ASSERT(instance->instIdx < FTM_INSTANCE_COUNT);
        if (pwmPalCombChnFlag[instance->instIdx][channel] == true)
        {
            DEV_ASSERT(((uint32_t)channel % (uint32_t)2U) == (uint32_t)0U);
            status = FTM_DRV_UpdatePwmChannel(instance->instIdx, channel, FTM_PWM_UPDATE_IN_TICKS, 0, (uint16_t)duty, true);
        }
        else
        {
            status = FTM_DRV_UpdatePwmChannel(instance->instIdx, channel, FTM_PWM_UPDATE_IN_TICKS, (uint16_t)duty, 0, true);
        }
    }
    else
    #endif

    #if (defined(PWM_OVER_EMIOS))
    if (instance->instType == PWM_INST_TYPE_EMIOS)
    {
        uint8_t hwChannel        = 0U;
        uint32_t tmpDuty         = 0U;
        uint8_t busSelect        = 0U;
        uint8_t restChannel      = 0U;
        bool restValidateChannel = false;

        /* Get hardware Channel from virtual Channel */
        status = PWM_EMIOS_VrChannelMapping(channel, &hwChannel);
        /* Validate channel support */
        restValidateChannel = PWM_EMIOS_ValidateChannel(hwChannel, &restChannel);
        if((STATUS_SUCCESS != status) || (restValidateChannel == false))
        {
            DEV_ASSERT(false);
        }

        if (((eMIOS[instance->instIdx]->UC[restChannel].C & eMIOS_C_BSL_MASK) >> eMIOS_C_BSL_SHIFT) == (uint32_t)EMIOS_BUS_SEL_A)
		{
			busSelect = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
		}
#if FEATURE_EMIOS_BUS_F_SELECT
		else if (((eMIOS[instance->instIdx]->UC[restChannel].C & eMIOS_C_BSL_MASK) >> eMIOS_C_BSL_SHIFT) == (uint32_t)EMIOS_BUS_SEL_F)
		{
			busSelect = (uint8_t)EMIOS_CNT_BUSF_DRIVEN;
		}
#endif
		else if (((eMIOS[instance->instIdx]->UC[restChannel].C & eMIOS_C_BSL_MASK) >> eMIOS_C_BSL_SHIFT) == (uint32_t)EMIOS_BUS_SEL_BCDE)
		{
			busSelect = (uint8_t)(channel & 0xF8U);
		}
		else
		{
			DEV_ASSERT(false);
		}

        if ((eMIOS[instance->instIdx]->UC[restChannel].C & eMIOS_C_MODE_MASK) == (uint32_t)EMIOS_MODE_OPWMB_FLAGX1)
        {
        	if (((eMIOS[instance->instIdx]->UC[restChannel].C & eMIOS_C_EDPOL_MASK) >> eMIOS_C_EDPOL_SHIFT) == (uint32_t)EMIOS_NEGATIVE_PULSE)
			{
        		/* new duty cycle */
        		if (duty <= EMIOS_DRV_MC_GetCounterPeriod(instance->instIdx, busSelect))
        		{
        			tmpDuty = EMIOS_DRV_MC_GetCounterPeriod(instance->instIdx, busSelect) - duty; /* tmpDuty = 0UL =>  100% duty cycle */
        		}
        		else
        		{
        			tmpDuty = 0UL; /* 100% duty cycle */
        		}
			}
			else
			{
				/* new duty cycle */
				tmpDuty = duty; /* tmpDuty = period => 100% duty cycle */
			}
        }
        else /* EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX1 */
        {
        	if (((eMIOS[instance->instIdx]->UC[restChannel].C & eMIOS_C_EDPOL_MASK) >> eMIOS_C_EDPOL_SHIFT) == (uint32_t)EMIOS_NEGATIVE_PULSE)
			{
        		/* new duty cycle */
        		if (duty < EMIOS_DRV_MC_GetCounterPeriod(instance->instIdx, busSelect))
        		{
        			tmpDuty = EMIOS_DRV_MC_GetCounterPeriod(instance->instIdx, busSelect) - duty;
        		}
        		else
        		{
        			tmpDuty = 0UL; /* 100% duty cycle */
        		}
			}
			else
			{
				/* new duty cycle */
				tmpDuty = duty; /* tmpDuty = period => 100% duty cycle */
			}
        }

        status = EMIOS_DRV_PWM_SetDutyCycle(instance->instIdx, hwChannel, tmpDuty);

        if(STATUS_SUCCESS != status)
        {
            DEV_ASSERT(false);
        }

        status = STATUS_SUCCESS;
    }
    else
    #endif

    #if (defined (PWM_OVER_ETIMER))
    if (instance->instType == PWM_INST_TYPE_ETIMER)
    {
        /* read current compare values */
        uint16_t comp1;
        uint16_t comp2;
        ETIMER_DRV_GetCompareThresholdBuffered(instance->instIdx, channel, &comp1, &comp2);

        /* the duty cycle must be lower or equal the period */
        uint16_t ton = comp1 + 1u;
        uint16_t toff = comp2 + 1u;
        uint16_t period = ton + toff;
        if(duty <= period)
        {
            if (duty == 0u)
            {
                ton = 1u;
            }
            else if (period == duty)
            {
                ton = period - 1u;
            }
            else
            {
                ton = duty;
            }
            toff = period - ton;
            comp1 = ton - 1u;
            comp2 = toff - 1u;
            /* set it */
            ETIMER_DRV_SetCompareThresholdBuffered(instance->instIdx, channel, comp1, comp2);
            /* all ok */
            status = STATUS_SUCCESS;
        }
        else
        {
            /* duty is higher than period */
            DEV_ASSERT(false);
        }
    }
    else
    #endif

    #if (defined (PWM_OVER_FLEXPWM))
    if (instance->instType == PWM_INST_TYPE_FLEXPWM)
    {
        FLEXPWM_DRV_UpdatePulseWidth(instance->instIdx, (flexpwm_module_t)channel, duty, 0U, FlexPwmEdgeAligned);
        FLEXPWM_DRV_LoadCommands(instance->instIdx, (uint32_t)(1UL << channel));

        status = STATUS_SUCCESS;
    }
    else
    #endif
    {
        DEV_ASSERT(false);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PWM_UpdatePeriod
 * Description   : Update period for specific a specific channel. This function changes period for
 * all channels which shares the timebase with targeted channel.
 * Implements    : PWM_UpdatePeriod_Activity
 *
 *END**************************************************************************/
status_t PWM_UpdatePeriod(const pwm_instance_t * const instance, uint8_t channel, uint32_t period)
{

    DEV_ASSERT(instance != NULL);

    status_t status = STATUS_ERROR;

    #if (defined(PWM_OVER_FTM))
    if (instance->instType == PWM_INST_TYPE_FTM)
    {
        (void)FTM_DRV_UpdatePwmPeriod(instance->instIdx, FTM_PWM_UPDATE_IN_TICKS, (uint16_t)period, true);
        (void)channel;
        status = STATUS_SUCCESS;
    }
    else
    #endif

    #if (defined(PWM_OVER_EMIOS))
    if (instance->instType == PWM_INST_TYPE_EMIOS)
    {
        uint8_t hwChannel = 0U;

        /* Get hardware Channel from virtual Channel */
        status = PWM_EMIOS_VrChannelMapping(channel, &hwChannel);
        if(STATUS_SUCCESS != status)
        {
            DEV_ASSERT(false);
        }

        EMIOS_DRV_PWM_SetPeriod(instance->instIdx, hwChannel, period);

        status = STATUS_SUCCESS;
    }
    else
    #endif

    #if (defined (PWM_OVER_ETIMER))
    if (instance->instType == PWM_INST_TYPE_ETIMER)
    {
        /* read current compare values */
        uint16_t comp1;
        ETIMER_DRV_GetCompareThresholdBuffered(instance->instIdx, channel, &comp1, NULL);
        uint16_t ton = comp1 + 1u;
        /* the period must be higher than the duty (Ton)
         * because the eTimer cannot generate a 100% duty signal
         */
        if(period > ton)
        {
            /* set it */
            DEV_ASSERT((period - ton) <= 0xFFFFu); /* check if the Toff length fits in 16 bits */
            uint16_t toff = period - ton;
            uint16_t comp2 = toff - 1u;
            /* set it */
            ETIMER_DRV_SetCompareThresholdBuffered(instance->instIdx, channel, comp1, comp2);
            /* all ok */
            status = STATUS_SUCCESS;
        }
        else
        {
            /* duty is higher than period */
            DEV_ASSERT(false);
        }
    }
    else
    #endif

    #if (defined (PWM_OVER_FLEXPWM))
    if (instance->instType == PWM_INST_TYPE_FLEXPWM)
    {
        FLEXPWM_DRV_UpdatePwmPeriod(instance->instIdx, (flexpwm_module_t)channel, period);
        FLEXPWM_DRV_LoadCommands(instance->instIdx, (uint32_t)(1UL << channel));

        status = STATUS_SUCCESS;
    }
    else
    #endif
    {
        DEV_ASSERT(false);
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PWM_OverwriteOutputChannels
 * Description   : This function change the output value for some channels. channelsMask select
 * which channels will be overwrite, each bit filed representing one channel: 1 - channel is controlled
 * by channelsValues, 0 - channel is controlled by pwm. channelsValues select output values to be write on corresponding
 * channel.
 * For PWM_PAL over FTM, when enable complementary channels, if this function is used to force output of complementary channels(n and n+1)
 * with value is high, the output of channel n is going to be high and the output of channel n+1 is going to be low.
 * Please refer to Software ouput control behavior table in the reference manual to get more details.
 * Implements    : PWM_OverwriteOutputChannels_Activity
 *
 *END**************************************************************************/
status_t PWM_OverwriteOutputChannels(const pwm_instance_t * const instance, uint32_t channelsMask, uint32_t channelsValues)
{
    DEV_ASSERT(instance != NULL);

    status_t status = STATUS_ERROR;

    #if (defined(PWM_OVER_FTM))
    if (instance->instType == PWM_INST_TYPE_FTM)
    {
        (void)FTM_DRV_SetAllChnSoftwareOutputControl(instance->instIdx, (uint8_t)channelsMask, (uint8_t)channelsValues, true);
        status =  STATUS_SUCCESS;
    }
    else
    #endif

    #if (defined(PWM_OVER_EMIOS))
    if (instance->instType == PWM_INST_TYPE_EMIOS)
    {
        status = STATUS_UNSUPPORTED;
    }
    else
    #endif

    #if (defined (PWM_OVER_ETIMER))
    if (instance->instType == PWM_INST_TYPE_ETIMER)
    {
        uint16_t channel;
        bool outputLogic;

        for (channel = 0U; channel < ETIMER_CH_COUNT; channel++)
        {

            if ((channelsMask & (1UL << channel)) ? true : false)
            {
                outputLogic = (channelsValues & (1UL << channel)) ? true : false;
                ETIMER_DRV_ForceOutputLogicLevel((uint16_t)instance->instIdx, channel, outputLogic);
            }

        }
        status = STATUS_SUCCESS;
    }
    else
    #endif

    
    #if (defined (PWM_OVER_FLEXPWM))
    if (instance->instType == PWM_INST_TYPE_FLEXPWM)
    {
        status = STATUS_UNSUPPORTED;
    }
    else
    #endif
    {
        DEV_ASSERT(false);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PWM_Deinit
 * Description   : Uninitialize PWM instance.
 * Implements    : PWM_Deinit_Activity
 *
 *END**************************************************************************/
status_t PWM_Deinit(const pwm_instance_t * const instance)
{
    DEV_ASSERT(instance != NULL);

    status_t status = STATUS_ERROR;

    #if (defined(PWM_OVER_FTM))
    if (instance->instType == PWM_INST_TYPE_FTM)
    {
        DEV_ASSERT(instance->instIdx < FTM_INSTANCE_COUNT);
        (void)FTM_DRV_DeinitPwm(instance->instIdx);
        (void)FTM_DRV_Deinit(instance->instIdx);
        PwmFreeState(FtmStateIsAllocated, FtmStateInstanceMapping, instance, NO_OF_FTM_INSTS_FOR_PWM);
        uint8_t index;
        for (index = 0U; index < FEATURE_FTM_CHANNEL_COUNT; index++)
        {
            pwmPalCombChnFlag[instance->instIdx][index] = false;
        }

        status = STATUS_SUCCESS;
    }
    else
    #endif

    #if (defined(PWM_OVER_EMIOS))
    if (instance->instType == PWM_INST_TYPE_EMIOS)
    {
        uint8_t index;

        for (index = 0U; index < FEATURE_EMIOS_CH_COUNT; index++)
        {
            uint8_t hwChannel = 0U;

            /* Get hardware Channel from virtual Channel */
            status = PWM_EMIOS_VrChannelMapping(index, &hwChannel);
            if(STATUS_SUCCESS != status)
            {
                DEV_ASSERT(false);
            }

            /* Disable channels in the output compare over EMIOS */
            EMIOS_DRV_DeInitChannel((uint8_t)instance->instIdx,hwChannel);
        }
        status =  STATUS_SUCCESS;
    }
    else
    #endif

    #if (defined (PWM_OVER_ETIMER))
    if (instance->instType == PWM_INST_TYPE_ETIMER)
    {
        /* Uninitialize ETIMERx peripheral */
        ETIMER_DRV_Deinit(instance->instIdx);
        status =  STATUS_SUCCESS;
    }
    else
    #endif

    #if (defined (PWM_OVER_FLEXPWM))
    if (instance->instType == PWM_INST_TYPE_FLEXPWM)
    {
        /* Revert FlexPWM instance settings to the after-reset values */
        FLEXPWM_DRV_Deinit(instance->instIdx);
        status =  STATUS_SUCCESS;
    }
    else
    #endif

    {
        DEV_ASSERT(false);
    }

    return status;
}

#if (defined(PWM_OVER_EMIOS))
/*FUNCTION**********************************************************************
 *
 * Function Name : PWM_EMIOS_VrChannelMapping
 * Description   : This function maps the virtual channel to hardware
 *                 channel based on platform.
 *END**************************************************************************/
static status_t PWM_EMIOS_VrChannelMapping(uint8_t vrChannel, uint8_t * hwChannel)
{
    status_t status = STATUS_SUCCESS;

    if(vrChannel >= FEATURE_EMIOS_CH_COUNT)
    {
        status = STATUS_ERROR;
    }
    else
    {
#if (FEATURE_PWMPAL_EMIOS_HAS_CHANNEL_MAPPING)
        *hwChannel = s_VrChannelMapping[vrChannel];
#else
        *hwChannel = vrChannel;
#endif
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PWM_EMIOS_ValidateChannel
 * Description   : Validate a eMIOS channel can support.
 *END**************************************************************************/
static bool PWM_EMIOS_ValidateChannel(uint8_t inChVal, uint8_t * outChVal)
{
    bool tmp = false;
#if (FEATURE_EMIOS_BUS_B_SELECT == 1U)
	if (inChVal <= 7U)
	{
		*outChVal = FEATURE_EMIOS_BUS_B_SELECT_OFFSET(inChVal);
        tmp = true;
	}
#endif
#if (FEATURE_EMIOS_BUS_C_SELECT == 1U)
	if ((inChVal >= 8U) && (inChVal <= 15U))
	{
		*outChVal = FEATURE_EMIOS_BUS_C_SELECT_OFFSET(inChVal);
        tmp = true;
	}
#endif
#if (FEATURE_EMIOS_BUS_D_SELECT == 1U)
	if ((inChVal >= 16U) && (inChVal <= 23U))
	{
		*outChVal = FEATURE_EMIOS_BUS_D_SELECT_OFFSET(inChVal);
        tmp = true;
	}
#endif
#if (FEATURE_EMIOS_BUS_E_SELECT == 1U)
	if ((inChVal >= 24U) && (inChVal <= 31U))
	{
		*outChVal = FEATURE_EMIOS_BUS_E_SELECT_OFFSET(inChVal);
        tmp = true;
	}
#endif

    return tmp;
}
#endif
