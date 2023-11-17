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
 * @file adc_pal.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3,  Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is part of PAL API, so it defined to be used by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.3, Cast performed between a pointer to
 * object type and a pointer to a different object type.
 * The type of the configuration structure extension is platform dependent
 * and needs to be casted from a generic void pointer.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.5, Conversion from pointer to void
 * to pointer to other type
 * The type of the configuration structure extension is platform dependent
 * and needs to be casted from a generic void pointer.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or different essential type.
 * Assignment is required to calculate CTU FIFO IRQnumber.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially unsigned' to 'essentially enum<i>'
 * The cast is required to calculate CTU FIFO IRQnumber.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function 'ADC_SAR_CTU_Init(uint32_t, const adc_config_t *)'
 * Early return is used to reduce code nesting depth.
 *
 */

#include <stddef.h>
#include "adc_pal.h"
#include "adc_irq.h"
#include "device_registers.h"
#include "interrupt_manager.h"
#include "osif.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*! @cond DRIVER_INTERNAL_USE_ONLY */

typedef struct
{
    uint16_t bufferLength;            /*!< Length of the buffer associated with the current active conversion group */
    uint16_t currentBufferOffset;     /*!< Offset (in elements) of the next position to be written in the result buffer */
    bool notificationEn;              /*!< Flag for enabling/disabling notification */
    bool active;                      /*!< Flag for state of the group enabled/disabled */
#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)
    uint32_t groupIdx;                /*!< Index of the group for which the state is stored */
#endif
#if defined (ADC_PAL_TYPE_SDADC) || defined (ADC_PAL_TYPE_EQADC)
    uint8_t currentConvIdx;          /*!< Index of the current conversion in the active group */
#endif
} adc_group_state_t;

/*!
 * @brief Runtime state of the ADC PAL
 *
 * This structure is used by the ADC PAL for storing internal information, needed at runtime.
 */
typedef struct
{
    const adc_group_config_t * groupArray;                            /*!< Pointer to the array of group configurations */
    uint16_t numGroups;                                               /*!< Number of group configurations available in the input array */
#if defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined(ADC_PAL_TYPE_ADC_SAR_CTU) || defined(ADC_PAL_TYPE_EQADC)
    adc_group_state_t hwTrigGroupState[ADC_PAL_MAX_NUM_HW_GROUPS_EN]; /*!< State of Hw triggered groups */
#endif /* defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined(ADC_PAL_TYPE_ADC_SAR_CTU) || defined(ADC_PAL_TYPE_EQADC) */
    adc_group_state_t swTrigGroupState;                               /*!< State of the most recent activated SW trigger group */
    uint32_t latestGroupIdx;                                          /*!< Index of the most recently enabled group (HW or SW triggered) group (might not be active anymore) */
#if (defined (ADC_PAL_S32K1xx))
    pdb_clk_prescaler_div_t pdbPrescaler;                            /*!< PDB clock prescaler */
#endif
#if (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU))
    uint32_t stateIdxMapping[ADC_PAL_TOTAL_NUM_GROUPS];               /*!< Maps groupIdx to index in hwTrigGroupState. For groupIdx corresponding to swTrigGroups, value is invalid. */
#endif
    /* Features determined at runtime which might be available depending on ADC PAL type */
    bool hasHwTrigSupport;
#if defined (ADC_PAL_TYPE_EQADC)
    /* Save the extension members which need to be sent at each command */
    eqadc_sampling_time_t samplingTime;
    bool signEn;
#endif
} adc_pal_state_t;


#if defined (ADC_PAL_S32K1xx)

#if (ADC_PAL_MAX_NUM_HW_GROUPS_EN != 1u)
#error "The current device supports maximum 1 HW triggered group enabled"
#endif /* (ADC_PAL_MAX_NUM_HW_GROUPS_EN != 1) */

/* For S32K1xx, ADC PAL instances are mapped 1:1 with ADC instances (and PDB instances). */
#define NUMBER_OF_ADC_PAL_INSTANCES_S32K1xx             ADC_INSTANCE_COUNT

#define ADC_PAL_PDB_CHAN            (0u)                /*!< PDB channel used for triggering ADC */
#define ADC_PAL_TRGMUX_IDX          (0u)                /*!< TRGMUX instance used by ADC PAL */
#define ADC_PAL_MAX_CONVS_IN_GROUP  (PDB_DLY_COUNT)     /*!< Maximum number of ADC conversions in a group of conversions. */

/* For S32K1xx, the ADC PAL assumes that ADC and PDB instances are mapped 1:1, thus PDB_INSTANCE_COUNT == ADC_INSTANCE_COUNT */
static const trgmux_target_module_t adcPalTrgmuxTarget[ADC_INSTANCE_COUNT] = {
#if (ADC_INSTANCE_COUNT >= 1u)
    TRGMUX_TARGET_MODULE_PDB0_TRG_IN,
#endif
#if (ADC_INSTANCE_COUNT >= 2u)
    TRGMUX_TARGET_MODULE_PDB1_TRG_IN
#endif
};

static status_t ADC_Init_S32K1xx(const uint32_t instance,
                                 const adc_config_t * const config);

static inline void ADC_ConfigPdbAndPretriggers(const uint32_t instIdx,
                                               const pdb_trigger_src_t trgSrc,
                                               const adc_group_config_t * currentGroupCfg);

static void ADC_ConfigGroup(const uint32_t instance,
                            const uint32_t groupIdx,
                            const bool hwTriggerFlag);

static status_t ADC_StopGroupBlocking(const uint32_t instance,
                                      const uint32_t timeout);

#endif /* defined (ADC_PAL_S32K1xx) */


#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)

/* For ADC_PAL_TYPE_ADC_SAR_BCTU, ADC PAL instances are mapped 1:1 with BCTU instances.
 * Each BCTU instance may have multiple ADCs connected. */
#define NUMBER_OF_ADC_PAL_INSTANCES_ADC_SAR_BCTU        BCTU_INSTANCE_COUNT


#define ADC_PAL_MAX_CONVS_IN_GROUP_HW_TRIG (BCTU_LISTCHR__COUNT * 2u)  /*!< Maximum number of ADC conversions in a hw triggered group of conversions - each LISTCHR reg contains 2 list elements */

static status_t ADC_SAR_BCTU_Init(const uint32_t instIdx,
                                      const adc_config_t * const config);

static void ADC_ConfigHwTriggeredGroup(const uint32_t instIdx,
                                       const adc_group_config_t * const groupConfig,
                                       const uint8_t bctuStartListIdx);

static uint32_t ADC_GetGroupIdx(const uint32_t instIdx,
                                const adc_trigger_source_t hwTrigSource);
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_BCTU) */


#if defined (ADC_PAL_SAR)

/* When no hardware triggering unit is available, ADC PAL instances are mapped 1:1 with ADC SAR instances. */
#define NUMBER_OF_ADC_PAL_INSTANCES_ADC_SAR             ADC_INSTANCE_COUNT

static status_t ADC_SAR_Init(const uint32_t instIdx,
                             const adc_config_t * const config);

#endif /* defined (ADC_PAL_SAR) */


#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)

/* For ADC_PAL_TYPE_ADC_SAR_CTU, ADC PAL instances are mapped 1:1 with CTU instances.
 * Each CTU instance may have multiple ADCs connected. */
#define NUMBER_OF_ADC_PAL_INSTANCES_ADC_SAR_CTU         CTU_INSTANCE_COUNT

/* Each HW triggered group is mapped to a dedicated CTU FIFO, therefore
 * maximum number of HW triggered groups is the number of CTU FIFOs */
#define NUM_MAX_HW_TRIG_GROUPS CTU_FR_COUNT

/* Table of base addresses for CTU instances. */
static CTU_Type * const ctuBase[CTU_INSTANCE_COUNT] = CTU_BASE_PTRS;

static status_t ADC_SAR_CTU_Init(const uint32_t instIdx,
                                 const adc_config_t * const config);

#endif /* defined(ADC_PAL_TYPE_ADC_SAR_CTU) */


#if defined (ADC_PAL_TYPE_SDADC)

/* For ADC_PAL_TYPE_SDADC, ADC PAL instances are mapped 1:1 with SDADC instances. */
#define NUMBER_OF_ADC_PAL_INSTANCES_SDADC               SDADC_INSTANCE_COUNT

#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
static bool SDADC_CheckInputChannelMapping(const uint16_t inputChan);
#endif /* defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT) */

static status_t SDADC_Init(const uint32_t instIdx,
                           const adc_config_t * const config);

static void SDADC_StartConversion(const uint32_t adcIdx,
                                  const uint8_t groupIdx,
                                  const uint8_t convIdx);

#endif /* defined (ADC_PAL_TYPE_SDADC) */


#if defined (ADC_PAL_TYPE_EQADC)

#if (ADC_PAL_MAX_NUM_HW_GROUPS_EN != 1u)
#error "ADC_PAL_TYPE_EQADC supports maximum 1 HW triggered group enabled"
#endif /* (ADC_PAL_MAX_NUM_HW_GROUPS_EN != 1) */

/* For ADC_PAL_TYPE_EQADC, ADC PAL instances are mapped 1:1 with EQADC instances. */
#define NUMBER_OF_ADC_PAL_INSTANCES_EQADC               EQADC_INSTANCE_COUNT

#define EQADC_CFIFO_IDX_SW_TRIG                         (0u) /* SW triggered conversions mapped always to cfifo 0 */
#define EQADC_RFIFO_IDX_SW_TRIG                         (0u) /* SW triggered conversions mapped always to rfifo 0 */
#define EQADC_CFIFO_IDX_HW_TRIG                         (1u) /* HW triggered conversions mapped always to cfifo 1 */
#define EQADC_RFIFO_IDX_HW_TRIG                         (1u) /* HW triggered conversions mapped always to rfifo 1 */
#define EQADC_CFIFO_IDX_CALIB                           (2u) /* FIFO used for calibration procedures */
#define EQADC_RFIFO_IDX_CALIB                           (2u) /* FIFO used for calibration procedures */

static status_t EQADC_Init(const uint32_t instIdx,
                           const adc_config_t * const config);
static inline adc_group_state_t * EQADC_GetGroupState(adc_pal_state_t * const palState,
                                                      const uint32_t fifoIdx);
static status_t EQADC_StopConvProcedure(const uint16_t instIdx,
                                        const uint32_t fifoIdx,
                                        const uint32_t timeout);


static const IRQn_Type eqadcCfifoFillIRQn[EQADC_INSTANCE_COUNT][EQADC_NUM_USED_FIFOS] = {
#if (EQADC_INSTANCE_COUNT > 0u)
    {
        EQADC0_FIFO0_CFFF_IRQn,
        EQADC0_FIFO1_CFFF_IRQn
    },
#endif /* (EQADC_INSTANCE_COUNT > 0u) */
#if (EQADC_INSTANCE_COUNT > 1u)
    {
        EQADC1_FIFO0_CFFF_IRQn,
        EQADC1_FIFO1_CFFF_IRQn
    },
#endif /* (EQADC_INSTANCE_COUNT > 1u) */
};

static const IRQn_Type eqadcRfifoFillIRQn[EQADC_INSTANCE_COUNT][EQADC_NUM_USED_FIFOS] = {
#if (EQADC_INSTANCE_COUNT > 0u)
    {
        EQADC0_FIFO0_RFDF_IRQn,
        EQADC0_FIFO1_RFDF_IRQn
    },
#endif /* (EQADC_INSTANCE_COUNT > 0u) */
#if (EQADC_INSTANCE_COUNT > 1u)
    {
        EQADC1_FIFO0_RFDF_IRQn,
        EQADC1_FIFO1_RFDF_IRQn
    },
#endif /* (EQADC_INSTANCE_COUNT > 1u) */
};

#endif /* defined (ADC_PAL_TYPE_EQADC) */


#if (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_SAR) || defined (ADC_PAL_TYPE_EQADC))

static inline uint8_t ADC_GetChanIdx(const adc_input_chan_t adcPalInputChan);

#if (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC))
static inline uint8_t ADC_GetAdcAbsoluteIdx(const adc_input_chan_t adcPalInputChan);
#endif /* (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU)  || defined (ADC_PAL_TYPE_EQADC)) */

#endif /* (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_SAR) || defined (ADC_PAL_TYPE_EQADC)) */


#if (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_SAR))

static const IRQn_Type adcEocIrqNums[ADC_INSTANCE_COUNT] = {
    ADC0_EOC_IRQn, /* On some platforms this might be a define from features.h */
#if ADC_INSTANCE_COUNT > 1u
    ADC1_EOC_IRQn, /* On some platforms this might be a define from features.h */
#endif
#if ADC_INSTANCE_COUNT > 2u
    ADC2_EOC_IRQn, /* On some platforms this might be a define from features.h */
#endif
#if ADC_INSTANCE_COUNT > 3u
    ADC3_EOC_IRQn /* On some platforms this might be a define from features.h */
#endif
};

#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)
/* Table of base addresses for ADC instances. */
static ADC_Type * const adcBase[ADC_INSTANCE_COUNT] = ADC_BASE_PTRS;
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_CTU) */

static uint16_t ADC_GetUsedAdcInstances(const uint32_t instance);

static status_t ADC_SAR_Config(const uint32_t instIdx,
                               const adc_config_t * const config);

static status_t ADC_SAR_StopConversion(const uint32_t instIdx,
                                       const adc_group_config_t * currentGroupCfg,
                                       const uint32_t timeout);

static void ADC_SAR_Reset(const uint32_t instIdx);

#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
static void ADC_CheckChanDuplicate(const adc_config_t * const config);

#endif /* defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT) */

#endif /* (defined(ADC_PAL_TYPE_ADC_SAR_BCTU) || defined(ADC_PAL_TYPE_ADC_SAR_CTU)) */


#if (defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC))

static inline adc_group_state_t * ADC_GetHwGroupStatePtr(adc_pal_state_t * const palState,
                                                         const uint32_t groupIdx);

#endif /* (defined(ADC_PAL_S32K1xx) || defined(ADC_PAL_TYPE_ADC_SAR_BCTU) || defined(ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC)) */


static bool ADC_AnyHwTrigGroupActive(const adc_instance_t * const instance);

static void ADC_PalStateUpdateStart(adc_pal_state_t * const palState,
                                    const uint32_t groupIdx);

static void ADC_CallNotificationCb(const adc_pal_state_t * const palState,
                                   const uint32_t groupIdx,
                                   const adc_group_state_t * const groupState);

static adc_pal_state_t * ADC_GetPalState(const adc_instance_t * const instance);
static void ADC_InitStateRuntimeFeatures(adc_pal_state_t * const palState, const adc_inst_type_t instType);

/*! @endcond */

/*******************************************************************************
 * Variables
 ******************************************************************************/
#if defined (ADC_PAL_S32K1xx)
/* Static variable to store the PAL state information */
static adc_pal_state_t adcPalState[NUMBER_OF_ADC_PAL_INSTANCES_S32K1xx];
#endif

#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
/* Static variable to store the PAL state information */
static adc_pal_state_t adcPalState_adc_sar_bctu[NUMBER_OF_ADC_PAL_INSTANCES_ADC_SAR_BCTU];
#endif

#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)
/* Static variable to store the PAL state information */
static adc_pal_state_t adcPalState_adc_sar_ctu[NUMBER_OF_ADC_PAL_INSTANCES_ADC_SAR_CTU];
#endif

#if defined (ADC_PAL_SAR)
/* Static variable to store the PAL state information */
static adc_pal_state_t adcPalState_adc_sar[NUMBER_OF_ADC_PAL_INSTANCES_ADC_SAR];
#endif

#if defined (ADC_PAL_TYPE_SDADC)
/* Static variable to store the PAL state information */
static adc_pal_state_t adcPalState_sdadc[NUMBER_OF_ADC_PAL_INSTANCES_SDADC];
#endif

#if defined (ADC_PAL_TYPE_EQADC)
/* Static variable to store the PAL state information */
static adc_pal_state_t adcPalState_eqadc[NUMBER_OF_ADC_PAL_INSTANCES_EQADC];
#endif

/*******************************************************************************
 * Public Functions
 ******************************************************************************/


/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_Init
 * Description   : This function initializes the ADC PAL instance, including the
 * other platform specific HW units used together with ADC.
 *
 * Implements : ADC_Init_Activity
 * END**************************************************************************/
status_t ADC_Init(const adc_instance_t * const instance,
                  const adc_config_t * const config)
{
    DEV_ASSERT(instance != NULL);
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(config->extension != NULL);
    DEV_ASSERT(config->groupConfigArray != NULL);

    const uint32_t instIdx = instance->instIdx;
    const adc_inst_type_t instType = instance->instType;

    adc_pal_state_t * const palState = ADC_GetPalState(instance);
    status_t status = STATUS_ERROR;
    uint8_t idx = 0u;

    ADC_InitStateRuntimeFeatures(palState, instType);

    /* Initialize PAL state structure members common for all platforms */
    palState->groupArray     = config->groupConfigArray;
    palState->numGroups      = config->numGroups;

#if defined (ADC_PAL_TYPE_EQADC)
    /* Reset state members specific for this platform */
    palState->samplingTime   = EQADC_SAMPLING_TIME_2_CYCLES;
    palState->signEn         = false;
#endif /* defined (ADC_PAL_TYPE_EQADC) */

#if (defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC))
    if(palState->hasHwTrigSupport == true)
    {
        for (idx = 0; idx < ADC_PAL_MAX_NUM_HW_GROUPS_EN; idx++)
        {
            palState->hwTrigGroupState[idx].active              = false;
            palState->hwTrigGroupState[idx].bufferLength        = 0u;
            palState->hwTrigGroupState[idx].currentBufferOffset = 0u;
            palState->hwTrigGroupState[idx].notificationEn      = false;
#if defined (ADC_PAL_TYPE_SDADC) || defined (ADC_PAL_TYPE_EQADC)
            palState->hwTrigGroupState[idx].currentConvIdx      = 0u;
#endif /* defined (ADC_PAL_TYPE_SDADC) || defined (ADC_PAL_TYPE_EQADC) */
        }
#if (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU))
        for (idx = 0; idx < ADC_PAL_TOTAL_NUM_GROUPS; idx++)
        {
            palState->stateIdxMapping[idx]                      = 0u;
        }
#endif /* (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU)) */
    }
#endif /* (defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC)) */

    palState->swTrigGroupState.active               = false;
    palState->swTrigGroupState.bufferLength         = 0u;
    palState->swTrigGroupState.currentBufferOffset  = 0u;
    palState->swTrigGroupState.notificationEn       = false;
#if defined (ADC_PAL_TYPE_SDADC) || defined (ADC_PAL_TYPE_EQADC)
    palState->swTrigGroupState.currentConvIdx       = 0u;
#endif /* defined (ADC_PAL_TYPE_SDADC) || defined (ADC_PAL_TYPE_EQADC) */
    palState->latestGroupIdx    = 0u;

#if (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) ||  defined (ADC_PAL_SAR) || defined (ADC_PAL_TYPE_SDADC))

    /* Initialize state members specific for the ADC PAL type */
    uint8_t hwTrigGroupIdx = 0;

#endif /* (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) ||  defined (ADC_PAL_SAR) || defined (ADC_PAL_TYPE_SDADC)) */

#if (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU))
    if(palState->hasHwTrigSupport == true)
    {
        for (idx = 0; idx < palState->numGroups; idx++)
        {
            if (palState->groupArray[idx].hwTriggerSupport == true)
            {
                /* For hwTrigGroups, stateIdxMapping holds the index in hwTrigGroupState array */
                palState->stateIdxMapping[idx] = hwTrigGroupIdx;
                hwTrigGroupIdx++;
            }
            else
            {
                /* A single SW triggered group active is supported,
                 * so for swTrigGroups, stateIdxMapping holds a value which is not used. */
                palState->stateIdxMapping[idx] = 0u;
            }
        }
    }
#endif /* (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU)) */


#if (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) ||  defined (ADC_PAL_SAR) || defined (ADC_PAL_TYPE_SDADC) || defined (ADC_PAL_TYPE_EQADC))

#if (defined (ADC_PAL_SAR) || defined (ADC_PAL_TYPE_SDADC))
    if(palState->hasHwTrigSupport == false)
    {
        for (idx = 0; idx < palState->numGroups; idx++)
        {
            if (palState->groupArray[idx].hwTriggerSupport == true)
            {
                hwTrigGroupIdx++;
            }
        }
    }
#endif /* (defined (ADC_PAL_SAR) || defined (ADC_PAL_TYPE_SDADC)) */
#endif /* (defined(ADC_PAL_TYPE_ADC_SAR_BCTU) || defined(ADC_PAL_TYPE_ADC_SAR_CTU) ||  defined (ADC_PAL_SAR)) */

#if defined (ADC_PAL_S32K1xx)
    if(instType == ADC_INST_TYPE_ADC_S32K1xx)
    {
        /* Initialize state members specific for this platform */
        const extension_adc_s32k1xx_t * const extension = (extension_adc_s32k1xx_t *)(config->extension);
        palState->pdbPrescaler = extension->pdbPrescaler;

        status = ADC_Init_S32K1xx(instIdx, config);
    }
    else
#endif /* defined (ADC_PAL_S32K1xx) */

#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
    if(instType == ADC_INST_TYPE_ADC_SAR_BCTU)
    {
        status = ADC_SAR_BCTU_Init(instIdx, config);
    }
    else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_BCTU) */

#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)
    if(instType == ADC_INST_TYPE_ADC_SAR_CTU)
    {
        status = ADC_SAR_CTU_Init(instIdx, config);
    }
    else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_CTU) */

#if defined (ADC_PAL_SAR)
    if(instType == ADC_INST_TYPE_ADC_SAR)
    {
        if(hwTrigGroupIdx != 0u)
        {
            status = STATUS_UNSUPPORTED;
        }
        else
        {
            status = ADC_SAR_Init(instIdx, config);
        }
    }
    else
#endif /* defined (ADC_PAL_SAR) */

#if defined (ADC_PAL_TYPE_SDADC)
    if(instType == ADC_INST_TYPE_SDADC)
    {
        if(hwTrigGroupIdx != 0u)
        {
            status = STATUS_UNSUPPORTED;
        }
        else
        {
            status = SDADC_Init(instIdx, config);

#if FEATURE_SDADC_INSTANCES_HAVE_COMMON_INT_VECTOR
            /* This code should only execute if SDADC_Init() returns STATUS_SUCCESS.
            SDADC_Init() only returns STATUS_SUCCESS, but the condition should be updated if SDADC_Init is extended with new status code. */
           	sdadcInstEnabled[instIdx] = true;
#endif /* FEATURE_SDADC_INSTANCES_HAVE_COMMON_INT_VECTOR */

            INT_SYS_InstallHandler(SDADC_DRV_GetInterruptNumber(instIdx), sdadcIrqHandlers[instIdx], NULL);
            INT_SYS_EnableIRQ(SDADC_DRV_GetInterruptNumber(instIdx));
        }
    }
    else
#endif /* defined (ADC_PAL_TYPE_SDADC) */

#if defined (ADC_PAL_TYPE_EQADC)
    if(instType == ADC_INST_TYPE_EQADC)
    {
        status = EQADC_Init(instIdx, config);
        if(status != STATUS_SUCCESS)
        {
            status = STATUS_ERROR; /* platform specific error encountered while initializing one of the HW modules used by ADC PAL */
        }
    }
    else
#endif /* defined (ADC_PAL_TYPE_EQADC) */
    {
        DEV_ASSERT(false);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_Deinit
 * Description   : This function resets the ADC PAL instance, including
 * the other platform specific HW units used together with ADC.
 *
 * Implements : ADC_Deinit_Activity
 * END**************************************************************************/
status_t ADC_Deinit(const adc_instance_t * const instance)
{
    DEV_ASSERT(instance != NULL);

    const uint32_t instIdx         = instance->instIdx;
    const adc_inst_type_t instType = instance->instType;

    adc_pal_state_t * const palState = ADC_GetPalState(instance);
    bool hwTrigGroupActive           = false;
    status_t status                  = STATUS_ERROR;
#if defined (ADC_PAL_S32K1xx) ||  defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC) || defined (ADC_PAL_TYPE_SDADC)
    uint16_t idx;
#endif /* defined (ADC_PAL_S32K1xx) ||  defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC) || defined (ADC_PAL_TYPE_SDADC) */

    hwTrigGroupActive = ADC_AnyHwTrigGroupActive(instance);

    if ((palState->swTrigGroupState.active == true) || (hwTrigGroupActive == true))
    {
        status = STATUS_BUSY;
    }
    else
    {
#if defined (ADC_PAL_S32K1xx)
        if(instType == ADC_INST_TYPE_ADC_S32K1xx)
        {
            /* No state members specific for this platform to reset */

            PDB_DRV_Deinit(instIdx);

            status = TRGMUX_DRV_SetTrigSourceForTargetModule(ADC_PAL_TRGMUX_IDX, TRGMUX_TRIG_SOURCE_DISABLED, adcPalTrgmuxTarget[instIdx]);
            DEV_ASSERT(status == STATUS_SUCCESS);

            ADC_DRV_Reset(instIdx);

            /* Disable interrupt from INT manager */
            IRQn_Type adcIrqId;
            adcIrqId = ADC_DRV_GetInterruptNumber(instIdx);
            INT_SYS_DisableIRQ(adcIrqId);

            status = STATUS_SUCCESS;
        }
        else
#endif /* defined (ADC_PAL_S32K1xx) */

#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
        if(instType == ADC_INST_TYPE_ADC_SAR_BCTU)
        {
            /* Reset state members specific for this platform */
            for (idx = 0; idx < palState->numGroups; idx++)
            {
                palState->stateIdxMapping[idx] = 0u;
            }

            /* Deinit BCTU and ADCs */
            status_t bctuStatus;
            bctuStatus = BCTU_DRV_Reset(instIdx, /* default timeout value 2ms*/ 2u);
            if (bctuStatus == STATUS_TIMEOUT)
            {
                status = STATUS_BUSY;
            }
            else
            {
                ADC_SAR_Reset(instIdx);

                status = STATUS_SUCCESS;
            }
        }
        else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_BCTU) */

#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)
        if(instType == ADC_INST_TYPE_ADC_SAR_CTU)
        {
            /* Reset state members specific for this platform */
            for (idx = 0; idx < palState->numGroups; idx++)
            {
                palState->stateIdxMapping[idx] = 0u;
            }

            CTU_DRV_Reset(instIdx);

            ADC_SAR_Reset(instIdx);

            status = STATUS_SUCCESS;
        }
        else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_CTU) */

#if defined (ADC_PAL_SAR)
        if(instType == ADC_INST_TYPE_ADC_SAR)
        {
            ADC_SAR_Reset(instIdx);

            status = STATUS_SUCCESS;
        }
        else
#endif /* defined (ADC_PAL_SAR) */

#if defined (ADC_PAL_TYPE_SDADC)
        if(instType == ADC_INST_TYPE_SDADC)
        {
#if FEATURE_SDADC_INSTANCES_HAVE_COMMON_INT_VECTOR
            idx = 0u;
            bool anyInstEnabled = false;

            /* Current instance gets disabled */
            sdadcInstEnabled[instIdx] = false;

            while(idx < SDADC_INSTANCE_COUNT)
            {
            	if(sdadcInstEnabled[idx] == true)
            	{
            		anyInstEnabled = true;
            		break;
            	}
				idx++;
            }

            /* Disable interrupt if there is no instance enabled */
            if(anyInstEnabled == false)
            {
            	INT_SYS_DisableIRQ(SDADC_DRV_GetInterruptNumber(instIdx));
            }
#else
            INT_SYS_DisableIRQ(SDADC_DRV_GetInterruptNumber(instIdx));
#endif /* FEATURE_SDADC_INSTANCES_HAVE_COMMON_INT_VECTOR */

            SDADC_DRV_Reset(instIdx);
            status = STATUS_SUCCESS;
        }
        else
#endif /* defined (ADC_PAL_TYPE_SDADC) */

#if defined (ADC_PAL_TYPE_EQADC)
        if(instType == ADC_INST_TYPE_EQADC)
        {
            /* Reset state members specific for this platform */
            palState->samplingTime = EQADC_SAMPLING_TIME_2_CYCLES;
            palState->signEn       = false;

            EQADC_DRV_Reset(instIdx);
            status = STATUS_SUCCESS;
        }
        else
#endif /* defined (ADC_PAL_TYPE_EQADC) */
        {
            DEV_ASSERT(false);
        }

        /* Reset PAL state structure members common for all platforms */
        palState->groupArray = NULL;
        palState->numGroups  = 0u;
#if defined (ADC_PAL_S32K1xx) ||  defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC)
        if(palState->hasHwTrigSupport == true)
        {
            for (idx = 0; idx < ADC_PAL_MAX_NUM_HW_GROUPS_EN; idx++)
            {
                palState->hwTrigGroupState[idx].active = false;
            }
        }
#endif /* defined (ADC_PAL_S32K1xx) ||  defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC) */

        palState->swTrigGroupState.active = false;
        palState->latestGroupIdx          = 0u;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_EnableHardwareTrigger
 * Description   : Enables the selected HW trigger for a conversion group,
 * if the conversion group has support for HW trigger.
 *
 * Implements : ADC_EnableHardwareTrigger_Activity
 * END**************************************************************************/
status_t ADC_EnableHardwareTrigger(const adc_instance_t * const instance,
                                   const uint32_t groupIdx)
{
    DEV_ASSERT(instance != NULL);

    adc_pal_state_t * const palState = ADC_GetPalState(instance);
    const uint32_t instIdx           = instance->instIdx;
    const adc_inst_type_t instType   = instance->instType;

    DEV_ASSERT(groupIdx < palState->numGroups);
    DEV_ASSERT(palState->groupArray != NULL);

    const adc_group_config_t * const currentGroupCfg  = &(palState->groupArray[groupIdx]);
#if defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC)
    const adc_group_state_t * const currentGroupState = ADC_GetHwGroupStatePtr(palState, groupIdx);
#endif /* defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC) */
    status_t status = STATUS_SUCCESS;

    DEV_ASSERT(currentGroupCfg->hwTriggerSupport == true);
    (void)currentGroupCfg;

#if defined (ADC_PAL_SAR)
        if(instType == ADC_INST_TYPE_ADC_SAR)
        {
            (void) instIdx;
            (void) palState;
            (void) groupIdx;
            status = STATUS_UNSUPPORTED;
        }
        else
#endif /* defined (ADC_PAL_SAR) */

#if defined (ADC_PAL_TYPE_SDADC)
        if(instType == ADC_INST_TYPE_SDADC)
        {
            (void) instIdx;
            (void) palState;
            (void) groupIdx;
            status = STATUS_UNSUPPORTED;
        }
        else
#endif /* defined (ADC_PAL_TYPE_SDADC) */

#if defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC)
        {
            if ((palState->swTrigGroupState.active == true) || (currentGroupState->active == true))
            {
                /* A conversion group is already active */
                status = STATUS_BUSY;
            }
            else
            {
                /* Update ADC PAL and group state structures */
                ADC_PalStateUpdateStart(palState, groupIdx);

#if defined (ADC_PAL_S32K1xx)
                if(instType == ADC_INST_TYPE_ADC_S32K1xx)
                {
                    status_t statusTrgmux = STATUS_SUCCESS;

                    /* Enable the ADC interrupt from Interrupt Manager */
                    IRQn_Type adcIrqId;
                    adcIrqId = ADC_DRV_GetInterruptNumber(instIdx);
                    INT_SYS_EnableIRQ(adcIrqId);

                    /* The group shall be configured each time it is enabled. */
                    ADC_ConfigGroup(instIdx, groupIdx, true);

                    /* Enable in TRGMUX the selected HW trigger source for PDB.
                     * Must be called after all PDB pre-triggers have been configured, to make sure no triggers occur during configuration. */
                    statusTrgmux = TRGMUX_DRV_SetTrigSourceForTargetModule(ADC_PAL_TRGMUX_IDX, currentGroupCfg->triggerSource, adcPalTrgmuxTarget[instIdx]);
                    DEV_ASSERT(statusTrgmux == STATUS_SUCCESS);

                    status = STATUS_SUCCESS;
                }
                else
#endif /* defined (ADC_PAL_S32K1xx) */

#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
                if(instType == ADC_INST_TYPE_ADC_SAR_BCTU)
                {
                    const uint8_t numChans = (palState->groupArray[groupIdx]).numChannels;

                    if (numChans == 1u)
                    {
#if ADC_PAL_BCTU_CONV_UPDATE_IRQ_UNROLLED
                        uint8_t adcRelativeIndex; /* ADC index relative to the number of ADC instances per BCTU. E.g. if BCTU is connected to 2 ADC => ADC_3 relative index is 1 */
                        adcRelativeIndex = ADC_GetAdcAbsoluteIdx((palState->groupArray[groupIdx]).inputChannelArray[0u]) % FEATURE_BCTU_NUM_ADC;

                        INT_SYS_EnableIRQ(bctuConvUpdateIrqNum[instIdx][adcRelativeIndex]);
#else
                        INT_SYS_EnableIRQ(bctuConvUpdateIrqNum[instIdx]);
#endif /* ADC_PAL_BCTU_CONV_UPDATE_IRQ_UNROLLED */
                    }
                    else
                    {
                        INT_SYS_EnableIRQ(ADC_PAL_BCTU_LIST_LAST_IRQn);
                    }

                    /* De-activate low-power mode. Any pending triggers will become active, immediately triggering a conversion. */
                    BCTU_DRV_SetLowPowerMode(instIdx, false);

                    /* All groups get configured at ADC PAL initialization,
                     * so here they only need to be enabled. */
                    BCTU_DRV_EnableHwTrigger(instIdx, currentGroupCfg->triggerSource);

                    status = STATUS_SUCCESS;
                }
                else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_BCTU) */

#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)
                if(instType == ADC_INST_TYPE_ADC_SAR_CTU)
                {
                    /* internalTrigIdx is mapped 1:1 with stateIdx */
                    const uint32_t internalTrigIdx = palState->stateIdxMapping[groupIdx];
                    const uint32_t shift           = internalTrigIdx * CTU_THCR1_T1_ADCE_SHIFT;
                    CTU_Type * const ctuInstBase   = ctuBase[instIdx];

                    /* If GRE has been previously set (for example by a previous call to ADC_EnableHardwareTrigger)
                     * temporarily clear it, to allow writes to double buffered THCR register to take effect.
                     * NOTE: MRS signal must not occur when a new value is available in a double buffered register
                     * and GRE is disabled. If this happens, the current group will not execute for the first trigger
                     * and EFR->MRS_RE will be set. */
                    CTU_DRV_DisableGeneralReload(instIdx);

                    /* Enable internal trigger corresponding to current groupIdx */
#if NUM_MAX_HW_TRIG_GROUPS > 4u
#error "Write operation to THCR register assumes that fifoIdx < 4 "
#endif /* NUM_MAX_HW_TRIG_GROUPS > 4 */
                    ctuInstBase->THCR1 |= (uint32_t)CTU_OUTPUT_TRIG_EN << shift; /* Assumes: internalTrigIdx < 4 */

                    /* THCR is double-buffered, so mark that MRS can reload the value */
                    CTU_DRV_EnableGeneralReload(instIdx);

                    status = STATUS_SUCCESS;
                }
                else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_CTU) */

#if defined (ADC_PAL_TYPE_EQADC)
                if(instType == ADC_INST_TYPE_EQADC)
                {
                    /* Install handler for CFIFO fill request - interrupt issued when CFIFO is not full or current conversion command is not marked as End of Queue
                     * NOTE: make sure EQADC and CFIFO indexes match the ones actually used */
                    INT_SYS_InstallHandler(eqadcCfifoFillIRQn[instIdx][EQADC_CFIFO_IDX_HW_TRIG], eqadcCfifoFillHandlers[instIdx][EQADC_CFIFO_IDX_HW_TRIG], NULL);
                    INT_SYS_EnableIRQ(eqadcCfifoFillIRQn[instIdx][EQADC_CFIFO_IDX_HW_TRIG]);

                    /* Install handler for RFIFO drain request - interrupt issued when results are available in the selected RFIFO
                     * NOTE: make sure EQADC and RFIFO indexes match the ones actually used */
                    INT_SYS_InstallHandler(eqadcRfifoFillIRQn[instIdx][EQADC_CFIFO_IDX_HW_TRIG], eqadcRfifoFillHandlers[instIdx][EQADC_RFIFO_IDX_HW_TRIG], NULL);
                    INT_SYS_EnableIRQ(eqadcRfifoFillIRQn[instIdx][EQADC_CFIFO_IDX_HW_TRIG]);

                    /* CFIFO status needs to be in IDLE STATE to update */
                    DEV_ASSERT(EQADC_DRV_GetCfifoStatusCurrent(instIdx, EQADC_CFIFO_IDX_HW_TRIG) == CFIFO_STATUS_IDLE);

                    /* Set CFIFO operating mode according to Trigger Source Mode assigned for current HW triggered group */
                    const uint16_t trigSrcMode = ((uint16_t)currentGroupCfg->triggerSource & ADC_TRIGGER_SOURCE_MODE_MASK) >> ADC_TRIGGER_SOURCE_MODE_OFFSET;
                    const adc_trigger_source_mode_t triggerSourceMode = (adc_trigger_source_mode_t) trigSrcMode;
                    const uint8_t triggerSourceId = ((uint16_t)currentGroupCfg->triggerSource & ADC_TRIGGER_SOURCE_ID_MASK);

                    /* Check that trigger mode is in range of elements in adc_trigger_source_mode_t enum */
                    DEV_ASSERT((ADC_TRIG_SOURCE_MODE_LOW_LEVEL <= triggerSourceMode) && (triggerSourceMode <= ADC_TRIG_SOURCE_MODE_BOTH_EDGES));

                    EQADC_SetCfifoOpMode(instIdx, EQADC_CFIFO_IDX_HW_TRIG, (uint32_t)triggerSourceMode);

                    EQADC_SetCfifoTrig(instIdx, EQADC_CFIFO_IDX_HW_TRIG, triggerSourceId);

                    /* Enable detection of trigger events */
                    EQADC_DRV_SetSingleScanEnBit(instIdx, EQADC_CFIFO_IDX_HW_TRIG, false);

                    /* Enable interrupt requests after configuration has ended */
                    EQADC_DRV_EnableIntReq(instIdx, EQADC_CFIFO_IDX_HW_TRIG, EQADC_INT_EN_CFIFO_FILL);
                    EQADC_DRV_EnableIntReq(instIdx, EQADC_RFIFO_IDX_HW_TRIG, EQADC_INT_EN_RFIFO_DRAIN);

                    status = STATUS_SUCCESS;
                }
                else
#endif /* defined (ADC_PAL_TYPE_EQADC) */
                {
                    DEV_ASSERT(false);
                }
            }
        }
#else /* defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC) */
        {
            DEV_ASSERT(false);
        }
#endif /* defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC) */

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DisableHardwareTrigger
 * Description   : Disables the selected HW trigger for a conversion group,
 * if the conversion group has support for HW trigger.
 *
 * Implements : ADC_DisableHardwareTrigger_Activity
 * END**************************************************************************/
status_t ADC_DisableHardwareTrigger(const adc_instance_t * const instance,
                                    const uint32_t groupIdx,
                                    const uint32_t timeout)
{
    DEV_ASSERT(instance != NULL);

    adc_pal_state_t * const palState = ADC_GetPalState(instance);
    const uint32_t instIdx           = instance->instIdx;
    const adc_inst_type_t instType   = instance->instType;

#if defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU)
    adc_group_state_t * const currentGroupState = ADC_GetHwGroupStatePtr(palState, groupIdx);
#elif defined (ADC_PAL_TYPE_EQADC)
    const adc_group_state_t * const currentGroupState = ADC_GetHwGroupStatePtr(palState, groupIdx);
#endif /* defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) */

    DEV_ASSERT(groupIdx < palState->numGroups);
    DEV_ASSERT(palState->groupArray != NULL);
    DEV_ASSERT(palState->groupArray[groupIdx].hwTriggerSupport == true);

    status_t status = STATUS_SUCCESS;

#if defined (ADC_PAL_SAR)
    if(instType == ADC_INST_TYPE_ADC_SAR)
    {
        (void) instIdx;
        (void) palState;
        (void) timeout;
        (void) groupIdx;
        status = STATUS_UNSUPPORTED;
    }
    else
#endif /* defined (ADC_PAL_SAR) */

#if defined (ADC_PAL_TYPE_SDADC)
    if(instType == ADC_INST_TYPE_SDADC)
    {
        (void) instIdx;
        (void) palState;
        (void) timeout;
        (void) groupIdx;
        status = STATUS_UNSUPPORTED;
    }
    else
#endif /* defined (ADC_PAL_TYPE_SDADC) */

#if defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC)
    {
        /* Make sure no more notifications are received from current groupIdx */
        status = ADC_DisableNotification(instance, groupIdx);

        if ((currentGroupState->active == true) && (status == STATUS_SUCCESS))
        {
#if defined (ADC_PAL_S32K1xx)
            if(instType == ADC_INST_TYPE_ADC_S32K1xx)
            {
                /* Disable from TRGMUX the trigger source for PDB */
                status = TRGMUX_DRV_SetTrigSourceForTargetModule(ADC_PAL_TRGMUX_IDX, TRGMUX_TRIG_SOURCE_DISABLED, adcPalTrgmuxTarget[instIdx]);
                DEV_ASSERT(status == STATUS_SUCCESS);

                status = ADC_StopGroupBlocking(instIdx, timeout);

                if (status == STATUS_SUCCESS)
                {
                    currentGroupState->active = false;
                }
            }
            else
#endif /* defined (ADC_PAL_S32K1xx) */

#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
            if(instType == ADC_INST_TYPE_ADC_SAR_BCTU)
            {
                const adc_trigger_source_t triggerSource = palState->groupArray[groupIdx].triggerSource;
                const uint8_t numChans                   = palState->groupArray[groupIdx].numChannels;
                uint32_t startTime, deltaTime;

                if (numChans == 1u)
                {
#if ADC_PAL_BCTU_CONV_UPDATE_IRQ_UNROLLED
                    uint8_t adcRelativeIndex; /* ADC index relative to the number of ADC instances per BCTU. E.g. if BCTU is connected to 2 ADC => ADC_3 relative index is 1 */
                    adcRelativeIndex = ADC_GetAdcAbsoluteIdx((palState->groupArray[groupIdx]).inputChannelArray[0u]) % FEATURE_BCTU_NUM_ADC;

                    INT_SYS_DisableIRQ(bctuConvUpdateIrqNum[instIdx][adcRelativeIndex]);
#else
                    INT_SYS_DisableIRQ(bctuConvUpdateIrqNum[instIdx]);
#endif /* ADC_PAL_BCTU_CONV_UPDATE_IRQ_UNROLLED */
                }
                else
                {
                    INT_SYS_DisableIRQ(ADC_PAL_BCTU_LIST_LAST_IRQn);
                }

                OSIF_TimeDelay(0u); /* Make sure OSIF timer is initialized. */

                startTime = OSIF_GetMilliseconds();
                deltaTime = 0u;

                /* Prevent new conversions from starting */
                BCTU_DRV_DisableHwTrigger(instIdx, triggerSource);

                /* Check if conversion is still running until execution finished or timeout occurred */
                while ((BCTU_DRV_IsConvRunning(instIdx, triggerSource) == true) && (deltaTime < timeout))
                {
                    deltaTime = OSIF_GetMilliseconds() - startTime;
                }

                if (deltaTime >= timeout)
                {
                    status = STATUS_TIMEOUT;
                }
                else
                {
                    status = STATUS_SUCCESS;

                    /* Activate low-power mode */
                    BCTU_DRV_SetLowPowerMode(instIdx, true);

                    currentGroupState->active = false;
                }
            }
            else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_BCTU) */

#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)
            if(instType == ADC_INST_TYPE_ADC_SAR_CTU)
            {
                /* internalTrigIdx is mapped 1:1 with stateIdx */
                const uint32_t internalTrigIdx = palState->stateIdxMapping[groupIdx];
                const uint32_t shift           = internalTrigIdx * CTU_THCR1_T1_ADCE_SHIFT;
                CTU_Type * const ctuInstBase   = ctuBase[instIdx];

                (void)timeout;

                /* If GRE has been previously set (for example by a previous call to ADC_DisableHardwareTrigger)
                 * temporarily clear it, to allow writes to double buffered THCR register to take effect.
                 * NOTE: MRS signal must not occur when a new value is available in a double buffered register
                 * and GRE is disabled. If this happens, the current group will not execute for the first trigger
                 * and EFR->MRS_RE will be set. */
                CTU_DRV_DisableGeneralReload(instIdx);

                /* Disable internal trigger corresponding to current groupIdx */
                /* Register is double-buffered so the new value will not have effect in the current control cycle
                 * - only after a new MRS occurs */
#if NUM_MAX_HW_TRIG_GROUPS > 4u
#error "Write operation to THCR register assumes that fifoIdx < 4 "
#endif /* NUM_MAX_HW_TRIG_GROUPS > 4 */
                ctuInstBase->THCR1 &= ~((uint32_t)CTU_OUTPUT_TRIG_EN << shift); /* Assumes: fifoIdx < 4 */

                /* THCR is double-buffered, so mark that MRS can reload the value. */
                CTU_DRV_EnableGeneralReload(instIdx);

                currentGroupState->active = false;
            }
            else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_CTU) */

#if defined (ADC_PAL_TYPE_EQADC)
            if(instType == ADC_INST_TYPE_EQADC)
            {
                status = EQADC_StopConvProcedure(instIdx, EQADC_RFIFO_IDX_HW_TRIG, timeout);

                if(status == STATUS_SUCCESS)
                {
                    palState->hwTrigGroupState[0u].active         = false; /*Support only a single active group at a time, so any active groupIdx will be mapped to state idx 0 */
                    palState->hwTrigGroupState[0u].currentConvIdx = 0u;
                }
            }
            else
#endif /* defined (ADC_PAL_TYPE_EQADC) */
            {
                DEV_ASSERT(false);
            }
        }
    }
#else /* defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC) */
    {
        DEV_ASSERT(false);
    }
#endif /* defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC) */

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_StartGroupConversion
 * Description   : Starts the execution of a selected ADC conversion group.
 *
 * Implements : ADC_StartGroupConversion_Activity
 * END**************************************************************************/
status_t ADC_StartGroupConversion(const adc_instance_t * const instance,
                                  const uint32_t groupIdx)
{
    DEV_ASSERT(instance != NULL);

    adc_pal_state_t * const palState = ADC_GetPalState(instance);
    const uint32_t instIdx           = instance->instIdx;
    const adc_inst_type_t instType   = instance->instType;

    DEV_ASSERT(groupIdx < palState->numGroups);
    DEV_ASSERT(palState->groupArray != NULL);
    DEV_ASSERT(palState->groupArray[groupIdx].hwTriggerSupport == false);

    status_t status;

#if (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_SAR))
    const adc_group_config_t * currentGroupCfg  = &(palState->groupArray[groupIdx]);
    uint8_t idx;
    uint32_t currentChan;
#endif /* (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_SAR)) */

    if ((ADC_AnyHwTrigGroupActive(instance) == true) || (palState->swTrigGroupState.active == true))
    {
        /* Any other conversion group is already active */
        status = STATUS_BUSY;
    }
    else
    {
        /* Update ADC PAL and group state structures */
        ADC_PalStateUpdateStart(palState, groupIdx);

#if defined (ADC_PAL_S32K1xx)
        if(instType == ADC_INST_TYPE_ADC_S32K1xx)
        {
            DEV_ASSERT(palState->groupArray[groupIdx].numChannels <= ADC_PAL_MAX_CONVS_IN_GROUP);

            bool hwTriggerEnabled = false;

            /* Enable the ADC interrupt from Interrupt Manager */
            IRQn_Type adcIrqId;
            adcIrqId = ADC_DRV_GetInterruptNumber(instIdx);
            INT_SYS_EnableIRQ(adcIrqId);

            ADC_ConfigGroup(instIdx, groupIdx, hwTriggerEnabled);

            /* Sw trigger PDB */
            PDB_DRV_SoftTriggerCmd(instIdx);
        }
        else
#endif /* defined (ADC_PAL_S32K1xx) */

#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
        if(instType == ADC_INST_TYPE_ADC_SAR_BCTU)
        {
            /* Assumes that all channels in the group target the same ADC instance */
            const uint32_t adcIdx = ADC_GetAdcAbsoluteIdx(currentGroupCfg->inputChannelArray[0u]);
            (void) instIdx;

            INT_SYS_InstallHandler(adcEocIrqNums[adcIdx], adcEocIrqHandlers[adcIdx], NULL);
            INT_SYS_EnableIRQ(adcEocIrqNums[adcIdx]);

            for (idx = 0u; idx < currentGroupCfg->numChannels; idx++)
            {
                DEV_ASSERT(adcIdx == ADC_GetAdcAbsoluteIdx(currentGroupCfg->inputChannelArray[idx])); /* channels in the group must target the same ADC instance */
                currentChan = ADC_GetChanIdx(currentGroupCfg->inputChannelArray[idx]);

                ADC_DRV_EnableChannel(adcIdx, ADC_CONV_CHAIN_NORMAL, currentChan);
            }

            ADC_DRV_StartConversion(adcIdx, ADC_CONV_CHAIN_NORMAL);
        }
        else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_BCTU) */

#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)
        if(instType == ADC_INST_TYPE_ADC_SAR_CTU)
        {
            /* Assumes that all channels in the group target the same ADC instance */
            const uint32_t adcIdx = ADC_GetAdcAbsoluteIdx(currentGroupCfg->inputChannelArray[0u]);
            (void) instIdx;

            adcBase[adcIdx]->MCR &= ~(ADC_MCR_CTUEN(1u)); /* set ADC to CTU MODE DISABLED, to allow SW triggering */

            INT_SYS_InstallHandler(adcEocIrqNums[adcIdx], adcEocIrqHandlers[adcIdx], NULL);
            INT_SYS_EnableIRQ(adcEocIrqNums[adcIdx]);

            for (idx = 0u; idx < currentGroupCfg->numChannels; idx++)
            {
                DEV_ASSERT(adcIdx == ADC_GetAdcAbsoluteIdx(currentGroupCfg->inputChannelArray[idx])); /* channels in the group must target the same ADC instance */
                currentChan = ADC_GetChanIdx(currentGroupCfg->inputChannelArray[idx]);

                ADC_DRV_EnableChannel(adcIdx, ADC_CONV_CHAIN_NORMAL, currentChan);
            }

            ADC_DRV_StartConversion(adcIdx, ADC_CONV_CHAIN_NORMAL);
        }
        else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_CTU) */

#if defined (ADC_PAL_SAR)
        if(instType == ADC_INST_TYPE_ADC_SAR)
        {
            const uint32_t adcIdx = instIdx;

            INT_SYS_InstallHandler(adcEocIrqNums[adcIdx], adcEocIrqHandlers[adcIdx], NULL);
            INT_SYS_EnableIRQ(adcEocIrqNums[adcIdx]);

            for (idx = 0u; idx < currentGroupCfg->numChannels; idx++)
            {
                currentChan = ADC_GetChanIdx(currentGroupCfg->inputChannelArray[idx]);

                ADC_DRV_EnableChannel(adcIdx, ADC_CONV_CHAIN_NORMAL, currentChan);
            }

            ADC_DRV_StartConversion(adcIdx, ADC_CONV_CHAIN_NORMAL);
        }
        else
#endif /* defined (ADC_PAL_SAR) */

#if defined (ADC_PAL_TYPE_SDADC)
        if(instType == ADC_INST_TYPE_SDADC)
        {
            const uint32_t adcIdx = instIdx;

            /* Config and start conversion for first conversion in the group */
            SDADC_StartConversion(adcIdx, groupIdx, 0u);
        }
        else
#endif /* defined (ADC_PAL_TYPE_SDADC) */

#if defined (ADC_PAL_TYPE_EQADC)
        if(instType == ADC_INST_TYPE_EQADC)
        {
            /* Install handler for CFIFO fill request - interrupt issued when CFIFO is not full or current conversion command is not marked as End of Queue
             * NOTE: make sure EQADC and CFIFO indexes match the ones actually used */
            INT_SYS_InstallHandler(eqadcCfifoFillIRQn[instIdx][EQADC_CFIFO_IDX_SW_TRIG], eqadcCfifoFillHandlers[instIdx][EQADC_CFIFO_IDX_SW_TRIG], NULL);
            INT_SYS_EnableIRQ(eqadcCfifoFillIRQn[instIdx][EQADC_CFIFO_IDX_SW_TRIG]);

            /* Install handler for RFIFO drain request - interrupt issued when results are available in the selected RFIFO
             * NOTE: make sure EQADC and RFIFO indexes match the ones actually used */
            INT_SYS_InstallHandler(eqadcRfifoFillIRQn[instIdx][EQADC_CFIFO_IDX_SW_TRIG], eqadcRfifoFillHandlers[instIdx][EQADC_RFIFO_IDX_SW_TRIG], NULL);
            INT_SYS_EnableIRQ(eqadcRfifoFillIRQn[instIdx][EQADC_CFIFO_IDX_SW_TRIG]);

            EQADC_DRV_EnableIntReq(instIdx, EQADC_CFIFO_IDX_SW_TRIG, EQADC_INT_EN_CFIFO_FILL);
            EQADC_DRV_EnableIntReq(instIdx, EQADC_RFIFO_IDX_SW_TRIG, EQADC_INT_EN_RFIFO_DRAIN);

            /* Make sure the CFIFO used for SW triggered groups is configured as software triggered mode 
            Assumption based on PAL state machine: CFIFO mode previously configured is DISABLED - EQADC only accepts transition from DISABLED to non-DISABLED */
            EQADC_SetCfifoOpMode(instIdx, EQADC_CFIFO_IDX_SW_TRIG, (uint32_t) EQADC_CFIFO_MODE_SW_TRIG_SINGLE);

            EQADC_DRV_SetSingleScanEnBit(instIdx, EQADC_CFIFO_IDX_SW_TRIG, false);
        }
        else
#endif /* defined (ADC_PAL_TYPE_EQADC) */
        {
            DEV_ASSERT(false);
        }

        status = STATUS_SUCCESS;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_StopGroupConversion
 * Description   : Stops the execution of the ADC group currently executing.
 *
 * Implements : ADC_StopGroupConversion_Activity
 * END**************************************************************************/
status_t ADC_StopGroupConversion(const adc_instance_t * const instance,
                                 const uint32_t groupIdx,
                                 const uint32_t timeout)
{
    DEV_ASSERT(instance != NULL);

    adc_pal_state_t * const palState = ADC_GetPalState(instance);
    const uint32_t instIdx           = instance->instIdx;
    const adc_inst_type_t instType   = instance->instType;

    DEV_ASSERT(groupIdx < palState->numGroups);
    DEV_ASSERT(palState->groupArray != NULL);
    DEV_ASSERT(palState->groupArray[groupIdx].hwTriggerSupport == false);

    status_t status = STATUS_ERROR;

    if (palState->swTrigGroupState.active == true)
    {
#if defined (ADC_PAL_S32K1xx)
        if(instType == ADC_INST_TYPE_ADC_S32K1xx)
        {
            (void)groupIdx;

            /* Disable the ADC interrupt from Interrupt Manager */
            IRQn_Type adcIrqId;
            adcIrqId = ADC_DRV_GetInterruptNumber(instIdx);
            INT_SYS_DisableIRQ(adcIrqId);

            status = ADC_StopGroupBlocking(instIdx, timeout);

            if (status == STATUS_SUCCESS)
            {
                palState->swTrigGroupState.active = false;
            }
        }
        else
#endif /* defined (ADC_PAL_S32K1xx) */

#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
        if(instType == ADC_INST_TYPE_ADC_SAR_BCTU)
        {
            const adc_group_config_t * currentGroupCfg  = &(palState->groupArray[groupIdx]);

            status = ADC_SAR_StopConversion(instIdx, currentGroupCfg, timeout);

            if (status == STATUS_SUCCESS)
            {
                palState->swTrigGroupState.active = false;
            }
        }
        else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_BCTU) */

#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)
        if(instType == ADC_INST_TYPE_ADC_SAR_CTU)
        {
            const adc_group_config_t * currentGroupCfg  = &(palState->groupArray[groupIdx]);

            status = ADC_SAR_StopConversion(instIdx, currentGroupCfg, timeout);

            if (status == STATUS_SUCCESS)
            {
                palState->swTrigGroupState.active = false;
            }
       }
        else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_CTU) */

#if defined (ADC_PAL_SAR)
        if(instType == ADC_INST_TYPE_ADC_SAR)
        {
            const adc_group_config_t * currentGroupCfg  = &(palState->groupArray[groupIdx]);

            status = ADC_SAR_StopConversion(instIdx, currentGroupCfg, timeout);

            if (status == STATUS_SUCCESS)
            {
                palState->swTrigGroupState.active = false;
            }
        }
        else
#endif /* defined (ADC_PAL_SAR) */

#if defined (ADC_PAL_TYPE_SDADC)
        if(instType == ADC_INST_TYPE_SDADC)
        {
            (void) timeout;

            /* Prevent other conversion from starting */
            SDADC_DRV_Powerdown(instIdx);

            /* Clear any other conversions which might have completed in the meantime */
            SDADC_DRV_ClearStatusFlags(instIdx, SDADC_FLAG_DATA_FIFO_FULL);

            palState->swTrigGroupState.active = false;

            status = STATUS_SUCCESS;
        }
        else
#endif /* defined (ADC_PAL_TYPE_SDADC) */

#if defined (ADC_PAL_TYPE_EQADC)
        if(instType == ADC_INST_TYPE_EQADC)
        {
            status = EQADC_StopConvProcedure(instIdx, EQADC_RFIFO_IDX_SW_TRIG, timeout);

            if(status == STATUS_SUCCESS)
            {
                palState->swTrigGroupState.active         = false;
                palState->swTrigGroupState.currentConvIdx = 0u;
            }
        }
        else
#endif /* defined (ADC_PAL_TYPE_EQADC) */
        {
            DEV_ASSERT(false);
        }
    }
    else
    {
        status = STATUS_ERROR;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_EnableNotification
 * Description   : Enables the notification callback for a configured group.
 *
 * Implements : ADC_EnableNotification_Activity
 * END**************************************************************************/
status_t ADC_EnableNotification(const adc_instance_t * const instance,
                                const uint32_t groupIdx)
{
    DEV_ASSERT(instance != NULL);

    adc_pal_state_t * const palState = ADC_GetPalState(instance);
    const adc_group_config_t * currentGroupCfg = &(palState->groupArray[groupIdx]);
    adc_group_state_t * groupState;

    DEV_ASSERT(groupIdx < palState->numGroups);

    status_t status = STATUS_SUCCESS;

#if (defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC))
    if (currentGroupCfg->hwTriggerSupport == true)
    {
        groupState = ADC_GetHwGroupStatePtr(palState, groupIdx);
    }
    else
    {
        groupState = &(palState->swTrigGroupState);
    }
#elif (defined (ADC_PAL_SAR) || defined (ADC_PAL_TYPE_SDADC))
    (void) currentGroupCfg;
    groupState = &(palState->swTrigGroupState);

#endif /* (defined (ADC_PAL_S32K1xx) || (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC)) */

    if (groupState->active == false)
    {
        status = STATUS_ERROR;
    }
    else
    {
        DEV_ASSERT(palState->groupArray[groupIdx].callback != NULL);

        groupState->notificationEn = true;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_DisableNotification
 * Description   : Disables the notification callback for a configured group.
 *
 * Implements : ADC_DisableNotification_Activity
 * END**************************************************************************/
status_t ADC_DisableNotification(const adc_instance_t * const instance,
                                 const uint32_t groupIdx)
{
    DEV_ASSERT(instance != NULL);

    adc_pal_state_t * const palState = ADC_GetPalState(instance);
    const adc_group_config_t * currentGroupCfg = &(palState->groupArray[groupIdx]);
    adc_group_state_t * groupState;

    DEV_ASSERT(groupIdx < palState->numGroups);

    status_t status = STATUS_SUCCESS;

#if (defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC))
    if (currentGroupCfg->hwTriggerSupport == true)
    {
        groupState = ADC_GetHwGroupStatePtr(palState, groupIdx);
    }
    else
    {
        groupState = &(palState->swTrigGroupState);
    }
#elif (defined (ADC_PAL_SAR) || defined (ADC_PAL_TYPE_SDADC))
    (void) currentGroupCfg;
    groupState = &(palState->swTrigGroupState);

#endif /* (defined (ADC_PAL_S32K1xx) || (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC)) */

    if (groupState->active == false)
    {
        status = STATUS_ERROR;
    }
    else
    {
       groupState->notificationEn = false;
    }

    return status;
}


/*******************************************************************************
 * Private Functions
 ******************************************************************************/

#if defined (ADC_PAL_S32K1xx)

static status_t ADC_Init_S32K1xx(const uint32_t instance,
                                 const adc_config_t * const config)
{
    const extension_adc_s32k1xx_t * const extension = (extension_adc_s32k1xx_t *)(config->extension);
    adc_converter_config_t adcCfg;
    status_t status;

    DEV_ASSERT(extension->inputClock <= NUMBER_OF_ALT_CLOCKS);

#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    {
        uint16_t idx;
        uint16_t idx0, idx1;
        const adc_group_config_t * currentGroupCfg;

        /* Iterate over all conversion groups */
        for (idx = 0u; idx < adcPalState[instance].numGroups; idx++)
        {
            currentGroupCfg = &(adcPalState[instance].groupArray[idx]);
            /* Error if number of conversions is larger than max supported */
            DEV_ASSERT(currentGroupCfg->numChannels <= ADC_PAL_MAX_CONVS_IN_GROUP);

            if (currentGroupCfg->delayType == ADC_DELAY_TYPE_INDIVIDUAL_DELAY)
            {
                /* Delay values are measured relative to the trigger event.
                 * The pretriggers must not occur while another conversion in the group is running, otherwise the ADC freezes.
                 * This code only checks that the values are not identical, but it is the users' responsibility
                 * to make sure they do not overlap, i.e. delayN_plus_1 > (delayN + conversion_duration) */
                for (idx0 = 0u; idx0 < currentGroupCfg->numChannels; idx0++)
                {
                    for (idx1 = idx0 + 1u; idx1 < currentGroupCfg->numChannels; idx1++)
                    {
                        DEV_ASSERT(currentGroupCfg->delayArray[idx0] != currentGroupCfg->delayArray[idx1]);
                    }
                }
            }
        }
    }
#endif /* defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT) */

    /* ADC configuration */
    ADC_DRV_Reset(instance);

    ADC_DRV_InitConverterStruct(&adcCfg);
    adcCfg.trigger                = ADC_TRIGGER_HARDWARE; /* ADC in hw trigger mode - sw triggering shall be done via PDB */
    adcCfg.sampleTime             = config->sampleTicks;
    adcCfg.clockDivide            = extension->clockDivide;
    adcCfg.resolution             = extension->resolution;
    adcCfg.inputClock             = extension->inputClock;
    adcCfg.voltageRef             = extension->voltageRef;
    adcCfg.supplyMonitoringEnable = extension->supplyMonitoringEnable;
    adcCfg.pretriggerSel          = ADC_PRETRIGGER_SEL_PDB; /* configure pretriggers 0->3 to be routed from PDB */

    ADC_DRV_ConfigConverter(instance, &adcCfg);

    ADC_DRV_AutoCalibration(instance);

    /* PDB init shall only be called from StartConversion() & EnableHardwareTrigger()
     * because PDB input source and continuous conversion enable need to be configured for each call. */

    /* Only reset the trgmux target register corresponding to PDB instance.
     * Calling TRGMUX init would reset all TRGMUX target registers - affecting other modules. */
    status = TRGMUX_DRV_SetTrigSourceForTargetModule(ADC_PAL_TRGMUX_IDX, TRGMUX_TRIG_SOURCE_DISABLED, adcPalTrgmuxTarget[instance]);
    if (status != STATUS_SUCCESS)
    {
        status = STATUS_ERROR;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_S32K1xx_IrqHandler
 * Description   : Interrupt handler functionality to be called from
 * interrupt handlers for each ADC.
 * Implements : ADC_Interrupt_Activity
 * END**************************************************************************/
void ADC_S32K1xx_IrqHandler(const uint32_t instIdx)
{
    adc_pal_state_t * const palState = &(adcPalState[instIdx]);
    const uint32_t currentGroupIdx   = palState->latestGroupIdx;
    uint8_t controlChanIdx           = 0u;
    const adc_group_config_t * activeGroupCfg;
    adc_group_state_t * groupState;

    activeGroupCfg = &(palState->groupArray[currentGroupIdx]);

    if (activeGroupCfg->hwTriggerSupport == false)
    {
        groupState = &(palState->swTrigGroupState);
    }
    else
    {
        groupState = &(palState->hwTrigGroupState[0u]); /* A single HW trigger enabled is supported by this platform */
    }

    uint16_t * result = &(activeGroupCfg->resultBuffer[groupState->currentBufferOffset]);

    /* Read all conversion results */
    for (controlChanIdx = 0u; controlChanIdx < activeGroupCfg->numChannels; controlChanIdx++)
    {
        ADC_DRV_GetChanResult(instIdx, controlChanIdx, result); /* interrupt flag is cleared when reading the result */
        result++;
    }

    /* Increment offset in result buffer */
    groupState->currentBufferOffset = (uint16_t)((groupState->currentBufferOffset + activeGroupCfg->numChannels) % groupState->bufferLength);


    if (activeGroupCfg->hwTriggerSupport == false) /* Continuous mode currently supported only for SW triggered groups */
    {
        if (activeGroupCfg->continuousConvEn == true)
        {
            /* Sw trigger PDB */
            PDB_DRV_SoftTriggerCmd(instIdx);
        }
        else
        {
            groupState->active = false;
        }
    }

    /* Call notification callback, if it is enabled */
    ADC_CallNotificationCb(palState, currentGroupIdx, groupState);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_ConfigPdbAndPretriggers
 * Description   : Configures PDB global parameters, PDB pretriggers and delays (if used)
 *
 * END**************************************************************************/
static inline void ADC_ConfigPdbAndPretriggers(const uint32_t instIdx,
                                               const pdb_trigger_src_t trgSrc,
                                               const adc_group_config_t * currentGroupCfg)
{
    pdb_timer_config_t pdbCfg;
    pdb_adc_pretrigger_config_t pdbPretrigCfg;
    uint8_t idx;

    /* PDB driver instance configuration */
    pdbCfg.loadValueMode        = PDB_LOAD_VAL_IMMEDIATELY;
    pdbCfg.seqErrIntEnable      = false;
    pdbCfg.clkPreDiv            = adcPalState[instIdx].pdbPrescaler;
    pdbCfg.clkPreMultFactor     = PDB_CLK_PREMULT_FACT_AS_1;
    pdbCfg.dmaEnable            = false;
    pdbCfg.intEnable            = false;
    pdbCfg.continuousModeEnable = false; /* Continuous mode refers to Counter being reset at zero - not used in ADC PAL */

    pdbCfg.triggerInput         = trgSrc;
    PDB_DRV_Init(instIdx, &pdbCfg);

    if (currentGroupCfg->delayType == ADC_DELAY_TYPE_NO_DELAY)
    {
        /* PDB pre-triggers configuration */
        pdbPretrigCfg.preTriggerEnable           = true;
        pdbPretrigCfg.preTriggerOutputEnable     = false; /* pretrigger asserts one clock cycle after input trigger is asserted (hw or sw)  */
        pdbPretrigCfg.preTriggerBackToBackEnable = false; /* the first pretrigger in the group must have BB disabled */
        pdbPretrigCfg.adcPreTriggerIdx           = 0u;
        PDB_DRV_ConfigAdcPreTrigger(instIdx, ADC_PAL_PDB_CHAN, &pdbPretrigCfg);

        pdbPretrigCfg.preTriggerBackToBackEnable = true; /* the rest of pretriggers in the group must have BB enabled */
        for (idx = 1u; idx < currentGroupCfg->numChannels; idx++)
        {
            pdbPretrigCfg.adcPreTriggerIdx = idx;
            PDB_DRV_ConfigAdcPreTrigger(instIdx, ADC_PAL_PDB_CHAN, &pdbPretrigCfg);
        }
    }
    else if (currentGroupCfg->delayType == ADC_DELAY_TYPE_GROUP_DELAY)
    {
        DEV_ASSERT(currentGroupCfg->delayArray != NULL);

        /* PDB pre-triggers configuration */
        pdbPretrigCfg.preTriggerEnable           = true;
        pdbPretrigCfg.preTriggerOutputEnable     = true;  /* the first pretrigger asserts when counter reaches delay value plus one */
        pdbPretrigCfg.preTriggerBackToBackEnable = false; /* the first pretrigger in the group must have BB disabled */
        pdbPretrigCfg.adcPreTriggerIdx           = 0u;
        PDB_DRV_ConfigAdcPreTrigger(instIdx, ADC_PAL_PDB_CHAN, &pdbPretrigCfg);

        PDB_DRV_SetAdcPreTriggerDelayValue(instIdx, ADC_PAL_PDB_CHAN, pdbPretrigCfg.adcPreTriggerIdx, *(currentGroupCfg->delayArray));

        pdbPretrigCfg.preTriggerOutputEnable     = false; /* the rest of pretriggers in the group must ignore the delay value */
        pdbPretrigCfg.preTriggerBackToBackEnable = true;  /* the rest of pretriggers in the group must have BB enabled */
        for (idx = 1u; idx < currentGroupCfg->numChannels; idx++)
        {
            pdbPretrigCfg.adcPreTriggerIdx = idx;
            PDB_DRV_ConfigAdcPreTrigger(instIdx, ADC_PAL_PDB_CHAN, &pdbPretrigCfg);
        }
    }
    else /* corresponds to currentGroupCfg->delayType == ADC_DELAY_TYPE_INDIVIDUAL_DELAY */
    {
        DEV_ASSERT(currentGroupCfg->delayArray != NULL);

        /* PDB pre-triggers configuration */
        pdbPretrigCfg.preTriggerEnable           = true;
        pdbPretrigCfg.preTriggerOutputEnable     = true;  /* each pretrigger asserts when counter reaches corresponding delay value plus one */
        pdbPretrigCfg.preTriggerBackToBackEnable = false; /* all pretriggers in the group must have BB disabled */
        for (idx = 0u; idx < currentGroupCfg->numChannels; idx++)
        {
            pdbPretrigCfg.adcPreTriggerIdx = idx;
            PDB_DRV_ConfigAdcPreTrigger(instIdx, ADC_PAL_PDB_CHAN, &pdbPretrigCfg);

            PDB_DRV_SetAdcPreTriggerDelayValue(instIdx, ADC_PAL_PDB_CHAN, pdbPretrigCfg.adcPreTriggerIdx, currentGroupCfg->delayArray[idx]);
        }
    }

    PDB_DRV_Enable(instIdx);
    PDB_DRV_LoadValuesCmd(instIdx);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_ConfigGroup
 * Description   : Configures ADC input channels and PDB for a conversion group
 *
 * END**************************************************************************/
static void ADC_ConfigGroup(const uint32_t instance,
                            const uint32_t groupIdx,
                            const bool hwTriggerFlag)
{
    const adc_group_config_t * const currentGroupCfg  = &(adcPalState[instance].groupArray[groupIdx]);

    pdb_trigger_src_t pdbTrigSrc;
    if (hwTriggerFlag == true)
    {
        pdbTrigSrc = PDB_TRIGGER_IN0;
    }
    else
    {
        pdbTrigSrc = PDB_SOFTWARE_TRIGGER;
        /* SW triggered groups do not support delays */
        DEV_ASSERT(currentGroupCfg->delayType == ADC_DELAY_TYPE_NO_DELAY);
    }

    /* Configure PDB instance and pre-triggers */
    ADC_ConfigPdbAndPretriggers(instance, pdbTrigSrc, currentGroupCfg);

    /* Configure ADC channels */
    adc_chan_config_t adcChanCfg;
    uint8_t idx;

    adcChanCfg.interruptEnable = false; /* interrupt is disabled for all conversion, except the last one in the group */
    for (idx = 0u; idx < (currentGroupCfg->numChannels - 1u); idx++)
    {
        adcChanCfg.channel = currentGroupCfg->inputChannelArray[idx]; /* set the ADC input channel */

        ADC_DRV_ConfigChan(instance, idx, &adcChanCfg); /* conversion complete flag is cleared implicitly when writing a new configuration */
    }

    adcChanCfg.interruptEnable = true; /* enable interrupt for last conversion in the group */
    adcChanCfg.channel         = currentGroupCfg->inputChannelArray[idx]; /* set the ADC input channel */
    ADC_DRV_ConfigChan(instance, idx, &adcChanCfg); /* configure the last conversion in the group */
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_StopGroupBlocking
 * Description   : Stops a group of conversions within a given timeout interval
 *
 * END**************************************************************************/
static status_t ADC_StopGroupBlocking(const uint32_t instance,
                                      const uint32_t timeout)
{
    const adc_pal_state_t * const palState = &(adcPalState[instance]);
    uint32_t startTime, deltaTime;
    status_t status;
    pdb_adc_pretrigger_config_t pdbPretrigCfg;
    uint8_t idx;

    /* Make sure OSIF timer is initialized. */
    OSIF_TimeDelay(0u);

    startTime = OSIF_GetMilliseconds();
    deltaTime = 0u;

    /* Reset PDB pre-trigger configurations to stop PDB from triggering other conversions in the group */
    pdbPretrigCfg.preTriggerEnable           = false;
    pdbPretrigCfg.preTriggerOutputEnable     = false;
    pdbPretrigCfg.preTriggerBackToBackEnable = false;
    for (idx = 0u; idx < palState->groupArray[palState->latestGroupIdx].numChannels; idx++)
    {
        pdbPretrigCfg.adcPreTriggerIdx = idx;
        PDB_DRV_ConfigAdcPreTrigger(instance, ADC_PAL_PDB_CHAN, &pdbPretrigCfg);
    }

    /* Completely stop PDB */
    PDB_DRV_Deinit((uint32_t)instance);

    /* Wait for current ADC active conversion to finish execution */
    static ADC_Type * const adcBase[ADC_INSTANCE_COUNT] = ADC_BASE_PTRS;
    const ADC_Type * const base                  = adcBase[instance];
    while ((ADC_GetConvActiveFlag(base) == true) && (deltaTime < timeout))
    {
        deltaTime = OSIF_GetMilliseconds() - startTime;
    }

    if (deltaTime >= timeout)
    {
        status = STATUS_TIMEOUT;
    }
    else
    {
        status = STATUS_SUCCESS;
    }

    return status;
}

#elif defined (ADC_PAL_TYPE_ADC_SAR_BCTU)


static status_t ADC_SAR_BCTU_Init(const uint32_t instIdx,
                                  const adc_config_t * const config)
{
    status_t funcStatus = STATUS_SUCCESS;
    status_t periphStatus;

    periphStatus = ADC_SAR_Config(instIdx, config);
    if (periphStatus != STATUS_SUCCESS)
    {
        funcStatus = STATUS_ERROR;
    }

#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    {
        ADC_CheckChanDuplicate(config);
    }
#endif /* defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT) */

    const adc_group_config_t * currentGroup;
    uint8_t idx;

    if (funcStatus == STATUS_SUCCESS)
    {
        /* Configure BCTU */
        periphStatus = BCTU_DRV_Reset(instIdx, /* default timeout value 2ms*/ 2u);
        if (periphStatus == STATUS_TIMEOUT)
        {
            funcStatus = STATUS_BUSY;
        }
    }

    if (funcStatus == STATUS_SUCCESS)
    {
        bctu_config_t bctuCfg;
        BCTU_DRV_GetDefaultConfig(&bctuCfg);
        bctuCfg.lowPowerModeEn   = true; /* Low power mode is enabled by default until a HW triggered group is enabled (only HW triggered groups use BCTU) */
        /* Used for HW triggered groups with a single conversion. By default enable new data interrupt for both ADCs */
        bctuCfg.newDataIntEnMask = (((uint8_t)1u << FEATURE_BCTU_NUM_ADC) - 1u);
        bctuCfg.listIntEn        = true;  /* Used for HW triggered groups with multiple conversions */

        BCTU_DRV_Config(instIdx, &bctuCfg);

        uint8_t bctuNumChansList = 0u; /* total number of channels programmed in BCTU list */

        /* Configure all HW triggered groups in BCTU.
         * SW triggered groups will be configured directly in ADC, each time ADC_StartGroupConversion() is called. */
        for (idx = 0u; idx < config->numGroups; idx++)
        {
            currentGroup = &(config->groupConfigArray[idx]);
            DEV_ASSERT(currentGroup->inputChannelArray != NULL);

            if (currentGroup->hwTriggerSupport == true)
            {
                ADC_ConfigHwTriggeredGroup(instIdx, currentGroup, bctuNumChansList);

                if (currentGroup->numChannels > 1u)
                {
                    bctuNumChansList += currentGroup->numChannels; /* Increment total number of channels programmed into BCTU LIST */
                }
            }
        }

        /* Move BCTU out of configuration mode */
        BCTU_DRV_WriteGlobalTriggerEn(instIdx, true);
    }

    return funcStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_SAR_BCTU_HwTrigIrqHandler
 * Description   : Interrupt handler functionality for HW triggered
 * conversion groups - to be called from BCTU interrupt handler.
 * Implements : ADC_Interrupt_Activity
 * END**************************************************************************/
void ADC_SAR_BCTU_HwTrigIrqHandler(const uint32_t instIdx,
                                   const uint8_t adcIdx)
{
    adc_pal_state_t * const palState = &(adcPalState_adc_sar_bctu[instIdx]);
    const adc_group_config_t * activeGroupCfg;
    adc_group_state_t * groupState;
    uint32_t currentGroupIdx;
    bctu_conv_result_t bctuResult;
    uint16_t * result;

    /* Get trigger idx for which the conv has finished */
    BCTU_DRV_GetConvResult(instIdx, adcIdx, &bctuResult);
    /* Get group idx assigned to the trigger idx */
    currentGroupIdx = ADC_GetGroupIdx(instIdx, bctuResult.triggerIdx);

    groupState = ADC_GetHwGroupStatePtr(palState, currentGroupIdx);

    activeGroupCfg = &(palState->groupArray[currentGroupIdx]);

    result = &(activeGroupCfg->resultBuffer[groupState->currentBufferOffset]);

    if (activeGroupCfg->numChannels == 1u)
    {
        *result = bctuResult.adcData;

        /* Increment offset in result buffer */
        groupState->currentBufferOffset = (groupState->currentBufferOffset + 1u) % groupState->bufferLength;

        BCTU_DRV_ClearStatusFlag(instIdx, adcIdx, BCTU_FLAG_NEW_DATA_AVAILABLE);
        DEV_ASSERT(BCTU_DRV_GetStatusFlag(instIdx, adcIdx, BCTU_FLAG_DATA_OVERRUN) == false);
    }
    else
    {
        /* Read all conversion results for the current conversion group */
        uint8_t idx;
        for (idx = 0u; idx < activeGroupCfg->numChannels; idx++)
        {
            *result = ADC_DRV_GetConvResult(adcIdx, ADC_GetChanIdx(activeGroupCfg->inputChannelArray[idx]));
            result++;
        }

        /* Increment offset in result buffer */
        groupState->currentBufferOffset = (groupState->currentBufferOffset + activeGroupCfg->numChannels) % groupState->bufferLength;

        BCTU_DRV_ClearStatusFlag(instIdx, adcIdx, BCTU_FLAG_LIST_LAST_CONV);
        /* Clear because conversion results are not read from BCTU result register */
        BCTU_DRV_ClearStatusFlag(instIdx, adcIdx, BCTU_FLAG_NEW_DATA_AVAILABLE);
        /* Clear because BCTU result register is overwritten */
        BCTU_DRV_ClearStatusFlag(instIdx, adcIdx, BCTU_FLAG_DATA_OVERRUN);
    }

    /* Call notification callback, if it is enabled */
    ADC_CallNotificationCb(palState, currentGroupIdx, groupState);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_ConfigHwTriggeredGroup
 * Description   : Configure BCTU for a ADC PAL HW triggered group
 *
 * END**************************************************************************/
static void ADC_ConfigHwTriggeredGroup(const uint32_t instIdx,
                                       const adc_group_config_t * const groupConfig,
                                       const uint8_t bctuStartListIdx)
{
    const adc_input_chan_t * const inputChans = groupConfig->inputChannelArray;
    bctu_trig_config_t bctuTrigConfig;
    status_t bctuStatus;
    const uint8_t numChans   = groupConfig->numChannels;
    uint8_t adcRelativeIndex; /* ADC index relative to the number of ADC instances per BCTU. E.g. if BCTU is connected to 2 ADC => ADC_3 relative index is 1 */

    DEV_ASSERT(numChans > 0u);
    DEV_ASSERT(groupConfig->delayArray == NULL); /* Delays are not supported */
    DEV_ASSERT(groupConfig->delayType == ADC_DELAY_TYPE_NO_DELAY); /* Delays are not supported */

    adcRelativeIndex = ADC_GetAdcAbsoluteIdx(inputChans[0]) % FEATURE_BCTU_NUM_ADC;
    bctuTrigConfig.adcTargetMask = (uint8_t)1u << adcRelativeIndex;
    bctuTrigConfig.hwTriggersEn  = false;
    bctuTrigConfig.loopEn        = groupConfig->continuousConvEn;
#if FEATURE_BCTU_HAS_CCP
    bctuTrigConfig.ccp      = 0u; /* CCP feature is not used by ADC PAL, so set to 0 */
    bctuTrigConfig.dest2PSI = false;
    bctuTrigConfig.tag      = 0u;
#endif

#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    {
        /* Check that a trigger source is only used by a single HW triggered group. */
        static const BCTU_Type * const bctuBase[BCTU_INSTANCE_COUNT] = BCTU_BASE_PTRS;
        const BCTU_Type * bctu;
        bctu = bctuBase[instIdx];
        if (bctu->TRGCFG_[groupConfig->triggerSource] != 0u) /* Check if the BCTU trigger register has been already written */
        {
            /* A trigger source cannot be assigned to multiple HW triggered groups. */
            DEV_ASSERT(false);
        }
    }
#endif /* defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT) */


    if (numChans == 1u) /* Configure groups containing a single conversion. */
    {

        bctuStatus = BCTU_DRV_ConfigTriggerSingle(instIdx, groupConfig->triggerSource, &bctuTrigConfig, ADC_GetChanIdx(inputChans[0]));
        DEV_ASSERT(bctuStatus == STATUS_SUCCESS);
        (void)bctuStatus;

#if ADC_PAL_BCTU_CONV_UPDATE_IRQ_UNROLLED
        INT_SYS_InstallHandler(bctuConvUpdateIrqNum[instIdx][adcRelativeIndex], bctuConvUpdateIrqHandlers[instIdx][adcRelativeIndex], NULL);
#else
        INT_SYS_InstallHandler(bctuConvUpdateIrqNum[instIdx], BCTU_ConvUpdate_IRQHandler, NULL);
#endif /* ADC_PAL_BCTU_CONV_UPDATE_IRQ_UNROLLED */
    }
    else /* Configure groups containing more than a single conversion. */
    {
        /* Prepare array with adc channel indexes expressed in normal format, accepted by the BCTU driver */
        uint8_t adcChanIdx[ADC_PAL_MAX_CONVS_IN_GROUP_HW_TRIG];
        uint8_t idx;
        for (idx = 0u; idx < numChans; idx++)
        {
            adcChanIdx[idx] = ADC_GetChanIdx(inputChans[idx]);

#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
            const uint8_t firstChanIdx = ADC_GetAdcAbsoluteIdx(inputChans[0u]);
            /* Channels in the group must target the same ADC instance */
            DEV_ASSERT(ADC_GetAdcAbsoluteIdx(inputChans[idx]) == firstChanIdx);
#endif /* defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT) */
        }

        /* For groups which include a minimum of 2 conversions, the total number of conversions within all these groups shall be less than
         * or equal with the number of BCTU LIST HW registers. Otherwise the first groups will be overwritten. */
        if ((bctuStartListIdx + numChans) >= (uint8_t)ADC_PAL_MAX_CONVS_IN_GROUP_HW_TRIG)
        {
            DEV_ASSERT(false);
        }
        else
        {
            bctuStatus = BCTU_DRV_ConfigTriggerList(instIdx, groupConfig->triggerSource, &bctuTrigConfig, bctuStartListIdx);
            DEV_ASSERT(bctuStatus == STATUS_SUCCESS);

            BCTU_DRV_SetConvListArray(instIdx, bctuStartListIdx, adcChanIdx, numChans);

            /* Install custom interrupt handler for BCTU because default one has different names depending on device */
            INT_SYS_InstallHandler(ADC_PAL_BCTU_LIST_LAST_IRQn, BCTU_ListLast_IRQHandler, NULL);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_GetGroupIdx
 * Description   : Returns the ADC conversion group index, for which the HW trigger
 * source has been configured.
 *
 * END**************************************************************************/
static uint32_t ADC_GetGroupIdx(const uint32_t instIdx,
                                const adc_trigger_source_t hwTrigSource)
{
    uint32_t groupIdx = 0u;
    bool found = false;
    const adc_pal_state_t * const palState = &(adcPalState_adc_sar_bctu[instIdx]);

    while ((found == false) && (groupIdx < palState->numGroups))
    {
        if ((palState->groupArray[groupIdx].triggerSource == hwTrigSource) && \
            (palState->groupArray[groupIdx].hwTriggerSupport == true))
        {
            found = true;
        }

        groupIdx++;
    }

    groupIdx--; /* compensate for additional groupIdx increment */
    DEV_ASSERT(found == true);

    return groupIdx;
}

#elif defined (ADC_PAL_TYPE_ADC_SAR_CTU)


static status_t ADC_SAR_CTU_Init(const uint32_t instIdx,
                                 const adc_config_t * const config)
{
    const extension_adc_sar_ctu_t * const extension = (extension_adc_sar_ctu_t *)(config->extension);
    const adc_group_config_t * currentGroup;
    ctu_config_t ctuCfg;
    status_t status = STATUS_SUCCESS;
    uint8_t idx;

    static const IRQn_Type ctuFirstFifoIrqNums[CTU_INSTANCE_COUNT] = {
        CTU_0_FIFO0_IRQn,
#if CTU_INSTANCE_COUNT > 1u
        CTU_1_FIFO0_IRQn
#endif
    };

    status = ADC_SAR_Config(instIdx, config);
    if (status != STATUS_SUCCESS)
    {
        return status;
    }

    /* Configure CTU */
    CTU_DRV_Reset(instIdx);

    uint8_t ctuNumChansList = 0u; /* total number of channels programmed in CTU list */
    uint8_t internalTrigIdx = 0u; /* each HW triggered group has associated a single internal trigger and a single result FIFO */
    uint8_t fifoIdx         = 0u;
    uint16_t delay          = 0u;
    ctu_trig_config_t trigConfig;
    ctu_res_fifo_config_t resFifoConfig;
    uint32_t inputTriggerSelectMask = 0u;
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    /* Length of result FIFOs are mentioned in the ReferenceManual */
    static const uint8_t resultFifoLengths[NUM_MAX_HW_TRIG_GROUPS] = { 16u, 16u, 4u, 4u };
    uint16_t previousDelays[NUM_MAX_HW_TRIG_GROUPS];
    uint8_t noDelayTrigIdx  = NUM_MAX_HW_TRIG_GROUPS;   /* index of the HW trigger with no delay, initialized with an out of range value */
    uint8_t i;

    /* this for is a workaround to avoid MISRA violation */
    for(i = 0u; i < NUM_MAX_HW_TRIG_GROUPS; i++)
    {
        previousDelays[i] = 0u;
    }
#endif /* defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT) */

    /* Configure all HW triggered groups in CTU.
     * SW triggered groups will be configured directly in ADC, each time ADC_StartGroupConversion() is called. */
    for (idx = 0u; idx < config->numGroups; idx++)
    {
        currentGroup = &(config->groupConfigArray[idx]);
        DEV_ASSERT(currentGroup->inputChannelArray != NULL);

        if (currentGroup->hwTriggerSupport == true)
        {
            DEV_ASSERT(internalTrigIdx < NUM_MAX_HW_TRIG_GROUPS); /* Maximum supported number of HW triggered groups is number of result FIFOs */

            /* The total number of conversions in all HW triggered groups must be <= CTU_CHANNEL_COUNT */
            DEV_ASSERT((ctuNumChansList + currentGroup->numChannels) <= CTU_CHANNEL_COUNT);

            switch (currentGroup->delayType)
            {
                case ADC_DELAY_TYPE_NO_DELAY:
                    delay = 0u;
                    break;
                case ADC_DELAY_TYPE_GROUP_DELAY:
                    DEV_ASSERT(currentGroup->delayArray != NULL);
                    delay = currentGroup->delayArray[0u]; /* single delay value per group */
                    break;
                default:
                    DEV_ASSERT(false); /* Other type not supported */
                    break;
            }

#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
            {
                /* only one HW group with no delay should be used */
                if (delay == 0u)
                {
                    DEV_ASSERT(noDelayTrigIdx == NUM_MAX_HW_TRIG_GROUPS);
                    noDelayTrigIdx = internalTrigIdx;
                }

                /* The number of conversions in the current HW triggered group must be <= than
                 * the length of the assigned result FIFO. */
                DEV_ASSERT(currentGroup->numChannels <= resultFifoLengths[internalTrigIdx]);

                /* Conversions must not overlap, otherwise CTU issues Trigger Overrun Error.
                 * Delay values for HW triggered groups must allow sufficient time for the rest of the conversion groups to finish execution.
                 * The actual duration of each group depends on frequency, number of ADC samples and number of conversions,
                 * however this check only conducts minimal validation that the values are not equal.
                 * Also, there cannot be multiple HW triggered groups with 'No delay' - because this gets translated in 0 delay. */
                for(i = 0u; i < internalTrigIdx; i++)
                {
                    if (i != noDelayTrigIdx)
                    {
                        DEV_ASSERT(previousDelays[i] != delay);
                    }
                }
                previousDelays[internalTrigIdx] = delay;
            }
#endif /* defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT) */

            trigConfig.cmdListStartAdr  = ctuNumChansList;
            trigConfig.compareVal       = delay;
            trigConfig.intEn            = false;
            trigConfig.outputTrigEnMask = CTU_OUTPUT_TRIG_ADC_CMD_EN;
            CTU_DRV_ConfigInternalTrigger(instIdx, internalTrigIdx, &trigConfig);

            /* Input trigger select register is common for all internal triggers (i.e. HW triggered groups) */
            inputTriggerSelectMask      |= currentGroup->triggerSource;

            fifoIdx = internalTrigIdx; /* Each HW triggered group has associated a single internal trigger and a single result FIFO.*/
            resFifoConfig.dmaEn         = false;
            resFifoConfig.fifoIntEnMask = CTU_FIFO_OVERFLOW;
            resFifoConfig.fifoThreshold = (currentGroup->numChannels - 1u); /* -1 because OVERFLOW interrupt gets triggered when the number of available results is above this value */
            CTU_DRV_ConfigResultFifo(instIdx, fifoIdx, &resFifoConfig);

            IRQn_Type fifoIrqNum;
            const uint16_t fifoIrqNumU = ((uint16_t)ctuFirstFifoIrqNums[instIdx] + fifoIdx); /* MISRA workaround */
            fifoIrqNum = (IRQn_Type)fifoIrqNumU;
            INT_SYS_EnableIRQ(fifoIrqNum);

            /* Config ADC command for each conversion in the group */
            uint8_t idx1 = 0u;
            uint8_t pos, adcPort, adcAbsoluteIdx;
            ctu_adc_cmd_t adcCmd;
            adcCmd.convMode   = CTU_CONV_MODE_SINGLE;
            adcCmd.fifoIdx    = fifoIdx;
            adcCmd.intEn      = false;
            adcCmd.selfTestEn = false;

            adcCmd.adcChanA = ADC_GetChanIdx(currentGroup->inputChannelArray[idx1]);
            adcAbsoluteIdx = ADC_GetAdcAbsoluteIdx(currentGroup->inputChannelArray[idx1]);
            adcPort = adcAbsoluteIdx % FEATURE_CTU_NUM_ADC;
            switch (adcPort)
            {
                case 0u:
                    adcCmd.adcPort = CTU_ADC_PORT_A;
                    break;
                case 1u:
                    adcCmd.adcPort = CTU_ADC_PORT_B;
                    break;
                default:
                    DEV_ASSERT(false);
                    break;
            }

            pos = trigConfig.cmdListStartAdr;
            CTU_DRV_ConfigAdcCmdList(instIdx, pos, &adcCmd, 1u); /* Write one command at a time */
            for (idx1 = 1u; idx1 < currentGroup->numChannels; idx1++)
            {
                CTU_Type * const ctuInstBase = ctuBase[instIdx];

                adcCmd.adcChanA = ADC_GetChanIdx(currentGroup->inputChannelArray[idx1]);

                /* For uniformity with SW triggered groups, even if CTU supports the feature,
                 * do not allow multiple ADC instances in the same conversion group. */
                DEV_ASSERT(ADC_GetAdcAbsoluteIdx(currentGroup->inputChannelArray[idx1]) == adcAbsoluteIdx);

                pos = (trigConfig.cmdListStartAdr + idx1) % CTU_CHANNEL_COUNT;
                CTU_DRV_ConfigAdcCmdList(instIdx, pos, &adcCmd, 1u);

                /* Workaround: clear LC bit starting from second iteration,
                 * because each CTU_DRV_ConfigAdcCmdList call marks the end of the list. */
                ctuInstBase->CHANNEL[pos].CLR_A &= (~CTU_CLR_A_LC_MASK);
            }

            internalTrigIdx++; /* Move to next trigger index */
            ctuNumChansList += currentGroup->numChannels; /* Increment total number of channels programmed into CTU LIST */
        }
    }

    CTU_DRV_GetDefaultConfig(&ctuCfg);
    ctuCfg.prescaler            = extension->ctuPrescaler;
    ctuCfg.tgsMode              = CTU_TGS_MODE_TRIGGERED; /* any HW trigger signal generates an MRS */
    ctuCfg.inputTrigSelectMask  = inputTriggerSelectMask;
    ctuCfg.tgsCounterCompareVal = CTU_TGSCCR_TGSCCV_MASK; /* set to max */
    ctuCfg.tgsCounterReloadVal  = 0u; /* set to minimum */

    CTU_DRV_Config(instIdx, &ctuCfg);

    /* Mark that writes to double-buffered configuration registers have ended. */
    CTU_DRV_EnableGeneralReload(instIdx);

    /* Sw trigger MRS to load all double-buffered registers and start a new CTU control cycle */
    CTU_DRV_SwMasterReloadSignal(instIdx);

#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    {
        ADC_CheckChanDuplicate(config);
    }
#endif /* defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT) */

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_SAR_CTU_HwTrigIrqHandler
 * Description   : Interrupt handler functionality for HW triggered
 * conversion groups - to be called from CTU FIFO interrupt handler.
 *
 * END**************************************************************************/
void ADC_SAR_CTU_HwTrigIrqHandler(const uint32_t instIdx,
                                  const uint8_t fifoIdx)
{
    adc_pal_state_t * const palState = &(adcPalState_adc_sar_ctu[instIdx]);
    const adc_group_config_t * activeGroupCfg;
    adc_group_state_t * groupState;
    uint32_t currentGroupIdx;
    ctu_conv_result_t ctuResult;
    uint16_t * result;

    DEV_ASSERT(CTU_DRV_GetFifoStatusFlags(instIdx, fifoIdx) != CTU_FIFO_OVERRUN);

    CTU_DRV_GetConvResult(instIdx, fifoIdx, CTU_RESULT_ALIGN_RIGHT_UNSIGNED, &ctuResult);

    groupState = &(palState->hwTrigGroupState[fifoIdx]);

    currentGroupIdx = groupState->groupIdx;

    activeGroupCfg = &(palState->groupArray[currentGroupIdx]);

    result = &(activeGroupCfg->resultBuffer[groupState->currentBufferOffset]);

    *result = ctuResult.convData;

    uint8_t idx;
    for (idx = 0u; idx < (activeGroupCfg->numChannels - 1u); idx++)
    {
        result++;
        *result = CTU_DRV_GetConvData(instIdx, fifoIdx, CTU_RESULT_ALIGN_RIGHT_UNSIGNED);
    }

    /* Increment offset in result buffer */
    groupState->currentBufferOffset = (groupState->currentBufferOffset + activeGroupCfg->numChannels) % groupState->bufferLength;

    /* Call notification callback, if it is enabled */
    ADC_CallNotificationCb(palState, currentGroupIdx, groupState);
}

#endif /* defined(ADC_PAL_TYPE_ADC_SAR_CTU) */

#if defined (ADC_PAL_SAR)

static status_t ADC_SAR_Init(const uint32_t instIdx,
                                      const adc_config_t * const config)
{
    status_t funcStatus = STATUS_SUCCESS;
    status_t periphStatus;

    periphStatus = ADC_SAR_Config(instIdx, config);
    if (periphStatus != STATUS_SUCCESS)
    {
        funcStatus = STATUS_ERROR;
    }

#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    {
        ADC_CheckChanDuplicate(config);
    }
#endif /* defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT) */

    return funcStatus;
}

#endif /* defined (ADC_PAL_SAR) */


#if defined (ADC_PAL_TYPE_SDADC)

/*FUNCTION**********************************************************************
 *
 * Function Name : SDADC_Init
 * Description   : Initialization of SDADC customized for ADC PAL integration
 * END**************************************************************************/
static status_t SDADC_Init(const uint32_t instIdx,
                           const adc_config_t * const config)
{
    status_t status = STATUS_SUCCESS;
    const extension_sdadc_t * const extension = (extension_sdadc_t *)(config->extension);
    sdadc_conv_config_t sdadcConfig;

    /* Initialize SDADC */
    SDADC_DRV_Reset(instIdx);
    /* Perform offset calibrations */
    SDADC_DRV_OffsetCalibration(instIdx, extension->inputGain);

    /* Perform gain calibrations */
    SDADC_DRV_GainCalibration(instIdx, extension->gainCalibNumConv);

    SDADC_DRV_GetConverterDefaultConfig(&sdadcConfig);

    /* ADC PAL doesn't use SDADC FIFO */
    sdadcConfig.enableFifo    = false;
    sdadcConfig.fifoThreshold = 0u;

    sdadcConfig.decimaRate     = extension->decimaRate;
    sdadcConfig.inputGain      = extension->inputGain;
    sdadcConfig.highPassFilter = extension->highPassFilter;
    sdadcConfig.stopInDebug    = extension->stopInDebug;

    SDADC_DRV_ConfigConverter(instIdx, &sdadcConfig);

#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    uint16_t groupIdx, chanIdx;

    /* Check that input channels are among values available in SDADC driver enum sdadc_inputchannel_sel_t */
    for(groupIdx = 0; groupIdx < config->numGroups; groupIdx++)
    {
    	for(chanIdx = 0; chanIdx < config->groupConfigArray[groupIdx].numChannels; chanIdx++)
    	{
    		DEV_ASSERT(SDADC_CheckInputChannelMapping(config->groupConfigArray[groupIdx].inputChannelArray[chanIdx]) == true);
    	}
    }
#endif /* defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT) */


    return status;
}

#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
/*FUNCTION**********************************************************************
 *
 * Function Name : SDADC_CheckInputChannelMapping
 * Description   : Check that input channel is mapped correctly on the sdadc_inputchannel_sel_t enum exposed by SDADC driver
 * END**************************************************************************/
static bool SDADC_CheckInputChannelMapping(const uint16_t inputChan)
{
	bool status;

	switch(inputChan)
	{
	case SDADC_CHAN_AN0_VREFN:
	case SDADC_CHAN_AN1_VREFN:
	case SDADC_CHAN_AN2_VREFN:
	case SDADC_CHAN_AN3_VREFN:
	case SDADC_CHAN_AN4_VREFN:
	case SDADC_CHAN_AN5_VREFN:
	case SDADC_CHAN_AN6_VREFN:
	case SDADC_CHAN_AN7_VREFN:
	case SDADC_CHAN_AN0_VREFP2:
	case SDADC_CHAN_AN1_VREFP2:
	case SDADC_CHAN_AN2_VREFP2:
	case SDADC_CHAN_AN3_VREFP2:
	case SDADC_CHAN_AN4_VREFP2:
	case SDADC_CHAN_AN5_VREFP2:
	case SDADC_CHAN_AN6_VREFP2:
	case SDADC_CHAN_AN7_VREFP2:
	case SDADC_CHAN_AN0_AN1:
	case SDADC_CHAN_AN2_AN3:
	case SDADC_CHAN_AN4_AN5:
	case SDADC_CHAN_AN6_AN7:
	case SDADC_CHAN_VREFN_VREFN:
	case SDADC_CHAN_VREFP2_VREFP2:
	case SDADC_CHAN_VREFP_VREFN:
	case SDADC_CHAN_VREFN_VREFP:
		status = true;
		break;
	default:
		status = false;
		break;
	}

	return status;
}
#endif /* defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT) */


/*FUNCTION**********************************************************************
 *
 * Function Name : SDADC_StartConversion
 * Description   : Configure and start an SDADC conversion with corresponding delay
 * END**************************************************************************/
static void SDADC_StartConversion(const uint32_t adcIdx,
                                  const uint8_t groupIdx,
                                  const uint8_t convIdx)
{
    adc_pal_state_t * const palState = &(adcPalState_sdadc[adcIdx]);
    const adc_group_config_t * currentGroupCfg  = &(palState->groupArray[groupIdx]);

    SDADC_DRV_Powerdown(adcIdx);
    /* Flush data fifo to make sure the fifo does not contains data of previous conversions */
    SDADC_DRV_FlushDataFifo(adcIdx);
    SDADC_DRV_ClearStatusFlags(adcIdx, SDADC_SFR_DFFF_MASK | SDADC_SFR_DFORF_MASK | SDADC_SFR_WTHL_MASK | SDADC_SFR_WTHH_MASK);
    SDADC_DRV_Powerup(adcIdx);

    SDADC_DRV_SelectInputChannel(adcIdx, (sdadc_inputchannel_sel_t) currentGroupCfg->inputChannelArray[convIdx]);

    DEV_ASSERT(currentGroupCfg->delayArray[convIdx] <= SDADC_OSDR_OSD_MASK); /* maximum value accepted for output delay */
    SDADC_DRV_SetOutputSettlingDelay(adcIdx, (uint8_t)(currentGroupCfg->delayArray[convIdx]));

    palState->swTrigGroupState.currentConvIdx = convIdx;

    SDADC_DRV_EnableInterruptEvents(adcIdx, SDADC_EVENT_FIFO_FULL);

    SDADC_DRV_RefreshConversion(adcIdx);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SDADC_SwTrigIrqHandler
 * Description   : Interrupt handler functionality for SW triggered
 * conversion groups - to be called from SDADC Interrupt.
 * Implements : ADC_Interrupt_Activity
 * END**************************************************************************/
void SDADC_SwTrigIrqHandler(const uint32_t instIdx)
{
    adc_pal_state_t * const palState = &(adcPalState_sdadc[instIdx]);
    adc_group_state_t * const groupState            = &(palState->swTrigGroupState);
    const uint32_t currentGroupIdx                  = palState->latestGroupIdx;
    const adc_group_config_t * const activeGroupCfg = &(palState->groupArray[currentGroupIdx]);

    uint16_t * result = &(activeGroupCfg->resultBuffer[groupState->currentBufferOffset]);

    DEV_ASSERT(activeGroupCfg->hwTriggerSupport == false); /* This is only used for SW triggered conversion groups */

    /* Read the current conversion result for the current conversion group */
    uint8_t len = SDADC_DRV_GetConvDataFifo(instIdx, 1u, (int16_t *) result);
    DEV_ASSERT(len == 1u);
    (void) len;

    SDADC_DRV_ClearStatusFlags(instIdx, SDADC_FLAG_DATA_FIFO_FULL);

    /* Increment offset in result buffer */
    groupState->currentBufferOffset = (uint16_t)((groupState->currentBufferOffset + 1u) % groupState->bufferLength);

    palState->swTrigGroupState.currentConvIdx++;

    if(palState->swTrigGroupState.currentConvIdx == activeGroupCfg->numChannels)
    {
        /* Last conversion in the group */

        /* Call notification callback, if it is enabled */
        ADC_CallNotificationCb(palState, currentGroupIdx, groupState);

        if (activeGroupCfg->continuousConvEn == true) /* Continuous mode */
        {
            palState->swTrigGroupState.currentConvIdx = 0u;

            /* Restart first conversion */
            SDADC_StartConversion(instIdx, currentGroupIdx, palState->swTrigGroupState.currentConvIdx);
        }
        else
        {
            /* Prevent other conversion from starting */
            SDADC_DRV_Powerdown(instIdx);

            /* Clear any other conversions which might have completed in the meantime */
            SDADC_DRV_ClearStatusFlags(instIdx, SDADC_FLAG_DATA_FIFO_FULL);

            groupState->active = false;
        }
    }
    else
    {
        /* Start next conversion */
        SDADC_StartConversion(instIdx, currentGroupIdx, palState->swTrigGroupState.currentConvIdx);
    }
}

#endif /* defined (ADC_PAL_TYPE_SDADC) */


#if defined (ADC_PAL_TYPE_EQADC)

/*FUNCTION**********************************************************************
 *
 * Function Name : EQADC_Init
 * Description   : Initialization of EQADC customized for ADC PAL integration
 * END**************************************************************************/
static status_t EQADC_Init(const uint32_t instIdx,
                           const adc_config_t * const config)
{
    status_t status = STATUS_SUCCESS;
    const extension_eqadc_t * const extension = (extension_eqadc_t *)(config->extension);
    adc_pal_state_t * const palState          = &(adcPalState_eqadc[instIdx]);
    eqadc_config_t eqadcConfig;
    eqadc_adc_config_t adcConfig[EQADC_NUM_ADC];
    eqadc_cfifo_config_t cfifoConfig[EQADC_CFPR_COUNT];
    eqadc_calibration_params_t calibParam;

    /* Initialize EQADC */
    EQADC_DRV_Reset(instIdx);

    DEV_ASSERT(config->sampleTicks == 0u); /* ADC_PAL_TYPE_EQADC doesn't use sampleTicks so should be set to 0. It uses similar field in extension */

    eqadcConfig.numAdcConfigs           = EQADC_NUM_ADC;
    eqadcConfig.adcConfigArray          = adcConfig;
    eqadcConfig.numCfifoConfigs         = EQADC_CFPR_COUNT;
    eqadcConfig.cfifoConfigArray        = cfifoConfig;
    eqadcConfig.numResultDmaConfigs     = 0u;
    eqadcConfig.numAlternateConfigs     = 0u;
    eqadcConfig.numExtAlternateConfigs  = 0u;
    eqadcConfig.numChanPullConfigs      = 0u;
    eqadcConfig.numStacBusClientConfigs = 0u;

    EQADC_DRV_GetDefaultConfig(&eqadcConfig);

    eqadcConfig.digitalFilterLen = extension->digitalFilterLen;
    eqadcConfig.adcConfigArray[0u].clkPrescale      = extension->clkPrescale;
    eqadcConfig.adcConfigArray[0u].clkOddPrescaleEn = extension->clkOddPrescaleEn;
    eqadcConfig.adcConfigArray[0u].clkDutySel       = extension->clkDutySel;
    eqadcConfig.adcConfigArray[1u].clkPrescale      = extension->clkPrescale;
    eqadcConfig.adcConfigArray[1u].clkOddPrescaleEn = extension->clkOddPrescaleEn;
    eqadcConfig.adcConfigArray[1u].clkDutySel       = extension->clkDutySel;

    /* Save the extension members which need to be sent at each command */
    palState->samplingTime = extension->samplingTime;
    palState->signEn       = extension->signEn;

    eqadcConfig.cfifoConfigArray[EQADC_CFIFO_IDX_SW_TRIG].opMode = EQADC_CFIFO_MODE_SW_TRIG_SINGLE;
    eqadcConfig.cfifoConfigArray[EQADC_CFIFO_IDX_HW_TRIG].opMode = EQADC_CFIFO_MODE_DISABLED; /* HW triggered mode will be selected when enabling the hardware triggers */

    EQADC_DRV_Init(instIdx, &eqadcConfig);

    /* Perform calibrations */
    calibParam.samplingTime = extension->samplingTime;
    calibParam.cfifoIdx     = EQADC_CFIFO_IDX_CALIB;
    calibParam.rfifoIdx     = EQADC_RFIFO_IDX_CALIB;
    calibParam.calibTarget  = EQADC_CALIBRATION_TARGET_MAIN; /* calibrate only main calibration gain and offset, because alternate configurations are nor used in ADC PAL  */
    calibParam.timeout      = 2u; /* default timeout value 2ms */
    status = EQADC_DRV_DoCalibration(instIdx, 0u, &calibParam); /* calibrate ADC sub-module 0 */
    if(status == STATUS_SUCCESS)
    {
        status = EQADC_DRV_DoCalibration(instIdx, 1u, &calibParam); /* calibrate ADC sub-module 1 */
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EQADC_GetGroupState
 * Description   : Return pointer to groupState depending on fifoIdx
 * END**************************************************************************/
static inline adc_group_state_t * EQADC_GetGroupState(adc_pal_state_t * const palState,
                                                      const uint32_t fifoIdx)
{
    adc_group_state_t * groupState;
    const uint32_t hwTrigFifoIdx = fifoIdx - 1u; /* Assumption: CFIFO0 and RFIFO0 used for SW triggered groups. The rest are assumed to be used for HW triggered groups */

    if(fifoIdx == EQADC_CFIFO_IDX_SW_TRIG)
    {
        groupState = &(palState->swTrigGroupState); /* CFIFO0 and RFIFO0 are assumed to be used for SW triggered groups */
    }
    else
    {
        DEV_ASSERT(hwTrigFifoIdx < ADC_PAL_MAX_NUM_HW_GROUPS_EN);
        groupState = &(palState->hwTrigGroupState[hwTrigFifoIdx]);
    }

    return groupState;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EQADC_RfifoDrainRequest
 * Description   : Function called from RFIFO DRAIN interrupt handlers,
 * called each time there is a new result available in the RFIFO
 * END**************************************************************************/
void EQADC_RfifoDrainRequest(const uint32_t instIdx,
                             const uint32_t fifoIdx)
{
    adc_pal_state_t * const palState                = &(adcPalState_eqadc[instIdx]);
    adc_group_state_t * const groupState            = EQADC_GetGroupState(palState, fifoIdx);
    const uint32_t currentGroupIdx                  = palState->latestGroupIdx;
    const adc_group_config_t * const activeGroupCfg = &(palState->groupArray[currentGroupIdx]);

    uint16_t * result = &(activeGroupCfg->resultBuffer[groupState->currentBufferOffset]);

    *result = EQADC_DRV_PopRfifoData(instIdx, fifoIdx);  /* Pop result from RFIFO0 */

    /* Increment offset in result buffer */
    groupState->currentBufferOffset = (uint16_t)((groupState->currentBufferOffset + 1u) % groupState->bufferLength);

    if((groupState->currentBufferOffset % activeGroupCfg->numChannels) == 0u) /* Result received from last conversion in the group */
    {
        ADC_CallNotificationCb(palState, currentGroupIdx, groupState);

        if((activeGroupCfg->continuousConvEn == true) && (activeGroupCfg->hwTriggerSupport == false)) /* Only sw triggered conversions support continuous mode */
        {
            EQADC_DRV_EnableIntReq(instIdx, fifoIdx, EQADC_INT_EN_CFIFO_FILL);

            EQADC_DRV_SetSingleScanEnBit(instIdx, fifoIdx, false);
        }
        else
        {
        	if(activeGroupCfg->hwTriggerSupport == false) /* If software triggered conversion group */
        	{
        		 /* Only reset active flag if group is software triggered.
        		  * Hardware triggered groups remain active until DisableHardwareTrigger() is called. */
        		groupState->active = false;
        	}
        	else /* If hardware triggered conversion group */
        	{
                EQADC_DRV_EnableIntReq(instIdx, fifoIdx, EQADC_INT_EN_CFIFO_FILL);

                EQADC_DRV_SetSingleScanEnBit(instIdx, fifoIdx, false);
        	}
        }
    }

    /* Overflow events should not occur. In case it does occur the new result is discarded - results do not get overwritten. */
    DEV_ASSERT(EQADC_DRV_GetFifoStatus(instIdx, fifoIdx, EQADC_FIFO_STATUS_SEL_RFIFO_OVERFLOW) == 0u);

    EQADC_DRV_ClearFifoStatus(instIdx, fifoIdx, EQADC_FIFO_STATUS_FLAG_RFIFO_DRAIN);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : EQADC_CfifoFillRequest
 * Description   : Function called from CFIFO FILL interrupt handlers,
 * called each time a new command needs to be pushed in the CFIFO
 * END**************************************************************************/
void EQADC_CfifoFillRequest(const uint32_t instIdx,
                            const uint32_t fifoIdx)
{
    adc_pal_state_t * const palState                = &(adcPalState_eqadc[instIdx]);
    adc_group_state_t * const groupState            = EQADC_GetGroupState(palState, fifoIdx);
    const uint32_t currentGroupIdx                  = palState->latestGroupIdx;
    const adc_group_config_t * const activeGroupCfg = &(palState->groupArray[currentGroupIdx]);
    const uint32_t msgTag = (uint32_t)EQADC_MESSAGE_TAG_RFIFO0 + fifoIdx;;
    eqadc_conv_cmd_t convCmd = {
        .eoq = false,
        .pause = false,
        .repeatStart = false,
        .cbufferNum = 0u,
        .calibEn = true,
        .msgTag = EQADC_MESSAGE_TAG_RFIFO0,
        .samplingTime = EQADC_SAMPLING_TIME_64_CYCLES,
        .channelNum = 0u,
        .signEn = true,
        .altConfigSel = EQADC_ALT_CONFIG_SEL_DISABLED,
        .flushCompanion = false
    };

    /* Populate current conversion command with runtime values */
    convCmd.cbufferNum   = ADC_GetAdcAbsoluteIdx(activeGroupCfg->inputChannelArray[groupState->currentConvIdx]);
    convCmd.channelNum   = ADC_GetChanIdx(activeGroupCfg->inputChannelArray[groupState->currentConvIdx]);
    convCmd.msgTag       = (eqadc_message_tag_t) msgTag;
    convCmd.samplingTime = palState->samplingTime;
    convCmd.signEn       = palState->signEn;

    if(groupState->currentConvIdx == (activeGroupCfg->numChannels - 1u))
    {
        /* Mark the conversion command appropriately for the last conversion in the group */
        convCmd.eoq = true;

        /* Disable CFIFO FILL interrupt when all the required commands have been transfered */
        EQADC_DRV_DisableIntReq(instIdx, fifoIdx, EQADC_INT_EN_CFIFO_FILL);
    }

    groupState->currentConvIdx = (groupState->currentConvIdx + 1u) % activeGroupCfg->numChannels;

    /* Push conversion commands in the selected CFIFO */
    EQADC_DRV_PushCfifoConvCmd(instIdx, fifoIdx, &convCmd);

    EQADC_DRV_ClearFifoStatus(instIdx, fifoIdx, EQADC_FIFO_STATUS_FLAG_CFIFO_FILL);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : EQADC_StopConvProcedure
 * Description   : Function with procedures for stopping EQADC conversion: disable cfifo,
 * wait for cfifo to move to status idle or timeout to occur, invalidate cfifo, flush rfifo, update state if successful
 * called each time a new command needs to be pushed in the CFIFO
 * END**************************************************************************/
static status_t EQADC_StopConvProcedure(const uint16_t instIdx,
                                        const uint32_t fifoIdx,
                                        const uint32_t timeout)
{
        uint32_t startTime, deltaTime;
        eqadc_cfifo_status_t cfifoState;
        status_t status, eqadcStatus;

        INT_SYS_DisableIRQ(eqadcCfifoFillIRQn[instIdx][fifoIdx]);
        INT_SYS_DisableIRQ(eqadcRfifoFillIRQn[instIdx][fifoIdx]);

        EQADC_DRV_DisableIntReq(instIdx, fifoIdx, EQADC_INT_EN_CFIFO_FILL);
        EQADC_DRV_DisableIntReq(instIdx, fifoIdx, EQADC_INT_EN_RFIFO_DRAIN); /* TODO: can be optimized to be called only in Init */

        /* Disable CFIFO */
        EQADC_SetCfifoOpMode(instIdx, fifoIdx, (uint32_t)EQADC_CFIFO_MODE_DISABLED);

        OSIF_TimeDelay(0u); /* Make sure OSIF timer is initialized. */

        startTime = OSIF_GetMilliseconds();
        deltaTime = 0u;

        cfifoState = EQADC_DRV_GetCfifoStatusCurrent(instIdx, fifoIdx);

        /* Wait for CFIFO to reach IDLE STATE or timeout to occur */
        while ((cfifoState != CFIFO_STATUS_IDLE) && (deltaTime < timeout))
        {
            deltaTime = OSIF_GetMilliseconds() - startTime;

            cfifoState = EQADC_DRV_GetCfifoStatusCurrent(instIdx, fifoIdx);
        }

        if (deltaTime >= timeout)
        {
            status = STATUS_TIMEOUT;
        }
        else
        {
            /* Invalidate CFIFO - only if CFIFO status is IDLE */
            eqadcStatus = EQADC_DRV_InvalidateCfifo(instIdx, fifoIdx);
            (void) eqadcStatus;
            /* Optionally: reset TC_CFx */

            /* Flush RFIFO */
            status = EQADC_DRV_FlushRfifo(instIdx, fifoIdx, (timeout - deltaTime));
        }

        return status;
}

#endif /* defined (ADC_PAL_TYPE_EQADC) */


#if (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_SAR) || defined (ADC_PAL_TYPE_EQADC))

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_GetChanIdx
 * Description   : Returns ADC channel index, demapped from an ADC PAL input channel
 *
 * END**************************************************************************/
static inline uint8_t ADC_GetChanIdx(const adc_input_chan_t adcPalInputChan)
{
#if (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU))
    return (uint8_t)(adcPalInputChan & ADC_CHAN_CHAN_IDX_MASK);
#else
    return (uint8_t)(adcPalInputChan);
#endif
}


#if (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC))
/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_GetAdcIdx
 * Description   : Returns ADC absolute index, demapped from an ADC PAL input channel.
 * E.g. if each BCTU/CTU instance is linked to 2 ADCs => for ADC_3, idx = 3.
 *
 * END**************************************************************************/
static inline uint8_t ADC_GetAdcAbsoluteIdx(const adc_input_chan_t adcPalInputChan)
{
    return (uint8_t)((adcPalInputChan & ADC_CHAN_ADC_IDX_MASK) >> ADC_CHAN_ADC_IDX_OFFSET);
}
#endif /* (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC)) */

#endif /* (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_SAR) || defined (ADC_PAL_TYPE_EQADC)) */



#if (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_SAR))

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_GetUsedAdcMask
 * Description   : Returns a bitmask with each bit (starting from LSB) corresponding to
 * an ADC instance. If the corresponding bit is set to "1" the ADC instance is targeted
 * by at least one conversion within a group.
 *
 * END**************************************************************************/
static uint16_t ADC_GetUsedAdcInstances(const uint32_t instance)
{
    uint16_t usedAdcMask = 0u;
    uint16_t adcIdx;

#if (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU))
#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
    const adc_pal_state_t * const palState = &(adcPalState_adc_sar_bctu[instance]);
#elif defined (ADC_PAL_TYPE_ADC_SAR_CTU)
    const adc_pal_state_t * const palState = &(adcPalState_adc_sar_ctu[instance]);
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_BCTU) */
    uint8_t idx;
    for (idx = 0u; idx < palState->numGroups; idx++)
    {
        const adc_group_config_t * currentGroup;
        currentGroup = &(palState->groupArray[idx]);

        /* Assumes that a group contains channels targeting the same ADC index.
         * This assumption is checked with DEV_ASSERT in other part of the code. */
        adcIdx = ADC_GetAdcAbsoluteIdx(currentGroup->inputChannelArray[0u]);

        usedAdcMask |= ((uint16_t)1u << adcIdx);
    }
#elif defined (ADC_PAL_SAR)
    adcIdx = (uint16_t) instance; /* If there is no HW triggering unit, ADC PAL is mapped to a single ADC instance */
    usedAdcMask |= ((uint16_t)1u << adcIdx);
#endif /* (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU)) */

    return usedAdcMask;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_SAR_SwTrigIrqHandler
 * Description   : Interrupt handler functionality for SW triggered
 * conversion groups - to be called from ADC End of Chain (ECH) interrupt handler.
 * Implements : ADC_Interrupt_Activity
 * END**************************************************************************/
void ADC_SAR_SwTrigIrqHandler(const uint32_t instIdx,
                              const uint8_t adcIdx)
{
#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
    adc_pal_state_t * const palState = &(adcPalState_adc_sar_bctu[instIdx]);
#elif defined (ADC_PAL_TYPE_ADC_SAR_CTU)
    adc_pal_state_t * const palState = &(adcPalState_adc_sar_ctu[instIdx]);
#elif defined (ADC_PAL_SAR)
    adc_pal_state_t * const palState = &(adcPalState_adc_sar[instIdx]);
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_BCTU) */
    adc_group_state_t * const groupState            = &(palState->swTrigGroupState);
    const uint32_t currentGroupIdx                  = palState->latestGroupIdx;
    const adc_group_config_t * const activeGroupCfg = &(palState->groupArray[currentGroupIdx]);

    uint16_t * result = &(activeGroupCfg->resultBuffer[groupState->currentBufferOffset]);

    DEV_ASSERT(activeGroupCfg->hwTriggerSupport == false); /* This is only used for SW triggered conversion groups */

    /* Read all conversion results for the current conversion group */
    uint8_t idx;
    for (idx = 0u; idx < activeGroupCfg->numChannels; idx++)
    {
        *result = ADC_DRV_GetConvResult(adcIdx, ADC_GetChanIdx(activeGroupCfg->inputChannelArray[idx]));
        result++;
    }

    /* Increment offset in result buffer */
    groupState->currentBufferOffset = (uint16_t)((groupState->currentBufferOffset + activeGroupCfg->numChannels) % groupState->bufferLength);

    ADC_DRV_ClearStatusFlags(adcIdx, ADC_FLAG_NORMAL_ENDCHAIN);

    /* Call notification callback, if it is enabled */
    ADC_CallNotificationCb(palState, currentGroupIdx, groupState);

    if (activeGroupCfg->continuousConvEn == true) /* Continuous mode currently supported only for SW triggered groups */
    {
        ADC_DRV_StartConversion(adcIdx, ADC_CONV_CHAIN_NORMAL);
    }
    else
    {
        /* Reset the SW triggered conversion channels for the current adcIdx.
         * Assumption: a single SW triggered group is active at a time. */
        for (idx = 0u; idx < activeGroupCfg->numChannels; idx++)
        {
            ADC_DRV_DisableChannel(adcIdx, ADC_CONV_CHAIN_NORMAL, ADC_GetChanIdx(activeGroupCfg->inputChannelArray[idx]));
        }

#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)
        adcBase[adcIdx]->MCR |= ADC_MCR_CTUEN(1u); /* set ADC to CTU CONTROL MODE, to allow HW triggering */
#endif /* defined(ADC_PAL_TYPE_ADC_SAR_CTU) */

        groupState->active = false;
    }
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_SAR_Config
* Description   : Configure ADC instances used by the ADC_PAL for ADC_PAL_TYPE_ADC_SAR_BCTU and ADC_PAL_TYPE_ADC_SAR_CTU
* Return STATUS_ERROR if not all the configured ADCs have been calibrated successfully.
* Otherwise return STATUS_SUCCESS.
*
* END**************************************************************************/
static status_t ADC_SAR_Config(const uint32_t instIdx,
                                     const adc_config_t * const config)
{
#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
    const extension_adc_sar_bctu_t * const extension = (extension_adc_sar_bctu_t *)(config->extension);
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    const uint8_t numAdcPerTrigUnit = FEATURE_BCTU_NUM_ADC;
#endif
#elif defined (ADC_PAL_TYPE_ADC_SAR_CTU)
    const extension_adc_sar_ctu_t * const extension = (extension_adc_sar_ctu_t *)(config->extension);
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    const uint8_t numAdcPerTrigUnit = FEATURE_CTU_NUM_ADC;
#endif
#elif defined (ADC_PAL_SAR)
    const extension_adc_sar_t * const extension = (extension_adc_sar_t *)(config->extension);
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    const uint8_t numAdcPerTrigUnit = 1u;
#endif
#endif /* defined(ADC_PAL_TYPE_ADC_SAR_CTU) */
    adc_conv_config_t adcCfg;
    uint16_t idx;
    status_t status = STATUS_ERROR;

    ADC_DRV_GetDefaultConfigConverter(&adcCfg);
    adcCfg.sampleTime0  = config->sampleTicks;
    adcCfg.sampleTime1  = config->sampleTicks;
    adcCfg.sampleTime2  = config->sampleTicks;
    adcCfg.clkSelect    = extension->clkSelect;
    adcCfg.refSelect    = extension->refSelect;
    adcCfg.autoClockOff = extension->autoClockOff;
    adcCfg.convMode     = ADC_CONV_MODE_ONESHOT;
#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
    /* ADC may be triggered from BCTU and also from SW. */
    adcCfg.ctuMode      = ADC_CTU_MODE_TRIGGER;
#elif defined (ADC_PAL_TYPE_ADC_SAR_CTU)
    /* ADC may be triggered only from CTU. To allow SW triggered groups, will toggle to CTU control disabled/enabled when starting/stopping a SW triggered conversion. */
    adcCfg.ctuMode      = ADC_CTU_MODE_CONTROL;
#endif
    adcCfg.overwriteEnable = true;

    /* Configure only the ADC instances which are used,
     * i.e. which have at least one channel assigned to at least one group. */
    uint16_t activeAdcMask = ADC_GetUsedAdcInstances(instIdx);
    for (idx = 0u; idx < ADC_INSTANCE_COUNT; idx++)
    {
        if ((activeAdcMask & ((uint16_t)1u << idx)) != 0u)
        {
            /* Check that the selected ADC is connected to the selected triggering unit.
             * Triggering unit index is mapped 1:1 with adc_pal instance index */
            DEV_ASSERT(idx >= (instIdx * numAdcPerTrigUnit));
            DEV_ASSERT(idx < ((instIdx + 1u) * numAdcPerTrigUnit));

            ADC_DRV_Reset(idx);
            status = ADC_DRV_DoCalibration(idx);
            if (status == STATUS_SUCCESS)
            {
                ADC_DRV_ConfigConverter(idx, &adcCfg);

                /* Used for SW triggered groups */
                ADC_DRV_EnableInterrupts(idx, ADC_FLAG_NORMAL_ENDCHAIN);
            }
            else
            {
                break; /* Do not calibrate the rest of the ADCs, if at least one calibration failed */
            }
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_SAR_StopConversion
* Description   : Stop ADC SAR SW triggered conversion
* Common for all ADC PAL types that use ADC SAR
*
* END**************************************************************************/
static status_t ADC_SAR_StopConversion(const uint32_t instIdx,
                                       const adc_group_config_t * currentGroupCfg,
                                       const uint32_t timeout)
{
    status_t status = STATUS_SUCCESS;
    uint8_t idx, adcIdx;
    uint32_t currentChan;
    uint32_t startTime, deltaTime;

    OSIF_TimeDelay(0u); /* Make sure OSIF timer is initialized. */

    startTime = OSIF_GetMilliseconds();
    deltaTime = 0u;

#if (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU))
    /* Assumes that all channels in the group target the same ADC instance */
    adcIdx = ADC_GetAdcAbsoluteIdx(currentGroupCfg->inputChannelArray[0u]);
    (void) instIdx;
#elif defined (ADC_PAL_SAR)
    adcIdx = instIdx;
#endif /* (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU)) */

    INT_SYS_DisableIRQ(adcEocIrqNums[adcIdx]);

    /* Disable channels from current conversion group */
    for (idx = 0u; idx < currentGroupCfg->numChannels; idx++)
    {
        currentChan = ADC_GetChanIdx(currentGroupCfg->inputChannelArray[idx]);

        ADC_DRV_DisableChannel(adcIdx, ADC_CONV_CHAIN_NORMAL, currentChan);
    }

    /* Check if conversion is still running, until execution finished or timeout occurred */
    while (((ADC_DRV_GetStatusFlags(adcIdx) & ADC_FLAG_NORMAL_STARTED) != 0u) && (deltaTime < timeout))
    {
        deltaTime = OSIF_GetMilliseconds() - startTime;
    }

    /* Clear status & pending flags corresponding to a conversion group
     * which may have finished after interrupts have been disabled
     * The flags are cleared with dummy read operations */
    ADC_DRV_ClearStatusFlags(adcIdx, ADC_FLAG_NORMAL_ENDCHAIN);
    for (idx = 0u; idx < currentGroupCfg->numChannels; idx++)
    {
        uint16_t dummyRes;
        currentChan = ADC_GetChanIdx(currentGroupCfg->inputChannelArray[idx]);

        dummyRes = ADC_DRV_GetConvResult(adcIdx, currentChan);
        (void) dummyRes;
    }

    if (deltaTime >= timeout)
    {
        status = STATUS_TIMEOUT;
    }
    else
    {
#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)
        adcBase[adcIdx]->MCR |= ADC_MCR_CTUEN(1u); /* set ADC to CTU CONTROL MODE, to allow HW triggering */
#endif /* defined(ADC_PAL_TYPE_ADC_SAR_CTU) */

        status = STATUS_SUCCESS;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_ResetAdcInstances
 * Description   : Reset active ADC instances used by the ADC_PAL for ADC_PAL_TYPE_ADC_SAR_BCTU and ADC_PAL_TYPE_ADC_SAR_CTU
 *
 * END**************************************************************************/
static void ADC_SAR_Reset(const uint32_t instIdx)
{
#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
    const uint8_t numAdcPerTrigUnit = FEATURE_BCTU_NUM_ADC;
#elif defined (ADC_PAL_TYPE_ADC_SAR_CTU)
    const uint8_t numAdcPerTrigUnit = FEATURE_CTU_NUM_ADC;
#elif defined (ADC_PAL_SAR)
    const uint8_t numAdcPerTrigUnit = 1u;
#endif /* defined(ADC_PAL_TYPE_ADC_SAR_CTU) */

    uint16_t idx;

    /* Deinit only the ADC instances which correspond to the current ADC PAL instance,
     * and are active, i.e. have at least one channel assigned to at least one group. */
    uint16_t activeAdcMask  = ADC_GetUsedAdcInstances(instIdx);
    const uint16_t startIdx = instIdx * numAdcPerTrigUnit;
    const uint16_t endIdx   = (instIdx + 1u) * numAdcPerTrigUnit;
    for (idx = startIdx; idx < endIdx; idx++)
    {
        if ((activeAdcMask & ((uint16_t)1u << idx)) != 0u)
        {
            ADC_DRV_Reset(idx);
        }
    }
}

#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_CheckChanDuplicate
 * Description   : Check that a channel doesn't appear multiple times within a group
 * for ADC_PAL_TYPE_ADC_SAR_BCTU: check for SW and HW triggered groups
 * for ADC_PAL_TYPE_ADC_SAR_CTU: check only for SW triggered groups. HW triggered groups may contain
 * the same channel multiple times, because results are stored in CTU result FIFOs.
 *
 * END**************************************************************************/
static void ADC_CheckChanDuplicate(const adc_config_t * const config)
{
    uint8_t numOccurrences[ADC_CDR_COUNT] = {0}; /* total number of ADC channels is given by ADC_CDR_COUNT */
    uint8_t idx1, idx;
    uint8_t chanIdx;
    const adc_group_config_t * currentGroup;

    /* Workaround for MISRA false positive reported for Rule 9.1 - array conceivably not initialized.
     * The local array is already initialized at definition. */
    for (idx = 0u; idx < ADC_CDR_COUNT; idx++)
    {
        numOccurrences[idx] = 0u;
    }

    for (idx = 0u; idx < config->numGroups; idx++)
    {
        currentGroup = &(config->groupConfigArray[idx]);

#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)
        if (currentGroup->hwTriggerSupport == true)
        {
            continue;
        }

#endif /* defined(ADC_PAL_TYPE_ADC_SAR_CTU) */

        /* First reset the number of occurrences of each channel */
        for (idx1 = 0u; idx1 < currentGroup->numChannels; idx1++)
        {
            chanIdx = ADC_GetChanIdx(currentGroup->inputChannelArray[idx1]);
            numOccurrences[chanIdx] = 0u;
        }

        /* Count the number of occurrences for each channel */
        for (idx1 = 0u; idx1 < currentGroup->numChannels; idx1++)
        {
            chanIdx = ADC_GetChanIdx(currentGroup->inputChannelArray[idx1]);
            numOccurrences[chanIdx]++;
        }

        for (idx1 = 0u; idx1 < currentGroup->numChannels; idx1++)
        {
            chanIdx = ADC_GetChanIdx(currentGroup->inputChannelArray[idx1]);
            if (numOccurrences[chanIdx] > 1u)
            {
                DEV_ASSERT(false); /* A channel must not appear multiple times inside a SW triggered group */
            }
        }
    }
}

#endif /* defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT) */

#endif /* (defined(ADC_PAL_TYPE_ADC_SAR_BCTU) || defined(ADC_PAL_TYPE_ADC_SAR_CTU)) */


#if (defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC))
/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_GetHwGroupStatePtr
 * Description   : Returns pointer to the HW triggered state structure.
 * Mapping of groupIndex to index of the HW triggered state is platform specific.
 *
 * END**************************************************************************/
static inline adc_group_state_t * ADC_GetHwGroupStatePtr(adc_pal_state_t * const palState,
                                                         const uint32_t groupIdx)
{
#if defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_EQADC)

    (void)groupIdx;

#if ADC_PAL_MAX_NUM_HW_GROUPS_EN > 1u
#error "Support for only a single active HW group at a time"
#endif /* ADC_PAL_MAX_NUM_HW_GROUPS_EN > 1u */

    return &(palState->hwTrigGroupState[0u]); /*Support only a single active group at a time, so any active groupIdx will be mapped to state idx 0 */

#elif (defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU))

    const uint32_t stateIdx = palState->stateIdxMapping[groupIdx];

    return &(palState->hwTrigGroupState[stateIdx]);

#endif /* (defined(ADC_PAL_TYPE_ADC_SAR_BCTU) || defined(ADC_PAL_TYPE_ADC_SAR_CTU)) */
}
#endif /*  (defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC)) */


/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_GetPalStateArray
 * Description   : Returns pointer to ADC PAL state, for the input ADC PAL type and instance
 *
 * END**************************************************************************/
static adc_pal_state_t * ADC_GetPalState(const adc_instance_t * const instance)
{
    adc_pal_state_t * palState = NULL;
    const adc_inst_type_t instType = instance->instType;
    const uint32_t instIdx = instance->instIdx;

#if defined (ADC_PAL_S32K1xx)
    if(instType == ADC_INST_TYPE_ADC_S32K1xx)
    {
        DEV_ASSERT(instIdx < NUMBER_OF_ADC_PAL_INSTANCES_S32K1xx);
        palState = &(adcPalState[instIdx]);
    }
    else
#endif /* defined (ADC_PAL_S32K1xx) */

#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
    if(instType == ADC_INST_TYPE_ADC_SAR_BCTU)
    {
        DEV_ASSERT(instIdx < NUMBER_OF_ADC_PAL_INSTANCES_ADC_SAR_BCTU);
        palState = &(adcPalState_adc_sar_bctu[instIdx]);
    }
    else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_BCTU) */

#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)
    if(instType == ADC_INST_TYPE_ADC_SAR_CTU)
    {
        DEV_ASSERT(instIdx < NUMBER_OF_ADC_PAL_INSTANCES_ADC_SAR_CTU);
        palState = &(adcPalState_adc_sar_ctu[instIdx]);
    }
    else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_CTU) */

#if defined (ADC_PAL_SAR)
    if(instType == ADC_INST_TYPE_ADC_SAR)
    {
        DEV_ASSERT(instIdx < NUMBER_OF_ADC_PAL_INSTANCES_ADC_SAR);
        palState = &(adcPalState_adc_sar[instIdx]);
    }
    else
#endif /* defined (ADC_PAL_SAR) */

#if defined (ADC_PAL_TYPE_SDADC)
    if(instType == ADC_INST_TYPE_SDADC)
    {
        DEV_ASSERT(instIdx < NUMBER_OF_ADC_PAL_INSTANCES_SDADC);
        palState = &(adcPalState_sdadc[instIdx]);
    }
    else
#endif /* defined (ADC_PAL_TYPE_SDADC) */

#if defined (ADC_PAL_TYPE_EQADC)
    if(instType == ADC_INST_TYPE_EQADC)
    {
        DEV_ASSERT(instIdx < NUMBER_OF_ADC_PAL_INSTANCES_EQADC);
        palState = &(adcPalState_eqadc[instIdx]);
    }
    else
#endif /* defined (ADC_PAL_TYPE_EQADC) */

    {
        DEV_ASSERT(false);
    }

    return palState;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_InitStateRuntimeFeatures
 * Description   : Initializes ADC PAL state member flags with available runtime features,
 * depending on ADC PAL type
 *
 * END**************************************************************************/
static void ADC_InitStateRuntimeFeatures(adc_pal_state_t * const palState, const adc_inst_type_t instType)
{

#if defined (ADC_PAL_S32K1xx)
    if(instType == ADC_INST_TYPE_ADC_S32K1xx)
    {
        palState->hasHwTrigSupport = true;
    }
    else
#endif /* defined (ADC_PAL_S32K1xx) */

#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
    if(instType == ADC_INST_TYPE_ADC_SAR_BCTU)
    {
        palState->hasHwTrigSupport = true;
    }
    else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_BCTU) */

#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)
    if(instType == ADC_INST_TYPE_ADC_SAR_CTU)
    {
        palState->hasHwTrigSupport = true;
    }
    else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_CTU) */

#if defined (ADC_PAL_SAR)
    if(instType == ADC_INST_TYPE_ADC_SAR)
    {
        palState->hasHwTrigSupport = false;
    }
    else
#endif /* defined (ADC_PAL_SAR) */

#if defined (ADC_PAL_TYPE_SDADC)
    if(instType == ADC_INST_TYPE_SDADC)
    {
        palState->hasHwTrigSupport = false;
    }
    else
#endif /* defined (ADC_PAL_TYPE_SDADC) */

#if defined (ADC_PAL_TYPE_EQADC)
    if(instType == ADC_INST_TYPE_EQADC)
    {
        palState->hasHwTrigSupport = true;
    }
    else
#endif /* defined (ADC_PAL_TYPE_EQADC) */
    {
        DEV_ASSERT(false);
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_AnyHwTrigGroupActive
 * Description   : Returns true if at least one hwTriggered group is enabled.
 * Otherwise returns false.
 *
 * END**************************************************************************/
static bool ADC_AnyHwTrigGroupActive(const adc_instance_t * const instance)
{
    const adc_inst_type_t instType = instance->instType;
    const uint32_t instIdx         = instance->instIdx;
    bool active = false;

#if defined (ADC_PAL_S32K1xx)
    if(instType == ADC_INST_TYPE_ADC_S32K1xx)
    {
        const adc_pal_state_t * const palState = &(adcPalState[instIdx]);

        active = palState->hwTrigGroupState[0u].active; /* S32K1xx supports only a single active group at a time, so any active groupIdx will be mapped to state idx 0 */
    }
    else
#endif /* defined (ADC_PAL_S32K1xx) */

#if defined (ADC_PAL_TYPE_ADC_SAR_BCTU)
    if(instType == ADC_INST_TYPE_ADC_SAR_BCTU)
    {
        const adc_pal_state_t * const palState = &(adcPalState_adc_sar_bctu[instIdx]);

        active = false;
        uint8_t idx = 0u;

        while ((active == false) && (idx < ADC_PAL_MAX_NUM_HW_GROUPS_EN))
        {
            if (palState->hwTrigGroupState[idx].active == true)
            {
                active = true;
            }

            idx++;
        }
    }
    else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_BCTU) */

#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)
    if(instType == ADC_INST_TYPE_ADC_SAR_CTU)
    {
        const adc_pal_state_t * const palState = &(adcPalState_adc_sar_ctu[instIdx]);

        active = false;
        uint8_t idx = 0u;

        while ((active == false) && (idx < ADC_PAL_MAX_NUM_HW_GROUPS_EN))
        {
            if (palState->hwTrigGroupState[idx].active == true)
            {
                active = true;
            }

            idx++;
        }
    }
    else
#endif /* defined (ADC_PAL_TYPE_ADC_SAR_CTU) */

#if defined (ADC_PAL_SAR)
    if(instType == ADC_INST_TYPE_ADC_SAR)
    {
        (void) instIdx;
        active = false; /* doesn't support HW triggered groups */
    }
    else
#endif /* defined (ADC_PAL_SAR) */

#if defined (ADC_PAL_TYPE_SDADC)
    if(instType == ADC_INST_TYPE_SDADC)
    {
        (void) instIdx;
        active = false; /* doesn't support HW triggered groups */
    }
    else
#endif /* defined (ADC_PAL_TYPE_SDADC) */

#if defined (ADC_PAL_TYPE_EQADC)
    if(instType == ADC_INST_TYPE_EQADC)
    {
        const adc_pal_state_t * const palState = &(adcPalState_eqadc[instIdx]);

        active = palState->hwTrigGroupState[0u].active; /* supports only a single active group at a time, so any active groupIdx will be mapped to state idx 0 */
    }
    else
#endif /* defined (ADC_PAL_TYPE_EQADC) */
    {
        DEV_ASSERT(false);
    }

    return active;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_PalStateUpdateStart
 * Description   : Update PAL state structure with changes for StartGroupConversion and
 * EnableHardwareTigger.
 *
 * END**************************************************************************/
static void ADC_PalStateUpdateStart(adc_pal_state_t * const palState,
                                    const uint32_t groupIdx)
{
    const adc_group_config_t * const currentGroupCfg = &(palState->groupArray[groupIdx]);
    adc_group_state_t * groupState;

#if (defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC))
    if (currentGroupCfg->hwTriggerSupport == true)
    {
        groupState = ADC_GetHwGroupStatePtr(palState, groupIdx);
    }
    else
    {
        groupState = &(palState->swTrigGroupState);
    }
#else
    groupState = &(palState->swTrigGroupState);
#endif /* (defined (ADC_PAL_S32K1xx) || defined (ADC_PAL_TYPE_ADC_SAR_BCTU) || defined (ADC_PAL_TYPE_ADC_SAR_CTU) || defined (ADC_PAL_TYPE_EQADC)) */

    palState->latestGroupIdx        = groupIdx;
    groupState->active              = true;
    groupState->currentBufferOffset = 0u;
#if defined (ADC_PAL_TYPE_ADC_SAR_CTU)
    groupState->groupIdx            = groupIdx;
#endif
#if defined (ADC_PAL_TYPE_SDADC) || defined (ADC_PAL_TYPE_EQADC)
    groupState->currentConvIdx      = 0u;
#endif
    groupState->bufferLength        = (uint16_t)currentGroupCfg->numChannels * currentGroupCfg->numSetsResultBuffer;
    if (currentGroupCfg->callback != NULL)
    {
        groupState->notificationEn = true; /* enable notification by default if callback is available */
    }
    else
    {
        groupState->notificationEn = false; /* disable notification if callback is not available */
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ADC_CallNotificationCb
 * Description   : Calls the notification callback for the current active group,
 * if it is enabled.
 *
 * END**************************************************************************/
static void ADC_CallNotificationCb(const adc_pal_state_t * const palState,
                                   const uint32_t groupIdx,
                                   const adc_group_state_t * const groupState)
{
    const adc_group_config_t * const activeGroupCfg = &(palState->groupArray[groupIdx]);

    /* Call notification callback, if it is enabled */
    if (groupState->notificationEn)
    {
        adc_callback_info_t cbInfo;
        cbInfo.groupIndex = groupIdx;

        if (groupState->currentBufferOffset == 0u)
        {
            cbInfo.resultBufferTail = (uint16_t)(groupState->bufferLength - 1u); /* set tail to the last position in buffer */
        }
        else
        {
            cbInfo.resultBufferTail = (uint16_t)(groupState->currentBufferOffset - 1u); /* set tail to last written position  */
        }

        (*(activeGroupCfg->callback))(&cbInfo, activeGroupCfg->callbackUserData);
    }
}



/*******************************************************************************
 * EOF
 ******************************************************************************/
