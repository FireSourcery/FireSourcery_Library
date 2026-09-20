#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   Analog_ADC.h
    @author FireSourcery
    @brief
*/
/******************************************************************************/
// #include "Analog.h"
#include "HAL_ADC.h"

#include <stdint.h>
#include <stdbool.h>


#ifdef HAL_ADC_FIFO_LENGTH_MAX
#define ADC_FIFO_LENGTH_MAX HAL_ADC_FIFO_LENGTH_MAX
#endif

#ifndef ADC_FIFO_LENGTH_MAX
#define ADC_FIFO_LENGTH_MAX 1U
#endif

#if (ADC_FIFO_LENGTH_MAX > 1U)
#define ADC_FIFO_ENABLED
#endif

/******************************************************************************/
/*
    Module Common Defs
    Organized by unit of execution
*/
/******************************************************************************/
/******************************************************************************/
/*
    Channel
    Board HAL compile time define.
*/
/******************************************************************************/
typedef uint8_t analog_channel_t; /* Virtual Channel Index. resolve to Analog_Conversion_T */
typedef uint32_t analog_mask_t;
static inline analog_mask_t Analog_Mask(analog_channel_t channel) { return ((analog_mask_t)1UL << channel); }

/******************************************************************************/
/*
*/
/******************************************************************************/
typedef void (*Analog_Callback_T)(void * p_context);
typedef void (*Analog_Capture_T)(void * p_context, adc_result_t value);


/******************************************************************************/
/*
    Conversion Channel/Unit
    holds ADC_OnComplete context per channel
    granular unit of execution for ADC
*/
/******************************************************************************/
typedef union Analog_ConversionState
{
    struct
    {
        uint32_t Result : 16U;
        // uint32_t IsMarked : 1U; /* depreciate */
        // uint32_t Reserved : 15U;
        // uint32_t IsNewResult : 1U; // new result sync flag
        // volatile bool IsActive; // allow mark while active /* !IsComplete */
    };
    uint32_t State; /* setting State effectively clears IsMarked */
}
Analog_ConversionState_T;

#define ANALOG_CONVERSION_STATE_ALLOC() (&(Analog_ConversionState_T){})

/* AdcChannel on complete context */
typedef const struct Analog_ConversionChannel
{
    analog_channel_t ID;  /* Virtual Channel Index. Index into ADC.P_CHANNELS */
    adc_pin_t PIN;        /* Physical Id of the Pin */
    // const uint32_t ID_MASK;
    /* each channel allocates it own buffer. directly use by oncomplete needs HAL only, accounts for results buffer map */
    volatile Analog_ConversionState_T * P_CONVERSION_STATE;

    Analog_Capture_T CAPTURE; /* Overwrite capture to ADC Buffer */
    void * P_CONTEXT;
}
Analog_ConversionChannel_T;

#define ANALOG_CONVERSION_CHANNEL_INIT(ChannelId, PinId, p_State, p_Context, CaptureFn) (Analog_ConversionChannel_T) \
    { .ID = ChannelId, .PIN = PinId , .CAPTURE = (Analog_Capture_T)CaptureFn, .P_CONTEXT = p_Context, .P_CONVERSION_STATE = p_State, }

// #define ADC_CHANNEL_INIT(ChannelId, PinId, p_Context, CaptureFn )
//     { .ID = ChannelId, .PIN = PinId , .CAPTURE = (Analog_Capture_T)CaptureFn, .P_CONTEXT = p_Context, .P_CONVERSION_STATE = ANALOG_CONVERSION_STATE_ALLOC(), }


/******************************************************************************/
/*
    Per ADC Batch
*/
/******************************************************************************/
/* p_results is the Hw transfer destination. const volatile: the batch reads what the transfer wrote */
typedef void (*Analog_CaptureBatch_T)(void * p_context, const volatile adc_result_t * p_results, uint8_t count);

typedef const struct ADC_ConversionBatch
{
    analog_channel_t ID_START;
    uint8_t COUNT;
    // pins and buffer ADC common
    // Analog_ConversionChannel_T * P_CONVERSION_CHANNELS;
    analog_mask_t CHANNELS;         /* Channel IDs. Contiguous for Hw sequencing */
    // Analog_Callback_T ON_COMPLETE;  /* Results in P_CHANNEL_RESULTS. NULL for scan only */
    void * P_CONTEXT;
    Analog_CaptureBatch_T CAPTURE; /* pulls from the ADC results buffer */
}
ADC_ConversionBatch_T;


/******************************************************************************/
/*
    ADC State
    Critical section buffer shared by ISR and StartConversions.
*/
/******************************************************************************/
typedef struct Analog_ADC_State
{
    /*
        Reg/Fifo State
        maintained by software in case of fifo where direct id map is not available.

        Result buffer outside of ADC_State allows for the ADC state use as purely setup control
        remain unmodified through the entire conversion process.

        More concise to let compile optimize array of 1.
    */
    const Analog_ConversionChannel_T * ActiveConversions[ADC_FIFO_LENGTH_MAX]; /* Array of pointers */
    uint8_t ActiveConversionCount; /* Hw fifo only. Number of active channels being processed by ADC */
    // const adc_channel_t ActiveConversions[ADC_FIFO_LENGTH_MAX];     // alternative simplified implementation, map by adc_channel_t
    // const Analog_OnComplete_T * ActiveConversions[ADC_FIFO_LENGTH_MAX];

    /* Batch/Queue State */
    /* If left non atomic. a mark channel call may be missed. */
    volatile uint32_t ChannelMarkers; /* Bitmask of selected channels. 1 << ChannelIndex */

    /* ActiveBatch Const context */
    /* Selectable conversions context */
    /* Separate execution from per channel operation */
    // full context include call back in context
    // const Analog_ConversionContext_T * p_ActiveContext;
    // const Analog_ConversionChannel_T * p_BatchChannels;

    /* Hw Sequenced Batch. Active is set only in the complete ISR window, Next by any thread */
    const struct ADC_ConversionBatch * volatile p_ActiveBatch;
    const struct ADC_ConversionBatch * volatile p_NextBatch;

#ifndef NDEBUG
    uint32_t ErrorCount;
    uint32_t IncompleteCycles;
    uint32_t FifoMismatch;
#endif
}
Analog_ADC_State_T;

/******************************************************************************/
/*
    ADC Peripheral Control
        - Context Per Thread
        - Wraps HAL_ADC with State, callback context
    ADC_Module_T
*/
/******************************************************************************/
typedef const struct Analog_ADC
{
    HAL_ADC_T * P_HAL_ADC;              /* ADC register map base address */
    Analog_ADC_State_T * P_ADC_STATE;   /* State data not retained by registers */
    Analog_ConversionChannel_T * P_CONVERSION_CHANNELS; /*  In this case, ADC Structs must be defined for each Board HAL. */
    uint8_t CHANNEL_COUNT; /* Number of channels in the ADC */ /* allow repeat pins for different callbacks */

    /* map by adc_channel_t. handle with parallel arrays for DMA compatibility */
    const adc_pin_t * P_CHANNEL_PINS;
    volatile adc_result_t * P_CHANNEL_RESULTS; /* Hw transfer destination. [Channel ID] == [Slot] */

    ADC_ConversionBatch_T * P_CONVERSION_BATCHS;
}
Analog_ADC_T;


/******************************************************************************/
/*! */
/******************************************************************************/
/*
    sufficient For lower priority thread check. lower priority thread cannot override ISR update
    HAL_ADC_ReadConversionCompleteFlag will not be set if called from lower priority thread
*/
static inline bool Analog_ADC_ReadIsActive(const Analog_ADC_T * p_adc) { return HAL_ADC_ReadConversionActiveFlag(p_adc->P_HAL_ADC); }
static inline void Analog_ADC_Deactivate(const Analog_ADC_T * p_adc) { HAL_ADC_Deactivate(p_adc->P_HAL_ADC); }


/******************************************************************************/
/*!
    API on indexes ensure no adc mismatch
*/
/******************************************************************************/
static inline void Analog_ADC_MarkConversion(const Analog_ADC_T * p_adc, analog_channel_t channel) { p_adc->P_ADC_STATE->ChannelMarkers |= (1U << channel); }
static inline bool Analog_ADC_IsMarked(const Analog_ADC_T * p_adc, analog_channel_t channel) { return (p_adc->P_ADC_STATE->ChannelMarkers & (1U << channel)) != 0UL; }
static inline void Analog_ADC_MarkAll(const Analog_ADC_T * p_adc, analog_mask_t mask) { p_adc->P_ADC_STATE->ChannelMarkers |= mask; }
// static inline void _ADC_ClearMarker(const Analog_ADC_T * p_adc, analog_channel_t channel) { p_adc->P_ADC_STATE->ChannelMarkers &= ~(1U << channel); }

static inline Analog_ConversionChannel_T * Analog_ADC_ConversionOf(const Analog_ADC_T * p_adc, analog_channel_t channel) { return &p_adc->P_CONVERSION_CHANNELS[channel]; }
// static inline adc_result_t Analog_ADC_ResultOf(const Analog_ADC_T * p_adc, analog_channel_t channel) { return p_adc->P_CONVERSION_CHANNELS[channel].P_CONVERSION_STATE->Result; }

static inline adc_result_t Analog_ADC_ChannelResultOf(const Analog_ADC_T * p_adc, analog_channel_t channel) { return p_adc->P_CHANNEL_RESULTS[channel]; }


/******************************************************************************/
/*!
    Fixed ADC Batch interface
*/
/******************************************************************************/
#define ANALOG_MASK_RANGE(FirstChannel, Count) ((analog_mask_t)(((1UL << (Count)) - 1UL) << (FirstChannel)))

static inline bool Analog_Mask_IsContiguous(analog_mask_t mask) { return ((mask + (mask & (0U - mask))) & mask) == 0U; }

static inline const ADC_ConversionBatch_T * Analog_ADC_GetActiveBatch(const Analog_ADC_T * p_adc) { return p_adc->P_ADC_STATE->p_ActiveBatch; }

/*
    Deferred. Applied in the complete ISR of the active batch, before the next trigger.
    Any thread. Last request wins.
*/
static inline void Analog_ADC_SetBatch(const Analog_ADC_T * p_adc, uint8_t batchId) { p_adc->P_ADC_STATE->p_NextBatch = &p_adc->P_CONVERSION_BATCHS[batchId]; }


/*
    Immediate activation is Analog_ADC_ActivateBatch, in _Analog_ADC.h. It resolves the Hw activation.
    Without dma use Analog_ADC_MarkAll
*/


/******************************************************************************/
/*! async overwrite*/
/******************************************************************************/
// static inline void _ADC_StartConversion(Analog_ADC_T * p_adc, analog_channel_t channel) { ADC_StartFrom(p_adc, &p_adc->P_CONVERSION_CHANNELS[channel], (1UL << channel)); }
// static inline void _ADC_StartConversions(Analog_ADC_T * p_adc, uint32_t channels) { ADC_StartFrom(p_adc, &p_adc->P_CONVERSION_CHANNELS[0], channels); }


/******************************************************************************/
/*!
*/
/******************************************************************************/
static void Analog_ADC_Init(const Analog_ADC_T * p_adc)
{
    HAL_ADC_Init(p_adc->P_HAL_ADC);
    HAL_ADC_Deactivate(p_adc->P_HAL_ADC);
}



