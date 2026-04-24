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
// #if     defined(ANALOG_ADC_HW_FIFO_DISABLE)
// #elif   defined(ANALOG_ADC_HW_FIFO_ENABLE)
// #else
//     #define ANALOG_ADC_HW_FIFO_DISABLE
// #endif

// #if     defined(ANALOG_ADC_HW_CONTINUOUS_CONVERSION_ENABLE)
// #elif   defined(ANALOG_ADC_HW_CONTINUOUS_CONVERSION_DISABLE)
// #else
//     #define ANALOG_ADC_HW_CONTINUOUS_CONVERSION_DISABLE
// #endif


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
        uint32_t IsMarked : 1U; /* depreciate */
        uint32_t Reserved : 15U;
        // uint32_t IsNewResult : 1U; // new result sync flag
        // volatile bool IsActive; // allow mark while active /* !IsComplete */
    };
    uint32_t State; /* setting State effectively clears IsMarked */
}
Analog_ConversionState_T;

#define ANALOG_CONVERSION_STATE_ALLOC() (&(Analog_ConversionState_T){0})

/* AdcChannel */
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

// #define ANALOG_CONVERSION_CHANNEL_INIT_ALLOC(ChannelId, PinId, p_Context, CaptureFn, ...) (Analog_ConversionChannel_T) \
//     { .ID = ChannelId, .PIN = PinId , .CAPTURE = (Analog_Capture_T)CaptureFn, .P_CONTEXT = p_Context, .P_CONVERSION_STATE = ANALOG_CONVERSION_STATE_ALLOC(), }

/******************************************************************************/
/*
*/
/******************************************************************************/
// // Per ADC batch part
// typedef void (*Analog_CaptureBatch_T)(void * p_context, Analog_ConversionChannel_T * p_states);
// typedef const struct
// {
//     uint32_t CHANNELS_MASK;  /* directly maps to adc so no batch state is needed, adc holds batch operator  */
//     adc_result_t * P_RESULTS;
//     Analog_Callback_T ON_COMPLETE;
//     void * P_CONTEXT;
// }
// Analog_AdcBatch_T; Analog_ConversionContext_T


/******************************************************************************/
/*
    ADC State
    Critical section buffer shared by ISR and StartConversions.
*/
/******************************************************************************/
// typedef struct Analog_ChannelContext / Substription
// Analog_ConversionChannel_T
// typedef struct Analog_OnComplete
// {
//     analog_channel_t CHANNEL;
//     Analog_Capture_T CAPTURE;
//     void * P_CONTEXT;
// }
// Analog_OnComplete_T;

// static inline void _ADC_OnComplete(Analog_OnComplete_T * p_conversion, adc_result_t * p_buffer, adc_result_t result)
// {
//     /* eliminate double buffer, at additional interrupt time */
//     if (p_conversion->CAPTURE != NULL) { p_conversion->CAPTURE(p_conversion->P_CONTEXT, result); }
//     else { p_buffer[p_conversion->CHANNEL] = result; }
// }


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
    // void * ChannelContexts[ADC_CHANNEL_COUNT_MAX];

    /* ActiveBatch Const context */
    /* Selectable conversions context */
    /* Separate execution from per channel operation */
    // full context include call back in context
    // const Analog_ConversionContext_T * p_ActiveContext;

#ifndef NDEBUG
    uint32_t ErrorCount;
    uint32_t IncompleteCycles;
    uint32_t FifoMismatch;
#endif
}
Analog_ADC_State_T;

#define ANALOG_ADC_STATE_ALLOC() (&(Analog_ADC_State_T){0})


/******************************************************************************/
/*
    ADC Peripheral Control
        - Context Per Thread
        - Wraps HAL_ADC with State, callback context
    ADConverter_T
*/
/******************************************************************************/
typedef const struct Analog_ADC
{
    HAL_ADC_T * P_HAL_ADC;              /* ADC register map base address */
    Analog_ADC_State_T * P_ADC_STATE;   /* State data not retained by registers */
    const Analog_ConversionChannel_T * P_CONVERSION_CHANNELS; /*  In this case, ADC Structs must be defined for each Board HAL. */
    uint8_t CHANNEL_COUNT; /* Number of channels in the ADC */ /* allow repeat pins for different callbacks */
    /* map by adc_channel_t. handle DMA */
    const adc_pin_t * P_CHANNEL_PINS;
    adc_result_t * P_CHANNEL_RESULTS;
    // const struct { const adc_pin_t PIN; adc_result_t * P_RESULT; } * P_CHANNELS;
    // const struct { Analog_Capture_T CAPTURE; void * P_CONTEXT; } * P_CHANNEL_CALLBACKS; /* optionally, overrides P_RESULT */
}
Analog_ADC_T;

#define ANALOG_ADC_INIT(p_HalAnalog, ChannelCount, p_ConvChannels, p_AdcState) (Analog_ADC_T) \
    { .P_HAL_ADC = p_HalAnalog, .CHANNEL_COUNT = ChannelCount, .P_CONVERSION_CHANNELS = p_ConvChannels, .P_ADC_STATE = p_AdcState, }


/******************************************************************************/
/*!

*/
/******************************************************************************/
static inline void Analog_ADC_MarkConversion(const Analog_ADC_T * p_adc, analog_channel_t channel) { p_adc->P_ADC_STATE->ChannelMarkers |= (1U << channel); }
static inline bool Analog_ADC_IsMarked(const Analog_ADC_T * p_adc, analog_channel_t channel) { return (p_adc->P_ADC_STATE->ChannelMarkers & (1U << channel)) != 0UL; }
// static inline void Analog_ADC_ClearMark(const Analog_ADC_T * p_adc, analog_channel_t channel) { p_adc->P_ADC_STATE->ChannelMarkers &= ~(1U << channel); }
static inline void Analog_ADC_MarkAll(const Analog_ADC_T * p_adc, analog_mask_t mask) { p_adc->P_ADC_STATE->ChannelMarkers |= mask; }

static inline Analog_ConversionChannel_T * Analog_ADC_ConversionOf(const Analog_ADC_T * p_adc, analog_channel_t channel) { return &p_adc->P_CONVERSION_CHANNELS[channel]; }
static inline adc_result_t Analog_ADC_ResultOf(const Analog_ADC_T * p_adc, analog_channel_t channel) { return p_adc->P_CONVERSION_CHANNELS[channel].P_CONVERSION_STATE->Result; }


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
*/
/******************************************************************************/
static void Analog_ADC_Init(const Analog_ADC_T * p_adc)
{
    HAL_ADC_Init(p_adc->P_HAL_ADC);
    HAL_ADC_Deactivate(p_adc->P_HAL_ADC);
}



