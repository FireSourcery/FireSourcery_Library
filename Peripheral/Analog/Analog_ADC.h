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
#include "Analog.h"
#include "HAL_ADC.h"
#include "Config.h"

#include <stdint.h>
#include <stdbool.h>


#ifndef ADC_FIFO_LENGTH_MAX

#ifdef HAL_ADC_FIFO_LENGTH_MAX
    #define ADC_FIFO_LENGTH_MAX HAL_ADC_FIFO_LENGTH_MAX
#else
    #define ADC_FIFO_LENGTH_MAX 1U
#endif

#endif

#if (ADC_FIFO_LENGTH_MAX > 1U)
#define ADC_FIFO_ENABLED
#endif




/******************************************************************************/
/*
*/
/******************************************************************************/
// typedef uint32_t analog_markers_t;

/*
    Batch Part + subsitute fixed channels
    per adc
*/
// typedef void (*Analog_CaptureBatch_T)(void * p_context, uint32_t * p_states);

// typedef struct Analog_ContextState
// {
//     volatile uint32_t ChannelMarkers; /* Bitmask of active channels. 1 << ChannelIndex */
//     volatile uint32_t CompleteMarkers; /* marked on complete. channel markers 0 may indicate not started */
// }
// Analog_ContextState_T;

// /*
//     subsitute fixed channels
//     Analog_ConversionTable/Mux
//     Analog_ADC_Conversion
// */
// typedef const struct Analog_ConversionContext
// {
//     Analog_ConversionChannel_T * P_CONVERSION_CHANNELS; /* [0,1,2,3] => [adc_channel_x, adc_channel_y, adc_channel_z] */
//     Analog_ContextState_T * P_STATE;

//     Analog_Context_T CALLBACK; /* on all complete */
// }
// Analog_ConversionContext_T;


/******************************************************************************/
/*
    ADC State
    Critical section buffer shared by ISR and StartConversions.
*/
/******************************************************************************/
/*
    Result buffer outside of ADC_State allows for the ADC state use as purely setup control
        remain unmodified through the entire conversion process.
*/
typedef struct Analog_ADC_State
{
    /*
        Fifo State
        maintained by software in case of fifo where direct id map is not available.

        More concise to let compile optimize array of 1.
    */
    const Analog_ConversionChannel_T * ActiveConversions[ADC_FIFO_LENGTH_MAX]; /* Array of pointers */
    uint8_t ActiveConversionCount; /* Hw fifo only. Number of active channels being processed by ADC */

    /* If left non atomic. a mark channel call may be missed. */
    volatile uint32_t ChannelMarkers; /* Bitmask of active channels. 1 << ChannelIndex */
    // volatile uint32_t CompleteMarkers;

    /* Selectable conversions context */
    /* Seperate execution from per channel operation */
    // this way batch can set with on going conversion,
    // regular calls to mark will mark base markers
    const Analog_ConversionChannel_T * p_ConversionChannels; // awaiting // active batch
    // volatile uint32_t * p_ChannelMarkers;
    // const Analog_ConversionContext_T * p_ConversionContext;

    // const Analog_ConversionBatch_T * ActiveConversionBatch; channels + marker

#ifndef NDEBUG
    uint32_t ErrorCount;
    uint32_t IncompleteCycles;
#endif
}
Analog_ADC_State_T;


/******************************************************************************/
/*
    ADC Context - Context Per Thread
*/
/******************************************************************************/
typedef const struct Analog_ADC
{
    HAL_ADC_T * P_HAL_ADC;              /* ADC register map base address */
    Analog_ADC_State_T * P_ADC_STATE;   /* State data not retained by registers */

    analog_channel_t CHANNEL_COUNT;     /* Number of channels in the ADC */ /* allow repeat pins for different callbacks */
    const Analog_ConversionChannel_T * P_CONVERSION_CHANNELS; /*  In this case, ADC Structs must be defined for each Board HAL. */
    // const Analog_ConversionChannel_T ** P_CONVERSION_CHANNELS; /* alternatively, may simplify compile time maping */

    // /*
    //    at a single point of reference, utilize memory locality
    // */
    // volatile Analog_ConversionState_T * P_CONVERSION_STATES;
}
Analog_ADC_T;

// #define ANALOG_ADC_INIT(p_HalAnalog, ChannelCount, p_ConvChannels, p_ConvStates, p_AdcState) \
//     { .P_HAL_ADC = p_HalAnalog, .CHANNEL_COUNT = ChannelCount, .P_CONVERSION_CHANNELS = p_ConvChannels, .P_CONVERSION_STATES = p_ConvStates, .P_ADC_STATE = p_AdcState, }

// #define ANALOG_ADC_ALLOC(p_HalAnalog, ChannelCount, p_ConvChannels) \
//     ANALOG_ADC_INIT(p_HalAnalog, ChannelCount, p_ConvChannels, (Analog_ConversionState_T[ChannelCount]){0}, &(Analog_ADC_State_T){0})

#define ANALOG_ADC_INIT(p_HalAnalog, ChannelCount, p_ConvChannels, p_AdcState) \
    { .P_HAL_ADC = p_HalAnalog, .CHANNEL_COUNT = ChannelCount, .P_CONVERSION_CHANNELS = p_ConvChannels, .P_ADC_STATE = p_AdcState, }

#define ANALOG_ADC_ALLOC(p_HalAnalog, ChannelCount, p_ConvChannels) \
    ANALOG_ADC_INIT(p_HalAnalog, ChannelCount, p_ConvChannels, &(Analog_ADC_State_T){0})


/******************************************************************************/
/*!

*/
/******************************************************************************/
typedef uint32_t analog_marker_t;
static inline analog_marker_t Analog_ADC_Marker(analog_channel_t channel) { return (analog_marker_t)(1UL << channel); }
static inline analog_marker_t Analog_ADC_MarkerOf(const Analog_ConversionChannel_T * p_channel) { return Analog_ADC_Marker(p_channel->CHANNEL.ID); }
// static inline uint8_t ADC_MarkersOf(const Analog_ConversionChannel_T * p_source, uint32_t sourceMarkers)
// {
//     uint32_t adcMarkers;
//     for (uint8_t count = 0U; (count < ADC_FIFO_LENGTH_MAX) && (sourceMarkers != 0UL); count++)
//     {
//         adcMarkers |= Analog_ADC_MarkerOf(&p_source[__builtin_ctz(sourceMarkers)]);
//     }
//     return adcMarkers;
// }


// static inline void _ADC_CaptureChannel(const Analog_ADC_T * p_adc, analog_channel_t channel, adc_result_t result) { p_adc->P_CONVERSION_STATES[channel].State = (uint32_t)result; }
// static inline adc_result_t Analog_ADC_ResultOf(const Analog_ADC_T * p_adc, analog_channel_t channel) { return p_adc->P_CONVERSION_STATES[channel].Result; }
// static inline void Analog_ADC_MarkConversion(const Analog_ADC_T * p_adc, analog_channel_t channel) { p_adc->P_CONVERSION_STATES[channel].IsMarked = true; }
// static inline bool Analog_ADC_IsMarked(const Analog_ADC_T * p_adc, analog_channel_t channel) { return p_adc->P_CONVERSION_STATES[channel].IsMarked; }

static inline void Analog_ADC_MarkConversion(const Analog_ADC_T * p_adc, analog_channel_t channel) { p_adc->P_ADC_STATE->ChannelMarkers |= (1U << channel); }
static inline bool Analog_ADC_IsMarked(const Analog_ADC_T * p_adc, analog_channel_t channel) { return (p_adc->P_ADC_STATE->ChannelMarkers & (1U << channel)) != 0UL; }
static inline void Analog_ADC_ClearMark(const Analog_ADC_T * p_adc, analog_channel_t channel) { p_adc->P_ADC_STATE->ChannelMarkers &= ~(1U << channel); }

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




// typedef const struct Analog_ConversionBatchEntry
// {
//     const Analog_ConversionChannel_T * P_CONVERSION_CHANNEL;   // [0,1,2,3] => [adc_channel_1, adc_channel_9, adc_channel_3]
//     const volatile uint32_t * p_ChannelMarkers;                 // [1,0,1]
// };

/******************************************************************************/
/*!
    ( Analog_ConversionChannel_T,  uint32_t markers) interface
     select from mapped or parameters
*/
/******************************************************************************/
/* Cancels ongoing conversions */
// start a conversion immediately
// single threaded or atomic flag test and set
// static void _Analog_ADC_StartConversions(const Analog_ADC_T * p_adc, Analog_ConversionChannel_T * p_conversions, uint32_t markers)
// {
//     if (Analog_ADC_ReadIsActive(p_adc) == false)
//     {
//         ADC_StartConversions(p_adc->P_ADC_STATE, p_conversions, markers);
//     }
//     else
//     {
//         Analog_ADC_MarkConversion(p_adc, p_conversions); // mask as adc fixed
//     }
// }

// static void _Analog_ADC_StartConversion(const Analog_ADC_T * p_adc, Analog_ConversionChannel_T * p_conversion)
// {
//     ADC_StartConversions(p_adc->P_ADC_STATE, p_conversion, 1U);
// }



/*
    Multiple ADC Batch
*/
// void Analog_ADC_StartConversionBatch(const Analog_ADC_T * p_adc, const Analog_ConversionBatch_T * p_batch)
// {

// }


/******************************************************************************/
/*!

*/
/******************************************************************************/
static void Analog_ADC_Init(const Analog_ADC_T * p_adc)
{
    HAL_ADC_Init(p_adc->P_HAL_ADC);
    HAL_ADC_Deactivate(p_adc->P_HAL_ADC);
}