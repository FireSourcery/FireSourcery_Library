
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
    @file   Analog.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Analog_ADC.h"
#include "Config.h"

#include <stdint.h>
#include <stdbool.h>


/*
    Application handle
    handle requires at least one dereference, either P_ADC or P_CONVERSION_STATE
*/
typedef const struct Analog_Conversion
{
    Analog_ConversionState_T * P_CONVERSION_STATE;
    /* reserve interface for extension */
    // Config
    /* for direct start, overwrite */
    // Analog_ConversionChannel_T * P_CONVERSION_CHANNEL;
    // Analog_ADC_T * const P_ADC;
}
Analog_Conversion_T;

#define ANALOG_CONVERSION_INIT_FROM(AdcStruct, ChannelIndex) { .P_CONVERSION_STATE =  &((AdcStruct).P_CONVERSION_STATES[ChannelIndex]), }

// #define ANALOG_CONVERSION_INIT_FROM_TYPED(AdcStruct, ChannelId, ExpectedType) \
// { \
//     .P_CONVERSION_STATE = &((AdcStruct).P_CONVERSION_STATES[ChannelId]), \
//     static_assert(ChannelId < (AdcStruct).CHANNEL_COUNT, "Channel ID out of range") \
// }

static inline adc_result_t Analog_Conversion_GetResult(const Analog_Conversion_T * p_conv) { return p_conv->P_CONVERSION_STATE->Result; }
static inline void Analog_Conversion_MarkConversion(const Analog_Conversion_T * p_conv) { p_conv->P_CONVERSION_STATE->IsMarked = true; }
static inline bool Analog_Conversion_IsMarked(const Analog_Conversion_T * p_conv) { return p_conv->P_CONVERSION_STATE->IsMarked; }


// typedef const struct Analog_ChannelInit
// {
//     Analog_ADC_T * const P_ADC; /* pointer to shared ADC State, in the case shared */
//     const analog_channel_t ID; /* Virtual Channel Index. index into ADC.P_CONVERSION_STATES */
//     // const adc_pin_t PIN; /* Physical Id of the Pin */
// }
// Analog_ChannelInit_T;


/******************************************************************************/
/*
    Conversion Batch
    A typed unit interface across multiple ADCs.
    Synchronized start with 1 callback, seperate state buffer
*/
/******************************************************************************/
// typedef struct Analog_BatchState
// {
//     uint16_t CompleteFlags; /* result of ADCs async run. Check complete without loop */
//     adc_result_t Results[]; /* allocate to size at compile time */
// }
// Analog_BatchState_T;

// channels per adc at compile time
// typedef const struct Analog_BatchAdcChannels
// {
//     Analog_Channel_T * const P_CHANNELS;
// }
// Analog_BatchAdcChannels_T;
// or map adc to batch

// typedef const struct Analog_ConversionBatch
// {
//     uint8_t BATCH_ID;
//     Analog_Channel_T * const P_CHANNELS;
//     const uint8_t COUNT; /* > 1 for Batch */

//     void * const P_CONTEXT;
//     const Analog_Callback_T ON_COMPLETE; /* On complete, runs at ISR prioity */
//     // Analog_ConversionState_T * const P_STATES; //not inline with iterateb by adc
//     Analog_BatchState_T * const P_BATCH_STATE;
// }
// Analog_ConversionBatch_T;

// #define ANALOG_CONVERSION_BATCH_INIT(p_Channels, Count, p_Context, Callback, p_States) \
// {                                       \
//     .P_CHANNELS         = p_Channels,   \
//     .CONVERSION_COUNT   = Count,        \
//     .P_CONTEXT          = p_Context,    \
//     .ON_COMPLETE        = Callback,     \
//     .P_STATES           = p_States,     \
// }

// #define ANALOG_CONVERSION_BATCH_ALLOC(p_Channels, Count, p_Context, Callback) \
//     ANALOG_CONVERSION_BATCH_INIT(p_Channels, Count, p_Context, Callback, (Analog_ConversionState_T[Count]){})

// /* mark as batch, without batch on complete */
// // static inline void _Analog_Channels_Mark(const Analog_Channel_T * p_channels, uint8_t count) { void_array_foreach(p_channels, sizeof(Analog_Channel_T), count, (proc_t)Analog_MarkConversion); }

// /* by adc state */
// // static inline adc_result_t _Analog_Batch_Result_ADC(const Analog_ConversionBatch_T * p_batch, uint8_t batchIndex) { return _Analog_Channel_GetResult(&p_batch->P_CHANNELS[batchIndex]); }
// // static inline bool _Analog_Batch_IsComplete_ADC(const Analog_ConversionBatch_T * p_batch)
// // {
// //     for (uint8_t index = 0U; index < p_batch->COUNT; index++)
// //     {
// //         if (_Analog_Channel_IsMarked(&p_batch->P_CHANNELS[index]) == false) { return false; } marked channels clear on start
// //     }
// // }

// static inline bool Analog_Batch_IsComplete(const Analog_ConversionBatch_T * p_batch)
// {
//     return (p_batch->P_BATCH_STATE->CompleteFlags == ((1U << p_batch->COUNT) - 1U));
// }










/******************************************************************************/
/*
    Channel
    Board HAL compile time define.
    Implementation as mapping ADC Host to ADC Channel.
    This way ADC_T does not need to be defined per Board HAL.
    stuctured by caller source, alternatively split by threading
*/
/******************************************************************************/
// typedef const struct Analog_Channel
// {
//     const analog_channel_t ID; /* Virtual Channel Index. index into ADC.P_CONVERSION_STATES */
//     Analog_ADC_T * const P_ADC; /* pointer to shared ADC State, in the case shared  */
//     // HAL_ADC_T * const P_HAL_ADC; /* Case where registers hold entire fifo state */
//     const adc_pin_t PIN; /* Physical Id of the Pin */
// }
// Analog_Channel_T;

// #define ANALOG_CHANNEL_INIT(Channel, p_AnalogAdc, PinId) { .ID = Channel, .P_ADC = p_AnalogAdc, .PIN = PinId, }



// /*
//     Add Conversion by marking its flag
//     Analog_ConversionChannel_T provides ADC select abstraction
// */
// static inline adc_result_t _Analog_Channel_GetResult(const Analog_Channel_T * p_channel) { return Analog_ADC_ResultOf(p_channel->P_ADC, p_channel->ID); }
// static inline void _Analog_Channel_MarkConversion(const Analog_Channel_T * p_channel) { Analog_ADC_MarkConversion(p_channel->P_ADC, p_channel->ID); }
// static inline bool _Analog_Channel_IsMarked(const Analog_Channel_T * p_channel) { return Analog_ADC_IsMarked(p_channel->P_ADC, p_channel->ID); }

/******************************************************************************/
/*
    Conversion Channel with Callback
    A typed unit interface a single ADC.
*/
/******************************************************************************/
// typedef const struct Analog_ConversionChannel
// {
//     const Analog_Channel_T CHANNEL; /* small enough to copy/store by value */

//     // / Analog_ConversionState_T * P_CHANNEL_STATE; /* alternatively replace channel, directely write on oncomplete as well */
//     // const adc_pin_t PIN;
//     void * const P_CONTEXT;
//     const Analog_Capture_T CAPTURE; /* Overwrite capture to ADC Buffer */
// }
// Analog_ConversionChannel_T;


// #define ANALOG_CONVERSION_CHANNEL_INIT(Channel, p_Context, CaptureFn) { .CHANNEL = Channel, .P_CONTEXT = p_Context, .CAPTURE = CaptureFn, }

// #define ANALOG_CONVERSION_CHANNEL_INIT_WITH_STATE(Channel, p_Context, CaptureFn, p_State) \
// {                                   \
//     .CHANNEL        = Channel,      \
//     .P_CONTEXT      = p_Context,    \
//     .ON_COMPLETE    = CaptureFn,    \
//     .P_STATE        = p_State,      \
// }

/*
    Result from the ADC buffer by default, redirect with callback
*/
// static inline adc_result_t Analog_Channel_GetResult(const Analog_ConversionChannel_T * p_channel) { return _Analog_Channel_GetResult(&p_channel->CHANNEL); }
// static inline void Analog_Channel_MarkConversion(const Analog_ConversionChannel_T * p_channel) { _Analog_Channel_MarkConversion(&p_channel->CHANNEL); }
// static inline bool Analog_Channel_IsMarked(const Analog_ConversionChannel_T * p_channel) { return _Analog_Channel_IsMarked(&p_channel->CHANNEL); }