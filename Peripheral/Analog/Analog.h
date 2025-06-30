
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
#include "HAL_ADC.h"
#include "Config.h"

#include <stdint.h>
#include <stdbool.h>


/******************************************************************************/
/*
    Module Common Defs
*/
/******************************************************************************/
typedef uint8_t analog_channel_t; /* Virtual Channel Index. resolve to Analog_Conversion_T */

/******************************************************************************/
/*
    Channel
    Board HAL compile time define.
*/
/******************************************************************************/
typedef const struct Analog_Channel
{
    const analog_channel_t ID;  /* Virtual Channel Index. Index into ADC.P_CONVERSION_STATES */
    const adc_pin_t PIN;        /* Physical Id of the Pin */
}
Analog_Channel_T;

#define ANALOG_CHANNEL_INIT(Id, PinId) { .ID = Id, .PIN = PinId, }
// #define ANALOG_CHANNEL_INIT(Id, p_AnalogAdc, PinId) { .ID = Id, .P_ADC = p_AnalogAdc, .PIN = PinId, }

/******************************************************************************/
/*
*/
/******************************************************************************/
typedef void (*Analog_Callback_T)(void * p_context);
typedef void (*Analog_Capture_T)(void * p_context, adc_result_t value);
// typedef void (*Analog_CaptureBatch_T)(void * p_context, uint32_t * p_states); /* caller mask with 16 */

/*
    Callback Context
*/
typedef const struct Analog_Context
{
    void * P_CONTEXT;
    Analog_Capture_T CAPTURE; /* Overwrite capture to ADC Buffer */
}
Analog_Context_T;

#define ANALOG_CONTEXT_INIT(p_Context, CaptureFn) { .P_CONTEXT = (void *)(p_Context), .CAPTURE = (Analog_Capture_T)(CaptureFn), }

/******************************************************************************/
/*
    Conversion Channel with Callback
    hold entire oncomplete context
*/
/******************************************************************************/
typedef const struct Analog_ConversionChannel
{
    Analog_Channel_T CHANNEL;   /* Small enough to copy/store by value */
    Analog_Context_T CONTEXT;   /* Context for the callback, can be NULL */
    // Analog_ConversionState_T * P_CHANNEL_STATE; /* alternatively replace channel id, directly use by oncomplete */
}
Analog_ConversionChannel_T;

#define ANALOG_CONVERSION_CHANNEL_INIT(AnalogChannel, AnalogContext) { .CHANNEL = AnalogChannel, .CONTEXT = AnalogContext, }
#define ANALOG_CONVERSION_CHANNEL_INIT_FROM(ChannelId, PinId, p_Context, CaptureFn) { .CHANNEL = ANALOG_CHANNEL_INIT(ChannelId, PinId), .CONTEXT = ANALOG_CONTEXT_INIT(p_Context, CaptureFn) }

// #define ANALOG_CONVERSION_CHANNEL_INIT_FROM_MAP(ChannelArrayTable, Length)

/*

*/
typedef union Analog_ConversionState
{
    struct
    {
        uint32_t Result : 16U;
        uint32_t IsMarked : 1U;
        uint32_t Reserved : 15U;
        // uint32_t IsComplete : 1U; // new result sync flag
        // volatile bool IsActive; // allow mark while active /* !IsComplete */
    };
    uint32_t State; /* setting State effectively clears IsMarked */
}
Analog_ConversionState_T;


/******************************************************************************/
/*
    Application handle
    Feature module holds the pointer. ADC owns the state.
    handle requires at least one dereference, either P_ADC or P_CONVERSION_STATE
*/
/******************************************************************************/
/* Analog_ConversionHandle */
typedef const struct Analog_Conversion
{
    volatile Analog_ConversionState_T * P_CONVERSION_STATE;
    /* reserve interface for extension */
    // Options/Config
}
Analog_Conversion_T;

#define ANALOG_CONVERSION_INIT(p_ConversionState) { .P_CONVERSION_STATE = p_ConversionChannel, }
#define ANALOG_CONVERSION_INIT_FROM(AdcStruct, ChannelIndex) { .P_CONVERSION_STATE = &((AdcStruct).P_CONVERSION_STATES[ChannelIndex]), }

static inline adc_result_t Analog_Conversion_GetResult(const Analog_Conversion_T * p_conv) { return p_conv->P_CONVERSION_STATE->Result; }
static inline void Analog_Conversion_ClearResult(const Analog_Conversion_T * p_conv) { p_conv->P_CONVERSION_STATE->Result = 0; }
static inline void Analog_Conversion_Mark(const Analog_Conversion_T * p_conv) { p_conv->P_CONVERSION_STATE->IsMarked = true; }
static inline bool Analog_Conversion_IsMarked(const Analog_Conversion_T * p_conv) { return p_conv->P_CONVERSION_STATE->IsMarked; }


// typedef const struct Analog_ChannelInit
// {
//     const analog_channel_t ID; /* Virtual Channel Index. index into ADC.P_CONVERSION_STATES */
//     Analog_ADC_T * const P_ADC; /* pointer to shared ADC State, in the case shared */
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



// typedef struct Analog_OptionsFlags
// {
//     uint32_t HwTriggerConversion      : 1U;
//     uint32_t ContinuousConversion     : 1U;
//     uint32_t CaptureLocalPeak         : 1U; /* for now, conversion stops on 1 local peak in channel set, user must also set ContinuousConversion */
//     uint32_t HwAveraging              : 1U;
//     uint32_t HwTriggerChannel         : 1U; /* Per Hw buffer complete. Per Channel if both are set*/
//     uint32_t Interrupt                : 1U;
//     uint32_t Dma                      : 1U;
//     uint8_t Priority
// }
// Analog_OptionsFlags_T;





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