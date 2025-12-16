
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
// #include "Config.h"
#include "Analog_ADC.h"

#include <stdint.h>
#include <stdbool.h>


/******************************************************************************/
/*
    Application handle
    Feature module holds the pointer. ADC owns the state.
    handle requires at least one dereference, either P_ADC or P_CONVERSION_STATE
*/
/******************************************************************************/
/* Analog_VirtualChannel/Analog_ConversionHandler */
typedef const struct Analog_Conversion
{
    Analog_ADC_T * P_ADC;
    analog_channel_t CHANNEL;
    Analog_ConversionChannel_T * P_CONVERSION_CHANNEL;
    /* reserve interface for extension */
    // Analog_Options_T OPTIONS;
}
Analog_Conversion_T;

#define ANALOG_CONVERSION_INIT(p_AdcStruct, p_ConversionChannel) { .P_ADC = (p_AdcStruct), .CHANNEL = (p_ConversionChannel)->CHANNEL.ID, .P_CONVERSION_CHANNEL = (p_ConversionChannel), }
#define ANALOG_CONVERSION_INIT_FROM(AdcStruct, ChannelIndex) { .P_ADC = &AdcStruct, .CHANNEL = ChannelIndex, .P_CONVERSION_CHANNEL = &((AdcStruct).P_CONVERSION_CHANNELS[ChannelIndex]), }
// #define ANALOG_CONVERSION_INIT_FROM(AdcStruct, ChannelIndex) { .P_CONVERSION_STATE = &((AdcStruct).P_CONVERSION_STATES[ChannelIndex]), }
// #define ANALOG_CONVERSION_INIT_FROM(p_AdcStruct, ChannelIndex, p_ConversionChannels) ANALOG_CONVERSION_INIT(&(p_AdcStruct), &((p_ConversionChannels)[ChannelIndex]))

static inline void Analog_Conversion_Mark(const Analog_Conversion_T * p_conv) { Analog_ADC_MarkConversion(p_conv->P_ADC, p_conv->CHANNEL); }
static inline bool Analog_Conversion_IsMarked(const Analog_Conversion_T * p_conv) { return Analog_ADC_IsMarked(p_conv->P_ADC, p_conv->CHANNEL); }

static inline adc_result_t Analog_Conversion_GetResult(const Analog_Conversion_T * p_conv) { return p_conv->P_CONVERSION_CHANNEL->P_CONVERSION_STATE->Result; }
static inline void Analog_Conversion_ClearResult(const Analog_Conversion_T * p_conv) { p_conv->P_CONVERSION_CHANNEL->P_CONVERSION_STATE->Result = 0U; }
// static inline adc_result_t Analog_Conversion_GetResult(const Analog_Conversion_T * p_conv) { return p_conv->P_CONVERSION_STATE->Result; }
// static inline void Analog_Conversion_ClearResult(const Analog_Conversion_T * p_conv) { p_conv->P_CONVERSION_STATE->Result = 0U; }


/******************************************************************************/
/*
    Unsynchronize Activation
*/
/******************************************************************************/
// static void Analog_ActivateConversion(const Analog_Conversion_T * p_conversion)
// {
//     // if (Analog_ADC_ReadIsActive(p_conversion->P_ADC) == false)
//     // {
//         _Analog_ADC_StartConversions(p_conversion->P_ADC, p_conversion->P_CONVERSION_CHANNEL, 1U);
//     // }
//     // else
//     // {
//     //     Analog_Conversion_Mark(p_conversion); // mask as adc fixed
//     // }
// }

// static void Analog_ActivateConversions(const Analog_Conversion_T * p_conversions, uint32_t markers)
// {
//     // if (Analog_ADC_ReadIsActive(p_conversion->P_ADC) == false)
//     // {
//     //     Analog_ADC_StartConversions(p_conversion->P_ADC, p_conversion->P_CONVERSION_CHANNEL, markers);
//     // }
//     // else
//     // {
//     //     Analog_Conversion_Mark(p_conversion); // mark each
//     // }
// }


/******************************************************************************/
/*
    Export
*/
/******************************************************************************/
// #include "Analog_ADC.h"
// #include "Analog_Conversion.h"


/******************************************************************************/
/*
    Conversion Batch
    interface across multiple ADCs.
    Synchronized start with 1 callback, seperate state buffer
*/
/******************************************************************************/
// typedef const struct Analog_Batch
// {
    // Analog_Conversion_T * P_CONVERSIONS;          // [0,1,2,3] => [adc_channel_1, adc_channel_9, adc_channel_3]
    // uint8_t COUNT;                                // Number of conversions in the batch
    // Collective mask or
//     Analog_BatchPart_T * P_BATCH_PARTS;  // per map channel per  adc
//     uint8_t ADC_COUNT;
//
// }
// Analog_Batch_T;

// #define ANALOG_CONVERSION_BATCH_ALLOC(p_Channels, Count, p_Context, Callback) \
//     ANALOG_CONVERSION_BATCH_INIT(p_Channels, Count, p_Context, Callback, (Analog_ConversionState_T[Count]){})


// static void _Analog_MarkBatch(const Analog_Conversion_T * p_conversions, uint32_t markers, Analog_BatchContext_T * p_batch_context)
// {

//     //  Analog_Conversion_Mark(p_conversion); // mark each
// //    for (uint32_t i = 0; i < 32; i++)
// //    {
// //        if (markers & (1U << i))
// //        {
// //            Analog_Conversion_Mark(&p_conversions[i]);
// //        }
// //    }
//     uint32_t BATCH_MATCH; /* Bitmask of channels to match for completion */
//     volatile uint32_t * P_BATCH_STATE;
//     Analog_Callback_T ON_COMPLETE;
//     void * P_CONTEXT;
// }


/* by adc state */
// static inline adc_result_t _Analog_Batch_Result_ADC(const Analog_Batch_T * p_batch, uint8_t batchIndex) { return _Analog_Channel_GetResult(&p_batch->P_CHANNELS[batchIndex]); }

// static inline bool _Analog_Batch_IsComplete_ADC(const Analog_Batch_T * p_batch)
// void _Analog_Batch_IsComplete(uint32_t * p_completed, uint8_t count)
// {
//     for (uint8_t index = 0U; index < count; index++)
//     {
//         if ((p_completed[index] & 0xFFFFFFFFUL/* or batchMMask */) != 0UL) { return false; } // check if any channel is marked
//     }
//     return true;
// }

// static inline bool Analog_Batch_IsComplete(const Analog_Batch_T * p_batch)
// {
//
// }

// static   void AddActiveConversion(Analog_ADC_State_T * p_state, const Analog_ConversionChannel_T * p_handle)
// {

// }

// void _Analog_Batch_ActivateConversions(const Analog_Batch_T * p_batch)
// {

// }


// typedef struct Analog_Options
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
// Analog_Options_T;


