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
    @file   Analog_Batch.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "Analog.h"


/******************************************************************************/
/*
    Conversion Batch
    interface across multiple ADCs.
    Synchronized start with 1 callback, seperate state buffer
*/
/******************************************************************************/
// typedef const struct Analog_BatchPart
// {
//     Analog_ADC_T * P_ADC;
//     const Analog_ConversionContext_T ADC_CONTEXT;
// }
// Analog_BatchPart_T;


// typedef const struct Analog_ConversionBatch
// {
//     // Analog_Conversion_T * P_CONVERSIONS;          // [0,1,2,3] => [adc_channel_1, adc_channel_9, adc_channel_3]
//     // uint8_t COUNT;                                // Number of conversions in the batch
//     Analog_BatchPart_T * P_BATCH_PARTS;  // per map channel per  adc
//     uint8_t ADC_COUNT;
// }
// Analog_ConversionBatch_T;



// #define ANALOG_CONVERSION_BATCH_ALLOC(p_Channels, Count, p_Context, Callback) \
//     ANALOG_CONVERSION_BATCH_INIT(p_Channels, Count, p_Context, Callback, (Analog_ConversionState_T[Count]){})

// void Analog_Batch_MarkConversions(const Analog_ConversionBatch_T * p_batch)
// {
//     for (uint8_t index = 0U; index < p_batch->ADC_COUNT; index++)
//     {
//         Analog_BatchPart_T * p_part = &p_batch->P_BATCH_PARTS[index];
//         Analog_ADC_T * p_adc = p_part->P_ADC;
//         Analog_ContextState_T * p_state = p_part->ADC_CONTEXT.P_STATE;

//         // mark all channels
//         p_state->ChannelMarkers |= p_part->ADC_CONTEXT.P_STATE->ChannelMarkers;
//         // p_state->CompleteMarkers |= p_part->ADC_CONTEXT.P_STATE->CompleteMarkers;

//         // mark all channels
//         for (uint8_t channelIndex = 0U; channelIndex < p_adc->CHANNEL_COUNT; channelIndex++)
//         {
//             if (p_adc->P_CONVERSION_CHANNELS[channelIndex].P_CONVERSION_STATE != NULL)
//             {
//                 p_adc->P_CONVERSION_CHANNELS[channelIndex].P_CONVERSION_STATE->IsMarked = true;
//             }
//         }
//     }
// }


/* by adc state */
// static inline adc_result_t _Analog_Batch_Result_ADC(const Analog_ConversionBatch_T * p_batch, uint8_t batchIndex) { return _Analog_Channel_GetResult(&p_batch->P_CHANNELS[batchIndex]); }

// static inline bool _Analog_Batch_IsComplete_ADC(const Analog_ConversionBatch_T * p_batch)
// {
//     for (uint8_t index = 0U; index < p_batch->COUNT; index++)
//     {
//         if (_Analog_Channel_IsMarked(&p_batch->P_CHANNELS[index]) == false) { return false; } marked channels clear on start
//     }
// }

// static inline bool Analog_Batch_IsComplete(const Analog_ConversionBatch_T * p_batch)
// {
//
// }



// static   void AddActiveConversion(Analog_ADC_State_T * p_state, const Analog_ConversionChannel_T * p_handle)
// {

// }

// void _Analog_Batch_ActivateConversions(const Analog_ConversionBatch_T * p_batch)
// {

// }

