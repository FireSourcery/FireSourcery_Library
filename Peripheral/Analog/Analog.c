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
    @file   Analog.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "Analog.h"



// start a conversion immediately
// single threaded or atomic flag test and set
// static inline void _Analog_Channel_ActivateConversion(const Analog_ConversionChannel_T * p_conversion)
// {
//     if (_Analog_ADC_ReadIsActive(p_conversion->p) == false)
//     {

//     }
//     else
//     {
//         Analog_MarkConversion(p_conversion);
//     }
// }

// void Analog_Batch_MarkConversions(const Analog_ConversionBatch_T * p_batch)
// {
//     p_analog->CONST.P_BATCH_ENTRIES[p_batch->BATCH].IsMarked = true;
// }

// static inline void AddActiveConversion(Analog_ADC_State_T * p_state, const Analog_ConversionChannel_T * p_handle)
// {
//     if (p_state->ActiveConversionCount < ADC_FIFO_LENGTH_MAX)
//     {
//         p_state->ActiveConversions[p_state->ActiveConversionCount] = p_handle;
//         p_state->ActiveConversionCount += 1U;
//     }
// }

// void _Analog_Batch_ActivateConversions(const Analog_ConversionBatch_T * p_batch)
// {
//     // foreach adc
//     // for (uint8_t index = 0U; index < p_batch->ADC; index++)
//     // {
//     //     Analog_ADC_T * p_adc = p_batch->P_ADCS[index];
//     //     // return FillActiveConversions(p_adc->P_ADC_STATE, &p_adc->P_BATCH[p_batch->BATCH_ID], &p_adc->P_BATCH[p_batch->BATCH_ID],  );
//     //     // p_state->ChannelIndex += ADC_FillActiveConversions(p_adc, p_state, p_state->ChannelIndex);
//     //     ADC_Start(p_adc);
//     // }
//     // if (_Analog_ADC_ReadIsActive(p_batch->p) == false)
//     // {
//     // }
/// block each adc while iterating
//     for (uint8_t index = 0U; index < p_batch->COUNT; index++)
//     {
//         return AddActiveConversion(p_adc->P_ADC_STATE, &p_adc->P_BATCH[p_batch->BATCH_ID] );
//         // p_state->ChannelIndex = ;
//     }
//     ADC_Start(p_adc);
// }




// static inline void _Analog_CaptureLocalPeak(Analog_T * p_analog, adc_result_t result)
// adc_result_t result = HAL_ADC_ReadResult(p_analog->CONST.P_HAL_ADC, p_activeConversion->PIN);
// if(p_analog->ActiveOptions.CaptureLocalPeak == true)
// {
//     if(result > p_activeConversion->P_RESULTS_BUFFER[p_activeConversion->CHANNEL])
//     {
//         p_activeConversion->P_RESULTS_BUFFER[p_activeConversion->CHANNEL] = result;
//     }
//     else
//     {
//         p_analog->IsLocalPeakFound[p_activeConversion->CHANNEL] = true;
//     }
// }
