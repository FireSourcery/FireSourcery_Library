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
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Analog_ADC.h"
#include "Analog.h"

/******************************************************************************/
/*!
    Private
*/
/******************************************************************************/
static inline void _ADC_CaptureResult(const Analog_ConversionChannel_T * p_conversion, adc_result_t result)
{
    /* eliminate double buffer, at additional interrupt time */
    if (p_conversion->CAPTURE != NULL) { p_conversion->CAPTURE(p_conversion->P_CONTEXT, result); }
    else { p_conversion->P_CONVERSION_STATE->State = result; }
}

static inline void ADC_CaptureConversion(const Analog_ADC_T * p_adc, const Analog_ConversionChannel_T * p_conversion)
{
    _ADC_CaptureResult(p_conversion, HAL_ADC_ReadResult(p_adc->P_HAL_ADC, p_conversion->CHANNEL.PIN));
}

/*
    p_conversion->P_CONVERSION_STATE->State = HAL_ADC_ReadResult(p_hal, p_conversion->CHANNEL.PIN);
*/
static inline void ADC_CaptureSingle(const Analog_ADC_T * p_adc, const Analog_ADC_State_T * p_state)
{
    ADC_CaptureConversion(p_adc, p_state->ActiveConversions[0U]);
}

static inline void ADC_ActivateSingle(const Analog_ADC_T * p_adc, const Analog_ADC_State_T * p_state)
{
    HAL_ADC_Activate(p_adc->P_HAL_ADC, p_state->ActiveConversions[0U]->CHANNEL.PIN);
}

/*
    FIFO Version
*/
static inline void ADC_CaptureFifo(const Analog_ADC_T * p_adc, const Analog_ADC_State_T * p_state)
{
    if (p_state->ActiveConversionCount == HAL_ADC_ReadFifoCount(p_adc->P_HAL_ADC))
    {
        /* Read in the same way it was pushed */
        for (uint8_t index = 0U; index < p_state->ActiveConversionCount; index++)
        {
            ADC_CaptureConversion(p_adc, p_state->ActiveConversions[index]);
        }
    }
    /* HAL_ADC_Deactivate clears the FIFO, handle otherwise */
    // else
    // {

    // }
}

/*
*/
static inline void ADC_ActivateFifo(const Analog_ADC_T * p_adc, const Analog_ADC_State_T * p_state)
{
    /* This should optimize away. */
    adc_pin_t pins[ADC_FIFO_LENGTH_MAX];
    for (uint8_t index = 0U; index < p_state->ActiveConversionCount; index++)
    {
        pins[index] = p_state->ActiveConversions[index]->CHANNEL.PIN;
    }

    HAL_ADC_WriteFifoCount(p_adc->P_HAL_ADC, p_state->ActiveConversionCount);
    HAL_ADC_WriteFifo_ActivateOnLast(p_adc->P_HAL_ADC, pins, p_state->ActiveConversionCount);
}

/*
*/
static inline void ADC_Capture(const Analog_ADC_T * p_adc, const Analog_ADC_State_T * p_state)
{
    // assert(p_state->ActiveConversionCount > 0U);
#ifdef ANALOG_ADC_HW_FIFO_ENABLE
    ADC_CaptureFifo(p_adc, p_state);
#else
    ADC_CaptureSingle(p_adc, p_state);
#endif
}

/*
    Activate and wait for return
*/
static inline void ADC_Activate(const Analog_ADC_T * p_adc, const Analog_ADC_State_T * p_state)
{
    // assert(p_state->ActiveConversionCount > 0U);
#ifdef ANALOG_ADC_HW_FIFO_ENABLE
    ADC_ActivateFifo(p_adc, p_state);
#else
    ADC_ActivateSingle(p_adc, p_state);
#endif
}

/******************************************************************************/
/*

*/
/******************************************************************************/
/*
    Set [ActiveConversions] to reflect fifo registers state
    Markers corresponding to this particular list of channels.
    if p_source = p_adc->P_CONVERSION_CHANNELS => p_source[index].CHANNEL.ID = index
*/
static inline uint32_t ADC_SetStateFrom(Analog_ADC_State_T * p_state, const Analog_ConversionChannel_T * p_source, uint32_t sourceMarkers)
{
    uint8_t count;
    uint32_t markers = sourceMarkers;

    for (count = 0U; (count < ADC_FIFO_LENGTH_MAX) && (markers != 0UL); count++)
    {
        p_state->ActiveConversions[count] = &p_source[__builtin_ctz(markers)];
        markers &= (markers - 1);
    }

    p_state->ActiveConversionCount = count;
    p_state->ChannelMarkers = markers; /* Update ChannelMarkers with remaining markers */

    return markers ^ sourceMarkers; /* return processed markers */
}


static void ADC_StartConversions(const Analog_ADC_T * p_adc, Analog_ConversionChannel_T * p_conversions, uint32_t markers)
{
    ADC_SetStateFrom(p_adc->P_ADC_STATE, p_conversions, markers);
    ADC_Activate(p_adc, p_adc->P_ADC_STATE);
}

// switchs context
// static void ADC_StartConversionsFrom(const Analog_ADC_T * p_adc, Analog_ConversionChannel_T * p_conversions, uint32_t markers)
// {
//     p_adc->P_ADC_STATE->p_ConversionChannels = p_conversions;
//     ADC_SetStateFrom(p_adc->P_ADC_STATE, p_conversions, markers);
//     ADC_Activate(p_adc, p_adc->P_ADC_STATE);
// }

/******************************************************************************/
/*
    Handle Threads
*/
/******************************************************************************/
/*
    ActivateMarked
    Update State and Start

    ADC_SetStateInternal
*/
static void ADC_ProcStart(const Analog_ADC_T * p_adc, Analog_ADC_State_T * p_state)
{
    ADC_SetStateFrom(p_state, &p_adc->P_CONVERSION_CHANNELS[0U], p_state->ChannelMarkers);
    ADC_Activate(p_adc, p_state);
    // ADC_StartConversions(p_adc, &p_adc->P_CONVERSION_CHANNELS[0U], p_state->ChannelMarkers);
    // p_state->ChannelMarkers &= ~processed;
}

// from settable pointer
// static void ADC_ProcStartFlex(const Analog_ADC_T * p_adc, Analog_ADC_State_T * p_state)
// {
// p_adc->P_ADC_STATE->p_ConversionChannels = p_conversions;
//     ADC_StartConversions(p_state, &p_adc->P_ADC_STATE->p_ConversionChannels, p_state->ChannelMarkers);
// }

static inline void ADC_OnComplete(const Analog_ADC_T * p_adc, Analog_ADC_State_T * p_state)
{
    HAL_ADC_ClearConversionCompleteFlag(p_adc->P_HAL_ADC);

    if (p_state->ActiveConversionCount > 0U)
    {
        ADC_Capture(p_adc, p_state);
        p_state->ActiveConversionCount = 0U;

        // if (p_state->p_Batch  != NULL)
        // {
         //     ADC_StartConversionsFrom(p_state, p_state->MarkedChannel, p_state->ChannelMarkers); //channel marker maped per adc
         // ADC_ProcStartBatch
        //     // Analog_Batch_OnComplete(p_adc->p_Batch );
        // }

        /* Continue incrementing. Channels do not repeat until all marked channels have completed once */
        if (p_state->ChannelMarkers != 0UL) { ADC_ProcStart(p_adc, p_state); }
        else { HAL_ADC_Deactivate(p_adc->P_HAL_ADC); } /* alternatively flag for successful conversions */

        // optionally proc on complete in parallel, after ADC_Activate
        // if (p_conversion->ON_COMPLETE != NULL) { p_conversion->ON_COMPLETE(p_conversion->P_CONTEXT); }
    }
#ifndef NDEBUG
    else
    {
        HAL_ADC_Deactivate(p_adc->P_HAL_ADC);
        p_state->ErrorCount++;
    }
#endif
}




/******************************************************************************/
/*!

*/
/******************************************************************************/
// need 2 returns
// static inline uint32_t _ADC_FillActiveConversions1(const Analog_ConversionChannel_T ** pp_buffer, uint8_t * p_count, const Analog_ConversionChannel_T * p_handles, uint32_t markers)
// {
//     for (uint8_t count = 0U; (count < ADC_FIFO_LENGTH_MAX) && (markers != 0UL); count++)
//     {
//         pp_buffer[count] = &p_handles[__builtin_ctz(markers)];
//         markers &= (markers - 1);
//     }
//     *p_count = count;
// }




// void ADC_WriteOptions(Analog_ADC_T * p_adc, const Analog_Options_T * p_options)
// {
//     // xor with previous
//     if(p_options->FLAGS.HwTriggerConversion == 1U)      { HAL_ADC_EnableHwTrigger(p_adc->CONST.P_HAL_ADC); }
//     else                                                { HAL_ADC_DisableHwTrigger(p_adc->CONST.P_HAL_ADC); }
// #ifdef CONFIG_ANALOG_HW_CONTINOUS_CONVERSION_ENABLE
//     if(p_options->FLAGS.ContinuousConversion == 1U)     { HAL_ADC_EnableContinuousConversion(p_adc->CONST.P_HAL_ADC): }
//     else                                                { HAL_ADC_DisableContinuousConversion(p_adc->CONST.P_HAL_ADC); }
// #endif
//     if(p_options->ON_OPTIONS != 0U) { p_options->ON_OPTIONS(p_options->P_CALLBACK_CONTEXT); }
// }

/*
using index
*/
/* For conversion list  configured at compile time as a global list for all adcs */
// static inline uint8_t FillActiveConversions(Analog_ADC_State_T * p_state, const volatile Analog_ConversionState_T * p_markers, const Analog_ConversionChannel_T * const * pp_handles, uint8_t length)
// {
//     uint8_t indexCount = (length < ADC_FIFO_LENGTH_MAX) ? length : ADC_FIFO_LENGTH_MAX;

//     for (uint8_t index = 0U; index < indexCount; index++)
//     {
//         if ((pp_handles[index] != NULL) && (p_markers[index].IsMarked == true))
//         {
//             if (pp_handles[index]->CHANNEL.P_ADC == p_state->ActiveConversions[index]->CHANNEL.P_ADC)
//             {
//                 p_state->ActiveConversions[activeConversionCount] = pp_handles[index];
//                 p_state->ActiveConversionCount += 1U; /* counts added, accounts for nulls */
//             }
//         }
//     }
//     return indexCount;
// }




// static inline uint8_t _ADC_FillActiveConversions(const Analog_ConversionChannel_T ** pp_buffer, uint8_t count, const Analog_ConversionChannel_T * p_handles, const volatile Analog_ConversionState_T * p_markers)
// {
//     uint8_t activeConversionCount = 0;

//     for (uint8_t index = 0U; index < count; index++)
//     {
//         if (p_markers[index].IsMarked == true) /* Find the next marked channel */
//         {
//             pp_buffer[activeConversionCount] = &p_handles[index];
//             activeConversionCount += 1U;
//         }
//     }

//     return activeConversionCount;
// }

// /*
//     indexCount == ActiveConversionCount, if there are no NULL entreis in the table
// */
// static inline uint8_t ADC_SetStateFrom(Analog_ADC_State_T * p_state, const Analog_ConversionChannel_T * p_source, const volatile Analog_ConversionState_T * p_markers, uint8_t length)
// {
//     uint8_t indexCount = (length < ADC_FIFO_LENGTH_MAX) ? length : ADC_FIFO_LENGTH_MAX;
//     p_state->ActiveConversionCount = _ADC_FillActiveConversions(&p_state->ActiveConversions[0U], indexCount, p_source, p_markers);
//     return indexCount;
// }


// /*
//     Update buffer with marked
//     relative to const config
//     p_state->ActiveConversions[0]
//     p_markers[index]
//     pp_convHandles[index]
//     length as p_adc->CHANNEL_COUNT = index
// */
// static inline uint8_t _ADC_SetActiveState(const Analog_ADC_T * p_adc, Analog_ADC_State_T * p_state, analog_channel_t index)
// {
//     return ADC_SetStateFrom(p_state, &p_adc->P_CONVERSION_CHANNELS[index], &p_adc->P_CONVERSION_STATES[index], p_adc->CHANNEL_COUNT - index);
// }

// /*
//     Effectively sets State for OnComplete
//     Write Critical Section Buffer.
//     single threaded access or lock
//     In a single thread while ADC is inactive
//     In the ADC ISR
// */
// static inline void ADC_SetActiveState(const Analog_ADC_T * p_adc, Analog_ADC_State_T * p_state)
// {
//     p_state->ChannelIndex += _ADC_SetActiveState(p_adc, p_state, p_state->ChannelIndex);
// }


// /******************************************************************************/
// /*
//     Handle Threads
// */
// /******************************************************************************/

// static inline void ADC_OnComplete(const Analog_ADC_T * p_adc, Analog_ADC_State_T * p_state)
// {
//     HAL_ADC_ClearConversionCompleteFlag(p_adc->P_HAL_ADC);

//     if (p_state->ActiveConversionCount > 0U)
//     {
//         ADC_Capture(p_adc, p_state);
//         p_state->ActiveConversionCount = 0U;

//         /* Continue incrementing. Channels do not repeat until all marked channels have completed once */
//         if (p_state->ChannelIndex < p_adc->CHANNEL_COUNT)
//         {
//             ADC_ProcStart(p_adc, p_state);
//         }
//         else
//         {
//             /* Only reset to 0U if all channels have been processed. Otherwise continue from Index. */
//             p_state->ChannelIndex = 0U; /* Reset to 0U */
//             HAL_ADC_Deactivate(p_adc->P_HAL_ADC);
//         }

//         // optionally proc on complete in parallel, after ADC_Activate
//         // if (p_conversion->ON_COMPLETE != NULL) { p_conversion->ON_COMPLETE(p_conversion->P_CONTEXT); }
//     }
// #ifndef NDEBUG
//     else
//     {
//         HAL_ADC_Deactivate(p_adc->P_HAL_ADC);
//         p_state->ErrorCount++;
//     }
// #endif
// }
