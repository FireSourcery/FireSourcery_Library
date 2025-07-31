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

/*

*/
static inline void ADC_CaptureConversion(const Analog_ADC_T * p_adc, const Analog_ConversionChannel_T * p_conversion)
{
    _ADC_CaptureResult(p_conversion, HAL_ADC_ReadResult(p_adc->P_HAL_ADC, p_conversion->CHANNEL.PIN));
}

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
    //     // p_state->FifoMismatch++;
    //     // HAL_ADC_Deactivate(p_adc->P_HAL_ADC);
    // }
}
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
    // HAL_ADC_ActivateEach(p_adc->P_HAL_ADC, pins, p_state->ActiveConversionCount);
}

/*
    On HAL and Analog_ADC_State_T
*/
static inline void ADC_Capture(const Analog_ADC_T * p_adc, const Analog_ADC_State_T * p_state)
{
    // assert(p_state->ActiveConversionCount > 0U);
#ifdef ANALOG_ADC_HW_FIFO_ENABLE
    ADC_CaptureFifo(p_adc, p_state);
#else
    ADC_CaptureSingle(p_adc, p_state);
#endif
// #ifdef ANALOG_ADC_HW_FIFO_ENABLE
//     // ADC_CaptureTo(p_adc->P_HAL_ADC, p_state->ActiveConversions, p_state->ActiveConversionCount);
// #else
//     // ADC_CaptureTo(p_adc->P_HAL_ADC, p_state->ActiveConversions, 1U);
    // _ADC_CaptureResult(p_conversion, HAL_ADC_ReadResult(p_hal, p_conversion->CHANNEL.PIN));
// #endif
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
// #ifdef ANALOG_ADC_HW_FIFO_ENABLE
//     ADC_ActivateFrom(p_adc, p_state->ActiveConversions, p_state->ActiveConversionCount);
// #else
//     ADC_ActivateFrom(p_adc, &p_state->ActiveConversions[0U], 1U);
    // HAL_ADC_Activate(p_adc->P_HAL_ADC, p_state->ActiveConversions[0U]->CHANNEL.PIN);
// #endif
}

/******************************************************************************/
/*!
    With outer context
    move to Analog_ADC.h
*/
/******************************************************************************/

/*
    Set [ActiveConversions] to reflect fifo registers state
    Markers corresponding to this particular list of channels.
    if p_source = p_adc->P_CONVERSION_CHANNELS => p_source[index].CHANNEL.ID = index
*/
/*
    Effectively sets State for OnComplete
    Write Critical Section Buffer.
    single threaded access or lock
    In a single thread while ADC is inactive
    In the ADC ISR
*/
static inline uint32_t ADC_SetStateFrom(Analog_ADC_State_T * p_state, Analog_ConversionChannel_T * p_source, uint32_t sourceMarkers)
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


static void ADC_StartFrom(const Analog_ADC_T * p_adc, Analog_ConversionChannel_T * p_conversions, uint32_t markers)
{
    if (markers != 0U)
    {
        ADC_SetStateFrom(p_adc->P_ADC_STATE, p_conversions, markers);
        // p_state->ChannelMarkers =
        ADC_Activate(p_adc, p_adc->P_ADC_STATE);
    }
}

// static void ADC_CaptureTo(const Analog_ADC_T * p_adc, Analog_ConversionChannel_T * p_conversions, uint32_t markers)
// {
    // ADC_Capture(p_adc, p_state);
    // p_state->p_context->ChannelComplete |=  ~p_state->p_context->ChannelMarkers ;
// }

// static void ADC_CaptureContext(const Analog_ADC_T * p_adc, Analog_ConversionContext_T * p_context)
// {
//     ADC_Capture(p_adc, p_adc->P_ADC_STATE);
//     p_context->P_COMPLETE_MARKERS |= ~p_adc->P_ADC_STATE->ChannelMarkers;
// }

// static void ADC_StartContext(const Analog_ADC_T * p_adc, Analog_ConversionContext_T * p_context)
// {
//     ADC_StartFrom(p_adc, p_context->P_CONVERSION_CHANNELS, p_context->CHANNELS);
//     p_adc->P_ADC_STATE->ChannelMarkers = p_context->CHANNELS;
// }


/******************************************************************************/
/*!
    Unsynchronized Activation
    start a conversion immediately cancels ongoing conversions
    single threaded or atomic flag test and set
    ( Analog_ConversionChannel_T,  uint32_t markers) interface
     select from mapped or parameters
*/
/******************************************************************************/
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

// void Analog_ADC_StartConversionBatch(const Analog_ADC_T * p_adc, const Analog_ConversionBatch_T * p_batch)
// {

// }



/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
    HAl version
*/
// static inline void _ADC_CaptureResult(Analog_ConversionChannel_T * p_conversion, adc_result_t result)
// {
//     /* eliminate double buffer, at additional interrupt time */
//     if (p_conversion->CAPTURE != NULL) { p_conversion->CAPTURE(p_conversion->P_CONTEXT, result); }
//     else { p_conversion->P_CONVERSION_STATE->State = result; }
// }

// static inline void ADC_CaptureConversion(const HAL_ADC_T * p_hal, const Analog_ConversionChannel_T * p_conversion)
// {
//     _ADC_CaptureResult(p_conversion, HAL_ADC_ReadResult(p_hal, p_conversion->CHANNEL.PIN));
// }

// static inline void ADC_CaptureTo(const HAL_ADC_T * p_hal, Analog_ConversionChannel_T * p_conversions, uint8_t count)
// {
//     for (uint8_t index = 0U; index < count; index++)
//     {
//         _ADC_CaptureResult(&p_conversions[index], HAL_ADC_ReadResult(p_hal, p_conversions[index].CHANNEL.PIN));
//     }
// }

// static void ADC_ActivateFrom(HAL_ADC_T * p_hal, Analog_ConversionChannel_T * p_conversions, uint8_t count)
// {
//     adc_pin_t pins[ADC_FIFO_LENGTH_MAX]; /* This should optimize away. */
//     for (uint8_t index = 0U; index < count; index++) { pins[index] = p_conversions[index].CHANNEL.PIN; }
//     HAL_ADC_ActivateEach(p_hal, pins, count);
// }




// static inline uint8_t ADC_ReadActiveCount(const Analog_ADC_T * p_adc, const Analog_ADC_State_T * p_state)
// {
// #ifdef ANALOG_ADC_HW_FIFO_ENABLE
//     return HAL_ADC_ReadFifoCount(p_adc->P_HAL_ADC);
// #else
//     return 1;
// #endif
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


// need 2 returns
/* For conversion list  configured at compile time as a global list for all adcs */
// (pp_handles[index]->CHANNEL.P_ADC == p_state->ActiveConversions[index]->CHANNEL.P_ADC)
// static inline uint32_t _ADC_FillActiveConversions(const Analog_ConversionChannel_T ** pp_buffer, uint8_t * p_count, const Analog_ConversionChannel_T * p_handles, uint32_t markers)
// {
//     for (uint8_t count = 0U; (count < ADC_FIFO_LENGTH_MAX) && (markers != 0UL); count++)
//     {
//         pp_buffer[count] = &p_handles[__builtin_ctz(markers)];
//         markers &= (markers - 1);
//     }
//     *p_count = count;
// }





