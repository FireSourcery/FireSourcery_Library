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
/*
    HAl + Analog_ConversionChannel_T contains entire state
*/
static inline void _ADC_CaptureResult(Analog_ConversionChannel_T * p_conversion, adc_result_t result)
{
    /* eliminate double buffer, at additional interrupt time */
    if (p_conversion->CAPTURE != NULL) { p_conversion->CAPTURE(p_conversion->P_CONTEXT, result); }
    else { p_conversion->P_CONVERSION_STATE->State = result; }
}

// static inline void _ADC_CaptureConversion(const HAL_ADC_T * p_hal, Analog_ConversionChannel_T * p_conversion)
// {
//     _ADC_CaptureResult(p_conversion, HAL_ADC_ReadResult(p_hal, p_conversion->CHANNEL.PIN));
// }

static inline void _ADC_CaptureTo(const HAL_ADC_T * p_hal, Analog_ConversionChannel_T * const * p_conversions, uint8_t count)
{
    if (count == HAL_ADC_ReadFifoCount(p_hal))
    {
        for (uint8_t index = 0U; index < count; index++) /* Read in the same way it was pushed */
        {
            _ADC_CaptureResult(p_conversions[index], HAL_ADC_ReadResult(p_hal, p_conversions[index]->CHANNEL.PIN));
        }
    }
}

static void _ADC_ActivateFrom(HAL_ADC_T * p_hal, Analog_ConversionChannel_T * const * p_conversions, uint8_t count)
{
    adc_pin_t pins[ADC_FIFO_LENGTH_MAX]; /* This should optimize away. */
    for (uint8_t index = 0U; index < count; index++) { pins[index] = p_conversions[index]->CHANNEL.PIN; }
    HAL_ADC_ActivateEach(p_hal, pins, count);
}

/*
    Unified Interface implementation independent
*/
static inline void ADC_Capture(Analog_ADC_T * p_adc, const Analog_ADC_State_T * p_state)
{
#ifdef ANALOG_ADC_HW_FIFO_ENABLE
    _ADC_CaptureTo(p_adc->P_HAL_ADC, &p_state->ActiveConversions[0U], p_state->ActiveConversionCount);
#else
    _ADC_CaptureResult(p_state->ActiveConversions[0U], HAL_ADC_ReadResult(p_adc->P_HAL_ADC, p_state->ActiveConversions[0U]->CHANNEL.PIN));
#endif
    // _ADC_CaptureTo(p_adc->P_HAL_ADC, &p_state->ActiveConversions[0U], ADC_ReadActiveCount());
}

/*
    Activate and wait for return
*/
static inline void ADC_Activate(Analog_ADC_T * p_adc, const Analog_ADC_State_T * p_state)
{
#ifdef ANALOG_ADC_HW_FIFO_ENABLE
    _ADC_ActivateFrom(p_adc->P_HAL_ADC, &p_state->ActiveConversions[0U], p_state->ActiveConversionCount);
#else
    HAL_ADC_Activate(p_adc->P_HAL_ADC, p_state->ActiveConversions[0U]->CHANNEL.PIN);
#endif
    // _ADC_ActivateFrom(p_adc, &p_state->ActiveConversions[0U], ADC_ReadActiveCount());
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
    uint32_t markers = sourceMarkers;
    uint8_t count;

    for (count = 0U; (count < ADC_FIFO_LENGTH_MAX) && (markers != 0UL); count++)
    {
        p_state->ActiveConversions[count] = &p_source[__builtin_ctz(markers)];
        markers &= (markers - 1);
    }
    p_state->ActiveConversionCount = count;
    p_state->ChannelMarkers = markers; /* Update ChannelMarkers with remaining markers */

    return markers ^ sourceMarkers; /* return processed markers */
}


static void ADC_StartFrom(Analog_ADC_T * p_adc, Analog_ConversionChannel_T * p_conversions, uint32_t markers)
{
    ADC_SetStateFrom(p_adc->P_ADC_STATE, p_conversions, markers);
    // p_state->ChannelMarkers =
    ADC_Activate(p_adc, p_adc->P_ADC_STATE);
}


/******************************************************************************/
/*!
    Unsynchronized Activation
    start a conversion immediately cancels ongoing conversions
    single threaded or atomic flag test and set
    ( Analog_ConversionChannel_T,  uint32_t markers) interface
     select from mapped or parameters
*/
/******************************************************************************/
static void _Analog_ADC_StartConversions(Analog_ADC_T * p_adc, Analog_ConversionChannel_T * p_conversions, uint32_t markers)
{
    if (Analog_ADC_ReadIsActive(p_adc) == false) { ADC_StartFrom(p_adc, p_conversions, markers); }
    else { Analog_ADC_MarkAll(p_adc, markers); }
}

static void _Analog_ADC_StartConversion(Analog_ADC_T * p_adc, Analog_ConversionChannel_T * p_conversion)
{
    _Analog_ADC_StartConversions(p_adc, p_conversion, (1UL << p_conversion->CHANNEL.ID)); // mask as adc fixed
}

// static void _Analog_ADC_StartConversionBatch(Analog_ADC_T * p_adc, Analog_ConversionBatch_T * p_batch)
// {
//     // _Analog_ADC_StartConversions(p_adc->P_ADC_STATE, p_batch->P_CONVERSION_CHANNELS, p_batch->CHANNELS);
//     _Analog_ADC_StartConversions(p_adc->P_ADC_STATE, p_adc->P_CONVERSION_CHANNELS, p_batch->CHANNELS);
//     p_adc->P_ADC_STATE->Callback = p_batch->P_CONTEXT;
// }



/******************************************************************************/
/*!
*/
/******************************************************************************/
// static inline uint8_t ADC_ReadActiveCount(const Analog_ADC_T * p_adc, const Analog_ADC_State_T * p_state)
// {
// #ifdef ANALOG_ADC_HW_FIFO_ENABLE
//     return HAL_ADC_ReadFifoCount(p_adc->P_HAL_ADC);
// #else
//     return 1U;
// #endif
// }

// void ADC_WriteOptions(Analog_ADC_T * p_adc, const Analog_Options_T * p_options)
// {
//     // xor with previous
//     if(p_options->FLAGS.HwTriggerConversion == 1U)      { HAL_ADC_EnableHwTrigger(p_adc->CONST.P_HAL_ADC); }
//     else                                                { HAL_ADC_DisableHwTrigger(p_adc->CONST.P_HAL_ADC); }
// #ifdef ANALOG_HW_CONTINOUS_CONVERSION_ENABLE
//     if(p_options->FLAGS.ContinuousConversion == 1U)     { HAL_ADC_EnableContinuousConversion(p_adc->CONST.P_HAL_ADC): }
//     else                                                { HAL_ADC_DisableContinuousConversion(p_adc->CONST.P_HAL_ADC); }
// #endif
//     if(p_options->ON_OPTIONS != 0U) { p_options->ON_OPTIONS(p_options->P_CALLBACK_CONTEXT); }
// }


// static inline uint32_t _ADC_FillActiveConversions(const Analog_ConversionChannel_T ** pp_buffer, uint8_t * p_count, const Analog_ConversionChannel_T * p_handles, uint32_t markers)
// {
//     for (uint8_t count = 0U; (count < ADC_FIFO_LENGTH_MAX) && (markers != 0UL); count++)
//     {
//         pp_buffer[count] = &p_handles[__builtin_ctz(markers)];
//         markers &= (markers - 1);
//     }
//     *p_count = count;
// }





