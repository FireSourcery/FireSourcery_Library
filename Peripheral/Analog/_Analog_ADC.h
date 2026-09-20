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

// static inline void _ADC_CaptureTo(const HAL_ADC_T * p_hal,Analog_ConversionChannel_T *  p_pins, Analog_ConversionState_T   * p_state, Analog_Callbacks_T * p_callbacks,  uint8_t count)
static inline void _ADC_CaptureTo(const HAL_ADC_T * p_hal, Analog_ConversionChannel_T * const * p_conversions, uint8_t count)
{
    if (count == HAL_ADC_ReadFifoCount(p_hal))
    {
        for (uint8_t index = 0U; index < count; index++) /* Read in the same way it was pushed */
        {
            _ADC_CaptureResult(p_conversions[index], HAL_ADC_ReadResult(p_hal, p_conversions[index]->PIN));
        }
    }
}

static void _ADC_ActivateFrom(HAL_ADC_T * p_hal, Analog_ConversionChannel_T * const * p_conversions, uint8_t count)
{
    adc_pin_t pins[ADC_FIFO_LENGTH_MAX]; /* This should optimize away. */
    for (uint8_t index = 0U; index < count; index++) { pins[index] = p_conversions[index]->PIN; }
    HAL_ADC_ActivateEach(p_hal, pins, count);
}

/*
    Unified Interface implementation independent
*/
static inline void _ADC_Capture(const HAL_ADC_T * p_hal, const Analog_ADC_State_T * p_state)
{
#ifdef ANALOG_ADC_HW_FIFO_ENABLE
    _ADC_CaptureTo(p_hal, &p_state->ActiveConversions[0U], p_state->ActiveConversionCount);
#else
    _ADC_CaptureResult(p_state->ActiveConversions[0U], HAL_ADC_ReadResult(p_hal, p_state->ActiveConversions[0U]->PIN));
#endif
    // _ADC_CaptureTo(p_adc->P_HAL_ADC, &p_state->ActiveConversions[0U], ADC_ReadActiveCount());
}


static inline void ADC_Capture(Analog_ADC_T * p_adc, const Analog_ADC_State_T * p_state)
{
#ifdef ANALOG_ADC_HW_FIFO_ENABLE
    _ADC_CaptureTo(p_adc->P_HAL_ADC, &p_state->ActiveConversions[0U], p_state->ActiveConversionCount);
#else
    _ADC_CaptureResult(p_state->ActiveConversions[0U], HAL_ADC_ReadResult(p_adc->P_HAL_ADC, p_state->ActiveConversions[0U]->PIN));
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
    HAL_ADC_Activate(p_adc->P_HAL_ADC, p_state->ActiveConversions[0U]->PIN);
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
    if p_source = p_adc->P_CONVERSION_CHANNELS => p_source[index].ID = index
*/
/*
    Effectively sets State for OnComplete
    Write Critical Section Buffer.
    single threaded access or lock
    In a single thread while ADC is inactive
    In the ADC ISR

    p_state->ChannelMarkers writes in an ISR preempting this function are lost
*/
static inline uint32_t ADC_SetStateFrom(Analog_ADC_State_T * p_state, Analog_ConversionChannel_T * p_source, uint32_t sourceMarkers)
{
    uint32_t markers = sourceMarkers;
    uint8_t count = 0U;

    while ((count < ADC_FIFO_LENGTH_MAX) && (markers != 0UL))
    {
        p_state->ActiveConversions[count] = &p_source[__builtin_ctz(markers)];
        markers &= (markers - 1);
        count++;
    }

    p_state->ActiveConversionCount = count;
    p_state->ChannelMarkers = markers;  /* Update ChannelMarkers with remaining markers */

    return markers ^ sourceMarkers; /* return processed markers */
}


static void ADC_StartFrom(Analog_ADC_T * p_adc, Analog_ConversionChannel_T * p_conversions, uint32_t markers)
{
    ADC_SetStateFrom(p_adc->P_ADC_STATE, p_conversions, markers);
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
    _Analog_ADC_StartConversions(p_adc, p_conversion, (1UL << p_conversion->ID)); // mask as adc fixed
}

/*

*/


/******************************************************************************/
/*!
*/
/******************************************************************************/
static inline void _Analog_ADC_ActivateBatch_Dma(const Analog_ADC_T * p_adc, const ADC_ConversionBatch_T * p_batch)
{
    p_adc->P_ADC_STATE->p_ActiveBatch = p_batch;
    // HAL_ADC_ActivateBatch(p_adc->P_HAL_ADC, p_batch->CHANNELS);
    HAL_ADC_ActivateDmaBatch(p_adc->P_HAL_ADC, p_batch->ID_START, p_batch->COUNT);
}

static inline void _Analog_ADC_ActivateBatch_Each(const Analog_ADC_T * p_adc, const ADC_ConversionBatch_T * p_batch)
{
    p_adc->P_ADC_STATE->p_ActiveBatch = p_batch;
    _Analog_ADC_StartConversions(p_adc, &p_adc->P_CONVERSION_CHANNELS[0], p_batch->CHANNELS);
}

static inline void _Analog_ADC_ActivateBatch(const Analog_ADC_T * p_adc, const ADC_ConversionBatch_T * p_batch)
{
    assert(Analog_Mask_IsContiguous(p_batch->CHANNELS) && ((p_batch->CHANNELS >> p_adc->CHANNEL_COUNT) == 0U));
    _Analog_ADC_ActivateBatch_Dma(p_adc, p_batch);
}

/*
    Immediate. On init, or while the trigger source is inactive.
    Sets Next as well as Active. OnCompleteBatch applies Next, it must not be left unset.
*/
static inline void Analog_ADC_ActivateBatch(const Analog_ADC_T * p_adc, uint8_t batchId)
{
    p_adc->P_ADC_STATE->p_NextBatch = &p_adc->P_CONVERSION_BATCHS[batchId];
    _Analog_ADC_ActivateBatch(p_adc, &p_adc->P_CONVERSION_BATCHS[batchId]);
}

/******************************************************************************/
/*!
*/
/******************************************************************************/
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

