#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   _ADC.h
    @author FireSourcery
    @brief  Private. Fifo capture and activation.

    The channel list is ids. [P_CHANNELS] resolves the pin, and the id is the results index,
    so the Hw facing blocks take the pin table and the base never sees a handler.
*/
/******************************************************************************/
#include "ADC.h" /* Included from ADC.h after its typedefs. #pragma once resolves either order */


/******************************************************************************/
/*
    Capture. Software activation only. The Hw transfer fills the results buffer directly.
*/
/******************************************************************************/
/* 1 channel. The pin reads the Hw, the id addresses the buffer */
static inline void _ADC_CaptureChannel(const HAL_ADC_T * p_hal, adc_pin_t pin, volatile adc_result_t * p_dest)
{
    *p_dest = HAL_ADC_ReadResult(p_hal, pin);
}

/*
    Read in the same way it was pushed, store by [ID]. Returns the captured channels.
    A fifo mismatch captures nothing, and those channels convert again.
*/
static inline adc_mask_t _ADC_CaptureTo(const HAL_ADC_T * p_hal, const adc_pin_t * p_pins, const adc_channel_t * p_channels, uint8_t count, volatile adc_result_t * p_results)
{
    adc_mask_t captured = 0UL;
#if (ADC_FIFO_LENGTH_MAX > 1U)
    if (count == HAL_ADC_ReadFifoCount(p_hal))     /* The fifo must hold what was pushed, or the results map onto the wrong channels */
#endif
    {
        for (uint8_t index = 0U; index < count; index++)
        {
            _ADC_CaptureChannel(p_hal, p_pins[p_channels[index]], &p_results[p_channels[index]]);
            captured |= ADC_MaskOf(p_channels[index]);
        }
    }
    return captured;
}

static inline void _ADC_ActivateFrom(HAL_ADC_T * p_hal, const adc_pin_t * p_pins, const adc_channel_t * p_channels, uint8_t count)
{
    adc_pin_t pins[ADC_FIFO_LENGTH_MAX]; /* This should optimize away. */
    for (uint8_t index = 0U; index < count; index++) { pins[index] = p_pins[p_channels[index]]; }
    /* Nothing active is not a push. The callers gate on the markers, so this only holds under NDEBUG, where their assert is gone */
    if (count > 0U) { HAL_ADC_ActivateEach(p_hal, pins, count); }
}

/*
    The captured channels clear their markers and set their flags, so the 2 words hold the whole
    conversion state: what is left to convert, and what has landed unconsumed.
*/
static inline adc_mask_t _ADC_Capture(ADC_T * p_adc)
{
    return _ADC_CaptureTo(p_adc->P_HAL_ADC, p_adc->P_CHANNEL_PINS, &p_adc->P_STATE->ActiveChannels[0U], p_adc->P_STATE->ActiveChannelCount, p_adc->P_CHANNEL_RESULTS);
}

static inline void _ADC_Activate(ADC_T * p_adc)
{
    _ADC_ActivateFrom(p_adc->P_HAL_ADC, p_adc->P_CHANNEL_PINS, &p_adc->P_STATE->ActiveChannels[0U], p_adc->P_STATE->ActiveChannelCount);
}

/******************************************************************************/
/*
    Channel markers
*/
/******************************************************************************/
/*
    Set [ActiveChannels] to reflect the fifo registers, fifo depth at a time, in channel order.

    Write critical section buffer. In the ISR, or in a single thread while the ADC is inactive.
    Marker writes from an ISR preempting this function are lost.
*/
static inline adc_mask_t _ADC_SetStateFrom(ADC_State_T * p_state, adc_mask_t sourceMarkers)
{
    adc_mask_t markers = sourceMarkers;
    uint8_t count = 0U;

    while ((markers != 0UL) && (count < ADC_FIFO_LENGTH_MAX))
    {
        p_state->ActiveChannels[count] = (adc_channel_t)__builtin_ctz(markers);
        markers &= (markers - 1);
        count++;
    }

    p_state->ActiveChannelCount = count;

    return markers ^ sourceMarkers; /* return processed markers */
}
