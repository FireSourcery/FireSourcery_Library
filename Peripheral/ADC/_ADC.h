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
    @brief  Private. Capture and activation. Resolves Hw sequenced against software activation.
*/
/******************************************************************************/
#include "ADC.h"

/******************************************************************************/
/*
    Hw sequencer import.
    Route the trigger to [channelStart, channelStart + count). The Hw converts and transfers on each trigger.
    Board or platform provided, when ADC_HW_SEQUENCER_ENABLE is true.
*/
/******************************************************************************/
#if ADC_HW_SEQUENCER_ENABLE
static inline void HAL_ADC_ActivateSequence(HAL_ADC_T * p_hal, uint32_t channelStart, uint32_t count);
#endif

/******************************************************************************/
/*
    Capture. Software activation only. The Hw transfer fills the results buffer directly.
*/
/******************************************************************************/
/*
    The results buffer is the canonical store, so the channel's CAPTURE is in addition, not instead.
    A pull read then sees the same value the push delivered.
*/
static inline void _ADC_CaptureChannel(const HAL_ADC_T * p_hal, const ADC_Channel_T * p_channel, volatile adc_result_t * p_dest)
{
    adc_result_t result = HAL_ADC_ReadResult(p_hal, p_channel->PIN);

    *p_dest = result;
    if (p_channel->CAPTURE != NULL) { p_channel->CAPTURE(p_channel->P_CONTEXT, result); }
}

/*
    Read in the same way it was pushed, store by [ID]. Returns the captured channels.
*/
static inline adc_mask_t _ADC_CaptureTo(const HAL_ADC_T * p_hal, const ADC_Channel_T * const * p_channels, uint8_t count, volatile adc_result_t * p_results)
{
    adc_mask_t captured = 0UL;
#if (ADC_FIFO_LENGTH_MAX > 1U)
    if (count == HAL_ADC_ReadFifoCount(p_hal))     /* The fifo must hold what was pushed, or the results map onto the wrong channels */
#endif
    {
        for (uint8_t index = 0U; index < count; index++)
        {
            _ADC_CaptureChannel(p_hal, p_channels[index], &p_results[p_channels[index]->ID]);
            captured |= ADC_MaskOf(p_channels[index]->ID);
        }
    }
    return captured;
}

/*
    The captured channels clear their markers, so [ChannelMarkers] alone tracks what is left to convert.
    A fifo mismatch captures nothing, and those channels convert again.
*/
static inline adc_mask_t _ADC_Capture(const HAL_ADC_T * p_hal, const ADC_State_T * p_state, volatile adc_result_t * p_results)
{
    return _ADC_CaptureTo(p_hal, &p_state->ActiveChannels[0U], p_state->ActiveChannelCount, p_results);
}


static inline void _ADC_ActivateFrom(HAL_ADC_T * p_hal, ADC_Channel_T * const * p_channels, uint8_t count)
{
    adc_pin_t pins[ADC_FIFO_LENGTH_MAX]; /* This should optimize away. */
    for (uint8_t index = 0U; index < count; index++) { pins[index] = p_channels[index]->PIN; }
    HAL_ADC_ActivateEach(p_hal, pins, count);
}

static inline void _ADC_Activate(HAL_ADC_T * p_hal, const  ADC_State_T * p_state)
{
    _ADC_ActivateFrom(p_hal, &p_state->ActiveChannels[0U], p_state->ActiveChannelCount);
}


/******************************************************************************/
/*
    Software activation
    Fills the Hw fifo, or the single conversion register, from the marked channels.
*/
/******************************************************************************/
/*
    Write critical section buffer. In the ISR, or in a single thread while the ADC is inactive.
    Marker writes from an ISR preempting this function are lost.
*/
static inline adc_mask_t _ADC_SetStateFrom(ADC_State_T * p_state, ADC_Channel_T * p_channels, adc_mask_t sourceMarkers)
{
    adc_mask_t markers = sourceMarkers;
    uint8_t count = 0U;

    while ((markers != 0UL) && (count < ADC_FIFO_LENGTH_MAX))
    {
        p_state->ActiveChannels[count] = &p_channels[__builtin_ctz(markers)];
        markers &= (markers - 1);
        count++;
    }

    p_state->ActiveChannelCount = count;

    return markers ^ sourceMarkers; /* return processed markers */
}

// static void ADC_StartFrom(ADC_T * p_adc, ConversionChannel_T * p_conversions, uint32_t markers)
// {
//     ADC_SetStateFrom(p_adc->P_ADC_STATE, p_conversions, markers);
//     ADC_Activate(p_adc, p_adc->P_ADC_STATE);
// }

/******************************************************************************/
/*
    Sequence activation
    Set writes state. Activate writes Hw registers.
*/
/******************************************************************************/
/******************************************************************************/
/*
    Set. State only. Activation is in _ADC.h, it writes the Hw
*/
/******************************************************************************/
/* The set converting now, and for software activation the channels it converts */
static inline void _ADC_SetSequence(ADC_T * p_adc, const ADC_Sequence_T * p_sequence)
{
    assert(p_sequence != NULL);
    assert(((uint64_t)p_sequence->CHANNELS >> p_adc->CHANNEL_COUNT) == 0ULL);

    p_adc->P_STATE->p_ActiveSequence = (ADC_Sequence_T *)p_sequence;
#if !ADC_HW_SEQUENCER_ENABLE
    ADC_MarkAll(p_adc, p_sequence->CHANNELS);   /* Any set. The thread or the ISR walks them */
#endif
}

/*
    Hw registers. The sequencer routes the trigger to the set's slot range, and converts it on every trigger.

    Software has no sequencer to write. Its register write is the fifo push, which is _ADC_ProcMarked,
    driven by the markers that _ADC_SetSequence left. Pushing here as well would re-push a fifo in flight.
*/
static inline void _ADC_ActivateSequence(ADC_T * p_adc, const ADC_Sequence_T * p_sequence)
{
#if ADC_HW_SEQUENCER_ENABLE
    assert(ADC_Mask_IsContiguous(p_sequence->CHANNELS)); /* The Hw converts a slot range. A sparse set has no Hw sequence */
    HAL_ADC_ActivateSequence(p_adc->P_HAL_ADC, ADC_Sequence_Start(p_sequence), ADC_Sequence_Count(p_sequence));
#else
    (void)p_adc; (void)p_sequence;
#endif
}

/*
    Set and send it to the Hw. On init, or while the trigger source is inactive.
    Sets Next as well as Active. The completion applies Next, it must not be left unset.
*/
static inline void ADC_ActivateSequence(ADC_T * p_adc, uint8_t sequenceId)
{
    const ADC_Sequence_T * p_sequence = ADC_SequenceOf(p_adc, sequenceId);

    p_adc->P_STATE->p_NextSequence = (ADC_Sequence_T *)p_sequence;
    _ADC_SetSequence(p_adc, p_sequence);
    _ADC_ActivateSequence(p_adc, p_sequence);
}

/******************************************************************************/
/*
    Sequence completion
    Hw sequenced: the transfer is done. Software: the last channel of the set was captured.
*/
/******************************************************************************/
/*
    Applies the pending selection. The ADC reports nothing and pushes nothing upward: the completion
    is the call itself, which the consumer wraps. ADC_Batch.h
*/
static inline void _ADC_CompleteSequence(ADC_T * p_adc)
{
    ADC_State_T * p_state = p_adc->P_STATE;
    const ADC_Sequence_T * p_completed = p_state->p_ActiveSequence;
    const ADC_Sequence_T * p_next = p_state->p_NextSequence;    /* Snapshot, written by other threads */

    if (p_next != p_completed) { _ADC_SetSequence(p_adc, p_next); _ADC_ActivateSequence(p_adc, p_next); }
#if !ADC_HW_SEQUENCER_ENABLE
    else { p_state->p_ActiveSequence = NULL; } /* Software converts the set once per request. Idle until the next */
#endif
}
