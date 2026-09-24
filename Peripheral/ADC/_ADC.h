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

static inline void _ADC_ActivateFrom(HAL_ADC_T * p_hal, ADC_Channel_T * const * p_channels, uint8_t count)
{
    adc_pin_t pins[ADC_FIFO_LENGTH_MAX]; /* This should optimize away. */
    for (uint8_t index = 0U; index < count; index++) { pins[index] = p_channels[index]->PIN; }
    HAL_ADC_ActivateEach(p_hal, pins, count);
}

/*
    The captured channels clear their markers, so [ChannelMarkers] alone tracks what is left to convert.
    A fifo mismatch captures nothing, and those channels convert again.
*/
static inline adc_mask_t _ADC_Capture(const HAL_ADC_T * p_hal, const ADC_State_T * p_state, volatile adc_result_t * p_results)
{
    return _ADC_CaptureTo(p_hal, &p_state->ActiveChannels[0U], p_state->ActiveChannelCount, p_results);
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
    Set [ActiveConversions] to reflect fifo registers state
    Markers corresponding to this particular list of channels.
    if p_source = p_adc->P_CONVERSION_CHANNELS => p_source[index].ID = index
*/
/*
    Sets State for OnComplete
    Write Critical Section Buffer.
    single threaded access or lock
    In a single thread while ADC is inactive
    In the ADC ISR

    p_state->ChannelMarkers writes in an ISR preempting this function are lost
*/
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

// static void ADC_StartFrom(HAL_ADC_T * p_hal, ADC_State_T * p_state, ADC_Channel_T * p_channels, uint32_t markers)
// {
//     _ADC_SetStateFrom(p_state, p_channels, markers);
//     _ADC_Activate(p_hal, p_state);
// }


/******************************************************************************/
/*
    Sequence
    The leaf resolves the Hw sequencer against software activation. ADC_Thread.h and ADC_Batch.h
    compose it, and neither repeats the choice.
*/
/******************************************************************************/
/*
    Hw registers. The sequencer routes the trigger to the set's slot range, and converts it on every trigger.

    Software has no sequencer to write. Its register write is the fifo push, which _ADC_ActivateMarked does,
    driven by the markers left here. Pushing here as well would re-push a fifo in flight.
*/
static inline void _ADC_ActivateSequence(HAL_ADC_T * p_hal, ADC_State_T * p_state, ADC_Sequence_T * p_sequence)
{
    p_state->p_ActiveSequence = p_sequence;
#if ADC_HW_SEQUENCER_ENABLE
    assert(ADC_Mask_IsContiguous(p_sequence->CHANNELS)); /* The Hw converts a slot range. A sparse set has no Hw sequence */
    HAL_ADC_ActivateSequence(p_hal, ADC_Sequence_Start(p_sequence), ADC_Sequence_Count(p_sequence));
#else
    p_state->ChannelMarkers |= p_sequence->CHANNELS;    /* Any set. The thread or the ISR walks them */
    (void)p_hal;                                        /* The fifo push is _ADC_ActivateMarked's */
#endif
}

/*
    Software activation. The capture held one of the set's channels, and none of them are left.
    The remainder alone is a level: it stays true until the next request re-marks, and would repeat.
*/
static inline bool _ADC_IsSequenceComplete(const ADC_State_T * p_state, adc_mask_t captured)
{
    return ((p_state->p_ActiveSequence != NULL) &&
            ((captured & p_state->p_ActiveSequence->CHANNELS) != 0UL) &&
            ((p_state->ChannelMarkers & p_state->p_ActiveSequence->CHANNELS) == 0UL));
}

/*
    The set is complete, and its results are in the buffer. COMPLETE consumes the set as a unit,
    so a consumer needs no completion mask of its own.

    It runs before the pending selection is applied, so a selection made within it takes effect on
    the conversion that follows. Both must finish before the next trigger.

    Hw sequenced: the transfer is done, and the sequencer repeats the set. A selection rewrites it
                  here, where the ADC is idle.
    Software:     the last of the set's channels was captured. The request already applied Next,
                  so there is nothing to rewrite.
*/
static inline ADC_Sequence_T * _ADC_OnCompleteSequence(ADC_T * p_adc, ADC_State_T * p_state)
{
    ADC_Sequence_T * p_completed = p_state->p_ActiveSequence;
    ADC_Sequence_T * p_next = p_state->p_NextSequence;    /* Snapshot, written by other threads */

    if (p_completed->COMPLETE != NULL) { p_completed->COMPLETE(p_completed->P_CONTEXT); }
    if (p_next != p_completed) { _ADC_ActivateSequence(p_adc->P_HAL_ADC, p_state, p_next); }

    return p_completed;
}

/******************************************************************************/
/*
    Selection. Any thread, last request wins
*/
/******************************************************************************/
/*
    Set and send it to the Hw. On init, while the trigger source is inactive, or in a batch join,
    where every part of the trigger has landed and its ADCs are idle.
    Sets Next as well as Active. The completion applies Next, it must not be left unset.
*/
static inline void ADC_ActivateSequence(ADC_T * p_adc, uint8_t sequenceId)
{
    ADC_Sequence_T * p_sequence = ADC_SequenceOf(p_adc, sequenceId);

    p_adc->P_STATE->p_NextSequence = p_sequence;
    _ADC_ActivateSequence(p_adc->P_HAL_ADC, p_adc->P_STATE, p_sequence);
}

/*
    Request the sequence.

    Hw sequenced: the completion activates it, before the trigger that follows. The set then converts
                  on every trigger, until another is selected.
    Software:     the set is marked here, and converts once per request. A request while the previous
                  set is in flight merges into it, and that set's completion is dropped.
*/
static inline void ADC_MarkSequence(ADC_T * p_adc, uint8_t sequenceId)
{
    p_adc->P_STATE->p_NextSequence = ADC_SequenceOf(p_adc, sequenceId);
#if !ADC_HW_SEQUENCER_ENABLE
    _ADC_ActivateSequence(p_adc->P_HAL_ADC, p_adc->P_STATE, p_adc->P_STATE->p_NextSequence);
#endif
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
// static void _Analog_ADC_StartConversions(Analog_ADC_T * p_adc, Analog_ConversionChannel_T * p_conversions, uint32_t markers)
// {
//     if (Analog_ADC_ReadIsActive(p_adc) == false) { ADC_StartFrom(p_adc, p_conversions, markers); }
//     else { Analog_ADC_MarkAll(p_adc, markers); }
// }

// static void _Analog_ADC_StartConversion(Analog_ADC_T * p_adc, Analog_ConversionChannel_T * p_conversion)
// {
//     _Analog_ADC_StartConversions(p_adc, p_conversion, (1UL << p_conversion->ID)); // mask as adc fixed
// }

// /*
