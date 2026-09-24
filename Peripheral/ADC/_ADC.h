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
#else
static inline void HAL_ADC_ActivateSequence(HAL_ADC_T * p_hal, uint32_t channelStart, uint32_t count) { (void)p_hal; (void)channelStart; (void)count; }
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

static void _ADC_ActivateMarked(ADC_T * p_adc, ADC_State_T * p_state)
{
    _ADC_SetStateFrom(p_state, &p_adc->P_CHANNELS[0U], p_state->ChannelMarkers);
    assert(p_state->ActiveChannelCount > 0U); /* Callers gate on ChannelMarkers, so the selection is never empty */
    _ADC_Activate(p_adc->P_HAL_ADC, p_state);
}

/******************************************************************************/
/*
    Sequence
    The leaf resolves the Hw sequencer against software activation.
*/
/******************************************************************************/
// static inline ADC_Sequence_T * _ADC_OnCompleteSequenceCommon(ADC_State_T * p_state)
// {
//     ADC_Sequence_T * p_completed = p_state->p_ActiveSequence;
//     ADC_Sequence_T * p_next = p_state->p_NextSequence;
//     if (p_completed->COMPLETE != NULL) { p_completed->COMPLETE(p_completed->P_CONTEXT); }
//     if (p_next != p_completed) { p_state->p_ActiveSequence = p_next; }
//     return p_completed;
// }

/*
    Software activation. No sequencer to write: the register write is the fifo push, which
    _ADC_ActivateMarked does from the markers left here.

    Markers before Active, so a completion landing between them still measures the set that is
    converting. Publishing Active first leaves a window where the new set has no markers, and the
    old set's capture reads as the new set's last channel.
*/
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

static inline ADC_Sequence_T * _ADC_OnCompleteSequenceFifo(ADC_State_T * p_state)
{
    ADC_Sequence_T * p_completed = p_state->p_ActiveSequence;
    ADC_Sequence_T * p_next = p_state->p_NextSequence;    /* Snapshot, written by other threads */
    if (p_completed->COMPLETE != NULL) { p_completed->COMPLETE(p_completed->P_CONTEXT); }
    if (p_next != p_completed)
    {
        p_state->p_ActiveSequence = p_next;
        p_state->ChannelMarkers |= p_next->CHANNELS;
    }
    return p_completed;
}

static inline void _ADC_SetSequence(ADC_State_T * p_state, ADC_Sequence_T * p_sequence)
{
    p_state->ChannelMarkers |= p_sequence->CHANNELS;
    p_state->p_NextSequence = p_sequence;
    p_state->p_ActiveSequence = p_sequence;
}

/******************************************************************************/
/*
    Hw Sequence
*/
/******************************************************************************/
/*
    Hw registers. The sequencer routes the trigger to the set's slot range, and converts it on every trigger.

    Software has no sequencer to write. Its register write is the fifo push, which _ADC_ActivateMarked does,
    driven by the markers left here. Pushing here as well would re-push a fifo in flight.
*/
static inline void _ADC_ActivateSequenceDma(HAL_ADC_T * p_hal, ADC_State_T * p_state, ADC_Sequence_T * p_sequence)
{
    assert(ADC_Mask_IsContiguous(p_sequence->CHANNELS)); /* The Hw converts a slot range. A sparse set has no Hw sequence */
    p_state->p_NextSequence = p_sequence;
    p_state->p_ActiveSequence = p_sequence;
    HAL_ADC_ActivateSequence(p_hal, ADC_Sequence_Start(p_sequence), ADC_Sequence_Count(p_sequence));
}

static inline ADC_Sequence_T * _ADC_OnCompleteSequenceDma(HAL_ADC_T * p_hal, ADC_State_T * p_state)
{
    ADC_Sequence_T * p_completed = p_state->p_ActiveSequence;
    ADC_Sequence_T * p_next = p_state->p_NextSequence;    /* Snapshot, written by other threads */
    if (p_completed->COMPLETE != NULL) { p_completed->COMPLETE(p_completed->P_CONTEXT); }
    if (p_next != p_completed) { _ADC_ActivateSequenceDma(p_hal, p_state, p_next); }
    return p_completed;
}



/******************************************************************************/
/*!
    Unsynchronized Activation
    start a conversion immediately cancels ongoing conversions
    single threaded or atomic flag test and set
    (ConversionChannel_T,  uint32_t markers) interface
     select from mapped or parameters
*/
/******************************************************************************/
// static void ADC_StartConversionsADC_T * p_adc,ConversionChannel_T * p_conversions, uint32_t markers)
// {
//     if ADC_ReadIsActive(p_adc) == false) { ADC_StartFrom(p_adc, p_conversions, markers); }
//     else {ADC_MarkAll(p_adc, markers); }
// }

// static void ADC_StartConversionADC_T * p_adc,ConversionChannel_T * p_conversion)
// {
//     ADC_StartConversions(p_adc, p_conversion, (1UL << p_conversion->ID)); // mask as adc fixed
// }
