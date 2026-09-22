// #pragma once

// /******************************************************************************/
// /*!
//     @section LICENSE

//     Copyright (C) 2026 FireSourcery

//     This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.

//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.

//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
// */
// /******************************************************************************/
// /******************************************************************************/
// /*!
//     @file   _ADC.h
//     @author FireSourcery
//     @brief  Private. Capture and activation. Resolves Hw sequenced against software activation.
// */
// /******************************************************************************/
// #include "ADC.h"

// /******************************************************************************/
// /*
//     Hw sequencer import.
//     Route the trigger to [channelStart, channelStart + count). The Hw converts and transfers on each trigger.
//     Board or platform provided, when ADC_HW_SEQUENCER_ENABLE is defined.
// */
// /******************************************************************************/
// #ifdef ADC_HW_SEQUENCER_ENABLE
// static inline void HAL_ADC_ActivateSequence(HAL_ADC_T * p_hal, uint32_t channelStart, uint32_t count);
// #endif


// static inline void _ADC_CaptureChannel(const HAL_ADC_T * p_hal, ADC_Channel_T * p_channel, volatile adc_result_t * p_dest)
// {
//     adc_result_t result = HAL_ADC_ReadResult(p_hal, p_channel->PIN);
//     if (p_channel->CAPTURE != NULL) { p_channel->CAPTURE(p_channel->P_CONTEXT, result); }
//     *p_dest = result;
// }

// static inline void _ADC_CaptureTo(const HAL_ADC_T * p_hal, ADC_Channel_T * const * p_channels, uint8_t count, adc_result_t * p_results)
// {
//     if (count == HAL_ADC_ReadFifoCount(p_hal))
//     {
//         for (uint8_t index = 0U; index < count; index++) /* Read in the same way it was pushed */
//         {
//             // p_results[index] = HAL_ADC_ReadResult(p_hal, p_channels[index]->PIN);
//             // _ADC_CaptureResult(p_channels[index], HAL_ADC_ReadResult(p_hal, p_channels[index]->PIN), &p_results[p_channels[index]->ID]);
//             _ADC_CaptureChannel(p_hal, p_channels[index], &p_results[p_channels[index]->ID]);
//         }
//     }
// }

// static void _ADC_ActivateFrom(HAL_ADC_T * p_hal, ADC_Channel_T * const * p_channels, uint8_t count)
// {
//     adc_pin_t pins[ADC_FIFO_LENGTH_MAX]; /* This should optimize away. */
//     for (uint8_t index = 0U; index < count; index++) { pins[index] = p_channels[index]->PIN; }
//     HAL_ADC_ActivateEach(p_hal, pins, count);
// }

// /******************************************************************************/
// /*
//     Capture. Software activation only. The Hw transfer fills the results buffer directly.
// */
// /******************************************************************************/
// static inline void _ADC_CaptureChannel(const ADC_T * p_adc, adc_channel_t channel)
// {
//     p_adc->P_CHANNEL_RESULTS[channel] = HAL_ADC_ReadResult(p_adc->P_HAL_ADC, p_adc->P_CHANNELS[channel].PIN);
// }

// /*
//     Read in the same way it was pushed. Returns the captured channels.
// */
// static inline adc_mask_t _ADC_CaptureActive(const ADC_T * p_adc)
// {
//     ADC_State_T * p_state = p_adc->P_STATE;
//     adc_mask_t captured = 0UL;
//     uint8_t count = p_state->ActiveChannelCount;

// #if (ADC_FIFO_LENGTH_MAX > 1U)
//     /* The fifo must hold what was pushed, or the results map onto the wrong channels */
//     if (count != HAL_ADC_ReadFifoCount(p_adc->P_HAL_ADC))
//     {
//         count = 0U;
// #ifndef NDEBUG
//         p_state->FifoMismatch++;
// #endif
//     }
// #endif

//     for (uint8_t index = 0U; index < count; index++)
//     {
//         _ADC_CaptureChannel(p_adc, p_state->ActiveChannels[index]);
//         captured |= ADC_MaskOf(p_state->ActiveChannels[index]);
//     }
//     p_state->ActiveChannelCount = 0U;

//     return captured;
// }

// /******************************************************************************/
// /*
//     Software activation
//     Fills the Hw fifo, or the single conversion register, from the marked channels.
// */
// /******************************************************************************/
// /*
//     Write critical section buffer. In the ISR, or in a single thread while the ADC is inactive.
//     Marker writes from an ISR preempting this function are lost.
// */
// static inline void _ADC_StartMarked(const ADC_T * p_adc)
// {
//     ADC_State_T * p_state = p_adc->P_STATE;
//     adc_pin_t pins[ADC_FIFO_LENGTH_MAX]; /* Optimizes away for a length of 1 */
//     adc_mask_t markers = p_state->ChannelMarkers;
//     uint8_t count = 0U;

//     while ((count < ADC_FIFO_LENGTH_MAX) && (markers != 0UL))
//     {
//         p_state->ActiveChannels[count] = (adc_channel_t)__builtin_ctz(markers);
//         pins[count] = p_adc->P_CHANNEL_PINS[p_state->ActiveChannels[count]];
//         markers &= (markers - 1UL);
//         count++;
//     }

//     p_state->ActiveChannelCount = count;
//     p_state->ChannelMarkers = markers; /* The remainder continues on the next completion */

//     HAL_ADC_ActivateEach(p_adc->P_HAL_ADC, pins, count);
// }

// /******************************************************************************/
// /*
//     Sequence activation
// */
// /******************************************************************************/
// static inline void _ADC_ActivateSequence(const ADC_T * p_adc, const ADC_Sequence_T * p_sequence)
// {
//     assert(p_sequence != NULL);
//     assert(((uint64_t)p_sequence->CHANNELS >> p_adc->CHANNEL_COUNT) == 0ULL);

//     p_adc->P_STATE->p_ActiveSequence = p_sequence;
//     p_adc->P_STATE->PendingMarkers = p_sequence->CHANNELS;

// #ifdef ADC_HW_SEQUENCER_ENABLE
//     /* The Hw converts a slot range. A sparse set has no Hw sequence */
//     assert(ADC_Mask_IsContiguous(p_sequence->CHANNELS));
//     HAL_ADC_ActivateSequence(p_adc->P_HAL_ADC, ADC_Sequence_ChannelStart(p_sequence), ADC_Sequence_Count(p_sequence));
// #else
//     /* Any set. The thread or the ISR starts the marked channels */
//     ADC_MarkAll(p_adc, p_sequence->CHANNELS);
// #endif
// }

// /*
//     Immediate. On init, or while the trigger source is inactive.
//     Sets Next as well as Active. The completion applies Next, it must not be left unset.
// */
// static inline void ADC_ActivateSequence(const ADC_T * p_adc, uint8_t sequenceId)
// {
//     p_adc->P_STATE->p_NextSequence = ADC_SequenceOf(p_adc, sequenceId);
//     _ADC_ActivateSequence(p_adc, ADC_SequenceOf(p_adc, sequenceId));
// }

// /*
//     Request the sequence. Any thread.

//     Hw sequenced: deferred. Applied in the completion ISR, before the next trigger. Last request wins,
//                   then the sequence repeats on every trigger until another is selected.
//     Software:     immediate. The request marks the channels for 1 conversion of the set.
//                   A request while the previous set is still converting merges into it, and the
//                   previous completion is dropped.
// */
// static inline void ADC_SetSequence(const ADC_T * p_adc, uint8_t sequenceId)
// {
// #ifdef ADC_HW_SEQUENCER_ENABLE
//     p_adc->P_STATE->p_NextSequence = ADC_SequenceOf(p_adc, sequenceId);
// #else
//     ADC_ActivateSequence(p_adc, sequenceId);
// #endif
// }


// /******************************************************************************/
// /*
//     Sequence completion
//     Hw sequenced: the transfer is done. Software: the last channel of the sequence was captured.
// */
// /******************************************************************************/
// /*
//     ON_COMPLETE runs before the pending selection is applied, so a selection made within it
//     takes effect on the next trigger. Both must finish before that trigger.
// */
// static inline void _ADC_CompleteSequence(const ADC_T * p_adc)
// {
//     ADC_State_T * p_state = p_adc->P_STATE;
//     const ADC_Sequence_T * p_sequence = p_state->p_ActiveSequence;
//     const ADC_Sequence_T * p_next = p_state->p_NextSequence; /* Snapshot, written by other threads */

//     if (p_sequence->ON_COMPLETE != NULL) { p_sequence->ON_COMPLETE(p_sequence->P_CONTEXT); }

//     if (p_next != p_sequence) { _ADC_ActivateSequence(p_adc, p_next); }
// #ifndef ADC_HW_SEQUENCER_ENABLE
//     else { p_state->p_ActiveSequence = NULL; } /* Software activation converts the set once per request. Idle until the next */
// #endif
// }

// #ifndef ADC_HW_SEQUENCER_ENABLE
// #define ADC_HW_SEQUENCER_ENABLE false
// #endif