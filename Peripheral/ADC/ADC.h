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
    @file   ADC.h
    @author FireSourcery
    @brief  ADC module. Core types, per ADC accessors.

    ADC.h, _ADC.h, ADC_Thread.h handle everything around 1 ADC, and know nothing above it:
        ADC_Channel_T   - 1 Hw input. The single channel unit. [ID] == [Hw slot] == [P_CHANNELS index] == [Results index]
        ADC_Sequence_T  - a channel set on 1 ADC, converted as a unit. Its results land in the buffer by [ID]
        ADC_T           - 1 converter. Owns the channel table, the results buffer, and the sequence table

    Above, and depending only downward:
        ADC_Batch_T      - sequences over 1..N ADCs, converted by 1 trigger and joined. ADC_Batch.h
        ADC_Conversion_T - the application handle, a single channel or a batch alike. ADC_Conversion.h

    There is no upward link. The ADC calls nothing above it, and holds no pointer to anything above it.
    A set's consumer belongs to its batch, and the Board's ISR is the only place that names both
    an ADC and the trigger it converts on. ADC_TriggerState_T below is state the ADC never reads.

    ADC_Channel_T.CAPTURE stays the user handler for a single channel, software activation.

    Activation is either:
        Hw sequenced    - ADC_HW_SEQUENCER_ENABLE. A trigger converts the sequence, a transfer fills the results.
                          A selected sequence repeats on every trigger until another is selected.
        Software        - Channels are marked, the ISR walks them. A selected sequence converts once per request.
*/
/******************************************************************************/
#include "HAL_ADC.h"    /* HAL contract. HAL_ADC_T, adc_result_t, adc_pin_t */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>

#ifndef ADC_HW_SEQUENCER_ENABLE
#define ADC_HW_SEQUENCER_ENABLE false
#endif

#ifdef HAL_ADC_FIFO_LENGTH_MAX
#define ADC_FIFO_LENGTH_MAX HAL_ADC_FIFO_LENGTH_MAX
#endif

#ifndef ADC_FIFO_LENGTH_MAX
#define ADC_FIFO_LENGTH_MAX 1U
#endif

/******************************************************************************/
/*
    Channel mask
*/
/******************************************************************************/
typedef uint8_t adc_channel_t;
typedef uint32_t adc_mask_t;

/* Constant expressions, for config time masks */
#define ADC_MASK(Channel)                       ((adc_mask_t)(1UL << (Channel)))
#define ADC_MASK_RANGE(ChannelStart, Count)     ((adc_mask_t)(((1UL << (Count)) - 1UL) << (ChannelStart)))

static inline adc_mask_t ADC_MaskOf(adc_channel_t channel) { return ADC_MASK(channel); }
static inline adc_mask_t ADC_MaskRange(adc_channel_t channelStart, uint8_t count) { return ADC_MASK_RANGE(channelStart, count); }
static inline bool ADC_Mask_IsContiguous(adc_mask_t mask) { return (((mask + (mask & (0UL - mask))) & mask) == 0UL); }

static inline adc_channel_t ADC_ChannelStartOf(adc_mask_t sequence) { return (adc_channel_t)__builtin_ctz(sequence); }
static inline uint8_t ADC_ChannelCountOf(adc_mask_t sequence) { return (uint8_t)__builtin_popcount(sequence); }

typedef void (*ADC_Callback_T)(void * p_context);
typedef void (*ADC_Capture_T)(void * p_context, adc_result_t value);
// typedef void (*ADC_CaptureSeq_T)(void * p_context, const volatile adc_result_t * p_values); /* The sequence's values, in channel order */
// typedef const volatile adc_result_t * ADC_DemuxEntry_T;

/******************************************************************************/
/*`
    Channel
    The single channel unit. A sequence of 1, addressed as [ADC, Channel]
*/
/******************************************************************************/
typedef const struct ADC_Channel
{
    adc_channel_t ID;           /* Index into ADC.P_CHANNELS */
    adc_pin_t PIN;              /* Physical Id of the Pin */
    ADC_Capture_T CAPTURE;      /* Optional. Software activation only, per conversion. The result is stored either way */
    void * P_CONTEXT;
    // ADC_DemuxEntry_T DEMUX;
    // uint32_t RESULT_SCALING;    /* Consumer concern, known at the Board layer */
}
ADC_Channel_T;

/* Casts the handler to its registration type. CAPTURE and P_CONTEXT are optional */
#define ADC_CHANNEL_INIT(ChannelId, PinId, p_Context, CaptureFn) (ADC_Channel_T) \
    { .ID = (ChannelId), .PIN = (PinId), .CAPTURE = (ADC_Capture_T)(CaptureFn), .P_CONTEXT = (p_Context), }

/******************************************************************************/
/*
    Sequence
    The channel set 1 ADC converts per request. A channel mask, so it serves both activations:

        Hw sequenced - the mask must be contiguous. It maps onto the Hw slot range and 1 transfer.
        Software     - any mask. The ISR walks it, fifo depth at a time, in channel order.

    On completion the sequence pushes its values to CAPTURE, in channel order. The consumer is opaque:
    a batch part, or any handler the Board registers.
*/
/******************************************************************************/
typedef const struct ADC_Sequence
{
    adc_mask_t CHANNELS;
    adc_channel_t START;
    adc_channel_t COUNT;
    ADC_Callback_T COMPLETE;   /* Runs in the completion ISR of its ADC. NULL for scan only */
    void * P_CONTEXT;           /* Opaque. A batch join record, or any handler the Board registers */
    // ADC_DemuxEntry_T * DEMUX_MAP; /* [Source] -> [Destination] */
}
ADC_Sequence_T;

/* Derives the Hw slot range from the mask, at config time. Casts the handler to its registration type */
#define ADC_SEQUENCE_INIT_COMPLETE(Channels, CompleteFn, p_Context) (ADC_Sequence_T) \
    { .CHANNELS = (Channels), .START = (adc_channel_t)__builtin_ctz(Channels), .COUNT = (adc_channel_t)__builtin_popcount(Channels), \
      .COMPLETE = (ADC_Callback_T)(CompleteFn), .P_CONTEXT = (void *)(p_Context), }

/* Scan only. The results stand in the buffer, read by [ID] */
#define ADC_SEQUENCE_INIT(Channels) ADC_SEQUENCE_INIT_COMPLETE(Channels, NULL, NULL)

static inline adc_channel_t ADC_Sequence_Start(const ADC_Sequence_T * p_sequence) { return p_sequence->START; }
static inline uint8_t ADC_Sequence_Count(const ADC_Sequence_T * p_sequence) { return p_sequence->COUNT; }



/******************************************************************************/
/*
    ADC State
    Critical section buffer, shared by the ISR and the thread that requests conversions.
*/
/******************************************************************************/
typedef struct ADC_State
{
    /*
        Selected sequence. Active is written in the completion ISR window only, Next by any thread.
        Last request wins, and applies before the next trigger.
    */
    ADC_Sequence_T * volatile p_ActiveSequence;
    ADC_Sequence_T * volatile p_NextSequence;

    /*
        Software activation. Not used when the sequencer is Hw.
        Markers are the channels still to convert, Pending the active sequence channels still to capture.
    */
    volatile adc_mask_t ChannelMarkers;
    // volatile adc_mask_t PendingMarkers;
    /*
        Reg/Fifo State
        maintained by software where direct channel read is not available.
        Result buffer outside of ADC_State.
        ADC state as purely setup control remains unmodified through the entire conversion process.
    */
    const ADC_Channel_T * ActiveChannels[ADC_FIFO_LENGTH_MAX];
    uint8_t ActiveChannelCount;

#ifndef NDEBUG
    uint32_t ErrorCount;
    uint32_t IncompleteCycles;
    uint32_t FifoMismatch;
#endif
}
ADC_State_T;

#define ADC_STATE_ALLOC() (&(ADC_State_T){})

/******************************************************************************/
/*
    ADC Peripheral Control
    Wraps HAL_ADC with the channel table, the results buffer, and the sequence table.
        - Context Per Thread
        - Wraps HAL_ADC with State, callback context
*/
/******************************************************************************/
/* Tag is ADC_Module, not ADC. Vendor headers define ADC as a peripheral base, e.g. MKE06Z4 */
typedef const struct ADC_Module
{
    HAL_ADC_T * P_HAL_ADC;
    ADC_State_T * P_STATE;

    const ADC_Channel_T * P_CHANNELS;           /* [Channel] */
    uint8_t CHANNEL_COUNT;                      /* Pins may repeat, for a channel per consumer */

    /* Parallel to P_CHANNELS, for the Hw transfer */
    volatile adc_result_t * P_CHANNEL_RESULTS;  /* [Channel] -> result. Hw transfer destination */

    /* Sequence share completion markers [ChannelMarkers] so long as its single consumer, i.e sequences are non-overlapping. */
    const ADC_Sequence_T * P_SEQUENCES;         /* [SequenceId] */
    uint8_t SEQUENCE_COUNT;
    // volatile adc_mask_t * P_SEQUENCES_STATES;
}
ADC_T;

/******************************************************************************/
/*
    Hw status
*/
/******************************************************************************/
/*
    Sufficient for a lower priority thread check. A lower priority thread cannot override an ISR update
*/
static inline bool ADC_ReadIsActive(ADC_T * p_adc) { return HAL_ADC_ReadConversionActiveFlag(p_adc->P_HAL_ADC); }
static inline void ADC_Deactivate(ADC_T * p_adc) { HAL_ADC_Deactivate(p_adc->P_HAL_ADC); }

/******************************************************************************/
/*
    Channel access
*/
/******************************************************************************/
static inline const ADC_Channel_T * ADC_ChannelOf(ADC_T * p_adc, adc_channel_t channel) { return &p_adc->P_CHANNELS[channel]; }
static inline adc_result_t ADC_ResultOf(ADC_T * p_adc, adc_channel_t channel) { return p_adc->P_CHANNEL_RESULTS[channel]; }

/******************************************************************************/
/*
    Channel markers. Software activation only
*/
/******************************************************************************/
static inline void ADC_MarkChannel(ADC_T * p_adc, adc_channel_t channel) { p_adc->P_STATE->ChannelMarkers |= ADC_MaskOf(channel); }
static inline bool ADC_IsMarked(ADC_T * p_adc, adc_channel_t channel) { return ((p_adc->P_STATE->ChannelMarkers & ADC_MaskOf(channel)) != 0UL); }

static inline void ADC_MarkAll(ADC_T * p_adc, adc_mask_t mask) { p_adc->P_STATE->ChannelMarkers |= mask; }
static inline bool ADC_IsMarkedAll(ADC_T * p_adc, adc_mask_t mask) { return ((p_adc->P_STATE->ChannelMarkers & mask) == mask); }


/******************************************************************************/
/*
    Sequence selection
    Activation is in _ADC.h, it resolves the Hw
*/
/******************************************************************************/
static inline const ADC_Sequence_T * ADC_SequenceOf(ADC_T * p_adc, uint8_t sequenceId) { return &p_adc->P_SEQUENCES[sequenceId]; }
static inline const ADC_Sequence_T * ADC_GetActiveSequence(ADC_T * p_adc) { return p_adc->P_STATE->p_ActiveSequence; }
static inline bool ADC_IsSequenceActive(ADC_T * p_adc, uint8_t sequenceId) { return (p_adc->P_STATE->p_ActiveSequence == ADC_SequenceOf(p_adc, sequenceId)); }



/*
    Compacting decode. The channels of [mask], in channel order, packed from p_dest[0].
    For any other order, or across ADCs, see ADC_Map.h
*/
// static inline void _ADC_GetResultsOf(ADC_T * p_adc, adc_mask_t mask, adc_result_t * p_dest)
// {
//     uint8_t index = 0U;
//     for (adc_mask_t markers = mask; markers != 0UL; markers &= (markers - 1UL))
//     {
//         p_dest[index++] = p_adc->P_CHANNEL_RESULTS[__builtin_ctz(markers)];
//     }
// }

// static inline void ADC_GetResultsOf(ADC_T * p_adc, uint8_t sequenceId, adc_result_t * p_dest)
// {
//     _ADC_GetResultsOf(p_adc, ADC_SequenceOf(p_adc, sequenceId)->CHANNELS, p_dest);
// }


/******************************************************************************/
/*
    Init
    Hw trigger and transfer resources are set up by the Board, before the first activation.
*/
/******************************************************************************/
static inline void ADC_Init(ADC_T * p_adc)
{
    HAL_ADC_Init(p_adc->P_HAL_ADC);
    HAL_ADC_Deactivate(p_adc->P_HAL_ADC);
}
