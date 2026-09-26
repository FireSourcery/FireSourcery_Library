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
    @brief  ADC base. 1 converter, addressed by channel mask.

    The base holds no handler and no set. It converts what is marked, reports what landed, and
    names nothing above itself. [adc_channel_t] is its only address: [ID] == [Hw slot] ==
    [P_CHANNELS index] == [P_CHANNEL_RESULTS index].

    The collaborators are optional, one per activation style, each depending only downward:
        ADC_Channel_T  - [channel] -> a push handler. ADC_Channel.h
        ADC_Sequence_T - a channel set on 1 ADC, selected against a trigger. ADC_Sequence.h
        ADC_Batch_T    - sets over 1..N ADCs, joined by 1 trigger. ADC_Batch.h

    Completion is 2 mask words, maintained the same way by both activation styles:
        [ChannelMarkers] - to convert. The software walker's work queue, cleared as each lands.
        [CompleteFlags]  - landed, not yet consumed. Set by the ISR, cleared by whoever takes it.

    They are separate words because the clear authority differs. Sharing one would have the
    walker's clear erase a consumer's notification, and a consumer's clear requeue conversions.
    [CompleteFlags] is the only state both activation styles maintain, so it is what lets the
    Board read completion the same way whichever style an ADC runs.

    Activation is either:
        Hw sequenced    - ADC_HW_SEQUENCER_ENABLE. A trigger converts the slot range, a transfer
                          fills the results, and the Board's ISR flags the set it armed.
        Software        - Channels are marked, the ISR walks them, fifo depth at a time.
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
#define ADC_MASK_ALL                            ((adc_mask_t)(~0UL))

static inline adc_mask_t ADC_MaskOf(adc_channel_t channel) { return ADC_MASK(channel); }
static inline adc_mask_t ADC_MaskRange(adc_channel_t channelStart, uint8_t count) { return ADC_MASK_RANGE(channelStart, count); }
static inline bool ADC_Mask_IsContiguous(adc_mask_t mask) { return (((mask + (mask & (0UL - mask))) & mask) == 0UL); }

static inline adc_channel_t ADC_ChannelStartOf(adc_mask_t mask) { return (adc_channel_t)__builtin_ctz(mask); }
static inline uint8_t ADC_ChannelCountOf(adc_mask_t mask) { return (uint8_t)__builtin_popcount(mask); }

/******************************************************************************/
/*
    Handler types. Vocabulary shared by the collaborators, which the base never calls.
*/
/******************************************************************************/
typedef void (*ADC_Callback_T)(void * p_context);
typedef void (*ADC_Capture_T)(void * p_context, adc_result_t value);

/******************************************************************************/
/*
    ADC State
    Critical section buffer, shared by the ISR and the thread that requests conversions.
*/
/******************************************************************************/
typedef struct ADC_State
{
    volatile adc_mask_t ChannelMarkers;     /* To convert. Software activation only, the walker clears as each lands */
    volatile adc_mask_t CompleteFlags;      /* Landed, not yet consumed. The consumer clears what it takes */

    /*
        Mirrors the fifo registers, where a direct channel read is unavailable.
        Setup control is unmodified through the conversion, so the results buffer is outside this.
    */
    adc_channel_t ActiveChannels[ADC_FIFO_LENGTH_MAX];
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
    Wraps HAL_ADC with the pin table, the results buffer, and the completion state.
*/
/******************************************************************************/
/* Tag is ADC_Module, not ADC. Vendor headers define ADC as a peripheral base, e.g. MKE06Z4 */
typedef const struct ADC_Module
{
    HAL_ADC_T * P_HAL_ADC;
    ADC_State_T * P_STATE;

    const adc_pin_t * P_CHANNEL_PINS;               /* [Channel] -> Hw pin. The base's whole notion of a channel */
    uint8_t CHANNEL_COUNT;                      /* Pins may repeat, for a channel per consumer */

    /* Parallel to P_CHANNELS, for the Hw transfer */
    volatile adc_result_t * P_CHANNEL_RESULTS;  /* [Channel] -> result. Hw transfer destination */
}
ADC_T;

#include "_ADC.h" /* Include the private header after typedefs */

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
static inline adc_pin_t ADC_PinOf(ADC_T * p_adc, adc_channel_t channel) { return p_adc->P_CHANNEL_PINS[channel]; }
static inline adc_result_t ADC_ResultOf(ADC_T * p_adc, adc_channel_t channel) { return p_adc->P_CHANNEL_RESULTS[channel]; }

/******************************************************************************/
/*
    Channel markers. The software walker's work queue
*/
/******************************************************************************/
static inline void ADC_MarkChannel(ADC_T * p_adc, adc_channel_t channel) { p_adc->P_STATE->ChannelMarkers |= ADC_MaskOf(channel); }
static inline bool ADC_IsMarked(ADC_T * p_adc, adc_channel_t channel) { return ((p_adc->P_STATE->ChannelMarkers & ADC_MaskOf(channel)) != 0UL); }

static inline void ADC_MarkAll(ADC_T * p_adc, adc_mask_t mask) { p_adc->P_STATE->ChannelMarkers |= mask; }
static inline bool ADC_IsMarkedAll(ADC_T * p_adc, adc_mask_t mask) { return ((p_adc->P_STATE->ChannelMarkers & mask) == mask); }

/******************************************************************************/
/*
    Completion flags.
    The Board's view, identical whichever activation style the ADC runs.

    Peek is a level, and stays true until taken. Take is the edge: it consumes, so a handler
    driven by it runs once per set, and a set the caller did not name is left for its own consumer.
*/
/******************************************************************************/
static inline adc_mask_t ADC_CompleteFlags(ADC_T * p_adc) { return p_adc->P_STATE->CompleteFlags; }
static inline bool ADC_IsComplete(ADC_T * p_adc, adc_mask_t mask) { return ((p_adc->P_STATE->CompleteFlags & mask) == mask); }

/*
    The clear is a read modify write against the ISR's OR, and Cortex-M0+ has no LDREX.
    Take from 1 context per ADC: its completion ISR, or the thread that polls it. Taking from a
    thread while an ISR flags the same ADC loses bits, and needs a Critical section around both.
*/
/* All or nothing. A partial set keeps its flags, so it can still complete */
static inline bool ADC_TakeComplete(ADC_T * p_adc, adc_mask_t mask)
{
    bool isComplete = ((p_adc->P_STATE->CompleteFlags & mask) == mask);
    if (isComplete == true) { p_adc->P_STATE->CompleteFlags &= ~mask; }
    return isComplete;
}

/* Greedy. Takes whatever of [mask] has landed */
static inline adc_mask_t ADC_TakeFlags(ADC_T * p_adc, adc_mask_t mask)
{
    adc_mask_t taken = p_adc->P_STATE->CompleteFlags & mask;
    p_adc->P_STATE->CompleteFlags &= ~taken;
    return taken;
}

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
