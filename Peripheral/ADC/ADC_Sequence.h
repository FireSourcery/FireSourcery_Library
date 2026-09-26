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
    @file   ADC_Sequence.h
    @author FireSourcery
    @brief  The channel set collaborator. A set on 1 ADC, converted as a unit.

    [ADC_Sequence_T] names its own ADC, so it is a complete activation record and serves both styles:
        Hw sequenced - the mask must be contiguous. It maps onto the Hw slot range and 1 transfer.
        Software     - any mask. The ISR walks it, fifo depth at a time, in channel order.

    Completion is the same test either way: the set's channels, taken from [CompleteFlags]. Taking
    consumes, so the handler fires once per set without the module comparing set pointers to find
    the edge. That is also why a batch part is an [ADC_Sequence_T] and not a type of its own.

    [ADC_SequenceTrigger_T] is the selection, and belongs to the trigger rather than to a set.
    The sets that alternate on 1 trigger share 1 state, so only 1 of them is ever in flight.
*/
/******************************************************************************/
#include "ADC_Thread.h"  /* The per ADC base a sequence composes. ADC.h, _ADC.h below it */

typedef const struct ADC_Sequence
{
    adc_mask_t CHANNELS;
    ADC_Callback_T COMPLETE;    /* Optional. Runs where the set is taken complete */
    void * P_CONTEXT;           /* Opaque. Any handler the Board registers */
    ADC_T * P_ADC;
}
ADC_Sequence_T;

/* Casts the handler to its registration type. Braces only, so a sequence can nest in a batch's parts */
#define ADC_SEQUENCE_FIELDS(p_Adc, Channels, CompleteFn, p_Context) \
    { .P_ADC = (p_Adc), .CHANNELS = (Channels), .COMPLETE = (ADC_Callback_T)(CompleteFn), .P_CONTEXT = (void *)(p_Context), }

#define ADC_SEQUENCE(p_Adc, Channels, CompleteFn, p_Context) (ADC_Sequence_T)ADC_SEQUENCE_FIELDS(p_Adc, Channels, CompleteFn, p_Context)

static inline adc_channel_t ADC_Sequence_Start(ADC_Sequence_T * p_sequence) { return ADC_ChannelStartOf(p_sequence->CHANNELS); }
static inline uint8_t ADC_Sequence_Count(ADC_Sequence_T * p_sequence) { return ADC_ChannelCountOf(p_sequence->CHANNELS); }

/******************************************************************************/
/*
    Activation
*/
/******************************************************************************/
/* Software. The markers are the request, and the set converts once per request */
static inline void ADC_Sequence_Mark(ADC_Sequence_T * p_sequence) { ADC_MarkAll(p_sequence->P_ADC, p_sequence->CHANNELS); }

static inline void ADC_Sequence_Activate(ADC_Sequence_T * p_sequence)
{
    ADC_Sequence_Mark(p_sequence);
    ADC_ActivateMarked(p_sequence->P_ADC);
}

/* Hw sequenced. The set repeats on every trigger until another range is written */
static inline void ADC_Sequence_ActivateDma(ADC_Sequence_T * p_sequence)
{
    assert(ADC_Mask_IsContiguous(p_sequence->CHANNELS)); /* The Hw converts a slot range. A sparse set has no Hw sequence */
    ADC_ActivateSlots(p_sequence->P_ADC, ADC_Sequence_Start(p_sequence), ADC_Sequence_Count(p_sequence));
}

/******************************************************************************/
/*
    Completion
*/
/******************************************************************************/
static inline bool ADC_Sequence_IsComplete(ADC_Sequence_T * p_sequence) { return ADC_IsComplete(p_sequence->P_ADC, p_sequence->CHANNELS); }

/* All or nothing. A partial set keeps its flags, so a set that straddles 2 ISRs still closes */
static inline bool ADC_Sequence_TakeComplete(ADC_Sequence_T * p_sequence) { return ADC_TakeComplete(p_sequence->P_ADC, p_sequence->CHANNELS); }

/*!
    @brief  Close the set where it has landed. Runs COMPLETE once, from the ISR or from a thread.
    @return true where the set closed on this call
*/
static inline bool ADC_Sequence_Poll(ADC_Sequence_T * p_sequence)
{
    bool isComplete = ADC_Sequence_TakeComplete(p_sequence);
    if ((isComplete == true) && (p_sequence->COMPLETE != NULL)) { p_sequence->COMPLETE(p_sequence->P_CONTEXT); }
    return isComplete;
}

/******************************************************************************/
/*
    Trigger
    The selection, shared by the sets that alternate on 1 trigger.
    Active is written in the completion window only, Next by any thread. Last request wins.
*/
/******************************************************************************/
typedef struct ADC_SequenceTrigger
{
    ADC_Sequence_T * volatile p_ActiveSequence;
    ADC_Sequence_T * volatile p_NextSequence;
}
ADC_SequenceTrigger_T;

#define ADC_SEQUENCE_TRIGGER_ALLOC() (&(ADC_SequenceTrigger_T){})

/* Deferred, 1 store. Any thread. Applied at the completion that follows */
static inline void ADC_SequenceTrigger_Select(ADC_SequenceTrigger_T * p_trigger, ADC_Sequence_T * p_sequence) { p_trigger->p_NextSequence = p_sequence; }

/* Converting. A deferred selection is not applied until the active set completes */
static inline bool ADC_SequenceTrigger_IsActive(const ADC_SequenceTrigger_T * p_trigger, ADC_Sequence_T * p_sequence) { return (p_trigger->p_ActiveSequence == p_sequence); }

/* Selected. Active already, or applied at the next completion */
static inline bool ADC_SequenceTrigger_IsSelected(const ADC_SequenceTrigger_T * p_trigger, ADC_Sequence_T * p_sequence) { return (p_trigger->p_NextSequence == p_sequence); }

/*
    Immediate. On init, or while the trigger source is inactive.
    Sets Next as well as Active: nothing is left pending, and the completion has no change to see.
*/
static inline void ADC_SequenceTrigger_ActivateDma(ADC_SequenceTrigger_T * p_trigger, ADC_Sequence_T * p_sequence)
{
    p_trigger->p_NextSequence = p_sequence;
    p_trigger->p_ActiveSequence = p_sequence;
    ADC_Sequence_ActivateDma(p_sequence);
}

/*!
    @brief  1 transfer, 1 set. Run in the transfer complete ISR, the caller clears the Hw flag.

    The selection is read after the handler, so a set selected from there applies now, and read
    once, so the set sent to the Hw is the set recorded.
*/
static inline void ADC_SequenceTrigger_OnCompleteDma_ISR(ADC_SequenceTrigger_T * p_trigger)
{
    ADC_Sequence_T * p_completed = p_trigger->p_ActiveSequence;
    assert(p_completed != NULL); /* A sequence must be activated before the trigger is enabled */

    ADC_OnCompleteTransfer_ISR(p_completed->P_ADC, p_completed->CHANNELS);
    ADC_Sequence_Poll(p_completed);

    ADC_Sequence_T * p_next = p_trigger->p_NextSequence;
    if (p_next != p_completed) { ADC_Sequence_ActivateDma(p_next); p_trigger->p_ActiveSequence = p_next; }
}

/*!
    @brief  Software. The base ISR has captured and flagged; this closes the set and starts the next.
            Run after ADC_OnComplete_ISR, or from the thread that polls.
*/
static inline void ADC_SequenceTrigger_OnComplete(ADC_SequenceTrigger_T * p_trigger)
{
    ADC_Sequence_T * p_completed = p_trigger->p_ActiveSequence;
    assert(p_completed != NULL);

    if (ADC_Sequence_Poll(p_completed) == true)
    {
        ADC_Sequence_T * p_next = p_trigger->p_NextSequence;
        if (p_next != p_completed) { p_trigger->p_ActiveSequence = p_next; }
        ADC_Sequence_Activate(p_trigger->p_ActiveSequence);
    }
}
