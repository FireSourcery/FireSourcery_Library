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
    @file   ADC_Batch.h
    @author FireSourcery
    @brief  The channel sets converted as a unit, and their completion. Board layer.
*/
/******************************************************************************/
#include "ADC_Thread.h"  /* The per ADC protocol a part composes. ADC.h, _ADC.h below it */


/******************************************************************************/
/*
    A batch is what 1 trigger converts: 1..N parts, a part being 1 sequence on 1 ADC. The single ADC
    case is a batch of 1 part, so nothing above distinguishes 1 channel, 1 ADC, or several ADCs.

    [ADC_BatchPart_T] is an activation record, and the only link between a batch and an ADC.
        Activation - the batch sends each part's set to its ADC, and arms that part's channels.
        Completion - the Board's ISR calls ADC_Trigger_OnComplete_ISR with the ADC that completed
                     and the trigger it converts on. That ADC clears its own set from its markers,
                     and the last part to land finds none marked and joins.
    Nothing below holds a batch pointer, and no sequence handler is spent on the join.

    [ADC_TriggerState_T] belongs to the trigger, not to the batch. The batches that alternate on 1
    trigger share 1 state, so only 1 of them is ever in flight, and:
        - a selection is 1 write. Batches cannot end up split across a trigger
        - the join applies it, where every part of the trigger has landed and its ADCs are idle
        - a part of a batch that is no longer active is ignored, so no join state survives a switch
    Batches sharing a state must span the same ADCs, in the same order. The switch reprograms them all.

    Part ISRs must share a priority, the marker update is a read modify write.

        static ADC_TriggerState_T MOTOR0_TRIGGER;   // 1 per trigger. MOTOR0_I_BATCH and MOTOR0_V_BATCH alternate on it

        // A part carries its own set. The ADCs' sequence tables are not involved
        static ADC_BatchPart_T MOTOR0_I_PARTS[] =
        {
            [0U] = ADC_BATCH_PART(&ADCS[0U], ADC_MASK_RANGE(ADC0_IA, 2U)),
            [1U] = ADC_BATCH_PART(&ADCS[1U], ADC_MASK(ADC1_IC)),
        };

        static const ADC_Batch_T MOTOR0_I_BATCH = ADC_BATCH_INIT(MOTOR0_I_PARTS, Motor_OnBatchComplete, &Motors[0U]);

        // The Board's ISRs. 1 per ADC of the trigger
        void BOARD_ADC0_DMA_ISR(void) { Board_ADC0_ClearComplete(); ADC_Trigger_OnComplete_ISR(&MOTOR0_TRIGGER, &ADCS[0U]); }
        void BOARD_ADC1_DMA_ISR(void) { Board_ADC1_ClearComplete(); ADC_Trigger_OnComplete_ISR(&MOTOR0_TRIGGER, &ADCS[1U]); }
*/
/******************************************************************************/
/*
    A part is a set bound to an ADC. It holds the set itself, so the join reads it without the ADC's
    sequence table, and a batch is declared in 1 place. An id would be a reference the Board has to
    keep in step with a table the part cannot see.
*/
typedef const struct ADC_BatchPart
{
    ADC_T * P_ADC;
    ADC_Sequence_T SEQUENCE;
}
ADC_BatchPart_T;

#define ADC_BATCH_PART(p_Adc, Channels) { .P_ADC = (p_Adc), .SEQUENCE = ADC_SEQUENCE_FIELDS(Channels, NULL, NULL) }

typedef const struct ADC_Batch
{
    ADC_BatchPart_T * P_PARTS;
    uint8_t PART_COUNT;
    ADC_Callback_T ON_COMPLETE;
    void * P_CONTEXT;
}
ADC_Batch_T;

/* Derives the part count from the array, and casts the handler to its registration type */
#define ADC_BATCH_INIT(Parts, OnComplete, p_Context) (ADC_Batch_T) \
    { .P_PARTS = (Parts), .PART_COUNT = (sizeof(Parts) / sizeof(ADC_BatchPart_T)), .ON_COMPLETE = (ADC_Callback_T)(OnComplete), .P_CONTEXT = (void *)(p_Context), }

typedef struct ADC_TriggerState
{
    ADC_Batch_T * volatile p_Active;
    ADC_Batch_T * volatile p_Next;
    //  volatile adc_mask_t Pending; simplified on complete check
}
ADC_TriggerState_T;

/******************************************************************************/
/*
    Parts
    A part reports through its own ADC's markers. The join arms every part, each ADC clears its own
    in its own ISR, and the set is complete when none is still marked. The trigger holds no
    completion state, and no ADC writes another's. Sequences on 1 ADC must not overlap.
*/
/******************************************************************************/
static inline adc_mask_t _ADC_Batch_PartChannels(ADC_BatchPart_T * p_part) { return p_part->SEQUENCE.CHANNELS; }

static inline bool _ADC_Batch_IsPartComplete(ADC_BatchPart_T * p_part) { return ((p_part->P_ADC->P_STATE->ChannelMarkers & _ADC_Batch_PartChannels(p_part)) == 0UL); }

/* Repeats over every part on every part completion. Part count is small, and there is no shared count to maintain */
static inline bool _ADC_Batch_IsEachComplete(ADC_Batch_T * p_batch)
{
    for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++) { if (_ADC_Batch_IsPartComplete(&p_batch->P_PARTS[iPart]) == false) { return false; } }
    return true;
}

/* Arms the phase. A part that missed its trigger is re-armed here, so a dropped completion costs 1 cycle and not the phase */
static inline void _ADC_Batch_MarkEach(ADC_Batch_T * p_batch)
{
    for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++) { ADC_MarkAll(p_batch->P_PARTS[iPart].P_ADC, _ADC_Batch_PartChannels(&p_batch->P_PARTS[iPart])); }
}

/*
    The batch is the selector, so it sets Next as well as Active: nothing is left pending, and the
    ADC's own completion has no change to see.
*/
static inline void _ADC_Batch_ActivatePart(ADC_BatchPart_T * p_part)
{
    p_part->P_ADC->P_STATE->p_NextSequence = &p_part->SEQUENCE;
    _ADC_ActivateSequenceDma(p_part->P_ADC->P_HAL_ADC, p_part->P_ADC->P_STATE, &p_part->SEQUENCE);
}

/* Sends every part to its Hw. Only where every ADC of the trigger is idle: on init, or in the join */
static inline void _ADC_Batch_ActivateEach(ADC_Batch_T * p_batch)
{
    for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++) { _ADC_Batch_ActivatePart(&p_batch->P_PARTS[iPart]); }
}

// static inline ADC_Sequence_T * _ADC_OnCompletePart(HAL_ADC_T * p_hal, ADC_State_T * p_state)
// {
//     ADC_Sequence_T * p_completed = p_state->p_ActiveSequence;
//     ADC_Sequence_T * p_next = p_state->p_NextSequence;
//     if (p_next != p_completed) { _ADC_ActivateSequenceDma(p_hal, p_state, p_next); }
//     return p_completed;
// }

/******************************************************************************/
/*
    Join
*/
/******************************************************************************/
/* Joins where every part of the trigger has landed, and its ADCs are idle */
static inline void _ADC_Trigger_OnComplete(ADC_TriggerState_T * p_trigger, ADC_Batch_T * p_active)
{
    assert(p_active != NULL);   /* A batch must be activated before the trigger is enabled */

    if (_ADC_Batch_IsEachComplete(p_active) == true)
    {
        if (p_active->ON_COMPLETE != NULL) { p_active->ON_COMPLETE(p_active->P_CONTEXT); }

        /*
            The selection applies here, 1 place reprogramming every part, so they cannot end up split
            across a trigger. Read after the handler, so a set selected from there applies now, and read
            once, so the set sent to the Hw is the set recorded and the set armed.
        */
        ADC_Batch_T * p_next = p_trigger->p_Next;
        if (p_next != p_active) { _ADC_Batch_ActivateEach(p_next); p_trigger->p_Active = p_next; }

        _ADC_Batch_MarkEach(p_next);    /* The set now converting: the selection, or the active set where it repeats */
    }
}

/*!
    @brief  1 per ADC of the trigger, in its transfer complete ISR. The caller clears the Hw flag.
            The last part to land runs the join. A part's own sequence handler is not used.

    Part ISRs must share a priority: the join arms every part, and each part clears its own.
*/
static inline void ADC_Trigger_OnComplete_ISR(ADC_TriggerState_T * p_trigger, ADC_T * p_adc)
{
    p_adc->P_STATE->ChannelMarkers &= ~p_adc->P_STATE->p_ActiveSequence->CHANNELS;  /* This part landed */
    // _ADC_OnCompletePart
    _ADC_Trigger_OnComplete(p_trigger, p_trigger->p_Active);   /* 1 read of Active, held across the join */
}

/******************************************************************************/
/*
    Request
*/
/******************************************************************************/
/*
    Immediate. On init, or while the trigger source is inactive.
    Sets Next as well as Active, the join applies Next and it must not be left unset.
*/
static inline void ADC_Batch_Activate(ADC_TriggerState_T * p_trigger, ADC_Batch_T * p_batch)
{
    _ADC_Batch_ActivateEach(p_batch);
    _ADC_Batch_MarkEach(p_batch);
    p_trigger->p_Active = p_batch;
    p_trigger->p_Next = p_batch;
}

/*
    Deferred, 1 store. Any thread, last request wins.
    Applied in the join, before the trigger that follows it.
*/
static inline void ADC_Batch_Select(ADC_TriggerState_T * p_trigger, ADC_Batch_T * p_batch) { p_trigger->p_Next = p_batch; }

/* Converting. A deferred selection is not applied until the active batch joins */
static inline bool ADC_Batch_IsActive(const ADC_TriggerState_T * p_trigger, const ADC_Batch_T * p_batch) { return (p_trigger->p_Active == p_batch); }

/* Selected. Active already, or applied at the next join */
static inline bool ADC_Batch_IsSelected(const ADC_TriggerState_T * p_trigger, const ADC_Batch_T * p_batch) { return (p_trigger->p_Next == p_batch); }

// ADC_TriggerState_T MultiAdcState;
// void BOARD_ADC0_ISR(void) { ADC_Batch_OnComplete_ISR(&MultiAdcState, &ANALOG_ADCS[0U]); }
// void BOARD_ADC1_ISR(void) { ADC_Batch_OnComplete_ISR(&MultiAdcState, &ANALOG_ADCS[1U]); }





// /* 1. The arm follows the ADC, not the batch. A part may switch on its own, so the arm must name
//       what that ADC will convert next, not what the batch says it converts */
// static inline adc_mask_t _ADC_Batch_PartChannels(ADC_BatchPart_T * p_part) { return p_part->P_ADC->P_STATE->p_ActiveSequence->CHANNELS; }

// static inline void ADC_Trigger_OnCompletePart_ISR(ADC_TriggerState_T * p_trigger, ADC_T * p_adc)
// {
//     assert(p_trigger->p_Active != NULL);

//     p_adc->P_STATE->ChannelMarkers &= ~p_adc->P_STATE->p_ActiveSequence->CHANNELS;  /* this part landed */

//     /* 2. It starts its own next set here, before the arm. The other parts switched at their own
//           completions earlier in this trigger, so by the arm every part is on its next set */
//     _ADC_OnCompleteSequenceDma(p_adc->P_HAL_ADC, p_adc->P_STATE);

//     if (_ADC_Batch_IsEachComplete(p_trigger->p_Active) == true)
//     {
//         if (p_trigger->p_Active->ON_COMPLETE != NULL) { p_trigger->p_Active->ON_COMPLETE(p_trigger->p_Active->P_CONTEXT); }

//         p_trigger->p_Active = p_trigger->p_Next;    /* the join no longer reprograms anything */
//         _ADC_Batch_MarkEach(p_trigger->p_Active);   /* 3. the arm is still only at the join: it is the phase boundary */
//     }
// }

// /* N stores, 1 per part. Each ADC applies its own at its own completion */
// static inline void ADC_Batch_SelectParts(ADC_TriggerState_T * p_trigger, ADC_Batch_T * p_batch)
// {
//     for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++)
//     { ADC_SetNextSequenceDma(p_batch->P_PARTS[iPart].P_ADC, p_batch->P_PARTS[iPart].SEQUENCE_ID); }
//     p_trigger->p_Next = p_batch;
// }