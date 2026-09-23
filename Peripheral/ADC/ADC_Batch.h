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

    A batch is what 1 trigger converts: 1..N parts, a part being 1 sequence on 1 ADC. The single ADC
    case is a batch of 1 part, so nothing above distinguishes 1 channel, 1 ADC, or several ADCs.

    [ADC_BatchPart_T] is an activation record, and the only link between a batch and an ADC.
        Activation - the batch activates each part's sequence on its ADC.
        Completion - the Board's ISR calls ADC_Batch_OnCompleteSequence_ISR with the ADC that
                     completed and the trigger it converts on. The batch resolves the part from
                     the sequence that completed. The last part joins.
    Nothing below holds a batch pointer, and no sequence handler is spent on the join.

    [ADC_TriggerState_T] belongs to the trigger, not to the batch. The batches that alternate on 1
    trigger share 1 state, so only 1 of them is ever in flight, and:
        - a selection is 1 write. Batches cannot end up split across a trigger
        - the join applies it, where every part of the trigger has landed and its ADCs are idle
        - a part of a batch that is no longer active is ignored, so no join state survives a switch
    Batches sharing a state must span the same ADCs, in the same order. The switch reprograms them all.

    Part ISRs must share a priority, the marker update is a read modify write.

        static ADC_TriggerState_T MOTOR0_TRIGGER;   // 1 per trigger. MOTOR0_I_BATCH and MOTOR0_V_BATCH alternate on it
        static const ADC_Batch_T MOTOR0_I_BATCH;    // tentative definition. The sequence tables register into it, and the ADCs hold those

        static const ADC_BatchPart_T MOTOR0_I_PARTS[] =
        {
            [0U] = { .P_ADC = &ADCS[0U], .SEQUENCE_ID = ADC0_SEQUENCE_IAB },
            [1U] = { .P_ADC = &ADCS[1U], .SEQUENCE_ID = ADC1_SEQUENCE_IC  },
        };

        static const ADC_Batch_T MOTOR0_I_BATCH = ADC_BATCH_INIT(&MOTOR0_TRIGGER, MOTOR0_I_PARTS, Motor_OnBatchComplete, &Motors[0U]);

        // In each ADC's sequence table. A part's sequence is an ordinary sequence
        [ADC0_SEQUENCE_IAB] = ADC_SEQUENCE_INIT(ADC_MASK_RANGE(ADC0_IA, 2U)),
        [ADC1_SEQUENCE_IC]  = ADC_SEQUENCE_INIT(ADC_MASK(ADC1_IC)),

        // The Board's ISRs. 1 per ADC of the trigger
        void BOARD_ADC0_DMA_ISR(void) { Board_ADC0_ClearComplete(); ADC_Batch_OnCompleteSequence_ISR(&MOTOR0_TRIGGER, &ADCS[0U]); }
        void BOARD_ADC1_DMA_ISR(void) { Board_ADC1_ClearComplete(); ADC_Batch_OnCompleteSequence_ISR(&MOTOR0_TRIGGER, &ADCS[1U]); }
*/
/******************************************************************************/
#include "_ADC.h"
#include "ADC.h"

/******************************************************************************/
/*
    Batch
*/
/******************************************************************************/
/* Where a part converts. The batch activates it, and nothing else reads it */
typedef const struct ADC_BatchPart
{
    const ADC_T * P_ADC;
    uint8_t SEQUENCE_ID;    /* Index into P_ADC->P_SEQUENCES */
    ADC_Sequence_T SEQUENCE;    /* The sequence this part converts */
}
ADC_BatchPart_T;

typedef const struct ADC_Batch
{
    const ADC_BatchPart_T * P_PARTS;
    uint8_t PART_COUNT;
    ADC_TriggerState_T * P_STATE;       /* The trigger's. Shared with the batches that alternate on it */
    ADC_Callback_T ON_COMPLETE;         /* All parts converted. Values read through the Board's map */
    void * P_CONTEXT;
}
ADC_Batch_T;

/* Derives the part count from the array, and casts the handler to its registration type */
#define ADC_BATCH_INIT(p_State, Parts, OnComplete, p_Context) (ADC_Batch_T) \
    { .P_PARTS = (Parts), .PART_COUNT = (sizeof(Parts) / sizeof(ADC_BatchPart_T)), .P_STATE = (p_State), .ON_COMPLETE = (ADC_Callback_T)(OnComplete), .P_CONTEXT = (p_Context), }

static inline adc_mask_t _ADC_Batch_PartMarkers(const ADC_Batch_T * p_batch) { return ((1UL << p_batch->PART_COUNT) - 1UL); }

/******************************************************************************/
/*
    Activation
*/
/******************************************************************************/
static inline void _ADC_Batch_ActivateParts(const ADC_Batch_T * p_batch)
{
    for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++) { ADC_ActivateSequence(p_batch->P_PARTS[iPart].P_ADC, p_batch->P_PARTS[iPart].SEQUENCE_ID); }
}

/* Batches on 1 trigger convert the same ADCs. The join leaves all of them idle, and the switch reprograms all of them */
static inline bool _ADC_Batch_IsSameSpan(const ADC_Batch_T * p_batch, const ADC_Batch_T * p_next)
{
    bool isSame = (p_batch->PART_COUNT == p_next->PART_COUNT);
    for (uint8_t iPart = 0U; (iPart < p_next->PART_COUNT) && (isSame == true); iPart++) { isSame = (p_batch->P_PARTS[iPart].P_ADC == p_next->P_PARTS[iPart].P_ADC); }
    return isSame;
}

/*
    In the join of the batch that is completing. Every part of the trigger has landed, its ADCs are idle.
*/
static inline void _ADC_Batch_ApplyNext(ADC_TriggerState_T * p_state)
{
    const ADC_Batch_T * p_next = p_state->p_Next;   /* Snapshot, written by other threads */

    if (p_next != p_state->p_Active)
    {
        assert(_ADC_Batch_IsSameSpan(p_state->p_Active, p_next));
        _ADC_Batch_ActivateParts(p_next);
        p_state->p_Active = p_next;
    }
}

/******************************************************************************/
/*
    Completion
*/
/******************************************************************************/
/*
    The part on this ADC. PART_COUNT where the active batch has none.
    1 ADC converts 1 sequence at a time, so a batch holds at most 1 part per ADC and the ADC names it.
*/
static inline uint8_t _ADC_Batch_PartIndexOf(const ADC_Batch_T * p_batch, const ADC_T * p_adc)
{
    for (uint8_t index = 0U; index < p_batch->PART_COUNT; index++)
    {
        if (p_batch->P_PARTS[index].P_ADC == p_adc) { return index; }
    }
    return p_batch->PART_COUNT;
}

/*
    Markers, not a count: a part completing twice is idempotent, so a missed trigger on one ADC
    does not leave the join skewed. An ADC the active batch does not hold is not a part, so an ADC
    the trigger was selected away from cannot advance the join.

    A partial set cannot reach here. The caller only calls on a completion, and a set completes
    when its last channel lands, or when its transfer does.
*/
static inline void _ADC_Batch_OnPartComplete(ADC_TriggerState_T * p_state, const ADC_T * p_adc)
{
    const ADC_Batch_T * p_batch = p_state->p_Active;
    uint8_t index = (p_batch != NULL) ? _ADC_Batch_PartIndexOf(p_batch, p_adc) : 0U;

    if ((p_batch != NULL) && (index < p_batch->PART_COUNT))
    {
        p_state->CompleteMarkers |= (1UL << index);

        if (p_state->CompleteMarkers == _ADC_Batch_PartMarkers(p_batch))
        {
            p_state->CompleteMarkers = 0UL;
            if (p_batch->ON_COMPLETE != NULL) { p_batch->ON_COMPLETE(p_batch->P_CONTEXT); }
            _ADC_Batch_ApplyNext(p_state);
        }
    }
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
static inline void ADC_Batch_Activate(const ADC_Batch_T * p_batch)
{
    p_batch->P_STATE->CompleteMarkers = 0UL;
    p_batch->P_STATE->p_Next = p_batch;
    p_batch->P_STATE->p_Active = p_batch;
    _ADC_Batch_ActivateParts(p_batch);
}


/*
    Any thread. Last request wins.

    Hw sequenced: deferred. Applied in the join, before the trigger that follows it. The batch then
                  converts on every trigger, until another batch of the trigger is selected.
    Software:     immediate. The request converts the set once.
*/
static inline void ADC_Batch_Select(const ADC_Batch_T * p_batch)
{
#if ADC_HW_SEQUENCER_ENABLE
    p_batch->P_STATE->p_Next = p_batch;
#else
    ADC_Batch_Activate(p_batch);
#endif
}

/* Converting. A deferred selection is not applied until the active batch joins */
static inline bool ADC_Batch_IsActive(const ADC_Batch_T * p_batch) { return (p_batch->P_STATE->p_Active == p_batch); }

/* Selected. Active already, or applied at the next join */
static inline bool ADC_Batch_IsSelected(const ADC_Batch_T * p_batch) { return (p_batch->P_STATE->p_Next == p_batch); }


// static inline void _ADC_Batch_OnPartComplete(const ADC_T * p_adc, const ADC_Sequence_T * p_sequence)
// {
//     ADC_Batch_T * p_batch = NULL;
//     ADC_Map_T map = p_batch->MAP;
//     // map points to p_adc->P_CHANNEL_RESULTS
//     ADC_Map_Decode(map, p_batch->LENGTH,  p_batch->P_BUFFER);
//     if (Batch_IsComplete(p_batch)) { p_batch->CAPTURE(p_batch->P_CONTEXT, p_batch->P_BUFFER); }
//     else { _ADC_Batch_ApplyNext(p_state); }
// }

// static inline void Motor_Analog_CaptureIabc(Motor_T * p_motor, const volatile adc_result_t * p_values)
// {
//     Motor_Analog_CaptureIa(p_motor, p_values[MOTOR_ANALOG_IA]);
//     Motor_Analog_CaptureIb(p_motor, p_values[MOTOR_ANALOG_IB]);
//     Motor_Analog_CaptureIc(p_motor, p_values[MOTOR_ANALOG_IC]);
//     Motor_ProcIabc(p_motor);
// }