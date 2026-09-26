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
    @brief  Sets over 1..N ADCs, converted by 1 trigger and joined. Board layer.
*/
/******************************************************************************/
#include "ADC_Sequence.h"  /* A part is a sequence. It already names its own ADC */

/******************************************************************************/
/*
    A batch is what 1 trigger converts: 1..N parts, a part being 1 sequence on 1 ADC. The single
    ADC case is a batch of 1 part, so nothing above distinguishes 1 ADC from several.

    The join reads [CompleteFlags], the same word a lone sequence reads:
        - each part's transfer ISR flags that part's channels
        - the last to land finds every part flagged, takes them all, and runs ON_COMPLETE
    Taking is what makes the join an edge, so the trigger holds no completion count and no ADC
    writes another's state.

    [ADC_BatchTrigger_T] belongs to the trigger, not to a batch. The batches that alternate on 1
    trigger share 1 state, so only 1 of them is ever in flight, and:
        - a selection is 1 write. Batches cannot end up split across a trigger
        - the join applies it, where every part has landed and its ADCs are idle
    Batches sharing a state must span the same ADCs. The switch reprograms them all.

    A part whose completion is dropped leaves the others flagged, and the join runs one trigger
    late. The results buffer is written in place, so what it reads then is the newest of each.

        static ADC_BatchTrigger_T MOTOR0_TRIGGER;   // MOTOR0_I_BATCH and MOTOR0_V_BATCH alternate on it

        static const ADC_Sequence_T MOTOR0_I_PARTS[] =
        {
            [0U] = ADC_SEQUENCE_FIELDS(&ADCS[0U], ADC_MASK_RANGE(ADC0_IA, 2U), NULL, NULL),
            [1U] = ADC_SEQUENCE_FIELDS(&ADCS[1U], ADC_MASK(ADC1_IC), NULL, NULL),
        };

        static ADC_Batch_T MOTOR0_I_BATCH = ADC_BATCH_INIT(MOTOR0_I_PARTS, Motor_OnBatchComplete, &Motors[0U]);

        // The Board's ISRs. 1 per ADC of the trigger, at 1 priority
        void BOARD_ADC0_DMA_ISR(void) { Board_ADC0_ClearComplete(); ADC_BatchTrigger_OnComplete_ISR(&MOTOR0_TRIGGER, &ADCS[0U]); }
        void BOARD_ADC1_DMA_ISR(void) { Board_ADC1_ClearComplete(); ADC_BatchTrigger_OnComplete_ISR(&MOTOR0_TRIGGER, &ADCS[1U]); }
*/
/******************************************************************************/
typedef const struct ADC_Batch
{
    ADC_Sequence_T * P_PARTS;     /* [Part]. Each names its own ADC */
    uint8_t PART_COUNT;
    ADC_Callback_T ON_COMPLETE;
    void * P_CONTEXT;
}
ADC_Batch_T;

/* Derives the part count from the array, and casts the handler to its registration type */
#define ADC_BATCH_INIT(Parts, OnComplete, p_Context) (ADC_Batch_T) \
    { .P_PARTS = (Parts), .PART_COUNT = (sizeof(Parts) / sizeof(ADC_Sequence_T)), .ON_COMPLETE = (ADC_Callback_T)(OnComplete), .P_CONTEXT = (void *)(p_Context), }

typedef struct ADC_BatchTrigger
{
    ADC_Batch_T * volatile p_Active;
    ADC_Batch_T * volatile p_Next;
}
ADC_BatchTrigger_T;

#define ADC_BATCH_TRIGGER_ALLOC() (&(ADC_BatchTrigger_T){})

/******************************************************************************/
/*
    Parts
*/
/******************************************************************************/
/* The part this ADC converts in [p_batch]. NULL where the ADC is not one of its parts */
static inline const ADC_Sequence_T * _ADC_Batch_PartOf(ADC_Batch_T * p_batch, ADC_T * p_adc)
{
    for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++) { if (p_batch->P_PARTS[iPart].P_ADC == p_adc) { return &p_batch->P_PARTS[iPart]; } }
    return NULL;
}

/* Peek every part before taking any, so a partial set keeps its flags */
static inline bool _ADC_Batch_IsEachComplete(ADC_Batch_T * p_batch)
{
    for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++) { if (ADC_Sequence_IsComplete(&p_batch->P_PARTS[iPart]) == false) { return false; } }
    return true;
}

static inline void _ADC_Batch_TakeEach(ADC_Batch_T * p_batch)
{
    for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++) { (void)ADC_TakeFlags(p_batch->P_PARTS[iPart].P_ADC, p_batch->P_PARTS[iPart].CHANNELS); }
}

/* Sends every part to its Hw. Only where every ADC of the trigger is idle: on init, or in the join */
static inline void _ADC_Batch_ActivateEachDma(ADC_Batch_T * p_batch)
{
    for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++) { ADC_Sequence_ActivateDma(&p_batch->P_PARTS[iPart]); }
}

/******************************************************************************/
/*
    Join
*/
/******************************************************************************/
/* Joins where every part has landed. Takes the whole set, so the handler runs once */
static inline bool ADC_Batch_TakeComplete(ADC_Batch_T * p_batch)
{
    bool isComplete = _ADC_Batch_IsEachComplete(p_batch);
    if (isComplete == true) { _ADC_Batch_TakeEach(p_batch); }
    return isComplete;
}

static inline bool ADC_Batch_Poll(ADC_Batch_T * p_batch)
{
    bool isComplete = ADC_Batch_TakeComplete(p_batch);
    if ((isComplete == true) && (p_batch->ON_COMPLETE != NULL)) { p_batch->ON_COMPLETE(p_batch->P_CONTEXT); }
    return isComplete;
}

/*!
    @brief  1 per ADC of the trigger, in its transfer complete ISR. The caller clears the Hw flag.
            The last part to land runs the join. A part's own COMPLETE is not used.

    Part ISRs must share a priority: the join peeks every part, then takes them all.
*/
static inline void ADC_BatchTrigger_OnComplete_ISR(ADC_BatchTrigger_T * p_trigger, ADC_T * p_adc)
{
    ADC_Batch_T * p_active = p_trigger->p_Active;
    assert(p_active != NULL);   /* A batch must be activated before the trigger is enabled */

    const ADC_Sequence_T * p_part = _ADC_Batch_PartOf(p_active, p_adc);
    if (p_part != NULL) { ADC_OnCompleteTransfer_ISR(p_adc, p_part->CHANNELS); }    /* This part landed. A part of a batch no longer active is ignored */

    if (ADC_Batch_Poll(p_active) == true)
    {
        /*
            The selection applies here, 1 place reprogramming every part, so they cannot end up
            split across a trigger. Read after the handler, so a set selected from there applies now.
        */
        ADC_Batch_T * p_next = p_trigger->p_Next;
        if (p_next != p_active) { _ADC_Batch_ActivateEachDma(p_next); p_trigger->p_Active = p_next; }
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
static inline void ADC_BatchTrigger_ActivateDma(ADC_BatchTrigger_T * p_trigger, ADC_Batch_T * p_batch)
{
    _ADC_Batch_ActivateEachDma(p_batch);
    p_trigger->p_Active = p_batch;
    p_trigger->p_Next = p_batch;
}

/*
    Deferred, 1 store. Any thread, last request wins.
    Applied in the join, before the trigger that follows it.
*/
static inline void ADC_BatchTrigger_Select(ADC_BatchTrigger_T * p_trigger, ADC_Batch_T * p_batch) { p_trigger->p_Next = p_batch; }

/* Converting. A deferred selection is not applied until the active batch joins */
static inline bool ADC_BatchTrigger_IsActive(const ADC_BatchTrigger_T * p_trigger, ADC_Batch_T * p_batch) { return (p_trigger->p_Active == p_batch); }

/* Selected. Active already, or applied at the next join */
static inline bool ADC_BatchTrigger_IsSelected(const ADC_BatchTrigger_T * p_trigger, ADC_Batch_T * p_batch) { return (p_trigger->p_Next == p_batch); }
