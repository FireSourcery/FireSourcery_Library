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

    A batch is 1..N parts. A part is 1 sequence on 1 ADC. The single ADC case is a batch of 1 part,
    so nothing above distinguishes 1 channel, 1 ADC, or several ADCs.

    The application does not hold a batch. It holds ADC_Conversion_T, which names the batch to request.

    Each part sequence registers ADC_Batch_OnPartComplete as its ON_COMPLETE, with the part as context.
    Parts complete in their own ADC ISRs, the last one runs the batch ON_COMPLETE.
    Part ISRs must share a priority, the marker update is a read modify write.

        static const ADC_Batch_T MOTOR0_I_BATCH;    // tentative definition

        static const ADC_BatchPart_T MOTOR0_I_PARTS[] =
        {
            [0U] = ADC_BATCH_PART_INIT(0U, &ADCS[0U], ADC0_SEQUENCE_IAB, &MOTOR0_I_BATCH),
            [1U] = ADC_BATCH_PART_INIT(1U, &ADCS[1U], ADC1_SEQUENCE_IC,  &MOTOR0_I_BATCH),
        };

        static const ADC_Batch_T MOTOR0_I_BATCH = ADC_BATCH_INIT(MOTOR0_I_PARTS, 2U, Motor_Analog_CaptureIabc, &Motors[0U]);

        // In each ADC's sequence table
        [ADC0_SEQUENCE_IAB] = ADC_SEQUENCE_INIT_PART(ADC_MASK_RANGE(ADC0_IA, 2U), &MOTOR0_I_PARTS[0U]),
        [ADC1_SEQUENCE_IC]  = ADC_SEQUENCE_INIT_PART(ADC_MASK(ADC1_IC), &MOTOR0_I_PARTS[1U]),
*/
/******************************************************************************/
#include "_ADC.h"
#include "ADC.h"


typedef struct ADC_BatchState
{
    volatile adc_mask_t CompleteMarkers; /* 1 << PartIndex. Cleared on join */
}
ADC_BatchState_T;

// typedef void (*ADC_CaptureBatch_T)(void * p_context, const volatile adc_result_t ** p_values, adc_mask_t * p_marker);

typedef const struct ADC_BatchPart
{
    const ADC_T * P_ADC;
    uint8_t SEQUENCE_ID;                /* Index into P_ADC->P_SEQUENCES */
    const struct ADC_Batch * P_BATCH;   /* Join */
    adc_mask_t PART_MARKER;             /* 1 << PartIndex */
}
ADC_BatchPart_T;

typedef const struct ADC_Batch
{
    const ADC_BatchPart_T * P_PARTS;
    uint8_t PART_COUNT;
    ADC_BatchState_T * P_STATE;
    ADC_Callback_T ON_COMPLETE;         /* All parts converted. Values read through ADC_Channel_T */
    void * P_CONTEXT;
}
ADC_Batch_T;

static inline adc_mask_t _ADC_Batch_PartMarkers(const ADC_Batch_T * p_batch) { return ((1UL << p_batch->PART_COUNT) - 1UL); }

/*
    Registered as each part sequence's ON_COMPLETE. Runs in that ADC's completion ISR.
    Markers, not a count: a part completing twice is idempotent, so a missed trigger on one ADC
    does not leave the join permanently skewed.
*/
static void ADC_Batch_OnPartComplete(const ADC_BatchPart_T * p_part)
{
    const ADC_Batch_T * p_batch = p_part->P_BATCH;
    adc_mask_t markers = (p_batch->P_STATE->CompleteMarkers | p_part->PART_MARKER);
    bool isComplete = (markers == _ADC_Batch_PartMarkers(p_batch));

    p_batch->P_STATE->CompleteMarkers = isComplete ? 0UL : markers;
    if (isComplete && (p_batch->ON_COMPLETE != NULL)) { p_batch->ON_COMPLETE(p_batch->P_CONTEXT); }
}

/******************************************************************************/
/*
    Request
*/
/******************************************************************************/
/*
    Deferred. Each part applies on its own ADC's next completion, before that ADC's next trigger.
    Any thread. Last request wins.

    Hw sequenced: the batch then converts on every trigger, until another batch is selected.
    Software: the batch converts once per request.
*/
static inline void ADC_Batch_Select(const ADC_Batch_T * p_batch)
{
    for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++) { ADC_SetSequence(p_batch->P_PARTS[iPart].P_ADC, p_batch->P_PARTS[iPart].SEQUENCE_ID); }
}

/*
    Immediate. On init, or while the trigger source is inactive.
*/
static inline void ADC_Batch_Activate(const ADC_Batch_T * p_batch)
{
    p_batch->P_STATE->CompleteMarkers = 0UL;
    for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++) { ADC_ActivateSequence(p_batch->P_PARTS[iPart].P_ADC, p_batch->P_PARTS[iPart].SEQUENCE_ID); }
}

/* All parts converting this batch. A deferred selection is not applied until each ADC completes */
static inline bool ADC_Batch_IsActive(const ADC_Batch_T * p_batch)
{
    bool isActive = true;
    for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++)
    {
        isActive &= ADC_IsSequenceActive(p_batch->P_PARTS[iPart].P_ADC, p_batch->P_PARTS[iPart].SEQUENCE_ID);
    }
    return isActive;
}
