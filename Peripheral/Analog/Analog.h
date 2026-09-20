
#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   Analog.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "HAL_ADC.h"
#include "Analog_ADC.h"
#include "_Analog_ADC.h"    /* Analog_ADC_ActivateBatch resolves the Hw activation */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/******************************************************************************/
/*
    Application handle
    Feature module holds the pointer. ADC owns the state.
    handle requires at least one dereference, either P_ADC or P_CONVERSION_STATE
*/
/******************************************************************************/
/* Analog_VirtualChannel */
typedef const struct Analog_Conversion
{
    Analog_ADC_T * P_ADC;
    analog_channel_t CHANNEL;
    // analog_mask_t
    // Analog_Capture_T CAPTURE;
    // void * P_CONTEXT;
    /* reserve interface for extension */
    // Analog_Options_T OPTIONS;
}
Analog_Conversion_T;


static inline void Analog_Conversion_Mark(const Analog_Conversion_T * p_conv) { Analog_ADC_MarkConversion(p_conv->P_ADC, p_conv->CHANNEL); }
static inline bool Analog_Conversion_IsMarked(const Analog_Conversion_T * p_conv) { return Analog_ADC_IsMarked(p_conv->P_ADC, p_conv->CHANNEL); }

/* resolve at runtime */
static inline Analog_ConversionChannel_T * Analog_Conversion_ChannelOf(const Analog_Conversion_T * p_conv) { return Analog_ADC_ConversionOf(p_conv->P_ADC, p_conv->CHANNEL); }

static inline adc_result_t Analog_Conversion_GetResult(const Analog_Conversion_T * p_conv) { return Analog_Conversion_ChannelOf(p_conv)->P_CONVERSION_STATE->Result; }
static inline void Analog_Conversion_ClearResult(const Analog_Conversion_T * p_conv) { Analog_Conversion_ChannelOf(p_conv)->P_CONVERSION_STATE->Result = 0U; }

// uniform handling for dma
// static inline adc_result_t Analog_Conversion_GetResult(const Analog_Conversion_T * p_conv) { return Analog_ADC_ChannelResultOf(p_conv->P_ADC, p_conv->CHANNEL); }

/******************************************************************************/
/*
    Unsynchronize Activation
*/
/******************************************************************************/
// static void Analog_Conversion_Activate(const Analog_Conversion_T * p_conversion)
// {
//     _Analog_ADC_StartConversion(p_conversion->P_ADC, p_conversion->P_CONVERSION_CHANNEL, 1U);
// }

// static void Analog_ActivateConversions(const Analog_Conversion_T * p_conversions, uint32_t markers)
// {
//     // if (Analog_ADC_ReadIsActive(p_conversion->P_ADC) == false)
    // {
    //     Analog_ADC_StartConversions(p_conversion->P_ADC, p_conversion->P_CONVERSION_CHANNEL, markers);
    // }
    // else
    // {
    //     Analog_Conversion_Mark(p_conversion); // mark each
    // }
// }





/******************************************************************************/
/*
    Multi ADC Batch
    Channel sets on separate ADCs, converted on a common trigger, joined into 1 completion.
    For channels that must sample simultaneously but are wired to different converters.

    Composed over the per ADC batch. Each part batch's CAPTURE is ADC_MultiBatch_OnPartComplete,
    with the part as context. Parts complete in their own ADC ISRs; the last one runs ON_COMPLETE.

    A part is the pair [ADC, BatchId]. Each ADC keeps its own batch table, so a part batch is
    selectable on its own as well, e.g. for a board where only 1 ADC carries the channel set.

    Part ISRs must share priority. The marker update is a read modify write.

        static const ADC_MultiBatch_T MOTOR0_I_BATCH;    // tentative definition

        static const ADC_MultiBatchPart_T MOTOR0_I_PARTS[] =
        {
            [0U] = ADC_MULTI_BATCH_PART_INIT(0U, &ANALOG_ADCS[0U], ADC0_BATCH_IAB, &MOTOR0_I_BATCH),
            [1U] = ADC_MULTI_BATCH_PART_INIT(1U, &ANALOG_ADCS[1U], ADC1_BATCH_IC,  &MOTOR0_I_BATCH),
        };

        static const ADC_MultiBatch_T MOTOR0_I_BATCH = ADC_MULTI_BATCH_INIT(MOTOR0_I_PARTS, 2U, Motor_Analog_CaptureIabc, &Motors[0U]);

        // In each ADC's batch table
        [ADC0_BATCH_IAB] = ADC_CONVERSION_BATCH_INIT_PART(0U, 2U, &MOTOR0_I_PARTS[0U]),
        [ADC1_BATCH_IC]  = ADC_CONVERSION_BATCH_INIT_PART(0U, 1U, &MOTOR0_I_PARTS[1U]),
*/
/******************************************************************************/
typedef struct ADC_MultiBatchState
{
    volatile analog_mask_t CompleteMarkers; /* 1 << PartIndex. Cleared on join */
}
ADC_MultiBatchState_T;

typedef const struct ADC_MultiBatchPart
{
    const Analog_ADC_T * P_ADC;
    // ADC_ConversionBatch_T * P_BATCH;
    uint8_t BATCH_ID;                       /* Index into P_ADC->P_CONVERSION_BATCHS */
    const struct ADC_MultiBatch * P_BATCH;  /* Join */
    analog_mask_t PART_MARKER;              /* 1 << PartIndex */
}
ADC_MultiBatchPart_T;

typedef const struct ADC_MultiBatch
{
    const ADC_MultiBatchPart_T * P_PARTS;
    uint8_t PART_COUNT;
    ADC_MultiBatchState_T * P_STATE;
    Analog_Callback_T ON_COMPLETE;  /* All parts captured. Results in each ADC's P_CHANNEL_RESULTS */
    void * P_CONTEXT;
}
ADC_MultiBatch_T;

static inline analog_mask_t _ADC_MultiBatch_PartMarkers(const ADC_MultiBatch_T * p_batch) { return ((1UL << p_batch->PART_COUNT) - 1UL); }

/*
    Registered as each part batch's CAPTURE. Runs in that ADC's complete ISR.
    Results stay in each ADC's buffer, ON_COMPLETE reads them through Analog_Conversion_T handles.
*/
static void ADC_MultiBatch_OnPartComplete(const ADC_MultiBatchPart_T * p_part, const volatile adc_result_t * p_results, uint8_t count)
{
    (void)p_results; (void)count;

    const ADC_MultiBatch_T * p_batch = p_part->P_BATCH;
    analog_mask_t markers = (p_batch->P_STATE->CompleteMarkers | p_part->PART_MARKER);
    bool isComplete = (markers == _ADC_MultiBatch_PartMarkers(p_batch));

    p_batch->P_STATE->CompleteMarkers = isComplete ? 0UL : markers;
    if (isComplete && (p_batch->ON_COMPLETE != NULL)) { p_batch->ON_COMPLETE(p_batch->P_CONTEXT); }
}

#define ADC_MULTI_BATCH_STATE_ALLOC() (&(ADC_MultiBatchState_T){})

#define ADC_MULTI_BATCH_INIT(p_Parts, PartCount, OnComplete, p_Context) (ADC_MultiBatch_T) \
    { .P_PARTS = (p_Parts), .PART_COUNT = (PartCount), .P_STATE = ADC_MULTI_BATCH_STATE_ALLOC(), .ON_COMPLETE = (Analog_Callback_T)(OnComplete), .P_CONTEXT = (p_Context), }

#define ADC_MULTI_BATCH_PART_INIT(PartIndex, p_Adc, BatchId, p_Batch) (ADC_MultiBatchPart_T) \
    { .P_ADC = (p_Adc), .BATCH_ID = (BatchId), .P_BATCH = (p_Batch), .PART_MARKER = (1UL << (PartIndex)), }

/* Entry in an ADC's batch table. Completion joins the multi batch instead of capturing directly */
#define ADC_CONVERSION_BATCH_INIT_PART(IdStart, Count, p_Part) (ADC_ConversionBatch_T) \
    { .ID_START = (IdStart), .COUNT = (Count), .CHANNELS = ANALOG_MASK_RANGE(IdStart, Count), .CAPTURE = (Analog_CaptureBatch_T)ADC_MultiBatch_OnPartComplete, .P_CONTEXT = (void *)(p_Part), }

/*
    Deferred. Each part applies on its own ADC's next completion, before that ADC's next trigger.
    Any thread. Last request wins.
*/
static inline void ADC_MultiBatch_SetBatch(const ADC_MultiBatch_T * p_batch)
{
    for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++) { Analog_ADC_SetBatch(p_batch->P_PARTS[iPart].P_ADC, p_batch->P_PARTS[iPart].BATCH_ID); }
}

/*
    Immediate. On init, or while the trigger source is inactive.
*/
static inline void ADC_MultiBatch_ActivateBatch(const ADC_MultiBatch_T * p_batch)
{
    p_batch->P_STATE->CompleteMarkers = 0UL;
    for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++) { Analog_ADC_ActivateBatch(p_batch->P_PARTS[iPart].P_ADC, p_batch->P_PARTS[iPart].BATCH_ID); }
}

/* All parts converting this batch. A deferred selection is not applied until each ADC completes */
static inline bool ADC_MultiBatch_IsActive(const ADC_MultiBatch_T * p_batch)
{
    bool isActive = true;
    for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++)
    {
        isActive &= (Analog_ADC_GetActiveBatch(p_batch->P_PARTS[iPart].P_ADC) == &p_batch->P_PARTS[iPart].P_ADC->P_CONVERSION_BATCHS[p_batch->P_PARTS[iPart].BATCH_ID]);
    }
    return isActive;
}
