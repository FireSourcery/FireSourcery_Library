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
//     @file   Analog_ADC_Batch.h
//     @author FireSourcery
//     @brief  Hardware sequenced conversion batches.

//     A batch is a fixed channel set on one ADC, converted and transferred as a unit by hardware,
//         e.g. Trigger -> PDB back-to-back pre-triggers -> SC1[slots] -> COCO -> DMA R[slots] -> P_CHANNEL_RESULTS[slots]
//     with 1 callback on complete.

//     Channel ID == SC1 slot == R[] index == P_CHANNEL_RESULTS index.
//     Unselected batch results remain valid, as the last converted values.

//     Board integration:
//         - Configure trigger and transfer resources. Transfer destination P_CHANNEL_RESULTS.
//         - Provide Analog_ADC_T.SELECT_BATCH, routing triggers to a channel mask.
//         - Transfer complete ISR: clear the Hw flag, then Analog_ADC_OnBatchComplete_ISR().
// */
// /******************************************************************************/
// #include "Analog_ADC.h"

// #include <stdint.h>
// #include <stdbool.h>
// #include <stddef.h>
// #include <assert.h>

// #define ANALOG_MASK_RANGE(FirstChannel, Count) ((analog_mask_t)(((1UL << (Count)) - 1UL) << (FirstChannel)))

// static inline bool Analog_Mask_IsContiguous(analog_mask_t mask) { return ((mask + (mask & (0U - mask))) & mask) == 0U; }


// // static inline void HAL_ADC_InitDmaBatch(HAL_ADC_T * p_hal, const adc_pin_t * p_pins, volatile adc_result_t * p_results, uint8_t count, uint32_t completeMask);
// // // static inline void HAL_ADC_ReadBatch(HAL_ADC_T * p_hal, const adc_pin_t * p_pins, uint8_t count, uint32_t completeMask, volatile adc_result_t * p_results);
// // static inline void HAL_ADC_ActivateBatch(HAL_ADC_T * p_hal, uint32_t slotMask);
// // // static inline void HAL_ADC_ActivateBatch(HAL_ADC_T * p_hal, const adc_pin_t * p_pins, uint8_t counts);
// // static inline void HAL_ADC_ClearBatchComplete(HAL_ADC_T * p_hal, const adc_pin_t * p_pins, uint8_t count);
// // static inline bool HAL_ADC_ReadBatchComplete(HAL_ADC_T * p_hal, const adc_pin_t * p_pins, uint8_t count);

// /******************************************************************************/
// /*
//     Per ADC Batch
// */
// /******************************************************************************/
// typedef const struct ADC_ConversionBatch
// {
//     // Analog_ConversionChannel_T * P_CONVERSION_CHANNELS;
//     analog_mask_t CHANNELS;         /* Channel IDs. Contiguous for Hw sequencing */
//     Analog_Callback_T ON_COMPLETE;  /* Results in P_CHANNEL_RESULTS. NULL for scan only */
//     void * P_CONTEXT;
// }
// ADC_ConversionBatch_T;

// /* Handler declared with its concrete context type */
// #define ANALOG_ADC_BATCH_INIT(Channels, OnComplete, p_Context) (ADC_ConversionBatch_T) \
//     { .CHANNELS = (Channels), .ON_COMPLETE = (Analog_Callback_T)(OnComplete), .P_CONTEXT = (p_Context), }

// static inline const ADC_ConversionBatch_T * Analog_ADC_GetActiveBatch(const Analog_ADC_T * p_adc) { return p_adc->P_ADC_STATE->p_ActiveBatch; }
// static inline bool Analog_ADC_IsBatchActive(const Analog_ADC_T * p_adc, const ADC_ConversionBatch_T * p_batch) { return (p_adc->P_ADC_STATE->p_ActiveBatch == p_batch); }
// static inline adc_result_t Analog_ADC_ChannelResultOf(const Analog_ADC_T * p_adc, analog_channel_t channel) { return p_adc->P_CHANNEL_RESULTS[channel]; }

// static inline void _Analog_ADC_ActivateBatch(const Analog_ADC_T * p_adc, const ADC_ConversionBatch_T * p_batch)
// {
//     // HAL_ADC_ActivateBatch(p_adc->P_HAL_ADC, p_batch->CHANNELS);
//     p_adc->P_ADC_STATE->p_ActiveBatch = p_batch;
// }

// /*
//     Immediate. On init, or while the trigger source is inactive.
// */
// static inline void Analog_ADC_StartBatch(const Analog_ADC_T * p_adc, const ADC_ConversionBatch_T * p_batch)
// {
//     assert(Analog_Mask_IsContiguous(p_batch->CHANNELS) && ((p_batch->CHANNELS >> p_adc->CHANNEL_COUNT) == 0U));
//     p_adc->P_ADC_STATE->p_NextBatch = p_batch;
//     _Analog_ADC_ActivateBatch(p_adc, p_batch);
// }

// /*
//     Deferred. Applied in the complete ISR of the active batch, before the next trigger.
//     Any thread. Last request wins.
// */
// static inline void Analog_ADC_SelectBatch(const Analog_ADC_T * p_adc, const ADC_ConversionBatch_T * p_batch) { p_adc->P_ADC_STATE->p_NextBatch = p_batch; }

// /*!
//     @brief Sequence complete. e.g. DMA major loop complete. Caller clears the Hw flag.

//     ON_COMPLETE runs before the pending selection is applied; a selection made within ON_COMPLETE takes effect on the next trigger.
//     ON_COMPLETE and the selection must finish before the next trigger.
// */
// static inline void Analog_ADC_OnBatchComplete_ISR(const Analog_ADC_T * p_adc)
// {
//     Analog_ADC_State_T * p_state = p_adc->P_ADC_STATE;
//     const ADC_ConversionBatch_T * p_batch = p_state->p_ActiveBatch;

//     if (p_batch->ON_COMPLETE != NULL) { p_batch->ON_COMPLETE(p_batch->P_CONTEXT); }

//     const ADC_ConversionBatch_T * p_next = p_state->p_NextBatch; /* snapshot, written by other threads */
//     if (p_next != p_batch) { _Analog_ADC_ActivateBatch(p_adc, p_next); }
// }


// /******************************************************************************/
// /*
//     Multi ADC Batch
//     Per ADC parts on a common trigger. 1 callback when all parts complete.

//     Composed over per ADC batches: each part's ON_COMPLETE is Analog_Batch_OnPartComplete, with the Analog_Batch_T as context.
//     Each part completes once per trigger. Part ISRs must share priority (non-nesting) for the count update.

//     static const Analog_Batch_T BATCH_IAB;
//     static const Analog_BatchPart_T BATCH_IAB_PARTS[] =
//     {
//         ANALOG_BATCH_PART_INIT(&ANALOG_ADCS[0U], ANALOG_MASK_RANGE(0U, 2U), &BATCH_IAB),
//         ANALOG_BATCH_PART_INIT(&ANALOG_ADCS[1U], ANALOG_MASK_RANGE(0U, 2U), &BATCH_IAB),
//     };
//     static const Analog_Batch_T BATCH_IAB = ANALOG_BATCH_INIT(BATCH_IAB_PARTS, 2U, Motor_Analog_CaptureIab, &MOTORS[0U]);
// */
// /******************************************************************************/
// typedef struct Analog_BatchState
// {
//     volatile uint8_t CompleteCount;
// }
// Analog_BatchState_T;

// typedef const struct Analog_BatchPart
// {
//     const Analog_ADC_T * P_ADC;
//     const ADC_ConversionBatch_T * P_ADC_BATCH;
// }
// Analog_BatchPart_T;

// typedef struct
// {
//     Analog_Conversion_T * P_ADC_BATCHES;
//     uint8_t ADC_COUNT;
// } Analog_BatchTable_T;

// typedef const struct Analog_Batch
// {
//     const Analog_BatchPart_T * P_PARTS;
//     // const struct { Analog_ADC_T * P_ADC; const ADC_ConversionBatch_T * P_ADC_BATCH;  Analog_BatchState_T * P_STATE; } P_PARTS;
//     uint8_t PART_COUNT;
//     Analog_BatchState_T * P_STATE;
//     Analog_Callback_T ON_COMPLETE;
//     void * P_CONTEXT;
// }
// Analog_Batch_T;

// /* Part complete. Registered as each part's ON_COMPLETE */
// static void Analog_Batch_OnPartComplete(const Analog_Batch_T * p_batch)
// {
//     Analog_BatchState_T * p_state = p_batch->P_STATE;
//     uint8_t count = p_state->CompleteCount + 1U;

//     p_state->CompleteCount = (count < p_batch->PART_COUNT) ? count : 0U;
//     if ((count == p_batch->PART_COUNT) && (p_batch->ON_COMPLETE != NULL)) { p_batch->ON_COMPLETE(p_batch->P_CONTEXT); }
// }

// #define ANALOG_BATCH_STATE_ALLOC() (&(Analog_BatchState_T){})

// #define ANALOG_BATCH_INIT(p_Parts, PartCount, OnComplete, p_Context) (Analog_Batch_T) \
//     { .P_PARTS = (p_Parts), .PART_COUNT = (PartCount), .P_STATE = ANALOG_BATCH_STATE_ALLOC(), .ON_COMPLETE = (Analog_Callback_T)(OnComplete), .P_CONTEXT = (p_Context), }

// #define ANALOG_BATCH_PART_INIT(p_Adc, Channels, p_Batch) (Analog_BatchPart_T) \
//     { .P_ADC = (p_Adc), .P_ADC_BATCH = &ANALOG_ADC_BATCH_INIT(Channels, Analog_Batch_OnPartComplete, (void *)(p_Batch)), }

// static inline void Analog_Batch_Select(const Analog_Batch_T * p_batch)
// {
//     for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++) { Analog_ADC_SelectBatch(p_batch->P_PARTS[iPart].P_ADC, p_batch->P_PARTS[iPart].P_ADC_BATCH); }
// }

// /* Immediate. On init, or while the trigger source is inactive */
// static inline void Analog_Batch_Start(const Analog_Batch_T * p_batch)
// {
//     p_batch->P_STATE->CompleteCount = 0U;
//     for (uint8_t iPart = 0U; iPart < p_batch->PART_COUNT; iPart++) { Analog_ADC_StartBatch(p_batch->P_PARTS[iPart].P_ADC, p_batch->P_PARTS[iPart].P_ADC_BATCH); }
// }



