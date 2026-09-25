#pragma once

/*
    Host test double. Software activation converts SC1[0] immediately, so the marked path can be
    stepped without hardware. The Hw sequenced path converts on Test_Trigger, in ADC_Batch_Test.c.
*/
#include "HAL_Types.h"

static inline uint32_t HAL_ADC_ReadResult(const HAL_ADC_T * p_hal, uint32_t pinChannel) { (void)pinChannel; return p_hal->R[0U]; }
static inline void HAL_ADC_Activate(HAL_ADC_T * p_hal, uint32_t pinChannel) { p_hal->SC1[0U] = pinChannel; p_hal->IsActive = true; }

static inline void HAL_ADC_WriteFifoCount(HAL_ADC_T * p_hal, uint32_t count) { (void)p_hal; (void)count; }
static inline uint8_t HAL_ADC_ReadFifoCount(const HAL_ADC_T * p_hal) { (void)p_hal; return 1U; }
static inline void HAL_ADC_WriteFifoPin(HAL_ADC_T * p_hal, uint32_t pinChannel) { (void)p_hal; (void)pinChannel; }
static inline void HAL_ADC_ActivateFifo(HAL_ADC_T * p_hal, uint32_t pinChannel) { HAL_ADC_Activate(p_hal, pinChannel); }

static inline void HAL_ADC_Deactivate(HAL_ADC_T * p_hal) { p_hal->IsActive = false; }

static inline void HAL_ADC_EnableInterrupt(HAL_ADC_T * p_hal) { (void)p_hal; }
static inline void HAL_ADC_DisableInterrupt(HAL_ADC_T * p_hal) { (void)p_hal; }
static inline void HAL_ADC_ClearConversionCompleteFlag(HAL_ADC_T * p_hal) { p_hal->IsComplete = false; }
static inline bool HAL_ADC_ReadConversionCompleteFlag(const HAL_ADC_T * p_hal) { return p_hal->IsComplete; }
static inline bool HAL_ADC_ReadConversionActiveFlag(const HAL_ADC_T * p_hal) { return p_hal->IsActive; }

static inline void HAL_ADC_AbortConversion(HAL_ADC_T * p_hal) { p_hal->IsActive = false; }
static inline void HAL_ADC_EnableHwTrigger(HAL_ADC_T * p_hal) { (void)p_hal; }
static inline void HAL_ADC_DisableHwTrigger(HAL_ADC_T * p_hal) { (void)p_hal; }
static inline void HAL_ADC_EnableContinuousConversion(HAL_ADC_T * p_hal) { (void)p_hal; }
static inline void HAL_ADC_DisableContinuousConversion(HAL_ADC_T * p_hal) { (void)p_hal; }

static inline void HAL_ADC_Init(HAL_ADC_T * p_hal) { (void)p_hal; }

/* The Board's hook. On S32K1 this is HAL_PDB_ConfigChannel(PDB, channel, start, count) */
static inline void HAL_ADC_ActivateSequence(HAL_ADC_T * p_hal, uint32_t channelStart, uint32_t count)
{
    p_hal->RangeStart = (uint8_t)channelStart;
    p_hal->RangeCount = (uint8_t)count;
    p_hal->RangeWrites++;
}
