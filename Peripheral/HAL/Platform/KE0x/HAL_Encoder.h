/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   HAL_Encoder.h
    @author FireSourcery
    @brief     Encoder Timer Counter HAL for S32K
    @version V0
*/
/******************************************************************************/
#ifndef HAL_ENCODER_PLATFORM_H
#define HAL_ENCODER_PLATFORM_H

#include "Include.h"

#include <stdint.h>
#include <stdbool.h>

typedef FTM_Type HAL_Encoder_Pin_T;
typedef FTM_Type HAL_Encoder_Timer_T;
typedef FTM_Type HAL_Encoder_Counter_T;

/* Clear interrupt. Read-after-write sequence to guarantee required serialization of memory operations */
static inline void HAL_Encoder_ClearTimerOverflow(HAL_Encoder_Timer_T * p_encoder)          { p_encoder->SC &= ~FTM_SC_TOF_MASK; (void)p_encoder->SC; }
static inline bool HAL_Encoder_ReadTimerOverflow(const HAL_Encoder_Timer_T * p_encoder)     { return p_encoder->SC & FTM_SC_TOF_MASK; }
static inline uint32_t HAL_Encoder_ReadTimer(const HAL_Encoder_Timer_T * p_encoder)         { return p_encoder->CNT; }
static inline void HAL_Encoder_WriteTimer(HAL_Encoder_Timer_T * p_encoder, uint32_t count)  { p_encoder->CNT = FTM_CNT_COUNT(count); }

static inline void HAL_Encoder_Enable(HAL_Encoder_Timer_T * p_encoder)                      { p_encoder->SC |= FTM_SC_CLKS(0b01U); }

#ifndef HAL_ENCODER_CLOCK_SOURCE_FREQ
#define HAL_ENCODER_CLOCK_SOURCE_FREQ CPU_FREQ
#endif

// #ifndef HAL_ENCODER_TIMER_FREQ
// #define HAL_ENCODER_TIMER_FREQ (0xFFFFFFFFU/1000U)
// #endif

/*!
    @return freq set

    Prescale Factor Selection
    000 Divide by 1
    001 Divide by 2
    010 Divide by 4
    011 Divide by 8
    100 Divide by 16
    101 Divide by 32
    110 Divide by 64
    111 Divide by 128
*/
static inline uint32_t HAL_Encoder_InitTimerFreq(HAL_Encoder_Timer_T * p_encoder, uint32_t freq)
{
    uint32_t preScalerValue = HAL_ENCODER_CLOCK_SOURCE_FREQ / freq;
    uint8_t preScaler = 0U;
    while((preScalerValue >> preScaler) > 1U) { preScaler++; } /* log2 */
    p_encoder->SC &= ~FTM_SC_CLKS_MASK;
    p_encoder->SC = (p_encoder->SC & ~FTM_SC_PS_MASK) | FTM_SC_PS(preScaler);
    p_encoder->SC |= FTM_SC_CLKS(0b01U);
    return HAL_ENCODER_CLOCK_SOURCE_FREQ / ((uint32_t)1UL << preScaler);
}

static inline void HAL_Encoder_InitTimer(HAL_Encoder_Timer_T * p_encoder)
{
    p_encoder->MOD = FTM_MOD_MOD(0xFFFFU);
    p_encoder->SC |= FTM_SC_CLKS(0b01U);
    // HAL_Encoder_InitTimerFreq(p_encoder, HAL_ENCODER_TIMER_FREQ);
}

/*
    Counter Mode - KE0x No Counter/quadrature decoder mode support
*/
static inline bool HAL_Encoder_ReadCounterOverflow(const HAL_Encoder_Counter_T * p_encoder) { (void)p_encoder; }
static inline void HAL_Encoder_ClearCounterOverflow(HAL_Encoder_Counter_T * p_encoder) { (void)p_encoder; }
static inline uint32_t HAL_Encoder_ReadCounter(const HAL_Encoder_Counter_T * p_encoder) { (void)p_encoder; }
static inline void HAL_Encoder_WriteCounter(HAL_Encoder_Counter_T * p_encoder, uint32_t count) { (void)p_encoder; }
static inline void HAL_Encoder_WriteCounterMax(HAL_Encoder_Counter_T * p_encoder, uint32_t max) { (void)p_encoder; }
static inline void HAL_Encoder_InitCounter(HAL_Encoder_Counter_T * p_encoder) { (void)p_encoder; }
/* Quadrature Decoder Counter */
static inline bool HAL_Encoder_ReadCounterDirection(const HAL_Encoder_Counter_T * p_encoder) { (void)p_encoder; return (0U); }
static inline bool HAL_Encoder_ReadCounterOverflowIncrement(const HAL_Encoder_Counter_T * p_encoder) { (void)p_encoder; return (0U); }
static inline bool HAL_Encoder_ReadCounterOverflowDecrement(const HAL_Encoder_Counter_T * p_encoder) { (void)p_encoder; return (0U); }

/*
    Emulated via edge interrupt
*/
static inline bool HAL_Encoder_ReadPinInterrupt(const HAL_Encoder_Pin_T * p_encoder, uint32_t phaseId)   { return (p_encoder->CONTROLS[phaseId].CnSC & FTM_CnSC_CHF_MASK); }
static inline void HAL_Encoder_ClearPinInterrupt(HAL_Encoder_Pin_T * p_encoder, uint32_t phaseId)        { p_encoder->CONTROLS[phaseId].CnSC &= ~FTM_CnSC_CHF_MASK; }

/*
    ELSnB:ELSnA
    01 Input Capture Capture on Rising Edge Only
    10 Capture on Falling Edge Only
*/
static inline void HAL_Encoder_EnablePinInterrupt(HAL_Encoder_Pin_T * p_encoder, uint32_t phaseId)          { p_encoder->CONTROLS[phaseId].CnSC |= FTM_CnSC_CHIE_MASK; }
static inline void HAL_Encoder_DisablePinInterrupt(HAL_Encoder_Pin_T * p_encoder, uint32_t phaseId)         { p_encoder->CONTROLS[phaseId].CnSC &= ~FTM_CnSC_CHIE_MASK; }
static inline void HAL_Encoder_InitPinInterruptDualEdge(HAL_Encoder_Pin_T * p_encoder, uint32_t phaseId)     { p_encoder->CONTROLS[phaseId].CnSC = FTM_CnSC_CHIE_MASK | FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK; }
static inline void HAL_Encoder_InitPinInterruptFallingEdge(HAL_Encoder_Pin_T * p_encoder, uint32_t phaseId)  { p_encoder->CONTROLS[phaseId].CnSC = FTM_CnSC_CHIE_MASK | FTM_CnSC_ELSB_MASK; }
static inline void HAL_Encoder_InitPinInterruptRisingEdge(HAL_Encoder_Pin_T * p_encoder, uint32_t phaseId)   { p_encoder->CONTROLS[phaseId].CnSC = FTM_CnSC_CHIE_MASK | FTM_CnSC_ELSA_MASK; }

// static inline void HAL_Encoder_EnablePinInterrupt(HAL_Encoder_Pin_T * p_encoder, uint32_t phaseId)
// {
//     switch((uintptr_t)p_encoder)
//     {
//         case 0: NVIC_EnableIRQ(FTM0_IRQn); break;
//         case 1: NVIC_EnableIRQ(FTM1_IRQn); break;
//         case 2: NVIC_EnableIRQ(FTM2_IRQn); break;
//     }
// }

#endif
