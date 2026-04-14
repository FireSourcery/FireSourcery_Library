// #pragma once

// /******************************************************************************/
// /*!
//     @section LICENSE

//     Copyright (C) 2025 FireSourcery

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
//     @file   PulseTimer.h
//     @author FireSourcery
//     @brief  Thin pulse timer/timer HAL wrapper.
// */
// /******************************************************************************/
// #include "HAL_ClockTimer.h"

// #include <stdint.h>
// #include <stdbool.h>

// #ifndef PULSE_TIMER_MAX
// #define PULSE_TIMER_MAX (0xFFFFU)
// #endif

// /*
//     Square wave counter
// */
// /******************************************************************************/
// /*
//     State - Timer-specific only
//     Counter/FreqD state lives in AngleCounter_T (math layer)
// */
// /******************************************************************************/
// typedef struct PulseTimer_State
// {
//     /* Timer Capture */
//     uint32_t DeltaT;        /* Timer counts between 2 pulse edges. Units in raw timer ticks */
//     uint32_t SampleTh;       /* Ts timer value at last sample */
//     uint32_t SampleT;
// }
// PulseTimer_State_T;

// /******************************************************************************/
// /*
//     Const Instance
// */
// /******************************************************************************/
// typedef const struct PulseTimer
// {
//     HAL_ClockTimer_T * P_HAL_TIMER;      /* DeltaT Timer */
//     uint32_t TIMER_FREQ;
//     uint32_t SAMPLE_FREQ;       /* Speed sample freq (e.g. 1kHz) */
//     uint32_t SAMPLE_TIME;       /* TIMER_FREQ / SAMPLE_FREQ */
//     PulseTimer_State_T * P_STATE;
// }
// PulseTimer_T;

// #define PULSE_TIMER_INIT(p_TimerHal, TimerFreq, SampleFreq, p_State) (PulseTimer_T) \
// {                                                           \
//     .P_HAL_TIMER        = (p_TimerHal),                     \
//     .TIMER_FREQ         = (TimerFreq),                      \
//     .SAMPLE_FREQ        = (SampleFreq),                     \
//     .SAMPLE_TIME        = (TimerFreq) / (SampleFreq),       \
//     .P_STATE            = (p_State),                        \
// }

// #define PULSE_TIMER_STATE_ALLOC() (&(PulseTimer_State_T){0})

// /******************************************************************************/
// /*!
//     @brief  Capture Sample Time - Pulse Edge Polling/ISR
// */
// /******************************************************************************/
// /*
//     Capture DeltaT - Pulse Edge Polling/ISR
//     [0:PULSE_TIMER_MAX] => (0xFFFF / TIMER_FREQ) [seconds]
// */
// static inline uint32_t PulseTimer_CaptureDeltaT(const PulseTimer_T * p_timer)
// {
//     PulseTimer_State_T * p_state = p_timer->P_STATE;
//     uint32_t timerValue;
//     if (!HAL_ClockTimer_ReadOverflow(p_timer->P_HAL_TIMER))
//     {
//         timerValue = HAL_ClockTimer_Read(p_timer->P_HAL_TIMER);
//     }
//     else
//     {
//         timerValue = PULSE_TIMER_MAX;
//         HAL_ClockTimer_ClearOverflow(p_timer->P_HAL_TIMER);
//     }
//     HAL_ClockTimer_Write(p_timer->P_HAL_TIMER, 0U);
//     return timerValue;
// }

// /*
//     Per SAMPLE_TIME
// */
// static inline uint32_t PulseTimer_CaptureSampleTh(const PulseTimer_T * p_timer, uint32_t prevSampleTh)
// {
//     return HAL_ClockTimer_ReadOverflow(p_timer->P_HAL_TIMER) ? (prevSampleTh + p_timer->SAMPLE_TIME) : HAL_ClockTimer_Read(p_timer->P_HAL_TIMER);
// }

// static inline uint32_t PulseTimer_CaptureSampleTk(const PulseTimer_T * p_timer, uint32_t prevSampleTh)
// {
//     /* updates SampleTh and returns 0 on overflow */
//     return p_timer->SAMPLE_TIME + prevSampleTh - PulseTimer_CaptureSampleTh(p_timer, prevSampleTh);
// }

// static inline uint32_t PulseTimer_CaptureSampleTk_Freq(const PulseTimer_T * p_timer, uint32_t prevSampleTh)
// {
//     uint32_t sampleTk = PulseTimer_CaptureSampleTk(p_timer, prevSampleTh);
//     return (sampleTk > p_timer->SAMPLE_TIME / 2) ? (p_timer->TIMER_FREQ / sampleTk) : 0;
// }

// static inline bool PulseTimer_IsStop(const PulseTimer_T * p_timer) { return (p_timer->P_STATE->DeltaT >= PULSE_TIMER_MAX); }

// /******************************************************************************/
// /*
//     Capture Period < 1S
//     DeltaT / TimerFreq = DeltaT [Seconds]
// */
// /******************************************************************************/
// static inline uint32_t PulseTimer_DeltaT_Freq(const PulseTimer_T * p_timer) { return p_timer->TIMER_FREQ / p_timer->P_STATE->DeltaT; }
// static inline uint32_t PulseTimer_DeltaT_Ms(const PulseTimer_T * p_timer) { return p_timer->P_STATE->DeltaT * 1000U / p_timer->TIMER_FREQ; }



// /******************************************************************************/
// /*!
//     @brief Initialize the pulse timer.
// */
// /******************************************************************************/
// static inline void PulseTimer_Init(const PulseTimer_T * p_timer)
// {
//     HAL_ClockTimer_Init(p_timer->P_HAL_TIMER);
//     HAL_ClockTimer_InitFreq(p_timer->P_HAL_TIMER, p_timer->TIMER_FREQ);
// }

// static inline void PulseTimer_SetInitial(const PulseTimer_T * p_timer)
// {
//     HAL_ClockTimer_ClearOverflow(p_timer->P_HAL_TIMER);
//     HAL_ClockTimer_Write(p_timer->P_HAL_TIMER, 0U);
// }

// /*
//     Capture with overflow
// */
// // static inline void PulseTimer_Overflow_ISR(const PulseTimer_T * p_timer)
// // {
// //     p_timer->P_STATE->OverflowCount++;
// // }

// // static inline void _DeltaT_CaptureExtended(const PulseTimer_T * p_timer)
// // {
// //     uint32_t periodT = (p_timer->P_STATE->OverflowCount << 16) | HAL_Encoder_ReadTimer(p_timer->P_HAL_ENCODER_TIMER);
// //     HAL_Encoder_WriteTimer(p_timer->P_HAL_ENCODER_TIMER, 0U);
// //     HAL_Encoder_ClearTimerOverflow(p_timer->P_HAL_ENCODER_TIMER);
// //     p_timer->P_STATE->OverflowCount = 0U;
// //     p_timer->P_STATE->DeltaT = periodT;
// // }

// // // Stop detection — single threshold in base timer ticks
// // static inline bool _DeltaT_IsExtendedStop(const PulseTimer_T * p_timer)
// // {
// //     uint32_t elapsed = (p_timer->P_STATE->OverflowCount << 16) | HAL_Encoder_ReadTimer(p_timer->P_HAL_ENCODER_TIMER);
// //     return (elapsed > p_timer->P_STATE->Config.DeltaTStop);
// // }




