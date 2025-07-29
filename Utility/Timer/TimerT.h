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
    @file   Timer_Static.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "_TimerT.h"

/******************************************************************************/
/*
    Static Polymorphic Implementation
    - Compile-time constants for maximum optimization
    - Macro-like performance with type safety
    - Hardware register addresses inlined
*/
/******************************************************************************/
typedef const struct TimerT
{
    Timer_Base_T BASE;
    Timer_State_T * P_STATE; /* Pointer to Timer State */
}
TimerT_T;

#define TIMER_STATE_ALLOC() (&(Timer_State_T){0})
#define TIMER_T_INIT(p_BaseTimer, BaseFreq, p_State) { .BASE = TIMER_BASE_INIT(p_BaseTimer, BaseFreq), .P_STATE = p_State, }
#define TIMER_T_ALLOC(p_BaseTimer, BaseFreq) TIMER_T_INIT(p_BaseTimer, BaseFreq, TIMER_STATE_ALLOC())

/******************************************************************************/
/*

*/
/******************************************************************************/
static inline uint32_t TimerT_GetElapsed(TimerT_T * p_timer) { return _TimerT_Elapsed(&p_timer->BASE, p_timer->P_STATE); }
static inline bool TimerT_IsElapsed(TimerT_T * p_timer) { return _TimerT_IsElapsed(&p_timer->BASE, p_timer->P_STATE); }
static inline void TimerT_Restart(TimerT_T * p_timer) { _TimerT_Restart(&p_timer->BASE, p_timer->P_STATE); }

/******************************************************************************/
/*

*/
/******************************************************************************/
static inline bool TimerT_Periodic_Poll(TimerT_T * p_timer) { return _TimerT_Periodic_Poll(&p_timer->BASE, p_timer->P_STATE); }
static inline bool TimerT_OneShot_Poll(TimerT_T * p_timer) { return _TimerT_OneShot_Poll(&p_timer->BASE, p_timer->P_STATE); }
static inline bool TimerT_CountN_Poll(TimerT_T * p_timer) { return _TimerT_CountN_Poll(&p_timer->BASE, p_timer->P_STATE); }
static inline bool TimerT_Counter_Poll(TimerT_T * p_timer) { return _TimerT_Counter_Poll(&p_timer->BASE, p_timer->P_STATE); }

/* Context counter operations */
static inline void TimerT_Counter_Init(TimerT_T * p_timer, uint32_t period) { _TimerT_Counter_Init(&p_timer->BASE, p_timer->P_STATE, period); }
static inline bool TimerT_Counter_IsAligned(TimerT_T * p_timer, uint32_t mask) { return _TimerT_Counter_IsAligned(p_timer->P_STATE, mask); }
static inline bool TimerT_Counter_IsAlignedDivider(TimerT_T * p_timer, uint32_t divider) { return _TimerT_Counter_IsAligned(p_timer->P_STATE, divider - 1U); }
static inline void TimerT_Counter_Set(TimerT_T * p_timer, uint32_t period) { _TimerT_Counter_Set(&p_timer->BASE, p_timer->P_STATE, period); }


static inline void TimerT_Periodic_Init(TimerT_T * p_timer, uint32_t period) { _TimerT_Periodic_Init(&p_timer->BASE, p_timer->P_STATE, period); }
static inline void TimerT_Periodic_Set(TimerT_T * p_timer, uint32_t period) { _TimerT_Periodic_Set(&p_timer->BASE, p_timer->P_STATE, period); }

static inline void TimerT_OneShot_Start(TimerT_T * p_timer, uint32_t ticks) { _TimerT_OneShot_Start(&p_timer->BASE, p_timer->P_STATE, ticks); }

static inline void TimerT_CountN_Start(TimerT_T * p_timer, uint32_t period, uint32_t shots) { _TimerT_CountN_Start(&p_timer->BASE, p_timer->P_STATE, period, shots); }

/******************************************************************************/
/* General/Modal */
/******************************************************************************/
static inline bool TimerT_Poll(TimerT_T * p_timer) { return _TimerT_Poll(&p_timer->BASE, p_timer->P_STATE); }
static inline bool TimerT_Modal_Poll(TimerT_T * p_timer) { return _TimerT_Modal_Poll(&p_timer->BASE, p_timer->P_STATE); }

/* Context initialization and control */
static inline void TimerT_InitMode(TimerT_T * p_timer, Timer_Mode_T mode) { _TimerT_InitMode(p_timer->P_STATE, mode); }
static inline void TimerT_Start(TimerT_T * p_timer, uint32_t period) { _TimerT_Start(&p_timer->BASE, p_timer->P_STATE, period); }
static inline void TimerT_Stop(TimerT_T * p_timer) { _TimerT_Stop(p_timer->P_STATE); }
static inline void TimerT_SetPeriod(TimerT_T * p_timer, uint32_t ticks) { _TimerT_SetPeriod(p_timer->P_STATE, ticks); }

/*  */
static inline void TimerT_StartPeriodic(TimerT_T * p_timer, uint32_t period) { _TimerT_StartPeriodic(&p_timer->BASE, p_timer->P_STATE, period); }
static inline void TimerT_StartOneShot(TimerT_T * p_timer, uint32_t period) { _TimerT_StartOneShot(&p_timer->BASE, p_timer->P_STATE, period); }
static inline void TimerT_StartCounterN(TimerT_T * p_timer, uint32_t period, uint32_t shots) { _TimerT_StartCounterN(&p_timer->BASE, p_timer->P_STATE, period, shots); }
static inline void TimerT_RestartOneShot(TimerT_T * p_timer) { _TimerT_RestartOneShot(&p_timer->BASE, p_timer->P_STATE); }

