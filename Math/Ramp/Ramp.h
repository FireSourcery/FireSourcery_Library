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
    @file   Ramp.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "../Accumulator/Accumulator.h"
#include "../math_general.h"

#include <stdint.h>
#include <stdbool.h>

/*
    Accumulator Max INT32_MAX/2
        Accumulator + Input_Shifted < INT32_MAX/2
        Shift 14 for value range of [-UINT16_MAX:UINT16_MAX]
        Shift 15 for value range of [-INT16_MAX:INT16_MAX]
*/
#define RAMP_SHIFT 14U

#define RAMP_TICKS_OF_RATE(UpdateFreq_Hz, Delta, UnitsPerSecond) ((UpdateFreq_Hz) * (Delta) / (UnitsPerSecond))

#define RAMP_TICKS_OF_MILLIS(updateFreq_Hz, duration_Ms) (updateFreq_Hz * duration_Ms / 1000U)

#define RAMP_RATE_PER_SECOND(updateFreq_Hz, ratePerS) (((int32_t)ratePerS << RAMP_SHIFT) / updateFreq_Hz)

// typedef struct Ramp_Config_T
// {
//     uint16_t Delta;    /*   */
//     uint32_t Ticks;    /* Time to reach Units */
// }
// Ramp_Config_T;
/******************************************************************************/
/*
    Implementation by shifted accumulator.
    This way output state can be set directly.
        Index based implementation need inverse function
*/
/******************************************************************************/
typedef struct Ramp
{
    Accumulator_T Accumulator;
    int16_t Target;
}
Ramp_T;


/******************************************************************************/
/*

*/
/******************************************************************************/
static inline void Ramp_SetTarget(Ramp_T * p_ramp, int32_t target) { p_ramp->Target = target; }
/* GetInput */
static inline int32_t Ramp_GetTarget(const Ramp_T * p_ramp) { return p_ramp->Target; }

/*  */
static inline int32_t Ramp_GetOutput(const Ramp_T * p_ramp) { return (p_ramp->Accumulator.Accumulator >> RAMP_SHIFT); }
/* Match Output */
static inline void Ramp_SetOutput(Ramp_T * p_ramp, int32_t match) { p_ramp->Accumulator.Accumulator = (match << RAMP_SHIFT); }

/* Set both */
static inline void Ramp_SetOutputState(Ramp_T * p_ramp, int32_t match)
{
    Ramp_SetOutput(p_ramp, match);
    Ramp_SetTarget(p_ramp, match);
}

static inline void Ramp_ZeroOutputState(Ramp_T * p_ramp)
{
    p_ramp->Accumulator.Accumulator = 0;
    p_ramp->Target = 0;
}

/* ProcAsDisabled */
// static inline int32_t Ramp_ProcEndState(Ramp_T * p_ramp)
// {
//     Ramp_SetOutput(p_ramp, p_ramp->Target);
//     return Ramp_GetOutput(p_ramp);
// }

/* single step proc only */
static inline bool _Ramp_IsDisabled(const Ramp_T * p_ramp) { return (p_ramp->Accumulator.Coefficient == (UINT16_MAX << RAMP_SHIFT)); }
static inline bool _Ramp_IsEnabled(const Ramp_T * p_ramp) { return !_Ramp_IsDisabled(p_ramp); }
static inline void _Ramp_Disable(Ramp_T * p_ramp) { p_ramp->Accumulator.Coefficient = (UINT16_MAX << RAMP_SHIFT); }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern int32_t Ramp_ProcOutput(Ramp_T * p_ramp);
extern int32_t Ramp_ProcNextOf(Ramp_T * p_ramp, int16_t target);

extern void Ramp_Init(Ramp_T * p_ramp, uint32_t duration_Ticks, uint16_t range);
extern void Ramp_Init_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, uint16_t range);
extern void Ramp_SetSlope(Ramp_T * p_ramp, uint32_t duration_Ticks, uint16_t range);
extern void Ramp_SetSlope_Fract16(Ramp_T * p_ramp, uint16_t rate);
extern void Ramp_SetSlope_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, uint16_t range);

extern void Ramp_Set(Ramp_T * p_ramp, uint32_t duration_Ticks, int32_t initial, int32_t final);
extern void Ramp_Set_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t initial, int32_t final);
