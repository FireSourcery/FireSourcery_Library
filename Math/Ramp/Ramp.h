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
    @file   Ramp.h
    @author FireSourcery

    @brief
*/
/******************************************************************************/
#ifndef RAMP_H
#define RAMP_H

#include "../Accumulator/Accumulator.h"
#include "../math_general.h"

#include <stdint.h>
#include <stdbool.h>

#define RAMP_SHIFT 14U /* Output range without overflow [-UINT16_MAX:UINT16_MAX] */

#define RAMP_TICKS_OF_RATE(UpdateFreq_Hz, Delta, UnitsPerSecond) ((UpdateFreq_Hz) * (Delta) / (UnitsPerSecond))

// typedef struct Ramp_Config_T
// {
//     uint16_t Units;         /*   */
//     uint32_t Time_Cycles;   /* Time to reach Units */
// }
// Ramp_Config_T;

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
/* [-UINT16_MAX:UINT16_MAX] */
static inline int32_t Ramp_GetTarget(const Ramp_T * p_ramp) { return (p_ramp->Target >> p_ramp->Accumulator.Shift); }
static inline void Ramp_SetTarget(Ramp_T * p_ramp, int32_t target) { p_ramp->Target = (target << p_ramp->Accumulator.Shift); }

/*  */
static inline int32_t Ramp_GetOutput(const Ramp_T * p_ramp) { return (p_ramp->Accumulator.State >> p_ramp->Accumulator.Shift); }
/* Match Output */
static inline void Ramp_SetOutput(Ramp_T * p_ramp, int32_t match) { p_ramp->Accumulator.State = (match << p_ramp->Accumulator.Shift); }

static inline void Ramp_SetOutputState(Ramp_T * p_ramp, int32_t match)
{
    p_ramp->Target = (match << p_ramp->Accumulator.Shift);
    p_ramp->Accumulator.State = p_ramp->Target;
}

static inline void Ramp_ZeroOutputState(Ramp_T * p_ramp)
{
    p_ramp->Target = 0;
    p_ramp->Accumulator.State = 0;
}

/* ProcAsDisabled */
static inline int32_t Ramp_ProcEndState(Ramp_T * p_ramp)
{
    p_ramp->Accumulator.State = p_ramp->Target;
    return Ramp_GetOutput(p_ramp);
}

/* single step proc only */
static inline bool _Ramp_IsDisabled(const Ramp_T * p_ramp) { return (p_ramp->Accumulator.Coefficient == (UINT16_MAX << RAMP_SHIFT)); }
static inline bool _Ramp_IsEnabled(const Ramp_T * p_ramp) { return  !_Ramp_IsDisabled(p_ramp); }
static inline void _Ramp_Disable(Ramp_T * p_ramp) { p_ramp->Accumulator.Coefficient = (UINT16_MAX << RAMP_SHIFT); }

// static inline bool Ramp_IsDisabled(const Ramp_T * p_ramp) { return (p_ramp->Accumulator.Coefficient == 0); }
// static inline void Ramp_Disable(Ramp_T * p_ramp) { p_ramp->Accumulator.Coefficient = 0; }

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

#endif


