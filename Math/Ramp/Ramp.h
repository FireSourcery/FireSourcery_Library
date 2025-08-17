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

#define RAMP_TICKS_OF_MILLIS(updateFreq_Hz, duration_Ms) (updateFreq_Hz * duration_Ms / 1000U)

#define RAMP_RATE_PER_SECOND(updateFreq_Hz, ratePerS) (((int32_t)ratePerS << RAMP_SHIFT) / updateFreq_Hz)

// typedef struct Ramp_Config_T
// {
//     uint16_t Delta;    /*   */
//     uint32_t Ticks;    /* Time to reach Units */
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
/* GetInput */
/* [-UINT16_MAX:UINT16_MAX] */
// static inline int32_t Ramp_GetTarget(const Ramp_T * p_ramp) { return (p_ramp->Target >> p_ramp->Accumulator.Shift); }
// static inline void Ramp_SetTarget(Ramp_T * p_ramp, int32_t target) { p_ramp->Target = (target << p_ramp->Accumulator.Shift); }
static inline int32_t Ramp_GetTarget(const Ramp_T * p_ramp) { return p_ramp->Target; }
static inline void Ramp_SetTarget(Ramp_T * p_ramp, int32_t target) { p_ramp->Target = target; }

/*  */
static inline int32_t Ramp_GetOutput(const Ramp_T * p_ramp) { return (p_ramp->Accumulator.State >> p_ramp->Accumulator.Shift); }
/* Match Output */
static inline void Ramp_SetOutput(Ramp_T * p_ramp, int32_t match) { p_ramp->Accumulator.State = (match << p_ramp->Accumulator.Shift); }

/* Set both */
static inline void Ramp_SetOutputState(Ramp_T * p_ramp, int32_t match)
{
    Ramp_SetOutput(p_ramp, match);
    // p_ramp->Target = p_ramp->Accumulator.State; /* Set Target to current output */
    p_ramp->Target = match; /* Set Target to current output */
}

static inline void Ramp_ZeroOutputState(Ramp_T * p_ramp)
{
    p_ramp->Accumulator.State = 0;
    p_ramp->Target = 0;
}

/* ProcAsDisabled */
static inline int32_t Ramp_ProcEndState(Ramp_T * p_ramp)
{
    p_ramp->Accumulator.State = (p_ramp->Target << p_ramp->Accumulator.Shift);
    return Ramp_GetOutput(p_ramp);
}

/* single step proc only */
static inline bool _Ramp_IsDisabled(const Ramp_T * p_ramp) { return (p_ramp->Accumulator.Coefficient == (UINT16_MAX << RAMP_SHIFT)); }
static inline bool _Ramp_IsEnabled(const Ramp_T * p_ramp) { return  !_Ramp_IsDisabled(p_ramp); }
static inline void _Ramp_Disable(Ramp_T * p_ramp) { p_ramp->Accumulator.Coefficient = (UINT16_MAX << RAMP_SHIFT); }
// static inline void _Ramp_Enable(Ramp_T * p_ramp, int32_t) { p_ramp->Accumulator.Coefficient =  ; }

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


// static inline int32_t RateLimit_Update(int32_t input, int32_t prev_output, int32_t max_increase, int32_t max_decrease)
// {
//     int32_t delta = input - prev_output;

//     if (delta > max_increase)
//         return prev_output + max_increase;
//     else if (delta < -max_decrease)
//         return prev_output - max_decrease;
//     else
//         return input;
// }

// static inline int32_t RateLimit_Symmetric(int32_t input, int32_t prev_output, int32_t max_rate)
// {
//     return RateLimit_Update(input, prev_output, max_rate, max_rate);
// }

// /* Percentage-based rate limiting */
// static inline int32_t RateLimit_Percent(int32_t input, int32_t prev_output, uint8_t max_change_percent)
// {
//     int32_t max_delta = (int32_t)((int64_t)ABS(prev_output) * max_change_percent / 100);
//     if (max_delta == 0) max_delta = 1;  /* Minimum change */

//     return RateLimit_Symmetric(input, prev_output, max_delta);
// }


/* Speed ramping with hysteresis */
// static inline int32_t Hysteresis_RampValue(int32_t threshold_upper, int32_t threshold_lower, int32_t prev_output, int32_t input, int32_t ramp_rate)
// {
//     int32_t filtered = Hysteresis_FilterValue(threshold_upper, threshold_lower, prev_output, input);

//     /* Apply ramping if value changed */
//     if (filtered != prev_output)
//     {
//         int32_t delta = filtered - prev_output;
//         int32_t ramp_delta = CLAMP(delta, -ramp_rate, ramp_rate);
//         return prev_output + ramp_delta;
//     }

//     return prev_output;
// }