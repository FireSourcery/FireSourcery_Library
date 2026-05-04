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
    @file   Ramp.c
    @author FireSourcery

    @brief
*/
/******************************************************************************/
#include "Ramp.h"
#include <assert.h>


/*
    Boundary chatter mostly shifts away.
*/
static int32_t NextOf(const Ramp_T * p_ramp, int32_t cw32, int32_t ccw32, int32_t target32)
{
    int32_t output32 = p_ramp->Accumulator.Accumulator + math_sign(target32 - p_ramp->Accumulator.Accumulator) * p_ramp->Accumulator.Coefficient;
    return math_clamp(output32, cw32, ccw32);
}

int32_t Ramp_ProcNextOf(Ramp_T * p_ramp, int16_t target)
{
    p_ramp->Accumulator.Accumulator = NextOf(p_ramp, p_ramp->Accumulator.LimitLower, p_ramp->Accumulator.LimitUpper, target << RAMP_SHIFT);
    return Ramp_GetOutput(p_ramp);
}


/*
    Limits are applied to the target. Ramp smoothing applies on limit update
    an out of bounds output state is gradually brough back into bounds.

    clamp: if step overshoots, saturate at target
*/
static int32_t NextOnInputOf(const Ramp_T * p_ramp, int32_t target32)
{
    int32_t diff = target32 - p_ramp->Accumulator.Accumulator;
    return (math_abs(diff) <= (uint32_t)p_ramp->Accumulator.Coefficient) ? target32 : (p_ramp->Accumulator.Accumulator + math_sign(diff) * (int32_t)p_ramp->Accumulator.Coefficient);
}

// caller hold input limits
int32_t _Ramp_ProcNextOnInputOf(Ramp_T * p_ramp, int16_t target)
{
    p_ramp->Accumulator.Accumulator = NextOnInputOf(p_ramp, target << RAMP_SHIFT);
    return Ramp_GetOutput(p_ramp);
}

int32_t Ramp_ProcNextOnInputOf(Ramp_T * p_ramp, int16_t target)
{
    p_ramp->Accumulator.Accumulator = NextOnInputOf(p_ramp, math_clamp(target << RAMP_SHIFT, p_ramp->Accumulator.LimitLower, p_ramp->Accumulator.LimitUpper));
    return Ramp_GetOutput(p_ramp);
}

/*

*/
int32_t Ramp_ProcNext(Ramp_T * p_ramp)
{
    p_ramp->Accumulator.Accumulator = NextOnInputOf(p_ramp, p_ramp->Target);
    return Ramp_GetOutput(p_ramp);
}

/******************************************************************************/
/*
    Ramp
*/
/******************************************************************************/
/*
    range as positive only [0:UINT16_MAX]
*/
void _Ramp_Init(Ramp_T * p_ramp, uint16_t coeff)
{
    p_ramp->Accumulator.Shift = RAMP_SHIFT;
    p_ramp->Accumulator.Coefficient = coeff << RAMP_SHIFT;
    Ramp_SetOutputLimit(p_ramp, INT16_MIN, INT16_MAX);
    p_ramp->Accumulator.Accumulator = 0;
    p_ramp->Target = 0;
}

/* symetric limits */
void Ramp_Init(Ramp_T * p_ramp, uint32_t duration_Ticks, uint16_t range)
{
    p_ramp->Accumulator.Shift = RAMP_SHIFT;
    Ramp_SetSlope(p_ramp, duration_Ticks, range);
    Ramp_SetOutputLimit(p_ramp, -(int32_t)range, range);
    p_ramp->Accumulator.Accumulator = 0;
    p_ramp->Target = 0;
}

/******************************************************************************/
/*
    Set Slope
*/
/******************************************************************************/
/* rate_accum32 >> 15 as step per tick */
void Ramp_SetCoefficient(Ramp_T * p_ramp, uint32_t rate_accum32)
{
    p_ramp->Accumulator.Coefficient = rate_accum32 >> (15U - RAMP_SHIFT); // or RAMP_SHIFT update to 15
}

void Ramp_SetCoefficient_Fract32(Ramp_T * p_ramp, uint32_t rate) { Ramp_SetCoefficient(p_ramp, rate >> 1); }

/* range: final value from 0 */
/* duration_Ticks != 0  */
void Ramp_SetSlope(Ramp_T * p_ramp, uint32_t duration_Ticks, uint16_t range)
{
    assert(duration_Ticks != 0U);
    p_ramp->Accumulator.Coefficient = ((int32_t)range << RAMP_SHIFT) / duration_Ticks;
}

/*
    (duration_Ms * updateFreq_Hz) [0:131,071,000]
*/
void Ramp_SetSlope_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, uint16_t range)
{
    uint32_t ticks = (duration_Ms != 0U) ? (updateFreq_Hz * duration_Ms / 1000U) : 1U;
    Ramp_SetSlope(p_ramp, ticks, range);
}

void Ramp_SetSlope_PerSecond(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t rate_PerS)
{
    p_ramp->Accumulator.Coefficient = RAMP_COEF_OF_SLOPE(updateFreq_Hz, rate_PerS);
}


/******************************************************************************/
/*
    Set Slope and initial state for dynamically generated ramp
    Divide input over control period intervals
    Acceleration proportional to change in userCmd
*/
/******************************************************************************/
void Ramp_SetInterpolate(Ramp_T * p_ramp, uint32_t duration_Ticks, int32_t final)
{
    Ramp_SetSlope(p_ramp, duration_Ticks, math_abs(final - Ramp_GetOutput(p_ramp)));
}

void Ramp_SetInterpolate_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t final)
{
    Ramp_SetSlope_Millis(p_ramp, updateFreq_Hz, duration_Ms, math_abs(final - Ramp_GetOutput(p_ramp)));
}