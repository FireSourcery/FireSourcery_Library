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

int32_t Ramp_ProcNextWith(Ramp_T * p_ramp, int16_t lower, int16_t upper, int16_t target)
{
    p_ramp->Accumulator.Accumulator = NextOf(p_ramp, lower << RAMP_SHIFT, upper << RAMP_SHIFT, target << RAMP_SHIFT);
    return Ramp_GetOutput(p_ramp);
}


/*
    Bounds at target
    clamp: if step overshoots, saturate at target
*/
static int32_t NextOnInputOf(const Ramp_T * p_ramp, int32_t target32)
{
    int32_t diff = target32 - p_ramp->Accumulator.Accumulator;
    return (math_abs(diff) <= (uint32_t)p_ramp->Accumulator.Coefficient) ? target32 : (p_ramp->Accumulator.Accumulator + math_sign(diff) * (int32_t)p_ramp->Accumulator.Coefficient);
}

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

int32_t Ramp_ProcNextOnInputWith(Ramp_T * p_ramp, int16_t lower, int16_t upper, int16_t target)
{
    p_ramp->Accumulator.Accumulator = NextOnInputOf(p_ramp, math_clamp(target, lower, upper) << RAMP_SHIFT);
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
void Ramp_Init(Ramp_T * p_ramp, uint32_t duration_Ticks, uint16_t range)
{
    p_ramp->Accumulator.Shift = RAMP_SHIFT;
    Ramp_SetSlope(p_ramp, duration_Ticks, range);
    Ramp_SetOutputState(p_ramp, 0);
    p_ramp->Accumulator.LimitLower = (int32_t)INT16_MIN * (1 << RAMP_SHIFT);
    p_ramp->Accumulator.LimitUpper = (int32_t)INT16_MAX * (1 << RAMP_SHIFT);
}

void _Ramp_Init(Ramp_T * p_ramp, uint16_t coeff)
{
    p_ramp->Accumulator.Shift = RAMP_SHIFT;
    p_ramp->Accumulator.Coefficient = coeff << RAMP_SHIFT;
    Ramp_SetOutputState(p_ramp, 0);
    p_ramp->Accumulator.LimitLower = (int32_t)INT16_MIN * (1 << RAMP_SHIFT);
    p_ramp->Accumulator.LimitUpper = (int32_t)INT16_MAX * (1 << RAMP_SHIFT);
}


/******************************************************************************/
/*
    Set Slope
*/
/******************************************************************************/
/* range: final value from 0 */
/* duration_Ticks != 0  */
void Ramp_SetSlope(Ramp_T * p_ramp, uint32_t duration_Ticks, uint16_t range)
{
    assert(duration_Ticks != 0U);
    p_ramp->Accumulator.Coefficient = ((int32_t)range << RAMP_SHIFT) / duration_Ticks;
}

/* [0:UINT16_MAX] Per Tick */
/* rate_accum32 as output << 15 */
void _Ramp_SetCoefficient(Ramp_T * p_ramp, uint32_t rate_accum32)
{
    p_ramp->Accumulator.Coefficient = rate_accum32 >> (15U - RAMP_SHIFT); // or RAMP_SHIFT update to 15
}

void Ramp_SetOutputLimit(Ramp_T * p_ramp, int16_t lower, int16_t upper)
{
    p_ramp->Accumulator.LimitLower = (int32_t)lower << RAMP_SHIFT;
    p_ramp->Accumulator.LimitUpper = (int32_t)upper << RAMP_SHIFT;
}

/* todo handle with config options */
// void Ramp_Init_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, uint16_t range)
// {
//     p_ramp->Accumulator.Shift = RAMP_SHIFT;
//     Ramp_SetSlope_Millis(p_ramp, updateFreq_Hz, duration_Ms, range);
//     Ramp_SetOutputState(p_ramp, 0);
// }

/*
    (duration_Ms * updateFreq_Hz) [0:131,071,000]
*/
void Ramp_SetSlope_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, uint16_t range)
{
    uint32_t ticks = (duration_Ms != 0U) ? (updateFreq_Hz * duration_Ms / 1000U) : 1U;
    Ramp_SetSlope(p_ramp, ticks, range);
}

void Ramp_SetSlope_PerSecond(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t ratePerS)
{
    p_ramp->Accumulator.Coefficient = RAMP_RATE_PER_SECOND(updateFreq_Hz, ratePerS);
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