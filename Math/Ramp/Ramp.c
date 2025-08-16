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
    target is updated independently
*/
/* Slope as abs */
static int32_t NextOf(const Ramp_T * p_ramp, int32_t target)
{
    int32_t target32 = target << p_ramp->Accumulator.Shift;
    int32_t output32 = p_ramp->Accumulator.State;

    if (target32 > p_ramp->Accumulator.State) // incrementing
    {
        output32 = math_limit_upper(p_ramp->Accumulator.State + p_ramp->Accumulator.Coefficient, target32);
    }
    else if (target32 < p_ramp->Accumulator.State) // decrementing
    {
        output32 = math_limit_lower(p_ramp->Accumulator.State - p_ramp->Accumulator.Coefficient, target32);
    }

    return output32;
}

int32_t Ramp_ProcNextOf(Ramp_T * p_ramp, int16_t target)
{
    if (Ramp_GetOutput(p_ramp) != target) { p_ramp->Accumulator.State = NextOf(p_ramp, target); }
    return Ramp_GetOutput(p_ramp);
}

// int32_t Ramp_ProcNext_With(Ramp_T * p_ramp, int16_t lowerLimit, int16_t upperLimit, int16_t target)
// {
//     if (Ramp_GetOutput(p_ramp) != target) { p_ramp->Accumulator.State = NextOf(p_ramp, math_clamp(target, lowerLimit, upperLimit)); }
//     return Ramp_GetOutput(p_ramp);
// }

// int32_t Ramp_ProcNext_InternalLimits(Ramp_T * p_ramp, int16_t target)
// {
//    return Ramp_ProcNextWith(p_ramp, target, p_ramp->Accumulator.LimitLower, p_ramp->Accumulator.LimitUpper);
// }

int32_t Ramp_ProcOutput(Ramp_T * p_ramp)
{
    return Ramp_ProcNextOf(p_ramp, Ramp_GetTarget(p_ramp));
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
}

// void Ramp_Init_Rate(Ramp_T * p_ramp, uint32_t rate_fract16)
// {
//     p_ramp->Accumulator.Shift = RAMP_SHIFT;
//     Ramp_SetSlope_Fract16(p_ramp, rate_fract16);
//     Ramp_SetOutputState(p_ramp, 0);
// }

void Ramp_Init_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, uint16_t range)
{
    p_ramp->Accumulator.Shift = RAMP_SHIFT;
    Ramp_SetSlope_Millis(p_ramp, updateFreq_Hz, duration_Ms, range);
    Ramp_SetOutputState(p_ramp, 0);
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
    p_ramp->Accumulator.Coefficient = ((int32_t)range << p_ramp->Accumulator.Shift) / duration_Ticks;
}

/* [0:UINT16_MAX] / Tick */
void Ramp_SetSlope_Rate(Ramp_T * p_ramp, uint16_t rate)
{
    p_ramp->Accumulator.Coefficient = rate << RAMP_SHIFT;
}

// void Ramp_SetSlope_Rate(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t ratePerS)
// {
//     p_ramp->Accumulator.Coefficient = rate << RAMP_SHIFT;
// }

/*
    (duration_Ms * updateFreq_Hz) [0:131,071,000]
*/
void Ramp_SetSlope_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, uint16_t range)
{
    uint32_t ticks = (duration_Ms != 0U) ? (updateFreq_Hz * duration_Ms / 1000U) : 1U;
    Ramp_SetSlope(p_ramp, ticks, range);
}

/******************************************************************************/
/*
    Set
*/
/******************************************************************************/
/* config excluding shift */
/* if initial > final AND acceleration is positive, ramp returns final value */
void Ramp_Set(Ramp_T * p_ramp, uint32_t duration_Ticks, int32_t initial, int32_t final)
{
    Ramp_SetSlope(p_ramp, duration_Ticks, math_abs(final - initial));
    Ramp_SetOutput(p_ramp, initial);
    Ramp_SetTarget(p_ramp, final);
}

void Ramp_Set_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t initial, int32_t final)
{
    Ramp_SetSlope_Millis(p_ramp, updateFreq_Hz, duration_Ms, math_abs(final - initial));
    Ramp_SetOutput(p_ramp, initial);
    Ramp_SetTarget(p_ramp, final);
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
    Ramp_SetSlope(p_ramp, duration_Ticks, final - Ramp_GetOutput(p_ramp));
    Ramp_SetTarget(p_ramp, final);
}

void Ramp_SetInterpolate_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t final)
{
    Ramp_SetSlope_Millis(p_ramp, updateFreq_Hz, duration_Ms, final - Ramp_GetOutput(p_ramp));
    Ramp_SetTarget(p_ramp, final);
}