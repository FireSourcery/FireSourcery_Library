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
    @version V0
    @brief
*/
/******************************************************************************/
#include "Ramp.h"
#include <assert.h>

#define RAMP_SHIFT 14U /* Output range [-UINT16_MAX:UINT16_MAX]x2 without overflow */

/*
    return ouput as shifted
*/
static int32_t OutputOf(const Ramp_T * p_ramp, int32_t steps)
{
    int32_t output = p_ramp->State;

    // Ramp slope always positive
    if (p_ramp->State < p_ramp->Target) // incrementing
    {
        output = math_limit_upper(p_ramp->State + (p_ramp->Coefficient * steps), p_ramp->Target);
    }
    else if (p_ramp->State > p_ramp->Target) // decrementing
    {
        output = math_limit_lower(p_ramp->State - (p_ramp->Coefficient * steps), p_ramp->Target);
    }

    return output;
}

int32_t Ramp_ProcOutputN(Ramp_T * p_ramp, int32_t steps)
{
    if (p_ramp->State != p_ramp->Target) { p_ramp->State = OutputOf(p_ramp, steps); }
    return Ramp_GetOutput(p_ramp);
}

int32_t Ramp_ProcOutput(Ramp_T * p_ramp)
{
    return Ramp_ProcOutputN(p_ramp, 1U);
}

/******************************************************************************/
/*
    Ramp using Linear Aliases
*/
/******************************************************************************/
/*
    using fixed RAMP_SHIFT, so that interpolate ramps do not need to recalculate.

    range as positive only for now [0:UINT16_MAX]
*/
void Ramp_Init(Ramp_T * p_ramp, uint32_t duration_Ticks, int32_t range)
{
    p_ramp->Shift = RAMP_SHIFT;
    Ramp_SetSlope(p_ramp, duration_Ticks, range);
    Ramp_SetOutputState(p_ramp, 0);
}

void Ramp_Init_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t range)
{
    p_ramp->Shift = RAMP_SHIFT;
    Ramp_SetSlope_Millis(p_ramp, updateFreq_Hz, duration_Ms, range);
    Ramp_SetOutputState(p_ramp, 0);
}

/******************************************************************************/
/*
    Set Slope
*/
/******************************************************************************/
/* duration_Ticks != 0  */
void Ramp_SetSlope(Ramp_T * p_ramp, uint32_t duration_Ticks, int32_t range)
{
    p_ramp->Coefficient = (range << p_ramp->Shift) / duration_Ticks;
}

/*
    (duration_Ms * updateFreq_Hz) [0:131,071,000]
*/
void Ramp_SetSlope_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t range)
{
    uint32_t ticks = (duration_Ms != 0U) ? (updateFreq_Hz * duration_Ms / 1000U) : 1U;
    Ramp_SetSlope(p_ramp, ticks, range);
}

/******************************************************************************/
/*
    Set
*/
/******************************************************************************/
static inline void Ramp_SetOutput(Ramp_T * p_ramp, int32_t match) { p_ramp->State = (match << p_ramp->Shift); }

/* if initial > final AND acceleration is positive, ramp returns final value */
void Ramp_Set(Ramp_T * p_ramp, uint32_t duration_Ticks, int32_t initial, int32_t final)
{
    Ramp_SetSlope(p_ramp, duration_Ticks, final - initial);
    Ramp_SetOutput(p_ramp, initial);
    Ramp_SetTarget(p_ramp, final);
}

void Ramp_Set_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t initial, int32_t final)
{
    Ramp_SetSlope_Millis(p_ramp, updateFreq_Hz, duration_Ms, final - initial);
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