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
    @file   Linear_Ramp.c
    @author FireSourcery
    @version V0
    @brief
*/
/******************************************************************************/
#include "Linear_Ramp.h"

#define LINEAR_RAMP_SHIFT 14U /* Output range [INT16_MIN:INT16_MAX]x2 without overflow */

/*
    store as y shifted
    y_shifted = y0_shifted + m_shifted

    alternatively, y0 = ((m_shifted * x0) >> shift)
*/
static inline int32_t CalcOutputInner(const Linear_T * p_linear, int32_t currentRampValue, int32_t steps)
{
    return ((currentRampValue) + (p_linear->Slope * steps));
}

static int32_t CalcOutput(const Linear_T * p_linear, int32_t currentRampValue, int32_t steps)
{
    int32_t newRampValue;

    if(currentRampValue < p_linear->YReference)
    {
        newRampValue = CalcOutputInner(p_linear, currentRampValue, steps);
        if(newRampValue > p_linear->YReference) { newRampValue = p_linear->YReference; }
    }
    else if(currentRampValue > p_linear->YReference)
    {
        newRampValue = CalcOutputInner(p_linear, currentRampValue, 0 - steps);
        if(newRampValue < p_linear->YReference) { newRampValue = p_linear->YReference; }
    }
    else { newRampValue = currentRampValue; }

    return newRampValue;
}

/* Seek output */
int32_t Linear_Ramp_CalcOutputN(const Linear_T * p_linear, int32_t currentRampValue, int32_t steps)
{
    return CalcOutput(p_linear, currentRampValue, steps) >> p_linear->SlopeShift;
}

int32_t Linear_Ramp_ProcOutputN(Linear_T * p_linear, int32_t steps)
{
    if(p_linear->YOffset != p_linear->YReference) { p_linear->YOffset = CalcOutput(p_linear, p_linear->YOffset, steps); }
    return Linear_Ramp_GetOutput(p_linear);
}

/* Next sequential output */
int32_t Linear_Ramp_CalcOutput(const Linear_T * p_linear, int32_t currentRampValue) { return Linear_Ramp_CalcOutputN(p_linear, currentRampValue, 1U); }
int32_t Linear_Ramp_ProcOutput(Linear_T * p_linear) { return Linear_Ramp_ProcOutputN(p_linear, 1U); }

/******************************************************************************/
/*
    Ramp using Linear Aliases
*/
/******************************************************************************/
/*
    if initial > final AND acceleration is positive, ramp returns final value
    Overflow: final > 65535
*/
void Linear_Ramp_Init(Linear_T * p_linear, uint32_t duration_Ticks, int32_t initial, int32_t final)
{
    p_linear->SlopeShift = LINEAR_RAMP_SHIFT;
    p_linear->InvSlopeShift = LINEAR_RAMP_SHIFT;
    Linear_Ramp_SetSlope(p_linear, duration_Ticks, initial, final);
    Linear_Ramp_SetState(p_linear, 0);
    Linear_Ramp_SetTarget(p_linear, 0);
}

/*
    Overflow: (duration_Ms * updateFreq_Hz) > 131,071,000
*/
void Linear_Ramp_Init_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t initial, int32_t final)
{
    p_linear->SlopeShift = LINEAR_RAMP_SHIFT;
    p_linear->InvSlopeShift = LINEAR_RAMP_SHIFT;
    Linear_Ramp_SetSlope_Millis(p_linear, updateFreq_Hz, duration_Ms, initial, final);
    Linear_Ramp_SetState(p_linear, 0);
    Linear_Ramp_SetTarget(p_linear, 0);
}

/******************************************************************************/
/*
    Set Slope
*/
/******************************************************************************/
/* duration_Ticks != 0  */
void Linear_Ramp_SetSlope(Linear_T * p_linear, uint32_t duration_Ticks, int32_t initial, int32_t final)
{
    _Linear_SetSlope(p_linear, final - initial, duration_Ticks);
}

void Linear_Ramp_SetSlope_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t initial, int32_t final)
{
    uint32_t ticks = (duration_Ms != 0U) ? (duration_Ms * updateFreq_Hz / 1000U) : 1U;
    Linear_Ramp_SetSlope(p_linear, ticks, initial, final);
}

/******************************************************************************/
/*
    Set
*/
/******************************************************************************/
void Linear_Ramp_Set(Linear_T * p_linear, uint32_t duration_Ticks, int32_t initial, int32_t final)
{
    Linear_Ramp_SetSlope(p_linear, duration_Ticks, initial, final);
    Linear_Ramp_SetState(p_linear, initial);
    Linear_Ramp_SetTarget(p_linear, final);
}

void Linear_Ramp_Set_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t initial, int32_t final)
{
    Linear_Ramp_SetSlope_Millis(p_linear, updateFreq_Hz, duration_Ms, initial, final);
    Linear_Ramp_SetState(p_linear, initial);
    Linear_Ramp_SetTarget(p_linear, final);
}

/******************************************************************************/
/*
    Set Slope and initial state for dynamically generated ramp
    Divide input over control period intervals
    Acceleration proportional to change in userCmd
*/
/******************************************************************************/
void Linear_Ramp_SetInterpolate(Linear_T * p_linear, uint32_t duration_Ticks, int32_t final)
{
    Linear_Ramp_Set(p_linear, duration_Ticks, Linear_Ramp_GetOutput(p_linear), final);
}

void Linear_Ramp_SetInterpolate_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t final)
{
    Linear_Ramp_Set_Millis(p_linear, updateFreq_Hz, duration_Ms, Linear_Ramp_GetOutput(p_linear), final);
}