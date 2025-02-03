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

#define LINEAR_RAMP_SHIFT 14U /* Output range [-UINT16_MAX:UINT16_MAX]x2 without overflow */


/*
    return ouput store as shifted => y0_shifted + m_shifted
    Linear_Of before shift back
*/
static inline int32_t OutputValueOf(const Linear_T * p_linear, int32_t currentRampValue, int32_t steps)
{
    return (currentRampValue + (p_linear->Slope * steps));
}

static int32_t OutputOf(const Linear_T * p_linear, int32_t currentRampValue, int32_t steps)
{
    int32_t newRampValue = currentRampValue;

    if (currentRampValue < p_linear->YReference) // incrementing
    {
        newRampValue = OutputValueOf(p_linear, currentRampValue, steps);
        if (newRampValue > p_linear->YReference) { newRampValue = p_linear->YReference; }
    }
    else if (currentRampValue > p_linear->YReference) // decrementing
    {
        newRampValue = OutputValueOf(p_linear, currentRampValue, 0 - steps); // Ramp slope always positive
        if (newRampValue < p_linear->YReference) { newRampValue = p_linear->YReference; }
    }

    return newRampValue;
}


// int32_t Ramp_NextOutputOfState(const Linear_T * p_linear, int32_t start, int32_t steps)
// {
//
// }

/* todo store as X */
// return Linear_Of(p_linear, steps);
// int32_t Ramp_OutputOf(const Linear_T * p_linear, int32_t steps)
// {
//     int32_t directionalSteps =steps;
//     int32_t output;
//      int32_t sign = math_sign(steps);

//     // if(Y0 == YReference) { return Y0; } /* no change */

//     /* instead of inverting slope, which requires handling synchronization */
//     if      (p_linear->Y0 < p_linear->YReference) { directionalSteps = steps;          }   // incrementing
//     else if (p_linear->Y0 > p_linear->YReference) { directionalSteps = 0 - steps;      }   // decrementing

//     // return (directionalSteps != 0) ?
//     //     linear_shift_f_y0(p_linear->Slope, p_linear->SlopeShift, p_linear->Y0, directionalSteps) : p_linear->Y0;

//     if (directionalSteps == 0)
//     {
//         output = p_linear->Y0;
//     }
//     else
//     {
//         output = linear_shift_f_y0(p_linear->Slope, p_linear->SlopeShift, p_linear->Y0, directionalSteps);
//         if      (p_linear->Y0 < p_linear->YReference && output > p_linear->YReference) { output = p_linear->YReference; }
//         else if (p_linear->Y0 > p_linear->YReference && output < p_linear->YReference) { output = p_linear->YReference; }
//     }
// }


// LinearOf, sat output
int32_t Ramp_ProcOutputN(Linear_T * p_linear, int32_t steps)
{
    if(p_linear->Y0 != p_linear->YReference) { p_linear->Y0 = OutputOf(p_linear, p_linear->Y0, steps); }
    return Ramp_GetOutput(p_linear);
}

int32_t Ramp_ProcOutput(Linear_T * p_linear)
{
    return Ramp_ProcOutputN(p_linear, 1U);
}

/******************************************************************************/
/*
    Ramp using Linear Aliases
*/
/******************************************************************************/
/*
    if initial > final AND acceleration is positive, ramp returns final value
    using fixed LINEAR_RAMP_SHIFT, so that dynamic ramps do not need to recalculate.
    Overflow: final > 65535*2 to include percent16 input
*/
void Ramp_Init(Linear_T * p_linear, uint32_t duration_Ticks, int32_t initial, int32_t final)
{
    p_linear->SlopeShift = LINEAR_RAMP_SHIFT;
    p_linear->InvSlopeShift = LINEAR_RAMP_SHIFT; // unused
    Ramp_SetSlope(p_linear, duration_Ticks, initial, final);
    Ramp_SetOutputState(p_linear, 0);
    Ramp_SetTarget(p_linear, 0);
}

/*
    Overflow: (duration_Ms * updateFreq_Hz) > 131,071,000
*/
void Ramp_Init_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t initial, int32_t final)
{
    p_linear->SlopeShift = LINEAR_RAMP_SHIFT;
    p_linear->InvSlopeShift = LINEAR_RAMP_SHIFT;
    Ramp_SetSlope_Millis(p_linear, updateFreq_Hz, duration_Ms, initial, final);
    Ramp_SetOutputState(p_linear, 0);
    Ramp_SetTarget(p_linear, 0);
}

/******************************************************************************/
/*
    Set Slope
*/
/******************************************************************************/

/* NonError Checked */
static inline void _Linear_SetSlope(Linear_T * p_linear, int32_t slopeFactor, int32_t slopeDivisor)
{
    p_linear->Slope = (slopeFactor << p_linear->SlopeShift) / slopeDivisor;
    p_linear->InvSlope = (slopeDivisor << p_linear->InvSlopeShift) / slopeFactor;
}


/* duration_Ticks != 0  */
void Ramp_SetSlope(Linear_T * p_linear, uint32_t duration_Ticks, int32_t initial, int32_t final)
{
    _Linear_SetSlope(p_linear, final - initial, duration_Ticks);
}

void Ramp_SetSlope_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t initial, int32_t final)
{
    uint32_t ticks = (duration_Ms != 0U) ? (duration_Ms * updateFreq_Hz / 1000U) : 1U;
    Ramp_SetSlope(p_linear, ticks, initial, final);
}

/******************************************************************************/
/*
    Set
*/
/******************************************************************************/
void Ramp_Set(Linear_T * p_linear, uint32_t duration_Ticks, int32_t initial, int32_t final)
{
    Ramp_SetSlope(p_linear, duration_Ticks, initial, final);
    Ramp_SetOutputState(p_linear, initial);
    Ramp_SetTarget(p_linear, final);
}

void Ramp_Set_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t initial, int32_t final)
{
    Ramp_SetSlope_Millis(p_linear, updateFreq_Hz, duration_Ms, initial, final);
    Ramp_SetOutputState(p_linear, initial);
    Ramp_SetTarget(p_linear, final);
}

/******************************************************************************/
/*
    Set Slope and initial state for dynamically generated ramp
    Divide input over control period intervals
    Acceleration proportional to change in userCmd
*/
/******************************************************************************/
void Ramp_SetInterpolate(Linear_T * p_linear, uint32_t duration_Ticks, int32_t final)
{
    Ramp_Set(p_linear, duration_Ticks, Ramp_GetOutput(p_linear), final);
}

void Ramp_SetInterpolate_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t final)
{
    Ramp_Set_Millis(p_linear, updateFreq_Hz, duration_Ms, Ramp_GetOutput(p_linear), final);
}