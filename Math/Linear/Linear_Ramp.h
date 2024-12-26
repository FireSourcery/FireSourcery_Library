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
    @file   Linear_Ramp.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef LINEAR_RAMP_H
#define LINEAR_RAMP_H

#include "Linear.h"
#include <stdint.h>

// typedef struct Linear_Sum
// {
//     Linear_T Linear;
//     int32_t Sum32;
// }
// Linear_Sum_T;

// static inline int32_t sum32_of(int32_t slope32, int32_t value32, int32_t x)
// {
//     return (value32 + (slope32 * x));
// }

/******************************************************************************/
/*

*/
/******************************************************************************/
/* Aliases */
static inline int32_t Linear_Ramp_GetTarget(const Linear_T * p_linear) { return (p_linear->YReference >> p_linear->SlopeShift); }
static inline int32_t Linear_Ramp_GetOutput(const Linear_T * p_linear) { return (p_linear->Y0 >> p_linear->SlopeShift); }

static inline void Linear_Ramp_SetTarget(Linear_T * p_linear, int32_t target) { p_linear->YReference = (target << p_linear->SlopeShift); } //alternatively update slope if sign changed
static inline void Linear_Ramp_ZeroOutputState(Linear_T * p_linear)
{
    p_linear->YReference = 0;
    p_linear->Y0 = 0;
}
static inline void Linear_Ramp_SetOutputState(Linear_T * p_linear, int32_t match)
{
    p_linear->YReference = (match << p_linear->SlopeShift);
    p_linear->Y0 = (match << p_linear->SlopeShift);
}

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void Linear_Ramp_Init(Linear_T * p_linear, uint32_t duration_Ticks, int32_t initial, int32_t final);
extern void Linear_Ramp_Init_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t initial, int32_t final);
extern void Linear_Ramp_SetSlope(Linear_T * p_linear, uint32_t duration_Ticks, int32_t initial, int32_t final);
extern void Linear_Ramp_SetSlope_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t initial, int32_t final);
extern void Linear_Ramp_Set(Linear_T * p_linear, uint32_t duration_Ticks, int32_t initial, int32_t final);
extern void Linear_Ramp_Set_Millis(Linear_T * p_linear, uint32_t updateFreq_Hz, uint16_t duration_Ms, int32_t initial, int32_t final);

extern int32_t Linear_Ramp_NextOutputOfState(const Linear_T * p_linear, int32_t currentRampValue, int32_t steps);
extern int32_t Linear_Ramp_NextOutputOf(const Linear_T * p_linear, int32_t currentRampValue);
extern int32_t Linear_Ramp_ProcOutputN(Linear_T * p_linear, int32_t steps);
extern int32_t Linear_Ramp_ProcOutput(Linear_T * p_linear);

#endif

